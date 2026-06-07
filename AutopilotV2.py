from pythonosc import dispatcher, osc_server
import pygame
import threading
import time
from collections import deque
from gpiozero import Servo
import pigpio
import numpy as np

# ================== CONFIG ==================

LISTEN_IP = "0.0.0.0"   # listen on all interfaces
LISTEN_PORT = 5000      # match this with Mind Monitor / Muse app

ARTIFACT_LATCH_SEC = 0.5   # how long blink stays "lit" after event

WINDOW_WIDTH = 900
WINDOW_HEIGHT = 600
FPS = 30  # window redraw / event loop

# How many times per second to update displayed text (independent of FPS)
TEXT_UPDATE_HZ = 15.0
TEXT_UPDATE_INTERVAL = 1.0 / TEXT_UPDATE_HZ

# Use derived bands (FFT on /muse/eeg)
USE_DERIVED_BANDS = True

# EEG / FFT parameters (Muse is typically 256 Hz; adjust if needed)
EEG_SAMPLE_RATE = 256.0
FFT_WINDOW_SEC = 1.0
FFT_WINDOW_SAMPLES = int(EEG_SAMPLE_RATE * FFT_WINDOW_SEC)

# ====== Graph config (lower-right band history) ======
GRAPH_ENABLED_DEFAULT = True           # starting state; toggle with "G" key
GRAPH_WIDTH = 350                      # pixels
GRAPH_HEIGHT = 200                     # pixels
GRAPH_MARGIN = 10                      # distance from window edges
GRAPH_SAMPLE_HZ = 30.0                 # how often to sample band values (e.g. 30 or 60)
GRAPH_SAMPLE_INTERVAL = 1.0 / GRAPH_SAMPLE_HZ
GRAPH_LINE_THICKNESS = 2               # line thickness in pixels
GRAPH_TIME_WINDOW_SEC = 10.0           # seconds visible on X axis
GRAPH_MAX_SAMPLES = int(GRAPH_TIME_WINDOW_SEC * GRAPH_SAMPLE_HZ)

# How to combine 4 sensors into one graph value per band:
#   "max" -> spike-sensitive, shows the strongest channel
#   "avg" -> smoother average of all 4 channels
GRAPH_COMBINE_MODE_DEFAULT = "max"     # toggle with "C"

# Band graph modes (which bands to show at once)
BAND_GRAPH_MODES = [
    ("ALL",   ["delta", "theta", "alpha", "beta", "gamma"]),
    ("SLOW",  ["delta", "theta"]),
    ("ALPHA", ["alpha"]),
    ("FAST",  ["beta", "gamma"]),
    # "SINGLE" mode will be last, one band + one channel
    ("SINGLE", ["alpha"]),
]

SINGLE_BAND_SEQUENCE = ["delta", "theta", "alpha", "beta", "gamma"]


# ================== STATE ==================

state = {
    "eeg": [0.0, 0.0, 0.0, 0.0],    # TP9, AF7, AF8, TP10 (last sample)
    # per-channel bands: [TP9, AF7, AF8, TP10]
    "delta": [0.0, 0.0, 0.0, 0.0],
    "theta": [0.0, 0.0, 0.0, 0.0],
    "alpha": [0.0, 0.0, 0.0, 0.0],
    "beta":  [0.0, 0.0, 0.0, 0.0],
    "gamma": [0.0, 0.0, 0.0, 0.0],
    "acc": (0.0, 0.0, 0.0),
    "gyro": (0.0, 0.0, 0.0),
    "horseshoe": [0, 0, 0, 0],      # TP9, AF7, AF8, TP10 contact quality
    "batt_pct": None,
    "batt_volt": None,
    "batt_temp": None,
    "batt_status": None,
    "artifact_blink": 0,
    "artifact_blink_ts": 0.0,
    "last_addr": "",
    "last_args": (),
    "last_update": None,
}

# Raw EEG buffers for FFT (one per channel)
eeg_buffers = [
    deque(maxlen=FFT_WINDOW_SAMPLES),
    deque(maxlen=FFT_WINDOW_SAMPLES),
    deque(maxlen=FFT_WINDOW_SAMPLES),
    deque(maxlen=FFT_WINDOW_SAMPLES),
]

# Band history for graph: combined per-band values over time
# Each entry: (timestamp, combined_value)
band_history = {
    "delta": deque(maxlen=GRAPH_MAX_SAMPLES),
    "theta": deque(maxlen=GRAPH_MAX_SAMPLES),
    "alpha": deque(maxlen=GRAPH_MAX_SAMPLES),
    "beta":  deque(maxlen=GRAPH_MAX_SAMPLES),
    "gamma": deque(maxlen=GRAPH_MAX_SAMPLES),
}

osc_server_instance = None
osc_thread = None
RUNNING = True


# ================== BAND DERIVATION ==================

def compute_bands_from_eeg():
    """
    Use the last FFT_WINDOW_SAMPLES from each EEG channel to compute
    delta/theta/alpha/beta/gamma band power per channel.

    Stores log-scaled band power in state["delta"/"theta"/...][channel_index].
    """
    if any(len(buf) < FFT_WINDOW_SAMPLES for buf in eeg_buffers):
        return

    data = np.array([list(buf) for buf in eeg_buffers], dtype=np.float64)
    data = data - data.mean(axis=1, keepdims=True)

    window = np.hamming(FFT_WINDOW_SAMPLES)
    data_win = data * window

    fft_vals = np.fft.rfft(data_win, axis=1)
    freqs = np.fft.rfftfreq(FFT_WINDOW_SAMPLES, d=1.0 / EEG_SAMPLE_RATE)

    psd = np.abs(fft_vals) ** 2

    def band_power_log(psd_channel, f_low, f_high):
        mask = (freqs >= f_low) & (freqs < f_high)
        if not np.any(mask):
            return 0.0
        raw_power = float(psd_channel[mask].mean())
        return float(np.log10(raw_power + 1e-8))

    bands = {
        "delta": (1.0, 4.0),
        "theta": (4.0, 8.0),
        "alpha": (8.0, 13.0),
        "beta":  (13.0, 30.0),
        "gamma": (30.0, 45.0),
    }

    for band_name, (f_low, f_high) in bands.items():
        values = []
        for ch_idx in range(4):
            ch_psd = psd[ch_idx]
            bp = band_power_log(ch_psd, f_low, f_high)
            values.append(bp)
        state[band_name] = values


# ================== OSC HANDLER ==================

def osc_handler(address, *args):
    addr = str(address)

    state["last_addr"] = addr
    state["last_args"] = args
    state["last_update"] = time.time()

    if addr == "/muse/eeg" and len(args) >= 4:
        ch_vals = [
            float(args[0]),
            float(args[1]),
            float(args[2]),
            float(args[3]),
        ]
        state["eeg"] = ch_vals

        for i in range(4):
            eeg_buffers[i].append(ch_vals[i])

        if USE_DERIVED_BANDS:
            compute_bands_from_eeg()

    elif not USE_DERIVED_BANDS:
        if addr == "/muse/elements/delta_absolute" and args:
            v = float(args[0])
            state["delta"] = [v, v, v, v]
        elif addr == "/muse/elements/theta_absolute" and args:
            v = float(args[0])
            state["theta"] = [v, v, v, v]
        elif addr == "/muse/elements/alpha_absolute" and args:
            v = float(args[0])
            state["alpha"] = [v, v, v, v]
        elif addr == "/muse/elements/beta_absolute" and args:
            v = float(args[0])
            state["beta"] = [v, v, v, v]
        elif addr == "/muse/elements/gamma_absolute" and args:
            v = float(args[0])
            state["gamma"] = [v, v, v, v]

    if addr == "/muse/acc" and len(args) >= 3:
        state["acc"] = (
            float(args[0]),
            float(args[1]),
            float(args[2]),
        )

    if addr == "/muse/gyro" and len(args) >= 3:
        state["gyro"] = (
            float(args[0]),
            float(args[1]),
            float(args[2]),
        )

    if "horseshoe" in addr.lower() and len(args) >= 4:
        try:
            state["horseshoe"] = [
                int(args[0]),
                int(args[1]),
                int(args[2]),
                int(args[3]),
            ]
        except Exception:
            pass

    if addr == "/muse/batt" and len(args) >= 4:
        soc_raw = float(args[0])
        mv_fg = float(args[1])
        mv_adc = float(args[2])
        temp_c = float(args[3])

        state["batt_pct"] = soc_raw / 100.0
        state["batt_volt"] = mv_fg / 1000.0
        state["batt_temp"] = temp_c
        state["batt_status"] = None

    if "blink" in addr.lower() and args:
        try:
            val = float(args[0])
        except Exception:
            val = 0.0
        if val > 0.5:
            state["artifact_blink"] = 1
            state["artifact_blink_ts"] = time.time()


# ================== OSC SERVER THREAD ==================

def osc_server_loop(ip, port):
    global osc_server_instance

    disp = dispatcher.Dispatcher()
    disp.set_default_handler(osc_handler)

    osc_server_instance = osc_server.ThreadingOSCUDPServer((ip, port), disp)
    print(f"[OSC] Listening for OSC on {ip}:{port}")
    osc_server_instance.serve_forever()
    print("[OSC] Server thread exiting.")


def start_osc_server_thread(ip, port):
    global osc_thread
    osc_thread = threading.Thread(
        target=osc_server_loop,
        args=(ip, port),
        daemon=True,
    )
    osc_thread.start()


# ================== UI HELPERS ==================

def hs_label(v):
    try:
        v = int(v)
    except Exception:
        return "?"
    if v == 1:
        return "good"
    elif v == 2:
        return "ok"
    elif v == 3:
        return "bad"
    else:
        return "?"


def fmt(x, prec=2):
    try:
        return f"{float(x):.{prec}f}"
    except Exception:
        return " ?"


def sample_graph_values(now, graph_enabled, combine_mode, active_band_keys, single_channel_index):
    """
    Sample combined band values into band_history if graph is enabled.

    - For SINGLE mode (len(active_band_keys) == 1), uses only that band and
      the chosen channel index.
    - For other modes, uses max/avg over channels.
    """
    if not graph_enabled:
        return

    single_mode = (len(active_band_keys) == 1)

    for key in active_band_keys:
        vals = state[key]
        if not isinstance(vals, (list, tuple)) or len(vals) != 4:
            continue

        if single_mode and single_channel_index is not None and 0 <= single_channel_index < 4:
            combined = vals[single_channel_index]
        else:
            if combine_mode == "max":
                combined = max(vals)
            else:
                combined = sum(vals) / 4.0

        band_history[key].append((now, combined))


def draw_band_graph(screen, now, font_small, active_band_keys, mode_name, combine_mode, single_channel_index):
    """Draw band history graph in lower-right corner."""
    rect_x = WINDOW_WIDTH - GRAPH_WIDTH - GRAPH_MARGIN
    rect_y = WINDOW_HEIGHT - GRAPH_HEIGHT - GRAPH_MARGIN
    rect_w = GRAPH_WIDTH
    rect_h = GRAPH_HEIGHT

    border_color = (180, 180, 180)
    chan_labels = ["TP9", "AF7", "AF8", "TP10"]

    pygame.draw.rect(screen, (5, 5, 15), (rect_x, rect_y, rect_w, rect_h))
    pygame.draw.rect(screen, border_color, (rect_x, rect_y, rect_w, rect_h), 1)

    if mode_name == "SINGLE" and len(active_band_keys) == 1 and 0 <= single_channel_index < 4:
        band_key = active_band_keys[0]
        chan_name = chan_labels[single_channel_index]
        label_text = f"{band_key.upper()}:{chan_name} [{mode_name}] (C={combine_mode}) [G/B/V/1-4]"
    else:
        label_text = f"Bands ({combine_mode}, {mode_name}) [G/B/C]"

    label = font_small.render(label_text, True, border_color)
    screen.blit(label, (rect_x + 4, rect_y + 4))

    t_min = now - GRAPH_TIME_WINDOW_SEC
    all_vals = []

    for key in active_band_keys:
        for (t, v) in band_history[key]:
            if t >= t_min:
                all_vals.append(v)

    if not all_vals:
        return

    v_min = min(all_vals)
    v_max = max(all_vals)
    if v_min == v_max:
        v_min -= 1.0
        v_max += 1.0

    pad_top = 18
    pad_bottom = 6
    pad_left = 4
    pad_right = 4

    x_left = rect_x + pad_left
    x_right = rect_x + rect_w - pad_right
    y_top = rect_y + pad_top
    y_bottom = rect_y + rect_h - pad_bottom

    colors = {
        "delta": (255, 0, 0),       # red
        "theta": (160, 32, 240),    # purple
        "alpha": (0, 0, 255),       # blue
        "beta":  (0, 200, 0),       # green
        "gamma": (255, 255, 0),     # yellow
    }

    def map_y(v):
        frac = (v - v_min) / (v_max - v_min)
        return y_bottom - frac * (y_bottom - y_top)

    for key in active_band_keys:
        hist = band_history[key]
        points = []
        for (t, v) in hist:
            if t < t_min:
                continue
            frac_t = (t - t_min) / GRAPH_TIME_WINDOW_SEC
            x = x_left + frac_t * (x_right - x_left)
            y = map_y(v)
            points.append((x, y))
        if len(points) >= 2:
            pygame.draw.lines(screen, colors.get(key, border_color), False, points, GRAPH_LINE_THICKNESS)


# ================== PYGAME UI ==================

def run_pygame_ui():
    global RUNNING

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Autopilot OSC Monitor (pygame)")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("monospace", 18)
    font_small = pygame.font.SysFont("monospace", 14)

    bg_color = (10, 10, 20)
    text_color = (220, 220, 220)
    accent_color = (80, 160, 255)
    warn_color = (255, 80, 80)
    up_color = (80, 220, 120)
    down_color = (255, 80, 80)

    band_name_colors = {
        "Delta": (255, 0, 0),       # red
        "Theta": (160, 32, 240),    # purple
        "Alpha": (0, 0, 255),       # blue
        "Beta":  (0, 200, 0),       # green
        "Gamma": (255, 255, 0),     # yellow
    }

    last_text_update = 0.0
    last_graph_sample = 0.0

    graph_enabled = GRAPH_ENABLED_DEFAULT
    graph_combine_mode = GRAPH_COMBINE_MODE_DEFAULT
    band_mode_index = 0  # index into BAND_GRAPH_MODES

    single_channel_index = 0  # 0=TP9, 1=AF7, 2=AF8, 3=TP10
    single_band_seq_index = SINGLE_BAND_SEQUENCE.index("alpha")

    prev_bands = {
        "Delta": [None, None, None, None],
        "Theta": [None, None, None, None],
        "Alpha": [None, None, None, None],
        "Beta":  [None, None, None, None],
        "Gamma": [None, None, None, None],
    }

    while RUNNING:
        now = time.time()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                RUNNING = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    RUNNING = False
                elif event.key == pygame.K_g:
                    graph_enabled = not graph_enabled
                    print(f"[UI] Graph enabled: {graph_enabled}")
                elif event.key == pygame.K_b:
                    band_mode_index = (band_mode_index + 1) % len(BAND_GRAPH_MODES)
                    mode_name, mode_bands = BAND_GRAPH_MODES[band_mode_index]
                    print(f"[UI] Band graph mode: {mode_name} -> {mode_bands}")
                elif event.key == pygame.K_c:
                    graph_combine_mode = "avg" if graph_combine_mode == "max" else "max"
                    print(f"[UI] Graph combine mode: {graph_combine_mode}")
                elif event.key == pygame.K_v:
                    mode_name, _ = BAND_GRAPH_MODES[band_mode_index]
                    if mode_name == "SINGLE":
                        single_band_seq_index = (single_band_seq_index + 1) % len(SINGLE_BAND_SEQUENCE)
                        new_band = SINGLE_BAND_SEQUENCE[single_band_seq_index]
                        BAND_GRAPH_MODES[band_mode_index] = ("SINGLE", [new_band])
                        print(f"[UI] SINGLE band now: {new_band}")
                elif event.key == pygame.K_1:
                    single_channel_index = 0
                    print("[UI] SINGLE channel: TP9")
                elif event.key == pygame.K_2:
                    single_channel_index = 1
                    print("[UI] SINGLE channel: AF7")
                elif event.key == pygame.K_3:
                    single_channel_index = 2
                    print("[UI] SINGLE channel: AF8")
                elif event.key == pygame.K_4:
                    single_channel_index = 3
                    print("[UI] SINGLE channel: TP10")

        mode_name, active_band_keys = BAND_GRAPH_MODES[band_mode_index]

        if graph_enabled and (now - last_graph_sample >= GRAPH_SAMPLE_INTERVAL):
            last_graph_sample = now
            sample_graph_values(now, graph_enabled, graph_combine_mode, active_band_keys, single_channel_index)

        if now - last_text_update >= TEXT_UPDATE_INTERVAL:
            last_text_update = now

            if now - state["artifact_blink_ts"] > ARTIFACT_LATCH_SEC:
                state["artifact_blink"] = 0

            screen.fill(bg_color)

            y = 20
            line_spacing = 24

            title_surf = font.render("AUTOPILOT OSC VIEW", True, accent_color)
            screen.blit(title_surf, (20, y))
            y += line_spacing * 2

            eeg = state["eeg"]
            eeg_text = (
                "EEG [TP9 AF7 AF8 TP10]: "
                f"{fmt(eeg[0])}, {fmt(eeg[1])}, {fmt(eeg[2])}, {fmt(eeg[3])}"
            )
            screen.blit(font.render(eeg_text, True, text_color), (20, y))
            y += line_spacing

            band_rows = [
                ("Delta", "delta"),
                ("Theta", "theta"),
                ("Alpha", "alpha"),
                ("Beta",  "beta"),
                ("Gamma", "gamma"),
            ]

            chan_labels = ["TP9", "AF7", "AF8", "TP10"]
            base_x = 20
            value_start_x = 150
            value_gap_px = 130

            for band_label, band_key in band_rows:
                vals = state[band_key]

                name_color = band_name_colors.get(band_label, text_color)
                label_text = font.render(f"{band_label:>6}:", True, name_color)
                screen.blit(label_text, (base_x, y))

                x = value_start_x
                for ch_idx in range(4):
                    cur_val = vals[ch_idx]
                    prev_val = prev_bands[band_label][ch_idx]

                    if prev_val is None:
                        col = text_color
                    else:
                        if cur_val > prev_val:
                            col = up_color
                        elif cur_val < prev_val:
                            col = down_color
                        else:
                            col = text_color

                    text_str = f"{chan_labels[ch_idx]}={fmt(cur_val)}"
                    val_surf = font.render(text_str, True, col)
                    screen.blit(val_surf, (x, y))

                    prev_bands[band_label][ch_idx] = cur_val
                    x += value_gap_px

                y += line_spacing

            acc = state["acc"]
            acc_text = (
                f"Accel:   x={fmt(acc[0])}  y={fmt(acc[1])}  z={fmt(acc[2])}"
            )
            screen.blit(font.render(acc_text, True, text_color), (20, y))
            y += line_spacing

            gyro = state["gyro"]
            gyro_text = (
                f"Gyro:    x={fmt(gyro[0])}  y={fmt(gyro[1])}  z={fmt(gyro[2])}"
            )
            screen.blit(font.render(gyro_text, True, text_color), (20, y))
            y += line_spacing

            hs = state["horseshoe"]
            hs_labels = [
                hs_label(hs[0]),
                hs_label(hs[1]),
                hs_label(hs[2]),
                hs_label(hs[3]),
            ]

            hs_text = (
                "Horseshoe: "
                f"TP9={hs_labels[0]}  "
                f"AF7={hs_labels[1]}  "
                f"AF8={hs_labels[2]}  "
                f"TP10={hs_labels[3]}"
            )

            if all(lbl == "good" for lbl in hs_labels):
                hs_color = (80, 220, 120)
            elif any(lbl in ("bad", "?") for lbl in hs_labels):
                hs_color = (255, 60, 60)
            else:
                hs_color = text_color

            screen.blit(font.render(hs_text, True, hs_color), (20, y))
            y += line_spacing

            if state["batt_pct"] is None:
                batt_text = "Battery:   n/a"
            else:
                batt_text = (
                    f"Battery:   {state['batt_pct']:.0f}%  "
                    f"{state['batt_volt']:.2f}V  "
                    f"{state['batt_temp']:.1f}°C"
                )
            screen.blit(font.render(batt_text, True, text_color), (20, y))
            y += line_spacing

            art_text = f"Artifacts: blink={state['artifact_blink']}"
            art_color = warn_color if state["artifact_blink"] else text_color
            screen.blit(font.render(art_text, True, art_color), (20, y))
            y += line_spacing

            last_addr = state["last_addr"] or "none"
            la0 = ""
            if state["last_args"]:
                try:
                    la0 = f"{float(state["last_args"][0]):.2f}"
                except Exception:
                    la0 = str(state["last_args"][0])[:8]
            last_text = f"Last msg:  {last_addr}"
            if la0:
                last_text += f"  (arg0={la0} …)"
            screen.blit(font_small.render(last_text, True, text_color), (20, y))
            y += line_spacing

            if state["last_update"] is None:
                age_text = "Last update: n/a"
            else:
                age = now - state["last_update"]
                age_text = f"Last update: {age:.3f}s ago"
            screen.blit(font_small.render(age_text, True, text_color), (20, y))
            y += line_spacing * 2

            footer_text = (
                "ESC=quit  G=graph  B=mode  C=combine  V=single band  1-4=chan  "
                f"OSC {LISTEN_IP}:{LISTEN_PORT}"
            )
            screen.blit(
                font_small.render(footer_text, True, (180, 180, 180)),
                (20, WINDOW_HEIGHT - 40),
            )

        if graph_enabled:
            draw_band_graph(
                screen,
                now,
                font_small,
                active_band_keys,
                mode_name,
                graph_combine_mode,
                single_channel_index,
            )

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


# ================== MAIN ==================

if __name__ == "__main__":
    try:
        print("=" * 60)
        print(f"[MAIN] Starting OSC server on {LISTEN_IP}:{LISTEN_PORT}")
        print("[MAIN] Then launching pygame UI.")
        print("=" * 60)

        start_osc_server_thread(LISTEN_IP, LISTEN_PORT)
        run_pygame_ui()

    finally:
        if osc_server_instance is not None:
            try:
                print("[MAIN] Shutting down OSC server...")
                osc_server_instance.shutdown()
                osc_server_instance.server_close()
            except Exception as e:
                print(f"[MAIN] Error shutting down OSC server: {e}")
        if 'osc_thread' in globals() and osc_thread is not None and osc_thread.is_alive():
            osc_thread.join(timeout=1.0)
        print("[MAIN] Exit.")
