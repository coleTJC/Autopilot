from pythonosc import dispatcher, osc_server
from collections import deque
import time
import datetime
import csv
import os
import math
import serial
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog
from tkinter.scrolledtext import ScrolledText
import tkinter.font as tkfont
from gpiozero import Servo
import pigpio


# ================== CONFIG ==================

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5000
WINDOW_SECONDS = 2.0         # EEG window for band metrics
PRINT_INTERVAL = 0.3
LOG_FILE = "autopilot_log.csv"  # will be overwritten by snapshot path

# Raw EEG band derivation. This alternate branch prefers FFT-derived bands from
# /muse/eeg over Mind Monitor's separate band endpoints once raw bands are ready.
USE_RAW_EEG_DERIVED_BANDS = True
EEG_SAMPLE_RATE = 256.0
FFT_WINDOW_SEC = 1.0
FFT_WINDOW_SAMPLES = int(EEG_SAMPLE_RATE * FFT_WINDOW_SEC)
RAW_EEG_BAND_UPDATE_INTERVAL = 0.10
RAW_EEG_BAND_RANGES = {
    "delta": (1.0, 4.0),
    "theta": (4.0, 8.0),
    "alpha": (8.0, 13.0),
    "beta":  (13.0, 30.0),
    "gamma": (30.0, 45.0),
}

DEFAULT_ARDUINO_PORT = "COM3"
ARDUINO_BAUD = 115200

MIN_FOCUS_DEN = 0.10
MIN_DELTA_DEN = 0.05
MAX_FOCUS_FOR_STATS = 5.0

REWARD_ANGLE = 180
NEUTRAL_ANGLE = 0

OVERLOAD_LIMIT = 50.0        # global-ish overload tolerance
MIN_HISTORY_FOR_REWARD = 10  # warmup length in focus samples

# reward decay half-life (seconds) for servo "memory"
REWARD_DECAY_HALFLIFE = 2.0  # tweakable

ARTIFACT_WINDOW_SECONDS = 0.5     # you said 0.5s
ARTIFACT_MIN_SAMPLES    = 8       # need at least this many points in the window
ARTIFACT_FLAT_EPS       = 0.01    # how "flat" full_band_mean has to be
ARTIFACT_SPIKE_FACTOR   = 2.5     # spike if jump > FACTOR * recent median abs diff
ARTIFACT_HIT_THRESHOLD  = 3       # how many bad frames before flagging

# rolling buffer (seconds of data to keep in RAM)
BUFFER_SECONDS = 10.0


AVAILABLE_MODES = [
    "engaged",
    "calm",
    "recovery",
]

DEFAULT_MODE = "engaged"

# NOTE: hemo now has two columns: pct (0–100) and raw baseline
#       plus per-channel bands + fNIRS channels + HR

CSV_HEADER = [
    "timestamp",

    # global band means (windowed)
    "delta_mean", "theta_mean", "alpha_mean", "beta_mean", "gamma_mean",

    # per-channel band power (instantaneous, TP9/AF7/AF8/TP10)
    "delta_tp9", "delta_af7", "delta_af8", "delta_tp10",
    "theta_tp9", "theta_af7", "theta_af8", "theta_tp10",
    "alpha_tp9", "alpha_af7", "alpha_af8", "alpha_tp10",
    "beta_tp9",  "beta_af7",  "beta_af8",  "beta_tp10",
    "gamma_tp9", "gamma_af7", "gamma_af8", "gamma_tp10",

    # full-band per-channel + hemis
    "full_tp9", "full_af7", "full_af8", "full_tp10",
    "full_band_mean",
    "full_left",
    "full_right",

    # fast hemi metrics
    "left_fast_rel",
    "right_fast_rel",

    # state metrics
    "focus",
    "calm",
    "overload",
    "focus_mean",
    "focus_std",

    # hemo / fNIRS
    "hemo_pct",
    "hemo_raw",
    "hemo_ch1",
    "hemo_ch2",
    "hemo_ch3",
    "hemo_ch4",

    # heart rate
    "hr_bpm",

    # motion
    "motion_level",
    
    # reward + tags
    "reward",
    "bucket",
    "artifact",
    "tag",
]


# ================== $GLOBALS ==================

osc_server_instance = None
osc_thread = None


engine = None             # AutopilotEngine instance
hardware = None           # HardwareController instance
buzzer = None             # BuzzerController instance

last_state_line = ""
last_reward_value = 0.0   # what UI shows (servo-level reward)

RECORD_ENABLED = False    # legacy full-session logging toggle
DEBUG_OSC = False   # set False later if you want to stop the spam


# UI-controlled hardware params (mirrored into HardwareController when it exists)
FULL_THROTTLE = False     # False = phased 3-level, True = binary full send
ANALOG_SERVO = False      # False = discrete, True = analog angle
SERVO_FLOOR = 0.0         # 0..1 fraction of range as minimum angle
AMPLIFY_FACTOR = 1.0      # gain (0.5–3.0 via UI)
SOUND_ENABLED = True      # master toggle for buzzer sound

state_text_height_size = 12		# size of live state feed


# rolling buffer: (t_monotonic, row_list matching CSV_HEADER)
rolling_buffer = deque()

# visualizer history for "Band timeline" mode

VIS_HISTORY_MAX = 300  # ~30s at 100ms update
VIS_MAX_SAMPLES = 120
VIS_HISTORY = deque(maxlen=120)   # ~10–12s of visual history
VIS_BAND_MAX = {                  # per-band max tracking for bars
    "delta": 1.0,
    "theta": 1.0,
    "alpha": 1.0,
    "beta":  1.0,
    "gamma": 1.0,
}

# ===== Timeline event tick controls =====
TIMELINE_MAX_SINGLE_TICKS = 4   # how many reward/interrupt ticks in single mode
TIMELINE_TICK_STRIDE      = 3   # draw every Nth tick in continuous mode
TIMELINE_EVENT_THRESHOLD  = 0.10  # min value to count as an event
# ===== PROFILE BAND TARGETS =====

# How many timeline samples to color around each reward/interrupt edge
# 0 = only that single sample, 1 = one step on each side, etc.
TIMELINE_EVENT_COLOR_RADIUS_STEPS = 0


# Which bands are "positively reinforced" for each profile/mode.
# Key = mode name lowercased (as shown in the Mode dropdown).
# "*" = fallback for anything not explicitly listed.
PROFILE_BAND_TARGETS = {
    "*": {            # default profile
        "delta": 0.0, # not targeted
        "theta": 0.0,
        "alpha": 0.5, # somewhat targeted
        "beta":  1.0, # strongly targeted
        "gamma": 1.0,
    },
    # example if you want later:
    # "focus left": {"beta": 1.0, "gamma": 1.0, "alpha": 0.3},
    # "recovery":   {"alpha": 1.0, "theta": 0.8, "beta": 0.0, "gamma": 0.0},
}

# solid background highlight for reward/interrupt in Band timeline
TIMELINE_EVENT_HALO_MARGIN_PX = 0  # expand each colored segment horizontally by this many pixels

# Width (in pixels) of reward/interrupt highlight stripes in Band timeline
TIMELINE_EVENT_HALO_PX = 3

# ===== TIMELINE TICK CONFIG =====
TIMELINE_REWARD_BASE_H     = 50    # px above bottom for reward ticks
TIMELINE_REWARD_MAX_H      = 100   # max reward tick height (px)

TIMELINE_INTERRUPT_BASE_H  = 50    # px above bottom for interrupt ticks
TIMELINE_INTERRUPT_MAX_H   = 100   # max interrupt tick height (px)






# ================== HELPERS ==================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def bell(x, center, width):
    d = (x - center) / (width + 1e-6)
    return math.exp(-d * d)


def ensure_log_header():
    global LOG_FILE
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)


def append_to_rolling_buffer(row):
    """Keep a sliding BUFFER_SECONDS window of rows in RAM."""
    now = time.monotonic()
    rolling_buffer.append((now, row))
    cutoff = now - BUFFER_SECONDS
    while rolling_buffer and rolling_buffer[0][0] < cutoff:
        rolling_buffer.popleft()


# ================== WINDOWS (EEG / HEMO / REWARD) ==================

class EEGWindow:
    """
    Time window over band-averaged power (across channels).
    Used for classic focus/calm/overload metrics.
    """
    def __init__(self, window_seconds: float):
        self.window_seconds = window_seconds
        self.bands = {
            "delta": deque(),
            "theta": deque(),
            "alpha": deque(),
            "beta": deque(),
            "gamma": deque(),
        }

    def add_sample(self, band_name: str, values):
        now = time.monotonic()
        vals = [float(v) for v in values]
        avg_val = sum(vals) / len(vals) if vals else 0.0
        self.bands[band_name].append((now, avg_val))
        self._prune_old(now)

    def _prune_old(self, now: float):
        cutoff = now - self.window_seconds
        for dq in self.bands.values():
            while dq and dq[0][0] < cutoff:
                dq.popleft()

    def mean(self, band_name: str) -> float:
        dq = self.bands[band_name]
        return (sum(v for _, v in dq) / len(dq)) if dq else 0.0

    def all_means(self):
        return {b: self.mean(b) for b in self.bands.keys()}


class HemoWindow:
    """
    Holds hemo samples over a time window and exposes:
    - raw mean
    - normalized 0–100% score based on local min/max
    """
    def __init__(self, window_seconds: float):
        self.window_seconds = window_seconds
        self.samples = deque()  # (t, avg_val)

    def add_sample(self, values):
        now = time.monotonic()
        vals = [float(v) for v in values]
        avg_val = sum(vals) / len(vals) if vals else 0.0
        self.samples.append((now, avg_val))
        self._prune_old(now)

    def _prune_old(self, now: float):
        cutoff = now - self.window_seconds
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def get_values(self):
        """
        Returns (hemo_raw, hemo_pct)
        - hemo_raw: mean of window
        - hemo_pct: 0–100, normalized within current window min/max
        """
        if not self.samples:
            return 0.0, 0.0

        vals = [v for _, v in self.samples]
        mean_raw = sum(vals) / len(vals)
        min_v = min(vals)
        max_v = max(vals)

        if max_v - min_v < 1e-6:
            # flat window → just park at 50 if >0, else 0
            hemo_pct = 50.0 if mean_raw > 0 else 0.0
        else:
            frac = (mean_raw - min_v) / (max_v - min_v)
            hemo_pct = clamp(frac, 0.0, 1.0) * 100.0

        return mean_raw, hemo_pct


class RewardWindow:
    """
    Holds last ~1–2s of instantaneous reward-ish scores.
    Lets us compute: fraction of recent samples that look 'good'.
    """
    def __init__(self, window_seconds=1.5, min_samples=3):
        self.window_seconds = window_seconds
        self.min_samples = min_samples
        self.buffer = deque()  # (t, sample, base_score)

    def add(self, t, sample, base_score):
        self.buffer.append((t, sample, base_score))
        self._trim_old(t)

    def _trim_old(self, now):
        cutoff = now - self.window_seconds
        while self.buffer and self.buffer[0][0] < cutoff:
            self.buffer.popleft()

    def is_ready(self):
        return len(self.buffer) >= self.min_samples

    def match_fraction(self, thresh: float):
        """
        Fraction of samples whose base_score >= thresh.
        """
        if not self.buffer:
            return 0.0
        good = 0
        total = 0
        for _, _, base in self.buffer:
            total += 1
            if base >= thresh:
                good += 1
        return good / max(1, total)

    def avg_base_score(self):
        if not self.buffer:
            return 0.0
        total = sum(base for _, _, base in self.buffer)
        return total / len(self.buffer)


# ================== REWARD ENGINE (profiles + quality gate) ==================


def compute_penalty_signal():
    """
    Returns a 0–1 'penalty' based on the current engine state.

    High when:
    - overload is high (fried)
    - very low focus + slow-heavy (drowsy / fog)
    - wrong hemisphere is hot for the current mode
    - fast is misaligned with calm in calm/recovery

    NOTE: does NOT use artifact_flag – artifacts should not be punished.
    Also updates FORMULA_TRACK['penalty'] for the UI.
    """
    if engine is None or engine.last_scores is None:
        update_formula_track("penalty", "engaged", {})
        return 0.0

    s, focus_mean, focus_std = engine.last_scores
    mode = getattr(engine, "mode_name", "engaged").lower().strip()

    p = 0.0

    overload = s.get("overload", 0.0)
    focus = s.get("focus", 0.0)
    calm = s.get("calm", 0.0)
    fast_total = s.get("fast_total", 0.0)
    slow_total = s.get("slow_total", 0.0)
    left_fast_rel = s.get("left_fast_rel", 0.5)
    right_fast_rel = s.get("right_fast_rel", 0.5)

    # components for UI
    over_pen = 0.0
    fog_pen = 0.0
    hemi_mismatch = 0.0
    calm_fast_pen = 0.0

    # 1) overload
    if overload > 5.0:
        over_pen = clamp((overload - 5.0) / 5.0, 0.0, 1.0)
        p = max(p, over_pen)

    # 2) drowsy / fog
    if focus < 0.25 and slow_total > fast_total * 1.3:
        fog_pen = 0.6
        p = max(p, fog_pen)

    # 3) mode-specific hemi mistakes
    if fast_total >= 0.05:
        if mode == "engaged":
            if right_fast_rel - left_fast_rel > 0.18:
                hemi_mismatch = 0.7
                p = max(p, hemi_mismatch)
        elif mode == "calm":
            if left_fast_rel - right_fast_rel > 0.18:
                hemi_mismatch = 0.7
                p = max(p, hemi_mismatch)
        elif mode == "recovery":
            if abs(left_fast_rel - right_fast_rel) > 0.22:
                hemi_mismatch = 0.7
                p = max(p, hemi_mismatch)

    # 4) anti-pattern: high fast but low calm in calm/recovery
    if mode in ("calm", "recovery"):
        if calm < 0.3 and fast_total > slow_total:
            calm_fast_pen = 0.5
            p = max(p, calm_fast_pen)

    # push to UI tracker
    update_formula_track(
        "penalty",
        mode,
        {
            "overload_pen": clamp(over_pen, 0.0, 1.0),
            "fog_pen": clamp(fog_pen, 0.0, 1.0),
            "hemi_mismatch": clamp(hemi_mismatch, 0.0, 1.0),
            "calm_fast_pen": clamp(calm_fast_pen, 0.0, 1.0),
        },
    )

    return clamp(p, 0.0, 1.0)



def quality_gate(s, overload_limit):
    """
    Hard gate only on overload; slow_rel is handled as a soft penalty in profiles.
    """
    return s["overload"] <= overload_limit
    
# ============ FORMULA VISUALIZATION (GLOBAL STATE) ============

# Tracks current per-term values for reward & penalty (interrupt)
FORMULA_TRACK = {
    "reward": {
        "mode": "",
        "terms": {}  # id -> {"value": float, "fulfilled": bool, "fail_until": float}
    },
    "penalty": {
        "mode": "",
        "terms": {}
    },
}

# Config for how to show the formulas.
# You can add entries for new profiles by key = profile name (lowercase).
# "*" is the default layout for any profile that doesn't define its own.
FORMULA_CONFIG = {
    "*": {
        "reward": {
            "expr": "Reward = focus × arousal × (0.7 + 0.3·alpha) × (1 - 0.4·drowse) × (1 - 0.5·overload) × (1 + 0.25·hemi)",
            "terms": [
                {"id": "focus_term",    "label": "β/θ focus",       "thr": 0.6},
                {"id": "arousal_term",  "label": "fast arousal",    "thr": 0.6},
                {"id": "alpha_term",    "label": "α support",       "thr": 0.6},
                {"id": "drowse_pen",    "label": "drowse low",      "thr": 0.5},  # we invert penalty
                {"id": "overload_pen",  "label": "overload low",    "thr": 0.5},
                {"id": "hemi_boost",    "label": "hemi target",     "thr": 0.5},
            ],
        },
        "penalty": {
            "expr": "Interrupt = max(overload, fog, hemi error, calm/fast error)",
            "terms": [
                {"id": "overload_pen",  "label": "overload high",    "thr": 0.4},
                {"id": "fog_pen",       "label": "drowsy / fog",     "thr": 0.6},
                {"id": "hemi_mismatch", "label": "wrong hemisphere", "thr": 0.6},
                {"id": "calm_fast_pen", "label": "fast vs calm",     "thr": 0.5},
            ],
        },
    },
    # later you can add:
    # "engaged": {...}, "calm": {...}, "recovery": {...}
}


def _get_formula_layout(kind: str, mode_name: str):
    """
    kind: 'reward' or 'penalty'
    mode_name: engine mode, e.g. 'engaged'
    """
    m = (mode_name or "").lower().strip() or "engaged"
    cfg = FORMULA_CONFIG.get(m, FORMULA_CONFIG.get("*", {}))
    return cfg.get(kind, {"expr": "", "terms": []})


def update_formula_track(kind: str, mode_name: str, term_values: dict):
    """
    kind: 'reward' or 'penalty'
    term_values: id -> 0–1 scalar for that term
    Handles:
    - marking fulfilled (green)
    - red flash for ~1s when a previously fulfilled term drops
    """
    now = time.monotonic()
    if kind not in FORMULA_TRACK:
        return

    track = FORMULA_TRACK[kind]
    track["mode"] = mode_name.lower().strip()
    terms_state = track["terms"]

    layout = _get_formula_layout(kind, track["mode"])
    layout_terms = layout.get("terms", [])

    # build easy lookup for thresholds
    thr_map = {t["id"]: float(t.get("thr", 0.6)) for t in layout_terms}

    # update or create entries
    for tid, val in term_values.items():
        val = float(max(0.0, min(1.0, val)))
        thr = thr_map.get(tid, 0.6)

        prev = terms_state.get(tid, {"value": 0.0, "fulfilled": False, "fail_until": 0.0})
        was_fulfilled = prev.get("fulfilled", False)

        # for reward terms that are inverted penalties (like drowse_pen):
        # we treat 'fulfillment' as val >= thr; you can encode that by flipping val upstream.
        fulfilled = val >= thr

        fail_until = prev.get("fail_until", 0.0)
        # red flash trigger: was fulfilled, now not fulfilled
        if was_fulfilled and not fulfilled:
            fail_until = now + 1.0  # 1 second red flash

        terms_state[tid] = {
            "value": val,
            "fulfilled": fulfilled,
            "fail_until": fail_until,
        }

    # remove terms no longer present (optional, but keeps it clean)
    for tid in list(terms_state.keys()):
        if tid not in term_values:
            # decay any old fail_until
            st = terms_state[tid]
            if now > (st.get("fail_until", 0.0) + 1.0):
                del terms_state[tid]



PROFILES = {
    # =========================================================
    # LEFT EXEC / PERFORMANCE MODE
    # =========================================================
    "engaged": {
        "name": "Left Executive Engagement",
        "hemi_target": "left",
        # Easier buckets: low hits sooner, full reward doesn’t require perfection
        "bucket_edges":  [0.0, 0.30, 0.55, 0.80],
        "bucket_values": [0.0, 0.30, 0.65, 1.0],
        # Lower base_thresh → window-level match doesn’t need to be insanely high
        "base_thresh": 0.30,
        # Allow more overload before we hard-gate
        "overload_limit": OVERLOAD_LIMIT * 0.55,
    },

    # =========================================================
    # RIGHT INSIGHT / CALM INTEGRATION MODE
    # =========================================================
    "calm": {
        "name": "Right Insight & Recovery",
        "hemi_target": "right",
        "bucket_edges":  [0.0, 0.25, 0.55, 0.80],
        "bucket_values": [0.0, 0.30, 0.60, 1.0],
        "base_thresh": 0.25,
        "overload_limit": OVERLOAD_LIMIT * 0.20,
    },

    "recovery": {
        "name": "Recovery / Neurometabolic Reset",
        "hemi_target": "balanced",  # just for your own reference

        # match_fraction buckets:
        #   <0.25 → none
        #   0.25–0.50 → low
        #   0.50–0.75 → mid
        #   >0.75 → full
        "bucket_edges": [0.0, 0.25, 0.50, 0.75],
        "bucket_values": [0.0, 0.35, 0.70, 1.0],

        # easier to get some reward in recovery
        "base_thresh": 0.25,

        # stricter overload cap → if you're really fried, stop rewarding
        "overload_limit": OVERLOAD_LIMIT * 0.35,
    },
}


def compute_instant_base(s, z, mode_name: str):
    """
    Instantaneous 0–1 score based on current mode.
    This is what we feed into RewardWindow as 'base_score'.

    Also updates FORMULA_TRACK['reward'] for the UI.
    """
    delta = s["delta"]
    theta = s["theta"]
    alpha = s["alpha"]
    beta = s["beta"]
    gamma = s["gamma"]
    overload = s["overload"]

    left_fast_rel = s["left_fast_rel"]
    right_fast_rel = s["right_fast_rel"]

    total = delta + theta + alpha + beta + gamma + 1e-6
    slow = delta + theta
    fast = beta + gamma

    slow_rel = slow / total
    alpha_rel = alpha / total
    fast_rel = fast / total
    beta_theta_ratio = beta / (theta + 1e-6)

    mode = mode_name.lower().strip()
    if mode not in PROFILES:
        mode = "engaged"

    # components to send to formula tracker
    terms = {
        "focus_term": 0.0,
        "arousal_term": 0.0,
        "alpha_term": 0.0,
        "drowse_pen": 0.0,
        "overload_pen": 0.0,
        "hemi_boost": 0.0,
        # optional ones used in calm/recovery
        "low_fast": 0.0,
        "slow_ok": 0.0,
        "fast_term": 0.0,
        "slow_term": 0.0,
        "hemi_bal_term": 0.0,
        "z_term": 1.0,
    }

    # ===================== ENGAGED =====================
    if mode == "engaged":
        # 1) focus: β/θ ratio
        focus_term = clamp((beta_theta_ratio - 0.5) / 1.0, 0.0, 1.0)

        # 2) arousal: fast_rel slightly left of center
        arousal_term = bell(fast_rel, center=0.40, width=0.20)

        # 3) alpha
        alpha_term = bell(alpha_rel, center=0.18, width=0.12)

        # 4) drowse penalty: slow-heavy
        drowse_pen = clamp((slow_rel - 0.45) / 0.30, 0.0, 1.0)

        base = focus_term * arousal_term * (0.7 + 0.3 * alpha_term)
        base *= (1.0 - 0.4 * drowse_pen)

        # 5) overload soft safety
        eng_limit = PROFILES["engaged"]["overload_limit"]
        overload_pen = 0.0
        if overload > eng_limit:
            overload_pen = clamp((overload - eng_limit) / (eng_limit * 2.5), 0.0, 1.0)
            base *= (1.0 - 0.5 * overload_pen)

        # 6) hemisphere bias
        hemi_bias = left_fast_rel - right_fast_rel
        hemi_boost = clamp((hemi_bias - 0.02) / 0.30, 0.0, 1.0)
        base *= (1.0 + 0.25 * hemi_boost)

        terms.update({
            "focus_term": focus_term,
            "arousal_term": arousal_term,
            "alpha_term": alpha_term,
            "drowse_pen": drowse_pen,
            "overload_pen": overload_pen,
            "hemi_boost": hemi_boost,
        })

        base = clamp(base, 0.0, 1.0)

    # ===================== CALM =====================
    elif mode == "calm":
        alpha_term = clamp((alpha_rel - 0.18) / 0.20, 0.0, 1.0)
        low_fast = bell(fast_rel, center=0.20, width=0.12)
        slow_ok = bell(slow_rel, center=0.40, width=0.20)

        base = alpha_term * low_fast * slow_ok

        if z > 1.0:
            base *= 0.4

        calm_limit = PROFILES["calm"]["overload_limit"]
        overload_pen = 0.0
        if overload > calm_limit:
            overload_pen = 1.0
            base *= 0.7

        terms.update({
            "alpha_term": alpha_term,
            "low_fast": low_fast,
            "slow_ok": slow_ok,
            "overload_pen": overload_pen,
        })

        base = clamp(base, 0.0, 1.0)

    # ===================== RECOVERY =====================
    elif mode == "recovery":
        alpha_term = bell(alpha_rel, center=0.28, width=0.10)
        fast_term = bell(fast_rel, center=0.25, width=0.15)
        slow_term = bell(slow_rel, center=0.45, width=0.20)
        hemi_bal_term = bell(left_fast_rel, center=0.50, width=0.12)

        if z > 1.0:
            z_term = 0.4
        elif z < -2.0:
            z_term = 0.5
        else:
            z_term = 1.0

        base = alpha_term * fast_term * slow_term * (0.6 + 0.4 * hemi_bal_term)
        base *= z_term

        rec_limit = PROFILES["recovery"]["overload_limit"]
        overload_pen = 0.0
        if overload > rec_limit:
            overload_pen = clamp((overload - rec_limit) / (rec_limit * 2.0), 0.0, 1.0)
            base *= (1.0 - 0.7 * overload_pen)

        terms.update({
            "alpha_term": alpha_term,
            "fast_term": fast_term,
            "slow_term": slow_term,
            "hemi_bal_term": hemi_bal_term,
            "z_term": z_term,
            "overload_pen": overload_pen,
        })

        base = clamp(base, 0.0, 1.0)

    else:
        # fallback calm-style
        alpha_term = clamp((alpha_rel - 0.18) / 0.20, 0.0, 1.0)
        low_fast = bell(fast_rel, center=0.20, width=0.12)
        slow_ok = bell(slow_rel, center=0.40, width=0.20)

        base = alpha_term * low_fast * slow_ok

        if z > 1.0:
            base *= 0.4

        calm_limit = PROFILES["calm"]["overload_limit"]
        overload_pen = 0.0
        if overload > calm_limit:
            overload_pen = 1.0
            base *= 0.3

        terms.update({
            "alpha_term": alpha_term,
            "low_fast": low_fast,
            "slow_ok": slow_ok,
            "overload_pen": overload_pen,
        })

        base = clamp(base, 0.0, 1.0)

    # push to formula HUD
    update_formula_track("reward", mode, terms)
    return base



def compute_reward_for_sample(sample, z, mode_name, reward_window: RewardWindow):
    """
    Returns (reward_value 0–1, bucket_label, tag_component, debug).
    Pure logic: no hardware, no UI.
    """
    mode = mode_name.lower().strip()
    if mode not in PROFILES:
        mode = "engaged"

    profile = PROFILES[mode]

    # 1) global quality gate (overload only)
    if not quality_gate(sample, profile["overload_limit"]):
        now = time.monotonic()
        reward_window.add(now, sample, 0.0)
        return 0.0, "none", "BAD_STATE", {"reason": "overload_gate_fail"}

    # 2) instant base
    base = compute_instant_base(sample, z, mode)
    now = time.monotonic()
    reward_window.add(now, sample, base)

    # 3) warmup
    if not reward_window.is_ready():
        return 0.0, "none", "WARMUP", {"match_frac": 0.0, "base": base}

    # 4) window-level match
    match_frac = reward_window.match_fraction(profile["base_thresh"])

    edges = profile["bucket_edges"]
    values = profile["bucket_values"]

    bucket_idx = 0
    for i, edge in enumerate(edges):
        if match_frac >= edge:
            bucket_idx = i

    reward_value = values[bucket_idx]
    bucket_label = ["none", "low", "mid", "high"][bucket_idx]

    debug = {
        "match_frac": match_frac,
        "base": base,
        "bucket_idx": bucket_idx,
        "avg_base": reward_window.avg_base_score(),
    }

    return reward_value, bucket_label, bucket_label.upper(), debug


# ================== CORE ENGINE ==================

class AutopilotEngine:
    """
    Core logic: takes EEG / hemo / HR inputs, maintains windows,
    computes scores + reward, logs state. No hardware.
    """
    def __init__(self, window_seconds: float, mode_name: str = DEFAULT_MODE):
        self.mode_name = mode_name

        self.eeg_window = EEGWindow(window_seconds)
        self.hemo_window = HemoWindow(window_seconds)
        self.reward_window = RewardWindow(window_seconds=1.5, min_samples=4)

        self.last_print_time = 0.0
        self.focus_history = deque(maxlen=300)

        # last state
        self.last_scores = None   # (s, focus_mean, focus_std)
        self.last_tag = "INIT"
        self.last_bucket = "none"
        self.last_reward_value = 0.0  # smoothed 0–1 (engine side)

        # hemisphere UI values from raw EEG
        self.raw_channels = [0.0, 0.0, 0.0, 0.0]
        self.left_brain_value = 0.5
        self.right_brain_value = 0.5
        self.raw_eeg_buffers = [
            deque(maxlen=FFT_WINDOW_SAMPLES),
            deque(maxlen=FFT_WINDOW_SAMPLES),
            deque(maxlen=FFT_WINDOW_SAMPLES),
            deque(maxlen=FFT_WINDOW_SAMPLES),
        ]
        self.raw_bands_ready = False
        self.last_raw_band_time = 0.0
        self.raw_fft_window = [
            0.54 - 0.46 * math.cos((2.0 * math.pi * n) / (FFT_WINDOW_SAMPLES - 1))
            for n in range(FFT_WINDOW_SAMPLES)
        ]
        self.raw_fft_tables = self._build_raw_fft_tables()

        # spectral holder (per-band per-channel)
        self.band_channels = {
            "delta": [0.0, 0.0, 0.0, 0.0],
            "theta": [0.0, 0.0, 0.0, 0.0],
            "alpha": [0.0, 0.0, 0.0, 0.0],
            "beta":  [0.0, 0.0, 0.0, 0.0],
            "gamma": [0.0, 0.0, 0.0, 0.0],
        }

        # artifact detection
        self.artifact_flag = False
        self.artifact_history = deque()  # (t, full_band_mean)
        self.artifact_counter = 0
        self._last_full_band = None
        self._recent_diffs = deque(maxlen=50)  # for adaptive spike threshold
        
                # fNIRS / hemo raw channels & HR
        self.hemo_last_values = []  # raw channels as last seen from /ppg or /optics
        self.heart_rate_bpm = 0.0

        # motion from /muse/acc and /muse/gyro (smoothed magnitude)
        self.motion_level = 0.0


    # ---- raw EEG from /muse/eeg ----

    def _build_raw_fft_tables(self):
        tables = {}
        bin_hz = EEG_SAMPLE_RATE / FFT_WINDOW_SAMPLES
        for band_name, (f_low, f_high) in RAW_EEG_BAND_RANGES.items():
            bins = []
            k_start = max(1, math.ceil(f_low / bin_hz))
            k_stop = max(k_start + 1, math.ceil(f_high / bin_hz))
            for k in range(k_start, k_stop):
                cos_terms = []
                sin_terms = []
                for n in range(FFT_WINDOW_SAMPLES):
                    angle = (2.0 * math.pi * k * n) / FFT_WINDOW_SAMPLES
                    cos_terms.append(math.cos(angle))
                    sin_terms.append(math.sin(angle))
                bins.append((cos_terms, sin_terms))
            tables[band_name] = bins
        return tables

    def _derive_bands_from_raw_eeg(self):
        """
        Use the last FFT_WINDOW_SAMPLES raw EEG samples per channel to compute
        delta/theta/alpha/beta/gamma log power, matching the V2 monitor path.
        """
        if any(len(buf) < FFT_WINDOW_SAMPLES for buf in self.raw_eeg_buffers):
            return None

        channel_data = []
        for buf in self.raw_eeg_buffers:
            vals = [float(v) for v in buf]
            mean_val = sum(vals) / len(vals)
            channel_data.append([
                (vals[i] - mean_val) * self.raw_fft_window[i]
                for i in range(FFT_WINDOW_SAMPLES)
            ])

        derived = {band_name: [] for band_name in RAW_EEG_BAND_RANGES.keys()}
        for ch_idx in range(4):
            samples = channel_data[ch_idx]
            for band_name, bins in self.raw_fft_tables.items():
                powers = []
                for cos_terms, sin_terms in bins:
                    re = 0.0
                    im = 0.0
                    for i, sample in enumerate(samples):
                        re += sample * cos_terms[i]
                        im -= sample * sin_terms[i]
                    powers.append((re * re) + (im * im))

                raw_power = sum(powers) / len(powers) if powers else 0.0
                derived[band_name].append(math.log10(raw_power + 1e-8))

        return derived

    def _update_raw_derived_bands(self):
        now = time.monotonic()
        if now - self.last_raw_band_time < RAW_EEG_BAND_UPDATE_INTERVAL:
            return False

        derived = self._derive_bands_from_raw_eeg()
        if derived is None:
            return False

        self.last_raw_band_time = now
        self.raw_bands_ready = True

        for band_name, values in derived.items():
            self.band_channels[band_name] = values
            self.eeg_window.add_sample(band_name, values)

        return True

    def update_raw_eeg(self, values):
        vals = [float(v) for v in values]
        chans = None
        if len(vals) >= 4:
            chans = vals[:4]
        if chans is None:
            return
        self.raw_channels = chans

        for i in range(4):
            self.raw_eeg_buffers[i].append(chans[i])

        if USE_RAW_EEG_DERIVED_BANDS:
            if self._update_raw_derived_bands():
                self._process_sample()
        else:
            self._process_sample()

    # ---- band endpoints ----

    def update_band(self, band_name: str, values, chan_index=None):
        if USE_RAW_EEG_DERIVED_BANDS and self.raw_bands_ready:
            return

        self.eeg_window.add_sample(band_name, values)

        vals = [float(v) for v in values]
        if not vals:
            return

        if chan_index is not None:
            self.band_channels[band_name][chan_index] = vals[0]
        else:
            if len(vals) >= 4:
                self.band_channels[band_name] = vals[:4]
            elif len(vals) == 2:
                L, R = vals
                self.band_channels[band_name] = [L, L, R, R]
            elif len(vals) == 1:
                v = vals[0]
                self.band_channels[band_name] = [v, v, v, v]
            else:
                base = (vals * 2)[:4]
                self.band_channels[band_name] = base

        self._process_sample()

    def update_hemo(self, values):
        # windowed hemo
        self.hemo_window.add_sample(values)
        # keep last raw fNIRS channels
        self.hemo_last_values = [float(v) for v in values]
        self._process_sample()

    def update_hr(self, values):
        """Update heart rate (BPM) if present."""
        try:
            if values:
                self.heart_rate_bpm = float(values[0])
        except Exception:
            pass
            
    def update_motion(self, addr, values):
        """
        Update a simple motion metric from Muse accelerometer / gyro.
        - /muse/acc: uses |acc|-1g
        - /muse/gyro: uses magnitude scaled down
        Result is smoothed into self.motion_level (0+).
        """
        try:
            vals = [float(v) for v in values]
        except Exception:
            return
        if not vals:
            return

        delta = 0.0
        al = addr.lower()

        if "acc" in al:
            # accelerometer: assume resting ≈ 1g
            if len(vals) >= 3:
                ax, ay, az = vals[:3]
                mag = math.sqrt(ax * ax + ay * ay + az * az)
                delta = abs(mag - 1.0)
        elif "gyro" in al:
            # gyro: magnitude, scaled down
            if len(vals) >= 3:
                gx, gy, gz = vals[:3]
                mag = math.sqrt(gx * gx + gy * gy + gz * gz)
                delta = mag / 500.0

        if delta <= 0.0:
            return

        delta = clamp(delta, 0.0, 5.0)
        alpha = 0.2  # smoothing
        self.motion_level = (1.0 - alpha) * self.motion_level + alpha * delta

            
            
            

    # ===== metrics =====

    def _full_band_per_channel(self):
        full = [0.0, 0.0, 0.0, 0.0]
        for chan in range(4):
            s = 0.0
            for band in ("delta", "theta", "alpha", "beta", "gamma"):
                s += float(self.band_channels[band][chan])
            full[chan] = s / 5.0
        return full

    def _hemi_fast_metrics(self):
        beta = self.band_channels["beta"]
        gamma = self.band_channels["gamma"]

        left_beta = (beta[0] + beta[1]) / 2.0
        right_beta = (beta[2] + beta[3]) / 2.0
        left_gamma = (gamma[0] + gamma[1]) / 2.0
        right_gamma = (gamma[2] + gamma[3]) / 2.0

        left_fast = max(0.0, left_beta + left_gamma)
        right_fast = max(0.0, right_beta + right_gamma)
        total_fast = left_fast + right_fast

        if total_fast < 1e-4:
            left_fast_rel = 0.5
            right_fast_rel = 0.5
        else:
            left_fast_rel = left_fast / total_fast
            right_fast_rel = right_fast / total_fast

        return {
            "left_fast": left_fast,
            "right_fast": right_fast,
            "total_fast": total_fast,
            "left_fast_rel": left_fast_rel,
            "right_fast_rel": right_fast_rel,
        }

    def compute_scores(self):
        m = self.eeg_window.all_means()
        delta = max(0.0, m["delta"])
        theta = max(0.0, m["theta"])
        alpha = max(0.0, m["alpha"])
        beta = max(0.0, m["beta"])
        gamma = max(0.0, m["gamma"])

        # ----- channel-level power -----
        full_tp9, full_af7, full_af8, full_tp10 = self._full_band_per_channel()
        full_left = (full_tp9 + full_af7) / 2.0
        full_right = (full_af8 + full_tp10) / 2.0
        full_band_mean = (full_tp9 + full_af7 + full_af8 + full_tp10) / 4.0

        # ----- fast-hemisphere metrics -----
        hemi_fast = self._hemi_fast_metrics()
        left_fast_rel = hemi_fast["left_fast_rel"]
        right_fast_rel = hemi_fast["right_fast_rel"]
        fast_total = hemi_fast["total_fast"]
        slow_total = delta + theta

        # ===== "real life" state metrics =====
        total_power = max(delta + theta + alpha + beta + gamma, 1e-6)
        rel_delta = delta / total_power
        rel_theta = theta / total_power
        rel_alpha = alpha / total_power
        rel_beta = beta / total_power
        rel_gamma = gamma / total_power

        slow_rel = rel_delta + rel_theta
        fast_rel = rel_beta + rel_gamma

        # ---- FOCUS: fast (β+γ) vs slow (δ+θ), squashed 0–1
        focus_raw = fast_rel / (slow_rel + 1e-3)
        focus = clamp(focus_raw / 3.0, 0.0, 1.0)

        # ---- CALM: good alpha + not too crazy fast
        calm_alpha = bell(rel_alpha, center=0.25, width=0.12)
        calm_fast = bell(fast_rel, center=0.30, width=0.15)
        calm = clamp(calm_alpha * calm_fast, 0.0, 1.0)

        # ---- OVERLOAD: fast vs stabilizing bands (δ+θ+α)
        stabilizing = slow_rel + rel_alpha
        overload_raw = fast_rel / (stabilizing + 1e-3)
        overload = overload_raw * 5.0  # 0–10-ish+ when cooked

        # ---- HEMO: raw + 0–100% from window
        hemo_raw, hemo_pct = self.hemo_window.get_values()

        # update raw hemi UI values
        raw = self.raw_channels
        if len(raw) >= 4:
            L_raw = raw[0] + raw[1]
            R_raw = raw[2] + raw[3]
            total_raw = L_raw + R_raw + 1e-6
            self.left_brain_value = clamp(L_raw / total_raw, 0.0, 1.0)
            self.right_brain_value = clamp(R_raw / total_raw, 0.0, 1.0)

        return {
            "delta": delta,
            "theta": theta,
            "alpha": alpha,
            "beta": beta,
            "gamma": gamma,

            "full_band_mean": full_band_mean,
            "full_left": full_left,
            "full_right": full_right,
            "full_tp9": full_tp9,
            "full_af7": full_af7,
            "full_af8": full_af8,
            "full_tp10": full_tp10,

            "left_fast_rel": left_fast_rel,
            "right_fast_rel": right_fast_rel,

            "focus": focus,        # 0–1
            "calm": calm,          # 0–1
            "overload": overload,  # 0–10-ish

            "hemo": hemo_pct,      # main hemo metric = percent
            "hemo_pct": hemo_pct,  # explicit percent
            "hemo_raw": hemo_raw,  # raw mean from sensor

            "fast_total": fast_total,
            "slow_total": slow_total,
        }

    def _focus_stats(self):
        if not self.focus_history:
            return None, None
        mean = sum(self.focus_history) / len(self.focus_history)
        var = sum((x - mean) ** 2 for x in self.focus_history) / len(self.focus_history)
        return mean, math.sqrt(var)

    def _update_artifact(self, s):
        """
        Update artifact_flag based on:
        - flat full_band_mean over short window
        - big spikes vs recent change stats
        Uses a small hysteresis counter so it doesn't flicker.
        """
        now = time.monotonic()
        fb = float(s["full_band_mean"])

        # history for flat detection
        self.artifact_history.append((now, fb))
        cutoff = now - ARTIFACT_WINDOW_SECONDS
        while self.artifact_history and self.artifact_history[0][0] < cutoff:
            self.artifact_history.popleft()

        # track successive diffs for spike detection
        prev_full_band = self._last_full_band
        if prev_full_band is not None:
            diff = abs(fb - prev_full_band)
            self._recent_diffs.append(diff)

        # not enough samples yet → no artifact
        if len(self.artifact_history) < ARTIFACT_MIN_SAMPLES:
            self._last_full_band = fb
            self.artifact_flag = False
            self.artifact_counter = 0
            return

        vals = [v for _, v in self.artifact_history]
        v_min = min(vals)
        v_max = max(vals)
        flat_range = v_max - v_min

        # flat detector
        is_flat = flat_range < ARTIFACT_FLAT_EPS

        # spike detector (adaptive threshold from median diff)
        if self._recent_diffs:
            sorted_diffs = sorted(self._recent_diffs)
            median_diff = sorted_diffs[len(sorted_diffs) // 2]
            spike_thresh = max(ARTIFACT_FLAT_EPS * 5.0, median_diff * ARTIFACT_SPIKE_FACTOR)
            cur_diff = abs(fb - prev_full_band) if prev_full_band is not None else 0.0
            is_spike = cur_diff > spike_thresh
        else:
            is_spike = False

        self._last_full_band = fb

        bad_frame = is_flat or is_spike

        # hysteresis: accumulate bad frames, decay with good ones
        if bad_frame:
            self.artifact_counter += 1
        else:
            self.artifact_counter = max(0, self.artifact_counter - 1)

        self.artifact_flag = self.artifact_counter >= ARTIFACT_HIT_THRESHOLD

    def _build_csv_row(self, s, focus_mean, focus_std):
        """Assemble one CSV row (matches CSV_HEADER)."""
        # per-channel band power
        def bc(band, idx):
            try:
                return float(self.band_channels[band][idx])
            except Exception:
                return 0.0

        delta_tp9 = bc("delta", 0)
        delta_af7 = bc("delta", 1)
        delta_af8 = bc("delta", 2)
        delta_tp10 = bc("delta", 3)

        theta_tp9 = bc("theta", 0)
        theta_af7 = bc("theta", 1)
        theta_af8 = bc("theta", 2)
        theta_tp10 = bc("theta", 3)

        alpha_tp9 = bc("alpha", 0)
        alpha_af7 = bc("alpha", 1)
        alpha_af8 = bc("alpha", 2)
        alpha_tp10 = bc("alpha", 3)

        beta_tp9 = bc("beta", 0)
        beta_af7 = bc("beta", 1)
        beta_af8 = bc("beta", 2)
        beta_tp10 = bc("beta", 3)

        gamma_tp9 = bc("gamma", 0)
        gamma_af7 = bc("gamma", 1)
        gamma_af8 = bc("gamma", 2)
        gamma_tp10 = bc("gamma", 3)

        hemo_vals = (self.hemo_last_values or [])
        h1 = hemo_vals[0] if len(hemo_vals) > 0 else 0.0
        h2 = hemo_vals[1] if len(hemo_vals) > 1 else 0.0
        h3 = hemo_vals[2] if len(hemo_vals) > 2 else 0.0
        h4 = hemo_vals[3] if len(hemo_vals) > 3 else 0.0

        ts = datetime.datetime.now().isoformat()

        row = [
            ts,

            # global band means
            s["delta"], s["theta"], s["alpha"], s["beta"], s["gamma"],

            # per-channel band power
            delta_tp9, delta_af7, delta_af8, delta_tp10,
            theta_tp9, theta_af7, theta_af8, theta_tp10,
            alpha_tp9, alpha_af7, alpha_af8, alpha_tp10,
            beta_tp9,  beta_af7,  beta_af8,  beta_tp10,
            gamma_tp9, gamma_af7, gamma_af8, gamma_tp10,

            # full per-channel + hemis
            s["full_tp9"], s["full_af7"], s["full_af8"], s["full_tp10"],
            s["full_band_mean"], s["full_left"], s["full_right"],

            # fast hemi
            s["left_fast_rel"], s["right_fast_rel"],

            # state
            s["focus"], s["calm"], s["overload"],
            (focus_mean or 0.0), (focus_std or 0.0),

            # hemo / fNIRS
            s["hemo_pct"], s["hemo_raw"],
            h1, h2, h3, h4,

            # heart rate
            self.heart_rate_bpm,
            
            # motion
            self.motion_level,

            # reward + tags
            self.last_reward_value,
            self.last_bucket,
            int(self.artifact_flag),
            self.last_tag,
        ]

        return row

    def _process_sample(self):
        global last_state_line, LOG_FILE, RECORD_ENABLED

        s = self.compute_scores()
        focus = s["focus"]
        self.focus_history.append(focus)
        focus_mean, focus_std = self._focus_stats()

        # update artifact detection
        self._update_artifact(s)

        if self.artifact_flag:
            # hard clamp during artifact
            self.last_reward_value = 0.0
            self.last_bucket = "none"
            self.last_tag = "ARTIFACT"
            self.last_scores = (s, focus_mean, focus_std)
        else:
            # warmup: fill history before any RL
            if (
                focus_mean is None
                or focus_std is None
                or len(self.focus_history) < MIN_HISTORY_FOR_REWARD
            ):
                self.last_reward_value = 0.0
                self.last_bucket = "none"
                self.last_tag = "WARMUP"
            else:
                z = (focus - focus_mean) / (focus_std + 1e-6)

                reward_raw, bucket, tag_component, debug = compute_reward_for_sample(
                    s, z, self.mode_name, self.reward_window
                )

                # apply global gain and smoothing here (core engine)
                reward_raw *= AMPLIFY_FACTOR
                reward_raw = clamp(reward_raw, 0.0, 1.0)

                alpha_smooth = 0.3
                self.last_reward_value = (
                    (1.0 - alpha_smooth) * self.last_reward_value
                    + alpha_smooth * reward_raw
                )
                self.last_bucket = bucket

                # semantic tags
                left_fast_rel = s["left_fast_rel"]
                right_fast_rel = s["right_fast_rel"]
                fast_total = s.get("fast_total", 0.0)

                hemi_tag = ""

                # only label drift/engaged if there's enough fast power
                if fast_total >= 0.05:
                    if left_fast_rel > 0.62 and right_fast_rel < 0.38:
                        hemi_tag = "LEFT_ENGAGED"
                    elif right_fast_rel > 0.62 and left_fast_rel < 0.38:
                        hemi_tag = "RIGHT_ENGAGED"
                    elif 0.46 < left_fast_rel < 0.54:
                        hemi_tag = "BALANCED"

                base_tag = f"{tag_component}(r={self.last_reward_value:.2f})"

                if hemi_tag:
                    self.last_tag = f"{hemi_tag}|{base_tag}"
                else:
                    self.last_tag = base_tag

        self.last_scores = (s, focus_mean, focus_std)

        now = time.monotonic()
        if now - self.last_print_time >= PRINT_INTERVAL:
            self.last_print_time = now

            s, focus_mean, focus_std = self.last_scores

            # ----- build vertical, labeled status block -----
            line_parts = []

            line_parts.append("[STATE]")

            signal_status = "ARTIFACT" if self.artifact_flag else "OK"
            line_parts.append(f"Signal: {signal_status}")

            # Bands
            line_parts.append(
                "Bands : "
                f"Δ(Delta)={s['delta']:.3f}  "
                f"Θ(Theta)={s['theta']:.3f}  "
                f"α(Alpha)={s['alpha']:.3f}  "
                f"β(Beta)={s['beta']:.3f}  "
                f"γ(Gamma)={s['gamma']:.3f}"
            )

            # Power
            line_parts.append(
                "Power : "
                f"Global={s['full_band_mean']:.3f}  "
                f"Left={s['full_left']:.3f}  "
                f"Right={s['full_right']:.3f}"
            )

            # Hemispheres
            line_parts.append(
                "Hemi  : "
                f"Fast L={s['left_fast_rel']:.2f}  "
                f"Fast R={s['right_fast_rel']:.2f}  "
                f"Raw L={self.left_brain_value:.2f}  "
                f"Raw R={self.right_brain_value:.2f}"
            )

                        # State  (Hemo now as percent + raw + motion)
            line_parts.append(
                "State : "
                f"Focus={s['focus']:.2f}  "
                f"Calm={s['calm']:.2f}  "
                f"Overload={s['overload']:.2f}  "
                f"Hemo={s['hemo_pct']:.0f}% (raw={s['hemo_raw']:.3f})  "
                f"Motion={self.motion_level:.3f}"
            )



            # Stats
            line_parts.append(
                "Stats : "
                f"μ={(focus_mean or 0):.2f}  "
                f"σ={(focus_std or 0):.2f}"
            )

            # Tags
            line_parts.append(
                f"Tags  : {self.last_tag}"
            )

            # Join as multiline block for the UI
            line = "\n".join(line_parts)
            last_state_line = line

        # ====== build row & feed logging paths ======

        row = self._build_csv_row(s, focus_mean, focus_std)

        # rolling buffer always on (only last BUFFER_SECONDS kept)
        append_to_rolling_buffer(row)

        # optional legacy full-session logging (if you ever want it)
        if RECORD_ENABLED:
            ensure_log_header()
            with open(LOG_FILE, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(row)


# ================== HARDWARE LAYER (reward + interrupt via pigpio) ==================

class HardwareController:
    """
    Smooth, velocity-limited servo controller using pigpio hardware-timed PWM.

    - Main servo on GPIO18 = reward level (good state).
    - Interrupt servo on GPIO17 = penalty / "bad state" signal.
      Fires when suboptimal patterns hit (overload, wrong hemi, fog, etc.).
      Artifacts do NOT drive this servo.
    """

    CMD_INTERVAL   = 0.01   # rate-limit main angle updates
    
    MAX_LEVEL_STEP = 0.06   # smaller = slower reward memory

    # single speed knob for both servos
    SERVO_STEP = 0.02       # how fast BOTH servos chase their targets

    
    

    # pulse widths for both servos
    MIN_PW = 500
    MAX_PW = 2500

    # pins (BCM numbering)
    MAIN_PIN = 18
    INT_PIN  = 17

    # interrupt decay (seconds to half)
    INTERRUPT_HALFLIFE = 0.5  # tweak if you want longer/shorter "sting"

    def __init__(self, neutral_angle=0, reward_angle=180):
        self.neutral_angle = neutral_angle
        self.reward_angle = reward_angle

        # interrupt servo smoothing
        self.interrupt_servo_level = 0.0
        self.interrupt_servo_target = 0.0

        

        # pigpio init
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("[HW] ERROR: could not connect to pigpio daemon")
            self.pi = None
        else:
            # main reward servo
            self.pi.set_mode(self.MAIN_PIN, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(self.MAIN_PIN, 0)

            # interrupt servo
            self.pi.set_mode(self.INT_PIN, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(self.INT_PIN, 0)

            print("[HW] pigpio servos on GPIO18 (main) + GPIO17 (interrupt)")

        self.last_angle_sent = None
        self.last_int_angle_sent = None

        # UI controlled flags
        self.full_throttle = FULL_THROTTLE
        self.analog_servo = ANALOG_SERVO
        self.servo_floor = SERVO_FLOOR

        # reward memory / smoothing
        self.decay_level = 0.0
        self.last_engine_reward = 0.0
        now = time.monotonic()
        self.last_time = now
        self.last_update_time = now

        # reward-level smoothing
        self.current_level = 0.0
        self.target_level = 0.0

        # servo-level smoothing (reward)
        self.servo_level = 0.0
        self.servo_target = 0.0

        # interrupt state
        self.interrupt_level = 0.0    # 0–1 penalty intensity
        self.last_penalty = 0.0       # last raw penalty input

        if REWARD_DECAY_HALFLIFE > 0:
            self.decay_lambda = math.log(2.0) / REWARD_DECAY_HALFLIFE
        else:
            self.decay_lambda = 0.0

        if self.INTERRUPT_HALFLIFE > 0:
            self.interrupt_lambda = math.log(2.0) / self.INTERRUPT_HALFLIFE
        else:
            self.interrupt_lambda = 0.0

    def connect(self, port: str):
        """
        Kept for UI compatibility; Arduino COM is ignored here.
        """
        print(f"[HW] Using pigpio on GPIO18/17. Ignoring COM port '{port}'.")

    def _angle_to_pulse(self, angle: int) -> int:
        """Map 0–180° to MIN_PW..MAX_PW."""
        angle = int(clamp(angle, 0, 180))
        frac = angle / 180.0  # 0..1
        pw = int(self.MIN_PW + frac * (self.MAX_PW - self.MIN_PW))
        return pw
    
    def _level_to_angle(self, level: float) -> int:
        """
        Map a 0–1 level into the floored servo range for BOTH servos.
        Respects self.servo_floor, neutral_angle, reward_angle.
        """
        lvl = clamp(level, 0.0, 1.0)
        floor = clamp(self.servo_floor, 0.0, 0.95)

        # shared min/max for both main + interrupt
        min_angle = self.neutral_angle + floor * (self.reward_angle - self.neutral_angle)
        max_angle = self.reward_angle

        angle = min_angle + lvl * (max_angle - min_angle)
        return int(clamp(angle, 0, 180))
    

    def _send_main_angle(self, angle: int):
        if not self.pi:
            return
        angle = int(clamp(angle, 0, 180))
        if angle == self.last_angle_sent:
            return
        self.last_angle_sent = angle
        pw = self._angle_to_pulse(angle)
        self.pi.set_servo_pulsewidth(self.MAIN_PIN, pw)

    def _send_interrupt_angle(self, angle: int):
        if not self.pi:
            return
        angle = int(clamp(angle, 0, 180))
        if angle == self.last_int_angle_sent:
            return
        self.last_int_angle_sent = angle
        pw = self._angle_to_pulse(angle)
        self.pi.set_servo_pulsewidth(self.INT_PIN, pw)

    def update_from_ui(self):
        self.full_throttle = FULL_THROTTLE
        self.analog_servo = ANALOG_SERVO
        self.servo_floor = SERVO_FLOOR

    def apply_reward(self, reward_value: float):
        """
        Main loop for hardware:
        - Smooth reward signal into main servo movement.
        - Compute penalty and drive interrupt servo.
        - Fire buzzer tones on reward spikes / interrupt hits.
        """
        global last_reward_value, buzzer, SOUND_ENABLED

        self.update_from_ui()

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # ======== MAIN REWARD PATH ========

        # 1) decay stored reward (reward memory)
        if self.decay_lambda > 0 and dt > 0:
            decay_factor = math.exp(-self.decay_lambda * dt)
            self.decay_level *= decay_factor

        # 2) incoming reward from engine
        r = clamp(float(reward_value), 0.0, 1.0)

        # treat rising edges as proper hits
        if r > self.last_engine_reward + 0.05 and r > 0.15:
            self.decay_level = clamp(self.decay_level + r, 0.0, 1.0)

            # HIGH TONE on reward spike
            if SOUND_ENABLED and buzzer is not None:
                try:
                    buzzer.reward_tone()
                except Exception as e:
                    print(f"[HW] buzzer reward_tone error: {e}")

        self.last_engine_reward = r

        # 3) target_level from decay_level
        self.target_level = clamp(self.decay_level, 0.0, 1.0)

        # 4) current_level chases target_level
        if self.target_level > self.current_level:
            self.current_level = min(
                self.current_level + self.MAX_LEVEL_STEP,
                self.target_level
            )
        else:
            self.current_level = max(
                self.current_level - self.MAX_LEVEL_STEP,
                self.target_level
            )

        v = self.current_level  # 0..1

        # ======== PENALTY / INTERRUPT PATH ========

        # 5) decay interrupt level (how long the "sting" lingers)
        if self.interrupt_lambda > 0 and dt > 0:
            int_decay = math.exp(-self.interrupt_lambda * dt)
            self.interrupt_level *= int_decay

        # 6) compute penalty from engine state (already ignores artifacts)
        penalty = compute_penalty_signal()  # 0–1

        # shield: if reward is high, downscale penalty further
        if engine is not None:
            good_reward = getattr(engine, "last_reward_value", 0.0)
        else:
            good_reward = 0.0

        if good_reward > 0.6:
            penalty *= 0.3
        elif good_reward > 0.4:
            penalty *= 0.6

        # Only "hit" when penalty meaningfully rises.
        rising = penalty > self.last_penalty + 0.05

        if penalty > 0.20 and rising:
            if penalty > 0.70:
                hit_level = 1.0          # full interrupt
            elif penalty > 0.40:
                hit_level = 0.65         # strong
            else:
                hit_level = 0.35         # light cue

            self.interrupt_level = max(self.interrupt_level, hit_level)

            # LOW TONE on interrupt hit
            if SOUND_ENABLED and buzzer is not None:
                try:
                    buzzer.interrupt_tone()
                except Exception as e:
                    print(f"[HW] buzzer interrupt_tone error: {e}")

        self.last_penalty = penalty

        # ======== MAIN + INTERRUPT SERVO OUTPUTS ========

        if now - self.last_update_time >= self.CMD_INTERVAL:
            self.last_update_time = now

            # ----- MAIN SERVO TARGET FROM REWARD v -----
            if self.analog_servo:
                self.servo_target = v
            else:
                if self.full_throttle:
                    self.servo_target = 0.0 if v < 0.3 else 1.0
                else:
                    if v < 0.2:
                        bucket = 0
                    elif v < 0.5:
                        bucket = 1
                    elif v < 0.8:
                        bucket = 2
                    else:
                        bucket = 3
                    self.servo_target = bucket / 3.0

            step = self.SERVO_STEP  # 1:1 speed for both servos

            # ----- MAIN SERVO SMOOTHING -----
            if self.servo_target > self.servo_level:
                self.servo_level = min(self.servo_level + step, self.servo_target)
            else:
                self.servo_level = max(self.servo_level - step, self.servo_target)

            level_for_angle = clamp(self.servo_level, 0.0, 1.0)

            # MAIN SERVO OUTPUT
            if v < 0.05 and self.servo_floor <= 0.0:
                # Only fully detach if floor is zero and reward basically off
                last_reward_value = 0.0
                if self.pi:
                    self.pi.set_servo_pulsewidth(self.MAIN_PIN, 0)
                    self.last_angle_sent = None
            else:
                main_angle = self._level_to_angle(level_for_angle)
                self._send_main_angle(main_angle)

            # ======== INTERRUPT SERVO OUTPUT (INDEPENDENT OF v) ========

            # target is just the current interrupt_level 0–1
            self.interrupt_servo_target = clamp(self.interrupt_level, 0.0, 1.0)

            # smooth toward target at same rate
            if self.interrupt_servo_target > self.interrupt_servo_level:
                self.interrupt_servo_level = min(
                    self.interrupt_servo_level + step,
                    self.interrupt_servo_target
                )
            else:
                self.interrupt_servo_level = max(
                    self.interrupt_servo_level - step,
                    self.interrupt_servo_target
                )

            # INTERRUPT OUTPUT
            if self.interrupt_servo_level < 0.01 and self.servo_floor <= 0.0:
                # Very tiny + no floor → detach
                if self.pi:
                    self.pi.set_servo_pulsewidth(self.INT_PIN, 0)
                    self.last_int_angle_sent = None
            else:
                int_angle = self._level_to_angle(self.interrupt_servo_level)
                self._send_interrupt_angle(int_angle)

        # still track reward level for the UI bar
        last_reward_value = v



    def reset_servos(self):
        """Hard reset: clear levels and stop PWM on both servos."""
        # internal state
        self.current_level = 0.0
        self.target_level = 0.0
        self.servo_level = 0.0
        self.servo_target = 0.0
        self.decay_level = 0.0
        self.last_engine_reward = 0.0

        self.interrupt_level = 0.0
        self.interrupt_servo_level = 0.0
        self.interrupt_servo_target = 0.0

        # stop pulses on both pins
        if self.pi:
            try:
                self.pi.set_servo_pulsewidth(self.MAIN_PIN, 0)
                self.pi.set_servo_pulsewidth(self.INT_PIN, 0)
            except Exception as e:
                print(f"[HW] reset_servos error: {e}")

        self.last_angle_sent = None
        self.last_int_angle_sent = None

    def smooth_reset(self, duration=0.6, steps=20):
        """
        Smoothly move BOTH servos (main + interrupt) back to neutral,
        then detach (pulse = 0). Call from a background thread.
        """
        if not self.pi:
            return

        start_main = self.last_angle_sent if self.last_angle_sent is not None else self.neutral_angle
        start_int  = self.last_int_angle_sent if self.last_int_angle_sent is not None else self.neutral_angle

        for i in range(steps + 1):
            frac = i / steps

            main_angle = start_main + (self.neutral_angle - start_main) * frac
            int_angle  = start_int  + (self.neutral_angle - start_int)  * frac

            main_pw = self._angle_to_pulse(int(main_angle))
            int_pw  = self._angle_to_pulse(int(int_angle))

            try:
                self.pi.set_servo_pulsewidth(self.MAIN_PIN, main_pw)
                self.pi.set_servo_pulsewidth(self.INT_PIN, int_pw)
            except Exception as e:
                print(f"[HW] smooth_reset step error: {e}")
                break

            time.sleep(duration / steps)

        try:
            self.pi.set_servo_pulsewidth(self.MAIN_PIN, 0)
            self.pi.set_servo_pulsewidth(self.INT_PIN, 0)
        except Exception as e:
            print(f"[HW] smooth_reset detach error: {e}")

        self.current_level = 0.0
        self.target_level = 0.0
        self.servo_level = 0.0
        self.servo_target = 0.0
        self.decay_level = 0.0
        self.last_engine_reward = 0.0

        self.interrupt_level = 0.0
        self.interrupt_servo_level = 0.0
        self.interrupt_servo_target = 0.0

        self.last_angle_sent = None
        self.last_int_angle_sent = None


# ================== BUZZER LAYER ==================

class BuzzerController:
    """
    Simple passive buzzer driver on a single GPIO pin using pigpio PWM.

    - High tone = reward (good hit).
    - Low tone  = interrupt (penalty).

    All tones run in tiny background threads so they don't block UI or servo logic.
    """

    def __init__(self, pin=27):
        self.pin = pin
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("[HW] ERROR: could not connect to pigpio daemon for buzzer")
            self.pi = None
        else:
            self.pi.set_mode(self.pin, pigpio.OUTPUT)
            self.pi.set_PWM_dutycycle(self.pin, 0)
            print(f"[HW] Buzzer ready on GPIO{self.pin}")

    def _tone(self, freq: int, duration: float, duty: int = 220):
        """Fire a tone in a non-blocking way."""
        if not self.pi:
            return

        def _worker():
            try:
                self.pi.set_PWM_frequency(self.pin, freq)
                self.pi.set_PWM_dutycycle(self.pin, duty)
                time.sleep(duration)
            finally:
                # always shut off PWM
                try:
                    self.pi.set_PWM_dutycycle(self.pin, 0)
                except Exception:
                    pass

        threading.Thread(target=_worker, daemon=True).start()

    # ----- public helpers -----

    def reward_tone(self):
        """High, short 'ding' for reward."""
        self._tone(freq=2000, duration=0.3, duty=230)

    def interrupt_tone(self):
        """Lower, slightly longer tone for interrupt / penalty."""
        self._tone(freq=300, duration=0.5, duty=230)

    def stop(self):
        """Turn off PWM and release pigpio."""
        if not self.pi:
            return
        try:
            self.pi.set_PWM_dutycycle(self.pin, 0)
        except Exception:
            pass
        self.pi.stop()
        self.pi = None
        print("[HW] Buzzer stopped")


# ================== OSC LAYER ==================

def osc_handler(address, *args):
    global engine, hardware
    if engine is None:
        return

    addr = str(address).lower()

    if "/muse/eeg" in addr:
        engine.update_raw_eeg(args)
        if hardware is not None:
            hardware.apply_reward(engine.last_reward_value)
        return

    # motion endpoints (accel + gyro)
    if "/muse/acc" in addr or "/muse/gyro" in addr:
        engine.update_motion(addr, args)
        if hardware is not None:
            hardware.apply_reward(engine.last_reward_value)
        return

    chan_index = None
    if "tp9" in addr:
        chan_index = 0
    elif "af7" in addr:
        chan_index = 1
    elif "af8" in addr:
        chan_index = 2
    elif "tp10" in addr:
        chan_index = 3

    if "delta" in addr:
        engine.update_band("delta", args, chan_index)
    elif "theta" in addr:
        engine.update_band("theta", args, chan_index)
    elif "alpha" in addr:
        engine.update_band("alpha", args, chan_index)
    elif "beta" in addr:
        engine.update_band("beta", args, chan_index)
    elif "gamma" in addr:
        engine.update_band("gamma", args, chan_index)
    elif "ppg" in addr or "optics" in addr or "hemo" in addr:
        engine.update_hemo(args)
    elif "hr" in addr or "bpm" in addr or "heart" in addr:
        engine.update_hr(args)

    if hardware is not None:
        hardware.apply_reward(engine.last_reward_value)



def start_osc_server(ip: str, port: int, mode_name: str):
    global osc_server_instance, osc_thread, engine

    if osc_server_instance is not None:
        print("[AUTOPILOT] Server already running.")
        return

    disp = dispatcher.Dispatcher()
    disp.set_default_handler(osc_handler)

    osc_server_instance = osc_server.BlockingOSCUDPServer(
        (ip, port),
        disp
    )

    engine = AutopilotEngine(window_seconds=WINDOW_SECONDS, mode_name=mode_name)

    def server_loop():
        print(f"[AUTOPILOT] Listening on {ip}:{port}")
        print("[AUTOPILOT] Ready.\n")
        osc_server_instance.serve_forever()

    osc_thread = threading.Thread(target=server_loop, daemon=True)
    osc_thread.start()


def stop_osc_server():
    global osc_server_instance, osc_thread, engine, hardware
    if osc_server_instance is not None:
        print("[AUTOPILOT] Stopping server...")
        try:
            osc_server_instance.shutdown()
        except Exception:
            pass
        osc_server_instance = None
        osc_thread = None
        engine = None

    # clean up hardware / servo so we can re-init on next Start
    if hardware is not None:
        try:
            servo_obj = getattr(hardware, "servo", None)
            if servo_obj is not None:
                try:
                    servo_obj.value = None   # detach
                except Exception:
                    pass
                try:
                    servo_obj.close()       # free GPIO18
                except Exception:
                    pass
        except Exception as e:
            print(f"[HW] Error while closing servo: {e}")
        hardware = None

    print("[AUTOPILOT] Server stopped.")


# ================== TK UI ==================

def run_ui():
    global LISTEN_IP, LISTEN_PORT, LOG_FILE
    global RECORD_ENABLED, FULL_THROTTLE, ANALOG_SERVO, SERVO_FLOOR, AMPLIFY_FACTOR
    global last_state_line, last_reward_value, hardware, engine, SOUND_ENABLED, buzzer
    global FORMULA_TRACK

    root = tk.Tk()
    root.title("Autopilot v0.5 UI")
    root.geometry("1200x580")
    root.minsize(600, 600)

    style = ttk.Style()
    try:
        style.theme_use("clam")
    except Exception:
        pass

    style.configure(
        "Reward.Horizontal.TProgressbar",
        troughcolor="#202020",
        background="#00cc44",
    )

    style.configure(
        "Interrupt.Horizontal.TProgressbar",
        troughcolor="#202020",
        background="#cc0044",
    )

    # ---------- TK VARS ----------
    mode_var = tk.StringVar(value=DEFAULT_MODE)
    ip_var = tk.StringVar(value=LISTEN_IP)
    port_var = tk.StringVar(value=str(LISTEN_PORT))
    com_var = tk.StringVar(value=DEFAULT_ARDUINO_PORT)
    log_path_var = tk.StringVar(value=LOG_FILE)

    full_var = tk.BooleanVar(value=FULL_THROTTLE)
    analog_var = tk.BooleanVar(value=ANALOG_SERVO)
    servo_floor_var = tk.DoubleVar(value=SERVO_FLOOR * 100.0)   # %
    amplify_var = tk.DoubleVar(value=AMPLIFY_FACTOR * 100.0)    # %
    sound_var = tk.BooleanVar(value=SOUND_ENABLED)

    status_var = tk.StringVar(value="Stopped")
    record_status_var = tk.StringVar(value="Recording: OFF")
    reward_var = tk.DoubleVar(value=0.0)
    interrupt_var = tk.DoubleVar(value=0.0)

    # visualizer vars
    visual_enable_var = tk.BooleanVar(value=False)
    visual_mode_var = tk.StringVar(value="Band timeline")
    vis_delta_var = tk.BooleanVar(value=True)
    vis_theta_var = tk.BooleanVar(value=True)
    vis_alpha_var = tk.BooleanVar(value=True)
    vis_beta_var = tk.BooleanVar(value=True)
    vis_gamma_var = tk.BooleanVar(value=True)
    vis_ticks_on_var = tk.BooleanVar(value=True)     # master tick on/off
    vis_tick_single_var = tk.BooleanVar(value=False) # single peak tick mode
    
    # visual event highlighting toggles
    vis_event_halo_var = tk.BooleanVar(value=False)   # background stripes OFF by default
    vis_band_recolor_var = tk.BooleanVar(value=True)  # band recolor ON by default




    pad = 6

    # ---------- SCROLLABLE MAIN AREA ----------
    # root just owns a single scrollable area now
    root.rowconfigure(0, weight=1)
    root.columnconfigure(0, weight=1)

    outer = ttk.Frame(root)
    outer.grid(row=0, column=0, sticky="nsew", padx=pad, pady=pad)

    outer.rowconfigure(0, weight=1)
    outer.columnconfigure(0, weight=1)

    # canvas + vertical scrollbar
    main_canvas = tk.Canvas(outer, highlightthickness=0, bd=0)
    vscroll = ttk.Scrollbar(outer, orient="vertical", command=main_canvas.yview)
    main_canvas.configure(yscrollcommand=vscroll.set)

    main_canvas.grid(row=0, column=0, sticky="nsew")
    vscroll.grid(row=0, column=1, sticky="ns")

    # inner frame that actually holds the UI
    main_frame = ttk.Frame(main_canvas)
    main_window_id = main_canvas.create_window(
        (0, 0), window=main_frame, anchor="nw"
    )

    # 1) frame resize → update scrollregion only
    def _on_frame_configure(event):
        main_canvas.configure(scrollregion=main_canvas.bbox("all"))

    # 2) canvas resize → force inner frame width to match canvas width
    def _on_canvas_configure(event):
        main_canvas.itemconfigure(main_window_id, width=event.width)

    main_frame.bind("<Configure>", _on_frame_configure)
    main_canvas.bind("<Configure>", _on_canvas_configure)
    
    # ---------- MOUSE WHEEL SCROLLING ----------
    
    # how many "units" to move per wheel notch
    SCROLL_UNITS = 2  # try 5; bump to 8–10 if you want it faster

    def _on_mousewheel(event):
        # event.delta is ±120 per notch on Windows/macOS
        if event.delta != 0:
            direction = -1 if event.delta > 0 else 1
            main_canvas.yview_scroll(direction * SCROLL_UNITS, "units")

    def _on_linux_scroll(event):
        # Linux: Button-4 (up) / Button-5 (down)
        if event.num == 4:
            main_canvas.yview_scroll(-SCROLL_UNITS, "units")
        elif event.num == 5:
            main_canvas.yview_scroll(SCROLL_UNITS, "units")


    # bind to the whole app so any wheel in this window scrolls the canvas
    root.bind_all("<MouseWheel>", _on_mousewheel, add="+")
    root.bind_all("<Button-4>", _on_linux_scroll, add="+")
    root.bind_all("<Button-5>", _on_linux_scroll, add="+")




    # main_frame is the new "root" for all the other frames
    main_frame.columnconfigure(0, weight=1)
    main_frame.columnconfigure(1, weight=1)
    main_frame.rowconfigure(0, weight=0)  # top row (Reward Profile)
    main_frame.rowconfigure(1, weight=0)  # cfg/control row
    main_frame.rowconfigure(2, weight=1)  # Live State row stretches





    # ---------- REWARD PROFILE ----------
    top_frame = ttk.LabelFrame(main_frame, text="Reward Profile")
    top_frame.grid(row=0, column=0, columnspan=2,
                   sticky="ew", padx=pad, pady=(pad, 0))
    for c in range(8):
        top_frame.columnconfigure(c, weight=0)
    top_frame.columnconfigure(1, weight=1)

    ttk.Label(top_frame, text="Mode:").grid(
        row=0, column=0, sticky="w", padx=pad, pady=pad
    )
    

    mode_menu = ttk.OptionMenu(
        top_frame, mode_var, mode_var.get(), *AVAILABLE_MODES
    )
    mode_menu.grid(row=0, column=1, sticky="ew", padx=pad, pady=pad)
    
    



    def on_full_change(*_):
        global FULL_THROTTLE
        FULL_THROTTLE = full_var.get()

    def on_analog_change(*_):
        global ANALOG_SERVO
        ANALOG_SERVO = analog_var.get()

    def on_servo_floor_change(*_):
        global SERVO_FLOOR
        try:
            v = float(servo_floor_var.get()) / 100.0
        except Exception:
            v = 0.0
        SERVO_FLOOR = clamp(v, 0.0, 0.95)

    def on_amplify_change(*_):
        global AMPLIFY_FACTOR
        try:
            v = float(amplify_var.get()) / 100.0
        except Exception:
            v = 1.0
        AMPLIFY_FACTOR = clamp(v, 0.5, 3.0)

    def on_sound_change(*_):
        global SOUND_ENABLED
        SOUND_ENABLED = sound_var.get()

    full_var.trace_add("write", on_full_change)
    analog_var.trace_add("write", on_analog_change)
    servo_floor_var.trace_add("write", on_servo_floor_change)
    amplify_var.trace_add("write", on_amplify_change)
    sound_var.trace_add("write", on_sound_change)

    ttk.Checkbutton(
        top_frame,
        text="Full throttle (binary reward)",
        variable=full_var
    ).grid(row=0, column=2, sticky="w", padx=pad, pady=pad)
    



    ttk.Checkbutton(
        top_frame,
        text="Continuous servo (analog reward)",
        variable=analog_var
    ).grid(row=0, column=3, sticky="w", padx=pad, pady=pad)

    ttk.Label(top_frame, text="Servo floor (%):").grid(
        row=0, column=4, sticky="e", padx=(pad * 2, 2), pady=pad
    )
    tk.Spinbox(
        top_frame,
        from_=0, to=95, increment=5, width=4,
        textvariable=servo_floor_var
    ).grid(row=0, column=5, sticky="w", padx=(0, pad), pady=pad)

    ttk.Label(top_frame, text="Reward gain (%):").grid(
        row=0, column=6, sticky="e", padx=(pad, 2), pady=pad
    )
    tk.Spinbox(
        top_frame,
        from_=50, to=300, increment=10, width=5,
        textvariable=amplify_var
    ).grid(row=0, column=7, sticky="w", padx=(0, pad), pady=pad)

    ttk.Checkbutton(
        top_frame,
        text="Sound on (buzzer)",
        variable=sound_var
    ).grid(row=1, column=0, sticky="w", padx=pad, pady=(0, pad))

    # ---------- CONNECTION & LOGGING (left) ----------
    cfg_frame = ttk.LabelFrame(main_frame, text="Connection & Logging")
    cfg_frame.grid(row=1, column=0, sticky="nsew", padx=pad, pady=pad)
    for c in range(2):
        cfg_frame.columnconfigure(c, weight=1)

    ttk.Label(cfg_frame, text="Listen IP:").grid(row=0, column=0, sticky="w", padx=pad, pady=(pad, 2))
    ttk.Entry(cfg_frame, textvariable=ip_var).grid(row=0, column=1, sticky="ew", padx=pad, pady=(pad, 2))

    ttk.Label(cfg_frame, text="Listen Port:").grid(row=1, column=0, sticky="w", padx=pad, pady=2)
    ttk.Entry(cfg_frame, textvariable=port_var, width=10).grid(row=1, column=1, sticky="ew", padx=pad, pady=2)

    ttk.Label(cfg_frame, text="Arduino COM:").grid(row=2, column=0, sticky="w", padx=pad, pady=2)
    ttk.Entry(cfg_frame, textvariable=com_var, width=10).grid(row=2, column=1, sticky="ew", padx=pad, pady=2)

    ttk.Label(cfg_frame, text="Last saved CSV:").grid(row=3, column=0, sticky="w", padx=pad, pady=2)
    log_entry = ttk.Entry(cfg_frame, textvariable=log_path_var)
    log_entry.grid(row=3, column=1, sticky="ew", padx=pad, pady=2)

    def on_browse():
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if path:
            log_path_var.set(path)

    ttk.Button(cfg_frame, text="Browse…", command=on_browse).grid(
        row=4, column=1, sticky="w", padx=pad, pady=(2, pad)
    )

    # ---------- CONTROL (right) ----------
    ctrl_frame = ttk.LabelFrame(main_frame, text="Control")
    ctrl_frame.grid(row=1, column=1, sticky="nsew", padx=(0, pad), pady=pad)
    for c in range(2):
        ctrl_frame.columnconfigure(c, weight=1)

    def on_start():
        global LISTEN_PORT, LISTEN_IP, LOG_FILE, hardware, engine, buzzer

        if osc_server_instance is not None:
            messagebox.showinfo("Autopilot", "Server already running.")
            return

        try:
            port_val = int(port_var.get())
        except ValueError:
            messagebox.showerror("Autopilot", "Listen Port must be an integer.")
            return

        mode_name = mode_var.get().strip() or DEFAULT_MODE
        LISTEN_PORT = port_val
        LISTEN_IP = ip_var.get().strip()
        LOG_FILE = log_path_var.get().strip() or "autopilot_log.csv"

        hardware = HardwareController(
            neutral_angle=NEUTRAL_ANGLE,
            reward_angle=REWARD_ANGLE
        )
        com_port = com_var.get().strip() or DEFAULT_ARDUINO_PORT
        hardware.connect(com_port)

        # set up buzzer on GPIO27 when starting, if not already
        if buzzer is None:
            try:
                buzzer_local = BuzzerController(pin=27)
                buzzer = buzzer_local
            except Exception as e:
                print(f"[HW] Failed to init buzzer: {e}")

        start_osc_server(LISTEN_IP, LISTEN_PORT, mode_name=mode_name)
        mode_txt = (
            f"{mode_name} | "
            f"{'FULL' if FULL_THROTTLE else 'PHASED'} | "
            f"{'ANALOG' if ANALOG_SERVO else 'DISCRETE'}"
        )
        status_var.set(f"Running ({mode_txt})")

    def on_stop():
        global hardware, buzzer

        status_var.set("Stopping...")

        hw_ref = hardware
        buzz_ref = buzzer

        # clear globals so UI knows we're stopping
        hardware = None
        buzzer = None

        def _worker():
            # graceful servo shutdown if possible
            if hw_ref is not None:
                try:
                    if hasattr(hw_ref, "smooth_reset"):
                        hw_ref.smooth_reset(duration=0.6, steps=20)
                    elif hasattr(hw_ref, "reset_servos"):
                        hw_ref.reset_servos()
                except Exception as e:
                    print(f"[UI] hardware stop failed: {e}")

            # buzzer shutdown
            if buzz_ref is not None:
                try:
                    if hasattr(buzz_ref, "stop"):
                        buzz_ref.stop()
                except Exception as e:
                    print(f"[UI] buzzer stop failed: {e}")

            stop_osc_server()

        threading.Thread(target=_worker, daemon=True).start()

    def on_record_toggle():
        # legacy full-session logging toggle
        global RECORD_ENABLED, LOG_FILE
        RECORD_ENABLED = not RECORD_ENABLED
        if RECORD_ENABLED:
            LOG_FILE = log_path_var.get().strip() or LOG_FILE
            record_status_var.set("Recording: ON (legacy)")
        else:
            record_status_var.set("Recording: OFF")

    def on_capture():
        """Capture last BUFFER_SECONDS from rolling buffer into ~/Documents."""
        if not rolling_buffer:
            messagebox.showinfo("Autopilot", "No data in rolling buffer yet.")
            return

        name = simpledialog.askstring(
            "Capture rolling window",
            f"Name for last {int(BUFFER_SECONDS)}s CSV (no extension):"
        )
        if name is None:
            return

        safe = "".join(c for c in name if c.isalnum() or c in ("_", "-", " ")).strip()
        if not safe:
            safe = "autopilot"

        ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{safe}_{ts_str}.csv"
        docs_dir = os.path.expanduser("~/Documents")
        try:
            os.makedirs(docs_dir, exist_ok=True)
        except Exception:
            pass
        path = os.path.join(docs_dir, filename)

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
            # write the current window only
            for _, row in list(rolling_buffer):
                writer.writerow(row)

        log_path_var.set(path)
        messagebox.showinfo("Autopilot", f"Saved last {int(BUFFER_SECONDS)}s to:\n{path}")

    btn_row = ttk.Frame(ctrl_frame)
    btn_row.grid(row=0, column=0, columnspan=2, sticky="ew", padx=pad, pady=(pad, 2))
    for i in range(4):
        btn_row.columnconfigure(i, weight=1)

    ttk.Button(btn_row, text="Start", command=on_start).grid(
        row=0, column=0, padx=pad, pady=2, sticky="ew"
    )
    ttk.Button(btn_row, text="Stop", command=on_stop).grid(
        row=0, column=1, padx=pad, pady=2, sticky="ew"
    )
    ttk.Button(btn_row, text="Record On/Off", command=on_record_toggle).grid(
        row=0, column=2, padx=pad, pady=2, sticky="ew"
    )
    ttk.Button(btn_row, text="Capture last 10s…", command=on_capture).grid(
        row=0, column=3, padx=pad, pady=2, sticky="ew"
    )

    ttk.Label(ctrl_frame, text="Status:").grid(row=1, column=0, sticky="w", padx=pad, pady=(4, 2))
    ttk.Label(ctrl_frame, textvariable=status_var).grid(row=1, column=1, sticky="w", padx=pad, pady=(4, 2))

    ttk.Label(ctrl_frame, textvariable=record_status_var).grid(
        row=2, column=0, columnspan=2, sticky="w", padx=pad, pady=(0, 4)
    )

    # ---------- FORMULA HUD (reward / interrupt) ----------
    # Toggle to show/hide the formula panel so it doesn't dominate the Control box
    formula_visible_var = tk.BooleanVar(value=False)

    def toggle_formula_visibility():
        if formula_visible_var.get():
            formula_frame.grid(row=4, column=0, columnspan=2, sticky="nsew", padx=pad, pady=(0, pad))
        else:
            formula_frame.grid_remove()

    ttk.Checkbutton(
        ctrl_frame,
        text="Show formulas",
        variable=formula_visible_var,
        command=toggle_formula_visibility,
    ).grid(row=3, column=0, columnspan=2, sticky="w", padx=pad, pady=(0, 2))

    formula_frame = ttk.LabelFrame(ctrl_frame, text="Reward / Interrupt Formula")
    formula_frame.grid(row=4, column=0, columnspan=2, sticky="nsew", padx=pad, pady=(0, pad))
    formula_frame.columnconfigure(0, weight=1)

    ttk.Label(formula_frame, text="Current reward formula:").grid(
        row=0, column=0, sticky="w", padx=pad, pady=(4, 0)
    )

    formula_font_size_var = tk.IntVar(value=10)

    # safe background color for Text widgets
    style_obj = ttk.Style()
    frame_bg = style_obj.lookup("TFrame", "background")
    if not frame_bg:
        frame_bg = root.cget("bg")

    reward_formula_text = tk.Text(
        formula_frame,
        height=2,
        wrap="word",
        font=("Consolas", formula_font_size_var.get()),
        bd=0,
        highlightthickness=0,
    )
    reward_formula_text.grid(row=1, column=0, sticky="ew", padx=pad, pady=(0, 4))
    reward_formula_text.configure(state="disabled", bg=frame_bg)

    ttk.Label(formula_frame, text="Current interrupt formula:").grid(
        row=2, column=0, sticky="w", padx=pad, pady=(0, 0)
    )

    interrupt_formula_text = tk.Text(
        formula_frame,
        height=2,
        wrap="word",
        font=("Consolas", formula_font_size_var.get()),
        bd=0,
        highlightthickness=0,
    )
    interrupt_formula_text.grid(row=3, column=0, sticky="ew", padx=pad, pady=(0, 4))
    interrupt_formula_text.configure(state="disabled", bg=frame_bg)

    # simple font size slider (only used if formula gets cramped)
    def on_formula_font_change(*_):
        try:
            fs = int(float(formula_font_size_var.get()))
        except Exception:
            fs = 10

        # apply to both Text widgets
        if reward_formula_text is not None:
            reward_formula_text.configure(font=("Consolas", fs))
        if interrupt_formula_text is not None:
            interrupt_formula_text.configure(font=("Consolas", fs))

    font_row = ttk.Frame(formula_frame)
    font_row.grid(row=4, column=0, sticky="ew", padx=pad, pady=(0, 4))
    ttk.Label(font_row, text="Formula size:").grid(row=0, column=0, sticky="w")
    ttk.Scale(
        font_row,
        from_=8,
        to=16,
        variable=formula_font_size_var,
        command=lambda _v: on_formula_font_change(),
        orient="horizontal",
    ).grid(row=0, column=1, sticky="ew", padx=(4, 0))
    font_row.columnconfigure(1, weight=1)

    # hide by default until the toggle is turned on
    formula_frame.grid_remove()

    
    # safe background color for Text widgets
    style_obj = ttk.Style()
    frame_bg = style_obj.lookup("TFrame", "background")
    if not frame_bg:
        frame_bg = root.cget("bg")


    reward_formula_text = tk.Text(
        formula_frame,
        height=2,
        wrap="word",
        font=("Consolas", formula_font_size_var.get()),
        bd=0,
        highlightthickness=0,
    )
    reward_formula_text.grid(row=1, column=0, sticky="ew", padx=pad, pady=(0, 4))
    reward_formula_text.configure(state="disabled", bg=frame_bg)

    ttk.Label(formula_frame, text="Current interrupt formula:").grid(
        row=2, column=0, sticky="w", padx=pad, pady=(0, 0)
    )

    interrupt_formula_text = tk.Text(
        formula_frame,
        height=2,
        wrap="word",
        font=("Consolas", formula_font_size_var.get()),
        bd=0,
        highlightthickness=0,
    )
    interrupt_formula_text.grid(row=3, column=0, sticky="ew", padx=pad, pady=(0, 4))
    interrupt_formula_text.configure(state="disabled", bg=frame_bg)

    # simple font size slider (only used if formula gets cramped)

    def on_formula_font_change(*_):
        try:
            fs = int(float(formula_font_size_var.get()))
        except Exception:
            fs = 10
        fs = max(8, min(16, fs))
        # use plain tuples, no shared named font
        reward_formula_text.configure(font=("Consolas", fs))
        interrupt_formula_text.configure(font=("Consolas", fs))


    font_row = ttk.Frame(formula_frame)
    font_row.grid(row=4, column=0, sticky="ew", padx=pad, pady=(0, 4))
    ttk.Label(font_row, text="Formula size:").grid(row=0, column=0, sticky="w")
    ttk.Scale(
        font_row,
        from_=8,
        to=16,
        variable=formula_font_size_var,
        command=lambda _v: on_formula_font_change(),
        orient="horizontal",
    ).grid(row=0, column=1, sticky="ew", padx=(4, 0))
    font_row.columnconfigure(1, weight=1)

    # ---------- BRAINWAVE VISUALIZER (under Control) ----------
    visual_frame = ttk.LabelFrame(ctrl_frame, text="Brainwave Visualizer")


    def on_visual_toggle():
        if visual_enable_var.get():
            visual_frame.grid(row=4, column=0, columnspan=2, sticky="nsew", padx=pad, pady=(0, pad))
        else:
            visual_frame.grid_remove()
            
    # Math visualizer toggle lives in Reward Profile, but uses this handler
    ttk.Checkbutton(
        top_frame,
        text="Math visualizer (bands → image)",
        variable=visual_enable_var,
        command=on_visual_toggle,
    ).grid(row=2, column=0, columnspan=4, sticky="w", padx=pad, pady=(4, 6))




    # visualizer controls
    vis_ctrl_row = ttk.Frame(visual_frame)
    vis_ctrl_row.grid(row=0, column=0, sticky="ew", padx=pad, pady=(pad, 2))
    vis_ctrl_row.columnconfigure(0, weight=1)
    vis_ctrl_row.columnconfigure(1, weight=1)

    ttk.Label(vis_ctrl_row, text="Mode:").grid(row=0, column=0, sticky="w", padx=(0, 4), pady=2)
    ttk.OptionMenu(
        vis_ctrl_row,
        visual_mode_var,
        visual_mode_var.get(),
        "Neuro-primitives",
        "Orbital bloom",
        "Band flower",
        "Band timeline",
    ).grid(row=0, column=1, sticky="ew", pady=2)

    vis_band_row = ttk.Frame(visual_frame)
    vis_band_row.grid(row=1, column=0, sticky="w", padx=pad, pady=(0, 4))

    ttk.Checkbutton(vis_band_row, text="Δ", variable=vis_delta_var).grid(row=0, column=0, padx=2)
    ttk.Checkbutton(vis_band_row, text="Θ", variable=vis_theta_var).grid(row=0, column=1, padx=2)
    ttk.Checkbutton(vis_band_row, text="α", variable=vis_alpha_var).grid(row=0, column=2, padx=2)
    ttk.Checkbutton(vis_band_row, text="β", variable=vis_beta_var).grid(row=0, column=3, padx=2)
    ttk.Checkbutton(vis_band_row, text="γ", variable=vis_gamma_var).grid(row=0, column=4, padx=2)

    # tick controls
    tick_chk = ttk.Checkbutton(
        vis_band_row,
        text="Show event ticks",
        variable=vis_ticks_on_var,
    )
    tick_chk.grid(row=1, column=0, columnspan=5, sticky="w", padx=2, pady=(2, 0))

    single_tick_chk = ttk.Checkbutton(
        vis_band_row,
        text="Single event tick",
        variable=vis_tick_single_var,
    )
    single_tick_chk.grid(row=2, column=0, columnspan=5, sticky="w", padx=18, pady=(0, 0))

    def update_tick_visibility(*_):
        if vis_ticks_on_var.get():
            single_tick_chk.grid()
        else:
            single_tick_chk.grid_remove()

    # keep relation: single checkbox only visible when ticks are on
    vis_ticks_on_var.trace_add("write", update_tick_visibility)
    update_tick_visibility()
    
    # highlight mode toggles
    highlight_row = ttk.Frame(visual_frame)
    highlight_row.grid(row=2, column=0, sticky="w", padx=pad, pady=(0, 4))

    ttk.Checkbutton(
        highlight_row,
        text="Timeline stripes (reward/interrupt)",
        variable=vis_event_halo_var,
    ).grid(row=0, column=0, sticky="w", padx=(0, 8), pady=0)

    ttk.Checkbutton(
        highlight_row,
        text="Color bands on reward/interrupt",
        variable=vis_band_recolor_var,
    ).grid(row=0, column=1, sticky="w", padx=(0, 8), pady=0)



    ttk.Checkbutton(vis_band_row, text="Δ", variable=vis_delta_var).grid(row=0, column=0, padx=2)
    ttk.Checkbutton(vis_band_row, text="Θ", variable=vis_theta_var).grid(row=0, column=1, padx=2)
    ttk.Checkbutton(vis_band_row, text="α", variable=vis_alpha_var).grid(row=0, column=2, padx=2)
    ttk.Checkbutton(vis_band_row, text="β", variable=vis_beta_var).grid(row=0, column=3, padx=2)
    ttk.Checkbutton(vis_band_row, text="γ", variable=vis_gamma_var).grid(row=0, column=4, padx=2)
    ttk.Checkbutton(
        vis_band_row,
        text="Single event tick",
        variable=vis_tick_single_var
    ).grid(row=1, column=0, columnspan=5, sticky="w", padx=2, pady=(2, 0))


    # main + hemi canvases
    vis_canvas_row = ttk.Frame(visual_frame)
    vis_canvas_row.grid(row=3, column=0, sticky="nsew", padx=pad, pady=(0, pad))
    vis_canvas_row.columnconfigure(0, weight=3)
    vis_canvas_row.columnconfigure(1, weight=1)
    vis_canvas_row.rowconfigure(0, weight=1)

    vis_canvas = tk.Canvas(
        vis_canvas_row, bg="#000000",
        height=220, highlightthickness=0, bd=0
    )
    vis_canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

    hemi_canvas = tk.Canvas(
        vis_canvas_row, bg="#050505",
        height=220, width=140, highlightthickness=0, bd=0
    )
    hemi_canvas.grid(row=0, column=1, sticky="nsew", padx=(4, 0))

    visual_frame.rowconfigure(2, weight=1)
    visual_frame.columnconfigure(0, weight=1)

    # start hidden
    on_visual_toggle()

    # ---------- LIVE STATE (bottom) ----------
    state_frame = ttk.LabelFrame(main_frame, text="Live State")
    state_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=pad, pady=(0, pad))
    state_frame.columnconfigure(0, weight=1)
    state_frame.rowconfigure(2, weight=1)  # text is now row=2

    # Reward bar
    ttk.Label(state_frame, text="Reward Level:").grid(
        row=0, column=0, sticky="w", padx=pad, pady=(pad, 0)
    )

    ttk.Progressbar(
        state_frame,
        orient="horizontal",
        mode="determinate",
        style="Reward.Horizontal.TProgressbar",
        maximum=100,
        variable=reward_var
    ).grid(row=0, column=0, sticky="ew", padx=(100, pad), pady=(pad, 0))

    # Interrupt bar (servo angle)
    ttk.Label(state_frame, text="Interrupt Servo (°):").grid(
        row=1, column=0, sticky="w", padx=pad, pady=(4, 0)
    )

    ttk.Progressbar(
        state_frame,
        orient="horizontal",
        mode="determinate",
        style="Interrupt.Horizontal.TProgressbar",
        maximum=180,
        variable=interrupt_var
    ).grid(row=1, column=0, sticky="ew", padx=(100, pad), pady=(2, 2))

    # Live state text
    state_text = ScrolledText(
        state_frame,
        wrap="word",
        height=state_text_height_size,
        font=("Consolas", 11),
    )
    state_text.grid(row=2, column=0, sticky="nsew", padx=pad, pady=pad)
    state_text.insert("end", "No data yet")
    state_text.configure(state="disabled")

    # color tags
    state_text.tag_config("hemi_left", foreground="#00cc00")
    state_text.tag_config("hemi_right", foreground="#ff3333")
    state_text.tag_config("hemi_bal", foreground="#00cccc")
    state_text.tag_config("bad_state", foreground="#ff00ff")
    state_text.tag_config("warmup", foreground="#aaaa00")
    state_text.tag_config("artifact", foreground="#ff0000")
    state_text.tag_config("bucket_none", foreground="#888888")
    state_text.tag_config("bucket_low", foreground="#33aaff")
    state_text.tag_config("bucket_mid", foreground="#ffaa33")
    state_text.tag_config("bucket_high", foreground="#ff33ff")

    

    # ---------- tagging helpers ----------
    def apply_tag(pattern, tagname):
        start = "1.0"
        while True:
            idx = state_text.search(pattern, start, "end")
            if not idx:
                break
            end = f"{idx}+{len(pattern)}c"
            state_text.tag_add(tagname, idx, end)
            start = end

    def apply_all_tags():
        apply_tag("LEFT_ENGAGED", "hemi_left")
        apply_tag("RIGHT_ENGAGED", "hemi_right")
        apply_tag("RIGHT_DRIFT", "hemi_right")
        apply_tag("BALANCED", "hemi_bal")
        apply_tag("ARTIFACT", "artifact")
        apply_tag("BAD_STATE", "bad_state")
        apply_tag("WARMUP", "warmup")
        apply_tag("NONE(", "bucket_none")
        apply_tag("LOW(", "bucket_low")
        apply_tag("MID(", "bucket_mid")
        apply_tag("HIGH(", "bucket_high")

        # if we're currently in an artifact, also paint the Motion field red
        if engine is not None and getattr(engine, "artifact_flag", False):
            apply_tag("Motion=", "artifact")

    def update_visual():
        global VIS_HISTORY, VIS_BAND_MAX, hardware

        if not visual_enable_var.get():
            vis_canvas.delete("all")
            hemi_canvas.delete("all")
            return

        if engine is None or engine.last_scores is None:
            vis_canvas.delete("all")
            hemi_canvas.delete("all")
            return

        try:
            s, focus_mean, focus_std = engine.last_scores
        except Exception as e:
            print(f"[UI] last_scores error: {e}")
            vis_canvas.delete("all")
            hemi_canvas.delete("all")
            return

        # ----- artifact flag -----
        try:
            artifact_on = bool(getattr(engine, "artifact_flag", False))
        except Exception:
            artifact_on = False

        # ----- band powers -----
        bands = {
            "delta": float(s.get("delta", 0.0)),
            "theta": float(s.get("theta", 0.0)),
            "alpha": float(s.get("alpha", 0.0)),
            "beta":  float(s.get("beta",  0.0)),
            "gamma": float(s.get("gamma", 0.0)),
        }

        # ----- reward / interrupt sampling -----
        reward_level = 0.0      # servo memory
        interrupt_level = 0.0   # interrupt amplitude
        reward_inst = 0.0       # phasic reward from engine

        try:
            if hardware is not None:
                reward_level    = float(getattr(hardware, "current_level", 0.0))
                interrupt_level = float(getattr(hardware, "interrupt_level", 0.0))
        except Exception:
            pass

        try:
            if engine is not None:
                reward_inst = float(getattr(engine, "last_reward_value", 0.0))
        except Exception:
            pass

        # push into history for timeline
        try:
            VIS_HISTORY.append({
                "delta":         float(bands["delta"]),
                "theta":         float(bands["theta"]),
                "alpha":         float(bands["alpha"]),
                "beta":          float(bands["beta"]),
                "gamma":         float(bands["gamma"]),
                "reward_level":  reward_level,
                "reward":        reward_inst,
                "interrupt":     interrupt_level,
            })
        except Exception:
            pass

        # update maxima for hemi bars
        for k, v in bands.items():
            v_clamped = max(0.0, v)
            prev = VIS_BAND_MAX.get(k, 1.0)
            VIS_BAND_MAX[k] = max(prev, v_clamped, 1e-3)

        # which bands to show
        active_keys = []
        if vis_delta_var.get():
            active_keys.append("delta")
        if vis_theta_var.get():
            active_keys.append("theta")
        if vis_alpha_var.get():
            active_keys.append("alpha")
        if vis_beta_var.get():
            active_keys.append("beta")
        if vis_gamma_var.get():
            active_keys.append("gamma")
        if not active_keys:
            active_keys = ["delta", "theta", "alpha", "beta", "gamma"]

        # relative mix
        total = sum(max(v, 0.0) for v in bands.values()) + 1e-6
        rel = {k: max(v, 0.0) / total for k, v in bands.items()}

        # slow/fast totals
        slow_total = float(s.get("slow_total", 0.0))
        fast_total = float(s.get("fast_total", 0.0))
        slow_rel = slow_total / (slow_total + fast_total + 1e-6)
        fast_rel = 1.0 - slow_rel

        # ==== LEFT / RIGHT FAST RATIO (true ratio) ====
        left_fast  = float(s.get("left_fast",  0.0))
        right_fast = float(s.get("right_fast", 0.0))

        if left_fast <= 0.0 and right_fast <= 0.0:
            lf_raw = float(s.get("left_fast_rel", 0.0))
            rf_raw = float(s.get("right_fast_rel", 0.0))
            pair_sum = lf_raw + rf_raw
            if pair_sum > 1e-6:
                left_fast_ratio  = lf_raw / pair_sum
                right_fast_ratio = rf_raw / pair_sum
            else:
                left_fast_ratio = right_fast_ratio = 0.5
        else:
            pair_sum = left_fast + right_fast
            if pair_sum > 1e-6:
                left_fast_ratio  = left_fast / pair_sum
                right_fast_ratio = right_fast / pair_sum
            else:
                left_fast_ratio = right_fast_ratio = 0.5

        # +1 = strongly LEFT, -1 = strongly RIGHT
        hemi_bias = clamp(left_fast_ratio - right_fast_ratio, -1.0, 1.0)
        visual_bias = -hemi_bias
        twist = hemi_bias * (math.pi / 5.0)

        t = time.monotonic()
        mode = visual_mode_var.get().lower()

        # ===== HEMI MINI-PANEL (draw first, so timeline can't kill it) =====
        try:
            hemi_canvas.delete("all")
            w2 = hemi_canvas.winfo_width()
            h2 = hemi_canvas.winfo_height()

            # rear sensor powers from Mind Monitor mapping (Muse 2):
            # TP9 = left rear, TP10 = right rear
            full_tp9  = float(s.get("full_tp9", 0.0))
            full_tp10 = float(s.get("full_tp10", 0.0))
            full_mean = float(s.get("full_band_mean", 0.0))

            if full_mean < 1e-6:
                tp_thresh = 0.0
            else:
                tp_thresh = max(1e-4, 0.05 * full_mean)

            tp9_ok  = (full_tp9  > tp_thresh) and (not artifact_on)
            tp10_ok = (full_tp10 > tp_thresh) and (not artifact_on)
            rear_ok = tp9_ok and tp10_ok

            # "Rear sensors" + TP9/TP10 status ABOVE the box
            title_y = 4
            hemi_canvas.create_text(
                w2 / 2.0,
                title_y,
                text="Rear sensors",
                fill="#cccccc",
                font=("TkDefaultFont", 9, "bold"),
                anchor="n",
            )

            tp_label_y = title_y + 12
            hemi_canvas.create_text(
                w2 / 2.0 - 24,
                tp_label_y,
                text="TP9",
                fill="#00ff55" if tp9_ok else "#ff4444",
                font=("TkDefaultFont", 9),
                anchor="n",
            )
            hemi_canvas.create_text(
                w2 / 2.0 + 24,
                tp_label_y,
                text="TP10",
                fill="#00ff55" if tp10_ok else "#ff4444",
                font=("TkDefaultFont", 9),
                anchor="n",
            )

            # hemi box
            box_top = tp_label_y + 10

            hc_color = "#ff0000" if artifact_on else "#333333"
            hc_width = 2 if artifact_on else 1

            hemi_canvas.create_rectangle(
                1, box_top,
                w2 - 1, h2 - 1,
                outline=hc_color,
                width=hc_width
            )

            # stacked EEG bars
            band_defs = [
                ("delta", "δ", "#4466ff"),
                ("theta", "θ", "#44ffaa"),
                ("alpha", "α", "#ffff66"),
                ("beta",  "β", "#ff8844"),
                ("gamma", "γ", "#ff44ff"),
            ]

            top_margin = box_top + 8
            row_h = 10
            row_gap = 5
            x0 = 24
            x1 = w2 - 10

            for i, (bkey, label, col) in enumerate(band_defs):
                raw_val = max(0.0, float(bands.get(bkey, 0.0)))
                max_val = max(VIS_BAND_MAX.get(bkey, 1.0), 1e-6)

                strength = raw_val / max_val
                strength = clamp(strength, 0.0, 1.0)

                # minimum stub so non-zero activity never looks like pure zero
                if raw_val > 0.0 and strength < 0.06:
                    strength = 0.06

                y0 = top_margin + i * (row_h + row_gap)
                y1 = y0 + row_h


                hemi_canvas.create_rectangle(
                    x0, y0, x1, y1,
                    outline="#333333",
                    fill="#111111"
                )

                fill_x = x0 + strength * (x1 - x0)
                hemi_canvas.create_rectangle(
                    x0, y0, fill_x, y1,
                    outline="",
                    fill=col
                )

                hemi_canvas.create_text(
                    6, (y0 + y1) / 2.0,
                    text=label,
                    fill=col,
                    font=("TkDefaultFont", 9),
                    anchor="w"
                )

            bands_block_bottom = top_margin + len(band_defs) * (row_h + row_gap) + 4

            mid_x = w2 / 2.0
            hemi_canvas.create_line(
                mid_x, bands_block_bottom,
                mid_x, h2 - 10,
                fill="#444444"
            )
            hemi_canvas.create_text(
                mid_x - 30, bands_block_bottom + 6,
                text="L", fill="#66ff66",
                font=("TkDefaultFont", 9)
            )
            hemi_canvas.create_text(
                mid_x + 30, bands_block_bottom + 6,
                text="R", fill="#ff6666",
                font=("TkDefaultFont", 9)
            )

            bar_y = h2 - 28
            hemi_canvas.create_rectangle(
                10, bar_y,
                w2 - 10, bar_y + 6,
                outline="#333333",
                fill="#111111"
            )

            # "high confidence" if rear OK, no artifact, some fast power
            hemi_valid = rear_ok and (not artifact_on) and (fast_total > 1e-4)

            # +bias (left > right) moves marker LEFT on the bar
            frac_bias = 0.5 - 0.5 * hemi_bias
            frac_bias = clamp(frac_bias, 0.0, 1.0)
            marker_x = 10 + frac_bias * (w2 - 20)

            if hemi_valid:
                marker_color = "#66ff66" if hemi_bias >= 0 else "#ff6666"
            else:
                marker_color = "#777777"

            hemi_canvas.create_oval(
                marker_x - 6, bar_y - 6,
                marker_x + 6, bar_y + 6,
                outline=marker_color, fill=marker_color
            )

            # always compute L/R fast %; grey-out when low confidence
            l_pct = left_fast_ratio * 100.0
            r_pct = right_fast_ratio * 100.0
            ratio_text = f"L fast: {l_pct:4.1f}%\nR fast: {r_pct:4.1f}%"
            ratio_color = "#cccccc" if hemi_valid else "#777777"

            hemi_canvas.create_text(
                mid_x, bar_y - 18,
                text=ratio_text, fill=ratio_color,
                font=("TkDefaultFont", 9),
                justify="center"
            )
        except Exception as e:
            print(f"[UI] hemi panel error: {e}")

        # ===== MAIN VIS CANVAS =====
        w = vis_canvas.winfo_width()
        h = vis_canvas.winfo_height()
        if w < 10 or h < 10:
            return

        cx = w / 2.0
        cy = h / 2.0
        base_r = min(w, h) * 0.18
        max_r  = min(w, h) * 0.45

        vis_canvas.delete("all")

        # ===== BAND TIMELINE MODE =====
        if "timeline" in mode:
            history_list = list(VIS_HISTORY)
            n = len(history_list)
            if n < 2:
                return

            pad_x = 8
            pad_y = 6
            inner_w = w - 2 * pad_x
            inner_h = h - 2 * pad_y

            # downsample to ~80 steps
            MAX_STEPS = 80
            if n > MAX_STEPS:
                step = n / MAX_STEPS
                reduced = []
                f = 0.0
                while int(f) < n:
                    reduced.append(history_list[int(f)])
                    f += step
                history_list = reduced
                n = len(history_list)

            # collect values for scaling (ignore exact zeros so vmin isn't stuck at 0)
            vals = []
            for sample in history_list:
                for key in active_keys:
                    v = float(sample.get(key, 0.0))
                    if v > 0.0:
                        vals.append(v)

            if not vals:
                vals = [1.0]

            vmin = min(vals)
            vmax = max(vals)

            # keep low region from being crushed at the bottom
            span = vmax - vmin
            if span < 1e-6:
                vmin = 0.0
                vmax = 1.0
            else:
                target_floor = vmax * 0.1  # 10% of vmax as the visual "bottom"
                if vmin < target_floor:
                    vmin = target_floor

            # precompute (x,y) for all bands / all samples using this shared vmin/vmax
            band_points = {key: [] for key in active_keys}

            for key in active_keys:
                pts = []
                for i, sample in enumerate(history_list):
                    frac_x = i / (n - 1) if n > 1 else 0.0
                    x = pad_x + frac_x * inner_w

                    val = max(0.0, float(sample.get(key, 0.0)))

                    if val <= 0.0 or vmax <= vmin + 1e-6:
                        norm = 0.0
                    else:
                        norm = (val - vmin) / (vmax - vmin)
                        norm = clamp(norm, 0.0, 1.0)

                    # give small-but-nonzero activity some visible height
                    if val > 0.0 and norm < 0.1:
                        norm = 0.1

                    y = pad_y + inner_h * (1.0 - norm)
                    pts.append((x, y))

                band_points[key] = pts


            # outer box
            outline_color = "#ff0000" if artifact_on else "#333333"
            vis_canvas.create_rectangle(
                pad_x, pad_y, w - pad_x, h - pad_y,
                outline=outline_color,
                width=2 if artifact_on else 1,
                fill="#000000"
            )

            # horizontal grid lines
            for frac in (0.25, 0.5, 0.75):
                y = pad_y + inner_h * (1.0 - frac)
                vis_canvas.create_line(
                    pad_x, y, w - pad_x, y,
                    fill="#111111"
                )

            # --- reward / interrupt helpers ---
            thr = float(globals().get("TIMELINE_EVENT_THRESHOLD", 0.20))

            def _get_reward(sample):
                return float(
                    sample.get("reward",
                    sample.get("reward_inst",
                    sample.get("reward_level",
                    sample.get("reward_value", 0.0))))
                )

            def _get_interrupt(sample):
                return float(
                    sample.get("interrupt",
                    sample.get("interrupt_level",
                    sample.get("penalty",
                    sample.get("penalty_value", 0.0))))
                )

            # detect rising edges for reward / interrupt
            edges = []  # list of (i, type) where type = 1 reward, 2 interrupt
            prev_r = 0.0
            prev_p = 0.0
            for i, sample in enumerate(history_list):
                r_val = _get_reward(sample)
                p_val = _get_interrupt(sample)

                reward_edge    = (r_val > thr) and (prev_r <= thr)
                interrupt_edge = (p_val > thr) and (prev_p <= thr)

                prev_r = r_val
                prev_p = p_val

                # interrupt wins if both happen at once
                if interrupt_edge:
                    edges.append((i, 2))
                elif reward_edge:
                    edges.append((i, 1))

            # --- background stripes: per-edge solid blocks (simple, no merging) ---
            try:
                halo_on = vis_event_halo_var.get()
            except Exception:
                halo_on = False

            if halo_on and edges:
                halo_px = int(globals().get("TIMELINE_EVENT_HALO_PX", 3))
                if halo_px < 1:
                    halo_px = 1

                for i, etype in edges:
                    frac_x = i / (n - 1) if n > 1 else 0.0
                    x = pad_x + frac_x * inner_w
                    x0 = x - halo_px
                    x1 = x + halo_px

                    fill_col = "#331111" if etype == 2 else "#113311"
                    vis_canvas.create_rectangle(
                        x0, pad_y,
                        x1, pad_y + inner_h,
                        outline="",
                        fill=fill_col,
                    )

            # --- profile targets (which bands get recolored) ---
            profile_targets = globals().get("PROFILE_BAND_TARGETS", {})
            profile_name = mode_var.get().strip().lower()
            profile_cfg = profile_targets.get(profile_name, profile_targets.get("*", {}))

            def band_is_target(bname: str) -> bool:
                if not profile_cfg:
                    return True
                weight = profile_cfg.get(bname, profile_cfg.get("*", 1.0))
                return abs(weight) > 1e-6

            try:
                recolor_on = vis_band_recolor_var.get()
            except Exception:
                recolor_on = True

            band_colors = {
                "delta": "#4466ff",
                "theta": "#44ffaa",
                "alpha": "#ffff66",
                "beta":  "#ff8844",
                "gamma": "#ff44ff",
            }

            # precompute (x,y) for all bands / all samples
            band_points = {
                key: [] for key in active_keys
            }
            for key in active_keys:
                pts = []
                for i, sample in enumerate(history_list):
                    frac_x = i / (n - 1) if n > 1 else 0.0
                    x = pad_x + frac_x * inner_w
                    val = max(0.0, float(sample.get(key, 0.0)))

                    # protect against divide-by-zero
                    if vmax <= vmin + 1e-6:
                        norm = 0.0
                    else:
                        norm = (val - vmin) / (vmax - vmin)

                    # clamp + give a floor so tiny activity isn't literally glued to the bottom
                    norm = clamp(norm, 0.0, 1.0)
                    if val > 0.0 and norm < 0.05:
                        norm = 0.05   # minimum visible height

                    y = pad_y + inner_h * (1.0 - norm)
                    pts.append((x, y))


                band_points[key] = pts

            # --- draw base lines first (always) ---
            for key in active_keys:
                pts = band_points[key]
                if len(pts) < 2:
                    continue
                flat = []
                for (x, y) in pts:
                    flat.extend([x, y])
                vis_canvas.create_line(
                    *flat,
                    fill=band_colors.get(key, "#ffffff"),
                    width=1,
                    smooth=True
                )

            # --- overlay recolor markers ONLY near edges (instant flashes) ---
            if recolor_on and edges:
                radius = int(globals().get("TIMELINE_EVENT_COLOR_RADIUS_STEPS", 0))
                if radius < 0:
                    radius = 0

                for key in active_keys:
                    if not band_is_target(key):
                        continue

                    pts = band_points[key]
                    if len(pts) < 1:
                        continue

                    for i_edge, etype in edges:
                        color = "#00ff55" if etype == 1 else "#ff2222"

                        j0 = max(0, i_edge - radius)
                        j1 = min(n - 1, i_edge + radius)

                        for j in range(j0, j1 + 1):
                            if j < 0 or j >= len(pts):
                                continue
                            x, y = pts[j]
                            # small vertical tick centered on the band line
                            vis_canvas.create_line(
                                x, y - 4,
                                x, y + 4,
                                fill=color,
                                width=2,
                                smooth=False
                            )


            # (optional) you can still add bottom ticks here if you want
            return



            pad_x = 8
            pad_y = 6
            inner_w = w - 2 * pad_x
            inner_h = h - 2 * pad_y

            # downsample to ~80 steps
            MAX_STEPS = 80
            if n > MAX_STEPS:
                step = n / MAX_STEPS
                reduced = []
                f = 0.0
                while int(f) < n:
                    reduced.append(history_list[int(f)])
                    f += step
                history_list = reduced
                n = len(history_list)

            # gather band values to scale
            vals = []
            for sample in history_list:
                for key in active_keys:
                    vals.append(max(0.0, float(sample.get(key, 0.0))))

            if not vals:
                return

            vmin = min(vals)
            vmax = max(vals)
            if abs(vmax - vmin) < 1e-6:
                vmin = 0.0
                vmax = 1.0

            # outer box
            outline_color = "#ff0000" if artifact_on else "#333333"
            vis_canvas.create_rectangle(
                pad_x, pad_y, w - pad_x, h - pad_y,
                outline=outline_color,
                width=2 if artifact_on else 1,
                fill="#000000"
            )

            # horizontal grid lines
            for frac in (0.25, 0.5, 0.75):
                y = pad_y + inner_h * (1.0 - frac)
                vis_canvas.create_line(
                    pad_x, y, w - pad_x, y,
                    fill="#111111"
                )
                
            # === EVENT HIGHLIGHT STRIPES (reward / interrupt) ===
            # configurable width in px (half-width around each event x)
            halo_px = int(globals().get("TIMELINE_EVENT_HALO_PX", 3))
            if halo_px < 1:
                halo_px = 1

            # threshold for "real" events
            thr = float(globals().get("TIMELINE_EVENT_THRESHOLD", 0.20))

            # helpers to read reward / interrupt from history sample
            def _get_reward(sample):
                return float(
                    sample.get("reward",
                    sample.get("reward_inst",
                    sample.get("reward_level",
                    sample.get("reward_value", 0.0))))
                )

            def _get_interrupt(sample):
                return float(
                    sample.get("interrupt",
                    sample.get("interrupt_level",
                    sample.get("penalty",
                    sample.get("penalty_value", 0.0))))
                )

            # use same logic as the single/continuous tick mode
            try:
                single_mode = vis_tick_single_var.get()
            except Exception:
                single_mode = False

            prev_r = 0.0
            prev_p = 0.0

            for i, sample in enumerate(history_list):
                frac_x = i / (n - 1) if n > 1 else 0.0
                x = pad_x + frac_x * inner_w

                r_val = _get_reward(sample)
                p_val = _get_interrupt(sample)

                if single_mode:
                    # rising edge only
                    reward_evt    = (r_val > thr) and (prev_r <= thr)
                    interrupt_evt = (p_val > thr) and (prev_p <= thr)
                else:
                    # any sample above threshold
                    reward_evt    = (r_val > thr)
                    interrupt_evt = (p_val > thr)

                prev_r = r_val
                prev_p = p_val

                if not (reward_evt or interrupt_evt):
                    continue

                # choose color: interrupt overrides reward if both fire
                if interrupt_evt:
                    fill_col = "#442222"  # darker red so bands still visible
                else:
                    fill_col = "#224422"  # darker green

                x0 = x - halo_px
                x1 = x + halo_px
                vis_canvas.create_rectangle(
                    x0, pad_y,
                    x1, pad_y + inner_h,
                    outline="",
                    fill=fill_col,
                )


            band_colors = {
                "delta": "#4466ff",
                "theta": "#44ffaa",
                "alpha": "#ffff66",
                "beta":  "#ff8844",
                "gamma": "#ff44ff",
            }

            # band lines
            for key in active_keys:
                pts = []
                col = band_colors.get(key, "#ffffff")
                for i, sample in enumerate(history_list):
                    frac_x = i / (n - 1) if n > 1 else 0.0
                    x = pad_x + frac_x * inner_w
                    val = max(0.0, float(sample.get(key, 0.0)))
                    norm = (val - vmin) / (vmax - vmin)
                    y = pad_y + inner_h * (1.0 - norm)
                    pts.extend([x, y])

                if len(pts) >= 4:
                    vis_canvas.create_line(
                        *pts,
                        fill=col,
                        width=1,
                        smooth=True
                    )

            # --- TICK CONFIG (pull from globals if defined) ---
            thr = float(globals().get("TIMELINE_EVENT_THRESHOLD", 0.20))
            stride = int(globals().get("TIMELINE_TICK_STRIDE", 3))
            stride = max(1, stride)

            reward_base_h    = float(globals().get("TIMELINE_REWARD_BASE_H", 6.0))
            reward_max_h     = float(globals().get("TIMELINE_REWARD_MAX_H", 36.0))
            interrupt_base_h = float(globals().get("TIMELINE_INTERRUPT_BASE_H", 6.0))
            interrupt_max_h  = float(globals().get("TIMELINE_INTERRUPT_MAX_H", 36.0))

            # helpers (get values from history dict)
            def get_reward(sample):
                return float(
                    sample.get("reward",
                    sample.get("reward_inst",
                    sample.get("reward_level",
                    sample.get("reward_value", 0.0))))
                )

            def get_interrupt(sample):
                return float(
                    sample.get("interrupt",
                    sample.get("interrupt_level",
                    sample.get("penalty",
                    sample.get("penalty_value", 0.0))))
                )

            def _norm_above_thr(v: float) -> float:
                if v <= thr:
                    return 0.0
                span = max(1e-6, 1.0 - thr)
                return clamp((v - thr) / span, 0.0, 1.0)

            def _reward_h(v: float) -> float:
                nrm = _norm_above_thr(v)
                return reward_base_h + (reward_max_h - reward_base_h) * nrm

            def _interrupt_h(v: float) -> float:
                nrm = _norm_above_thr(v)
                return interrupt_base_h + (interrupt_max_h - interrupt_base_h) * nrm

            # ticks on/off + single/continuous
            try:
                ticks_on = vis_ticks_on_var.get()
            except Exception:
                ticks_on = True

            if ticks_on:
                try:
                    single_mode = vis_tick_single_var.get()
                except Exception:
                    single_mode = False

                y_bottom = pad_y + inner_h

                if single_mode:
                    # rising-edge ticks only, scaled height
                    prev_r = 0.0
                    prev_p = 0.0

                    for i, sample in enumerate(history_list):
                        frac_x = i / (n - 1) if n > 1 else 0.0
                        x = pad_x + frac_x * inner_w

                        r_inst = get_reward(sample)
                        p_inst = get_interrupt(sample)

                        reward_edge    = (r_inst > thr) and (prev_r <= thr)
                        interrupt_edge = (p_inst > thr) and (prev_p <= thr)

                        prev_r = r_inst
                        prev_p = p_inst

                        if interrupt_edge:
                            hp = _interrupt_h(p_inst)
                            vis_canvas.create_line(
                                x, y_bottom,
                                x, y_bottom - hp,
                                fill="#ff2222",
                                width=2
                            )

                        if reward_edge:
                            hr = _reward_h(r_inst)
                            vis_canvas.create_line(
                                x, y_bottom,
                                x, y_bottom - hr,
                                fill="#00ff55",
                                width=2
                            )
                else:
                    # continuous ticks at stride, scaled height
                    for i, sample in enumerate(history_list):
                        if i % stride != 0:
                            continue

                        frac_x = i / (n - 1) if n > 1 else 0.0
                        x = pad_x + frac_x * inner_w

                        r_val = get_reward(sample)
                        p_val = get_interrupt(sample)

                        if p_val > thr:
                            hp = _interrupt_h(p_val)
                            vis_canvas.create_line(
                                x, y_bottom,
                                x, y_bottom - hp,
                                fill="#ff2222",
                                width=2
                            )

                        if r_val > thr:
                            hr = _reward_h(r_val)
                            vis_canvas.create_line(
                                x, y_bottom,
                                x, y_bottom - hr,
                                fill="#00ff55",
                                width=2
                            )

            # timeline mode done
            return

        # ===== NON-TIMELINE MODES (neuro-primitives) =====
        border_color = "#ff0000" if artifact_on else "#222222"
        border_width = 3 if artifact_on else 1
        vis_canvas.create_rectangle(
            2, 2, w - 2, h - 2,
            outline=border_color,
            width=border_width
        )

        # radial / flower style
        if "radial" in mode or "flower" in mode:
            base_scale = 0.65 + 0.25 * math.sin(t * 0.8)
            r0 = base_r * base_scale
            vis_canvas.create_oval(
                cx - r0, cy - r0,
                cx + r0, cy + r0,
                outline="#222222",
                width=2
            )

            slow_r = base_r + slow_rel * (max_r - base_r) * 0.6
            fast_r = base_r + fast_rel * (max_r - base_r) * 0.6

            vis_canvas.create_oval(
                cx - slow_r, cy - slow_r,
                cx + slow_r, cy + slow_r,
                outline="#3355ff",
                width=int(1 + slow_rel * 3)
            )

            vis_canvas.create_oval(
                cx - fast_r, cy - fast_r,
                cx + fast_r, cy + fast_r,
                outline="#ff7744",
                width=int(1 + fast_rel * 3)
            )

            stroke_count = 14
            for i in range(stroke_count):
                frac = i / stroke_count
                ang = 2 * math.pi * frac + t * 0.3 + twist
                r_inner = base_r
                r_outer = base_r + 0.6 * (max_r - base_r) * (0.7 + 0.3 * fast_rel)
                x0 = cx + math.cos(ang) * r_inner
                y0 = cy + math.sin(ang) * r_inner
                x1 = cx + math.cos(ang) * r_outer
                y1 = cy + math.sin(ang) * r_outer

                hemi_weight = 0.5 + 0.5 * math.cos(ang - twist) * hemi_bias
                band_mix = (
                    rel["beta"] * 0.5 +
                    rel["gamma"] * 0.3 +
                    rel["alpha"] * 0.2
                )
                brightness = clamp(hemi_weight * band_mix * 2.0, 0.0, 1.0)
                if brightness < 0.05:
                    continue

                col = "#%02x%02x%02x" % (
                    int(255 * brightness),
                    int(80 * brightness),
                    int(40 + 150 * brightness),
                )

                vis_canvas.create_line(
                    x0, y0, x1, y1,
                    fill=col,
                    width=2
                )

        # theta ribbon
        if vis_theta_var.get():
            th_strength = rel["theta"]
            amp = th_strength * 60.0
            pts = []
            steps = 32
            y_min = cy - max_r * 0.9
            y_max = cy + max_r * 0.9
            for i in range(steps):
                frac = i / (steps - 1)
                y = y_min + (y_max - y_min) * frac
                x = cx + math.sin(0.015 * (y - cy) + t * 0.7) * amp
                pts.extend([x, y])
            vis_canvas.create_line(
                *pts,
                fill="#44ffaa",
                width=max(1, int(1 + th_strength * 3)),
                smooth=True
            )

        # alpha flower
        if vis_alpha_var.get():
            a_strength = rel["alpha"]
            petal_count = 12
            for i in range(petal_count):
                frac = i / petal_count
                ang = 2 * math.pi * frac + t * 0.4
                r_inner = base_r * (1.0 - 0.2 * a_strength)
                r_outer = base_r + a_strength * (max_r - base_r)
                x0 = cx + math.cos(ang + visual_bias * 0.4) * r_inner
                y0 = cy + math.sin(ang + visual_bias * 0.4) * r_inner
                x1 = cx + math.cos(ang + visual_bias * 0.4) * r_outer
                y1 = cy + math.sin(ang + visual_bias * 0.4) * r_outer

                col = "#%02x%02x%02x" % (
                    int(200 + 55 * a_strength),
                    int(200 + 55 * a_strength),
                    int(80),
                )
                vis_canvas.create_line(
                    x0, y0, x1, y1,
                    fill=col,
                    width=2
                )

        # beta arcs
        if vis_beta_var.get():
            b_strength = rel["beta"]
            ring_count = 4
            for i in range(ring_count):
                frac = i / ring_count
                r = base_r + frac * (max_r - base_r)
                jitter = 0.08 * math.sin(t * 1.0 + i)
                start = (20 + 60 * visual_bias) + jitter * 180 / math.pi
                extent = 140 + 20 * b_strength
                vis_canvas.create_arc(
                    cx - r, cy - r,
                    cx + r, cy + r,
                    start=start, extent=extent,
                    style="arc",
                    outline="#ff8844",
                    width=max(1, int(1 + b_strength * 3))
                )

        # gamma sparkle
        if vis_gamma_var.get():
            g_strength = rel["gamma"]
            dot_count = int(40 + 80 * g_strength)
            for i in range(dot_count):
                frac = (i + (t * 2.0) % 1.0) / dot_count
                ang = 2 * math.pi * frac
                r = base_r + (max_r - base_r) * (0.5 + 0.5 * math.sin(t * 1.5 + i))
                x = cx + math.cos(ang) * r
                y = cy + math.sin(ang) * r
                vis_canvas.create_oval(
                    x - 1, y - 1, x + 1, y + 1,
                    outline="#ff44ff",
                    fill="#ff44ff"
                )


        # ===== HEMI MINI-PANEL =====
        try:
            hemi_canvas.delete("all")
            w2 = hemi_canvas.winfo_width()
            h2 = hemi_canvas.winfo_height()

            # hemi panel border (artifact indicator)
            hc_color = "#ff0000" if artifact_on else "#333333"
            hc_width = 2 if artifact_on else 1

            # --- rear sensor / hemi validity check ---
            full_tp9  = float(s.get("full_tp9", 0.0))
            full_tp10 = float(s.get("full_tp10", 0.0))
            full_mean = float(s.get("full_band_mean", 0.0))

            # threshold for "receiving data" from each rear sensor
            if full_mean < 1e-6:
                tp_thresh = 0.0
            else:
                tp_thresh = max(1e-4, 0.05 * full_mean)

            tp9_ok  = (full_tp9  > tp_thresh) and (not artifact_on)
            tp10_ok = (full_tp10 > tp_thresh) and (not artifact_on)
            rear_ok = tp9_ok and tp10_ok

            # --- "Rear sensors" + TP9/TP10 status ABOVE the hemi box ---
            title_y = 6
            hemi_canvas.create_text(
                w2 / 2.0,
                title_y,
                text="Rear sensors",
                fill="#cccccc",
                font=("TkDefaultFont", 9, "bold"),
                anchor="n",
            )

            tp_label_y = title_y + 12
            hemi_canvas.create_text(
                w2 / 2.0 - 24,
                tp_label_y,
                text="TP9",
                fill="#00ff55" if tp9_ok else "#ff4444",
                font=("TkDefaultFont", 9),
                anchor="n",
            )
            hemi_canvas.create_text(
                w2 / 2.0 + 24,
                tp_label_y,
                text="TP10",
                fill="#00ff55" if tp10_ok else "#ff4444",
                font=("TkDefaultFont", 9),
                anchor="n",
            )

            # draw the "hemi box" rectangle BELOW the status text
            box_top = tp_label_y + 10
            hemi_canvas.create_rectangle(
                1, box_top,
                w2 - 1, h2 - 1,
                outline=hc_color,
                width=hc_width
            )

            # band bars stacked INSIDE the box
            band_defs = [
                ("delta", "δ", "#4466ff"),
                ("theta", "θ", "#44ffaa"),
                ("alpha", "α", "#ffff66"),
                ("beta",  "β", "#ff8844"),
                ("gamma", "γ", "#ff44ff"),
            ]

            top_margin = box_top + 8
            row_h = 10
            row_gap = 5
            x0 = 24
            x1 = w2 - 10

            for i, (bkey, label, col) in enumerate(band_defs):
                raw_val = max(0.0, float(bands.get(bkey, 0.0)))
                max_val = max(VIS_BAND_MAX.get(bkey, 1.0), 1e-6)

                strength = raw_val / max_val
                strength = clamp(strength, 0.0, 1.0)

                # minimum stub so non-zero activity never looks like pure zero
                if raw_val > 0.0 and strength < 0.06:
                    strength = 0.06


                y0 = top_margin + i * (row_h + row_gap)
                y1 = y0 + row_h

                hemi_canvas.create_rectangle(
                    x0, y0, x1, y1,
                    outline="#333333",
                    fill="#111111"
                )

                fill_x = x0 + strength * (x1 - x0)
                hemi_canvas.create_rectangle(
                    x0, y0, fill_x, y1,
                    outline="",
                    fill=col
                )

                hemi_canvas.create_text(
                    6, (y0 + y1) / 2.0,
                    text=label,
                    fill=col,
                    font=("TkDefaultFont", 9),
                    anchor="w"
                )

            bands_block_bottom = top_margin + len(band_defs) * (row_h + row_gap) + 4

            mid_x = w2 / 2.0
            hemi_canvas.create_line(
                mid_x, bands_block_bottom,
                mid_x, h2 - 10,
                fill="#444444"
            )
            hemi_canvas.create_text(
                mid_x - 30, bands_block_bottom + 6,
                text="L", fill="#66ff66",
                font=("TkDefaultFont", 9)
            )
            hemi_canvas.create_text(
                mid_x + 30, bands_block_bottom + 6,
                text="R", fill="#ff6666",
                font=("TkDefaultFont", 9)
            )

            bar_y = h2 - 28
            hemi_canvas.create_rectangle(
                10, bar_y,
                w2 - 10, bar_y + 6,
                outline="#333333",
                fill="#111111"
            )

            # decide if hemi ratio is "high confidence" (rear sensors OK, no artifact, some fast power)
            hemi_valid = rear_ok and (not artifact_on) and (fast_total > 1e-4)

            # marker position: +bias (left > right) moves marker LEFT
            frac_bias = 0.5 - 0.5 * hemi_bias
            frac_bias = clamp(frac_bias, 0.0, 1.0)
            marker_x = 10 + frac_bias * (w2 - 20)

            if hemi_valid:
                marker_color = "#66ff66" if hemi_bias >= 0 else "#ff6666"
            else:
                # still show the ratio (approximate) but visually de-emphasize
                marker_color = "#777777"

            hemi_canvas.create_oval(
                marker_x - 6, bar_y - 6,
                marker_x + 6, bar_y + 6,
                outline=marker_color, fill=marker_color
            )

            # always compute L/R fast as ratios, even if rear is not connected
            l_pct = left_fast_ratio * 100.0
            r_pct = right_fast_ratio * 100.0

            ratio_text = f"L fast: {l_pct:4.1f}%\nR fast: {r_pct:4.1f}%"
            ratio_color = "#cccccc" if hemi_valid else "#777777"

            hemi_canvas.create_text(
                mid_x, bar_y - 18,
                text=ratio_text, fill=ratio_color,
                font=("TkDefaultFont", 9),
                justify="center"
            )
        except Exception as e:
            print(f"[UI] hemi panel error: {e}")



    # ---------- FORMULA RENDERER ----------
    def render_formulas():
        now = time.monotonic()

        # reward
        r_track = FORMULA_TRACK.get("reward", {})
        r_mode = r_track.get("mode", "engaged")
        r_terms_state = r_track.get("terms", {})

        r_layout = _get_formula_layout("reward", r_mode)
        r_expr = r_layout.get("expr", "")
        r_terms_cfg = r_layout.get("terms", [])

        reward_formula_text.configure(state="normal")
        reward_formula_text.delete("1.0", "end")

        if r_expr:
            reward_formula_text.insert("end", r_expr + "\n")

        # each term: label with arrow
        for idx, t in enumerate(r_terms_cfg):
            tid = t.get("id", "")
            label = t.get("label", tid or "?")
            thr = float(t.get("thr", 0.6))

            st = r_terms_state.get(tid, {"value": 0.0, "fulfilled": False, "fail_until": 0.0})
            val = float(st.get("value", 0.0))
            fulfilled = bool(st.get("fulfilled", False))
            fail_until = float(st.get("fail_until", 0.0))

            # default style
            color = "#888888"
            arrow = "↑"

            # fulfilled = green
            if fulfilled and val >= thr:
                color = "#00ff55"
            # red flash if recently lost
            elif now < fail_until:
                color = "#ff3333"

            tag = f"reward_term_{idx}"
            start = reward_formula_text.index("end")
            reward_formula_text.insert("end", f"{arrow} {label}  ")
            end = reward_formula_text.index("end")
            reward_formula_text.tag_add(tag, start, end)
            reward_formula_text.tag_config(tag, foreground=color)

        reward_formula_text.configure(state="disabled")

        # penalty / interrupt
        p_track = FORMULA_TRACK.get("penalty", {})
        p_mode = p_track.get("mode", r_mode)
        p_terms_state = p_track.get("terms", {})

        p_layout = _get_formula_layout("penalty", p_mode)
        p_expr = p_layout.get("expr", "")
        p_terms_cfg = p_layout.get("terms", [])

        interrupt_formula_text.configure(state="normal")
        interrupt_formula_text.delete("1.0", "end")

        if p_expr:
            interrupt_formula_text.insert("end", p_expr + "\n")

        for idx, t in enumerate(p_terms_cfg):
            tid = t.get("id", "")
            label = t.get("label", tid or "?")
            thr = float(t.get("thr", 0.6))

            st = p_terms_state.get(tid, {"value": 0.0, "fulfilled": False, "fail_until": 0.0})
            val = float(st.get("value", 0.0))
            fulfilled = bool(st.get("fulfilled", False))
            fail_until = float(st.get("fail_until", 0.0))

            # default style
            color = "#888888"
            arrow = "↓"

            # for penalty, "fulfilled" = penalty term is high
            if fulfilled and val >= thr:
                color = "#ff3333"   # red when interrupt condition satisfied
            elif now < fail_until:
                # just dropped below threshold -> flash green for a moment
                color = "#00ff55"

            tag = f"penalty_term_{idx}"
            start = interrupt_formula_text.index("end")
            interrupt_formula_text.insert("end", f"{arrow} {label}  ")
            end = interrupt_formula_text.index("end")
            interrupt_formula_text.tag_add(tag, start, end)
            interrupt_formula_text.tag_config(tag, foreground=color)

        interrupt_formula_text.configure(state="disabled")





    # ---------- poll loop ----------
    def poll_state():
        global last_state_line, last_reward_value, hardware

        # ----- LIVE STATE TEXT -----
        try:
            state_text.configure(state="normal")
            state_text.delete("1.0", "end")
            if last_state_line:
                state_text.insert("end", last_state_line)
                apply_all_tags()
            else:
                state_text.insert("end", "No data yet")
            state_text.configure(state="disabled")
        except Exception as e:
            print(f"[UI] state_text error: {e}")

        # ----- REWARD BAR -----
        try:
            rv = clamp(float(last_reward_value), 0.0, 1.0)
        except Exception as e:
            print(f"[UI] reward_var error: {e}")
            rv = 0.0
        reward_var.set(rv * 100.0)

        # ----- INTERRUPT BAR -----
        try:
            if hardware is not None:
                ang = getattr(hardware, "last_int_angle_sent", 0) or 0
                ang = int(clamp(float(ang), 0.0, 180.0))
            else:
                ang = 0
        except Exception as e:
            print(f"[UI] interrupt_var error: {e}")
            ang = 0
        interrupt_var.set(ang)

        # ----- WINDOW TITLE -----
        try:
            root.title(f"Autopilot v0.5 | Reward: {int(rv * 100)}% | Int: {ang}°")
        except Exception:
            pass

        # ----- VISUALIZER -----
        try:
            if visual_enable_var.get():
                update_visual()
            else:
                vis_canvas.delete("all")
                hemi_canvas.delete("all")
        except Exception as e:
            print(f"[UI] update_visual error: {e}")
            
        try:
            render_formulas()
        except Exception as e:
            print(f"[UI] formula render error: {e}")

        root.after(33, poll_state)

    poll_state()

    def on_close():
        stop_osc_server()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


# ================== MAIN ==================

if __name__ == "__main__":
    run_ui()
