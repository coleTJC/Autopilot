# Autopilot Roadmap

Autopilot is currently a working experimental prototype with real-time OSC input,
reward/interrupt logic, hardware feedback, logging, and multiple live
visualizations. The next phase should preserve that exploratory speed while
reducing the amount of behavior concentrated in `Autopilot.py`.

This roadmap prioritizes the changes that make future prediction, telemetry,
new reward profiles, and hardware variants easier to add without destabilizing
real-time feedback.

## Current Architecture Summary

- `Autopilot.py` is the main application. It owns configuration, global runtime
  state, OSC routing, EEG/hemo/motion windows, scoring, reward profiles,
  interrupt logic, formula tracking, Tkinter UI, timeline visualization, CSV
  logging, pigpio servo output, and buzzer output.
- `AutopilotV2.py` is a smaller pygame OSC monitor. It has useful raw EEG to
  band-derivation logic using FFT, plus a simpler live band graph, but it does
  not contain the full reward/interrupt/hardware system.
- `AGENTS.md` correctly describes the long-term direction: explainable real-time
  EEG state training with feedback, reinforcement, visualization, telemetry,
  prediction, and future biosignal integration.

## Highest Technical Debt

1. Monolithic main file

   `Autopilot.py` is doing too much. The main risk is not file length by itself;
   it is that unrelated systems share globals and nested UI closures. A small UI
   change can accidentally affect scoring, hardware output, or timeline state.

2. Shared mutable globals across threads

   OSC callbacks, hardware output, and Tkinter polling all read or write global
   state such as `engine`, `hardware`, `buzzer`, `last_state_line`,
   `last_reward_value`, `FORMULA_TRACK`, and visualization history. There is no
   explicit event bus, state snapshot, or lock boundary.

3. Mixed pure logic and side effects

   Reward scoring is mostly pure, but formula tracking is updated inside scoring
   functions. Hardware output also calls interrupt scoring directly. This makes
   it harder to test state estimation independently from UI and hardware.

4. Duplicate and unreachable UI/visualizer code

   The formula UI widgets and visualizer controls are duplicated in the Tkinter
   setup. The timeline renderer also contains older unreachable code after an
   early return. This increases the cost of UI changes and hides behavior.

5. Split EEG band strategy

   The main app expects incoming band endpoints. `AutopilotV2.py` derives bands
   from raw `/muse/eeg`. These should become one shared signal pipeline with
   selectable input modes: raw-derived bands, Mind Monitor band endpoints, or
   replayed/session data.

6. Hardware coupling

   Hardware control is tied directly to pigpio GPIO pins. The UI still exposes
   legacy Arduino COM fields even though the current hardware path ignores them.
   There is no explicit mock/null hardware implementation for development.

7. Limited test surface

   The highest-value behavior currently has no obvious automated tests:
   artifact detection, reward buckets, interrupt thresholds, formula term
   states, servo mapping, and CSV row shape.

## Easiest Wins

1. Remove unreachable timeline code

   Delete the older timeline implementation after the active timeline return
   once its behavior is confirmed unused. This is low-risk cleanup because the
   code path cannot execute.

2. Deduplicate formula UI construction

   Keep one pair of reward/interrupt formula text widgets and one formula font
   control. This should reduce confusion before larger UI work.

3. Introduce a null hardware mode

   Add a `NullHardwareController` with the same public methods as
   `HardwareController`. Use it when pigpio is unavailable or when the user runs
   on Windows/development hardware. This makes local testing less fragile.

4. Extract constants into `config.py`

   Move stable constants, available modes, profile names, timeline constants,
   hardware pins, and CSV headers into one module. Keep runtime UI variables
   separate from static defaults.

5. Add smoke tests for pure calculations

   Start with tests for `clamp`, `bell`, reward profile bucket selection,
   amplifier conversion, artifact spike detection, and servo level-to-angle
   mapping. These tests can exist before the full architecture is split.

6. Align raw EEG band derivation

   Move the FFT band derivation from `AutopilotV2.py` into a reusable module and
   let the main engine optionally use it. This avoids maintaining two separate
   interpretations of EEG bands.

## Architecture Improvements

### Phase 1: Create Clean Boundaries

Suggested module split:

- `config.py`: constants, profile names, default runtime settings.
- `models.py`: typed state records such as band samples, score snapshots,
  reward results, penalty results, timeline samples, and telemetry samples.
- `signal_pipeline.py`: EEG windowing, raw EEG FFT band derivation, hemo window,
  HR, motion, artifact detection.
- `reward_profiles.py`: profile definitions and formula metadata.
- `reward_engine.py`: instant reward, reward windowing, bucket assignment,
  smoothing policy.
- `interrupt_engine.py`: penalty terms, interrupt intensity, rising-edge logic.
- `hardware.py`: pigpio controller, null/mock controller, buzzer controller.
- `timeline.py`: history sampling, event detection, band scaling, target band
  recoloring, timeline draw data.
- `logging_store.py`: rolling buffer, CSV writer, replay/session file loading.
- `osc_input.py`: endpoint routing and normalized input events.
- `ui_tk.py`: Tkinter layout and rendering only.

The goal is not to over-abstract. The first useful boundary is to make scoring,
artifact detection, reward, interrupt, and timeline event detection callable
without Tkinter, pigpio, or OSC.

### Phase 2: Introduce Runtime State Snapshots

Create one immutable or copy-safe `RuntimeSnapshot` object that represents what
the UI, hardware, logger, timeline, telemetry, and prediction systems need at a
given moment.

The engine would produce snapshots containing:

- timestamp
- per-band global and per-channel values
- derived focus/calm/overload metrics
- left/right fast ratios
- hemo, HR, and motion
- artifact state
- reward value, bucket, and tags
- penalty/interrupt value and active reasons
- formula term values

This reduces direct global reads and gives prediction/telemetry systems a clean
integration point.

### Phase 3: Event Bus / Queues

Use a small internal event queue between OSC input and the engine. OSC should
only normalize incoming messages and enqueue them. A single engine loop should
consume events, update state, and publish snapshots.

This makes real-time behavior easier to reason about and avoids doing too much
work inside OSC callbacks.

## Prediction Engine Integration Points

Prediction should be added after state snapshots exist. Avoid bolting prediction
directly into Tkinter or hardware control.

Best integration points:

1. Snapshot stream

   Feed recent `RuntimeSnapshot` objects into a prediction module. This gives
   the predictor access to all current derived metrics without depending on
   implementation details inside the engine.

2. Feature extraction layer

   Add a `features.py` module that converts snapshots into compact feature
   vectors:

   - band relatives and ratios
   - focus/calm/overload
   - left/right fast ratios and drift
   - reward and interrupt history
   - hemo/HR/motion trends
   - artifact flags and confidence

3. Prediction result object

   Predictor output should be explicit and explainable:

   - predicted state label
   - confidence
   - forecast horizon, such as 1s, 3s, or 10s
   - top contributing features
   - recommended UI display text
   - optional reward/interrupt advisory signal

4. Non-authoritative first use

   Prediction should initially be display-only. Do not let prediction drive
   servo/tVNS or reward decisions until it has replay validation against logged
   sessions.

5. Replay validation

   Add a replay mode that reads captured CSV/session data and runs predictors
   without hardware. This is the safest place to tune prediction before live use.

## Telemetry Integration Points

Telemetry should observe the system, not own the system.

Recommended telemetry events:

- `input.osc_message`: endpoint, timestamp, payload shape, not necessarily raw
  high-volume payload by default.
- `engine.snapshot`: throttled derived state metrics.
- `reward.event`: bucket transitions, reward value, profile, formula terms.
- `interrupt.event`: penalty value, active penalty reasons, interrupt intensity.
- `artifact.event`: artifact on/off, reason, recent full-band stats.
- `hardware.output`: main servo target/angle, interrupt servo target/angle,
  buzzer events.
- `ui.action`: start, stop, capture, profile change, hardware setting change.
- `session.capture`: CSV path, duration, sample count, profile.

Recommended telemetry architecture:

1. Add a `TelemetrySink` interface.
2. Provide `NullTelemetrySink` by default.
3. Add local JSONL telemetry sink before any network sink.
4. Add throttling and redaction rules before sending anything remotely.
5. Keep telemetry failures non-fatal.

Telemetry should be fed from snapshots and event objects. It should not scrape
Tkinter widgets or hardware internals.

## UI Improvements

1. Separate layout from rendering logic

   Move visualizer drawing code into functions that accept a snapshot and canvas
   dimensions. Tkinter should call renderers, not compute signal logic.

2. Clean profile panel

   Split reward profile selection, hardware mode, sound, gain, and servo floor
   into clearly separated controls. The current top row is dense and mixes
   concepts.

3. Make hardware status explicit

   Show whether pigpio is connected, whether main/interrupt servos are active,
   whether buzzer is active, and whether the app is in null/mock hardware mode.

4. Improve formula panel

   Add profile-specific formula descriptions for `engaged`, `calm`, and
   `recovery`. Show current numeric values beside terms so the user can see not
   just pass/fail, but margin.

5. Add session/replay controls

   Give captured sessions a clear workflow:

   - start live session
   - capture rolling window
   - save full session
   - load replay
   - compare profile/prediction behavior without hardware

6. Improve timeline controls

   Group event tick, stripe, recolor, and band-selection options. Add a small
   legend for reward vs interrupt and for each band color.

7. Add error/status log panel

   Hardware failures, OSC binding errors, CSV write failures, and telemetry
   failures should be visible in the UI instead of only printed to stdout.

## Suggested Development Order

1. Clean obvious UI duplication and unreachable timeline code.
2. Extract configuration and profile definitions.
3. Extract pure signal/reward/interrupt/artifact functions and add tests.
4. Add snapshot objects and make UI/hardware consume snapshots.
5. Add null hardware and local replay mode.
6. Move FFT band derivation from `AutopilotV2.py` into the shared signal
   pipeline.
7. Add local JSONL telemetry.
8. Add prediction as display-only using replay validation.
9. Add optional prediction-assisted reward/interrupt experiments behind an
   explicit toggle.

## Success Criteria

- The app can run live with hardware, live without hardware, and from replayed
  data.
- Reward and interrupt outputs can be tested without Tkinter, OSC, or pigpio.
- The UI explains why rewards and interrupts happen using the same data the
  engine uses.
- Prediction and telemetry consume snapshots/events instead of reading globals.
- New reward profiles can be added without editing UI code or hardware code.
- Hardware platforms can be swapped without changing reward logic.
