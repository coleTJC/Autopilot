# Autopilot

Autopilot is an experimental real-time EEG state-training platform.

The long-term goal is to help users enter and maintain desired mental states through feedback, reinforcement, visualization, and future prediction systems.

The project values experimentation and rapid iteration. New ideas, visualizations, reward profiles, and architectures are encouraged when they improve the user experience or help reveal useful information.

## Current major systems

* Mind Monitor OSC EEG input
* Reward engine
* Interrupt engine
* Formula visualization
* Timeline visualization
* EEG analysis and derived metrics
* Servo/tVNS output
* Buzzer feedback
* Tkinter user interface
* Session logging

## Design philosophy

Autopilot is not intended to be a black box.

The user should be able to understand what the system is doing, why rewards occur, and how mental-state estimates are being generated.

Visibility and explainability are generally preferred when practical.

However, the project is experimental and should not be constrained by existing implementations if a better architecture is discovered.

## Development guidance

When making changes:

* Understand the existing architecture before modifying it.
* Preserve useful functionality when possible.
* Prefer modular systems that can support future profiles and features.
* Be willing to refactor when the benefits are significant.
* Keep the code understandable.
* Avoid unnecessary complexity.
* Avoid blocking real-time processing or the UI thread.
* Explain major architectural changes.

## Future directions

Possible future areas include:

* State prediction
* Telemetry integration
* Additional biosignals
* New reward profiles
* Replay systems
* Session analysis
* Hardware platforms
* Portable Autopilot devices
* AI-assisted state modeling

This project is exploratory. The best solution may not yet exist inside the current codebase.
