# CARLA Manual Driving with Calm NPC Traffic

A configurable, human-friendly manual driving environment built on the CARLA Simulator, focusing on smooth control and realistic traffic behavior.

## Overview

This project implements a manual driving environment in the CARLA Simulator, focused on smooth vehicle control and realistic, calm traffic behavior.

Unlike default CARLA setups where NPC vehicles can behave aggressively or erratically, this script prioritizes:

- Natural driving feel for the human-controlled vehicle

- Predictable, human-like NPC traffic

- Clean software architecture with centralized configuration

The result is a controllable, stable simulation environment suitable for experimentation, demonstrations, and educational use.

## Key Features

### Manual Ego Vehicle Control

- Smooth throttle and brake ramping

- Speed-sensitive steering to reduce oversteer at high speeds

- Engine braking for realistic deceleration

- Keyboard-based control with reverse and emergency braking

### Chase Camera

- Third-person chase camera attached to the ego vehicle

- Automatically updates every simulation tick

### Calm NPC Traffic

- Reduced average speeds relative to speed limits

- Increased following distances

- Very low lane-change frequency

- Optional physics detuning:
  - Reduced engine torque

  - Softer braking

  - Less aggressive steering

### Pedestrians

- AI-controlled walkers

- Calm walking pace

- Random navigation targets

### Traffic Manager Integration

- Fully synchronous simulation

- Deterministic stepping

- Hybrid physics and dormant vehicle handling (CARLA version dependent)

### HUD & Debug Info

- FPS counter

- Vehicle speed (km/h)

- Real-time control values

- Key bindings reference

## Controls

**W / ↑** – Throttle  
**S / ↓** – Brake  
**A / ←** – Steer left  
**D / →** – Steer right  
**Space** – Emergency brake  
**R** – Toggle reverse  
**Esc / Q** – Quit simulation

## Architecture & Configuration

The script is structured around Python dataclasses, acting as a single source of truth for configuration:

- Camera configuration
  - Height, distance, pitch

- Ego vehicle control
  - Throttle & brake ramp speeds

  - Steering sensitivity

  - Engine braking strength

- NPC traffic
  - Vehicle & pedestrian counts

  - Speed differences

  - Following distances

  - Lane change behavior

  - Physics detuning options

- Display & performance
  - Resolution

  - FPS limits

  - Font configuration

This design makes the simulation easy to modify, extend, or experiment with without changing core logic.

## Built With

- CARLA Simulator

- Python 3.8+

- pygame

- CARLA Traffic Manager

## Requirements

- CARLA Simulator

- Python 3.8 or higher

- Anaconda (recommended)

## Anaconda Environment Setup

This project was developed and tested using Anaconda Prompt to manage the Python environment.

### Create a Virtual Environment

1. Launch Anaconda Prompt

2. Create a new Conda environment:

```bash
conda create --name carla-env python=3.12
```

3. Activate the environment:

```bash
conda activate carla-env
```

### Install Required Packages

Install the required Python packages using pip:

```bash
pip install carla
pip install pygame
pip install numpy
pip install jupyter
pip install opencv-python
```

## Usage

### Start the CARLA Simulator

Make sure the CARLA simulator is running before executing any scripts.

```bash
CarlaUE4.exe
```

> Make sure to give this time to load and let the simulator start up before trying to run a script!

### Run the Script

Navigate to the directory where the Python scripts are located
(for example, Carla/PythonAPI/examples), then run:

```bash
python drive_simulator.py
```

Ensure CARLA is running on 127.0.0.1:2000.

## Clean Shutdown

On exit, the application:

- Destroys all spawned vehicles and pedestrians

- Stops pedestrian AI controllers

- Restores original CARLA world settings

This prevents ghost actors and unstable simulator states.

## Notes

Some Traffic Manager features are CARLA version dependent

Physics detuning may not apply to all vehicle blueprints

Designed for synchronous, deterministic simulation

## Future Improvements

- Support for steering wheels and gamepads

- Recording and playback of driving sessions

- Additional camera modes

- Scenario-based traffic presets
