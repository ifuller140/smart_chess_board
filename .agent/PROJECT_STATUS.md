# Smart Chess Board - Project Status & Master Plan

> **The Central Brain**: This document tracks the heartbeat of the project. All agents should consult and update this file.

## 🧠 Strategic Initiatives

### 1. Architecture Refactor: dedicated `smart_chess_interfaces`
**Problem**: `.srv` and `.msg` definitions are potentially scattered or located inside logic packages.
**Solution**: Create a dedicated `smart_chess_interfaces` package.
**Why**: Avoid circular dependencies and cleaner build process.
- [ ] Create `smart_chess_interfaces` package
- [ ] Move `RequestMove.srv` from `chess_logic`
- [ ] Move `BoardState.msg` from `chess_perception`

### 2. "Bone Structure" to "Muscle"
**Problem**: Many nodes are empty shells (`pass`).
**Goal**: Systematically flesh out each node.
**Priority Order**:
1.  Hardware Interface (so we can move things)
2.  Gantry Control (kinematics)
3.  Perception (seeing the board)
4.  Game Logic (playing the game)

---

## 📋 Comprehensive To-Do List

### 🤖 Hardware Agent (Focus: `chess_hw_interface`, `gantry_control`)
- [ ] **Steppers**: Implement `stepper_driver_node.py` (Currently `pass`)
- [ ] **Servos**: Implement `servo_node.py`
- [ ] **Limits**: Implement `limit_switch_node.py`
- [ ] **Kinematics**: Implement `gantry_kinematics_node.py` (CoreXY math)
- [ ] **Motion**: Implement `motion_planner_node.py` (Path planning)
- [ ] **Config**: verify `pins.yaml` against actual hardware

### 🧠 Logic Agent (Focus: `chess_logic`, `chess_engine`)
- [ ] **Game Manager**:
    - [ ] Fix "In a real app" mock in `game_manager_node.py`
    - [ ] Implement actual state machine transitions
- [ ] **Engine**: Implement `chess_engine_node.py` (Stockfish integration)
- [ ] **Services**: Standardize `RequestMove` and capture triggers

### 👁️ Perception Agent (Focus: `chess_perception`)
- [ ] **Camera**: Implement `camera_node.py` (Capture frames)
- [ ] **Detection**: Implement `piece_detector_node.py` (CV logic)
- [ ] **Calibration**: Implement perspective transform logic

### 🔧 System Architect (Focus: Build, Launch, Docs)
- [ ] **Interfaces**: Create `smart_chess_interfaces` package
- [ ] **Launch**: Create master launch file to start all nodes
- [ ] **License**: Fix "TODO: License declaration" in all `package.xml` files
- [ ] **Docs**: Ensure `docs/` stays in sync with code

---

## 🔍 In-Code Annotations (Auto-Generated Audit)

| File | Context |
|------|---------|
| `src/chess_logic/.../game_manager_node.py` | `In a real app, we'd compare this msg.fen...` |
| `src/chess_logic/.../game_manager_node.py` | `In reality, we'd use the NEW FEN from perception` |
| `src/gantry_control/.../homing_node.py` | `pass` (Empty implementation) |
| `All package.xml files` | `TODO: License declaration` |

---

## 🛠 Active Workflows

To start working on an area, adopt a **Persona**:

- **"I am the Hardware Agent"**: I will ignore game logic and focus purely on making motors spin and reading sensors.
- **"I am the Logic Agent"**: I will mock hardware and focus on chess rules and state machines.

_Last Updated: 2026-01-21_
