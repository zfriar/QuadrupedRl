# Quadruped Reinforcement Learning with PyBullet

![CI](https://github.com/zfriar/QuadrupedRl/actions/workflows/ci.yml/badge.svg)
[![coverage](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/zfriar/QuadrupedRl/gh-pages/badges/coverage.json)](https://github.com/zfriar/QuadrupedRl)

This repository provides a starting point for reinforcement learning with quadruped robots in simulation. It uses PyBullet and a custom Ant environment as a testbed, with the goal of exploring RL for locomotion and later transferring to real hardware.

---

## Overview

Key components:

* Custom Gymnasium environments wrapping PyBullet robots.
* Training pipeline using PPO (Stable-Baselines3).
* Configurations for reproducible experiments.
* Scripts for visualizing trained policies.
* Modular structure for adding new environments, curricula, and robots.

---

## Installation

Uses Poetry for dependency management:

```bash
pip install poetry
poetry install
poetry shell
```

## Quickstart (dev)

Run the quick developer setup and basic checks:

```bash
# install dependencies (including dev)
poetry install

# install pre-commit hooks (once)
make precommit-install

# run the pre-commit hooks across the repo (verify)
make precommit-run

# run basic checks (format, lint, typecheck)
make code-checks

# run tests
make test

# run coverage (prints missing lines)
make coverage
```


---

## Training

Train a PPO agent with:

```bash
python training/train_ppo.py --config configs/ppo_ant.yaml
```

This will:

* Train in the PyBullet Ant environment.
* Save checkpoints to `checkpoints/`.
* Save final and best models to `trained_models/`.

---

## Visualizing a Policy

```bash
python scripts/visualize_policy.py --model checkpoints/best_model.zip
```

Runs the policy in PyBullet’s GUI for inspection.

---

## Repository Structure

```
QuadrupedRl/
│
├── envs/                  # Gymnasium environments
│   ├── __init__.py
│   ├── ant_env.py
│   └── curriculum_ant_env.py
│
├── training/              # Training scripts and utilities
│   ├── train_ppo.py
│   ├── curriculum.py
│   ├── callbacks.py
│   └── utils.py
│
├── configs/               # YAML configs for experiments
│   └── ppo_ant.yaml
│
├── scripts/               # Helper scripts
│   └── visualize_policy.py
│
├── assets/                # Robot URDFs and meshes
│   └── quadruped/
│       ├── quadruped.urdf
│       └── meshes/
│           ├── base.stl
│           ├── leg_upper.stl
│           └── leg_lower.stl
│
├── tests/                 # Unit tests
│   └── unit/
│       ├── __init__.py
│       ├── conftest.py
│       ├── test_ant_env.py
│       ├── test_curriculum.py
│       └── test_configs.py
│
├── Makefile
├── pyproject.toml
└── README.md
```

---

## Tests

Unit tests cover:

* Environment behavior (`envs/`)
* Curriculum logic (`training/curriculum.py`)
* Config loading and validation (`configs/`)

Run all tests:

```bash
poetry run pytest tests/unit
```

Minimal examples in tests demonstrate correct shapes, reward calculation, curriculum progression, and config parsing. Contributors should add tests for new modules under `tests/unit/`.

---

## Theoretical Foundations

1. **Reinforcement Learning Basics** – policy gradients, actor-critic, PPO.
2. **Simulation and Robotics** – PyBullet, URDFs, kinematics.
3. **Legged Locomotion** – reward shaping, curriculum learning, domain randomization.

Useful references:

* [Spinning Up in Deep RL](https://spinningup.openai.com)
* [Stable-Baselines3](https://stable-baselines3.readthedocs.io)
* [PyBullet Quickstart](https://pybullet.org/wordpress/)
* [URDF Tutorials](https://wiki.ros.org/urdf/Tutorials)

---

## Extensions

* Add noise or randomization to environments.
* Implement curriculum stages.
* Replace Ant with your own URDF quadruped.
* Integrate logging and experiment tracking.
* Experiment with different control strategies (velocity, torque, PD).
* Vectorize environments for faster training.

---

## License

MIT License
