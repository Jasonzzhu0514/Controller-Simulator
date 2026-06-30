#!/usr/bin/env python3
"""A tiny NMPC demo with no third-party dependencies.

This script demonstrates the core NMPC loop:

1. Measure the current state.
2. Enumerate possible future control sequences.
3. Predict the future with a model.
4. Score each sequence with a cost function.
5. Apply only the first control input, then repeat.

The plant is a 1D point mass with a small nonlinear drag term:

    x_dot = v
    v_dot = u - drag * v * |v|

The controller tries to move x to TARGET_X while stopping at the target.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from math import isfinite


DT = 0.1
TARGET_X = 10.0
HORIZON_STEPS = 20
SIM_STEPS = 120

U_MIN = -1.0
U_MAX = 1.0
CONTROL_LEVELS = (-1.0, -0.5, 0.0, 0.5, 1.0)
SEQUENCE_BLOCKS = 4
BLOCK_LENGTH = HORIZON_STEPS // SEQUENCE_BLOCKS

DRAG = 0.08
MAX_SPEED = 4.0

POSITION_WEIGHT = 1.0
VELOCITY_WEIGHT = 0.18
TERMINAL_POSITION_WEIGHT = 6.0
TERMINAL_VELOCITY_WEIGHT = 7.0
CONTROL_WEIGHT = 0.04
CONTROL_DELTA_WEIGHT = 0.15


@dataclass(frozen=True)
class State:
    x: float
    v: float


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def plant_step(state: State, u: float, dt: float = DT) -> State:
    """Nonlinear plant model used by both simulation and prediction."""
    u = clamp(u, U_MIN, U_MAX)
    acceleration = u - DRAG * state.v * abs(state.v)
    next_v = clamp(state.v + acceleration * dt, -MAX_SPEED, MAX_SPEED)
    next_x = state.x + next_v * dt
    return State(x=next_x, v=next_v)


def rollout(state: State, controls: tuple[float, ...], previous_u: float) -> tuple[State, float]:
    """Predict one candidate control sequence and return terminal state + cost."""
    current = state
    cost = 0.0
    last_u = previous_u

    for step, u in enumerate(controls):
        u = clamp(u, U_MIN, U_MAX)
        current = plant_step(current, u)

        position_error = current.x - TARGET_X
        terminal = step == len(controls) - 1
        position_weight = TERMINAL_POSITION_WEIGHT if terminal else POSITION_WEIGHT
        velocity_weight = TERMINAL_VELOCITY_WEIGHT if terminal else VELOCITY_WEIGHT

        cost += position_weight * position_error * position_error
        cost += velocity_weight * current.v * current.v
        cost += CONTROL_WEIGHT * u * u
        cost += CONTROL_DELTA_WEIGHT * (u - last_u) * (u - last_u)

        last_u = u

    return current, cost


def expand_block_controls(block_controls: tuple[float, ...]) -> tuple[float, ...]:
    """Turn a short sequence into a horizon-length piecewise-constant sequence."""
    controls: list[float] = []
    for u in block_controls:
        controls.extend([u] * BLOCK_LENGTH)
    while len(controls) < HORIZON_STEPS:
        controls.append(block_controls[-1])
    return tuple(controls[:HORIZON_STEPS])


def solve_nmpc(state: State, previous_u: float) -> tuple[float, float, State, int]:
    """Brute-force shooting NMPC.

    For a teaching demo we enumerate piecewise-constant sequences. This keeps
    the demo fast while still showing the NMPC idea: predict multiple futures,
    score them, then apply only the first control.
    """
    best_u = 0.0
    best_cost = float("inf")
    best_terminal = state
    candidate_count = 0

    for block_controls in product(CONTROL_LEVELS, repeat=SEQUENCE_BLOCKS):
        candidate_count += 1
        controls = expand_block_controls(block_controls)
        terminal, cost = rollout(state, controls, previous_u)
        if cost < best_cost:
            best_cost = cost
            best_u = controls[0]
            best_terminal = terminal

    if not isfinite(best_cost):
        return 0.0, float("inf"), state, candidate_count
    return best_u, best_cost, best_terminal, candidate_count


def main() -> None:
    state = State(x=0.0, v=0.0)
    previous_u = 0.0

    print("step     x      v      u      cost     predicted_terminal_x  candidates")
    print("----  ------  ------  -----  --------  --------------------  ----------")

    for step in range(SIM_STEPS):
        u, cost, terminal, candidate_count = solve_nmpc(state, previous_u)
        state = plant_step(state, u)
        previous_u = u

        if step % 2 == 0 or abs(state.x - TARGET_X) < 0.05:
            print(
                f"{step:4d}  "
                f"{state.x:6.2f}  "
                f"{state.v:6.2f}  "
                f"{u:5.2f}  "
                f"{cost:8.2f}  "
                f"{terminal.x:20.2f}  "
                f"{candidate_count:10d}"
            )

        if abs(state.x - TARGET_X) < 0.03 and abs(state.v) < 0.05:
            print(f"\nReached target at step {step}.")
            break


if __name__ == "__main__":
    main()
