# AGENTS.md — AILamp (branch `post-course`)

## What this repository is

- **AILamp** is a 5-DOF ST3215 desk lamp on a Jetson Nano. It is built on LeLamp and lelamp_runtime by Human Computer Lab (GPL-3.0); see NOTICE.md.
- **The course deliverable** is TUM INHN0018 Group 8; see TEAM.md. It is frozen in `CPSCourse-TUM-HN/TUM-HN-Team8_AILamp` at commit `546cbeb`.
- **Branch `post-course`** continues the software after the course.
  - Never push to the course repository.
  - Keep the TEAM.md and NOTICE.md attributions intact.

## Environment and commands

- **Python:** 3.11 (same as CI).
- **Install:** `pip install uv && uv sync --extra test --extra simulation`
- **Tests:** `uv run pytest -q`
- **Simulation tests** (available once P2-7 is done): `MUJOCO_GL=osmesa uv run pytest -m sim -q`
- **Coverage:** `uv run pytest --cov=ailamp --cov-branch --cov-report=term-missing`
- **Static hardware check:** `uv run ailamp hardware-check`
- **Lockfile:** check it with `uv lock --check`. When you add a dependency, run `uv lock` and commit the updated lockfile.

## Ground rules

1. **No hardware is available.**
   - Do not claim hardware validation anywhere: not in code, docs, commit messages or PR text.
   - Anything verified only in simulation or with fakes must say so.
   - The README "Verification status" table (task P2-11) must stay accurate.
2. **Units.** Joint targets are LeLamp normalized units (−100…100), not degrees.
   - Use explicit names such as `delta_units` and `step_deg`.
   - Convert between units and degrees through the calibration file.
3. **Single writer.** `LampController` is the only writer to the motors. No new code path may drive the motors or the LED board directly.
4. **Shared safety constraints.** The LLM decision layer and the voice agent share one set of safety constraints:
   - the tool whitelist;
   - the per-joint delta limit;
   - at most one motor action per plan;
   - the generation, deadline and staleness checks.

   Reuse them. Never loosen or duplicate them.
5. **Test first.**
   - Hardware-facing tests use fakes (`FakeBus`, `FakeSerial`, `FakeMotor`, `FakeLed`).
   - MuJoCo tests carry `@pytest.mark.sim` and skip cleanly when mujoco is not installed.
6. **Never commit secrets.** The maintainer does real OpenAI and LiveKit runs locally; tests use fakes.
7. **Commits and PRs.**
   - Keep the existing code style and make small, reviewable commits.
   - One task per PR.
   - The PR description states what was verified and how.

## Tasks

See `docs/codex/TASKS.md`: tasks P2-1 to P2-11, in order.
