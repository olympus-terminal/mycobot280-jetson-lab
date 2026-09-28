# Instructions for AI coding agents (Claude Code) in this repo

These are the rules the lab ran under, condensed. If you point an agent at your own copy of this setup, keep them.

## Data integrity
- Never use synthetic, simulated or random data in place of measurements, figures or results. Random numbers are fine for
  clearly marked unit tests and ML training operations.
- Synthetic video (e.g. Cosmos outputs) is always labelled synthetic and linked to the real session it came from.
- If real data is missing, stop and ask for it.

## Lab notebook
- Keep a `NOTES.md` with a "Current status" section and dated log entries. Read it (and any plan) at the start of every session.
- Write entries **as work happens**: every hardware change, measurement, fix, decision and operator instruction, with numbers.
  The chat transcript is not the record.
- Correct wrong entries visibly (strike through, then give the correction), and commit documentation after each meaningful step.

## Provenance
- New programs and result files get a timestamp (YYYYMMDD_HHMMSS) so outputs never overwrite each other, and every result
  can be traced to the program that produced it.
- Session folders store the git hash, arguments and software versions. Keep rejected calibrations (`config/rejected/`).

## Motion safety
- No motion without the operator's standing OK. Low speed first, with a hand near the power switch. Unattended runs only when the
  operator explicitly authorises them.
- Never release the servos or cut power from a raised pose: park at REST first.
- Gate every move on the cat guard and **test its exit code**. Never chain commands after a check without checking the result.
- Temperature: don't start above the start limit, cool while *parked*, never hold a raised pose while waiting.
- Chains of moves stop on the first refusal (`scripts/chain.sh`, `set -e`).
- Plan from a fresh camera look, not an old one. Verify outcomes with the camera; "gripper opened" is not "placed".

## Working style
- Test on recorded data and in dry runs (planning + camera, no motion) before live runs.
- When explaining results: show the code and the actual numbers first, and say "I don't know" rather than guessing.
- If the operator can't type in the agent's terminal, put anything that needs them (sudo, hardware) in a short script
  they can run themselves.
