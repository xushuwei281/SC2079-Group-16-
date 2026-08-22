# Working car by Friday's lab — checklist

Goal: by the Week 2 lab session, an Android button press moves the
physical robot via RPi → STM32, with status feedback flowing back. This is
**not** the full autonomous run (CV/algorithm come later) — it's proving
every communication link works, since hardware/movement is already done.
CV is tracked separately below since it doesn't block or get blocked by
the comms work — it just needs to *start* today, since training runs for
hours.

Everything below assumes nothing is set up yet. Protocol draft is in
[`docs/protocol.md`](protocol.md) — read that first, it's what makes the
four tracks below independent of each other.

## Today (blocks everyone if skipped)

| # | Task | Owner | Depends on |
|---|---|---|---|
| 1 | Read & sanity-check `docs/protocol.md` as a group (15 min sync) — agree or amend before anyone starts coding against it | Everyone | — |
| 2 | Confirm STM32 board flashes and the existing move-test code still runs (you said hardware/movement already works — just confirm from a clean flash) | STM32 owner | — |
| 3 | Pair up RPi + STM32 owners for today — the RPi↔STM32 link is the critical path, get it working before anything else | RPi + STM32 owners | #1 |
| 4 | Kick off the A100 training job (`raspberry-pi/cv/scripts/train_edge.sh` and/or `train_server.sh`, after `pull_dataset.sh`) — start this in parallel today, don't wait for comms to be done first, it just needs to be running | RPi/CV owner | — |

## STM32 — robot & hardware

| Task | Target | Acceptance check |
|---|---|---|
| Wrap existing movement code behind a serial command parser matching `docs/protocol.md` (`FW:`, `BW:`, `TL:`, `TR:`, `STP`) | Today | Send `FW:50` over a USB-serial terminal (e.g. `screen`/PuTTY) directly to the board — robot moves ~50mm, board replies `ACK` then `DONE` |
| Confirm `STP` interrupts an in-progress move | Today | Send `FW:500`, then `STP` mid-motion — robot stops before completing |

## Raspberry Pi — comms hub

| Task | Target | Acceptance check |
|---|---|---|
| UART link to STM32: open serial port, send a command, read `ACK`/`DONE` | Today | Script sends `FW:50`, prints `ACK` then `DONE` received from the board |
| Bluetooth server stub for Android: accept a connection, echo whatever it receives | Today | Pair with a phone (or the AMDTOOL desktop app — see Android row below), send text, see it echoed |
| TCP client stub for Algorithm PC: connect, send a `path_request`, print whatever comes back | By Fri | Connects to the Algorithm stub server (below) and prints the dummy path |
| Wire the three together: Android command → relay to STM32 → STM32 `DONE` → relay `STATUS`/`DONE` back to Android | By Fri | Full loop test (see bottom of this doc) |

## Android — remote controller

| Task | Target | Acceptance check |
|---|---|---|
| Bluetooth device scan + connect GUI (checklist item C.2) | Today | Connect list shows the RPi, tapping it connects |
| Movement buttons sending protocol strings (checklist item C.3) — forward/back/left/right, wired to the exact strings in `docs/protocol.md` | Today | **Use the AMDTOOL desktop app** (`Technical Materials/Andriod/AMDTOOL User Guide.pdf` + `.apk`) to simulate the RPi side over Bluetooth before the real RPi code is ready — this is what it's for, don't wait on RPi being done first |
| Status TextView showing only curated `STATUS,`/`DONE` messages, not the raw stream (checklist item C.4) | By Fri | Send `STATUS,moving` from AMDTOOL/RPi — only that text appears, not a firehose of raw bytes |

## Algorithm — PC

| Task | Target | Acceptance check |
|---|---|---|
| Scaffold the arena representation (pick option 1 or 2 from the Algorithms briefing, §7–10) | Today | Can print/plot a 200×200cm grid with a hardcoded obstacle list |
| Stub TCP server: on `path_request`, reply with a hardcoded 2-point path (real pathfinding comes later — this just unblocks RPi integration) | By Fri | RPi's TCP client stub (above) connects and receives the dummy path |

## CV — image recognition

Not on the critical path for Friday's loop-test demo, but training takes
real wall-clock time, so start it today rather than after comms is done.
Full pipeline, dataset, and reasoning already written up in
[`raspberry-pi/cv/README.md`](../raspberry-pi/cv/README.md) — this is just
the "make sure it's actually running" checklist.

| Task | Target | Acceptance check |
|---|---|---|
| `hf auth login`, then `scripts/pull_dataset.sh` on the A100 (NSCC) | Today | `./data/data.yaml` present, paths fixed (no `../`) |
| Kick off `scripts/train_edge.sh` (and/or `train_server.sh`) — let it run in the background this week | Today | Job running on NSCC; don't wait on it to finish before moving on to other tasks |
| Once training finishes: `scripts/export_edge.sh` (Pi) and `export_server_onnx.sh` + `build_engine_4060.sh` (server) | Later this week | Weights pulled down via `hf download` on the Pi/4060, confirmed loadable |
| `pi_infer.py` wired to the real camera + model (currently a skeleton with `TODO`s) | Later this week | Not required for Friday's demo — full autonomous run comes after the comms loop is proven |

## Friday: full loop test

Android button → RPi relays → STM32 moves → STM32 `DONE` → RPi relays
`STATUS`/`DONE` → Android shows it. If every row above passes its own
acceptance check individually, this is mostly wiring, not new code — that's
the point of testing each link in isolation today/this week rather than
integrating everything for the first time on Friday.
