# Go2 Open Object Kinds (shared step, then the Gazebo detector path A) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Object kinds come from one data file (`voice_ltl/vocabulary.json`, 50 household kinds) on every path; big furniture gets near cells measured from its edge; and in Gazebo a YOLOE container can publish `/go2/objects` from the rendered front camera and the LiDAR (`OBJECTS_SOURCE=detector`), with the logical camera kept as ground truth.

**Architecture:**
- A stdlib-only loader (`voice_ltl/vocabulary.py`) checks the file at import and fails closed. `objects.py` builds the id/proposition allowlist, names, sizes, word matching and messages from it. The translator, the grid, the page, the fake Go2, the Gazebo bridge and the detector all read it the same way.
- The grid blocks by footprint distance and adds near band B (edge/core based) beside today's band A. The language side adds one reply field (`unresolved_objects`), a word note, a kind guard and count messages. Layout files (`gazebo/layouts/*.json`) replace the hard-coded box and chair.
- Part A is a separate `detector` compose service: a pure numpy core (projection, clustering, tracking, forgetting) behind a swappable `Detector` protocol, with YOLOE-11s (ultralytics, AGPL-3.0) in one file and one image. The bridge publishes the truth on `/gazebo/objects_truth` and publishes `/go2/objects` only in logical mode.

**Tech Stack:** Python 3.10 (app, ROS 2 Humble) and 3.12 (gazebo and detector images, ROS 2 Jazzy); numpy (1.26 in app and gazebo, 2.x in the detector venv); pytest and the node harness of `tests/test_server.py`; Gazebo Harmonic, ros_gz_bridge; ultralytics 8.4.210 YOLOE-11s text-prompt (detector image only); docker compose.

**Spec:** `docs/superpowers/specs/2026-11-11-go2-open-objects-design.md` (v2, commit `06c7dd5`; "the spec"). Read it with:
- `~/work/physicalAI/go2/.superpowers/sdd/2026-11-11-go2-open-objects/decisions.md`: D1–D11 and **"Rulings after the builder check"** (Q2, Q4, C1–C6 and the user's answers). **The rulings override the spec where they differ.**
- In the same folder: `builder-check.md` (its minor defaults M1–M15 are adopted where a task says so), `plan-review.md` (its findings I1–I8 and M1–M15 are folded into this plan; the rulings it asked for are taken by default and listed in "Notes for the controller" at the end; this plan cites them as "review I1", "review M7" and so on, apart from builder-check's M1–M15 and the spec's measurements M0–M5), `design-near.md`, `design-vocabulary.md`, `design-language.md`, `design-detector.md`, `vocabulary.draft.json`, `code-map.md`, `spike/gazebo-camera.md`, `spike/detector.md`. Where the spec and a part design disagree, the spec wins (M1); `design-language.md` was written for vocabulary rev 1, so its `desk → table` and `bookshelf → shelf` examples are replaced by the spec's rev-2 values (`lookup("desk") == "desk"`, 本棚 → bookshelf, 戸棚 → cabinet).
- The objects design `docs/superpowers/specs/2026-11-05-go2-objects-design.md` ("the objects design"): everything this spec does not change, and its §2 acceptance numbers.

## Global Constraints

- Branches: work only on `feat/open-objects` and task branches `objects2/<task>` cut from it; never push; no PR; nothing goes into `main` (D10).
- Commits: `git add` names only the task's own files (never `-A` or `.`); every commit message ends with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`; keep the repo's configured author.
- Secrets: never open, print, copy or commit `env.txt`; never print environment variables; never run `docker compose config` (it prints `env_file` values).
- Paths: no real user name, `/Users/<name>` path or temp path in any tracked file; write `~`.
- The user's stack: never stop, restart or wait on the user's containers; our stacks run as compose project `go2oo` with `GO2_GAZEBO_PORT=6081` and `GO2_UI_PORT=8811` (D11, O41).
- Heavy work (image builds, Gazebo runs, the detector on images, real-Claude trials, the slow near sweep) happens only in the `[HEAVY]` steps of Wave 3, one at a time, only while `go2-gazebo-1` is not running, recording the other containers' CPU and the power state before and after (D11, the user's answer).
- `voice_ltl/vocabulary.py`, `voice_ltl/objects.py` and `voice_ltl/layout.py` import only the standard library (the Jazzy bridge and the detector import them).
- Only `detector/yoloe*.py` imports `ultralytics` or `torch`; neither enters the app or gazebo image (D5). Other `detector/*.py` (except `node.py`) import only the stdlib, numpy and this repo, and run under numpy 1.26 and 2.x.
- Formulas without object propositions behave exactly as today: the 12-state `go2_world`, the same plans and messages, no slow-down.
- The default layout World is unchanged: 8 near keys and 9 blocked cells for each of `box_1` and `chair_1`; PLANT sizes 1 704 / 1 704 / 9 984 / 12 000 / 28 416 for `F near_chair_1`, `F near_box_1`, `F around_box_1`, `F(around_box_1 & X F near_chair_1)`, `F(around_box_1 & X F around_cw_box_1)` (spec §10.7).
- With `OBJECTS_SOURCE=logical` the robot description and the main `ros_gz_bridge` arguments are byte-identical to `0f6e06a` (spec §2 goal 7).
- Vocabulary v1 = `vocabulary.draft.json` rev 2 verbatim: 50 kinds, 165 words, 87 `detect`, `detect_ignore` person/dog/cat, 44 `en`, `not_words` 長椅子・車椅子・テレビ番組.
- Labels: 1–4 segments `[a-z][a-z0-9]*` joined by `_`, ≤ 24 characters, first segment not `cw` or `sim`, no segment in {near, around, cw, to, last, true, false, end} ∪ the segments of `world.ACTION_NAMES`, `PROPOSITIONS` and `UNCONTROLLABLE` (a set built from the code, never a pinned count; C5).
- Ids `<label>_<n>`, n in 1..99 without a leading zero; never reused within one bridge or detector run; numbering restarts when Gazebo or the detector restarts; a label whose ids run out ignores new objects with one note (C6).
- Vocabulary fields: `ja` 1–12 characters; `words` 1–20 per kind, ≤ 20 characters each; kana, kanji, ー and 々 only, unchanged by NFKC; one-character words only 箱 棚 机 籠 鞄; `detect` 1–3 per kind matching `[a-z][a-z -]{0,39}`; `en` ≤ 8; `size` = [long, short], each in (0, 3.0]; `height` in (0, 3.0]; ≤ 200 kinds; a warning above 100 `detect` prompts.
- Prompt: at most 8 listed objects (`MAX_PROMPT_OBJECTS`), the instruction's kinds first; word note ≤ 4 words; at most 10 `unresolved_objects` items read, label ≤ 64 characters; `### 断るもの` unchanged; the 30 general examples keep 3 keys.
- Only vocabulary `ja` and the user's matched vocabulary words reach the prompt, page and messages: never detector text or Claude's English (spec §4, O3, R21). `source`, `confidence` and `measured` never reach the page or the prompt (O23).
- Object distance ≤ 6 m (`MAX_OBJECT_DISTANCE`); `MAX_SIZE` = 3.0 m.
- Grid (spec §10, §16): blocked iff `edge_distance` ≤ 0.45; band A unchanged (r + 0.75 … min(r + 1.5, 1.3), heading within 40°); band B: d_e ≥ 0.65, d_k ≤ 1.075 (small) or 1.25 (large), for small objects also d_c ≥ 1.05, heading within 40° of the nearest spine point; small iff both sides ≤ 0.6 m; ring r + 0.75 … r + 1.75 and `LAP_DONE` 5 unchanged; ≤ 256 cells with a 1-cell margin (3 once on R15, O15); a lap is refused by size alone when r ≥ 1.75.
- Acceptance lines (spec §10.8): near — final d_e to the target ≤ 1.5 m, some footprint point within 45° of the final yaw, least d_e to every footprint over the run ≥ 0.30 m (final d_e ≥ 0.50 reported); lap — winding in [0.9, 1.5), least d_e to the target ≥ 0.6 m; the objects design's §2 keys for `box_1`/`chair_1` unchanged; the box's centre check (1.5 m / 45°) in logical mode only (O16, O28).
- Bridge sizing: measuring radius max(0.6, hypot(L, S)/2 + 0.3) m; a first size of [L, L] with `measured: false`, `measured: true` after the first LiDAR size; fake Go2 rows carry no `measured`/`confidence` and stay byte-identical (O43).
- Wire: `/go2/objects` at 2 Hz, stale after 2 s, `frame: odom`; optional message `source` ∈ {logical, detector, fake} (any other value drops the message); optional row `confidence` (finite, in [0, 1], not a bool) and `measured` (bool), a bad value drops the row.
- Camera (D7 ruling): 640 × 480 `R8G8B8`, hfov 2.094 rad, 2 Hz sim time, periodic, clip 0.1–10 m, no noise, on the existing `logical_camera` link; K: fx = fy = 184.836, cx 320, cy 240.
- Still gate: today's `bridge_core.StillGate` (each angular rate ≤ 0.1 rad/s, speed ≤ 0.05 m/s, for ≥ 0.3 s sim time).
- Detector (spec §16): thresholds sighting 0.25, confirm 0.35, unsupported 0.5, predict floor 0.10; confirmation 2 of the last 3 in-view still frames ≥ 0.5 s sim apart (one LiDAR-supported, one ≥ confirm), unsupported 3 of 3 ≥ 0.5; tentative TTL 30 s; association 0.5 + 0.5·max(r) m (+0.5 for a fallback); vote IoU 0.4; merge IoU 0.5 or a centre inside; relabel when another label sums ≥ 2× over the last 5 hits; `visible` within max(5, 2 × idle period) s; vacant when n_in < 3 and n_through ≥ 5, retired after 2 vacant scans ≥ 0.5 s sim apart; one inference in flight, gap max(1.0 s, p50 of the last 10), 3 settle frames then every 3.0 s (doubled while p50 > 1.5 s, ≤ 12 s); camera timeout 10 s; 3 errors in a row → 30 s pause; 1 torch thread; compose `cpus: "2.0"`, `restart: on-failure:3`.
- Images: detector `go2-voice-ltl-detector:yoloe-11s` (base `ros:jazzy-ros-base-noble`, venv `/opt/det`, torch 2.14.1, torchvision 0.29.1 from the CPU index, ultralytics 8.4.210, CLIP `7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40`, weights and sha256 as spec §11.1); gazebo `go2-voice-ltl-gazebo:patch-0005-fuel2` (the room fix, R-H3g; `patch-0005-fuel1`, the first room's image, is kept and never built over) on top of `patch-0005`, which is never rebuilt; Fuel zips pinned by sha256 (spec §12.2).
- Accuracy lines (spec §13.3): target position 0.15 m, any position 0.35 m, median position 0.25 m, size 0.4 m, label accuracy 0.8, recall 0.6, at most 1 false positive; with the oracle every position and size line holds and `covers` holds on the observed extent (C2).
- Trials (spec §13.2, O10): 70 cases; exit 0 when lap, near, kinds and refusals are each ≥ 0.9; acceptance = two runs with lap 10/10, near 10/10, kinds ≥ 9/10, refusals ≥ 13/14, regression ≥ 12/13.
- Timing (spec §13.5): inference median ≤ 0.6 s and p90 ≤ 0.8 s; still → listed ≤ 5 s; RTF has no line of its own, the lap and near runs must pass at the measured RTF (D7 ruling).
- Japanese user-facing copy is copied exactly from this plan or the spec section it names.
- Computed numbers (PLANT sizes, key counts, areas) are pinned as given. If the code gives another value, stop and report it to the controller (spec §15: numbers may be corrected with a reason; rules, lines and copy may not change without the controller).
- If a test outside your task's files fails because of your change, stop and report; do not edit another task's file.

## Review Focus

The five inputs most likely to bite a person that no spec-listed test exercises, most likely first. Each has a test in the task that owns the code.
1. **A hand-edited `vocabulary.json` with a mistake** (a trailing comma, a typo key, a word of another kind). The whole point of D1 is that people add kinds by editing the file. The app must refuse to start with one message that names the file, the rule, the label and the value (line and column for JSON errors), not a traceback. Test: `test_errors_name_the_file_rule_label_and_value` (Task 1.1).
2. **Gazebo restarted while the detector keeps running.** The docs tell users to restart Gazebo after a fall; sim time then jumps back to 0 and a stale `StillGate.still_since` would block every detection forever. The detector must reset its gate, pose buffer and tracks, list objects again within the normal latency, and restart its ids. Test: `test_sim_time_going_backwards_resets_gate_buffer_and_tracks` (Task 2.7).
3. **`OBJECTS_SOURCE=detector` without `--profile detector`** (or the detector stopped for good). Every object instruction then says 「見つかりません」 with no hint. The bridge must log, every 10 s, that nobody publishes `/go2/objects`, naming `--profile detector`. Test: `test_publisher_note_names_the_missing_detector` (Task 2.3).
4. **Spaces in the instruction** (typed on the page, or a Whisper output) between the object word and まで / 行って, e.g. 「ゴミ箱 まで　行って」. The count messages and the kind guard must behave as without spaces; otherwise `F near_box_1` walks to the box. Test: `test_kind_guard_and_count_messages_ignore_spaces` (Task 1.4).
5. **A wrong `GO2_LAYOUT` name, or a layout model the image lacks** (an older `GO2_GAZEBO_IMAGE` without the Fuel models). The bridge must stop before spawning anything, with one log line naming the bad value and the valid choices, instead of a half-furnished world. Tests: `test_unknown_layout_names_the_valid_layouts` (Task 1.6) and `test_missing_model_folder_stops_the_bridge_before_spawning` (Task 2.3).

---

## Execution

### Waves, stages and merging

| Wave / stage | Tasks (parallel within a stage) | Cut from |
|---|---|---|
| 1a | 1.1 Vocabulary file and loader | `feat/open-objects` (`0f6e06a` + docs) |
| 1b | 1.2 Object core | 1a merged |
| 1c | 1.3 Grid near band · 1.4 Language · 1.5 Page · 1.6 Layout files and fake Go2 | 1b merged |
| 1d | 1.7 Detour rebuild (O15) · 1.8 Phrasing trials script | 1c merged |
| 2a | 2.1 Detector camera and geometry · 2.2 Detector protocol and tracker · 2.3 Bridge and compose for Gazebo · 2.4 objects_check · 2.5 Household layout · 2.6 Fuel image | wave 1 merged |
| 2b | 2.7 Detector core and oracle · 2.8 Detector image and compose service · 2.9 Gazebo camera (xacro, launch) | 2a merged |
| 2c | 2.10 Detector node · 2.11 YOLOE adapter · 2.12 Recording and replay tools | 2b merged |
| 3 `[HEAVY]` | H1 … H7, strictly one at a time (order below; each step's header says what it waits for) | the previous step merged |
| 4 | 4.1 Docs | H6 merged and H4 passed |

- Each task runs in its own worktree on branch `objects2/<short-name>` (the name is given in each task header), cut from `feat/open-objects` at the start of its stage. The controller merges with `git merge --no-ff`, one branch at a time in task-number order, and runs the full suite after each merge.
- A stage is the unit that runs in parallel, and its tasks own disjoint files (table below). A file that two stages of one wave touch (`compose.yaml`, `tests/test_detector_protocol.py`) is edited by the later stage only after the earlier stage has merged.
- Wave-3 steps run in their own worktree too (`objects2/h<k>`), cut after the previous step merged, and commit only records and the pins they are told to change.
- Heavy-step order: H4 (trials; first, as soon as wave 1 is merged) → H1 (Fuel image; after Task 2.6 merged) → H7 (slow tests; after wave 2 merged) → H2 (detector image) → H3 (detection preview) → H5 (S7, logical Gazebo) → H6 (A7, detector Gazebo). H1 → H2 → H3 → H5 → H6 is fixed (spec §14: A6 before S7). H4 may move later but must pass before H5 (the H4 gate below). H7 may move later, but only to a time when no code task of ours runs a `T` suite and no other heavy step runs (its timing asserts need a quiet CPU); after wave 2 is merged no code task runs until Task 4.1.
- **H4 gate (ruling on the O10 timing; see "Notes for the controller").** Tasks 1.4 and 1.8 merge into `feat/open-objects` at stages 1c and 1d as planned: Task 1.8 and wave 2 are built on them, and nothing in wave 2 depends on the prompt text. H4 gates the end of the shared step instead: H5 (S7) and Task 4.1 do not start until both H4 runs have met the O10 acceptance. If H4 misses, H4 stops and reports every miss with its numbers, and the controller picks one of:
  - fix forward: a prompt change (spec §9.2's first lever, the hypernym example) on `objects2/h4-fix`, its full suite, merge, then H4 again (two new runs);
  - revert: `git revert -m 1` of the Task 1.8 merge commit, then of the Task 1.4 merge commit, on `objects2/h4-revert`, its full suite, merge, and a new plan for the language part.
  Wave-2 code tasks and H1–H3 go on meanwhile.

### File ownership

| Stage | Task | Files (create or modify) |
|---|---|---|
| 1a | 1.1 | `voice_ltl/vocabulary.json`, `voice_ltl/vocabulary.schema.json`, `voice_ltl/vocabulary.py`, `tests/test_vocabulary.py`; small hooks in `voice_ltl/objects.py`, `voice_ltl/world.py`, `voice_ltl/translator.py` |
| 1b | 1.2 | `voice_ltl/objects.py`, `voice_ltl/ros_link.py`, `tests/test_objects.py`, `tests/test_ros_link.py`; pins its change moves in other test files (it is alone in its stage) |
| 1c | 1.3 | `voice_ltl/grid.py`, `tests/test_grid.py`, `tests/test_session_objects.py` |
| 1c | 1.4 | `voice_ltl/translator.py`, `voice_ltl/prompt.py`, `voice_ltl/session.py` (only `_redacted`), `tests/test_object_language.py`, `tests/test_grammar_translator.py`, `tests/test_llm.py`, `tests/test_session_language.py` (new) |
| 1c | 1.5 | `voice_ltl/static/app.js`, `voice_ltl/static/style.css`, `tests/test_server.py` |
| 1c | 1.6 | `voice_ltl/layout.py`, `gazebo/layouts/default.json`, `docker/fuel_models.json`, `sim/sim_core.py`, `sim/fake_go2.py`, `compose.yaml`, `tests/test_layout.py`, `tests/test_sim_objects.py`, `tests/test_sim_core.py` |
| 1d | 1.7 | `voice_ltl/session.py`, `tests/test_session_detour.py` |
| 1d | 1.8 | `scripts/object_phrasing_trials.py`, `tests/test_phrasing_trials.py` |
| 2a | 2.1 | `detector/__init__.py`, `detector/camera.py`, `detector/geometry.py`, `tests/test_detector_geometry.py` |
| 2a | 2.2 | `detector/protocol.py`, `detector/tracker.py`, `detector/scripted.py`, `tests/test_detector_tracker.py`, `tests/test_detector_protocol.py` |
| 2a | 2.3 | `gazebo/bridge_core.py`, `gazebo/gazebo_bridge.py`, `compose.yaml`, `tests/test_gazebo_objects.py`, `tests/test_bridge_core.py` |
| 2a | 2.4 | `gazebo/tools/objects_check.py`, `tests/test_objects_check.py` |
| 2a | 2.5 | `gazebo/layouts/household.json`, `tests/test_household.py`, `tests/test_sim_household.py` |
| 2a | 2.6 | `docker/Dockerfile.gazebo-fuel`, `docker/fetch_fuel_models.py`, `gazebo/tools/model_extent.py`, `tests/test_fuel_image.py`, `tests/fixtures/models/` |
| 2b | 2.7 | `detector/core.py`, `detector/oracle.py`, `tests/test_detector_core.py`, `tests/test_detector_oracle.py` |
| 2b | 2.8 | `docker/Dockerfile.detector`, `compose.yaml`, `pytest.ini`, `tests/test_detector_image.py` |
| 2b | 2.9 | `gazebo/go2_sensors.urdf.xacro`, `gazebo/launch/go2_gazebo.launch.py`, `gazebo/launch_config.py`, `tests/test_gazebo_camera.py` |
| 2c | 2.10 | `detector/node.py`, `tests/test_detector_node.py` |
| 2c | 2.11 | `detector/yoloe.py`, `tests/test_detector_yoloe.py` |
| 2c | 2.12 | `gazebo/tools/record_frames.py`, `gazebo/tools/detector_check.py`, `gazebo/tools/png_rgb.py`, `tests/test_detector_check.py`, `tests/fixtures/detector/` |
| 3 | H1–H7 | `docs/reference/gazebo_open_objects.md` (new), `docs/reference/object_phrasing_trials.md`, and the pins each step names |
| 4 | 4.1 | `README.md`, `docs/simulation.md`, `docs/design.md`, `docs/vocabulary.md`, the two specs, `tests/test_docs_open_objects.py`, doc pins in `tests/test_gazebo_objects.py` |

### Commands

The commands are shell functions, not text macros, so a variable written in front of one (`OBJECTS_SOURCE=detector G up …`) reaches compose. Each tool call starts a fresh shell: put the definitions at the top of the same command, or write them once into a throwaway file in the scratch area (never added to git) and `source` it. `WT` is the task's worktree (absolute path; written `<wt>` below) and `DOMAIN` the task's ROS domain id from the table.

```bash
WT=<wt>; DOMAIN=<id from the table>
# TA <command>: any command in the app image (rulings Q2: the main checkout's compose project, the worktree mounted at /wt)
TA() { (cd ~/work/physicalAI/go2 && docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID="$DOMAIN" \
          -v "$WT":/wt -w /wt -e PYTHONPATH=/wt:/wt/nl_control app "$@"); }
# T <pytest args>: the light test run
T()  { TA python3 -m pytest -q -m "not slow" "$@"; }
# TD <pytest args>: the detector image (only in Wave 3)
TD() { docker run --rm -v "$WT":/work -v /dev/null:/work/env.txt:ro -w /work \
          go2-voice-ltl-detector:yoloe-11s python3 -m pytest -q "$@"; }
# G <compose args>: our Gazebo stack (only in Wave 3); always `up -d --no-build`
G()  { (cd "$WT" && GO2_GAZEBO_PORT=6081 GO2_UI_PORT=8811 docker compose -p go2oo "$@"); }
```

- **`T`:** `PYTHONPATH=/wt:/wt/nl_control`, so a bare `TA python3 -c "import detector.geometry"` finds `go2_nl_control` (imported by `gazebo/bridge_core.py`); pytest also gets it from `tests/conftest.py`. Before the first run in a worktree, check the import: `TA python3 -c "import voice_ltl; print(voice_ltl.__file__)"` must print `/wt/voice_ltl/__init__.py`.
- **Full suite:** `T tests nl_control/tests`. Expected: `0 failed`. Every task ends with it.
- **`G`:** our compose project `go2oo` (its own network, so its default ROS domain cannot meet the user's stack). It has no Claude login (the worktree has no `env.txt`); that is enough for everything except the voice step of H6, which has its own command there.
- **ROS domain ids** (`DOMAIN`): one per task or step, all in 1..101 (ROS 2's safe range on Linux; higher ids can collide with ephemeral ports), never 0 (the default of the user's stack, whose network `T` shares) and never 77 (the id of the user's measurement commands):

  | Task / step | id | Task / step | id | Task / step | id |
  |---|---|---|---|---|---|
  | 1.1 | 31 | 2.1 | 41 | H1 | 61 |
  | 1.2 | 32 | 2.2 | 42 | H2 | 62 |
  | 1.3 | 33 | 2.3 | 43 | H3 | 63 |
  | 1.4 | 34 | 2.4 | 44 | H4 | 64 |
  | 1.5 | 35 | 2.5 | 45 | H5 | 65 |
  | 1.6 | 36 | 2.6 | 46 | H6 | 66 |
  | 1.7 | 37 | 2.7 | 47 | H7 | 67 |
  | 1.8 | 38 | 2.8 | 48 | 4.1 | 71 |
  | — | — | 2.9 | 49 | controller's merge runs | 81 |
  | — | — | 2.10 | 50 | fix branches (`objects2/*-fix`) | 82 |
  | — | — | 2.11 | 51 | — | — |
  | — | — | 2.12 | 52 | — | — |
- **D11 gate**, before every `[HEAVY]` step and before every Gazebo run inside one: `docker ps --format '{{.Names}}'` must not list `go2-gazebo-1` (otherwise do not start; try again later; never stop it), and no other heavy step of ours may be running. Record `docker stats --no-stream --format '{{.Name}} {{.CPUPerc}} {{.MemUsage}}'`, `pmset -g batt` and `pmset -g custom | grep -i lowpower` before and after into the step's record.

---

## Wave 1: the shared step

### Task 1.1: Vocabulary file and loader (stage 1a, branch `objects2/vocabulary`)

**Files:**
- Create: `voice_ltl/vocabulary.json` (copy of `~/work/physicalAI/go2/.superpowers/sdd/2026-11-11-go2-open-objects/vocabulary.draft.json`, byte for byte), `voice_ltl/vocabulary.schema.json` (`design-vocabulary.md` §3.1, verbatim), `voice_ltl/vocabulary.py`, `tests/test_vocabulary.py`
- Modify: `voice_ltl/objects.py` (move `_KATAKANA`, `_HIRAGANA`, `_KANJI`, `_PARTICLES_BEFORE`, `_PARTICLES_AFTER`, `_script`, `word_start`, `word_end`, `_word_pattern` into `vocabulary.py` unchanged and import them back; nothing else), `voice_ltl/world.py` (one check at import), `voice_ltl/translator.py` (one check at import)

**Interfaces:**
- Consumes: `world.ACTION_NAMES`, `world.PROPOSITIONS`, `world.UNCONTROLLABLE`, `translator._handles_object` (all unchanged).
- Produces (`voice_ltl/vocabulary.py`, stdlib only):

```python
VOCABULARY_PATH: Path                      # voice_ltl/vocabulary.json
MAX_KINDS = 200; MAX_SIZE = 3.0; DETECT_WARN = 100
LABEL_SHAPE = re.compile(r"(?!(?:cw|sim)(?:_|$))[a-z][a-z0-9]*(?:_[a-z][a-z0-9]*){0,3}")   # fullmatch, len <= 24
FIXED_RESERVED = frozenset({"near", "around", "cw", "to", "last", "true", "false", "end"})
ONE_CHAR_WORDS = frozenset("箱棚机籠鞄")
SUFFIXES = ("", "まで", "の周り", "のところ", "の前", "を", "に", "へ")        # rule 5
class VocabularyError(ValueError): ...      # str(e) == "<file name>: rule <k>: <label or '-'>: <value>: <reason>"
@dataclass(frozen=True)
class Kind: label: str; word: str; start: int; end: int     # word in the vocabulary's spelling; offsets on the NFKC, masked text
@dataclass(frozen=True)
class Entry: label: str; ja: str; words: tuple[str, ...]; detect: tuple[str, ...]; en: tuple[str, ...]
             size: tuple[float, float]; height: float
@dataclass(frozen=True)
class Vocabulary:
    entries: tuple[Entry, ...]; not_words: tuple[str, ...]; detect_ignore: tuple[str, ...]
    labels: frozenset[str]; detect: Mapping[str, str]           # detect prompt -> label
    def entry(self, label: str) -> Entry | None
    def name_of(self, label: str) -> str                        # ja, else label.replace("_", " "); never raises
    def lookup(self, name: object) -> str | None                # spec §9.4; never raises
    def kinds_in(self, text: str) -> tuple[Kind, ...]           # spec §9.5
def normalise_alias(name: str) -> str                          # NFKC, lower, strip, runs of [\s-] -> "_", strip "_"
def parse(data: object, source: str = "vocabulary.json") -> Vocabulary   # spec §7.3 rules 1-5 and 7-11
def load(path: Path = VOCABULARY_PATH) -> Vocabulary           # JSON errors -> VocabularyError with line and column
def check_reserved_segments(vocab: Vocabulary, segments: Iterable[str], source: str) -> None   # rule 2 (world part)
def check_words(vocab: Vocabulary, ok: Callable[[str], bool], rule: int, reason: str) -> None   # rule 6
VOCABULARY: Vocabulary = load()                                # at import: fail closed
LABELS = VOCABULARY.labels; DETECT = VOCABULARY.detect; DETECT_IGNORE = frozenset(VOCABULARY.detect_ignore)
name_of = VOCABULARY.name_of; lookup = VOCABULARY.lookup; kinds_in = VOCABULARY.kinds_in
def size_of(label: str) -> tuple[float, float] | None
def height_of(label: str) -> float | None
word_start, word_end, _word_pattern                            # moved from objects.py, unchanged; objects.py re-exports them
```
- `voice_ltl/world.py` gains `RESERVED_LABEL_SEGMENTS: frozenset[str]` (every `_`-segment of `ACTION_NAMES + PROPOSITIONS + UNCONTROLLABLE`) and calls `vocabulary.check_reserved_segments(vocabulary.VOCABULARY, RESERVED_LABEL_SEGMENTS, "voice_ltl/world.py")` at import. `translator.py` calls `vocabulary.check_words(vocabulary.VOCABULARY, lambda w: not _handles_object(w), 6, "reads as a handling verb")` right after `_handles_object`. So the app fails closed on rules 2 and 6 too; the bridge and the detector, which never parse formulas, check rules 1–5 and 7–11.

- [ ] **Step 1: Write the failing tests** in `tests/test_vocabulary.py`.

```python
DATA = json.loads(V.VOCABULARY_PATH.read_text(encoding="utf-8"))

def test_committed_file_loads():
    v = V.VOCABULARY
    assert len(v.entries) == 50 and sum(len(e.words) for e in v.entries) == 165
    assert sum(len(e.detect) for e in v.entries) == 87 and sum(len(e.en) for e in v.entries) == 44
    assert v.detect_ignore == ("person", "dog", "cat") and v.not_words == ("長椅子", "車椅子", "テレビ番組")
    assert {"box", "chair", "table", "desk", "bookshelf", "refrigerator", "trash_can", "potted_plant",
            "sofa", "lamp", "suitcase", "ball", "television"} <= V.LABELS
    assert V.name_of("trash_can") == "ゴミ箱" and V.name_of("no_such") == "no such"
    assert V.size_of("sofa") == (1.8, 0.9) and V.height_of("chair") == 0.9 and V.DETECT["couch"] == "sofa"

@pytest.mark.parametrize("rule,mutate", BROKEN)        # one copy per row, each breaks exactly one rule
def test_each_rule_rejects_a_broken_copy(rule, mutate):
    data = copy.deepcopy(DATA); mutate(data)
    with pytest.raises(V.VocabularyError, match=rf"rule {rule}:"):
        V.parse(data)
```
  `BROKEN` rows (write each as a small function): rule 1 — `version` 2; an unknown key `word` in an entry; an unknown top-level key. Rule 2 — label `Box`; `cw_box`; `sim_box`; `near_box`; a 25-character label; a duplicate label; `box` removed; `chair` removed. Rule 3 — `ja` 「椅子」 with brackets; `ja` not among its words; a 13-character `ja`. Rule 4 — word `TV`; a 21-character word; word 本; a word of another kind (椅子 added to `sofa`); a `not_word` used as a word; 21 words in one kind. Rule 5 — `ball` gets the word 箱を (so 「箱を」 resolves to ball). Rule 7 — detect `Chair`; a 41-character prompt; another kind's prompt; `person`; 4 prompts. Rule 8 — 9 `en`; `en` `couch` on `bench` (an alias of `sofa`). Rule 9 — size `[0.5, 3.1]`; size `[0.3, 0.5]` (short first); height 0. Rule 10 — not_word ほげ (contains no word). Rule 11 — 201 kinds (the 151 extra kinds are deep copies of `box` relabelled `pad1` … `pad151`, nothing else changed; their copied `ja` and words would fail rules 3 and 4, so this row raises rule 11 only because the count is checked right after rule 1, Step 3).
  One owner per condition: spec rule 4 (a word is not a `not_word`) and rule 10 (a not_word is not itself a word) state the same condition, and `parse` raises on the first failure, so the condition belongs to **rule 4 only** (its row "a `not_word` used as a word"); rule 10 checks only that every not_word contains some word. The rule numbers stay as the spec's, because `VocabularyError` messages cite them.

```python
def test_reserved_segments_come_from_world():                         # C5: a set built from the code, no count
    want = {s for n in world.ACTION_NAMES + world.PROPOSITIONS + world.UNCONTROLLABLE for s in n.split("_")}
    assert world.RESERVED_LABEL_SEGMENTS == frozenset(want) and {"turned", "obstacle", "blocked", "stand"} <= want
    bad = copy.deepcopy(DATA); entry(bad, "stool")["label"] = "turned_stool"
    with pytest.raises(V.VocabularyError, match="rule 2:"):
        V.check_reserved_segments(V.parse(bad), world.RESERVED_LABEL_SEGMENTS, "world.py")

def test_no_word_reads_as_a_handling_verb():
    assert not [w for e in V.VOCABULARY.entries for w in e.words if translator._handles_object(w)]
    bad = copy.deepcopy(DATA); entry(bad, "cabinet")["words"].append("押し入れ")
    with pytest.raises(V.VocabularyError, match="rule 6:"):
        V.check_words(V.parse(bad), lambda w: not translator._handles_object(w), 6, "reads as a handling verb")

@pytest.mark.parametrize("phrase", RESERVED_PHRASES)    # the 154 below
def test_reserved_phrases_are_no_kind(phrase):
    assert V.kinds_in(phrase) == ()

@pytest.mark.parametrize("phrase,label", EXPECTED)       # the 26 below
def test_expected_phrases_resolve_to_their_kind(phrase, label):
    assert V.kinds_in(phrase)[-1].label == label

def test_every_word_resolves_to_its_own_kind():          # rule 5, also checked at load
    for e in V.VOCABULARY.entries:
        for w in e.words:
            for s in V.SUFFIXES:
                assert V.kinds_in(w + s)[-1].label == e.label, (w, s)

def test_kinds_in_is_longest_and_last():
    assert [(k.label, k.word, k.start, k.end) for k in V.kinds_in("ゴミ箱と椅子")] == \
        [("trash_can", "ゴミ箱", 0, 3), ("chair", "椅子", 4, 6)]
    assert V.kinds_in("チェアーまで")[0].word == "チェア" and V.kinds_in("ｲｽ")[0].label == "chair"
    assert V.kinds_in("車椅子まで") == () and [k.label for k in V.kinds_in("観葉植物")] == ["potted_plant"]

def test_hypernyms_and_living_beings_are_not_kinds():
    for w in ("家具", "家電", "物", "もの", "障害物", "何か", "人", "グリーン", "キッチン", "台所", "2階", "1本", "2台"):
        assert V.kinds_in(w) == (), w
    assert not {"person", "dog", "cat"} & V.LABELS and all(V.lookup(x) is None for x in ("person", "dog", "cat"))

@pytest.mark.parametrize("name,label", [
    ("desk", "desk"), ("Desk", "desk"), ("dining_table", "table"), ("coffee table", "table"),
    ("potted plant", "potted_plant"), ("Potted-Plants", "potted_plant"), ("houseplant", "potted_plant"),
    ("fridge", "refrigerator"), ("couch", "sofa"), ("tv", "television"), ("bookcase", "bookshelf"),
    ("cupboard", "cabinet"), ("trash can", "trash_can"), ("trash bin", "trash_can"), ("boxes", "box"),
    ("chairs", "chair"), ("benches", "bench"), ("tables", "table"),
    ("person", None), ("dog", None), ("kitchen", None), ("sim_box", None), ("", None), ("furniture", None),
    ("cup", None), ("tv_stand", None), (42, None)])
def test_lookup(name, label):
    assert V.lookup(name) == label

def test_event_names_never_collide_for_label_pairs():   # spec §7.3 / §8, 1225 pairs
    # for every pair (A, B): names of event_name(action, s) for action in ("forward", "turn_left") and every
    # subset s (size <= 3) of {near_A_1, around_A_1, around_cw_A_1, near_B_1, around_B_1, "standing"} are distinct

def test_schema_matches_the_loader():                   # the schema is for docs and tests only
    schema = json.loads((V.VOCABULARY_PATH.parent / "vocabulary.schema.json").read_text(encoding="utf-8"))
    kind = schema["$defs"]["kind"]
    assert set(kind["required"]) == {"label", "ja", "words", "detect", "size", "height"}
    assert set(kind["properties"]) == {"label", "ja", "words", "detect", "en", "size", "height"}
    assert set(schema["properties"]) == {"version", "labels", "not_words", "detect_ignore"}

def test_errors_name_the_file_rule_label_and_value(tmp_path):        # Review Focus 1
    bad = copy.deepcopy(DATA); entry(bad, "sofa")["words"].append("椅子")
    p = tmp_path / "vocabulary.json"; p.write_text(json.dumps(bad, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(V.VocabularyError) as e:
        V.load(p)
    assert all(s in str(e.value) for s in ("vocabulary.json", "rule 4", "sofa", "椅子"))
    p.write_text('{"version": 1, "labels": [],}', encoding="utf-8")
    with pytest.raises(V.VocabularyError, match=r"vocabulary\.json: rule 1: -: .*line 1 column"):
        V.load(p)

def test_many_detect_prompts_warn(monkeypatch, caplog):
    monkeypatch.setattr(V, "DETECT_WARN", 86)
    V.parse(copy.deepcopy(DATA))
    assert "87 detect prompts" in caplog.text
```
  `RESERVED_PHRASES` (154, from the rev-2 check of `design-vocabulary.md` §2.4; space-separated here, write them as a tuple):
  立って 挨拶して ダンスして ダンス 座って おすわり 伏せて 寝そべって しゃがんで 伸びをして 喜んで ハートを描いて お辞儀して おじぎ 前 後ろ 右 左 左前 右前 一周して 周りを回って 回れ右 進んで 下がって バックして バック 歩いて 走って 障害物 物 もの 何か キッチン 台所 階段を上って ジャンプ 握手 お手 おかわり こっちに来て 部屋 壁 床 家具 家電 あれ それ 1本 2台 3個 1枚 2つ 一人 二人で 1匹 1袋 1冊 2脚 本 台 人 花 木 草 球 鉢 袋 電球 地球 花火 花瓶 箱根 植物 植物園 トランプ スカート ベンチャー ボールペン かわいすぎる アイス たなばた したな はこぶ はこんで ボックスステップ テーブルクロス デスクトップ 自転車置き場 棚卸し 机上 バケツリレー コーンスープ ランプシェード テレビ番組 長椅子 車椅子 荷物 にもつ トイレ 便器 窓 ドア カーテン 絨毯 ラグ マット クッション 布団 押し入れ 運動器具 ライト 電気 照明 スタンド チャリティー 洗濯機置き場 冷凍庫 紙袋 ランドセル サーキュレーター オットマン 台車 はしご 梯子 踏み台 こたつ コタツ 炬燵 ちゃぶ台 座卓 衣装ケース コートハンガー ポールハンガー コート掛け 洋服掛け まり くま グリーン クーラー モニター キーボード トランク バギー ハンガー アイロン ボード バイク タンク ケース ルンバ 腰掛け すいそう しょうかき

  `EXPECTED` (26): 本棚の本→bookshelf 椅子の上の箱→box 箱の横の椅子→chair 机といす→chair ゴミ箱と椅子→chair ゴミ箱まで→trash_can 段ボール箱→box 木箱→box テレビ台→television ドラム式洗濯機→washing_machine 液晶テレビ→television 石油ストーブ→heater 食器棚→cabinet 戸棚→cabinet シューズボックス→shoe_rack ダストボックス→trash_can くずかご→trash_can ソファベッド→sofa ベビーベッド→crib バスケットボール→ball 段ボール→box 丸椅子→stool 座椅子→chair チェアー→chair 学習机→desk ｲｽ→chair

- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_vocabulary.py`. Expected: FAIL, `ModuleNotFoundError: No module named 'voice_ltl.vocabulary'`.
- [ ] **Step 3: Implement `voice_ltl/vocabulary.py` and the three hooks.**
  - `parse` checks spec §7.3 in this order and raises on the first failure with the message form above: rule 1; rule 11's count (≤ `MAX_KINDS` kinds, so a huge file fails before the per-kind work); rules 2–5; rules 7–10; then rule 11's warning. Rule 4 owns "a word is not a not_word"; rule 10 checks only that each not_word contains a word. JSON errors from `load` are rule 1 with "line L column C".
  - `kinds_in`: NFKC the text; replace every occurrence of each `not_word` by the same number of 「・」; one compiled alternation of `_word_pattern(w)` for all words, longest first; `finditer` left to right (so at one position the longest word wins); map the matched text to the vocabulary spelling (strip a trailing ー when the bare form is the word).
  - `lookup`: `normalise_alias`; a label → itself; else the label whose normalised `detect` or `en` alias it is; else retry once without a trailing `es`, then without a trailing `s`; else None. `detect_ignore` words are never aliases.
  - Rule 5 runs inside `parse` with the new matcher over `SUFFIXES`.
  - Rule 11's warning goes to `logging.getLogger("voice_ltl.vocabulary")` as "<n> detect prompts (more than <DETECT_WARN>)".
- [ ] **Step 4: Run the tests.** Run: `T tests/test_vocabulary.py tests/test_objects.py tests/test_object_language.py`. Expected: PASS (`objects.label_in` is unchanged in this task; `translator.word_start is objects.word_start` still holds).
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.**
  ```bash
  git add voice_ltl/vocabulary.json voice_ltl/vocabulary.schema.json voice_ltl/vocabulary.py tests/test_vocabulary.py voice_ltl/objects.py voice_ltl/world.py voice_ltl/translator.py
  git commit -m "feat: object kinds come from voice_ltl/vocabulary.json, checked at import"
  ```

### Task 1.2: Object core (stage 1b, branch `objects2/object-core`)

Ids, propositions, rows and wire fields, names and sizes, word matching, directions, focus, messages, and the footprint geometry the grid and `objects_check` share.

**Files:**
- Modify: `voice_ltl/objects.py`, `voice_ltl/ros_link.py`, `tests/test_objects.py`, `tests/test_ros_link.py`; pins its change moves in other test files (listed in Step 4)

**Interfaces:**
- Consumes (Task 1.1): `vocabulary.LABELS`, `name_of`, `size_of`, `height_of`, `kinds_in`, `Kind`.
- Produces (`voice_ltl/objects.py`; `LABEL_JA`, `LABEL_WORDS` and `_LABEL_RES` are removed):

```python
LABELS: frozenset[str]                                  # = vocabulary.LABELS
ID_RE = re.compile(rf"({_ALT})_([1-9][0-9]?)")           # _ALT: re.escape'd labels, longest first; fullmatch only
PROP_RE = re.compile(rf"(near|around_cw|around)_((?:{_ALT})_[1-9][0-9]?)")
SOURCES = ("logical", "detector", "fake")
DEFAULT_SIZE = (0.5, 0.5)                               # only Obj's field default now
@dataclass(frozen=True)
class Obj:                                              # + two optional fields, both default None
    id: str; label: str; x: float; y: float; size: tuple[float, float] = DEFAULT_SIZE
    visible: bool = True; seen: float | None = None
    confidence: float | None = None; measured: bool | None = None
    name -> name_of(self.label); radius (unchanged); to_row() adds "confidence" / "measured" only when not None
@dataclass(frozen=True)
class Scene: objects: tuple[Obj, ...] = (); pose: Pose | None = None; focus: frozenset[str] = frozenset()
def name_of(label: str) -> str
def default_size(label: str) -> tuple[float, float]     # (L, L) from the vocabulary; DEFAULT_SIZE if unknown
def measure_radius(label: str) -> float                  # max(0.6, hypot(L, S)/2 + 0.3); 0.6 if unknown
Kind = vocabulary.Kind
def kinds_in(text: str) -> tuple[Kind, ...]
def label_in(text: str) -> str | None                    # label of the LAST kind (O1)
def labels_in(text: str) -> frozenset[str]
def direction(obj: Obj, pose: Pose | None) -> str | None # the 8-sector word; where() uses it
def parse_source(payload: object) -> str | None          # payload["source"] when it is in SOURCES
def parse(payload: object) -> tuple[Obj, ...]            # also () when "source" is present and not in SOURCES
def format_message(objs: Iterable[Obj], stamp: float, source: str | None = None) -> dict
def msg_not_found(label: str, word: str | None = None) -> str
def msg_too_far(label: str, word: str | None = None) -> str
def msg_ambiguous(label: str, n: int, directions: Sequence[str | None] = (), word: str | None = None) -> str
def msg_not_listed(label: str, word: str | None = None) -> str
def msg_lap_too_big(label: str) -> str
def msg_too_big_here(label: str) -> str
def spine_point(x: float, y: float, obj: Obj) -> tuple[float, float]
def core_distance(x: float, y: float, obj: Obj) -> float
def spine_angle(x: float, y: float, yaw: float, obj: Obj) -> float       # rad: |wrap(bearing(p -> spine_point) - yaw)|
def footprint_angle(x: float, y: float, yaw: float, obj: Obj) -> float   # rad, 0 when the yaw points into the footprint
```
- `ros_link.RosLink` keeps `(time.time(), rows, parse_source(payload))` and gains `objects_source() -> str | None` (None when stale, like `objects()`).

Exact copy (`{name}` = `word` if given, else `name_of(label)`; `{ja}` = `name_of(label)`; count = `"{n} つ"` for n ≤ 9, `"{n} 個"` from 10):
- `msg_not_found`: 「「{name}」が見つかりません。Go2 の前に置くか、Go2 をそちらに向けてから、もう一度言ってください。」
- `msg_too_far`: 「「{name}」が遠すぎます（6 m まで）。」 (6 from `MAX_OBJECT_DISTANCE:g`)
- `msg_ambiguous`: 「「{name}」が {count}あるので、どれか決められません。{hint}のように言ってください。」 where `hint` is 「「{d}の{name}」」 for each of 2–3 directions that are all given and all different; 「「近いほうの{name}」「遠いほうの{name}」」 for exactly 2 equal directions; otherwise 「「左の{name}」」 (today's hint). Example: 「「箱」が 2 つあるので、どれか決められません。「左前の箱」「右前の箱」のように言ってください。」
- `msg_not_listed`: 「「{name}」はほかの物より遠いので、選べません（近い順に 8 個まで）。」 (8 from `MAX_PROMPT_OBJECTS`)
- `msg_lap_too_big`: 「「{ja}」は大きすぎて、周りを回る計画が作れません。」
- `msg_too_big_here`: 「「{ja}」は大きいので、ここからは計画できません。近づいてから、もう一度言ってください。」

`spine_point` and `core_distance` follow `design-near.md` §3.1 (a = sx/2, b = sy/2, m = min(a, b), e = |a − b|; the spine runs along the longer side; d_k = |p − s(p)| − m). `footprint_angle` (spec §10.8, O29):

```python
if edge_distance(x, y, obj) == 0.0: return 0.0
c = wrap_rad(atan2(obj.y - y, obj.x - x) - yaw)                     # the centre is inside the angular hull
rel = [wrap_rad(wrap_rad(atan2(cy - y, cx - x) - yaw) - c) for cx, cy in corners(obj)]
lo, hi = c + min(rel), c + max(rel)                                 # hull width < pi outside the rectangle
return 0.0 if lo <= 0.0 <= hi else min(abs(wrap_rad(lo)), abs(wrap_rad(hi)))
```

`Scene.listed()` (spec §9.1, O6): order every object as today (distance from the pose, then `_id_key`; `_id_key` order without a pose); take up to 8 objects whose label is in `focus` in that order, fill up to 8 with the rest in that order, and return them in that order. An empty focus gives exactly today's list.

- [ ] **Step 1: Write the failing tests** in `tests/test_objects.py` (keep every other test).

```python
def test_ids_and_props_follow_the_vocabulary():
    assert split_prop("near_potted_plant_1") == ("near", "potted_plant_1")
    assert split_prop("around_cw_potted_plant_12") == ("around_cw", "potted_plant_12")
    assert split_prop("around_cw_box_1") == ("around_cw", "box_1") and split_prop("near_table_1") == ("near", "table_1")
    for bad in ("near_box_1_2", "near_box_01", "near_box_100", "near_box_0", "near_sim_box_1", "near_Box_1",
                "near_person_1", "moved_forward"):
        assert split_prop(bad) is None, bad
    assert ID_RE.fullmatch("sim_box_1") is None and ID_RE.fullmatch("cw_box_1") is None
    assert split_prop("around_cw_fire_extinguisher_99") == ("around_cw", "fire_extinguisher_99")

def test_rows_accept_only_vocabulary_labels_and_sort_by_number():
    rows = [row("potted_plant_3", "potted_plant"), row("sim_box_1", "sim_box"), row("person_1", "person"),
            row("box_10", "box"), row("box_2", "box")]
    assert [o.id for o in parse_rows(rows)] == ["potted_plant_3", "box_10", "box_2"]
    assert sorted(["box_10", "box_2", "bean_bag_1"], key=objects._id_key) == ["bean_bag_1", "box_2", "box_10"]

def test_names_sizes_and_radius_come_from_the_vocabulary():
    assert Obj("potted_plant_1", "potted_plant", 0, 0).name == "観葉植物" and name_of("no_such") == "no such"
    assert parse_rows([{"id": "chair_1", "label": "chair", "x": 1, "y": 2}])[0].size == (0.45, 0.45)
    assert default_size("sofa") == (1.8, 1.8) and default_size("no_such") == DEFAULT_SIZE
    assert measure_radius("box") == pytest.approx(0.6536, abs=1e-4)
    assert measure_radius("sofa") == pytest.approx(1.3062, abs=1e-4) and measure_radius("ball") == 0.6

def test_confidence_and_measured_are_optional_and_checked():
    good = row("sofa_1", "sofa", confidence=0.71, measured=True)
    (o,) = parse_rows([good]); assert (o.confidence, o.measured) == (0.71, True)
    for bad in ({"confidence": True}, {"confidence": 1.2}, {"confidence": -0.1}, {"confidence": float("nan")},
                {"confidence": "0.5"}, {"measured": "yes"}, {"measured": 1}):
        assert parse_rows([{**good, **bad}]) == (), bad
    assert DEFAULT_LAYOUT[0].to_row() == {"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.5, 0.5],
                                          "seen": None, "visible": True}          # byte-identical without them
    assert {"confidence", "measured"} <= set(o.to_row())

def test_message_source():
    msg = format_message(DEFAULT_LAYOUT, 1.0, source="detector")
    assert msg["source"] == "detector" and parse_source(msg) == "detector" and parse(msg) == DEFAULT_LAYOUT
    assert "source" not in format_message(DEFAULT_LAYOUT, 1.0) and parse_source({"frame": "odom"}) is None
    assert parse({**msg, "source": "yoloe"}) == () and parse_source({**msg, "source": "yoloe"}) is None

def test_label_in_is_longest_and_last():                 # O1, spec §9.5
    for text, label in (("ゴミ箱", "trash_can"), ("ごみ箱", "trash_can"), ("くず箱", "trash_can"), ("本棚", "bookshelf"),
                        ("本箱", "bookshelf"), ("食器棚", "cabinet"), ("戸棚", "cabinet"), ("下駄箱", "shoe_rack"),
                        ("丸椅子", "stool"), ("椅子の上の箱", "box"), ("段ボール箱", "box"), ("木箱", "box"),
                        ("机といす", "chair"), ("観葉植物", "potted_plant"), ("オフィスチェア", "chair")):
        assert label_in(text) == label, text
    for text in ("車椅子", "家具", "家電", "人", "グリーン", "キッチン", "台所", "2階", "1本", "2台"):
        assert label_in(text) is None, text
    assert labels_in("ゴミ箱と椅子") == {"trash_can", "chair"}

def test_scene_focus_lists_the_named_kind_first():
    boxes = tuple(Obj(f"box_{k}", "box", -1.5, -1.75 + 0.5 * (k - 1), (0.3, 0.3)) for k in range(1, 9))
    plant = Obj("potted_plant_1", "potted_plant", 3.0, 0.0, (0.4, 0.4))
    crowded = Scene(boxes + (plant,), (0.0, 0.0, 0.0))
    assert plant not in crowded.listed() and len(crowded.listed()) == 8
    focused = dataclasses.replace(crowded, focus=frozenset({"potted_plant"}))
    assert plant in focused.listed() and "box_8" not in [o.id for o in focused.listed()]
    assert [o.id for o in focused.listed()] == sorted((o.id for o in focused.listed()),
                                                      key=lambda i: distance_of(focused, i))   # still nearest first
    assert "near_potted_plant_1" in focused.propositions() and "near_potted_plant_1" in focused.descriptions()
    # no pose: id order with the focus kinds first; more than 8 of the focus kind: the nearest 8
    # an empty focus equals today's listing (the existing listing tests stay as they are)

def test_direction_is_the_first_word_of_where():
    for o in DEFAULT_LAYOUT:
        assert direction(o, (0, 0, 0)) == where(o, (0, 0, 0)).split()[0]
    assert direction(DEFAULT_LAYOUT[0], None) is None

def test_messages():                                     # exact copy, spec §9.9 and §10.6
    assert msg_not_found("box") == "「箱」が見つかりません。Go2 の前に置くか、Go2 をそちらに向けてから、もう一度言ってください。"
    assert msg_not_found("desk", "机").startswith("「机」が見つかりません。")
    assert msg_too_far("chair") == "「椅子」が遠すぎます（6 m まで）。"
    assert msg_ambiguous("box", 2, ("左前", "右前")) == \
        "「箱」が 2 つあるので、どれか決められません。「左前の箱」「右前の箱」のように言ってください。"
    assert msg_ambiguous("box", 2, ("前", "前")) == \
        "「箱」が 2 つあるので、どれか決められません。「近いほうの箱」「遠いほうの箱」のように言ってください。"
    assert msg_ambiguous("box", 4) == "「箱」が 4 つあるので、どれか決められません。「左の箱」のように言ってください。"
    assert msg_ambiguous("box", 12, ("左前", "右前", "前", "後ろ")).startswith("「箱」が 12 個あるので、")
    assert msg_not_listed("potted_plant") == "「観葉植物」はほかの物より遠いので、選べません（近い順に 8 個まで）。"
    assert msg_lap_too_big("sofa") == "「ソファ」は大きすぎて、周りを回る計画が作れません。"
    assert msg_too_big_here("bed") == "「ベッド」は大きいので、ここからは計画できません。近づいてから、もう一度言ってください。"

def test_footprint_geometry():                           # spec §10.10 item 13
    sofa = Obj("sofa_1", "sofa", 0.0, 0.0, (2.0, 0.9))   # spine along x, e = 0.55, m = 0.45
    assert spine_point(0.3, 2.0, sofa) == (0.3, 0.0) and spine_point(3.0, 1.0, sofa) == (0.55, 0.0)
    assert core_distance(0.3, 1.5, sofa) == pytest.approx(1.05)            # beside the long side: d_k == d_e
    box = Obj("box_1", "box", 0.0, 0.0, (0.5, 0.5))
    assert core_distance(2.0, 0.0, box) == pytest.approx(2.0 - 0.25)       # square: d_k == d_c - a
    assert spine_angle(0.3, 1.5, -math.pi / 2, sofa) == pytest.approx(0.0)
    assert footprint_angle(0.0, 2.0, -math.pi / 2 + 0.3, sofa) == 0.0      # still pointing into the sofa
    assert footprint_angle(3.0, 0.0, math.pi / 2, box) == pytest.approx(math.pi / 2 - math.atan2(0.25, 2.75), abs=1e-9)
    assert footprint_angle(0.1, 0.1, 1.0, box) == 0.0                      # inside
```
  In `tests/test_ros_link.py`: `test_detector_rows_keep_confidence_measured_and_source` — a published detector message reaches `objects()` with `confidence` and `measured`; `objects_source() == "detector"`; None once stale.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_objects.py tests/test_ros_link.py`. Expected: FAIL (`ImportError: cannot import name 'kinds_in'` and others).
- [ ] **Step 3: Implement** the Interfaces above in `voice_ltl/objects.py` and `voice_ltl/ros_link.py`. `_parse_row` uses `ID_RE` (the allowlist), the label's `default_size` for a missing size, and the two optional fields. `label_in`, `labels_in` and `kinds_in` delegate to the vocabulary. `where()` calls `direction()`.
- [ ] **Step 4: Update the pins this change moves, then run.** `test_prop_names_and_split` (`near_table_1` is a valid prop now); `test_label_words_match_only_whole_words` (「ボックス・椅子」 → chair, 「車椅子まで」 → None, 「オフィスチェア」 → chair; every other pin keeps today's answer); `test_parse_keeps_only_valid_rows` (the chair without a size gets (0.45, 0.45)); `test_messages` (the new not-found copy). If the full suite shows other moved pins in files no stage-1b task owns, update them the same way and list them in the commit message. Run: `T tests/test_objects.py tests/test_ros_link.py tests/test_object_language.py`. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add voice_ltl/objects.py voice_ltl/ros_link.py tests/test_objects.py tests/test_ros_link.py <moved pins>` and `git commit -m "feat: ids, rows, names, words and messages for every vocabulary kind"`.

### Task 1.3: Grid near band (stage 1c, branch `objects2/grid-band`)

**Files:**
- Modify: `voice_ltl/grid.py`, `tests/test_grid.py`, `tests/test_session_objects.py` (one test rewritten)

**Interfaces:**
- Consumes (Task 1.2): `Obj`, `edge_distance`, `spine_point`, `core_distance`, `split_prop`, `msg_not_found`, `msg_too_far`, `msg_lap_too_big`, `msg_too_big_here`, `MSG_TOO_SPREAD`, `MAX_OBJECT_DISTANCE`; `vocabulary.VOCABULARY.entries` for the sweeps.
- Produces (`voice_ltl/grid.py`):

```python
BLOCK_MARGIN = 0.45                     # now: blocked iff edge_distance(cell centre in odom, o) <= 0.45 + EPS
NEAR_MIN_MARGIN = 0.75; NEAR_MAX_MARGIN = 1.5; NEAR_MAX_DISTANCE = 1.3; NEAR_MAX_ANGLE_DEG = 40.0   # band A, unchanged
NEAR_MIN_EDGE = 0.65; NEAR_SMALL_SIDE = 0.6; NEAR_SMALL_MIN_CENTRE = 1.05
NEAR_SMALL_MAX_CORE = 1.075; NEAR_MAX_CORE = 1.25
LAP_TOO_BIG_RADIUS = 1.75
def is_small(o: Obj) -> bool             # max(o.size) <= NEAR_SMALL_SIDE + EPS
def core_limit(o: Obj) -> float          # NEAR_SMALL_MAX_CORE if small else NEAR_MAX_CORE
@dataclass(frozen=True)
class GridShape:
    grid: Grid; near: dict[str, frozenset[tuple[tuple[int, int], int]]]
    ring: dict[str, dict[tuple[int, int], int]]; lap_props: tuple[str, ...]; start_covered: bool
def grid_shape(anchor: Pose, objects: Sequence[Obj], atoms: frozenset[str], margin: int = 1) -> GridShape
def grid_world(posture: str, front: bool, back: bool, anchor: Pose, objects: Sequence[Obj],
               atoms: frozenset[str], margin: int = 1) -> GridWorld      # grid_shape + today's BFS
```
- `grid_shape` raises `GridError` in this order: an id not in the Scene (`msg_not_found(label)`); farther than 6 m (`msg_too_far(label)`); a lap target with r ≥ 1.75 (`msg_lap_too_big(label)`, decided before the rectangle); a rectangle over 256 cells with exactly one referenced object (`msg_too_big_here(label)`); otherwise over 256 (`MSG_TOO_SPREAD`).
- Near key (spec §10.4): a free cell c and heading h with band A (today) or band B: d_e ≥ 0.65, d_k ≤ `core_limit(o)`, `is_small(o)` ⇒ d_c ≥ 1.05, and |wrap(bearing(centre(c) → spine_point(centre(c))) − heading_yaw(h))| ≤ 40°, all in odom (`Grid.center`, `Grid.heading_yaw`). Ring unchanged.
- Rectangle (spec §10.5): per near target, half extents max(min(r + 1.5, 1.3), core box) with core box = e·|cos ψ| + m + core along grid x and e·|sin ψ| + m + core along grid y (ψ = spine direction − anchor yaw); lap r + 1.75; plus the robot's cell and `margin` cells.

- [ ] **Step 1: Write a characterization test, then the failing tests** in `tests/test_grid.py`. First write `test_default_layout_world_is_unchanged` with the values the unchanged code gives (the rectangles, the blocked sets, the ring sets, the state and transition counts of the 5 default formulas), and run it once against the unchanged `grid.py` (it must pass now and after). Then add:

```python
BOX_KEYS = {((3, 2), 0), ((3, 4), 0), ((4, 1), 1), ((4, 5), 3), ((6, 1), 1), ((6, 5), 3), ((7, 2), 2), ((7, 4), 2)}
CHAIR_KEYS = {((3, -4), 0), ((3, -2), 0), ((4, -5), 1), ((4, -1), 3), ((6, -5), 1), ((6, -1), 3), ((7, -4), 2), ((7, -2), 2)}

def test_numbers():                                       # item 1
    assert (NEAR_MIN_EDGE, NEAR_SMALL_SIDE, NEAR_SMALL_MIN_CENTRE, NEAR_SMALL_MAX_CORE, NEAR_MAX_CORE) == \
        (0.65, 0.6, 1.05, 1.075, 1.25)
    assert (BLOCK_MARGIN, NEAR_MIN_MARGIN, NEAR_MAX_MARGIN, NEAR_MAX_DISTANCE, NEAR_MAX_ANGLE_DEG) == (0.45, 0.75, 1.5, 1.3, 40.0)

def test_default_layout_world_is_unchanged():             # item 2, spec §10.7 (a); + the captured literals
    gw = gw_of(frozenset({"near_box_1", "near_chair_1"}))
    assert gw.near["near_box_1"] == BOX_KEYS and gw.near["near_chair_1"] == CHAIR_KEYS
    for formula, states in (("F near_chair_1", 1704), ("F near_box_1", 1704), ("F around_box_1", 9984),
                            ("F(around_box_1 & X F near_chair_1)", 12000), ("F(around_box_1 & X F around_cw_box_1)", 28416)):
        assert len(gw_of(formula_atoms(formula)).world.states) == states

def test_measured_gazebo_box_keeps_near_cells():          # item 3
    box = Obj("box_1", "box", 2.5, 1.5, (0.523, 0.529))
    assert len(grid_shape((0, 0, 0), (box,), frozenset({"near_box_1"})).near["near_box_1"]) == 8

def test_big_objects_get_near_cells_at_every_offset_and_rotation():   # item 4, the fast subset
    # object about 3 m ahead; offsets 0..0.45 step 0.05 on both axes; anchor yaw 0..45 step 3 plus 41 and 44.75 deg
    # >= 8 keys: table 1.6x0.8, sofa 2.0x0.9, sofa 1.8x0.9, shelf 0.9x0.4, shelf 1.8x0.4, TV 1.0x0.3, fridge 0.65x0.65, 3.0x3.0
    # >= 4 keys: chair 0.45x0.45, box 0.5x0.5;  >= 5 keys: measured box 0.523x0.529
    # explicit regressions: shelf 1.8x0.4 at offset (0.25, 0.25), 44.75 deg (rule Q: 0); TV at (0.025, 0.025), 40.25 deg (rule Q: 2)

def test_every_near_key_meets_the_edge_acceptance():      # item 5, same sweep, every vocabulary size
    # every key: free; d_e >= 0.65 - 1e-9 or a band-A key; d_e <= 1.25 + 1e-9; heading within 40 deg of the
    # nearest spine point or the centre; for 16 directions of a 0.15 m stop error: d_e <= 1.40, and for objects
    # whose shorter side >= 0.22 m footprint_angle <= 45 deg with the yaw off by +-3 deg

def test_chair_plan_stays_within_n1_at_every_placement(): # item 6
    # 0.45 m square over the sweep: every key d_c <= 1.3 + 1e-9 and centre angle <= 40 deg; 0.5 m box: d_c <= 1.325 + 1e-9

def test_small_profile_is_rule_q():                       # item 7
    # box, chair, measured box, trash can 0.27x0.38 over the sweep: keys == rule Q's (band B edge 0.575 instead of 0.65)

def test_blocked_is_footprint_plus_margin():              # item 8
    # rotated anchor (0.65 rad) and a 2.0x0.9 sofa: blocked iff edge_distance(centre(c)) <= 0.45; a cell 0.5 m from the
    # sofa's long side is free

def test_moves_keep_the_body_clear_of_every_footprint():  # item 9, replaces the r + 0.35 invariant
    # worlds with off-lattice default objects, a sofa and a table: every move segment >= 0.37 m from every footprint;
    # lap progress 1-4 only on ring cells

def test_spine_facing_is_the_centre_rule_for_squares():   # item 10
def test_large_profile_has_no_centre_floor():              # item 10: a lattice-aligned TV 1.0x0.3 has keys 0.65-1.0 m
                                                           # from the middle of its long face
def test_near_rectangle_contains_every_near_key():        # item 11: a rotated sofa and a 3.0x3.0 row

def test_lap_messages():                                   # item 12
    # 2.5x2.5 lap -> msg_lap_too_big at every placement; 3.0x1.8 lap at (3.0, 0) fits (225 cells);
    # bed 2.0x1.0 lap at (5.5, 0.25), anchor (0, 0, 0) -> msg_too_big_here("bed") (own 182 cells, 266 with the robot);
    # two spread objects -> MSG_TOO_SPREAD

def test_margin_widens_the_rectangle():                    # for Task 1.7
    s1, s3 = (grid_shape((0, 0, 0), DEFAULT_LAYOUT, frozenset({"near_chair_1"}), m) for m in (1, 3))
    assert s3.grid.i_range == (s1.grid.i_range[0] - 2, s1.grid.i_range[1] + 2) and s3.near == s1.near

@pytest.mark.slow
def test_near_keys_full_sweep():                          # O32: 0.025 m / 0.25 deg plus 30 000 random, 26 sizes
```
  Rewrite: `test_rotated_anchor_gives_the_same_cells` (near and ring sets equal for any anchor angle; blocked cells and the World equal only at t = π/2 with sx, sy swapped); `test_moves_keep_the_body_clear_of_off_lattice_objects` → removed in favour of item 9; `test_near_cells_face_the_object_within_the_band` asserts A or B; `test_the_default_layout_keeps_near_cells_for_both_objects` asserts the key sets and gets a new docstring; `test_too_far_too_spread_and_not_found` adds the two size messages. In `tests/test_session_objects.py`, `test_object_with_no_near_cell_gives_the_no_path_reason` gets a scene where every near key of the target is blocked by other objects (a 1.0 × 1.0 box now has keys) and still asserts `MSG_NO_PATH` and `link.sim.accepted == []`.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_grid.py`. Expected: FAIL (`ImportError: cannot import name 'NEAR_MIN_EDGE'`); the characterization test passes.
- [ ] **Step 3: Implement** `grid_shape` and band B as in the Interfaces; `grid_world` calls `grid_shape` then today's `_explore`.
- [ ] **Step 4: Run.** Run: `T tests/test_grid.py tests/test_session_objects.py tests/test_gazebo_objects.py tests/test_phrasing_trials.py tests/test_sim_objects.py`. Expected: PASS, and `test_grid.py` without `slow` runs in under 60 s.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add voice_ltl/grid.py tests/test_grid.py tests/test_session_objects.py` and `git commit -m "feat: near band B measured from the footprint, so big furniture has near cells"`.

### Task 1.4: Language — prompt, reply field and translator (stage 1c, branch `objects2/language`)

**Files:**
- Modify: `voice_ltl/translator.py`, `voice_ltl/prompt.py`, `voice_ltl/session.py` (only `_redacted`), `tests/test_object_language.py`, `tests/test_grammar_translator.py`, `tests/test_llm.py`
- Create: `tests/test_session_language.py`

**Interfaces:**
- Consumes (Task 1.2): `Scene.focus`, `kinds_in`, `labels_in`, `Kind`, `name_of`, `direction`, `msg_not_found / msg_too_far / msg_ambiguous / msg_not_listed` with `word`, `MAX_PROMPT_OBJECTS`; (Task 1.1) `vocabulary.lookup`.
- Produces:

```python
# voice_ltl/translator.py
MAX_UNRESOLVED = 10; MAX_UNRESOLVED_LABEL = 64
_GOING_TAIL = re.compile(                 # fullmatch on the stripped rest of an item; .match on an instruction clause
                                          # every part is optional, so it also matches "" (see "Object items" below)
    r"(?:の?(?:ところ|所|前|そば|横|近く|方|周り|まわり|周囲))?"
    r"(?:まで|へ|に|を)?"
    r"(?:(?:ぐるっと|ぐるりと)?(?:歩く|歩いて|あるく|あるいて|行く|行って|いく|いって|進む|進んで|近づく|近づいて"
    r"|ちかづく|ちかづいて|向かう|向かって|回る|回って|まわる|まわって"
    r"|(?:[0-9]+|[一二三四五六七八九十]+)周(?:する|して)?)){0,2}")
@dataclass(frozen=True)
class Entry: said: str; label: str | None             # label: the accepted vocabulary label or None
@dataclass
class Attempt: formula: str | None; unsupported: list[str]; errors: list[str]
               unresolved: list[dict] = field(default_factory=list)        # [{"said": ..., "label": ... or None}]
def _unresolved(raw: object, instruction: str) -> list[Entry]             # spec §9.3
def going_targets(instruction: str) -> dict[str, str]                    # label -> the user's word, §9.6 step 3
def _object_messages(items: list[str], view: Scene | None, entries: list[Entry],
                     instruction: str) -> tuple[list[str], list[str]]    # (messages, rest), each kind told once
def _unsupported_message(items: list[str], view: Scene | None = None, entries: Sequence[Entry] = (),
                         instruction: str = "") -> str
def _kind_guard(formula: str, view: Scene | None, instruction: str) -> list[str]   # O35 messages, [] if fine
def translate(instruction: str, llm: LLM, max_tries: int = 3, scene: Scene | None = None) -> Translation
# voice_ltl/prompt.py
SCHEMA                                     # "unresolved_objects": array of {said: string, label: string}, both required,
                                           # additionalProperties False; required = all four keys (Codex strict)
EXAMPLE_SCENE                              # + Obj("potted_plant_9", "potted_plant", 1.5, 1.5, (0.4, 0.4))
_OBJECT_EXAMPLES                           # 9 five-tuples (instruction, formula, unsupported, substitutions, unresolved)
def _example_line(instruction, formula, unsupported, substitutions, unresolved=None) -> str   # 4th key only for a list
def word_note(instruction: str, scene: Scene | None) -> str              # "" or the note line
# voice_ltl/session.py
def _redacted(value)                       # also every string inside a dict, recursively
```

Rules the signatures do not settle:
- **Translation order (spec §9.8).** `view = scene` with `focus = labels_in(instruction)` when there is a kind; `propositions = PROPOSITIONS + view.propositions()`; each reply: `entries = _unresolved(reply.get("unresolved_objects"), instruction)` (a missing key reads as `[]`); a refusal returns `items = unsupported + [e.said for e in entries if no item contains it]` with `_unsupported_message(items, view, entries, instruction)` and an `Attempt` holding `unresolved`; after `check` passes: (O8) entries whose label is None or not a label the formula uses refuse with their `said`s as items; then `_kind_guard`; then `_too_far` (message name = the user's word for that kind, else `ja`); then success as today. `Translation.scene` stays the frozen Scene.
- **Object items (spec §9.6).** An item names an object only when it has no handling verb, its object word (the last vocabulary word in the item, or the last contained entry `said`; on a tie the `said` wins) is followed by nothing or by a `_GOING_TAIL` fullmatch, and — when nothing follows — the word's clause in the instruction (from the word up to the next 「、。,.，．！!？?」) starts with a going/circling phrase other than a bare particle, or with と / や / 、 / ・ and another object word that passes the same test (depth ≤ 4). A word that is not in the instruction is trusted. If the word occurs more than once, any occurrence that passes is enough (M5). **Whitespace is removed from the clause before matching** (Review Focus 4).
  - Because every part of `_GOING_TAIL` is optional, `.match` succeeds on any clause, at worst with an empty match. So the clause test counts a match only when the matched text contains a place word (ところ … 周囲), まで, or a verb (歩く … 〈数〉周して); an empty match or a bare を / に / へ does not count (spec §9.6 rule 3 excludes exactly those three particles). `going_targets` uses the same test. The item test (`fullmatch` on the rest of the item after the object word) keeps the empty rest as "nothing follows", which then goes to the clause test.
- **Kind of an object word (spec §9.7).** A vocabulary word gives its label, named by that word. An entry's `said` gives the label of its own head word when that word ends the `said` or is followed by a going tail, else Claude's accepted label named `ja`, else no kind.
- **Message per kind (spec §9.9 table).** Scene count 0 → `msg_not_found`; every one > 6 m from the frozen pose → `msg_too_far`; ≥ 2 listed → `msg_ambiguous(label, Scene count, directions of the listed ones in listing order, word)`; 0 listed → `msg_not_listed`; 1 listed → no count message (the item keeps 「「…」は単語帳にないので、まだできません。」).
- **Kind guard (O35).** Named targets = `going_targets(instruction)`. If the formula uses an object whose label is not a named target and some named target has no object in the Scene, refuse before standing up with `msg_not_found(label, word)` once per such target.

Exact prompt copy (spec §9.2; everything else in the prompt stays as it is):
- Goal sentence: 「答えは JSON で、formula（式の文字列。翻訳できないときは null）、unsupported（表せなかった部分の一覧）、substitutions（言われた言葉と違う形にした部分の一覧）、unresolved_objects（Objects の 1 つに決められなかった物の名前。ふつうは []）を返します。」
- Re-ask: 「直した答え（formula・unsupported・substitutions・unresolved_objects のすべて）を返してください。」
- `### 物`, two new lines after the 「名前（箱・椅子 など）に当てはまる物が…」 line:
  - 「- 同じ物の別の言い方（いす・チェア など）はその物ですが、違う種類の物（棚と冷蔵庫 など）にはしません。「近いほうの椅子」「奥の箱」のように言われたら、Objects の距離で選びます。」
  - 「- 「家具」「家電」のように種類をまとめた言葉は、当てはまる物が Objects にちょうど 1 つのときだけその物にして、substitutions に {"said": "家具", "as": "椅子（〈id〉）"} のように入れます。そうでなければ unsupported だけに入れます。「あれ」「それ」も unsupported です。」
- The line 「当てはまる物が Objects にないとき…formula を null にします。」 gains: 「物まで行く・物の周りを回る指示なら、unresolved_objects にも {"said": 指示の中の物の名前, "label": 物の種類の英語（小文字の単数形。例：box、potted_plant）} を入れます。ほかは [] です。」 The 障害物 line stays last.
- Examples: the 9 object examples and the intro of spec §9.2 (the table there; the intro ends 「…ここより前の例の unresolved_objects は、どれも [] です）」).
- Word note (O33), after the object lines, only when an object is listed and a found word differs from its kind's `ja`; words in instruction order, once each, at most 4, in the vocabulary's spelling: 「指示の「鉢植え」は観葉植物のことです。」, for two 「指示の「いす」は椅子、「鉢植え」は観葉植物のことです。」.

- [ ] **Step 1: Write the failing tests.** In `tests/test_object_language.py` (the list of `design-language.md` §7 with the spec's rev-2 values):

```python
def test_unresolved_entry_picks_the_count_message():     # FakeLLM replies carry the field
    # Scene() + {机, desk} -> msg_not_found("desk", "机"); TWO_PLANTS + {観葉植物, potted_plant} ->
    # msg_ambiguous("potted_plant", 2, ("左前", "右前"), "観葉植物"); a plant at 7 m -> msg_too_far;
    # CROWDED + {グリーン, potted_plant} -> msg_not_listed("potted_plant")
def test_unresolved_label_must_be_a_vocabulary_label():  # kitchen, person, sim_box, "" -> 単語帳; "Potted Plant",
    # "potted-plants" -> potted_plant; "desk" -> desk; a 65-character label -> None; malformed items dropped, no re-ask
def test_unresolved_said_must_be_words_of_the_instruction():   # a copied 「机」 dropped, the formula stands; a note cut
def test_an_entry_refuses_a_formula_unless_its_kind_is_used():
    # F hello + {机, desk} -> refused, unsupported == ["机"], attempts[0].unresolved == [{"said": "机", "label": "desk"}]
    # F near_chair_1 + {椅子, chair} -> ok
def test_words_come_first_then_claudes_label():          # 段ボール+chair -> box message; グリーン+potted_plant ->
    # 観葉植物 message; テーブルの上のコップ+cup -> 単語帳; an entry in no item is appended to unsupported
def test_only_going_or_circling_words_get_count_messages():   # the two tables of spec §9.6, on Scene() and TWO_BOXES
    # plus the empty-tail and bare-particle cases: going_targets("箱。椅子まで行って") == {"chair": "椅子"} and
    # going_targets("箱を、椅子まで行って") == {"chair": "椅子"}; the item 「箱」 refused under either instruction gives the
    # 単語帳 message, not a count message; going_targets("箱まで、椅子を押して") == {"box": "箱"} (まで alone counts)
def test_kind_guard_refuses_another_kind():
    t = translate("ゴミ箱まで行って", llm_for("F near_box_1"), scene=SCENE)
    assert t.formula is None and t.message == msg_not_found("trash_can", "ゴミ箱")
    assert translate("冷蔵庫の横の椅子まで行って", llm_for("F near_chair_1"), scene=SCENE).ok
    assert translate("ゴミ箱まで行って", llm_for("F near_box_1"), scene=with_a_trash_can).ok
def test_kind_guard_and_count_messages_ignore_spaces():  # Review Focus 4
    t = translate("ゴミ箱 まで　行って", llm_for("F near_box_1"), scene=SCENE)
    assert t.formula is None and t.message == msg_not_found("trash_can", "ゴミ箱")
    assert refusal_message("机 の 前まで 行って", ["机"], SCENE) == msg_not_found("desk", "机")
def test_word_note():                                     # いすまで行って -> 「指示の「いす」は椅子のことです。」 after
    # the list; none for 椅子まで, for 「x」, or with nothing listed; two words joined; at most 4; an injected 「」 in the
    # instruction never appears in the note
def test_focus_lists_a_named_kind_beyond_the_8_nearest():   # CROWDED: 「観葉植物まで行って」 passes
    # F near_potted_plant_1 on the first try; 「x」 with the same reply is re-asked; the prompt lists 8
def test_multi_word_ids_in_the_prompt():                  # potted_plant_1（観葉植物） lines and their 3 vocabulary lines
def test_message_uses_the_users_word():                   # 机まで行って on SCENE -> 「「机」が見つかりません。…」;
    # a far table named 机 -> 「「机」が遠すぎます（6 m まで）。」
```
  Also in that file: `example()` takes an optional 5th argument; `test_refusal_line_and_the_object_rules` keeps every pin and adds the two lines and the field sentence; `test_object_examples_follow_the_others_with_their_scene` pins the new intro, `EXAMPLE_SCENE.listed()` ids `["box_9", "potted_plant_9", "chair_9"]`, 4-key object examples and no `"unresolved_objects"` before the intro; `test_object_examples_pass_the_check_with_the_example_scene` uses 5-tuples (each `said` is in its instruction and `lookup` of its label is a vocabulary label: `desk`); the ambiguous tests expect `msg_ambiguous(…, directions)`; `test_prompt_with_8_objects_and_a_reask_…` gains a variant with 8 `water_dispenser_92..99`, a re-ask and a 4-word note (< 20 000 characters); the two boundary tests rewrite the 10 items of spec §13.1 (箱の前でおすわり, 椅子のそばでおすわりして, 箱だとおもって, 椅子をなおして, 箱のまわりをもっと回る, 箱のあいだをとおして, 箱のあいだをとおします, 椅子のあいだをとおさないで, 箱とにもつのあいだ, 箱のことをおしえて) so the kana trap stays and the item ends in a going phrase (e.g. 「おすわりのあと箱まで」), and a new `test_former_boundary_items_read_as_vocabulary` asserts the original 10 now give the 単語帳 message.
  In `tests/test_grammar_translator.py`: `test_retry_asks_for_the_whole_answer` (new re-ask), `test_schema_requires_substitutions_of_said_and_as` (4 required keys, the item shape), new `test_goal_names_unresolved_objects`. In `tests/test_llm.py`: new `test_reply_without_unresolved_objects_is_unreadable` (ClaudeCLI with a fake runner whose `structured_output` lacks the key, and Codex's output file without it → `LLMError(MSG_UNREADABLE)`). In `tests/test_session_language.py`: `test_unresolved_object_refusal_does_not_stand_up` (「観葉植物まで行って」 refused with the field on the default layout → snapshot message `msg_not_found("potted_plant", "観葉植物")`, `link.sim.accepted == []`, the attempt shows the said), `test_kind_guard_does_not_stand_up` (「ゴミ箱まで行って」 + `F near_box_1`), `test_snapshot_redacts_unresolved_said` (a secret-like `said` is hidden inside `attempts[].unresolved`).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_object_language.py tests/test_grammar_translator.py tests/test_llm.py tests/test_session_language.py`. Expected: FAIL.
- [ ] **Step 3: Implement** the Interfaces and rules above. `llm.py` needs no change (`_validated` already checks `required` and item types); if the new test shows otherwise, stop and report.
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add voice_ltl/translator.py voice_ltl/prompt.py voice_ltl/session.py tests/test_object_language.py tests/test_grammar_translator.py tests/test_llm.py tests/test_session_language.py` and `git commit -m "feat: reply field for unresolved objects, kind guard and count messages for every kind"`.

### Task 1.5: Page (stage 1c, branch `objects2/page`)

**Files:**
- Modify: `voice_ltl/static/app.js`, `voice_ltl/static/style.css` (the `--obj-fill` comment becomes 「(a chair, a plant, …)」), `tests/test_server.py`

**Interfaces:**
- Consumes: snapshot rows (`id`, `label`, `name`, `x`, `y`, `size`, `visible`, `where`), `s.pose` = `[x, y, yaw]` or null; message functions of Task 1.2 for `NOTICE_KINDS`.
- Produces (`app.js`):

```js
function objectStyle(o)            // {fill: "--box-fill", line: "--box-line"} for label "box", else the --obj-* pair
function objectName(o)             // o.name || o.label.replaceAll("_", " ")
function clipText(ctx, text, maxWidth)   // whole text if it fits, else the longest prefix + "…"; width = ctx.measureText(t)?.width ?? 0
function objectsLine(objects, pose = null)   // visible rows only, nearest to pose first (snapshot order without a pose),
                                             // at most 8, then 「、ほか N 個」; none -> 「見えている物：なし」
```
- The map draws each name with `clipText(ctx, objectName(o), Math.max(rectWidthPx + 16, 48))` at 11 px; the aria label and the line keep the full name. The call site is `objectsLine(s.objects ?? [], s.pose ?? null)`. Nothing shows `source`, `confidence` or `measured` (O23).

- [ ] **Step 1: Write the failing tests** in `tests/test_server.py` (node harness): add to `NOTICE_KINDS` the rows `("error", msg_not_listed("box"), "error")`, `("error", msg_not_found("potted_plant"), "error")`, `("error", msg_not_found("table", "机"), "error")`, `("error", msg_ambiguous("box", 2, ("左前", "右前")), "error")`, `("error", msg_lap_too_big("sofa"), "error")`, `("error", msg_too_big_here("bed"), "error")`; new tests:
  - `test_objects_line_sorts_by_distance_and_caps_at_8`: 10 visible rows → the first 8 by distance from the pose and 「、ほか 2 個」; without a pose, snapshot order; e.g. 「見えている物：箱（左前 2.9 m）、椅子（右前 2.9 m）、観葉植物（左 3.0 m）、ゴミ箱（右 3.0 m）」.
  - `test_map_clips_long_names`: the CANVAS stub's `measureText` returns 11 px per character; a 0.35 m `water_dispenser_1` draws 「ウォーター…」-style clipped text no wider than max(rect + 16, 48) px; the aria label keeps 「ウォーターサーバー」.
  - `test_map_new_kinds_use_obj_colours`: a `potted_plant` row is filled with `--obj-fill`, a `box` row with `--box-fill`.
  - `test_page_name_falls_back_to_the_label`: a row without `name` draws 「potted plant」.
  - `test_page_never_shows_source_or_confidence`: a detector row with `confidence` and `measured` changes no drawn text and no line text.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_server.py -rs`. Expected: FAIL, and no `node is not installed` skip.
- [ ] **Step 3: Implement** the four functions and the call sites.
- [ ] **Step 4: Run.** Same command. Expected: PASS with no node skip.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add voice_ltl/static/app.js voice_ltl/static/style.css tests/test_server.py` and `git commit -m "feat: page names, colours and the seen line for every kind"`.

### Task 1.6: Layout files and fake Go2 (stage 1c, branch `objects2/layouts`)

**Files:**
- Create: `voice_ltl/layout.py`, `gazebo/layouts/default.json`, `docker/fuel_models.json`, `tests/test_layout.py`
- Modify: `sim/sim_core.py`, `sim/fake_go2.py`, `compose.yaml` (the `sim` service only), `tests/test_sim_objects.py`, `tests/test_sim_core.py`

**Interfaces:**
- Consumes: `vocabulary.LABELS`, `objects.Obj`, `objects._id_key`, `objects.format_message(source=)`.
- Produces:

```python
# voice_ltl/layout.py (stdlib only; spec §12.5)
ROOT: Path; LAYOUTS_DIR = ROOT / "gazebo" / "layouts"; FUEL_MODELS = ROOT / "docker" / "fuel_models.json"
PRIMITIVES = {"primitive:box": "box", "primitive:chair": "chair"}
class LayoutError(ValueError): ...           # names the file, the id and the value; an unknown name lists names()
@dataclass(frozen=True)
class Entry:
    id: str; label: str; model: str; x: float; y: float; yaw_deg: int
    offset: tuple[float, float]; size: tuple[float, float]       # model axes (as measured on the model)
    height: float; z: float; detectable: bool; accept: tuple[str, ...]
    def world_size(self) -> tuple[float, float]                  # x/y swapped at 90 and 270 deg
    def world_offset(self) -> tuple[float, float]                # offset rotated by yaw_deg
    def centre(self) -> tuple[float, float]                      # (x, y) + world_offset()
    def obj(self) -> Obj                                         # Obj(id, label, *centre(), world_size())
    @property
    def folder(self) -> str | None                               # "Sofa" for model://Sofa, None for primitives
@dataclass(frozen=True)
class Layout:
    name: str; entries: tuple[Entry, ...]
    def objs(self) -> tuple[Obj, ...]; def find(self, obj_id: str) -> Entry | None
def names() -> tuple[str, ...]                                   # the layout files' stems
def model_folders() -> frozenset[str]                            # folders listed in docker/fuel_models.json
def parse(data: object, name: str, folders: frozenset[str] | None = None) -> Layout
def load(name: str) -> Layout
# sim/sim_core.py
@dataclass
class SimObject: id; label; x; y; sx; sy; height: float = BOX_HEIGHT
def layout_objects(name: str) -> tuple[tuple[Obj, ...], dict[str, float]]   # objs and id -> height
SimGo2.place_objects(layout: Iterable[Obj] = DEFAULT_LAYOUT, heights: Mapping[str, float] | None = None)
# FakeCamera.see orders by objects._id_key; cloud_lidar uses each object's own height (button boxes keep BOX_HEIGHT)
# sim/fake_go2.py
def objects_message(camera: FakeCamera, sim: SimGo2, now: float) -> dict   # format_message(..., source="fake")
# FakeGo2 places layout_objects(os.environ.get("GO2_LAYOUT", "default")) when it builds its own SimGo2
```
- Layout entry keys: `id`, `model` (`primitive:box`, `primitive:chair` for those labels only, or `model://<folder>` listed in `docker/fuel_models.json`), `x`, `y`, `yaw_deg` (a multiple of 90), `footprint` `{offset: [dx, dy], size: [sx, sy]}` in model axes, `height`, optional `z` (default 0.0, `model://` only), `detectable`, `accept` (vocabulary labels including the id's own). Sizes and heights in (0, 3.0]; ids unique, each id's label in the vocabulary.
- `docker/fuel_models.json`: `{"version": 1, "base": "https://fuel.gazebosim.org/1.0/OpenRobotics/models", "models": [...]}` with, per model, `folder`, `version`, `bytes`, `sha256`, `author`, `licence`, `licence_url`: Sofa 3 (1 311 028 B, `6d19d1f109427626d88667de047ca80a2c175b3f8f12dc2d0e75ddb3b88a0db1`, Wan Yi Seow, CC BY 4.0), TrashBin 2 (1 045 623 B, `ff38c8c7d7163145083e75166201a0d8c594c3170451ecf753fc77e173c00de3`, Roselle Carmen, CC BY 4.0), LampAndStand 3 (676 942 B, `f8610eb71132cc511634937e17fef21f6c15d9ddc9612dbedcd45979566485eb`, Roselle Carmen, CC BY 4.0), Table 4 (899 222 B, `3a720ef2a8e2f416e0ed017b2e13936f08f6f7c97829c9ea4b0d15380e36163d`, Nate Koenig, CC0 1.0), Suitcase1 2 (1 802 850 B per Fuel, `sha256: null` until H1, author null, CC0 1.0).
- `gazebo/layouts/default.json`: `box_1` (`primitive:box`, 2.5, 1.5, 0.5 × 0.5, height 0.5, detectable false, accept [box]) and `chair_1` (`primitive:chair`, 2.5, −1.5, 0.45 × 0.45, height 0.9, false, [chair]).
- `compose.yaml`, service `sim`: `environment: GO2_LAYOUT: "${GO2_LAYOUT:-default}"` (M8; the app gets no `GO2_LAYOUT`).

- [ ] **Step 1: Write the failing tests** in `tests/test_layout.py`:

```python
def test_default_layout_equals_the_objects_default_layout():
    assert layout.load("default").objs() == objects.DEFAULT_LAYOUT

def test_world_axes_footprint():
    e = entry_with(x=1.0, y=2.0, yaw_deg=90, offset=(0.19, 0.0), size=(0.9, 0.3))
    assert e.world_size() == (0.3, 0.9) and e.world_offset() == pytest.approx((0.0, 0.19))
    assert e.centre() == pytest.approx((1.0, 2.19)) and e.obj().size == (0.3, 0.9)
    assert entry_with(yaw_deg=180, offset=(0.02, 0.0)).world_offset() == pytest.approx((-0.02, 0.0))

@pytest.mark.parametrize("bad", ["an id whose label is not a kind", "a duplicate id", "primitive:table",
    "model://Unknown", "yaw_deg 45", "size 0", "size 3.1", "height 0", "accept without its own label",
    "accept with an unknown label", "an unknown key", "version 2", "z on a primitive"])
def test_loader_refuses(bad): ...                       # each raises LayoutError

def test_unknown_layout_names_the_valid_layouts():      # Review Focus 5
    with pytest.raises(layout.LayoutError) as e:
        layout.load("houshold")
    assert "houshold" in str(e.value) and "default" in str(e.value)

def test_fuel_models_list_is_the_spec_pins():
    m = {x["folder"]: x for x in json.loads(layout.FUEL_MODELS.read_text())["models"]}
    assert {k: v["version"] for k, v in m.items()} == {"Sofa": 3, "TrashBin": 2, "LampAndStand": 3, "Table": 4, "Suitcase1": 2}
    assert m["Sofa"]["sha256"] == "6d19d1f109427626d88667de047ca80a2c175b3f8f12dc2d0e75ddb3b88a0db1"
    assert layout.model_folders() == frozenset(m)

def test_compose_passes_go2_layout_to_the_fake_go2():   # sim service only
```
  In `tests/test_sim_objects.py`: `test_fake_lidar_uses_object_heights` (chair points reach 0.9 m, box 0.5 m, a button box 0.4 m); `test_fake_camera_orders_ids_by_number` (`box_2` before `box_10`); `test_layout_objects_reads_a_layout` (`layout_objects("default")` gives the two objects and heights 0.5 and 0.9); `test_fake_go2_message_says_fake_and_keeps_its_rows` (`objects_message(...)["source"] == "fake"` and every row equals `Obj.to_row()` without `confidence` or `measured`).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_layout.py tests/test_sim_objects.py`. Expected: FAIL (`No module named 'voice_ltl.layout'`).
- [ ] **Step 3: Implement** the Interfaces. `layout.load` reads `LAYOUTS_DIR / f"{name}.json"`; unknown names raise with `names()`.
- [ ] **Step 4: Run.** Run: `T tests/test_layout.py tests/test_sim_objects.py tests/test_sim_core.py tests/test_session_objects.py`. Expected: PASS (if a session test moves because of the taller fake LiDAR walls, stop and report: that file belongs to Task 1.3).
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add voice_ltl/layout.py gazebo/layouts/default.json docker/fuel_models.json sim/sim_core.py sim/fake_go2.py compose.yaml tests/test_layout.py tests/test_sim_objects.py tests/test_sim_core.py` and `git commit -m "feat: layout files with model-axis footprints; the fake Go2 reads GO2_LAYOUT"`.

### Task 1.7: Detour rebuild (O15) (stage 1d, branch `objects2/detour`)

**Files:**
- Modify: `voice_ltl/session.py`
- Create: `tests/test_session_detour.py`

**Interfaces:**
- Consumes (Task 1.3): `grid_world(..., margin=)`, `GridError`; `automata.synthesize(formula, world, workdir, timeout_s)`.
- Produces: `session.DETOUR_MARGIN = 3`. In `Session._synthesize`, when the supervisor is empty and `_empty_message` gives `MSG_NO_PATH`, build the grid once more with `margin=DETOUR_MARGIN` (skip it when that raises `GridError`, e.g. over 256 cells) and synthesize again with the time left of the same 20 s budget (M6). A non-empty result is used as usual; an empty one keeps `MSG_NO_PATH`.

- [ ] **Step 1: Write the failing tests** in `tests/test_session_detour.py`:
  - `test_detour_rebuild_finds_a_path_with_a_wider_margin`: a scene whose 1-cell-margin grid has no path but the 3-cell one has (suggested: a 0.5 m box at (3, 0) and a row of 0.3 m boxes at x = 1.5 for y = −1.75 … 1.75 in 0.5 m steps); assert the precondition with `grid_world` at margins 1 and 3 first, then that the session plans `F near_box_1` and reaches `done`.
  - `test_detour_rebuild_is_skipped_over_256_cells`: a target near the cap → `MSG_NO_PATH`, `synthesize` called once.
  - `test_detour_rebuild_runs_once`: an enclosed target → `MSG_NO_PATH`, `synthesize` called exactly twice.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_session_detour.py`. Expected: FAIL.
- [ ] **Step 3: Implement** the rebuild in `Session._synthesize`.
- [ ] **Step 4: Run.** Run: `T tests/test_session_detour.py tests/test_session_objects.py`. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add voice_ltl/session.py tests/test_session_detour.py` and `git commit -m "feat: rebuild once with a 3-cell margin when no path reaches an object (O15)"`.

### Task 1.8: Phrasing trials script (stage 1d, branch `objects2/trials`)

**Files:**
- Modify: `scripts/object_phrasing_trials.py`, `tests/test_phrasing_trials.py`

**Interfaces:**
- Consumes: Task 1.4 (`translate`, `Attempt.unresolved`, `msg_*`), Task 1.3 (plans for the BIG scene), Task 1.2 (`Scene.focus`, `labels_in`).
- Produces: scenes `KINDS`, `CROWDED`, `TWO_BOXES_AHEAD`, `TWO_PLANTS`, `SHELF`, `BIG` (spec §13.2 table; `SHELF` holds `bookshelf_1` 0.9 × 0.3) in `SCENE_NAMES`; the 70 cases of spec §13.2 in `GROUPS` (lap 10, near 10, kinds 10, objects 13, refusals 14, regression 13); `REFUSAL_WORDS` gains 「選べません」; `PASS_GROUPS = ("lap", "near", "kinds", "refusals")`; `Result.record()` adds per attempt `"unresolved"` (accepted) and `"raw_unresolved"` (Claude's raw field, kept by a per-case LLM wrapper inside this script only); `check_trial_vocabulary() -> None` raises before any LLM call unless every scene label is a vocabulary label, 観葉植物・鉢植え・ゴミ箱・ごみ箱・机・冷蔵庫・ボール・本棚・ソファ・テレビ・食卓 are vocabulary words and グリーン・家具・人 are not. The docstring states the exit-code rule (0 when each pass group ≥ 0.9; a `--groups` without all four → `"pass": null`, exit 2).

- [ ] **Step 1: Write the failing tests** in `tests/test_phrasing_trials.py`: `test_groups_follow_the_plan` (counts `{lap: 10, near: 10, kinds: 10, objects: 13, refusals: 14, regression: 13}`, the new scenes' `where()` lines as in spec §13.2, `REFUSAL_WORDS`, `PASS_GROUPS`); `test_every_reference_formula_passes_the_check_and_has_a_plan` (check against `replace(scene, focus=labels_in(instruction)).propositions()`); `test_run_groups_counts_per_group` (the 「机」 FakeLLM reply now gives 見つかりません); `test_refusal_needs_the_right_message` (`msg_ambiguous` with directions); `test_main_exits_0_only_when_the_four_groups_pass` and `test_main_prints_the_summary_after_an_unexpected_error` (run with `--groups lap,near,kinds,refusals`; `LLMByInstruction` gets right answers for kinds and refusals; `lap,near` alone → exit 2); `test_pass_needs_the_four_groups` (13/14 refusals passes, 12/14 fails, a missing group → None); new `test_refusal_cases_dry_run` (each refusal case's ideal FakeLLM reply — formula null, the noun in `unsupported` and in the field — gives a message with its expected word, guard cases included); `test_trial_labels_and_words_are_in_the_vocabulary`; `test_record_has_unresolved_and_raw_unresolved`.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_phrasing_trials.py`. Expected: FAIL.
- [ ] **Step 3: Implement** the script changes.
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add scripts/object_phrasing_trials.py tests/test_phrasing_trials.py` and `git commit -m "feat: 70 phrasing trials with new kinds, refusals as a pass group"`.

---

## Wave 2: the detector path (A) and the Gazebo side of the shared step

### Task 2.1: Detector camera and geometry (stage 2a, branch `objects2/det-geometry`)

**Files:**
- Create: `detector/__init__.py` (empty), `detector/camera.py`, `detector/geometry.py`, `tests/test_detector_geometry.py`

**Interfaces:**
- Consumes: `objects.CAMERA_IN_BODY`, `objects.measure_radius`, `bridge_core.LIDAR_IN_BODY`, `bridge_core.lidar_points_world`, `bridge_core.quat_to_matrix`.
- Produces:

```python
# detector/camera.py (numpy only)
CAMERA_W = 640; CAMERA_H = 480; CAMERA_RATE_HZ = 2.0; CAMERA_HFOV = 2.094; CAMERA_CLIP = (0.1, 10.0); PROJ_NEAR = 0.1
@dataclass(frozen=True)
class CameraModel:
    width: int; height: int; fx: float; fy: float; cx: float; cy: float
    @classmethod
    def from_hfov(cls, width: int = CAMERA_W, height: int = CAMERA_H, hfov: float = CAMERA_HFOV) -> "CameraModel"
    @classmethod
    def from_k(cls, k: Sequence[float], width: int, height: int) -> "CameraModel"
    def agrees(self, other: "CameraModel", tol: float = 0.01) -> bool
    def project(self, p_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]   # (uv N x 2, mask: x > PROJ_NEAR and inside)
    def ray(self, u: float, v: float) -> np.ndarray                         # unit direction, camera frame
def t_cam_lidar(lidar_in_body=LIDAR_IN_BODY, camera_in_body=CAMERA_IN_BODY) -> np.ndarray   # 4 x 4, translation only here
def camera_pose(base_pos, base_quat) -> tuple[np.ndarray, np.ndarray]        # camera position (3) and R (3 x 3) in odom
def world_to_camera(points_world: np.ndarray, base_pos, base_quat) -> np.ndarray   # frame x forward, y left, z up
# detector/geometry.py (numpy only; spec §11.6)
GROUND_Z = 0.05; HEIGHT_CAP = 2.5; MASK_ERODE_PX = 2; BOX_SHRINK = (0.10, 0.05); SEED_EPS = 0.25
MIN_CLUSTER_POINTS = 5; MIN_CLUSTER_SHARE = 0.25; EDGE_PAD = 0.05; SIZE_MIN = 0.1; DET_RANGE = 8.0
FALLBACK_BOTTOM_PX = 3; VACANCY_GROW = 0.1; VACANCY_MIN_HEIGHT = 0.3
@dataclass(frozen=True)
class Footprint:
    x: float; y: float; size: tuple[float, float]
    filled: bool                                       # the depth came from the vocabulary prior (C2)
    observed: tuple[float, float, float, float]        # (cx, cy, sx, sy) of the observed rectangle, no depth fill
def cut(points_world: np.ndarray) -> np.ndarray
def seed_points(points_world, uv, in_image, box, mask) -> np.ndarray
def clusters(points_xy: np.ndarray, eps: float = SEED_EPS) -> list[np.ndarray]   # index arrays, single linkage
def pick_cluster(points_world: np.ndarray, groups: list[np.ndarray], cam_xy) -> np.ndarray | None
def grow(seed: np.ndarray, points_world: np.ndarray, radius: float, z_max: float) -> np.ndarray
def footprint(points_xy: np.ndarray, cam_xy, prior: tuple[float, float]) -> Footprint | None
def plausible(fp: Footprint, prior: tuple[float, float]) -> bool                # max side <= 2L + 0.5
def ground_ray(box, camera: CameraModel, cam_pos, cam_r, prior) -> Footprint | None
def vacancy_counts(points_world, lidar_pos, centre, size, height) -> tuple[int, int]   # (n_in, n_through)
def is_vacant(n_in: int, n_through: int) -> bool                                 # n_in < 3 and n_through >= 5
@dataclass(frozen=True)
class Sighting:                                        # what the tracker needs from one detection (Task 2.2's Observation)
    footprint: Footprint | None                        # None: no usable geometry (seed alone implausible, spec §11.6 step 7)
    supported: bool                                    # LiDAR-supported; False for the ground-ray fallback and for None
    fallback: bool                                     # footprint from ground_ray
    points_xy: np.ndarray | None                       # N x 2 odom XY of the picked cluster (grown, or the seed alone when the
                                                       # growth was implausible), before the MAD trim; None unless supported
    cam_xy: tuple[float, float]                        # camera position in odom at the image stamp
def observe(box, mask, points_world, base_pos, base_quat, camera: CameraModel,
            label: str) -> Sighting
```
- `observe` maps 1:1 onto `tracker.Observation` (Task 2.2), so Task 2.7 builds it without redoing the geometry: `Observation(det.label, det.confidence, det.box, fp.x, fp.y, fp.size, fp.filled, s.points_xy, s.supported, s.fallback, s.cam_xy)` with `fp = s.footprint`. When supported, `s.footprint == footprint(s.points_xy, s.cam_xy, prior)`, so the tracker's `fuse` over the last 6 hits uses the same points and the same function.
- The algorithms are spec §11.6 and `design-detector.md` §7.2–§7.5: seed from the eroded mask (or the shrunk box), seed clusters at 0.25 m, the nearest cluster with ≥ max(5, 25 % of the largest), growth over all cut points (360°) with eps max(0.15, 1.5·r·1°) within `measure_radius(label)` of the seed median and z ≤ height + 0.3, the plausibility fallback to the seed, the one-view footprint with MAD trim and depth completion (`filled = True` whenever the depth is not the observed one), `EDGE_PAD` on each side, sides clamped to [0.1, 3.0]; the ground-ray fallback gives [L, L] and is never "supported".

- [ ] **Step 1: Write the failing tests** in `tests/test_detector_geometry.py` (`design-detector.md` §16 "Camera and extrinsics" and "Selection, clusters, growth, footprint", with these values):
  - `test_fx_from_hfov_matches_the_spike`: `CameraModel.from_hfov(320, 240).fx == pytest.approx(92.418, abs=1e-3)` and `from_hfov().fx == pytest.approx(184.836, abs=1e-3)`, `cx == 320`, `cy == 240`.
  - `test_spike_box_projects_to_its_recorded_pixel`: camera point (2.172, 1.5, −0.147) → (192.4, 252.5) ± 0.1 px.
  - `test_t_cam_lidar_matches_the_xacro`: the translation equals `LIDAR_IN_BODY − CAMERA_IN_BODY` = (−0.04715, 0.00003, 0.05703), both parsed from `gazebo/go2_sensors.urdf.xacro`'s `utlidar_joint` and `logical_camera_joint`.
  - `test_points_behind_or_outside_are_dropped`.
  - `test_cut_with_a_lying_pitch`; `test_mask_seed_drops_edge_points`; `test_seed_clusters` (two groups 0.4 m apart → 2; a chain with 0.2 m gaps → 1); `test_pick_nearest_substantial_cluster` (face + wall → the face; a 2-point sliver in front → ignored; a big occluder in front → chosen, documented limit).
  - `test_growth` (a table half outside the image → full width; a chair 0.3 m from a table is not joined; a sofa against a wall → implausible → seed-only footprint; the eps grows with range).
  - `test_footprint_from_one_view` (a face-on 0.5 m box with prior (0.5, 0.5) → centre within 0.03 m, sides 0.5–0.62, `filled` True; a corner view → no completion, `filled` False; a 1.8 × 0.9 sofa from its short side → depth L; a 30°-rotated sofa → the AABB of the rectangle; a stray point is trimmed).
  - `test_published_footprint_covers_the_object` (O12): true boxes 0.3–2.0 m, yaws 0–90° in 15° steps, ranges 1–5 m, simulated 1° × 4° returns of the visible faces: the AABB contains the true footprint whenever the visible faces are fully inside the LiDAR band; the exceptions are listed in the test.
  - `test_ground_ray_fallback` (distance h·fy/(v − cy) on a flat floor; box at the bottom edge → None; above the horizon → None; a tilted camera; size [L, L]).
  - `test_vacancy` (present → not vacant; removed with a wall behind → vacant; removed behind the Go2 → vacant; an occluder in front → not vacant; behind the Go2's own body → not vacant; rays passing over a low object are not counted).
  - `test_observe_gives_what_the_tracker_needs` (a supported face-on box: `supported` True, `fallback` False, `points_xy` N × 2 inside the true footprint ± 0.1 m, `cam_xy` equal to `camera_pose(...)`'s XY, and `footprint(points_xy, cam_xy, prior) == footprint`; a box with 2 LiDAR points and a visible bottom edge: `fallback` True, `supported` False, `points_xy` None, size [L, L]; an implausible seed: `footprint` None, `supported` False).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_detector_geometry.py`. Expected: FAIL (`No module named 'detector'`).
- [ ] **Step 3: Implement** `detector/camera.py` and `detector/geometry.py`.
- [ ] **Step 4: Run.** Same command. Expected: PASS, and `TA python3 -c "import detector.geometry, sys; assert 'cv2' not in sys.modules and 'torch' not in sys.modules"` exits 0 (in the app image; `TA` puts `/wt/nl_control` on the path for `gazebo.bridge_core`'s `go2_nl_control` import).
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add detector/__init__.py detector/camera.py detector/geometry.py tests/test_detector_geometry.py` and `git commit -m "feat: detector camera model and LiDAR footprint geometry"`.

### Task 2.2: Detector protocol and tracker (stage 2a, branch `objects2/det-tracker`)

**Files:**
- Create: `detector/protocol.py`, `detector/tracker.py`, `detector/scripted.py` (the shared test detector, review M7), `tests/test_detector_tracker.py`, `tests/test_detector_protocol.py` (`detector/__init__.py` comes from Task 2.1; until the merge `detector` imports as a namespace package)

**Interfaces:**
- Consumes: `vocabulary.LABELS`, `objects.Obj`, `objects._id_key`.
- Produces:

```python
# detector/protocol.py (spec §11.10)
@dataclass(frozen=True)
class Detection: label: str; confidence: float; box: tuple[float, float, float, float]; mask: np.ndarray | None = None
@dataclass(frozen=True)
class Thresholds: sighting: float; confirm: float; confirm_unsupported: float; by_label: Mapping[str, float] = field(default_factory=dict)
class Detector(Protocol):
    name: str; thresholds: Thresholds
    def set_vocabulary(self, prompts: Mapping[str, Sequence[str]], ignore: Sequence[str]) -> None
    def detect(self, image_rgb: np.ndarray) -> list[Detection]
    def close(self) -> None
    # optional: def prepare(self, view) -> None — the core calls it before detect() when present (view: detector.core.View)
YOLOE_THRESHOLDS = Thresholds(0.25, 0.35, 0.5)
def check_detections(dets: Sequence[Detection], image_shape: tuple[int, ...], labels: frozenset[str]) -> None   # the contract
def load_detector(name: str | None = None) -> Detector   # env DETECTOR, default "yoloe"; lazy import: yoloe -> detector.yoloe,
                                                         # oracle -> detector.oracle, yoloe_pf -> detector.yoloe_pf; else ValueError
# detector/scripted.py (stdlib + numpy; the one test detector, used by the tests of 2.2, 2.7, 2.10 and 2.12)
class ScriptedDetector:                                  # name "scripted"; satisfies Detector
    def __init__(self, script: Sequence[list[Detection]] | Callable[[int, np.ndarray], list[Detection]],
                 thresholds: Thresholds = YOLOE_THRESHOLDS, delay_s: float = 0.0, fail_on: frozenset[int] = frozenset()) -> None
        # detect() call k returns script[k] (the last one repeats) or script(k, image); sleeps delay_s first;
        # raises RuntimeError on the calls in fail_on (for the 3-errors backoff tests)
    calls: int; vocabulary: tuple[Mapping[str, Sequence[str]], Sequence[str]] | None   # what set_vocabulary got
    def set_vocabulary(self, prompts, ignore) -> None; def detect(self, image_rgb) -> list[Detection]; def close(self) -> None
# detector/tracker.py (spec §11.7)
M = 2; N = 3; MIN_HIT_GAP_S = 0.5; TENTATIVE_TTL_S = 30.0; ASSOC_BASE = 0.5; ASSOC_GAIN = 0.5; FALLBACK_EXTRA = 0.5
XLABEL_IOU = 0.4; MERGE_IOU = 0.5; RELABEL_HITS = 5; RELABEL_RATIO = 2.0; CONF_WINDOW = 10; TRACK_POINT_FRAMES = 6
VACANT_FRAMES = 2; MISS_UNSUPPORTED = 3; MAX_ID = 99; VISIBLE_HOLD_MIN_S = 5.0
@dataclass(frozen=True)
class Observation:
    label: str; confidence: float; box: tuple[float, float, float, float]
    x: float; y: float; size: tuple[float, float]; filled: bool
    points: np.ndarray | None; supported: bool; fallback: bool; cam_xy: tuple[float, float]
class Tracker:
    def __init__(self, thresholds: Thresholds, priors: Mapping[str, tuple[float, float]],
                 fuse: Callable[[np.ndarray, tuple[float, float], tuple[float, float]], object | None]) -> None
        # fuse(points_xy, cam_xy, prior) returns an object with x, y, size, filled (geometry.footprint), or None
    def update(self, observations: Sequence[Observation], frame_t: float, wall_t: float,
               in_view: Callable[[float, float], bool]) -> None          # one still image
    def vacancy_targets(self) -> list[tuple[str, float, float, tuple[float, float], str]]   # (key, x, y, size, label)
    def mark_vacant(self, key: str, vacant: bool, scan_t: float) -> None
    def rows(self, wall_now: float, idle_period_s: float) -> list[Obj]   # confirmed tracks, _id_key order, confidence/measured set
    def stats(self) -> dict      # tracks_tentative, tracks_confirmed, ids_issued {label: n}, ids_retired, filled [ids], notes
                                 # notes: one string per event, e.g. "box: ids 1-99 used, new box tracks ignored" (C6)
    def reset(self) -> None      # forget every track and restart every label's numbering at 1 (notes cleared too)
```
- `Observation` is filled 1:1 from `geometry.Sighting` (Task 2.1) by the core (Task 2.7); a detection whose `Sighting.footprint` is None gets no `Observation`.
- The tracker never reads names from Gazebo, `/sim/world`, the layout or the truth topic; never publishes a tentative track; and re-checks every published label against `vocabulary.LABELS` (spec §11.7).

- [ ] **Step 1: Write the failing tests** in `tests/test_detector_tracker.py` (`design-detector.md` §16 "Tracker", with a stub `fuse`): sightings (0.24 ignored; 0.30 refreshes a confirmed track's `visible` but creates no id; a new id needs one hit ≥ 0.35; a `by_label` override raises it for that label only); confirmation (1 hit → nothing; 2 hits in 3 in-view frames ≥ 0.5 s apart → `chair_1`; two identical frames 0.5 s apart count twice; two hits within 0.5 s count once; frames out of view or beyond 6 m do not count toward N; a tentative track older than 30 s is dropped without an id; no supported hit → 3 of 3 at ≥ 0.5); association (same label within the gate → one track; two boxes 1.0 m apart → `box_1`, `box_2`; counters per label); ids (`test_ids_are_never_reused_and_stop_at_99`: retire `box_1` → the next is `box_2`; after `box_99` a new box is not published and `stats()["notes"]` has one note (C6); rows in `_id_key` order); votes (`bench` on a confirmed `box` → no new track; overlapping confirmed tracks → older id kept; a chair under a table → both kept; relabel at 2× over 5 hits with one hit ≥ confirm → old id retired, new id); forgetting (vacant twice ≥ 0.5 s apart → retired; misses with LiDAR still occupied → kept, `visible` false after 5 s; out of view → kept; an unsupported track → retired after 3 in-view misses, not when occluded); `test_fused_size_recovers_after_one_bad_frame` (it leaves the window after 6 hits); `test_visible_hold_follows_the_idle_period`; `test_filled_ids_are_reported`; `test_reset_restarts_numbering`; `test_rows_round_trip_through_objects_parse` (with `source="detector"`, confidences in [0, 1], `measured`).
  In `tests/test_detector_protocol.py`, parametrised over the implementations available (`ScriptedDetector` here; Task 2.7 adds `OracleDetector`, Task 2.11 `YoloeDetector` marked `detector`): a list; labels in the vocabulary; confidences in [0, 1]; boxes inside the image with x0 < x1 and y0 < y1; masks of the image shape; the input image unchanged; `sighting ≤ confirm ≤ confirm_unsupported`; overrides only for vocabulary labels; `load_detector("nope")` raises `ValueError` and importing `detector.protocol` imports neither `ultralytics` nor `torch`.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_detector_tracker.py tests/test_detector_protocol.py`. Expected: FAIL.
- [ ] **Step 3: Implement** `detector/protocol.py` and `detector/tracker.py`.
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add detector/protocol.py detector/tracker.py detector/scripted.py tests/test_detector_tracker.py tests/test_detector_protocol.py` and `git commit -m "feat: detector protocol and tracker (confirmation, ids, votes, forgetting)"`.

### Task 2.3: Bridge — layout spawn, truth topic, source switch, sizing — and compose for Gazebo (stage 2a, branch `objects2/bridge`)

**Files:**
- Modify: `gazebo/bridge_core.py`, `gazebo/gazebo_bridge.py`, `compose.yaml`, `tests/test_gazebo_objects.py`, `tests/test_bridge_core.py` (only if a pin moves)

**Interfaces:**
- Consumes: `layout.load`, `layout.Layout`, `layout.Entry`; `objects.default_size`, `objects.measure_radius`, `objects._id_key`, `objects.format_message(source=)`.
- Produces (`gazebo/bridge_core.py`; `OBJECT_SPAWN_Z` and `object_sdf` stay for the primitives):

```python
OBJECTS_SOURCES = ("logical", "detector"); TRUTH_TOPIC = "/gazebo/objects_truth"
def objects_source(environ: Mapping[str, str] = os.environ) -> str     # OBJECTS_SOURCE, default "logical"; else ValueError
def objects_topics(source: str) -> tuple[str, ...]                     # logical -> (OBJECTS_TOPIC, TRUTH_TOPIC);
                                                                       # detector -> (TRUTH_TOPIC,); else ValueError
def objects_publisher_note(source: str, count: int) -> str | None      # None for count == 1; a line naming OBJECTS_SOURCE
                                                                       # for count >= 2; for detector and 0, a line naming --profile detector
def layout_name(environ: Mapping[str, str] = os.environ) -> str        # GO2_LAYOUT, default "household"
def missing_models(lay: Layout, resource_path: str) -> list[str]       # sorted folders found in no GZ_SIM_RESOURCE_PATH entry
def spawn_fields(e: Entry) -> tuple[str, float, float, float, float]   # (sdf, x, y, z, yaw_rad); primitives: object_sdf and
                                                                       # OBJECT_SPAWN_Z; model://: <include> uri/name/static, z = e.z
def box_sdf(name: str, box: Box, cardboard: bool = False) -> str       # cardboard visual (ros_gz_sim_demos cardboard_box
                                                                       # texture) only when True; collision unchanged
def logical_rows(models: Iterable[tuple[str, float, float]], lay: Layout) -> list[tuple[str, str, float, float]]
    # (name, origin x, origin y) -> (id, label, centre x, centre y) for layout ids only (C3: origin + world offset)
class ObjectMemory:
    def __init__(self, sizes: Mapping[str, tuple[float, float]] | None = None) -> None
        # sizes=None: the measured memory (first size default_size(label), measured False, then True);
        # sizes given: the truth memory (fixed layout sizes, measured None)
    def update(...), set_size(...)        # unchanged semantics; objects() in _id_key order
    def message(self, stamp: float, source: str = "logical", layout: str | None = None) -> dict   # + top-level "layout"
```
- `gazebo_bridge.py`: read `objects_source()` and `layout_name()` once; load the layout and call `missing_models` before spawning (log one line naming the folders and `GZ_SIM_RESOURCE_PATH`, then exit non-zero; an unknown layout logs the `LayoutError`); spawn every entry with `spawn_fields`; publish truth always and `/go2/objects` only in logical mode (`objects_topics`); in detector mode skip `_measure_sizes` and give button boxes the cardboard visual; size with `footprint_size(..., radius=measure_radius(label))` around the footprint centre; every 10 s log `objects_publisher_note(source, count_publishers(OBJECTS_TOPIC))` as an error when not None.
- `compose.yaml`, service `gazebo`: `image: ${GO2_GAZEBO_IMAGE:-go2-voice-ltl-gazebo:patch-0005-fuel1}`; `build: {context: ./docker, dockerfile: Dockerfile.gazebo-fuel}` (the controller image `patch-0005` is built with `Dockerfile.gazebo` first); `environment: OBJECTS_SOURCE: "${OBJECTS_SOURCE:-logical}"`, `GO2_LAYOUT: "${GO2_LAYOUT:-household}"`; `ports: "127.0.0.1:${GO2_GAZEBO_PORT:-6080}:6080"`. The comments about the older images stay, with the lines that pin (`Never run`, `this tree cannot rebuild it`, the two `GO2_GAZEBO_IMAGE=… up` examples). Two comment lines change, because `build:` is now the thin image (review M13):
  - the header line `# patches 0001-0005 (docker/patches). …` becomes `# go2-voice-ltl-gazebo:patch-0005 (patches 0001-0005, docker/patches) is built first with docker/Dockerfile.gazebo (docs/simulation.md); build here makes the thin image patch-0005-fuel1 on top of it (docker/Dockerfile.gazebo-fuel: the pinned Fuel models). The older images are kept on this Mac only:`;
  - the warning's last sentence `compose would build 0001-0005 from this tree and put that image under the given tag.` becomes `compose would build the thin Fuel image (Dockerfile.gazebo-fuel, on top of patch-0005) and put it under the given tag.`
  `test_the_gazebo_image_carries_the_controller_patches` keeps its pins and its docstring says the same.

- [ ] **Step 1: Write the failing tests** in `tests/test_gazebo_objects.py`:

```python
def test_objects_topics_and_source():
    assert objects_topics("logical") == ("/go2/objects", "/gazebo/objects_truth")
    assert objects_topics("detector") == ("/gazebo/objects_truth",)
    for bad in ("Logical", "", "detecter"):
        with pytest.raises(ValueError): objects_topics(bad)
    assert objects_source({}) == "logical" and objects_source({"OBJECTS_SOURCE": "detector"}) == "detector"

def test_publisher_note_names_the_missing_detector():    # Review Focus 3
    assert objects_publisher_note("logical", 1) is None and objects_publisher_note("detector", 1) is None
    assert "--profile detector" in objects_publisher_note("detector", 0)
    assert "OBJECTS_SOURCE" in objects_publisher_note("logical", 2)

def test_missing_model_folder_stops_the_bridge_before_spawning(tmp_path):   # Review Focus 5
    (tmp_path / "Sofa").mkdir()
    assert missing_models(household_like_layout(), str(tmp_path)) == ["LampAndStand", "Suitcase1", "Table", "TrashBin"]
    assert missing_models(layout.load("default"), "") == []

def test_logical_rows_use_the_footprint_centre():        # C3
    rows = logical_rows([("box_1", 2.5, 1.5), ("chair_1", 2.5, -1.5), ("sim_box_1", 1.0, 0.0), ("ground_plane", 0, 0)],
                        layout.load("default"))
    assert rows == [("box_1", "box", 2.5, 1.5), ("chair_1", "chair", 2.5, -1.5)]   # today's positions
    # an entry with offset (0.19, 0) at yaw 90 is published 0.19 m along +y of its origin

def test_truth_rows_come_from_the_layout():
    # sizes from the layout (sofa (1.044, 1.8) world axes), visible/seen from the camera, no button boxes,
    # message source "logical" and "layout": "<name>"; objects.parse accepts it

def test_memory_starts_at_the_vocabulary_square_and_marks_measured():
    # chair first size (0.45, 0.45) measured False; after set_size measured True; larger per axis kept (today)

def test_measuring_radius_follows_the_kind():
    # a 1.8 x 1.04 sofa point set measured with measure_radius("sofa") gives (1.8, 1.04) +- 0.05 (radius 0.6 gave <= 1.2)

def test_spawn_fields_for_primitives_and_fuel_models():
    # primitive box: (object_sdf("box_1", "box"), 2.5, 1.5, 0.25, 0.0); model://Sofa: sdf has <uri>model://Sofa</uri>,
    # <name>sofa_1</name>, <static>true</static>; yaw 90 -> yaw_rad pi/2

def test_box_sdf_cardboard_only_in_detector_mode():
    # collision elements equal; the visual references cardboard_box only with cardboard=True

def test_compose_gazebo_service():
    # the image, build, OBJECTS_SOURCE, GO2_LAYOUT and port lines above
```
  Update the pins this moves in the same file: `test_memory_remembers_flags_visibility_and_keeps_the_largest_size` (first size and `measured`), `test_objects_stand_on_the_floor_with_the_layout_footprint` (through `spawn_fields`), `test_the_gazebo_image_carries_the_controller_patches` (the image line `${GO2_GAZEBO_IMAGE:-go2-voice-ltl-gazebo:patch-0005-fuel1}`), and keep `test_the_readme_echo_shows_the_whole_objects_message` passing.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_gazebo_objects.py tests/test_bridge_core.py`. Expected: FAIL.
- [ ] **Step 3: Implement** the Interfaces (the node code in `gazebo_bridge.py` has no unit tests; keep its logic in the pure functions above).
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add gazebo/bridge_core.py gazebo/gazebo_bridge.py compose.yaml tests/test_gazebo_objects.py tests/test_bridge_core.py` and `git commit -m "feat: bridge spawns the layout, publishes the truth and switches the object source"`.

### Task 2.4: objects_check — target runs, resolve, accuracy, RTF (stage 2a, branch `objects2/objects-check`)

**Files:**
- Modify: `gazebo/tools/objects_check.py`, `tests/test_objects_check.py`

**Interfaces:**
- Consumes: `layout.load`, `objects.edge_distance`, `objects.footprint_angle`, `objects.winding_number`, `objects.parse`, `objects.parse_source`.
- Produces:

```python
THRESHOLDS["target_near"] = {"max_edge": 1.5, "max_angle_deg": 45.0, "min_clearance": 0.30}
THRESHOLDS["target_lap"] = {"winding": (0.9, 1.5), "min_edge": 0.6}
THRESHOLDS["accuracy"] = {"max_pos_err_target": 0.15, "max_pos_err": 0.35, "median_pos_err": 0.25, "max_size_err": 0.4,
                          "min_label_acc": 0.8, "min_recall": 0.6, "max_false_pos": 1}
def first_action_required(samples: list[Sample]) -> str | None    # "立つ" when the first sample's z < STAND_Z, else None (C1)
def summarize(samples, expect, exit_code, log, layout=DEFAULT_LAYOUT, target: Obj | None = None,
              others: Sequence[Obj] = (), source: str | None = None, rtf: dict | None = None) -> dict
    # no target: today's §2 keys; target: final_edge, final_footprint_angle_deg, min_clearance (every footprint), final_edge
    # reported, or winding/min_edge for laps; plus the §2 keys when the target is chair_1 (near) or box_1 (lap); the box's
    # centre line (1.5 m / 45°) only when target is box_1, expect near and source == "logical" (O16)
    # exit_code None and log None (watch only): the exit-code and first-action checks are skipped
def last_source(payloads: Sequence[object]) -> str | None        # objects.parse_source of the last payload; None if none
def settled(samples: list[Sample], moved_m: float = 0.3, quiet_s: float = 10.0) -> bool
    # True once the body has moved >= moved_m from the first sample and the last quiet_s (sim) of samples stay
    # within 0.05 m and 3 deg (watch's early end)
def pair_rows(det: Sequence[Obj], truth: Sequence[Obj]) -> list[tuple[Obj, Obj]]   # greedy by centre distance,
                                                                                   # label-agnostic, within max(0.75, truth r)
def covers(det: Obj, truth: Obj, filled: bool, cam_xy: tuple[float, float]) -> tuple[bool, float]   # C2, below
def accuracy(det_rows, truth_rows, lay: Layout, button_boxes, robot_xy, targets, thresholds=THRESHOLDS["accuracy"],
             filled: frozenset[str] = frozenset()) -> dict                     # spec §11.11; "passed", "failures"
def resolve(det_rows, truth_rows, target: str) -> str | None                  # the detector id paired with the truth id (M10)
def rtf_stats(clock: list[tuple[float, float]], samples: list[Sample]) -> dict   # median of 10 s windows, walking, still
```
- CLI: `run [--expect near|lap] [--target <truth id>] [--layout <name>] <formula>` (the layout defaults to `GO2_LAYOUT`, else `household`; `run` records `/clock` and adds `rtf_stats`; it also subscribes to `/go2/objects` and passes `last_source(<the messages it got>)` as `source`, so `--target box_1` in logical mode gets the box's centre line); `watch --expect near|lap --target <truth id> [--layout <name>] [--seconds 300]` (records odometry and `/clock` like `run` but starts no child: the app drives the Go2, as in the voice step of H6; it ends at `--seconds` or when `settled(samples)`, then prints `summarize(samples, expect, None, None, target=…, …)` with `"child": null`; exit 0 pass, 1 fail, 2 no odometry); `accuracy [--seconds 15] [--range 5.0] [--target <id> ...]` (waits until the Go2 is still and `/go2/detector_status` reports `still_inferences` ≥ 3 or the seconds run out; takes `filled` from the same status and the button boxes from `/sim/world`); `resolve --target <truth id> [--timeout 30]` (waits until a `/go2/objects` row pairs with the target and has `measured: true`; prints `{"target", "id", "measured", "listed_after_still_s"}`; exit 0 found, 1 timeout, 2 no data); `rtf` and `objects` as today. `/go2/objects` without `source: "detector"` → `accuracy` exit 2; a truth message whose `layout` differs from `--layout` → exit 2.
- `covers` (C2): not filled → the detector AABB contains the truth AABB (1 mm tolerance), fill shortfall 0. Filled → the depth axis is the axis on which |truth centre − cam_xy| is larger; the detector AABB must contain the truth AABB on both sides of the other axis and on the near side of the depth axis; the shortfall is max(0, truth far edge − detector far edge) along the depth axis, reported as `fill_short_m`, never a failure.

- [ ] **Step 1: Write the failing tests** in `tests/test_objects_check.py`:
  - `test_target_near_lines_on_synthetic_paths`: a sofa 1.044 × 1.8; a path ending 1.5 m from its edge facing a footprint point → pass; 1.51 m → fail `final_edge`; facing 46° off the footprint → fail; a path passing 0.29 m from another footprint → fail `min_clearance`.
  - `test_target_lap_lines`: winding 1.0 and least edge 0.6 → pass; 0.59 → fail.
  - `test_runs_starting_standing_need_no_stand` (C1): first sample z 0.30 → a first action 「右を向く」 passes; z 0.10 → 「立つ」 required.
  - `test_box_near_centre_line_only_in_logical_mode` (O16).
  - `test_section2_keys_unchanged_without_target` (today's tests keep passing).
  - `test_accuracy_*`: perfect rows pass; a 0.4 m error fails `max_pos_err`; a target at 0.2 m fails the target line; a wrong label lowers label accuracy and an `accept` alternative counts as right; a missing object lowers recall; an extra row is a false positive; a row on a button box is `button_boxes_seen`; a non-detectable truth is out of recall but still matches; an undersized footprint gives `covers` False; a filled sofa whose far edge is 0.14 m short has `covers` True and `fill_short_m` 0.14 (C2); a wrong `source` → exit 2.
  - `test_resolve_pairs_the_detector_row`; `test_layout_mismatch_is_no_data`; `test_rtf_stats_split_walking_and_still`.
  - `test_last_source` (`[{"source": "detector", …}, {"source": "logical", …}]` → "logical"; `[]` → None; a last message without `source` → None, so `summarize` gives no centre line).
  - `test_watch_skips_the_child_checks` (`summarize(..., exit_code=None, log=None)` on a standing-start near path has no `exit_code` or `first_action` failure, and still fails `final_edge` at 1.51 m); `test_settled` (still from the start → False; moved 1 m then 10 s still → True; moved then 9 s still → False).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_objects_check.py`. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add gazebo/tools/objects_check.py tests/test_objects_check.py` and `git commit -m "feat: objects_check target runs, resolve, accuracy and RTF per run"`.

### Task 2.5: Household layout and its grid checks (stage 2a, branch `objects2/household`)

**Files:**
- Create: `gazebo/layouts/household.json`, `tests/test_household.py`, `tests/test_sim_household.py`

**Interfaces:**
- Consumes: `layout.load`, `grid.grid_shape`, `grid.grid_world`, `sim_core.in_camera_view`, `session.Session`, `DirectSimLink`, `FakeLLM`.
- Produces: `gazebo/layouts/household.json` with spec §12.5's 7 entries, all `yaw_deg` 0: `box_1` (`primitive:box`, (2.5, 1.5), 0.5 × 0.5, 0.5, detectable false, [box]); `chair_1` (`primitive:chair`, (2.5, −1.5), 0.45 × 0.45, 0.9, false, [chair]); `trash_can_1` (`model://TrashBin`, (−1.5, −1.5), 0.266 × 0.375, 0.446, false until H3, [trash_can]); `suitcase_1` (`model://Suitcase1`, (−1.5, 1.5), 0.6 × 0.4, 0.8, false, [suitcase]; a bound until H1); `sofa_1` (`model://Sofa`, (−4.4, 0), offset (0.02, 0), 1.044 × 1.8, 0.943, true, [sofa]); `lamp_1` (`model://LampAndStand`, (0.8, −4.4), 0.622 × 0.525, 1.021, true, [lamp]); `table_1` (`model://Table`, (−2.4, 4.4), offset (0.002, 0), 1.5 × 0.8, 1.015, false, [table, desk]).

- [ ] **Step 1: Write the failing tests** in `tests/test_household.py` (spec §12.1–§12.4; "test pose" B = facing 180°, L = facing +90°, R = facing −90°, standing at (0, 0); "moves" = the shortest controllable path in the PLANT to a state with the target proposition; no synthesis):
  - `test_household_layout_loads` (7 ids, positions, world footprints, `detectable` and `accept` as above).
  - `test_no_new_object_in_the_spawn_view`: none of the 5 new centres is in `in_camera_view((0, 0, 0), …)`.
  - `test_acceptance_plants_are_unchanged_by_the_household_objects`: for `F around_box_1`, `F near_chair_1`, `F(near_chair_1 & X F hello)` and `F near_box_1`, starts (0, 0, 0), (0.06, −0.04, 3°), (−0.05, 0.05, −3°), (0.1, 0.1, 5°), (−0.1, −0.1, −5°), with the design sizes and with box 0.53 × 0.52 / chair 0.5 × 0.5: the GridShape and the PLANT (states, transitions, event labels, forbidden events) are equal with and without the 5 new objects.
  - `test_every_test_formula_has_a_plan`: from the test poses, area / moves / keys equal spec §12.4's "new" column — `F near_trash_can_1` B 64 / 4 / 12, `F around_trash_can_1` B 81 / 23, `F near_suitcase_1` B 64 / 4 / 8, `F around_suitcase_1` B 121 / 39, `F near_sofa_1` B 165 / 6 / 18, `F around_sofa_1` B 221 / 50, `F near_lamp_1` R 112 / 7 / 15, `F around_lamp_1` R 160 / 36, `F near_table_1` L 165 / 10 / 22, `F around_table_1` L 221 / 44 — with PLANT states 1 416, 6 912, 1 320, 8 736, 3 144, 14 448, 2 328, 10 608, 3 096, 13 488; and the largest area over the 4 jittered starts (±0.1 m, ±5°) is 72, 100, 72, 121, 165, 238, 120, 160, 168, 221, each with a plan.
  - `test_vocabulary_sizes_also_plan` (each test formula has a plan with the vocabulary sizes laid along x: sofa near 144 cells / 17 keys, table near 165 / 18, the lamp small with 5 keys).
  - `test_inflated_sizes_lose_the_sofa_lap` (sofa 1.8 × 1.8, table 1.5 × 1.5, suitcase 0.7 × 0.5, lamp 0.7 × 0.7: `F around_sofa_1` has no plan at 255 cells; every near still plans) — why the procedure waits for `measured`.
  - `test_nearest_footprint_gaps` (trash can 2.28, suitcase 2.12, sofa 2.12, lamp 2.68, table 2.30 m ± 0.01).
  In `tests/test_sim_household.py` (spec §13.1): with the fake Go2 on `household` and the §12.4 procedure (turn first, then the target formula through `Session` with a FakeLLM), `F near_<id>` and `F around_<id>` for the 5 new ids meet the acceptance lines (Global Constraints); `@pytest.mark.slow test_sofa_synthesis_is_fast` (sofa near ≤ 1 s, sofa lap ≤ 3 s).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_household.py tests/test_sim_household.py`. Expected: FAIL (no `household` layout).
- [ ] **Step 3: Write `gazebo/layouts/household.json`.**
- [ ] **Step 4: Run.** Same command. Expected: PASS. A table value that differs is reported, not edited (Global Constraints).
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add gazebo/layouts/household.json tests/test_household.py tests/test_sim_household.py` and `git commit -m "feat: household layout with five Fuel models, checked against the acceptance grids"`.

### Task 2.6: Fuel image (stage 2a, branch `objects2/fuel-image`)

**Files:**
- Create: `docker/Dockerfile.gazebo-fuel`, `docker/fetch_fuel_models.py`, `gazebo/tools/model_extent.py`, `tests/test_fuel_image.py`, `tests/fixtures/models/` (tiny synthetic models)

**Interfaces:**
- Consumes: `docker/fuel_models.json` (Task 1.6).
- Produces:
  - `docker/Dockerfile.gazebo-fuel`: `ARG BASE=go2-voice-ltl-gazebo:patch-0005`, `FROM ${BASE}`; `COPY fuel_models.json fetch_fuel_models.py /opt/go2_fuel/`; `RUN python3 /opt/go2_fuel/fetch_fuel_models.py /opt/go2_fuel/fuel_models.json /opt/go2_models`; `ENV GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:+${GZ_SIM_RESOURCE_PATH}:}/opt/go2_models`; a header comment naming the tag `go2-voice-ltl-gazebo:patch-0005-fuel1` and the build command.
  - `docker/fetch_fuel_models.py` (stdlib): `verify(data: bytes, sha256: str | None) -> None` (raises on a mismatch or a null pin); `extract(zip_path, dest: Path, folder: str) -> Path` (refuses absolute paths and `..`; strips one top-level directory; requires `model.config` and `model.sdf` at `dest/folder`); `main(list_path, dest) -> int` (downloads `{base}/{folder}/{version}/{folder}.zip`, then verify and extract; any failure exits non-zero, so the build stops).
  - `gazebo/tools/model_extent.py` (stdlib): `extent(model_dir: Path) -> dict` with `{"size": [sx, sy], "offset": [dx, dy], "height": h, "z_min": z}` in model axes from the collision geometry of `model.sdf` (boxes, cylinders, and OBJ/DAE mesh vertices with `<scale>` and link/collision `<pose>`); unknown mesh formats raise. CLI prints the JSON.
- [ ] **Step 1: Write the failing tests** in `tests/test_fuel_image.py`: `test_dockerfile_pins_the_base_and_reads_the_list` (the `ARG BASE` default, the COPY and RUN lines, the ENV line); `test_verify_refuses_a_mismatch_and_a_null_pin`; `test_extract_refuses_zip_slip_and_strips_one_folder` (synthetic zips in `tmp_path`); `test_extract_requires_model_files`; `test_model_extent_of_synthetic_models` (a box collision 0.5 × 0.4 × 0.8 at pose (0.1, 0, 0.4); an OBJ cube scaled 2; a DAE triangle set) with exact sizes and offsets.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_fuel_image.py`. Expected: FAIL.
- [ ] **Step 3: Implement.** No download and no build in this task (H1 does both).
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add docker/Dockerfile.gazebo-fuel docker/fetch_fuel_models.py gazebo/tools/model_extent.py tests/test_fuel_image.py tests/fixtures/models` and `git commit -m "feat: thin Gazebo image with pinned Fuel models (not built yet)"`.

### Task 2.7: Detector core and oracle (stage 2b, branch `objects2/det-core`)

**Files:**
- Create: `detector/core.py`, `detector/oracle.py`, `tests/test_detector_core.py`, `tests/test_detector_oracle.py`
- Modify: `tests/test_detector_protocol.py` (add `OracleDetector` to the parametrisation; Task 2.2's file is free in stage 2b)

**Interfaces:**
- Consumes: Task 2.1 (`CameraModel`, `camera_pose`, `world_to_camera`, `observe`, `Sighting`, `footprint`, `vacancy_counts`, `is_vacant`), Task 2.2 (`Detector`, `Thresholds`, `Tracker`, `Observation`, `ScriptedDetector`), `bridge_core.StillGate`, `bridge_core.lidar_points_world`, `objects.format_message`, `vocabulary.VOCABULARY`.
- Produces:

```python
# detector/core.py
POSE_BUFFER_S = 2.0; PAIR_WAIT_S = 0.1; PAIR_MAX_SHIFT_M = 0.02; PAIR_MAX_SHIFT_DEG = 1.0
DET_MIN_GAP_S = 1.0; DET_SETTLE_FRAMES = 3; DET_IDLE_PERIOD_S = 3.0; DET_IDLE_MAX_S = 12.0; SLOW_LATENCY_S = 1.5
CAMERA_TIMEOUT_S = 10.0; ERROR_LIMIT = 3; ERROR_BACKOFF_S = 30.0; VACANCY_MAX_HZ = 2.0; TIME_BACK_S = 1.0
@dataclass(frozen=True)
class View: base_pos: tuple[float, float, float]; base_quat: tuple[float, float, float, float]; camera: CameraModel; stamp: float
class PoseBuffer:
    def add(self, t: float, pos, quat) -> None
    def at(self, t: float) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None   # interpolated
    def latest(self) -> float | None
    def clear(self) -> None
@dataclass(frozen=True)
class Job: image: np.ndarray; image_t: float; cloud: np.ndarray; cloud_t: float; still_since: float
class DetectorCore:
    def __init__(self, detector: Detector, vocab: Vocabulary = vocabulary.VOCABULARY,
                 wall: Callable[[], float] = time.monotonic, clock: Callable[[], float] = time.time) -> None
    def mark_loaded(self) -> None                         # status leaves "loading" once loaded and sensors arrived
    def on_odom(self, t: float, pos, quat, linear, angular) -> None
    def on_image(self, t: float, image_rgb: np.ndarray) -> None
    def on_cloud(self, t: float, points_lidar: np.ndarray) -> None
    def on_camera_info(self, k: Sequence[float], width: int, height: int) -> None
    def next_job(self) -> Job | None                       # pairing and rate limits (spec §11.5)
    def run(self, job: Job) -> None                        # prepare(view) if present, detect, observe, track; errors counted
    def message(self) -> dict                              # format_message(rows, clock(), source="detector")
    def health(self) -> dict                               # spec §11.9 keys + "filled": [ids] + "shapeless": n
                                                           # + "notes": the tracker's notes (C6, review M14)
    def reset(self, reason: str) -> None
# detector/oracle.py (measurement only, never the default)
class OracleDetector:                                      # name "oracle", thresholds YOLOE_THRESHOLDS
    def set_vocabulary(self, prompts, ignore) -> None
    def set_truth(self, rows: Sequence[Obj], heights: Mapping[str, float] | None = None) -> None
        # heights by id; None -> vocabulary.height_of(row.label) for every row (spec §11.10: truth footprint x
        # vocabulary height). Every caller (node, detector_check) passes None, so the oracle box uses the
        # vocabulary height; only detector_check.preview projects with the layout height (Task 2.12)
    def prepare(self, view: View) -> None
    def detect(self, image_rgb: np.ndarray) -> list[Detection]   # truth footprint x height boxes, confidence 1.0, no mask
    def close(self) -> None
```
- `run` turns each detection into an `Observation` through `observe(...)` and the 1:1 mapping of Task 2.1; a `Sighting` with `footprint` None makes no `Observation` (there is nothing to associate it with) and adds 1 to `health()["shapeless"]`. `health()["notes"]` is a copy of `tracker.stats()["notes"]`, so a label whose ids ran out (C6) shows on `/go2/detector_status` without reading logs.
- Timers and rate limits use the wall clock (`wall`, steady); the gate, the pose lookups and every "0.5 s apart" use sensor stamps (M3). Any odometry, image or scan stamp more than `TIME_BACK_S` earlier than the latest seen calls `reset("sim time went backwards")`: gate, pose buffer, kept frames and tracks are cleared and the numbering restarts (M4).

- [ ] **Step 1: Write the failing tests** in `tests/test_detector_core.py` (`design-detector.md` §16 "Core", with `ScriptedDetector`): no job before the gate opens; an image stamped before `still_since + 0.3` is refused even if still now; image and scan from different still intervals never pair; a job waits for odometry to pass both stamps (≤ 0.1 s); poses 0.03 m apart drop the job; one job in flight; gap max(1.0, p50); idle period after 3 frames; motion resets it; a result applied after the Go2 moved uses the stamp poses; start-up gives empty lists with status `loading`, then `ok` after the first odometry, scan and image; labels outside the vocabulary and ignored prompts never become tracks; no camera for 10 s → `no_camera`, tracks kept, vacancy still runs; no scan in the still interval → the image is dropped and the status says `no_cloud`; no odometry → no job and `no_odometry`; an image whose size differs from the CameraInfo or the constants → refused once with `bad_camera`; 3 exceptions → 30 s backoff then recovery; a slow p50 doubles the idle period; `health()` has every spec §11.9 key plus `filled`, `shapeless` and `notes`; `test_used_up_ids_show_in_the_status` (a tracker with `box_99` issued: the next confirmed box gives one entry in `health()["notes"]`); `test_shapeless_sighting_makes_no_observation`; and

```python
def test_sim_time_going_backwards_resets_gate_buffer_and_tracks():   # Review Focus 2
    core = confirmed_chair_core()                         # chair_1 confirmed at sim t ~ 120
    feed_still(core, t0=0.5)                              # Gazebo restarted: stamps from 0.5 s
    assert core.health()["tracks_confirmed"] == 0 and core.next_job() is not None   # the gate works again
    confirm_chair(core, t0=1.0)
    assert [r["id"] for r in core.message()["objects"]] == ["chair_1"]              # numbering restarted
```
  In `tests/test_detector_oracle.py`: the oracle projects a truth box (footprint × `vocabulary.height_of(label)` when no heights are given) into the image box the camera model gives (±1 px), returns confidence 1.0 and no mask; through `DetectorCore` on a synthetic still scene (truth sofa and lamp, simulated LiDAR) it publishes both with position error ≤ 0.15 m and `covers` on the observed extent.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_detector_core.py tests/test_detector_oracle.py tests/test_detector_protocol.py`. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add detector/core.py detector/oracle.py tests/test_detector_core.py tests/test_detector_oracle.py tests/test_detector_protocol.py` and `git commit -m "feat: detector core (still gating, pairing, rate limits, health) and the oracle"`.

### Task 2.8: Detector image and compose service (stage 2b, branch `objects2/det-image`)

**Files:**
- Create: `docker/Dockerfile.detector`, `tests/test_detector_image.py`
- Modify: `compose.yaml`, `pytest.ini`

**Interfaces:**
- Produces:
  - `docker/Dockerfile.detector`: `design-detector.md` §3.2 as written (header comments naming the tag and AGPL-3.0; `FROM ros:jazzy-ros-base-noble`; the three ARG version pins and the CLIP commit; `ASSETS=https://github.com/ultralytics/assets/releases/download/v8.4.0`; both sha256 ARGs; apt `python3-venv libgl1 libglib2.0-0t64 ros-jazzy-rmw-cyclonedds-cpp curl ca-certificates`; the venv; torch from `https://download.pytorch.org/whl/cpu`; ultralytics, clip and `pytest==8.4.2`; `curl` + `sha256sum -c` for `yoloe-11s-seg.pt` and `mobileclip_blt.ts`; `ENV YOLO_CONFIG_DIR=/opt/yoloe/config YOLO_AUTOINSTALL=False YOLO_OFFLINE=True RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONPATH=/work:/work/nl_control OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`; `yolo settings weights_dir=/opt/yoloe sync=False`; the ENTRYPOINT sourcing Jazzy; `CMD ["python3", "-m", "detector.node"]`).
  - `compose.yaml`: a `detector` service **inserted before `gazebo:`** as `design-detector.md` §3.6 (profile `detector`, build `./docker` + `Dockerfile.detector`, image `go2-voice-ltl-detector:yoloe-11s`, `init: true`, volumes `./:/work`, `/dev/null:/work/env.txt:ro`, `detector-cache:/cache`, environment `OBJECTS_SOURCE: "${OBJECTS_SOURCE:-logical}"` and `DETECTOR: "${DETECTOR:-yoloe}"`, `cpus: "2.0"`, `restart: "on-failure:3"`, command `python3 -m detector.node`, no `env_file`, the default ROS domain like `gazebo`, M9); `detector-cache:` under `volumes:`.
  - `pytest.ini`: marker `detector: needs the detector image (ultralytics); skipped elsewhere`.
- [ ] **Step 1: Write the failing tests** in `tests/test_detector_image.py`: `test_dockerfile_pins` (every version, the commit, both URLs and both sha256 from spec §11.1, `YOLO_OFFLINE=True`, `YOLO_AUTOINSTALL=False`, no `-pf` and no `11m`); `test_compose_detector_service` (the shape above, no `env_file`, `env.txt` masked); `test_detector_marker_is_registered`; `test_licence_isolation` (walk every `*.py` in the tree except `.git`: only `detector/yoloe*.py` has a line matching `re.compile(r"^\s*(?:import|from)\s+(?:torch|ultralytics)\b", re.M)`, indented imports included; the pattern is anchored to import statements so the test's own pattern string does not match itself). Every test file obeys it too: the `detector`-marked tests (Task 2.11) skip with `importlib.util.find_spec("ultralytics") is None` and read torch state only through adapter methods, never importing torch or ultralytics themselves (spec §4).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_detector_image.py`. Expected: FAIL.
- [ ] **Step 3: Implement.** No build here (H2 builds).
- [ ] **Step 4: Run.** Run: `T tests/test_detector_image.py tests/test_gazebo_objects.py`. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add docker/Dockerfile.detector compose.yaml pytest.ini tests/test_detector_image.py` and `git commit -m "feat: detector image and compose service (profile detector, not built yet)"`.

### Task 2.9: Gazebo camera — xacro and launch (stage 2b, branch `objects2/gz-camera`)

**Files:**
- Create: `gazebo/launch_config.py`, `tests/test_gazebo_camera.py`
- Modify: `gazebo/go2_sensors.urdf.xacro`, `gazebo/launch/go2_gazebo.launch.py`

**Interfaces:**
- Consumes (Task 2.3): `bridge_core.objects_source`.
- Produces:

```python
# gazebo/launch_config.py (no ROS imports)
GZ_BRIDGE_ARGS: tuple[str, ...]        # today's six arguments, byte for byte
CAMERA_BRIDGE_ARGS = ("/gazebo/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                      "/gazebo/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")
def xacro_mappings(source: str) -> dict[str, str]          # logical -> {}, detector -> {"camera": "true"}
def camera_bridge_args(source: str) -> tuple[str, ...] | None   # detector only
```
- The xacro gains `<xacro:arg name="camera" default="false"/>` and, inside `<xacro:if value="$(arg camera)">` (comments inside the block, so the logical output is unchanged), the `front_camera` sensor of `design-detector.md` §4.1 on `logical_camera` with 640, 480, `R8G8B8`, 2.094, clip 0.1/10, `update_rate` 2, topic `gazebo/front_camera/image`, `camera_info_topic` `gazebo/front_camera/camera_info`, `gz_frame_id` `front_camera`. The launch file adds the repo and `nl_control` to `sys.path`, reads `objects_source()` once (an invalid value stops the launch), processes the xacro with `xacro_mappings(source)`, and adds a second `parameter_bridge` node named `front_camera_bridge` (`use_sim_time`) only when `camera_bridge_args(source)` is not None.
- [ ] **Step 1: Write the failing tests** in `tests/test_gazebo_camera.py`: `test_xacro_has_no_camera_by_default` (the camera block is inside the `xacro:if`, the arg defaults to false); `test_camera_block_values` (each value above; the logical camera block unchanged); `test_main_bridge_arguments_are_unchanged` (`GZ_BRIDGE_ARGS` equals the six strings of `0f6e06a`'s launch file, read with `git show 0f6e06a:gazebo/launch/go2_gazebo.launch.py` and `ast`); `test_camera_bridge_only_in_detector_mode`; `test_logical_robot_description_is_unchanged` (`pytest.importorskip("xacro")`: today's xacro with no mappings equals `0f6e06a`'s output byte for byte).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_gazebo_camera.py`. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Run: `T tests/test_gazebo_camera.py tests/test_gazebo_objects.py tests/test_bridge_core.py`. Expected: PASS. Then check the robot description in the gazebo image (light, no Gazebo): in `docker run --rm -v <wt>:/wt -w /wt --entrypoint bash go2-voice-ltl-gazebo:patch-0005 -c 'source /opt/ros/jazzy/setup.bash && python3 …'`, compare `xacro.process_file(...).toxml()` of the current `gazebo/go2_sensors.urdf.xacro` with that of `0f6e06a`'s version (taken with `git show` on the host and passed in through a throwaway file that is never added), and count the camera sensors with `mappings={"camera": "true"}`. Expected: equal, and exactly 1 camera sensor.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add gazebo/go2_sensors.urdf.xacro gazebo/launch/go2_gazebo.launch.py gazebo/launch_config.py tests/test_gazebo_camera.py` and `git commit -m "feat: front camera and its bridge only in detector mode"`.

### Task 2.10: Detector node (stage 2c, branch `objects2/det-node`)

**Files:**
- Create: `detector/node.py`, `tests/test_detector_node.py`

**Interfaces:**
- Consumes: `DetectorCore`, `View`, `OracleDetector.set_truth`, `load_detector`, `bridge_core.objects_source`, `bridge_core.objects_publisher_note`, `objects.OBJECTS_TOPIC`, `bridge_core.TRUTH_TOPIC`, `vocabulary.VOCABULARY`; `detector.camera` constants.
- Produces (`detector/node.py`, rclpy; std_msgs, sensor_msgs, nav_msgs only):

```python
STATUS_TOPIC = "/go2/detector_status"; IMAGE_TOPIC = "/gazebo/front_camera/image"
CAMERA_INFO_TOPIC = "/gazebo/front_camera/camera_info"; CLOUD_TOPIC = "/utlidar/cloud"; ODOM_TOPIC = "/model/robot/odometry"
def startup_refusal(environ: Mapping[str, str]) -> str | None    # one line when OBJECTS_SOURCE != "detector", else None
                                                                  # main() prints it and returns 0, so compose's
                                                                  # restart: on-failure:3 does not restart it (review M9)
def image_from_msg(msg) -> np.ndarray                             # rgb8 H x W x 3 (np.frombuffer, no cv_bridge)
def points_from_cloud(msg) -> np.ndarray                          # N x 3 float64
def stamp_of(msg) -> float
class DetectorNode(Node):                                         # spec §11.1 start-up order
    def __init__(self, make_detector: Callable[[], Detector] = load_detector) -> None
def main() -> int                                                 # python3 -m detector.node
```
- Exit codes: `startup_refusal` not None → print the line, return 0 before `rclpy.init` (a refusal is not a failure, so `restart: on-failure:3` leaves it stopped); a vocabulary, model or weight load failure → log it, return 1 (compose restarts it up to 3 times, spec §11.9).
- Start-up: publishers and the 2 Hz (objects) and 1 Hz (status) steady wall-clock timers first, publishing `{"frame": "odom", "source": "detector", "objects": []}` and `{"ok": false, "reason": "loading"}` at once; then the vocabulary (exit non-zero on error), the detector (`set_vocabulary` with every kind's `detect` and the ignore list), `mark_loaded()`; then the subscriptions (`qos_profile_sensor_data`, image depth 1); a worker thread runs `next_job`/`run`; every 10 s an error from `objects_publisher_note("detector", count)` when not None; with `DETECTOR=oracle` it subscribes to the truth topic and feeds `set_truth(rows)` (no heights: the oracle uses the vocabulary height, review M8). It never subscribes to `/sim/*` (D9).
- [ ] **Step 1: Write the failing tests** in `tests/test_detector_node.py`: `test_startup_refusal` (logical or unset → a line naming `OBJECTS_SOURCE`; detector → None); `test_main_returns_0_on_refusal` (`OBJECTS_SOURCE=logical`: `main()` returns 0, prints the line and never calls `rclpy.init`); `test_main_returns_1_when_the_detector_fails_to_load` (`OBJECTS_SOURCE=detector DETECTOR=nope`: `load_detector` raises, `main()` logs it and returns 1; rclpy, skipped without it, the task's `ROS_DOMAIN_ID`); `test_image_and_cloud_conversion` (synthetic messages round-trip); `test_camera_constants_match_the_xacro` (`CAMERA_W`, `CAMERA_H`, `CAMERA_RATE_HZ`, `CAMERA_HFOV` and the clip equal the xacro's camera block); `test_node_publishes_an_empty_detector_list_at_once` (rclpy, skipped without it: with a slow scripted detector factory, the first `/go2/objects` message within 1 s has `source: "detector"` and no objects, and the status says `loading`; uses the task's `ROS_DOMAIN_ID`).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_detector_node.py`. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Same command. Expected: PASS (the rclpy test runs in the app image).
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add detector/node.py tests/test_detector_node.py` and `git commit -m "feat: detector node publishes from start-up and detects only while still"`.

### Task 2.11: YOLOE adapter (stage 2c, branch `objects2/yoloe`)

**Files:**
- Create: `detector/yoloe.py`, `tests/test_detector_yoloe.py`
- Modify: `tests/test_detector_protocol.py` (add `YoloeDetector`, marked `detector`)

**Interfaces:**
- Consumes: `Detection`, `Thresholds`, `YOLOE_THRESHOLDS`.
- Produces (`detector/yoloe.py`; the only module importing `ultralytics` and `torch`, both inside functions so the module imports without them):

```python
MODEL_PATH = "/opt/yoloe/yoloe-11s-seg.pt"; CACHE_DIR = "/cache"; DET_TORCH_THREADS = 1
PREDICT = {"conf": 0.10, "iou": 0.7, "agnostic_nms": True, "imgsz": 640}
BY_LABEL: dict[str, float] = {}                       # per-label confirm overrides, set by H3 only
def class_list(prompts: Mapping[str, Sequence[str]], ignore: Sequence[str]) -> tuple[list[str], list[str | None]]
    # (names = every prompt then every ignore prompt, label per index; None for ignore)
def embedding_key(model_sha256: str, names: Sequence[str]) -> str   # sha256 of json([model sha, names in order])
class YoloeDetector:                                   # name "yoloe-11s-seg"; thresholds = Thresholds(0.25, 0.35, 0.5, BY_LABEL)
    def __init__(self, model_path: str = MODEL_PATH, cache_dir: str = CACHE_DIR) -> None
    def set_vocabulary(self, prompts, ignore) -> None  # cache /cache/yoloe-pe-<key>.npz; first try ultralytics'
                                                       # save/load_prompt_embeddings, else get_text_pe + set_classes(names, pe)
    def detect(self, image_rgb) -> list[Detection]     # RGB -> BGR, PREDICT, masks to image size, ignore dropped after NMS
    def close(self) -> None
    def threads(self) -> dict[str, int]                # {"torch": torch.get_num_threads(), "interop": …, "cv2": cv2.getNumThreads()}
    embedding_source: str | None                       # "cache" or "encoder" after set_vocabulary (which path made the embeddings)
```
- The test files never import torch, ultralytics or cv2 (Task 2.8's `test_licence_isolation` scans them too). The `detector`-marked tests skip with `pytest.mark.skipif(importlib.util.find_spec("ultralytics") is None, reason=…)` and read thread counts through `threads()` and the cache path through `embedding_source`.
- Threads (spec §11.1): `torch.set_num_interop_threads(1)` before any parallel work, one warm-up predict on a black 640 × 480 frame, then `torch.set_num_threads(DET_TORCH_THREADS)`; `cv2.setNumThreads(1)`.
- [ ] **Step 1: Write the failing tests** in `tests/test_detector_yoloe.py`. Unmarked (app image): `test_class_list_puts_ignore_last_and_maps_labels`; `test_embedding_key_changes_with_the_list_and_order`; `test_module_imports_without_ultralytics`. Marked `detector` (skipped without ultralytics; run in H2): `test_prompts_of_one_label_merge`; `test_ignore_classes_are_dropped_and_still_suppress`; `test_agnostic_nms_is_on`; `test_threads_are_1_after_warm_up` (`threads() == {"torch": 1, "interop": 1, "cv2": 1}`); `test_embedding_cache_skips_the_encoder` (a second start with the same list has `embedding_source == "cache"` and does not load `mobileclip_blt.ts`; a changed list gives `"encoder"`).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_detector_yoloe.py`. Expected: FAIL on the unmarked tests; the marked ones are skipped.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Same command. Expected: PASS for the unmarked tests, `detector` tests skipped.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed (`test_licence_isolation` passes).
- [ ] **Step 6: Commit.** `git add detector/yoloe.py tests/test_detector_yoloe.py tests/test_detector_protocol.py` and `git commit -m "feat: YOLOE-11s text-prompt adapter behind the detector protocol"`.

### Task 2.12: Recording and replay tools (stage 2c, branch `objects2/det-tools`)

**Files:**
- Create: `gazebo/tools/record_frames.py`, `gazebo/tools/detector_check.py`, `gazebo/tools/png_rgb.py`, `tests/test_detector_check.py`, `tests/fixtures/detector/` (scans, poses and truth JSON of synthetic frames; **no images**, spec §11.11)

**Interfaces:**
- Consumes: `DetectorCore`, `OracleDetector`, `ScriptedDetector`, `load_detector`, `objects_check.pair_rows`, `objects_check.accuracy`, `layout.load`, `vocabulary.height_of`, `geometry` and `camera`.
- Produces:
  - `png_rgb.py` (stdlib `zlib`/`struct` + numpy; no PIL, no cv2): `write_png(path, rgb: np.ndarray) -> None` (H × W × 3 uint8, colour type 2, bit depth 8, filter 0 on every row) and `read_png(path) -> np.ndarray` (8-bit RGB or RGBA, not interlaced, filter types 0–4; anything else raises `ValueError`; RGBA drops alpha). `record_frames` writes with it and `detector_check` reads with it, so neither needs an image library in its container.
  - `record_frames.py` (runs inside the gazebo container: Jazzy rclpy, numpy 1.26, the `gz` CLI; **deviation from spec §11.11**, which puts it in the detector image: `--shift` needs `gz service`, which only the gazebo image has; the gazebo image has no OpenCV and no verified PIL, so the PNG goes through `png_rgb`): `python3 gazebo/tools/record_frames.py <out_dir> --view <name> [--shift dx,dy,dyaw_deg] [--layout household]` waits for an open still gate, saves one pair (`<view>.png`, `<view>_scan.npy` in the LiDAR frame, `<view>.json` with both stamps' poses, CameraInfo, the truth message, `/sim/world`, the status); `--shift` first moves every static layout model by the inverse of the view shift with `gz service -s /world/empty/set_pose` (the spike's way to emulate a viewpoint) and moves them back afterwards. rclpy and the message types are imported inside `main()` only. Pure helpers: `shifted_pose(x, y, yaw_deg, shift) -> tuple[float, float, float]`, `pair_record(...) -> dict`.
  - `detector_check.py` (detector image for yoloe, app image for oracle): `python3 gazebo/tools/detector_check.py <set_dir> [--detector yoloe|oracle|yoloe_pf] [--layout household]` replays each frame through `DetectorCore` and prints one JSON line per frame (the `accuracy` result) and one for the set, plus `confusion(frames) -> dict` (per prompt label and per truth id: hits with confidences) and `preview(frames, targets, confirm=0.35) -> dict` (spec §12.8 step 3: per target, a hit with an `accept` label ≥ 0.35 and IoU ≥ 0.5 with the truth box projected with the **layout height** (the layout is the truth here), in the pose view and in ≥ 2 of its 4 shifted views, and no other target's label on it; for the standing spawn view `spawn`, no confirmed label other than box and chair; a `spawn_lying` frame is replayed and printed but `preview` ignores it). With `--detector oracle` it calls `set_truth(rows)` without heights, so the oracle box uses the vocabulary height (review M8, as in the node).
- [ ] **Step 1: Write the failing tests** in `tests/test_detector_check.py`: `test_shifted_pose_inverts_the_view`; `test_pair_record_shape`; `test_png_round_trip` (`write_png` then `read_png` gives the same array; the file starts with the PNG signature and has one IHDR, IDAT and IEND); `test_read_png_undoes_every_filter` (a 3 × 4 image whose rows the test encodes with filters 1–4 by hand); `test_record_frames_imports_without_rclpy` (in a fresh `subprocess` with the test's `PYTHONPATH`, importing `gazebo.tools.record_frames` leaves `rclpy`, `cv2` and `PIL` out of `sys.modules`); `test_replay_with_the_oracle_passes_accuracy` (fixture frames; the images are made by the test with `write_png`, never committed); `test_confusion_counts_prompts_per_truth_object`; `test_preview_rule` (synthetic detections from `ScriptedDetector`: a target hit in the pose view and 2 of 4 shifts passes; 1 of 4 fails; another target's label on it fails; a new label in the `spawn` view fails; the same label in `spawn_lying` changes nothing).
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_detector_check.py`. Expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run.** Same command. Expected: PASS. Then the light check that `record_frames` starts in the gazebo image (no Gazebo, no simulation; `patch-0005` has the same Python as the `patch-0005-fuel*` images, which add only models). `PYTHONPATH` is set before ROS is sourced, as the recording command does with `docker exec -e` (R-H3f): `setup.bash` then adds the ROS paths to it, while `PYTHONPATH=… python3` after `source` would replace them and lose `rclpy` (H3 record):
  ```bash
  docker run --rm -v "$WT":/wt -w /wt -e PYTHONPATH=/wt:/wt/nl_control --entrypoint bash go2-voice-ltl-gazebo:patch-0005 -c \
    'source /opt/ros/jazzy/setup.bash && command -v gz && python3 -c \
     "import numpy, rclpy, sensor_msgs.msg, nav_msgs.msg, std_msgs.msg, gazebo.tools.record_frames; from gazebo.tools.png_rgb import write_png; write_png(\"/dev/null\", numpy.zeros((2, 2, 3), \"uint8\"))"'
  ```
  Expected: exit 0, with the path of `gz` printed.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add gazebo/tools/record_frames.py gazebo/tools/detector_check.py gazebo/tools/png_rgb.py tests/test_detector_check.py tests/fixtures/detector` and `git commit -m "feat: record still frames in Gazebo and replay them through the detector core"`.

---

## Wave 3 `[HEAVY]`: acceptance

Every step: the D11 gate first; one step at a time; Gazebo and the detector restarted together for every run (`G down` then `G up -d --no-build …`; M4); a record section in `docs/reference/gazebo_open_objects.md` (or `object_phrasing_trials.md` for H4) with the time, power state, other containers' CPU, commands and every result, in Japanese like the other records. A step that fails its line stops and reports to the controller with the numbers; it changes no rule, line or copy. The commands use the functions of "Commands" (`TA`, `T`, `TD`, `G`) with `WT` = the step's worktree and `DOMAIN` = its id; Gazebo runs go through `G` and its own network, so they need no domain id. For a GUI-off run, Gazebo starts with `OBJECTS_SOURCE=<source> G run -d --no-deps --service-ports --name go2oo-gazebo gazebo ros2 launch /work/gazebo/launch/go2_gazebo.launch.py gui:=false` (in detector mode plus `OBJECTS_SOURCE=detector DETECTOR=<oracle|yoloe> G --profile detector up -d --no-build detector`) and stops with `docker rm -f go2oo-gazebo` (our own container) and `G down`.

**The room after the room fix** (branch `objects2/room-fix`, rulings R-H3a–R-H3g; the room proposal after the H3 screening). H1 and H3 below record the first room. Every step after the room fix uses this one:
- Image `go2-voice-ltl-gazebo:patch-0005-fuel2` (Sofa, Table, Suitcase1, Construction Cone), the compose default. `patch-0005-fuel1` (the first room: TrashBin and LampAndStand, no Construction Cone) is kept on this Mac only; with it, `household` stops the bridge (no `construction_cone`).
- `household`: `box_1`, `chair_1` (unchanged), `suitcase_1` (−1.62, −1.14), `traffic_cone_1` (−0.3, −4.45), `sofa_1` (−4.6, 0), `table_1` (unchanged). `trash_can_1` and `lamp_1` are gone.
- Detector targets (`detector_check.PREVIEW_TARGETS`, `detectable: true`): `sofa_1` and `suitcase_1` from Bf, `traffic_cone_1` from R5. `table_1` stays logical-only, from L. B, L and R are no longer detector test poses.
- Test poses, each reached from the spawn of a fresh Gazebo with `G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "<formula>"`, one formula after the other:
  - **Bf:** `F standing`, `F(turned_left & X F turned_left)`, `F moved_forward`. The screening measured x −0.506…−0.560, y −0.054…−0.141, heading −166.6…−172.4°.
  - **R5:** `F standing`, `F turned_right`, `F(moved_forward & X F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward))))`. Measured: x −0.225…−0.342, y −2.654…−2.804, heading −95.1…−97.6°.
  - **L** (logical only): `F turned_left`.
  The turns overshoot: one turn is about ±96°, two about 191.5° (R-H3e). The grid numbers of every target formula from these poses are pinned in `tests/test_household.py`, nominal and measured.
- The preview set is `spawn_lying`, `spawn`, `Bf`, `Bf_s1` … `Bf_s4`, `R5`, `R5_s1` … `R5_s4`. A shift that starts with a minus is written `--shift=-0.1,0.1,-5` (R-H3f).
- Thin margins (room proposal; `tests/test_household.py`). R-H3b's ±35° is measured from the image centre, which is the camera, 0.327 m ahead of the base; the room proposal's bearings and distances are base → footprint centre. The numbers below come from H3's Bf (−0.506, −0.062, −169.2°) and the box of measured Bf poses (its corners); the R5 pose in `MEASURED_POSES` is derived (H3's R plus five 0.55 m steps), and the 4 real R5 runs make up the box:
  - the cone's IoU with its truth box is 0.51–0.57 at R5, because its shadow falls inside the box;
  - at `Bf_s4` the suitcase was called trash_can in both runs, so its preview rests on `Bf` and two of `Bf_s1`, `Bf_s2` and `Bf_s3`, and `Bf_s2` is the clipped view below;
  - the suitcase's bearing follows the turn. Its centre is +41.3° from the camera at H3's Bf and +35.8…+47.2° over the measured box (from the base: +33.3°, and +28.5…+38.1°; the screening's runs gave +29…+36°). So it is outside ±35° at every measured pose. The −172.4° run, which gave 36°, is the least overshoot (187.6° turned), not the most-turned;
  - in `Bf_s2` (dyaw −5°) the suitcase's box is cut by the left image edge at every measured heading (its footprint reaches +61.2…+70.9°, past the 60° edge). A miss in `Bf_s2` reads as clipping, not as a detector fault. In `Bf` its left edge is at u 6–108 px;
  - the suitcase is 1.457–1.556 m from the base (1.184–1.308 m from the camera), under R-H3b's 1.5 m in part of the measured range;
  - table_1 stays at least 2.3° outside every Bf view at the measured headings (the corner (−0.506, −0.054, −172.4°) with s3). At a nominal 180° Bf it enters the pose view and the s2 and s3 views. Re-run the view check (`test_no_undetectable_model_in_a_detector_test_view`) if the turn controller changes;
  - the sofa lap from Bf is lost from one corner of the measured box jittered by (+0.1, +0.1, +5°).
  H3b is the gate for all of these (R-H3h (4)); the placement stands as R-H3h accepted it.

### H1 `[HEAVY]`: Build the Fuel image and record Suitcase1 (branch `objects2/h1-fuel`; after Task 2.6)

- [ ] **Fetch Suitcase1 once** into a new empty folder in the scratch area: `curl -fsSL -o <scratch>/fuel/Suitcase1.zip https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1/2/Suitcase1.zip`; record `shasum -a 256` and the byte count. Do not unpack it on the host; the build unpacks it after checking the pin.
- [ ] **Pin the zip:** `docker/fuel_models.json` gets Suitcase1's sha256 and bytes (the build refuses a null pin). Run `T tests/test_layout.py tests/test_fuel_image.py`.
- [ ] **Build:** `docker build --build-arg BASE=go2-voice-ltl-gazebo:patch-0005 -t go2-voice-ltl-gazebo:patch-0005-fuel1 -f docker/Dockerfile.gazebo-fuel docker`. Expected: success; then `docker run --rm --entrypoint bash go2-voice-ltl-gazebo:patch-0005-fuel1 -c 'ls /opt/go2_models; echo "$GZ_SIM_RESOURCE_PATH"'` lists the 5 folders and ends with `/opt/go2_models`.
- [ ] **Measure the five models inside the built image** (review M12: only the image has all five unpacked; nothing runs on the host):
  ```bash
  docker run --rm -v "$WT":/wt -w /wt --entrypoint bash go2-voice-ltl-gazebo:patch-0005-fuel1 -c \
    'for m in /opt/go2_models/*; do echo "$m"; python3 -I /wt/gazebo/tools/model_extent.py "$m"; done; cat /opt/go2_models/Suitcase1/model.config'
  ```
  Suitcase1's `size`, `offset`, `height` and `z_min` give its layout footprint, height and `z`; its author comes from `model.config` if present. The four other models are a cross-check against spec §12.2's footprints: differences are reported, and the spec values stay unless the controller says otherwise.
- [ ] **Pin the model:** `docker/fuel_models.json` gets Suitcase1's author; `gazebo/layouts/household.json` gets its footprint, height and `z`. If the footprint exceeds 0.6 × 0.4, re-place it within spec §12.1's rules and re-run `T tests/test_household.py` (a moved number is reported). Run the full suite.
- [ ] **Commit** the pins and the record (`docs: H1 Fuel image built, Suitcase1 pinned`).

### H2 `[HEAVY]`: Build the detector image and run its tests (branch `objects2/h2-detector`; after wave 2)

- [ ] **Build:** `docker build -t go2-voice-ltl-detector:yoloe-11s -f docker/Dockerfile.detector docker`. Record the build time and `docker image ls` size.
- [ ] **A1 check first:** `TD -m detector tests/test_detector_yoloe.py -k embedding_cache`. Record which embedding path works on 8.4.210 (ultralytics' save/load or the `get_text_pe` fallback, as `embedding_source` shows); if neither, fix `detector/yoloe.py` in this branch and re-run.
- [ ] **In-image tests:** `TD -m detector tests/test_detector_yoloe.py tests/test_detector_protocol.py` and `TD -m "not slow" tests/test_detector_geometry.py tests/test_detector_tracker.py tests/test_detector_core.py tests/test_detector_oracle.py tests/test_detector_check.py` (numpy 2.x). Expected: PASS.
- [ ] **Commit** any adapter fix and the record.

### H3 `[HEAVY]`: Detection preview, A6 / M2 (branch `objects2/h3-preview`; after H1 and H2; before H5)

- [ ] **Check `record_frames` in the Fuel image** (light, before any Gazebo): Task 2.12's Step 4 command with the tag `go2-voice-ltl-gazebo:patch-0005-fuel1` (`patch-0005-fuel2` after the room fix). Expected: exit 0.
- [ ] **Record the views** headless on `household` (review M4: the list is pinned here). Each frame is taken by `docker exec -e PYTHONPATH=/work:/work/nl_control go2oo-gazebo bash -c 'source /opt/ros/jazzy/setup.bash && python3 /work/gazebo/tools/record_frames.py /work/out/h3-preview --view <name> [--shift=<s>]'` into the worktree's `out/h3-preview/` (`out/` is git-ignored; the images are CC BY renders and never enter git, spec §11.11), and each pose is driven with `G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "<formula>"`. The four shifts `<s>` are `0.1,0.1,5`, `0.1,-0.1,-5`, `-0.1,0.1,-5` and `-0.1,-0.1,5` (dx, dy in m, dyaw in degrees; names `<pose>_s1` … `<pose>_s4`). Write them as `--shift=<s>`, for example `--shift=-0.1,0.1,-5`: with a space, argparse takes a value that starts with a minus for another option and exits 2 (R-H3f).
  1. Fresh GUI-off detector-mode Gazebo (`OBJECTS_SOURCE=detector`, the GUI-off form above, no detector service): `spawn_lying` right away (offline data for the lying viewpoint, which spec §11.5 has not measured; `preview` ignores it), then `F standing` and `spawn` (plain only).
  2. Same Gazebo: `F(turned_left & X F turned_left)`, then `B` and `B_s1` … `B_s4`.
  3. Restart; `F turned_left`, then `L` and `L_s1` … `L_s4`.
  4. Restart; `F turned_right`, then `R` and `R_s1` … `R_s4`.
  That is 17 frames, of which `preview` uses 16: `spawn` plus B, L and R × (plain + 4 shifts). Spec §12.8 counts 15 views (the 15 pose views); Task 4.1 writes the count with this list. Check the first render: every Fuel model is visible in its views (M15).
- [ ] **Detect:** `docker run --rm -v "$WT":/work -v /dev/null:/work/env.txt:ro -w /work go2-voice-ltl-detector:yoloe-11s python3 gazebo/tools/detector_check.py /work/out/h3-preview --detector yoloe` (YOLOE-11s text at 640, the 87 prompts + 3 ignore).
- [ ] **Decide (spec §12.8):** a target passing `preview` gets `"detectable": true`; from the confusion table set `BY_LABEL` overrides in `detector/yoloe.py` where a wrong name sits between 0.35 and the right name's score, or drop a prompt (box, desk, cabinet, bench, stool first) from `detect` into `en` through spec §7.5 (`T tests/test_vocabulary.py` and the language tests). If fewer than 3 new kinds pass: (1) the word fixes, (2) an alternate model of spec §12.9 (re-run H1 for its pin and `tests/test_household.py`), (3) re-placement within spec §12.1, (4) report to the controller with the numbers. If the sofa fails (O38): one forward step after the turn (3.9 m), then MiniSofa, then fewer targets behind the spawn.
- [ ] **Re-check** spec §12.4 with `T tests/test_household.py` after any change; full suite; commit the layout, vocabulary, overrides and the record (nothing under `out/`).

### H4 `[HEAVY]`: Real-Claude phrasing trials ×2, S6 (branch `objects2/h4-trials`; first heavy step, as soon as wave 1 is merged; gates H5)

- [ ] **Run twice** (rulings Q2; nothing opens or copies `env.txt`: compose reads the main checkout's through `env_file`), `DOMAIN=64`:
  ```bash
  cd ~/work/physicalAI/go2 && docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=64 -e LLM_BACKEND=claude \
    -v "$WT":/wt -w /wt -e PYTHONPATH=/wt:/wt/nl_control app python3 scripts/object_phrasing_trials.py --workers 4
  ```
  after checking that `voice_ltl` imports from `/wt` (`TA python3 -c "import voice_ltl; print(voice_ltl.__file__)"` prints `/wt/voice_ltl/__init__.py`). Keep each run's JSON lines.
- [ ] **Judge (O10):** both runs lap 10/10, near 10/10, kinds ≥ 9/10, refusals ≥ 13/14, regression ≥ 12/13. Record every miss with Claude's raw `unresolved_objects` and what the code accepted in a new section of `docs/reference/object_phrasing_trials.md`.
- [ ] **Gate (Execution, "H4 gate"):** if both runs pass, say so to the controller: the shared step's language part is accepted and H5 may start once H3 has merged. On a miss, stop and report; the controller picks fix forward (`objects2/h4-fix`, spec §9.2's first lever: the hypernym example, then two new runs) or the revert of the Task 1.8 and 1.4 merges. H5 and Task 4.1 wait for that.
- [ ] **Commit** the record.

### H5 `[HEAVY]`: Logical Gazebo measurements, S7 (branch `objects2/h5-logical`; after H3, and after H4 has passed)

- [ ] **M0 logical baseline:** RTF of a fresh `household` Gazebo, GUI on and GUI off, 75 s windows after 20 s (`G run --rm --no-deps -T app python3 gazebo/tools/objects_check.py rtf --seconds 75`), 3 runs each; the old records are not reused (C4).
- [ ] **§2 set 1 (spec §13.6)** in `household`, logical, a fresh Gazebo per run, each through `G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run …`; no fall:
  - `run --expect lap "F around_box_1"` ×5 (≥ 4 pass, 3 in a row; the §2 lap keys);
  - `run --expect near "F near_chair_1"` ×3 and `run --expect near "F(near_chair_1 & X F hello)"` ×1 (the §2 chair keys; no target: today's summary measures `chair_1`);
  - `run --expect near --target box_1 "F near_box_1"` ×1 (「箱の前まで」: the target lines on the box, plus the box's centre line 1.5 m / 45°, which `summarize` adds only for `--target box_1` with `source == "logical"`, O16; without `--target` the run would be judged on the chair).
- [ ] **New kinds, logical (spec §13.4):** for `sofa_1` and `suitcase_1` (from Bf), `traffic_cone_1` (from R5) and `table_1` (from L): the test pose's procedure ("The room after the room fix" above), `objects_check.py resolve --target <id>` (waits for `measured: true`), then `objects_check.py run --expect near --target <id> "F near_<id>"` 2/2 and `--expect lap … "F around_<id>"` 2/2. Record whether the front obstacle bit was set during the sofa near (O14).
- [ ] **Commit** the record.

### H6 `[HEAVY]`: Detector-mode Gazebo measurements, A7 (branch `objects2/h6-detector`; after H5)

Detector-mode runs start with `OBJECTS_SOURCE=detector DETECTOR=<oracle|yoloe> G --profile detector up -d --no-build gazebo detector` (GUI on) or the GUI-off form of the Wave 3 preamble, and stop with `G down`.
- [ ] **M0 before (spec §13.5, M0 at A7; D11: the baseline is taken under the same conditions, before and after):** logical `household` RTF, a fresh Gazebo per run, 20 s, then `objects_check.py rtf --seconds 75`: 3 runs with the GUI on and 3 with the GUI off.
- [ ] **M0 detector, still:** `DETECTOR=yoloe`, 3 runs with the GUI on and 3 with the GUI off: a fresh stack, the R5 procedure (`traffic_cone_1` in view, so the detector keeps inferring every 3 s), then `rtf --seconds 75` while the Go2 stands still. Record the status' `latency_ms_p50` / `latency_ms_p90` with each.
- [ ] **M3 accuracy (spec §13.3):** `DETECTOR=oracle`, then `objects_check.py accuracy` at the spawn (lying), after `F standing`, and at Bf and R5, the detector test poses (a fresh stack per pose, as in H3); then the same with `DETECTOR=yoloe`. The oracle must meet every position and size line with `covers` true on the observed extent; record `fill_short_m`, `ids_retired` and the status' `notes`.
- [ ] **M4 formula runs (spec §13.4, §13.5), GUI on:** for each `detectable` new kind (≥ 3): its test pose's procedure (Bf for `sofa_1` and `suitcase_1`, R5 for `traffic_cone_1`), `resolve` (record `listed_after_still_s` ≤ 5 s and the status' `latency_ms_p50` ≤ 600, `latency_ms_p90` ≤ 800), then `F near_<detector id>` 3/3 and `F around_<detector id>` 3/3 with `--target <truth id>`; record each run's `rtf_stats` (walking and still: the GUI-on detector-mode walking RTF of M0). If a lap or near fails, re-run it GUI-off; if it still fails, run M1 (camera in the xacro, no camera bridge) and report to the controller (O42).
- [ ] **M0 detector, walking, GUI off:** for the first detectable kind, one `F near_<detector id>` and one `F around_<detector id>` run with the GUI off (same procedure as M4); record their `rtf_stats` and their lines (a GUI-off re-run of M4 counts here too).
- [ ] **Voice step (spec §13.4, O26; ruling on review I6, see "Notes for the controller"):** per detectable kind, 「〈名前〉まで歩いて」 1/1 and 「〈名前〉の周りを回って」 1/1 typed into the operation page with the real Claude, judged by M4's lines.
  - The app runs from the main checkout's directory under our project name `go2oo`: compose then reads the main checkout's `env.txt` through `env_file` as it does for the user's own stack (nobody opens, prints or copies it), and the app joins our network `go2oo_default`, where our Gazebo and detector run. An override puts the worktree at `/work` instead of the main checkout. Write it into the scratch area (never added to git):
    ```yaml
    # go2oo-app.override.yaml: only for H6's voice step
    services:
      app:
        volumes:
          # replaces ./:/work, which resolves against --project-directory (the main checkout);
          # compose merges a service's volumes by their container path
          - ${GO2OO_WT:?set GO2OO_WT to the worktree}:/work
        environment:
          LLM_BACKEND: claude
    ```
    and define, next to the other functions (`OV` = that file's absolute path):
    ```bash
    GA() { (cd ~/work/physicalAI/go2 && GO2OO_WT="$WT" GO2_UI_PORT=8811 SIM_KIND=gazebo \
             docker compose -p go2oo --project-directory . -f "$WT/compose.yaml" -f "$OV" "$@"); }
    ```
    Only `app` runs this way; Gazebo and the detector stay on `G`. Never `GA config` (it prints `env_file` values).
  - **Check before use:** `GA run --rm --no-deps -T app python3 -c "import voice_ltl.vocabulary as v; print(v.__file__, len(v.VOCABULARY.entries))"` must print `/work/voice_ltl/vocabulary.py 50`. `/work` is the same path for both checkouts, so the check uses a module that only the worktree has. If it fails, or the app reports the login message, do not work around it: record the voice step as not run, and it falls to the user (O26).
  - **Each instruction:** a fresh detector-mode stack (`DETECTOR=yoloe`, GUI on); the test pose's procedure and `resolve --target <truth id>` as in M4, both before the app starts (so `run_formula` and the app never drive the Go2 at once); `GA up -d --no-build --no-deps app` and wait until the page at http://localhost:8811 shows the Go2 ready and the target in 「見えている物」; start `G run --rm --no-deps -T app python3 gazebo/tools/objects_check.py watch --expect near|lap --target <truth id>`; type the instruction into the page's text field (the Browser pane; the field posts to `/api/text`). Record the formula Claude gave, the page's messages and the `watch` line. `G down` also removes the app.
- [ ] **M5 (D9):** with the Go2 standing, 「前に箱を置く」 in view; record whether a `box` row appears (cardboard look). Not a pass line.
- [ ] **M0 after:** the M0-before block again (logical, 3 + 3 runs), now that the detector block is done.
- [ ] **§2 set 2 (spec §13.6):** the four commands of H5's "§2 set 1" again (logical, `run --expect near --target box_1 "F near_box_1"` included); record whether box_1 and chair_1 were found in detector mode during M3 and M4 (Q4).
- [ ] **Resources:** record the containers' CPU and memory during the runs (spec §11.12).
- [ ] **Commit** the record.

### H7 `[HEAVY]`: Slow tests (branch `objects2/h7-slow`; after wave 2 is merged; in the heavy order before H2)

It needs Task 2.5's `test_sofa_synthesis_is_fast` (spec §13.1's synthesis-time line, stage 2a), so it never runs before stage 2a has merged; and its timing asserts need a quiet CPU, so it runs only when no code task of ours runs a `T` suite and no other heavy step or Gazebo run of ours is up. After wave 2 is merged both hold until Task 4.1.
- [ ] **D11 gate**, plus `docker ps` shows no other container of ours. `DOMAIN=67`.
- [ ] **Timing tests** (wall-clock lines, so no `nice` and no CPU pin): `TA python3 -m pytest -q -m slow tests nl_control/tests --deselect tests/test_grid.py::test_near_keys_full_sweep`. Expected: PASS (sofa near ≤ 1 s and sofa lap ≤ 3 s, the existing `< 25 s` refusal, the Whisper fixture; that test may fetch its model into the main project's `hf-cache` on first use).
- [ ] **Near sweep** (M13; about 30 min on one core): `TA nice -n 19 taskset -c 0 python3 -m pytest -q -m slow tests/test_grid.py::test_near_keys_full_sweep`. Expected: PASS.
- [ ] Record both times and the CPU before and after; commit the record.

---

## Wave 4: docs

### Task 4.1: Docs (branch `objects2/docs`; after H6, and after H4 has passed)

**Files:**
- Modify: `README.md`, `docs/simulation.md`, `docs/design.md`, `docs/vocabulary.md`, `docs/superpowers/specs/2026-11-05-go2-objects-design.md` (one line), `docs/superpowers/specs/2026-11-11-go2-open-objects-design.md` (measured numbers), `tests/test_gazebo_objects.py` (doc pins whose anchors move)
- Create: `tests/test_docs_open_objects.py`

**Content:**
- `README.md` (commands only, M12): the two Gazebo builds (`docker build -t go2-voice-ltl-gazebo:patch-0005 -f docker/Dockerfile.gazebo docker`, then `docker compose --profile gazebo build gazebo`); `GO2_LAYOUT=default` for the old room; the detector build `docker compose --profile detector build detector` and start `OBJECTS_SOURCE=detector SIM_KIND=gazebo docker compose --profile gazebo --profile detector up -d`.
- `docs/simulation.md`: the household room (kinds, places, the turn-first habit), `GO2_LAYOUT`; the two builds; detector mode (start, what it sees, about 3.5 s after stopping, nothing on the page, restart detector with Gazebo); **the plain box and chair are not recognised by the detector** (Q4); licences: the detector container uses ultralytics, the YOLOE weights and the CLIP tokenizer fork under AGPL-3.0, the MobileCLIP text encoder weights are under Apple's research-only, non-commercial terms (the user accepted research use), ultralytics stays out of the app and gazebo images and behind `detector/protocol.py`, the prompt-free YOLOE and OWLv2/Grounding DINO are the alternatives, adding a LICENSE file is a separate decision; the section 「Gazebo の物の出典」 (spec §12.7 table and sentences, `design-vocabulary.md` §4.8 text, Suitcase1's author from H1); the `/go2/objects` example with `source` and `measured` (M14); the RTF sentence for the household world from H5/H6 (C4).
- `docs/design.md`: the grid lines (blocking from the footprint, bands A and B, size messages), the language lines (vocabulary file, `unresolved_objects`, kind guard, the five object messages), R16 rewritten and R-new (spec §9.8); a short licence paragraph (spec §11.2 puts the licence facts in both `docs/design.md` and `docs/simulation.md`, review M5): the detector container uses ultralytics, the YOLOE weights and the CLIP tokenizer fork under AGPL-3.0; the MobileCLIP text encoder weights are under Apple's research-only, non-commercial terms (the user accepted research use); ultralytics stays out of the app and gazebo images and behind `detector/protocol.py`; adding a LICENSE file to the repo is a separate decision for the user; a pointer to the spec.
- `docs/vocabulary.md`: the object section for every kind (how to add a kind, spec §7.5) and the messages table; keep or move the pinned anchors together with `tests/test_gazebo_objects.py`.
- The objects design: one line pointing readers to the spec. The spec: numbers measured in H1–H7 that differ, each with its reason (spec §15), including §12.8's view count (16 views used by the preview, as listed in H3).
- The spec's household room after the room fix (R-H3e, R-H3h (5), R-H3i; the numbers are pinned in `tests/test_household.py`):
  - the measured headings: two left turns −166.6…−172.4°, R5 −95.1…−97.6°. The R5 pose in `MEASURED_POSES` is derived (H3's R plus five 0.55 m steps along −95.8°), not a run's;
  - bearings and distances in both frames, each named: from the base to the footprint centre (the room proposal's), and from the camera, which is what R-H3b's ±35° of the image centre means. For the suitcase from Bf: +28.5…+38.1° and 1.457–1.556 m from the base, +35.8…+47.2° and 1.184–1.308 m from the camera. It is outside ±35° and partly under 1.5 m, accepted for v1 by R-H3h (4) with H3b as the gate; in `Bf_s2` its box is cut by the left image edge;
  - R-H3c holds at the measured headings only. At a nominal 180° Bf, table_1 enters the pose view and the s2 and s3 views (both −5°). At the measured headings it stays at least 2.3° outside. Re-run the view check if the turn controller changes;
  - the right edge of §12.1 as a footprint edge, about y ≤ −4.2 (the cone's is −4.1985): every new footprint is at least 0.7 m from every acceptance cell centre (the screening's line; cone 0.768 m, suitcase 0.701 m), and the acceptance PLANTs are unchanged (the rule). The room proposal's y ≤ −4.5 is the centre line of a 0.6 m object, not the edge and not every object's centre (at y −4.0 the cone changed `F near_chair_1`).
- [ ] **Step 1: Write the failing tests** in `tests/test_docs_open_objects.py`: the AGPL-3.0 sentence, the MobileCLIP research-only sentence and the LICENSE-file sentence, each in both `docs/simulation.md` and `docs/design.md`; every Fuel model's name, version, URL and licence; the Q4 sentence; the detector start command (in README and `docs/simulation.md`); `GO2_LAYOUT`; and no `/Users/` in any doc.
- [ ] **Step 2: Run them to see them fail.** Run: `T tests/test_docs_open_objects.py tests/test_gazebo_objects.py`. Expected: FAIL.
- [ ] **Step 3: Write the docs** (Japanese, like the existing ones).
- [ ] **Step 4: Run.** Same command. Expected: PASS.
- [ ] **Step 5: Full suite.** Run: `T tests nl_control/tests`. Expected: 0 failed.
- [ ] **Step 6: Commit.** `git add README.md docs/simulation.md docs/design.md docs/vocabulary.md docs/superpowers/specs/2026-11-05-go2-objects-design.md docs/superpowers/specs/2026-11-11-go2-open-objects-design.md tests/test_docs_open_objects.py tests/test_gazebo_objects.py` and `git commit -m "docs: open object kinds, the detector container, licences and Fuel attribution"`.

---

## Coverage map

| Spec | Task |
|---|---|
| §7 vocabulary, rules, contents, adding kinds | 1.1 (loader, tests), 4.1 (§7.5 in docs) |
| §8 ids, propositions, rows | 1.2 |
| §9.1–9.9 prompt, field, lookup, words, messages, flow | 1.1 (lookup, matcher), 1.2 (focus, messages), 1.4 |
| §9.10 page | 1.5 |
| §10 near band, rectangle, laps, lines, tests | 1.3, 1.7 (O15), 2.4 (lines), H7 (slow sweep, after stage 2a) |
| §11.1–11.2 container, image, licence | 2.8, 2.11, H2, 4.1 |
| §11.3 switch and topics | 2.3, 2.9, 2.10 |
| §11.4–11.5 camera, when to detect | 2.9, 2.7 |
| §11.6–11.8 geometry, tracking, wire | 2.1, 2.2, 1.2 |
| §11.9–11.10 failures, swap interface | 2.7 (status, `notes`), 2.10 (exit codes), 2.2 (`ScriptedDetector`) |
| §11.11 accuracy, offline tools | 2.4, 2.12 (`record_frames` in the gazebo container, PNG via `png_rgb`) |
| §11.13 button boxes (D9) | 2.3, H6 |
| §12 household room, layout, Fuel image, attribution, preview | 1.6, 2.5, 2.6, H1, H3, 4.1 |
| §13.1 tests | every wave-1/2 task; the synthesis-time line: 2.5's slow test, run in H7 |
| §13.2 trials, O10 acceptance | 1.8, H4 (gates H5 and 4.1) |
| §13.3–13.6 Gazebo acceptance | H5 (§13.4 logical, §13.6 set 1 with `--target box_1`), H6 (§13.3 M3; §13.4 M4 and the voice step through `GA` and `objects_check watch`; §13.5 M0 before and after, GUI on and off, still and walking; §13.6 set 2) |
| Rulings C1–C6, Q2, Q4, user answers | 2.4 (C1, C2), 2.3 (C3), H5 and H6 (C4), 1.1 (C5), 2.2 and 2.7 (C6, the note on the status), `T`/H4/H6 voice step (Q2), 4.1 (Q4, O24) |

---

## Notes for the controller

Rulings this plan takes by default (from `plan-review.md`), and the places where it deviates from the spec's wording. Each can be overridden; the task that changes with it is named.

1. **O10 timing (review I5).** Spec §13.2 and `design-language.md` §6.3 put the two-run trial acceptance "before merging into `feat/open-objects`". The plan merges Tasks 1.4 and 1.8 at stages 1c and 1d and lets H4 gate the end of the shared step instead (Execution, "H4 gate"): H5 and Task 4.1 wait for it, and a miss means fix forward or revert, by the controller's choice. Reason: Task 1.8 and wave 2 are cut from `feat/open-objects` after those merges, and no wave-2 file depends on the prompt. To keep the literal reading instead: hold 1.4 and 1.8 on their branches, cut 1.8 from 1.4's branch, run H4 there, and merge both after it passes; Task 1.7's `session.py` edit then merges beside 1.4's `_redacted` change later.
2. **Voice step (review I6).** Option (a) is adopted: the app runs from the main checkout's directory as project `go2oo` with an override that mounts the worktree at `/work` (H6, `GA`), so compose reads the main checkout's `env.txt` through `env_file` and nobody opens it, as in ruling Q2. It is measured with the new `objects_check.py watch` (Task 2.4). If the import check or the login fails, the step is recorded as not run and falls to the user (O26).
3. **Rules 2 (world part) and 6 run only in the app (review M15).** `vocabulary.py` stays stdlib-only and imports neither `world.py` nor `translator.py`, so the reserved segments from `world` and the handling-verb check run when `world.py` and `translator.py` import (Task 1.1). The bridge and the detector check rules 1–5 and 7–11 without rule 2's world part. Spec §7.3 says the loader checks 1–11; the app still fails closed, and the bridge and the detector never parse formulas.
4. **Rules 4 and 10 (review I1).** The spec states "a not_word is not a word" in both; rule 4 owns it and rule 10 checks only that each not_word contains a word. Rule numbers are unchanged.
5. **Rule 11's order (review M11).** The kind count is checked right after rule 1, before the per-kind rules; the detect-prompt warning stays last. The number stays 11.
6. **`record_frames.py` (review I8).** It runs in the gazebo container, not the detector image (spec §11.11), because `--shift` needs `gz service`; it writes PNGs with the stdlib `gazebo/tools/png_rgb.py`, since the gazebo image has no OpenCV and no verified PIL. Task 2.12 and H3 check that it imports there before any heavy run.
7. **Preview views (review M4).** H3 pins 17 frames, of which `preview` uses 16 (the standing spawn view plus B, L and R × (plain + 4 shifts)); spec §12.8 says 15. The lying spawn frame is offline data only. Task 4.1 corrects the count in the spec (spec §15: a number, not a rule).
8. **Shapeless sightings (review I2).** `geometry.observe` returns a `Sighting` that maps 1:1 onto `tracker.Observation`. A detection with no usable geometry (`design-detector.md` §7.2 step 7) makes no `Observation`, because nothing can associate it; it is counted in `health()["shapeless"]`.
9. **Oracle height (review M8).** The oracle projects the truth footprint × the vocabulary height (spec §11.10) everywhere it runs; only `detector_check.preview` projects with the layout height, because it compares against the truth.
10. **Status keys (review M14).** `/go2/detector_status` carries the spec §11.9 keys plus `filled`, `shapeless` and `notes` (the C6 "ids used up" note).
11. **Slow tests (review I4).** H7 runs after wave 2 is merged, when no `T` suite or other heavy step of ours runs; its timing tests run without `nice`, and only the near sweep runs under `nice -n 19 taskset -c 0`.
