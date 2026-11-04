# 声で指示 → LTL → 監督制御 → Go2（シミュレーション）実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ブラウザでスペースを押して話した日本語の指示を、LLM（Claude / Codex の CLI）が LTLf に翻訳し、flloat と pitct で監督器を合成し、シミュレーションの Go2 を ROS 2 経由で安全に動かす。

**Architecture:**
- **純粋な Python のパッケージ `voice_ltl/`（ROS なし）：** 単語帳と環境オートマトン、翻訳、合成、実行時の監督、センサー判定、全体の進行役を持つ。
- **ROS 2 の窓口：** 純粋なパッケージの外側に、ROS 2 の窓口（`ros_link.py`）と Web サーバー（`server.py`）をかぶせる。
- **シミュレーションの Go2：** 中身の計算（`sim/sim_core.py`）と ROS 2 ノード（`sim/fake_go2.py`）に分ける。
- **テスト：** 中身の計算どうしを直接つないだ結合テストで、受け入れの表を ROS なしで試す。
- **再利用：** 実機に効く部品（実行器・障害物判定・`api_id`）は zip の `go2_nl_control` を書き換えずに使う。

**Tech Stack:**
- Ubuntu 22.04 + ROS 2 Humble（Python 3.10）、unitree_ros2（`unitree_api` / `unitree_go`）、CycloneDDS
- pitct 1.0.8、flloat 0.3.0、graphviz
- faster-whisper 1.2.1
- FastAPI + uvicorn
- Claude Code CLI 2.1.303 / Codex CLI
- Docker Desktop（arm64）と docker compose

**Spec:** `go2/docs/design.md`（以下「設計書」）。講義のノートブックは `go2/docs/reference/nl2fl_grid.ipynb` にコピーして参照する（Task 1）。

**コミットについて：**
- `go2/` は git リポジトリで、作業はブランチ `feat/voice-ltl-gazebo` で行う（ユーザー承認済み）。
- 各タスクの最後に、そのタスクのファイルだけを `git add` してコミットする。
- コミットメッセージの最後に `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` を入れる。
- push はしない。`env.txt` は絶対にコミットしない。

**実行の順番：** 1 → 15 → 2 → 3 → … → 14 → 16 → 17。
- Task 15（Gazebo で立って歩けるかを試す）を早めに行う。
- Task 15 が「使えない」で終わったら、そこで止めてユーザーに相談する。

## Global Constraints

- 動かす場所はすべて Docker コンテナの中。イメージは `ros:humble-ros-base-jammy`（Ubuntu 22.04、Python 3.10）。arm64 と x86_64 の両方でビルドできること。
- ROS 2 のトピックと型は実機と同じにする：
  - `/api/sport/request`（`unitree_api/msg/Request`）
  - `/sportmodestate`（`unitree_go/msg/SportModeState`）
  - `/utlidar/cloud`（`sensor_msgs/msg/PointCloud2`、frame `utlidar_lidar`）
- `api_id` と姿勢の移り方は `nl_control/go2_nl_control/actions.py` のものを使い、書き換えない。
- 制御の周期は 10 Hz。
- 止まる距離は 0.5 m、動き出す距離は 0.7 m。センサーが 0.5 秒途切れたら移動を保留する。保留が 15 秒続いたら中止する。
- 前進・後退の1回は 0.5 m、旋回の1回は 90 度。繰り返しは1つの動作につき 6 回まで。
- 翻訳の聞き直しは最大 3 回。
- 「止まって」「ストップ」「停止」は LLM を通さず、すぐ StopMove を送る。判定には zip の `is_stop_command` を使う。
- 終了するときは StopMove を 3 回送る。
- LLM には道具を使わせない。Claude は `--bare` を使わない。
- トークンや認証のファイルは `env.txt` と Docker のボリュームにだけ置く。実装者はトークンの値を読まない・書かない・表示しない。
- 画面と利用者への文言は日本語。
- Web ページはブラウザの `http://localhost:8810` で開く（ポートは `GO2_UI_PORT` で変えられる。空なら既定の 8810）。Mac ではコンテナの 8000 番を `127.0.0.1:${GO2_UI_PORT:-8810}` に出す。
- 危ない動作（宙返り・逆立ち・二足歩行・ジャンプ）は語彙に入れない。

## Review Focus

1. **スペースの押しっぱなし：** キーのリピートで録音が何度も始まらず、ページもスクロールしない。押したままウィンドウから離れたら（blur）、そこで録音を終える。→ Task 13 のブラウザテスト
2. **一瞬だけ押した・何も話さなかった：** 書き起こしが空なら「聞き取れませんでした」と出し、LLM は呼ばない。→ Task 9 `test_empty_transcript_does_not_call_llm`
3. **LLM が考えている間に次の指示が来た：** 古い結果は捨て、新しい指示だけを実行する。→ Task 9 `test_newer_instruction_wins_over_slow_translation`
4. **CLI がログインしていない・時間切れ・出力が壊れている：** わかる文言を出し、Go2 は動かさず、アプリは落ちない。→ Task 6 `test_claude_cli_failure_raises_llm_error`、Task 9 `test_llm_failure_reports_and_stays_idle`
5. **計画と実際の姿勢が食い違う（リモコンで伏せさせた等）：** 次の動作を送らずに計画を中止し、理由を出す。→ Task 9 `test_posture_mismatch_aborts`

---

## ファイル構成

すべて `~/work/physicalAI/go2/` の下に置く。コンテナでは `/work` に bind mount する。

```
go2/
  compose.yaml               # Mac 用：app + sim（ブリッジネットワーク、127.0.0.1:${GO2_UI_PORT:-8810}）
  compose.robot.yaml         # Linux PC 用：app のみ、network_mode: host、SIM=0
  docker/Dockerfile
  docker/unitree_setup.sh    # コンテナ内の ~/unitree_ros2/setup.sh になる
  env.txt.example
  .gitignore                 # env.txt, .env, __pycache__/, out/
  README.md                  # 日本語の使い方（Task 14）
  docs/design.md, docs/reference/nl2fl_grid.ipynb, docs/superpowers/plans/...
  scripts/demo_script.py, scripts/keyboard_velocity.py   # 渡されたものをそのまま
  nl_control/                # zip の go2_nl_control/ を名前だけ変えて置く（中身は無変更）
    nl_control.py, offline_sim.py, README.md, env.txt.example, go2_nl_control/, tests/
  voice_ltl/
    __init__.py
    automata.py   # World / Automaton / Synthesis、spec_from_formula、synthesize（pitct・flloat）
    world.py      # Go2 の単語帳と環境オートマトン（「先に用意しておくもの」）
    planner.py    # Runtime：監督器の上で次のイベントを選ぶ
    grammar.py    # check()
    prompt.py     # 4パートのプロンプトと JSON Schema
    translator.py # translate()：聞き直しのループ
    llm.py        # ClaudeCLI / CodexCLI / FakeLLM / make_llm
    sensing.py    # 前方・後方の判定（ObstacleMonitor の再利用）
    diagrams.py   # 状態遷移図の SVG
    stt.py        # Transcriber（faster-whisper）
    session.py    # Link / DirectSimLink / Session（全体の進行役）
    ros_link.py   # RosLink、SimControl（ROS 2）
    server.py     # FastAPI、制御ループ、main()
    static/index.html, static/app.js, static/style.css
  sim/
    __init__.py
    sim_core.py   # SimGo2（ROS なし）
    fake_go2.py   # ROS 2 ノード
  gazebo/                    # Task 15〜16
    __init__.py
    bridge_core.py           # 橋渡しの中身（ROS なし）：指令 → 制御の入力、状態 → error_code
    gazebo_bridge.py         # ROS 2 ノード：/api/sport/request ↔ unitree_guide、/sportmodestate、/utlidar/cloud、箱
    go2_sensors.urdf.xacro   # LiDAR と真の位置（p3d）を Go2 に足す
    launch/go2_gazebo.launch.py
  docker/Dockerfile.gazebo, docker/gazebo_entry.sh   # Task 15
  voice_ltl/run_formula.py   # Task 17：式から直接動かす道具（LLM なし）
  tests/
    conftest.py   # sys.path に /work と /work/nl_control を足す
    fixtures/      # 音声のサンプル（Task 11）
    test_world.py, test_automata.py, test_planner.py, test_grammar_translator.py,
    test_llm.py, test_sensing.py, test_sim_core.py, test_session.py,
    test_diagrams.py, test_stt.py, test_server.py
```

**zip のフォルダ名を `nl_control/` に変えた理由（私の判断）：** 外側も内側も `go2_nl_control` だと、`/work` を import パスに入れたとき名前がぶつかって紛らわしいため。中身のファイルは1つも変えない。

**テストの実行：** 以降、テストのコマンドはすべてコンテナの中で動かす。

```bash
cd ~/work/physicalAI/go2
docker compose run --rm --no-deps app python3 -m pytest -q tests/<file>.py
```

---

### Task 1: フォルダとコンテナ（ROS 2 Humble + unitree_ros2 + 依存）

**Files:**
- Create: `compose.yaml`、`docker/Dockerfile`、`docker/unitree_setup.sh`、`env.txt.example`、`.gitignore`、`tests/conftest.py`
- Copy:
  - `~/Downloads/demo_script.py` と `~/Downloads/keyboard_velocity.py` → `scripts/`
  - zip の `go2_nl_control/` → `nl_control/`
  - `~/Downloads/nl2fl_grid.ipynb` → `docs/reference/`
- 既存の `a.txt`（空）はそのまま残す。

**Interfaces:**
- Produces:
  - `docker compose run --rm --no-deps app <cmd>` で `/work`（= `go2/`）を作業場所にしたコンテナが起動する。
  - そのとき `source ~/unitree_ros2/setup.sh` 済み、`PYTHONPATH=/work:/work/nl_control`。
  - compose のサービスは `app`（Web + 頭脳）と `sim`（にせ Go2）。同じイメージを使う。

- [ ] **Step 1: ファイルを置く**

  - zip は scratchpad に展開済みのもの（`<scratchpad>/zip_extract/go2_nl_control/`）を `nl_control/` にコピーする。展開済みのものが無ければ、zip から新しい空のフォルダに展開し直す。
  - `.gitignore` に `env.txt`、`.env`、`__pycache__/`、`out/`、`.pytest_cache/` を書く。

- [ ] **Step 2: `docker/Dockerfile` を書く**

  - **ベース：** `ros:humble-ros-base-jammy`
  - **apt：**
    - `ros-humble-rmw-cyclonedds-cpp`、`ros-humble-rosidl-generator-dds-idl`、`ros-humble-sensor-msgs-py`
    - `python3-pip`、`graphviz`、`git`、`curl`、`ca-certificates`
    - Node.js 22（NodeSource）
  - **unitree_ros2：**
    - `git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2`。ビルド時点の commit を `git rev-parse HEAD` で確かめ、`git checkout <sha>` で固定して Dockerfile に書く。
    - `cd ~/unitree_ros2/cyclonedds_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select unitree_api unitree_go`
    - Humble なので cyclonedds 自体はビルドしない（公式手順どおり）。
  - **pip（固定）：** `numpy==1.26.4`、`pitct==1.0.8`、`flloat==0.3.0`、`faster-whisper==1.2.1`、`fastapi`、`uvicorn[standard]`、`python-multipart`、`httpx`、`pytest`。fastapi・uvicorn・httpx はビルド時点の版を `==` で固定する。
  - **Claude Code：** `curl -fsSL https://claude.ai/install.sh | bash -s 2.1.303`。PATH に `~/.local/bin` を足す。
  - **Codex：** `npm install -g @openai/codex@<その時点の最新版を固定>`
  - `ENV PYTHONPATH=/work:/work/nl_control`、`WORKDIR /work`
  - ENTRYPOINT は `bash -lc 'source ~/unitree_ros2/setup.sh && exec "$@"' --`

- [ ] **Step 3: `docker/unitree_setup.sh` を書く**

  - `/opt/ros/humble/setup.bash` と `~/unitree_ros2/cyclonedds_ws/install/setup.bash` を source する。
  - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` を設定する。
  - `GO2_IFACE` が空でなければ、`CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="'$GO2_IFACE'" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'` を設定する。空なら設定しない（コンテナのネットワークに任せる）。
  - Dockerfile でこれを `~/unitree_ros2/setup.sh` として上書き配置する。

- [ ] **Step 4: `compose.yaml` を書く**

  - `app` と `sim` は同じ build（`docker/Dockerfile`）を使う。イメージ名は `image: go2-voice-ltl:dev` にする（Task 15 の Gazebo のイメージがこれを土台にする）。
  - `sim` には `profiles: ["fake"]` を付ける。`docker compose --profile fake up` で app とにせ Go2 が起動する。`docker compose up -d sim` のようにサービス名を指定すれば、profile を書かなくても起動する。
  - volumes：`./:/work`、`hf-cache:/root/.cache/huggingface`、`codex-home:/root/.codex`
  - `env_file: [{path: env.txt, required: false}]`
  - `app`：
    - `ports: ["127.0.0.1:${GO2_UI_PORT:-8810}:8000"]`
    - `environment: SIM=1`
    - `command: python3 -m voice_ltl.server`
  - `sim`：`command: python3 -m sim.fake_go2`
  - `env.txt.example` に置くキー：
    - `LLM_BACKEND=claude`
    - `LLM_MODEL=`
    - `CLAUDE_CODE_OAUTH_TOKEN=`
    - `WHISPER_MODEL=small`
    - `GO2_IFACE=`
  - `CLAUDE_CODE_OAUTH_TOKEN` という変数名は、Claude Code の公式ドキュメント（setup-token / 認証の項）で確かめ、違っていたら直す。

- [ ] **Step 5: `tests/conftest.py`**

  `/work` と `/work/nl_control` が `sys.path` に無ければ先頭に足す。

- [ ] **Step 6: ビルドして確かめる**

```bash
cd ~/work/physicalAI/go2
docker compose build
docker compose run --rm --no-deps app ros2 interface show unitree_api/msg/Request
docker compose run --rm --no-deps app python3 -c "import pitct, flloat, faster_whisper, fastapi; from flloat.parser.ltlf import LTLfParser; print(LTLfParser()('F a & G !b').to_automaton().states)"
docker compose run --rm --no-deps app bash -c "cd nl_control && python3 -m pytest -q tests"
docker compose run --rm --no-deps app bash -c "claude --version && codex --version && dot -V"
```

Expected:
- `Request` の定義（`header`、`parameter`、`binary`）が出る。
- 状態の集合が出る。
- zip のテストが `16 passed`。
- 3つの版が出る。

---

### Task 2: Go2 の単語帳と環境オートマトン（`world.py`）＋ 共通の型（`automata.py` の型だけ）

**Files:**
- Create: `voice_ltl/__init__.py`、`voice_ltl/automata.py`（この Task では型だけ）、`voice_ltl/world.py`
- Test: `tests/test_world.py`

**Interfaces:**
- Produces（`automata.py`）：
  - `World(states: list, initial, transitions: list[tuple[src, event: str, dst, kind: "c"|"u"]], marked: list, event_label: dict[str, frozenset[str]], initial_label: frozenset[str], propositions: tuple[str, ...])`
    - `states[0]` は必ず `initial`。
    - `event_label` は可制御イベントのラベル。
- Produces（`world.py`）：
  - `PlantState(posture: str, front: bool, back: bool)`（frozen dataclass、姿勢は `actions.LYING/STANDING/SITTING`）
  - `ACTION_NAMES: tuple[str, ...]`：`stand, lie_down, sit, rise_sit, hello, stretch, content, heart, scrape, dance, forward, backward, turn_left, turn_right` の14個
  - `ACTION_PROP: dict[str, str | None]`：`hello→hello, stretch→stretched, content→content, heart→heart, scrape→scraped, dance→danced, forward→moved_forward, backward→moved_backward, turn_left→turned_left, turn_right→turned_right`。`stand / lie_down / sit / rise_sit` は `None`。
  - `POSTURE_PROP = {LYING: "lying", STANDING: "standing", SITTING: "sitting"}`
  - `SENSOR_PROPS = ("obstacle_front", "obstacle_back")`
  - `PROPOSITIONS: tuple[str, ...]`：上の15個を並べたもの
  - `PROP_DESCRIPTION_JA: dict[str, str]`：プロンプトに出す説明
  - `UNCONTROLLABLE = ("front_blocked", "front_clear", "back_blocked", "back_clear")`
  - `SAFETY_FORMULA = "G(!(obstacle_front & moved_forward)) & G(!(obstacle_back & moved_backward))"`
  - `STEP_DISTANCE = 0.5`、`STEP_ANGLE = 90.0`、`MAX_REPEAT = 6`
  - `go2_label(state: PlantState, action: str | None) -> frozenset[str]`：姿勢の命題 + 動作の命題 + 立っているセンサーの命題
  - `go2_step(state: PlantState, action: str) -> tuple[str, PlantState] | None`：イベント名と移った先。できない動作なら `None`。
  - `go2_sensor_events(state: PlantState) -> list[tuple[str, PlantState]]`：その状態から起こりうる不可制御イベント（前方・後方それぞれ1本ずつ）
  - `event_name(action: str, label: frozenset[str]) -> str`：`f"{action}_to_" + "_".join(sorted(label))`
  - `event_action(event: str) -> str | None`：可制御イベントの動作名。不可制御なら `None`
  - `go2_world(initial: PlantState) -> World`：12状態、全状態が marked
  - `step_value(action: str) -> float`：forward / backward は `STEP_DISTANCE`、旋回は `STEP_ANGLE`、それ以外は 0
- Consumes：`go2_nl_control.actions` の `ACTIONS`（各動作の `transitions`）、`LYING / STANDING / SITTING`

- [ ] **Step 1: 失敗するテストを書く**

```python
def test_go2_world_counts():
    w = go2_world(PlantState(LYING, False, False))
    assert len(w.states) == 12 and w.states[0] == PlantState(LYING, False, False)
    assert len(w.transitions) == 80
    assert sum(1 for t in w.transitions if t[3] == "c") == 56
    assert sum(1 for t in w.transitions if t[3] == "u") == 24
    assert len({t[1] for t in w.transitions}) == 60
    assert set(w.marked) == set(w.states)

def test_labels_and_events():
    s = PlantState(STANDING, True, False)
    assert go2_label(s, "hello") == {"standing", "hello", "obstacle_front"}
    ev, dst = go2_step(PlantState(LYING, False, False), "stand")
    assert dst == PlantState(STANDING, False, False) and event_action(ev) == "stand"
    assert go2_step(PlantState(LYING, False, False), "hello") is None
    assert go2_step(PlantState(SITTING, False, False), "forward") is None
    assert event_action("front_clear") is None

def test_sensor_events():
    evs = dict(go2_sensor_events(PlantState(STANDING, False, True)))
    assert evs == {"front_blocked": PlantState(STANDING, True, True),
                   "back_clear": PlantState(STANDING, False, False)}

def test_vocabulary_is_safe_and_complete():
    assert len(PROPOSITIONS) == 15 and set(PROP_DESCRIPTION_JA) == set(PROPOSITIONS)
    assert not {"flip", "jump", "handstand"} & set(ACTION_NAMES)
    assert step_value("forward") == 0.5 and step_value("turn_left") == 90.0 and step_value("hello") == 0
```

- [ ] **Step 2: 失敗を確かめる**

  Run: `docker compose run --rm --no-deps app python3 -m pytest -q tests/test_world.py`
  Expected: `ModuleNotFoundError: voice_ltl`

- [ ] **Step 3: 実装する**

  - 姿勢の移り方は `ACTIONS[a].transitions` から引く（`UNKNOWN` の行は無視する）。
  - `go2_world` は、`initial` を先頭にして12状態を並べる。全状態から `go2_step` と `go2_sensor_events` で遷移を作る。

- [ ] **Step 4: 通ることを確かめる**

  Run: 同上。Expected: `4 passed`

---

### Task 3: 合成（`automata.py`：式 → SPEC → sync → trim → supcon）

**Files:**
- Modify: `voice_ltl/automata.py`
- Test: `tests/test_automata.py`（講義の 3×3 の部屋の `World` は、このテストファイルの中の `grid_world()` で作る）

**Interfaces:**
- Consumes：`World`（Task 2）、`world.go2_world`、`world.SAFETY_FORMULA`
- Produces：
  - `Automaton(name: str, size: int, trans: list[tuple[int, str, int]], marked: set[int], uncontrollable: set[str])`
  - `Synthesis(formula: str, plant: Automaton, spec: Automaton, product: Automaton, trimmed: Automaton, sup: Automaton)`
    - `empty` プロパティ：`sup.size == 0`
  - `class SpecError(Exception)`：式が読めない、または時間切れ
  - `spec_from_formula(formula: str, world: World) -> Automaton`
  - `synthesize(formula: str, world: World, workdir: Path, timeout_s: float = 20.0) -> Synthesis`

**作り方（ノートブックと同じ。`docs/reference/nl2fl_grid.ipynb` の cell 21・23・26・28・36 を写してよい）：**
- pitct の `SAVE_FOLDER` は `workdir`（`Path`）にする。`str` だと失敗する（ノートブック cell 21 の注記）。
- PLANT：`create("PLANT", n, [(id(src), ev, id(dst), kind)], marked_ids)`
- SPEC：
  - `q_start = dfa_step(dfa.initial_state, world.initial_label)` を状態0にする。
  - 可制御イベント `e` は `dfa_step(q, world.event_label[e])` へ移す。
  - 不可制御イベントは自己ループで、kind `"u"`。
  - 受理状態を marked にする。
  - SPEC に与えるイベントは、PLANT に出てくるものだけにする。
- `sync("PRODUCT", "PLANT", "SPEC")`、`trim("TRIMMED", "PRODUCT")`、`supcon("SUP", "PRODUCT", "TRIMMED")`
- 結果は `trans()`・`marker()`・`statenum()` で読み戻して `Automaton` にする。
- 時間切れは `timeout_s` を超えたら `SpecError("式が大きすぎます")`。flloat の変換を別プロセス（`multiprocessing`）で走らせて測る。
- Go2 で使うときは、呼ぶ側（Task 9）が `f"({formula}) & {SAFETY_FORMULA}"` を渡す。この関数自体は渡された式をそのまま使う。

- [ ] **Step 1: 失敗するテストを書く（講義の数値）**

```python
def test_lecture_grid_numbers(tmp_path):
    s = synthesize("F at_goal & G !collide", grid_world(), tmp_path)
    assert (s.plant.size, len(s.plant.trans)) == (9, 21)
    assert (s.spec.size, len(s.spec.trans)) == (3, 30)
    assert (s.product.size, len(s.product.trans)) == (17, 38)
    assert (s.trimmed.size, len(s.trimmed.trans)) == (8, 14)
    assert (s.sup.size, len(s.sup.trans)) == (8, 14)

def test_go2_unsatisfiable_is_empty(tmp_path):
    w = go2_world(PlantState(LYING, False, False))
    assert synthesize(f"(F sitting & G !sitting) & {SAFETY_FORMULA}", w, tmp_path).empty

def test_go2_uncontrollable_kept(tmp_path):
    w = go2_world(PlantState(LYING, False, False))
    s = synthesize(f"(F hello) & {SAFETY_FORMULA}", w, tmp_path)
    assert {"front_blocked", "front_clear"} <= {e for _, e, _ in s.sup.trans}
    assert not s.empty

def test_six_repeats_within_budget(tmp_path):
    f = "F(hello & X F(hello & X F(hello & X F(hello & X F(hello & X F hello)))))"
    s = synthesize(f"({f}) & {SAFETY_FORMULA}", go2_world(PlantState(LYING, False, False)), tmp_path)
    assert not s.empty

def test_bad_formula_raises(tmp_path):
    with pytest.raises(SpecError):
        synthesize("F (hello", go2_world(PlantState(LYING, False, False)), tmp_path)
```

  `grid_world()` はノートブック cell 3・5・19・21 の部屋をそのまま `World` にしたもの。
  - 状態は「マス」、イベントは `f"{a}_to_{'free' or '_'.join(label)}"`、kind はすべて `"c"`。
  - ゴールは吸収状態で、marked はゴールだけ。
  - 初期状態は (0,1)。
  - `propositions = ("at_goal", "collide")`

- [ ] **Step 2: 失敗を確かめる**

  Run: `... pytest -q tests/test_automata.py`。Expected: `ImportError: synthesize`

- [ ] **Step 3: 実装する**（上の「作り方」どおり）

- [ ] **Step 4: 通ることを確かめる。** 6回の繰り返しのテストにかかった秒数も控えておく

  Run: `... pytest -q tests/test_automata.py --durations=5`
  Expected: `5 passed`。6回の繰り返しが 20 秒以内。
  20 秒を超えるなら、`MAX_REPEAT` を通る最大の回数に下げ、設計書 §7.4.0 とプロンプトの例（Task 5）もその値に合わせる。

---

### Task 4: 監督器の上で次の一手を選ぶ（`planner.py`）

**Files:**
- Create: `voice_ltl/planner.py`
- Test: `tests/test_planner.py`

**Interfaces:**
- Consumes：`Automaton`（`synthesize(...).sup`）
- Produces：
  - `class PlanError(Exception)`
  - `Runtime(sup: Automaton)`
    - `.state: int`（初期 0）
    - `.done -> bool`（今の状態が marked か）
  - `Runtime.plan(avoid: frozenset[tuple[int, str]] = frozenset()) -> list[str] | None`
    - 今の状態から marked まで、**可制御イベントだけ**を通る最短の列（幅優先）を返す。
    - 道が無ければ `None`、もう着いているなら `[]`。
    - `avoid` は (状態, イベント) の組で、その組の遷移を使わない。
  - `Runtime.next_event() -> str | None`：`plan()` の先頭
  - `Runtime.apply(event: str) -> None`：今の状態に `event` の遷移が無ければ `PlanError`
  - `Runtime.allowed() -> list[str]`：今の状態から出ているイベント

- [ ] **Step 1: 失敗するテストを書く**

```python
def test_lecture_plan_and_reroute(tmp_path):
    rt = Runtime(synthesize("F at_goal & G !collide", grid_world(), tmp_path).sup)
    assert [a for a in map(grid_action, rt.plan())] == ["up", "right", "right", "down"]
    rt.apply(rt.next_event())                       # up → (0,2)
    blocked = frozenset({(rt.state, "right_to_free")})  # 次に入る (1,2) にカバン
    assert len(rt.plan(avoid=blocked)) == 5

@pytest.mark.parametrize("formula,actions", [
    ("F hello", ["stand", "hello"]),
    ("F(hello & F danced)", ["stand", "hello", "dance"]),
    ("(!sitting U hello) & F sitting", ["stand", "hello", "sit"]),
    ("G !danced & F stretched", ["stand", "stretch"]),
    ("F(hello & X F(hello & X F hello))", ["stand", "hello", "hello", "hello"]),
    ("F(moved_forward & X F moved_forward)", ["stand", "forward", "forward"]),
])
def test_go2_plans(tmp_path, formula, actions):
    rt = Runtime(go2_sup(tmp_path, formula, PlantState(LYING, False, False)))
    assert [event_action(e) for e in rt.plan()] == actions

def test_waits_when_front_blocked_then_resumes(tmp_path):
    rt = Runtime(go2_sup(tmp_path, "F moved_forward", PlantState(LYING, True, False)))
    assert rt.next_event() is None
    rt.apply("front_clear")
    assert [event_action(e) for e in rt.plan()] == ["stand", "forward"]

def test_forward_never_allowed_while_blocked(tmp_path):
    rt = Runtime(go2_sup(tmp_path, "F moved_forward", PlantState(STANDING, True, False)))
    assert not any(event_action(e) == "forward" for e in rt.allowed())

def test_apply_unknown_event_raises(tmp_path):
    rt = Runtime(go2_sup(tmp_path, "F hello", PlantState(LYING, False, False)))
    with pytest.raises(PlanError):
        rt.apply("hello_to_hello_standing")   # 伏せのまま挨拶はできない
```

  補助関数：
  - `go2_sup(tmp_path, f, init)` = `synthesize(f"({f}) & {SAFETY_FORMULA}", go2_world(init), tmp_path).sup`
  - `grid_action(e)` は `e.split("_to_")[0]`
  - `grid_world` は `tests/test_automata.py` から import する（`conftest.py` に移してもよい）。

- [ ] **Step 2: 失敗を確かめる**

  Run: `... pytest -q tests/test_planner.py`。Expected: ImportError

- [ ] **Step 3: 実装する**

  - 不可制御イベントは `sup.uncontrollable` で見分ける。
  - 同じ距離の道が複数あるときは、イベント名の辞書順で先のものを選ぶ（結果を毎回同じにするため）。

- [ ] **Step 4: 通ることを確かめる**

  Run: 同上。Expected: `10 passed`

---

### Task 5: 文法の検査とプロンプトと聞き直し（`grammar.py`・`prompt.py`・`translator.py`）

**Files:**
- Create: `voice_ltl/grammar.py`、`voice_ltl/prompt.py`、`voice_ltl/translator.py`
- Test: `tests/test_grammar_translator.py`

**Interfaces:**
- Consumes：`world.PROPOSITIONS`、`PROP_DESCRIPTION_JA`、`MAX_REPEAT`、`STEP_DISTANCE`、`STEP_ANGLE`
- Produces：
  - `grammar.MAX_FORMULA_LEN = 300`
  - `grammar.check(formula: str, propositions: Sequence[str]) -> list[str]`
    - 日本語の却下理由を返す。空なら合格。
    - 見る点：空でない、300 文字以下、括弧の対応、知らない名前が無い、flloat で読める。
    - 使ってよい名前：命題、`F`、`G`、`X`、`U`、`last`
    - 使ってよい記号：`!`、`&`、`|`、`->`、`(`、`)`
    - **式は決して直さない。**
  - `prompt.SCHEMA`：`{"type": "object", "properties": {"formula": {"type": ["string", "null"]}, "unsupported": {"type": "array", "items": {"type": "string"}}}, "required": ["formula", "unsupported"], "additionalProperties": false}`
  - `prompt.build_prompt(instruction: str, rejected: str | None = None, error: str | None = None) -> str`
  - `translator.Attempt(formula: str | None, unsupported: list[str], errors: list[str])`
  - `translator.Translation(ok: bool, formula: str | None, unsupported: list[str], attempts: list[Attempt], message: str)`
  - `translator.translate(instruction: str, llm: LLM, max_tries: int = 3) -> Translation`
    - `llm` は Task 6 の `LLM` 型（`(prompt, schema) -> dict`）。
    - `LLMError` はここでは捕まえず、呼ぶ側（Task 9）に任せる。

**プロンプトの決まり（設計書 §7.3・§7.4.0・§7.4.1）：**
- 講義の4パートの順に並べる。
  - **Goal：** 指示を LTLf の式に翻訳する
  - **Grammar：** 上の記号。`X` は「次の動作」、`last` は「最後の時点」と説明する
  - **Vocabulary：** 命題と日本語の説明
  - **Examples：** 次の5つ
    - 「立って挨拶して」→ `F hello`
    - 「挨拶してからダンスして」→ `F(hello & F danced)`
    - 「挨拶するまで座らないで」→ `!sitting U hello`
    - 「1メートル進んで」→ `F(moved_forward & X F moved_forward)`
    - 「キッチンへ行って」→ `formula: null, unsupported: ["キッチン"]`
  - **Query：** 変換したい指示
- 「単語帳で表せない部分は、近い言葉に置き換えず `unsupported` に入れて、`formula` を null にする」と書く。
- 「数は 0.5 m・90 度・1回を単位とした繰り返しで表す。繰り返しは `MAX_REPEAT` 回まで。割り切れない数や、それを超える数は `unsupported` に入れる」と書く。
- 聞き直しでは、末尾に「前の答えは {rejected} で、{error} という理由で却下されました。直した式だけを返してください。」を足す。

- [ ] **Step 1: 失敗するテストを書く**

```python
def test_check_accepts_and_rejects():
    P = PROPOSITIONS
    assert check("F(hello & X F hello) & G !danced", P) == []
    assert check("(!sitting U hello) & F(last & sitting)", P) == []
    assert any("括弧" in e for e in check("F (hello", P))
    assert any("at_kitchen" in e for e in check("F at_kitchen", P))
    assert check("", P) and check("F hello " + "& F hello " * 40, P)

def test_check_never_repairs():
    f = "F (hello"
    check(f, PROPOSITIONS); assert f == "F (hello"

def test_translate_first_try():
    llm = FakeLLM([{"formula": "F hello", "unsupported": []}])
    t = translate("立って挨拶して", llm)
    assert t.ok and t.formula == "F hello" and len(t.attempts) == 1

def test_translate_reprompts_with_reason_then_gives_up():
    llm = FakeLLM([{"formula": "F at_kitchen", "unsupported": []}] * 3)
    t = translate("台所へ", llm)
    assert not t.ok and len(t.attempts) == 3
    assert "at_kitchen" in llm.prompts[1] and "却下" in llm.prompts[1]

def test_unsupported_stops_without_retry():
    llm = FakeLLM([{"formula": None, "unsupported": ["キッチン"]}])
    t = translate("キッチンへ行って", llm)
    assert not t.ok and t.unsupported == ["キッチン"] and len(llm.prompts) == 1
    assert "キッチン" in t.message and "単語帳" in t.message

def test_prompt_has_four_parts():
    p = build_prompt("挨拶して")
    for part in ("Grammar", "Vocabulary", "Examples", "挨拶して", "unsupported"):
        assert part in p
```

  `FakeLLM` は Task 6 で作る。この Task では `tests/test_grammar_translator.py` の中に、同じ形（`replies` を順に返し、`prompts` に記録する）の仮のものを書く。Task 6 のあとで `voice_ltl.llm.FakeLLM` の import に置き換える。

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError

- [ ] **Step 3: 実装する**

  - `check` の名前の判定は、`re.findall(r"[A-Za-z_][A-Za-z0-9_]*", f)` で取った名前が「命題 ∪ {F, G, X, U, last}」に入っているかで見る。
  - flloat で読めるかは `LTLfParser()(f)` が例外を出さないかで見る。

- [ ] **Step 4: 通ることを確かめる。** Expected: `6 passed`

---

### Task 6: LLM の窓口（`llm.py`：Claude CLI / Codex CLI / 偽物）

**Files:**
- Create: `voice_ltl/llm.py`
- Test: `tests/test_llm.py`

**Interfaces:**
- Produces：
  - `LLM = Callable[[str, dict], dict]`（プロンプトと JSON Schema → 読み込んだ JSON）
  - `class LLMError(Exception)`：利用者に見せる日本語の短い文
  - `ClaudeCLI(model: str | None = None, timeout_s: float = 60.0, runner=subprocess.run)`
  - `CodexCLI(model: str | None = None, timeout_s: float = 60.0, runner=subprocess.run)`
  - `FakeLLM(replies: list[dict | Exception], delay_s: float = 0.0)`。`.prompts: list[str]` に受け取ったプロンプトを記録する。
  - `make_llm(env: Mapping[str, str]) -> LLM`
    - `LLM_BACKEND` が `claude`（既定）なら `ClaudeCLI`、`codex` なら `CodexCLI`、それ以外なら `ValueError`。
    - `LLM_MODEL` が空でなければ `model` に渡す。

**呼び方：**
- **Claude：**
  - argv：`["claude", "-p", "--output-format", "json", "--json-schema", json.dumps(schema), "--tools", "", "--no-session-persistence"]`。`model` があれば `["--model", model]` を足す。
  - プロンプトは標準入力で渡す。
  - 返ってきた JSON に `is_error` が真ならエラー。構造化出力は `structured_output` から取り、無ければ `result` の文字列を JSON として読む。
  - 実際のフィールド名は Step 5 で本物を呼んで確かめ、違っていたら直す。
- **Codex：**
  - 一時フォルダに schema を書く。
  - argv：`["codex", "exec", "--skip-git-repo-check", "--ephemeral", "--sandbox", "read-only", "--output-schema", <schema.json>, "-o", <out.json>]`。`model` があれば `codex exec --help` で確かめたモデル指定のフラグを足す。最後にプロンプトを渡す（標準入力か引数かは `codex exec --help` で確かめる）。
  - `out.json` を読む。
- **共通：**
  - 終了コードが 0 以外、時間切れ、JSON が読めない、schema の形でない場合は、どれも `LLMError` にする。
  - 文言：
    - ログインしていない：「LLM にログインできていません（README の手順でログインしてください）」
    - 時間切れ：「LLM の返事が {timeout} 秒で返ってきませんでした」
    - 壊れた出力：「LLM の返事が読めませんでした」
  - 標準エラーの中身は、ログに最初の 300 文字だけ残す。トークンらしき文字列（`sk-`、`oauth` を含む行）は残さない。

- [ ] **Step 1: 失敗するテストを書く（本物の CLI は呼ばない）**

```python
def fake_run(stdout="", returncode=0, stderr=""):
    calls = []
    def run(argv, **kw):
        calls.append((argv, kw)); return subprocess.CompletedProcess(argv, returncode, stdout, stderr)
    return run, calls

def test_claude_argv_and_parse():
    run, calls = fake_run(json.dumps({"is_error": False, "structured_output": {"formula": "F hello", "unsupported": []}}))
    out = ClaudeCLI(runner=run)("PROMPT", SCHEMA)
    argv, kw = calls[0]
    assert argv[:2] == ["claude", "-p"] and "--json-schema" in argv and "--bare" not in argv
    assert argv[argv.index("--tools") + 1] == "" and kw["input"] == "PROMPT"
    assert out == {"formula": "F hello", "unsupported": []}

def test_claude_cli_failure_raises_llm_error():
    run, _ = fake_run(returncode=1, stderr="Invalid API key · Please run /login")
    with pytest.raises(LLMError, match="ログイン"):
        ClaudeCLI(runner=run)("p", SCHEMA)
    run, _ = fake_run(stdout="not json")
    with pytest.raises(LLMError, match="読めません"):
        ClaudeCLI(runner=run)("p", SCHEMA)

def test_timeout_is_llm_error():
    def run(argv, **kw): raise subprocess.TimeoutExpired(argv, 60)
    with pytest.raises(LLMError, match="60"):
        ClaudeCLI(runner=run)("p", SCHEMA)

def test_codex_argv(tmp_path):
    # runner が -o のファイルに答えを書いたことにする
    def run(argv, **kw):
        Path(argv[argv.index("-o") + 1]).write_text(json.dumps({"formula": "F hello", "unsupported": []}))
        return subprocess.CompletedProcess(argv, 0, "", "")
    assert CodexCLI(runner=run)("p", SCHEMA)["formula"] == "F hello"

def test_make_llm():
    assert isinstance(make_llm({}), ClaudeCLI)
    assert isinstance(make_llm({"LLM_BACKEND": "codex"}), CodexCLI)
    with pytest.raises(ValueError):
        make_llm({"LLM_BACKEND": "gpt"})
```

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError

- [ ] **Step 3: 実装する**

  `ClaudeCLI` の「ログインしていない」判定は、終了コードが 0 以外で、標準エラーか標準出力に `login`、`auth`、`API key` のどれかを含むとき。

- [ ] **Step 4: 通ることを確かめる。** Expected: `5 passed`。Task 5 の仮の FakeLLM を `voice_ltl.llm.FakeLLM` に置き換え、`tests/test_grammar_translator.py` も通ることを確かめる。

- [ ] **Step 5: 本物の Claude で形を確かめる（Mac 上。トークンは使わない）**

```bash
cd ~/work/physicalAI/go2
echo '「立って挨拶して」を LTL にして' | claude -p --output-format json --json-schema '{"type":"object","properties":{"formula":{"type":["string","null"]},"unsupported":{"type":"array","items":{"type":"string"}}},"required":["formula","unsupported"]}' --tools "" --no-session-persistence | python3 -c "import sys,json; d=json.load(sys.stdin); print(sorted(d)); print(d.get('structured_output'))"
```

  Expected：キーの一覧に構造化出力の入ったフィールドがある。名前が `structured_output` でなければ、`ClaudeCLI` とテストを本物に合わせて直す。
  Codex はこの Mac に入っていないので、ここでは確かめない。README に「未確認」と書く（Task 14）。

---

### Task 7: 前方・後方のセンサー判定（`sensing.py`）

**Files:**
- Create: `voice_ltl/sensing.py`
- Test: `tests/test_sensing.py`

**Interfaces:**
- Consumes：`go2_nl_control.perception.ObstacleMonitor`、`PerceptionConfig`、`go2_nl_control.supervisor.CLEAR / BLOCKED / SENSOR_LOST`
- Produces：
  - `Reading(front: str, back: str, front_dist: float | None, back_dist: float | None)`
    - `front_blocked` プロパティ：`front != CLEAR`（途切れも「あり」として安全側に倒す）
    - `back_blocked` プロパティ：同じく後方
  - `Sensing(cfg: PerceptionConfig)`
  - `Sensing.update(points_lidar: np.ndarray, now: float) -> Reading`
  - `Sensing.current(now: float) -> Reading`

**作り方：**
- 前方は `ObstacleMonitor(cfg)` をそのまま使う。
- 後方は、前方のモニタの `_to_body()` で機体座標にした点を `(x, y) → (-x, -y)` に反転する。それを、取り付けを0にした設定 `replace(cfg, lidar_x=0, lidar_z=0, lidar_pitch=0, lidar_yaw=0)` の2つ目の `ObstacleMonitor` に渡す。こうすると判定の中身（ヒステリシス、外れ値対策、途切れ）が前方と同じになる。

- [ ] **Step 1: 失敗するテストを書く**

```python
def box_points(cx, cfg):           # 機体座標で x=cx にある板を、LiDAR 座標に戻した点群
    ...                            # perception._to_body の逆変換（テスト内の補助関数）

def test_front_and_back_hysteresis():
    cfg = PerceptionConfig(); s = Sensing(cfg)
    assert s.update(box_points(0.45, cfg), 0.0).front == BLOCKED
    assert s.update(box_points(0.60, cfg), 0.1).front == BLOCKED      # 0.7 m までは戻らない
    assert s.update(box_points(0.75, cfg), 0.2).front == CLEAR
    r = s.update(box_points(-0.45, cfg), 0.3)
    assert r.back == BLOCKED and r.front == CLEAR and abs(r.back_dist - 0.45) < 0.03

def test_stale_is_lost_and_counts_as_blocked():
    s = Sensing(PerceptionConfig()); s.update(np.zeros((0, 3)), 0.0)
    r = s.current(0.6)
    assert r.front == SENSOR_LOST and r.front_blocked and r.back_blocked
```

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError
- [ ] **Step 3: 実装する**
- [ ] **Step 4: 通ることを確かめる。** Expected: `2 passed`

---

### Task 8: シミュレーションの Go2 の中身（`sim/sim_core.py`）

**Files:**
- Create: `sim/__init__.py`、`sim/sim_core.py`
- Test: `tests/test_sim_core.py`

**Interfaces:**
- Consumes：`go2_nl_control.actions` の `api_id` 定数、`PerceptionConfig`
- Produces：
  - `Box(x: float, y: float, size: float = 0.3)`
  - `SimGo2(perception: PerceptionConfig = PerceptionConfig(), posture: str = LYING)`
    - 属性：`.x, .y, .yaw, .posture: str, .error_code: int, .collided: bool, .boxes: list[Box], .accepted: list[int]`（受け付けた `api_id` の記録）
  - `SimGo2.handle(api_id: int, parameter: str, now: float) -> None`
  - `SimGo2.step(now: float) -> None`
  - `SimGo2.place_box(where: Literal["front", "back"], distance: float = 1.2) -> Box`
  - `SimGo2.clear_boxes() -> None`
  - `SimGo2.cloud_lidar() -> np.ndarray`（N×3、LiDAR 座標）
  - 定数：`FOOTPRINT = 0.35`、`MOVE_HOLD_S = 1.0`

**決めた値（実機に寄せる。`actions.py` の `_ERROR_CODE_TO_STATE` と合わせる）：**

| 指令 | 受け付ける条件 | 変化 |
|---|---|---|
| StandUp 1004 | 伏せ | 1.5 秒後に `error_code` 1002（StandLock、姿勢は判定上 UNKNOWN） |
| BalanceStand 1002 | 1002 か 1013 | 0.5 秒後に 1013（立ち） |
| StandDown 1005 | 立ち | 1.5 秒後に 1004（伏せ） |
| Sit 1009 | 立ち | 2.0 秒後に 1007（おすわり） |
| RiseSit 1010 | おすわり | 1.5 秒後に 1013 |
| Hello・Stretch・Content・Dance1・Scrape・Heart | 立ち | 3.0 秒の間は他の指令を受け付けない |
| Move 1008 | 1013 か 1015 | 速度を設定し、`error_code` 1015。`MOVE_HOLD_S` の間、送り直しが無ければ速度0・1013 |
| StopMove 1003 | いつでも | 速度0 |

- 動作中（上の「〜秒後」を待っている間）に来た指令は受け付けない（`accepted` に入れない）。
- 位置は、機体の向きに沿って速度を積分する。
- 中心から箱（正方形）までの距離が `FOOTPRINT` 未満になる動きは打ち切り、`collided = True` にする。
- 点群は、箱の4つの側面を 0.05 m 間隔、高さ 0〜0.4 m で点にする。それを機体座標（地面は z = -0.30）にし、`perception._to_body` の逆変換で LiDAR 座標にする。

- [ ] **Step 1: 失敗するテストを書く**

```python
def run(sim, t0, t1, dt=0.1):
    t = t0
    while t < t1: sim.step(t); t += dt
    return t

def test_stand_sequence_and_ignore_invalid():
    sim = SimGo2(); sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    assert MOVE not in sim.accepted                       # 伏せたまま歩けない
    sim.handle(STAND_UP, "", 0.0); run(sim, 0, 1.6)
    sim.handle(BALANCE_STAND, "", 1.6); run(sim, 1.6, 2.2)
    assert sim.error_code == 1013 and state_from_error_code(sim.error_code) == STANDING

def test_move_expires_without_resend():
    sim = standing_sim(); sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0); run(sim, 0, 2.0)
    assert 0.25 < sim.x < 0.35                            # 1 秒だけ進んで止まる

def test_box_appears_in_cloud_and_blocks_motion():
    sim = standing_sim(); sim.place_box("front", 1.2)
    s = Sensing(PerceptionConfig()); assert s.update(sim.cloud_lidar(), 0).front_dist == pytest.approx(1.05, abs=0.05)
    t = 0.0
    while t < 10: sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t); t = run(sim, t, t + 0.1)
    assert sim.collided and sim.x < 1.2 - 0.15 - FOOTPRINT + 0.05
```

  `standing_sim()` は、立った状態（1013）の `SimGo2` を作るテスト内の補助関数。箱の近い面は中心から 1.2 − 0.15 = 1.05 m。

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError
- [ ] **Step 3: 実装する**
- [ ] **Step 4: 通ることを確かめる。** Expected: `3 passed`

---

### Task 9: 全体の進行役（`session.py`）と、ROS なしの受け入れテスト

**Files:**
- Create: `voice_ltl/session.py`
- Test: `tests/test_session.py`

**Interfaces:**
- Consumes：
  - Task 2〜8 のすべて
  - `go2_nl_control.executor.Executor / ExecConfig / Command / STOP`
  - `go2_nl_control.supervisor.Step`
  - `go2_nl_control.actions.ACTIONS / state_from_error_code`
  - `go2_nl_control.translator.is_stop_command`
- Produces：
  - `class Link(Protocol)`：
    - `send(cmd: Command) -> None`
    - `posture() -> str`
    - `pose() -> tuple[float, float, float] | None`
    - `reading(now: float) -> Reading`
  - `DirectSimLink(sim: SimGo2, perception: PerceptionConfig = PerceptionConfig())`
    - テスト用。`.sim` で中の `SimGo2` を見せる。
    - `send` は `sim.handle`、`advance(now)` は `sim.step` と点群の更新、`reading` は `Sensing` を使う。
  - `Session(link: Link, llm: LLM, transcribe: Callable[[bytes], str] | None, workdir: Path, background: bool = True)`
  - `Session.submit_audio(audio: bytes) -> None`
  - `Session.submit_text(text: str) -> None`
  - `Session.stop() -> None`
  - `Session.tick(now: float) -> None`（制御ループから 10 Hz で呼ぶ）
  - `Session.snapshot() -> dict`
    - キー：`status`, `transcript`, `attempts`, `formula`, `unsupported`, `message`, `plan`, `current`, `posture`, `pose`, `front`, `back`, `front_dist`, `back_dist`, `sup_state`, `diagram_version`, `log`
  - `status` の値：`"idle" | "transcribing" | "thinking" | "executing" | "waiting" | "done" | "error"`
  - `Session.synthesis: Synthesis | None`、`Session.runtime: Runtime | None`（Task 10・13 が読む）
  - `EXEC_ACTIONS: dict[str, ActionSpec]`：`ACTIONS` の写し。`backward` だけ `forward=True` に変える（`dataclasses.replace`）。

**決めたふるまい：**
1. **`submit_text(text)`：**
   - 空白だけなら、`message` を「聞き取れませんでした」にして終わる。
   - `is_stop_command(text)` なら、`stop()` を呼んで終わる。
   - それ以外は、世代番号を1つ増やし、実行中なら止める（StopMove を送り、計画を捨てる）。状態を `thinking` にする。
   - そのうえで、別スレッド（`background=False` なら同じスレッド）で次を行う。
     1. 翻訳する。
     2. `go2_world(初期状態)` で合成する。式は `f"({formula}) & {SAFETY_FORMULA}"`。
   - 初期状態は今の姿勢と `reading.front_blocked / back_blocked` から作る。姿勢が `UNKNOWN` なら `LYING` とみなす（StandUp → BalanceStand はどこからでも立てるため）。
   - 結果はキューに入れ、`tick` で受け取る。世代番号が古い結果は捨てる。
2. **`submit_audio(audio)`：** `status = transcribing` にして、`transcribe(audio)` の結果で `submit_text` と同じ流れに入る。
3. **伝える文言：**
   - `Translation.ok` が偽：`translation.message`
   - 合成結果が空：「この指示は満たせません」
   - `SpecError`：その文
   - `LLMError`：その文
   - どの場合も Go2 は動かさず、`status = error`。
4. **`tick(now)`（実行中）：**
   1. `reading` の前方・後方の「あり / なし」が前回と変わったら、`front_blocked / front_clear / back_blocked / back_clear` を `runtime.apply` し、ログに残す。
   2. 実行器が空なら、次の可制御イベントを選ぶ。
      - `runtime.done` なら `done` にする。
      - `next_event()` が `None` なら `waiting` にする。15 秒続いたら「〇〇秒待っても進めません」で中止する。
      - それ以外は、動作を読み出して `Step(action, step_value(action))` を1つだけ `executor.load` する。
   3. 次の動作を送る前に、`link.posture()` を「計画上の今の姿勢」と比べる。
      - 計画上の姿勢は、完了した動作の `ACTIONS[a].transitions` で進める。
      - 違っていたら「姿勢が計画と違います（計画: 〜 / 実際: 〜）」で中止する。
   4. 実行器の環境の文字列は、今の動作で決める。
      - `forward` なら前方の状態、`backward` なら後方の状態。
      - それ以外は、どちらかが `SENSOR_LOST` なら `SENSOR_LOST`、そうでなければ `CLEAR`。
   5. `executor.tick` が返した指令を、すべて `link.send` する。
   6. 完了の判定：`len(executor.steps) == 1 and executor.idx == 1` なら完了。今のセンサーの「あり / なし」で `go2_step` を使ってイベント名を求め、`runtime.apply` する。
   7. 中止の判定：`len(executor.steps) == 0` なのに完了していなければ中止。計画を捨て、実行器の最後の報告を `message` に入れる。
5. **`stop()`：** StopMove を1回送り、計画を捨てて `idle` にする。翻訳の途中なら、世代番号を増やして結果を捨てる。

- [ ] **Step 1: 失敗するテストを書く**（補助関数 `drive(session, link, seconds, dt=0.1)` は `now` を進めながら `link.advance(now)` → `session.tick(now)` を繰り返す。`llm_for(*formulas)` は `FakeLLM` を返す）

```python
@pytest.mark.parametrize("formula,expect", [
    ("F hello", [STAND_UP, BALANCE_STAND, HELLO]),
    ("F(hello & F danced)", [STAND_UP, BALANCE_STAND, HELLO, DANCE1]),
    ("(!sitting U hello) & F sitting", [STAND_UP, BALANCE_STAND, HELLO, SIT]),
    ("G !danced & F stretched", [STAND_UP, BALANCE_STAND, STRETCH]),
    ("F(hello & X F(hello & X F hello))", [STAND_UP, BALANCE_STAND, HELLO, HELLO, HELLO]),
])
def test_acceptance_gestures(tmp_path, formula, expect):
    sess, link = make(tmp_path, llm_for(formula))
    sess.submit_text("指示"); drive(sess, link, 60)
    assert [a for a in link.sim.accepted if a != MOVE and a != STOP_MOVE] == expect
    assert sess.snapshot()["status"] == "done"

def test_two_meters_stops_for_box_then_continues(tmp_path):
    sess, link = make(tmp_path, llm_for("F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward)))"))
    sess.submit_text("2メートル進んで"); drive(sess, link, 6)          # 立って歩き始める
    link.sim.place_box("front", 1.2); drive(sess, link, 10)
    assert not link.sim.collided and sess.snapshot()["status"] == "executing"
    assert link.sim.accepted[-1] == STOP_MOVE                           # 途中で止まった
    link.sim.clear_boxes(); drive(sess, link, 20)
    assert sess.snapshot()["status"] == "done" and link.sim.x == pytest.approx(2.0, abs=0.15)

def test_box_never_cleared_aborts_after_15s(tmp_path): ...   # 「秒待っても」を含む message、status error
def test_back_box_holds_backward(tmp_path): ...             # F moved_backward、後方に箱 → 後退しない、どければ下がる
def test_stop_word_stops_immediately(tmp_path): ...         # 実行中に submit_text("止まって") → 直後の送信が STOP_MOVE、LLM の呼び出しは増えない、status idle
def test_new_instruction_replaces_plan(tmp_path): ...       # hello 実行中に別の指示 → StopMove のあと新しい計画
def test_unsupported_and_unsatisfiable_do_not_move(tmp_path): ...  # unsupported → 動かない。"F sitting & G !sitting" → 「満たせません」、accepted が空
def test_empty_transcript_does_not_call_llm(tmp_path): ...  # submit_text("  ") → prompts が空、message「聞き取れませんでした」
def test_newer_instruction_wins_over_slow_translation(tmp_path): ...  # background=True、FakeLLM(delay_s=0.5)。1つ目の直後に2つ目 → 実行されるのは2つ目の計画だけ
def test_llm_failure_reports_and_stays_idle(tmp_path): ...  # FakeLLM([LLMError("LLM にログインできていません")]) → status error、accepted が空
def test_posture_mismatch_aborts(tmp_path): ...             # F(hello & F danced) で hello 完了後に sim を強制的に伏せ（error_code 1004）→ 「姿勢が計画と違います」、DANCE1 は送られない
```

  `...` の行は、それぞれのコメントに書いた条件を assert する。どれも `make(tmp_path, llm)` = `Session(DirectSimLink(SimGo2()), llm, None, tmp_path, background=False)` で作る。ただし遅い翻訳のテストだけは `background=True`。

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError
- [ ] **Step 3: 実装する**（「決めたふるまい」どおり）
- [ ] **Step 4: 通ることを確かめる**

  Run: `... pytest -q tests/test_session.py`。Expected: `15 passed`

  - 前進の途中で止まっている間（実行器の一時停止中）も、`status` は `executing` のままにする。
  - `waiting` は「許される可制御イベントが今ない」ときだけに使う。

- [ ] **Step 5: ここまでの全部を通す**

  Run: `docker compose run --rm --no-deps app python3 -m pytest -q tests`。Expected：失敗 0

---

### Task 10: 状態遷移図の画像（`diagrams.py`）

**Files:**
- Create: `voice_ltl/diagrams.py`
- Test: `tests/test_diagrams.py`

**Interfaces:**
- Consumes：`Automaton`、`world.event_action`
- Produces：`render_svg(a: Automaton, title: str, highlight_state: int | None = None, highlight_events: Sequence[tuple[int, str]] = ()) -> str`（SVG の文字列）

**決めた描き方：**
- graphviz の `dot` を使う（python の `graphviz` パッケージ、pitct の依存で入っている）。
- 同じ2つの状態の間の矢印は1本にまとめ、ラベルには動作名（`event_action`）を重複なしで並べる。4つを超えたら「ほか N 件」と書く。
- 不可制御イベントは点線。marked の状態は二重丸。
- `highlight_state` は塗りつぶし、`highlight_events` は太い色付きの線にする（`class="hl"` を付ける）。
- 状態が 0 の Automaton は「（空：この指示は満たせません）」とだけ書いた SVG を返す。

- [ ] **Step 1: 失敗するテストを書く**

```python
def test_svg_merges_edges_and_highlights(tmp_path):
    s = synthesize(f"(F hello) & {SAFETY_FORMULA}", go2_world(PlantState(LYING, False, False)), tmp_path)
    rt = Runtime(s.sup)
    svg = render_svg(s.sup, "SUP", highlight_state=0, highlight_events=[(0, rt.plan()[0])])
    assert svg.startswith("<?xml") or svg.lstrip().startswith("<svg")
    assert "stand" in svg and 'class="hl"' in svg and "dashed" in svg

def test_empty_automaton_svg():
    assert "満たせません" in render_svg(Automaton("SUP", 0, [], set(), set()), "SUP")
```

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError
- [ ] **Step 3: 実装する**
- [ ] **Step 4: 通ることを確かめる。** Expected: `2 passed`
- [ ] **Step 5: `Session` につなぐ**
  - 合成のあとと、`runtime.apply` のたびに `session.diagrams: dict[str, str]`（キー `PLANT, SPEC, TRIMMED, SUP`）を作り直し、`diagram_version` を1つ増やす。
  - SUP は今の状態と、残りの計画の矢印を色付きにする。PLANT・SPEC・TRIMMED は合成のときだけ描く。
  - 描くのは翻訳と同じ別スレッド、または制御ループの外で行う。制御ループを 100 ミリ秒以上止めないこと。
  - Task 9 のテストがまだ通ることを確かめる。

---

### Task 11: 書き起こし（`stt.py`）

**Files:**
- Create: `voice_ltl/stt.py`、`tests/fixtures/stand_hello.wav`
- Test: `tests/test_stt.py`

**Interfaces:**
- Produces：`Transcriber(model_size: str = "small", compute_type: str = "int8")`、`Transcriber.__call__(audio: bytes) -> str`
  - 音声の形式はブラウザの `audio/webm;codecs=opus` と wav の両方を受ける。faster-whisper に `io.BytesIO` を渡すと PyAV で読める。
  - `language="ja"`、`vad_filter=False`（押している間だけ録るため）、`beam_size=5`。
  - 結果は文をつなげて前後の空白を取る。
- モデルは初回に Hugging Face から `hf-cache` ボリュームへ落ちてくる。

- [ ] **Step 1: 試験用の音声を作る（Mac 上）**

```bash
cd ~/work/physicalAI/go2 && mkdir -p tests/fixtures
say -v Kyoko "立って挨拶して" -o /tmp/claude-501/stand_hello.aiff && afconvert -f WAVE -d LEI16@16000 /tmp/claude-501/stand_hello.aiff tests/fixtures/stand_hello.wav
```

  `Kyoko` が無ければ `say -v '?' | grep ja_JP` で日本語の声を選ぶ。

- [ ] **Step 2: 失敗するテストを書く**

```python
@pytest.mark.slow
def test_transcribes_japanese_fixture():
    text = Transcriber(os.environ.get("WHISPER_MODEL", "small"))(Path("tests/fixtures/stand_hello.wav").read_bytes())
    assert "挨拶" in text

def test_empty_audio_is_silent():
    assert Transcriber.is_silent(b"") and Transcriber.is_silent(b"not audio")
```

  `Transcriber.is_silent(audio: bytes) -> bool`（staticmethod）：
  - 空のバイト列、PyAV で読めない、読めた長さが 0.3 秒未満のどれかなら真を返す。
  - `__call__` はそのとき、モデルを呼ばずに `""` を返す。
  - Task 9 の Session は、空の書き起こしを「聞き取れませんでした」として扱う。

- [ ] **Step 3: 失敗を確かめる。** Expected: ImportError
- [ ] **Step 4: 実装する**
- [ ] **Step 5: 通ることを確かめ、速さを測ってモデルを決める**

```bash
docker compose run --rm --no-deps app python3 -m pytest -q tests/test_stt.py
for m in small medium large-v3-turbo; do docker compose run --rm --no-deps -e WHISPER_MODEL=$m app python3 -c "
import time; from pathlib import Path; from voice_ltl.stt import Transcriber
t=Transcriber('$m'); a=Path('tests/fixtures/stand_hello.wav').read_bytes(); t(a); s=time.time(); print('$m', repr(t(a)), round(time.time()-s,2),'s')"; done
```

  Expected：`2 passed`。モデルごとに文と秒数が出る。
  - 基準（設計書 §11）：5 秒くらいの指示を 3 秒程度以内。この音声は約 1.5 秒なので、1 秒以内を目安にする。
  - 「挨拶」が正しく出るもののうち、いちばん大きいものを `env.txt.example` の `WHISPER_MODEL` の既定にし、結果を README に書く。

---

### Task 12: ROS 2 の窓口（`ros_link.py`）とにせ Go2 のノード（`sim/fake_go2.py`）

**Files:**
- Create: `voice_ltl/ros_link.py`、`sim/fake_go2.py`

**Interfaces:**
- Consumes：`Link`、`Reading`、`Sensing`（Task 7・9）、`SimGo2`（Task 8）、unitree のメッセージ
- Produces：
  - `RosLink(perception: PerceptionConfig, node_name: str = "go2_voice_ltl")`。`Link` を満たす。
    - `spin_once() -> None`
    - `other_senders() -> int`
    - `wait_ready(timeout_s: float = 5.0) -> None`：Go2 が見つからなければ `RuntimeError`。文言は `demo_script.py` と同じ。
    - `close(stop_times: int = 3) -> None`
  - `SimControl(node)`：`/sim/obstacles` に `std_msgs/String`（JSON `{"cmd": "front" | "back" | "clear"}`）を送る。`latest_world() -> dict | None` は `/sim/world` から最後に受けたもの。
  - にせ Go2 のノード：
    - `/api/sport/request` を受けて `SimGo2.handle` に渡す。
    - 50 Hz で `step` を回し、`/sportmodestate` を出す（`position[0..1]`、`imu_state.rpy[2]`、`error_code`）。
    - 10 Hz で `/utlidar/cloud` を出す（PointCloud2、frame `utlidar_lidar`、fields x/y/z float32）。
    - `/sim/obstacles` を受け、`/sim/world` を出す（`{"boxes": [[x, y, size], ...], "collided": bool}`）。
- QoS：
  - 受ける側は `nl_control.py` と同じ `qos_profile_sensor_data`。
  - にせ Go2 が出す側も、`/sportmodestate` と `/utlidar/cloud` は best effort にする。

**安全（`nl_control.py` から移す）：**
- `/api/sport/request` に自分以外の送り手がいたら、起動を断る。
- 二重起動を `/tmp/go2_voice_ltl.lock` の `flock` で防ぐ。
- SIGTERM・SIGHUP・Ctrl+C では、`close(3)` で StopMove を 3 回送る。

- [ ] **Step 1: 実装する**（ROS に触る薄い層なので、単体テストは書かない。次の Step で動かして確かめる）

- [ ] **Step 2: にせ Go2 が実機と同じ窓口を出しているか確かめる**

```bash
cd ~/work/physicalAI/go2
docker compose up -d sim
docker compose exec sim bash -lc 'source ~/unitree_ros2/setup.sh && ros2 topic list && timeout 3 ros2 topic echo --once /sportmodestate --field error_code && timeout 3 ros2 topic hz /utlidar/cloud'
```

  Expected：`/api/sport/request`、`/sportmodestate`、`/utlidar/cloud` が並ぶ。`error_code` は 1004。周期は約 10 Hz。

- [ ] **Step 3: 渡されたスクリプトが、にせ Go2 を相手に最後まで動くか確かめる**

```bash
docker compose exec sim bash -lc 'source ~/unitree_ros2/setup.sh && cd /work/scripts && printf "\n" | timeout 60 python3 demo_script.py'
```

  Expected：「立つ」から「終わりました」まで順に出る。
  - もし「前進 0.5 m」で止まったら、にせ Go2 の Move の受け付けか `position` の出し方を直す。
  - `demo_script.py` は、立つときに StandUp → BalanceStand を送っている。

- [ ] **Step 4: 後片付け**

  Run: `docker compose down`

---

### Task 13: Web サーバーと操作画面（`server.py`・`static/`）

**Files:**
- Create: `voice_ltl/server.py`、`voice_ltl/static/index.html`、`voice_ltl/static/app.js`、`voice_ltl/static/style.css`
- Test: `tests/test_server.py`

**Interfaces:**
- Consumes：`Session`、`Transcriber`、`make_llm`、`RosLink`、`SimControl`、`render_svg` の結果（`session.diagrams`）
- Produces：
  - `create_app(session: Session, sim: SimControl | None) -> FastAPI`
    - `GET /`：画面
    - `POST /api/audio`：本文が音声。`session.submit_audio` に渡して `202` を返す。
    - `POST /api/text`：`{"text": str}`。`submit_text` に渡して `202` を返す。打ち込みでの試験用。
    - `POST /api/stop`：`204`
    - `POST /api/obstacle`：`{"cmd": "front" | "back" | "clear"}`。`sim` が `None` なら `404`。
    - `GET /api/diagram/{name}.svg`：`name` は `PLANT | SPEC | TRIMMED | SUP`
    - `WS /ws`：0.2 秒ごとに `session.snapshot()` を送る。シミュレーション中なら `sim.latest_world()` を `world` キーで足す。
  - `main()`：
    1. `env.txt` の値は compose が環境変数で渡す。
    2. `RosLink`、`make_llm(os.environ)`、`Transcriber(os.environ.get("WHISPER_MODEL", "small"))`、`Session(..., workdir=Path("/work/out"))` を作る。
    3. `wait_ready()` を待つ。
    4. 制御ループのスレッドを起こす（10 Hz：`link.spin_once()` → `session.tick(now)`）。
    5. uvicorn を `0.0.0.0:8000` で動かす（`SIM=0` なら `127.0.0.1:${GO2_UI_PORT}`。空・未設定なら 8810）。
    6. 終わるときは `link.close(3)`。

**画面（設計書 §8）：**
- 上：大きな表示。押していない間は「スペースを押している間、話せます」、押している間は「録音中…」。
- 左の列：
  - 書き起こし
  - LLM の試行（式、合否、却下理由）
  - 最終的な式
  - 計画（動作名の列。今の動作を強調する）
  - メッセージ
  - ログ
- 右の列：状態遷移図のタブ（SUP / TRIMMED / SPEC / PLANT）。`diagram_version` が変わったら読み直す。
- 下：
  - Go2 の状態（姿勢、位置、前方・後方の距離と判定）
  - 上から見た小さな図（canvas：Go2 の位置と向き、箱、0.5 m の線）
  - ボタン「前に箱を置く」「後ろに箱を置く」「箱を全部どける」「止める」。箱のボタンはシミュレーション中だけ出す。
- **キーの扱い（`app.js`）：**
  - `keydown` で `e.code === "Space" && !e.repeat` かつ入力欄にフォーカスが無いとき、`preventDefault()` して録音を始める。
  - `keyup`（Space）と `window.blur` で録音を止め、`POST /api/audio` する。
  - マイクの許可は、ページを開いて最初のクリック（「マイクを使う」ボタン）で `getUserMedia({audio: true})` を1回だけ取る。
  - 録音は `MediaRecorder`（`audio/webm;codecs=opus`）を使う。

- [ ] **Step 1: 失敗するテストを書く（TestClient。ROS なし）**

```python
def make_client(tmp_path):
    sess = Session(DirectSimLink(SimGo2()), FakeLLM([{"formula": "F hello", "unsupported": []}]), lambda b: "立って挨拶して", tmp_path, background=False)
    return TestClient(create_app(sess, sim=None)), sess

def test_text_endpoint_runs_pipeline(tmp_path):
    c, sess = make_client(tmp_path)
    assert c.post("/api/text", json={"text": "立って挨拶して"}).status_code == 202
    sess.tick(0.0); assert sess.snapshot()["formula"] == "F hello"
    assert c.get("/api/diagram/SUP.svg").headers["content-type"].startswith("image/svg")

def test_audio_and_stop_and_obstacle_404(tmp_path):
    c, sess = make_client(tmp_path)
    assert c.post("/api/audio", content=b"RIFF....", headers={"content-type": "audio/wav"}).status_code == 202
    assert c.post("/api/stop").status_code == 204
    assert c.post("/api/obstacle", json={"cmd": "front"}).status_code == 404

def test_ws_sends_snapshot(tmp_path):
    c, _ = make_client(tmp_path)
    with c.websocket_connect("/ws") as ws:
        assert "status" in ws.receive_json()

def test_index_has_push_to_talk(tmp_path):
    c, _ = make_client(tmp_path)
    html = c.get("/").text
    assert "スペース" in html and "app.js" in html
```

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError
- [ ] **Step 3: 実装する**
- [ ] **Step 4: 通ることを確かめる。** Expected: `4 passed`

- [ ] **Step 5: コンテナ一式で、本物の流れを確かめる**

  前提：ユーザーが `env.txt` を作り、`claude setup-token` で作ったトークンを自分で書いてあること。無ければ、この Step はユーザーに頼んでから行う。

```bash
cd ~/work/physicalAI/go2 && docker compose --profile fake up -d
curl -s -X POST localhost:8810/api/audio -H 'content-type: audio/wav' --data-binary @tests/fixtures/stand_hello.wav -o /dev/null -w '%{http_code}\n'
```

  Expected：`202`。続けて Playwright（MCP）で `http://localhost:8810` を開き、次を確かめる。
  - 書き起こしに「挨拶」、式に `hello` が出る。
  - SUP の図が出る。
  - 姿勢が「立ち」になる。
  - 「前に箱を置く」で小さな図に箱が出る。
  - 前のテストの続きで `POST /api/text {"text":"前に進んで"}` を送ると、前進せずに「待機中」になる。「箱を全部どける」で前進する。
  - 本物の LLM で、設計書 §7.4 のうち LLM の答え方しだいの行を確かめる。どちらも Go2 は動かない。
    - `{"text":"キッチンへ行って"}` → メッセージに「キッチン」と「単語帳」
    - `{"text":"0.7メートル進んで"}` → メッセージに「指定できません」
    - 違う答えになったら、プロンプトの例（Task 5）を直して確かめ直す。
  - **Review Focus 1：** ページで Space の `keydown` を `repeat=true` で何度か送っても、録音の開始が1回だけで、ページがスクロールしない。`window.blur` で録音が止まる（`page.evaluate` でイベントを送り、`app.js` の録音状態の変数を読む）。

  終わったら `docker compose down`。

---

### Task 14: 実機用の compose と README

**Files:**
- Create: `compose.robot.yaml`、`README.md`
- Modify: `env.txt.example`（Task 11 で決めた `WHISPER_MODEL`）

**Interfaces:**
- Consumes：Task 1 の Dockerfile、Task 13 の `main()`（`SIM=0` で 127.0.0.1 待ち受け、箱のボタンを出さない）

- [ ] **Step 1: `compose.robot.yaml`**

  - `app` だけ。`network_mode: host`、`environment: SIM=0`。
  - `GO2_IFACE` は `env.txt` から読む。
  - volumes は Task 1 と同じ。
  - `sim` は無い。

- [ ] **Step 2: `README.md`（日本語）を書く**

  章立て：
  1. これは何か（設計書 §1 の要約と、図1つ）
  2. 準備（Mac）
     - Docker Desktop
     - `cp env.txt.example env.txt`
     - LLM のログイン
       - Claude：Mac で `claude setup-token` を実行し、出たトークンを自分で `env.txt` に書く
       - Codex：`docker compose run --rm app codex login`（詳しくは公式ドキュメント。**この計画の時点では未確認**と明記する）
     - `docker compose build`
  3. 使い方
     - `docker compose up`
     - ブラウザで `http://localhost:8810`（`GO2_UI_PORT` で変えられる）
     - 「マイクを使う」→ スペースを押して話す
     - 止め方：「止まって」と言う、「止める」ボタン、`docker compose down`
  4. 話せること：単語帳の一覧と、設計書 §7.4 の表
  5. 単語帳と状態遷移図の足し方：`voice_ltl/world.py` の1行と、`PROP_DESCRIPTION_JA` の1行
  6. Linux PC へ移す（設計書 §10）
     - フォルダをコピーする
     - 有線の設定：`192.168.123.x`、`.161` は使わない
     - 有線と Wi-Fi の両方を使う
     - `ping 192.168.123.161`
     - `env.txt` の `GO2_IFACE`
     - `docker compose -f compose.robot.yaml up`
     - `ros2 topic echo /sportmodestate` での確認
     - **後方の LiDAR を `rviz2` で確かめる手順**と、見えない場合に `ACTION_NAMES` から `backward` を外す方法
     - 安全の注意（前方 1 m と周りを空ける、リモコンを持った人がそばにいる）
  7. テスト：`docker compose run --rm --no-deps app python3 -m pytest -q tests`
  8. わかっていないこと：Codex の実地確認、実機での後方の検知、Whisper の正確さ

- [ ] **Step 3: 確かめる**

```bash
cd ~/work/physicalAI/go2
docker compose -f compose.robot.yaml config > /dev/null && echo robot-compose-ok
docker compose run --rm --no-deps app python3 -m pytest -q tests nl_control/tests
```

  Expected：`robot-compose-ok`。テストはすべて通る（`slow` の印のものも含む）。

  README の「使い方」に、Gazebo の起動（`docker compose --profile gazebo up`、`http://localhost:6080`）と、Gazebo でできること・できないこと（設計書 §9.1 の表）も書く。Task 16・17 が終わったあとに書き足してよい。

---

### Task 15: Gazebo のコンテナと、Go2 が立って歩けるかの試し（実行順は Task 1 の直後）

**Files:**
- Create: `docker/Dockerfile.gazebo`、`docker/gazebo_entry.sh`、`docs/reference/gazebo_feasibility.md`（測った結果の記録）、`docs/reference/gazebo_feasibility.png`
- Modify: `compose.yaml`（`gazebo` サービスを足す）

**Interfaces:**
- Consumes：Task 1 のイメージ `go2-voice-ltl:dev`
- Produces：
  - `docker compose --profile gazebo up` で、Gazebo Classic の中に Go2 が出る。
    - 制御：unitree_guide controller。起動時は脱力の状態。
    - 画面：`http://localhost:6080/vnc.html` で見える。
  - `docs/reference/gazebo_feasibility.md` に、Task 16 が使う次の事実を書く。
    - unitree_guide controller の状態の番号と、そこへ切り替える `control_input_msgs/msg/Inputs` の値
    - 速度を入れるフィールドと向き
    - 位置・向き・状態を読めるトピック
    - real-time factor の実測
    - 判定（GO / NO-GO）

- [ ] **Step 1: `docker/Dockerfile.gazebo` を書く**

  - `FROM go2-voice-ltl:dev`
  - apt：`ros-humble-gazebo-ros-pkgs`、`ros-humble-gazebo-ros2-control`、`ros-humble-ros2-controllers`、`ros-humble-xacro`、`xvfb`、`fluxbox`、`x11vnc`、`novnc`、`websockify`、`mesa-utils`
  - `git clone -b humble https://github.com/legubiao/quadruped_ros2_control ~/gz_ws/src/quadruped_ros2_control`。ビルド時点の commit を `git checkout <sha>` で固定する。
  - `rosdep install --from-paths src --ignore-src -r -y`
  - `colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input`
  - `ENV LIBGL_ALWAYS_SOFTWARE=1`、`ENV DISPLAY=:1`
  - arm64 で apt の Gazebo のパッケージが無い・足りないときは、無いパッケージ名と `apt-cache policy` の出力を `gazebo_feasibility.md` に書き、BLOCKED で報告する。

- [ ] **Step 2: `docker/gazebo_entry.sh` を書く**

  1. `Xvfb :1 -screen 0 1280x800x24` を起動する。
  2. `fluxbox` と `x11vnc -display :1 -forever -nopw -shared -rfbport 5900` を起動する。
  3. `websockify --web /usr/share/novnc 6080 localhost:5900` を起動する。
  4. `source ~/unitree_ros2/setup.sh && source ~/gz_ws/install/setup.bash` のあと、`exec "$@"` する。

  `compose.yaml` の `gazebo` サービス：
  - `profiles: ["gazebo"]`、build は `docker/Dockerfile.gazebo`、`image: go2-voice-ltl-gazebo:dev`
  - `ports: ["127.0.0.1:6080:6080"]`、volumes は app と同じ
  - entrypoint は `gazebo_entry.sh`
  - command：`ros2 launch unitree_guide_controller gazebo_classic.launch.py pkg_description:=go2_description`（Task 16 で自分の launch に差し替える）

- [ ] **Step 3: ビルドして起動する**

```bash
cd ~/work/physicalAI/go2
docker compose build app && docker compose --profile gazebo build gazebo
docker compose --profile gazebo up -d gazebo && sleep 40
docker compose exec gazebo bash -lc 'source ~/unitree_ros2/setup.sh; source ~/gz_ws/install/setup.bash; ros2 topic list; ros2 control list_controllers'
```

  Expected：トピックの一覧に `/control_input` か、それに当たる入力のトピックがある。unitree_guide controller が `active`。

- [ ] **Step 4: 状態の切り替えと速度の入れ方を、コードから読み取る**

  - `~/gz_ws/src/quadruped_ros2_control` の `controllers/unitree_guide_controller` の FSM と、`commands/keyboard_input` を読む。
  - 「伏せ（fixed down）」「立つ（fixed stand）」「歩く（trotting）」「脱力（passive）」に切り替える `Inputs.command` の値と、速度を入れるフィールド（`lx` / `ly` / `rx` など）と、その向きと大きさの範囲を表にする。
  - それを `gazebo_feasibility.md` に書く。

- [ ] **Step 5: 立って歩けるかを測る**

  - Step 4 の値を `ros2 topic pub --once` で送る。順番は「立つ」→ 5 秒待つ →「歩く」→ 前向きの速度を 5 秒。
  - 胴体の高さ・位置・傾きは、Gazebo のモデルの状態（`/gazebo/get_entity_state` サービス、または `gz model -m <name> -p`）で、立つ前・立った後・歩いた後に読む。
  - real-time factor は `gz stats -p` の数秒分の平均で測る。
  - 合格の目安（すべてを満たせば GO）：
    - 立った後の胴体の高さが、立つ前より 0.1 m 以上高い
    - 5 秒の前進で 0.3 m 以上進む
    - 前進の間、転ばない（roll と pitch の絶対値が 0.5 rad 未満）
    - real-time factor が 0.3 以上
  - 結果の数値と判定を `gazebo_feasibility.md` に書く。

- [ ] **Step 6: 画面が見えることを確かめる**

  - Playwright（MCP）で `http://localhost:6080/vnc.html?autoconnect=1` を開き、Go2 が見えるスクリーンショットを `docs/reference/gazebo_feasibility.png` に保存する。
  - 終わったら `docker compose --profile gazebo down`。
  - **NO-GO のとき：** 試した内容と数値を報告に書き、DONE_WITH_CONCERNS で返す。controller（呼び出し元）がユーザーに相談する。

---

### Task 16: Gazebo の橋渡し（`gazebo/`）

**Files:**
- Create: `gazebo/__init__.py`、`gazebo/bridge_core.py`、`gazebo/gazebo_bridge.py`、`gazebo/go2_sensors.urdf.xacro`、`gazebo/launch/go2_gazebo.launch.py`
- Modify: `compose.yaml`（`gazebo` の command を自分の launch に）
- Test: `tests/test_bridge_core.py`

**Interfaces:**
- Consumes：
  - `docs/reference/gazebo_feasibility.md` の表（状態の番号、速度のフィールド、トピック）
  - `go2_nl_control.actions` の `api_id` と `_ERROR_CODE_TO_STATE`
  - Task 8 の `Box` / `FOOTPRINT` と点群の作り方の考え方
  - Task 12 の `/sim/obstacles` と `/sim/world` の形
- Produces：
  - `BridgeCore(now: float)`
    - `handle(api_id: int, parameter: str, now: float) -> None`
    - `step(now: float) -> ControllerInput | None`：今送るべき制御の入力。変化が無ければ `None`。
    - `error_code -> int`
    - `notes: list[str]`：「（Gazebo では再現なし）」などのログ
  - `ControllerInput(command: int, lx: float, ly: float, rx: float, ry: float)`
  - ROS 2 ノード `python3 -m gazebo.gazebo_bridge`
  - launch `gazebo/launch/go2_gazebo.launch.py`：Go2 を `go2_sensors.urdf.xacro` 付きで出し、controller と橋渡しを起動する。

**決めたふるまい（設計書 §9.1 の表のとおり）：**

| 指令 | 受け付ける状態 | 制御の入力 | 終わったときの `error_code` |
|---|---|---|---|
| StandUp 1004 | 伏せ（脱力も含む） | 「立つ」 | 2.0 秒後に 1002 |
| BalanceStand 1002 | 1002・1013 | 「歩く」に切り替え、速度 0 | 0.5 秒後に 1013 |
| StandDown 1005 | 立ち | 「伏せ」 | 2.0 秒後に 1004 |
| Sit 1009 | 立ち | 「伏せ」 | 2.0 秒後に 1007 |
| RiseSit 1010 | 1007 | 「立つ」→ 2.0 秒後に「歩く」 | 1013 |
| Move 1008 | 1013・1015 | 「歩く」のまま速度を入れる。前後の速度 `x`、左右 `y`、旋回 `z` を、Task 15 の表の向きと範囲に合わせて変換する | 動いている間は 1015、1.0 秒送り直しが無ければ速度 0 にして 1013 |
| StopMove 1003 | いつでも | 速度 0 | — |
| ジェスチャー（1016・1017・1020・1022・1029・1036） | 立ち | 変えない。3.0 秒の間は他の指令を受け付けない | 1013 のまま。`notes` に「（Gazebo では再現なし）」 |

- 受け付けない指令は無視し、`notes` に書く。
- 起動直後は脱力の状態で、`error_code` は 1001（伏せ扱い）。

**センサーと箱：**
- **LiDAR：** `go2_sensors.urdf.xacro` で、`base` リンクから (x=0.28, y=0, z=0.10)、傾きなし（`perception.py` の既定 `lidar_x=0.28`、`lidar_z=0.10`、`lidar_pitch=0`）の位置に Gazebo Classic の ray センサーを付ける。
  - 設定：水平 360 度・垂直 −30〜+30 度、10 Hz、範囲 0.05〜10 m
  - 出力：`libgazebo_ros_ray_sensor.so` の `sensor_msgs/PointCloud2`。トピック `/utlidar/cloud`、frame `utlidar_lidar`
  - 自分の脚が点群に映るときは、橋渡しで機体の外形（中心から前後 0.40 m・左右 0.20 m・高さ −0.35〜0.15 m の箱の中）の点を取り除いてから出す。
- **真の位置：** `libgazebo_ros_p3d.so` で `base` の位置と向きを 50 Hz で受け、`/sportmodestate` の `position` と `imu_state.rpy` にする。
- **箱：**
  - `/sim/obstacles` を受けたら、Go2 の今の位置から前（後ろ）1.2 m に 0.3 m の立方体を `/spawn_entity` で出す。`clear` なら `/delete_entity` で全部消す。
  - `/sim/world` には、にせ Go2 と同じ形（`{"boxes": [[x, y, size], ...], "collided": bool}`）を出す。`collided` は、中心から箱までが `FOOTPRINT`（0.35 m）未満になったら真にする。

- [ ] **Step 1: 失敗するテストを書く（`BridgeCore`、ROS なし）**

```python
def test_stand_then_walk_then_expire():
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0); assert b.step(0.0).command == STAND_CMD
    b.step(2.1); assert b.error_code == 1002
    b.handle(BALANCE_STAND, "", 2.1); assert b.step(2.1).command == TROT_CMD
    b.step(2.7); assert b.error_code == 1013
    b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 3.0); i = b.step(3.0)
    assert i.command == TROT_CMD and (i.lx, i.ly) != (0, 0)
    assert b.error_code == 1015
    i = b.step(4.2); assert (i.lx, i.ly, i.rx) == (0, 0, 0) and b.error_code == 1013

def test_gesture_is_pretend_and_sit_uses_crouch():
    b = standing_core()
    b.handle(HELLO, "", 10.0); assert b.step(10.0) is None and "再現なし" in b.notes[-1]
    b.handle(SIT, "", 13.5); assert b.step(13.5).command == DOWN_CMD
    b.step(15.6); assert b.error_code == 1007 and state_from_error_code(1007) == SITTING

def test_invalid_command_ignored():
    b = BridgeCore(0.0); b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    assert b.step(0.0) is None and b.error_code == 1001
```

  `STAND_CMD`・`TROT_CMD`・`DOWN_CMD` は `bridge_core.py` の定数で、値は `gazebo_feasibility.md` の表から取る。`standing_core()` は 1013 まで進めたものを返すテスト内の補助関数。速度の向き（`lx` / `ly` のどちらが前後か）も表に合わせ、上のテストの前進の assert をその向きで書き直してよい。

- [ ] **Step 2: 失敗を確かめる。** Expected: ImportError
- [ ] **Step 3: `bridge_core.py`、`gazebo_bridge.py`、xacro、launch を実装する**
- [ ] **Step 4: 単体テストが通ることを確かめる。** Expected: `3 passed`

- [ ] **Step 5: Gazebo の上で、窓口が実機と同じになっているか確かめる**

```bash
cd ~/work/physicalAI/go2
docker compose --profile gazebo build gazebo && docker compose --profile gazebo up -d gazebo && sleep 40
docker compose exec gazebo bash -lc 'source ~/unitree_ros2/setup.sh; ros2 topic list | grep -E "api/sport/request|sportmodestate|utlidar/cloud"; timeout 5 ros2 topic echo --once /sportmodestate --field error_code; timeout 6 ros2 topic hz /utlidar/cloud'
```

  Expected：3つのトピックが並ぶ。`error_code` は 1001。点群は 5 Hz 以上。

- [ ] **Step 6: `demo_script.py` を Gazebo 相手に最後まで流す**

```bash
docker compose exec gazebo bash -lc 'source ~/unitree_ros2/setup.sh && cd /work/scripts && printf "\n" | timeout 120 python3 demo_script.py'
```

  Expected：「立つ」→「前進 0.5 m」→「左に 90°」→「挨拶」→「座る」→「立ち上がる」→「終わりました」。
  - 途中で Playwright で `http://localhost:6080/vnc.html?autoconnect=1` のスクリーンショットを撮る。少なくとも「立った」「前進した後」「伏せ（座るの代用）」の3枚を `docs/reference/gazebo_demo_*.png` に保存する。
  - 終わったら `docker compose --profile gazebo down`。

---

### Task 17: 式から直接動かす道具と、アプリ一式での Gazebo の確認

**Files:**
- Create: `voice_ltl/run_formula.py`、`docs/reference/gazebo_e2e.md`
- Modify: `README.md`（Gazebo の使い方。Task 14 の Step 2 の追記分）

**Interfaces:**
- Consumes：
  - `Session`（Task 9）。`llm` には、式を1つ返す `FakeLLM` を内部で作って渡す。
  - `RosLink`（Task 12）
- Produces：`python3 -m voice_ltl.run_formula "<LTLf の式>" [--timeout 120]`
  - 標準出力に、計画・各動作の開始と完了・最後の `status` を日本語で出す。
  - 終了コード：`done` なら 0、それ以外は 1。
  - LLM も音声も使わない。翻訳以外は本番と同じ道（`check()`、合成、監督器、実行器、センサー）を通る。

- [ ] **Step 1: 実装する**

  - 中身：`Session(RosLink(...), FakeLLM([{"formula": f, "unsupported": []}]), None, Path("/work/out"), background=False)` を作り、`submit_text("(run_formula)")` してから、10 Hz で `spin_once` と `tick` を回す。
  - 終わり方：`status` が `done` か `error` になったら止まる。`--timeout` を超えても止まる。
  - 終了時は `RosLink.close(3)` を呼ぶ。

- [ ] **Step 2: Gazebo の上で基本動作を確かめる**

```bash
cd ~/work/physicalAI/go2
docker compose --profile gazebo up -d gazebo && sleep 40
docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(turned_left & X F lying))"; echo "exit=$?"
```

  Expected：計画 `stand → forward → turn_left → lie_down` が順に完了し、`exit=0`。
  - Playwright のスクリーンショットで、Go2 が前に出て、向きを変え、伏せたことを確かめる。`docs/reference/gazebo_e2e_*.png` に保存する。
  - `app` が `gazebo` と同じネットワークで DDS をやり取りできること。compose の既定のネットワークのままでよい。

- [ ] **Step 3: 箱で止まるかを確かめる**

  1. `ros2 topic pub --once /sim/obstacles std_msgs/msg/String '{data: "{\"cmd\":\"clear\"}"}'` で箱を消す。
  2. 前と同じ手順で、Go2 を立たせて前向きにしたところで、`{"cmd":"front"}` で前に箱を出す。
  3. `run_formula "F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward)))" --timeout 40` を流す。
  4. Expected：
     - Go2 が箱の手前で止まる（`/sim/world` の `collided` が偽のまま）。
     - 15 秒後に「秒待っても」の文言で中止し、`exit=1`。
     - スクリーンショットを1枚残す。
  5. 箱を消してもう一度流すと、`exit=0` になる。

- [ ] **Step 4: 結果を `docs/reference/gazebo_e2e.md` に書き、README に Gazebo の使い方を足す**

  - `gazebo_e2e.md` には、コマンド・出力の要点・スクリーンショットの名前・real-time factor を書く。
  - README には、起動、`http://localhost:6080/vnc.html`、`run_formula` の使い方、できること・できないことを書く。
  - 終わったら `docker compose --profile gazebo down`。
