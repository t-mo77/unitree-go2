# Gazebo で開いた物の種類を測った記録（open-objects の重い段 H1〜H7）

open-objects の計画（`docs/superpowers/plans/2026-11-11-go2-open-objects.md`）の Wave 3 の重い段の記録。
段ごとに 1 つの節で、測った順に足していく。設計書は `docs/superpowers/specs/2026-11-11-go2-open-objects-design.md`（「設計書」）。

## H1：Fuel のイメージを作り、Suitcase1 を固定した（2026-11-12）

- 枝：`objects2/h1-fuel`（`feat/open-objects` の `21e1a1f` から）。worktree `~/work/physicalAI/.go2-worktrees/oo-h1`。ROS の domain は 61。
- 場所：この Mac の Docker Desktop。
- 1 回目の試み（10:29〜10:35）は、zip の取得・固定・build・5 つの測りまで済ませたところで API の障害で止まった。
  2 回目（13:16〜）は、作り直さずに、同じ入力から作ったイメージかを確かめ、測りと固定とテストを確かめ直して、記録を書いた。

### 結論

**H1 の線はすべて満たした。** Suitcase1（版 2）の zip は sha256 `8e0c23b7c34cd19f443253eecf7185ca1a949f955aba278f01029775972f8fe7`、
1 802 850 B（Fuel の情報と同じ）。`go2-voice-ltl-gazebo:patch-0005-fuel1` は `patch-0005` の上に 2 つの層を重ねただけで、5 つのモデルの
フォルダがあり、`GZ_SIM_RESOURCE_PATH` の最後が `/opt/go2_models`。Suitcase1 の足跡はモデルの軸で 0.291 × 0.468、高さ 0.752、
置く高さ 0。計画の上限 0.6 × 0.4 に世界の軸で収まるよう、`suitcase_1` を 90° 回した（位置は (−1.5, 1.5) のまま）。
そのため `F around_suitcase_1` の PLANT が 8 736 → 8 064、ずれのいちばん大きい面積が 121 → 110 に動いた（下の「動いた数」）。
受け入れの回の PLANT（§12.4）は変わらない。全部のテストは 0 failed。

| 線（計画 H1） | 結果 |
|---|---|
| Suitcase1 を 1 回取って sha256 と大きさを記録 | 済み。上の値。host では開いていない |
| 固定して `T tests/test_layout.py tests/test_fuel_image.py` | 通った（`test_household.py` と合わせて 113 passed） |
| build が通り、5 つのフォルダと `GZ_SIM_RESOURCE_PATH` の最後が `/opt/go2_models` | 通った |
| イメージの中で 5 つを測る | 済み。ほかの 4 つの比べは下の表 |
| Suitcase1 の作者・足跡・高さ・z を固定、上限を超えれば §12.1 の中で置き直す | 済み。90° 回した。`T tests/test_household.py` と全部のテストが通った |

### D11 の門と、ほかの container・電源

どの回の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | `lowpowermode` |
|---|---|---|---|---|
| 10:30:35 | build の前 | なし | AC、100%、charged | 0 |
| 13:20:20 | 確かめ直しの前 | なし | 電池、100%、discharging | 0 |
| 13:27:13 | 確かめ直しと全部のテストの後 | なし | 電池、100%、discharging | 0 |

- build の後の記録は、1 回目の試みとともに失われた。その次の記録（13:20）でも、動いている container はなかった。
- 確かめ直しで動かしたのは、名前を付けた使い捨ての container（`docker run --rm --name oo-h1-…`）と、本体の compose project の
  `app` の `run --rm` だけ。終わった後に残った container はない。compose project `go2oo` は使っていない（H1 は Gazebo を動かさない）。

### Suitcase1 の zip

```bash
curl -fsSL -o <scratch>/fuel/Suitcase1.zip https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1/2/Suitcase1.zip
shasum -a 256 <scratch>/fuel/Suitcase1.zip; wc -c < <scratch>/fuel/Suitcase1.zip
```

- 作業用の新しい空のフォルダに 1 回だけ取った（10:30）。2 回目の試みは取り直さず、同じファイルの sha256 と大きさを数え直した（同じ値）。
- 版のある REST の形（設計書 §12.2 で「確かめていない」とした形）で取れた。
- build も同じ形で 5 つの zip を取り、固定の sha256 と比べて通った（下）。

### 固定：`docker/fuel_models.json`

- Suitcase1：`sha256` を上の値に、`bytes` は Fuel の情報の 1 802 850 のまま（取った zip と同じ）、`author` を `Wan Yi Seow` に
  （イメージの中の `model.config` の `<author><name>`）。ライセンスの項目は `model.config` になく、CC0 1.0 は Fuel の情報から（設計書 §12.2）。
- `tests/test_layout.py` の `test_fuel_models_list_pins_every_field` の Suitcase1 の行を、同じ値に固定した（null の 2 つが H1 の印だった）。

### build

```bash
docker build --build-arg BASE=go2-voice-ltl-gazebo:patch-0005 -t go2-voice-ltl-gazebo:patch-0005-fuel1 \
    -f docker/Dockerfile.gazebo-fuel docker
```

- 元：`go2-voice-ltl-gazebo:patch-0005`（`sha256:39fd601dc2a3…`、2026-11-11 10:08 に作ったもの）。作り直していない。
- できたイメージ：`go2-voice-ltl-gazebo:patch-0005-fuel1`（`sha256:5ee232801b4e…`、2026-11-12 10:30:54、7.28 GB）。
  元の 22 の層はそのままで、足したのは `COPY`（32.8 kB）と `RUN`（6.21 MB、12.4 秒）の 2 つと `ENV` だけ。
- `RUN` の出力（5 つとも固定の sha256 と同じ）：

  ```
  fetch_fuel_models: Sofa 3: 1311028 B, sha256 6d19d1f1…0db1 ok -> /opt/go2_models/Sofa
  fetch_fuel_models: TrashBin 2: 1045623 B, sha256 ff38c8c7…0de3 ok -> /opt/go2_models/TrashBin
  fetch_fuel_models: LampAndStand 3: 676942 B, sha256 f8610eb7…85eb ok -> /opt/go2_models/LampAndStand
  fetch_fuel_models: Table 4: 899222 B, sha256 3a720ef2…163d ok -> /opt/go2_models/Table
  fetch_fuel_models: Suitcase1 2: 1802850 B, sha256 8e0c23b7…8fe7 ok -> /opt/go2_models/Suitcase1
  ```
- build の注意が 1 つ：`UndefinedVar: Usage of undefined variable '$GZ_SIM_RESOURCE_PATH'`。元のイメージにこの値がないためで、
  `${GZ_SIM_RESOURCE_PATH:+…:}` の形なので値は `/opt/go2_models` だけになる（意図どおり）。
- **同じ入力から作ったかの確かめ（2 回目）：** イメージの中の `/opt/go2_fuel/fetch_fuel_models.py` は枝のものと同じ。
  `/opt/go2_fuel/fuel_models.json` は、Suitcase1 の `author` だけが違う（build の時は null）。計画の順（zip を固定 → build → 測る →
  作者を固定）のとおりで、build は `folder`・`version`・`sha256` しか読まないので、中身のモデルは同じ。だから作り直していない。

### イメージの中の確かめ

```bash
docker run --rm --entrypoint bash go2-voice-ltl-gazebo:patch-0005-fuel1 -c 'ls /opt/go2_models; …'
```

- `/opt/go2_models` には `LampAndStand`、`Sofa`、`Suitcase1`、`Table`、`TrashBin` の 5 つ（ほかに隠れたファイルはない。zip は消えている）。
  どれにも `model.config` と `model.sdf` があり、`<plugin` は 0 個。
- `GZ_SIM_RESOURCE_PATH`：環境の値は表示せず、`:` で区切った数と最後の項目だけを確かめた。`--entrypoint bash` では 1 項目で、
  それが `/opt/go2_models`。いつもの入り口（`gazebo_entry.sh` が ROS 2 と Gazebo の workspace を読み込んだ後）では 2 項目で、
  `/opt/go2_models` はそのうちの 1 つ（workspace が足した項目があっても消えない）。

### 5 つのモデルを測った（イメージの中で、`gazebo/tools/model_extent.py`）

```bash
docker run --rm -v ~/work/physicalAI/.go2-worktrees/oo-h1:/wt -w /wt --entrypoint bash go2-voice-ltl-gazebo:patch-0005-fuel1 -c \
  'for m in /opt/go2_models/*; do echo "$m"; python3 -I /wt/gazebo/tools/model_extent.py "$m"; done; cat /opt/go2_models/Suitcase1/model.config'
```

1 回目（10:31）と 2 回目（13:21 ごろ）で、出力は 1 文字も違わなかった。`model_extent.py` は collision の形を測る。Suitcase1 と LampAndStand の
collision は 8 頂点の箱のメッシュ（`*_Col.obj`）。比べのため、visual のメッシュ（`<名前>.obj`）の頂点の外枠も数えた（作業用の小さいスクリプト、
`<scale>` はどの SDF にもない）。

| モデル | 測った足跡（モデルの x × y）× 高さ、ずれ、z_min（collision） | visual のメッシュの外枠 | 設計書 §12.2 | 違い |
|---|---|---|---|---|
| Suitcase1 | 0.290914 × 0.467992 × 0.751915、(0, 0)、0.0 | 0.2941 × 0.4873、z 0.0003〜0.7701、中心 (−0.0001, −0.0073) | 測っていない（上限 0.6 × 0.4 × 0.8） | — |
| Sofa | 1.04353 × 1.8 × 0.942733、(0.021071, 0)、−0.00012 | — | 1.044 × 1.800 × 0.943、(+0.02, 0) | ずれ x が 1 mm |
| TrashBin | 0.265936 × 0.375438 × 0.446126、(0, 0)、−0.0005 | — | 0.266 × 0.375 × 0.446、0 | なし（1 mm 未満） |
| LampAndStand | 0.624152 × 0.624152 × 1.019779、(0, 0)、0.0 | 0.6222 × 0.5246、z 0〜1.0209 | 0.622 × 0.525 × 1.021、0 | **y が 0.099 m**（collision は正方形の箱） |
| Table | 1.5 × 0.8 × 1.015、(0, 0)、0.0 | — | 1.500 × 0.800 × 1.015、(0.002, 0) | ずれ x が 2 mm |

- 設計書の 4 つの値は、カメラの試しの枠（描いた形 = visual）から取ったもので、ランプの visual のメッシュの外枠と一致する。
  `model_extent.py` は collision を測るので、ランプだけ y が 0.099 m 大きく出る。計画のとおり、4 つは設計書の値のままにした。
- Suitcase1 の collision の箱は、visual より 3 mm × 19 mm 小さく、18 mm 低い（取っ手とふくらみの分）。足跡には計画どおり
  `model_extent.py` の値を使った。違いは位置と大きさのどの線（§13.3：0.15 m・0.35 m・0.4 m）よりずっと小さい。
- Sofa と TrashBin の z_min（−0.12 mm、−0.5 mm）は床の誤差で、置く高さは設計書のとおり 0。
- Suitcase1 の `model.config`：作者 Wan Yi Seow、説明は青い大きいスーツケース（取っ手は下げてある）。
  ライセンスの項目はない。

### 固定：`gazebo/layouts/household.json` の `suitcase_1`

| 項目 | 前（計画の上限） | 後（H1） |
|---|---|---|
| `footprint.size`（モデルの軸） | [0.6, 0.4] | [0.291, 0.468] |
| `footprint.offset` | [0.0, 0.0] | [0.0, 0.0] |
| `height` | 0.8 | 0.752 |
| `z` | なし（0） | なし（0。z_min が 0.0 なので、ほかのモデルと同じく既定の 0） |
| `yaw_deg` | 0 | **90** |
| 世界での足跡（x × y） | 0.6 × 0.4 | 0.468 × 0.291 |
| `x`, `y`、`detectable`、`accept` | −1.5, 1.5、false、["suitcase"] | 同じ |

- **なぜ 90° か：** 設計書 §12.3 の表は suitcase_1 の向きを「x の長さ ≤ 0.6」、世界での足跡を「≤ 0.6 × 0.4」とした。0° のままでは
  世界で 0.291 × 0.468 になり、y が 0.4 を超える。90° 回すと 0.468 × 0.291 で上限に収まる（`yaw_deg` は 90° の倍数だけ、§12.5）。
  長い辺は世界の x の軸に合う（§12.3 の「長い辺は世界の軸に合わせる」）。§12.5 の下書きの「どれも yaw 0」とはここだけ違う。
- **§12.1 の決まりの確かめ**（`tests/test_household.py` と、作業用のスクリプトで同じ関数を呼んで）：
  - 始めの位置の後ろで足跡の縁が x ≤ −1.2：−1.266（0.6 × 0.4 のときは −1.2 ちょうど）。
  - 始めの位置のカメラの 120° の外、試験の姿勢 B の視野の中（`test_each_test_pose_sees_its_targets`）。
  - 受け入れの回の PLANT（`F around_box_1`、`F near_chair_1`、挨拶つき、`F near_box_1`）は、5 つの始めと 2 組の大きさのどれでも、
    新しい物があってもなくても同じ（`test_acceptance_plants_are_unchanged_by_the_household_objects`）。
  - 10 の試験の式は、試験の姿勢と 4 つのずれのどれでも計画がある（周回の輪とほかの物の塞ぐ所の間は、計画で確かめる）。

### 動いた数（設計書 §12.3・§12.4 と違う。報告）

| 数 | 設計書（上限 0.6 × 0.4） | H1 の測った足跡（90°） | 0° なら（比べ） | visual の外枠・90° なら（比べ） |
|---|---|---|---|---|
| `F around_suitcase_1` の PLANT の状態 | 8 736 | **8 064** | 8 064 | 8 064 |
| `F around_suitcase_1` のずれのいちばん大きい面積 | 121 | **110** | 110 | 121 |
| `F around_suitcase_1` の面積 / 手 | 121 / 39 | 121 / 39 | 121 / 39 | 121 / 39 |
| `F near_suitcase_1`（面積 / 手 / キー / PLANT / ずれの面積） | 64 / 4 / 8 / 1 320 / 72 | 同じ | 同じ | 同じ |
| いちばん近いほかの足跡まで：スーツケース | 2.12（コードでは 2.10） | **2.17** | 2.24 | 2.16 |
| 同：ソファ（いちばん近いのはスーツケース） | 2.12（コードでは 2.10） | **2.17** | 2.24 | 2.16 |
| 同：テーブル（いちばん近いのはスーツケース） | 2.30 | **2.35** | 2.27 | 2.35 |

- 「コードでは」は、ソファの 0.02 m のずれを入れた、H1 の前のテストの固定の値。ほかの試験の式（ゴミ箱・ソファ・ランプ・テーブル）の
  数は変わらない。
- `tests/test_household.py` の固定のうち、スーツケースの行（配置の行、世界の足跡、`F around_suitcase_1`、間の 3 つ）だけを、
  コードの数に替えた。どの行にも設計書の数を横に書いてある。設計書の数の直しは Task 4.1（docs）で。

### テスト

- `T tests/test_layout.py tests/test_fuel_image.py tests/test_household.py`：113 passed（21.2 秒）。
- 全部（`T tests nl_control/tests`）：2553 passed, 18 skipped, 29 deselected（287.6 秒）。`feat/open-objects` の `21e1a1f` と同じ数。
- 変えたファイルは `docker/fuel_models.json`、`gazebo/layouts/household.json`、`tests/test_layout.py`、`tests/test_household.py`、
  この記録だけ。PR #4 のファイル（D12）には触れていない。

## H7：遅いテスト（2026-11-12）

- 枝：`objects2/h7-slow`（`feat/open-objects` の `fca0522` から）。worktree `~/work/physicalAI/.go2-worktrees/oo-h7`。ROS の domain は 67。
- 場所：この Mac の Docker Desktop（VM は 8 CPU、7.7 GiB）。app のイメージ `go2-voice-ltl:dev`（`a35b9bb8c8d3`）を、計画の `TA` の形
  （本体の checkout の compose project の `app` の `run --rm`、worktree を `/wt` に）で使った。Gazebo も compose project `go2oo` も使っていない。
- 1 回で済んだ（前の試みはない）。コードもテストも変えていない。変えたのはこの記録だけ。

### 結論

**H7 の線はすべて満たした。** 遅いテスト 29 件（時間のテスト 3 件と、手前の全部の掃き出し 26 件）がすべて通った。
ソファの手前の合成は 0.40 秒（線は 1 秒）、周回は 1.52 秒（線は 3 秒）。掃き出しは 1 つの CPU で 9 分 3 秒（計画の見込みは約 30 分）。

| 線（計画 H7） | 結果 |
|---|---|
| D11 の門、ほかのこちらの container がない | 済み。3 回の記録のどれでも `docker ps` は空（`go2-gazebo-1` もなし） |
| 時間のテスト（`nice` なし、CPU の固定なし）が通る | **3 passed**（26.77 秒） |
| ・ソファの手前 ≤ 1 秒（設計書 §13.1） | **0.40 秒**（PLANT 3 144、SUP 6 180） |
| ・ソファの周回 ≤ 3 秒（設計書 §13.1） | **1.52 秒**（PLANT 14 256、SUP 14 256） |
| ・今の「式が大きすぎます」の断り < 25 秒 | **20.03 秒** |
| ・Whisper の録音（`stand_hello.wav` に「挨拶」） | 通った（2.45 秒。`faster-whisper-small` は本体の `hf-cache` の volume に 2026-11-04 からあり、取りに行っていない） |
| 手前の全部の掃き出し（M13：`nice -n 19`、CPU 0 だけ）が通る | **26 passed**（542.88 秒 = 9 分 3 秒） |
| 2 つの時間と前後の CPU を記録 | この節 |

### D11 の門と、ほかの container・電源

どの回の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった。こちらのほかの重い段や `T` の回もなかった
（host で `pytest` の process なし）。`load` は host の `sysctl -n vm.loadavg`（1・5・15 分）。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | `lowpowermode` | load |
|---|---|---|---|---|---|
| 13:38:17 | 時間のテストの前 | なし | 電池、99%、discharging | 0 | 2.64 3.03 4.91 |
| 13:39:02 | 時間のテストの後・掃き出しの前 | なし | 電池、99%、discharging | 0 | 3.15 3.08 4.82 |
| 13:39 ごろ | 掃き出しの間（1 回だけ見た） | こちらの `go2-app-run-…` だけ、CPU 101.92 %、82.75 MiB | — | — | — |
| 13:48:30 | 掃き出しの後 | なし | 電池、96%、discharging | 0 | 2.33 2.44 3.62 |

- 掃き出しの間の 101.92 % は CPU 1 つ分で、`taskset -c 0` が効いていた。
- 終わった後に残った container はない（`docker ps -a` も空）。ほかに動かしたのは、`hf-cache` の中を見るだけの使い捨ての
  `docker run --rm --name oo-h7-hfcache-ls`（alpine、読むだけ）。

### 時間のテスト

```bash
TA python3 -m pytest -q -m slow tests nl_control/tests --deselect tests/test_grid.py::test_near_keys_full_sweep --durations=0 -rP
```

- 計画の形に、時間を出すだけの `--durations=0 -rP` を足した（選ぶテストと測り方は変わらない）。13:38:26〜13:38:54、`real` 27.95 秒。
- 選ばれたのは 3 件（2 597 deselected）：

| テスト | 時間（`call`） | 中で測った値 | 線 |
|---|---|---|---|
| `tests/test_sim_household.py::test_sofa_synthesis_is_fast` | 1.94 秒 | `F near_sofa_1`：PLANT 3 144、SUP 6 180、0.40 秒。`F around_sofa_1`：PLANT 14 256、SUP 14 256、1.52 秒 | 1 秒・3 秒 |
| `tests/test_synthesis_memory.py::test_latent_go2_formula_is_refused_in_time` | 20.03 秒 | 「式が大きすぎます」で断った | < 25 秒 |
| `tests/test_stt.py::test_transcribes_japanese_fixture` | 2.45 秒 | 「挨拶」が出た | — |

- ソファの数は試験の姿勢 B（`GO2_LAYOUT=household`、立って (0, 0)、向き π）から。設計書 §13.1 の【仮】の見込み（手前 約 0.45 秒、
  周回 1.39 秒、どちらも混んだ Mac）に近い。周回は 0.13 秒多いが、線の半分ほど。
- 断りの 20.03 秒は線 25 秒まで約 5 秒（20 %）。静かな Mac での値なので、混んだ Mac では線に近づきうる（今回は落ちていない。報告だけ）。

### 手前の全部の掃き出し（M13、O32）

```bash
TA nice -n 19 taskset -c 0 python3 -m pytest -q -m slow tests/test_grid.py::test_near_keys_full_sweep --durations=0
```

- 計画の形に `--durations=0` だけを足した。13:39:07〜13:48:11、`real` 543.82 秒。
- 1 つの大きさで 102 400 通り（0.025 m・0.25° の 72 400 通りと、乱数の 30 000 通り）、26 の大きさで 2 662 400 通り。1 通りの平均は
  約 0.20 ms で、設計書 §10.10 の「1 通り約 1 ミリ秒」と計画の「約 30 分」より速かった。
- 大きさごとの時間（`call`、秒）：

| 大きさ | 秒 | 大きさ | 秒 | 大きさ | 秒 |
|---|---|---|---|---|---|
| 0.45 × 0.45 | 11.93 | 0.9 × 0.4 | 16.69 | 1.6 × 0.8 | 25.70 |
| 0.5 × 0.5 | 12.17 | 0.914 × 0.914 | 16.70 | 1.8 × 0.9 | 27.00 |
| 0.523 × 0.529 | 12.27 | 1.0 × 0.3 | 17.18 | 1.8 × 1.04 | 26.84 |
| 0.6 × 0.6 | 12.64 | 1.2 × 0.4 | 17.96 | 1.8 × 0.4 | 27.20 |
| 0.59 × 0.55 | 12.59 | 1.2 × 0.35 | 17.97 | 2.0 × 0.9 | 28.28 |
| 0.65 × 0.65 | 15.75 | 1.2 × 0.75 | 18.05 | 2.0 × 1.0 | 28.33 |
| 0.75 × 0.75 | 16.07 | 1.5 × 0.6 | 19.70 | 2.0 × 2.0 | 27.50 |
| 0.8 × 0.35 | 16.35 | 1.5 × 0.4 | 24.15 | 3.0 × 0.5 | 35.82 |
| — | — | 1.5 × 0.8 | 25.07 | 3.0 × 3.0 | 32.68 |

- どの大きさも、どの置き方でもキーの数が `FULL_SWEEP_LEAST` 以上（大きい物 8 以上、椅子と箱 4、測った箱 5、0.6 m 角と 0.59 × 0.55 が 6）で、
  どのキーも縁まで 0.65〜1.25 m だった。

### 遅いテストの数

- `T tests nl_control/tests`（`-m "not slow"`）の「29 deselected」が、この 29 件（3 + 26）。どれも通った。

## H2：検出器のイメージを作り、イメージの中でテストを流した（2026-11-12）

- 枝：`objects2/h2-detector`（`feat/open-objects` の `1ecb696` から）。worktree `~/work/physicalAI/.go2-worktrees/oo-h2`。ROS の domain は 62。
- 場所：この Mac の Docker Desktop（VM は 8 CPU、7.7 GiB）。Gazebo も compose project `go2oo` も使っていない（H2 は Gazebo を動かさない）。
- 1 回で済んだ（前の試みの worktree・枝・イメージはなかった）。コードもテストも変えていない。変えたのはこの記録だけ。

### 結論

**H2 の線はすべて満たした。** `go2-voice-ltl-detector:yoloe-11s` を 700.6 秒（11 分 41 秒）で作った。`docker image ls` の大きさは 4.51 GB
（content 1.32 GB）。重み 2 つは build の中で固定の sha256 と合った。A1：ultralytics 8.4.210 の `save_prompt_embeddings` /
`load_prompt_embeddings` がそのまま動き、検出器はその道で埋め込みを取っておき、読み戻した（`get_text_pe` の代わりの道も動くことを
別に確かめた）。イメージの中のテストは 15 passed と 192 passed で、落ちたもの・飛ばしたものはない。`detector/yoloe.py` は直していない。

| 線（計画 H2） | 結果 |
|---|---|
| build が通る。時間と `docker image ls` の大きさを記録 | 通った。700.6 秒、4.51 GB（下の「build」） |
| A1：`TD -m detector tests/test_detector_yoloe.py -k embedding_cache` | **1 passed**（20.0 秒）。道は ultralytics の save/load |
| ・どちらの道も動かなければ `detector/yoloe.py` を直して流し直す | 要らなかった（どちらの道も動いた） |
| `TD -m detector tests/test_detector_yoloe.py tests/test_detector_protocol.py` | **15 passed**, 80 deselected（54.0 秒） |
| `TD -m "not slow" tests/test_detector_geometry.py tests/test_detector_tracker.py tests/test_detector_core.py tests/test_detector_oracle.py tests/test_detector_check.py`（numpy 2.x） | **192 passed**（6.0 秒） |
| 直したものと記録を commit | 直したものはない。この記録だけ |

### D11 の門と、ほかの container・電源

どの回の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった。こちらのほかの重い段もなかった
（host で `docker build`・`pytest` の process なし）。`load` は host の `sysctl -n vm.loadavg`（1・5・15 分）。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | `lowpowermode` | load |
|---|---|---|---|---|---|
| 13:57:30 | build の前 | なし | 電池、94%、discharging | 0 | 2.23 2.24 2.95 |
| 14:09:49 | build の後・テストの前 | なし | 電池、92%、discharging | 0 | 2.46 2.21 2.45 |
| 14:12:29 | テストの後 | なし | 電池、92%、discharging | 0 | 3.22 2.73 2.63 |

- 動かしたのは、build と、計画の `TD`（`docker run --rm`、worktree を `/work` に、`env.txt` を `/dev/null` で隠す）と、名前を付けた
  使い捨ての `docker run --rm --name oo-h2-…`（A1 の確かめと、イメージの中を読むだけのもの）だけ。終わった後に残った container はない
  （`docker ps -a` も空）。

### build

```bash
docker build -t go2-voice-ltl-detector:yoloe-11s -f docker/Dockerfile.detector docker
```

- 計画の形に、段ごとの時間を出すだけの `--progress=plain` を足した。13:57:41〜14:09:22、`real` 700.55 秒。build の注意（warning）はない。
- 元の `ros:jazzy-ros-base-noble`（`sha256:8eed0a4669bb…`）は手元にあった（CACHED）。ほかの段は新しく作った：

| 段 | 秒 | 層の大きさ |
|---|---|---|
| `apt-get install`（`python3-venv libgl1 libglib2.0-0t64 ros-jazzy-rmw-cyclonedds-cpp curl ca-certificates`） | 42.0 | 211 MB |
| venv `/opt/det`（`--system-site-packages`） | 1.6 | 16.1 MB |
| torch 2.14.1+cpu・torchvision 0.29.1+cpu（CPU の wheel の index） | 42.5 | 843 MB |
| ultralytics 8.4.210・clip（`7ffa84b3…`）・pytest 8.4.2 | 114.1 | 492 MB |
| 重み 2 つ（`curl` と `sha256sum -c`） | 473.4 | 628 MB |
| `yolo settings weights_dir=/opt/yoloe sync=False` | 1.0 | 24.6 kB |
| 書き出し | 25.3 | — |

- 時間のほとんどは重みを取る段（`mobileclip_blt.ts` が 600 MB）。`sha256sum -c` の出力は `yoloe-11s-seg.pt: OK` と `mobileclip_blt.ts: OK`。
  イメージの中の大きさは 27 803 986 B と 599 764 649 B で、設計書 §11.1 と同じ。
- pip が入れた主なもの：numpy 2.5.4、opencv-python 5.0.0.93、pillow 13.0.0、matplotlib 3.11.2、polars 2.1.0、clip 1.0。Python は 3.12.3。
- できたイメージ：`go2-voice-ltl-detector:yoloe-11s`（`sha256:016348e47306…`、2026-11-12 14:08:56）。`docker image ls` で
  DISK USAGE 4.51 GB、CONTENT SIZE 1.32 GB。設計書 §11.1 の見込み「約 3.5 GB」より大きい（線ではない。試しのイメージは重み 5 つで 4.8 GB）。
- イメージの中：`/opt/yoloe` には `yoloe-11s-seg.pt` と `mobileclip_blt.ts` と `config` だけ。ultralytics の設定は `weights_dir` が
  `/opt/yoloe`、`sync` が False。いつもの入り口（Jazzy を読み込む ENTRYPOINT）で、numpy は venv の 2.5.4（`/opt/det/…`）、
  ultralytics も venv から、rclpy は `/opt/ros/jazzy` から読まれた。

### A1：文字の埋め込みを取っておく道

```bash
TD -m detector tests/test_detector_yoloe.py -k embedding_cache -rA --log-level=INFO
```

- 計画の形に、ログを出すだけの `-rA --log-level=INFO` を足した（選ぶテストは変わらない）。**1 passed**, 23 deselected（20.02 秒）。
- 検出器のログ（90 の言葉。そのうち捨てる言葉 3）：1 回目の起動は符号化器で 9.44 秒（はじめのファイルの読み込みを含む）、
  2 回目の起動は取っておいたもので 0.00 秒、言葉の順を逆にした 3 回目は符号化器で 6.46 秒。モデルを読むのは 0.45・0.05・0.04 秒。
- どの道か：`detector/yoloe.py` は ultralytics の `save_prompt_embeddings` / `load_prompt_embeddings` を先に試し、失敗すると
  `not cached in … (ultralytics)` / `not loaded (ultralytics)` を warning で書いてから自分の npz の道に移る。そのログは 1 つも出なかった。
  `embedding_source` は "encoder" と "cache" だけを分けるので、どちらの道かは、このログと次の確かめから読んだ。
- 確かめ（作業用の場所の使い捨てのスクリプト。リポジトリには入れていない。`docker run --rm --name oo-h2-a1-probe`、worktree は読むだけ）：
  - ultralytics 8.4.210 の `YOLOE` に `save_prompt_embeddings` と `load_prompt_embeddings` がある。
  - 1 回目の起動で呼ばれたのは `save_prompt_embeddings`（成功）だけ、2 回目は `load_prompt_embeddings`（成功）だけ。検出器の
    自分の npz の読み書き（`_load_npz`・`_save_npz`）は呼ばれなかった。
  - できたファイル `yoloe-pe-<鍵>.npz` の中は `embeddings`（float32、1 × 90 × 512）、`names`（90）、`model`（モデルの名前）。
  - `set_vocabulary` 全体（空の絵の predict を含む）：符号化器で 6.62 秒、取っておいたもので 0.38 秒。その後のスレッドは
    torch 1・interop 1・cv2 1。
  - 代わりの道も 8.4.210 で動く：`model.get_text_pe(names)` が torch.float32 の 1 × 90 × 512 を 6.22 秒で返し、
    `model.set_classes(names, pe)` で 90 の名前が付いた。自分の npz に書いて新しいモデルで読み戻しても同じ。
- だから `detector/yoloe.py` はそのまま（ultralytics の道を先に、自分の npz を後に）。
- 符号化器の時間（90 の言葉、静かな Mac。はじめの 9.44 秒の後は、この節と次の節の 10 回で 6.09〜6.49 秒）は、設計書 §11.1 の【仮】（63 の言葉で 2.3〜2.9 秒、Gazebo の横で
  5.3〜8.5 秒）より長い。線ではない。取っておけば起動のたびにはかからない（単語帳の `detect` を変えた後の 1 回だけ）。

### イメージの中のテスト

```bash
TD -m detector tests/test_detector_yoloe.py tests/test_detector_protocol.py -rA --log-level=INFO
TD -m "not slow" tests/test_detector_geometry.py tests/test_detector_tracker.py tests/test_detector_core.py \
    tests/test_detector_oracle.py tests/test_detector_check.py -rfEs
```

- 1 つ目（14:10:50〜14:11:45）：**15 passed**, 80 deselected（54.01 秒）。`test_detector_yoloe.py` の `detector` の 5 つ
  （`test_prompts_of_one_label_merge`、`test_ignore_classes_are_dropped_and_still_suppress`、`test_agnostic_nms_is_on`、
  `test_threads_are_1_after_warm_up`、`test_embedding_cache_skips_the_encoder`）と、`test_detector_protocol.py` の `[yoloe]` の 10。
  埋め込みは符号化器から 8 回（90 の言葉で 7 回、6.09〜6.49 秒。3 つの言葉の 1 回は 1.17 秒）、取っておいたものから 10 回。
  検出器の warning はない。
  出た注意は torch の `torch.jit.load` の FutureWarning（ultralytics が `mobileclip_blt.ts` を読むところ）だけで、結果には関わらない。
- 2 つ目（14:11:56〜14:12:03）：**192 passed**（5.99 秒）。飛ばしたもの・選ばれなかったものはない。numpy 2.5.4 の上で、幾何・追跡・
  検出の核・正解の検出器・記録と流し直しの道具のテストが通った。

### 変えたもの

- この記録だけ。`detector/yoloe.py` を含めてコード・テスト・固定は変えていない。PR #4 のファイル（D12）には触れていない。
  app のイメージにも本来の Linux の準備にも、新しい依存は入っていない（ultralytics と torch は検出器のイメージの中だけ）。

## H3：検出の下見（A6 / M2、2026-11-12）

- 枝：`objects2/h3-preview`（`feat/open-objects` の `ced4e87` から）。worktree `~/work/physicalAI/.go2-worktrees/oo-h3`。ROS の domain は 63。
- 場所：この Mac の Docker Desktop（VM は 8 CPU、7.7 GiB）。Gazebo は compose project `go2oo`（画面なし、`go2oo-gazebo`）、
  検出は `go2-voice-ltl-detector:yoloe-11s`（H2 のイメージ、作り直していない）。
- 1 回で済んだ（前の試みの worktree・枝・記録はなかった）。コード・テスト・固定は変えていない。変えたのはこの記録だけ（下の「決めたこと」）。
- 絵（`out/h3-preview/`・`out/h3-o38/`）は CC BY のモデルを描いたものなので git に入れていない（設計書 §11.11）。worktree の中に残してある。

### 結論

**H3 の線は満たさなかった。** 計画の 16 の見え方で `preview` を通った新しい種類は **0**（線は 3 以上。設計書 §12.8 の 5、§13.4）。
O38 の最初の手（B に回った後に 1 歩前へ、ソファまで 3.87 m）で撮り直すと、**ソファだけは 5/5 で通る**（sofa 0.775〜0.873、IoU 0.78〜0.80）。
ゴミ箱・スーツケース・ランプは、自分の名前の確からしさが、試験の姿勢の見え方で `confirm` 0.35 に届かない。だから、間違いの言葉を外しても
（§12.8 の手 (1)）通らない。残る手は (2) 同じ種類の代わりのモデル（§12.9、H1 のやり直し）と (3) §12.1 の中での置き直しで、どちらも部屋を
変えるので、数を添えて controller に返す（手 (4)）。

| 線（計画 H3） | 結果 |
|---|---|
| `record_frames` が Fuel のイメージで起動する（Task 2.12 の Step 4 の形） | 計画の形のままでは **exit 1**（`rclpy` がない。下の「軽い確かめ」）。`PYTHONPATH` を ROS を読み込む前に渡す形（記録で使う `docker exec -e` と同じ）で **exit 0** |
| 17 枚を撮る（伏せ 1、立ち 1、B・L・R × 5） | 済み。17 枚とも exit 0 |
| 最初の絵で、どの Fuel のモデルも自分の見え方に写る（M15） | 写った（下の表）。ただしスーツケースは B では絵の右の端で半分切れる（向きの行き過ぎ） |
| `detector_check.py … --detector yoloe` | exit 1。`preview` の `detectable` は `[]`、`enough` は false。始めの位置の見え方は通った（box と chair のほかに確かになる名前なし） |
| 通った目標に `detectable: true` | 通った目標なし。O38 の 1 歩でソファが通る（下の「決めたこと」） |
| `BY_LABEL` の上書き | なし（当てはまる組がない） |
| 外す検出の言葉 | なし（どの言葉を外しても、通る目標が増えない） |
| 3 つに足りなければ (1)〜(4) | (1) では足りない。(2)・(3) は部屋を変えるので controller へ（下の「controller に返すこと」） |
| `T tests/test_household.py`、全部のテスト | 下の「テスト」 |

### D11 の門と、ほかの container・電源

どの Gazebo の回と検出の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった。こちらのほかの重い段もなかった。
`load` は host の `sysctl -n vm.loadavg`（1・5・15 分）。`lowpowermode` はどの時も 0。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | load |
|---|---|---|---|---|
| 14:23:37 | 回 1（伏せ・立ち・B）の前 | なし | 電池、89%、discharging | 1.66 2.05 2.30 |
| 14:25:49 | 回 1 の終わり（Gazebo あり） | `go2oo-gazebo` 190.27 %、1.003 GiB | 電池、88% | 3.96 2.60 2.47 |
| 14:25:50 | 回 1 を止めた後・回 2（L）の前 | なし | 電池、88% | 3.96 2.60 2.47 |
| 14:26:54 | 回 2 の終わり | `go2oo-gazebo` 223.92 %、779.5 MiB | 電池、88% | 3.23 2.62 2.48 |
| 14:26:56 | 回 2 を止めた後・回 3（R）の前 | なし | 電池、88% | 3.23 2.62 2.48 |
| 14:27:56 | 回 3 の終わり | `go2oo-gazebo` 205.51 %、774.7 MiB | 電池、87% | 3.90 2.89 2.59 |
| 14:27:58 | 回 3 を止めた後 | なし | 電池、87% | 3.90 2.89 2.59 |
| 14:28:13 | 検出（`detector_check`）の前 | なし | 電池、87% | 3.09 2.77 2.55 |
| 14:28:37 | 検出の後 | なし | 電池、87% | 2.69 2.70 2.53 |
| 14:30:21 | 種類ごとの確からしさの確かめの前 | なし | 電池、87% | 1.67 2.35 2.41 |
| 14:36:19 | その後 | なし | 電池、84% | 6.85 5.79 4.11 |
| 14:37:47 | 回 4（O38：B ＋ 1 歩）の前 | なし | 電池、84% | 4.47 5.13 4.01 |
| 15:52:48 | 回 4 の終わり（下の「眠り」の後） | `go2oo-gazebo` 263.01 %、778.8 MiB | 電池、82% | 60.59 26.24 12.85 |
| 15:52:51 | 回 4 を止めた後 | なし | 電池、82% | 60.59 26.24 12.85 |
| 15:53:29 | 回 4 の検出の前 | なし | 電池、82% | 32.97 23.44 12.44 |
| 15:53:51 | その後 | なし | 電池、82% | 24.31 22.08 12.21 |
| 15:54:05 | 言葉ごとの確からしさの確かめの前 | なし | 電池、82% | 19.66 21.16 12.05 |
| 15:57:49 | その後 | なし | 電池、80% | 10.77 15.05 11.41 |

- **眠り：** 回 4 で `Bf_s1` を撮った直後の 14:38:30 に Mac が蓋を閉じて眠った（`pmset -g log`：`Clamshell Sleep`）。15:52:33 に起きるまで、
  ときどき短く起きた間（DarkWake）に `Bf_s2`〜`Bf_s4` が撮れた。Gazebo のシミュレーションの時間は 24.0 → 44.0 秒と続いていて、どの組も
  止まりの門・位置の組（0.02 m・1°）・正解の確かめを通っている。だから絵は使える。眠りの間は検出も Gazebo も止まっていて、測った時間はない。
  15:52 の load の高さは、起きた直後のもの。
- 動かしたのは、`G` の Gazebo（`go2oo-gazebo`、`docker rm -f` と `G down` で止めた）と `G run --rm … app`（`run_formula`）、
  計画の `TA`・`T`、名前を付けた使い捨ての `docker run --rm --name oo-h3-…`（検出器のイメージと Fuel のイメージ）だけ。

### 軽い確かめ：`record_frames` を Fuel のイメージで

```bash
docker run --rm -v "$WT":/wt -w /wt --entrypoint bash go2-voice-ltl-gazebo:patch-0005-fuel1 -c \
  'source /opt/ros/jazzy/setup.bash && command -v gz && PYTHONPATH=/wt:/wt/nl_control python3 -c "import numpy, rclpy, …"'
```

- 計画の形（Task 2.12 の Step 4 のタグを `patch-0005-fuel1` にしたもの）は **exit 1**：`gz` の場所は出るが、`ModuleNotFoundError: No module named 'rclpy'`。
  `patch-0005` でも同じ。わけ：`PYTHONPATH=… python3` は、`setup.bash` が足した ROS の場所を消して置き換える。
- `PYTHONPATH` を `docker run -e PYTHONPATH=/wt:/wt/nl_control` で先に渡す（`setup.bash` がその前に ROS の場所を足す。記録で使う
  `docker exec -e PYTHONPATH=…` と同じ形）と **exit 0**、`gz` は `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`。道具には問題がなく、
  計画の確かめの書き方の誤り。

### 撮った見え方（17 枚、`out/h3-preview/`）

```bash
OBJECTS_SOURCE=detector G run -d --no-deps --service-ports --name go2oo-gazebo gazebo \
    ros2 launch /work/gazebo/launch/go2_gazebo.launch.py gui:=false
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "<式>"
docker exec -e PYTHONPATH=/work:/work/nl_control go2oo-gazebo bash -c \
    'source /opt/ros/jazzy/setup.bash && python3 /work/gazebo/tools/record_frames.py /work/out/h3-preview --view <名前> [--shift=<s>]'
```

- 回 1：起動してすぐ `spawn_lying`（`--timeout 180` を足した。Gazebo の立ち上がりを待つため。撮れたのはシミュレーションの 1.0 秒）、
  `F standing`、`spawn`、`F(turned_left & X F turned_left)`、`B`、`B_s1`〜`B_s4`。回 2：起動し直し、`F turned_left`、`L`、`L_s1`〜`L_s4`。
  回 3：起動し直し、`F turned_right`、`R`、`R_s1`〜`R_s4`。どの `run_formula` も「結果：done」。
- ずらしは計画の 4 つ。負の数で始まるずらしは `--shift -0.1,…` と書くと argparse が別の引数とみなして exit 2 になるので、`--shift=-0.1,0.1,-5` と書いた
  （`B_s3` で 1 回ずつ失敗して撮り直した。ほかの形は同じ）。
- 回 2・回 3 は Gazebo の起動直後に `run_formula` を始めたので、app の側に rmw の `string data is not null-terminated` が何行か出た（立ち上がり中の
  文字列のトピック）。動きは変わらず、done まで行った。
- 1 枚あたり 2〜8 秒（ずらしは置き直しと 1 秒の落ち着きを含む）。

| 見え方 | 本当の向き（odom） | 位置 m | 論理カメラが見た物 | 方位（左 +）・距離 |
|---|---|---|---|---|
| spawn_lying | −1.5° | (0.004, 0.002)、高さ 0.068 | box_1, chair_1 | — |
| spawn | −1.2° | (0.037, −0.024) | box_1, chair_1 | 箱 +32.9° 2.90 m、椅子 −29.7° 2.87 m |
| B | **−168.5°**（左に 191.5° 回った） | (0.043, −0.024) | sofa_1, trash_can_1 | ソファ −11.9° 4.42 m、ゴミ箱 +32.2° 2.14 m、スーツケース **−56.2°** 2.17 m |
| L | 96.1° | (0.011, 0.037) | suitcase_1, table_1 | スーツケース +39.8° 2.10 m、テーブル +22.8° 4.98 m |
| R | −95.8° | (0.027, 0.006) | lamp_1, trash_can_1 | ランプ +15.8° 4.47 m、ゴミ箱 −39.6° 2.14 m |

- ずらした 4 枚ずつは、どれも同じ姿勢（物を動かして見え方をずらす）。B のずらしで論理カメラがスーツケースを見たのは `B_s2`・`B_s3`（−5°）だけ。
- **向きの行き過ぎ：** `run_formula` の 1 回の左・右は 96.1°・−95.8°、2 回の左は 191.5°（向き −168.5°）。前の記録（`gazebo_turn_fix.md` の
  demo_script：94°、2 回で −168°）と同じ。設計書 §12.3 の方位（B でスーツケースが右前 45°）は、ちょうど 90°・180° として出したもの。
- **M15：** 最初の絵（各見え方の plain）で、5 つの Fuel のモデルはどれも写った。TrashBin（B・R）、Sofa（B）、Suitcase1（L は全体、B は右の端で
  半分）、Table（L）、LampAndStand（R、小さい台の上のランプ）。箱と椅子は spawn に写った。

### 検出（`detector_check.py`、YOLOE-11s 文字 @ 640、87 の言葉 + 捨てる 3 つ）

```bash
docker run --rm -v "$WT":/work -v /dev/null:/work/env.txt:ro -w /work go2-voice-ltl-detector:yoloe-11s \
    python3 gazebo/tools/detector_check.py /work/out/h3-preview --detector yoloe
```

- 14:28:13〜14:28:33、`real` 19.1 秒、**exit 1**。17 枚、`errors` なし。推論 1 回の時間は中央値 324.8 ms、最大 343.6 ms（1 スレッド、静かな Mac）。
- 絵ごとの当たり（`sighting` 0.25 以上、名前・確からしさ、どの正解の枠に重なったか。agnostic NMS の後なので、1 つの物に 1 つの名前）：

| 見え方 | 当たり |
|---|---|
| spawn_lying | chair 0.729・0.418（chair_1）、box 0.302（box_1）（`preview` は見ない） |
| spawn | なし |
| B | **chair 0.734（sofa_1）**、shelf 0.500（trash_can_1） |
| B_s1 | sofa 0.811、shelf 0.559（ゴミ箱） |
| B_s2 | sofa 0.721、television 0.665（suitcase_1、端で切れた）、shelf 0.549（ゴミ箱） |
| B_s3 | **chair 0.751（sofa_1）**、shelf 0.541（ゴミ箱）、television 0.430（スーツケース） |
| B_s4 | sofa 0.774、**trash_can 0.569**、television 0.267（どの正解にも重ならない） |
| L | なし |
| L_s1 | box 0.267（スーツケース）、desk 0.260（テーブル） |
| L_s2 | desk 0.507（テーブル） |
| L_s3 | trash_can 0.555（スーツケース） |
| L_s4 | trash_can 0.317（スーツケース）、desk 0.296（テーブル） |
| R | desk 0.361（lamp_1 の枠、IoU 0.53）、box 0.327（ゴミ箱）、lamp 0.313（ランプの笠だけ、IoU 0.18） |
| R_s1 | desk 0.256（ランプの枠） |
| R_s2 | desk 0.561（ランプの枠、IoU 0.49）、potted_plant 0.412（笠、IoU 0.17）、box 0.381（ゴミ箱） |
| R_s3 | lamp 0.337（笠、IoU 0.17）、box 0.310（ゴミ箱）、desk 0.305（ランプの枠） |
| R_s4 | box 0.451（ゴミ箱）、chair 0.266（左の端、どの正解にも重ならない） |

- `preview`（設計書 §12.8 の 3）：

| 目標 | 姿勢 | 姿勢の見え方 | ずらしで当たった数（線は 2） | 他の目標の名前 | 合否 |
|---|---|---|---|---|---|
| sofa_1 | B | 外れ（chair 0.734） | 3（s1 0.811、s2 0.721、s4 0.774） | なし | **不合格** |
| trash_can_1 | B | 外れ（shelf 0.500） | 1（s4 0.569） | なし | **不合格** |
| suitcase_1 | B | 外れ（正解の枠に当たりなし） | 0 | なし | **不合格** |
| lamp_1 | R | 外れ | 0 | なし | **不合格** |
| spawn（始めの位置） | — | box と chair のほかの名前なし | — | — | 合格 |

- 絵ごとの正しさの線（§13.3）は、L・L_s1・L_s4・spawn・spawn_lying のほかはどれも落ちた（名前の誤りと、誤った名前の大きさで補った足跡）。
  H3 の線ではないが、H6 のために残す。

### わけを探した：種類ごと・言葉ごとの確からしさ

`detector_check` の取り違えの表は agnostic NMS の後なので、1 つの物に勝った名前しか出ない。言葉の直しで勝ち負けが変わるかを見るため、
使い捨てのスクリプト（作業用の場所。リポジトリには入れていない）で、同じ YOLOE と同じ埋め込みを使い、1 つの種類（またはその 1 つの言葉）
だけを `set_classes` して、各正解の枠（`detector_check.truth_boxes`、配置の高さ）と IoU 0.5 以上の箱のいちばん高い確からしさを数えた
（`conf` 0.05）。YOLOE の確からしさは名前ごとの sigmoid なので、ほかの名前があってもなくても同じ。全部の一覧で勝った名前の値（chair 0.734、
trash_can 0.569、trash_can 0.555 など）と、1 つずつの値が一致することも確かめた。14:30〜14:36 と 15:54〜15:58。

ソファ（sofa_1）：

| 見え方 | 距離 | sofa | couch | chair | office chair | bench |
|---|---|---|---|---|---|---|
| B | 4.42 m | 0.307 | 0.660 | **0.734** | 0.296 | 0.342 |
| B_s1 | | 0.518 | **0.811** | 0.513 | 0.102 | 0.202 |
| B_s2 | | 0.347 | **0.721** | 0.677 | 0.183 | 0.308 |
| B_s3 | | 0.348 | 0.657 | **0.751** | 0.304 | 0.364 |
| B_s4 | | 0.441 | **0.774** | 0.659 | 0.165 | 0.341 |
| Bf | 3.87 m | 0.531 | **0.775** | 0.716 | 0.200 | 0.274 |
| Bf_s1 | | 0.707 | **0.873** | 0.404 | — | 0.196 |
| Bf_s2 | | 0.660 | **0.850** | 0.448 | 0.069 | 0.181 |
| Bf_s3 | | 0.427 | **0.789** | 0.672 | 0.154 | 0.181 |
| Bf_s4 | | 0.611 | **0.819** | 0.548 | 0.128 | 0.242 |

- ソファの種類の確からしさ（`couch` が引く）はどの見え方でも 0.35 を十分に超える。B と B_s3 で負けるのは、chair の言葉 `chair`
  （0.734・0.751）にだけ。4.4 m で絵ごとに五分五分という設計書 §12.3 の見込み（O38）のとおり。3.87 m ではどの見え方でも couch が勝つ。

ゴミ箱（trash_can_1）、主な言葉：

| 見え方 | trash can | garbage bin | wastebasket | shelf | cabinet | chest of drawers | dresser | box | stool |
|---|---|---|---|---|---|---|---|---|---|
| B | 0.093 | 0.084 | 0.251 | **0.500** | 0.481 | 0.443 | 0.271 | 0.246 | 0.264 |
| B_s1 | 0.053 | — | 0.227 | **0.559** | 0.543 | 0.480 | 0.265 | 0.256 | 0.400 |
| B_s2 | 0.164 | 0.161 | 0.307 | **0.549** | 0.520 | 0.512 | 0.278 | 0.270 | 0.214 |
| B_s3 | 0.063 | 0.053 | 0.204 | **0.541** | 0.521 | 0.453 | 0.337 | 0.184 | 0.232 |
| B_s4 | 0.234 | 0.203 | **0.569** | 0.288 | 0.276 | 0.116 | 0.058 | 0.148 | 0.192 |
| R | 0.074 | 0.094 | 0.162 | 0.254 | 0.213 | — | — | **0.327** | — |
| R_s1 | — | — | — | 0.128 | 0.115 | — | — | **0.159** | — |
| R_s2 | 0.067 | 0.077 | 0.141 | 0.300 | 0.264 | 0.073 | 0.080 | **0.381** | — |
| R_s3 | 0.107 | 0.093 | 0.279 | 0.221 | 0.192 | — | — | **0.310** | — |
| R_s4 | 0.075 | 0.070 | 0.183 | 0.318 | 0.281 | 0.099 | 0.098 | **0.451** | — |
| Bf（1.75 m） | 0.083 | 0.079 | 0.166 | 0.597 | **0.602** | 0.479 | 0.438 | 0.376 | 0.133 |

- ゴミ箱の 3 つの言葉のいちばん高い値は、B の 5 枚で 0.251・0.227・0.307・0.204・0.569、R の 5 枚で 0.279 まで。**試験の姿勢 B の見え方で
  0.35 に届かない**ので、shelf・cabinet・dresser・box・stool を全部外しても通らない。上の口の開いた焦げ茶の箱が、横から棚・戸棚に見えている。

スーツケース（suitcase_1）：

| 見え方 | suitcase | television | tv | trash can | box |
|---|---|---|---|---|---|
| B | 0.055 | 0.203 | 0.118 | 0.159 | 0.124 |
| B_s1・B_s4 | — | — | — | — | — |
| B_s2 | 0.060 | **0.665** | 0.589 | 0.050 | 0.065 |
| B_s3 | — | **0.430** | 0.329 | — | 0.085 |
| L | — | — | — | — | 0.102 |
| L_s1 | 0.185 | 0.062 | 0.055 | 0.122 | **0.267** |
| L_s2 | 0.132 | 0.159 | 0.063 | — | 0.079 |
| L_s3 | 0.322 | 0.205 | 0.135 | **0.527** | 0.360 |
| L_s4 | 0.121 | 0.078 | 0.071 | 0.245 | 0.262 |

- B ではスーツケースは −56°（端で半分切れる。向きの行き過ぎのため）で、青い四角が television と呼ばれる。L は同じ (0, 0) から同じ面を
  （絵の別の場所で）全体に見ているが、`suitcase` は 0.322 まで。だから、行き過ぎがなく B でちょうど 45° に見えても、0.35 に届く見込みは低い。
  言葉の直しでは通らない。

ランプ（lamp_1、正解の枠は台とランプを合わせた 0.62 × 0.52 × 1.02 m、4.47 m 先で約 31〜41 × 49〜55 px）：

| 見え方 | floor lamp | lamp | desk | office desk | table | coffee table | bench |
|---|---|---|---|---|---|---|---|
| R | — | 0.066 | **0.361** | 0.099 | 0.050 | 0.103 | 0.094 |
| R_s1 | — | 0.058 | **0.256** | 0.056 | — | 0.054 | 0.071 |
| R_s2 | — | — | —（IoU 0.49 の desk 0.561） | — | — | — | — |
| R_s3 | — | — | **0.305** | 0.114 | 0.055 | 0.089 | 0.138 |
| R_s4 | — | 0.145 | **0.237** | 0.051 | — | 0.081 | 0.060 |

- 枠全体には `lamp` は 0.145 まで、`floor lamp` は出ない。`lamp` は笠だけを 0.313・0.337（R・R_s3、IoU 0.17〜0.18）で見ている。台は desk。
  desk を外しても枠の名前は table・bench（0.25 まで）になり、ランプは通らない。

### O38：B に回った後に 1 歩前へ（回 4、`out/h3-o38/`）

設計書 §12.8 の 5（O38）の最初の手。部屋は変えず、試験の手順が 1 歩長くなる。

```bash
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F(turned_left & X F turned_left)"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F moved_forward"
# record_frames.py /work/out/h3-o38 --view Bf、Bf_s1 … Bf_s4（同じ 4 つのずらし）
```

- 起動し直した Gazebo で。向き −169.2°、位置 (−0.506, −0.062)、ソファまで 3.87 m（方位 −11.7°）、ゴミ箱は +44.5° 1.75 m、スーツケースは見えない。
- `detector_check` の `PREVIEW_TARGETS` は sofa_1 を "B" に固定しているので、同じ関数（`detector_check.replay`・`preview`）を目標
  `{"sofa_1": "Bf"}` で呼ぶ使い捨てのスクリプトで判定した（15:53:29〜15:53:51）：

| 見え方 | 当たり | ソファの組（位置の誤り・大きさの誤り） |
|---|---|---|
| Bf | **sofa 0.775**（IoU 0.780）、cabinet 0.602（ゴミ箱） | 0.138 m・0.163 m |
| Bf_s1 | **sofa 0.873**（0.785）、cabinet 0.605（ゴミ箱） | 0.078・0.118 |
| Bf_s2 | **sofa 0.850**（0.778）、box 0.380（左の端、どの正解にも IoU 0.3 未満） | 0.074・0.123 |
| Bf_s3 | **sofa 0.789**（0.797）、dresser 0.506（ゴミ箱） | 0.083・0.152 |
| Bf_s4 | **sofa 0.819**（0.802）、shelf 0.585（ゴミ箱） | 0.077・0.185 |

- `preview`：sofa_1 は姿勢の見え方で当たり、ずらし 4/4、他の目標の名前なし → **合格**。推論 1 回は 330〜356 ms。
- ソファの位置と大きさは §13.3 の線（0.15 m・0.4 m）の中。ただし同じ絵のゴミ箱が cabinet・dresser・shelf と呼ばれ、その種類の大きさで
  補った足跡になる（位置の誤り 0.33 m、大きさの誤り 0.74 m）。正しさの線は組のすべてで数えるので、この見え方の名前の正しさ（0.5）と
  大きさの線は、ゴミ箱のために落ちる。
- **1 歩先からの計画**（`tests/test_household.py` の関数をそのまま使う使い捨てのスクリプト、app のイメージ、PLANT だけ、合成なし）：

| 式 | 始め | 面積 | 手 | 手前のキー | PLANT の状態 | 4 つのずれ（面積・手） |
|---|---|---|---|---|---|---|
| `F near_sofa_1` | B（設計書） | 165 | 6 | 18 | 3 144 | (150,6)(165,6)(165,6)(150,6) |
| `F near_sofa_1` | B（測った向き −168.5°） | 165 | 6 | 21 | 3 264 | (150,6)(165,6)(165,6)(165,6) |
| `F near_sofa_1` | B ＋ 0.5 m（−0.5, 0, 180°） | 154 | 5 | 18 | 2 880 | (140,5)(154,5)(154,5)(140,5) |
| `F near_sofa_1` | Bf（測った、−169.2°） | 140 | 5 | 22 | 2 640 | (154,5)(140,5)(140,5)(154,5) |
| `F around_sofa_1` | B（設計書） | 221 | 50 | — | 14 256 | (221,50)(221,54)(221,55)(238,65) |
| `F around_sofa_1` | B（測った向き） | 221 | 51 | — | 14 592 | (238,49)(221,50)(221,51)(221,56) |
| `F around_sofa_1` | B ＋ 0.5 m | 208 | 49 | — | 13 632 | (224,55)(208,51)(208,54)(224,64) |
| `F around_sofa_1` | Bf（測った） | 224 | 54 | — | 14 208 | (208,51)(224,59)(208,56)(208,55) |

- どの始めでも、手前と周回の計画がある。1 歩先の数は H5・H6 と Task 4.1 のための参考で、固定はしていない。

### 決めたこと

- **数える種類（`detectable`）：** この部屋で `preview` を通るのは sofa_1 だけで、それも O38 の 1 歩（試験の姿勢 B ＋ `F moved_forward`）から。
  sofa_1 は配置で `true` のまま（O38 の手順で通る）。trash_can_1・suitcase_1 は `false` のまま。**lamp_1 は H3 の判断では `false`**（枠に
  ランプの名前が 0.35 で当たらない）。ただし配置の `lamp_1` を `false` にすると、Task 2.12 のテスト
  `tests/test_detector_check.py::test_detector_check_command_line_with_the_oracle` が落ちた（そのテストは見出しで「H3 が配置を変えても
  変わらない」と書くが、この 1 つは `--layout household` で本物の配置ファイルを読み、R が `no_recall` に入る）。ほかの task のファイルは
  直さない決まりなので、配置は元に戻し、`lamp_1` は `true` のまま controller に返す。
- **`BY_LABEL`：** なし（`{}` のまま）。計画の決まり「間違いの名前が 0.35 と正しい名前の確からしさの間にあるとき」に当たる組が、取り違えの
  表にない。表に出た間違いの名前（ソファの chair 0.734・0.751、ゴミ箱の shelf・cabinet・dresser・box、スーツケースの television・trash_can、
  ランプの desk）は、どれも正しい名前より高いか、正しい名前が 0.35 に届かない。上書きは間違いの名前を確かにしないだけで、正しい名前を
  出しはしない（agnostic NMS で 1 つの物に 1 つの名前）。
- **外す検出の言葉：** なし（`vocabulary.json` は変えない）。ゴミ箱・スーツケース・ランプは自分の名前が試験の姿勢で 0.35 に届かないので、
  競う言葉（box・desk・cabinet・bench・stool、shelf・dresser も）を外しても通らない。外すと、次に高い言葉に名前が移るだけ（ゴミ箱なら
  shelf・cabinet・dresser 0.44〜0.60）。B のソファを通す言葉の直しは chair の `chair` を外すことだけで、椅子という基本の種類の検出の言葉を
  なくす。O38 の 1 歩で、言葉を変えずに通るので、外さない。

### controller に返すこと

1. **3 つに足りない（1 つ）。** 残る手は部屋を変える：(2) 代わりのモデル（§12.9）：ゴミ箱 → FIRST 2015 trash can（CC0）か LitterBin（CC BY 4.0）、
   スーツケース → Suitcase2（CC0）。ランプには §12.9 の代わりがない（小さい予備の目標：Construction Cone → traffic_cone、Backpack・
   JanSport Backpack Red → backpack）。どれも H1 のやり直し（取得・固定・Fuel のイメージの作り直し）と H3 の撮り直しが要る。
   (3) §12.1 の中での置き直し（ランプを 4.47 m より近く、スーツケースを向きの行き過ぎの後も視野の中に）。(4) 別の大きさのモデル（11m）や別の検出器。
2. **O38 を採るなら：** ソファの試験の姿勢は B ＋ 1 歩（3.87 m）。`detector_check.PREVIEW_TARGETS` の sofa_1 は "B"（Task 2.12 のファイル）で、
   今の道具では B で判定される。H5・H6 の手順と、§12.4 の数（上の表）は Task 4.1 で。
3. **lamp_1 の `detectable`：** 上のとおり、テスト 1 つが本物の配置を読むため、`false` にできなかった。テストを凍った `SCENE_LAYOUT` に
   するか、期待を R 込みにする直しが要る（Task 2.12 のファイル）。
4. **向きの行き過ぎ：** 1 回の旋回で約 6°、2 回で 11.5°。§12.3 の方位はちょうど 90°・180° で出したもの。置き直すなら、測った向きで。
5. **B・Bf のゴミ箱の名前の誤り** は、ソファの試験（H6）の §13.3 の正しさの線（名前の正しさ、大きさ）も落とす（ゴミ箱は範囲の中）。
   1 と合わせて考える必要がある。
6. 計画の書き方：Task 2.12 の Step 4 の確かめ（`PYTHONPATH=… python3` は `rclpy` をなくす）と、負のずらしは `--shift=` の形で書くこと。

### テスト

- `lamp_1` を `false` にした試しで `T tests/test_household.py tests/test_layout.py tests/test_detector_check.py tests/test_objects_check.py`：
  1 failed, 216 passed（26.1 秒）。落ちたのは上の `test_detector_check_command_line_with_the_oracle` だけで、`test_household.py` は通った。
  配置とその固定の 1 行を元に戻した。
- 全部（`T tests nl_control/tests`、元に戻した木で、16:00:54〜16:05:52）：**2553 passed, 18 skipped, 29 deselected**（296.95 秒）。
  `feat/open-objects` の `ced4e87` と同じ数。前後とも動いていたのはこの回の app の container だけ（電池 79% → 78%、load 4.02 → 3.02）。
- 部屋・単語帳・上書きは変えていないので、§12.4 の数（`tests/test_household.py`）はそのまま通る。

### 変えたもの

- この記録だけ。配置・単語帳・`detector/yoloe.py` の `BY_LABEL`・テストは変えていない（上の「決めたこと」と「controller に返すこと」）。
  PR #4 のファイル（D12）には触れていない。app のイメージにも本来の Linux の準備にも、新しい依存は入っていない。

## H3 screening：部屋の選び直しの下見（R-H3b、2026-11-12）

- 枝：`objects2/h3-screen`（`feat/open-objects` の `7fecae7` から）。worktree `~/work/physicalAI/.go2-worktrees/oo-h3s`。ROS の domain は 64、`GZ_PARTITION` は `oo-h3s`。
- 1 回で済んだ（前の試みはなかった）。コード・テスト・配置・固定・イメージは変えていない。変えたのはこの記録だけ。
- 提案（モデル、配置、手順、表のすべて）は controller の作業場所の `room-proposal.md` と `room-proposal-household.json`
  （`~/work/physicalAI/go2/.superpowers/sdd/2026-11-11-go2-open-objects/`）。zip・モデル・絵・使い捨ての道具は worktree の `out/h3s/` に
  置いた（git に入れていない。CC BY / CC0 のモデルとその絵）。

### 結論

**新しい種類 3 つが本当の手順（Part B）で通った**：sofa_1（Bf）、suitcase_1（Bf）、traffic_cone_1（R5）。目標の 4 つには届かなかった。
受け入れの PLANT はどれも同じで、どの目標の `F near_<id>`・`F around_<id>` も、試験の姿勢と 4 つのずれのすべてから計画がある。
YOLOE-11s で 3 つ通ったので、11m は流していない。

| 目標 | モデル | 試験の姿勢 | Part B（姿勢の見え方＋ずれ 4 つ） |
|---|---|---|---|
| sofa_1 | Sofa（3）。0.2 m 後ろの (−4.6, 0) へ | Bf | 5/5（0.775〜0.840） |
| suitcase_1 | Suitcase1（2）。(−1.62, −1.14)、yaw 90 のまま | Bf | 姿勢＋3/4（0.488〜0.665。s4 は trash_can） |
| traffic_cone_1 | Construction Cone（3、CC0、新しい予備の種類） | R5 | 5/5（0.634〜0.913） |

- 部屋から出す：TrashBin（trash_can_1）と LampAndStand（lamp_1）。どこに置いても自分の名前で確かにならない（R-H3c）。ゴミ箱とランプの種類は目標の一覧から外れる。
- 試験の姿勢：**Bf** = `F standing`、`F(turned_left & X F turned_left)`、`F moved_forward`。**R5** = `F standing`、`F turned_right`、
  `F moved_forward` を 5 回（1 つの式）。B・L・R は検出の試験の姿勢でなくなる。`PREVIEW_TARGETS` は `{"sofa_1": "Bf", "suitcase_1": "Bf", "traffic_cone_1": "R5"}`。

### 候補と測った足跡（Fuel の今の版、`model_extent.py`、当たり判定）

| Fuel の名前（版） | フォルダ | ライセンス | 面 | 足跡（モデルの軸）× 高さ | Part A |
|---|---|---|---|---|---|
| TrashBin（2） | TrashBin | CC BY 4.0 | 304 | 0.266 × 0.375 × 0.446 | 0/30（前から光）、2/30（後ろから光） |
| Suitcase1（2） | Suitcase1 | CC0 1.0 | 1 732 | 0.291 × 0.468 × 0.752 | 1.5 m の −35・−20・+35° でだけ |
| LampAndStand（3） | LampAndStand | CC BY 4.0 | 164 | 0.624 × 0.624 × 1.020 | 台が desk。lamp は 3.5・4.5 m で 1/5 |
| FIRST 2015 trash can（3） | first_2015_trash_can | CC0 1.0 | 5 764 | 0.565 × 0.638 × 0.737 | 2〜3/5（0.38〜0.57） |
| LitterBin（2） | LitterBin | CC BY 4.0 | 560 | 0.453 × 0.453 × 1.013 | 1.5 m で 3/5 |
| Suitcase2（2） | Suitcase2 | CC0 1.0 | 1 732 | 0.209 × 0.336 × 0.539 | 1.5 m の −35・−20・+35° でだけ |
| Construction Cone（3） | construction_cone | CC0 1.0 | 166 | 0.503 × 0.503 × 1.089 | 1.5 m で 4/5（0.60〜0.98）、2.0 m で 3/5 |
| Backpack（3） | Backpack | CC0 1.0 | 2 062 | 0.442 × 0.264 × 0.522 | 0（suitcase・bag） |

- zip の sha256・大きさ・作者は提案に全部書いた。今の 3 つは固定の sha256 と同じ。どの zip も `fetch_fuel_models` の決まり（絶対パスと `..` を断る）で、
  空の新しいフォルダに開いた。zip から出た物は何も実行していない。
- JanSport Backpack Red（6）は使わない：model.sdf が別の Fuel のモデル（Artifact Proximity Detector）を URL で `<include>` していて、Gazebo が
  起動のときに取りに行く。Backpack は model.sdf の先頭の空行と DAE の重なった id で `model_extent.py` が断るので、空行と id だけを直した写しで測った
  （Gazebo には元のまま）。
- Construction Cone の zip は、Fuel の名前の URL（`Construction%20Cone`）でないと取れない（フォルダの名前の URL は 404）。固定するには
  `fetch_fuel_models.py` にフォルダと別の Fuel の名前が要る（設計書 §12.6 の見込みどおり）。

### わかったこと：光の向き

世界の太陽は +x から当たる（方向 (−0.5, 0.1, −0.9)）。始めの向き（+x）のカメラは物の影の面を見て、B・Bf（−x 向き）は光の当たった面を見る。
R（−y 向き）は横から当たり、影が絵の右に伸びる。だから Part A を 2 回撮った：A1 は始めの向き（後ろから光）、A2 は 2 回左に回った後
（−168.0°、前から光、B・Bf と同じ）。どちらも 1 つの物だけが視野にある（ほかの候補は 30 m 先、描く範囲の 10 m の外）。
小さい物は 1.5・2.0・2.5 m、ランプは 3.5・4.5 m も、方位 0・±20・±35°、モデルの向き 2 つ。A3 は Bf の置き場所（1.5 m、+30・+35°）を
Part B と同じ 4 つのずれで撮った。

### 置ける場所が 2 つしかないわけ

- §12.1 の広さ（受け入れの長方形のマスの中心から 0.7 m。5 つの始めで作る）：後ろは x ≤ −1.5、右は y ≤ −4.5（設計書の y ≤ −3.7 では
  始め (0.1, 0.1, 5°) の `F near_chair_1` の PLANT が変わった）。
- ソファの周回：ソファの中心から約 3.2 m の内に小さい物があると、どれかの Bf の向きとずれで `F around_sofa_1` の計画がなくなる。
- ±35° と測った向き：2 回の左は −166.9〜−172.4°（この下見の 7 回）。後ろの +y の側を ±35° で見る姿勢は、どれも table_1 が視野に入る（R-H3c）。
- 2 つの小さい物の周回には 2.1 m より広い間が要る（右の帯に 2.06 m で 2 つ置くと両方の周回が落ちた）。
- だから置き場所は、Bf から 1.5 m・+30〜35° と、R5 から見る右の帯の 2 つ。Bf の場所は Suitcase1（今のモデル、yaw 90。A3 で +0.18・+0.34、
  Part B で +0.138）が、R5 の場所はコーン（FIRST 2015 trash can は R5 で 5/5 とも何も出ない）が取る。

### Part B（本当の手順、毎回 Gazebo を起動し直し）

| 回 | 配置 | 結果 |
|---|---|---|
| b1・r1 | Suitcase1 yaw 0、コーン (−0.3, −4.55)、ソファ (−4.4, 0) | 3 つ通るが、ソファの周回が 2 つの Bf の向きで 1 つのずれを落とす |
| a1b・a1r | FIRST 2015 trash can を Bf の場所と R5 の場所に | Bf は通る（0.360〜0.882、余裕 +0.010）、R5 は 0/5 |
| v5b・v5r | Suitcase1 yaw 0、ソファ (−4.4, 0.2) | スーツケースが television（0.563）で落ちる |
| x3b | Suitcase1 yaw 90、ソファ (−4.5, 0.4) | 通る（余裕 +0.037。B でソファが bench） |
| **x4b・v6r（提案）** | Suitcase1 yaw 90 (−1.62, −1.14)、ソファ (−4.6, 0)、コーン (−0.3, −4.45) | **3 つ通る**（上の表）。始めの位置の見え方も通る（chair だけ） |

- Bf の位置は x −0.506〜−0.560、y −0.054〜−0.141、向き −166.6〜−172.4°（6 回）。R5 は 5 歩で 2.66〜2.81 m、向き −95.1〜−97.6°（4 回）。
- 薄いところ：コーンの正解の枠との IoU は 0.51〜0.57（影が箱に入る）。スーツケースは Bf_s4 で trash_can と呼ばれる。スーツケースの方位は
  回り方で +29〜+36° に動く。

### 確かめ（app のイメージ、PLANT だけ、`tests/test_household.py` の関数を使う使い捨てのスクリプト）

- 受け入れの PLANT：4 つの式 × 5 つの始め × 2 つの大きさで、新しい物があってもなくても同じ（テスト関数の 7 個の assert だけ外して本体を流した。
  新しい部屋は 6 個）。
- 新しい物は始めの位置のカメラに入らない。どの検出の試験の姿勢（Bf 6 つ、R5 5 つ、それぞれ 4 つのずれ）にも、検出できない物（table_1・box_1・
  chair_1）は入らない。ソファを隠す目標はない。
- 計画：Bf のソファとスーツケース、R5 のコーン、L のテーブルの `F near`・`F around` が、どの測った姿勢と名目の姿勢と 4 つのずれからも計画がある（44 行）。
  名目の数（Bf (−0.5, 0, 180°)、R5 (0, −2.75, −90°)）：ソファ 手前 154 / 5 手 / 18 キー / 3 072、周回 208 / 49 / 14 208。スーツケース 手前 49 / 2 / 8 / 1 032、
  周回 100 / 29 / 8 016。コーン 手前 72 / 1 / 6 / 1 584、周回 100 / 29 / 8 256。テーブル 手前 165 / 10 / 20 / 3 408、周回 204 / 45 / 13 488。

### D11 の門と電源

どの Gazebo の回・検出の回の前にも `go2-gazebo-1` はなく、待たなかった。こちらの重い段は 1 つずつ。電池で 73 % → 43 %、`lowpowermode` 0。
`pmset -g log` に眠りはない（16:00〜18:00）。Gazebo の回の間の container は `go2oo-gazebo` だけ（189〜285 %、0.72〜1.2 GiB）。
時刻ごとの記録は提案に書いた。終わりに、こちらの container とネットワークはすべて止めて消した。

## H1b：部屋の直しの後の Fuel のイメージ（patch-0005-fuel2）を作り、4 つのモデルを確かめた（2026-11-13）

- 枝：`objects2/h1b-fuel`（`feat/open-objects` の `49a4158`、部屋の直しの merge から）。worktree `~/work/physicalAI/.go2-worktrees/oo-h1b`。ROS の domain は 61。
- 場所：この Mac の Docker Desktop。
- 前の試みはなかった（worktree・枝・`go2-voice-ltl-gazebo:patch-0005-fuel2` のどれもなかった）。1 回で済んだ（15:56〜16:06）。
- 部屋は R-H3h・R-H3i の後のもの。モデルは Sofa（3）、Table（4）、Suitcase1（2）、Construction Cone（3）。TrashBin と LampAndStand は出た。
  計画 H1 の Suitcase1 だけの手順は 1 回目の H1 で済んでいるので、同じ確かめを新しい 4 つの一覧に当てた。

### 結論

**H1b の線はすべて満たした。** `go2-voice-ltl-gazebo:patch-0005-fuel2` は `patch-0005` の 22 の層をそのままに、2 つの層と `ENV` を足しただけで、
4 つのモデルのフォルダがあり、どの zip も固定の sha256 と大きさに同じ。作者は `model.config` と固定が同じ。`model_extent.py` の足跡・高さ・
置く高さは、配置（`gazebo/layouts/household.json`）といちばん大きくて 2 mm しか違わない（設計書の値のまま置く Table のずれ。R-H1c）。
だから固定は 1 つも動かしていない。受け入れの PLANT も目標の試験の数も変わらない。全部のテストは 0 failed。変えたのはこの記録だけ。

| 線（計画 H1 を新しい一覧に） | 結果 |
|---|---|
| zip を 1 回だけ取り、sha256 と大きさを記録（host では開かない） | 取ったのは build の 4 つの zip だけ。どれも固定と同じ。Suitcase1 と Construction Cone は H3 の下見の zip も数え直して同じ（取り直していない） |
| 固定のまま `T tests/test_layout.py tests/test_fuel_image.py` | 94 passed（build の前） |
| build が通り、フォルダがそろい、`GZ_SIM_RESOURCE_PATH` の最後が `/opt/go2_models` | 通った。4 つのフォルダ |
| イメージの中で、sha256・ライセンス・作者が固定と同じ | 同じ（ライセンスは下の注） |
| イメージの中で 4 つを測り、足跡・高さ・z が配置と許す違いの中 | 中。測った値を写した 2 つは 0.11 mm 以内、設計書の値の 2 つは 2 mm 以内 |
| `T tests/test_household.py` と全部のテスト | 122 passed、2561 passed・18 skipped・29 deselected |

**許す違い。** 計画 H1 は、測ったモデル（ここでは Suitcase1 と Construction Cone）の足跡を `model_extent.py` から配置に写し、ほかのモデルは
設計書 §12.2 の値と比べて違いを報告し、値はそのままにする（R-H1c）。だから、写した 2 つは mm への丸め（0.5 mm）の中、設計書の値の 2 つは
H1 で報告した違い（1〜2 mm）から増えないことを線にした。どちらも §13.3 の位置と大きさの線（0.15・0.35・0.4 m）よりずっと小さい。

### D11 の門と、ほかの container・電源

どの回の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった（待たなかった）。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | `lowpowermode` |
|---|---|---|---|---|
| 15:56:52 | 始め（門） | なし | AC、100%、charged | 0 |
| 15:57:07 | build の直前 | なし | AC、100%、charged | 0 |
| 15:59:17 | build とイメージの中の確かめの後 | なし | AC、100%、charged | 0 |
| 16:06:14 | 全部のテストの後 | なし | AC、100%、charged | 0 |

- `pmset -g log` に、14:43 の後の眠りはない。
- 動かしたのは、名前を付けた使い捨ての container（`docker run --rm --name oo-h1b-…`、`--network none`）と、本体の compose project の `app` の
  `run --rm`（`T`）だけ。終わった後に残った container はない。compose project `go2oo` は使っていない（H1b は Gazebo を動かさない）。

### zip

- 新しく取ったのは、build の `RUN` が取った 4 つの zip だけ（R-H3b が認めた、固定した Fuel の zip）。host では開いていない。
- Suitcase1 と Construction Cone は、H3 の下見が取った zip（`oo-h3s` の git に入れていない `out/h3s/models/`）を、取り直さずに数え直した：

  ```bash
  shasum -a 256 <h3s>/models/Suitcase1/Suitcase1.zip <h3s>/models/ConstructionCone/ConstructionCone.zip; wc -c < …
  ```
  Suitcase1 `8e0c23b7c34cd19f443253eecf7185ca1a949f955aba278f01029775972f8fe7`（1 802 850 B）、Construction Cone
  `a6962fb5f787ba0ee21d0a9f1e96d95dde5160146852abcc45df35b33a46a34e`（622 427 B）。どちらも `docker/fuel_models.json` と同じ。
- Construction Cone は、固定の `name`（R-H3h (3)）から作る Fuel の名前の URL（`Construction%20Cone/3/Construction%20Cone.zip`）で取れ、
  フォルダ `construction_cone/` に開いた。

### build

```bash
docker build --build-arg BASE=go2-voice-ltl-gazebo:patch-0005 -t go2-voice-ltl-gazebo:patch-0005-fuel2 \
    -f docker/Dockerfile.gazebo-fuel docker
```

- 元：`go2-voice-ltl-gazebo:patch-0005`（`sha256:39fd601dc2a3…`、2026-11-11 10:08）。作り直していない（`FROM` は CACHED）。
- できたイメージ：`go2-voice-ltl-gazebo:patch-0005-fuel2`（一覧の ID `a67719c022bc`、config `sha256:64551a7a3c5a…`、2026-11-13 15:57:17、7.28 GB）。
  全部で 11 秒。層は 24 で、はじめの 22 は元と同じ。足したのは `COPY`（32.8 kB）と `RUN`（5.05 MB、9.7 秒）の 2 つと `ENV` だけ。
- `patch-0005-fuel1`（`5ee232801b4e`、2026-11-12 10:30:54）は残してある（作り直していない）。
- `RUN` の出力（4 つとも固定の sha256 と同じ）：

  ```
  fetch_fuel_models: Sofa 3: 1311028 B, sha256 6d19d1f1…0db1 ok -> /opt/go2_models/Sofa
  fetch_fuel_models: Table 4: 899222 B, sha256 3a720ef2…163d ok -> /opt/go2_models/Table
  fetch_fuel_models: Suitcase1 2: 1802850 B, sha256 8e0c23b7…8fe7 ok -> /opt/go2_models/Suitcase1
  fetch_fuel_models: construction_cone 3: 622427 B, sha256 a6962fb5…a34e ok -> /opt/go2_models/construction_cone
  ```
- build の注意は H1 と同じ 1 つ（`UndefinedVar`：元のイメージにこの値がない。`${…:+…:}` の形なので値は `/opt/go2_models` だけになる）。

### イメージの中の確かめ

使い捨ての container で、作業用の小さいスクリプト（読むだけ。モデルの中の物は何も実行しない）を `python3 -I` で流した。

- `/opt/go2_models` には `Sofa`、`Suitcase1`、`Table`、`construction_cone` の 4 つ（隠れたファイルも zip もない）。`/opt/go2_fuel` の
  `fuel_models.json` と `fetch_fuel_models.py` は、枝のファイルと sha256 が同じ。
- どれにも `model.config` と `model.sdf` があり、どの `.sdf` にも `<plugin` は 0 個。Construction Cone には `model-1_3.sdf`・`model-1_4.sdf` もあるが、
  Gazebo は `model.config` のいちばん新しい版（1.5）の `model.sdf` を読み、`model_extent.py` もそれを測る（3 つとも当たり判定は同じ DAE と `<scale>`）。
- 中身が同じこと：Suitcase1 と `construction_cone` のフォルダのどのファイルも、zip の中の同じ名前のファイルと sha256 が同じ（ファイルの数も同じ）。
  Sofa・Table・Suitcase1 のフォルダは、`patch-0005-fuel1` の同じフォルダとファイルごとに同じ。
- `GZ_SIM_RESOURCE_PATH`：値は表示せず、`:` で区切った数と、最後の項目が `/opt/go2_models` か（はい・いいえ）だけを確かめた。
  `--entrypoint bash` では 1 項目で、最後が `/opt/go2_models`。いつもの入り口（`gazebo_entry.sh` が workspace を読み込んだ後）では 2 項目で、
  `/opt/go2_models` はそのうちの 1 つ。
- 橋渡しの止まる確かめ（`gazebo/bridge_core.bridge_settings`、`GO2_LAYOUT=household`、`OBJECTS_SOURCE=logical`、いつもの入り口の
  `GZ_SIM_RESOURCE_PATH`）：`patch-0005-fuel2` では通り、6 つの物を置き、足りないフォルダはない。`patch-0005-fuel1` では
  「`construction_cone` がない」の 1 行で止まる（計画の Wave 3「部屋の直しの後の部屋」のとおり）。

### 作者・ライセンス・sha256 と固定

| フォルダ | Fuel の名前（版） | zip の sha256（build で確かめた）・大きさ | 作者（`model.config`） | 固定の作者 | 固定のライセンス |
|---|---|---|---|---|---|
| Sofa | Sofa（3） | `6d19d1f1…0db1`、1 311 028 B | Wan Yi Seow | 同じ | CC BY 4.0 |
| Table | Table（4） | `3a720ef2…163d`、899 222 B | Nate Koenig | 同じ | CC0 1.0 |
| Suitcase1 | Suitcase1（2） | `8e0c23b7…8fe7`、1 802 850 B | Wan Yi Seow | 同じ | CC0 1.0 |
| construction_cone | Construction Cone（3） | `a6962fb5…a34e`、622 427 B | Cole Biesemeyer、Nate Koenig | 同じ（`, ` でつないだ 2 人） | CC0 1.0 |

- **ライセンスの注：** どの `model.config` にもライセンスの項目はなく、モデルのフォルダのどのファイルにも「licen」の字はない（設計書 §12.2 のとおり）。
  だからイメージの中のライセンスは固定（`/opt/go2_fuel/fuel_models.json`、枝と同じ）だけで、モデルのファイルと比べることはできない。
  取ってよいのは固定の zip だけなので、Fuel の情報は取り直していない。固定の値は、設計書 §12.2・§12.9 と、H3 の下見が Fuel の情報
  （`license_name`、2026-11-12）から読んだ値と同じ。
- Construction Cone の `model.config` の名前は `Construction Cone`、説明は「オレンジの工事用のコーン」。

### 4 つのモデルを測った（イメージの中で、`gazebo/tools/model_extent.py`）

```bash
docker run --rm -v ~/work/physicalAI/.go2-worktrees/oo-h1b:/wt:ro -w /wt --entrypoint bash go2-voice-ltl-gazebo:patch-0005-fuel2 -c \
  'for m in /opt/go2_models/*; do echo "$m"; python3 -I /wt/gazebo/tools/model_extent.py "$m"; done; cat /opt/go2_models/construction_cone/model.config'
```

| モデル（id） | 測った足跡（モデルの x × y）× 高さ、ずれ、z_min（collision） | 配置 | 違い（いちばん大きい） | 配置の値の出どころ |
|---|---|---|---|---|
| Sofa（sofa_1） | 1.04353 × 1.8 × 0.942733、(0.021071, 0)、−0.00012 | 1.044 × 1.8 × 0.943、(0.02, 0)、yaw 0 | ずれ x 1.07 mm | 設計書 §12.2（R-H1c） |
| Suitcase1（suitcase_1） | 0.290914 × 0.467992 × 0.751915、(0, 0)、0.0 | 0.291 × 0.468 × 0.752、(0, 0)、yaw 90 | 0.09 mm | H1 の `model_extent.py` |
| Table（table_1） | 1.5 × 0.8 × 1.015、(0, 0)、0.0 | 1.5 × 0.8 × 1.015、(0.002, 0)、yaw 0 | ずれ x 2.0 mm | 設計書 §12.2（R-H1c） |
| Construction Cone（traffic_cone_1） | 0.50292 × 0.50292 × 1.088889、(0, 0)、0.0 | 0.503 × 0.503 × 1.089、(0, 0)、yaw 0 | 0.11 mm | H3 の下見の `model_extent.py` |

- Sofa・Suitcase1・Table の出力は、H1（`patch-0005-fuel1`）の出力と 1 文字も違わない。Construction Cone は H3 の下見の値と同じ。
- 置く高さ：z_min は 0 か −0.12 mm（ソファの床の誤差）なので、配置に `z` はなく、どれも既定の 0 のまま。
- だから R-H1a のような固定の直しは要らない。配置も `docker/fuel_models.json` も `tests/test_household.py` の数も変えていない。

### テスト

- `T tests/test_layout.py tests/test_fuel_image.py`：94 passed（build の前。固定はそのまま）。
- `T tests/test_layout.py tests/test_fuel_image.py tests/test_household.py`：122 passed（39.9 秒）。受け入れの PLANT が変わらないことと、
  Bf・R5・L からの目標の数の固定（R-H3g）はこの中にある。
- 全部（`T tests nl_control/tests`）：2561 passed, 18 skipped, 29 deselected（317.1 秒）。`feat/open-objects` の `49a4158` の merge と同じ数。
- 変えたファイルはこの記録だけ。PR #4 のファイル（D12）には触れていない。

### 2 回目：同じ依頼がもう一度来た（2026-11-13 16:16〜16:20）

- 依頼は「`objects2/room-fix` の今の先から `objects2/h1b-fuel` を切る。前の試みのものがあれば使う」。worktree `oo-h1b` と枝
  `objects2/h1b-fuel` は 1 回目のもの（`d5c957a`、`feat/open-objects` に `051fdb5` で merge 済み）があったので、そのまま使った。
- **作った元は `objects2/room-fix` の `68122c3`**（16:16 の時の枝の先）。部屋の直しの見直しは同時に動いているので、その後に枝に足される
  commit は入っていない。1 回目の元の `49a4158` は `e8c40aa` に `68122c3` を merge したもので、`git diff 68122c3 49a4158` は空（木が同じ）。
  だから 1 回目も 2 回目も、`68122c3` のファイルから作ったのと同じ。
- 結論：**線はすべて、もう一度満たした。** 固定は 1 つも動かない。変えたのはこの記録だけ。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | `lowpowermode` |
|---|---|---|---|---|
| 16:16:30 | 始め（門） | なし | AC、100%、charged | 0 |
| 16:18:17 | build とイメージの中の確かめの後 | `go2-app-run-d54c9d2d2af7` 100.10%、124.6 MiB（ほかの仕事の `T`。私のものではない） | AC、100%、charged | 0 |
| 16:19:18 | テストの後 | `go2-app-run-adf75cbea411` 15.73%、53.5 MiB（同じく、私のものではない） | AC、100%、charged | 0 |

- `go2-gazebo-1` はどの時も動いていなかった（待たなかった）。`pmset -g log` に、14:43 の後の眠りはない。
- 動かしたのは、名前を付けた使い捨ての container（`oo-h1b2-check`、`oo-h1b2-extent`。`--rm`、`--network none`）と `T` だけ。
  終わった後に、私の container は残っていない。compose project `go2oo` は使っていない。

**build。** 同じ命令（`docker build --build-arg BASE=go2-voice-ltl-gazebo:patch-0005 -t go2-voice-ltl-gazebo:patch-0005-fuel2
-f docker/Dockerfile.gazebo-fuel docker`）で 0.6 秒。`COPY` も `RUN` も CACHED だった。だから build に渡した
`fuel_models.json` と `fetch_fuel_models.py` は 1 回目と 1 バイトも違わず、build は zip を取っていない。

- config は `sha256:64551a7a3c5a…` のまま、作った時刻も 15:57:17 のまま。層は 24 で、はじめの 22 は `patch-0005` と同じ。
- 一覧の ID だけが `a67719c022bc` から `14c50167b0b8` に変わった。新しい build が、中身の同じイメージに新しい attestation の manifest を
  書いたため。中身（config と層）は 1 回目と同じ。
- `patch-0005`（`39fd601dc2a3`）と `patch-0005-fuel1`（`5ee232801b4e`）には触れていない。
- `docker history --no-trunc` で層を見たとき、Dockerfile の `ENV` の行（値は Dockerfile に書いた `/opt/go2_models`）が出た。
  ほかの環境の値は出していない。

**zip。** 固定した 4 つの zip だけを、作業用の新しい空のフォルダに 1 回ずつ取った（R-H3b が認めたもの。host では開いていない）。
URL は `https://fuel.gazebosim.org/1.0/OpenRobotics/models/<Fuel の名前>/<版>/<Fuel の名前>.zip`（Construction Cone は `Construction%20Cone`）。

| フォルダ | 大きさ | sha256 | 固定と |
|---|---|---|---|
| Sofa | 1 311 028 B | `6d19d1f109427626d88667de047ca80a2c175b3f8f12dc2d0e75ddb3b88a0db1` | 同じ |
| Table | 899 222 B | `3a720ef2a8e2f416e0ed017b2e13936f08f6f7c97829c9ea4b0d15380e36163d` | 同じ |
| Suitcase1 | 1 802 850 B | `8e0c23b7c34cd19f443253eecf7185ca1a949f955aba278f01029775972f8fe7` | 同じ |
| construction_cone | 622 427 B | `a6962fb5f787ba0ee21d0a9f1e96d95dde5160146852abcc45df35b33a46a34e` | 同じ |

**イメージの中の確かめ**（使い捨ての container、zip は読むだけで mount、作業用の小さいスクリプトを `python3 -I`。モデルの中の物は何も実行しない）：

- 4 つの zip の sha256 と大きさを、イメージの中の `fetch_fuel_models.verify` でも固定と比べた：同じ。
- **どのモデルも、フォルダのどのファイルも、zip の中の同じ名前のファイルと sha256 が同じで、余分なファイルはない**（Sofa 10、Table 11、
  Suitcase1 15、`construction_cone` 11 のファイル。zip の上のフォルダは `fetch_fuel_models` と同じ決まりで外した）。1 回目は Sofa と Table を
  `patch-0005-fuel1` のフォルダと比べただけだったので、2 回目は 4 つとも zip と比べた。
- `/opt/go2_models` には 4 つのフォルダだけ（隠れたファイルも zip もない）。`/opt/go2_fuel` の 2 つのファイルは枝のファイルと sha256 が同じ。
- 作者：`model.config` の `<author>` の名前（2 人なら `, ` でつなぐ）が、4 つとも固定と同じ。どの `.sdf` にも `<plugin` は 0 個。
- ライセンス：1 回目と同じく、モデルのファイルにライセンスの字はない（「licen」は 0 個）。固定のライセンスは、設計書 §12.2（Sofa CC BY 4.0、
  Table CC0 1.0、Suitcase1 CC0 1.0）と §12.9・H3 の下見（Construction Cone CC0 1.0）と同じ。
- `GZ_SIM_RESOURCE_PATH`：値は表示せず、`--entrypoint python3` で 1 項目、最後が `/opt/go2_models`（はい）だけを確かめた。

**測った（`model_extent.py`、上の「4 つのモデルを測った」と同じ命令）。** 出力は 1 回目と 1 文字も違わない。配置との違いのいちばん大きいところ：
suitcase_1 0.09 mm、traffic_cone_1 0.11 mm、sofa_1 ずれ x 1.07 mm、table_1 ずれ x 2.00 mm。どれも許す違いの中なので、R-H1a のような
固定の直しは要らない。配置にない Fuel のモデルはイメージにない。

**テスト。** `T tests/test_layout.py tests/test_fuel_image.py tests/test_household.py`：122 passed（44.7 秒。受け入れの PLANT と、Bf・R5・L
からの目標の数の固定はこの中）。全部のテストは流していない（依頼が求めていない。木は 1 回目と同じで、1 回目は 0 failed）。
