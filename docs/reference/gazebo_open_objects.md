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
