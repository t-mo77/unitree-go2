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

### 3 回目：部屋の直しの見直しの後に作り直した（2026-11-13 17:47〜17:52）

- わけ：見直しの直し（`bfdc636`、`feat/open-objects` の `206fe97`）が `docker/fetch_fuel_models.py` を変えた（`zip_url` が `""`・`.`・`..` を
  断る。5 行足し、1 行直し）。`docker/fuel_models.json` と `docker/Dockerfile.gazebo-fuel` は同じ。イメージはこのファイルを `COPY` するので作り直した。
- `feat/open-objects` の `206fe97` を `objects2/h1b-fuel` に `--no-ff` で merge した（`0a39c77`）。違いはこの記録だけ。
- 結論：**線はすべて、もう一度満たした。** 固定は 1 つも動かない。変えたのはこの記録だけ。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | `lowpowermode` |
|---|---|---|---|---|
| 17:47:22 | 始め（門） | なし | — | — |
| 17:47:47 | build の直前 | なし | 電池、90%、discharging | 0 |
| 17:51:52 | テストの後 | なし | 電池、89%、discharging | 0 |

- `go2-gazebo-1` はどの時も動いていなかった（待たなかった）。眠りは 16:41:33〜16:56:54（蓋を閉じた。この回の前）で、その後はない。
- 動かしたのは、名前を付けた使い捨ての container（`oo-h1b3-check`、`oo-h1b3-extent`、`oo-h1b3-bridge`、`oo-h1b3-bridge1`。`--rm`、
  `--network none`）と `T` だけ。終わった後に残った container はない。compose project `go2oo` は使っていない。

**build**（同じ命令、17:48:25〜17:48:36）：`FROM` は CACHED、`COPY` と `RUN` は流れ直し、`RUN` が 4 つの zip を取り直した。4 つとも固定の
sha256 と大きさに同じ（上の表と同じ値）。できたイメージは一覧の ID `c40a28c22024`（linux/arm64 の manifest `0e747c2aaaf1`）、17:48:36、
7.28 GB。層は 24 で、はじめの 22 は `patch-0005` と同じ。足したのは `COPY`（32.8 kB）と `RUN`（5.05 MB）と `ENV` だけ。注意は同じ
`UndefinedVar` の 1 つ。`patch-0005` と `patch-0005-fuel1` には触れていない。

**確かめ**（2 回目と同じ。zip は作業用の新しい空のフォルダに 1 回ずつ取り直し、host では開いていない。大きさと sha256 は固定と同じ）：

- `/opt/go2_models` は 4 つのフォルダだけ（隠れたファイルも zip もない）。`/opt/go2_fuel` の 2 つのファイルは枝のものと sha256 が同じ
  （新しい `fetch_fuel_models.py` が入った）。
- どのフォルダのどのファイルも zip の中の同じ名前のファイルと sha256 が同じで、余分も不足もない（Sofa 10、Table 11、Suitcase1 15、
  `construction_cone` 11）。作者は 4 つとも固定と同じ。`<plugin` 0 個、「licen」0 個。
- `GZ_SIM_RESOURCE_PATH`：値は表示せず、`--entrypoint python3` で 1 項目、最後が `/opt/go2_models`。いつもの入り口では 2 項目で、
  `/opt/go2_models` はその 1 つ。
- 橋渡しの止まる確かめ（`household`、`logical`）：`patch-0005-fuel2` では 6 つの物を置き、足りないフォルダはない。`patch-0005-fuel1` では
  「`construction_cone` がない」の 1 行で止まる。
- `model_extent.py` の出力は 1 回目・2 回目と 1 文字も違わない。

**テスト。** build の前に `T tests/test_layout.py tests/test_fuel_image.py`：95 passed。後に `T tests/test_layout.py tests/test_fuel_image.py
tests/test_household.py`：125 passed（39.1 秒）。2 回目より 1 つと 3 つ多いのは、見直しが足したテストの分。全部のテストは流していない
（依頼が求めていない。見直しの直しの `bfdc636` で 2564 passed・18 skipped・29 deselected・0 failed）。

## H3b：部屋の直しの後の検出の下見（`patch-0005-fuel2`、2026-11-13）

- 枝：`objects2/h3b-preview`（`feat/open-objects` の `25d8bba`、H1b の 3 回目の merge から）。worktree `~/work/physicalAI/.go2-worktrees/oo-h3b`。
  ROS の domain は 63。
- 場所：この Mac の Docker Desktop。Gazebo は compose project `go2oo`（画面なし、`go2oo-gazebo`、`OBJECTS_SOURCE=detector`、検出の
  service なし）、イメージは `go2-voice-ltl-gazebo:patch-0005-fuel2`（H1b の 3 回目、一覧の ID `c40a28c22024`、17:48:36）。検出は
  `go2-voice-ltl-detector:yoloe-11s`（H2、`016348e47306`）。どちらも作り直していない。
- 前の試みはなかった（worktree・枝・記録・`out/` のどれもなかった）。1 回で済んだ（18:01〜18:10）。
- 部屋は R-H3h・R-H3i の後のもの（`gazebo/layouts/household.json`）：sofa_1 (−4.6, 0)、suitcase_1 (−1.62, −1.14) yaw 90、
  traffic_cone_1 (−0.3, −4.45)、table_1（論理だけ）、box_1・chair_1。検出の目標は `detector_check.PREVIEW_TARGETS` の
  sofa_1・suitcase_1（Bf）と traffic_cone_1（R5）。
- 絵（`out/h3-preview/`、`out/h3b-L/`）は CC BY / CC0 のモデルを描いたものなので git に入れていない（設計書 §11.11）。worktree の中に残してある。

### 結論

**H3b の線を満たした。** `preview` を通った新しい種類は **3**（sofa_1、suitcase_1、traffic_cone_1。線は 3 以上、設計書 §12.8 の 5、§13.4）。
始めの位置の見え方も通った（box と chair のほかに確かになる名前なし）。どの検出の試験の見え方にも、検出できないモデル（table_1・
box_1・chair_1）の体は写っていない（R-H3c）。配置・単語帳・`BY_LABEL` は変えていない。変えたのはこの記録だけ。

| 線（計画 H3、部屋の直しの後） | 結果 |
|---|---|
| `record_frames` が `patch-0005-fuel2` で起動する（Task 2.12 の Step 4、R-H3f の形） | exit 0、`gz` は `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` |
| 見え方を撮る（`spawn_lying`、`spawn`、Bf・R5 × (そのまま＋ずらし 4)、測った向きの本当の手順） | 12 枚とも exit 0（止まりの門・位置の組・正解の確かめを通った）。撮り直しはなし |
| 最初の絵で、どの Fuel のモデルも自分の見え方に写る（M15） | 写った。Sofa と Suitcase1（Bf）、Construction Cone（R5）。Table は足した L の 1 枚で写った |
| `detector_check.py … --detector yoloe` | 12 枚、`errors` なし。`preview` の `detectable` は `[sofa_1, suitcase_1, traffic_cone_1]`、`enough` は true。exit 1（`Bf_s2` の絵ごとの線、下） |
| 通った目標に `detectable: true` | 3 つとも、もう `true`（部屋の直しのまま）。変えていない |
| `BY_LABEL`・外す検出の言葉 | なし（下の「決めたこと」） |
| R-H3c：検出できないモデルは、どの試験の見え方にも入らない | 入らない（投影・論理カメラ・検出・絵の 4 つで確かめた。下の「R-H3c の確かめ」） |
| 眠り | なし（`pmset -g log` の最後の眠りは 16:55:28〜16:56:54、この回の前） |
| `T tests/test_household.py`、全部のテスト | 下の「テスト」 |

### D11 の門と、ほかの container・電源

どの Gazebo の回と検出の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった（待たなかった）。
こちらのほかの重い段もなかった。`load` は host の `sysctl -n vm.loadavg`（1・5・15 分）。`lowpowermode` はどの時も 0。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | 電源（`pmset -g batt`） | load |
|---|---|---|---|---|
| 18:01:14 | 始め（門） | なし | 電池、87%、discharging | 2.04 2.60 3.43 |
| 18:01:44 | 回 1（伏せ・立ち・Bf）の前 | なし | 電池、86% | 2.06 2.56 3.39 |
| 18:03:07 | 回 1 の終わり（Gazebo あり） | `go2oo-gazebo` 217.10 %、985.2 MiB | 電池、85% | 3.66 2.96 3.46 |
| 18:03:19 | 回 1 を止めた後・回 2（R5）の前 | なし | 電池、85% | 3.79 3.01 3.47 |
| 18:04:26 | 回 2 の終わり | `go2oo-gazebo` 280.27 %、745.7 MiB。`go2-app-run-a266eb0789de` 140.23 %、80.07 MiB | 電池、85% | 5.05 3.54 3.63 |
| 18:04:34 | 回 2 を止めた後 | `go2-app-run-a266eb0789de` 99.89 %、124.6 MiB | 電池、85% | 4.96 3.55 3.64 |
| 18:06:35 | 検出（`detector_check`）の前 | なし | 電池、84% | 2.57 3.14 3.46 |
| 18:06:50 | 検出の後 | `go2-app-run-5834b80b3397` 99.34 %、206.3 MiB | 電池、84% | 2.98 3.20 3.48 |
| 18:07:49 | 種類ごとの確からしさの確かめの前 | `go2-app-run-5834b80b3397` 48.64 %、238.2 MiB | 電池、83% | 4.27 3.48 3.56 |
| 18:08:03 | その後 | `go2-app-run-5834b80b3397` 100.10 %、219.7 MiB | 電池、83% | 4.23 3.50 3.57 |
| 18:09:09 | 回 3（L、Table の M15）の前 | `go2-app-run-5834b80b3397` 99.91 %、236.6 MiB | 電池、83% | 3.60 3.49 3.55 |
| 18:09:33 | 回 3 の終わり | `go2oo-gazebo` 236.58 %、742 MiB。`go2-app-run-5834b80b3397` 101.39 %、248.1 MiB | 電池、82% | 4.02 3.59 3.59 |

- `go2-app-run-…` の 2 つは、本体の compose project の `app` の `run`（ほかの仕事の軽いテスト）で、私のものではない。`go2-gazebo-1` でも
  こちらの重い段でもない。どちらも Gazebo の回 2・回 3 の終わりと、検出の後に重なった。絵は時間を測らないので影響しない。検出の推論は
  中央値 333.1 ms で、H3（324.8 ms）・H3 の下見（322〜388 ms）と同じくらい。
- 動かしたのは、`G` の Gazebo（`go2oo-gazebo`、`docker rm -f` と `G down` で止めた）と `G run --rm … app`（`run_formula`）、計画の
  `TA`・`T`、名前を付けた使い捨ての `docker run --rm --name oo-h3b-…`（検出器のイメージと Fuel のイメージ）だけ。終わった後に残った
  こちらの container とネットワークはない。

### 軽い確かめ：`record_frames` を `patch-0005-fuel2` で

```bash
docker run --rm --network none -v "$WT":/wt -w /wt -e PYTHONPATH=/wt:/wt/nl_control --entrypoint bash \
    go2-voice-ltl-gazebo:patch-0005-fuel2 -c 'source /opt/ros/jazzy/setup.bash && command -v gz && python3 -c \
    "import numpy, rclpy, sensor_msgs.msg, nav_msgs.msg, std_msgs.msg, gazebo.tools.record_frames; …write_png(…)"'
```

- **exit 0**、`gz` は `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`。R-H3f の形（`PYTHONPATH` を ROS を読み込む前に渡す）なので、H3 の
  1 回目のような `rclpy` の不足は出ない。

### 撮った見え方（12 枚、`out/h3-preview/`）

```bash
OBJECTS_SOURCE=detector G run -d --no-deps --service-ports --name go2oo-gazebo gazebo \
    ros2 launch /work/gazebo/launch/go2_gazebo.launch.py gui:=false
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "<式>"
docker exec -e PYTHONPATH=/work:/work/nl_control go2oo-gazebo bash -c \
    'source /opt/ros/jazzy/setup.bash && python3 /work/gazebo/tools/record_frames.py /work/out/h3-preview --view <名前> [--shift=<s>]'
```

- 回 1（18:01:44〜18:03:07）：起動してすぐ `spawn_lying`（`--timeout 180`。撮れたのはシミュレーションの 1.0 秒）、`F standing`、`spawn`、
  `F(turned_left & X F turned_left)`、`F moved_forward`、`Bf`、`Bf_s1`〜`Bf_s4`。
- 回 2（18:03:19〜18:04:26）：起動し直し、`F standing`、`F turned_right`、
  `F(moved_forward & X F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward))))`、`R5`、`R5_s1`〜`R5_s4`。
- どの `run_formula` も「結果：done」。ずらしは計画の 4 つで、負の数で始まるものも含めてすべて `--shift=<s>` の形（R-H3f）。どの
  `record_frames` も 1 回で exit 0（止まりの門、位置の組 0.02 m・1°、ずらしの後の正解の確かめを通った）。撮り直した見え方はない。
- 回 2 は Gazebo の起動直後に `F standing` を始めたので、app の側に rmw の `invalid data size` が出た（H3 と同じ、立ち上がり中の
  トピック）。動きは変わらず、done まで行った。
- 1 枚あたり 5〜7 秒（ずらしは置き直しと 1 秒の落ち着きを含む）。B（Bf へ行く途中）は撮っていない（試験の姿勢でない）。

| 見え方 | 本当の姿勢（odom、絵の時刻） | 絵・点群の時刻（sim 秒） | 点群の点 | 論理カメラが見た物 |
|---|---|---|---|---|
| spawn_lying | (0.004, 0.002)、高さ 0.068、−1.5° | 1.0・1.7 | 209 | box_1, chair_1 |
| spawn | (0.001, −0.012)、−1.2° | 19.0・19.9 | 216 | box_1, chair_1 |
| Bf | **(−0.540, −0.090)、−171.6°** | 42.0・42.8 | 349 | sofa_1, suitcase_1 |
| Bf_s1〜s4 | 同じ（物を動かして見え方をずらす） | 46.0〜60.5 | 304〜411 | sofa_1, suitcase_1（4 枚とも） |
| R5 | **(−0.359, −2.709)、−98.0°** | 31.0・31.5 | 245 | traffic_cone_1 |
| R5_s1〜s4 | 同じ | 35.0〜50.0 | 242〜270 | traffic_cone_1（4 枚とも） |

- **測った向きとの比べ：** Bf は H3 の下見の測った箱（x −0.506〜−0.560、y −0.054〜−0.141、向き −166.6〜−172.4°）の中。R5 は y が箱の中、
  **x −0.359 と向き −98.0° が箱（x −0.342〜−0.225、向き −95.1〜−97.6°）の外**（x で 0.017 m、向きで 0.38°）。`tests/test_household.py`
  の `MEASURED_RANGES` はこの箱を固定している（下の「controller に返すこと」）。
- 目標の方位と距離（`tests/test_household.py` の `sight` で、この回の姿勢から）：Bf でソファはカメラから −10.5°・3.72 m（台から −9.7°・
  4.04 m）、スーツケースはカメラから +44.5°・1.256 m（台から +35.8°・1.507 m）。R5 でコーンはカメラから +12.2°・1.421 m（台から +9.9°・1.742 m）。
- **M15：** 最初の絵で、Sofa と Suitcase1 は Bf に、Construction Cone は R5 に、箱と椅子は spawn に写った。ソファの前にスーツケースは
  かぶらない（Bf でスーツケースの枠は u 33〜203、ソファは u 304〜412）。Table の試験の姿勢 L は検出の試験の姿勢でなく、12 枚にないので、
  回 3（18:09:09〜18:09:33、起動し直し、`F standing`、`F turned_left`）で `out/h3b-L/` に `L` を 1 枚だけ撮った：(0.019, 0.029)、96.5°、
  論理カメラは table_1、Table は絵の左寄りに全体が写った。この 1 枚は `detector_check` にかけていない。

### R-H3c の確かめ：検出できないモデルは試験の見え方に入らない

検出の試験の見え方（Bf・R5 とそれぞれのずらし 4 つ、10 枚）で、4 つのやり方で確かめた。

1. **投影**（作業用の小さいスクリプト、Fuel のイメージ、`--network none`）：絵の時刻の台の姿勢と、ずらした絵では記録の `moved`
   （動かした後のモデルの姿勢）から、配置のどのモデルの体（足跡 × 高さ、モデルの yaw で回した 8 つの角）も
   `detector.camera.world_to_camera` と記録の K で絵に写した。絵（u 0〜640）に入るのは、Bf の 5 枚ではソファとスーツケースだけ、R5 の 5 枚では
   コーンだけ。table_1 の体のいちばん内側の u は Bf 813、s1 1067、s2 717、s3 693、s4 925（どれも 640 の外）。box_1・chair_1 は後ろ。
2. **`tests/test_household.py` の関数**（`rendered`、`bearings`、`shifted`、app のイメージ）をこの回の 2 つの姿勢で：Bf と 4 つのずらしで
   見えるのは {sofa_1, suitcase_1}、R5 と 4 つのずらしで {traffic_cone_1}。table_1 の視野の外への余裕は Bf 9.49°、s1 16.13°、s2 5.09°、
   **s3 3.67°**、s4 13.03°（固定の最小 2.31° より広い）。R5 では table_1 は 84.8° 以上外。
3. **論理カメラ**（記録の正解の `visible`）：上の表のとおり、Bf の 5 枚は sofa_1・suitcase_1、R5 の 5 枚は traffic_cone_1 だけ。
4. **検出と絵**：Bf・R5 の 10 枚の当たりは、どれも目標の正解の枠の上（下の表）。絵を目で見ても、ほかのモデルはない。

- 1 つだけ：**`Bf_s3` の右の端に table_1 の影の先が 4 画素**（行 264、u 635〜639、灰色 90〜174、床は 194）写った。世界の太陽
  （方向 (−0.5, 0.1, −0.9)）で落ちる影を同じように投影すると、table_1 の影だけが u 631 から絵に入る。モデルの体ではなく、検出の当たりも
  ない（`Bf_s3` の当たりはソファとスーツケースだけ）。

### 検出（`detector_check.py`、YOLOE-11s 文字 @ 640、87 の言葉 + 捨てる 3 つ）

```bash
docker run --rm --name oo-h3b-detect -v "$WT":/work -v /dev/null:/work/env.txt:ro -w /work go2-voice-ltl-detector:yoloe-11s \
    python3 gazebo/tools/detector_check.py /work/out/h3-preview --detector yoloe
```

- 18:06:35〜18:06:50、`real` 14.7 秒、**exit 1**（`failed` は `Bf_s2` だけ）。12 枚、`errors` なし。推論 1 回の時間は中央値 333.1 ms、
  最大 353.5 ms（1 スレッド）。

| 見え方 | 当たり（`sighting` 0.25 以上。名前・確からしさ、正解の枠との IoU） | §13.3 の絵ごとの線 |
|---|---|---|
| spawn_lying | chair 0.707（chair_1）、chair 0.414、box 0.278（box_1） | 数えない（`preview` は見ない。`no_recall`） |
| spawn | chair 0.545（chair_1、0.549）、chair 0.351（0.963） | `no_recall`（検出できる正解なし）、落ちた線なし |
| Bf | **sofa 0.782**（0.805）、**suitcase 0.559**（0.861） | 通った |
| Bf_s1 | sofa 0.816（0.792）、suitcase 0.766（0.868） | 通った |
| Bf_s2 | **chair 0.732（sofa_1、0.79）**。スーツケースは当たりなし | **落ちた**：sofa_1・suitcase_1 の組なし、recall 0.000 |
| Bf_s3 | sofa 0.738（0.804）、suitcase 0.627（0.851） | 通った |
| Bf_s4 | sofa 0.754（0.806）、suitcase **0.384**（0.890） | 通った |
| R5 | **traffic_cone 0.429**（0.543） | 通った |
| R5_s1 | traffic_cone 0.472（0.573） | 通った |
| R5_s2 | traffic_cone 0.958（0.514） | 通った |
| R5_s3 | traffic_cone 0.807（0.521） | 通った |
| R5_s4 | traffic_cone 0.870（0.552） | 通った |

- `preview`（設計書 §12.8 の 3）：

| 目標 | 姿勢 | 姿勢の見え方 | ずらしで当たった数（線は 2） | 他の目標の名前 | いちばん低い余裕（確からしさ − 0.35） | 合否 |
|---|---|---|---|---|---|---|
| sofa_1 | Bf | sofa 0.782 | 3（s1 0.816、s3 0.738、s4 0.754。s2 は chair） | なし | +0.388（s3） | **合格** |
| suitcase_1 | Bf | suitcase 0.559 | 3（s1 0.766、s3 0.627、s4 0.384。s2 は当たりなし） | なし | +0.034（s4）。線に要る姿勢＋2 つでは +0.209 | **合格** |
| traffic_cone_1 | R5 | traffic_cone 0.429 | 4（0.472、0.958、0.807、0.870） | なし | +0.079（姿勢の見え方） | **合格** |
| spawn（始めの位置） | — | box と chair のほかの名前なし | — | — | — | 合格 |

- `detectable` は `["sofa_1", "suitcase_1", "traffic_cone_1"]`、`enough` は true。
- 取り違えの表（`confusion`）で目標に乗った違う名前は、`Bf_s2` のソファの chair 0.732 だけ。chair は目標の名前でないので、`preview` の
  「他の目標の名前」には入らない。
- H3 の下見（Part B の最後の部屋、x4b・v6r）と比べて：ソファは 5/5 から姿勢＋3/4（`Bf_s2` が chair に）、スーツケースは姿勢＋3/4 のまま
  （下見は s4 が trash_can、この回は s2 が切れて当たりなし）、コーンは 5/5 のままだが姿勢の見え方が 0.634 から 0.429 に下がった。
- 線の 3 は満たすが、`detector_check` の終わりのコードは 1。`Bf_s2` の絵ごとの §13.3 の線（ソファの名前の誤りと、端で切れた
  スーツケース）のため。H3b の線（`preview`）ではない。H6 は §13.3 を本当の姿勢（ずらしなし）で数える。

### わけを探した：種類ごと・言葉ごとの確からしさ（`Bf_s2` とうすい余裕）

H3 と同じやり方：使い捨てのスクリプト（作業用の場所。リポジトリには入れていない）で、同じ YOLOE と同じ 90 の言葉で、名前ごとの NMS
（`conf` 0.05）にかけ、各正解の枠（記録の正解と配置の高さ）と IoU 0.5 以上の箱のいちばん高い確からしさを言葉ごとに数えた
（18:07:49〜18:08:03）。全部の一覧で勝った名前の値（sofa 0.782 = `couch` 0.782、chair 0.732 など）と一致する。

| 見え方 | ソファ：`couch` | ソファ：`chair` | スーツケース：`suitcase` | スーツケースの次の言葉 | コーン：`traffic cone`（IoU） |
|---|---|---|---|---|---|
| Bf | **0.782** | 0.618 | **0.559** | television 0.292 | — |
| Bf_s1 | **0.816** | 0.05 未満 | **0.766** | — | — |
| Bf_s2 | 0.716 | **0.732** | 0.05 未満 | television 0.151 | — |
| Bf_s3 | **0.738** | 0.650 | **0.627** | garbage bin 0.198 | — |
| Bf_s4 | **0.754** | 0.621 | **0.384** | garbage bin 0.252 | — |
| R5・R5_s1〜s4 | — | — | — | — | 0.429（0.54）、0.472（0.57）、0.958（0.51）、0.807（0.52）、0.870（0.55） |

- **`Bf_s2` のソファ：** `couch` 0.716 と `chair` 0.732 の差は 0.016 で、五分五分。agnostic NMS で 1 つの物に 1 つの名前なので、chair が残る。
  ほかの 4 枚では `couch` が勝つ（chair との差は 0.088 以上）。H3 が B（4.4 m）で見た、ソファと椅子が絵ごとに五分五分になる取り違えが、
  この回は Bf の 1 枚（−5° 回した s2）だけに出た。ソファはカメラから 3.72 m。
- **`Bf_s2` のスーツケース：** 正解の枠は u 0〜140（左の端で切れる。体の投影は u −158〜139）。`suitcase` は 0.05 にも届かない。
  部屋の直しの見直しの予想（「`Bf_s2` は測ったどの向きでも左の端で切れる。外れは切れたためで、検出の誤りではない」）のとおり。
- **コーン：** `traffic cone` のほかに 0.05 以上の言葉はない。IoU 0.51〜0.57 は下見と同じ（影が正解の枠に入る）。姿勢の見え方の 0.429 は
  下見（0.634）より低い。この回の R5 の向き −98.0° は、下見の 4 回でいちばん回った −97.6° より 0.4° 多い（わけかどうかは確かめていない）。

### 決めたこと

- **数える種類（`detectable`）：** `preview` を通ったのは sofa_1・suitcase_1・traffic_cone_1 で、配置はもうこの 3 つが `true`、
  box_1・chair_1・table_1 が `false`（部屋の直しのまま）。だから配置は変えない。
- **lamp_1（R-H3d）：** ランプは部屋を出た（`household.json` に lamp_1 も trash_can_1 もない）。R-H3d の直しで
  `test_detector_check_command_line_with_the_oracle` は凍った `SCENE_LAYOUT` を読むので、配置の旗との縛りもない。決めることはない。
- **`BY_LABEL`：** なし（`{}` のまま）。目標に乗った違う名前は `Bf_s2` の chair 0.732 だけで、正しい名前（`couch` 0.716）より高い。
  計画の決まり「違う名前が 0.35 と正しい名前の確からしさの間にあるとき」に当たらない。chair の閾を 0.732 より上げると、始めの位置の
  椅子（0.545・0.707）も確かにならなくなり、しかもソファの名前は出ない（agnostic NMS でソファの箱は消えたまま）。
- **外す検出の言葉：** なし（`vocabulary.json` は変えない）。`Bf_s2` を直す言葉は chair の `chair` を外すことだけで、椅子という基本の
  種類の検出の言葉をなくす。`preview` は言葉を変えずに通る。

### controller に返すこと

1. **R5 の姿勢が固定の箱の外：** この回の R5 は (−0.359, −2.709, −98.0°) で、`tests/test_household.py` の `MEASURED_RANGES["R5"]`
   （x −0.342〜−0.225、向き −95.1〜−97.6°）から x で 0.017 m、向きで 0.38° 外。この姿勢で、R-H3c（上の 2）と、目標の式の計画（下の表）は
   どれも成り立つ。箱を広げて数を固定し直すかは、H3b のファイルの外（テスト）なので controller に返す。
2. **`Bf_s2` の絵ごとの §13.3 の線が落ちた：** ソファが chair（0.016 の差）、スーツケースは端で切れて当たりなし。`preview` には響かない。
   H6 は本当の姿勢（ずらしなし）で数える。この回の Bf の見え方ではソファは `couch` 0.782 で、chair 0.618 より 0.164 高い。
3. **うすい余裕：** スーツケースは `Bf_s4` で 0.384（+0.034）、コーンは R5 の姿勢の見え方で 0.429（+0.079）、コーンの IoU は 0.51〜0.57。
   どれも線の中だが、R-H3h (4) が受け入れたうすさのとおり。
4. **`Bf_s3` の右の端の table_1 の影（4 画素）：** モデルではなく、検出もない。メモだけ。

- この回の姿勢からの計画（`tests/test_household.py` の `build`・`area`・`moves`、app のイメージ、PLANT だけ、合成なし。手前のキーは
  `gw.near` の数）：

| 式 | 始め | 面積 | 手 | 手前のキー | PLANT の状態 | 4 つのずれ（面積・手） |
|---|---|---|---|---|---|---|
| `F near_sofa_1` | Bf（この回） | 140 | 5 | 20 | 2 760 | (154,5)(140,5)(140,5)(154,5) |
| `F around_sofa_1` | Bf（この回） | 208 | 49 | — | 14 256 | (208,52)(208,50)(224,55)(208,53) |
| `F near_suitcase_1` | Bf（この回） | 56 | 2 | 7 | 1 224 | (56,5)(56,1)(49,2)(49,2) |
| `F around_suitcase_1` | Bf（この回） | 100 | 26 | — | 7 776 | (100,28)(100,28)(100,29)(100,26) |
| `F near_traffic_cone_1` | R5（この回） | 72 | 1 | 7 | 1 584 | (63,8)(56,1)(63,4)(56,1) |
| `F around_traffic_cone_1` | R5（この回） | 100 | 28 | — | 8 256 | (110,31)(110,31)(110,30)(121,35) |

- どの始めとずれからも計画がある。この表は H5・H6 のための参考で、固定はしていない（固定の数は `MEASURED_POSES` の H3 の Bf と
  導いた R5 のもの）。

### テスト

- 配置・単語帳・上書き・コードは変えていないので、§12.4 の数はそのまま。`T tests/test_household.py`：30 passed（41.3 秒、18:09:56〜18:10:38）。
- 全部（`T tests nl_control/tests`、18:10:38〜18:16:04）：**2564 passed, 18 skipped, 29 deselected**（324.7 秒）。`feat/open-objects` の
  `206fe97` と同じ数。終わり（18:16:13）は、こちらの container なし、電池 80%、`lowpowermode` 0、load 6.85 5.08 4.26。`pmset -g log` に、
  16:56:54 の後の眠りはない。

### 変えたもの

- この記録だけ。配置・単語帳・`detector/yoloe.py` の `BY_LABEL`・テスト・イメージは変えていない。PR #4 のファイル（D12）には触れていない。
  app のイメージにも本来の Linux の準備にも、新しい依存は入っていない。環境の値は表示していない。

## H3b replay：merge の後のコードで H3b の絵を判定し直した（2026-11-18）

- 枝：`objects2/h3b-replay`（`feat/open-objects` の `80f181d`。pre-final の直し `09e8863`、tracker `9b6550a`、H5 の固定 `80f181d` の
  merge の後）。worktree `~/work/physicalAI/.go2-worktrees/oo-h3b-replay`。ROS の domain は 63。**Gazebo は動かしていない。**
- 絵：H3b の 12 枚（`~/work/physicalAI/.go2-worktrees/oo-h3b/out/h3-preview/` の 36 ファイル）を、この worktree の `out/h3-preview/` に
  写した（sha256 は 36 とも同じ）。git に入れていない（設計書 §11.11）。
- 検出器のイメージ：`go2-voice-ltl-detector:yoloe-11s`（H2、`016348e47306`）。作り直していない（H3b の後に変わった `docker/` は
  `Dockerfile.gazebo` だけ）。

```bash
docker run --rm --network none --name oo-h3b-replay-detect -e ROS_DOMAIN_ID=63 -v "$WT":/work -w /work \
    go2-voice-ltl-detector:yoloe-11s python3 gazebo/tools/detector_check.py /work/out/h3-preview --detector yoloe
```

### 結論

**`preview` の線はそのまま満たす。** `detectable` は `["sofa_1", "suitcase_1", "traffic_cone_1"]`、`enough` は true（線は 3 以上）。
12 枚、`errors` なし、exit 1（`failed` は `Bf_s2` だけ、H3b と同じ）。どの当たりの名前・確からしさ・IoU も、`confusion` も H3b と同じ。
変わった数は 1 つの組だけ：`R5_s2` のコーンの位置の誤差（0.133 → 0.130 m）と大きさの誤差（0.132 → 0.133 m）。

| 目標 | 姿勢 | H3b（姿勢、s1〜s4。確からしさ・IoU） | この回 | ずらしの当たり | いちばん低い余裕 | 合否 |
|---|---|---|---|---|---|---|
| sofa_1 | Bf | 0.782 (0.805)、0.816 (0.792)、s2 は chair 0.732、0.738 (0.804)、0.754 (0.806) | 0.782 (0.805)、0.816 (0.792)、s2 は chair 0.732、0.738 (0.804)、0.754 (0.806) | 3 / 3 | +0.388 / +0.388 | 合格 / 合格 |
| suitcase_1 | Bf | 0.559 (0.861)、0.766 (0.868)、s2 は当たりなし、0.627 (0.851)、0.384 (0.890) | 0.559 (0.861)、0.766 (0.868)、s2 は当たりなし、0.627 (0.851)、0.384 (0.890) | 3 / 3 | +0.034 / +0.034 | 合格 / 合格 |
| traffic_cone_1 | R5 | 0.429 (0.543)、0.472 (0.573)、0.958 (0.514)、0.807 (0.521)、0.870 (0.552) | 0.429 (0.543)、0.472 (0.573)、0.958 (0.514)、0.807 (0.521)、0.870 (0.552) | 4 / 4 | +0.079 / +0.079 | 合格 / 合格 |
| spawn（始めの位置） | — | box と chair のほかの名前なし | 同じ | — | — | 合格 / 合格 |

（「/」の左が H3b、右がこの回。）

### 変わった数

| 見え方 | 組 | H3b（`265f17c`） | この回（`80f181d`） | §13.3 の線 |
|---|---|---|---|---|
| R5_s2 | traffic_cone_1 の `pos_err`（`max_pos_err`・`median_pos_err` も） | 0.133 | 0.130 | `max_pos_err_target` 0.15 |
| R5_s2 | traffic_cone_1 の `size_err`（`max_size_err` も） | 0.132 | 0.133 | `max_size_err` 0.4 |

- `R5_s2` の `covers` false・`filled` false・`fill_short_m` 0 は同じで、この絵の線はどちらでも通る。ほかの 11 枚の `pos_err`・
  `size_err`・`covers`・`filled`・`fill_short_m` はすべて同じ。
- 推論 1 回の時間は中央値 327.3 ms、最大 336.9 ms（H3b は 333.1 ms・353.5 ms。判定の線ではない）。

### どの merge で変わったか

同じイメージ・同じ絵で、merge ごとのコードを `git archive <commit>` で `out/code-<commit>/`（git に入れていない）に出し、
`-v out/code-<commit>:/work -v out/h3-preview:/frames:ro` で同じ `detector_check.py … --detector yoloe` にかけた（推論の時間のほか、
`set` の名前は絵の場所 `frames` になる）。

| コード | R5_s2 のコーン（`pos_err` / `size_err`） | ほかの数 |
|---|---|---|
| `265f17c`（H3b） | 0.133 / 0.132 | H3b の記録とすべて同じ |
| `09e8863`（pre-final の直しの merge） | 0.133 / 0.132 | `265f17c` と同じ |
| `9b6550a`（tracker の merge、R-PF3〜R-PF3d） | 0.130 / 0.133 | `80f181d` と同じ |
| `80f181d`（いまの先） | 0.130 / 0.133 | — |

- H3b のコードが今日も H3b の数をそのまま出すので、判定は決まった答えを出す。違いはコードから来ていて、ゆらぎではない。
- 数を変えたのは tracker の merge（`detector/geometry.py`・`detector/tracker.py` の遠い端と横の決まり）。pre-final の直しと H5 の固定
  （`gazebo/bridge_core.py` の論理の足跡の下限）は、どの数も変えない。

### D11 の門と、ほかの container・電源

どの検出の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった（待たなかった）。電源はどの時も
AC、100%、charged。`lowpowermode` は 0。load は host の `sysctl -n vm.loadavg`（1・5・15 分）。`pmset -g log` の最後の眠りは
10:36 の DarkWake で、10:36:28 から FullWake のまま（この回の間に眠りはない）。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | load |
|---|---|---|---|
| 14:43:33 | 始め（門） | なし | 2.28 3.96 4.43 |
| 14:43:59〜14:44:13 | 検出（`80f181d`、`real` 13.9 秒） | — | 前 2.28、後 2.57 3.80 4.35 |
| 14:45:31〜14:45:57 | 比べ（`265f17c`、`09e8863`） | — | 前 3.96 3.88 4.32、後 4.08 3.89 4.31 |
| 14:46:15〜14:46:28 | 比べ（`9b6550a`） | — | 前 3.33 3.73 4.25、後 3.31 3.71 4.23 |
| 14:46:28 | 終わり | なし | 3.31 3.71 4.23 |

- 14:45:04 と 14:45:20 の比べの 2 回は、出したコードに `nl_control/` と `sim/` がなく、読み込み（`ModuleNotFoundError`）で止まった。
  推論の前なので、木全体を出して 14:45:31 から撮り直した。
- 動かしたのは名前を付けた使い捨ての `docker run --rm --network none --name oo-h3b-replay-…`（検出器のイメージ）だけ。残った
  container はない。比べの線はどれも H3b と同じなので、load の決まり（R-PAR）で撮り直すものはない。

### 変えたもの

- この記録だけ。コード・配置・単語帳・`BY_LABEL`・テスト・イメージは変えていない。PR #4 のファイル（D12）には触れていない。
  app のイメージにも本来の Linux の準備にも、新しい依存は入っていない。環境の値は表示していない。

## H3b replay 2：finish6 の merge の後のコードで H3b の絵をもう一度判定した（2026-11-18）

- 枝：`objects2/h3b-replay2`（`feat/open-objects` の `ddbfdbe`。finish6 の merge の後：tracker の線 `91656de`（R-PF3e の cover125 の
  狭めは見直しの後で戻した。cover125 は書いた限りのまま）、ソファの奥 `e4ae4fc`（R-H6a）、H6 の直し `ddbfdbe`（R-H6e の段ボールの
  模様、R-H6c の待ち、R-H6d の `MEASURED_RANGES` の広げ））。worktree `~/work/physicalAI/.go2-worktrees/oo-h3b-replay2`。
  ROS の domain は 63。**Gazebo は動かしていない。**
- 絵：H3b の 12 枚（`~/work/physicalAI/.go2-worktrees/oo-h3b/out/h3-preview/` の 36 ファイル）を、この worktree の `out/h3-preview/` に
  写した（sha256 は 36 とも同じ）。git に入れていない（設計書 §11.11）。
- 検出器のイメージ：`go2-voice-ltl-detector:yoloe-11s`（H2、`016348e47306`）。作り直していない（`80f181d` の後に `docker/` は
  変わっていない）。

```bash
docker run --rm --network none --name oo-h3b-replay2-detect -e ROS_DOMAIN_ID=63 -v "$WT":/work -w /work \
    go2-voice-ltl-detector:yoloe-11s python3 gazebo/tools/detector_check.py /work/out/h3-preview --detector yoloe
```

### 結論

**`preview` の線はそのまま満たす。** `detectable` は `["sofa_1", "suitcase_1", "traffic_cone_1"]`、`enough` は true（線は 3 以上）。
12 枚、`errors` なし、exit 1（`failed` は `Bf_s2` だけ、H3b と同じ）。どの当たりの名前・確からしさ・IoU も、`confusion` も H3b・1 回目の
replay と同じ。変わった数は 2 つの組：`Bf_s3` のソファと `R5_s2` のコーン。どちらも oracle の `covers` と `filled` が false から true に
なり、位置の誤差は小さく、大きさの誤差は少し大きくなった（どちらの絵も §13.3 の線を前もいまも通る）。変えたのはソファの奥の merge
`e4ae4fc`（R-H6a、`detector/geometry.py`）。

| 目標 | 姿勢 | H3b（姿勢、s1〜s4。確からしさ・IoU） | 1 回目の replay（`80f181d`） | この回（`ddbfdbe`） | ずらしの当たり | いちばん低い余裕 | 合否 |
|---|---|---|---|---|---|---|---|
| sofa_1 | Bf | 0.782 (0.805)、0.816 (0.792)、s2 は chair 0.732、0.738 (0.804)、0.754 (0.806) | 0.782 (0.805)、0.816 (0.792)、s2 は chair 0.732、0.738 (0.804)、0.754 (0.806) | 0.782 (0.805)、0.816 (0.792)、s2 は chair 0.732、0.738 (0.804)、0.754 (0.806) | 3 / 3 / 3 | +0.388 / +0.388 / +0.388 | 合格 / 合格 / 合格 |
| suitcase_1 | Bf | 0.559 (0.861)、0.766 (0.868)、s2 は当たりなし、0.627 (0.851)、0.384 (0.890) | 0.559 (0.861)、0.766 (0.868)、s2 は当たりなし、0.627 (0.851)、0.384 (0.890) | 0.559 (0.861)、0.766 (0.868)、s2 は当たりなし、0.627 (0.851)、0.384 (0.890) | 3 / 3 / 3 | +0.034 / +0.034 / +0.034 | 合格 / 合格 / 合格 |
| traffic_cone_1 | R5 | 0.429 (0.543)、0.472 (0.573)、0.958 (0.514)、0.807 (0.521)、0.870 (0.552) | 0.429 (0.543)、0.472 (0.573)、0.958 (0.514)、0.807 (0.521)、0.870 (0.552) | 0.429 (0.543)、0.472 (0.573)、0.958 (0.514)、0.807 (0.521)、0.870 (0.552) | 4 / 4 / 4 | +0.079 / +0.079 / +0.079 | 合格 / 合格 / 合格 |
| spawn（始めの位置） | — | box と chair のほかの名前なし | 同じ | 同じ | — | — | 合格 / 合格 / 合格 |

（「/」は左から H3b、1 回目の replay、この回。）

### 変わった数

| 見え方 | 組 | 数 | H3b（`265f17c`） | 1 回目（`80f181d`） | この回（`ddbfdbe`） | §13.3 の線 |
|---|---|---|---|---|---|---|
| Bf_s3 | sofa_1 | `pos_err`（`max_pos_err` も） | 0.137 | 0.137 | 0.078 | `max_pos_err_target` 0.15 |
| Bf_s3 | sofa_1 | `size_err`（`max_size_err` も） | 0.169 | 0.169 | 0.179 | `max_size_err` 0.4 |
| Bf_s3 | sofa_1 | `covers`・`filled`・`fill_short_m` | false・false・0 | false・false・0 | true・true・0.017 | （C2 の判定。線ではない） |
| Bf_s3 | 絵 | `median_pos_err`・`covers_all` | 0.086・false | 0.086・false | 0.056・true | `median_pos_err` 0.25 |
| R5_s2 | traffic_cone_1 | `pos_err`（`max_pos_err`・`median_pos_err` も） | 0.133 | 0.130 | 0.090 | `max_pos_err_target` 0.15 |
| R5_s2 | traffic_cone_1 | `size_err`（`max_size_err` も） | 0.132 | 0.133 | 0.147 | `max_size_err` 0.4 |
| R5_s2 | traffic_cone_1 | `covers`・`filled`・`fill_short_m`・`covers_all` | false・false・0・false | false・false・0・false | true・true・0.069・true | （C2 の判定。線ではない） |

- `Bf_s3` のスーツケース（0.035・0.124、`covers` true・`filled` false）と、ほかの 10 枚の `pos_err`・`size_err`・`covers`・`filled`・
  `fill_short_m` はすべて 1 回目と同じ。`R5`（姿勢の見え方）のコーンの `covers` false（`filled` true、`fill_short_m` 0）も 3 回とも同じ。
- いまの先で `covers_all` が false の絵は `R5` だけになった（H3b と 1 回目は `Bf_s3`・`R5`・`R5_s2` の 3 枚）。
- 推論 1 回の時間は中央値 320.0 ms、最大 344.6 ms（H3b は 333.1・353.5 ms、1 回目は 327.3・336.9 ms。判定の線ではない）。

### どの merge で変わったか

1 回目と同じやり方：同じイメージ・同じ絵で、merge ごとのコードを `git archive <commit>` で `out/code-<commit>/`（git に入れていない）に
出し、`-v out/code-<commit>:/work -v out/h3-preview:/frames:ro` で同じ `detector_check.py … --detector yoloe` にかけた（推論の時間のほか、
`set` の名前は絵の場所 `frames` になる）。

| コード | Bf_s3 のソファ（`pos_err` / `size_err` / `covers`） | R5_s2 のコーン（`pos_err` / `size_err` / `covers`） | ほかの数 |
|---|---|---|---|
| `80f181d`（1 回目の replay のコード） | 0.137 / 0.169 / false | 0.130 / 0.133 / false | 1 回目の記録とすべて同じ |
| `d407854`（H6 の記録の merge） | 0.137 / 0.169 / false | 0.130 / 0.133 / false | `80f181d` と同じ |
| `91656de`（tracker の線、R-PF3e を戻した merge） | 0.137 / 0.169 / false | 0.130 / 0.133 / false | `80f181d` と同じ |
| `e4ae4fc`（ソファの奥の merge、R-H6a） | 0.078 / 0.179 / true | 0.090 / 0.147 / true | `ddbfdbe` と同じ |
| `ddbfdbe`（いまの先） | 0.078 / 0.179 / true | 0.090 / 0.147 / true | — |

- `80f181d` が今日も 1 回目の数をそのまま出すので、違いはコードから来ていて、ゆらぎではない。
- 数を変えたのはソファの奥の merge だけ（`detector/geometry.py` の `_side`：遠い側に並ぶ面を、カメラが手前の側から 60° 以内で見ている
  なら、物はその後ろに続くので端としない）。`Bf_s3` のソファでは、背もたれの前の面（ソファの前から 0.72〜0.83 m）が遠い端と読まれなく
  なり、奥は単語帳の奥で埋まる（`tests/test_detector_geometry.py::test_the_recorded_sofa_at_bf_takes_the_priors_depth` が同じ絵を
  固定している：`fill_short_m` 0.017）。`R5_s2` のコーンでは、見える弧の両端の列が遠い側に並んで手前から見えるので、同じく奥が埋まる
  （`detector/geometry.py` の頭の説明が挙げる「the H3b cone: R5_s2」）。
- tracker の線の merge は 1 回入れて戻したので、どの数も変えない。H6 の直しの merge（`gazebo/bridge_core.py` の段ボールの模様、
  `gazebo/tools/objects_check.py` の待ちの時間、`tests/test_household.py` の固定）も、どの数も変えない。

### D11 の門と、ほかの container・電源

どの検出の前にも `docker ps --format '{{.Names}}'` を見た。`go2-gazebo-1` はどの時も動いていなかった（待たなかった）。電源はどの時も
AC、100%、charged。`lowpowermode` は 0。load は host の `sysctl -n vm.loadavg`（1・5・15 分）。`pmset -g log` の最後の眠りは
10:36 の DarkWake で、10:36:28 から FullWake のまま（この回の間に眠りはない）。

| 時刻（日本時間） | いつ | 動いていた container（`docker stats`） | load |
|---|---|---|---|
| 22:25:22 | 始め（門） | なし | 1.71 3.33 5.18 |
| 22:25:41〜22:25:54 | 検出（`ddbfdbe`、13.5 秒） | — | 前 1.82 3.24 5.10、後 1.94 3.20 5.05 |
| 22:26:17〜22:27:06 | 比べ（`80f181d`、`d407854`、`91656de`、`e4ae4fc`、1 つ 12 秒ほど） | — | 前 1.87 3.10 4.98、後 1.89 2.92 4.80 |
| 22:27:48 | 終わり | なし | 2.17 2.87 4.70 |

- 動かしたのは名前を付けた使い捨ての `docker run --rm --network none --name oo-h3b-replay2-…`（検出器のイメージ）だけ。残った
  container はない。1 分の load はどの時も 4 より低く、load の決まり（R-PAR）で撮り直すものはない。

### 変えたもの

- この記録だけ。コード・配置・単語帳・`BY_LABEL`・テスト・イメージは変えていない。PR #4 のファイル（D12）には触れていない。
  app のイメージにも本来の Linux の準備にも、新しい依存は入っていない。環境の値は表示していない。

## H5：logical の Gazebo の測り（S7、2026-11-13〜14）

- 枝：`objects2/h5-logical`（`objects2/prefinal-fix` の `97da92a` から。R-PAR：logical の式は `objects_check` を通るだけで、
  並べて直している種類の見張り（guard3）と追跡（tracker3）は触れない）。worktree `~/work/physicalAI/.go2-worktrees/oo-h5`。ROS の domain は 65
  （`G` の回は compose project `go2oo` の自分の network で、domain は使わない）。
- Gazebo：イメージ `go2-voice-ltl-gazebo:patch-0005-fuel2`（一覧の ID `c40a28c22024`、作り直していない）、`GO2_LAYOUT=household`、
  `OBJECTS_SOURCE=logical`。§2 の組と新しい種類は画面あり（`G --profile gazebo up -d --no-build gazebo`、画面は port 6081）。
  **回ごとに `G down` で止めて起動し直した。** 橋渡しはどの回も「物 6 個」（box_1・chair_1・suitcase_1・traffic_cone_1・sofa_1・table_1。
  部屋の直しの後の部屋）を置いた。
- app：`G run --rm --no-deps -T -e SIM_KIND=gazebo app …`（`go2-voice-ltl:dev`）。
- コード・テスト・固定は変えていない。変えたのはこの記録だけ。
- 順は R-PAR のとおり：§2 の組 1 → 新しい種類 → M0（RTF）を最後に。13 日 20:29:55 に Mac が蓋を閉じて眠り（下の「眠り」）、
  14 日 06:27 に蓋が開き、06:35 の門の確かめから、R-H5b のとおり眠りで切れた 1 回を流し直して続けた（1 回目の記録は `affdbd7`）。

### 結論

**H5 の線はすべて満たした。** §2 の組 1 は 10/10。新しい種類は 4 つとも手前 2/2・周回 2/2（16/16）。どの回も転ばない。
ただし次の 4 つを controller に返す（下の「controller に返すこと」）：テーブルは L から 4 回のうち 3 回 `measured: true` にならなかった
（手順の「測られるのを待つ」を満たさないまま式を流した。わけは LiDAR の点の数）、重い負荷の下で手順の「立つ」が 4 回 error で終わった
（Go2 は立っていて、次の式から続いた）、着いた Bf と R5 の姿勢が固定の範囲の外にある（R-H5a）、コーンの測った足跡が y でモデルより小さい。

| 線（計画 H5、設計書 §13.6・§13.4・§10.8） | しきい値 | 結果 |
|---|---|---|
| 一周 `F around_box_1` ×5 | 5 回で 4 回以上、うち 3 回続けて。巻き数 0.9 以上 1.5 未満、箱の縁まで 0.6 m 以上 | **5/5**（巻き数 1.042〜1.069、箱の縁まで 0.797〜0.872 m）。合格 |
| 椅子 `F near_chair_1` ×3 | 3/3。中心から 1.5 m・45° 以内、縁まで 0.45 m 以上 | **3/3**（1.084〜1.135 m・25.2〜29.6°、縁まで 0.782〜0.838 m）。合格 |
| 椅子と挨拶 `F(near_chair_1 & X F hello)` ×1 | 1/1、同じ線 | **1/1**（1.101 m・35.8°、縁まで 0.789 m）。合格 |
| 「箱の前まで」`F near_box_1`（`--target box_1`）×1 | 1/1。縁まで 1.5 m 以内・足跡に 45° 以内・どの足跡にも 0.30 m 以上、箱の中心から 1.5 m・45° 以内（O16、logical） | **1/1**（縁まで 0.845 m・8.5°、どの足跡にも 0.841 m、中心まで 1.167 m・22.8°）。合格 |
| §2 の組：転ばない | 傾き 1.0 rad 未満 | 10 回とも（最大 0.220）。合格 |
| ソファ（Bf）`F near_sofa_1` ×2 | 2/2。縁まで 1.5 m 以内、足跡に 45° 以内、どの足跡にも 0.30 m 以上 | **2/2**（縁まで 1.102〜1.180 m・0.0〜2.3°、どの足跡にも 0.544〜0.780 m）。合格 |
| ソファ（Bf）`F around_sofa_1` ×2 | 2/2。巻き数 0.9 以上 1.5 未満、縁まで 0.6 m 以上 | **2/2**（巻き数 0.923・0.917、縁まで 0.883・0.749 m）。合格。眠りで切れた 1 回は数えず、流し直した |
| **スーツケース（Bf）**`F near_suitcase_1` ×2 | 手前の線 | **2/2**（縁まで 0.972・1.137 m、29.6°・2.7°、どの足跡にも 0.963・1.128 m）。合格 |
| **スーツケース（Bf）**`F around_suitcase_1` ×2 | 周回の線 | **2/2**（巻き数 1.033・1.035、縁まで 0.892・0.896 m）。合格 |
| **コーン（R5）**`F near_traffic_cone_1` ×2 | 手前の線 | **2/2**（縁まで 0.953・0.984 m、0.0°・0.0°、どの足跡にも 0.952・0.984 m）。合格 |
| **コーン（R5）**`F around_traffic_cone_1` ×2 | 周回の線 | **2/2**（巻き数 1.103・1.104、縁まで 0.880・0.881 m）。合格 |
| テーブル（L、logical だけ）`F near_table_1` ×2 | 手前の線 | **2/2**（縁まで 1.233・1.160 m、3.2°・14.4°、どの足跡にも 0.931・1.069 m）。合格。2 回とも `measured: false` のまま（下） |
| テーブル（L）`F around_table_1` ×2 | 周回の線 | **2/2**（巻き数 1.087・1.081、縁まで 0.775・0.955 m）。合格。1 回目は測られた後、2 回目は `measured: false` のまま |
| 新しい種類：転ばない | 傾き 1.0 rad 未満 | 16 回とも（最大 0.208）。合格 |
| ソファの手前で前の物のビット（O14） | 記録だけ | 2 回とも立たなかった（「前方に物が出た」の行 0） |
| M0：RTF、画面あり 3 回・なし 3 回 | 記録だけ（線なし。D7 の判断） | 画面あり 0.79・0.85・0.82、画面なし 0.87・0.93・0.82（記録） |

- 負荷：どの回も、ほかの組の container が動いていて、1 分の load は回の前後の記録で 2.8〜103.9（下の表。07:11 には 118.9 も見た）。**外れた線はないので、R-PAR の決まりでやり直す回はない。**

### 命令

計画の「Commands」の `G`（`WT` はこの worktree）。1 回ごとに：

```bash
docker ps --format '{{.Names}}'                       # D11：go2-gazebo-1 がないこと
uptime; docker stats --no-stream --format '{{.Name}} {{.CPUPerc}} {{.MemUsage}}'; pmset -g batt; pmset -g custom | grep -i lowpower
G --profile gazebo down --remove-orphans; G --profile gazebo up -d --no-build gazebo
# ログに「Configured and activated unitree_guide_controller」と「Gazebo の Go2 の橋渡しを起動しました」が出るまで待つ（8〜11 秒）
# §2 の組 1：
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F(near_chair_1 & X F hello)"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near --target box_1 "F near_box_1"
# 新しい種類（Bf の例）：試験の姿勢の手順（計画「The room after the room fix」）を 1 つずつ
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F standing"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F(turned_left & X F turned_left)"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py resolve --target sofa_1 --timeout 300 &   # 姿勢に着く式の直前（F11）
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F moved_forward"
wait                                                  # resolve の終わり
G run --rm --no-deps -T app python3 -c '<odometry を 1 通読んで x, y, 向きを出す>'   # 着いた姿勢（R-H3i）
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py objects
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near --target sofa_1 "F near_sofa_1"
G --profile gazebo down --remove-orphans
```

- 前の物のビット（O14）は、`run` の標準エラーに流れる Session のログの「前方に物が出た」（`front_blocked`）の行を数えた。センサーが途切れた（「センサー（LiDAR か /sportmodestate）が途切れています」）ときも、前と後ろの「物が出た」が一緒に出るので、別に数えた。
- 回の前後の `docker stats`・`pmset`・`uptime`、`resolve` の出力、`objects` の 1 通、Gazebo のログは worktree の外の作業用のフォルダに残した（git に入れていない）。

### D11 の門、ほかの container、電源、load

- `go2-gazebo-1` はどの回の前にも動いていなかった（待たなかった）。こちらのほかの重い段もなかった。
- **この Mac はほかの組（R-PAR の並べた道）と分けていた。** 回の間、ほかの組の container（compose project `go2` の `go2-app-run-…`、
  名前を付けた `t3cmp-…`・`t3-after`）が計 100〜944 % の CPU で動いていて、1 分の load は 13 日の夜 2.8〜15.6、14 日の朝 6.6〜103.9（回の前後の記録）、
  ほとんどの時に 4 を超えた。`pgrep -f pytest` は 0 か 2（ほかの組の `T`）。**R-PAR の決まりで、load が 4 を超えた間に外れた線は数える前に
  やり直す。外れた線はなかった。**
- 電源：13 日は電池（61 → 43 %）。14 日は 06:35 に電池 33 % から始め、06:57 に AC がつながった（17 %。その後は充電して、M0 の終わりに 99 %）。
  `lowpowermode` はどの時も 0。
- 14 日の回は、どの回の前にも「起きている」の門（R-H5b：`ioreg` の `AppleClamshellState` が No か、`pmset -g batt` が AC）を見て、
  回の間は 5 秒ごとに門と `pmset -g log` の眠りを見た（眠れば止めて門へ戻る）。14 日の回の間に眠りはなかった。

| 回 | 前（日本時間） | 前の load（1・5・15 分） | 前のほかの container（CPU） | 後の load | 後の `go2oo-gazebo-1`（CPU、メモリ） | 電池（前 → 後） | `pytest`（前 / 後） |
|---|---|---|---|---|---|---|---|
| lap1 | 20:03:11 | 2.81 2.00 2.11 | なし | 11.57 6.06 3.72 | 292 %、1.747 GiB | 61 → 59 % | 0 / 0 |
| lap2 | 20:05:56 | 10.85 6.18 3.81 | なし | 6.33 6.26 4.22 | 372 %、1.246 GiB | 59 → 57 % | 0 / 0 |
| lap3 | 20:08:26 | 6.06 6.21 4.21 | なし | 10.85 7.99 5.32 | 257 %、1.254 GiB | 57 → 55 % | 0 / 2 |
| lap4 | 20:11:39 | 10.86 8.04 5.36 | `go2-app-run-2e2252b8de67` 751 % | 8.07 8.51 6.08 | 399 %、1.27 GiB | 55 → 53 % | 0 / 0 |
| lap5 | 20:14:28 | 8.07 8.51 6.08 | なし | 9.66 8.35 6.38 | 292 %、1.254 GiB | 53 → 51 % | 0 / 0 |
| chair1 | 20:17:09 | 9.66 8.35 6.38 | `go2-app-run-892888f07a08` 816 % | 9.79 8.60 6.58 | 254 %、1.258 GiB | 51 → 50 % | 0 / 2 |
| chair2 | 20:18:00 | 9.79 8.60 6.58 | `go2-app-run-…` 4 つ、計 809 % | 13.43 9.95 7.23 | 229 %、1.262 GiB | 50 → 50 % | 2 / 2 |
| chair3 | 20:19:01 | 13.43 9.95 7.23 | 同じ 4 つ、計 805 % | 12.93 10.44 7.60 | 231 %、1.262 GiB | 50 → 50 % | 2 / 2 |
| hello1 | 20:20:00 | 12.93 10.44 7.60 | 同じ 4 つ、計 817 % | 11.44 10.47 7.80 | 184 %、1.244 GiB | 50 → 49 % | 2 / 2 |
| box1 | 20:21:03 | 13.01 10.81 7.93 | 同じ 4 つ、計 807 % | 15.61 12.16 8.63 | 334 %、1.259 GiB | 49 → 48 % | 2 / 2 |
| sofa_near1 | 20:23:17 | 6.46 10.10 8.14 | `t3cmp-AL` 100 %、`go2-app-run-06abdcfe63d7` 100 % | 5.66 9.15 7.93 | 391 %、1.261 GiB | 48 → 46 % | 2 / 2 |
| sofa_near2 | 20:24:40 | 4.60 8.68 7.79 | `go2-app-run-06abdcfe63d7` 100 % | 6.93 8.57 7.80 | 382 %、1.252 GiB | 46 → 45 % | 2 / 2 |
| sofa_lap1 | 20:25:44 | 7.02 8.56 7.80 | `go2-app-run-06abdcfe63d7` 100 % | 14.35 11.38 9.14 | 404 %、1.261 GiB | 45 → 43 % | 2 / 0 |
| sofa_lap2（眠りで切れた。数えない） | 20:29:45 | 14.00 11.36 9.14 | `t3-after` 100 % | — | — | 43 % → | 0 / — |
| sofa_lap2（流し直し） | 06:35:52 | 7.68 26.44 27.99 | なし | 7.40 16.63 23.33 | 420 %、1.243 GiB | 33 → 29 % | 0 / 0 |
| suitcase_near1 | 06:39:52 | 6.55 16.14 23.08 | なし | 7.77 14.84 22.16 | 441 %、1.249 GiB | 29 → 29 % | 0 / 0 |
| suitcase_near2 | 06:40:57 | 7.85 14.63 22.00 | なし | 7.34 13.03 20.78 | 434 %、1.252 GiB | 29 → 28 % | 0 / 0 |
| suitcase_lap1 | 06:42:19 | 6.99 12.87 20.67 | 計 99 % | 7.07 10.80 18.69 | 407 %、1.245 GiB | 28 → 26 % | 2 / 0 |
| suitcase_lap2 | 06:44:39 | 6.58 10.64 18.59 | なし | 14.05 11.34 17.52 | 338 %、1.257 GiB | 26 → 24 % | 0 / 0 |
| cone_near1 | 06:47:14 | 14.05 11.39 17.50 | 計 905 % | 18.76 13.60 17.77 | 335 %、1.258 GiB | 24 → 23 % | 0 / 0 |
| cone_near2 | 06:48:49 | 21.73 14.38 17.99 | 計 917 % | 22.10 16.61 18.52 | 320 %、1.239 GiB | 23 → 22 % | 0 / 2 |
| cone_lap1 | 06:50:23 | 22.15 16.81 18.57 | 計 898 % | 21.41 19.40 19.33 | 247 %、1.256 GiB | 21 → 19 % | 2 / 0 |
| cone_lap2 | 06:54:03 | 22.34 19.63 19.41 | 計 891 % | 39.12 25.26 21.60 | 276 %、1.258 GiB | 19 → 17 %（AC） | 0 / 0 |
| table_near1 | 06:57:49 | 37.93 25.46 21.71 | 計 933 % | 31.65 36.20 28.49 | 217 %、1.249 GiB | 17 → 23 %（AC） | 0 / 2 |
| table_near2 | 07:04:13 | 29.72 35.64 28.39 | 計 864 % | 95.41 61.27 41.84 | 266 %、1.25 GiB | 23 → 30 %（AC） | 2 / 2 |
| table_lap1 | 07:10:52 | 103.90 64.21 43.11 | 計 887 % | 68.36 89.03 65.59 | 264 %、1.243 GiB | 30 → 36 %（AC） | 2 / 2 |
| table_lap2 | 07:17:00 | 64.58 87.16 65.33 | 計 944 % | 75.29 71.38 64.86 | 376 %、1.263 GiB | 37 → 47 %（AC） | 2 / 0 |

### §2 の組 1（設計書 §13.6、logical、`household`）

どの回も起動し直した Gazebo で、伏せから始まり、最初の動作は「立つ」。`run_formula` の終了コードはどれも 0。

| 回 | 起動（日本時間） | 式 | 判定 | 巻き数 / 箱の縁まで | 椅子の中心まで・角度 / 椅子の縁まで | 最大の傾き | RTF（歩く間の中央値） | load（前 → 後） |
|---|---|---|---|---|---|---|---|---|
| lap1 | 20:03:11 | `F around_box_1` | 合格 | 1.069 / 0.797 m | — | 0.065 | 0.738 | 2.81 → 11.57 |
| lap2 | 20:05:56 | `F around_box_1` | 合格 | 1.043 / 0.872 m | — | 0.136 | 0.894 | 10.85 → 6.33 |
| lap3 | 20:08:26 | `F around_box_1` | 合格 | 1.044 / 0.857 m | — | 0.220 | 0.695 | 6.06 → 10.85 |
| lap4 | 20:11:39 | `F around_box_1` | 合格 | 1.042 / 0.869 m | — | 0.087 | 0.760（止まり 0.628） | 10.86 → 8.07 |
| lap5 | 20:14:28 | `F around_box_1` | 合格 | 1.042 / 0.868 m | — | 0.058 | 0.878 | 8.07 → 9.66 |
| chair1 | 20:17:09 | `F near_chair_1` | 合格 | — | 1.093 m・29.6° / 0.788 m | 0.128 | 0.771 | 9.66 → 9.79 |
| chair2 | 20:18:00 | `F near_chair_1` | 合格 | — | 1.135 m・25.2° / 0.838 m | 0.064 | 0.624 | 9.79 → 13.43 |
| chair3 | 20:19:01 | `F near_chair_1` | 合格 | — | 1.084 m・29.4° / 0.782 m | 0.058 | 0.664（止まり 0.642） | 13.43 → 12.93 |
| hello1 | 20:20:00 | `F(near_chair_1 & X F hello)` | 合格 | — | 1.101 m・35.8° / 0.789 m | 0.141 | 0.639 | 12.93 → 11.44 |
| box1 | 20:21:03 | `F near_box_1`（`--target box_1`） | 合格 | — | 箱：縁まで 0.845 m・足跡に 8.5°・中心まで 1.167 m・22.8°・どの足跡にも 0.841 m | 0.130 | 0.646 | 13.01 → 15.61 |

- 一周は 5 回とも通り、3 回続けての線も満たす（5 回続けて）。巻き数の幅は 1.042〜1.069、箱の縁までの最小は 0.797 m（線は 0.6）。
- 椅子は 3 回とも、中心まで 1.084〜1.135 m（線は 1.5）、角度 25.2〜29.6°（線は 45）、縁まで 0.782〜0.838 m（線は 0.45）。挨拶つきは 35.8°。
- 「箱の前まで」は §10.8 の縁の線と、O16 の中心の線（logical のときだけ）を両方満たす。`source` は `logical`。
- 回の RTF（`objects_check.py run` の `rtf_stats`、10 秒の窓の中央値、画面あり、上のほかの組の負荷の中）：歩いている間 0.624〜0.894。
- 橋渡しの測った大きさ（設計書 §10.9「箱と椅子では測る値は変わらない（§13.6 で確かめる）」）：`household` の箱 0.524 × 0.540、椅子 0.462 × 0.477
  （どちらも `measured: true`。ソファの回 1 の `/go2/objects`、始めの位置で見たもの）。設計書 §12.4 の「Gazebo で測ったような大きさ（箱 0.53 × 0.52、
  椅子 0.5 × 0.5）」と同じくらいで、まわりのほかの物の点は入っていない。

### 新しい種類（設計書 §13.4 の logical、計画 H5）

目標は部屋の直しの後の部屋の 4 つ：`sofa_1`・`suitcase_1`（Bf から）、`traffic_cone_1`（R5 から）、`table_1`（L から。logical だけ）。
回ごとに起動し直した Gazebo で、試験の姿勢の手順（計画「The room after the room fix」）を 1 つずつ流し、`resolve --target <id> --timeout 300` を
姿勢に着く式（Bf は `F moved_forward`、R5 は 5 歩の式、L は `F turned_left`）の直前に裏で始め（pre-final review F11）、`resolve` が終わってから
目標の式を流した。どの回も `run_formula` の目標の式の終了コードは 0。

列：着いた姿勢は `resolve` の後に odometry を 1 通読んだもの。「手順」は手順の式の終了コード（立つ・回る・前に出る）と `resolve` の終了コード。
「目標の行」は目標の式の直前の `/go2/objects`。「前」は「前方に物が出た」の行の数（かっこはセンサーが途切れた回数）。

| 回 | 起動 | 着いた姿勢（x, y, 向き） | 手順 / `resolve` | 目標の行の大きさ、測った | 計画の手 | 判定 | 線の値 | 最大の傾き | 前 | RTF（歩く間） | load（前 → 後） |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sofa_near1 | 13 日 20:23:17 | (−0.453, −0.112, −166.9°) | 0・0・0 / 0 | 1.052 × 1.797、済 | 5 | 合格 | 縁 1.180 m・2.3°・どの足跡にも 0.544（suitcase_1） | 0.022 | 0（0） | 0.877 | 6.46 → 5.66 |
| sofa_near2 | 13 日 20:24:40 | (−0.484, −0.060, −173.8°) | 0・0・0 / 0 | 1.054 × 1.830、済 | 5 | 合格 | 縁 1.102 m・0.0°・どの足跡にも 0.780（suitcase_1） | 0.093 | 0（0） | 0.857 | 4.60 → 6.93 |
| sofa_lap1 | 13 日 20:25:44 | (−0.494, −0.046, −171.6°) | 0・0・0 / 0 | 1.053 × 1.808、済 | 49 | 合格 | 巻き数 0.923・縁 0.883 m | 0.052 | 1（0） | 0.670 | 7.02 → 14.35 |
| sofa_lap2 | 14 日 06:35:52 | (−0.501, −0.125, −170.7°) | 0・0・0 / 0 | 1.053 × 1.782、済 | 55 | 合格 | 巻き数 0.917・縁 0.749 m | 0.055 | 1（0） | 0.868 | 7.68 → 7.40 |
| suitcase_near1 | 06:39:52 | (−0.453, −0.057, −170.6°) | 0・0・0 / 0 | 0.495 × 0.307、済 | 2 | 合格 | 縁 0.972 m・29.6°・どの足跡にも 0.963（suitcase_1） | 0.021 | 0（0） | 0.858 | 6.55 → 7.77 |
| suitcase_near2 | 06:40:57 | (−0.485, −0.141, −167.6°) | 0・0・0 / 0 | 0.506 × 0.312、済 | 6 | 合格 | 縁 1.137 m・2.7°・どの足跡にも 1.128（suitcase_1） | 0.047 | 0（0） | 0.868 | 7.85 → 7.34 |
| suitcase_lap1 | 06:42:19 | (−0.466, −0.062, −174.0°) | 0・0・0 / 0 | 0.498 × 0.309、済 | 26 | 合格 | 巻き数 1.033・縁 0.892 m | 0.037 | 0（0） | 0.846 | 6.99 → 7.07 |
| suitcase_lap2 | 06:44:39 | (−0.471, −0.032, −169.1°) | 0・0・0 / 0 | 0.501 × 0.305、済 | 26 | 合格 | 巻き数 1.035・縁 0.896 m | 0.037 | 0（0） | 0.766 | 6.58 → 14.05 |
| cone_near1 | 06:47:14 | (−0.345, −2.757, −97.5°) | 0・0・0 / 0 | 0.489 × 0.349、済 | 1 | 合格 | 縁 0.953 m・0.0°・どの足跡にも 0.952（traffic_cone_1） | 0.040 | 0（0） | 0.606 | 14.05 → 18.76 |
| cone_near2 | 06:48:49 | (−0.310, −2.723, −96.8°) | 0・0・0 / 0 | 0.482 × 0.345、済 | 1 | 合格 | 縁 0.984 m・0.0°・どの足跡にも 0.984（traffic_cone_1） | 0.208 | 0（0） | 0.547 | 21.73 → 22.10 |
| cone_lap1 | 06:50:23 | (−0.413, −2.718, −97.6°) | **1**・0・0 / 0 | 0.479 × 0.349、済 | 28 | 合格 | 巻き数 1.103・縁 0.880 m | 0.072 | 0（0） | 0.562 | 22.15 → 21.41 |
| cone_lap2 | 06:54:03 | (−0.374, −2.755, −96.9°) | **1**・0・0 / 0 | 0.506 × 0.352、済 | 28 | 合格 | 巻き数 1.104・縁 0.881 m | 0.056 | 4（4） | 0.554 | 22.34 → 39.12 |
| table_near1 | 06:57:49 | (0.037, −0.045, 96.6°) | **1**・0 / **1** | 1.500 × 1.200、**未** | 10 | 合格 | 縁 1.233 m・3.2°・どの足跡にも 0.931（suitcase_1） | 0.035 | 1（1） | 0.679 | 37.93 → 31.65 |
| table_near2 | 07:04:13 | (0.036, 0.035, 92.5°) | 0・0 / **1** | 1.500 × 1.200、**未** | 10 | 合格 | 縁 1.160 m・14.4°・どの足跡にも 1.069（suitcase_1） | 0.063 | 0（0） | 0.563 | 29.72 → 95.41 |
| table_lap1 | 07:10:52 | (0.012, 0.024, 94.6°) | **1**・0 / 0 | 1.412 × 0.807、済 | 56 | 合格 | 巻き数 1.087・縁 0.775 m | 0.057 | 1（1） | 0.581 | 103.90 → 68.36 |
| table_lap2 | 07:17:00 | (0.034, 0.019, 96.0°) | 0・0 / **1** | 1.500 × 1.200、**未** | 54 | 合格 | 巻き数 1.081・縁 0.955 m | 0.098 | 9（9） | 0.599 | 64.58 → 75.29 |

- **スーツケース（R-H3j）と、コーン（R-H3k）の結果を分けて書く：** スーツケースは Bf から手前 2/2（縁まで 0.972・1.137 m）、周回 2/2
  （巻き数 1.033・1.035、縁まで 0.892・0.896 m）。コーンは R5 から手前 2/2（縁まで 0.953・0.984 m、どちらも足跡に 0.0°）、周回 2/2
  （巻き数 1.103・1.104、縁まで 0.880・0.881 m）。どちらも logical（正解の位置）なので、detector のときの見え方の余りの薄さ（R-H3j の方位、
  R-H3k の確からしさ）はここでは試されていない。それは H6。
- **前の物のビット（O14）：** ソファの手前 2 回は「前方に物が出た」の行 0（立たなかった）。手前の最後の縁までは 1.10〜1.18 m で、
  `final_edge` 0.50 m 未満の notes はどの手前の回にもない。周回では立った：ソファの周回 2 回とも、最初の左に回る間に 1 回ずつ（すぐ消えた。
  センサーの途切れはない。周回の道はスーツケースの 0.465〜0.477 m を通る）。lap2・lap5・cone_lap2・table_near1・table_lap1・table_lap2 の
  「前方に物が出た」は、どれもセンサーの途切れ（load の高い間の「センサー（LiDAR か /sportmodestate）が途切れています」の一時停止）と一緒に出たもの。
- **`resolve` の待ち（`listed_after_still_s`）：** ソファ・スーツケース・コーンの 12 回は `null`（標準エラー「the row was measured in the first
  /go2/objects message this tool got」）。logical では目標は B か R（姿勢に着く式の前）で論理カメラに入り、その止まりで測られるので、
  `resolve` を始めたときにはもう `measured: true`。テーブルの table_lap1 は L に着いて **0.97 秒**。ほかのテーブルの 3 回は下の「テーブルが測られない」。
  待ちの線は detector のとき（H6 M4）だけで、logical の H5 には線がない。

### テーブルが L から測られない（手順の「測られるのを待つ」）

- 4 回のうち 3 回（table_near1・table_near2・table_lap2）、`resolve` は 300 秒待って終了コード 1（`"measured": false`）。行はずっと
  1.5 × 1.2（F2 の裁定の、測る前の [L, L] と配置の大きさの軸ごとの大きいほう）。`nk` の手順は `resolve` の後に目標の式へ進むので、
  この 3 回は**計画の「measured: true を待ってから式」を満たさないまま**流した。式はその足跡で計画され、線（正解の配置の足跡で測る）は通った。
- **わけ（証拠）：** table_near2 で `resolve` が待っている間（07:06、L で止まっている）に、読むだけの使い捨てのスクリプトで
  `/utlidar/cloud` と odometry を 15 秒受け、橋渡しと同じ変換（`lidar_points_world`）・同じ止まりの門（`StillGate`）・同じ半径
  （`measure_radius`）・`GROUND_Z` で、物ごとの点を数えた。止まった 90 枚のどれでも、**テーブルの半径 1.008 m の中の点は 2 個**
  （高さ 0.95〜0.96 m、天板の縁）で、`footprint_size` の `SIZE_MIN_POINTS`（10）に届かない。同じ枚で、ほかの物は box_1 28・chair_1 20・
  suitcase_1 63・traffic_cone_1 9・sofa_1 58。L からテーブルの中心までは約 5.0〜5.1 m で、LiDAR（16 本、4° ごと、±30°）の点はこの枚では天板の縁にだけ
  当たり、脚の点はなかった。
- 測られた table_lap1（L の (0.012, 0.024, 94.6°)）では、L に着いて 0.97 秒で 1.412 × 0.807 になった。姿勢がわずかに違うと点の数が
  10 を越えたり越えなかったりする、ぎりぎりの所。Bf の回（B を通る手順）では、目標の式の前の `/go2/objects` で、テーブルは
  8 回のうち 7 回 `measured: true`（1.423〜1.519 × 0.777〜0.836。B か Bf の止まりで測ったもの。テーブルまで約 4.8 m で、
  L からとは見る向きが違う）。R5 の回ではテーブルは一覧に入らない（見ていない）。
- コードは変えていない。どうするかは controller に返す（下）。

### 着いた姿勢（R-H5a。固定は変えていない）

| 姿勢 | 回 | x | y | 向き | `tests/test_household.py` の範囲（`MEASURED_RANGES`） | 外れ |
|---|---|---|---|---|---|---|
| Bf | ソファ 4、スーツケース 4 | −0.501〜−0.453 | −0.141〜−0.032 | −174.0〜−166.9° | x −0.560〜−0.506、y −0.141〜−0.054、向き −172.4〜−166.6° | x は 8 回とも前（0.005〜0.053 m）、y は 2 回 先（−0.046・−0.032、最大 0.022 m）、向きは 2 回 回りすぎ（−173.8・−174.0、最大 1.6°） |
| R5 | コーン 4 | −0.413〜−0.310 | −2.757〜−2.718 | −97.6〜−96.8° | x −0.359〜−0.225、y −2.804〜−2.654、向き −98.0〜−95.1° | x は 2 回 先（−0.374・−0.413、最大 0.054 m） |
| L | テーブル 4 | 0.012〜0.037 | −0.045〜0.035 | 92.5〜96.6° | 範囲なし（`MEASURED_POSES["L"]` は (0.011, 0.037, 96.1°)） | — |

どの姿勢からも、目標の手前と周回は計画があり、線は通った。R-H5a は Bf の範囲を H5 の姿勢まで広げる。R5 も外れたので、同じ扱いが要るかもしれない
（controller へ）。

### 手順の「立つ」が error で終わった回（重い負荷の下）

- cone_lap1・cone_lap2・table_near1・table_lap1 の 4 回、手順の最初の `run_formula "F standing"` が「完了：立つ（伏せ → 立ち）」の後に
  「姿勢が計画と違います（計画: 立ち / 実際: 伏せ）」の error（終了コード 1）で終わった。どれも 1 分の load が 22〜104 の間。
- 橋渡しのログでは、どの回も passive → fixed down の 3.3〜4.0 秒後に fixed down → fixed stand に切り替わっていて（Go2 は立った）、
  次の式（回る）はふつうに始まり、着いた姿勢の胴の高さは 0.333〜0.335 m。目標の式はどれも立った所から始まり、線は通った。
- 線ではない手順の段の外れなので、やり直しの決まり（R-PAR は線の外れ）には当たらないと判断して、数えた。進み具合の帳面の
  bridge-stand-sync の記録（負荷の下で BalanceStand が fixed down のうちに届き、橋渡しは切り替えの知らせまで 4 を保つ）と同じ負荷の下の形で、
  Session の姿勢の確かめが切り替えより先に来たもの。コードは変えていない（controller へ）。

### 測った足跡の大きさ（logical、`/go2/objects`）

- ソファ 1.052〜1.054 × 1.782〜1.830（配置 1.044 × 1.8）。スーツケース 0.495〜0.506 × 0.305〜0.312（配置の世界の軸で 0.468 × 0.291）。
  テーブル（測れた回）1.412 × 0.807、1.465 × 0.826（配置 1.5 × 0.8）。
- **コーン 0.479〜0.506 × 0.345〜0.352（配置 0.503 × 0.503）：y の幅がモデルの底より 0.15 m 小さい。** R5 からは −y を向いて見るので、
  LiDAR の点はコーンの +y の面の、底より上（円すいで細くなる所）に当たる。`footprint_size` は分かっている中心から 2·max|dy| で測るので、
  見えた高さの幅になる。設計書 O12 の前提（足跡は本当の物より小さくならない）は、細くなる形では logical の測った値で崩れる。
  線は正解の配置の足跡で測っていて、コーンのどの回もどの足跡にも 0.880 m 以上。controller へ。

### 眠り

- **13 日 20:29:55 に Mac が蓋を閉じて眠った**（`pmset -g log`：`Entering Sleep state due to 'Clamshell Sleep'`、電池 43%）。ソファの周回の 2 回目を
  20:29:45 に起動した 10 秒後で、その後は短く起きる間（`DarkWake`、数秒から十数秒、約 15 分ごと）だけが続いた。
- 眠りをまたいだ sofa_lap2 は数えない（R-H5b）：「立つ」は終わったが Session が「姿勢が計画と違います（計画: 立ち / 実際: 伏せ）」で error にし
  （橋渡しが fixed down → fixed stand を見たのは起動からシミュレーションの時間で約 228 秒後）、回る間に「センサーが途切れています」で 2 回止まり、
  `resolve` は 21:27、`F moved_forward` は 21:29〜21:47、目標の式は 21:51 に始まって終わらなかった。
- 22:25 の短い起きる間に、続きの回を止め、こちらの container（`go2oo-gazebo-1`、`go2oo-app-run-e367c87c07c1`）を消して `G down` した。
  ふつうに起きるのを 3 時間（01:33 まで）待っても起きなかったので、そこまでの記録を `affdbd7` に入れた。
- その後（controller の指示、R-H5b）、`ioreg` の `AppleClamshellState` が No か `pmset -g batt` が AC になるのを、1 回 `sleep 590` ずつ待った。
  **14 日 06:27:43 に蓋が開いて起きた**（`pmset -g log`：`Wake from Deep Idle … due to … lid`）。門は次の確かめの 06:35:15 に開いた（No、電池 33%）。06:35:52 から、眠りで切れた sofa_lap2 を同じ手順で流し直し、続きを流した。
  14 日の回の間に眠りはなかった（どの回の間も 5 秒ごとに門と眠りの記録を見た）。
- 眠りの前の 13 回は、どれも眠りの前に終わっている（最後の sofa_lap1 は 20:29:42 に Gazebo を止める所まで）。

### M0：logical の `household` の RTF（最後に。R-PAR、C4）

起動し直した Gazebo（logical、Go2 は伏せたまま、式なし）で、立ち上がって 20 秒の後に
`G run --rm --no-deps -T app python3 gazebo/tools/objects_check.py rtf --seconds 75`。画面ありは `G --profile gazebo up -d --no-build gazebo`、
画面なしは計画の Wave 3 の形（`OBJECTS_SOURCE=logical G run -d --no-deps --service-ports --name go2oo-gazebo gazebo ros2 launch
/work/gazebo/launch/go2_gazebo.launch.py gui:=false`、止めるのは `docker rm -f go2oo-gazebo` と `G down`）。
どの回も、始める前に、ほかの組の `pytest` がなく 1 分の load が 4 未満になるのを 60 秒ごとに待った（R-PAR。M0 は線がないので、60 分で
load が下がらなければ `pytest` がないことだけで始める決まりにしたが、そうなった回はない）。古い記録は使っていない（C4）。

| 回 | 窓の始め（日本時間） | 画面 | RTF（75 秒） | 始めの load（1・5・15 分） | 終わりの load | 窓の間のほかの container（`docker stats`、始め / 終わり） | `go2oo-gazebo`（CPU、メモリ） | 電源 |
|---|---|---|---|---|---|---|---|---|
| m0_on1 | 07:51:59 | あり | **0.79** | 3.70 14.49 32.06 | 7.42 12.70 29.32 | `go2-app-run-…` 2 つ、各 100 % / なし | 426 %、1.253 GiB | AC、69 → 71 % |
| m0_on2 | 08:00:47 | あり | **0.85** | 3.81 6.41 19.60 | 6.42 6.52 18.11 | なし / なし | 431 %、1.26 GiB | AC、78 → 79 % |
| m0_on3 | 08:14:37 | あり | **0.82** | 3.84 5.37 10.86 | 9.30 6.61 10.69 | `go2-app-run-…` 100 % / 別の `go2-app-run-…` 102 % | 411 %、1.261 GiB | AC、88 → 89 % |
| m0_off1 | 08:22:26 | なし | **0.87** | 2.00 3.76 7.91 | 8.95 5.97 8.30 | なし / `go2-app-run-…` 211 % | 237 %、651 MiB | AC、93 → 94 % |
| m0_off2 | 08:28:14 | なし | **0.93** | 3.40 4.54 7.08 | 11.67 7.41 7.91 | なし / `go2-app-run-…` 104 % | 257 %、639 MiB | AC、96 → 97 % |
| m0_off3 | 08:40:10 | なし | **0.82** | 3.45 11.37 12.64 | 11.18 11.28 12.42 | なし / `go2-app-run-…` 403 % | 251 %、638 MiB | AC、99 % |

- 画面あり 0.79〜0.85（中央値 0.82）、画面なし 0.82〜0.93（中央値 0.87）。始めはどれも 1 分の load が 4 未満で `pytest` はなかったが、
  ほかの組の container は 6 回のうち 5 回、始めか終わりに動いていて（`pytest` でないもの、100〜403 %）、終わりの load は 6.4〜11.7。いちばん低い画面なしの 0.82（m0_off3）は、終わりに 403 % の container があった回。
- 参考（線ではない）：§2 の組と新しい種類の回の `rtf_stats`（画面あり、歩いている間、ほかの組の重い負荷の中）は 0.547〜0.894。
  どの回も線を満たしたので、D7 の判断の「その RTF で一周と手前が通る」は logical では満たす。

### controller に返すこと（decisions_needed）

1. **テーブルが L から測られない（手順）。** 4 回のうち 3 回、L で `measured: true` にならず（LiDAR の点 2 個 < `SIZE_MIN_POINTS` 10、
   天板の縁だけ）、`resolve` が 300 秒で切れたまま式を流した。線はどれも通った（測る前の 1.5 × 1.2 の足跡で計画）。決めること：この 3 回を
   §13.4 の logical の 2/2 に数えてよいか（このまま数えると、テーブルは手前 2/2・周回 2/2）。数えないなら、テーブルの試験の姿勢を
   近くする（例えば L に 1 歩か 2 歩足す。§12.1 と周回の計画の確かめが要る）か、テーブルの測り方（半径・点の数）を変える設計の話になる。
   H5 ではコードも姿勢も変えていない。
2. **R5 も着いた姿勢が固定の範囲の外。** R-H5a は Bf を広げる。R5 のコーンの 2 回は x −0.374・−0.413（範囲 −0.359 まで、最大 0.054 m 先）。
   R-H3k (1) と同じく R5 の範囲も広げて固定し直すか。Bf の H5 の範囲は x −0.501〜−0.453、y −0.141〜−0.032、向き −174.0〜−166.9°
   （R-H5a に書かれた値より、y は −0.032 まで、向きは −174.0 まで広い）。
3. **重い負荷の下で、手順の「立つ」が error（4 回）。** Session の姿勢の確かめが、橋渡しの fixed down → fixed stand の知らせより先に来た形。
   Go2 は立っていて線は通ったので数えた。H6 は待ちの時間の線を測るので、H6 の間は R-PAR のとおりほかの組を軽くするか、手順の「立つ」の
   error を見たらその回をやり直すか。
4. **コーンの測った足跡が y でモデルの底より 0.15 m 小さい（0.345〜0.352 / 0.503）。** 細くなる形では O12 の前提が logical の測った値で
   崩れる。線は正解の足跡で通った。Task 4.1 に書く限りとしてよいか。

### 動かした container

- `G` の compose project `go2oo` の `go2oo-gazebo-1`（画面あり）と `go2oo-gazebo`（画面なし、名前を付けた `G run -d`）、`G run --rm … app`
  の `go2oo-app-run-…`（`run_formula`・`objects_check`・odometry を 1 通読むもの・読むだけの LiDAR の数え）だけ。どれも回ごとに `G down`
  （画面なしは `docker rm -f go2oo-gazebo` も）で止めた。終わった後に残った `go2oo` の container はない。
- ほかの組の container（`go2-app-run-…`、`t3cmp-…`、`t3-after`）には触れていない。利用者の `go2` の project（`go2-gazebo-1`・`go2-app-1`）は
  どの時も動いていなかった。

## H6：detector の Gazebo の測り（A7、2026-11-18）

- 枝：`objects2/h6-detector`（`feat/open-objects` の先 `eecbc97` から。H5 の merge の後）。worktree `~/work/physicalAI/.go2-worktrees/oo-h6`。
  ROS の domain は 66（`G` の回は compose project `go2oo` の自分の network で、domain は使わない）。
- 種類の見張りは R-PF5c の結論のとおり `97da92a` のもの（`guard-known-limits.md`：v1 は `97da92a` の見張りで、`objects2/guard3` は merge しない）。
  `feat/open-objects` の先には guard3 が入っていないので、この枝のコードがそれ。
- イメージ：Gazebo `go2-voice-ltl-gazebo:patch-0005-fuel2`（一覧の ID `c40a28c22024`）、検出器 `go2-voice-ltl-detector:yoloe-11s`（`016348e47306`）、
  app `go2-voice-ltl:dev`（`a35b9bb8c8d3`）。どれも作り直していない。`GO2_LAYOUT=household`（部屋の直しの後の部屋：box_1・chair_1・
  suitcase_1・traffic_cone_1・sofa_1・table_1）。橋渡しはどの回も「物 6 個」を置いた。
- detector の回は `OBJECTS_SOURCE=detector DETECTOR=<oracle|yoloe> G --profile gazebo --profile detector up -d --no-build gazebo detector`
  （画面あり）か、Wave 3 の画面なしの形（`G run -d --no-deps --service-ports --name go2oo-gazebo gazebo … gui:=false` と
  `G --profile detector up -d --no-build detector`）。**回ごとに Gazebo と検出器を一緒に止めて起動し直した**（`G … down`、画面なしは
  `docker rm -f go2oo-gazebo` も）。
- 試験の姿勢は `detector_check.PREVIEW_TARGETS` のとおり：ソファとスーツケースは Bf（`F standing`、`F(turned_left & X F turned_left)`、
  `F moved_forward`。R-H3a）、コーンは R5（`F standing`、`F turned_right`、5 歩の式）。テーブルは logical だけ（R-H3h・R-H3i）なので H6 の
  detector の目標ではない。
- コード・テスト・固定は変えていない。変えたのはこの記録だけ。
- 順は計画の H6 のとおり：M0 前 → M0 detector 止まり → M3 → M4 → M0 detector 歩く（画面なし）→ 声の段 → M5 → M0 後 → §2 の組 2。
  R-H5d の logical のコーンの流し直し（手前 ×2・周回 ×2）は logical の組の最後に足した。間に、M3 の oracle の `covers` のわけを探す回（下）を 2 つ足した。

### 結論

**H6 の線は、M3 の oracle の `covers` をのぞいてすべて満たした。** M4 は 3 種類とも手前 3/3・周回 3/3（18/18。ソファの周回 1 回は負荷の
決まりで流し直したもの）、待ちと推論の時間の線も満たす。声の段は 6/6（ソファの周回 1 回は同じく流し直し）。§2 の組 2 は 10/10。
**外れたのは M3 の「oracle はどの組でも `covers` が真」**：Bf のスーツケース（数 mm）とソファ（奥 0.22 m）、R5 のコーン（手前の面 1.5〜14 mm）。
わけは下の「M3：oracle の `covers` が偽になるわけ」（証拠つき）。コードは変えず、controller に返す。

| 線（計画 H6、設計書 §13.3〜§13.6・§10.8） | しきい値 | 結果 |
|---|---|---|
| M3 oracle：位置・大きさの線（spawn 伏せ・立ち、Bf、R5） | 目標 0.15 m、どれも 0.35 m、中央値 0.25 m、大きさ 0.4 m、名前 0.8、recall 0.6、誤り 1 以下 | **満たす**（目標の位置 0.024〜0.079 m、大きさ 0.180 m 以下、名前 1.0、recall 1.0、誤り 0）。spawn は範囲に `detectable` がなく recall は n/a |
| M3 oracle：どの組でも `covers` が真（見えた所だけ。C2） | 真 | **外れ**：Bf のスーツケース（`filled` 偽）と R5 のコーン（`filled` 真、手前の面）が偽。わけを探した回では Bf のソファも偽（奥が 0.22 m 足りず `filled` 偽） |
| M3 yoloe：同じ線（Bf、R5） | 同じ | **満たす**（Bf：ソファ 0.136 m・スーツケース 0.024 m、R5：コーン 0.012 m、名前 1.0、recall 1.0、誤り 0）。`covers` は線にない（ソファ・コーンが偽） |
| M4 ソファ（Bf）`F near_<id>` ×3 | 縁まで 1.5 m 以内・足跡に 45° 以内・どの足跡にも 0.30 m 以上 | **3/3**（縁 1.077〜1.083 m、0.0°、0.663〜0.729 m） |
| M4 ソファ（Bf）`F around_<id>` ×3 | 巻き数 0.9 以上 1.5 未満・縁まで 0.6 m 以上 | **3/3**（巻き数 1.143・0.934・0.920、縁 0.749〜0.883 m）。lap2 は前の物で止まり（終了コード 1、load 6.1）、R-PAR で流し直して合格 |
| **M4 スーツケース（Bf、R-H3j）**手前 ×3 | 手前の線 | **3/3**（縁 0.880〜1.083 m、0.0〜28.8°、0.864〜1.076 m） |
| **M4 スーツケース（Bf、R-H3j）**周回 ×3 | 周回の線 | **3/3**（巻き数 1.032〜1.037、縁 0.930〜0.972 m） |
| **M4 コーン（R5、R-H3k）**手前 ×3 | 手前の線 | **3/3**（縁 0.973〜1.053 m、0.0〜11.4°、0.971〜1.049 m） |
| **M4 コーン（R5、R-H3k）**周回 ×3 | 周回の線 | **3/3**（巻き数 0.928〜1.130、縁 0.812〜0.920 m） |
| M4：止まってから一覧に出るまで（`listed_after_still_s`） | 5 秒以内 | **満たす**：測れた 16 回で 1.08〜4.42 秒。測れなかった 2 回（ソファ）は Bf に止まったときにはもう一覧にあった（下） |
| M4：推論の時間（状態の `latency_ms_p50` / `latency_ms_p90`） | 600 ms 以内 / 800 ms 以内 | **満たす**：p50 360〜395 ms、p90 379〜525 ms（19 回の目標の式の前と後） |
| M4：転ばない | 傾き 1.0 rad 未満 | 19 回とも（最大 0.197） |
| M0 detector 歩く・画面なし（ソファ 手前 1・周回 1） | M4 の線 | **2/2**（縁 1.041 m・0.0°・0.671 m、巻き数 0.911・縁 0.883 m）。歩く RTF 0.91・0.938 |
| 声の段（実際の Claude、操作画面の文字の欄）：〈名前〉まで歩いて ×3 種類 | M4 の線、各 1/1 | **3/3**（式はどれも `F near_<id>`、1 回目で合格） |
| 声の段：〈名前〉の周りを回って ×3 種類 | M4 の線、各 1/1 | **3/3**（式はどれも `F around_<id>`）。ソファの 1 回目は前の物で止まり（load 5.1）、R-PAR で流し直して合格 |
| M5（D9）：立った Go2 の前の箱が `box` として一覧に入るか | 記録だけ | 入らなかった（40 秒、推論 21 回）。段ボールの絵が読めていない（下） |
| M0：RTF | 記録だけ（D7） | logical 前 画面あり 0.85〜0.89・なし 0.99〜1.0；detector 止まり あり 0.84〜0.87・なし 0.99；detector 歩く あり 0.811〜0.873（M4）・なし 0.91〜0.938；logical 後 あり 0.84〜0.90・なし 0.98〜1.00 |
| §2 の組 2（logical、設計書 §13.6） | §2 の線 | **10/10**：一周 5/5（巻き数 1.037〜1.044、箱の縁 0.797 m 以上）、椅子 3/3（1.063〜1.091 m・29.3〜29.7°、縁 0.761 m 以上）、挨拶つき 1/1、「箱の前まで」1/1（縁 0.848 m・8.2°、中心 1.169 m・22.5°） |
| R-H5d：logical のコーン（R5）手前 ×2・周回 ×2 | 手前・周回の線 | **4/4**（手前の縁 1.052・1.067 m、0.0・3.7°；周回の巻き数 1.099・1.101、縁 0.896・0.951 m）。コーンの行は 0.503 × 0.503・測った |

- 負荷（R-PAR）：こちらの画面ありの Gazebo だけで 1 分の load は 5〜7 になる（Gazebo 約 4.3 vCPU）。だから、どの回もほとんど 4 を超えていた
  （記録した 190 の時点のうち 155）。外れた線は 2 回（M4 のソファの周回 lap2、声の段のソファの周回）で、どちらも load 4 超えなので、load が 4 未満で
  `pgrep -f pytest` が 0 になるのを待って流し直し、流し直しは合格した。外れのわけ（前の物で止まった）は負荷ではなかったので、下に書く。
- R-H5e の「姿勢が計画と違います（計画: 立ち / 実際: 伏せ）」は、どの回の「立つ」でも出なかった（やり直した回はない）。
- 眠りはなかった（どの時点も AC、電池 100 %、`lowpowermode` 0）。

### 命令

計画の「Commands」の `G`・`TA`（`WT` はこの worktree、`DOMAIN=66`）と、声の段の `GA`（下）。回ごとに：

```bash
docker ps --format '{{.Names}}'                       # D11：go2-gazebo-1 がないこと（どの回も、ない）
uptime; pgrep -f pytest | wc -l; docker stats --no-stream --format '{{.Name}} {{.CPUPerc}} {{.MemUsage}}'; pmset -g batt; pmset -g custom | grep -i lowpower
OBJECTS_SOURCE=detector DETECTOR=yoloe G --profile gazebo --profile detector up -d --no-build gazebo detector
# ログに「Configured and activated unitree_guide_controller」「Gazebo の Go2 の橋渡しを起動しました」「detector yoloe-11s-seg loaded」が出るまで待つ（7〜9 秒）
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F standing"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py resolve --target sofa_1 --timeout 420 &   # 回る式の前（下の「待ちの測り方」）
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F(turned_left & X F turned_left)"
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py resolve --target sofa_1 --timeout 300 &   # 姿勢に着く式の前（F11、計画のとおり）
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 -m voice_ltl.run_formula "F moved_forward"
wait
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py objects
G run --rm --no-deps -T -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near --target sofa_1 "F near_<resolve の id>"
G --profile gazebo --profile detector down --remove-orphans
```

- 着いた姿勢（odometry 1 通）と検出器の状態（`/go2/detector_status` の最後の 1 通）は、作業用のフォルダの読むだけの使い捨てのスクリプトを
  `G run --rm --no-deps -T -v <作業用>:/h6 app python3 /h6/…` で読んだ。同じ形で、止まりの変わり目（`bridge_core.StillGate`、`_Feed` と同じ）・
  `/go2/objects` の行・状態の推論の数を時刻つきで書き出す記録係を、回る式の前から目標の式の前まで流した（M4 のソファの周回 1 回目から。
  「待ちの測り方」）。M5 の箱は、操作画面の「前に箱を置く」と同じ `{"cmd": "front"}` を `/sim/obstacles` に 1 通出して置いた。
- 回の前後の `docker stats`・`pmset`・`uptime`、`resolve`・`objects`・`run` の出力、Gazebo・検出器・app のログは、worktree の外の作業用のフォルダに
  残した（git に入れていない）。

### D11 の門、ほかの container、電源、load

- `go2-gazebo-1`（利用者の Gazebo）はどの回の前にもなかった（待たなかった）。こちらのほかの重い段もなかった。利用者の project `go2` には触れていない。
- ほかの組の container は 190 の時点のうち 9 つで見た：`stairs-tests`（M5 の前、M0 後の on1・on2・off1 の窓、声の段のコーンの周回の後、92〜113 %）、
  `stairs-gz` と `stairs-run`（m0a_on3 の前、144 % と 11 %。利用者の `go2-gazebo-1` ではない。load の門で待った）、`oo-perc-final-lap`（m0b_on3 の前、99 %）。
  `pgrep -f pytest` は 190 の時点のうち 8 回だけ 2〜3（M5 の前、M0 後の on1・on2・off1、声の段のコーンの周回の後）、ほかは 0。15:15 に一度 5 を見た
  （M0 前の始めの門で待った）。
- 1 分の load：1.79〜13.9。こちらの画面ありの Gazebo が動いている間は 5〜7（`go2oo-gazebo-1` 399〜457 %）、画面なしは 4〜5（`go2oo-gazebo` 161〜230 %）。
  13.9 は声の段のコーンの周回の前（`stairs-tests` の後）。
- 電源：どの時点も AC、電池 100 %（charged）、`lowpowermode` 0。眠りはなかった（15:08〜18:07）。
- M0（RTF）の回は、始める前に、1 分の load が 4 未満で `pytest` がないのを 60 秒ごとに待った（R-PAR。60 分を超えた回はない）。

### M0 前：logical の `household` の RTF（D11 の基準）

起動し直した Gazebo（logical、Go2 は伏せたまま、式なし）で、立ち上がって 20 秒の後に `objects_check.py rtf --seconds 75`。画面ありは
`OBJECTS_SOURCE=logical G --profile gazebo up -d --no-build gazebo`、画面なしは Wave 3 の形。ほかの組の container は窓の始めにも終わりにもなかった。

| 回 | 窓の始め | 画面 | RTF（75 秒） | 始めの load（1・5・15 分） | 終わりの load | `go2oo-gazebo`（終わり、CPU・メモリ） |
|---|---|---|---|---|---|---|
| m0b_on1 | 15:16:48 | あり | **0.87** | 3.62 3.23 3.22 | 6.24 4.24 3.61 | 440 %、1.247 GiB |
| m0b_on2 | 15:20:36 | あり | **0.85** | 4.16 3.84 3.53 | 5.89 4.43 3.77 | 432 %、1.242 GiB |
| m0b_on3 | 15:23:24 | あり | **0.89** | 4.91 4.32 3.78 | 5.63 4.73 3.99 | 431 %、1.241 GiB |
| m0b_off1 | 15:26:11 | なし | **1.00** | 3.71 4.19 3.84 | 4.49 4.37 3.94 | 166 %、638 MiB |
| m0b_off2 | 15:28:58 | なし | **0.99** | 3.50 3.96 3.82 | 4.93 4.31 3.96 | 196 %、642 MiB |
| m0b_off3 | 15:32:45 | なし | **1.00** | 3.85 4.16 3.96 | 4.51 4.22 3.99 | 203 %、639 MiB |

- 画面あり 0.85〜0.89（中央値 0.87）、画面なし 0.99〜1.00（中央値 1.00）。H5 の M0（あり 0.79〜0.85、なし 0.82〜0.93）より高いのは、
  ほかの組の container がなかったため（H5 は 6 回のうち 5 回、100〜403 % の container があった）。「窓の始め」の load はこちらの Gazebo が立ち上がった後。

### M0 detector、止まり（`DETECTOR=yoloe`）

起動し直した detector の組で `run_formula "F turned_right"`（伏せ → 立つ → 右に回る。姿勢 R：コーンが 4.4 m 先に見え、検出器は 3 秒ごとに推論し続ける。
計画の lamp_1 は部屋の直しで無くなった）、その後すぐに `rtf --seconds 75`（Go2 は立ったまま）。状態は窓の後の 1 通。

| 回 | 窓の始め | 画面 | RTF | `latency_ms_p50` / `p90` | 推論の数 | 始めの load | 終わりの load | 終わりの CPU（Gazebo / 検出器） | 着いた向き |
|---|---|---|---|---|---|---|---|---|---|
| m0s_on1 | 15:38:30 | あり | **0.87** | 393.6 / 399.0 | 32 | 3.93 | 5.35 | 444 % / 31 %、565 MiB | −97.0° |
| m0s_on2 | 15:41:18 | あり | **0.86** | 391.4 / 412.8 | 33 | 3.99 | 6.50 | 437 % / 5 %、531 MiB | −101.0° |
| m0s_on3 | 15:44:09 | あり | **0.84** | 389.9 / 428.1 | 30 | 4.27 | 6.25 | 438 % / 42 %、555 MiB | −98.4° |
| m0s_off1 | 15:46:58 | なし | **0.99** | 367.2 / 391.1 | 32 | 4.85 | 4.97 | 203 % / 3 %、529 MiB | −98.2° |
| m0s_off2 | 15:49:45 | なし | **0.99** | 362.4 / 367.5 | 33 | 4.12 | 4.92 | 186 % / 3 %、533 MiB | −98.2° |
| m0s_off3 | 15:53:32 | なし | **0.99** | 365.6 / 372.6 | 32 | 2.98 | 4.02 | 213 % / 39 %、557 MiB | −98.4° |

- 画面あり 0.84〜0.87（中央値 0.86。M0 前の 0.87 より 0.01）、画面なし 0.99（M0 前の 1.00 より 0.01）。止まって推論し続ける検出器の損は 0.01 くらい
  （設計書 §11.12 の見込み 0.025〜0.03 より小さい）。推論は 1 スレッドで p50 362〜394 ms、p90 368〜428 ms（§13.5 の 600 / 800 ms の内）。
  検出器の CPU は `docker stats` の 1 回の読みで 3〜44 %（推論の合間か最中かで変わる）。

### M3：検出の正しさ（設計書 §13.3、`objects_check.py accuracy`）

画面なし（H3 の撮り方と同じ。RTF は正しさに関わらない）、姿勢ごとに起動し直した組。spawn は伏せのまま 1 回、`F standing` の後にもう 1 回
（同じ組。H3 と同じ）。Bf は `--target sofa_1 suitcase_1`、R5 は `--target traffic_cone_1`。`accuracy` は止まって推論が 3 回になるのを待つ
（2〜4.5 秒で揃った）。B・L・R は部屋の直しの後は detector の試験の姿勢でないので測っていない。

| 検出器 | 姿勢（x, y, 向き） | 組（正解 → 名前、位置の誤差、大きさの誤差、`covers`、`filled`、補った奥行きの足りない分） | recall | 名前 | 誤り | 判定（線） |
|---|---|---|---|---|---|---|
| oracle | spawn 伏せ (0.004, 0.002, −1.5°) | box_1 → box 0.007・0.110・真・偽；chair_1 → chair 0.006・0.116・真・真 | n/a | 1.0 | 0 | 位置・大きさ・名前・誤りは満たす。範囲に `detectable` なし |
| oracle | spawn 立ち (0.045, 0.047, −0.3°) | box_1 0.019・0.153・真；chair_1 0.007・0.119・真 | n/a | 1.0 | 0 | 同じ |
| oracle | Bf (−0.529, −0.029, −173.6°) | box_1 0.028・0.180・真；chair_1 0.004・0.104・真；**sofa_1 0.079**・0.094・真・`filled` 真・0.098；**suitcase_1 0.037**・0.095・**偽**・`filled` 偽 | 1.0 | 1.0 | 0 | 線は満たす、**`covers` 偽 1 つ** |
| oracle | R5 (−0.435, −2.715, −98.6°) | box_1 0.029・0.180・真；chair_1 0.004・0.105・真；suitcase_1 0.019・0.095・真；**traffic_cone_1 0.024**・0.098・**偽**・`filled` 真・0.0 | 1.0 | 1.0 | 0 | 線は満たす、**`covers` 偽 1 つ** |
| yoloe | spawn 伏せ (0.004, 0.001, −1.5°) | chair_1 → chair 0.006・0.109・真 | n/a | 1.0 | 0 | 満たす（範囲に `detectable` なし）。box_1 は見つからない |
| yoloe | spawn 立ち (0.039, 0.049, −1.3°) | chair_1 → chair 0.008・0.124・真 | n/a | 1.0 | 0 | 同じ |
| yoloe | Bf (−0.522, −0.066, −168.8°) | chair_1 0.004・0.103；**sofa_1 → sofa 0.136**・0.142・偽・`filled` 偽；**suitcase_1 → suitcase 0.024**・0.082・真 | 1.0 | 1.0 | 0 | **満たす**（ソファの位置は 0.15 m の線に 0.014 m） |
| yoloe | R5 (−0.382, −2.658, −97.5°) | chair_1 0.004・0.103；suitcase_1 0.019・0.116・真；**traffic_cone_1 → traffic_cone 0.012**・0.050・偽・`filled` 真・0.023 | 1.0 | 1.0 | 0 | **満たす** |

- oracle の `ids_retired` はどの姿勢も 0、`notes` は空。yoloe は Bf で `ids_retired` 1（B で `bench_1` と呼んだソファを、Bf で sofa に名前を変えて
  `sofa_1` にした。状態の `ids_issued` は bench・chair・sofa・suitcase 各 1）、ほかは 0。`notes` はどれも空。推論は yoloe で p50 352〜357 ms、
  p90 377〜390 ms。
- `fill_short_m`（補った奥行きの足りない分、C2）：oracle の Bf のソファ 0.098 m、yoloe の R5 のコーン 0.023 m。ほかは 0。
- **R-H3k (2)（姿勢ごとの目標の正しさ）：** yoloe でソファ 0.136 m・スーツケース 0.024 m（Bf）、コーン 0.012 m（R5）。どれも名前は正しい。
- **R-H3k (3)（R5 のコーンの確からしさ）：** `/go2/objects` のコーンの行の `confidence`（最後の 10 回までの当たりの中央値。R から見た当たりも入る）は、
  M3 で 0.539、M4 の 6 回で 0.425〜0.708（下の表）。どれも confirm 0.35 より上。
- **Q4（box_1・chair_1 が detector のときに見つかったか）：** chair_1 は M3 の yoloe のどの姿勢でも、M4 の 18 回と画面なしの 2 回でもいつも
  `chair_1` として一覧にあった（spawn の伏せで見えたものが残る）。box_1（ただの四角）は M3 で一度も見つからず、M4 と画面なしの 21 回のうち
  1 回（m0w_sofa_lap、`confidence` 0.35）だけ `box_1` として一覧にあった。

### M3：oracle の `covers` が偽になるわけ（証拠）

oracle（名前は正解から、形と追跡は本物）なので、`covers` が偽なら形か追跡の誤り（§13.3）。同じ姿勢を起動し直した組でもう一度流し、
`/go2/objects` の 1 通ごとに `objects_check.covers()` を同じ組み方で計算する読むだけの使い捨てのスクリプトを 40 秒流した（あわせて `accuracy` を 3 回）。
「足りない分」は正解の足跡の辺から検出の辺までで、`COVER_TOL` は 1 mm。

| 回 | 物 | `filled` | 足りない辺 | 足りない分 | `accuracy` の `covers`（3 回） |
|---|---|---|---|---|---|
| M3 の 1 回目（Bf） | suitcase_1 | 偽 | 奥（−x、Go2 から遠い面） | 1 mm を超えた（`objects` の 1 通では 0 mm。足りない時と足りる時がある） | 偽（1 回だけ測った） |
| 探す回（Bf (−0.500, −0.055, −168.4°)） | suitcase_1 | 偽 | 奥（−x） | 9 通のうち最大 0.5 mm（線の内） | 真・真・真 |
| 探す回（Bf） | **sofa_1** | 最初の 1 通だけ真、後は偽 | **奥（−x）** | **0.218〜0.220 m** | **偽・偽・偽**（位置の誤差 0.138 m） |
| M3 の 1 回目（R5） | traffic_cone_1 | 真 | 手前（+y、Go2 に向いた面） | 1.5 mm | 偽 |
| 探す回（R5 (−0.359, −2.694, −97.6°)） | traffic_cone_1 | 真 | 手前（+y） | **13〜14 mm**（奥は 17〜20 mm 足りないが、`filled` なので C2 の補った分） | 偽・偽・偽 |

- **ソファ：** Bf からは −x を向いてソファの長い面を見る。LiDAR の点はソファの手前（座面と背もたれの前の面）に当たり、背もたれの後ろの面
  （正解の足跡の奥の辺 x −5.102）には届かない。測った奥行きは 0.88 m（縁の余りを入れて）で、単語帳の大きさ [1.8, 0.9] の短い辺 0.9 に
  遠い側の余りの内で届くので、形の決まり（R-G1 (3)：観た奥行きが事前の奥行きに届けば観たとみなす）は奥を「観た」とし、`filled` を偽にして
  余りを足さない。ところがモデルの奥行きは 1.044 m（単語帳の 0.9 より深い）なので、足跡は奥が 0.22 m 足りない。C2 は「単語帳の大きさを部屋の
  モデルに合わせない」なので、ソファのように**本当の物が単語帳の大きさより深いと、`filled` 偽のまま奥が欠ける**。同じ組の 1 通目は `filled` 真
  （M3 の 1 回目もそう、0.098 m）なので、推論ごとに観た奥行きが 0.9 の前後で揺れ、`filled` が入れ替わる。O12 の前提（足跡は本当の物より小さく
  ならない）が、この形で崩れる。yoloe の M3（Bf）でもソファは `covers` 偽・`filled` 偽で、位置の誤差 0.136 m の多くはこの奥の欠け（中心が Go2 の側へ
  約 0.11 m 寄る）。
- **コーン：** R5 からは −y を向いて見る。コーンは上に細くなる形で、LiDAR の帯は底（0.503 × 0.503 の四角）より上の細い所に当たるので、手前の面が
  底の辺より内側に出る（1.5〜14 mm）。H5 の R-H5d（logical で測った足跡が底より細い）と同じわけで、R-H5d は detector と実機ではこれを限りとして
  Task 4.1 に書くとしている。0.05 m の間引きの段（R-PF3）より小さい。
- **スーツケース：** 奥の面（Go2 から遠い面）の辺が、正解の辺とほぼ同じ所（0〜1.5 mm）に出る。四角い物の奥を観たとみなすときの余り（EDGE_PAD）が
  ちょうど正解の辺に重なる所で、1 mm の `COVER_TOL` をまたいで真と偽が入れ替わる。間引きの段より小さい。
- どの回も oracle の位置・大きさの線は満たした（ソファの探す回でも位置 0.138 m ≤ 0.15 m、大きさ 0.164 m ≤ 0.4 m）。手前と周回の線は正解の足跡で
  測り、M4 ではどれも通った（ソファの周回の縁まで 0.749 m 以上）。
- コードは変えていない。決めることは「controller に返すこと」。

### M4：式の段（設計書 §13.4・§13.5、`DETECTOR=yoloe`、画面あり）

回ごとに起動し直した組で、試験の姿勢の手順、`resolve`、目標の式（`objects_check.py run --expect near|lap --target <正解の id> "F near|around_<resolve の id>"`）。
どの回も検出器の id は正解と同じ番号（`sofa_1`・`suitcase_1`・`traffic_cone_1`）になった。手順の `run_formula` の終了コードはどれも 0
（立つ・回る・前に出る）、`resolve` もどれも 0。目標の式の最初の動作は、立った所からなので「立つ」でない（裁定 C1）。

列：「待ち」は回る式の前から始めた `resolve` の `listed_after_still_s`（秒。下の「待ちの測り方」）、「状態」は目標の式の前と後の
`latency_ms_p50` / `p90`（ms）、「行」は目標の式の前の `/go2/objects` の目標の行（大きさ、`confidence`）、「前」は「前方に物が出た」の数、
RTF は `rtf_stats` の歩く間の中央値（10 秒の窓。コーンの手前は 1 手で窓がない）。

| 回 | 起動 | 着いた姿勢 | 待ち | 状態（前 / 後） | 判定 | 線の値 | 傾き | 手 | 前 | RTF（歩く） | 行 | load（前 → 式の始め → 後） |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| sofa_near1 | 15:59:19 | (−0.517, −0.103, −170.8°) | null | 375 / 401、377 / 388 | 合格 | 縁 1.083 m・0.0°・どの足跡にも 0.667（suitcase_1） | 0.124 | 5 | 0 | 0.872 | 1.002 × 1.849、0.757 | 6.13 → 6.16 → 6.08 |
| sofa_near2 | 16:00:34 | (−0.525, −0.049, −171.2°) | 2.96 | 367 / 437、364 / 379 | 合格 | 縁 1.077・0.0°・0.729 | 0.031 | 5 | 0 | 0.860 | 1.002 × 1.869、0.856 | 6.07 → 5.98 → 5.91 |
| sofa_near3 | 16:01:48 | (−0.521, −0.105, −171.3°) | 2.53 | 395 / 454、372 / 404 | 合格 | 縁 1.081・0.0°・0.663 | 0.024 | 5 | 0 | 0.857 | 1.000 × 1.849、0.778 | 5.52 → 10.14 → 9.88 |
| sofa_lap1 | 16:03:05 | (−0.501, −0.030, −173.6°) | 2.58 | 385 / 428、383 / 428 | 合格 | 巻き数 1.143・縁 0.749 | 0.107 | 52 | 1 | 0.873 | 1.000 × 1.858、0.825 | 9.73 → 7.63 → 6.66 |
| sofa_lap2（数えない） | 16:06:53 | (−0.501, −0.130, −165.5°) | 2.80 | 366 / 400、364 / 393 | **不合格**（終了コード 1） | 巻き数 0.930・縁 0.872 | 0.037 | 55 | 1 | 0.862 | 1.002 × 1.861、0.827 | 6.21 → 6.10 → 5.29 |
| sofa_lap2（流し直し） | 16:11:46 | (−0.530, −0.137, −168.6°) | null | 367 / 462、379 / 445 | 合格 | 巻き数 0.934・縁 0.883 | 0.197 | 50 | 1 | 0.861 | 1.002 × 1.886、0.808 | 2.65 → 5.40 → 5.80 |
| sofa_lap3 | 16:15:30 | (−0.521, −0.033, −171.4°) | 2.83 | 368 / 440、372 / 436 | 合格 | 巻き数 0.920・縁 0.855 | 0.103 | 49 | 1 | 0.872 | 1.003 × 1.883、0.785 | 5.34 → 6.61 → 6.92 |
| suitcase_near1 | 16:19:01 | (−0.524, −0.104, −171.0°) | 3.12 | 369 / 398、367 / 498 | 合格 | 縁 0.880・27.3°・0.864（suitcase_1） | 0.045 | 2 | 0 | 0.812 | 0.550 × 0.390、0.396 | 6.93 → 6.74 → 6.75 |
| suitcase_near2 | 16:20:16 | (−0.474, −0.070, −170.6°) | 2.71 | 379 / 501、375 / 401 | 合格 | 縁 0.950・28.8°・0.941 | 0.031 | 2 | 0 | 0.811 | 0.562 × 0.369、0.457 | 6.53 → 6.35 → 6.21 |
| suitcase_near3 | 16:21:32 | (−0.498, −0.142, −166.5°) | 2.81 | 367 / 423、365 / 441 | 合格 | 縁 1.083・0.0°・1.076 | 0.034 | 3 | 0 | 0.828 | 0.533 × 0.401、0.594 | 6.04 → 6.82 → 7.19 |
| suitcase_lap1 | 16:22:57 | (−0.534, −0.113, −170.4°) | 4.42 | 373 / 406、374 / 438 | 合格 | 巻き数 1.036・縁 0.972 | 0.052 | 26 | 0 | 0.867 | 0.557 × 0.390、0.677 | 7.19 → 6.96 → 6.65 |
| suitcase_lap2 | 16:25:30 | (−0.512, −0.087, −168.2°) | 1.08 | 370 / 428、384 / 415 | 合格 | 巻き数 1.037・縁 0.930 | 0.039 | 26 | 0 | 0.859 | 0.549 × 0.384、0.404 | 6.20 → 6.60 → 6.55 |
| suitcase_lap3 | 16:27:58 | (−0.533, −0.029, −172.7°) | 2.68 | 363 / 412、376 / 453 | 合格 | 巻き数 1.032・縁 0.967 | 0.037 | 28 | 0 | 0.869 | 0.532 × 0.387、0.549 | 6.55 → 9.72 → 6.52 |
| cone_near1 | 16:30:29 | (−0.285, −2.736, −96.8°) | 2.50 | 373 / 467、360 / 383 | 合格 | 縁 0.973・0.0°・0.971（traffic_cone_1） | 0.082 | 1 | 0 | — | 0.600 × 0.608、0.522 | 6.24 → 6.92 → 6.84 |
| cone_near2 | 16:31:45 | (−0.210, −2.701, −95.4°) | 3.39 | 371 / 413、371 / 430 | 合格 | 縁 1.006・0.0°・1.003 | 0.067 | 1 | 0 | — | 0.540 × 0.460、0.425 | 6.53 → 6.84 → 7.85 |
| cone_near3 | 16:33:00 | (−0.489, −2.667, −100.5°) | 3.24 | 373 / 422、365 / 393 | 合格 | 縁 1.053・11.4°・1.049 | 0.066 | 1 | 0 | — | 0.652 × 0.549、0.708 | 7.54 → 6.46 → 6.38 |
| cone_lap1 | 16:34:14 | (−0.393, −2.631, −98.2°) | 2.90 | 382 / 441、378 / 525 | 合格 | 巻き数 1.098・縁 0.867 | 0.110 | 28 | 0 | 0.861 | 0.593 × 0.482、0.572 | 6.38 → 6.80 → 6.28 |
| cone_lap2 | 16:36:51 | (−0.122, −2.877, −93.5°) | 3.08 | 366 / 407、386 / 420 | 合格 | 巻き数 0.928・縁 0.812 | 0.051 | 37 | 0 | 0.863 | 0.619 × 0.529、0.674 | 7.38 → 6.71 → 5.78 |
| cone_lap3 | 16:40:05 | (−0.310, −2.662, −96.3°) | 3.13 | 363 / 439、373 / 406 | 合格 | 巻き数 1.130・縁 0.920 | 0.045 | 29 | 0 | 0.857 | 0.590 × 0.558、0.531 | 6.04 → 7.04 → 6.62 |

- **スーツケース（R-H3j、分けて書く）：** Bf から手前 3/3（縁まで 0.880〜1.083 m、足跡に 0.0〜28.8°、どの足跡にも 0.864 m 以上）、周回 3/3
  （巻き数 1.032〜1.037、縁まで 0.930〜0.972 m）。名前はどの回も `suitcase`（M4 の目標の行）。R-H3j が心配した「Bf でスーツケースが正しさか
  手前・周回で落ちる」は起きなかった。M3 の yoloe の Bf も、位置 0.024 m・名前は正しい。一覧に入ったのは 7 回のうち 4 回が B の止まり（回った直後、
  2.68〜3.12 秒）、3 回が Bf の止まり（1.08〜4.42 秒）。測った大きさは 0.532〜0.562 × 0.369〜0.401（配置の世界の軸で 0.468 × 0.291）。
- **コーン（R-H3k、分けて書く）：** R5 から手前 3/3（縁まで 0.973〜1.053 m、0.0〜11.4°）、周回 3/3（巻き数 0.928〜1.130、縁まで 0.812〜0.920 m）。
  6 回とも、R の止まり（コーンまで約 4.45 m）で 2.50〜3.39 秒後に `traffic_cone` として一覧に入り、R5 ではもう一覧にあった。行の `confidence`
  0.425〜0.708（R-H3k (3)）。測った大きさは 0.540〜0.652 × 0.460〜0.608（底 0.503 × 0.503）。cone_lap2 では、R で見たスーツケースが
  `television_1`（1.102 × 0.382）と呼ばれ、そのまま一覧に残った（目標ではないので線には効かない。正しさの誤りとして記録）。
- ソファ：Bf から手前 3/3（縁まで 1.077〜1.083 m、0.0°）、周回 3/3（巻き数 0.920〜1.143、縁まで 0.749〜0.883 m）。測った大きさ
  1.000〜1.003 × 1.849〜1.886（配置 1.044 × 1.8。奥が M3 と同じく少し短い）。
- 着いた姿勢（M3・M4・画面なし・声の段の準備・わけを探す回の、試験の姿勢に着いた全部）：Bf 23 回は x −0.557〜−0.474、y −0.142〜−0.024、
  向き −173.6〜−165.5°。R5 11 回は x −0.492〜−0.122、y −2.877〜−2.631、向き −100.5〜−93.5°。`tests/test_household.py` の `MEASURED_RANGES`
  （R-H5a の後：Bf x −0.560〜−0.453・y −0.141〜−0.032・向き −174.0〜−166.6°、R5 x −0.413〜−0.225・y −2.804〜−2.654・向き −98.0〜−95.1°）の外に出た回：
  - Bf 6 回：y が先（−0.029・−0.030・−0.029・−0.024、最大 0.008 m）、y −0.142（0.001 m）、向き −166.5°（0.1°）と −165.5°（sofa_lap2、1.1° 回り足りない）；
  - R5 6 回：x −0.435・−0.489・−0.492（最大 0.079 m 先）、x −0.210・−0.122（最大 0.103 m 手前）、y −2.631・−2.877、向き −98.2〜−100.5°・−93.5°
    （最大 2.5°）。いちばん外れたのは cone_lap2 (−0.122, −2.877, −93.5°) と cone_near3 (−0.489, −2.667, −100.5°)。
  どの姿勢からも目標の式は計画ができて線は通った（sofa_lap2 の外れは前の物で、姿勢の計画ではない）。固定は変えていない（controller へ）。

### 待ちの測り方と結果（設計書 §13.5「止まってから一覧に出るまで 5 秒以内」、pre-final F11）

- 計画どおり、`resolve` を姿勢に着く式（Bf は `F moved_forward`、R5 は 5 歩の式）の直前に別に始めた。ところがこの `resolve` は 19 回のうち
  17 回 `null`（「the Go2 was not standing still after a stop this tool saw」）。わけ：(1) 目標はたいてい**その前の止まり（B か R）でもう一覧に
  入っていた**ので、待つ行がなかった；(2) `F moved_forward` は 1 手で短く、`resolve` の container が odometry を受け始める前に Go2 が止まり終える回がある。
- そこで、同じ `resolve` を**回る式の前**にも始めた（F11 の「最後の動く式の前に始める」と同じ考えで、一つ前の動く式の前）。`resolve` の止まりの時刻は
  動いた後の止まりごとに置き直されるので、この値は「一覧に出た止まりから、一覧に出るまで」になる。表の「待ち」はこの値で、16 回（M4）で 1.08〜4.42 秒。
  記録係の時刻（止まりの変わり目と行の届いた時刻）でも同じ値だった（例：suitcase_lap1 は Bf に止まって 4.42 秒、suitcase_near1 は B に止まって 3.12 秒）。
- **`null` の 2 回（sofa_near1、sofa_lap2 の流し直し）：** 記録係のある流し直しでは、B に止まって 3.08 秒でソファが `bench_1` として一覧に入り、
  B に止まって 7.58 秒後（Go2 が Bf へ歩き出した直後に届いた、B で始めた推論の結果）に名前が sofa に変わって `sofa_1` になった。`resolve` は
  受け入れる名前（sofa）の行だけを待つので、その行が届いた時は「動いている」で `null`。Bf に止まった時には `sofa_1` はもう一覧にあったので、
  試験の姿勢（Bf）での待ちは 0 秒。sofa_near1 は記録係の前の回だが、`resolve` の標準エラーに同じ「bench_1 ('bench') pairs with sofa_1」が出ていて、
  同じ形と見る。**B（ソファまで 4.4 m、試験の姿勢ではない）では、受け入れる名前が付くまで 7.6 秒かかることがある**（ソファは O38 で Bf から試験する）。
- 判定：試験の姿勢に止まってから、目標が一覧にある（測られ、受け入れる名前）までは、どの回も 5 秒以内（測れた 16 回で最大 4.42 秒、2 回は止まった時に
  もう一覧にあった）。声の段の 7 回の準備と画面なしの 2 回も 1.23〜3.80 秒。

### ソファの周回が前の物で止まった 2 回（わけ）

- **M4 sofa_lap2（16:06:53、load 6.1）：** 55 手の計画の終わり近く（巻き数 0.930 まで回った所）で「前方に物が出た」→「一時停止：前進（前方に障害物が
  あります）」→ 15 秒後に「中止：前進（15 秒待っても 前方に障害物があります）」で `run_formula` が終了コード 1。線の値（巻き数・縁）は満たしていたが、
  終了コードの線で不合格。回の間でいちばん近い足跡は suitcase_1（0.501 m）。
- **声の段のソファの周回 1 回目（16:55、load 5.1）：** 計画の 8 手目（前進 ×5、左に回る、前進、左に回る の後の前進）で前の物、15 秒で中止。
  止まった姿勢は (−2.386, −1.001, 12.5°)、画面の「前方 0.48 m あり」。**スーツケースの −x の面が 0.53 m 前**（中心まで 0.80 m、方位 −21.9°）。
  止まる距離は 0.5 m（画面の点線）。計画は suitcase の足跡から 0.45 m を超えるマスだけを通るので、この前進のマスは塞がれていないが、そのマスへ向かう
  Go2 の前の 0.5 m の中にスーツケースが入る。
- **わけ：** ソファの周回の輪（ソファの縁から 0.75〜1.75 m）の東の外側の辺が、スーツケース（ソファの縁から約 2.2 m）の 0.45〜0.5 m の所を通り、
  その道でスーツケースに向いて進む手があると、Go2 の前の物の止まる距離（0.5 m）に入る。そこで止まると物は動かないので 15 秒で中止になる。
  止まった位置が少し違うと前の物は消えて続けられる（M4 の合格したソファの周回 4 回とも 1 回ずつ「前方に物が出た」があり、すぐ消えた。H5 の
  logical のソファの周回も同じ）。負荷ではない（どちらも流し直しは load 2.65・5.0 から始まり合格）。detector の足跡が少し大きい
  （スーツケース 0.52〜0.57 × 0.37〜0.60）ことで計画の道が回ごとに変わるが、logical でも同じ道を通る（H5：「周回の道はスーツケースの
  0.465〜0.477 m を通る」）。
- R-PAR のとおり、どちらも load 4 超えの外れなので、load が 4 未満で `pytest` がないのを待って流し直し、合格した回を数えた。計画の「外れたら画面なしで
  流し直し、それでも外れたら M1」は、数える外れが残らなかったので流していない（わけも RTF ではない）。決めることは「controller に返すこと」。

### M0 detector、歩く、画面なし（最初の種類：ソファ）

`detector_check.PREVIEW_TARGETS` の最初の種類（ソファ）で、M4 と同じ手順を画面なしで 1 回ずつ。

| 回 | 起動 | 着いた姿勢 | 待ち | 状態（前 / 後） | 判定 | 線の値 | 傾き | 手 | RTF（歩く） | CPU（Gazebo / 検出器） |
|---|---|---|---|---|---|---|---|---|---|---|
| m0w_sofa_near | 16:45:36 | (−0.556, −0.106, −171.0°) | 3.80 | 380 / 441、366 / 426 | 合格 | 縁 1.041 m・0.0°・0.671（suitcase_1） | 0.039 | 5 | **0.910** | 205 % / 4 % |
| m0w_sofa_lap | 16:46:59 | (−0.557, −0.111, −169.8°) | 2.59 | 364 / 435、380 / 485 | 合格 | 巻き数 0.911・縁 0.883 m | **0.410** | 52 | **0.938** | 218 % / 4 % |

- 歩く間の RTF：画面あり（M4 の 17 回）0.811〜0.873（中央値 0.861）、画面なし 0.910・0.938。D7 の判断の「その RTF で一周と手前が通る」は、
  画面ありで満たす（画面なしの合格の回は要らなかった）。m0w_sofa_lap の傾き 0.410 は線（1.0）の内だが、H6 のほかの回（最大 0.197）より大きい。

### 声の段（設計書 §13.4、O26、計画の I6 の裁定）

- app は main checkout の場所から、こちらの project 名 `go2oo` で起動した（上書きのファイルで worktree を `/work` に、`LLM_BACKEND: claude`。
  ファイルは作業用のフォルダにあり、git に入れていない）。compose が main checkout の `env.txt` を `env_file` で読む（だれも開いていない）。
  `GA config` は流していない。
  ```bash
  GA() { (cd ~/work/physicalAI/go2 && GO2OO_WT="$WT" GO2_UI_PORT=8811 SIM_KIND=gazebo \
           docker compose -p go2oo --project-directory . -f "$WT/compose.yaml" -f "$OV" "$@"); }
  GA run --rm --no-deps -T app python3 -c "import voice_ltl.vocabulary as v; print(v.__file__, len(v.VOCABULARY.entries))"
  # → /work/voice_ltl/vocabulary.py 50（15:11 と 16:51 の 2 回、どちらも同じ）
  ```
  ログインの言葉（「LLM にログインできていません」）はどの回の app のログにも、画面にも出なかった。
- 指示ごとに：起動し直した detector の組（yoloe、画面あり）で、M4 と同じ姿勢の手順と `resolve`（app を起動する前。`run_formula` と app が同時に
  Go2 を動かさない）、`GA up -d --no-build --no-deps app`、http://localhost:8811 のページで Go2 が「姿勢 立ち」と目標が「見えている物」にあるのを
  確かめ、`G run --rm --no-deps -T app python3 gazebo/tools/objects_check.py watch --expect near|lap --target <正解の id>` を始め、ページの
  「文字で指示する（試験用）」の欄に指示を入れて「送る」（Browser の画面。欄は `/api/text` に送る）。式は app のログと画面の「ロジック（LTLf）」から、
  メッセージは画面から写した。最後に `G … down`（app も消える）。

| 指示 | 起動 | 準備の姿勢・待ち | 「見えている物」（送る前） | Claude の式 | 画面のメッセージ | `watch` の判定と値 | RTF（歩く / 止まり） | load（始め / 後） |
|---|---|---|---|---|---|---|---|---|
| 「ソファまで歩いて」 | 16:51:36 | (−0.516, −0.080, −168.1°)、3.07 秒 | スーツケース（左前 1.5 m）、ソファ（前 4.0 m） | `F near_sofa_1`（1 回目で合格、「Claude が翻訳 · 単語帳と文法の検査に合格」、試行 1 回） | 「終わりました」「前進 → 前進 → 前進 → 前進 → 前進 を、計画どおりに実行しました。」 | **合格**：縁 1.091 m・0.0°・どの足跡にも 0.631（suitcase_1） | 0.846 / 0.829 | 5.72 / 6.34 |
| 「ソファの周りを回って」（数えない） | 16:55:31 | (−0.528, −0.080, −167.8°)、2.51 秒 | 同じ | `F around_sofa_1`（1 回目で合格、「こう受け取りました：「周りを回って」→ 左回りに 1 周」） | 「できませんでした」「続けられませんでした」「15 秒待っても 前方に障害物があります。」（8 手で中止、前方 0.48 m） | **不合格**：巻き数 −0.065（前の物で止まった。上の「わけ」） | 0.851 / 0.841 | 5.11 / 6.48 |
| 「ソファの周りを回って」（流し直し） | 17:00:13 | (−0.515, −0.032, −172.2°)、3.15 秒 | 同じ | `F around_sofa_1`（同じ置き換えの言葉） | 「終わりました」「前進 → …（52 手）… → 前進 を、計画どおりに実行しました。」 | **合格**：巻き数 0.917・縁 0.876 m | 0.877 / 0.853 | 5.04 / 5.71 |
| 「スーツケースまで歩いて」 | 17:05:08 | (−0.490, −0.045, −170.3°)、3.19 秒 | スーツケース（左前 1.5 m）、ソファ（前 4.0 m） | `F near_suitcase_1`（1 回目で合格） | 「終わりました」「前進 → 左に回る を、計画どおりに実行しました。」 | **合格**：縁 0.957 m・28.8°・0.944 | 0.827 / 0.827 | 5.14 / 5.56 |
| 「スーツケースの周りを回って」 | 17:07:08 | (−0.525, −0.024, −173.6°)、1.48 秒 | スーツケース（左前 1.6 m）、ソファ（前 4.0 m） | `F around_suitcase_1`（1 回目で合格、同じ置き換え） | 「終わりました」「右に回る → …（28 手）… → 前進 を、計画どおりに実行しました。」 | **合格**：巻き数 1.032・縁 0.958 m | 0.855 / 0.852 | 5.31 / 7.60 |
| 「コーンまで歩いて」 | 17:10:39 | (−0.294, −2.710, −96.4°)、1.23 秒 | コーン（前 1.7 m） | `F near_traffic_cone_1`（1 回目で合格） | 「終わりました」「前進 を、計画どおりに実行しました。」 | **合格**：縁 1.004 m・0.0°・1.004 | 0.749 / 0.833 | 7.61 / 7.16 |
| 「コーンの周りを回って」 | 17:12:51 | (−0.492, −2.713, −99.7°)、2.81 秒 | コーン（前 1.7 m） | `F around_traffic_cone_1`（1 回目で合格、同じ置き換え） | 「終わりました」「前進 → 右に回る → …（35 手）… → 前進 を、計画どおりに実行しました。」 | **合格**：巻き数 1.108・縁 0.837 m | 0.753 / 0.867 | 13.64 / 9.03 |

- 6 つの指示とも、Claude の式は期待どおりの物の命題 1 つ（検出器の id）で、単語帳と文法の検査に 1 回目で合格した。周回の 3 つは「「周りを回って」→ 左回りに 1 周」
  の置き換えの言葉が出た。種類の見張り（O35、`97da92a`）が断った指示はない。
- ソファの周回の 1 回目は前の物で中止（上の「わけ」）。`watch` は「動いた後 10 秒止まった」で終わるので、一時停止の 15 秒の途中（中止の 3 秒前）で
  終わり、巻き数 −0.065 を出した。R-PAR のとおり（load 5.1）流し直し、合格した。
- スーツケースの周回で 1 回、「センサー（LiDAR か /sportmodestate）が途切れています」の一時停止が 1 秒あり（08:09:28 UTC）、続けられた。
- 推論の時間（準備の後の状態）：p50 368〜473 ms、p90 407〜755 ms（コーンの 2 回は load 7.6・13.6 で p90 754・684 ms。§13.5 の 800 ms の内）。
- 利用者が話して流した回はない。Whisper はこの仕事で変わらない（O26）。

### M5（D9）：立った Go2 の前にボタンの箱

- detector の組（yoloe、画面あり、17:17）で `F standing`、8 秒の後に「前に箱を置く」（`/sim/obstacles` に `{"cmd": "front"}`、受け手 1）。
  `/sim/world` は箱 `[1.245, 0.019, 0.3]`（Go2 の前 1.2 m、0.3 m の箱）。
- 置いた後 5 秒ごとに 6 回 `/go2/objects` を読み、`accuracy` も流した：**一覧は `chair_1` だけで、`box` の行は出なかった**（`button_boxes_seen` 空、
  推論 21 回、`ids_issued` は chair 1 つで、box の仮の追跡もない）。
- **段ボールの絵が付いていない：** Gazebo のログに、サーバーと画面の両方で `Could not resolve file [cardboard_box.png]`。`bridge_core.CARDBOARD_MESH`
  は `file:///opt/ros/jazzy/share/ros_gz_sim_demos/models/cardboard_box/meshes/cardboard_box.dae` で、その DAE は絵を `cardboard_box.png`
  （名前だけ）で引くが、絵は `../materials/textures/cardboard_box.png` にある（イメージの中で確かめた）。だから箱は形だけで、段ボールの絵のない
  見た目で描かれた。設計書 §11.13 の「detector のときは段ボールの絵」は、この形では効いていない。線ではないので記録だけ（controller へ）。

### M0 後：logical の `household` の RTF（detector の段の後）

M0 前と同じ手順（logical、伏せたまま、20 秒の後に 75 秒の窓）。始める前に load 4 未満・`pytest` なしを 60 秒ごとに待った（どれも 6 分以内に開いた）。

| 回 | 窓の始め | 画面 | RTF（75 秒） | 始めの load | 終わりの load | 窓の間のほかの container（始め / 終わり） | `go2oo-gazebo`（終わり） | `pytest`（始め / 終わり） |
|---|---|---|---|---|---|---|---|---|
| m0a_on1 | 17:21:12 | あり | **0.84** | 3.83 6.07 6.37 | 6.62 6.55 6.53 | なし / `stairs-tests` 101 % | 416 %、1.247 GiB | 0 / 3 |
| m0a_on2 | 17:25:01 | あり | **0.84** | 4.68 5.48 6.08 | 6.62 5.92 6.19 | `stairs-tests` 105 % / 92 % | 417 %、1.243 GiB | 0 / 3 |
| m0a_on3 | 17:31:52 | あり | **0.90** | 4.67 4.30 5.27 | 4.40 4.30 5.18 | なし / なし | 417 %、1.240 GiB | 0 / 0 |
| m0a_off1 | 17:34:40 | なし | **0.98** | 3.24 3.85 4.91 | 4.54 4.11 4.92 | `stairs-tests` 107 % / 113 % | 216 %、639 MiB | 0 / 2 |
| m0a_off2 | 17:41:28 | なし | **1.00** | 2.44 2.84 4.04 | 3.45 2.99 3.99 | なし / なし | 161 %、639 MiB | 0 / 0 |
| m0a_off3 | 17:43:15 | なし | **1.00** | 4.12 3.21 4.03 | 5.59 3.75 4.15 | なし / なし | 199 %、638 MiB | 0 / 0 |

- 画面あり 0.84〜0.90（中央値 0.84）、画面なし 0.98〜1.00（中央値 1.00）。M0 前（0.85〜0.89 / 0.99〜1.00）と同じくらい。画面ありの 0.84 の 2 回は、
  窓の間にほかの組の `stairs-tests`（約 100 %）と `pytest` 3 があった回。
- まとめ（画面あり / なし、中央値）：logical 前 0.87 / 1.00、detector 止まり 0.86 / 0.99、detector 歩く 0.861（M4 の 17 回）/ 0.91・0.938、logical 後 0.84 / 1.00。
  detector の組の損は、止まりで 0.01、歩く間で logical の窓（止まり）よりさらに 0.01 くらい。D7 の判断の「その RTF で一周と手前が通る」は満たす。

### §2 の組 2（設計書 §13.6、logical、`household`）

H5 の「§2 の組 1」と同じ 4 つの命令を、logical、画面あり、回ごとに起動し直した Gazebo で（`objects_check.py run`）。どの回も伏せから始まり、
最初の動作は「立つ」、`run_formula` の終了コードは 0。ほかの組の container はどの回の前後にもなく、`pytest` も 0。

| 回 | 起動 | 式 | 判定 | 巻き数 / 箱の縁まで | 椅子の中心まで・角度 / 椅子の縁まで | 傾き | RTF（歩く） | load（前 → 後） |
|---|---|---|---|---|---|---|---|---|
| lap1 | 17:44:34 | `F around_box_1` | 合格 | 1.037 / 0.797 m | — | 0.064 | 0.901 | 5.59 → 6.14 |
| lap2 | 17:47:03 | `F around_box_1` | 合格 | 1.041 / 0.864 m | — | 0.058 | 0.899 | 6.14 → 5.47 |
| lap3 | 17:49:27 | `F around_box_1` | 合格 | 1.042 / 0.872 m | — | 0.058 | 0.898 | 5.44 → 5.28 |
| lap4 | 17:51:53 | `F around_box_1` | 合格 | 1.044 / 0.844 m | — | 0.102 | 0.912 | 5.28 → 6.29 |
| lap5 | 17:54:28 | `F around_box_1` | 合格 | 1.042 / 0.879 m | — | 0.176 | 0.905 | 6.29 → 6.19 |
| chair1 | 17:56:52 | `F near_chair_1` | 合格 | — | 1.063 m・29.3° / 0.761 m | 0.058 | 0.883 | 6.25 → 6.71 |
| chair2 | 17:57:32 | `F near_chair_1` | 合格 | — | 1.087 m・29.7° / 0.787 m | 0.058 | 0.866 | 6.97 → 6.54 |
| chair3 | 17:58:13 | `F near_chair_1` | 合格 | — | 1.091 m・29.5° / 0.789 m | 0.131 | 0.886 | 6.54 → 6.28 |
| hello1 | 17:58:53 | `F(near_chair_1 & X F hello)` | 合格 | — | 1.089 m・29.4° / 0.784 m | 0.058 | 0.867 | 6.28 → 6.82 |
| box1 | 17:59:39 | `F near_box_1`（`--target box_1`） | 合格 | — | 箱：縁まで 0.848 m・足跡に 8.2°・中心まで 1.169 m・22.5°・どの足跡にも 0.844 m（`source` logical） | 0.058 | 0.891 | 6.82 → 6.61 |

- **§2 の組 2 は 10/10：** 一周 5/5（5 回続けて。巻き数 1.037〜1.044、箱の縁まで 0.797 m 以上）、椅子 3/3（中心まで 1.063〜1.091 m、29.3〜29.7°、
  縁まで 0.761 m 以上）、挨拶つき 1/1、「箱の前まで」1/1（§10.8 の縁の線と O16 の中心の線）。どれも転ばない（最大 0.176）。前の物とセンサーの途切れはない。
  H5 の組 1（10/10）と同じ。A（detector の道）を入れても、logical の §2 は変わらない。
- **Q4：** detector のときに box_1・chair_1 が見つかったかは、上の M3 の「Q4」（chair_1 はいつも、box_1 は 21 回のうち 1 回）。

### R-H5d：logical のコーンの流し直し（R5、手前 ×2・周回 ×2）

R-H5d（logical の橋渡しは、測った後も足跡を配置の世界の大きさより小さくしない）の後に、H5 と同じ手順（logical、画面あり、R5 の手順、
`resolve` は回る式の前と 5 歩の式の前、目標の式）で 4 回。

| 回 | 起動 | 着いた姿勢 | 待ち（`resolve`） | 判定 | 線の値 | 傾き | 手 | RTF（歩く） | 目標の行（大きさ、測った） | load（前 → 後） |
|---|---|---|---|---|---|---|---|---|---|---|
| cone_near1 | 18:00:18 | (−0.264, −2.663, −96.0°) | 0.89 | 合格 | 縁 1.052 m・0.0°・どの足跡にも 1.046 m（traffic_cone_1） | 0.082 | 1 | — | 0.503 × 0.503、済 | 6.61 → 6.82 |
| cone_near2 | 18:01:23 | (−0.384, −2.642, −98.8°) | 0.80 | 合格 | 縁 1.067・3.7°・1.065 | 0.022 | 1 | — | 0.503 × 0.503、済 | 6.82 → 7.01 |
| cone_lap1 | 18:02:26 | (−0.448, −2.694, −100.1°) | 0.52 | 合格 | 巻き数 1.099・縁 0.896 m | 0.066 | 30 | 0.897 | 0.503 × 0.503、済 | 6.61 → 6.05 |
| cone_lap2 | 18:04:52 | (−0.339, −2.665, −97.9°) | 0.81 | 合格 | 巻き数 1.101・縁 0.951 m | 0.034 | 28 | 0.884 | 0.503 × 0.503、済 | 6.05 → 6.59 |

- **4/4 合格。** コーンの行は 4 回とも 0.503 × 0.503・`measured: true`（H5 では 0.479〜0.506 × 0.345〜0.352。R-H5d の直しで、測った後も配置の大きさを
  下回らない）。手前の縁まで 1.052〜1.067 m（H5 0.953〜0.984 m）、周回の縁まで 0.896〜0.951 m（H5 0.880〜0.881 m）で、足跡が大きくなった分だけ
  離れて通る。同じ一覧で box_1 0.516 × 0.529、chair_1 0.469 × 0.473、suitcase_1 0.495 × 0.303（どれも測った）。
- logical では R の止まり（回った直後）で論理カメラがコーンを見て、`resolve`（回る式の前に始めたもの）は 0.52〜0.89 秒。
- 着いた姿勢のうち 2 回が `MEASURED_RANGES['R5']` の外（cone_near2 の y −2.642（0.012 m）と向き −98.8°、cone_lap1 の x −0.448（0.035 m）と向き −100.1°）。
  どちらも計画があり線は通った。

### 資源（設計書 §11.12）

`docker stats --no-stream` の回の前後と目標の式の始めの読み（1 回の読みなので、推論の最中か合間かで検出器の CPU は変わる）：

| container | CPU | メモリ |
|---|---|---|
| `go2oo-gazebo-1`（画面あり、detector・logical） | 400〜457 % | 1.24〜1.39 GiB（起動直後の 1 回だけ 1.80 GiB） |
| `go2oo-gazebo`（画面なし） | 166〜230 % | 637〜763 MiB |
| `go2oo-detector-1`（yoloe） | 3〜53 % | 522〜567 MiB（はじめの起動で文字の埋め込みを作った 1 回だけ 953 MiB） |
| `go2oo-detector-1`（oracle） | 3〜4 % | 約 48〜50 MiB |
| `go2oo-app-1`（声の段の app） | 2〜4 % | 91〜122 MiB |

- 文字の埋め込みは、はじめの起動（15:13、数えない慣らしの回）で作った（「text embeddings of 90 classes (3 ignored) from the encoder in 8.39 s」）。
  その後はどの回も cache から（0.00 秒）。検出器が読み込みを終えるまで、組の起動から 7〜9 秒。
- 合計（画面あり、止まって推論中）：Gazebo 約 4.3 vCPU + 検出器 0.5 vCPU 以下 + app 0.05 で、8 vCPU の内。メモリは計 2 GiB くらいで 7.75 GiB の内。

### controller に返すこと（decisions_needed）

1. **M3 の oracle の `covers`（線の外れ）。** oracle は位置・大きさの線をすべて満たしたが、`covers` が偽の組がある：
   (a) **Bf のソファ：奥が 0.22 m 足りず `filled` 偽**（わけを探した回で 3/3。M3 の 1 回目は `filled` 真で 0.098 m の補い）。単語帳の大きさ
   [1.8, 0.9] の 0.9 がモデルの奥行き 1.044 m より浅く、観た奥行き 0.88 m が事前の奥行きに届くので、形の決まり（R-G1 (3)）が奥を観たとみなす。
   O12 の前提が崩れ、yoloe でも同じ（位置 0.136 m、線まで 0.014 m）。
   (b) **R5 のコーン：手前の面が 1.5〜14 mm 内側**（上に細い形。R-H5d と同じわけ）。(c) **Bf のスーツケース：奥の面が 0〜1.5 mm**（`COVER_TOL` 1 mm の前後）。
   決めること：(a) を v1 の限りとして Task 4.1 に書くか（C2 で単語帳は部屋に合わせない）、形の決まり（奥を観たとみなす条件）か単語帳のソファの大きさを
   直す枝を作り、M3 を流し直すか。(b)(c) を 0.05 m の間引きの段より小さい限りとして受けるか。H6 ではコードを変えていない。
2. **ソファの周回が前の物（スーツケース）で 15 秒止まって中止した 2 回**（M4 の lap2、声の段の 1 回目。画面ありのソファの周回 6 回のうち 2 回。画面なしの 1 回は合格）。
   周回の輪の外側の辺がスーツケースの 0.45〜0.5 m を通り、そこでスーツケースに向く手があると止まる距離 0.5 m に入る（止まった時、前方 0.48 m・
   スーツケースの面まで 0.53 m）。どちらも load 4 超えだったので R-PAR で流し直して合格を数えた。決めること：これを数えてよいか（わけは負荷でない）、
   部屋の限りとして書くか、スーツケースを輪から離す置き直し（R-H3j のとおり新しい下見が要る）か。
3. **待ちの測り方。** 計画どおり姿勢に着く式の前に始めた `resolve` は 19 回のうち 17 回 `null`（目標はたいてい B・R の止まりでもう一覧に入っていた）。
   H6 は回る式の前にも `resolve` を始め、その値で線を判定した（16 回で 1.08〜4.42 秒）。ソファの 2 回は、受け入れる名前（sofa）が B から歩き出した
   後に届き（B に止まって 7.6 秒。それまで `bench_1`）、Bf に止まった時にはもう一覧にあった。決めること：この測り方と判定（線は満たす）を受けるか。
   受けるなら、計画の H6 M4 と Task 4.1 の手順の文（`resolve` を始める時）を直す。
4. **着いた姿勢が `MEASURED_RANGES` の外**（Bf 6 回：y は最大 0.008 m 先、向きは最大 1.1° 回り足りない；R5 8 回：x は最大 0.103 m、y は最大 0.073 m、
   向きは最大 2.5°。いちばん外れたのは cone_lap2 (−0.122, −2.877, −93.5°) と cone_near3 (−0.489, −2.667, −100.5°)）。どれも計画があり線は通った。
   R-H5a と同じく範囲を広げて固定し直すか。
5. **M5：ボタンの箱に段ボールの絵が付かない。** `CARDBOARD_MESH`（file:// の DAE）は絵を `cardboard_box.png` の名前だけで引き、Gazebo は
   `../materials/textures/` を探さない（ログに `Could not resolve file [cardboard_box.png]`）。箱は一覧に入らなかった（40 秒、推論 21 回）。
   設計書 §11.13 の段ボールの見た目は効いていない。線ではないので記録だけ。直すなら別の枝（例えば model:// の URI か、絵の場所を
   GZ_SIM_RESOURCE_PATH に足す）で、M5 を流し直す。
6. **記録だけ：** cone_lap2 で R から見たスーツケースが `television_1` と呼ばれ一覧に残った（目標でないので線に効かない）。B ではソファが先に `bench`
   と呼ばれることがある（状態の `ids_issued` に bench が出たのは M4 のソファの 7 回のうち 3 回と画面なしの手前。記録係のある流し直しでは、
   それはソファの行だった）。

### 動かした container

- `G` の compose project `go2oo` の `go2oo-gazebo-1`（画面あり）・`go2oo-gazebo`（画面なし、名前を付けた `G run -d`）・`go2oo-detector-1`、
  `G run --rm … app` の `go2oo-app-run-…`（`run_formula`・`objects_check`・読むだけの使い捨てのスクリプト）、声の段の `go2oo-app-1`（`GA up`）。
  どれも回ごとに `G … down`（画面なしは `docker rm -f go2oo-gazebo` も）で止めた。終わった後に残った `go2oo` の container はない。
  名前付きの volume `go2oo_detector-cache`（文字の埋め込み）は compose が作った（残している）。
- ほかの組の container（`oo-perc-final-lap`、`stairs-tests`）には触れていない。利用者の project `go2`（`go2-gazebo-1`・`go2-app-1`）はどの時も
  動いていなかった。main checkout では `TA`（import の確かめ）と `GA` だけを流し、git は使っていない。

## H6b：detector の Gazebo の最後の測り（R-H6f、2026-11-18〜19）

- 枝：`objects2/h6b`（`feat/open-objects` の `ddbfdbe` から。finish6 の merge の後：R-H6a のソファの奥の直し `objects2/sofa-depth`、
  R-H6e の段ボールの絵の道と R-H6d の固定の広げ `objects2/h6-fixes`。トラッカーの cover125 の上限は戻した後のまま（R-F6a）、種類の見張りは
  `97da92a` のもの（R-PF5e）。worktree `~/work/physicalAI/.go2-worktrees/oo-h6b`。ROS の domain は 66（`G` の回は compose project `go2oo` の自分の network）。
- イメージは H6 と同じ：Gazebo `go2-voice-ltl-gazebo:patch-0005-fuel2`（`c40a28c22024`）、検出器 `go2-voice-ltl-detector:yoloe-11s`（`016348e47306`）、
  app `go2-voice-ltl:dev`（`a35b9bb8c8d3`）。どれも作り直していない。`GO2_LAYOUT=household`（box_1・chair_1・suitcase_1・traffic_cone_1・sofa_1・table_1）。
- 手順・試験の姿勢・線は H6 と同じ（ソファとスーツケースは Bf、コーンは R5。テーブルは logical だけ）。コード・テスト・固定は変えていない。変えたのはこの記録だけ。
  **表の「H6」の列や括弧の中の数は、H6 の記録の同じ測りの数。**
- **範囲（R-F6c）：** `d407854..ddbfdbe` で変わったのは `detector/geometry.py`（ソファ、detector だけ）、launch の detector のときだけの絵の道
  （`gazebo/launch/go2_gazebo.launch.py`・`gazebo/launch_config.py`。logical のときは何も足さない）、`gazebo/bridge_core.py` の定数 1 つ（`CARDBOARD_TEXTURES`、
  detector のときだけ launch が読む）、`gazebo/tools/objects_check.py` の説明の文だけ、テスト・固定・計画の文。
  `git diff --stat d407854 ddbfdbe` と、その 4 つの gazebo のファイルの差分を読んで確かめた：**logical の道のコードは変わっていない**。
  だから logical の段（M0 後、§2 の組 2、R-H5d のコーン）は流していない。M0 前（`m0b`）は流し終えていたので記録に残す。
- **負荷（R-F6b・R-F6c）：** H6b はレビュー・直し・文書の組と同時に流した。load が 4 を下回るのを待たず、どの回にも `uptime` を記録し、
  1 回目の結果をその load と一緒に書き、外れは 1 回だけすぐに流し直した（両方を記録）。
- **controller の止め（23:52）：** 前の H6b の係は M4 の `m4_cone_near1` の目標の式の途中で止められた。その回は数えず（R-F6c）、残った `go2oo` の
  container を止めて（`G … down`、`docker rm -f`）、`m4_cone_near1` から 1 回目として流し直した（23:53）。
- **2 つの組で流した：**
  - **1 回目の組（2026-11-18 22:33〜00:11）。** Mac は 00:11:15 から蓋を閉じた眠りに入った（`pmset -g log`：「Entering Sleep state due to 'Clamshell Sleep'」、AC、
    その後は「Maintenance Sleep」と 27 秒ほどの DarkWake のくり返し。02:47 に `AppleClamshellState` = Yes）。その間は DarkWake の短い間だけ動くので、
    1 つの `run_formula` や container の起動が 5〜16 分かかった。R-H5b のとおり、眠りの入った回は数えない（データは残した）。02:47 に、こちらの並べた残りの
    回を止め、組を下ろした。
  - **2 回目の組（2026-11-19 07:39〜08:28、蓋を開けた後）。** 1 回目の組の残り（コーンの周回 lap1・lap3、推論の時間の流し直し 3 回、M0 detector の
    画面なし 2 回、M3 の流し直し 3 つ、M5、声の段）を、どれも 1 回目として（流し直しは 2 回目として）流した。手順は 1 回目の組と同じで、次を足した
    （作業用のスクリプトだけ。git に入れていない）：
    - 回ごとに、始める前に `ioreg -r -k AppleClamshellState -d 4 | grep AppleClamshellState` を見る門（Yes なら何も始めず 2 分ごとに見る、最長 3 時間）。
      どの回も No で、待たなかった。
    - 回の前と後に `pmset -g log` の Sleep の数を数え、増えたら眠りの入った回として数えない門。2 回目の組の間ずっと 1502 のままで、眠りはなかった。
    - 1 回目の組の CPU の門（こちらのでない container が 300 % を超える間は 30 秒ごとに待つ、最長 15 分）。待った回はない。
    - M4 と M0 の歩く回では、目標の式の間 `/model/robot/odometry` の roll・pitch・高さ・姿勢を 50 Hz で全部記録する読むだけのスクリプトと、
      `run_formula` の出す行（開始：・完了：・前方に物が出た など）に実際の時刻を付けた写し（傾きが大きい回の「いつ・どの手・どこ」を見るため。最終レビューの safety-2）。
    - M5 は Gazebo のログを組を下ろす前に保存し、カメラの絵を 2 枚（箱の前と後）取って、検出器と同じ YOLOE で読んだ（下）。
    - この組の間、ほかの係は重い container を動かさない取り決めで、`docker stats` にほかの組の container が出たのは 1 回だけ（`oo-docs-tests-71` 25.8 %）。

### 結論

**R-H6a の直しが効き、M4 の 18 回は 1 回目でどれも合格した。** ソファは Bf で `filled` 真になり（H6 の「奥が 0.22 m 欠ける」は出ない）、yoloe のソファの位置の
誤差は 0.136 → 0.077 m。ソファの周回は 1 回目で 3/3（H6 は 2/3）、コーンの周回は 3/3、M0 detector の画面なしの 2 回も 1 回目で合格。声の段（実際の Claude）は
1 回目で 5/6、流し直しで 6/6：ソファの周回の 1 回目が、H6 と同じ所でスーツケースの前で止まった（R-H6b の部屋の限り）。安全の問題（動くべきでない時に
動いた、物に触れた、倒れそうになった）はない。外れはどれもわけがわかったので、R-H6f のとおり v1 の限りとして数と一緒に書く：

1. M3 の数える読み 3 つ（yoloe の R5、oracle と yoloe の spawn の伏せ）が、測りの道具の競り合いで読めなかった（1 回目）。流し直しは 3 つとも読めて線を満たした。
2. 推論の時間の線が、重い負荷の下で 1 回目に 3 回外れた（load 9.8〜42.7）。流し直しは 3 回とも線の内（p50 367〜394・p90 389〜438 ms、load 5.5〜11.0）。
3. suitcase_near2 の流し直し（2 回目）で、Bf に止まってからスーツケースが一覧に出るまで 7.09 秒（線は 5 秒）。1 回目の 18 回は 1.27〜4.33 秒。
   スーツケースは絵の端に近く（R-H3j）、点が確かめの線 0.35 の近くにあり、止まって 3 回の推論では確かにならず、4 回目は検出器の休みの間（3 秒）の後だった（下）。
4. 声の段のソファの周回の 1 回目が「15 秒待っても 前方に障害物があります」で中止（前方 0.48 m にスーツケース。R-H6b の部屋の限り）。

記録だけ：声の段の準備の 1 回で、ソファが Bf でまず `bench` と呼ばれ、`sofa` の名前が付くまで止まって 6.89 秒（R-H6c のとおり 5 秒の線とは別に書く）。
M5 の箱は H6 と同じく一覧に入らなかった。R-H6e の直しでログの `Could not resolve file` は消え、箱は段ボールの色で描かれたが、検出器の YOLOE は箱の所に
`air purifier` 0.103 しか出さず（見えたとみなす線 0.25 の下）、`box` の仮の追跡もできなかった。これが M5 の本当のわけ（下）。
傾きは H6b で最大 0.237（1 回目の組の sofa_near3）、2 回目の組では 0.127 以下、M0 detector の画面なしは 0.053・0.051（H6 の 0.410 は出なかった）。

| 線（計画 H6、設計書 §13.3〜§13.6） | しきい値 | H6b（1 回目。流し直しは別に書く） | H6 |
|---|---|---|---|
| M3 oracle：位置・大きさ（spawn 伏せ・立ち、Bf、R5） | 目標 0.15 m、どれも 0.35 m、中央値 0.25 m、大きさ 0.4 m、名前 0.8、recall 0.6、誤り 1 以下 | **立ち・Bf・R5 は満たす**（目標 0.009〜0.078 m、大きさ 0.181 m 以下、名前 1.0、recall 1.0、誤り 0）。**spawn 伏せは 1 回目が読めず**（道具の競り合い）、流し直し（2 回目の組）は読めて満たす（box_1 0.030 m・0.181 m、chair_1 0.004 m、範囲に `detectable` なし） | 満たす（目標 0.024〜0.079 m） |
| M3 oracle：`covers`（見えた所だけ、C2） | 真 | Bf はソファ・スーツケースとも真（数える読み）。R5 のコーンは偽（手前の面 2.5〜15 mm、R-H6a (b) の限り）。ソファの +y の横の面が 3 通に 1 通 2 mm 足りない（v1 の限り、下） | Bf のスーツケース偽、R5 のコーン偽、探す回でソファ偽（奥 0.22 m） |
| M3 yoloe：同じ線（Bf、R5、spawn） | 同じ | **Bf は満たす**（ソファ 0.077 m・スーツケース 0.031 m）。**R5 と spawn 伏せは 1 回目が読めず**（道具の競り合い）。流し直し（2 回目の組）は R5 がコーン 0.084 m・`covers` 真で合格、spawn 伏せは chair_1 0.006 m で満たす | 満たす（ソファ 0.136 m） |
| M4 ソファ（Bf）手前 ×3 | 縁まで 1.5 m 以内・足跡に 45° 以内・どの足跡にも 0.30 m 以上 | **3/3（1 回目）**（縁 1.046〜1.129 m、0.0〜1.8°、0.538 m 以上） | 3/3（1.077〜1.083 m） |
| M4 ソファ（Bf）周回 ×3 | 巻き数 0.9 以上 1.5 未満・縁まで 0.6 m 以上 | **3/3（1 回目）**（巻き数 0.917〜1.165、縁 0.801〜0.839 m） | 1 回目 2/3（流し直しで 3/3） |
| **M4 スーツケース（Bf、R-H3j）**手前 ×3 | 手前の線 | **3/3（1 回目）**（縁 0.885〜1.016 m、2.8〜26.1°、0.871 m 以上）。near2 の流し直しも合格（縁 0.957 m・24.2°） | 3/3（0.880〜1.083 m） |
| **M4 スーツケース（Bf、R-H3j）**周回 ×3 | 周回の線 | **3/3（1 回目）**（巻き数 1.032〜1.034、縁 0.918〜0.955 m） | 3/3（1.032〜1.037） |
| **M4 コーン（R5、R-H3k）**手前 ×3 | 手前の線 | **3/3（1 回目）**（縁 0.959〜1.009 m、0.0°）。near2 の流し直しも合格（縁 1.070 m） | 3/3（0.973〜1.053 m） |
| **M4 コーン（R5、R-H3k）**周回 ×3 | 周回の線 | **3/3（1 回目）**（lap2 は 1 回目の組で巻き数 1.132・縁 0.913 m、lap1・lap3 は 2 回目の組で 1.100・0.917 m と 1.103・0.933 m）。lap2 の流し直しも合格（1.135・0.846 m） | 3/3（0.928〜1.130） |
| M4：止まってから一覧に出るまで（時の `resolve`、R-H6c） | 5 秒以内 | **1 回目の 18 回は満たす**（1.27〜4.33 秒）。**流し直しの suitcase_near2 で 7.09 秒（外れ、v1 の限り、下）**。ほかの流し直し 2 回 2.87・3.08 秒。M4 の名前の変わり（bench → sofa）はなかった | 1.08〜4.42 秒 |
| M4：推論の時間（目標の式の前と後の状態） | p50 600 ms 以内 / p90 800 ms 以内 | **1 回目の 18 回のうち 15 回**。外れ 3 回：suitcase_near2 の後 p90 910 ms（load 16.9）、cone_near2 の後 p90 893 ms（load 9.8〜11.8）、cone_lap2 の後 p50 661 ms（load 28.7〜42.7）。**流し直しは 3 回とも線の内**（後 p50 367〜394・p90 389〜438 ms） | 満たす（p50 360〜395、p90 379〜525） |
| M4：転ばない | 傾き 1.0 rad 未満 | M4 の 21 回（1 回目 18、流し直し 3）とも（最大 0.237。2 回目の組は 0.127 以下） | 19 回とも（最大 0.197） |
| M0 detector 歩く・画面なし（ソファ 手前 1・周回 1） | M4 の線 | **2/2（1 回目、2 回目の組）**（縁 1.063 m・0.0°；巻き数 0.918・縁 0.821 m）、傾き 0.053・0.051 | 2/2（傾き 0.410 の回あり） |
| 声の段（実際の Claude）×6 | M4 の線、各 1/1 | **1 回目 5/6**（ソファの周回が前の物で中止、R-H6b）、**流し直しで 6/6** | 1 回目 5/6（流し直しで 6/6） |
| M5（D9）：箱が `box` として一覧に入るか、段ボールの絵 | 記録だけ | 入らなかった（推論 29 回、`box` の仮の追跡なし）。ログの `Could not resolve file` は 0 行（R-H6e の直しが効いた）。箱は段ボールの色で描かれ、YOLOE の箱の所の一番の名前は `air purifier` 0.103 | 入らなかった（わけは絵だと書いたが誤り、下の「訂正」） |
| M0：RTF | 記録だけ（D7） | logical 前 あり 0.91〜0.92・なし 1.00；detector 止まり（R5）あり 0.88〜0.89・なし 0.99；detector 歩く あり 0.734〜0.875（M4。1 回目の組のコーンの 2 回は負荷で 0.465〜0.551）、なし 0.949・0.951 | 0.85〜0.89・0.99〜1.00；0.84〜0.87・0.99；0.811〜0.873、なし 0.910・0.938 |
| M0 後、§2 の組 2、R-H5d | — | **流していない**（R-F6c：logical の道のコードは変わっていない） | 0.84〜0.90 / 0.98〜1.00；10/10；4/4 |

- **1 回目の数（R-H6b、R-F6c）：** M3 の数える読み 8 のうち、読めた 5 はどれも線を満たし、読めない 3 は道具の競り合い（流し直しは 3/3 満たす。spawn の
  recall は H6 と同じく n/a）。M4 は 18/18（ソファ・スーツケース・コーンの手前と周回 各 3/3）。待ちの線は 1 回目の 18/18、推論の時間の線は 1 回目の 15/18。
  M0 detector の画面なし 2/2。声の段 5/6。どの回の load も下の表に書いた（1 回目の組 1.52〜57.9、2 回目の組 3.06〜17.51）。
- **安全：** ロボットが動くべきでない時に動いた回、物に触れた回、倒れそうになった回はない。M4 のどの足跡にもいちばん近づいたのは 0.501 m、
  声の段のソファの周回の 1 回目は胴の中心からスーツケースの足跡まで 0.443 m（最終レビューの safety-2 が胴の角に使った 0.385 m より遠い）で、
  前方 0.48 m で止まって 15 秒待ち、中止した（正しい止まり方）。前の物や後ろの物での一時停止とセンサーの途切れの一時停止は、止まって続けた（下）。
  傾きはどの回も 0.237 以下（線は 1.0）。

### D11 の門、ほかの container、電源、load、眠り

- `go2-gazebo-1`（利用者の Gazebo）はどの回の前にもなかった（待たなかった）。利用者の project `go2` の `go2-gazebo-1`・`go2-app-1` には触れていない。
- 眠りの前の数える回と数えない回（M0 前・止まり、M3、M4、コーンの周回 1 の試み）の 110 の時点で、1 分の load は 1.52〜57.9、4 を超えたのは 90。
  controller の止め（23:52）の前は 1.52〜16.9（ほかの組の container は `docker stats` に出なかった。画面ありの Gazebo 約 4.3 vCPU）。止めの後はほかの組（レビュー・直し・文書）の container が
  並んだ：`go2-app-run-…`（main checkout の project で動く使い捨ての app の container）が 17〜857 %（23:55〜00:10。cone_lap1 の 6 回の試みの前後は
  どれも 491〜857 %）、`oo-perc2-rg13`・`oo-perc2-rg13-head` 180〜867 %（00:05〜00:10）。`pgrep -f pytest` は 110 の時点のうち 22 で 2〜6。R-F6b はレビューの組に 1 つのテストの
  ファイルだけを許しているが、8 コア近く使う container があった。
- 00:00 から、組を起動する前に、こちらのでない container が 300 % を超えて CPU を使う間は 30 秒ごとに待つ門（最長 15 分）を足した（cone_lap1 の
  「立つ」の error が 6 回続いたため。R-PAR の「load 4 未満」の待ちではない）。眠りの前に待ったのは 3 回（cone_lap2 の前 1 分、cone_lap3 の 1 回目の前 1.5 分、2 回目の前 1 分）。
- 00:47 から `sysctl vm.swapusage` も記録した：12,288 MB のうち 10,784〜10,996 MB を使っていた（メモリが足りず、外に書き出していた）。
- 電源：どの時点も AC、電池 100 %（charged）、`lowpowermode` 0。**00:11:15 に蓋が閉じて眠った**（上）。それより前（22:33〜00:11）に眠りはない。
- **2 回目の組（07:39〜08:28）：** `go2-gazebo-1` はどの回の前にもなかった。蓋はどの回の前も開いていた（`AppleClamshellState` = No）。`pmset -g log` の
  Sleep の数は 07:39 から 08:28 まで 1502 のままで、眠りはない。電源はどの時点も AC、電池 100 %（charged）、`lowpowermode` 0。50 の時点で 1 分の load は
  3.06〜17.51（4 を超えたのは 49。こちらの画面ありの Gazebo が 396〜456 %、画面なしが 194〜222 %、検出器が 3〜47 %）。ほかの組の container は 1 回だけ
  （`oo-docs-tests-71` 25.8 %）で、CPU の門で待った回はない。`pgrep -f pytest` は 50 の時点のうち 1 つで 2、ほかは 0。swap は 12 GB 前後のうち 9.1〜12.0 GB を使っていた。

### R-H5e：「立つ」の error（重い負荷の下）

「姿勢が計画と違います（計画: 立ち / 実際: 伏せ）」が 7 回出た：cone_lap1 の 1 回目の 3 回（23:58〜23:59、load 20.2〜21.9）と流し直しの 3 回（00:00〜00:01、
load 24.3〜61.7）、cone_lap3 の 1 回目（00:07、load 27.5）。どの回も、ほかの組の container（`go2-app-run-…`、`oo-perc2-rg13-head`）が合わせて 491〜857 % を使っていた。R-H5e のとおり
数えずにやり直した。cone_lap1 は 3 回ずつのやり直しを使い切ったので、1 回目の組では目標の式まで行っていない（2 回目の組で 1 回目として流した）。
H6 では 0 回、H6b の 2 回目の組でも 0 回（ほかの組の重い container がなかった）。

### M0 前：logical の `household` の RTF（D11 の基準。止めの前に流し終えていた）

H6 と同じ手順（起動し直した logical の Gazebo、Go2 は伏せたまま、20 秒の後に `rtf --seconds 75`）。この 6 回は並びの組が始まる前で、
始める前に load 4 未満・`pytest` なしを 60 秒ごとに待った（R-PAR の門。どの回も 3 分以内に開いた）。ほかの組の container は窓の始めにも終わりにもなかった。

| 回 | 窓の始め | 画面 | RTF（75 秒） | H6 | 始めの load（1・5・15 分） | 終わりの load | `go2oo-gazebo`（終わり） |
|---|---|---|---|---|---|---|---|
| m0b_on1 | 22:34:18 | あり | **0.92** | 0.87 | 2.33 2.05 3.59 | 5.27 3.04 3.82 | 422 %、1.738 GiB |
| m0b_on2 | 22:37:05 | あり | **0.91** | 0.85 | 3.79 2.95 3.70 | 5.66 3.81 3.97 | 425 %、1.252 GiB |
| m0b_on3 | 22:40:52 | あり | **0.92** | 0.89 | 3.96 3.81 3.94 | 5.49 4.26 4.09 | 417 %、1.245 GiB |
| m0b_off1 | 22:45:39 | なし | **1.00** | 1.00 | 3.17 4.40 4.25 | 4.22 4.46 4.28 | 196 %、640 MiB |
| m0b_off2 | 22:48:26 | なし | **1.00** | 0.99 | 2.93 3.84 4.05 | 4.60 4.12 4.13 | 200 %、639 MiB |
| m0b_off3 | 22:51:12 | なし | **1.00** | 1.00 | 2.02 3.40 3.85 | 4.50 3.91 4.01 | 202 %、641 MiB |

- 画面あり 0.91〜0.92（H6 0.85〜0.89）、画面なし 1.00（H6 0.99〜1.00）。

### M0 detector、止まり（`DETECTOR=yoloe`）

計画（`ddbfdbe`）のとおり R5 の手順（`F standing`、`F turned_right`、5 歩の式。コーンが見えて検出器は推論し続ける）の後に `rtf --seconds 75`
（Go2 は立ったまま）。H6 は R（右に回るだけ）で測った。始める前に load 4 未満・`pytest` なしを待った（止めの前、並びの組が始まる前。3 分以内）。

| 回 | 窓の始め | 画面 | RTF | H6 | `latency_ms_p50` / `p90` | 推論の数 | 始めの load | 終わりの load | 終わりの CPU（Gazebo / 検出器） | 着いた姿勢 |
|---|---|---|---|---|---|---|---|---|---|---|
| m0s_on1 | 22:56:10 | あり | **0.89** | 0.87 | 360.0 / 388.6 | 35 | 4.93 | 5.54 | 422 % / 5 %、927 MiB | (−0.407, −2.705, −99.0°) |
| m0s_on2 | 22:59:14 | あり | **0.89** | 0.86 | 369.8 / 376.9 | 35 | 5.27 | 7.85 | 424 % / 28 %、518 MiB | (−0.375, −2.657, −98.4°) |
| m0s_on3 | 23:03:19 | あり | **0.88** | 0.84 | 369.8 / 390.7 | 33 | 5.40 | 6.21 | 457 % / 5 %、525 MiB | (−0.234, −2.703, −95.4°) |
| m0s_off1 | 23:06:21 | なし | **0.99** | 0.99 | 356.9 / 366.6 | 34 | 5.36 | 3.56 | 213 % / 4 %、530 MiB | (−0.285, −2.679, −96.2°) |
| m0s_off2 | 23:09:24 | なし | **0.99** | 0.99 | 358.7 / 378.8 | 35 | 3.56 | 5.24 | 206 % / 4 %、528 MiB | (−0.417, −2.651, −98.7°) |
| m0s_off3 | 23:12:26 | なし | **0.99** | 0.99 | 358.2 / 362.6 | 34 | 4.02 | 4.08 | 202 % / 3 %、561 MiB | (−0.429, −2.684, −98.4°) |

- 画面あり 0.88〜0.89（H6 0.84〜0.87）、画面なし 0.99（H6 0.99）。M0 前（0.91〜0.92 / 1.00）との差は止まって推論し続ける検出器の損で、
  画面ありで 0.03、なしで 0.01（H6 0.01 / 0.01）。推論は p50 357〜370 ms、p90 363〜391 ms（H6 362〜394 / 368〜428 ms）。

### M3：検出の正しさ（設計書 §13.3、`objects_check.py accuracy`）

H6 と同じ（画面なし、姿勢ごとに起動し直した組。spawn は伏せで 1 回、`F standing` の後にもう 1 回。Bf は `--target sofa_1 suitcase_1`、
R5 は `--target traffic_cone_1`）。H6b では H6 の「`covers` のわけを探す回」をこの段に入れた：`/go2/objects` の 1 通ごとに `covers()` を
計算する読むだけの使い捨てのスクリプトを測りの間ずっと流し、Bf・R5 では `accuracy` を 3 回（1 回目が数える回、2・3 回目は 5 秒おきの確かめ）。
M3 の 6 回はどれも止めの前（23:13〜23:23）、load 4.1〜5.9。読めなかった 3 つの流し直しは 2 回目の組（07:57〜08:02、load 5.0〜15.1。下の「1 回目の外れ」の
H6 の順：数える `accuracy` を姿勢に着いた直後に始める）。

| 検出器 | 姿勢（x, y, 向き） | 組（正解 → 名前、位置の誤差、大きさの誤差、`covers`、`filled`、`fill_short_m`） | recall | 名前 | 誤り | 判定 | H6 |
|---|---|---|---|---|---|---|---|
| oracle | spawn 伏せ (0.004, 0.002, −1.5°) | **読めず**：`{"error": "no /gazebo/objects_truth"}`（終了コード 2、0.5 秒）。下の「1 回目の外れ」 | — | — | — | 流し直し（下） | 満たす（範囲に `detectable` なし） |
| oracle | spawn 立ち (0.027, −0.012, −1.4°) | box_1 → box 0.030・0.181・真・偽；chair_1 → chair 0.004・0.103・真・真 | n/a | 1.0 | 0 | 位置・大きさ・名前・誤りは満たす。範囲に `detectable` なし | 同じ（0.019・0.153；0.007・0.119） |
| oracle | Bf (−0.549, −0.058, −169.0°) | box_1 0.007・0.109・真；chair_1 0.008・0.113・真；**sofa_1 0.078**・0.047・**真**・`filled` **真**・0.097；**suitcase_1 0.025**・0.089・**真**・`filled` 偽 | 1.0 | 1.0 | 0 | **満たす、`covers` も真**（2・3 回目：ソファ 偽・真、下） | 線は満たす、`covers` 偽 1 つ（スーツケース） |
| oracle | R5 (−0.400, −2.738, −98.1°) | box_1 0.008・0.133・真；chair_1 0.007・0.116・真；suitcase_1 0.025・0.088・真；**traffic_cone_1 0.009**・0.121・**偽**・`filled` 真・0.0 | 1.0 | 1.0 | 0 | 線は満たす、`covers` 偽 1 つ（コーンの手前の面、R-H6a (b) の限り） | 同じ（コーン 0.024・偽） |
| yoloe | spawn 伏せ (0.004, 0.002, −1.5°) | **読めず**：oracle と同じ `no /gazebo/objects_truth`（0.5 秒） | — | — | — | 流し直し（下） | 満たす |
| yoloe | spawn 立ち (0.027, 0.039, −2.1°) | chair_1 → chair 0.006・0.111・真 | n/a | 1.0 | 0 | 満たす（範囲に `detectable` なし。box_1 は見つからない） | 同じ（0.008・0.124） |
| yoloe | Bf (−0.528, −0.038, −171.8°) | chair_1 0.004・0.103；**sofa_1 → sofa 0.077**・0.065・**真**・`filled` **真**・0.097；**suitcase_1 → suitcase 0.031**・0.093・真 | 1.0 | 1.0 | 0 | **満たす**（ソファの位置 0.077 m、線まで 0.073 m） | 満たす（ソファ 0.136 m・偽・`filled` 偽、線まで 0.014 m） |
| yoloe | R5 (−0.401, −2.702, −98.9°) | **読めず**：`no /gazebo/objects_truth`（終了コード 2、0.5 秒）。2・3 回目は読めた：chair_1 0.008；suitcase_1 0.036・偽；**traffic_cone_1 → traffic_cone 0.008・0.006**・0.067・0.074・偽・`filled` 真・0.014・0.012、recall 1.0・名前 1.0・誤り 0・合格 | — | — | — | **1 回目は外れ**（下）。流し直し（下） | 満たす（コーン 0.012） |
| oracle（流し直し） | spawn 伏せ (0.004, 0.002, −1.5°) | box_1 → box 0.030・0.181・真・偽；chair_1 → chair 0.004・0.103・真・真（`ready` まで 2.2 秒） | n/a | 1.0 | 0 | 位置・大きさ・名前・誤りは満たす。範囲に `detectable` なし（H6 と同じ判定） | 満たす |
| oracle（流し直し） | spawn 立ち (0.035, 0.044, −1.1°) | box_1 0.029・0.181；chair_1 0.004・0.105 | n/a | 1.0 | 0 | 同じ | 同じ |
| yoloe（流し直し） | spawn 伏せ (0.004, 0.001, −1.5°) | chair_1 → chair 0.006・0.109・真（`ready` まで 4.5 秒） | n/a | 1.0 | 0 | 満たす（範囲に `detectable` なし。box_1 は見つからない） | 満たす |
| yoloe（流し直し） | spawn 立ち (0.012, −0.021, −1.6°) | chair_1 0.008・0.120 | n/a | 1.0 | 0 | 満たす | 同じ |
| yoloe（流し直し） | R5 (−0.355, −2.781, −97.0°) | chair_1 0.006・0.108；suitcase_1 0.019・0.093・真；**traffic_cone_1 → traffic_cone 0.084**・0.037・**真**・`filled` 真・0.102（`ready` まで 3.8 秒。2・3 回目も同じ） | 1.0 | 1.0 | 0 | **合格** | 満たす（コーン 0.012） |

- oracle・yoloe とも `ids_retired` はどの姿勢も 0、`notes` は空（流し直しも同じ）。**yoloe の Bf で `bench` は出なかった**（状態の `ids_issued` は chair・sofa・suitcase 各 1。
  H6 は bench → sofa の名前の変わりで `ids_retired` 1）。推論は yoloe で p50 357〜359 ms、p90 362〜377 ms（H6 352〜357 / 377〜390）。
- `fill_short_m`：Bf のソファ 0.092〜0.097 m（oracle・yoloe とも。単語帳の奥行き 0.9 m で補った足跡が、モデルの 1.044 m に届かない分。C2 で別に記録する分）、
  R5 のコーン（yoloe）0.012〜0.014 m。ほかは 0。
- **R-H3k (2)：** yoloe でソファ 0.075〜0.077 m・スーツケース 0.031〜0.032 m（Bf）、コーン 0.006〜0.008 m（R5）。どれも名前は正しい。

### M3：R-H6a の直しの確かめ（ソファの奥）と `covers`

`/go2/objects` の 1 通ごとの `covers()`（読むだけのスクリプト。`covers` か辺が変わった通だけを書く）。「足りない分」は正解の足跡の辺から検出の辺まで、`COVER_TOL` は 1 mm。

| 回 | 物 | 通 | `covers` 真 | `filled` | 奥（Go2 から遠い面） | 手前・横の面でいちばん足りない所 | H6（わけを探した回） |
|---|---|---|---|---|---|---|---|
| oracle Bf | sofa_1 | 3 | 2 | 3/3 真 | 0.092〜0.097 m 足りないが `filled`（C2 の補った分） | +y の横の面 −2.0〜+2.5 mm（`COVER_TOL` の前後で入れ替わる） | 奥 0.218〜0.220 m 足りず `filled` 偽、`covers` 偽 3/3 |
| yoloe Bf | sofa_1 | 4 | 4 | 4/4 真 | 0.096〜0.097 m、`filled` | 横の面 +7〜+20 mm（足りる） | （M3 yoloe：偽・`filled` 偽・0.136 m） |
| oracle Bf | suitcase_1 | 6 | 6 | 偽 | 奥（−x）0〜16.5 mm 余る | 足りる | 奥 0〜1.5 mm（`COVER_TOL` の前後） |
| yoloe Bf | suitcase_1 | 2 | 2 | 偽 | 7.5〜36.5 mm 余る | +y 3.5 mm 余る | — |
| oracle R5 | traffic_cone_1 | 5 | 0 | 真 | 奥（−y）−1.0〜+13.5 mm（`filled`） | **手前（+y）2.5〜15 mm 足りない** | 手前 1.5〜14 mm（R-H6a (b) の限り） |
| yoloe R5 | traffic_cone_1 | 5 | 0 | 真 | 3〜14.5 mm 足りない（`filled`） | 手前（+y）19.5〜47 mm、横（−x）最大 48 mm・（+x）最大 6 mm 足りない | — |
| yoloe R5（流し直し） | traffic_cone_1 | 4 | 3 | 3/4 真 | 奥（−y）0.102 m 足りないが `filled`（C2 の補った分） | 足りる（手前 +0.066 m、横 +0.014〜+0.024 m） | — |

- **R-H6a (a) の直しは効いた：** Bf のソファは oracle・yoloe のどの通でも `filled` 真になり、奥は単語帳の大きさで補われた（足りない分 0.092〜0.097 m は
  C2 の `fill_short_m`）。H6 の「`filled` 偽のまま奥が 0.22 m 欠ける」は出なかった。位置の誤差は oracle 0.075〜0.078 m（H6 の探す回 0.138 m）、
  yoloe 0.075〜0.077 m（H6 0.136 m）。
- 残る `covers` 偽は (1) oracle の Bf のソファの +y の横の面の 2 mm（3 通のうち 1 通。`COVER_TOL` 1 mm の前後で入れ替わる。H6 のスーツケースの (c) と
  同じ形）と、(2) R5 のコーンの手前の面（oracle 2.5〜15 mm。R-H6a (b) で v1 の限り）。どちらも 0.05 m の間引きの段より小さい。
  **(1) は R-H6f の v1 の限り**（数は上のとおり、わけは横の面の辺が正解の辺に重なる所で `COVER_TOL` をまたぐこと）。
- 流し直しの yoloe の R5 のコーンの 1 通目の `covers` 偽は、辺が 2〜4 通目と同じで `filled` だけが偽：探すスクリプトが `filled` を 1 Hz の状態から取り、
  2 Hz の一覧と組にしたための遅れ（下の「H6 の記録の訂正」(1) と同じ形）。数える `accuracy` の読みは 3 回とも `covers` 真。
- yoloe の R5 のコーンは手前の面で最大 47 mm、横で最大 48 mm 足りない（わけは探していない）。`covers` は yoloe では
  線にない（H6 と同じ）。位置の誤差は 0.006〜0.008 m、大きさの誤差 0.067〜0.074 m で、線（0.15 m・0.4 m）の内。

### M3 の 1 回目の外れ（yoloe の R5、spawn の伏せ）とわけ

- **何が出たか：** yoloe の R5 の数える `accuracy` は `{"error": "no /gazebo/objects_truth", "ready": true, "waited_s": 0.5}`（終了コード 2、データなし）で、
  どの数も null。同じ形が oracle と yoloe の spawn の伏せの読みでも出た。同じ組の 2・3 回目（5 秒おき）は読めて、どれも線を満たした。
- **わけ（測りの道具の競り合い。製品のコードではない）：** `accuracy` は「Go2 が止まっていて、`/go2/detector_status` の `still_inferences` が 3 以上」に
  なったら読み終える（`objects_check._accuracy` の `ready()`）。正解の `/gazebo/objects_truth` は 2 Hz で、`ready()` はその 1 通目を待たない。
  H6b の M3 は `covers` を探すスクリプトを先に始め、3 秒待ち、姿勢を読んでから `accuracy` を始めた。そのときには検出器はもう止まって 3 回以上
  推論していたので、`ready()` は最初の状態の 1 通（0.5 秒）で真になり、正解の 1 通目が届く前に読み終えた。H6 は `accuracy` を姿勢に着いた直後に
  始めたので、推論 3 回を本当に待ち（1.5〜4.5 秒）、正解はいつも届いた（8 回とも）。H6b でも 0.5〜1.1 秒で終えた読み（2・3 回目など）は、
  正解が届いたかどうかの運で決まった。
- **spawn の判定は H6 と同じか：** 同じ。spawn の立ちの読みは H6 も H6b も `passed: false`・`failures: ["min_recall: no detectable truth in range"]`・
  `recall: null` で、範囲（`--range`）の中の正解が box_1 と chair_1 だけ（どちらも detector の目標ではない、Q4）なので recall がない。H6 はこれを
  「recall は n/a、位置・大きさ・名前・誤りの線は満たす」と判定し、計画（brief の M3）の線も同じ（oracle の位置と大きさの線、`covers`）。H6b も同じに判定した。
  だから spawn の「FAILED」は外れではない。外れは伏せの読みが読めなかったことだけ。
- **流し直し（R-F6c、1 回だけ）：** 数える `accuracy` を H6 と同じく姿勢に着いた直後に始め、`covers` のスクリプトはそれと同時に始めた（待ちなし）。
  手順・線は同じ。1 回目の組の yoloe の R5 の流し直し（01:47〜02:05）は合格だったが、5 歩の式の間に Mac が眠った（16 分）ので数えず（R-H5b）、
  oracle の spawn の流し直しは伏せの読みの後で止めた（眠り）。**2 回目の組で 3 つとも流し直した（07:57〜08:02）：** yoloe の R5 は合格（コーン 0.084 m・
  `covers` 真、`ready` まで 3.8 秒）、oracle と yoloe の spawn の伏せは読めて（`ready` まで 2.2・4.5 秒）、位置・大きさ・名前・誤りの線を満たした（上の表）。
  どの読みも正解は届いた（`no /gazebo/objects_truth` は 0 回）。だから (1) は測りの道具の限りで、M3 の線は流し直しで満たす。

### M4：式の段（設計書 §13.4・§13.5、`DETECTOR=yoloe`、画面あり）

H6 と同じ手順（回ごとに起動し直した組、試験の姿勢の手順、2 つの `resolve`、目標の式 `objects_check.py run --expect near|lap --target <正解の id> "F near|around_<id>"`）。
計画（`ddbfdbe`、R-H6c）のとおり、「待ち」は回る式の前に始めた `resolve`（時の `resolve`）の `listed_after_still_s`、式に渡す id は姿勢に着く式の前に
始めた `resolve`（id の `resolve`）のもの。どの回も検出器の id は正解と同じ番号になった。手順の `run_formula`（立つ・回る・前に出る）と 2 つの
`resolve` の終了コードはどの回も 0（R-H5e の「立つ」の error でやり直した回を除く）。列は H6 と同じ（「前」は「前方に物が出た」の数、RTF は
`rtf_stats` の歩く間の中央値。手が 1 つのコーンの手前には窓がないことがある）。表の下の 5 行は 2 回目の組（07:39〜07:51）：コーンの周回 lap1・lap3 の
1 回目と、推論の時間で外れた 3 回の流し直し（R-F6c の 2 回目。1 回目の組の suitcase_near2 の流し直しは眠りで数えないので、これが 2 回目）。

| 回 | 1 回目か | 起動 | 着いた姿勢 | 待ち | 状態（前 / 後、p50 / p90 ms） | 判定 | 線の値 | 傾き | 手 | 前 | RTF（歩く） | 行（大きさ、`confidence`） | load（前 → 式の始め → 後） |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| sofa_near1 | 1 回目 | 23:23 | (−0.546, −0.115, −172.1°) | 3.45 | 380 / 413、384 / 417 | 合格 | 縁 1.046 m・0.0°・どの足跡にも 0.690（suitcase_1） | 0.072 | 5 | 0 | 0.862 | 1.002 × 1.904、0.752 | 5.70 → 6.89 → 6.21 |
| sofa_near2 | 1 回目 | 23:24 | (−0.501, −0.148, −167.4°) | 2.87 | 375 / 416、368 / 397 | 合格 | 縁 1.129 m・1.8°・どの足跡にも 0.538（suitcase_1） | 0.046 | 5 | 0 | 0.856 | 1.005 × 1.854、0.833 | 6.35 → 7.16 → 6.25 |
| sofa_near3 | 1 回目 | 23:26 | (−0.484, −0.046, −171.2°) | 3.17 | 368 / 425、365 / 388 | 合格 | 縁 1.121 m・0.0°・どの足跡にも 0.706（suitcase_1） | 0.237 | 5 | 0 | 0.844 | 1.016 × 1.903、0.812 | 5.75 → 5.99 → 6.26 |
| sofa_lap1 | 1 回目 | 23:27 | (−0.492, −0.061, −170.5°) | 2.73 | 377 / 420、384 / 406 | 合格 | 巻き数 1.165・縁 0.801 m（どの足跡にも 0.545、suitcase_1） | 0.037 | 56 | 1 | 0.865 | 1.002 × 1.854、0.796 | 5.92 → 6.17 → 6.77 |
| sofa_lap2 | 1 回目 | 23:31 | (−0.523, −0.105, −171.5°) | 2.87 | 372 / 431、373 / 410 | 合格 | 巻き数 0.919・縁 0.839 m（どの足跡にも 0.501、suitcase_1） | 0.067 | 55 | 1 | 0.861 | 1.003 × 1.886、0.811 | 6.31 → 6.24 → 7.17 |
| sofa_lap3 | 1 回目 | 23:35 | (−0.544, −0.102, −171.4°) | 3.11 | 373 / 405、380 / 427 | 合格 | 巻き数 0.917・縁 0.819 m（どの足跡にも 0.524、suitcase_1） | 0.042 | 49 | 1 | 0.873 | 1.003 × 1.835、0.746 | 7.17 → 9.09 → 7.08 |
| suitcase_near1 | 1 回目 | 23:39 | (−0.544, −0.137, −169.1°) | 3.05 | 362 / 409、383 / 427 | 合格 | 縁 1.016 m・2.8°・どの足跡にも 1.006（suitcase_1） | 0.046 | 3 | 0 | 0.814 | 0.532 × 0.347、0.392 | 7.08 → 6.65 → 6.53 |
| suitcase_near2 | 1 回目 | 23:40 | (−0.536, −0.116, −170.9°) | 4.20 | 381 / 396、530 / 910 | 合格 | 縁 0.885 m・26.1°・どの足跡にも 0.871（suitcase_1） | 0.064 | 2 | 0 | 0.734 | 0.529 × 0.368、0.669 | 6.17 → 16.90 → 13.56 |
| suitcase_near3 | 1 回目 | 23:42 | (−0.516, −0.092, −172.3°) | 1.27 | 440 / 564、432 / 516 | 合格 | 縁 0.925 m・24.2°・どの足跡にも 0.912（suitcase_1） | 0.056 | 2 | 0 | 0.780 | 0.538 × 0.399、0.483 | 13.56 → 11.15 → 9.87 |
| suitcase_lap1 | 1 回目 | 23:43 | (−0.524, −0.083, −173.7°) | 3.36 | 432 / 477、439 / 534 | 合格 | 巻き数 1.033・縁 0.951 m（どの足跡にも 0.951、suitcase_1） | 0.033 | 28 | 0 | 0.852 | 0.535 × 0.367、0.653 | 9.16 → 8.36 → 6.08 |
| suitcase_lap2 | 1 回目 | 23:46 | (−0.534, −0.010, −173.6°) | 2.54 | 410 / 516、424 / 498 | 合格 | 巻き数 1.032・縁 0.955 m（どの足跡にも 0.955、suitcase_1） | 0.044 | 28 | 0 | 0.844 | 0.569 × 0.397、0.550 | 6.55 → 7.23 → 7.44 |
| suitcase_lap3 | 1 回目 | 23:48 | (−0.537, −0.095, −173.0°) | 4.33 | 413 / 492、385 / 425 | 合格 | 巻き数 1.034・縁 0.918 m（どの足跡にも 0.918、suitcase_1） | 0.052 | 26 | 0 | 0.837 | 0.537 × 0.403、0.460 | 7.24 → 6.66 → 7.02 |
| cone_near1 | 1 回目 | 23:53 | (−0.394, −2.635, −98.8°) | 2.72 | 395 / 476、408 / 476 | 合格 | 縁 1.009 m・0.0°・どの足跡にも 1.008（traffic_cone_1） | 0.039 | 4 | 0 | 0.850 | 0.555 × 0.476、0.400 | 6.75 → 7.81 → 11.88 |
| cone_near2 | 1 回目 | 23:55 | (−0.343, −2.755, −96.8°) | 3.08 | 430 / 536、588 / 893 | 合格 | 縁 0.959 m・0.0°・どの足跡にも 0.957（traffic_cone_1） | 0.015 | 1 | 0 | 0.551 | 0.579 × 0.580、0.657 | 12.77 → 9.77 → 11.80 |
| cone_near3 | 1 回目 | 23:56 | (−0.326, −2.712, −97.5°) | 3.54 | 464 / 663、572 / 706 | 合格 | 縁 0.993 m・0.0°・どの足跡にも 0.988（traffic_cone_1） | 0.081 | 1 | 0 | — | 0.576 × 0.486、0.643 | 12.30 → 10.41 → 13.77 |
| cone_lap2 | 1 回目 | 00:01 | (−0.290, −2.677, −96.5°) | 3.00 | 411 / 571、661 / 788 | 合格 | 巻き数 1.132・縁 0.913 m（どの足跡にも 0.913、traffic_cone_1） | 0.073 | 29 | 5 | 0.465 | 0.595 × 0.518、0.414 | 57.90 → 28.72 → 42.73 |
| cone_lap3（1 回目の組の 2 回目） | **数えない**（立つの error 1 回の後の 2 回目。目標の式の途中 00:11:15 から Mac が眠った、R-H5b） | 00:08 | (−0.251, −2.697, −96.0°) | 3.96 | 535 / 740、537 / 595 | 合格 | 巻き数 1.135・縁 0.948 m（どの足跡にも 0.948、traffic_cone_1） | 0.061 | 31 | 12 | 0.508 | 0.595 × 0.498、0.655 | 31.39 → 17.57 → 100.92 |
| cone_lap1（1 回目の組） | **試せず** | 23:58〜00:01 | — | — | — | 「立つ」が R-H5e の error で 6 回（1 回目の 3 回と流し直しの 3 回）。目標の式まで行っていない | — | — | — | — | — | — | 20.15 → … → 61.67 |
| cone_lap1 | 1 回目（2 回目の組） | 07:39 | (−0.349, −2.642, −98.1°) | 2.99 | 374 / 415、370 / 425 | 合格 | 巻き数 1.100・縁 0.917 m（どの足跡にも 0.917、traffic_cone_1） | 0.127 | 28 | 0 | 0.839 | 0.587 × 0.478、0.511 | 4.68 → 7.70 → 11.04 |
| cone_lap3 | 1 回目（2 回目の組） | 07:42 | (−0.326, −2.701, −97.6°) | 3.04 | 387 / 445、374 / 405 | 合格 | 巻き数 1.103・縁 0.933 m（どの足跡にも 0.933、traffic_cone_1） | 0.047 | 28 | 0 | 0.847 | 0.566 × 0.459、0.721 | 10.63 → 9.41 → 10.08 |
| cone_near2 | 2 回目（推論の時間、R-F6c） | 07:45 | (−0.255, −2.640, −95.5°) | 2.87 | 372 / 429、367 / 389 | 合格 | 縁 1.070 m・0.0°・どの足跡にも 1.067（traffic_cone_1） | 0.016 | 1 | 0 | — | 0.558 × 0.453、0.431 | 10.03 → 9.86 → 8.60 |
| cone_lap2 | 2 回目（推論の時間、R-F6c） | 07:47 | (−0.253, −2.713, −95.5°) | 3.08 | 375 / 427、394 / 438 | 合格 | 巻き数 1.135・縁 0.846 m（どの足跡にも 0.846、traffic_cone_1） | 0.047 | 29 | 0 | 0.875 | 0.571 × 0.504、0.820 | 8.64 → 10.09 → 6.25 |
| suitcase_near2 | 2 回目（推論の時間、R-F6c） | 07:49 | (−0.533, −0.048, −172.1°) | 7.09 | 369 / 428、371 / 422 | 合格 | 縁 0.957 m・24.2°・どの足跡にも 0.944（suitcase_1） | 0.038 | 2 | 0 | 0.819 | 0.546 × 0.386、0.385 | 5.51 → 7.18 → 7.25 |

### 待ちの測り方と結果（設計書 §13.5、R-H6c）

- 計画（`ddbfdbe`）のとおり、時の `resolve` を回る式の前に、id の `resolve` を姿勢に着く式の前に始めた。1 回目の 18 回（1 回目の組 16、2 回目の組の
  コーンの周回 2）の時の `resolve` は 1.27〜4.33 秒（H6 1.08〜4.42 秒）で、どれも 5 秒以内。1 回目の組の id の `resolve` の `listed_after_still_s` は
  16 回のうち 12 回 null（目標はもう一覧にあった、H6 と同じ）。
- 流し直し（2 回目）の 3 回：cone_near2 2.87 秒、cone_lap2 3.08 秒、**suitcase_near2 7.09 秒（5 秒の線の外れ）**。わけは下の「待ちが 5 秒を超えた回」。
  M0 detector の画面なし 2.69・2.80 秒、声の段の準備 2.63〜3.48 秒（ソファの名前の変わりの 1 回 6.89 秒を除く、下）。
- 時の `resolve` の標準エラーに「pairs with」（受け入れない名前で組になった行）はどの回もなく、**ソファの bench → sofa の名前の変わりは出なかった**
  （H6 は 3 回）。ただし suitcase_near3 では、目標でないソファが Bf で `bench_1`（0.498 × 1.867）として一覧にあり、そのまま残った（目標でないので
  線には効かない。名前の誤りとして記録）。2 回目の組の M4 と M0 でも「pairs with」はなかった。声の段の準備の 1 回（v_sofa_near）にだけ出た（下）。

### M4 のほかの記録

- **ソファの周回（R-H6b）：** M4 は 1 回目で 3/3。どの回も「前方に物が出た」が 1 回あり、すぐ消えて続けた。周回の道でいちばん近づいたのはスーツケース
  （0.501〜0.545 m）で、H6 で止まった 0.48〜0.53 m の所と同じ辺り。H6 の 2 回のような 15 秒の中止はなかった（R-F6d (b) の「計画がない」も出なかった：
  着いた向きは −170.5〜−171.5°）。声の段の 1 回目は、同じ所で 15 秒止まって中止した（下の「声の段」）。
- **スーツケース（R-H3j、分けて書く）：** Bf から手前 3/3（縁まで 0.885〜1.016 m、足跡に 2.8〜26.1°）、周回 3/3（巻き数 1.032〜1.034、縁まで 0.918〜0.955 m）。
  名前はどの回も `suitcase`。測った大きさは 0.529〜0.569 × 0.347〜0.403（H6 0.532〜0.562 × 0.369〜0.401）。推論の時間の外れが 1 回（suitcase_near2、下）。
  その流し直し（2 回目の組）は線を満たした（縁 0.957 m・24.2°、後の状態 p50 371 / p90 422 ms）が、待ちが 7.09 秒だった（下）。
- **コーン（R-H3k、分けて書く）：** R5 から手前 3/3（縁まで 0.959〜1.009 m、0.0°）、周回 3/3（1 回目。巻き数 1.100〜1.132・縁 0.913〜0.933 m。lap1 と lap3 は
  2 回目の組）。流し直しの 2 回も合格（near2 縁 1.070 m、lap2 巻き数 1.135・縁 0.846 m）。行の `confidence` は 0.400〜0.820（R-H3k (3)。H6 0.425〜0.708）。
  測った大きさは 0.555〜0.595 × 0.453〜0.580（底 0.503 × 0.503）。
  cone_near1 の計画は 4 手（左に回る から。H6 はどれも 1 手）。cone_lap2 は load 28.7〜57.9 の下で、センサーの途切れの一時停止が 5 回、前の物が 5 回あり、
  どれも続けた。歩く RTF 0.465。2 回目の組のコーンの 4 回には、センサーの途切れも前の物の一時停止もなかった（歩く RTF 0.839〜0.875）。
- **推論の時間の外れ（3 回、1 回目）：** suitcase_near2 の目標の式の後の状態で p50 530 / p90 910 ms（式の始めの load 16.9、後 13.6）、cone_near2 の後で
  p50 588 / p90 893 ms（load 9.8 → 11.8、ほかの組の container 509 %）、cone_lap2 の後で p50 661 / p90 789 ms（load 28.7 → 42.7、ほかの組の container 462 %）。式の前の状態は
  どれも線の内。わけは Mac の負荷（推論は 1 スレッドの CPU）。H6（load 5〜10）では p90 は 525 ms 以下だった。R-F6c の流し直しは suitcase_near2 だけ
  流せたが、それは眠りの中（02:30）で数えない（p50 412 / p90 509 ms、線の内、合格）。
- **推論の時間の流し直し（R-F6c の 2 回目、2 回目の組）：** 3 回とも線の内。後の状態は cone_near2 p50 367 / p90 389 ms（load 9.9 → 8.6）、
  cone_lap2 p50 394 / p90 438 ms（load 10.1 → 6.3）、suitcase_near2 p50 371 / p90 422 ms（load 7.2 → 7.3）。前の状態も 372〜375 / 427〜429 ms。
  2 回目の組の 1 回目（コーンの周回 2 回）と M0 の 2 回、声の段の 14 の読みも線の内（p50 358〜443、p90 373〜636 ms）。だから 1 回目の 3 回の外れは、
  ほかの組の重い container があった時の Mac の負荷のわけで、**R-H6f の v1 の限り：重い負荷（1 分の load 10〜40、ほかに 4〜8 コアを使う container）の下では
  推論の時間が線を超える（p90 893〜910 ms、p50 661 ms）。**
- **名前の誤り（目標でない物、線に効かない）：** sofa_lap1 でソファの一部が `television_1`（1.124 × 0.839）、sofa_lap3 でテーブル（logical だけ）が
  `stool_1`（0.713 × 0.498）として一覧に入った（どちらも周回の後の一覧）。2 回目の組でも、ソファの周回の後にテーブルが `stool_1`（M0 の画面なしの周回
  0.707 × 0.496、声の段の流し直し「スツール（前 5.5 m）」）。sofa_near3（`ids_issued` に box）と suitcase_near1（bench）で `ids_retired` 1。
- 着いた姿勢：Bf 12 回 x −0.546〜−0.484、y −0.148〜−0.010、向き −173.7〜−167.4°。R5 4 回 x −0.394〜−0.290、y −2.755〜−2.635、向き −98.8〜−96.5°。
  R-H6d の後の `MEASURED_RANGES`（Bf x −0.560〜−0.453・y −0.142〜−0.024・向き −174.0〜−165.5°、R5 x −0.492〜−0.122・y −2.877〜−2.631・
  向き −100.5〜−93.5°）の外に出たのは Bf の 2 回：sofa_near2 の y −0.148（0.006 m 外）と suitcase_lap2 の y −0.010（0.014 m 外）。どちらも
  目標の式は計画ができて線は通った。M3 の姿勢と眠りで数えない回の姿勢は、どれも内。
  2 回目の組の姿勢：Bf 8 回（M4 の流し直し、M0 の 2 回、声の段の準備 5 回）x −0.537〜−0.500、y −0.124〜−0.023、向き −172.1〜−168.3°、R5 7 回
  （M4 4 回、M3、声の段の準備 2 回）x −0.355〜−0.253、y −2.781〜−2.640、向き −98.1〜−95.5°。外に出たのは声の段の v_suitcase_near の y −0.023（0.001 m 外）
  だけで、計画ができて合格した。固定は変えていない（controller へ）。

### 眠りで数えない回（R-H5b。データは作業用のフォルダに残した）

| 回 | 時刻 | 何が眠りの中か | 結果（記録だけ） |
|---|---|---|---|
| cone_lap3（2 回目） | 00:08:58〜00:37:40 | 目標の式（00:10:05〜）の途中 00:11:15 から | 合格（巻き数 1.135・縁 0.948 m）。センサーの途切れの一時停止 12 回、歩く RTF 0.508 |
| m0w_sofa_near（M0 歩く、画面なし） | 00:38〜00:54 | 回る式（16 分かかった） | 合格（縁 1.092 m・0.0°・どの足跡にも 0.673）、歩く RTF 0.932 |
| m0w_sofa_lap（同） | 00:55〜01:47 | 立つ・前に出る（5 分ずつ）、目標の式（41 分） | 合格（巻き数 0.919・縁 0.855 m）、歩く RTF 0.897（`/clock` が届いた 15 の窓だけ） |
| m3_yoloe_R5 の流し直し | 01:47〜02:05 | 5 歩の式（16 分） | 合格：recall 1.0、名前 1.0、誤り 0、コーン 0.028 m・0.129 m・`covers` 真（`ready` まで 3.8 秒、正解は届いた） |
| M5 | 02:05〜02:29 | 箱を置いた後の読み（02:13〜02:29 の 16 分） | 箱は置けた（`/sim/world` の箱 [1.2, −0.042, 0.3]）が、6 回の一覧は `chair_1` だけで `box` の行はなかった |
| suitcase_near2 の流し直し | 02:29〜02:46 | 手順の container の起動 | 合格（縁 0.892 m・27.6°）、後の状態 p50 412 / p90 509 ms |
| m3_oracle_spawn の流し直し | 02:46〜02:47 | — | 伏せの読みが終わったところで止めた |

- M5 の Gazebo のログ（段ボールの絵の「Could not resolve file」が出るかの確かめ）は残っていない：止める時のログの保存が、M5 の組が下りた後に
  走って、M5 のフォルダのログを消した（こちらの手順の誤り）。2 回目の組の M5 はログを組を下ろす前に保存して確かめた（下の「M5」）。
- 表の回は、2 回目の組でどれも流し直した（コーンの周回 lap3・M0 の画面なし 2 回・M5 は 1 回目として、M3 と suitcase_near2 は R-F6c の 2 回目として）。

### 待ちが 5 秒を超えた回（suitcase_near2 の 2 回目、R-F6c）とわけ

- 2 回目の組の suitcase_near2（推論の時間の流し直し）で、時の `resolve` も id の `resolve` も 7.09 秒（名前の変わりの「pairs with」はない）。
  記録係（`rec.jsonl`、止まりの変わり目、状態、行の届いた時刻）：
  - B に止まった（07:50:32.0）後、止まりの推論 3 回でソファは一覧に入った（34.8）が、スーツケースは入らなかった。
  - Bf に止まった（07:50:42.2）後の止まりの推論は 44.3・45.3・46.3（1 秒おきに 3 回）で、スーツケースは一覧に出なかった。検出器は止まりの推論 3 回の
    後は休みの間（`DET_IDLE_PERIOD_S` 3 秒）ごとに 1 回なので、4 回目は 49.3。スーツケースの行はそのとき出た（49.307、止まってから 7.09 秒）。
- わけ：トラッカーは仮の追跡を、見える絵 3 回のうち 0.5 秒以上離れた当たり 2 回（1 回はその label の `confirm` 0.35 以上）で確かにする（`detector/tracker.py`）。
  Bf のスーツケースはカメラから +38〜+45°（R-H3j。絵の端に近い）にあり、点は確かめの線の近くにある（この回の行の `confidence` 0.385。M4 の 1 回目の 6 回は
  0.392〜0.669）。絵ごとの点は記録していないが、Bf の最初の 3 回の推論で確かにならず、4 回目で確かになったことは上の時刻のとおりで、4 回目が 3 秒の休みの
  後なので 7 秒になった。M4 の 1 回目の 6 回と声の段の準備の 2 回は 5 秒以内（1.27〜4.33 秒、3.48・2.96 秒）。
- 判定：R-F6c の 2 回目の外れで、R-H6f のとおり **v1 の限り：Bf のスーツケースは絵の端の近くで点が確かめの線の近くにあるので、止まってから一覧に出るまでが
  5 秒を超えることがある（M4 のスーツケースの 7 回のうち 1 回、7.09 秒）。** 安全の問題ではない（一覧に出る前の指示は、その物が見つからないとして断る）。

### M0 detector、歩く、画面なし（2 回目の組。最初の種類：ソファ）

M4 と同じ手順で、Gazebo の画面なし（`gui:=false`）。目標の式の間、`/model/robot/odometry` を 50 Hz で全部記録した（roll・pitch・胴の高さ・姿勢・速さ）。

| 回 | 1 回目か | 起動 | 着いた姿勢 | 待ち | 状態（前 / 後、p50 / p90 ms） | 判定 | 線の値 | 傾き | 手 | 前 | RTF（歩く） | 行（大きさ、`confidence`） | load（前 → 式の始め → 後） |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| m0w_sofa_near | 1 回目 | 07:51 | (−0.537, −0.043, −171.5°) | 2.69 | 360 / 416、358 / 392 | 合格 | 縁 1.063 m・0.0°・どの足跡にも 0.741（suitcase_1） | 0.053 | 5 | 0 | **0.949** | 1.003 × 1.863、0.808 | 7.07 → 5.57 → 5.60 |
| m0w_sofa_lap | 1 回目 | 07:53 | (−0.537, −0.101, −171.2°) | 2.80 | 361 / 436、394 / 536 | 合格 | 巻き数 0.918・縁 0.821 m（どの足跡にも 0.520、suitcase_1） | **0.051** | 55 | 1 | **0.951** | 1.003 × 1.865、0.758 | 5.23 → 4.74 → 17.51 |

- 歩く RTF：画面なし 0.949・0.951（H6 0.910・0.938）、画面ありの M4（2 回目の組）0.819〜0.875。Gazebo の CPU は画面なし 194〜222 %、画面あり 396〜456 %。
- **傾き（最終レビューの safety-2）：** 画面なしの周回は 0.051、手前は 0.053。**H6 の m0w_sofa_lap の 0.410 は出なかった**（0.3 rad を超えなかったので、
  わけを探す段は要らない）。記録した 8,427 の試料（周回、168 秒）で 0.2 rad を超えた時間は 0 秒、胴の高さは 0.333〜0.344 m（立った高さ、下がらない）。
  周回でいちばん傾いたのは roll 0.051（前進の途中、ソファの足跡から 1.66 m）。前の物の一時停止が 1 回あり、すぐ続けた。
- 2 回目の組で記録した 7 つの歩く回（M4 5、M0 2）の最大は 0.127（cone_lap1。最初の前進の途中の roll、コーンの足跡から 1.41 m、0.08 秒で 0.1 未満に
  戻った）。1 回目の組の最大 0.237（sofa_near3）は記録を足す前で、いつ・どの手かはわからない。H6 の 0.410 も、`objects_check` が最大しか残さないので、
  H6 のデータからはわけを探せない。H6b のどの回も 0.237 以下で、倒れる様子（傾きが続いて増える、胴が下がる）はなかった。

### 声の段（設計書 §13.4、O26、計画の I6 の裁定。2 回目の組）

- 計画の上書きのファイルの方法で、H6 と同じ（app は main checkout の場所から project 名 `go2oo` で起動、上書きのファイルで worktree を `/work` に、
  `LLM_BACKEND: claude`。ファイルは作業用のフォルダにあり、git に入れていない。compose が main checkout の `env.txt` を `env_file` で読む。だれも開いていない。
  `GA config` は流していない）。始める前の確かめ `GA run --rm --no-deps -T app python3 -c "import voice_ltl.vocabulary as v; print(v.__file__, len(v.VOCABULARY.entries))"`
  は `/work/voice_ltl/vocabulary.py 50`（08:04）。ログインの言葉はどの回の app のログにも、画面にも出なかった。
- 指示ごとに：起動し直した detector の組（yoloe、画面あり）で、M4 と同じ姿勢の手順と 2 つの `resolve`（app を起動する前）、`GA up -d --no-build --no-deps app`、
  http://localhost:8811 のページで「姿勢 立ち」と目標が「見えている物」にあるのを確かめ、`objects_check.py watch --expect near|lap --target <正解の id>` を始め、
  ページの「文字で指示する（試験用）」の欄に指示を入れて「送る」（Browser の画面）。式は app のログと画面の「ロジック（LTLf）」から、メッセージは画面から写した。
  最後に `G … down`（app も消える）。どの回も、蓋の門・D11 の門・CPU の門を通り、`pmset` の Sleep の数は変わらなかった。

| 指示 | 起動 | 準備の姿勢・待ち | 「見えている物」（送る前） | Claude の式 | 画面のメッセージ | `watch` の判定と値 | RTF（歩く / 止まり） | load（始め / 後） |
|---|---|---|---|---|---|---|---|---|
| 「ソファまで歩いて」 | 08:05:01 | (−0.518, −0.090, −170.8°)、6.89 秒（bench → sofa の名前の変わり、下） | スーツケース（左前 1.5 m）、ソファ（前 4.0 m） | `F near_sofa_1`（1 回目で合格、「Claude が翻訳 · 単語帳と文法の検査に合格」、試行 1 回） | 「終わりました」「前進 → 前進 → 前進 → 前進 → 前進 を、計画どおりに実行しました。」 | **合格**：縁 1.082 m・0.0°・どの足跡にも 0.676（suitcase_1）、傾き 0.024 | 0.859 / 0.851 | 3.06 / 5.24 |
| 「ソファの周りを回って」（1 回目） | 08:07:37 | (−0.516, −0.124, −168.8°)、3.06 秒 | 同じ | `F around_sofa_1`（1 回目で合格、「こう受け取りました：「周りを回って」→ 左回りに 1 周」） | 「できませんでした」「続けられませんでした」「15 秒待っても 前方に障害物があります。」（47 手を済ませ、次の前進で止まった。前方 0.48 m） | **中止**（画面）。`watch` は一時停止の 10 秒の止まりで終わり、そこまでで巻き数 0.912・縁 0.863 m・どの足跡にも 0.443（suitcase_1）、傾き 0.073 | 0.849 / 0.802 | 4.10 / 7.17 |
| 「ソファの周りを回って」（流し直し、R-F6c） | 08:12:42 | (−0.500, −0.070, −170.7°)、2.63 秒 | 同じ | `F around_sofa_1`（同じ置き換えの言葉） | 「終わりました」「前進 → …（51 手）… → 前進 を、計画どおりに実行しました。」 | **合格**：巻き数 0.919・縁 0.795 m・どの足跡にも 0.550、傾き 0.042 | 0.854 / 0.817 | 5.87 / 7.40 |
| 「スーツケースまで歩いて」 | 08:17:30 | (−0.512, −0.023, −170.7°)、3.48 秒 | スーツケース（左前 1.6 m）、ソファ（前 4.0 m） | `F near_suitcase_1`（1 回目で合格） | 「終わりました」「前進 → 左に回る を、計画どおりに実行しました。」 | **合格**：縁 0.978 m・27.1°・0.968、傾き 0.048 | 0.807 / 0.825 | 6.21 / 7.25 |
| 「スーツケースの周りを回って」 | 08:19:45 | (−0.522, −0.086, −168.3°)、2.96 秒 | スーツケース（左前 1.5 m）、ソファ（前 4.0 m） | `F around_suitcase_1`（1 回目で合格、同じ置き換え） | 「終わりました」「右に回る → …（26 手）… → 前進 を、計画どおりに実行しました。」 | **合格**：巻き数 1.037・縁 0.941 m、傾き 0.038 | 0.851 / 0.828 | 6.37 / 5.67 |
| 「コーンまで歩いて」 | 08:23:08 | (−0.280, −2.675, −96.1°)、3.02 秒 | コーン（前 1.8 m） | `F near_traffic_cone_1`（1 回目で合格） | 「終わりました」「前進 を、計画どおりに実行しました。」 | **合格**：縁 1.035 m・0.0°・1.034、傾き 0.034 | 0.860 / 0.852 | 6.21 / 6.97 |
| 「コーンの周りを回って」 | 08:25:11 | (−0.274, −2.699, −96.0°)、2.98 秒 | コーン（前 1.8 m） | `F around_traffic_cone_1`（1 回目で合格、同じ置き換え） | 「終わりました」「右に回る → …（29 手）… → 前進 を、計画どおりに実行しました。」 | **合格**：巻き数 1.133・縁 0.882 m、傾き 0.045 | 0.863 / 0.870 | 6.35 / 7.02 |

- **1 回目の数：5/6、流し直しで 6/6**（H6 と同じ）。6 つの指示とも、Claude の式は期待どおりの物の命題 1 つ（検出器の id）で、単語帳と文法の検査に
  1 回目で合格した（LLM の試行 1 回）。周回の 3 つは「「周りを回って」→ 左回りに 1 周」の置き換えの言葉が出た。種類の見張り（O35、`97da92a`）が断った指示はない。
- **ソファの周回の 1 回目（R-H6b の部屋の限り）：** 47 手の後、48 手目の前進（向き 11.5°）で「前方に障害物があります」（app のログ：front_blocked）になり、
  15 秒待って中止した。止まった姿勢 (−2.298, −1.507, 11.5°) から、スーツケースの足跡の角 (−1.854, −1.286) は胴の前 0.48 m・左 0.13 m で、止まる箱
  （胴の前 0.5 m、横 ±0.30 m）の内。H6 の 1 回目（止まった姿勢 (−2.386, −1.001, 12.5°)、前方 0.48 m）と同じ所で、ソファの奥の調べ（cause_lap）が示した
  わけ（周回の輪の外側のマスがスーツケースの縁から 0.46〜0.54 m を通る）と合う。R-H6b のとおり部屋の限りで、止まって中止したのは正しい安全の動き。
  物には触れていない（胴の中心から足跡まで 0.443 m、傾き 0.073）。`watch` が「合格」と出したのは、一時停止の 10 秒の止まりで終わったため（そこまでの
  巻き数 0.912）で、画面は中止なので 1 回目は数えない。流し直し（R-F6c）は 51 手で終わった（前の物の一時停止 0.7 秒が 1 回、続けた）。
- **ソファの名前の変わり（R-H6c、5 秒の線とは別に書く）：** 「ソファまで歩いて」の準備で、ソファは Bf の止まりでまず `bench_1` と呼ばれ、2 つの `resolve` は
  「bench_1 ('bench') pairs with sofa_1 … waiting」の後、Bf に止まって 6.89 秒で `sofa_1`（受け入れる名前）を出した（状態の `ids_retired` 1）。H6 の名前の変わりは
  B（7.6 秒）で、Bf に止まった時には `sofa_1` だった。ここでは Bf の止まりでの変わり。指示は名前が付いた後に送ったので、式と判定には効いていない。
- 推論の時間（準備の後の状態と終わりの状態、14 の読み）：p50 360〜443 ms、p90 373〜636 ms。どれも §13.5 の線の内。
- 利用者が話して流した回はない。Whisper はこの仕事で変わらない（O26）。

### M5（D9）：立った Go2 の前にボタンの箱（2 回目の組）

- detector の組（yoloe、画面あり、08:02）で `F standing`、8 秒の後に「前に箱を置く」（`/sim/obstacles` に `{"cmd": "front"}`、受け手 1）。
  `/sim/world` は箱 `[1.228, −0.032, 0.3]`（Go2 の前 1.2 m、0.3 m の箱）。
- 置いた後 5 秒ごとに 6 回 `/go2/objects` を読み、`accuracy` も流した：**一覧は `chair_1` だけで、`box` の行は出なかった**（`button_boxes_seen` 空、
  推論 29 回、`tracks_tentative` 0、`shapeless` 0、`ids_issued` は chair 1 つ）。H6 と同じ。
- **ログ（R-H6e の確かめ）：** Gazebo のログは組を下ろす前に保存した（284 行。最後の行が橋渡しの「箱を置きました（front）: sim_box_1 x=1.23 y=-0.03」）。
  `Could not resolve file` は 0 行、`Unable to find texture` も 0 行。H6 のログは、同じ箱を置いた行のすぐ後に `Could not resolve file [cardboard_box.png]` が
  2 行（サーバーと画面）あった。**R-H6e の直しで、この行は消えた。**
- **絵：** カメラの絵（`/gazebo/front_camera/image`、640 × 480、箱を置く前と後の 2 枚。作業用のフォルダに残した）では、箱は段ボールの茶色で描かれ、
  灰色ではない。カメラに向いた面は影の側で暗い（その面の明るさ 32〜64、平均 48）。
- **箱が一覧に入らないわけ（新しく確かめた）：** 同じ 2 枚を、検出器のイメージの使い捨ての container（`--network none`、worktree と検出器のキャッシュは
  読むだけ、`env.txt` は空のファイルで隠した）で、検出器と同じ `YoloeDetector` と同じ 90 のクラス（単語帳の `detect` の言葉と無視する言葉）で読んだ。
  - 検出器の予測の線（conf 0.10）で、箱を置いた絵の当たりは chair 0.325（chair_1）、chair 0.142（chair_1 の一部）、**air purifier 0.103（箱の所、
    [282, 233, 359, 336]）**。0.02 まで下げても、箱の所は air purifier 0.103 と chair 0.100（箱と影を囲む枠）だけで、`box` の言葉（「cardboard box」「box」）が
    一番になる所はない。箱を置く前の絵は chair 0.356 だけ（0.02 まで下げると、左の遠くの box_1（ただの形の箱）の所に air purifier 0.044）。
  - 検出器が当たりとみなすのは 0.25（`sighting`）以上なので、箱は一度も当たりにならず、仮の追跡もできない（状態の `tracks_tentative` 0・`shapeless` 0 と合う）。
  - **だから M5 で箱の行が出ないわけは、YOLOE-11s が、この部屋の明かりでカメラに暗い面を向けた Gazebo の段ボール箱を「箱」と呼ばないこと**（一番の名前は
    air purifier 0.103）。絵が付いていないことではない。設計書 §11.13・D9 の「箱と呼ばれれば一覧に入る」は、この箱のこの見え方では起きない。線ではないので記録だけ。

### H6 の記録の訂正

H6 の節（上）の文は H6 のときの記録として残し、ここで直す。H6 の数（測った値）はどれも変わらない。変わるのは、わけと数え方の文だけ。

**(1) Bf のソファの奥が 0.22 m 欠けたわけ（H6 の「M3：oracle の `covers` が偽になるわけ」のソファの項と、「controller に返すこと」の 1 (a)）**

- H6 の記録は、わけを形の決まり R-G1 (3)（観た奥行きが事前の奥行き 0.9 m に届けば奥を観たとみなす）とした。**これは誤り。** わけは角の決まり（R-G4）の
  面の判定で、Go2 に向いた背もたれの前の面（ソファの前から 0.72〜0.83 m）を、物の奥の面と読んだこと。
- 証拠（ソファの奥の調べ `~/work/physicalAI/go2/.superpowers/sdd/2026-11-11-go2-open-objects/reports/sofa-depth-report.md` の cause_far と、
  最後のコードレビューの perception-1）：
  - **深さの数：** `detector/geometry.py` の `_rect` で、R-G1 (3) の枝の足跡は EDGE_PAD + d_obs + pad_seen（d_obs + pad_seen ≥ 0.9）なので 0.95 m より
    浅くならない。補った（`filled`）足跡は 0.9 + 2 × 0.05 = 1.0 m 以上。H6 の行は、わけを探した回が 0.880 m（x −4.882〜−4.002、5 通とも）、
    M3 の yoloe の Bf が 0.902 m で、どちらの枝からも出ない。出せるのは角の枝（d_obs 0.78〜0.80 m に、面の判定が付けた奥の余り EDGE_PAD 0.05 m）だけ。
  - **LiDAR の輪：** Bf からは、+6° の輪が背もたれの前の面（高さ 0.84〜0.85 m、前から 0.72〜0.83 m）に当たり、+10° の輪は高さ 0.943 m のソファの
    上を越える。本当の背面（前から 1.044 m）は背もたれ自身に隠れて、どの輪にも見えない。面の判定は、この前の面を奥の面と読んだ。
  - **合成のソファ：** 座面 0.78 m・背もたれ 0.264 m の同じ形を `d407854` のコードに通すと、`filled` 偽・奥 0.214 m 欠け・行 0.88 × 1.879 m で、H6 と同じになる。
  - **直しの後（H6b）：** 直し `c9777ae`（`objects2/sofa-depth`、merge `e4ae4fc`）は奥の面の判定だけを変えた（手前から 60° 以内に見える奥の面は端と
    みなさない）。H6b の Bf のソファの行は、M3 の oracle 3 通・yoloe 4 通と、M4 の Bf の 12 回のうち `sofa` と呼ばれた 11 回
    （残る 1 回は `bench_1` と呼ばれた、目標でない行）のどれも深さ 1.000〜1.016 m（補った深さ）で、`filled` 真、
    奥は C2 の補いの分 0.092〜0.097 m だけ足りない。R-G1 (3) の深さ（0.95〜1.0 m で `filled` 偽）も、角の枝の 0.88〜0.90 m も出なかった
    （perception-1 の言う、状態の `filled` でなく行の深さで判定する方法）。単語帳の奥行き 0.9 m とモデルの 1.044 m の差が決めるのは、補った足跡が
    どれだけ足りないか（0.094〜0.098 m、C2 が許す分）だけで、`covers` 偽のわけではない。
- H6 の「同じ組の 1 通目は `filled` 真なので、推論ごとに観た奥行きが 0.9 の前後で揺れ、`filled` が入れ替わる」も誤り。わけを探した回の
  `covers` の記録の最初の 2 通は同じ辺（x −4.882〜−4.002）で、`filled` だけが真から偽に変わった。探すスクリプトは `filled` を 1 Hz の
  `/go2/detector_status` から取り、2 Hz の `/go2/objects` と組にしたので、1 通目は前の状態と組になった（読む側の遅れ）。足跡の深さは揺れていない。
  `objects_check accuracy` も最後の状態と最後の一覧を組にするので、同じ遅れがありうる（H6 と H6b の数える読みでは出ていない：
  H6 の M3 oracle の Bf の行は 1.004 m・`filled` 真）。

**(2) スーツケースが一覧に入った止まり（H6 の M4 の「スーツケース」の項の「7 回のうち 4 回が B」）**

- M4 の表のスーツケースは 6 回（手前 3・周回 3）で、7 回ではない。**B の止まりが 3 回、Bf の止まりが 3 回。** H6 の記録係（`rec.jsonl`、止まりの
  変わり目と行の届いた時刻）で数え直した：
  - B（回り終えて止まった所、前に出る式の前）：suitcase_near1 3.12 秒、suitcase_near2 2.71 秒、suitcase_lap3 2.68 秒；
  - Bf（前に出て止まった所）：suitcase_near3 2.81 秒、suitcase_lap1 4.42 秒、suitcase_lap2 1.08 秒。
- だから正しくは「6 回のうち 3 回が B の止まり（2.68〜3.12 秒）、3 回が Bf の止まり（1.08〜4.42 秒）」。どの回も 5 秒以内という判定は変わらない。

**(3) M5 で箱の行が出なかったわけ（H6 の「M5」の項、結論の表の M5 の行、「controller に返すこと」の 5）**

- H6 は「段ボールの絵が付いていない…箱は形だけで、段ボールの絵のない見た目で描かれた」とした。**これは誤り。絵は描かれていた。**
  - h6-fixes の確かめ（`objects2/h6-fixes`、R-H6e の枝）：`Could not resolve file [cardboard_box.png]` が出る時と出ない時のカメラの絵はバイトまで同じで、
    段ボールの絵が写っている。絵を本当に引けなくすると箱は灰色に描かれ、ログに `Unable to find texture`（Material.cc）も出る。H6 の M5 のログでは
    これは 0 行。メッシュの .dae は絵を名前だけで引き、gz-common は横の `../materials/textures` で見つけて描く。Gazebo が同じ名前を `GZ_SIM_RESOURCE_PATH`
    でも探し、見つからないとこの行を出すだけ（`ddbfdbe` の `gazebo/bridge_core.py` の説明も同じ）。
  - H6b の M5（上）：R-H6e の直しで行は 0 行になり、箱は段ボールの茶色で描かれ、それでも一覧に入らなかった。
- **本当のわけ：** 検出器（YOLOE-11s）がその箱を「箱」と呼ばないこと。H6b の M5 の絵を検出器と同じ YOLOE で読むと、箱の所の一番の名前は air purifier 0.103
  （当たりとみなす線 0.25 の下）で、`box` の言葉はどこでも一番にならない。だから当たりも仮の追跡もない。H6 の状態も `tracks_tentative` 0・`shapeless` 0 で
  同じ形。H6 の M5 の絵は残っていないが、箱の場所（H6 [1.245, 0.019]、H6b [1.228, −0.032]）、Go2 の姿勢、明かり、検出器の状態が同じなので、同じわけと見る。
- H6 の「controller に返すこと」の 5 が勧めた直し（絵の場所を GZ_SIM_RESOURCE_PATH に足す）は R-H6e で入り、ログの行を消したが、箱の行が出ないことは
  直らない（h6-fixes の見込みのとおり）。

### controller に返すこと（decisions_needed）

- 安全の問題（ロボットが動くべきでない時に動いた、物に触れた、倒れうる）はない。だから decisions_needed はない。
- R-H6f で v1 の限りにするもの（わけがわかり、安全の問題でない）：
  1. **M3 の `accuracy` の読みの競り合い：** `ready()` が正解の 1 通目（2 Hz）を待たないので、検出器がもう止まって 3 回推論した後に始めると、0.5 秒で
     `no /gazebo/objects_truth`（終了コード 2）になることがある（H6b の 1 回目で 3 回）。測りの道具の限りで、製品の道ではない。姿勢に着いた直後に始めた
     流し直しは 3 つとも読めて、線を満たした。
  2. **Bf のソファの +y の横の面が 2 mm 足りない時がある**（oracle、3 通に 1 通、`COVER_TOL` 1 mm の前後）。0.05 m の間引きの段より小さい。
  3. **推論の時間は、重い負荷の下で線を超える：** 1 回目の 3 回（load 9.8〜42.7、ほかの組の container 462〜509 %）で p90 893〜910 ms・p50 661 ms。
     流し直しの 3 回（load 5.5〜11.0、ほかの重い container なし）は p50 367〜394・p90 389〜438 ms で線の内。
  4. **Bf のスーツケースは、止まってから一覧に出るまでが 5 秒を超えることがある：** suitcase_near2 の 2 回目で 7.09 秒（M4 のスーツケースの 7 回のうち 1 回）。
     絵の端の近く（R-H3j）で点が確かめの線の近くにあり、止まりの推論 3 回で確かにならず、4 回目が 3 秒の休みの後だった（上）。
  5. **声の段のソファの周回の 1 回目の中止**（前方 0.48 m のスーツケースで 15 秒止まった）。R-H6b の部屋の限りと同じわけ・同じ所。流し直しは合格。
- 記録だけ：
  - 声の段の準備で、Bf の止まりでソファがまず `bench_1` と呼ばれ、`sofa` の名前まで 6.89 秒（R-H6c のとおり 5 秒の線とは別。H6 の変わりは B だった）。
  - M5：箱の行が出ないわけは検出器の名前（箱の所の一番の名前が air purifier 0.103）。R-H6e でログの行は消えた。下の「H6 の記録の訂正」(3)。
  - 傾き：H6b の最大 0.237（1 回目の組の sofa_near3、記録を足す前）。2 回目の組は 0.127 以下で、M0 の画面なしの周回は 0.051。H6 の 0.410 は出なかった。
  - 1 回目の組では R-F6b に反して、ほかの組の container が 4〜8.7 コア使った回があり、「立つ」の error（R-H5e）が 7 回続いた。2 回目の組では 0 回。
  - Mac は 00:11 に蓋を閉じて眠った（AC につないでいても、外の画面がないと蓋を閉じると眠る）。2 回目の組は蓋を開けたまま、眠りなし。
  - 着いた姿勢が `MEASURED_RANGES` の外に出たのは 3 回（Bf の y で 0.006・0.014・0.001 m）。どれも計画があり線は通った。固定は変えていない。
  - 目標でない物の名前の誤り：周回の後にテーブル（logical だけ）が `stool_1`（3 回）、ソファの一部が `television_1`（1 回）、ソファがまず `bench`（3 回。suitcase_near1・suitcase_near3・声の段の準備）。

### 動かした container

- 1 回目の組：`G` の compose project `go2oo` の `go2oo-gazebo-1`（画面あり）・`go2oo-gazebo`（画面なし）・`go2oo-detector-1`、`G run --rm … app` の
  `go2oo-app-run-…`（`run_formula`・`objects_check`・読むだけの使い捨てのスクリプト）。始めに、止めで残っていた `go2oo-gazebo-1`・`go2oo-detector-1` を
  `G … down` と `docker rm -f` で止めた。
- 2 回目の組：同じ `G` の container に加えて、声の段の `GA` の `go2oo-app-1`（`GA up`、終わりは `G … down`）と確かめの `GA run --rm`、M5 の絵を読む使い捨ての
  `go2oo-yprobe`（検出器のイメージ、`--network none`、読むだけのマウント、`env.txt` は空のファイルで隠した）。声の段では、Browser の画面でこちらのページ
  http://localhost:8811 を開いた。
- 終わりに、こちらの組と container をすべて止めた（`go2oo` の container は残っていない）。名前付きの volume `go2oo_detector-cache` は残している
  （M5 の読みでは読むだけでマウントした）。
- 利用者の project `go2` の `go2-gazebo-1`・`go2-app-1`、ほかの組の container（`go2-app-run-…`、`oo-perc2-rg13-…`、`oo-docs-tests-…`）には触れていない。
  main checkout では git を使っていない（`GA` の compose がそこから `env.txt` を `env_file` で読むだけ）。`~/work/physicalAI/.go2-worktrees/open-objects` では
  1 回目の組で読むだけの `git diff --stat`・`git diff`・`git log` だけ。
