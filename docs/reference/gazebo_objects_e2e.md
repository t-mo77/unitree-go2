# Gazebo での物の確認：箱の一周と椅子まで（Task I）

設計書 `docs/superpowers/specs/2026-11-05-go2-objects-design.md` の §2 の合格の条件を、LLM を通さずに Gazebo で測った記録。
この文書は **第 1 部**（計画の Task I の手順 4〜6：物の一覧・RTF・箱の一周・椅子まで）。操作画面の確認と画面の絵（手順 7）、
静かなときの RTF は **第 2 部** で足す（最後の「第 2 部：これから」）。

- 日付：2026-11-06。場所：この Mac の Docker Desktop（arm64、CPU 8、メモリ 7.7 GiB）。
- **負荷：** 同じ時間に、別の担当（Task H・Task L）がそれぞれのフォルダでテストを流していた（`obj-h-app-run-*`・`obj-l-app-run-*` が 100〜120 %）。
  RTF はその負荷のまま測った値。静かなときの値は第 2 部で測り直す。
- 使ったもの：app は `go2-voice-ltl:dev`（ROS 2 Humble）、gazebo は `go2-voice-ltl-gazebo:dev`（ROS 2 Jazzy + Gazebo Harmonic、`docker/patches/` の 0001・0002 入り。作り直していない）。
  コードは `objects/i` のフォルダ（`../.go2-worktrees/obj-i`、compose の project は `obj-i`）。最後の回は commit `e47e11e`。
- Gazebo は毎回、使い捨てのコンテナで起動し直した（compose の service は起動していない。計画の P8）：

```bash
docker stop go2-objects-gz       # 前の回の、自分で起動した使い捨てのコンテナだけ
docker compose --profile gazebo run -d --rm --no-deps -T -e ROS_DOMAIN_ID=77 --name go2-objects-gz gazebo \
    ros2 launch /work/gazebo/launch/go2_gazebo.launch.py gui:=false
# ログに Configured and activated unitree_guide_controller が出るまで待つ（イメージがキャッシュにあると 8〜10 秒）
```

  計画の手順 4 は `-d` なしで後ろで動かす形。ここでは `-d`（コンテナを切り離す）にしただけで、ほかは同じ。
- 測り方：
  - `gazebo/tools/objects_check.py`（コミット済み）。`run` は `/model/robot/odometry`（Gazebo の本当の位置と傾き）を記録しながら `run_formula` を子として流し、§2 の値と合否を JSON 1 行で出す。
  - 動作ごとの傾きを見るため、別の使い捨てコンテナ（`go2-objects-rec`）で、odometry（25 Hz）・`/api/sport/request`・`/go2/objects` の大きさの変化を記録した（測る道具はコミットしていない）。
    下の「動作ごとの最大傾き」はこの記録から出した（1 つの動作 = Move を送り始めてから次の動作の Move まで。止めた後の 0.5 秒も入る）。

## 結論

| 確かめたこと | 結果 |
|---|---|
| 物の一覧（手順 4） | `box_1`・`chair_1` だけ（`ground_plane` などは無い）、どちらも `visible: true`、位置は (2.5, 1.5)・(2.5, −1.5) ぴったり |
| 物の大きさ（手順 4） | **最初は合格しなかった**：箱 1.043 × 1.117、椅子 1.073 × 1.186（0.3〜0.8 の外）。Go2 を出した直後に床へ落ちる間の点群で膨らんでいた。**直した**（`StillGate`）後は 0.511 × 0.513・0.468 × 0.467 前後。走った後も 0.51〜0.53・0.47〜0.48 |
| RTF（手順 4、20 秒） | 0.96〜0.99（画面なし `gui:=false`）。負荷が重いときの走っている間は 0.6〜1.0、odometry が 1.2 秒止まることもあった（下の「負荷が重いとき」） |
| 箱の一周（手順 5、最後の設定で 3 回） | **0/3**。3 回とも途中で Go2 が転び（max_tilt 2.9〜3.1）、`姿勢が変わったので止めました` で `exit=1` |
| 椅子まで（手順 6、最後の設定で 3 回 + 挨拶 1 回） | **3/4 合格**（FN1・FN3・FH1）。FN2 は転ばずに着いたが、椅子まで 1.538 m（> 1.5）で不合格。挨拶の回は「開始：挨拶」まで行った。どの回も最初の動作は「立つ」、max_tilt 0.51〜0.74 |
| 調べた全部（設定を変えながら 15 回の一周、5 回の椅子） | 一周を終えたのは 1 回（調整の前の lap1：回転数 1.039、箱の縁まで 0.851 m、max_tilt 0.707）。ほかの 14 回は転んだ。椅子は 5 回のうち 4 回合格（転んだ回は無い） |
| 転ぶ原因 | Gazebo の unitree_guide controller。格子の計画は 0.5 m・90 度ごとに StopMove で止まり、その後の足踏みで胴が横に揺れ、ある割合で倒れる（一周の 39〜41 動作のうち 2〜38 動作目で）。止まる前・回る途中・歩き出しのどこでも起きる。実行器の値の調整で揺れは小さくなったが、転ぶのは無くならなかった（下の「転んだこと」） |

**§2 の合格の条件（一周 3 回・椅子 3 回）は、Gazebo では満たせていない。** 計画を立てる側（監督器・計画・Session）は、転ぶまでどの回も計画どおりに動いた（`位置のずれ` で止まった回は無い）。
転ぶのは Gazebo の controller の問題で、直すには controller にパッチを当ててイメージを作り直す必要がある（このタスクではしない。最後の「気づいたこと」）。

## 手順 4：物の一覧と RTF

1 回目（13:11、直す前）：

```bash
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py objects
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py rtf --seconds 20
```

```text
{"frame": "odom", "stamp": 1793938308.5855815, "objects": [{"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [1.043, 1.117], "seen": 1793938308.373, "visible": true}, {"id": "chair_1", "label": "chair", "x": 2.5, "y": -1.5, "size": [1.073, 1.186], "seen": 1793938308.373, "visible": true}]}
exit=0
{"rtf": 0.98}
exit=0
```

- 大きさが 0.3〜0.8 の外。r = hypot(sx, sy) / 2 が椅子で 0.80（> 0.65）になり、近い帯（R4）が空になる：「椅子まで歩いて」は `MSG_NO_PATH` になる。
- `objects_check.py objects` の終了コード 0 は「どの行も `objects.parse` を通った」の意味で、大きさの範囲は見ない。

### 大きさが膨らんだわけと、直したこと

- 調べ方：起動した Gazebo の `/utlidar/cloud` を app のコンテナで受け、`lidar_points_world` で世界の点にして、物の中心から 0.6 m 以内の点を見た。
  伏せて止まっているときは、箱の点は中心から ±0.257 m・椅子は ±0.234 m（測れば 0.51・0.47）。点群そのものは正しい。
- `/go2/objects` の大きさが変わった時刻と odometry を並べた（起動し直して記録）：

```text
   6.29 box_1 size=[1.066, 0.665] odom(t,x,y,z,roll,pitch,yaw,wx,wy,wz)=(0.46, 0.029, -0.011, 0.12, -0.001, 0.119, -0.059, 0.006, -0.566, 0.057)
   6.79 chair_1 size=[1.163, 0.475] odom(t,x,y,z,roll,pitch,yaw,wx,wy,wz)=(0.94, 0.027, -0.011, 0.068, -0.0, -0.058, -0.056, 0.0, -0.0, -0.0)
  45.29 box_1 size=[1.066, 1.107] odom(...)=(38.1, ..., 0.335, 0.018, 0.0, 0.104, 0.055, -0.039, 0.675)
  50.29 box_1 size=[1.186, 1.136] odom(...)=(42.86, ..., 0.337, 0.03, 0.032, 1.567, -1.568, 0.288, -0.057)
```

  - シミュレーションの 0.46 秒：Go2 は高さ 0.5 m に出て床へ落ちる途中（pitch の速さ 0.57 rad/s）。回る速さ（yaw）は 0.1 rad/s 以下なので、今までの「ほとんど回っていないときだけ測る」を通った。
    胴の位置と点群の時刻がずれて、地面の点が `GROUND_Z`（0.05 m）より上に出て、物の点に数えられた。`set_size` は大きい方を残すので、ずっと戻らない。
  - 立つ・その場で足踏みする・歩くときも同じ（roll の速さ 1.6 rad/s、yaw の速さは 0.1 以下）。
- 直したこと（`gazebo/bridge_core.py` の `StillGate`、`gazebo/gazebo_bridge.py`）：胴の角速度が 3 軸とも 0.1 rad/s 以下・速さ 0.05 m/s 以下の状態が、点群を撮った時刻の 0.3 秒前（シミュレーションの時間）から今まで続いているときだけ測る。
  テスト `tests/test_gazebo_still_gate.py`。
- 直した後（13:20 以降、毎回の手順 4）：

```text
{"frame": "odom", "stamp": 1793938821.8211143, "objects": [{"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.511, 0.528], "seen": 1793938821.721, "visible": true}, {"id": "chair_1", "label": "chair", "x": 2.5, "y": -1.5, "size": [0.468, 0.467], "seen": 1793938821.721, "visible": true}]}
```

（各回の手順 4 と走った後の大きさは、下の「手順 5・6」の表。）

## 手順 5・6：最後の設定（commit `e47e11e`）での回

1 回ごとに Gazebo を起動し直し、手順 4 の 2 つを流してから：

```bash
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect near "F(near_chair_1 & X F hello)"; echo "exit=$?"
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py objects   # 走った後の大きさ
```

| 回 | 式 | 手順 4 の RTF | 手順 4 の大きさ（箱 / 椅子） | 結果 | 値 | 動作 | 走った後の大きさ |
|---|---|---|---|---|---|---|---|
| FL1 | `F around_box_1` | 0.98 | 0.511×0.513 / 0.468×0.467 | 不合格：winding、max_tilt、exit_code（転んだ：右に回り終わる所） | 回転数 0.012、箱の縁 2.132 m、max_tilt 3.139 | 4/39 | 0.517×0.526 / 0.468×0.477 |
| FL2 | `F around_box_1` | 0.99 | 0.511×0.514 / 0.469×0.467 | 不合格：winding、max_tilt、exit_code（転んだ：前進 → 前進 の後の左に回る） | 回転数 0.201、箱の縁 0.957 m、max_tilt 3.117 | 10/41 | 0.511×0.526 / 0.469×0.471 |
| FL3 | `F around_box_1` | 0.98 | 0.511×0.51 / 0.468×0.467 | 不合格：winding、max_tilt、exit_code（転んだ：立って最初の左に回る途中） | 回転数 -0.011、箱の縁 2.477 m、max_tilt 3.109 | 2/39 | 0.512×0.51 / 0.468×0.47 |
| FN1 | `F near_chair_1` | 0.98 | 0.511×0.509 / 0.468×0.467 | **合格** | 椅子まで 1.109 m、27.2°、椅子の縁 0.704 m、max_tilt 0.588 | 8/8 | 0.512×0.509 / 0.468×0.47 |
| FN2 | `F near_chair_1` | 0.99 | 0.511×0.515 / 0.469×0.467 | 不合格：final_dist_chair_1 | 椅子まで 1.538 m、6.8°、椅子の縁 1.13 m、max_tilt 0.74 | 7/7 | 0.511×0.515 / 0.469×0.471 |
| FN3 | `F near_chair_1` | 0.99 | 0.511×0.527 / 0.462×0.461 | **合格** | 椅子まで 1.107 m、24.8°、椅子の縁 0.753 m、max_tilt 0.507 | 7/7 | 0.511×0.527 / 0.462×0.474 |
| FH1 | `F(near_chair_1 & X F hello)` | 1.0 | 0.513×0.516 / 0.47×0.471 | **合格** | 椅子まで 1.495 m、33.2°、椅子の縁 1.087 m、max_tilt 0.589 | 8/8 | 0.517×0.516 / 0.47×0.481 |

```text
# FL1（14:04:32）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.012, "min_edge_box_1": 2.132, "min_edge_chair_1": 2.183, "final_dist_chair_1": 2.554, "final_angle_chair_1_deg": 97.4, "max_tilt": 3.139, "passed": false, "failures": ["winding: 0.012 not in [0.9, 1.5)", "max_tilt: 3.139 >= 1", "exit_code: 1 != 0"], "samples": 941}
exit=1
# FL2（14:05:29）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.201, "min_edge_box_1": 0.957, "min_edge_chair_1": 1.04, "final_dist_chair_1": 1.386, "final_angle_chair_1_deg": 27.7, "max_tilt": 3.117, "passed": false, "failures": ["winding: 0.201 not in [0.9, 1.5)", "max_tilt: 3.117 >= 1", "exit_code: 1 != 0"], "samples": 1720}
exit=1
# FL3（14:06:43）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": -0.011, "min_edge_box_1": 2.477, "min_edge_chair_1": 2.361, "final_dist_chair_1": 3.231, "final_angle_chair_1_deg": 129.4, "max_tilt": 3.109, "passed": false, "failures": ["winding: -0.011 not in [0.9, 1.5)", "max_tilt: 3.109 >= 1", "exit_code: 1 != 0"], "samples": 630}
exit=1
# FN1（14:07:34）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
{"formula": "F near_chair_1", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.104, "min_edge_box_1": 2.302, "min_edge_chair_1": 0.704, "final_dist_chair_1": 1.109, "final_angle_chair_1_deg": 27.2, "max_tilt": 0.588, "passed": true, "failures": [], "samples": 1374}
exit=0
# FN2（14:08:40）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
{"formula": "F near_chair_1", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.152, "min_edge_box_1": 1.123, "min_edge_chair_1": 1.13, "final_dist_chair_1": 1.538, "final_angle_chair_1_deg": 6.8, "max_tilt": 0.74, "passed": false, "failures": ["final_dist_chair_1: 1.538 > 1.5"], "samples": 1139}
exit=1
# FN3（14:09:41）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
{"formula": "F near_chair_1", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.127, "min_edge_box_1": 1.052, "min_edge_chair_1": 0.753, "final_dist_chair_1": 1.107, "final_angle_chair_1_deg": 24.8, "max_tilt": 0.507, "passed": true, "failures": [], "samples": 1204}
exit=0
# FH1（14:10:43）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F(near_chair_1 & X F hello)"; echo "exit=$?"
{"formula": "F(near_chair_1 & X F hello)", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.083, "min_edge_box_1": 1.875, "min_edge_chair_1": 1.087, "final_dist_chair_1": 1.495, "final_angle_chair_1_deg": 33.2, "max_tilt": 0.589, "passed": true, "failures": [], "samples": 1498}
exit=0
```

- **箱の一周は 3 回とも転んだ**（4・10・2 動作目）。FL3 は立って最初の動作（左に回る）で倒れた。
- **椅子まで：4 回のうち 3 回合格**（FN1・FN3・FH1）。FH1 のログには「開始：挨拶」「完了：挨拶」がある。転んだ回は無い（max_tilt 0.51〜0.74）。
- **FN2 は距離で不合格（1.538 m > 1.5 m）。** 転んではいない（`結果：done`、run_formula の終了コード 0）。
  - 計画は「立つ → 前進 ×5 → 右に回る」。立ったときの向きが −0.02〜−0.05 rad で、行き先のマス (5, 0) は、近い帯（R4：r + 0.75 〜 min(r + 1.5, 1.4) m）の外側の線 1.4 m のすぐ内側だった
    （向き −0.045 rad なら 1.397 m。起点の正確な値は Session の内部のログにしか無いので、記録した位置から見積もった）。
  - 記録した位置：前進 5 回の後 (2.486, −0.031)、椅子まで 1.469 m。その場で右に回る間に胴が 0.1 m 流れ、(2.40, 0.04)、1.538 m で終わった。
  - R4 は終わりのずれに 0.1 m を残している。Gazebo では、前進の横のずれ（±0.1 m）とその場の旋回で胴が流れる分（0.1 m）を足すと、それを越えることがある。
    近い帯の外側の線（1.4 m）は `voice_ltl/grid.py`（このタスクの持ち物でない）。旋回の間に位置を直さないのは `GridExecutor` の作り（回る間は向きだけ見る）。
- 走った後の大きさ：箱 0.511〜0.517 × 0.509〜0.527、椅子 0.462〜0.470 × 0.470〜0.481（どれも 0.3〜0.8 の中）。

## 転んだこと：調べたことと調整

### 回ごと（時刻順）

設定の列：P7 = 計画の P7 のまま。そこから変えたものを足していく（下の「変えた値」）。「動作」は `開始：` の数 / 計画の動作の数。

| 回 | 時刻 | 設定 | 合否（run_formula の終了コード） | 動作 | 転んだ所 | max_tilt |
|---|---|---|---|---|---|---|
| lap0 | 13:14:22 | P7、大きさは膨らんだまま（StillGate の前）、負荷が重い | 不合格（exit 1） | 19/40 | 前進の歩き出し（z 0.26 rad/s で回しながら） | 3.098 |
| lap1 | 13:20:34 | P7 + StillGate | 合格（exit 0） | 39/39 | — | 0.707 |
| near0 | 13:24:50 | P7 + StillGate | 合格（exit 0） | 8/8 | — | 0.783 |
| lapA | 13:27:09 | + 歩くときの z の上限を速さに比べる（0.3） | 不合格（exit 1） | 7/39 | 前進 → 前進 → 前進 の 3 回目 | 1.058 |
| lapB | 13:29:48 | + 歩くときの直し 0.6・0.4、z の上限 0.15 | 不合格（exit 1） | 23/41 | 左に回り終わった直後（41° 回り続けた） | 3.055 |
| lapC | 13:34:09 | + 回るときの K_YAW 1.0（負荷が重い） | 不合格（exit 1） | 38/41 | 前進（負荷で一時停止・計画の変更が続いた後） | 3.063 |
| lapD | 13:37:56 | K_YAW 1.5 に戻し、回る速さの下限 0.25 | 不合格（exit 1） | 8/39 | 右に回る途中 | 3.135 |
| lapE | 13:45:28 | + 橋渡しで止めた後に立つ（入れなかった） | 不合格（exit 1） | 5/39 | 前進の終わりのゆっくりの所 | 1.079 |
| lapF | 13:47:05 | + 前進 clip(2.0×残り, 0.15, 0.3)（橋渡しはそのまま） | 不合格（exit 1） | 4/39 | 「歩く」に入り直した直後 | 3.115 |
| lapG | 13:49:21 | 橋渡しを戻し、回る速さ 0.3 rad/s | 不合格（exit 1） | 30/41 | 前進の歩き出し（止めた後の揺れの中） | 3.134 |
| L1 | 13:53:44 | + 加速 0.4 m/s²（commit 8c55d0e） | 不合格（exit 1） | 19/39 | 左に回り始めて 0.3 秒 | 3.133 |
| L2 | 13:59:53 | 同上 | 不合格（exit 1） | 10/39 | 左に回る途中 | 3.125 |
| L3 | 14:01:15 | 同上 | 不合格（exit 1） | 17/39 | 前進 | 3.14 |
| FL1 | 14:04:32 | 最後（e47e11e）：回る 0.5 rad/s・加速 0.6 に戻す | 不合格（exit 1） | 4/39 | 右に回り終わる所 | 3.139 |
| FL2 | 14:05:29 | 同上 | 不合格（exit 1） | 10/41 | 前進 → 前進 の後の左に回る | 3.117 |
| FL3 | 14:06:43 | 同上 | 不合格（exit 1） | 2/39 | 立って最初の左に回る途中 | 3.109 |
| FN1 | 14:07:34 | 同上 | 合格（exit 0） | 8/8 | — | 0.588 |
| FN2 | 14:08:40 | 同上 | 不合格（exit 0） | 7/7 | — | 0.74 |
| FN3 | 14:09:41 | 同上 | 合格（exit 0） | 7/7 | — | 0.507 |
| FH1 | 14:10:43 | 同上 | 合格（exit 0） | 8/8 | — | 0.589 |

```text
# lap0（13:14:22）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.267, "min_edge_box_1": 0.62, "min_edge_chair_1": 0.948, "final_dist_chair_1": 2.296, "final_angle_chair_1_deg": 126.3, "max_tilt": 3.098, "passed": false, "failures": ["winding: 0.267 not in [0.9, 1.5)", "max_tilt: 3.098 >= 1", "exit_code: 1 != 0"], "samples": 3278}
# lap1（13:20:34）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 0, "first_action": "立つ", "winding_box_1": 1.039, "min_edge_box_1": 0.851, "min_edge_chair_1": 1.189, "final_dist_chair_1": 2.254, "final_angle_chair_1_deg": 60.5, "max_tilt": 0.707, "passed": true, "failures": [], "samples": 6425}
# near0（13:24:50）
{"formula": "F near_chair_1", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.098, "min_edge_box_1": 2.147, "min_edge_chair_1": 0.803, "final_dist_chair_1": 1.241, "final_angle_chair_1_deg": 33.8, "max_tilt": 0.783, "passed": true, "failures": [], "samples": 1543}
# lapA（13:27:09）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.013, "min_edge_box_1": 1.068, "min_edge_chair_1": 1.962, "final_dist_chair_1": 2.445, "final_angle_chair_1_deg": 60.1, "max_tilt": 1.058, "passed": false, "failures": ["winding: 0.013 not in [0.9, 1.5)", "max_tilt: 1.058 >= 1", "exit_code: 1 != 0"], "samples": 1326}
# lapB（13:29:48）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.409, "min_edge_box_1": 0.864, "min_edge_chair_1": 1.115, "final_dist_chair_1": 3.513, "final_angle_chair_1_deg": 99.2, "max_tilt": 3.055, "passed": false, "failures": ["winding: 0.409 not in [0.9, 1.5)", "max_tilt: 3.055 >= 1", "exit_code: 1 != 0"], "samples": 4014}
# lapC（13:34:09）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.671, "min_edge_box_1": 0.495, "min_edge_chair_1": 1.156, "final_dist_chair_1": 3.765, "final_angle_chair_1_deg": 21.5, "max_tilt": 3.063, "passed": false, "failures": ["winding: 0.671 not in [0.9, 1.5)", "min_edge_box_1: 0.495 < 0.6", "max_tilt: 3.063 >= 1", "exit_code: 1 != 0"], "samples": 6745}
# lapD（13:37:56）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": -0.024, "min_edge_box_1": 0.985, "min_edge_chair_1": 1.808, "final_dist_chair_1": 3.125, "final_angle_chair_1_deg": 61.7, "max_tilt": 3.135, "passed": false, "failures": ["winding: -0.024 not in [0.9, 1.5)", "max_tilt: 3.135 >= 1", "exit_code: 1 != 0"], "samples": 1528}
# lapE（13:45:28）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.005, "min_edge_box_1": 1.958, "min_edge_chair_1": 2.292, "final_dist_chair_1": 2.623, "final_angle_chair_1_deg": 38.2, "max_tilt": 1.079, "passed": false, "failures": ["winding: 0.005 not in [0.9, 1.5)", "max_tilt: 1.079 >= 1", "exit_code: 1 != 0"], "samples": 1445}
# lapF（13:47:05）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": -0.041, "min_edge_box_1": 2.296, "min_edge_chair_1": 2.52, "final_dist_chair_1": 4.3, "final_angle_chair_1_deg": 178.5, "max_tilt": 3.115, "passed": false, "failures": ["winding: -0.041 not in [0.9, 1.5)", "max_tilt: 3.115 >= 1", "exit_code: 1 != 0"], "samples": 1003}
# lapG（13:49:21）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.624, "min_edge_box_1": 0.809, "min_edge_chair_1": 1.158, "final_dist_chair_1": 5.472, "final_angle_chair_1_deg": 76.2, "max_tilt": 3.134, "passed": false, "failures": ["winding: 0.624 not in [0.9, 1.5)", "max_tilt: 3.134 >= 1", "exit_code: 1 != 0"], "samples": 5679}
# L1（13:53:44）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.399, "min_edge_box_1": 0.774, "min_edge_chair_1": 1.196, "final_dist_chair_1": 3.139, "final_angle_chair_1_deg": 174.7, "max_tilt": 3.133, "passed": false, "failures": ["winding: 0.399 not in [0.9, 1.5)", "max_tilt: 3.133 >= 1", "exit_code: 1 != 0"], "samples": 3660}
# L2（13:59:53）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.02, "min_edge_box_1": 0.961, "min_edge_chair_1": 1.315, "final_dist_chair_1": 2.729, "final_angle_chair_1_deg": 71.5, "max_tilt": 3.125, "passed": false, "failures": ["winding: 0.020 not in [0.9, 1.5)", "max_tilt: 3.125 >= 1", "exit_code: 1 != 0"], "samples": 2036}
# L3（14:01:15）
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.222, "min_edge_box_1": 0.772, "min_edge_chair_1": 0.781, "final_dist_chair_1": 1.12, "final_angle_chair_1_deg": 136.8, "max_tilt": 3.14, "passed": false, "failures": ["winding: 0.222 not in [0.9, 1.5)", "max_tilt: 3.140 >= 1", "exit_code: 1 != 0"], "samples": 3461}
```

### 動作ごとの最大傾き（記録から）

1 つの動作の間の max(|roll|, |pitch|)。「≥ 1」は転んだ動作（橋渡しが 1.0 rad を越えたら転んだとして止める）。

| 設定（回） | 動作 | 数 | 中央値 | 上位 10 % | 0.5〜1.0 | 転んだ |
|---|---|---|---|---|---|---|
| P7（lap1・near0） | 前進 | 33 | 0.29 | 0.57 | 6 | 0 |
| P7（lap1・near0） | 回る | 12 | 0.24 | 0.35 | 0 | 0 |
| z の上限を速さに比べる 0.3（lapA） | 前進 | 4 | 0.62 | 1.06 | 2 | 1 |
| 歩くときの直しを弱く、回る 0.5 rad/s（lapB・lapD） | 前進 | 18 | 0.19 | 0.42 | 0 | 0 |
| 同上 | 回る | 11 | 0.31 | 3.06 | 0 | 2 |
| 回るときの K_YAW 1.0（lapC） | 前進 | 25 | 0.31 | 0.61 | 5 | 1 |
| 同上 | 回る | 15 | 0.40 | 0.90 | 6 | 0 |
| 歩くときの直しを弱く、終わりを短く、回る 0.3 rad/s（lapG・L1〜L3） | 前進 | 45 | 0.26 | 0.50 | 3 | 2 |
| 同上 | 回る | 27 | 0.32 | 0.88 | 3 | 2 |

- 前進は、歩くときの向きの直しを弱くした（下の「変えた値」）後で揺れが小さくなった（中央値 0.29 → 0.19〜0.26、上位 10 % 0.57 → 0.42〜0.50）。それでも転ぶ回が残った。
- 回るのは 0.3 rad/s にすると悪くなった（中央値 0.32、0.5〜1.0 が 3 回、転んだのが 2 回）。0.5 rad/s に戻した。

### 何が起きて転ぶか（記録から）

- **止めた後の横揺れ。** StopMove の後、controller は「歩く」のまま速度 0 で足踏みを続ける（`StateTrotting::checkStepOrNot`：指令も位置・速さのずれも小さいと足踏みを止め、揺れてずれが育つとまた踏む）。
  胴は 1.5 秒ほどの周期で横に揺れ（roll 0.6 rad、向き ±0.15 rad、横に ±0.1 m）、その揺れの中で次の動作を始めると倒れることがある。
  例：lapG で、前進の後の 0.5 秒の待ちの間に胴が後ろへ 0.14 m 流れ、次の前進の歩き出しで roll が 0.3 から 1.0 になって転んだ。
- **歩くときの向きの直しが揺れを育てる。** P7 の値（向き 1.5 /s、横のずれ 1.0 rad/(m·s)）だと、z（回る指令）が揺れに合わせて ±0.25 rad/s で振れ、前進 → 前進 で roll 0.55〜1.06 になった。
- **ゆっくり歩くと傾く。** 前進の終わりの 0.08〜0.14 m/s の 1 秒あまりで、胴が横へ流れながら傾いた（lapE で 1.07）。
- **回る途中・回り終わり。** 落ち着いて立っている所から回り始めて 0.3 秒で倒れた回（L1、向き 0°）、回り終わりの StopMove の後に 41 度回り続けて倒れた回（lapB）がある。
  回り終わり：controller は回る指令を積んで目標の向きを作る。0.5 rad/s の指令に対して Go2 は平均 0.42 rad/s で回り、目標は最大 10 度先に行った（橋渡しが送った値を積んで測った。lapB）。0.3 rad/s では 4 度以内だったが（L1）、転ぶ回は減らなかった。
- 原因ではなかったもの：
  - 負荷：手順 4 の RTF が 0.96〜1.0 で、`docker stats` でほかのコンテナがほぼ止まっていた回（L1〜L3・FL1〜FL3）でも転んだ。
  - 向き ±180 度：lap1 は ±180 度をまたいで何度も歩いて止まり、転ばなかった。向き 0°・−90°・+90° でも転んだ。
  - 物の大きさ・計画：転ぶ前はどの回も計画どおり（`位置のずれ` は 1 回も出なかった）。

### 変えた値（`voice_ltl/grid_executor.py`、テスト `tests/test_grid_executor.py`）

| 値 | P7 | 今 | わけ |
|---|---|---|---|
| 歩くときの向きの直し | `K_YAW` 1.5 /s | `K_WALK_YAW` 0.6 /s（その場で回るのは `K_YAW` 1.5 のまま） | 揺れに合わせて z が振れ、揺れを育てた |
| 歩くときの横のずれの直し | `K_CROSS` 1.0 | 0.4 rad/(m·s) | 同じ |
| 歩くときの z の上限 | ±0.3 rad/s | ±0.15 rad/s × (前進の速さ ÷ 0.3) | 歩き出し・止まる前（ゆっくり）に回さない。lap0 は歩き出しに 0.26 rad/s で回して転んだ |
| 前進の速さ | clip(1.2 × 残り, 0.08, 0.3) | clip(2.0 × 残り, 0.15, 0.3) | ゆっくり歩く時間を 1 秒あまりから 0.5 秒ほどに |
| 回る速さの下限 | 0.15 rad/s | 0.25 rad/s | controller は前進 0 で回る指令が 0.2 rad/s 未満だと足踏みしない。回り終わりに足を着いたままねじることになる |
| 回る速さ・加速 | 0.5 rad/s、(0.6, 0.6, 1.0) | そのまま | 0.3 rad/s・0.4 m/s² を試したが良くならなかった |
| 止めた後に待つ秒数 `GRID_SETTLE_S` | 0.5 | そのまま | 待っても揺れは収まらない（`gazebo_e2e.md` の 4 と同じ） |

### 試したが入れなかったこと：StopMove の後に「立つ」へ移す（橋渡し）

- 実機は StopMove で止まって立つ。Gazebo の橋渡しで、歩くのを止めた StopMove の 0.3 秒後に controller を「立つ」（fixed stand）に移し、次の Move で「歩く」に入り直す形を、テストを付けて入れ、一周を 2 回流した。
- 立っている間は胴がまったく揺れなかった（roll 0.00〜0.02）。前進の巡航も roll 0.1 以下だった。
- しかし、入り直すたびに 2.5 秒（立つの 1.8 秒 + 0.5 秒）止まり、2 回とも転んだ：1 回は前進の終わりのゆっくりの所、もう 1 回は「歩く」に入り直した直後（回る指令を入れる前）に倒れた。
  controller は「歩く」に入るときの足の位置を、その時の足の置き場所から取る（`FeetEndCalc::init`）。歩いた後の足はそろっていないので、入り直すと足の置き場所がずれる。立ち上がりの後（足がそろっている）と違う。
- 悪くなる場合があり、遅くもなるので入れなかった（`gazebo/bridge_core.py` は大きさの `StillGate` だけ）。

## 負荷が重いとき（lap0・lapC）

- lap0（13:14）・lapC（13:34）の間は、ほかの担当のテストで RTF 0.6〜0.9、`/clock`・odometry が 0.3〜1.2 秒止まることがあった（lap0 の間に 60 秒記録して、`/clock` の 0.3 秒より長い止まりが 4 回（最長 0.81 秒）、odometry は最長 1.19 秒。その 60 秒の RTF 0.6）。
- そのたびに `/sportmodestate` が 0.5 秒途切れて `センサー（LiDAR か /sportmodestate）が途切れています` で一時停止し、再開した（設計どおり）。
- lapC では傾いた LiDAR が地面を「前方の物」と見て `前方に物が出た` が何度か出て、計画が変わり、箱の縁まで 0.495 m まで近づいた（§2 の 0.6 m を割った。ぶつかってはいない）。

## 第 2 部：これから（Part 2: to do）

- 本体のフォルダ（`~/work/physicalAI/go2`）で、wave 3 を merge した後に：
  - 手順 7：操作画面（ポート 8811）で「箱の周りを一周して」と話し、上から見た図（箱・椅子・0.5 m の格子・灰色のふさがったマス・青い道）、
    Go2 の札の「見えている物：箱（左前 2.9 m）、椅子（右前 2.9 m）」、環境のタブの代わりの文を確かめ、`docs/reference/gazebo_objects_map.png` を撮る。
    転ぶことがあるので、転んだら Gazebo を起動し直してやり直す。
  - 静かなとき（ほかのテストが動いていないとき）の RTF を測り直して、この文書の手順 4 に足す。
  - 手順 5・6 を本体のフォルダでもう一度流すなら、この文書の「手順 5・6」の表に足す。
- 転ぶのを無くすには、Gazebo の controller を直す必要がある（下の「気づいたこと」）。その上で手順 5・6 を流し直す。

## 気づいたこと（このタスクでは直していない）

- **Gazebo の controller が、止まっては動く歩き方に弱い。** 直すなら `docker/patches/` にパッチを足してイメージを作り直す（このタスクの決まりでは作り直さない）。候補：
  - StopMove の後（指令 0）は足踏みをやめて立つ（`checkStepOrNot` がずれの大きさだけで足踏みを始め・止めるのをやめる）。
  - 「歩く」に入るときの足の置き場所を、決まった立ち方（`feet_pos_normal_stand_`）から取る（今は入った時の足の置き場所。歩いた後に入り直すとずれる）。
  - 回る目標の向き `yaw_cmd_` を、本当の向きから離れすぎないように切る（`gazebo_turn_fix.md` の H2）。
- **一周の計画は回る回数が多い。** BFS の同じ長さの道の選び方で、出だしが「左 → 前 → 右 → 前 → 前 → 前 → 右 → 前 → 左 …」の階段になり、一周 39〜41 動作のうち 10〜11 回が 90 度の旋回（Task B・A の planner。持ち主でないので触っていない）。
  旋回は転ぶ場所の 1 つなので、回る回数の少ない道を選べれば Gazebo では転びにくくなるはず。
- **最初の動作が、いちばん速い指令で一度に始まることがある。** Session の「立つ」が終わると最初の動作の Move が来るが、controller が「歩く」に入るのはその後のことがある。
  橋渡しは R22 のとおり、歩くに入って 0.5 秒（`TROT_WAIT_S`）まで速度を待たせ、そのとき届いている最後の値を一度に入れる。その間に実行器は加速を終えているので、
  回る指令が 0 から 0.5 rad/s（rx −1.00）に一度に変わる（FL1・FL2・FL3・FN1 の橋渡しのログ。lapG の後の回のように、歩くに入った後に Move が来た回はふつうに上がる）。
  FL3 はこの形で回り始め、胴がまだ上がりきらない（0.30 → 0.33 m）うちに揺れて倒れた。`gazebo_turn_fix.md` の turn_check は歩くに入って 1 秒待っていた。
  直すなら、橋渡しで待たせた速度を 0 から上げる（`tests/test_bridge_core.py` の `test_stand_then_walk_then_expire` が今の「1 通で入る」を決めていて、このタスクの持ち物でないので試していない）。
- **終わった後も「歩く」のまま。** `run_formula` が終わると Go2 は速度 0 で足踏みを続け、near0 では終わった後 1.5 秒で roll が 0.86 まで育ってから収まった。`objects_check.py` は終わって 1 秒（`TAIL_S`）しか見ないので、その後に転んでも数えない。
- `objects_check.py objects` の終了コードは大きさの範囲（0.3〜0.8）を見ない。手順 4 の大きさは目で確かめた。

## 再現のしかた

```bash
cd <このリポジトリ>
docker compose --profile gazebo run -d --rm --no-deps -T -e ROS_DOMAIN_ID=77 --name go2-objects-gz gazebo \
    ros2 launch /work/gazebo/launch/go2_gazebo.launch.py gui:=false      # controller が起動するまで待つ
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py objects
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py rtf --seconds 20
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
docker stop go2-objects-gz                                                   # 次の回の前（転んだ後はかならず）
```
