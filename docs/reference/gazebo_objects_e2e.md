# Gazebo での物の確認：箱の一周と椅子まで（Task I）

設計書 `docs/superpowers/specs/2026-11-05-go2-objects-design.md` の §2 の合格の条件を、LLM を通さずに Gazebo で測った記録。
この文書は **第 1 部**（計画の Task I の手順 4〜6：物の一覧・RTF・箱の一周・椅子まで）。操作画面の確認と画面の絵（手順 7）、
本体のフォルダでの RTF は **第 2 部** で足す（最後の「第 2 部：これから」）。

- 日付：2026-11-06。場所：この Mac の Docker Desktop（arm64、CPU 8、メモリ 7.7 GiB）。
- 回は 2 つの時間帯に分かれる：
  - **最初の回（13:11〜14:10）**：実行器の値を変えながら 15 回の一周と 5 回の椅子。この間、別の担当（Task H・Task L）がそれぞれのフォルダでテストを流していた
    （`obj-h-app-run-*`・`obj-l-app-run-*` が 100〜120 %）。とくに 13:14 と 13:34 は重かった（下の「負荷が重いとき」）。
  - **修正 1 回目（14:37〜14:46）**：レビューを受けて、実行器を計画の P7 に戻し、橋渡しを直して（下の「最初の動作の速度を 0 から上げる」）、手順 5・6 の 7 回を流し直した。
    ほかのコンテナは動いていなかった（各回の `docker stats` は `go2-objects-gz` と `go2-objects-rec` だけ）。**この 7 回が最後の結果**。
- 使ったもの：app は `go2-voice-ltl:dev`（ROS 2 Humble）、gazebo は `go2-voice-ltl-gazebo:dev`（ROS 2 Jazzy + Gazebo Harmonic、`docker/patches/` の 0001・0002 入り。作り直していない）。
  コードは `objects/i` のフォルダ（`../.go2-worktrees/obj-i`、compose の project は `obj-i`）。最後の回は commit `97c3c64`：
  実行器（`voice_ltl/grid_executor.py`）は計画の P7 の値のまま、橋渡し（`gazebo/bridge_core.py`）は大きさの `StillGate` と、待たせた速度を 0 から上げる `RELEASE_ACC` 入り。
- Gazebo は毎回、使い捨てのコンテナで起動し直した（compose の service は起動していない。計画の P8）：

```bash
docker stop go2-objects-gz       # 前の回の、自分で起動した使い捨てのコンテナだけ
docker compose --profile gazebo run -d --rm --no-deps -T -e ROS_DOMAIN_ID=77 --name go2-objects-gz gazebo \
    ros2 launch /work/gazebo/launch/go2_gazebo.launch.py gui:=false
# ログに Configured and activated unitree_guide_controller が出るまで待つ（イメージがキャッシュにあると 8〜10 秒）
```

  計画の手順 4 は `-d` なしで後ろで動かす形。ここでは `-d`（コンテナを切り離す）にしただけで、ほかは同じ。
- 測り方：
  - `gazebo/tools/objects_check.py`。`run` は `/model/robot/odometry`（Gazebo の本当の位置と傾き）を記録しながら `run_formula` を子として流し、§2 の値と合否を JSON 1 行で出す。
  - 動作ごとの傾きは、別の使い捨てコンテナ（`go2-objects-rec`）で `gazebo/tools/motion_log.py record` を回して、odometry（25 Hz）・`/api/sport/request`・`/go2/objects` の大きさの変化を記録し、
    `motion_log.py table` で数えた（1 つの動作 = StopMove の後の最初の Move から、次の動作の最初の Move の前まで。止めた後の 0.5 秒の待ちも入る）。
    最初の回は同じ中身の道具をコミットせずに使っていた。修正 1 回目で `motion_log.py` としてコミットし、最初の回の記録も数え直した（下の「動作ごとの最大傾き」の数はどれもこの道具の出力）。
  - 橋渡しが controller に送った値は、Gazebo のコンテナのログ（`controller へ: command …`）で見た。

## 結論

| 確かめたこと | 結果 |
|---|---|
| 物の一覧（手順 4） | `box_1`・`chair_1` だけ（`ground_plane` などは無い）、どちらも `visible: true`、位置は (2.5, 1.5)・(2.5, −1.5) ぴったり |
| 物の大きさ（手順 4） | **最初は合格しなかった**：箱 1.043 × 1.117、椅子 1.073 × 1.186（0.3〜0.8 の外）。Go2 を出した直後に床へ落ちる間の点群で膨らんでいた。**直した**（`StillGate`）後は 0.511 × 0.51〜0.53・0.462〜0.469 × 0.461〜0.467（2 回は測る前の既定の 0.5 × 0.5）。走った後も 0.51〜0.54・0.46〜0.48 |
| RTF（手順 4、20 秒） | 修正 1 回目（静か）は **0.99〜1.0**。最初の回は 0.96〜1.0。どちらも画面なし（`gui:=false`）。負荷が重いときの走っている間は 0.6〜1.0、odometry が 1.2 秒止まることもあった（下の「負荷が重いとき」） |
| 箱の一周（手順 5、最後の設定で 3 回） | **0/3**。3 回とも前進の途中で Go2 が転び（max_tilt 3.09〜3.13）、`姿勢が変わったので止めました` で `exit=1`。回る途中・最初の動作では転んでいない |
| 椅子まで（手順 6、最後の設定で 3 回 + 挨拶 1 回） | **3/4 合格**（RN1・RN3・RH1。椅子まで 1.16〜1.23 m、19〜25°、椅子の縁 0.81〜0.90 m）。RN2 は前進の歩き出しで転んだ（max_tilt 1.273）。挨拶の回は「開始：挨拶」「完了：挨拶」まで行った。どの回も最初の動作は「立つ」 |
| 最初の動作の速度の跳び（R22 の後） | **直した**。7 回とも最初の Move は controller が歩くに入る前に届き、橋渡しはそれを 0.5 秒かけて 20 Hz で上げた（rx は 0.1 ずつ、前進は ly 0.075 ずつ）。最初の動作で転んだ回は無い（傾き 0.10〜0.45） |
| 調べた全部（18 回の一周、9 回の椅子） | 一周を終えたのは 1 回だけ（最初の回の lap1：P7、回転数 1.039、箱の縁まで 0.851 m、max_tilt 0.707）。P7 の静かなときの一周は 4 回で 1 回、調整した値の一周は 13 回で 0 回。椅子は 9 回で 7 回合格 |
| 転ぶ原因 | Gazebo の unitree_guide controller。格子の計画は 0.5 m・90 度ごとに StopMove で止まり、止まった controller が次の Move で足踏みを始める所（歩き出し）と、ゆっくり歩く所で倒れる。最後の設定では、転んだ 4 回のうち 3 回が止まった所からの歩き出し（0.3〜0.5 秒で roll 1 を越える。RL2・RN2 は roll 0.02 以下の落ち着いた所から）、1 回が前進の終わりのゆっくりの所。回るのは 21 回で 0 回（下の「転んだこと」） |

**§2 の合格の条件（一周 3 回・椅子 3 回）は、Gazebo では満たせていない。** 計画の P7 の値でも、試した調整した値でも満たせなかった。
計画を立てる側（監督器・計画・Session）は、転ぶまでどの回も計画どおりに動いた（`位置のずれ` で止まった回は無い）。
転ぶのは Gazebo の controller の問題。実行器の値（P7 と試した値）と橋渡しの直し（大きさ・最初の動作の上げ方、止めた後に立たせる形も試した）では無くならなかった。
残る手は controller にパッチを当ててイメージを作り直すこと（このタスクの決まりではしない。最後の「気づいたこと」）。

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

- 修正 1 回目の RL3・RH1 の手順 4 では、大きさがどちらも `[0.5, 0.5]` だった。これは測る前の既定の大きさ（`objects.DEFAULT_SIZE`）で、`StillGate` を通った点群がまだ無かったため。
  走った後はどちらも測った値（0.516 × 0.516・0.47 × 0.479 など）になっていた。

（各回の手順 4 と走った後の大きさは、下の「手順 5・6」の表。）

## 手順 5・6：最後の設定（修正 1 回目、commit `97c3c64`）での回

実行器は計画の P7（`K_YAW` 1.5・`K_CROSS` 1.0・前進 clip(1.2 × 残り, 0.08, 0.3)・歩くときの z は ±0.3・回るのは 0.15〜0.5 rad/s・加速 (0.6, 0.6, 1.0)）。
橋渡しは `StillGate` と、待たせた速度を 0 から上げる `RELEASE_ACC`。1 回ごとに Gazebo を起動し直し、手順 4 の 2 つを流してから：

```bash
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 --name go2-objects-rec app \
    python3 gazebo/tools/motion_log.py record --seconds 900 > <回>.jsonl &      # 動作ごとの傾き（測るだけ）
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect near "F(near_chair_1 & X F hello)"; echo "exit=$?"
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py objects   # 走った後の大きさ
docker stop go2-objects-rec
```

| 回 | 式 | 手順 4 の RTF | 手順 4 の大きさ（箱 / 椅子） | 結果 | 値 | 動作 | 走った後の大きさ |
|---|---|---|---|---|---|---|---|
| RL1 | `F around_box_1` | 1.0 | 0.511×0.514 / 0.469×0.467 | 不合格：winding、min_edge_box_1、max_tilt、exit_code（転んだ：前進の歩き出し） | 回転数 0.345、箱の縁 0.205 m（転んだ後に転がって近づいた）、max_tilt 3.126 | 22/41 | 0.511×0.526 / 0.469×0.471 |
| RL2 | `F around_box_1` | 0.99 | 0.511×0.515 / 0.469×0.467 | 不合格：winding、max_tilt、exit_code（転んだ：回った後の前進の歩き出し） | 回転数 0.129、箱の縁 0.801 m、max_tilt 3.094 | 9/41 | 0.515×0.53 / 0.469×0.471 |
| RL3 | `F around_box_1` | 1.0 | 0.5×0.5 / 0.5×0.5（測る前） | 不合格：winding、max_tilt、exit_code（転んだ：前進の終わりのゆっくりの所） | 回転数 0.104、箱の縁 1.192 m、max_tilt 3.108 | 7/31 | 0.516×0.516 / 0.47×0.479 |
| RN1 | `F near_chair_1` | 0.99 | 0.511×0.514 / 0.469×0.467 | **合格** | 椅子まで 1.162 m、20.7°、椅子の縁 0.811 m、max_tilt 0.412 | 8/8 | 0.533×0.527 / 0.469×0.471 |
| RN2 | `F near_chair_1` | 0.99 | 0.511×0.515 / 0.469×0.467 | 不合格：final_dist_chair_1、max_tilt、exit_code（転んだ：回った後の前進の歩き出し） | 椅子まで 2.363 m、9.5°、max_tilt 1.273 | 6/8 | 0.519×0.527 / 0.469×0.474 |
| RN3 | `F near_chair_1` | 0.99 | 0.511×0.511 / 0.468×0.467 | **合格** | 椅子まで 1.228 m、19.3°、椅子の縁 0.902 m、max_tilt 0.879 | 9/8（途中で計画が変わった） | 0.513×0.515 / 0.468×0.468 |
| RH1 | `F(near_chair_1 & X F hello)` | 1.0 | 0.5×0.5 / 0.5×0.5（測る前） | **合格** | 椅子まで 1.185 m、25.3°、椅子の縁 0.858 m、max_tilt 0.446 | 8/8 | 0.514×0.526 / 0.462×0.479 |

「動作」は `開始：` の数 / 計画の動作の数（立つ・挨拶を含む）。

```text
# RL1（14:37:37）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.345, "min_edge_box_1": 0.205, "min_edge_chair_1": 1.137, "final_dist_chair_1": 2.83, "final_angle_chair_1_deg": 175.5, "max_tilt": 3.126, "passed": false, "failures": ["winding: 0.345 not in [0.9, 1.5)", "min_edge_box_1: 0.205 < 0.6", "max_tilt: 3.126 >= 1", "exit_code: 1 != 0"], "samples": 3719}
exit=1
# RL2（14:39:33）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.129, "min_edge_box_1": 0.801, "min_edge_chair_1": 1.527, "final_dist_chair_1": 1.947, "final_angle_chair_1_deg": 22.8, "max_tilt": 3.094, "passed": false, "failures": ["winding: 0.129 not in [0.9, 1.5)", "max_tilt: 3.094 >= 1", "exit_code: 1 != 0"], "samples": 1624}
exit=1
# RL3（14:40:43）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
{"formula": "F around_box_1", "expect": "lap", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.104, "min_edge_box_1": 1.192, "min_edge_chair_1": 0.788, "final_dist_chair_1": 1.1, "final_angle_chair_1_deg": 19.8, "max_tilt": 3.108, "passed": false, "failures": ["winding: 0.104 not in [0.9, 1.5)", "max_tilt: 3.108 >= 1", "exit_code: 1 != 0"], "samples": 1353}
exit=1
# RN1（14:41:49）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
{"formula": "F near_chair_1", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.1, "min_edge_box_1": 2.281, "min_edge_chair_1": 0.811, "final_dist_chair_1": 1.162, "final_angle_chair_1_deg": 20.7, "max_tilt": 0.412, "passed": true, "failures": [], "samples": 1457}
exit=0
# RN2（14:42:59）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
{"formula": "F near_chair_1", "expect": "near", "exit_code": 1, "first_action": "立つ", "winding_box_1": 0.05, "min_edge_box_1": 2.441, "min_edge_chair_1": 2.115, "final_dist_chair_1": 2.363, "final_angle_chair_1_deg": 9.5, "max_tilt": 1.273, "passed": false, "failures": ["final_dist_chair_1: 2.363 > 1.5", "max_tilt: 1.273 >= 1", "exit_code: 1 != 0"], "samples": 1127}
exit=1
# RN3（14:44:01）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F near_chair_1"; echo "exit=$?"
{"formula": "F near_chair_1", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.097, "min_edge_box_1": 2.193, "min_edge_chair_1": 0.902, "final_dist_chair_1": 1.228, "final_angle_chair_1_deg": 19.3, "max_tilt": 0.879, "passed": true, "failures": [], "samples": 1747}
exit=0
# RH1（14:45:14）
$ docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app python3 gazebo/tools/objects_check.py run --expect near "F(near_chair_1 & X F hello)"; echo "exit=$?"
{"formula": "F(near_chair_1 & X F hello)", "expect": "near", "exit_code": 0, "first_action": "立つ", "winding_box_1": 0.123, "min_edge_box_1": 1.106, "min_edge_chair_1": 0.858, "final_dist_chair_1": 1.185, "final_angle_chair_1_deg": 25.3, "max_tilt": 0.446, "passed": true, "failures": [], "samples": 1470}
exit=0
```

- **箱の一周は 3 回とも前進で転んだ**（22・9・7 動作目）。転ぶまでの 12 回の旋回では転んでいない（傾き 0.10〜0.60）。
  - RL1：止めた後 0.7 秒で前進を始めた所（roll −0.07）。x 0.06 → 0.24 m/s・z −0.07〜−0.13 rad/s の 0.42 秒で roll が −0.48・−0.89・−1.14。
    `min_edge_box_1` 0.205 は転んだ後の値：倒れた Go2 が箱の方へ 1.2 m 転がった。転ぶ前（立って 77 秒まで）の箱の縁までの最小は 0.80 m。
  - RL2：回り終わって落ち着いた所（roll −0.02）から前進を始め、x 0.06 → 0.24・|z| ≤ 0.06 の 0.32 秒で roll −0.15・−0.49・−1.10。
  - RL3：前進の終わりのゆっくりの所（x 0.20 → 0.08、z +0.08〜+0.10）で、0.64 秒かけて roll 0.11 → 0.29 → 0.64 → 0.91 → 1.02。
- **椅子まで：4 回のうち 3 回合格**（RN1・RN3・RH1）。RH1 のログには「開始：挨拶」「完了：挨拶」がある。
  - RN2：回り終わって落ち着いた所（roll +0.02）から前進を始め、x 0.06 → 0.24・|z| ≤ 0.05 の 0.47 秒で roll +0.16・+0.33・+0.69・+1.19。
  - RN3：左に回る途中で胴が 0.88 まで傾き、傾いた LiDAR が地面を前の物と見て `前方に物が出た`・`前方の物が消えた` が出た。
    その後の動作が計画から変わり（左に回った後「前進 → 右に回る → 左に回る → 前進」）、`結果：done` で合格（計画 8 動作に対して 9 動作）。lapC と同じ（下の「負荷が重いとき」）。
- 椅子の回の最後の距離は 1.16〜1.23 m で、R4 の帯（1.4 m まで）の中に余裕を持って着いた。
- 走った後の大きさ：箱 0.511〜0.533 × 0.515〜0.530、椅子 0.462〜0.470 × 0.468〜0.479（どれも 0.3〜0.8 の中）。

## 最初の動作の速度を 0 から上げる（橋渡し、修正 1 回目）

- 前：Session の「立つ」が終わると最初の動作の Move が来るが、controller が「歩く」に入るのはその後のことがある。橋渡しは R22 のとおり、歩くに入って 0.5 秒（`TROT_WAIT_S`、シミュレーションの時間）まで速度を待たせ、
  そのとき届いている最後の値を **一度に** 入れていた。その間に実行器は加速を終えているので、回る指令が 0 から 0.5 rad/s（rx −1.00）に一度に変わった。
  最初の回の FL1・FL2・FL3・FN1 の橋渡しのログにこの跳びがあり、FL3 はこの形で回り始めて、胴がまだ上がりきらない（0.30 → 0.33 m）うちに揺れて倒れた。
- 直したこと（`gazebo/bridge_core.py`）：状態を変えた後の最初の速度は、歩いてよくなった時刻（`_walk_from`）から、`RELEASE_ACC` = 前後 0.6 m/s²・左右 0.6 m/s²・旋回 1.0 rad/s²
  （`GridExecutor` の `GRID_EXEC_CONFIG.acc` と同じ）で、シミュレーションの時間に合わせて 0 から上げる。
  - 遅くする・止める（StopMove、転んだとき）は今までどおりすぐ入れる。
  - controller の速度が Move の値に追いついたら上げるのは終わり。その後の Move は今までどおりそのまま入れる（実行器が自分で加速を守っている）。
  - 伏せ・おすわりから立ってまた歩くに入ったら、また 0 から上げる。
  - テスト：`tests/test_bridge_core.py`（R22 の 4 つのテストを上げ方に合わせて直し、上げ方の 4 つを足した）。
- Gazebo で（RL1 の橋渡しのログ。controller は 1793943464.787 に歩くに入った）：

```text
[INFO] [1793943463.603357917] [go2_gazebo_bridge]: 受信: BalanceStand 1002
[INFO] [1793943463.641927209] [go2_gazebo_bridge]: controller へ: command 4, lx 0.00, ly 0.00, rx 0.00
[INFO] [1793943464.141846042] [go2_gazebo_bridge]: error_code 1002 → 1013
[INFO] [1793943464.842351126] [go2_gazebo_bridge]: error_code 1013 → 1015
[INFO] [1793943465.291950251] [go2_gazebo_bridge]: controller へ: command 4, lx 0.00, ly 0.00, rx -0.01
[INFO] [1793943465.341838460] [go2_gazebo_bridge]: controller へ: command 4, lx 0.00, ly 0.00, rx -0.11
[INFO] [1793943465.391873960] [go2_gazebo_bridge]: controller へ: command 4, lx 0.00, ly 0.00, rx -0.21
...（0.05 秒ごとに 0.1 ずつ）
[INFO] [1793943465.741802502] [go2_gazebo_bridge]: controller へ: command 4, lx 0.00, ly 0.00, rx -0.91
[INFO] [1793943465.791886210] [go2_gazebo_bridge]: controller へ: command 4, lx 0.00, ly 0.00, rx -1.00
```

- 7 回とも同じ：最初の Move は 1015 になってから 0.40〜0.50 秒待たされ、0.50〜0.55 秒かけて上がった（RL1〜RN3 は回る rx ±1.00 まで、RH1 は前進 ly 0.06 → 0.75）。
  最初の動作の傾きは 0.10〜0.45 で、転んだ回は無い（最初の回は、跳びのあった 4 回のうち FL3 が最初の動作で転んだ）。
- 7 回は少ないので、これで最初の動作の転びが無くなったとまでは言えない。言えるのは「跳びは無くなった」「上げた 7 回で最初の動作では転んでいない」まで。

## 転んだこと：調べたことと調整

### 回ごと（時刻順）

設定の列：P7 = 計画の P7 のまま。lapA〜FL3 は P7 から変えたもの（下の「試して戻した値」）。修正 1 回目（RL1〜RH1）は P7 に戻し、橋渡しの上げ方を足した。
「動作」は `開始：` の数 / 計画の動作の数。

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
| FL1 | 14:04:32 | e47e11e：回る 0.5 rad/s・加速 0.6 に戻す（ほかの調整は残す。修正 1 回目で戻した） | 不合格（exit 1） | 4/39 | 右に回り終わる所 | 3.139 |
| FL2 | 14:05:29 | 同上 | 不合格（exit 1） | 10/41 | 前進 → 前進 の後の左に回る | 3.117 |
| FL3 | 14:06:43 | 同上 | 不合格（exit 1） | 2/39 | 立って最初の左に回る途中（速度の跳び） | 3.109 |
| FN1 | 14:07:34 | 同上 | 合格（exit 0） | 8/8 | — | 0.588 |
| FN2 | 14:08:40 | 同上 | 不合格（exit 0） | 7/7 | — | 0.74 |
| FN3 | 14:09:41 | 同上 | 合格（exit 0） | 7/7 | — | 0.507 |
| FH1 | 14:10:43 | 同上 | 合格（exit 0） | 8/8 | — | 0.589 |
| RL1 | 14:37:37 | **P7 + StillGate + 上げ方（修正 1 回目、最後）**、静か | 不合格（exit 1） | 22/41 | 前進の歩き出し（止めた後 0.7 秒） | 3.126 |
| RL2 | 14:39:33 | 同上 | 不合格（exit 1） | 9/41 | 回った後の前進の歩き出し（落ち着いた所から） | 3.094 |
| RL3 | 14:40:43 | 同上 | 不合格（exit 1） | 7/31 | 前進の終わりのゆっくりの所 | 3.108 |
| RN1 | 14:41:49 | 同上 | 合格（exit 0） | 8/8 | — | 0.412 |
| RN2 | 14:42:59 | 同上 | 不合格（exit 1） | 6/8 | 回った後の前進の歩き出し（落ち着いた所から） | 1.273 |
| RN3 | 14:44:01 | 同上 | 合格（exit 0） | 9/8 | — | 0.879 |
| RH1 | 14:45:14 | 同上 | 合格（exit 0） | 8/8 | — | 0.446 |

最初の回の JSON（最後の設定の 7 回は上の「手順 5・6」）：

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

- FL1〜FH1（e47e11e）の手順 4 と走った後の大きさ：

| 回 | 手順 4 の RTF | 手順 4 の大きさ（箱 / 椅子） | 値 | 動作 | 走った後の大きさ |
|---|---|---|---|---|---|
| FL1 | 0.98 | 0.511×0.513 / 0.468×0.467 | 回転数 0.012、箱の縁 2.132 m、max_tilt 3.139 | 4/39 | 0.517×0.526 / 0.468×0.477 |
| FL2 | 0.99 | 0.511×0.514 / 0.469×0.467 | 回転数 0.201、箱の縁 0.957 m、max_tilt 3.117 | 10/41 | 0.511×0.526 / 0.469×0.471 |
| FL3 | 0.98 | 0.511×0.51 / 0.468×0.467 | 回転数 -0.011、箱の縁 2.477 m、max_tilt 3.109 | 2/39 | 0.512×0.51 / 0.468×0.47 |
| FN1 | 0.98 | 0.511×0.509 / 0.468×0.467 | 椅子まで 1.109 m、27.2°、椅子の縁 0.704 m、max_tilt 0.588 | 8/8 | 0.512×0.509 / 0.468×0.47 |
| FN2 | 0.99 | 0.511×0.515 / 0.469×0.467 | 椅子まで 1.538 m、6.8°、椅子の縁 1.13 m、max_tilt 0.74 | 7/7 | 0.511×0.515 / 0.469×0.471 |
| FN3 | 0.99 | 0.511×0.527 / 0.462×0.461 | 椅子まで 1.107 m、24.8°、椅子の縁 0.753 m、max_tilt 0.507 | 7/7 | 0.511×0.527 / 0.462×0.474 |
| FH1 | 1.0 | 0.513×0.516 / 0.47×0.471 | 椅子まで 1.495 m、33.2°、椅子の縁 1.087 m、max_tilt 0.589 | 8/8 | 0.517×0.516 / 0.47×0.481 |

- **FN2 は距離で不合格（1.538 m > 1.5 m）。** 転んではいない（`結果：done`、run_formula の終了コード 0）。このときの実行器は調整した値（歩くときの横のずれの直し `K_CROSS` 0.4 など。修正 1 回目で戻した）。
  - 計画は「立つ → 前進 ×5 → 右に回る」。立ったときの向きが −0.02〜−0.05 rad で、行き先のマス (5, 0) は、近い帯（R4：r + 0.75 〜 min(r + 1.5, 1.4) m）の外側の線 1.4 m のすぐ内側だった
    （向き −0.045 rad なら 1.397 m。起点の正確な値は Session の内部のログにしか無いので、記録した位置から見積もった）。
  - 記録した位置：前進 5 回の後 (2.486, −0.031)、椅子まで 1.469 m。その場で右に回る間に胴が 0.1 m 流れ、(2.40, 0.04)、1.538 m で終わった。
  - R4 は終わりのずれに 0.1 m を残している。Gazebo では、前進の横のずれ（±0.1 m）とその場の旋回で胴が流れる分（0.1 m）を足すと、それを越えることがある。
    弱くした横のずれの直し（P7 の 1.0 に対して 0.4）も、前進 5 回の間に横のずれが残った分（レビューの見積もりで 0.07 m ほど）に効いていたと考えられる。
    近い帯の外側の線（1.4 m）は `voice_ltl/grid.py`（このタスクの持ち物でない）。旋回の間に位置を直さないのは `GridExecutor` の作り（回る間は向きだけ見る）。
    P7 に戻した後の椅子の回（RN1・RN3・RH1）は 1.16〜1.23 m で着いた。

### 動作ごとの最大傾き（記録から）

1 つの動作の間の max(|roll|, |pitch|)。数は `motion_log.py table` の出力（最初の回の記録も同じ道具で数え直した）。「0.5〜1.0」は揺れの大きかった動作、「転んだ」は 1.0 以上（橋渡しが 1.0 rad を越えたら転んだとして止める）。
lapE・lapF（橋渡しで止めた後に立つ）は別の作りなので入れていない。

| 設定（回） | 動作 | 数 | 中央値 | 上位 10 % | 0.5〜1.0 | 転んだ |
|---|---|---|---|---|---|---|
| P7（lap1・near0） | 前進 | 33 | 0.29 | 0.57 | 6 | 0 |
| 同上 | 回る | 12 | 0.23 | 0.35 | 0 | 0 |
| z の上限を速さに比べる 0.3（lapA） | 前進 | 4 | 0.63 | 1.06 | 2 | 1 |
| 同上 | 回る | 2 | 0.16 | 0.17 | 0 | 0 |
| 歩くときの直しを弱く、回る 0.5 rad/s（lapB・lapD） | 前進 | 18 | 0.16 | 0.42 | 0 | 0 |
| 同上 | 回る | 11 | 0.31 | 3.06 | 0 | 2 |
| 回るときの K_YAW 1.0（lapC、負荷が重い） | 前進 | 25 | 0.31 | 0.61 | 5 | 1 |
| 同上 | 回る | 15 | 0.40 | 0.90 | 6 | 0 |
| 歩くときの直しを弱く、終わりを短く、回る 0.3 rad/s（lapG・L1〜L3） | 前進 | 45 | 0.26 | 0.50 | 3 | 2 |
| 同上 | 回る | 27 | 0.32 | 0.88 | 3 | 2 |
| e47e11e：同上で回る 0.5 rad/s・加速 0.6（FL1〜FL3・FN1〜FN3・FH1） | 前進 | 26 | 0.29 | 0.59 | 5 | 0 |
| 同上 | 回る | 13 | 0.43 | 3.04 | 2 | 3 |
| **P7 + 上げ方（最後：RL1〜RL3・RN1〜RN3・RH1）** | 前進 | 40 | 0.30 | 0.51 | 1 | 4 |
| 同上 | 回る | 21 | 0.38 | 0.58 | 3 | 0 |

- まとめると、P7（lap1・near0 と最後の 7 回）は前進 73 回で 4 回、回る 33 回で 0 回転んだ。調整した値（lapA〜lapD、lapG〜L3、FL1〜FH1）は前進 118 回で 4 回、回る 68 回で 7 回転んだ。
  回る 0.5 rad/s と下限 0.25 rad/s を合わせた回（lapD・FL1〜FL3。橋渡しを変えた lapE・lapF を除く）は、4 回とも回る所で転んだ。
- どの設定でも、一周（30〜41 動作）の間に 1 回以上転ぶことが多い。1 回の前進で転ぶ割合が 5 % ほどでも、一周の 20 回あまりの前進を転ばずに通るのは 3 回に 1 回ほどになる。数が少ないので、設定の違いの差ははっきりとは言えない。
- 前進の揺れは、歩くときの向きの直しを弱くすると中央値が下がった（P7 の 0.29 に対して 0.16〜0.29）が、転ぶ回は無くならず、一周も終わらなかった。それで P7 に戻した（下の「試して戻した値」）。

### 何が起きて転ぶか（記録から）

- **止まった controller が歩き始める所。** StopMove の後、controller は「歩く」のまま速度 0 で、足踏みを止めたり始めたりする（`StateTrotting::checkStepOrNot`：指令も位置・速さのずれも小さいと足踏みを止め、ずれが育つとまた踏む）。
  最後の設定で転んだ 4 回のうち 3 回（RL1・RL2・RN2）は、次の前進を始めて 0.3〜0.5 秒で roll が 1 を越えた。そのうち RL2・RN2 は roll 0.02 以下の落ち着いた所からで、z も 0.06 rad/s 以下、x は実行器の 0.6 m/s² の上げ方の途中（0.06〜0.24 m/s）。
  歩くときの向きの直しや止めた後の揺れが無くても転ぶので、controller が足踏みを始めるときの問題と考えられる。
- **止めた後の横揺れ。** 胴は 1.5 秒ほどの周期で横に揺れ（roll 0.6 rad、向き ±0.15 rad、横に ±0.1 m）、その揺れの中で次の動作を始めると倒れることがある。
  例：lapG で、前進の後の 0.5 秒の待ちの間に胴が後ろへ 0.14 m 流れ、次の前進の歩き出しで roll が 0.3 から 1.0 になって転んだ。
- **歩くときの向きの直しが揺れを育てる。** P7 の値（向き 1.5 /s、横のずれ 1.0 rad/(m·s)）だと、z（回る指令）が揺れに合わせて ±0.25 rad/s で振れ、前進 → 前進 で roll 0.55〜1.06 になったことがある（最初の回）。
  ただし、直しを弱くした回でも転ぶのは減らなかった（上の表）。
- **ゆっくり歩くと傾く。** 前進の終わりの 0.08〜0.16 m/s のところで、胴が横へ流れながら傾いた（lapE で 1.07、RL3 で 0.64 秒かけて 0.11 → 1.02）。
- **回る途中・回り終わり。** 落ち着いて立っている所から回り始めて 0.3 秒で倒れた回（L1、向き 0°）、回り終わりの StopMove の後に 41 度回り続けて倒れた回（lapB）がある。どちらも調整した値のとき。
  回り終わり：controller は回る指令を積んで目標の向きを作る。0.5 rad/s の指令に対して Go2 は平均 0.42 rad/s で回り、目標は最大 10 度先に行った（橋渡しが送った値を積んで測った。lapB）。0.3 rad/s では 4 度以内だったが（L1）、転ぶ回は減らなかった。
- **最初の動作の速度の跳び。** 上の「最初の動作の速度を 0 から上げる」。直した。
- 原因ではなかったもの：
  - 負荷：手順 4 の RTF が 0.96〜1.0 で、ほかのコンテナがほぼ止まっていた回（L1〜L3・FL1〜FL3）や、まったく止まっていた回（RL1〜RN2）でも転んだ。
  - 向き ±180 度：lap1 は ±180 度をまたいで何度も歩いて止まり、転ばなかった。向き 0°・−90°・+90° でも転んだ。
  - 物の大きさ・計画：転ぶ前はどの回も計画どおり（`位置のずれ` は 1 回も出なかった）。

### 試して戻した値（`voice_ltl/grid_executor.py`。修正 1 回目で P7 に戻した）

最初の回に下の値を入れ（commit `8c55d0e`・`e47e11e`）、修正 1 回目で P7 に戻した（commit `c2330be`。テスト `test_gains_are_p7` が P7 の値を決める）。

| 値 | P7（今） | 試した値 | 試したわけ | 戻したわけ |
|---|---|---|---|---|
| 歩くときの向きの直し | `K_YAW` 1.5 /s | `K_WALK_YAW` 0.6 /s | 揺れに合わせて z が振れ、揺れを育てた | 前進の揺れの中央値は下がったが、調整した値の一周は 0/13（P7 は静かなときに 1/4）。横のずれの直りが遅くなり、テストの線を 0.1 から 0.11 に緩める必要があった |
| 歩くときの横のずれの直し | `K_CROSS` 1.0 | 0.4 rad/(m·s) | 同じ | 同じ。FN2 の 0.038 m の外れにも効いていたと考えられる |
| 歩くときの z の上限 | ±0.3 rad/s | ±0.15 rad/s × (前進の速さ ÷ 0.3) | 歩き出し・止まる前（ゆっくり）に回さない。lap0 は歩き出しに 0.26 rad/s で回して転んだ | lapA（これだけ入れた回）は前進で転んだ。最後の設定で転んだ歩き出しは z 0.05 以下でも転んだ |
| 前進の速さ | clip(1.2 × 残り, 0.08, 0.3) | clip(2.0 × 残り, 0.15, 0.3) | ゆっくり歩く時間を 1 秒あまりから 0.5 秒ほどに | 一周は良くならず、StopMove で 0.15 m/s から 0 に変わる段が大きくなる（設計 §6.5「速さは急に変えない」に反する） |
| 回る速さの下限 | 0.15 rad/s | 0.25 rad/s | controller は前進 0 で回る指令が 0.2 rad/s 未満だと足踏みしない | 回る 0.5 rad/s と合わせた 4 回の一周（lapD・FL1〜FL3）が 4 回とも回る所で転んだ（P7 は回るのは 33 回で 0 回）。StopMove の段も大きくなる |
| 回る速さ・加速 | 0.5 rad/s、(0.6, 0.6, 1.0) | 0.3 rad/s・0.4 m/s² | 回り終わりに目標の向きが先に行くのを減らす | 回るのが悪くなった（lapG・L1〜L3：中央値 0.32、転んだのが 2 回）。`e47e11e` で先に戻した |
| 止めた後に待つ秒数 `GRID_SETTLE_S` | 0.5 | そのまま | 待っても揺れは収まらない（`gazebo_e2e.md` の 4 と同じ） | — |

### 試したが入れなかったこと：StopMove の後に「立つ」へ移す（橋渡し）

- 実機は StopMove で止まって立つ。Gazebo の橋渡しで、歩くのを止めた StopMove の 0.3 秒後に controller を「立つ」（fixed stand）に移し、次の Move で「歩く」に入り直す形を、テストを付けて入れ、一周を 2 回流した（lapE・lapF）。
- 立っている間は胴がまったく揺れなかった（roll 0.00〜0.02）。前進の巡航も roll 0.1 以下だった。
- しかし、入り直すたびに 2.5 秒（立つの 1.8 秒 + 0.5 秒）止まり、2 回とも転んだ：1 回は前進の終わりのゆっくりの所、もう 1 回は「歩く」に入り直した直後（回る指令を入れる前）に倒れた。
  controller は「歩く」に入るときの足の位置を、その時の足の置き場所から取る（`FeetEndCalc::init`）。歩いた後の足はそろっていないので、入り直すと足の置き場所がずれる。立ち上がりの後（足がそろっている）と違う。
- 悪くなる場合があり、遅くもなるので入れなかった。橋渡しに入れたのは、大きさの `StillGate` と、待たせた速度を 0 から上げることだけ。

## 負荷が重いとき（lap0・lapC）

- lap0（13:14）・lapC（13:34）の間は、ほかの担当のテストで RTF 0.6〜0.9、`/clock`・odometry が 0.3〜1.2 秒止まることがあった（lap0 の間に 60 秒記録して、`/clock` の 0.3 秒より長い止まりが 4 回（最長 0.81 秒）、odometry は最長 1.19 秒。その 60 秒の RTF 0.6）。
- そのたびに `/sportmodestate` が 0.5 秒途切れて `センサー（LiDAR か /sportmodestate）が途切れています` で一時停止し、再開した（設計どおり）。
- lapC では傾いた LiDAR が地面を「前方の物」と見て `前方に物が出た` が何度か出て、計画が変わり、箱の縁まで 0.495 m まで近づいた（§2 の 0.6 m を割った。ぶつかってはいない）。
  静かなときの RN3 でも、回る途中に胴が 0.88 傾いた所で 1 回出た（上の「手順 5・6」）。

## 第 2 部：これから（Part 2: to do）

- 本体のフォルダ（`~/work/physicalAI/go2`）で、wave 3 を merge した後に：
  - 手順 7：操作画面（ポート 8811）で「箱の周りを一周して」と話し、上から見た図（箱・椅子・0.5 m の格子・灰色のふさがったマス・青い道）、
    Go2 の札の「見えている物：箱（左前 2.9 m）、椅子（右前 2.9 m）」、環境のタブの代わりの文を確かめ、`docs/reference/gazebo_objects_map.png` を撮る。
    転ぶことがあるので、転んだら Gazebo を起動し直してやり直す。
  - 本体のフォルダで RTF を測り直して、この文書の手順 4 に足す（修正 1 回目の静かなときの値は、このフォルダで 0.99〜1.0）。
  - 手順 5・6 を本体のフォルダでもう一度流すなら、この文書の「手順 5・6」の表に足す。
- 転ぶのを無くすには、Gazebo の controller を直す必要がある（下の「気づいたこと」）。その上で手順 5・6 を流し直す。

## 気づいたこと（このタスクでは直していない）

- **Gazebo の controller が、止まっては動く歩き方に弱い。** 直すなら `docker/patches/` にパッチを足してイメージを作り直す（このタスクの決まりでは作り直さない）。候補：
  - StopMove の後（指令 0）は足踏みをやめて立ち、次の指令で決まった形から足踏みを始める（`checkStepOrNot` がずれの大きさだけで足踏みを始め・止めるのをやめる）。
    最後の設定の転び（落ち着いた所からの歩き出し）は、ここがいちばん疑わしい。
  - 「歩く」に入るときの足の置き場所を、決まった立ち方（`feet_pos_normal_stand_`）から取る（今は入った時の足の置き場所。歩いた後に入り直すとずれる）。
  - 回る目標の向き `yaw_cmd_` を、本当の向きから離れすぎないように切る（`gazebo_turn_fix.md` の H2）。
- **一周の計画は回る回数が多い。** BFS の同じ長さの道の選び方で、出だしが「左 → 前 → 右 → 前 → 前 → 前 → 右 → 前 → 左 …」の階段になり、一周 39〜41 動作のうち 10〜12 回が 90 度の旋回（Task B・A の planner。持ち主でないので触っていない）。
  止まって動き出す回数（動作の数）が減れば、Gazebo では転びにくくなるはず。
- **傾いた LiDAR の地面。** 胴が大きく傾いた（0.88）一瞬に `前方に物が出た` が出ることがある（lapC・RN3）。地面の点を除く `drop_ground_and_self_points`（R23）は、点群の時刻と胴の向きの時刻がずれると地面の点を残すことがある（大きさの膨らみと同じずれ）。
  今回は計画が変わっただけで合否には響かなかった（RN3 は合格）。直すなら、点群を撮った時刻の胴の向きを使う。
- **終わった後も「歩く」のまま。** `run_formula` が終わると Go2 は速度 0 で足踏みを続け、near0 では終わった後 1.5 秒で roll が 0.86 まで育ってから収まった。`objects_check.py` は終わって 1 秒（`TAIL_S`）しか見ないので、その後に転んでも数えない。
- `objects_check.py objects` の終了コードは大きさの範囲（0.3〜0.8）を見ない。手順 4 の大きさは目で確かめた。
- `objects_check.py run` の `min_edge_*` は立った後の記録すべてで測るので、転んだ後に転がった位置も入る（RL1 の 0.205 m）。

## 再現のしかた

```bash
cd <このリポジトリ>
docker compose --profile gazebo run -d --rm --no-deps -T -e ROS_DOMAIN_ID=77 --name go2-objects-gz gazebo \
    ros2 launch /work/gazebo/launch/go2_gazebo.launch.py gui:=false      # controller が起動するまで待つ
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py objects
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py rtf --seconds 20
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 --name go2-objects-rec app \
    python3 gazebo/tools/motion_log.py record --seconds 900 > lap.jsonl &   # 動作ごとの傾きも見るとき
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \
    python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"; echo "exit=$?"
docker stop go2-objects-rec
docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/motion_log.py table /dev/stdin < lap.jsonl
docker logs go2-objects-gz 2>&1 | grep "controller へ" | head -20          # 橋渡しが controller に送った値
docker stop go2-objects-gz                                                   # 次の回の前（転んだ後はかならず）
```
