# Gazebo Harmonic（ROS 2 Jazzy）の Go2 を動かせるかの試し（Task 15b の記録）

- 試した日：2026-11-04
- 場所：この Mac（Apple Silicon、arm64）の Docker Desktop。CPU 8、メモリ 8 GB、GPU なし。
- 土台のイメージ：`ros:jazzy-ros-base-noble`（Ubuntu 24.04、arm64）。app コンテナ（`go2-voice-ltl:dev`、Humble）とは別のイメージ（`go2-voice-ltl-gazebo:dev`）。
- Gazebo：Gazebo Sim 8.15.0（Harmonic。`ros-jazzy-gz-sim-vendor` 0.0.13）、`ros_gz` 1.0.24、`gz_ros2_control` 1.2.20、`controller_manager` 4.48.1
- 描画：ソフトウェア（`OpenGL renderer string: llvmpipe (LLVM 20.1.2, 128 bits)`、OpenGL 4.5、Mesa 25.2.8）
- 使うもの：[legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control) の `main`（既定のブランチ）
  - 固定した commit：`5434c5810d1a7fe223bcfd04550e9d3bfdd4b458`（2025-06-24、"x30 repaint"。2026-11-04 時点の `main` の先頭）
- unitree_ros2：`668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`（Task 1 の Humble のイメージと同じ commit。Jazzy の上で `unitree_api`・`unitree_go` をビルド）
- Gazebo Classic（Task 15）は arm64 に無いので不可。その記録は `gazebo_feasibility.md`。

## 判定：GO（条件つき）

**条件：Task 16 の launch では、Gazebo のセンサーを LiDAR だけにし、rviz2 を起動しない。**

upstream の `gz_quadruped_playground` の launch をそのまま使うと、カメラ 2 台（D435 の RGBD と前のカメラ）と rviz2 の描画で CPU が足りず、real-time factor が 0.13 になり、歩くと大きく傾く（NO-GO）。
カメラと rviz2 を外して LiDAR だけにすると、合格の目安を 4 つとも満たした。

| 合格の目安 | B：LiDAR だけ・画面あり・rviz2 なし | A：upstream の playground のまま |
|---|---|---|
| 立った後の胴体の高さが、立つ前より 0.1 m 以上高い | **合格**：0.068 → 0.344 m（+0.276 m） | 合格：0.068 → 0.338 m（+0.270 m） |
| 5 秒の前進で 0.3 m 以上進む | **合格**：1.411 m | 不合格：0.252 m |
| 前進の間、roll と pitch の絶対値が 0.5 rad 未満 | **合格**：最大 0.058 / 0.026 rad | 不合格：roll が最大 1.026 rad |
| real-time factor が 0.3 以上 | **合格**：前進の間 0.80、15 秒の平均 0.86 | 不合格：0.13 |

- B の画面：`gazebo_feasibility_jazzy.png`（Go2 が立ち、1 m 先に箱がある。3D の画面の右下の RTF の表示は 81.79 %）。
- 「5 秒」は**実際の時間の 5 秒**で測った（シミュレーションの中の時間は B で 4.00 秒、A で 0.80 秒）。
- 前進の速さは `ly` 0.75（= 0.3 m/s。executor の既定の前進の速さ）。

## 測り方

1. Go2 を出した直後（脱力。床に伏せている）の高さを読む。
2. `/control_input` に `command` 2 を送り、0.5 秒（実際の時間）後にもう一度 2 を送る（脱力 → 伏せ → 立つ）。シミュレーションの中で 5 秒待って高さを読む。
3. `command` 4（速度は 0）を送る（立つ → 歩く）。シミュレーションの中で 2 秒待って位置を読む。
4. `command` 0、`ly` 0.75 を送り、実際の時間で 5 秒待つ。その間の位置・傾きを全部記録する。
5. `ly` 0 を送って止める。

- 位置と傾き：`gz topic -e -t /world/empty/dynamic_pose/info` のモデル `robot` の姿勢（約 25 Hz）。
- real-time factor：`/clock`（ROS）の進みを実際の時間で割った値と、`gz topic -e -t /world/empty/stats` の `sim_time` / `real_time` の差の比。
- 測るための小さな python は一時的なもので、コミットしていない。

### 細かい数値

| 構成 | 立つ前 → 立った後の高さ | 前進（実際 5 秒） | 前進の間の最大 roll / pitch | 前進の後 |
|---|---|---|---|---|
| A：playground のまま | 0.068 → 0.338 m | シミュレーション 0.80 秒、0.252 m（dx 0.106、dy −0.229） | 1.026 / 0.068 rad | 歩くに入った 2 秒後に roll 0.963 rad。止めても揺れ続け、位置が 1 m 以上ずれた |
| B：LiDAR だけ | 0.068 → 0.344 m | シミュレーション 4.00 秒、1.411 m（dx 1.410、dy 0.055） | 0.058 / 0.026 rad | 止めた 2 秒後に roll 0.241 rad（1 回だけ揺れた） |

## real-time factor（実測）

`docker stats --no-stream` の CPU は gazebo のコンテナの値（8 CPU で 800 % が上限）。

| 構成 | 画面（Gazebo GUI） | rviz2 | センサー | real-time factor | gazebo の CPU | 同時に動いていた他のコンテナ |
|---|---|---|---|---|---|---|
| A：playground のまま | あり | あり | IMU、足の力、LiDAR、D435（RGBD 640×480、30 Hz）、前のカメラ（1280×720、15 Hz） | 0.124（25 個の平均）、歩いた試しの間 0.130 | 441〜459 % | 他の担当のテスト `logic-app-run-…`（CPU 105〜106 %） |
| A から rviz2 だけ止めた | あり | なし | 同上 | 約 0.15（Gazebo の画面の表示 15.44 %。1 回読んだだけ） | — | 同上 |
| B：LiDAR だけ | あり | なし | IMU、足の力、LiDAR | 0.861（150 個、15 秒）、歩いた試しの間 0.744 | 348〜472 % | 無し |
| C：外のセンサー無し | あり | なし | IMU、足の力 | 0.893（120 個、12 秒） | 292 % | 無し |
| D：LiDAR だけ・画面なし | なし（`gz sim -s`） | なし | IMU、足の力、LiDAR | 0.957（120 個、12 秒） | 178 % | 無し |

- A では、Gazebo のサーバーと画面（どちらも `ruby` の `gz sim`）がそれぞれ 160〜170 %、rviz2 が 40 % を使っていた。
- B〜D は、upstream の launch を元にした一時的な launch で測った（コミットしていない）。xacro を `EXTERNAL_SENSORS:=true` で展開した後、`type` が `camera` と `rgbd_camera` の `<sensor>` を含む `<gazebo>` を取り除いた。

### 歩き方の周期は実際の時間で決まる（注意）

- unitree_guide controller の `WaveGenerator`（`controllers/unitree_guide_controller/src/gait/WaveGenerator.cpp`）は、足を上げる周期（0.45 秒）を `std::chrono::system_clock`（実際の時間）で数えている。
- 他の計算（伏せ・立つの 1.8 秒、速度の積分）はシミュレーションの中の時間（controller は Gazebo の 1 歩ごとに同期して呼ばれる）。
- そのため、Gazebo が CPU 不足で止まったり進んだりすると（A：`real_time_factor` の瞬間の値が 0.002〜0.998 で揺れた）、足の動きとシミュレーションがずれ、大きく傾く。
- real-time factor が低くても一定なら歩けた。B の構成で、Gazebo の目標の速さを `/world/empty/set_physics`（`real_time_factor`）で下げて試した：

| 目標 | 実測の real-time factor | 高さ | 前進（実際 5 秒） | 最大 roll / pitch |
|---|---|---|---|---|
| 0.3 | 0.285 | +0.275 m | シミュレーション 1.38 秒、0.475 m | 0.022 / 0.031 rad |
| 0.13 | 0.128 | +0.275 m | シミュレーション 0.63 秒、0.283 m | 0.028 / 0.055 rad |

- つまり、効くのは「CPU に余裕があり、Gazebo が一定の速さで進むこと」。A の不合格は CPU 不足が原因。
- シミュレーションの中の速さは、目標の 0.3 m/s に対して 0.34〜0.45 m/s だった（周期が実際の時間なので、real-time factor が低いほど足が速く動く）。
- 実際の 1 秒あたりに進む距離は real-time factor に比例する。Task 16 の橋渡しで「0.5 m 進む」を作るときは、時間ではなく Gazebo の位置（下の odometry）で測るか、シミュレーションの時間で数える。

## ビルドの記録

`docker compose --profile gazebo build gazebo`：8 分 24 秒（apt 197 秒、rosdep 80 秒、colcon 55 秒）。イメージは 7.27 GB。

### apt（arm64、Ubuntu 24.04）：全部ある

| パッケージ | arm64 の Candidate |
|---|---|
| `ros-jazzy-ros-gz` | 1.0.24-1noble.20261006.044318 |
| `ros-jazzy-gz-ros2-control` | 1.2.20-1noble.20261006.043411 |
| `ros-jazzy-ros2-controllers` | 4.42.1-1noble.20260924.170548 |
| `ros-jazzy-xacro` | 2.1.1-1noble.20260519.011649 |
| `ros-jazzy-rmw-cyclonedds-cpp` | 2.2.4-1noble.20260911.054215 |
| `ros-jazzy-rosidl-generator-dds-idl` | 0.11.1-3noble.20260612.050142 |
| `ros-jazzy-rviz2` | 14.1.24-1noble.20260916.104902 |
| `ros-jazzy-ros2controlcli` | 4.48.1-1noble.20261006.041226 |
| `xvfb` | 2:21.1.12-1ubuntu1.9 |
| `fluxbox` | 1.3.7-1build2 |
| `feh` | 3.10.1-1ubuntu0.1 |
| `x11vnc` | 0.9.16-10 |
| `novnc` | 1:1.3.0-2 |
| `websockify` | 0.10.0+dfsg1-5build2 |
| `mesa-utils` | 9.0.0-2 |
| `python3-numpy` | 1:1.26.4+ds-6ubuntu1 |
| `git` | 1:2.43.0-1ubuntu7.4 |

- `ros-jazzy-rviz2`：upstream の launch が起動するので入れた（brief の一覧には無い）。
- `ros-jazzy-ros2controlcli`：`ros2 control list_controllers` のため（brief の一覧には無い）。

### rosdep（`rosdep install --from-paths src --ignore-src -r -y`）

- 解決できないもの：`magicdog_description` の `catkin`、`ocs2_quadruped_controller` の `ocs2_sphere_approximation`。どちらもビルドしないパッケージ。`-r` なので先へ進む。
- 入ったもの：`ros-jazzy-joint-state-publisher`、`ros-jazzy-joint-state-publisher-gui`、`ros-jazzy-grid-map-sdf`、`ros-jazzy-ros2-control`

### colcon（`--packages-up-to unitree_guide_controller go2_description keyboard_input gz_quadruped_playground`）

- 7 個が通った：`control_input_msgs`、`controller_common`、`go2_description`、`keyboard_input`、`gz_quadruped_hardware`、`gz_quadruped_playground`、`unitree_guide_controller`
- `gz_quadruped_hardware` は警告だけ（`tl_expected` が古い、`import_component` が古い）。
- パッケージの名前は brief のとおり（README と同じ）。
- Gazebo Classic で要った `leg_pd_controller` は要らない。`gz_quadruped_hardware` が位置・速度・トルク・kp・kd を自分で受ける。

## 起動

- `docker compose --profile gazebo up gazebo`（`up` だけだと、profile の無い `app` も起動する）
- 中で動く launch：`ros2 launch gz_quadruped_playground gazebo.launch.py pkg_description:=go2_description`（LiDAR の点群が出る方。`unitree_guide_controller` の `gazebo.launch.py` は外のセンサーを付けない）
  - world：`gz_quadruped_playground/worlds/default.sdf`。**world の名前は `empty`**（ファイルの名前と違う）。物理の 1 歩は 1 ms。
  - Go2 のモデルの名前：`robot`（高さ 0.5 m から落として出す）
  - 起動した直後は脱力（床に伏せて、胴体の高さ 0.068 m）。
- controller（起動から 1 分以内に全部 active。3 つの spawner が順に動く）：

```text
$ ros2 control list_controllers
unitree_guide_controller unitree_guide_controller/UnitreeGuideController  active
imu_sensor_broadcaster   imu_sensor_broadcaster/IMUSensorBroadcaster      active
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster    active
```

- `controller_manager` は 1000 Hz、unitree_guide controller は 200 Hz（`go2_description/config/gazebo.yaml`。ログ `Controller Manager Update Rate: 200 Hz`）。
- 主な ROS のトピック（`ros2 topic list -t` から）：`/clock`、`/control_input`、`/imu_sensor_broadcaster/imu`、`/joint_states`、`/scan`（LaserScan）、`/scan/points`（PointCloud2）、`/rgbd_d435/points`、`/rgbd_d435/image`、`/camera/image`、`/robot_description`、`/tf`、`/tf_static`
- 主な Gazebo のトピック（`gz topic -l`）：`/world/empty/stats`、`/world/empty/pose/info`、`/world/empty/dynamic_pose/info`、`/scan`、`/scan/points`、`/imu`、`/FL_foot_force` など
- 主な Gazebo のサービス（`gz service -l`）：`/world/empty/create`、`/world/empty/remove`、`/world/empty/set_pose`、`/world/empty/set_physics`、`/world/empty/control`、`/world/empty/state`

## unitree_guide controller の状態と切り替え（`main` で読み直した。Task 16 が使う表）

**`humble`（Task 15 の表）と `main` で、状態の番号・切り替え・速度の入れ方は同じ。**

- 固定した 2 つの commit（`a808acb…` と `5434c58…`）の差を、`libraries/controller_common`、`controllers/unitree_guide_controller`、`commands/keyboard_input`、`commands/control_input_msgs`、`descriptions/unitree/go2_description` で見た。
- 違いは ros2_control の API の書き方だけ（`set_value(…)` の戻り値を `std::ignore` で受ける、`get_value()` → `get_optional().value()`）。`checkChange()`、`getUserCmd()`、`enter()`、定数（立ち・伏せの角度、kp 80、kd 3.5、速度の上限）は変わっていない。
- `keyboard_input` と `control_input_msgs` は差が無い。
- `go2_description/config/gazebo.yaml` の差は `rl_quadruped_controller` の設定だけ。unitree_guide controller は 200 Hz のまま。

**Gazebo の上で確かめたこと（B の構成）：** 下の「立つ」（脱力から 2、0.5 秒後にもう一度 2）と「歩く」（4）と前進（`ly` 正 → 前）が表のとおり動いた。伏せ → 立つの切り替えまでの時間は、実際の時間で 2.50 秒（RTF 約 0.72）、7.64 秒（約 0.24）、14.59 秒（約 0.12）で、どれもシミュレーションの中の 1.8 秒にあたる。

### 状態

| 状態 | 番号（`FSMStateName`） | ログの名前 | 中身 |
|---|---|---|---|
| 脱力（passive） | 1 | `passive` | 全部の関節で kp = 0、kd = 1、トルク 0。**起動した直後はこの状態** |
| 伏せ（fixed down） | 2 | `fixed down` | 関節を伏せの角度（hip 0、thigh 1.3、calf −2.4 rad。4 本とも）へ動かす。kp 80、kd 3.5 |
| 立つ（fixed stand） | 3 | `fixed stand` | 関節を立ちの角度（hip 0、thigh 0.67、calf −1.3 rad。4 本とも）へ動かす。kp 80、kd 3.5 |
| free stand | 4 | `free stand` | 足を付けたまま胴を傾ける（使わない） |
| 歩く（trotting） | 5 | `trotting` | 速度の指令で trot で歩く。速度が 0 なら 4 本足で立ったまま |
| swing test / balance test | 6 / 7 | `swing test` / `balance test` | 試験用（使わない） |

- 番号は controller の中だけのもの。外には出ない（`INVALID` が 0）。
- 今の状態を知らせるトピックは**無い**。切り替わったときにログ `Switched from <前の状態> to <次の状態>` が出るだけ（`/rosout` にも出る）。

### 切り替え（`control_input_msgs/msg/Inputs` の `command`）

| 今の状態 | `command` | 次の状態 | いつ切り替わるか |
|---|---|---|---|
| 脱力 | 2 | 伏せ | すぐ |
| 伏せ | 2 | 立つ | 伏せに入ってから 1.8 秒たった後 |
| 伏せ | 1 | 脱力 | 同上 |
| 立つ | 2 | 伏せ | 立つに入ってから 1.8 秒たった後 |
| 立つ | 4 | 歩く | 同上 |
| 立つ | 1 | 脱力 | 同上 |
| 立つ | 3 / 5 / 6 | free stand / swing test / balance test | 同上（使わない） |
| 歩く | 2 | **立つ**（伏せではない） | すぐ |
| 歩く | 1 | 脱力 | すぐ |
| free stand・swing test | 2 / 1 | 立つ / 脱力 | すぐ |
| balance test | 2 / 1 | 立つ / **伏せ** | すぐ |
| 上に無い組み合わせ（`command` 0 を含む） | — | 変わらない | — |

決まり：

- **`command` の値は、次のメッセージが来るまで controller の中に残る。** 新しい状態に入ったときに 0 に戻される。だから `--once` の 1 通で、切り替わるのは 1 回だけ。
- 伏せ・立つに入って 1.8 秒たつ前に来た `command` は、捨てられずに残り、1.8 秒たった時点で効く（後から別のメッセージが来たら、それに上書きされる）。
  - ただし、切り替わる前に届いたものは、新しい状態に入ったときに 0 に戻されて消える。続けて 2 通送るときは、2 通目を少しあとにする（controller の 1 周期はシミュレーションの中で 5 ms。切り替えには 2 周期かかる）。今回は実際の時間で 0.5 秒あけた。
- 1.8 秒のわけ：伏せと立つは `percent_` が 1.5 になるまで次へ行かない。`percent_` は 1 周期に `1 / (update_rate × 1.2)` ずつ増える。`update_rate` は 200 Hz。だから 1.5 × 1.2 = 1.8 秒。
  - これはシミュレーションの中の時間。実際の時間では「1.8 秒 ÷ real-time factor」かかる（上の確認のとおり）。
  - 関節は `tanh(percent_)` の割合で目標の角度へ近づく。1.8 秒の時点で 90 %（tanh 1.5 ≈ 0.905）。
- **`command` 2 は「立つ」と「伏せ」の行き来**で、今の状態によって行き先が変わる。橋渡しは今の状態を自分で覚えておく。2 を繰り返し送り続けると、立つと伏せを繰り返してしまう。
- 速度だけを送るときは `command` 0 にする。

### したいことごとの送り方（Task 16 で使う）

| したいこと | 今の状態 | 送る `command` | かかる時間（シミュレーションの中） |
|---|---|---|---|
| 立つ | 脱力（起動直後） | 2、少しあけてもう一度 2（伏せを通って立つ） | 3.6 秒以上（伏せ 1.8 秒 + 立つ 1.8 秒） |
| 立つ | 伏せ | 2 | 1.8 秒以上 |
| 歩く（速度を受け付ける） | 立つ | 4 | 立つに入って 1.8 秒たった後にすぐ |
| 伏せ | 立つ | 2 | 1.8 秒以上 |
| 伏せ | 歩く | 2、少しあけてもう一度 2（立つを通って伏せ） | 1.8 秒以上（立つ）+ 1.8 秒以上（伏せ） |
| 脱力 | balance test 以外 | 1 | 伏せ・立つからは、入って 1.8 秒たった後 |
| 速度を変える | 歩く | 0（と `lx` / `ly` / `rx`） | すぐ |

定数にすると：脱力 `1`、立つ `2`、伏せ `2`（立つと同じ値）、歩く `4`、何もしない `0`。

### 速度の入れ方（歩くのときだけ効く）

| フィールド | 意味 | 換算 | 正の向き | 範囲 |
|---|---|---|---|---|
| `ly` | 前後の速さ v_x | v_x = 0.4 × `ly` [m/s] | 前（Gazebo で確かめた） | `ly` −1〜1 → −0.4〜+0.4 m/s |
| `lx` | 左右の速さ v_y | v_y = −0.3 × `lx` [m/s] | **`lx` が正なら右**へ | `lx` −1〜1 → +0.3〜−0.3 m/s |
| `rx` | 旋回の速さ ω | ω = −0.5 × `rx` [rad/s] | **`rx` が正なら右回り**（上から見て時計回り） | `rx` −1〜1 → +0.5〜−0.5 rad/s |
| `ry` | 歩くでは使わない | — | — | — |

- 換算は `StateTrotting::getUserCmd()` の `invNormalize(値, 下限, 上限)`（= (値 + 1) × (上限 − 下限) / 2 + 下限）から。上限は v_x ±0.4 m/s、v_y ±0.3 m/s、ω ±0.5 rad/s（`StateTrotting` のコンストラクタ）。
- 速さは胴体の向きを基準にした値（前 = 胴の向いている方）。
- controller は値を −1〜1 に切らない（keyboard_input の側で切っている）。橋渡しで −1〜1 に切る。
- 値は次のメッセージが来るまで残る。時間切れは無い。止めるには `lx`・`ly`・`rx`・`ry` を 0 にして送る。
- 状態が変わっても `lx`・`ly`・`rx`・`ry` は 0 に戻されない。歩くに入る前に送った値が残っていると、歩くに入ったとたんにその速さで動き出す。`command` 4 を送るときは、速度を 0 にしておく。
- 加速の制限：目標の速さは、毎周期「今の速さ ± 0.2 m/s」の中に抑えられる。旋回の指令は、毎周期「前の値 × 0.9 + 新しい値 × 0.1」でならされる（200 Hz で約 50 ms）。
- 足踏みを始める条件：|v_x| か |v_y| が 0.03 m/s を超える、または |ω| が 0.2 rad/s を超える（または胴の位置・速さのずれが大きい）。**その場の旋回は、|ω| が 0.2 rad/s 以下だと足踏みしない**（`|rx|` で 0.4 以下）。
- Unitree の Move（`actions.py`：`x` 前、`y` 左、`z` 左回りが正）からの換算：
  - `ly = x / 0.4`、`lx = −y / 0.3`、`rx = −z / 0.5`（どれも −1〜1 に切る）
  - executor の既定の速さ（前後 0.3 m/s、左右 0.3 m/s、旋回 0.8 rad/s）のうち、旋回の 0.8 rad/s は controller の上限 0.5 rad/s を超えるので、0.5 rad/s に切られる。
- 上の「歩き方の周期は実際の時間で決まる」も見ること（実際に出る速さは目標より 1〜5 割速かった）。

### keyboard_input の動き（参考。`main` も同じ）

- `/control_input` に出す（ノードの名前 `keyboard_input_node`）。
- 数字キー 1〜9 で `command` 1〜9、0 で `command` 10。数字キーを押すと、`lx`・`ly`・`rx`・`ry` を 0 にして送り、約 10 ms 後に `command` 0 で送り直す。
- `w` / `s` で `ly` を ±0.05、`d` / `a` で `lx` を ±0.05、`l` / `j` で `rx` を ±0.05、`i` / `k` で `ry` を ±0.05（±1 で止まる）。スペースで全部 0。

## 位置と向きを読む方法

| 方法 | 中身 | 確かめた結果 |
|---|---|---|
| Gazebo のトピック `/world/empty/dynamic_pose/info`（`gz.msgs.Pose_V`） | 動くものの姿勢。最初の要素が `name: "robot"`（Go2 の胴体 = `base` の位置と向き。世界の座標） | `gz topic -e -t /world/empty/dynamic_pose/info` で約 25 Hz（B）。今回の測定はこれを使った |
| 上を `ros_gz_bridge` で `tf2_msgs/msg/TFMessage` にする | `/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` | 届くが、**`child_frame_id` と `frame_id` が空**になり、どれが `robot` か分からない。使わない |
| **OdometryPublisher（Task 16 に勧める）** | Go2 の xacro に `gz-sim-odometry-publisher-system` を足す。Gazebo の `/model/robot/odometry`（`gz.msgs.Odometry`）を `ros_gz_bridge` で `nav_msgs/msg/Odometry` にする | 一時的な launch で確かめた。`frame_id: odom`、`child_frame_id: base`、位置は Gazebo の本当の位置と同じ（x 1.354、z 0.348 のとき、上の姿勢と一致）。50 Hz に設定 |
| `/imu_sensor_broadcaster/imu`（`sensor_msgs/msg/Imu`） | 胴体の向き・角速度（IMU のノイズ入り） | トピックがあることを確かめた |
| `/joint_states`（`sensor_msgs/msg/JointState`） | 関節の角度 | トピックがあることを確かめた |
| `gz model --list` | — | 「No world found.」と出て使えなかった |

足した設定（一時的な launch で、ロボットの URDF の最後に足した）：

```xml
<gazebo>
  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base</robot_base_frame>
    <odom_publish_frequency>50</odom_publish_frequency>
    <dimensions>3</dimensions>
  </plugin>
</gazebo>
```

橋渡しの引数：`/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry`

## LiDAR の点群

playground の launch は xacro を `EXTERNAL_SENSORS:=true` で展開する。そのとき `go2_description/xacro/gazebo.xacro` の `L1_lidar` が付く。

| 項目 | 値 |
|---|---|
| Gazebo のセンサー | `L1_lidar`、`type="gpu_lidar"`、10 Hz（シミュレーションの中）、`<gz_frame_id>lidar</gz_frame_id>` |
| 付いている場所 | リンク `lidar`。`trunk`（= `base`）から `xyz="0.28945 0 -0.046825" rpy="0 2.8782 0"`（実機の L1 と同じく頭の下で、上下が逆さ） |
| 走査 | 横 640 本（1.396〜4.887 rad。前を中心に 200 度）、縦 16 本（±0.2618 rad）、0.05〜10 m |
| ROS のトピック | `/scan/points`（`sensor_msgs/msg/PointCloud2`。playground の `ros_gz_bridge`：`/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked`）。ほかに `/scan`（`LaserScan`） |
| frame_id | `lidar` |
| 形 | height 16 × width 640、`point_step` 32、フィールド `x`・`y`・`z`・`intensity`（float32）・`ring`（uint16）、`is_dense: false`（当たらない本は inf） |
| 周期（`ros2 topic hz`、実際の時間） | B：7.5 Hz、D：9.5 Hz、Humble 側で受けて 8.6 Hz（どれも 10 Hz × real-time factor） |

箱が見えるかの確認（B）：Go2 の 1.0 m 前（箱の手前の面が胴体の中心から 0.85 m）に 0.3 × 0.6 × 0.4 m の箱を置き、点群 1 つを上の取り付け（`xyz`・`rpy`）で胴体の座標に直した。

- `perception.py` の前方の箱（x > 0、|y| ≤ 0.3、−0.25 ≤ z ≤ 0.40）の中の点：**1095 点**、いちばん近い点 x = 0.837 m（箱の面 0.85 m と合う）。
- 箱を消した後：**0 点**（床の点は z ≈ −0.35 なので入らない）。

Task 16 で合わせること：

- トピックの名前と frame：`/utlidar/cloud`、`utlidar_lidar` にする。例：自分の xacro で `<gz_frame_id>utlidar_lidar</gz_frame_id>` にし、橋渡しを `--ros-args -r /scan/points:=/utlidar/cloud` で起動する。
- `perception.py` の設定（`PerceptionConfig`）：この取り付けなら `lidar_x = 0.28945`、`lidar_z = −0.046825`、`lidar_pitch = 2.8782`（`_to_body` の y 軸まわりの回転と同じ向きで確かめた。上の 1095 点はこの値で数えた）。今の既定値（0.28、0.10、0.0）のままでは合わない。
- 自分で足す方法：自分の xacro で `go2_description` の `robot.xacro` を使い、`gazebo.xacro` の `L1_lidar` の `<gazebo reference="lidar">` の部分だけを持ってくる（D435 と前のカメラは付けない）。または今回のように、展開した後でカメラの `<gazebo>` を取り除く。

## 箱を出し入れする方法

world の名前は `empty`。3 つとも確かめた（どれも 1 秒以内に終わった）。

| 方法 | 出す | 消す | 結果 |
|---|---|---|---|
| `ros_gz_sim` の道具 | `ros2 run ros_gz_sim create -world empty -name box1 -file box.sdf -x 2.35 -y 0 -z 0.2`（`-string` で SDF の文字列も可） | `ros2 run ros_gz_sim remove --ros-args -p world:=empty -p entity_name:=box1` | `Entity creation successful.` / `Entity removal successful.` |
| `ros_gz_bridge` でつないだ ROS のサービス | `/world/empty/create`（`ros_gz_interfaces/srv/SpawnEntity`。`entity_factory: {name, sdf, pose}`） | `/world/empty/remove`（`ros_gz_interfaces/srv/DeleteEntity`。`entity: {name: box1, type: 2}`。2 = MODEL） | どちらも `success=True` |
| Gazebo のサービス（`gz service`） | `/world/empty/create`（`gz.msgs.EntityFactory`） | `/world/empty/remove`（`gz.msgs.Entity`） | `default.sdf` の冒頭の説明にある書き方 |

- 橋渡しの引数：`/world/empty/create@ros_gz_interfaces/srv/SpawnEntity`、`/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity`
- python のノードから使うなら、ROS のサービス（2 行目）が楽。
- 箱の SDF は `<static>true</static>` と `<collision>` を持たせる。Go2 は箱を通り抜けない。

## Humble（app）↔ Jazzy（gazebo）の通信

2 つのコンテナは同じ compose のネットワーク（`go2_default`）。どちらも CycloneDDS（Humble：`cyclonedds` 0.10.5、`rmw_cyclonedds_cpp` 1.3.5。Jazzy：`cyclonedds` 0.10.5、`rmw_cyclonedds_cpp` 2.2.4）。`unitree_go`・`unitree_api` は同じ commit からビルドしたもの。

| 向き | トピック | 型 | 結果 |
|---|---|---|---|
| Jazzy → Humble | `/sportmodestate` | `unitree_go/msg/SportModeState` | **受け取れた**。`ros2 topic echo --once` で中身（`error_code` 1002、`position`、`imu_state.rpy`、`velocity` など）が送った値と同じ。Humble 側の `ros2 topic hz` で 10.01 Hz。型を書かなくても echo できた |
| Humble → Jazzy | `/api/sport/request` | `unitree_api/msg/Request` | **受け取れた**。`ros2 topic pub --once`（Humble の `pub` は `--once` のとき相手を待つ）で送った `api_id` 1004、`id` 42、`parameter` `{"x": 0.3}`（文字列）がそのまま届いた |
| Jazzy → Humble | `/utlidar/cloud`（試しの 3 点、frame `utlidar_lidar`） | `sensor_msgs/msg/PointCloud2` | **受け取れた** |
| Jazzy → Humble | `/scan/points`（Gazebo の LiDAR、1 通 327,680 バイト） | `sensor_msgs/msg/PointCloud2` | **受け取れた**。Humble 側で 8.6 Hz |

出た警告（データは届く）：

- 両方の側で、相手が見えたときに次のメッセージが標準エラーに出る：

  ```text
  >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
  This error state is being overwritten:
    'string data is not null-terminated, at ./src/serdata.cpp:384'
  with this new error message:
    'invalid data size, at ./src/serdata.cpp:384'
  ```

- わけ：ROS 2 がノードの名前を伝えるのに使う `ros_discovery_info` の型 `rmw_dds_common/msg/Gid` が、Humble（`char[24] data`）と Jazzy（`char[16] data`）で違うため、相手のものを読めない。
- そのため、相手のノードの名前が分からない：
  - Humble 側の `ros2 node list` は失敗する（`Failed to get node names: empty node name returned by the RMW layer`）。
  - `ros2 topic info -v` の相手は `_NODE_NAME_UNKNOWN_` になる。
  - `ros2 topic list -t`（トピックと型）、出す側・受ける側の数、データそのものは問題ない。
- app と橋渡しのプログラムは、相手のノードの名前に頼らないこと。

試しの publisher（python、10 Hz）は一時的なもので、コミットしていない。

## 画面

- `http://localhost:6080/vnc.html?autoconnect=1` で見える（noVNC 1.3.0）。`gazebo_feasibility_jazzy.png` は Playwright で撮った B の画面。
- Gazebo の窓は 1015 × 655 で、画面（1280 × 800）いっぱいではない。左の 3D の部分は約 590 × 630。大きくすると描く量が増えて CPU を使うので、そのままにした。
- upstream の playground の launch では rviz2 の窓が Gazebo の上に重なる（A）。rviz2 を止めると Gazebo が見える。
- 注意：Task 15 の noVNC（1.0.0）を開いたことがあるブラウザでは、古い `ui.js` が残っていて「noVNC encountered an error: Cannot read properties of null (reading 'addEventListener')」になった。ブラウザのキャッシュを消す（強制再読み込み）と直った。

## Task 16 で気をつけること

1. **自分の launch を作る。** LiDAR だけ（D435 と前のカメラを付けない）、rviz2 と `ros_gz_image` を起動しない。橋渡しは `/clock`、LiDAR の点群（→ `/utlidar/cloud`）、odometry、箱の出し入れのサービス。
2. **CPU に余裕を残す。** 歩き方の周期が実際の時間なので、Gazebo が止まったり進んだりすると転びやすい。他の重い処理（例：Whisper の文字起こし）と同時に動かすときは、real-time factor を見る。
3. **前進の距離は位置で測る。** 実際の 1 秒に進む距離は real-time factor で変わり、シミュレーションの中の速さも目標より速く出る。
4. **止めた後の揺れ。** B で、前進を止めた 2 秒後に roll が 0.241 rad になったことが 1 回あった。転んではいない。
5. 起動は `docker compose --profile gazebo up gazebo`。`up` だけだと `app` も起動する（`app` の `python3 -m voice_ltl.server` は、そのモジュールができるまで失敗して止まる）。
6. 位置の真値は Gazebo のもの（odometry）を使い、`/sportmodestate` の `position` に入れる。

## Task 16 の launch での値

Task 16 の `gazebo/launch/go2_gazebo.launch.py`（`compose.yaml` の `gazebo` の command）で、上の 4 つの目安を測り直した（2026-11-04）。

- 構成：Gazebo の画面あり、センサーは LiDAR だけ（`go2_sensors.urdf.xacro` の `gpu_lidar`。水平 360 度 × 垂直 −30〜+30 度、360 × 16 本、10 Hz）、カメラなし、rviz2 なし、OdometryPublisher（50 Hz）、橋渡し（`gazebo/gazebo_bridge.py`）。
- 指令は 15b と違い、**橋渡しの窓口（`/api/sport/request`）から**送った：StandUp → 実際の 3 秒 → BalanceStand → シミュレーションで StandUp から 6 秒たつまで待つ → Move（`x` 0.3、10 Hz）を実際の 5 秒 → StopMove。
- 位置と傾きは `/sportmodestate`（中身は OdometryPublisher の本当の位置）。real-time factor は `/clock` の進み ÷ 実際の時間。
- 測った python は一時的なもので、コミットしていない。

| 合格の目安 | 1 回目 | 2 回目（コミットした橋渡し） |
|---|---|---|
| 立った後の胴体の高さが、立つ前より 0.1 m 以上高い | **合格**：0.068 → 0.332 m（+0.264 m） | **合格**：0.068 → 0.330 m（+0.262 m） |
| 5 秒の前進で 0.3 m 以上進む | **合格**：1.402 m（シミュレーション 3.83 秒） | **合格**：1.398 m（シミュレーション 3.89 秒） |
| 前進の間、roll と pitch の絶対値が 0.5 rad 未満 | **合格**：最大 0.063 / 0.031 rad | **合格**：最大 0.060 / 0.023 rad |
| real-time factor が 0.3 以上 | **合格**：止まっている 10 秒 0.773、前進の間 0.753 | **合格**：起動の直後 10 秒 0.642、前進の間 0.766 |

- gazebo のコンテナの CPU：397〜413 %。1 回目は他のコンテナ無し、2 回目は別の担当の `go2-app-1`・`go2-sim-1`（どちらも 10 % 未満）が動いていた。
- 1 回目は橋渡しの時計がシミュレーションの時間だけだったころ。体の動きの値は同じ。
- noVNC の画面をブラウザで開いている間（Playwright で 1 秒ごとにスクリーンショット）は、Gazebo の画面の表示で 58〜85 %。

### 窓口の確認（Humble ↔ Jazzy）

- gazebo の中：`ros2 topic list` に `/api/sport/request`・`/sportmodestate`・`/utlidar/cloud`。起動直後の `error_code` は 1001。`/utlidar/cloud` は 7.5 Hz。
- app のコンテナ（Humble）から：`ros2 topic info` で `/api/sport/request` の受け手 1、`/sportmodestate`・`/utlidar/cloud`・`/sim/world` の出し手 1、`/sim/obstacles` の受け手 1。`ros2 topic echo --once /sportmodestate --field error_code` で値が読め、`/utlidar/cloud` は 7.96 Hz。
- gazebo の中の `/control_input`：出し手 1（橋渡し）、受け手 1（unitree_guide controller）。
- 相手のノードの名前は使っていない（数だけ）。

### LiDAR と箱

- `/utlidar/cloud`：1 回の走査 5760 本のうち、当たらなかった本と自分の体（中心から前後 0.40 m・左右 0.20 m・高さ −0.35〜0.15 m）の点を除いて約 2030 点。
- `perception.py` の前方の箱（x > 0、|y| ≤ 0.3、−0.25 ≤ z ≤ 0.40）の中の点：
  - 立っているとき：3 点（9.3〜10 m 先の床。止まる 0.5 m・動き出す 0.7 m には関係しない）。
  - 1.2 m 前に箱（`/sim/obstacles` の front）：126 点・133 点、いちばん近い点 1.062 m・1.039 m（箱の面は 1.05 m）。箱を消すと 3 点に戻る。
  - **伏せている（脱力）とき：410 点、いちばん近い点 0.20 m。** 胴体の高さが 0.068 m なので、床が機体の座標で z ≈ −0.07 になり `z_min`（−0.25）より上に入る（伏せて開いた脚の点も入る）。伏せている間は前方「あり」と判定される。立つと消える。
- 箱：`/world/empty/create` で 0.3 × 0.3 m・高さ 0.4 m（にせ Go2 の `BOX_HEIGHT` と同じ）の動かない箱を出す。`/sim/world` は `{"boxes": [[2.567, -0.058, 0.3]], "collided": false}` の形。
  - 箱に向かって歩かせると、中心から箱の面までが 0.33 m になったところで箱に止められ、`collided` が true になった。`clear` で消え、`{"boxes": [], "collided": false}`。

### demo_script.py（Step 6）と、その場の旋回で転ぶこと

`printf "\n" | timeout 120 python3 demo_script.py` は「立つ → 前進 0.5 m → 左に 90° → 挨拶 → 座る → 立ち上がる → 終わりました」まで止まらずに流れた（2 回）。橋渡しのログ（1 回目、実際の時間）：

```text
31.10 受信: StandUp 1004 → command 2（31.12）、command 2（31.37）→ 33.12 error_code 1001 → 1002
34.10 受信: BalanceStand 1002 → command 4（34.12）→ 34.62 error_code 1002 → 1013
35.12 command 4, ly 0.75（前進）→ 38.61 StopMove → command 4, 速度 0
39.62 command 4, rx −1.00（左回り。0.8 rad/s は上限 0.5 rad/s に切られる）→ 43.75 StopMove
44.76 Hello 1016（Gazebo では再現なし）
49.77 Sit 1009（Gazebo では伏せで代用）→ command 2、command 2（50.02）→ 51.77 error_code 1013 → 1007
54.77 RiseSit 1010 → command 2（54.82）→ 56.82 error_code 1007 → 1013 → command 4（57.17）
```

**ただし、その場で 90 度回った後、Go2 は数秒で横に倒れる（2 回とも）。** 「挨拶」の間に倒れ、「座る」「立ち上がる」は倒れたまま送られた（`gazebo_demo_turn_fall.png`）。
橋渡しをはさまずに `/control_input` を直接送っても同じなので、unitree_guide controller の、この Gazebo の上でのその場の旋回の問題。試したこと（どれも 90 度回った後、3〜4 秒以内に roll が 3 rad 前後になって倒れた。前進だけなら倒れない）：

| 試したこと | 回っている間の最大 \|roll\| |
|---|---|
| 窓口から `z` 0.8（rx −1、0.5 rad/s）、画面あり | 0.37〜0.69 rad |
| 窓口から `z` 0.4・0.35（0.4・0.35 rad/s） | 0.74・0.69 rad |
| Gazebo の目標の速さを 0.5 に固定（`set_physics`）、`z` 0.8 | 0.17 rad（止めた後 0.80 rad、0.8 m 流れた） |
| 画面なし（`gz sim -s`、real-time factor 約 0.95）、`z` 0.8 | 0.55 rad |
| `/control_input` を直接：rx −1／1 秒かけて増減／前へ `ly` 0.25 を足す／回った直後に fixed stand（command 2）に入れる | 0.77／1.13／0.32／0.41 rad |
| 足の摩擦を 0.6 → 1.0（一時的に xacro を変更） | 1.22 rad |
| upstream の `robot.xacro` そのまま（LiDAR・odometry を足さない） | 0.31 rad（その後倒れた） |

- IMU（`/imu_sensor_broadcaster/imu`）の yaw と角速度は、Gazebo の本当の値と符号も大きさも合っていた。
- 0.5 rad/s を指令しても、実際に回る速さは約 0.24 rad/s（90 度にシミュレーションで 6.2〜6.7 秒）。controller の目標の向き（`yaw_cmd_`）は指令の速さで積分され、実際の向きとの差は切られない（位置の目標 `pcd_` は ±0.05 m に切られる）。止めた後もこの差を戻そうとして回り続け、倒れる様子だった（demo の 1 回目の画面では、StopMove の 2〜3 秒後もまだ向きが変わり続け、その 2 秒後に倒れていた）。
- Task 16 では直していない（controller の中の話）。Task 17 の「旋回」の確認はこのままでは通らない。

### 橋渡しの時計（Task 16 で決めたこと）

- アプリ（`demo_script.py`・executor）は実際の時間で待つ（例：StandUp の 3 秒後に BalanceStand）。一方、controller の伏せ・立つの 1.8 秒はシミュレーションの時間。
- 窓口の時間（error_code が変わるまでの 2.0 秒・0.5 秒、ジェスチャーの 3.0 秒、Move の 1.0 秒の期限）をシミュレーションの時間で数えると、real-time factor が 0.67 より低いとき、StandUp の 3 秒後の BalanceStand が「動作の途中」で捨てられる（noVNC を開いていると 0.6 前後まで落ちる）。
- そこで、窓口の時間は実際の時間で数え、controller へ command を送る間隔（前の command が効くまで待つ）はシミュレーションの時間で数える。error_code が先に変わり、体は後から追いつく（command は controller の中で待つ）。
- 例：上のログで、StandUp から 1002 までは実際の 2.0 秒、BalanceStand の 4 が効いて歩くに入るのは、2 通目の 2 が効いて（伏せの 1.8 秒の後）、さらに立つの 1.8 秒の後。

### 画面

`http://localhost:6080/vnc.html?autoconnect=1` を Playwright で撮った（launch がカメラを Go2 の右前 (1.3, −1.1, 0.7) に向ける）。

| ファイル | 中身 |
|---|---|
| `gazebo_demo_stand.png` | demo_script の「立つ」の後（歩き出す前） |
| `gazebo_demo_forward.png` | demo_script の「前進 0.5 m」の後 |
| `gazebo_demo_turn_fall.png` | demo_script の「左に 90°」の後、倒れたところ |
| `gazebo_demo_sit.png` | 伏せ（座るの代用）。demo_script では倒れていたので、同じ窓口・同じ待ち時間で「立つ → 前進 0.5 m → 座る（5 秒）→ 立ち上がる」だけを流した別の回 |
