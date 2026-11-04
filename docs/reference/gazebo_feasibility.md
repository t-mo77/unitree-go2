# Gazebo の Go2 を動かせるかの試し（Task 15 の記録）

> Classic は arm64 で不可 → Jazzy 版（ROS 2 Jazzy + Gazebo Harmonic、Task 15b）は `gazebo_feasibility_jazzy.md`。

- 試した日：2026-11-04
- 場所：この Mac（Apple Silicon、arm64）の Docker Desktop。CPU 8、メモリ 8 GB、GPU なし。
- 土台のイメージ：`go2-voice-ltl:dev`（`ros:humble-ros-base-jammy`、Ubuntu 22.04、arm64）
- 使うもの：[legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control) の `humble` ブランチ
  - 固定した commit：`a808acbd0bdb9d298d07de3c954010e3d57a15f6`（2025-05-14、"add readme for compile humble gazebo"。2026-11-04 時点の `humble` の先頭）

## 判定：BLOCKED（まだ GO / NO-GO を決められない）

この Mac（arm64）の apt には **Gazebo Classic 11 が無い**。そのため `docker/Dockerfile.gazebo` は、最初の apt の段で止まる。

- Go2 を Gazebo に出せていない。
- 合格の目安の 4 つ（立った高さ、5 秒の前進の距離、前進中の傾き、real-time factor）は、どれも測れていない。
- 画面のスクリーンショット（`gazebo_feasibility.png`）も撮れていない。

| 合格の目安 | 結果 |
|---|---|
| 立った後の胴体の高さが、立つ前より 0.1 m 以上高い | 測れていない |
| 5 秒の前進で 0.3 m 以上進む | 測れていない |
| 前進の間、roll と pitch の絶対値が 0.5 rad 未満 | 測れていない |
| real-time factor が 0.3 以上 | 測れていない |

### 無いパッケージ（arm64、Ubuntu 22.04）

| パッケージ | 中身 | arm64 での様子 |
|---|---|---|
| `ros-humble-gazebo-ros-pkgs` | Gazebo Classic と ROS 2 をつなぐもの | 一覧に無い（`apt-cache policy` が何も出さない） |
| `ros-humble-gazebo-ros2-control` | Gazebo Classic 用の ros2_control | 一覧に無い（同上） |
| `gazebo` | Gazebo Classic 11 の本体 | `Candidate: (none)` |
| `libgazebo-dev` | Gazebo Classic 11 の開発用ファイル | `Candidate: (none)` |

わけ：Ubuntu 22.04 の arm64 には、Gazebo Classic 11 の本体（`gazebo`、`libgazebo-dev`）が無い。そのため ROS の arm64 版にも、それに頼る `gazebo_ros_pkgs` と `gazebo_ros2_control` が作られていない。`ros-humble-gazebo-dev` だけは一覧にあるが、`gazebo` と `libgazebo-dev` に頼るので入れられない。

```text
$ apt-cache policy ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
（何も出ない）

$ apt-cache policy gazebo libgazebo-dev libgazebo11
gazebo:
  Installed: (none)
  Candidate: (none)
  Version table:
libgazebo-dev:
  Installed: (none)
  Candidate: (none)
  Version table:
（libgazebo11 は一覧に無いので何も出ない）

$ apt-cache show ros-humble-gazebo-dev | grep ^Depends
Depends: gazebo, libgazebo-dev, ros-humble-ros-workspace

$ apt-get install --dry-run ros-humble-gazebo-dev
The following packages have unmet dependencies:
 ros-humble-gazebo-dev : Depends: gazebo but it is not installable
                         Depends: libgazebo-dev but it is not installable
E: Unable to correct problems, you have held broken packages.
```

`docker compose --profile gazebo build gazebo` の止まったところ：

```text
#7 11.52 E: Unable to locate package ros-humble-gazebo-ros-pkgs
#7 11.52 E: Unable to locate package ros-humble-gazebo-ros2-control
failed to solve: process "/bin/bash -o pipefail -c apt-get update && apt-get install -y --no-install-recommends ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ..." did not complete successfully: exit code: 100
```

### arm64 で入るもの

| パッケージ | arm64 の Candidate |
|---|---|
| `ros-humble-ros2-controllers` | 2.54.1-1jammy.20261007.070836 |
| `ros-humble-xacro` | 2.1.1-1jammy.20260307.132851 |
| `ros-humble-rviz2` | 11.2.29-1jammy.20260916.165120 |
| `ros-humble-ros2controlcli` | 2.54.3-1jammy.20261005.195709 |
| `xvfb` | 2:21.1.4-2ubuntu1.7~22.04.16 |
| `fluxbox` | 1.3.5-2.1 |
| `feh` | 3.6.3-1 |
| `x11vnc` | 0.9.16-8 |
| `novnc` | 1:1.0.0-5 |
| `websockify` | 0.10.0+dfsg1-2build1 |
| `mesa-utils` | 8.4.0-1ubuntu1 |

### Gazebo Classic 以外の部分は arm64 で動く（確かめた）

`docker/Dockerfile.gazebo` から、無い 2 つ（`ros-humble-gazebo-ros-pkgs`、`ros-humble-gazebo-ros2-control`）の行だけを抜いたものを、作業用の別のタグでビルドした。このビルドはコミットしていない。確かめた後にイメージも消した。

- apt の段：上の表のものが全部入った。
- `git checkout a808acbd…` で固定できた。
- `rosdep install --from-paths src --ignore-src -r -y`：
  - 解決できないものが 2 つある。`ocs2_quadruped_controller` の `ocs2_sphere_approximation` と、`gz_quadruped_hardware` の `gz_plugin_vendor`。どちらも今回ビルドしないパッケージで、`-r` なので先へ進む。
  - 入ったもの：`ros-humble-joint-state-publisher`、`ros-humble-joint-state-publisher-gui`、`ros-humble-grid-map-sdf`、`ros-humble-ros2-control`
- `colcon build`：6 個（`control_input_msgs`、`controller_common`、`go2_description`、`keyboard_input`、`leg_pd_controller`、`unitree_guide_controller`）が約 40 秒で通った。
- `gazebo_entry.sh`：Xvfb・fluxbox・x11vnc・websockify が立ち上がった。
  - Playwright で `http://localhost:6080/vnc.html?autoconnect=1` を開くと、仮想の画面が見えた（試しに `glxgears` を出した）。
- OpenGL はソフトウェアで描いている：`glxinfo -B` の結果は `OpenGL renderer string: llvmpipe (LLVM 15.0.7, 128 bits)`。
- fluxbox は、壁紙を貼る道具が無いと、画面の真ん中に「can't find an app to set the wallpaper」の窓を出す。Gazebo の画面にかぶるので、`feh` を apt に足した。足した後は出ない。

### 参考：x86_64 なら ROS 側のパッケージはある

packages.ros.org の x86_64（amd64）の一覧には、2 つともある（この Mac で amd64 の一覧を読んで確かめた。入れてはいない）。

| パッケージ | amd64 の Candidate |
|---|---|
| `ros-humble-gazebo-ros-pkgs` | 3.9.0-1jammy.20260915.233200 |
| `ros-humble-gazebo-ros2-control` | 0.4.10-1jammy.20261005.224036 |

Gazebo Classic 11 の本体（`gazebo`）は、x86_64 の Ubuntu 22.04 なら universe から入るはず（ここでは確かめていない。この Mac の apt は arm64 用の ports.ubuntu.com を見ているので、x86_64 の Ubuntu の一覧は読めない）。設計書 §9.1 の「Ubuntu 22.04 では入る」は、x86_64 のときの話になる。

### 代わりの道（ユーザーに相談するための材料。どれも試していない）

| 道 | やること | 分かっていること |
|---|---|---|
| A. x86_64 のまねで動かす | Docker Desktop の x86_64 のエミュレーション（Rosetta か QEMU）で、gazebo のイメージを `linux/amd64` で作る | パッケージは揃う。土台の `go2-voice-ltl:dev` も amd64 で作り直す。物理の計算がまねの分だけ遅くなる。real-time factor が 0.3 に届くかは、測らないと分からない |
| B. Gazebo Classic 11 をソースから作る | arm64 の上で `gazebo11`、`gazebo_ros_pkgs`、`gazebo_ros2_control` をビルドする | Gazebo Classic は 2025 年 1 月で保守が終わっている。ビルドに長い時間と多くのメモリが要る（Docker のメモリは 8 GB） |
| C. Gazebo Fortress に替える | arm64 にある `ros-humble-ros-gz`（0.244.26）と `ros-humble-gz-ros2-control`（0.7.21）を使う | 設計書 §9.1 の「Gazebo Classic 11」を変えることになる。upstream が Humble 向けに書いている新しい Gazebo の手順は Harmonic 用（`gz_quadruped_hardware`、`gz_sim_vendor` に頼る）で、Fortress で動くかは書かれていない |
| D. Gazebo Harmonic に替える | OSRF の apt（packages.osrfoundation.org）の jammy・arm64 にある `gz-harmonic`（1.0.0-1~jammy）を使う | 同じ apt の arm64 には `ros-humble-ros-gzharmonic` が無いので、ROS 2 とつなぐ部分（ros_gz）をソースから作る。設計書の変更になる |

## unitree_guide controller の状態と切り替え（Task 16 が使う表）

**ここから下は、固定した commit のソースコードを読んで書いたもの。Gazebo の上ではまだ確かめていない。**
状態の切り替えのしくみは controller の中のものなので、シミュレーターの種類（Gazebo Classic・新しい Gazebo・MuJoCo）が変わっても同じ。

読んだファイル（`~/gz_ws/src/quadruped_ros2_control/` の下）：

- `libraries/controller_common/include/controller_common/common/enumClass.h`（状態の番号）
- `libraries/controller_common/src/FSM/StatePassive.cpp`、`StateFixedDown.cpp`、`BaseFixedStand.cpp`
- `controllers/unitree_guide_controller/src/FSM/StateFixedStand.cpp`、`StateTrotting.cpp`、`StateFreeStand.cpp`、`StateSwingTest.cpp`、`StateBalanceTest.cpp`
- `controllers/unitree_guide_controller/src/UnitreeGuideController.cpp`（`/control_input` を受けるところ、状態の切り替えの流れ）
- `commands/keyboard_input/src/KeyboardInput.cpp`
- `descriptions/unitree/go2_description/config/gazebo.yaml`（`update_rate`）

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
  - ただし、切り替わる前に届いたものは、新しい状態に入ったときに 0 に戻されて消える。続けて 2 通送るときは、2 通目を（実際の時間で）0.1 秒以上あとにする（controller の 1 周期はシミュレーションの中で 5 ms）。
- 1.8 秒のわけ：伏せと立つは `percent_` が 1.5 になるまで次へ行かない。`percent_` は 1 周期に `1 / (update_rate × 1.2)` ずつ増える。`update_rate` は `go2_description/config/gazebo.yaml` の `unitree_guide_controller` の 200 Hz。だから 1.5 × 1.2 = 1.8 秒。
  - これはシミュレーションの中の時間。実際の時間では「1.8 秒 ÷ real-time factor」かかる。
  - 関節は `tanh(percent_)` の割合で目標の角度へ近づく。1.8 秒の時点で 90 %（tanh 1.5 ≈ 0.905）。
- **`command` 2 は「立つ」と「伏せ」の行き来**で、今の状態によって行き先が変わる。橋渡しは今の状態を自分で覚えておく。2 を繰り返し送り続けると、立つと伏せを繰り返してしまう。
- 速度だけを送るときは `command` 0 にする。

### したいことごとの送り方（Task 16 で使う）

| したいこと | 今の状態 | 送る `command` | かかる時間（シミュレーションの中） |
|---|---|---|---|
| 立つ | 脱力（起動直後） | 2、0.1 秒以上あけてもう一度 2（伏せを通って立つ） | 3.6 秒以上（伏せ 1.8 秒 + 立つ 1.8 秒） |
| 立つ | 伏せ | 2 | 1.8 秒以上 |
| 歩く（速度を受け付ける） | 立つ | 4 | 立つに入って 1.8 秒たった後にすぐ |
| 伏せ | 立つ | 2 | 1.8 秒以上 |
| 伏せ | 歩く | 2、0.1 秒以上あけてもう一度 2（立つを通って伏せ） | 1.8 秒以上（立つ）+ 1.8 秒以上（伏せ） |
| 脱力 | balance test 以外 | 1 | 伏せ・立つからは、入って 1.8 秒たった後 |
| 速度を変える | 歩く | 0（と `lx` / `ly` / `rx`） | すぐ |

定数にすると：脱力 `1`、立つ `2`、伏せ `2`（立つと同じ値）、歩く `4`、何もしない `0`。

### 速度の入れ方（歩くのときだけ効く）

| フィールド | 意味 | 換算 | 正の向き | 範囲 |
|---|---|---|---|---|
| `ly` | 前後の速さ v_x | v_x = 0.4 × `ly` [m/s] | 前 | `ly` −1〜1 → −0.4〜+0.4 m/s |
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

### keyboard_input の動き（参考）

- `/control_input` に出す（ノードの名前 `keyboard_input_node`）。
- 数字キー 1〜9 で `command` 1〜9、0 で `command` 10。数字キーを押すと、`lx`・`ly`・`rx`・`ry` を 0 にして送り、約 10 ms 後に `command` 0 で送り直す。
- `w` / `s` で `ly` を ±0.05、`d` / `a` で `lx` を ±0.05、`l` / `j` で `rx` を ±0.05、`i` / `k` で `ry` を ±0.05（±1 で止まる）。スペースで全部 0。
- README の「Fixed Stand: Keyboard 2」は、脱力から 2 を 2 回押すこと（脱力 → 伏せ → 立つ）。

### トピック

| 用途 | トピック・道具 | 型 | 出すもの |
|---|---|---|---|
| controller への入力 | `/control_input` | `control_input_msgs/msg/Inputs` | 受けるのは unitree_guide controller（キューの深さ 10） |
| 今の状態 | 無い。ログ `Switched from … to …` だけ（`/rosout`） | `rcl_interfaces/msg/Log` | unitree_guide controller |
| 胴体の向き・角速度 | `/imu_sensor_broadcaster/imu` | `sensor_msgs/msg/Imu` | imu_sensor_broadcaster |
| 関節の角度 | `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster |
| 胴体の本当の位置・向き（Gazebo） | `gz model -m robot -p`（モデルの名前は `robot`。launch の `spawn_entity -entity robot`） | — | Gazebo Classic |
| 胴体の本当の位置・向き（ROS 2） | upstream の launch のままでは無い。`/gazebo/get_entity_state` は、world に `libgazebo_ros_state.so` を入れたときだけある。Task 16 の計画どおり `libgazebo_ros_p3d.so` を足す | `nav_msgs/msg/Odometry` | （Task 16 で足す） |

## Gazebo Classic を使えるようにしたときに直すこと（コードから読んだ。未確認）

1. **go2 の設定に leg_pd_controller が無い。** 固定した commit の `descriptions/unitree/go2_description/config/gazebo.yaml` には、`leg_pd_controller` の定義と、`unitree_guide_controller` の `command_prefix: "leg_pd_controller"` が無い。
   - Gazebo Classic（`gazebo_ros2_control`）が出すのは関節のトルク（effort）だけ。unitree_guide controller は位置・速度・kp・kd も要るので、leg_pd_controller を間にはさむ。
   - このままの `pkg_description:=go2_description` では、leg_pd_controller の起動が失敗し、unitree_guide controller が active にならないはず。
   - `go1_description/config/gazebo.yaml` には 2 つともある（関節の並びも同じ）。go2 の設定にも同じものを足す（例：Task 16 の自分の xacro・launch で、自分の設定を渡す）。
   - `go2_description/xacro/gazebo_classic.xacro` は、`$(find go2_description)/config/gazebo.yaml` を決め打ちで読む。
2. **upstream の `gazebo_classic.launch.py` は rviz2 も起動する。** そのため `ros-humble-rviz2` を入れた。ソフトウェアで描くので CPU を使う。Task 16 の自分の launch では外してよい。
3. **leg_pd_controller をビルドする。** upstream の README 1.2（Gazebo Classic）の手順。`colcon build` の対象に足した。

## real-time factor

測れていない（BLOCKED）。
