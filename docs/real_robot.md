# 実機の Go2 を有線でつなぐ

コマンドだけなら [README](../README.md#4-実機の-go2-で動かす有線linux-の-pc) にあります。

実機の Go2 と、もう Linux が入っている PC を LAN ケーブルでつなぎ、このアプリで動かす手順です（設計書 §10）。その PC のブラウザとマイクで操作します（`http://localhost:8810` で開くので、`localhost` のままマイクの許可が使えます）。

**この手順は、まだ実機で通していません。** コマンドと設定は、このリポジトリのコードと Unitree の公式の資料（各所にリンク）で確かめたものです。実機で確かめていないことには「（実機で未確認）」と書きます。

> **Mac からは実機につながりません。** Go2 とは DDS（CycloneDDS）で話し、DDS は相手を UDP のマルチキャストで見つけます。Mac の Docker Desktop はコンテナを Linux の VM の中で動かします。`network_mode: host` にしても TCP・UDP の層でつなぐだけで、コンテナは Mac の差し口に直接は届きません（Docker の [Host network driver](https://docs.docker.com/engine/network/drivers/host/#docker-desktop) の Limitations）。そのため Go2 のマルチキャストがコンテナに届かず、Go2 が見つかりません。Unitree も Mac・Windows での開発には対応していません（[Quick Start](https://support.unitree.com/home/en/developer/Quick_start)）。Mac では偽の Go2 と Gazebo を使い、実機は Linux の PC で動かします。

## 1. 用意するもの

| もの | 内容 |
|---|---|
| Go2 | **Go2 EDU**。公式の機種表では、二次開発ができるのは EDU だけで、Ethernet（RJ45）の差し口も EDU にしかありません（AIR・PRO には無い。[About Go2](https://support.unitree.com/home/en/developer/about_Go2)）。 |
| Go2 のソフトの版 | Unitree のアプリで確かめて、書き留めます。V1.1.6 で運動の窓口が改められました（[Motion Services Interface V2.0](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)。V1.1.6 より前は [High-Level Motion Service](https://support.unitree.com/home/en/developer/sports_services)）。V2.0 の文書にも、このアプリが使う動作（Damp・StandUp・StandDown・BalanceStand・StopMove・Move・Sit・RiseSit・Hello・Stretch・Content・Heart・Scrape・Dance1）と、姿勢の `error_code` の表が載っています。`api_id` の番号は V2.0 の文書には無く、公式 `unitree_sdk2_python` の `sport_api.py` と同じ番号を使っています。どの版で動くかは実機で未確認です。 |
| PC | 画面つきの、もう Linux が入っている PC（x86_64 か arm64）。アプリはコンテナの中の Ubuntu 22.04 + ROS 2 Humble で動くので、PC の Linux が 22.04 でなくても、PC に ROS 2 が入っていなくても動きます（PC に ROS 2 があるときの注意は [§2](#2-今ある-linux-の-pc-で確かめること) の 2）。要るのは Docker Engine（[§2](#2-今ある-linux-の-pc-で確かめること)）・ブラウザ・マイク・インターネット（LLM を呼ぶため。Wi-Fi など、有線とは別の経路）と、有線の差し口です。 |
| LAN ケーブル | 1 本。PC に有線の差し口が無ければ USB の有線 LAN アダプター（名前は `enx…` のようになります。公式 Quick Start の例もこれです）。 |
| Unitree のリモコン | 必ず手に持ちます（[§7](#7-安全の注意)）。 |

## 2. 今ある Linux の PC で確かめること

PC にもう入っているものを使います。入れ直したり、消したりはしません。

1. **Docker Engine：**

   ```bash
   docker version
   docker compose version
   docker context show
   ```

   - `docker context show` が `default` なら Docker Engine です。`desktop-linux` なら Docker Desktop で、Linux 版も VM の中でコンテナを動かすので、`network_mode: host` でも PC の差し口が見えません（[Docker の説明](https://docs.docker.com/desktop/setup/install/linux/)）。`docker context use default` で Docker Engine に切り替えます（Docker Engine が入っていなければ、次の手順で入れます）。
   - Docker が無ければ、公式の [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)（ほかの distro は同じ場所の各ページ）で入れます。`sudo` なしで使うには [Linux post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) に従います。
2. **PC の ROS 2（入っていれば）：** コンテナは PC の `~/.bashrc` の `source /opt/ros/...`・`ROS_DOMAIN_ID`・`CYCLONEDDS_URI` を読みません（`compose.robot.yaml` が渡すのは `env.txt` と `SIM=0` だけ）。ただし `network_mode: host` なので、PC で動いている ROS 2 のノード（Unitree の例の `sport_mode_ctrl` など）は、同じ DDS の相手として見えます。`/api/sport/request` に送るプログラムが PC で動いていると、サーバーは起動を断ります。動かす前に止めておきます。
3. **有線の設定がもうあるか：** 前に Unitree の手順で Go2 とつないだことがある PC なら、[§3](#3-pc-の有線の設定) の設定がもうあるかもしれません。[§3](#3-pc-の有線の設定) の手順 4 で確かめて、合っていれば作り直しません。

## 3. PC の有線の設定

Go2 の中の計算機は `192.168.123.161` です。PC の有線の差し口を、同じ `192.168.123.0/24` の別の番号に固定します（[Quick Start](https://support.unitree.com/home/en/developer/Quick_start)・[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)）。

1. Go2 の電源を入れ、LAN ケーブルで Go2 EDU の RJ45 と PC をつなぎます。
2. 差し口の名前を確かめます。ケーブルを挿すと `UP` になる差し口です（例：`enp3s0`）。

   ```bash
   ip -br link
   ```
3. その差し口を固定の番号にします。GUI なら「設定 → ネットワーク → 有線 → ⚙ → IPv4」で、方式を「手動」、アドレス `192.168.123.99`、ネットマスク `255.255.255.0`、ゲートウェイは空にして「適用」します（公式 unitree_ros2 の手順と同じ値）。コマンドなら（NetworkManager の場合）：

   ```bash
   sudo nmcli connection add type ethernet ifname enp3s0 con-name go2 ipv4.method manual ipv4.addresses 192.168.123.99/24 ipv4.never-default yes ipv6.method disabled
   sudo nmcli connection up go2
   ```

   - `.161` は Go2 本体なので使いません。公式の例は `.99`（unitree_ros2）と `.222`（unitree_sdk2 の Quick Start）です。
   - 有線にゲートウェイを付けません（`ipv4.never-default yes`）。インターネットへの道を Wi-Fi に残すためです。有線にゲートウェイがあると、Claude を呼ぶ通信が Go2 のほうへ向かいます。
   - 公式は、設定の後に PC を再起動してから試すよう勧めています。
4. つながったか確かめます。

   ```bash
   ping -c 3 192.168.123.161     # 返事が来る
   ip -br addr show enp3s0       # 192.168.123.99/24 が出る
   ip route get 1.1.1.1          # dev がインターネット側（Wi-Fi など）の差し口で、有線ではない
   ```

## 4. アプリを PC で用意する

既存のROS 2 HumbleとUnitreeメッセージがあるLinux PCでは、Dockerなしでも起動できます。
[READMEのLinux直接実行](../README.md#linux-pcで直接動かすdockerなし)を参照してください。
`bash scripts/run_local.sh check`・`sensors`・`robot` が、以下のDocker用コマンドに対応します。
以下はDockerを使う場合の手順です。

### Linux直接実行でGo2本体へ障害物判定を委譲する場合

`run_local.sh robot` は既定で `GO2_OBSTACLE_AUTHORITY=go2` です。
PCのLiDAR点群による停止判定は使いません。移動は本体の
`/api/obstacles_avoid/request`（Move: 1003、x/y/yaw/mode=0）へ送ります。
起動時にSwitchGetで本体の回避機能がONであることを検証し、
UseRemoteCommandFromApi(True)でAPI操作へ切り替えます。終了時はゼロ速度と
Sport.StopMoveを送り、UseRemoteCommandFromApi(False)でAPI操作を解除します。
実行中はAPI操作モードのため、リモコンの通常の移動入力との優先関係は
従来のSport.Move使用時と同じとは限りません。本体側の非常停止手段を確認してください。
回避サービスが無効・拒否・無応答なら、通常Sport.Moveへはフォールバックしません。
本体が停止して目標に届かない場合、PCは到達を偽って完了にはせず、動作時間上限で中止します。
通信断・処理異常・停止ボタン・目標到達の管理はPC側にも残ります。
これは全方向や段差での衝突回避を保証するものではありません。

以下のLiDAR校正手順はPC診断・従来のPC判定モード用です。
本体委譲モードのWeb画面は「Go2判断」と表示し、PCの「障害物なし」とは区別します。

1. このフォルダを PC にコピーします（`git clone https://github.com/t-mo77/unitree-go2.git` でも構いません）。`env.txt` は git に入っていないので、[README の「準備」](../README.md#1-準備mac)（詳しくは [setup.md](setup.md)）で作り直すか、自分でコピーします。
2. `env.txt` の `GO2_IFACE=` に、[§3](#3-pc-の有線の設定) の差し口の名前（例：`enp3s0`）を書きます。
   - `ROS_DOMAIN_ID` は書きません。Go2 は 0 番です（unitree_sdk2 の例も `ChannelFactory::Instance()->Init(0, …)`）。
   - `CYCLONEDDS_URI` と `RMW_IMPLEMENTATION` も書きません。コンテナが決めます（[§5](#5-dds-の設定コンテナが決めます)）。
   - ページのポートを変えるときは `GO2_UI_PORT=<ポート>` も書きます（`compose.robot.yaml` では `.env` やシェルの変数は使いません）。
3. イメージを作ります。初回は時間がかかります。

   ```bash
   docker compose -f compose.robot.yaml build
   ```

   Dockerfile は arm64 と x86_64 の両方を考えて書いてありますが、このリポジトリの記録にあるのは arm64 の Mac でのビルドだけです（x86_64 は未確認。設計書 §10）。
4. LLM：Claude は `env.txt` のトークンで動きます。Codex を使うなら、PC でもう一度ログインします（ログインは PC の Docker のボリュームに残るので、Mac のものは引き継がれません）。

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app codex login --device-auth
   ```

   Whisper のモデルは、最初に使うときにインターネットから落とします（Docker のボリューム `hf-cache` に残ります）。

## 5. DDS の設定（コンテナが決めます）

`compose.robot.yaml` で起動したコンテナは、どのコマンドの前にも `~/unitree_ros2/setup.sh`（中身は `docker/unitree_setup.sh`）を読み、次のようにします。

| 値 | 中身 |
|---|---|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` |
| `CYCLONEDDS_URI` | `<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="<GO2_IFACE>" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>`（`GO2_IFACE` が空なら作らない） |
| `ROS_DOMAIN_ID` | 決めない（0 番。Go2 と同じ） |

XML は公式 unitree_ros2 の `setup.sh` と同じで、差し口の名前だけを `GO2_IFACE` から入れます。Go2 の CycloneDDS は 0.10.2 です（公式）。コンテナは Humble の `ros-humble-rmw-cyclonedds-cpp` をそのまま使います（公式の手順でも、Humble では CycloneDDS のビルドを飛ばせます）。

シミュレーションから実機へ切り替えるときに変わるのは、使う compose のファイルだけです。コードは同じです。

| | シミュレーション | 実機 |
|---|---|---|
| 起動 | `docker compose --profile fake up`・`SIM_KIND=gazebo docker compose --profile gazebo up -d` | `docker compose -f compose.robot.yaml up` |
| `SIM` | `1` | `0`（ページの右上は「実機」） |
| ネットワーク | Docker の bridge。ページはホストの 127.0.0.1:`GO2_UI_PORT` に公開 | `network_mode: host`。サーバーが 127.0.0.1:`GO2_UI_PORT` で待ち受け |
| 偽の Go2・Gazebo | 起動する | 起動しない |
| 箱のボタン | ある | ない |
| Go2 を待つ秒数 | 5 秒（`SIM_KIND=gazebo` では 90 秒） | 5 秒（変えるときは `env.txt` に `GO2_WAIT_READY_S=`） |
| `GO2_IFACE` | 使わない | 使う |

道具（`robot_check`・`sensor_check`・`ros2`）も、実機では `docker compose -f compose.robot.yaml run --rm --no-deps app …` で動かします。`-f compose.robot.yaml` を忘れると bridge のネットワークで動き、Go2 は見えません。

## 6. 動かす前の確かめ（この順に）

手順 1〜5 は、サーバーを起動する前にします。どれも Go2 には何も送りません。

1. **ping：** `ping -c 3 192.168.123.161` で返事が来ること（[§3](#3-pc-の有線の設定) の手順 4）。
2. **まとめて確かめる（`robot_check`）：**

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.robot_check
   ```

   1 行ずつ［OK］［注意］［だめ］が出ます。［だめ］には直し方が付きます。最後が「全部 OK です。サーバーを起動できます。」なら次へ進みます（終了コードは、［だめ］が無ければ 0、あれば 1）。見ることは次のとおりです。

   | 見ること | ［OK］になる条件 |
   |---|---|
   | 設定 | `compose.robot.yaml` で動いている（`SIM=0`）・`GO2_IFACE` がある・`ROS_DOMAIN_ID` が 0 か空・RMW が CycloneDDS・`CYCLONEDDS_URI` が `GO2_IFACE` の差し口を指す（URI の中身は出しません） |
   | 差し口 | その名前の差し口があり、オンで、リンクがあり（ケーブルと Go2 の電源）、マルチキャストができ、`192.168.123.0/24` の IPv4（`.161` 以外）がついている |
   | `/sportmodestate` | 5 秒（`--seconds` で変えられる）の間に届いた回数と、最後の `error_code` と姿勢 |
   | `/utlidar/cloud` | 5 秒の間に届いた回数 |
   | `/api/sport/request` | 受け手（Go2 の運動の制御）がいて、ほかの送り手がいない |

   設定と差し口に［だめ］があれば、ROS 2 を起動せずに終わります。トピックの確かめを偽の Go2 で 4 秒流したときは、次のようでした（実機の回数は違います）。

   ```
   ［OK］ /sportmodestate：4 秒で 195 回（49 Hz）
   ［OK］ error_code 1004（伏せ）
   ［OK］ /utlidar/cloud：4 秒で 40 回（10 Hz）
   ［OK］ /api/sport/request の受け手（Go2 の運動の制御）：1
   ［OK］ /api/sport/request のほかの送り手：なし
   ```
3. **ROS 2 で見る：**

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic list --no-daemon --spin-time 3
   docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic echo --once /sportmodestate
   docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic echo --once --no-arr /utlidar/cloud
   ```

   - `topic list`：`/sportmodestate`・`/utlidar/cloud`・`/api/sport/request` が出れば、Go2 が見えています（公式の文書には、ほかに `/lf/sportmodestate`・`/lowstate`・`/wirelesscontroller` なども載っています。[ROS2 Services](https://support.unitree.com/home/en/developer/ROS2_service)）。`/parameter_events` と `/rosout` しか出なければ、Go2 が見えていません（[§8](#8-困ったとき)）。
   - `echo --once /sportmodestate`：`error_code`・`imu_state`・`position` などが 1 通出て終わります。
   - `echo --once --no-arr /utlidar/cloud`：点の並びを省いて 1 通出ます。`frame_id: utlidar_lidar` です（公式）。
4. **姿勢の値（`error_code`）を確かめる（必須）：** このプログラムは `/sportmodestate` の `error_code` で Go2 の姿勢を知ります。対応は `nl_control/go2_nl_control/actions.py` の `_ERROR_CODE_TO_STATE` で、公式の表（[Motion Services Interface V2.0](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)）から作ったものです（実機では未確認）。リモコンで Go2 を「立つ → 座る → 立つ → 伏せる」の順にして、そのたびに値を見ます。

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic echo /sportmodestate --field error_code
   ```

   | 姿勢 | 表の値 |
   |---|---|
   | 立つ（歩ける立ち方） | 1013（歩いている間は 1015。100・1016・1017・2010 も立ち） |
   | 座る（おすわり） | 1007 |
   | 伏せる | 1001・1004・2006 |

   1002 は StandUp の直後の「歩けない立ち方」なので、わざと「不明」にしています。
   表と違う値が出たら、そのときの姿勢と値を書き留めて知らせてください。表にない値は「不明」になり、計画を始められなかったり、動作の後に「姿勢が計画と違います」で止まったりします。
5. **LiDAR を確かめる（必須）：** 障害物で止まる・待つのは、LiDAR の点群を胴の座標に直した判定だけが頼りです。その直し方（`PerceptionConfig` の `lidar_x`・`lidar_z`・`lidar_pitch`）は仮の値なので、実機で確かめます。Go2 をリモコンで立たせ、前と後ろを 1 m 以上空けてから、次を実行します。

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.sensor_check
   ```

   アプリと同じ判定で、`前方 なし — ｜ 後方 なし —` のような 1 行が 0.5 秒ごとに出ます。Ctrl+C で終わります。距離は胴の中心（前足と後ろ足の真ん中）から測った値です。高さ 20〜40 cm ほどの箱を Go2 の正面（真後ろ）に置いて、次のとおりになるか見ます。

   | 箱の手前の面の位置（胴の中心から） | 出るはずの表示 |
   |---|---|
   | 前 0.4 m（頭の先から数 cm） | `前方 あり 0.40 m` 前後 |
   | 前 0.8 m | `前方 なし 0.80 m` 前後 |
   | 後ろ 0.4 m（お尻から数 cm） | `後方 あり 0.40 m` 前後 |
   | 後ろ 0.8 m | `後方 なし 0.80 m` 前後 |
   | 何も置かない | `前方 なし — ｜ 後方 なし —` |

   0.5 m より近いと「あり」、0.7 m より遠くなると「なし」に戻ります（その間は前の判定のまま）。0.4 m の次に 0.8 m を試すと、「あり」から「なし」に戻るのも確かめられます。

   - **前が合わないとき**（箱を置いても「なし」、何も無いのに「あり」、距離が大きくずれる）：LiDAR の取り付けの値が合っていません。`--lidar-x`（前後のずれ [m]）・`--lidar-z`（高さのずれ [m]）・`--lidar-pitch`（y 軸まわりの傾き [rad]）で値を変えて試します。

     ```bash
     docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.sensor_check --lidar-x 0.28945 --lidar-z -0.046825 --lidar-pitch 2.8782
     ```

     この値は Unitree の Go2 のモデル（`go2_description`）の LiDAR の取り付け（頭の下に上下逆さ）です（[docs/reference/gazebo_feasibility_jazzy.md](reference/gazebo_feasibility_jazzy.md)）。既定値で合わなければ、まずこれを試してください。何も無いのに「あり」で床が映っているなら、`z_min` を上げます。
     合う値が見つかったら、`nl_control/go2_nl_control/perception.py` の `PerceptionConfig` の既定値（`lidar_x`・`lidar_z`・`lidar_pitch`）に書きます。サーバーもこの既定値を使います。書いたら、もう一度オプション無しで `sensor_check` を流して確かめます。
     既定値を変えると、既定の取り付けを前提にしたテストが落ちます。Go2 のモデルの値で確かめると、次の 3 件でした（実機の PC では Gazebo を使わないので、そのままで構いません）。
     - `tests/test_bridge_core.py::test_self_points_dropped_and_mount_matches_perception`
     - `tests/test_sim_core.py::test_cloud_of_back_box_is_behind_the_body`
     - `nl_control/tests/test_core.py::test_obstacle_hysteresis_and_filters`
   - **後ろが見えないとき**（前は合うのに、後ろ 0.4 m の箱で「あり」にならない）：実機の LiDAR は頭の下にあるので、真後ろは胴体の陰になるかもしれません。後退の安全は後ろが見えることが前提なので、**`voice_ltl/world.py` の次の 3 行を消して、後退をやめます**（「後ろに下がって」は単語帳にない言葉として却下されます）。

     - `ACTION_NAMES` の `"backward",`
     - `ACTION_PROP` の `"backward": "moved_backward",`
     - `PROP_DESCRIPTION_JA` の `"moved_backward": "後ろへ下がった",`

     `obstacle_back`（後ろの障害物センサー）は消しません。
     後退の命題が消えるので、後退を前提にしたテストは落ちます。コードを読んで3行を消し、`-m "not slow" tests` を流した結果、落ちたのは次の 7 件でした（手元で確かめ済み）。
     これらは後退を前提にしているので、後退をやめるなら同じ commit で直すか外します。

     - `tests/test_world.py`：`test_vocabulary_is_safe_and_complete`（命題が 15 個、という確認）、`test_go2_world_counts`（状態・遷移の数）
     - `tests/test_automata.py`：`test_safety_events_never_in_sup[F moved_backward]`
     - `tests/test_session.py`：`test_back_box_holds_backward`
     - `tests/test_adversarial_session.py`：`test_fuzz_reaches_the_interesting_cases`、`test_property_holds_in_every_trial[P7]`、`[P7b]`
   - **rviz2 で点群を見たいとき：** このイメージには rviz2 が入っていないので、コンテナの中では動きません。PC に ROS 2 Humble が入っていれば、`sudo apt install ros-humble-rviz2 ros-humble-rmw-cyclonedds-cpp` で入れて、別の手段として使えます。コンテナと同じ設定（`docker/unitree_setup.sh` と同じ）にしてから起動し、Fixed Frame を `utlidar_lidar` にして `/utlidar/cloud` を PointCloud2 で表示します。

     ```bash
     source /opt/ros/humble/setup.bash
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
     rviz2
     ```

     `enp3s0` は `env.txt` の `GO2_IFACE` と同じ名前にします。

   `sensor_check` は `/api/sport/request` の送り手として見えるので、動かしたままサーバーを起動すると「別のプログラム」として断られます。終わってから手順 6 に進みます。
6. **起動して、ページで Go2 の様子を見る：**

   ```bash
   docker compose -f compose.robot.yaml up
   ```

   ログに `ページ: http://localhost:8810 （実機）` が出たら、PC のブラウザで開きます（ほかの PC からは開けません。サーバーは 127.0.0.1 だけで待ち受けます）。まだ何も話さずに、次を見ます。

   - 右上に「実機」と出る。
   - 「姿勢」「前方」「後方」「位置」「向き」に値が出て、リモコンで Go2 を伏せ・立ちにすると「姿勢」が変わる。箱を前に置くと「前方」が「あり」になる。
   - 起動のとき、Go2 が 5 秒で見つからなければ「Go2 が見つかりません …」、ほかの送り手がいれば「… 別のプログラムがあります」で止まります（[§8](#8-困ったとき)）。`/utlidar/cloud` が来ていなければ「注意: /utlidar/cloud が届いていません。届くまで移動は保留されます」と出ます。

## 7. 安全の注意

- **伏せから始めます。** リモコンで Go2 を伏せさせてから（`error_code` が 1001・1004・2006 のどれか）サーバーを起動します。最初の指示は「立って」です。
- 前方 1 m と周りを空けてください。人・物・段差から離した、平らな床で動かします（この 2 行は、ページとサーバーのログにも出ます）。
- リモコンを持った人が、必ずそばにいてください。
- **少しずつ：** 最初は「立って」「止まって」だけで動作を確かめます。次に「前に進んで」（0.5 m を 1 回）、そのあと旋回、最後にジェスチャーです。
- **すぐ止める：**
  1. ページの「止める」ボタンか、「止まって」と言う：LLM を通さず、すぐ StopMove を送ります（その場で止まり、立ったまま）。
  2. サーバーの端末で Ctrl+C：StopMove を 3 回送ってから終わります。
  3. **Unitree のリモコン：** このアプリとは別に、Go2 に直接届きます。このアプリは、リモコンを効かなくする指令（SwitchJoystick）を送りません。いちばん強い止め方はダンピング（公式の Damp：全関節が止まりダンピング状態になる、非常停止用。[Motion Services Interface V2.0](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)）です。リモコンのボタンの組み合わせは、Go2 の取扱説明書で確かめておきます（「L2 長押し + B」と書いた資料もありますが、公式の文書ではまだ確かめていません）。立っているときにダンピングにすると、脚の力が抜けて体が床に落ちます（実機で未確認）。
- **ケーブルが抜けたとき・PC が止まったとき：** 公式によると、Go2 は最後の Move を 1 秒保ちます（V2.0 の Move の説明）。アプリは `/sportmodestate` が 0.5 秒途切れると移動を保留しますが、ケーブルが抜けていると StopMove も届きません。リモコンで止めます（実機で未確認）。

## 8. 困ったとき

実機では、指令を送っていないnative DDS送り手も複数見える場合があります。
Linux直接実行の `sensors`（PC診断）では、Go2用のLiDAR取り付け値
（x=0.28945 m、z=-0.046825 m、pitch=2.8782 rad）と床平面の除去を使います。
床は傾き・高さ・広がり・支持点数を確認できた場合だけ除去し、配線や壁を一律には無視しません。
`bash scripts/run_local.sh sensors` で前方・後方の判定を確認できます。
検出距離は胴体中心からです。左側の配線も判定範囲内なら障害物として残ります。
接続チェックは受信中の指令も監視し、待機中のDDS送り手は注意として表示します。
PC判定モードでは、待機中のDDS送り手が指令を発行した場合は停止します。
本体委譲モードではGo2内部がSport指令を生成するため、その内部指令はPCの競合判定から区別し、
回避API側の他の指令を監視します。Sport API側では起動後に増えた送り手や他のROSノードを拒否します。
ただし回避APIのSwitchGet（1002）はGo2からも周期的に届く状態照会なので、競合に含めません。
回避APIでは送り手の存在数ではなく、実際の移動・設定変更などの操作要求を監視します。
Sport APIの1002（BalanceStand）は別の命令なので、この照会例外には含めません。
停止後も受信監視は続け、元からあるDDS送り手の指令が1秒以上途切れたら新しい指示を受け付けます。
停止前の計画は自動再開しません。他のROSノードや起動後に増えた送り手は終了させる必要があります。
これは同時操作を安全に統合する仕組みではないため、アプリ・リモコン・他プログラムでの同時操作は避けます。

| 様子 | 確かめること |
|---|---|
| `ping 192.168.123.161` が返らない | ケーブル、Go2 の電源（起動し切るまで待つ）、差し口の IPv4（`ip -br addr`）。設定の後に PC を再起動したか（公式の勧め）。AIR・PRO には有線の差し口がありません |
| ping は返るのに、`ros2 topic list` に `/parameter_events`・`/rosout` しか出ない／「Go2 が見つかりません」 | **差し口の取り違え：** `env.txt` の `GO2_IFACE` が、`ip -br addr` で `192.168.123.x` がついた差し口と同じ名前か。**compose：** `-f compose.robot.yaml` で起動したか（無いと bridge のネットワークで、Go2 は見えません）。**ドメイン：** `env.txt` に `ROS_DOMAIN_ID` を書いていないか（Go2 は 0）。`robot_check` の［だめ］を順に直します |
| 差し口は合っているのに見えない（マルチキャスト） | `ip link show enp3s0` に `MULTICAST` があるか（無ければ `sudo ip link set enp3s0 multicast on`）。間にスイッチやハブを挟んでいれば、直結で試します |
| ファイアウォール | `sudo ufw status` が `active` なら、Go2 の網からの受信を許します：`sudo ufw allow in on enp3s0 from 192.168.123.0/24`。ほかのファイアウォール（`firewalld` など）も同じように、有線の差し口からの UDP を通します |
| `docker context show` が `desktop-linux` | Docker Desktop です。`docker context use default` で Docker Engine に切り替えます（[§2](#2-今ある-linux-の-pc-で確かめること)） |
| `/sportmodestate` は届くのに `/utlidar/cloud` が届かない | `ros2 topic list` に `/utlidar/cloud` があるか、`docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic hz /utlidar/cloud` で届く速さを見ます（Ctrl+C で終わります）。届かなければ、Go2 のソフトの版と一緒に書き留めて知らせてください（実機で未確認） |
| 「`/api/sport/request` に指令を送っている別のプログラムがあります」 | サーバー・`sensor_check`・PC で動いている ROS 2 のプログラム（Unitree の例など）を終了します。どれも動かしていないのに見えるなら、`robot_check` の送り手の数を書き留めて知らせてください。Go2 の側に送り手が見えるなら、今のままではサーバーは起動できません（実機で未確認） |
| `robot_check` で「受け手がいません」 | Unitree のアプリの「デバイス → サービスの状態」で、運動の制御のサービス（`mcf`。V1.1.6 より前は `sport_mode`）が止まっていないか。公式 Quick Start は、低レベル制御の例のためにこれを止める手順を書いていますが、このアプリは高レベルの窓口を使うので、止めてはいけません |
| 時刻 | アプリは Go2 の時刻（メッセージの stamp）を使わず、受け取ったときの PC の時計で数えます（`voice_ltl/ros_link.py`）。PC と Go2 の時計を合わせる必要はありません。PC の時計は、LLM を呼ぶ HTTPS のために合わせておきます（`timedatectl` で `System clock synchronized: yes`） |
| 姿勢が「不明」／「姿勢が計画と違います」 | [§6](#6-動かす前の確かめこの順に) の手順 4 の表と、実際の `error_code` を比べます |
| ページは開くが、Claude を呼べない | 有線にゲートウェイを付けていないか（`ip route get 1.1.1.1` の `dev` が Wi-Fi 側か）。`env.txt` のトークン |
