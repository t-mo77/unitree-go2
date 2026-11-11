# 声で指示 → LTL → 監督制御 → Go2

声で話すと、LLM（Claude か Codex）が指示を LTL の式に翻訳し、監督制御が安全な動作の列を計算して、Unitree Go2 を動かします。仕組みは [docs/overview.md](docs/overview.md)、設計は [docs/design.md](docs/design.md) にあります。

## 目次

- [Linux PCで直接動かす（Dockerなし）](#linux-pcで直接動かすdockerなし)
- [1. 準備（Mac）](#1-準備mac)
- [2. 偽の Go2 で動かす](#2-偽の-go2-で動かす)
- [3. Gazebo で動かす](#3-gazebo-で動かす)
- [4. 実機の Go2 で動かす（有線・Linux の PC）](#4-実機の-go2-で動かす有線linux-の-pc)
- [5. テスト](#5-テスト)
- [詳しい説明](#詳しい説明)

## Linux PCで直接動かす（Dockerなし）

Ubuntu 22.04・ROS 2 Humbleと `~/unitree_ros2/cyclonedds_ws/install` があるPCでは、
Dockerを使わずに起動できます。アプリのPython依存は専用の `.venv` に入れます。
このPCでは既存のROS 2・Unitreeメッセージ・Graphviz・Codex CLIを使います。

```bash
cd ~/student_ws/unitree-go2
bash scripts/setup_local.sh                   # 初回。準備済みなら不要
# env.txtがなければ env.txt.example から作り、LLM_BACKEND=codex / GO2_IFACE=enp3s0 を設定
bash scripts/run_local.sh fake                # 偽Go2とWeb画面。Ctrl+Cで両方終了
```

ページは <http://localhost:8810>。偽Go2はlocalhost・ROSドメイン73で動き、
実機のドメイン0とは分離します。CodexはこのPCのログインを利用します。
モデルは既存の `../.models/whisper` があれば再利用します。
文字起こしは既定でCPUを使うため、CUDA 12の追加導入は不要です。

実機に切り替えるときは偽Go2をCtrl+Cで終了してから、有線LANを接続し、
リモコンを持った人がそばにいる状態で、次の順に確認します。

```bash
bash scripts/run_local.sh check               # 実機の接続確認。動作指令は送らない
bash scripts/run_local.sh sensors             # LiDAR確認。Ctrl+Cで終了
bash scripts/run_local.sh robot               # 実機のWeb操作
```

実機の準備は [docs/real_robot.md](docs/real_robot.md) の姿勢・LiDARの確認にも従います。
Linux直接実行の実機モードは、移動可否をGo2本体の障害物回避サービスに任せます。
PCはLiDAR点群から移動を禁止しません。画面の「Go2判断」は、PCが障害物なしと判断した意味ではありません。
起動時に本体の回避機能がONであることを確認し、API操作モードへ切り替えます。
アプリの実行中はAPI操作モードを使い、終了時に解除します。
移動は `/api/obstacles_avoid/request` へ送り、姿勢変更は従来のSport APIを使います。
本体回避機能が無効・未対応・無応答なら起動を中止し、通常Moveへは切り替えません。
`sensors` は診断専用でPC判定を表示するため、実機操作モードの停止条件とは異なります。
通信断・明示的停止・処理異常・動作完了/タイムアウトの管理はPC側にも残します。
既存の有線IPv4が `192.168.123.x/24` なら、`.99` に変更する必要はありません。
テストは `bash scripts/run_local.sh test`。

## 1. 準備（Mac）

Docker Desktop を起動してから：

```bash
cp env.txt.example env.txt                                       # env.txt は git に入れない・人に渡さない
claude setup-token                                               # Claude のとき。出たトークンを自分で env.txt の CLAUDE_CODE_OAUTH_TOKEN= に書く
docker compose run --rm --no-deps app codex login --device-auth  # Codex のとき（env.txt は LLM_BACKEND=codex）
docker compose build
```

`env.txt` があるフォルダで `docker compose config` を実行しないでください（トークンが画面に出ます）。詳しくは [docs/setup.md](docs/setup.md)。

## 2. 偽の Go2 で動かす

```bash
docker compose --profile fake up      # アプリと偽の Go2
docker compose --profile fake down    # 全部止める
```

1. <http://localhost:8810> を開き、「マイクを使う」を押します。
2. スペースを押している間話して、離します（例：「立って挨拶して」）。
3. 止めるときは「止まって」と言うか、画面の「止める」を押します。

詳しくは [docs/simulation.md](docs/simulation.md)、話せることは [docs/vocabulary.md](docs/vocabulary.md)。

## 3. Gazebo で動かす

```bash
docker compose --profile gazebo build gazebo             # 初回だけ（時間がかかります）
SIM_KIND=gazebo docker compose --profile gazebo up -d    # Gazebo とアプリ
docker compose logs -f app                               # 「ページ: http://localhost:8810」が出たら開く
docker compose --profile gazebo down                     # 全部止める
```

- 操作画面：<http://localhost:8810>
- Gazebo の画面：<http://localhost:6080/vnc.html?autoconnect=1>

LLM と声を使わずに、式から動かすとき：

```bash
docker compose --profile gazebo up -d gazebo
SIM_KIND=gazebo docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(turned_left & X F lying))"
```

詳しくは [docs/simulation.md](docs/simulation.md#gazebo-で動かす)。

## 4. 実機の Go2 で動かす（有線・Linux の PC）

Go2 EDU と、Linux の入った PC を LAN ケーブルでつなぎます。Mac からは実機につながりません。**この手順は、まだ実機で通していません。**

- 前方 1 m と周りを空けてください。
- リモコンを持った人が、必ずそばにいてください。
- Go2 はリモコンで伏せさせてから始めます。

有線の差し口（例：`enp3s0`）を `192.168.123.99/24` に固定します（ゲートウェイは付けない。NetworkManager のとき）：

```bash
ip -br link                                                      # 差し口の名前
sudo nmcli connection add type ethernet ifname enp3s0 con-name go2 ipv4.method manual ipv4.addresses 192.168.123.99/24 ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up go2
ping -c 3 192.168.123.161                                        # Go2 から返事が来る
```

`env.txt` に `GO2_IFACE=enp3s0` を書いてから（`ROS_DOMAIN_ID` は書かない）：

```bash
docker compose -f compose.robot.yaml build
docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.robot_check                     # つながったか（全部 OK になるまで直す）
docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic echo /sportmodestate --field error_code  # 姿勢の値（リモコンで立つ・座る・伏せる）
docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.sensor_check                    # LiDAR（Ctrl+C で終わる）
docker compose -f compose.robot.yaml up                                                                          # 起動。PC のブラウザで http://localhost:8810
```

止めるときは、画面の「止める」、「止まって」、端末の Ctrl+C、Unitree のリモコン。

用意するもの、値の見方、安全の注意、うまくいかないときは [docs/real_robot.md](docs/real_robot.md)。

## 5. テスト

```bash
docker compose run --rm --no-deps app python3 -m pytest -q -m "not slow" tests nl_control/tests
```

## 詳しい説明

| ドキュメント | 中身 |
|---|---|
| [docs/overview.md](docs/overview.md) | これは何か・仕組み |
| [docs/setup.md](docs/setup.md) | 準備の詳しい説明（LLM のログイン、`env.txt`、書き起こしのモデル、テスト） |
| [docs/simulation.md](docs/simulation.md) | 偽の Go2 と Gazebo（ページ、物、`run_formula`、controller のパッチ、転んだとき） |
| [docs/vocabulary.md](docs/vocabulary.md) | 話せること、物を使う指示、単語帳と状態遷移図の足し方 |
| [docs/real_robot.md](docs/real_robot.md) | 実機の Go2 を有線でつなぐ（用意するもの、確かめ、安全の注意、困ったとき） |
| [docs/unknowns.md](docs/unknowns.md) | わかっていないこと |
| [docs/design.md](docs/design.md) | 設計書 |
