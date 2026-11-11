# 声で指示 → LTL → 監督制御 → Go2

声で話すと、LLM（Claude か Codex）が指示を LTL の式に翻訳し、監督制御が安全な動作の列を計算して、Unitree Go2 を動かします。仕組みは [docs/overview.md](docs/overview.md)、設計は [docs/design.md](docs/design.md) にあります。

## 目次

- [1. 準備（Mac）](#1-準備mac)
- [2. 偽の Go2 で動かす](#2-偽の-go2-で動かす)
- [3. Gazebo で動かす](#3-gazebo-で動かす)
- [4. 実機の Go2 で動かす（有線・Linux の PC）](#4-実機の-go2-で動かす有線linux-の-pc)
- [5. テスト](#5-テスト)
- [詳しい説明](#詳しい説明)

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
