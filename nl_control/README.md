# go2_nl_control：自然言語で Go2 を動かす（ROS 2）

ターミナルに「立って1メートル前に進んで、左を向いて挨拶して」と打つと、LLM が動作計画に翻訳し、Go2 が実行します。
前方に人や物が来たら止まり、いなくなれば続きから動きます。

```
指示（日本語）
   ↓  LLM：決まった語彙の動作の列（JSON）に翻訳するだけ
計画 [stand → forward(1m) → turn_left(90度) → hello]
   ↓  監督器 check_plan()：語彙・値の範囲・状態遷移を検証。だめなら理由を添えて LLM に聞き直す（最大3回）
 ┄┄┄┄┄┄┄┄┄ ここから下は LLM を使わない ┄┄┄┄┄┄┄┄┄
実行器（10 Hz）── 毎周期、監督器に「今この動作をしてよいか」を確認
   ↑ 環境認識：/utlidar/cloud で前方の障害物、/sportmodestate で位置・向き
   ↓
/api/sport/request → Go2
```

## ファイル

| ファイル | 役割 |
|---|---|
| `nl_control.py` | **起動するのはこれ1本**。ROS 2 の送受信、ターミナル入力、メインループ |
| `go2_nl_control/actions.py` | LLM が使ってよい動作の一覧（語彙）、api_id、状態遷移 |
| `go2_nl_control/translator.py` | LLM への指示文（Goal / Context / Constraints / Done when）、構造化出力、聞き直し |
| `go2_nl_control/supervisor.py` | 監督器。計画の検証と、実行中の許可判定 |
| `go2_nl_control/perception.py` | LiDAR 点群から前方の障害物を判定（止まる距離と動き出す距離を分ける） |
| `go2_nl_control/executor.py` | 計画を1つずつ実行。距離・角度は /sportmodestate で測って止める |
| `offline_sim.py` | ロボットも ROS 2 も無しで、流れを確かめる模擬実行 |
| `tests/test_core.py` | テスト（`python3 -m pytest -q tests`） |

## 準備

1. いつもの環境（`demo_script.py` が動く PC）で、このフォルダを置く。
2. numpy が無ければ入れる：`pip3 install numpy`
3. このフォルダに `env.txt` を作る（`env.txt.example` をコピー）。キーはノートブックのセルではなくここに書く。

```
LLM_BASE_URL=https://api.groq.com/openai/v1
LLM_API_KEY=gsk_...
LLM_MODEL=openai/gpt-oss-20b
```

Ollama なら `LLM_BASE_URL=http://localhost:11434/v1`、`LLM_MODEL=gpt-oss:20b`、キーは何でもよい。
OpenAI 互換の API なら何でも動きます。構造化出力（JSON Schema）に対応していないサーバーでは、自動で JSON モードに切り替えます。

## 使い方

```bash
source ~/unitree_ros2/setup.sh
python3 nl_control.py
```

```
指示を入力して Enter（止まって = 停止 / q = 終了）
> 立って1メートル前に進んで、左を向いて挨拶して
[GO2] 考え中…（今の姿勢: LYING）
[GO2] LLM [1] 合格: {"steps": [...]}
[GO2] 計画: stand → forward(1m) → turn_left(90度) → hello
[GO2] 完了: stand
[GO2] 前方: 障害物 0.48 m
[GO2] 一時停止（forward(1m)）: 前方に障害物があります
[GO2] 前方: なし
[GO2] 再開（forward(1m)）
...
```

- **「止まって」「ストップ」「停止」** は LLM を通さず、すぐ StopMove を送ります。
- 動いている途中に新しい指示を打つと、いったん止めてから新しい計画に切り替えます（LLM が考えている間は動きません）。
- `q`・Ctrl+C・kill・SSH の切断では、StopMove を3回送ってから終わります。
- 前方 1 m と周り（特に後ろ）を空け、**リモコンですぐ止められる人がそばにいる状態**で動かしてください。

### 使える動作

`stand`（StandUp → 3秒 → BalanceStand）、`lie_down`、`sit`、`rise_sit`、
`forward` / `backward` / `left` / `right`（距離 0.05〜3 m）、`turn_left` / `turn_right`（5〜360 度）、`wait`（秒）、
`hello`、`stretch`、`content`、`heart`、`scrape`、`dance`。

宙返り・逆立ち・二足歩行・ジャンプは、危ないのであえて入れていません。足すときは `actions.py` に1行追加します。

## 安全のしくみ

| 仕組み | 内容 |
|---|---|
| LLM の出力を縛る | 語彙は enum、値は範囲つき。範囲外は丸めずに却下して聞き直す |
| 状態遷移の検証 | 伏せたまま歩く、おすわりのまま挨拶する、などの計画は実行前に却下 |
| 前方の障害物 | 0.5 m より近いと前へ進む動作を保留、0.7 m より遠くなると再開。後退・横歩き・旋回は続けられる |
| センサー途絶 | LiDAR か /sportmodestate が 0.5 秒途切れたら、移動をすべて保留 |
| 長い保留 | 15 秒待っても動けなければ計画を中止 |
| 移動のタイムアウト | 理論時間の 2.5 倍 + 5 秒たっても目標に届かなければ中止 |
| Move の送り方 | 毎周期送り直し、加速度を制限（公式 V2.0 の推奨どおり） |
| 1本だけで動く | 二重起動をロックで防止。/api/sport/request に自分以外の送り手がいれば、起動も指示も断る |

## LiDAR の取り付けに合わせる（最初に一度）

`nl_control.py` 冒頭の `PERCEPTION` の `lidar_x`, `lidar_z`, `lidar_pitch` は仮の値です。機体ごとに確かめてください。

1. Go2 を立たせ、前方 0.5 m に箱を置く。
2. `rviz2` で `/utlidar/cloud`（Fixed Frame は `utlidar_lidar`）を表示し、箱の点の座標を見る。
3. 箱の点が「機体の中心から前に 0.5 m、高さ 0〜0.4 m」に来るよう、`lidar_x`（前後のずれ）、`lidar_z`（高さのずれ）、`lidar_pitch`（下向きの傾き [rad]）を合わせる。
4. `python3 nl_control.py` を起動し、箱を近づけたり離したりして「前方: 障害物 / なし」の表示が切り替わる距離を確かめる。

床を障害物と誤検出するなら `z_min` を上げ、低い物を見逃すなら下げます。

## ロボットなしで確かめる

```bash
python3 offline_sim.py        # 指示 → 却下と聞き直し → 実行 → 障害物で停止 → 再開
python3 -m pytest -q tests    # 16 個のテスト
```

## 注意・わかっていないこと

- **error_code と姿勢の対応**（`actions.py` の `_ERROR_CODE_TO_STATE`）は公式 V2.0（ファームウェア V1.1.6 以上）の表から作りました。手元の Go2 で `ros2 topic echo /sportmodestate` の `error_code` を見て、立つ・座る・伏せるで値が表の通りか確かめてください。わからない値のときは、プログラムが覚えている姿勢を使います。
- **ジェスチャーの待ち時間**は固定値（挨拶 5 秒、ダンス 12 秒など）です。早く次に進んでしまうなら `actions.py` の秒数を延ばしてください。
- **Unitree のアプリやリモコン**は ROS 2 の送り手ではないので、「別の送り手」として検出できません。リモコンは止めるための手段として手元に残しておいてください。
- 障害物の判定は **「前方の箱の中に点があるか」だけ**です。段差・穴・ガラス・細い棒は見逃すことがあります。
- 「赤い箱の前まで行って」のような、**物を見分ける指示**にはまだ対応していません。カメラ画像を VLM で見る認識を足すなら、`perception.py` に命題（例：`at_red_box`）を返す関数を加え、監督器の判定に使う形で拡張できます。
