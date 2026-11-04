# Gazebo での一式の確認（Task 17）

- 日付：2026-11-04。場所：この Mac の Docker Desktop（arm64、CPU 8、メモリ 7.7 GiB）。同じ時間に、別の担当の読むだけのレビュー（別のフォルダ・別の compose）が動いていた。
- 使ったもの：app は `go2-voice-ltl:dev`（ROS 2 Humble）、gazebo は `go2-voice-ltl-gazebo:dev`（ROS 2 Jazzy + Gazebo Harmonic、`docker/patches/` の 0001・0002 入り）。橋渡し（`gazebo/bridge_core.py`・`gazebo_bridge.py`）は Task 16fix のまま（R22・R23 入り）。
- 測り方（測る道具はコミットしていない）：
  - gazebo のコンテナで、Gazebo の本当の位置と傾き（`/model/robot/odometry`）・`/sim/world`・`/clock`・`/sportmodestate` の `error_code` を 20 Hz で記録した。
  - `run_formula` の出力に時刻を付け、記録と合わせて、動作ごとの最大 |roll|・進んだ距離・回った角度を出した。
  - real-time factor は `/clock` の進み ÷ 実際の時間。
  - 画面は Playwright（ヘッドレスの Chromium）で 1 秒ごとに撮った（Gazebo は `http://127.0.0.1:6080/vnc.html?autoconnect=1`、操作画面は `http://127.0.0.1:8810/` を幅 1440 で）。

## 結論

| 確かめたこと | 結果 |
|---|---|
| `RosLink.wait_ready()` が Jazzy の橋渡しを見つけるか | 見つけた。`/api/sport/request` の購読者 1、1.1 秒。姿勢 LYING（1001）と位置が届いた |
| 基本動作：`run_formula` で 立つ → 前進 → 左に回る → 伏せる | 3 回とも計画どおりに完了、`exit=0` |
| 箱で止まる | 箱の手前で止まり、`collided` は偽のまま。15 秒後に「15秒待っても進めません」で `exit=1`。箱をどけて流し直すと `exit=0` |
| 歩いている途中に箱を見て止まる | 体の中心から箱の面まで 0.44 m で止まった。15 秒後に「15 秒待っても 前方に障害物があります」で `exit=1` |
| 本物の Claude（操作画面から） | 「立って、前に進んで、左を向いて、伏せて」→ `F(moved_forward & F(turned_left & F lying))`（1 回目で合格）→ 最後まで動いて「終わりました」 |
| 前進のすぐあとに回るときの揺れ | 8 回で最大 0.39 rad（0.5 未満） |
| ほかの続けた動き（前進 → 前進、旋回 → 前進） | 0.50・0.54・0.67 rad が 1 回ずつ。転ばなかった |
| 止めた後の落ち着く時間（1.0 秒、橋渡しだけ） | 試したが悪くなった（7 回のうち 1 回は転び、1 回は 1.12 rad）。**入れなかった** |
| real-time factor | 0.74〜0.90（画面を 2 つ撮りながら Claude も動かしたときが 0.76） |

## 準備：app から gazebo の橋渡しが見えるか

```bash
docker compose --profile gazebo up -d gazebo      # 40 秒待つ
docker compose run --rm --no-deps app python3 out/t17/check_ready.py   # RosLink を作って wait_ready（コミットしていない）
```

```text
wait_ready OK in 1.1 s; subscribers of /api/sport/request: 1; other senders: 0; posture LYING code 1001 pose (0.0037, 0.0019, -0.0267)
reading Reading(front='CLEAR', back='CLEAR', front_dist=None, back_dist=None)
```

- Humble と Jazzy で `ros_discovery_info` の型が違うので、標準エラーに `serdata.cpp:384` のエラーが出続ける（`gazebo_feasibility_jazzy.md`）。購読者の数・データは問題ない。
- app の Web のページが動いているときに `run_formula` を流すと、`/api/sport/request に指令を送っている別のプログラムがあります。そちらを終了してから起動してください` で `exit=1`（StopMove は送らない）。

## 1. 基本動作（式から直接）

```bash
docker compose --profile gazebo up -d gazebo && sleep 40
docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(turned_left & X F lying))"; echo "exit=$?"
```

1 回目の出力（標準出力そのまま）：

```text
指示：「(run_formula)」
式：F(moved_forward & X F(turned_left & X F lying))
計画：立つ → 前進 → 左に回る → 伏せる
開始：立つ
完了：立つ（伏せ → 立ち）
開始：前進
完了：前進
開始：左に回る
完了：左に回る
開始：伏せる
完了：伏せる（立ち → 伏せ）
終わりました
結果：done（終わりました）
exit=0
```

動作ごとの値（Gazebo の本当の位置。時間は実際の時間で、`開始` から `完了` まで）：

| 回 | 始め | 立つ | 前進 | 左に回る | 伏せる | 最大 \|roll\|（前進 / 回る） | RTF |
|---|---|---|---|---|---|---|---|
| 1（画面を撮りながら） | 起動したて（伏せ、向き −2°） | 4.1 s | 2.6 s、0.59 m | 4.4 s、+95° | 3.1 s（その間に +5°） | 0.138 / 0.302 | 0.85 |
| 2 | 1 の続き（伏せ、向き 100°） | 4.1 s | 1.8 s、0.55 m | 4.2 s、+94° | 3.1 s | 0.202 / 0.393 | 0.87 |
| 3 | 2 の続き（伏せ、向き −168°） | 4.1 s | 1.9 s、0.60 m | 3.9 s、+93° | 3.1 s | 0.124 / 0.075 | 0.89 |

- 3 回とも `exit=0`。1 回目で、立っている間の胴の高さは最低 0.28 m、伏せた後は 0.15 m。
- 前進は 0.5 m を測って StopMove を送るが、止まるまでに 0.05〜0.10 m 進む。旋回は 90 度で止めて 93〜95 度、伏せる間にもう 3〜5 度流れる。

画面（1 回目）：

- `gazebo_e2e_stand.png`：立ち終わった所（前進の前）。
- `gazebo_e2e_forward.png`：前進が終わって 0.5 秒後。画面の右手前へ 0.6 m 出た。
- `gazebo_e2e_turn.png`：左に回り終わって 1.1 秒後。向きが 90 度変わった。
- `gazebo_e2e_lie.png`：伏せ終わって 5 秒後。

## 2. 箱で止まる

```bash
docker compose run --rm app ros2 topic pub --once /sim/obstacles std_msgs/msg/String '{data: "{\"cmd\":\"clear\"}"}'
docker compose --profile gazebo restart gazebo     # きれいな状態から（40 秒待つ）
docker compose run --rm app python3 -m voice_ltl.run_formula "F turned_left"    # 立って、画面の奥（+y）を向く
docker compose run --rm app ros2 topic pub --once /sim/obstacles std_msgs/msg/String '{data: "{\"cmd\":\"front\"}"}'
docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward)))" --timeout 40; echo "exit=$?"
```

- 「立たせて前向きにした所」は、箱を画面に入れるため、立ってから左に 90 度回して画面の奥を向かせた（`F turned_left`、`exit=0`）。箱はいつも Go2 の今の向きの 1.2 m 前に出る（`箱を置きました（front）: sim_box_1 x=-0.09 y=1.20`、一辺 0.3 m）。

```text
計画：前進 → 前進 → 前進 → 前進
開始：前進
完了：前進
前方に物が出た
今は進める動作がないので待っています
15秒待っても進めません
結果：error（15秒待っても進めません）
exit=1
```

- 1 回目の前進（0.58 m）が終わった所で、箱が 0.5 m より近くなった（体の中心から箱の面まで 0.46 m）。監督器は「前に障害物があるときの前進」を許さないので、2 回目の前進を始めずに待ち、15 秒で中止した。
- `/sim/world` の `collided` はずっと偽。いちばん近づいたのは 0.464 m（`collided` は 0.35 m 未満）。
- 画面：`gazebo_e2e_box_stop.png`（待っている間。箱の手前で立っている）。
- 箱をどけて（`clear`）同じ式をもう一度流すと、前進 4 回（0.62・0.58・0.61・0.57 m）で `結果：done（終わりました）`、`exit=0`。

### 歩いている途中に箱が出たとき

上の流し方だと、箱までの距離（1.2 m − 0.15 m）が 1 回の前進とほぼ同じなので、いつも前進が終わった所で「前方に物が出た」になる。歩いている途中で止まることも確かめるため、前進 4 回の式を流している途中（2 回目の前進の中ほど）に `front` を送った（gazebo のコンテナから `ros2 topic pub`）。

```text
開始：前進
完了：前進
開始：前進
完了：前進
開始：前進
前方に物が出た
一時停止：前進（前方に障害物があります）
中止：前進（15 秒待っても 前方に障害物があります）
結果：error（15 秒待っても 前方に障害物があります）
exit=1
```

- 3 回目の前進の途中で実行器が StopMove を送り、その前進で 0.34 m 進んだ所、体の中心から箱の面まで 0.44 m で止まった（止まる距離は 0.5 m。LiDAR・制御とも 10 Hz なので、その分だけ近づく）。`collided` は偽のまま。

## 3. 本物の Claude で（アプリ一式）

```bash
docker compose --profile gazebo up -d gazebo              # 40 秒待つ
SIM_KIND=gazebo docker compose --profile gazebo up -d     # app
curl -X POST -H 'Content-Type: application/json' --data '{"text":"立って、前に進んで、左を向いて、伏せて"}' http://127.0.0.1:8810/api/text
```

- `env.txt` は開いていない（compose の `env_file` で app に渡る）。
- `POST /api/text` → 202。app のログ（`voice_ltl.session`、時刻は UTC）：

```text
14:44:57.971 指示: 立って、前に進んで、左を向いて、伏せて
14:45:05.050 式: F(moved_forward & F(turned_left & F lying))
14:45:05.450 監督器: 44 状態
14:45:05.450 計画: stand
14:45:09.649 イベント: stand_to_standing
14:45:09.749 計画: forward(0.5m)
14:45:13.752 イベント: forward_to_moved_forward_standing
14:45:13.850 計画: turn_left(90度)
14:45:18.650 イベント: turn_left_to_standing_turned_left
14:45:18.799 計画: lie_down
14:45:21.850 イベント: lie_down_to_lying
14:45:21.950 終わりました
```

- Claude の翻訳は 7.1 秒、1 回目で `check()` に合格。合成は 0.4 秒。
- Gazebo の値：前進 4.0 s・0.53 m、左に回る 4.8 s・+96°、最大 |roll| は前進 0.297・回る 0.303。立っている間の胴の高さは最低 0.24 m（前進の歩き出し）。RTF 0.76（画面を 2 つ撮っていた）。
- 画面：
  - `gazebo_e2e_claude_ui_executing.png`：操作画面、左に回っている所（計画の「左に回る」が実行中、計画の道の図が青い）。右上は「シミュレーション · Gazebo」。
  - `gazebo_e2e_claude_ui_done.png`：操作画面、「終わりました」（立つ → 前進 → 左に回る → 伏せる がすべて済み、姿勢は伏せ）。
  - `gazebo_e2e_claude_gazebo_turn.png`：同じ時刻の Gazebo（回っている所）。
  - `gazebo_e2e_claude_gazebo_lie.png`：終わって 5.5 秒後の Gazebo（伏せている）。
- **app と gazebo を同時に起動すると、app は動かない。** `docker compose --profile gazebo down` の後に `SIM_KIND=gazebo docker compose --profile gazebo up -d` で両方を一度に起動したところ、app は 5 秒（`wait_ready` の既定）待っても橋渡しが見えず、`Go2 が見つかりません` で終わった（`Exited (1)`）。gazebo が用意できてから同じコマンドをもう一度流すと起動した。README には「先に gazebo、40 秒待ってから app」と書いた。

## 4. 揺れと、止めた後の落ち着く時間

Task 16fix の報告（前進のすぐあとに回ると roll が 0.5 rad を越えることがある）を受けて、動作ごとの最大 |roll| を測った。回ごとに 1 行。数字は「開始」から「完了」まで（止めた後の揺れは、次の動作の行に入る）。

### 落ち着く時間なし（今の橋渡し）

| 回 | 動作の列 | 最大 \|roll\| [rad] |
|---|---|---|
| 基本動作 1〜3 | 前進 → 左 | 0.138 → 0.302、0.202 → 0.393、0.124 → 0.075 |
| 本物の Claude | 前進 → 左 | 0.297 → 0.303 |
| 箱の後（どけて流し直し） | 前進 ×4 | 0.051、**0.500**、0.446、**0.673** |
| 同じ | 前進 ×4 | 0.059、0.141、0.353、0.297 |
| 同じ | 前進 → 左 → 前進 → 右 | 0.252、0.238、0.092、0.173 |
| 同じ | 前進 → 左 → 前進 → 右 | 0.062、0.378、**0.543**、0.098 |

- 前進のすぐあとに回るとき（左 6 回・右 2 回）は最大 0.393 で、0.5 を越えなかった。
- 0.5 を越えたのは、前進 → 前進（0.500・0.673）と、左 → 前進（0.543）。どれも転ばずに次へ進んだ。

### 落ち着く時間 1.0 秒を試した（入れなかった）

ruling（R33）が認めた形で、`BridgeCore` だけに入れて試した：歩いていた（速度を controller に出していた）のを止めたら、次の速度を `STOP_SETTLE_S` = 1.0 秒（シミュレーションの時間）0 のまま待たせる。止める 0 はすぐ出す。テストを書き（先に落ちることを確かめ）、通してから、Gazebo を起動し直して 7 回流した。

| 回 | 動作の列 | 最大 \|roll\| [rad] |
|---|---|---|
| 1・2 | 立つ → 前進 → 左 → 伏せる | 0.205 → 0.241、0.292 → 0.309 |
| 4 | 前進 ×4 | 0.055、**1.903**、**3.040**（転んで裏返り、起き上がった）、**0.810** |
| 5 | 前進 ×4 | 0.128、0.140、**1.119**、**0.648** |
| 6 | 前進 → 左 → 前進 → 右 | 0.066、0.439、0.211、0.123 |
| 7 | 前進 → 左 → 前進 → 右 | 0.162、0.099、0.163、0.433 |

（3 回目は「立つ」だけ。）

- 4 回目で転んだ。橋渡しのログと記録を合わせると、1 回目の前進を止めた後、**速度 0 で待っている間に** roll が 0.10 → 0.21 と育ち、待ち終わって前進の速度（実行器の加速の途中の値ではなく、最後に届いた 0.3 m/s）を一度に入れた所で 0.69 → 1.48 → 裏返った。
- 止めた後にその場で足踏みさせても揺れは収まらず、待ち終わりに速度が一度に変わるぶん悪くなる。前進の後の旋回は、入れなくても 0.5 を越えなかった。そこで入れなかった（`gazebo/bridge_core.py` と `tests/test_bridge_core.py` は元のまま）。
- 経緯は `gazebo_turn_fix.md` の最後にも書いた。

## 5. 気づいたこと（このタスクでは直していない）

- **転んでも「完了」になることがある。** 上の 4 回目は、転んで横へ 0.67 m 流れた間に前進の距離（実行器は向きによらず、動いた距離で数える）を満たし、「完了：前進」になった。Gazebo の橋渡しは転んでも `error_code` を 1013 のままにするので、Session も姿勢のずれに気づかず、最後は「終わりました」だった。実機でも、転んだことは `error_code` だけではわからないかもしれない。
- **一緒に起動すると app が止まる**（上の 3）。compose に `restart` も、gazebo の用意を待つ仕組みも無い。
- 0.5 m の前進は 0.52〜0.64 m 進み、90 度の旋回は 92〜96 度回る（止めた後の流れ）。前進 → 前進を続けると、横へ 0.3 m ほどずれた（前進 4 回で x が −0.01 → −0.28）。

## real-time factor と CPU

- real-time factor（`/clock` の進み ÷ 実際の時間、回ごと）：0.85・0.87・0.89（基本動作）、0.87・0.90（箱）、0.82・0.80・0.81（揺れ）、0.74〜0.78（歩いている途中の箱）、0.76（本物の Claude、画面を 2 つ撮りながら）。Gazebo の画面右下の表示は 76〜91 %。
- `docker stats`：gazebo 443 %・1.08 GiB、app 2.7 %・132 MiB（app を起動して待っている間）。

## 再現のしかた

```bash
cd <このリポジトリ>
docker compose --profile gazebo up -d gazebo           # 40 秒待つ
docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(turned_left & X F lying))"; echo "exit=$?"
docker compose run --rm app ros2 topic pub --once /sim/obstacles std_msgs/msg/String '{data: "{\"cmd\":\"front\"}"}'
docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward)))" --timeout 40; echo "exit=$?"
SIM_KIND=gazebo docker compose --profile gazebo up -d  # 本物の Claude で：app も起動し、http://localhost:8810 で話す
docker compose --profile gazebo down
```
