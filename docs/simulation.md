# 偽の Go2 と Gazebo で動かす

コマンドだけなら [README](../README.md#2-偽の-go2-で動かす) にあります。

## 偽の Go2 で動かす

```bash
docker compose --profile fake up      # アプリと、偽の Go2（sim）を起動
```

1. ブラウザで <http://localhost:8810> を開きます。ポートを変えたいときは `GO2_UI_PORT` を渡します（例：`GO2_UI_PORT=8811 docker compose --profile fake up`）。シェルの環境変数でも、compose.yaml と同じ場所に置いた `.env` の `GO2_UI_PORT=8811` でも構いません。ホスト側は 127.0.0.1 だけに公開します。
2. 「マイクを使う」を押して許可します。
3. **スペースを押している間**話して、離します。画面に書き起こし、Claude の式、`check()` の合否、状態遷移図、Go2 の状態、上から見た図が出ます。言い換えや近い動作に置き換えたとき（「走って」→ 前へ歩く など）は、式の下の「こう受け取りました」に出ます。
4. 「前に箱を置く」「後ろに箱を置く」「箱を全部どける」ボタンで、障害物を試せます（シミュレーションのときだけ表示されます）。

**物（箱と椅子）と上から見た図：** 偽の Go2 にも Gazebo にも、初めから箱（`box_1`、(2.5, 1.5)）と椅子（`chair_1`、(2.5, −1.5)）が置いてあります。Go2 の前のカメラに写った物が「物の一覧」になり、「箱の周りを一周して」のような指示に使えます（[物を使う指示](vocabulary.md#物を使う指示箱と椅子)）。

- 上から見た図に、物が名前（箱・椅子）と大きさの四角で出ます。今は見えていない物は、うすく出ます。
- 物を使う指示のときは、0.5 m のマス目、入れないマス（灰色）、計画の道（青い線）と目標のマスも出ます。図の範囲は、歩いている間は動きません。
- 上から見た図の下に、今見えている物と、Go2 から見た向きと距離が出ます（最初の位置での例：「見えている物：箱（左前 2.9 m）、椅子（右前 2.9 m）」）。
- 状態遷移図は、200 状態を超えると描かずに「大きすぎるので図は省きました（N 状態）。」と出します（物を使う指示では「上から見た図を見てください」も添えます。物を使う指示の監督器は数千状態になります）。「計画の道」の図はいつも出ます。Go2 は、図を描き終わるのを待たずに動き出します。
- 箱と椅子は LiDAR にも写るので、近づきすぎると、ボタンで置いた箱と同じように止まります。「箱を全部どける」でどけるのは、ボタンで置いた箱だけです。

止め方は 3 つあります。

- 「止まって」と言う（LLM を通さず、すぐ StopMove）
- 画面の「止める」ボタン
- `docker compose --profile fake down`（全部止める）

`--profile fake` を付けずに `docker compose up` だけだと、偽の Go2 は起動しません。

## Gazebo で動かす

物理シミュレーション（Gazebo Harmonic）の中の Go2 を、実機と同じ窓口（`/api/sport/request`・`/sportmodestate`・`/utlidar/cloud`）で動かします（設計書 §9.1）。app は今までどおり ROS 2 Humble、gazebo のコンテナは ROS 2 Jazzy です。確かめた記録は [docs/reference/gazebo_e2e.md](reference/gazebo_e2e.md) にあります。

**起動と止め方**

```bash
docker compose --profile gazebo build gazebo             # 初回だけ（Gazebo と controller のビルドで時間がかかります）。イメージは go2-voice-ltl-gazebo:patch-0005
SIM_KIND=gazebo docker compose --profile gazebo up -d    # Gazebo と app（Web のページ）を一度に起動
docker compose --profile gazebo down                     # 全部止める
```

- app は、Gazebo の Go2 が用意できるまで最長 90 秒待ってから、ページを開きます（`SIM_KIND=gazebo` のときの既定。Gazebo が用意できるまでは、数秒から 40 秒ほど）。app のログ（`docker compose logs -f app`）に `ページ: http://localhost:8810` が出たら開けます。待つ秒数は `GO2_WAIT_READY_S` で変えられます（例：`GO2_WAIT_READY_S=150 SIM_KIND=gazebo docker compose --profile gazebo up -d`）。
- 90 秒待っても見つからなければ、app は「Go2 が見つかりません」で止まります（`docker compose ps` で app が出ていない）。Gazebo のログ（`docker compose logs gazebo`）を確かめてから、もう一度 `SIM_KIND=gazebo docker compose --profile gazebo up -d` を実行してください。
- 念のための別の方法：2 段に分けて起動することもできます。先に `docker compose --profile gazebo up -d gazebo` で Gazebo だけを起動し、Go2 と controller が用意できるまで 40 秒ほど待ってから、`SIM_KIND=gazebo docker compose --profile gazebo up -d` で app を起動します。
- 操作画面：<http://localhost:8810>（右上に「シミュレーション · Gazebo」と出ます）。話し方・箱のボタンは上と同じです。
- Gazebo の画面：<http://localhost:6080/vnc.html?autoconnect=1>（noVNC）。GPU が無いのでソフトウェアで描き、カクつきます。右下の % が実時間に対する速さ（real-time factor）です。この Mac では 0.74〜0.90 でした（`/clock` の進み ÷ 実際の時間。[gazebo_e2e.md](reference/gazebo_e2e.md)）。
- app の標準エラーに `serdata.cpp:384` のエラーが出続けますが、データは届いています（Humble と Jazzy で、ノードの名前を伝える型が違うため）。
- `SIM_KIND=gazebo` は、ページの表示を「Gazebo」にするだけです。動かし方は偽の Go2 と同じです。

**物（箱と椅子）**

- Gazebo を起動すると、橋渡しが箱 `box_1`（50 cm 角）を (2.5, 1.5) に、椅子 `chair_1`（座面 45 cm、背もたれつき）を (2.5, −1.5) に置きます。最初の向きから見て、箱は左前、椅子は右前です（odom の y は左向き）。
- Go2 の前のカメラ（Gazebo の logical camera。水平 120°・0.1〜8 m・5 Hz。画像は出さず、見えている物の名前と位置だけ）に写った物を、橋渡しが覚えて、2 Hz で `/go2/objects`（`std_msgs/String` の JSON、odom の座標）に出します。大きさは LiDAR の点で測ります。見えなくなった物も、Gazebo を起動し直すまで `visible: false` で残ります。
- 物の一覧を見るには：

  ```bash
  docker compose run --rm --no-deps -T app ros2 topic echo --once --full-length /go2/objects
  ```

  `data:` のあとに、`{"frame": "odom", "stamp": …, "objects": [{"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.5, 0.5], "seen": …, "visible": true}, …]}` の形の JSON が 1 通出ます。`size` は [x 方向の長さ, y 方向の長さ] [m] です。`--full-length` を付けないと、`ros2 topic echo` は文字列を 128 文字で切ります（最後が `...`）。物 2 つでも 128 文字を超えるので、椅子 `chair_1` が出ません。
- 設計は [docs/superpowers/specs/2026-11-05-go2-objects-design.md](superpowers/specs/2026-11-05-go2-objects-design.md)、Gazebo で確かめた記録は [docs/reference/gazebo_objects_e2e.md](reference/gazebo_objects_e2e.md) にあります。

**式から直接動かす（`run_formula`）**

LLM も音声も使わずに、LTLf の式から Go2 を動かします。LLM にログインしていなくても試せます。翻訳の代わりにその式を使うだけで、あとはページから話したときと同じ道（`check()`・合成・監督器・実行器・センサー）を通ります。

```bash
docker compose --profile gazebo up -d gazebo     # Gazebo だけ（app は起動しない）
SIM_KIND=gazebo docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(turned_left & X F lying))"
echo "exit=$?"
```

- `SIM_KIND=gazebo` を付けると、Gazebo の Go2 が用意できるまで最長 90 秒待ちます（付けないと 5 秒。`GO2_WAIT_READY_S` で変えられるのはページと同じ）。Gazebo を起動してすぐに流せます。
- 標準出力に、計画・各動作の開始と完了・最後の結果（例：`結果：done（終わりました）`）を出します。終了コードは `done` なら 0、それ以外（中止、時間切れ、式が単語帳に合わない、Go2 が見つからない）は 1 です。
- `--timeout 秒`（既定 120）：それまでに終わらなければ、止めて終わります。Ctrl+C でも止めます（どちらも StopMove を 3 回送ります）。
- 物の命題の式も使えます（例：`"F around_box_1"`、`"F near_chair_1"`）。式の物が `/go2/objects` にそろうまで最長 3 秒待ち、そろわなければ「「box_1」が見つかりません」と出して終わります（終了コード 1）。箱の周りの一周は、偽の Go2 で 43 手・2 分ほどかかるので、`--timeout` を 300 秒以上にします（既定の 120 秒では足りません）。
- 設計書（物）の §2 の合格の条件（回転数・物の縁までの距離・最後の位置と向き・傾き）を測るときは、`gazebo/tools/objects_check.py` で `run_formula` を動かします（使い方はファイルの先頭。記録は [gazebo_objects_e2e.md](reference/gazebo_objects_e2e.md)）。
- app（Web のページ）と同時には使えません。`/api/sport/request` の送り手が 2 つになるので、後から起動したほうが断ります。一度に起動した後なら、`docker compose stop app` で app だけ止めてから使います。偽の Go2（`docker compose --profile fake up -d sim`）や実機にも使えます。
- 箱を置く・どける（ページのボタンと同じ。`front`・`back`・`clear`）：

  ```bash
  docker compose run --rm app ros2 topic pub --once /sim/obstacles std_msgs/msg/String '{data: "{\"cmd\":\"front\"}"}'
  ```

  箱は Go2 の今の位置から 1.2 m 前（後ろ）に出ます。

**Gazebo で本当に動くものと、ふりのもの**

| 指令 | Gazebo での扱い |
|---|---|
| 立つ（StandUp → BalanceStand）・伏せる（StandDown） | 本当に立つ・伏せる |
| 前進・後退・旋回（Move・StopMove） | 本当に歩く・回る。前の箱は LiDAR で見て、体の中心から 0.5 m より近くなると止まる（確かめたときは 0.44〜0.46 m で止まった） |
| おすわり・立ち上がる（Sit・RiseSit） | ふり：伏せの姿勢で代用する（窓口の `error_code` はおすわりとして返す） |
| 挨拶・伸び・喜ぶ・ハート・おじぎ・ダンス | ふり：立ったまま、その動作の時間だけ待つ |

Gazebo のログには、ふりのとき「（Gazebo では伏せで代用）」「（Gazebo では再現なし）」と出ます。

**転んだとき**

- 立っているはずの Go2 が Gazebo の中で転ぶと（胴の |roll| か |pitch| が 1.0 rad を超えるか、胴の高さが 0.15 m 未満）、橋渡しは `error_code` を 1001（伏せ）にして、速度を 0 にします。Gazebo のログに「（Gazebo で転びました）」と出ます。アプリからは、Go2 が伏せたように見えます。
- 起き上がらせる仕組みはありません。転んだ後は StopMove のほかの指令を断るので、`docker compose --profile gazebo restart gazebo` で Gazebo を起動し直してください（Go2 は伏せた姿勢で出直します）。

**「立って」が終わったのに伏せたままのとき**

- 2026-11-11、Mac が重いとき（ほかの Gazebo が CPU を 310 % 使い、RTF 0.4〜0.6）に、「立って」がすぐ「終わりました」になり、その後の前進・旋回が「〜秒たっても終わりません」で止まりました。Go2 は伏せたまま（胴の高さ 0.15 m）で、`error_code` は 1013 でした。
- 原因：橋渡しは controller の状態を確かめず、見積もりの時刻で command を送っていました（2・2 の後、伏せの 1.8 秒 + 0.2 秒で 4）。controller の伏せ（fixed down）が見積もりより長くかかり、伏せが終わる前に届いた 4 が、残っていた 2 を上書きしました。伏せは 4 を無視するので、controller は伏せのまま動きません。
- 今の橋渡しは、controller が `/rosout` に出す `Switched from <a> to <b>` をすべて受け、確かめた状態からだけ次の command を送ります（脱力・伏せからは 2、立つからは 4）。状態に入った知らせから 2.0 秒（シミュレーションの時間）たってから送り、2.8 秒たっても切り替わりの知らせが来なければ送り直します（3 回まで）。
- controller が脱力・伏せの間は、立ち・歩きの `error_code`（1002・1013・1015）を返しません。Gazebo が遅くて（RTF がおよそ 0.5 より遅いと）立ち上がりがアプリの待ち時間に間に合わないと、アプリは「姿勢が計画と違います（計画: 立ち / 実際: 伏せ）」で止まります。橋渡しは立ち上がりを続けているので、もう一度「立って」と言ってください。
- Gazebo のログ（`docker compose logs gazebo`）には「controller の切り替え: fixed down → fixed stand」が出ます。待たせているときは「controller が fixed down のままなので、error_code は 1001 にしておきます…」、送り直すときは「…送り直します」、3 回でもだめなときは「…切り替わりません…。Gazebo を起動し直してください」と出ます。
- 前の橋渡しで伏せたまま止まった Gazebo は、`docker compose --profile gazebo restart gazebo` で起動し直せば直ります（controller が脱力から出直します）。

**回ると転ぶ件・揺れる件（controller のパッチ）**

そのままの unitree_guide controller（[legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control)）には、Gazebo の Go2 を転ばせる誤りがあり、ビルドのときに `docker/patches/` の 5 つのパッチを名前の順に当てて直しています（upstream は書き換えません）。

- 0001・0002：その場で 90 度回ると数秒で横に倒れていました。状態推定（Estimator）が胴の座標と世界の座標を混ぜていたのが原因です（記録は [docs/reference/gazebo_turn_fix.md](reference/gazebo_turn_fix.md)）。
- 0003：状態推定が、速度にかけたローパスの値をカルマンフィルタに書き戻していて（元の unitree_guide は書き戻しません）、推定の速さが約 0.14 秒遅れて小さく出ていました。0.22 m/s で歩き出すたびに胴が 0.6〜0.85 m/s まで出て、横の揺れ（roll）が育ち、歩き出し・旋回の終わり・角で転んでいました。
- 0004：歩くに入った最初の周期に足踏みを始めると、最長 0.225 秒、4 本の足がすべて浮くことがありました（立った後の最初の旋回で胴が落ちました）。4 本着いた状態から始めます。
- 0003・0004 の後、箱の一周は 2 組 10 回のうち 9 回転ばずに終わり（どちらの組も 3 回以上続けて）、旋回中の roll（95 %）は 0.23〜0.43 rad から 0.01〜0.05 rad に下がりました（記録は [docs/reference/gazebo_objects_e2e.md](reference/gazebo_objects_e2e.md) の「第 5 部」）。
- 0005：歩き方の周期（どの足を着き、どの足を振り出すか）を実際の時間で数えていたので、Gazebo が CPU 不足で止まると、止まった分だけ足の運びが先へ飛び、胴が跳ね上がって転ぶことがありました（0003・0004 の後の 10 回のうち 1 回）。周期を controller の時間（Gazebo ではシミュレーションの時間）で数えます。
  - Gazebo をわざと止めながら一周させると、0005 の前は 3 回とも転び、0005 の後は 3 回とも転びませんでした。0005 の後の組は一周 5/5・椅子 3/3 です（記録は同じファイルの「第 6 部」）。
- イメージは `go2-voice-ltl-gazebo:patch-0005` です。前のイメージは、作った Mac では残してあります：0001〜0004 の `go2-voice-ltl-gazebo:patch-0004`、0001・0002 の `go2-voice-ltl-gazebo:patch-0002`（前の `go2-voice-ltl-gazebo:dev` と同じイメージ）。`GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0004 SIM_KIND=gazebo docker compose --profile gazebo up -d`（または `GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0002 …`）で前のイメージで動きます。
  - **そのイメージがある Mac でだけ使えます。** このリポジトリの Dockerfile からは 0001〜0005 のイメージしか作れません。`GO2_GAZEBO_IMAGE` を付けたまま `docker compose --profile gazebo build` を流すと（イメージが無いときの `up` も）、0001〜0005 のイメージが付けたタグの名前で作られてしまいます。付けたまま `build` しないでください。
- Gazebo が CPU 不足で止まると、そのぶん動きが遅れます（0005 の後は、わざと止めた試験でも転びませんでした）。ほかのアプリ（音楽・写真の解析など）を止め、CPU に余裕を残すと速く進みます。

**Gazebo の結果は、実機の保証になりません**

脚を動かす中身が違います。実機の Go2 は本体の中の Unitree のソフトが歩かせますが、Gazebo では別のオープンソースの制御（unitree_guide controller）が歩かせます。窓口（トピック・`api_id`・`error_code`）は同じでも、歩き方・揺れ・止まるまでに進む距離・回る速さは違います。Gazebo で確かめられるのは、指示 → 式 → 監督器 → 指令の流れと、LiDAR で箱を見て止まることです。実機では [real_robot.md](real_robot.md) の手順で、立つ・止まるから確かめてください。
