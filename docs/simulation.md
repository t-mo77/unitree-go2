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

**物（箱と椅子）と上から見た図：** 偽の Go2 には、初めから箱（`box_1`、(2.5, 1.5)）と椅子（`chair_1`、(2.5, −1.5)）が置いてあります（`GO2_LAYOUT=default`）。Gazebo の既定は家具のある部屋（`household`、[家具の部屋](#家具の部屋go2_layout)）で、箱と椅子は同じ所にあります。Go2 の前のカメラに写った物が「物の一覧」になり、「箱の周りを一周して」のような指示に使えます（[物を使う指示](vocabulary.md#物を使う指示箱と椅子)）。偽の Go2 でも、`GO2_LAYOUT=household docker compose --profile fake up` で家具の部屋になります。

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
docker build -t go2-voice-ltl-gazebo:patch-0005 -f docker/Dockerfile.gazebo docker   # 初回だけ 1 つ目：Gazebo と controller（時間がかかります）
docker compose --profile gazebo build gazebo             # 初回だけ 2 つ目：その上に Fuel の家具を足す薄いイメージ go2-voice-ltl-gazebo:patch-0005-fuel2
SIM_KIND=gazebo docker compose --profile gazebo up -d    # Gazebo と app（Web のページ）を一度に起動
docker compose --profile gazebo down                     # 全部止める
```

- build は 2 段です。1 つ目の `go2-voice-ltl-gazebo:patch-0005`（controller のパッチ 0001〜0005。下の「回ると転ぶ件・揺れる件」）は作り直さず、2 つ目（`docker/Dockerfile.gazebo-fuel`）が、版と sha256 を決めた 4 つの Fuel のモデルをその上に足します（約 11 秒、層 2 つ）。Fuel が止まっているか中身が変わっていれば、sha256 が合わずに 2 つ目の build が止まります。
- 1 つ目がないまま 2 つ目を流すと、`go2-voice-ltl-gazebo:patch-0005` が見つからずに止まります。新しく clone した所では、上の順に 2 つとも流してください。

- app は、Gazebo の Go2 が用意できるまで最長 90 秒待ってから、ページを開きます（`SIM_KIND=gazebo` のときの既定。Gazebo が用意できるまでは、数秒から 40 秒ほど）。app のログ（`docker compose logs -f app`）に `ページ: http://localhost:8810` が出たら開けます。待つ秒数は `GO2_WAIT_READY_S` で変えられます（例：`GO2_WAIT_READY_S=150 SIM_KIND=gazebo docker compose --profile gazebo up -d`）。
- 90 秒待っても見つからなければ、app は「Go2 が見つかりません」で止まります（`docker compose ps` で app が出ていない）。Gazebo のログ（`docker compose logs gazebo`）を確かめてから、もう一度 `SIM_KIND=gazebo docker compose --profile gazebo up -d` を実行してください。
- 念のための別の方法：2 段に分けて起動することもできます。先に `docker compose --profile gazebo up -d gazebo` で Gazebo だけを起動し、Go2 と controller が用意できるまで 40 秒ほど待ってから、`SIM_KIND=gazebo docker compose --profile gazebo up -d` で app を起動します。
- 操作画面：<http://localhost:8810>（右上に「シミュレーション · Gazebo」と出ます）。話し方・箱のボタンは上と同じです。
- Gazebo の画面：<http://localhost:6080/vnc.html?autoconnect=1>（noVNC）。GPU が無いのでソフトウェアで描き、カクつきます。右下の % が実時間に対する速さ（real-time factor）です。この Mac では 0.74〜0.90 でした（`/clock` の進み ÷ 実際の時間。[gazebo_e2e.md](reference/gazebo_e2e.md)。箱と椅子だけの前の部屋の値）。
  家具の部屋（`household`、既定）の RTF は、測り直しました（前の記録は使わない。C4）：
  <!-- fill:H5: household の logical の RTF（GUI あり・なし、各 3 回の中央値と範囲。H5 の M0） -->
  <!-- fill:H6: detector のときの RTF（止まって・歩いて、GUI あり・なし）と、その前後の logical の基準（H6 の M0） -->
- app の標準エラーに `serdata.cpp:384` のエラーが出続けますが、データは届いています（Humble と Jazzy で、ノードの名前を伝える型が違うため）。
- `SIM_KIND=gazebo` は、ページの表示を「Gazebo」にするだけです。動かし方は偽の Go2 と同じです。

**物（`/go2/objects`）**

- Gazebo を起動すると、橋渡しが配置ファイル（`gazebo/layouts/<GO2_LAYOUT>.json`、既定 `household`）の物を置きます。どの部屋でも、箱 `box_1`（50 cm 角）は (2.5, 1.5)、椅子 `chair_1`（座面 45 cm、背もたれつき）は (2.5, −1.5) です。最初の向きから見て、箱は左前、椅子は右前です（odom の y は左向き）。ほかの物は[家具の部屋](#家具の部屋go2_layout)。
- Go2 の前のカメラ（Gazebo の logical camera。水平 120°・0.1〜8 m・5 Hz。画像は出さず、見えている物の名前と位置だけ）に写った物を、橋渡しが覚えて、2 Hz で `/go2/objects`（`std_msgs/String` の JSON、odom の座標）に出します（`OBJECTS_SOURCE=logical`、既定）。物の名前は単語帳（`voice_ltl/vocabulary.json`）の種類だけです。見えなくなった物も、Gazebo を起動し直すまで `visible: false` で残ります。
- 大きさは LiDAR の点で測ります。測るまでは単語帳のその種類の長い辺の正方形で `measured: false`、はじめて測ったら `measured: true` です。測る半径は種類ごとに max(0.6, 単語帳の大きさの対角の半分 + 0.3) m です（1.8 m のソファも小さく測らない）。
- 橋渡しは、どちらのときも、配置ファイルの本当の足跡を正解として `/gazebo/objects_truth` に出します（`objects_check.py` が比べる）。`/go2/objects` を出すのは、logical のときは橋渡し、detector のときは[検出器](#検出器で物を見つける)です。
- 物の一覧を見るには：

  ```bash
  docker compose run --rm --no-deps -T app ros2 topic echo --once --full-length /go2/objects
  ```

  `data:` のあとに、`{"frame": "odom", "stamp": …, "source": "logical", "objects": [{"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.523, 0.529], "seen": …, "visible": true, "measured": true}, …]}` の形の JSON が 1 通出ます。`size` は [x 方向の長さ, y 方向の長さ] [m] です。`source` は送り手（橋渡しは `logical`、検出器は `detector`、偽の Go2 は `fake`）で、検出器の行には `confidence` も付きます。`source`・`confidence`・`measured` は、画面にもプロンプトにも出ません。偽の Go2 の行には `measured` も `confidence` も付きません（前と同じ形）。`--full-length` を付けないと、`ros2 topic echo` は文字列を 128 文字で切ります（最後が `...`）。物 2 つでも 128 文字を超えるので、椅子 `chair_1` が出ません。
- 設計は [docs/superpowers/specs/2026-11-05-go2-objects-design.md](superpowers/specs/2026-11-05-go2-objects-design.md) と、種類を開いた [docs/superpowers/specs/2026-11-11-go2-open-objects-design.md](superpowers/specs/2026-11-11-go2-open-objects-design.md)、Gazebo で確かめた記録は [docs/reference/gazebo_objects_e2e.md](reference/gazebo_objects_e2e.md) と [docs/reference/gazebo_open_objects.md](reference/gazebo_open_objects.md) にあります。

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
- 今の橋渡しは、controller が `/rosout` に出す `Switched from <a> to <b>` をすべて受け、確かめた状態からだけ次の command を送ります（脱力・伏せからは 2、立つからは 4）。伏せ・立つに入った知らせからは 2.0 秒（シミュレーションの時間）たってから送ります（伏せ・立つは入って 1.8 秒は次へ行かないため）。2.8 秒たっても切り替わりの知らせが来なければ送り直します（待つ時間は送り直すたびに倍。送るのは合わせて 3 回まで。だめならあきらめてノートに書きます）。
- 知らせが遅れて届くときは、知らせに付いたログの時刻（`Log.stamp`）で、送り直した command がどの状態に届いたかを見て、そこから続けます。遅れを測った後は、送り直す前にその分も待ちます。StandUp などが来る前の切り替え（ほかの送り手）は、確かめた状態にするだけで何も送りません。
- controller が脱力・伏せの間は、立ち・歩きの `error_code`（1002・1013・1015）を返しません。Gazebo が遅くて（RTF がおよそ 0.5 より遅いと）立ち上がりがアプリの待ち時間に間に合わないと、アプリは「姿勢が計画と違います（計画: 立ち / 実際: 伏せ）」で止まります。橋渡しは立ち上がりを続けているので、もう一度「立って」と言ってください。
- Gazebo のログ（`docker compose logs gazebo`）には「controller の切り替え: fixed down → fixed stand」が出ます。待たせているときは「controller が fixed down のままなので、error_code は 1001 にしておきます…」、送り直すときは「…送り直します」、知らせが遅れているときは「command 2 は、controller が … に切り替わった後に届きました（/rosout の知らせが … 秒遅れています）…」、3 回でもだめなときは「…切り替わりません…。Gazebo を起動し直してください」、行き来が止まらないときは「…へ進まず、3 回戻りました…」と出ます。
- 前の橋渡しで伏せたまま止まった Gazebo は、`docker compose --profile gazebo restart gazebo` で起動し直せば直ります（controller が脱力から出直します）。

**回ると転ぶ件・揺れる件（controller のパッチ）**

そのままの unitree_guide controller（[legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control)）には、Gazebo の Go2 を転ばせる誤りがあり、ビルドのときに `docker/patches/` の 5 つのパッチを名前の順に当てて直しています（upstream は書き換えません）。

- 0001・0002：その場で 90 度回ると数秒で横に倒れていました。状態推定（Estimator）が胴の座標と世界の座標を混ぜていたのが原因です（記録は [docs/reference/gazebo_turn_fix.md](reference/gazebo_turn_fix.md)）。
- 0003：状態推定が、速度にかけたローパスの値をカルマンフィルタに書き戻していて（元の unitree_guide は書き戻しません）、推定の速さが約 0.14 秒遅れて小さく出ていました。0.22 m/s で歩き出すたびに胴が 0.6〜0.85 m/s まで出て、横の揺れ（roll）が育ち、歩き出し・旋回の終わり・角で転んでいました。
- 0004：歩くに入った最初の周期に足踏みを始めると、最長 0.225 秒、4 本の足がすべて浮くことがありました（立った後の最初の旋回で胴が落ちました）。4 本着いた状態から始めます。
- 0003・0004 の後、箱の一周は 2 組 10 回のうち 9 回転ばずに終わり（どちらの組も 3 回以上続けて）、旋回中の roll（95 %）は 0.23〜0.43 rad から 0.01〜0.05 rad に下がりました（記録は [docs/reference/gazebo_objects_e2e.md](reference/gazebo_objects_e2e.md) の「第 5 部」）。
- 0005：歩き方の周期（どの足を着き、どの足を振り出すか）を実際の時間で数えていたので、Gazebo が CPU 不足で止まると、止まった分だけ足の運びが先へ飛び、胴が跳ね上がって転ぶことがありました（0003・0004 の後の 10 回のうち 1 回）。周期を controller の時間（Gazebo ではシミュレーションの時間）で数えます。
  - Gazebo をわざと止めながら一周させると、0005 の前は 3 回とも転び、0005 の後は 3 回とも転びませんでした。0005 の後の組は一周 5/5・椅子 3/3 です（記録は同じファイルの「第 6 部」）。
- controller のイメージは `go2-voice-ltl-gazebo:patch-0005` で、compose が動かすのは、その上に家具を足した `go2-voice-ltl-gazebo:patch-0005-fuel2` です。前のイメージは、作った Mac では残してあります：最初の家具の部屋の `go2-voice-ltl-gazebo:patch-0005-fuel1`（TrashBin・LampAndStand。コーンはない）、0001〜0004 の `go2-voice-ltl-gazebo:patch-0004`、0001・0002 の `go2-voice-ltl-gazebo:patch-0002`（前の `go2-voice-ltl-gazebo:dev` と同じイメージ）。`GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0004 GO2_LAYOUT=default SIM_KIND=gazebo docker compose --profile gazebo up -d`（または `GO2_GAZEBO_IMAGE=go2-voice-ltl-gazebo:patch-0002 …`）で前のイメージで動きます。前のイメージには今の家具のモデルがないので、`GO2_LAYOUT=default` を付けます（付けないと、橋渡しが置く前に止まります）。
  - **そのイメージがある Mac でだけ使えます。** このリポジトリの Dockerfile からは、`patch-0005` と、その上の今の家具のイメージしか作れません。`GO2_GAZEBO_IMAGE` を付けたまま `docker compose --profile gazebo build` を流すと（イメージが無いときの `up` も）、今の家具のイメージが付けたタグの名前で作られてしまいます。付けたまま `build` しないでください。
- Gazebo が CPU 不足で止まると、そのぶん動きが遅れます（0005 の後は、わざと止めた試験でも転びませんでした）。ほかのアプリ（音楽・写真の解析など）を止め、CPU に余裕を残すと速く進みます。

**Gazebo の結果は、実機の保証になりません**

脚を動かす中身が違います。実機の Go2 は本体の中の Unitree のソフトが歩かせますが、Gazebo では別のオープンソースの制御（unitree_guide controller）が歩かせます。窓口（トピック・`api_id`・`error_code`）は同じでも、歩き方・揺れ・止まるまでに進む距離・回る速さは違います。Gazebo で確かめられるのは、指示 → 式 → 監督器 → 指令の流れと、LiDAR で箱を見て止まることです。実機では [real_robot.md](real_robot.md) の手順で、立つ・止まるから確かめてください。

## 家具の部屋（`GO2_LAYOUT`）

Gazebo の部屋は配置ファイル（`gazebo/layouts/<名前>.json`）で決まり、`GO2_LAYOUT` で選びます。

- `household`（Gazebo の既定）：箱・椅子と、Gazebo Fuel の家具 4 つ（[出典](#gazebo-の物の出典)）。
- `default`（偽の Go2 の既定）：箱と椅子だけの前の部屋。Gazebo では `GO2_LAYOUT=default SIM_KIND=gazebo docker compose --profile gazebo up -d`。
- 名前が違うと、橋渡しは何も置く前に止まり、正しい名前を 1 行で出します。イメージに家具のモデルのフォルダがないとき（前のイメージ `patch-0005-fuel1` には `construction_cone` がない）も、置く前に止まります。

| id | 物（モデル） | 位置（モデルの原点、m） | 大きさ（世界の x × y、m） | 検出器の目標 | 見る姿勢 |
|---|---|---|---|---|---|
| `box_1` | 箱（このリポジトリの簡単な形） | (2.5, 1.5) | 0.5 × 0.5 | いいえ | 始めの位置（左前） |
| `chair_1` | 椅子（このリポジトリの簡単な形） | (2.5, −1.5) | 0.45 × 0.45 | いいえ | 始めの位置（右前） |
| `suitcase_1` | スーツケース（Suitcase1。90° 回して長い辺を x に） | (−1.62, −1.14) | 0.468 × 0.291 | はい | Bf |
| `traffic_cone_1` | コーン（Construction Cone） | (−0.3, −4.45) | 0.503 × 0.503 | はい | R5 |
| `sofa_1` | ソファ（Sofa。座面は +x、始めの位置を向く） | (−4.6, 0) | 1.044 × 1.8 | はい | Bf |
| `table_1` | テーブル（Table） | (−2.4, 4.4) | 1.5 × 0.8 | いいえ（logical だけ） | L |

**まず回る。** 始めの位置のカメラ（前の 120°）に写るのは、今までどおり箱と椅子だけです（最初の物の一覧・プロンプト・「見えている物」は前の部屋と同じ）。ほかの物は、回って見てから使います。試験では、起動し直した Gazebo だけで（`docker compose --profile gazebo up -d gazebo`。`run_formula` は app と同時には使えない）次の順に流しました。ページからなら「立って」「左に2回まわって」「前に進んで」と言っても同じです：

```bash
SIM_KIND=gazebo docker compose run --rm --no-deps -T app python3 -m voice_ltl.run_formula "F standing"
SIM_KIND=gazebo docker compose run --rm --no-deps -T app python3 -m voice_ltl.run_formula "F(turned_left & X F turned_left)"   # Bf：左に 2 回
SIM_KIND=gazebo docker compose run --rm --no-deps -T app python3 -m voice_ltl.run_formula "F moved_forward"                   # Bf：1 歩前へ
```

- **Bf**（ソファとスーツケース）：立つ、左に 2 回、1 歩前へ。**R5**（コーン）：立つ、右に 1 回、前へ 5 歩（`F(moved_forward & X F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward))))` の 1 つの式）。**L**（テーブル、logical だけ）：左に 1 回。
- Gazebo の旋回は少し回りすぎます。1 回で約 ±96°、2 回で約 191.5°（Bf の向きは −166.6〜−172.4°、R5 は −95.1〜−97.6° でした）。部屋はこの測った向きで置いてあります。旋回の controller を変えたら、見え方の確かめ（`tests/test_household.py` の `test_no_undetectable_model_in_a_detector_test_view`）を流し直してください。ちょうど 180° の Bf では、テーブルの角が検出の見え方に入ります。
- 回った後は止まって、目標が「見えている物」に出るまで待ってから指示します。logical のときは、論理カメラが見て、次の止まったスキャンで大きさを測ります。測る前の大きさ（単語帳の長い辺の正方形）では、測った Bf からのソファの周回は計画がありません（238 マス）。
- 一周や「〜まで」の決まりは、どの物でも物の縁から測ります（[vocabulary.md](vocabulary.md#どの種類の物でも単語帳の-50-種類)）。偽の Go2 のテスト（`tests/test_sim_household.py`）では、4 つの家具の「〜まで」と一周が、Bf・R5・L から設計書の線（手前は縁まで 1.5 m 以内・足跡に 45° 以内・どの物にも 0.30 m 以上、一周は巻き数 0.9 以上 1.5 未満・縁まで 0.6 m 以上）を満たします。
- Gazebo（logical）での結果：
  <!-- fill:H5: sofa_1・suitcase_1・traffic_cone_1・table_1 の F near / F around 各 2/2 の結果と数、ソファの手前で前の物のビットが立ったか（O14）、回の本当の姿勢 -->
- 物の設計書 §2 の箱と椅子の受け入れ（一周・椅子まで・挨拶つき・箱の前まで）は、始めの位置から流すので、家具があってもなくても計画（PLANT）が同じです（`tests/test_household.py`）。家具の部屋で測り直した結果：
  <!-- fill:H5: §2 の 1 組目（一周 5 回・椅子 3 回・挨拶つき 1 回・箱の前まで 1 回）の結果 -->
  <!-- fill:H6: §2 の 2 組目の結果 -->

## 検出器で物を見つける

`OBJECTS_SOURCE=detector` のとき、`/go2/objects` は論理カメラではなく、検出器（compose の `detector`、別の container）が出します。検出器は、Gazebo が描く前のカメラの絵（640 × 480・2 Hz・水平 120°、論理カメラと同じ場所）を YOLOE-11s（ultralytics 8.4.210、文字で探す名前を渡す形、CPU で 1 スレッド）に見せ、見つけた物の位置と大きさを LiDAR の点で測ります。論理カメラは、どちらのときも正解（`/gazebo/objects_truth`）として残ります。

```bash
docker compose --profile detector build detector         # 初回だけ。イメージ go2-voice-ltl-detector:yoloe-11s（この Mac で 700.6 秒、4.51 GB）
OBJECTS_SOURCE=detector SIM_KIND=gazebo docker compose --profile gazebo --profile detector up -d
docker compose --profile gazebo --profile detector down  # 全部止める
```

- `--profile detector` を付けないと検出器が起動せず、物の指示はどれも「見つかりません」になります。そのとき Gazebo のログには、10 秒ごとに「…検出器は --profile detector で起動します…」が出ます。`OBJECTS_SOURCE=detector` を付けずに検出器だけを起動すると、検出器はわけを 1 行書いてすぐ終わります。
- **止まっている間だけ見ます。** Go2 が止まって（回る速さ 0.1 rad/s 以下・進む速さ 0.05 m/s 以下が 0.3 秒）から絵を見て、止まりの中の 3 枚のうち 2 枚（0.5 秒以上離れ、1 枚は LiDAR の点が支え、1 枚は確からしさ 0.35 以上）で見えたら、一覧に出します。設計の見込みは止まってから約 3.5 秒（最悪 4.6 秒）です。測った時間：
  <!-- fill:H6: 止まってから一覧に出るまで（listed_after_still_s）と、推論の中央値・p90（latency_ms_p50・p90） -->
  だから、回った後は止まって、目標が「見えている物」に出るまで待ってから指示します。歩いている間は推論しません。
- **ページには何も足しません。** 検出器の状態・`source`・`confidence` は、画面にもプロンプトにも出ません。一覧の物は、logical のときと同じく単語帳の名前で「見えている物」に出ます。検出器の状態は `/go2/detector_status`（1 Hz の JSON）で見ます：`docker compose run --rm --no-deps -T app ros2 topic echo --once --full-length /go2/detector_status`。
- 検出器に渡す言葉は、単語帳の `detect` の英語 87 と、見つけても捨てる 3 つ（person・dog・cat）です。文字の埋め込みは名前つきの volume `detector-cache` に取っておくので、はじめの起動だけ符号化器に 6〜9 秒かかり（90 の言葉。H2）、次からは読むだけです。単語帳の `detect` を変えると、次の起動で 1 回作り直します（イメージは作り直さない）。
- **Gazebo を起動し直したら、検出器も起動し直してください**（`docker compose --profile gazebo --profile detector restart gazebo detector`）。検出器は、シミュレーションの時間が戻ったのを見ると、止まりの見張り・位置の記録・覚えた物を消して始め直しますが、id は 1 から付け直すので、前の id は別の物を指しえます。転んだ後も同じです。

**見つかるもの（数は [gazebo_open_objects.md](reference/gazebo_open_objects.md)）**

| 目標 | 見る姿勢 | 下見（YOLOE-11s・640、本物の絵。姿勢の見え方と 4 つのずらし） | Gazebo の detector のとき |
|---|---|---|---|
| ソファ `sofa_1` | Bf（約 4.0 m） | 5/5（sofa 0.775〜0.840） | <!-- fill:H6: sofa_1 の F near / F around 3/3 と声の段 --> |
| スーツケース `suitcase_1` | Bf（約 1.5 m） | 姿勢の見え方と 3/4（suitcase 0.488〜0.665。Bf_s4 では trash_can 0.517） | <!-- fill:H6: suitcase_1 の F near / F around 3/3 と声の段 --> |
| コーン `traffic_cone_1` | R5（約 1.7 m） | 5/5（traffic_cone 0.634〜0.913。正解の枠との IoU 0.51〜0.57） | <!-- fill:H6: traffic_cone_1 の F near / F around 3/3 と声の段 --> |

- 下見の線は「姿勢の見え方と、4 つのずらしのうち 2 つ以上で、正しい名前が 0.35 以上（正解の枠と IoU 0.5 以上）」です。新しい種類で 3 つが要り、3 つちょうどで通りました（目指した 4 つには届かなかった）。推論 1 回はこの Mac で中央値 325〜388 ms（H3）。
- 薄いところ：コーンの枠は影を含み、正解の枠との重なりが線（0.5）に近い。スーツケースは回り方でカメラから +36〜+47° に動き、左に 5° ずらした見え方（Bf_s2）では絵の左の端で切れる。Bf_s4 ではスーツケースが trash_can と呼ばれる。
- 部屋を直した後のイメージ（`patch-0005-fuel2`）での下見のやり直し（H3b）：
  <!-- fill:H3b: patch-0005-fuel2 での下見の結果（preview の合否、目標ごとの確からしさ、推論の中央値） -->
- 検出の正しさ（`objects_check.py accuracy`。位置・大きさ・名前・recall・誤検出）：
  <!-- fill:H6: M3 の結果（oracle と yoloe。始めの位置・立った後・Bf・R5） -->

**見つからないもの・できないこと**

- ただの形の箱（box_1）と椅子（chair_1）は、検出器ではまず見つかりません。detector のときの目標にしていないので、物の設計書 §2 の箱と椅子の合格は logical のときに測ります（Q4）。試しの絵では箱 0/5、椅子 0/6 でした。下見では、伏せた始めの位置の絵で椅子が chair 0.729・箱が box 0.302、立った始めの位置で chair 0.517 と呼ばれたこともありますが、当てにはできません。模様のある箱・椅子にするのは、後の仕事です。
  <!-- fill:H6: detector のときに box_1・chair_1 が一覧に出たか（M3・M4 の間の記録） -->
- **ゴミ箱とランプは試して、部屋から出しました。** どこに置いても、自分の名前で確かにならなかったためです。検出できない物が検出の見え方にあると、名前の正しさと大きさの線（設計書 §13.3）が組のすべてで数えるので落ちます（R-H3c）。
  - ゴミ箱（TrashBin）：前から光の当たる向きで 30 の見え方のうち 0（box・air_purifier・humidifier と呼ばれた）、後ろから光で 2/30。最初の部屋の B・Bf では shelf・cabinet（0.50〜0.60）と呼ばれ、ゴミ箱の言葉は 0.31 まで（1 枚だけ 0.569）。代わりの FIRST 2015 trash can は Bf の場所でだけ通り（余裕 +0.010）、その場所はスーツケースが取った（今のモデルを先にする決まり）。R5 の場所では 0/5。LitterBin は R5 から 1.5 m の所に置けなかった。
  - ランプ（LampAndStand）：台が desk（0.43〜0.72）と呼ばれ、lamp が 0.35 を超えたのは 3.5 m と 4.5 m で 5 つの方位のうち 1 つずつだけ。代わりのモデルはない。
  - だからゴミ箱とランプの種類は、検出器の試験の目標から外れました。単語帳の種類としては残り、logical のときや偽の Go2 では今までどおり使えます。
- テーブル（Table）は試しの絵で 1 度も見つからなかった（いちばん高い確からしさ 0.18）ので、logical のときだけの目標です。
- **名前は間違えることがあります。** 最初の部屋の B（4.4 m）では、ソファが 5 枚のうち 2 枚で chair と呼ばれました（今の Bf の約 4.0 m では、下見の 5 枚ともソファ）。間違った名前の物は、言われた種類として扱いません（コードは名前を読み替えない）。そのときは「「ソファ」が見つかりません。…」と正直に断るので、「見えている物」に出た名前で言い直してください。
- **足跡が本当の物より小さくなりうる。** 1 つの見え方では 1〜2 面しか見えないので、見えない奥行きは単語帳の大きさで補います。ただ、手前の別の物に奥の端が隠れた L 字（2 面が見えている）の見え方では、見えた奥行きのままにするので、ソファで約 0.63 L（L は単語帳の長い辺）まで小さい足跡になりえます。マス目はその足跡だけを塞ぎます。LiDAR の前の 0.5 m で止まる決まりは、どの物にも今までどおり効きます。
- 画面のボタンで置く箱（「前に箱を置く」）は、detector のときだけ段ボールの見た目になり、箱と呼ばれれば一覧に入ります（実機と同じ）。約束はしません（試しの段ボール箱は 2/5）。
  <!-- fill:H6: M5 の記録（立った Go2 の前のボタンの箱が box として一覧に出たか） -->
- 実機の道（PR #4 の Linux 直接実行と `compose.robot.yaml`）は、この仕事で変えていません。検出器も家具の部屋も Gazebo だけのもので、実機の Go2 では今までどおり物の一覧は来ません（物の設計書 §11 の窓口のまま）。

### 検出器のライセンス

- 検出器の container は、ultralytics と YOLOE の重みと CLIP の tokenizer（ultralytics/CLIP の fork）を、AGPL-3.0 で使います。
- 文字の符号化器の重み `mobileclip_blt.ts`（Apple の MobileCLIP の text encoder）は、Apple の条件で研究のため（Research Purposes）だけに使えて、商用・製品づくりには使えません（research-only, non-commercial）。利用者は研究に使うことを承知しています（O24）。用途に合うかは使う人が決めてください（法律の判断ではありません）。
- ultralytics と torch は検出器のイメージと `detector/yoloe.py` にだけあり、app のイメージにも Gazebo のイメージにも、本来の Linux の準備にも入りません。検出器は取り替え口（`detector/protocol.py`）の後ろにあります。
- 代わり（同じ取り替え口の後ろに置ける）：符号化器の要らない prompt-free の YOLOE（`yoloe-11s-seg-pf.pt`。これも AGPL-3.0）、AGPL を避けるなら OWLv2 / OWL-ViT や Grounding DINO（Apache-2.0。速さはまだ測っていません）。
- 重みは、build のときに ultralytics/assets の v8.4.0 から URL を決めて取り、sha256 が違えば build が止まります（`docker/Dockerfile.detector`）。イメージはこの Mac で作るだけで、配っていません。
- リポジトリに LICENSE ファイルを足すかどうかは、利用者の別の判断です（この仕事では足していません）。

## Gazebo の物の出典

Gazebo の部屋（`household`）に置く物のうち、次のモデルは Gazebo Fuel（https://fuel.gazebosim.org、持ち主はどれも OpenRobotics）から、イメージを作るときに版と sha256 を決めて取ってきたものです（`docker/fuel_models.json`）。ファイルは変えず、名前・位置・向き・static だけを指定して置いています。

| モデル | 作者 | 部屋の id | Fuel の URL | ライセンス |
|---|---|---|---|---|
| 「Sofa」版 3 | Wan Yi Seow（Open Robotics） | `sofa_1` | https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sofa/3 | CC BY 4.0（https://creativecommons.org/licenses/by/4.0/） |
| 「Table」版 4 | Nate Koenig（Open Robotics） | `table_1` | https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table/4 | CC0 1.0（https://creativecommons.org/publicdomain/zero/1.0/。表示の義務はありませんが記します） |
| 「Suitcase1」版 2 | Wan Yi Seow（Open Robotics） | `suitcase_1` | https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1/2 | CC0 1.0（https://creativecommons.org/publicdomain/zero/1.0/。表示の義務はありませんが記します） |
| 「Construction Cone」版 3 | Cole Biesemeyer、Nate Koenig（Open Robotics） | `traffic_cone_1` | https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Cone/3 | CC0 1.0（https://creativecommons.org/publicdomain/zero/1.0/。表示の義務はありませんが記します） |

- ライセンスは Fuel のページ（Fuel の情報の `license_name`）にだけ書かれています。zip の中の `model.config` にはありません。作者は `model.config` のものです。
- Construction Cone は、Fuel の名前に空白があるので、URL は `Construction%20Cone`、フォルダは `construction_cone` です（`model.sdf` の中の `model://construction_cone`）。
- 箱（box_1）と椅子（chair_1）は、このリポジトリのコードが作る簡単な形です。
- 英語の 1 行の形（NOTICE やイメージの label 用）：
  - `"Sofa" (version 3) by Wan Yi Seow, Open Robotics — https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sofa/3 — CC BY 4.0 — unmodified.`
  - `"Table" (version 4) by Nate Koenig, Open Robotics — https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table/4 — CC0 1.0 — unmodified.`
  - `"Suitcase1" (version 2) by Wan Yi Seow, Open Robotics — https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1/2 — CC0 1.0 — unmodified.`
  - `"Construction Cone" (version 3) by Cole Biesemeyer and Nate Koenig, Open Robotics — https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Cone/3 — CC0 1.0 — unmodified.`
- **部屋から出したモデル：** TrashBin（版 2、作者 Roselle Carmen（Open Robotics）、https://fuel.gazebosim.org/1.0/OpenRobotics/models/TrashBin/2 、CC BY 4.0）と LampAndStand（版 3、作者 Roselle Carmen（Open Robotics）、https://fuel.gazebosim.org/1.0/OpenRobotics/models/LampAndStand/3 、CC BY 4.0）は、最初の部屋にあり、検出器で見つからないので出しました（[上](#検出器で物を見つける)）。今の部屋とイメージにはありません。前のイメージ `go2-voice-ltl-gazebo:patch-0005-fuel1`（この Mac にだけ残してある）には、この 2 つも入っています。
- 部屋を選び直す下見では、FIRST 2015 trash can、LitterBin、Suitcase2、Backpack も Fuel から取って試しました（作業用の場所だけ。どのイメージにも入れていない）。JanSport Backpack Red は、`model.sdf` が別の Fuel のモデルを URL で読み込むので使っていません。
- 描いた絵（CC BY のモデルが写る）は、リポジトリに入れていません。
