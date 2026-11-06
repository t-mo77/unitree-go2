# 声で指示 → LTL → 監督制御 → Go2

声で話しかけると、Claude（または Codex）が指示を LTL の式に翻訳し、オートマトンと監督制御が「安全に実行できる動作の列」を計算して、Unitree Go2（今はシミュレーション）を動かします。設計は [docs/design.md](docs/design.md) を見てください。

## 1. これは何か

Cay 先生の講義「自然言語の指示を、安全が保証されたロボットの行動に変える」の考え方を、Go2 で動く形にしたものです（設計書 §1）。

- LLM に任せるのは「言葉 → 形式仕様（LTL）」の翻訳だけです。
- 正しさと安全は、オートマトンと監督制御の計算で守ります。式が変わっても、プログラムは書き換えません。
- 先に用意するのは「Go2 の状態遷移図」と「ロジックの単語帳・文法」の 2 つです。この範囲内なら任意の指示ができます。
- 最終的には、このフォルダごと Go2 と有線でつないだ Linux PC に移して、実機を動かします（§6）。

```
 声 ─▶ 書き起こし ─▶ LLM が LTL に翻訳 ─▶ check()（文法と単語帳）
       (Whisper)       (Claude / Codex)        │ 却下なら最大 3 回やり直し
                                               ▼
   Go2 ◀─ 計画の実行 ◀─ 監督器（SPEC×PLANT→TRIMMED→SUP）
 (SIM / 実機)  止まって = LLM を通さず即 StopMove
```

## 2. 準備（Mac）

1. Docker Desktop を入れて起動します。
2. 設定ファイルを作ります。

   ```bash
   cp env.txt.example env.txt
   ```

   `env.txt` は git に入れません。人にも渡しません。
3. LLM にログインします。使うほうだけで構いません（`env.txt` の `LLM_BACKEND=claude` か `codex`）。
   - **Claude：** Mac のターミナルで `claude setup-token` を実行します。出てきた長期トークンを、自分で `env.txt` の `CLAUDE_CODE_OAUTH_TOKEN=` のあとに書きます。トークンをチャットや README、コミットに貼ってはいけません。
   - **Codex：** コンテナの中でログインします。コンテナにはブラウザが無く、ログインの戻り先（コンテナの中の localhost）にも届かないので、端末に出るコードを使う方法（`--device-auth`）にします。

     ```bash
     docker compose run --rm --no-deps app codex login --device-auth
     ```

     出てきた URL を Mac のブラウザで開き、端末に出たコードを入れます。ログインは Docker のボリューム `codex-home`（コンテナの `~/.codex`）に残るので、1 回で済みます。うまくいかないときは OpenAI の公式ドキュメントを見てください。
     API キーで使うときは、ログインの代わりに `env.txt` に `CODEX_API_KEY=` の行を足して、キーを自分で書きます（`codex exec` が読みます）。
     **Codex の実地での動作は未確認です。**Claude の構造化出力の形（`structured_output`）はコンテナの中で確認済みです。
4. イメージをビルドします。

   ```bash
   docker compose build
   ```

> **注意：`env.txt` があるフォルダで `docker compose config` を実行しないでください。** `env.txt` の中身（トークン）が画面に出ます。設定の文法だけ確かめたいときは、出力を捨てます：
> `docker compose -f compose.robot.yaml config > /dev/null`
> コンテナの中で `env` や `printenv` を実行するのも同じ理由で避けます。

> **コンテナの中の `/work/env.txt` は空です。** compose が `env.txt` を読んで環境変数として渡し、コンテナの中のファイルは `/dev/null` で覆います（`compose.yaml`・`compose.robot.yaml`）。翻訳役の LLM の CLI は、`/work` の外の空のフォルダで動き、もう一方の LLM のトークンは受け取りません。Codex にはシェル・画像の読み込み・Web 検索を使わせません（Claude は道具なし）。
> `env.txt` を作る前に compose を使うと、覆うための空の `env.txt` ができます。そのときは `cp env.txt.example env.txt` で上書きします（Linux で root の持ち物になって上書きできなければ、`sudo rm env.txt` してから作り直します）。

### 書き起こしのモデル

`env.txt` の `WHISPER_MODEL` は既定で `small` です。この Mac（CPU）では、1 秒前後に近いのは `small` だけでした。`medium` と `large-v3-turbo` は遅すぎます（比較は [docs/reference/whisper_models.md](docs/reference/whisper_models.md)）。GPU のある PC では `medium` などに書き換えて試せます。

## 3. 使い方

```bash
docker compose --profile fake up      # アプリと、偽の Go2（sim）を起動
```

1. ブラウザで <http://localhost:8810> を開きます。ポートを変えたいときは `GO2_UI_PORT` を渡します（例：`GO2_UI_PORT=8811 docker compose --profile fake up`）。シェルの環境変数でも、compose.yaml と同じ場所に置いた `.env` の `GO2_UI_PORT=8811` でも構いません。ホスト側は 127.0.0.1 だけに公開します。
2. 「マイクを使う」を押して許可します。
3. **スペースを押している間**話して、離します。画面に書き起こし、Claude の式、`check()` の合否、状態遷移図、Go2 の状態、上から見た図が出ます。言い換えや近い動作に置き換えたとき（「走って」→ 前へ歩く など）は、式の下の「こう受け取りました」に出ます。
4. 「前に箱を置く」「後ろに箱を置く」「箱を全部どける」ボタンで、障害物を試せます（シミュレーションのときだけ表示されます）。

**物（箱と椅子）と上から見た図：** 偽の Go2 にも Gazebo にも、初めから箱（`box_1`、(2.5, 1.5)）と椅子（`chair_1`、(2.5, −1.5)）が置いてあります。Go2 の前のカメラに写った物が「物の一覧」になり、「箱の周りを一周して」のような指示に使えます（§4）。

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

### Gazebo で動かす

物理シミュレーション（Gazebo Harmonic）の中の Go2 を、実機と同じ窓口（`/api/sport/request`・`/sportmodestate`・`/utlidar/cloud`）で動かします（設計書 §9.1）。app は今までどおり ROS 2 Humble、gazebo のコンテナは ROS 2 Jazzy です。確かめた記録は [docs/reference/gazebo_e2e.md](docs/reference/gazebo_e2e.md) にあります。

**起動と止め方**

```bash
docker compose --profile gazebo build gazebo             # 初回だけ（Gazebo と controller のビルドで時間がかかります）
SIM_KIND=gazebo docker compose --profile gazebo up -d    # Gazebo と app（Web のページ）を一度に起動
docker compose --profile gazebo down                     # 全部止める
```

- app は、Gazebo の Go2 が用意できるまで最長 90 秒待ってから、ページを開きます（`SIM_KIND=gazebo` のときの既定。Gazebo が用意できるまでは、数秒から 40 秒ほど）。app のログ（`docker compose logs -f app`）に `ページ: http://localhost:8810` が出たら開けます。待つ秒数は `GO2_WAIT_READY_S` で変えられます（例：`GO2_WAIT_READY_S=150 SIM_KIND=gazebo docker compose --profile gazebo up -d`）。
- 90 秒待っても見つからなければ、app は「Go2 が見つかりません」で止まります（`docker compose ps` で app が出ていない）。Gazebo のログ（`docker compose logs gazebo`）を確かめてから、もう一度 `SIM_KIND=gazebo docker compose --profile gazebo up -d` を実行してください。
- 念のための別の方法：2 段に分けて起動することもできます。先に `docker compose --profile gazebo up -d gazebo` で Gazebo だけを起動し、Go2 と controller が用意できるまで 40 秒ほど待ってから、`SIM_KIND=gazebo docker compose --profile gazebo up -d` で app を起動します。
- 操作画面：<http://localhost:8810>（右上に「シミュレーション · Gazebo」と出ます）。話し方・箱のボタンは上と同じです。
- Gazebo の画面：<http://localhost:6080/vnc.html?autoconnect=1>（noVNC）。GPU が無いのでソフトウェアで描き、カクつきます。右下の % が実時間に対する速さ（real-time factor）です。この Mac では 0.74〜0.90 でした（`/clock` の進み ÷ 実際の時間。[gazebo_e2e.md](docs/reference/gazebo_e2e.md)）。
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
- 設計は [docs/superpowers/specs/2026-11-05-go2-objects-design.md](docs/superpowers/specs/2026-11-05-go2-objects-design.md)、Gazebo で確かめた記録は [docs/reference/gazebo_objects_e2e.md](docs/reference/gazebo_objects_e2e.md) にあります。

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
- 設計書（物）の §2 の合格の条件（回転数・物の縁までの距離・最後の位置と向き・傾き）を測るときは、`gazebo/tools/objects_check.py` で `run_formula` を動かします（使い方はファイルの先頭。記録は [gazebo_objects_e2e.md](docs/reference/gazebo_objects_e2e.md)）。
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

**回ると転ぶ件**

そのままの unitree_guide controller（[legubiao/quadruped_ros2_control](https://github.com/legubiao/quadruped_ros2_control)）では、その場で 90 度回ると数秒で横に倒れていました。controller の状態推定（Estimator）が胴の座標と世界の座標を混ぜていたのが原因で、ビルドのときに `docker/patches/` の 2 つのパッチを当てて直しています（記録は [docs/reference/gazebo_turn_fix.md](docs/reference/gazebo_turn_fix.md)）。

それでも、歩いて止めた直後に次の動作（前進・旋回）を始めると、横の揺れ（roll）が 0.5〜0.7 rad まで育つことがあります。物を使わない指示で確かめた範囲では転んでいません。物を使う指示は、0.5 m・90 度ごとに止まっては動いていた間はこの揺れの中でよく転びました（箱の一周は 31 回のうち 1 回しか終えていません）。動作の間を止まらずにつなぐようにしてからは転ぶのが減りましたが、まだ転ぶことがあります（箱の一周は 3 回のうち 2 回終え、1 回は途中で転びました。§8）。

**Gazebo の結果は、実機の保証になりません**

脚を動かす中身が違います。実機の Go2 は本体の中の Unitree のソフトが歩かせますが、Gazebo では別のオープンソースの制御（unitree_guide controller）が歩かせます。窓口（トピック・`api_id`・`error_code`）は同じでも、歩き方・揺れ・止まるまでに進む距離・回る速さは違います。Gazebo で確かめられるのは、指示 → 式 → 監督器 → 指令の流れと、LiDAR で箱を見て止まることです。実機では §6 の手順で、立つ・止まるから確かめてください。

## 4. 話せること

使える言葉は単語帳（命題）で決まっています。

| 種類 | 命題（`voice_ltl/world.py`） |
|---|---|
| 姿勢 | `standing`（立っている）、`sitting`（おすわり）、`lying`（伏せ） |
| 動作をした | `hello`（挨拶）、`stretched`（伸び）、`content`（喜ぶ）、`heart`（ハート）、`scraped`（おじぎ）、`danced`（ダンス）、`moved_forward`、`moved_backward`、`turned_left`、`turned_right` |
| センサー | `obstacle_front`（前に障害物）、`obstacle_back`（後ろに障害物） |

前進・後退は 0.5 m、旋回は 90 度が 1 回です。数は繰り返しで表します（1 動作につき 12 回まで）。

設計書 §7.4 の例：

| 話す言葉 | 期待する式（例） | 期待する結果 |
|---|---|---|
| 立って挨拶して | `F hello` | 伏せからなら stand → hello |
| 挨拶してからダンスして | `F(hello & F danced)` | stand → hello → dance |
| 挨拶するまで座らないで、そのあと座って | `(!sitting U hello) & F sitting` | stand → hello → sit |
| ダンスはしないで、伸びをして | `G !danced & F stretched` | stand → stretch |
| 前に進んで | `F moved_forward` | stand → forward。前方 0.5 m 以内に箱があれば待つ。どけたら進む |
| 2メートル進んで（前方 1.2 m に箱） | 前進 4 回の式 | 箱に 0.5 m まで近づいたら止まる。どけたら続きを歩く |
| 後ろに下がって（後方に箱） | `F moved_backward` | 下がらずに待つ。どけたら下がる |
| しゃがんで / 寝そべって | `F sitting` / `F lying`（同じ動作の言い換え） | stand → sit / lie_down。言い換えは「こう受け取りました」に出す |
| その場で一周して | 左旋回 4 回の式 | 旋回 4 回 |
| キッチンへ行って / ジャンプして | （単語帳にない） | 「単語帳にない」と伝えて動かない |
| 座って、でも絶対に座らないで | `F sitting & G !sitting` | 満たせないと伝えて動かない |
| 止まって | （LLM を通さない） | すぐ StopMove |
| 1メートル進んで / 180度回って / 3回挨拶して | 繰り返し | forward 2 回 / 旋回 2 回 / hello 3 回 |
| 0.7メートル進んで / 45度回って | いちばん近い回数 | forward 1 回 / 旋回 1 回。「こう受け取りました」に「0.7メートル → 0.5 m」などと出す |
| 走って | `F moved_forward`（近い動作で代わり） | forward 1 回。「こう受け取りました」に「走って → 前へ歩く」と出す |
| 回れ右 / 20回挨拶して | 右旋回 2 回 / 挨拶 12 回（上限） | 置き換えと上限を「こう受け取りました」に出す |
| 前に何かあるまで進んで / 握手して | （断る） | 動かずに伝える |
| 0メートル進んで / 這って進んで / こっちに来て | （断る） | 動かずに伝える |

### 物を使う指示（箱と椅子）

物の一覧（`/go2/objects`）にある物は、物ごとの命題で式にします。id は `box_1`・`chair_1` のように「ラベル_番号」です。プロンプトには、Go2 から近い順に 8 個まで、向きと距離をつけて載ります（例：「`box_1`（箱）：左前 2.9 m」）。設計は [docs/superpowers/specs/2026-11-05-go2-objects-design.md](docs/superpowers/specs/2026-11-05-go2-objects-design.md) です。

| 命題（物ごと） | 意味 |
|---|---|
| `near_<id>` | その物の手前のマスにいて、物の方を向いている（物の中心から「物の半径 + 0.75 m」〜「min(物の半径 + 1.5 m, 1.3 m)」、向きの差 40° 以内） |
| `around_<id>` | その物の周りを左回りに 1 周し終えた（物から 1 m ほど離れた輪を歩き、入った所を過ぎてから一周とする） |
| `around_cw_<id>` | その物の周りを右回りに 1 周し終えた |

| 話す言葉 | 期待する式（例） | 期待する結果 |
|---|---|---|
| 箱の周りを一周して | `F around_box_1` | 伏せからなら stand → 箱の周りを左回りに 1 周。「こう受け取りました」に「周りを一周 → 左回りに 1 周」と出す。Gazebo ではまだ毎回は通らない（3 回のうち 2 回。1 回は途中で転んだ。§8） |
| 箱の周りを右回りで回って | `F around_cw_box_1` | 右回りに 1 周 |
| 箱を2周して | `F around_box_1` | 1 周だけ。「こう受け取りました」に「2周 → 左回りに 1 周（上限）」と出す |
| 椅子まで歩いて | `F near_chair_1` | stand → 椅子の手前まで歩き、椅子の方を向いて止まる |
| 立って、椅子まで歩いて、挨拶して | `F(near_chair_1 & X F hello)` | stand → 椅子の手前まで歩く → hello |
| 右の箱まで行って（箱が 2 つあるとき） | `F near_box_2`（右前にある箱） | 「## Objects」の向きで 1 つに決める |
| 障害物の周りを回って | 前にある物が 1 つなら、その物の `F around_<id>` | 前にある物が 2 つ以上（既定の置き方）なら、単語帳にないと伝えて動かない |
| 箱を持って | （断る） | 単語帳にないと伝えて動かない（持つ・押す・運ぶはしない） |

- 表の「期待する結果」は、偽の Go2 で確かめたものです。**Gazebo では、設計書（物）§2 の合格の条件をまだ満たしていません。** 今のコードで、一周は 3 回のうち 2 回終え、1 回は途中で転びました。椅子までは 3 回とも、「立って、椅子まで歩いて、挨拶して」も 1 回流して合格です（§8）。
- 回るのは 1 周までです。「2周して」は 1 周にして「（上限）」と出します。
- 使える物は、Go2 から 6 m までです。
- 道は 0.5 m のマス目の上で計画し、どの物のマスにも入りません（物の中心から「物の半径 + 0.45 m」以内のマス）。歩くときは 1 手ごとに、Go2 の位置をマスの中心と向きに合わせ直します。
- 物を使う指示の途中で、一覧にない物（「前に箱を置く」の箱など）が前に来ると、Go2 は止まって待ちます。どければ続きを歩き、15 秒たっても進めなければ止めます（「15秒待っても進めません」など）。LiDAR で前後に物が見えている間は、物を使う指示の計画は座る・伏せるもしません（座って立ち直すと、物が消えたことになってしまうため）。
- 物を使わない指示は、今までどおり 12 状態の状態遷移図で動きます。
- 実際の Claude で言い方を変えて試した結果は [docs/reference/object_phrasing_trials.md](docs/reference/object_phrasing_trials.md) にあります（2 回とも、箱の周りを回る 10 通り、椅子まで行く 10 通りが全部正しかった）。

断るとき・止めるときのメッセージ：

| メッセージ | どんなとき |
|---|---|
| 「箱」が見つかりません。Go2 の前に置いてから、もう一度言ってください。 | 言われた物が一覧にない。Gazebo を起動してすぐは一覧がまだ届いていないことがあるので、少し待ってから、もう一度言います |
| 「箱」が 2 つあるので、どれか決められません。「左の箱」のように言ってください。 | 同じ名前の物が 2 つ以上あって、どれか言われていない |
| 「箱」が遠すぎます（6 m まで）。 | 言われた物が Go2 から 6 m より遠い |
| 物どうしが離れすぎていて、一度には計画できません | Go2 と言われた物を含むマス目が 256 マスを超える |
| 位置がずれたので止めました。 | 1 手の後の Go2 が、いちばん近いマスの中心から 0.3 m より離れているか、向きが 30° よりずれている。計画と違うマスに着いて、そこからの続きがないときも |
| この指示は満たせません（物のまわりに通れる道がありません） | 物の手前や周りのマスに行けない（大きすぎる物など） |
| Go2 の位置がわからないので、物を使う指示はできません | Go2 の位置（`/sportmodestate`）が届いていない |

## 5. 単語帳と状態遷移図の足し方

新しい動作を足すときに書き換えるのは、次の場所です。1〜3 のどれかが足りないと、計画や実行、画面の表示でエラーになります。

1. **動作の中身：** `nl_control/go2_nl_control/actions.py` の `ACTIONS` にあるものを使います。ここに、送る `api_id` と送った後に待つ秒数（`sequence`）、できる姿勢と移った先（`transitions`）、距離・角度で動く動作なら `param` と `velocity` があります。`api_id` は公式 `sport_api.py` と同じ番号です。宙返り・逆立ち・二足歩行・ジャンプなど危ない動作は足しません。
2. **`voice_ltl/world.py`：**
   - `ACTION_NAMES` に名前を 1 行。
   - `ACTION_PROP` に、その動作をしたときに真になる命題（姿勢を変えるだけの動作なら `None`）。
   - `PROP_DESCRIPTION_JA` に、その命題の日本語の説明を 1 行。これがそのまま LLM への単語帳の説明になります。
   - 距離・角度で動く動作なら、`step_value()` に 1 回分の値（距離なら `STEP_DISTANCE`、角度なら `STEP_ANGLE`）。足さないと 0 になり、動きません。
3. **画面の名前：** `ACTION_JA`（画面と記録に出す、動作の日本語の名前）に 1 行。今は `voice_ltl/session.py` にあります（`grep -rn "ACTION_JA =" voice_ltl` で場所がわかります）。
4. **シミュレーション：** 新しい `api_id` のジェスチャーなら、にせ Go2（`sim/sim_core.py`）と Gazebo の橋渡し（`gazebo/bridge_core.py`）の `GESTURES` にも足します。足さないと、シミュレーションの Go2 はその指令を無視します。
5. **テスト：** 命題・状態・遷移の数を確かめるテスト（`tests/test_world.py`）の数を直します。

- 前進・後退のほかに体が動く動作（例：`actions.py` の横歩き `left`・`right`）は、その向きの障害物を見張りません。LiDAR の判定は前方と後方だけです。
- 式が変わるだけで、meet → trim → 監督器（supcon）→ 計画は自動で計算されます。足したらテスト（§7）を流してください。

## 6. Linux PC へ移す（設計書 §10）

移す先は画面つきの Ubuntu PC（22.04 想定）です。その PC のブラウザとマイクで操作します（`http://localhost:8810` で開くので、`localhost` のままマイクの許可が使えます）。

1. **フォルダをコピー**します（`env.txt` も自分でコピー、または作り直します）。
2. **有線の設定：** PC の有線ポートを `192.168.123.x` に固定します。ただし Go2 本体が使う `192.168.123.161` は使いません（例：`192.168.123.99/24`）。
3. **有線と Wi-Fi の両方を使う：** Go2 とは有線、LLM を呼ぶインターネットは Wi-Fi など別の経路にします。有線側にデフォルトゲートウェイを付けないでください。
4. **つながったか確かめる：**

   ```bash
   ping 192.168.123.161
   ```
5. **`env.txt` の `GO2_IFACE`** に、有線の差し口の名前（`ip a` で確認。例：`enp3s0`）を書きます。

最初に歩かせる前に、手順 6〜8 で実機の値を確かめます。どれもサーバーを起動する前に、同じイメージのコンテナを 1 回だけ起動して実行します。Go2 には何も送りません。

6. **実機とつながったか確かめる：**

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic echo /sportmodestate
   ```

   位置と姿勢の値が流れてくれば、実機とつながっています。Ctrl+C で終わります。
7. **姿勢の値（`error_code`）を確かめる（必須）：** このプログラムは `/sportmodestate` の `error_code` で Go2 の姿勢を知ります。対応は `nl_control/go2_nl_control/actions.py` の `_ERROR_CODE_TO_STATE` で、公式の表から作ったものです（実機では未確認）。リモコンで Go2 を「立つ → 座る → 立つ → 伏せる」の順にして、そのたびに値を見ます。

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
8. **LiDAR を確かめる（必須）：** 障害物で止まる・待つのは、LiDAR の点群を胴の座標に直した判定だけが頼りです。その直し方（`PerceptionConfig` の `lidar_x`・`lidar_z`・`lidar_pitch`）は仮の値なので、実機で確かめます。Go2 をリモコンで立たせ、前と後ろを 1 m 以上空けてから、次を実行します。

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

     この値は Unitree の Go2 のモデル（`go2_description`）の LiDAR の取り付け（頭の下に上下逆さ）です（[docs/reference/gazebo_feasibility_jazzy.md](docs/reference/gazebo_feasibility_jazzy.md)）。既定値で合わなければ、まずこれを試してください。何も無いのに「あり」で床が映っているなら、`z_min` を上げます。
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
   - **rviz2 で点群を見たいとき：** このイメージには rviz2 が入っていないので、コンテナの中では動きません。Linux PC に ROS 2 Humble が入っていれば、`sudo apt install ros-humble-rviz2 ros-humble-rmw-cyclonedds-cpp` で入れて、別の手段として使えます。コンテナと同じ設定（`docker/unitree_setup.sh` と同じ）にしてから起動し、Fixed Frame を `utlidar_lidar` にして `/utlidar/cloud` を PointCloud2 で表示します。

     ```bash
     source /opt/ros/humble/setup.bash
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp3s0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
     rviz2
     ```

     `enp3s0` は `env.txt` の `GO2_IFACE` と同じ名前にします。

   `sensor_check` は `/api/sport/request` の送り手として見えるので、動かしたままサーバーを起動すると「別のプログラム」として断られます。終わってから手順 9 に進みます。
9. **起動：**

   ```bash
   docker compose -f compose.robot.yaml up
   ```

   `network_mode: host`、`SIM=0` です。サーバーは 127.0.0.1 だけで待ち受け（ポートは既定で 8810。変えるときは `env.txt` に `GO2_UI_PORT=<ポート>` を書きます。`compose.robot.yaml` では `.env` やシェルの変数は使いません）、箱のボタンは出ません。偽の Go2 は起動しません。

### 安全の注意

- 前方 1 m と周りを空けてください。
- リモコンを持った人が、必ずそばにいてください。
- 最初は「立って」「止まって」だけで動作を確かめます。

## 7. テスト

```bash
docker compose run --rm --no-deps app python3 -m pytest -q -m "not slow" tests nl_control/tests
```

`run --rm` は、使い捨てのコンテナを 1 つ起動してコマンドを実行し、終わると消します。

- `-m "not slow"` は、遅いテストを除きます。普段はこれで足ります。
- 遅いテスト（`slow`）は、本物の Whisper のモデルを使います。初回はモデルを落とすので時間がかかります。流すときは `-m "not slow"` を外すか、`-m slow tests` にします。

## 8. わかっていないこと

- **Codex：** 実地で動かして確かめていません（`codex login --device-auth` と構造化出力）。道具を外す設定（`voice_ltl/llm.py`）は、ログインせずに `codex debug prompt-input` で確かめただけです。
- **実機の LiDAR：** 取り付けの値（`lidar_x`・`lidar_z`・`lidar_pitch`）と、前方・後方の物が見えるかは未確認です（§6 の手順 8）。
- **実機の `error_code`：** 姿勢と `error_code` の対応、ジェスチャーの間や後の値、ジェスチャーの待ち時間（`actions.py`）が実機に合うかは未確認です（§6 の手順 7）。
- **Whisper の正確さ：** 合成音声の「立って」は 3 モデルとも正しく書き起こせませんでした。本物の声と、実機の PC での精度は未確認です。
- **実機のカメラ（YOLOE）：** 物の一覧を出すのは、今は Gazebo の logical camera と偽の Go2 だけです。実機では、Go2 の前のカメラの画像を YOLOE に通し、LiDAR の点で位置と大きさを測るノードが、同じ `/go2/objects` を出すのが次の一歩です（物の設計書 §11）。まだ作っていません。アプリ側は変えずに使える形にしてあります。
- **Gazebo での物を使う指示（設計書（物）§2 の合格の条件）：まだ満たせていません（一周が 3 回のうち 2 回）。** 記録は [docs/reference/gazebo_objects_e2e.md](docs/reference/gazebo_objects_e2e.md) の「第 3 部」です。
  - 今のコード（commit `c2641ce`）で、1 回ごとに Gazebo を起動し直して `objects_check.py` で流しました。箱の一周は 3 回のうち 2 回終えました（巻き数 1.040・1.036、箱の縁まで 0.758・0.828 m、傾き 0.82・0.84 rad）。残る 1 回は 5 動作目（旋回からつないだ前進の歩き出し）で転びました。椅子までは 3/3（椅子まで 1.14〜1.37 m）、「立って、椅子まで歩いて、挨拶して」も 1/1 です。RTF は 1.0 でした。
  - 実行器は、格子の計画の動作の間を止まらずにつなぎます：続く前進と旋回の後の前進（M1）、前進の後の旋回（角）は StopMove なしでつなぎ、止めるのは計画の終わり（と、前の物・センサーの途切れで止まるとき）だけです。偽の Go2 の一周では、StopMove が 42 回から 1 回になりました（設計書（物）§6.5）。
  - その前（0.5 m・90 度ごとに StopMove で止まっては動く形と、曲がる前だけ止める M1）では、一周は最後まで流した 31 回のうち 1 回しか終えられませんでした（M1 の 3 回は 0/3）。止まってから動き出す所（歩き出し・回り始め）で多く転びました。椅子までは 16 回のうち 12 回合格でした。
  - 転ぶのは Gazebo の unitree_guide controller です。止まらずにつないでも、旋回の終わりや歩いている途中で横の揺れが育って倒れることがあります（今のコードで 108 動作のうち 1 回）。翻訳・監督器・計画は、転ぶまで計画どおりに動きました（「位置がずれたので止めました」は 0 回）。偽の Go2 では、一周も椅子までも通ります。
  - 残り（controller にパッチを当ててイメージを作り直す、回数を増やして測り直す、この段階では §2 を偽の Go2 だけで合格とする、など）をどうするかは、まだ決めていません。今のコードでは、操作画面（本物の Claude）からの一周と、一周を終えた所からの 2 つ目の指示は、Gazebo で流し直していません。
- **物を使う指示の実機での動き：** マス目に合わせて歩く利き（`voice_ltl/grid_executor.py`）は、偽の Go2 と Gazebo でしか動かしていません（Gazebo では上のとおり、一周は 3 回のうち 2 回終え、1 回は転びました）。実機は止めた後の惰性や回る速さが違うので、「位置がずれたので止めました」が出やすいかもしれません。
