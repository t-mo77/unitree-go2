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
   - **Codex：** `docker compose run --rm app codex login`（詳しくは OpenAI の公式ドキュメント）。**Codex の実地での動作は未確認です。**Claude の構造化出力の形（`structured_output`）はコンテナの中で確認済みです。
4. イメージをビルドします。

   ```bash
   docker compose build
   ```

> **注意：`env.txt` があるフォルダで `docker compose config` を実行しないでください。** `env.txt` の中身（トークン）が画面に出ます。設定の文法だけ確かめたいときは、出力を捨てます：
> `docker compose -f compose.robot.yaml config > /dev/null`
> コンテナの中で `env` や `printenv` を実行するのも同じ理由で避けます。

### 書き起こしのモデル

`env.txt` の `WHISPER_MODEL` は既定で `small` です。この Mac（CPU）では、1 秒前後に近いのは `small` だけでした。`medium` と `large-v3-turbo` は遅すぎます（比較は [docs/reference/whisper_models.md](docs/reference/whisper_models.md)）。GPU のある PC では `medium` などに書き換えて試せます。

## 3. 使い方

```bash
docker compose --profile fake up      # アプリと、偽の Go2（sim）を起動
```

1. ブラウザで <http://localhost:8810> を開きます。ポートを変えたいときは `GO2_UI_PORT` を渡します（例：`GO2_UI_PORT=8811 docker compose --profile fake up`）。シェルの環境変数でも、compose.yaml と同じ場所に置いた `.env` の `GO2_UI_PORT=8811` でも構いません。ホスト側は 127.0.0.1 だけに公開します。
2. 「マイクを使う」を押して許可します。
3. **スペースを押している間**話して、離します。画面に書き起こし、Claude の式、`check()` の合否、状態遷移図、Go2 の状態、上から見た図が出ます。
4. 「前に箱を置く」「後ろに箱を置く」「箱を全部どける」ボタンで、障害物を試せます（シミュレーションのときだけ表示されます）。

止め方は 3 つあります。

- 「止まって」と言う（LLM を通さず、すぐ StopMove）
- 画面の「止める」ボタン
- `docker compose --profile fake down`（全部止める）

`--profile fake` を付けずに `docker compose up` だけだと、偽の Go2 は起動しません。

### Gazebo で動かす（Task 17 で書き足す）

（Task 17 で、`docker compose --profile gazebo up`、<http://localhost:6080>、Gazebo でできること・できないこと（設計書 §9.1 の表）をここに書きます。）

## 4. 話せること

使える言葉は単語帳（命題）で決まっています。

| 種類 | 命題（`voice_ltl/world.py`） |
|---|---|
| 姿勢 | `standing`（立っている）、`sitting`（おすわり）、`lying`（伏せ） |
| 動作をした | `hello`（挨拶）、`stretched`（伸び）、`content`（喜ぶ）、`heart`（ハート）、`scraped`（おじぎ）、`danced`（ダンス）、`moved_forward`、`moved_backward`、`turned_left`、`turned_right` |
| センサー | `obstacle_front`（前に障害物）、`obstacle_back`（後ろに障害物） |

前進・後退は 0.5 m、旋回は 90 度が 1 回です。数は繰り返しで表します（1 動作につき 6 回まで）。

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
| キッチンへ行って | （単語帳にない） | 3 回却下され、「単語帳にない言葉」と伝えて動かない |
| 座って、でも絶対に座らないで | `F sitting & G !sitting` | 満たせないと伝えて動かない |
| 止まって | （LLM を通さない） | すぐ StopMove |
| 1メートル進んで / 180度回って / 3回挨拶して | 繰り返し | forward 2 回 / 旋回 2 回 / hello 3 回 |
| 0.7メートル進んで | 割り切れない数 | 「この数は指定できません」と伝えて動かない |

## 5. 単語帳と状態遷移図の足し方

新しい動作を足すときに書き換えるのは、`voice_ltl/world.py` の次の 2 か所です。

1. `ACTION_NAMES`（と、命題になる動作なら `ACTION_PROP`）に 1 行。動作そのものは `nl_control` の `actions.py` にあるものを使います。
2. `PROP_DESCRIPTION_JA` に、その命題の日本語の説明を 1 行。これがそのまま LLM への単語帳の説明になります。

式が変わるだけで、sync → trim → 監督器 → 計画は自動で計算されます。足したらテスト（§7）を流してください。

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
6. **起動：**

   ```bash
   docker compose -f compose.robot.yaml up
   ```

   `network_mode: host`、`SIM=0` です。サーバーは 127.0.0.1 だけで待ち受け（ポートは既定で 8810。変えるときは `env.txt` に `GO2_UI_PORT=<ポート>` を書きます。`compose.robot.yaml` では `.env` やシェルの変数は使いません）、箱のボタンは出ません。偽の Go2 は起動しません。
7. **実機の状態を確かめる**（別の端末で、同じイメージのコンテナを 1 回だけ起動して実行）：

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic echo /sportmodestate
   ```

   位置と姿勢の値が流れてくれば、実機とつながっています。
8. **後方の LiDAR を確かめる（必須）：** 後退の安全は後ろの LiDAR が見えていることが前提です。`rviz2` で `/utlidar/cloud` を表示し、Go2 の後ろに物を置いて、点群に映るか見ます。映るなら `backward` はそのまま使えます。**映らない場合は、`voice_ltl/world.py` の次の 3 行を消してください**（後退ができなくなり、「後ろに下がって」は単語帳にない言葉として却下されます）。

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

- **Codex：** 実地で動かして確かめていません（`codex login` と構造化出力）。
- **実機での後方の検知：** 実機の LiDAR で後方の物が見えるかは未確認です（§6 の手順 8）。
- **Whisper の正確さ：** 合成音声の「立って」は 3 モデルとも正しく書き起こせませんでした。本物の声と、実機の PC での精度は未確認です。
