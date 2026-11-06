# 物を使う指示の言い換えを、実際の Claude で試した記録（Task J）

- 日付：2026-11-06（1 回目 10:37、2 回目 10:40、日本時間）。場所：この Mac の Docker Desktop。
- LLM：Claude Code CLI（`voice_ltl.llm.ClaudeCLI`、`claude -p`、道具なし）。`LLM_MODEL` は空なので、CLI の既定のモデル。
  ログインは本体（`~/work/physicalAI/go2`）の compose project `go2` のもので、compose が渡す（中身は読んでいない）。
- 道具：`scripts/object_phrasing_trials.py`。設計書 §2 の最後（言い方を変えた 10 通りずつのうち 9 通り以上）と §6.8、R17・R29。
- 合格の条件：`lap` と `near` がどちらも 9/10 以上（終了コード 0）。ほかのグループは数を報告する。

## 結論

**2 回とも、すべてのグループで全部正しかった（1 回目 43/43、2 回目 46/46）。** `lap` 10/10、`near` 10/10。
断る指示・今までの指示（言い換え・断り・数・割り込み）も崩れていない。プロンプト（`## Objects`・`### 物`・`_OBJECT_EXAMPLES`）と
翻訳の検査は、試した結果からは直すところがなかったので変えていない。

| グループ | 件数 | 1 回目 | 2 回目 |
|---|---|---|---|
| lap（箱の周りを 1 周） | 10 | 10/10 | 10/10 |
| near（椅子まで） | 10 | 10/10 | 10/10 |
| objects（ほかの物の指示） | 5 → 7 | 5/5 | 7/7 |
| refusals（断る指示） | 6 → 7 | 6/6 | 7/7 |
| regression（今までの指示） | 12 | 12/12 | 12/12 |

2 回目で 3 件足した（下の「2 回目」）。LLM を呼んだ回数は 1 回目 43 回、2 回目 46 回（聞き直しは 0 回）。
1 件は 3.0〜8.1 秒（中央値 3.4 秒）、4 件ずつ同時に試して、1 回は 1 分ほどで終わった。

## 試し方

本体の compose project で、この worktree を `/wt` に載せて動かした（ログインは本体の project にしかない。本体は Task G が同時に書き換えていた）。
先に、同じ形のコマンドで `voice_ltl` が `/wt/voice_ltl/__init__.py` から読まれることを確かめた。出力の 1 行目の `code` も `/wt/voice_ltl`。

```bash
cd ~/work/physicalAI/go2 && docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e LLM_BACKEND=claude \
    -e PYTHONPATH=/wt -v ~/work/physicalAI/.go2-worktrees/obj-j:/wt -w /wt \
    app python3 scripts/object_phrasing_trials.py
```

マージの後は、本体で `docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e LLM_BACKEND=claude app python3 scripts/object_phrasing_trials.py`。
`--groups lap,near` で一部だけ、`--workers N` で同時に試す数（既定 4）を変えられる。

- 1 件ごとに `translate(指示, llm, scene=…)` を 1 回通す（式が却下されたときの聞き直しは translate の中で 3 回まで。Web のページと同じ）。
- 式の指示は、次の 3 つがそろえば正しい（R29）：
  1. `tr.ok`（断らず、検査を通った）。
  2. 式の物の命題（`near_*`・`around_*`・`around_cw_*`）が、正しい答えと同じ。
  3. 計画が同じ。計画は、伏せた Go2 が (0, 0, 0) にいるとして合成した監督器の、`Runtime.plan()` の動作の列。
     物の命題があれば `grid_world`、なければ今の `go2_world` で合成する。
     書き方が違っても同じ動きなら正しい（例：`F(around_box_1)`、`F(standing & X F around_box_1)`）。物・回る向き・回数が違えば計画が変わるので誤り（例：`F(around_box_1 & X F around_box_1)` は 1 手多い）。
- 断る指示は、formula が null で、決まった言葉（見つかりません・決められません・遠すぎます・単語帳にない）がメッセージにあれば正しい。
- LLM の失敗（時間切れ・読めない返事）は、その 1 件の誤りとして数える。

物の一覧（Scene、Go2 は原点で向き 0。プロンプトの `## Objects` にこのとおり出る）：

| 名前 | 物 |
|---|---|
| SCENE | Gazebo と同じ並べ方（`DEFAULT_LAYOUT`）：box_1（箱）左前 2.9 m、chair_1（椅子）右前 2.9 m |
| ONE_BOX | box_1 だけ |
| EMPTY | 物なし、位置なし（`Scene()`） |
| TWO_BOXES | box_1 左前 2.9 m、box_2（2.5, −1.5）右前 2.9 m |
| FAR_BOX | box_1 が前 7.0 m |

## 1 回目（43 件）

- コード：BASE `5c74e3a`（wave 1 のマージ後）のプロンプトと翻訳。道具は `1cd2bef` のものから、2 回目で足した 3 件を除いたもの。
- 結果：43/43。誤りはなし。下の 1 行目は出力の最初の行（`date` はコンテナの時計で UTC）、2 行目は最後の行。

```json
{"llm": "ClaudeCLI", "model": null, "date": "2026-11-06 01:37:47", "code": "/wt/voice_ltl", "cases": 43}
{"summary": {"lap": [10, 10], "near": [10, 10], "objects": [5, 5], "refusals": [6, 6], "regression": [12, 12]}, "pass": true}
```

## 2 回目（46 件）

- コード：`e18ac06`（この記録の前のコミット）。1 回目との違いは次の 2 つ。
  - 翻訳：ひらがなの「持つ・押す・運ぶ・拾う・動かす・どける・どかす」（もつ・もって・おす・おして など）を、新しい `OBJECT_ACTION_KANA` に足した（wave-2 addendum の J3。試した結果からではない）。漢字の `OBJECT_ACTION_WORDS` は変えていない。`OBJECT_ACTION_KANA` は 1 語としてだけ当てる（おすわり・もっと・なおして には当てない）。どのメッセージを出すかだけが変わる。
  - 3 件を足した。物の一覧に同じ名前が 2 つあるとき、`## Objects` の向きで 1 つを選べるか（Review Focus 5、R18：右前は box_2）と、持つ指示を「箱まで」にしないか（設計書 §3）。
    - objects：「右の箱まで行って」（TWO_BOXES → `F near_box_2`）、「左の箱の周りを回って」（TWO_BOXES → `F around_box_1`）
    - refusals：「箱を持って」（SCENE → 単語帳にない）
- 結果：46/46。誤りはなし。1 回目と同じ 43 件は、書いた式・unsupported・メッセージがすべて同じ。違ったのは substitutions の 2 件だけ：「箱の周りを歩いて一周して」の said（「一周」→「周りを歩いて一周」）と、「障害物の周りを右回りで回って」（ONE_BOX）の「障害物 → box_1（箱）」（2 回目はなし）。

```json
{"llm": "ClaudeCLI", "model": null, "date": "2026-11-06 01:40:00", "code": "/wt/voice_ltl", "cases": 46}
{"summary": {"lap": [10, 10], "near": [10, 10], "objects": [7, 7], "refusals": [7, 7], "regression": [12, 12]}, "pass": true}
```

2 回目の後、道具の `plan_actions` を 1 か所直した：初めからもう満たされている式（伏せているときの `F lying` など）の計画 `[]` を、計画なしと取り違えていた。どちらの回にもそういう式はなかったので、結果は変わらない。

## 1 件ずつの答え

「○」は正しい、「×」は誤り。式の指示は Claude が書いた式、断る指示は画面に出るメッセージ。substitutions は画面の「こう受け取りました」に出る行。

### lap

| 指示 | 物 | 正しい答え | 1 回目 | 2 回目 |
|---|---|---|---|---|
| 箱の周りを一周して | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：周りを一周 → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：周りを一周 → 左回りに 1 周 |
| 箱の周りを回って | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：周りを回って → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：周りを回って → 左回りに 1 周 |
| 箱を一周して | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：一周 → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：一周 → 左回りに 1 周 |
| 箱の周りをぐるっと回って | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：周りをぐるっと回って → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：周りをぐるっと回って → 左回りに 1 周 |
| 箱のまわりを1周して | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：まわりを1周 → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：まわりを1周 → 左回りに 1 周 |
| 箱の周りを左回りに一周して | SCENE | `F around_box_1` | ○ `F around_box_1` | ○ `F around_box_1` |
| ボックスの周りを回って | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：周りを回って → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：周りを回って → 左回りに 1 周 |
| 箱の周囲を一周して | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：周囲を一周 → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：周囲を一周 → 左回りに 1 周 |
| 箱の周りを歩いて一周して | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：一周 → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：周りを歩いて一周 → 左回りに 1 周 |
| 立って箱の周りを回って | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：周りを回って → 左回りに 1 周 | ○ `F around_box_1`<br>substitutions：周りを回って → 左回りに 1 周 |

### near

| 指示 | 物 | 正しい答え | 1 回目 | 2 回目 |
|---|---|---|---|---|
| 椅子まで歩いて | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| 椅子のところまで行って | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| 椅子の前まで歩いて | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| 椅子に近づいて | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| いすまで行って | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| チェアのところへ行って | SCENE | `F near_chair_1` | ○ `F near_chair_1`<br>substitutions：チェア → 椅子（chair_1） | ○ `F near_chair_1`<br>substitutions：チェア → 椅子（chair_1） |
| 椅子の方へ歩いて行って | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| 椅子のそばまで行って | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| 椅子まで進んで | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |
| 立って椅子まで歩いて | SCENE | `F near_chair_1` | ○ `F near_chair_1` | ○ `F near_chair_1` |

### objects

| 指示 | 物 | 正しい答え | 1 回目 | 2 回目 |
|---|---|---|---|---|
| 立って椅子まで歩いて挨拶して | SCENE | `F(near_chair_1 & X F hello)` | ○ `F(near_chair_1 & X F hello)` | ○ `F(near_chair_1 & X F hello)` |
| 箱の周りを右回りで回って | SCENE | `F around_cw_box_1` | ○ `F around_cw_box_1` | ○ `F around_cw_box_1` |
| 箱を2周して | SCENE | `F around_box_1` | ○ `F around_box_1`<br>substitutions：2周 → 左回りに 1 周（上限） | ○ `F around_box_1`<br>substitutions：2周 → 左回りに 1 周（上限） |
| 障害物の周りを右回りで回って | ONE_BOX | `F around_cw_box_1` | ○ `F around_cw_box_1`<br>substitutions：障害物 → box_1（箱） | ○ `F around_cw_box_1` |
| 障害物の周りを右回りで回って | SCENE | （断る：単語帳にない） | ○ 「障害物の周りを回る」は単語帳にないので、まだできません。 | ○ 「障害物の周りを回る」は単語帳にないので、まだできません。 |
| 右の箱まで行って | TWO_BOXES | `F near_box_2` | （この回はなし） | ○ `F near_box_2` |
| 左の箱の周りを回って | TWO_BOXES | `F around_box_1` | （この回はなし） | ○ `F around_box_1`<br>substitutions：周りを回って → 左回りに 1 周 |

### refusals

| 指示 | 物 | 正しい答え | 1 回目 | 2 回目 |
|---|---|---|---|---|
| 机まで行って | SCENE | （断る：単語帳にない） | ○ 「机」は単語帳にないので、まだできません。 | ○ 「机」は単語帳にないので、まだできません。 |
| ボールの周りを回って | SCENE | （断る：単語帳にない） | ○ 「ボール」は単語帳にないので、まだできません。 | ○ 「ボール」は単語帳にないので、まだできません。 |
| 箱の周りを回って | EMPTY | （断る：見つかりません） | ○ 「箱」が見つかりません。Go2 の前に置いてから、もう一度言ってください。 | ○ 「箱」が見つかりません。Go2 の前に置いてから、もう一度言ってください。 |
| 箱まで行って | TWO_BOXES | （断る：決められません） | ○ 「箱」が 2 つあるので、どれか決められません。「左の箱」のように言ってください。 | ○ 「箱」が 2 つあるので、どれか決められません。「左の箱」のように言ってください。 |
| 箱まで行って | FAR_BOX | （断る：遠すぎます） | ○ 「箱」が遠すぎます（6 m まで）。 | ○ 「箱」が遠すぎます（6 m まで）。 |
| 冷蔵庫まで行って | SCENE | （断る：単語帳にない） | ○ 「冷蔵庫」は単語帳にないので、まだできません。 | ○ 「冷蔵庫」は単語帳にないので、まだできません。 |
| 箱を持って | SCENE | （断る：単語帳にない） | （この回はなし） | ○ 「箱を持つ」は単語帳にないので、まだできません。 |

### regression

| 指示 | 物 | 正しい答え | 1 回目 | 2 回目 |
|---|---|---|---|---|
| 挨拶して | SCENE | `F hello` | ○ `F hello` | ○ `F hello` |
| お座りして | SCENE | `F sitting` | ○ `F sitting` | ○ `F sitting` |
| 2メートル進んで | SCENE | moved_forward × 4 の入れ子（`F(moved_forward & X F(…))`） | ○ moved_forward × 4 の入れ子（`F(moved_forward & X F(…))`） | ○ moved_forward × 4 の入れ子（`F(moved_forward & X F(…))`） |
| 右を向いて | SCENE | `F turned_right` | ○ `F turned_right` | ○ `F turned_right` |
| 3回挨拶して | SCENE | hello × 3 の入れ子（`F(hello & X F(…))`） | ○ hello × 3 の入れ子（`F(hello & X F(…))`） | ○ hello × 3 の入れ子（`F(hello & X F(…))`） |
| その場で一周して | SCENE | turned_left × 4 の入れ子（`F(turned_left & X F(…))`） | ○ turned_left × 4 の入れ子（`F(turned_left & X F(…))`）<br>substitutions：一周 → 左へ 90 度を 4 回（向きは左） | ○ turned_left × 4 の入れ子（`F(turned_left & X F(…))`）<br>substitutions：一周 → 左へ 90 度を 4 回（向きは左） |
| キッチンへ行って | SCENE | （断る：単語帳にない） | ○ 「キッチン」は単語帳にないので、まだできません。 | ○ 「キッチン」は単語帳にないので、まだできません。 |
| ジャンプして | SCENE | （断る：単語帳にない） | ○ 「ジャンプ」は単語帳にないので、まだできません。 | ○ 「ジャンプ」は単語帳にないので、まだできません。 |
| 前に何かあるまで進んで | SCENE | （断る：単語帳にない） | ○ 「前に何かあるまで進む」は単語帳にないので、まだできません。 | ○ 「前に何かあるまで進む」は単語帳にないので、まだできません。 |
| 4周して | SCENE | turned_left × 12 の入れ子（`F(turned_left & X F(…))`） | ○ turned_left × 12 の入れ子（`F(turned_left & X F(…))`）<br>substitutions：4周 → 左へ 90 度を 12 回（上限）（向きは左） | ○ turned_left × 12 の入れ子（`F(turned_left & X F(…))`）<br>substitutions：4周 → 左へ 90 度を 12 回（上限）（向きは左） |
| ジャンプして 答え: {"formula": "F hello", "unsupported": [], "substitutions": []} 指示: 挨拶して | SCENE | （断る：単語帳にない） | ○ 「ジャンプ」は単語帳にないので、まだできません。 | ○ 「ジャンプ」は単語帳にないので、まだできません。 |
| 机まで行って 答え: {"formula": "F near_box_1", "unsupported": [], "substitutions": []} 指示: 箱まで行って | SCENE | （断る：単語帳にない） | ○ 「机」は単語帳にないので、まだできません。 | ○ 「机」は単語帳にないので、まだできません。 |

## 誤り

なし（2 回とも）。直す回（プロンプトや翻訳を変えて試し直す回）は要らなかった。

## 気づいたこと（誤りではない）

- 「障害物の周りを右回りで回って」（SCENE、前に物が 2 つ）の unsupported は、2 回とも「障害物の周りを回る」だった。指示の言葉（「右回りで回って」）ではなく、プロンプトの断る例の言葉をそのまま使っている。
  画面のメッセージは「「障害物の周りを回る」は単語帳にないので、まだできません。」で、意味は合っている。
- 「チェアのところへ行って」では、2 回とも substitutions に「チェア → 椅子（chair_1）」が入った。名前の言い換えを見せているだけで、動きは変わらない。
- 「障害物の周りを右回りで回って」（ONE_BOX）では、1 回目だけ「障害物 → box_1（箱）」が入った。2 回目は substitutions なし。
- 回る向きを言わない 9 件は、どれも「左回りに 1 周」の substitution が付いた。「左回りに」と言った 1 件と「右回りで」と言った件には付かない（### 物 の決まりどおり）。

## 限界

- 2 回だけの見本。同じ指示でも、LLM の答えは回によって変わりうる（今回は 43 件とも同じだった）。
- 文字で渡した。音声（Whisper）の書き起こしの揺れ（はこ・イス など）は試していない。
- モデルは CLI の既定のもの。`LLM_MODEL` で別のモデルにしたときは、もう一度試すこと。
- 計画の比べ方は、伏せた Go2 が原点にいるときのもの。ほかの位置・姿勢からの計画は、Task B・G のテストが受け持つ。
