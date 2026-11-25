# 階段（公式の ClassicWalk）

Go2 に階段を登らせる（降りさせる）ときは、Unitree が用意した歩き方 `ClassicWalk` に切り替えます。このアプリが作った歩き方ではありません。段の上り下りは Go2 自身がします。このアプリは歩き方を切り替えて、いつもどおり「前へ進む」を送るだけです。

## 公式の説明

[Motion Services Interface V2.0](https://support.unitree.com/home/en/developer/Motion_Services_Interface_V2.0)（Go2 EDU、ソフト V1.1.6 以上）から：

| 窓口 | api_id | 公式の説明（要約） |
|---|---|---|
| `ClassicWalk(true)` | 2049 | AI-classic gait。複雑な地面に強く、階段・砂利・芝生・滑りやすい床に対応する。入っている間の `error_code` は 2010（Classic） |
| `ClassicWalk(false)` | 2049 | ClassicWalk をやめて、Agile（`error_code` 100）に入る |
| `StaticWalk()` | 1061 | ふだんの歩き方。複雑な地面には対応しない（このアプリは送りません） |

- `api_id` 2049 と `parameter` の形（`{"data": true}`／`{"data": false}`）は、公式 `unitree_sdk2` の `sport_api.hpp`（`ROBOT_SPORT_API_ID_CLASSICWALK`）と `unitree_sdk2_python` の `sport_client.py`（`ClassicWalk`）と同じです。
- [About Go2](https://support.unitree.com/home/en/developer/about_Go2) によると、登れる段の高さは約 16 cm（PRO・X・EDU。AIR は約 15 cm）、坂は 40°（AIR は 30°）までです。
- ソフトが V1.1.6 より前の Go2 には `ClassicWalk` がありません。古い `unitree_sdk2`（タグ 1.0.0）には `WalkStair`（api_id 1049）がありましたが、このアプリは対応していません。版は Unitree のアプリで確かめます（[real_robot.md](real_robot.md) の「用意するもの」）。

## 話し方

| 話す言葉 | 式 | すること |
|---|---|---|
| 階段を2メートル登って | `stairs_gait` → `moved_forward` 4 回 → `normal_gait` | 階段の歩き方に切り替え、前へ 2 m、ふだんの歩き方に戻す |
| 階段を1メートル降りて | `stairs_gait` → `moved_forward` 2 回 → `normal_gait` | 同じ（前を向いたまま降りる） |
| 階段モードにして／普通の歩き方に戻して | `F stairs_gait`／`F normal_gait` | 切り替えるだけ |
| 階段を登って／3段登って | `F stairs_gait` | 距離が無いので、切り替えるだけ。「こう受け取りました」に「進む距離を言ってください」と出す |

- 単語帳の命題：`stairs_gait`（階段も歩ける歩き方に切り替えた）、`normal_gait`（ふだんの歩き方に戻した）。どちらも立っているときだけできて、姿勢は変わりません（`nl_control/go2_nl_control/actions.py` の `stairs_gait`・`normal_gait`、`voice_ltl/world.py`）。
- 切り替えた後、次の動作まで 2 秒待ちます（`GAIT_SWITCH_S`。切り替わったという知らせは無いので、決まった秒数）。
- **Go2 は階段を見つけません。** 「前の階段を登って」と言っても、今の向きのまま、言われた距離だけ前へ進みます。階段の前に、まっすぐ向けて立たせてから話します。
- 前の障害物で止まる仕組みは、そのまま効きます（変えていません）。段を障害物と見て止まるかどうかは、まだ実機で確かめていません（下の手順 4）。
- 実際の LLM（Claude）で 8 通りの言い方を試し、8 通りとも上の表のとおりの式になりました（2026-11-18）。

## 偽の Go2 と Gazebo

- **Gazebo では階段を登れません。** Gazebo の Go2 は、平らな床を前提に歩く `unitree_guide` controller で動いています（足を上げるのは 8 cm まで、足を下ろす先の高さは床に決め打ち）。公式の `ClassicWalk` は Gazebo にはありません。
- 偽の Go2（`sim/sim_core.py`）と Gazebo の橋渡し（`gazebo/bridge_core.py`）は、立っているときの `ClassicWalk` を受け付けて、`/sportmodestate` の `error_code` だけを実機の表に合わせます（立ち・歩きの 1013・1015 を、切り替え後は 2010、戻した後は 100 にします）。歩き方は変わりません。橋渡しのログには「ClassicWalk 2049（Gazebo では歩き方は変わりません。階段は登れません）」と出ます。
- Gazebo で確かめたこと（2026-11-18、平らな床）：`run_formula` で「立つ → 階段の歩き方 → 前進 → 前進 → ふだんの歩き方 → 左に回る → 伏せる」が最後まで通りました（`done`）。`error_code` は 1013 → 2010（歩いている間も 2010）→ 100 → 1004 と変わり、転びませんでした。

  ```bash
  docker compose --profile gazebo up -d gazebo
  SIM_KIND=gazebo docker compose run --rm app python3 -m voice_ltl.run_formula \
      "F(stairs_gait & X F(moved_forward & X F(moved_forward & X F(normal_gait & X F(turned_left & X F lying)))))"
  ```

## キーボードで試す（実機、アプリなし）

登るところだけを試すなら、これがいちばん短い道です。アプリ（ページ・LLM・監督器）を使わずに、`scripts/keyboard_velocity.py` から公式の窓口を直接送ります。

- キー 3 で `ClassicWalk(true)`、キー 4 で `ClassicWalk(false)` を送ります（`/api/sport/request`、api_id 2049）。
- W を押している間だけ、前へ 0.3 m/s の `Move`（1008）を送ります。離すと 0.5 秒で `StopMove` を送って止まります。
- このアプリの LiDAR の止まる仕組みも、本体の障害物よけの窓口（`/api/obstacles_avoid`）も通りません。
- 画面には、`/sportmodestate` の `error_code` が変わるたびに `[STATE] error_code 2010 階段も歩ける歩き方 (ClassicWalk)` のような 1 行が出ます。
- 偽の Go2 で確かめたこと（2026-11-25）：1 → 3 → W を 2 秒 → 4 で、`error_code` は 1004 → 1002 → 1013 → 2010（歩いている間も 2010）→ 100 と変わりました。実機では、まだ確かめていません。

**用意：** Go2 と有線でつないだ Linux の PC（[real_robot.md](real_robot.md) の §1〜§3 と、§6 の `robot_check`）。ページ（サーバー）と `run_formula` は止めておきます（`/api/sport/request` の送り手が 2 つになります）。リモコンを持った人がそばにいて、すぐダンピング（Damp）にできるようにします。階段の下と横に人がいないようにします。

**起動：** Docker なしなら、PC の端末で：

```bash
source ~/unitree_ros2/setup.sh
python3 scripts/keyboard_velocity.py
```

Docker なら：

```bash
docker compose -f compose.robot.yaml run --rm --no-deps app python3 scripts/keyboard_velocity.py
```

止まるのが遅いと感じたら、キーのリピートを速くします（`xset r rate 200 30`。キーボードをつないだ PC のデスクトップの端末で）。

**手順：**

1. **版を確かめる：** Unitree のアプリで、Go2 のソフトが V1.1.6 以上かを確かめます。V1.1.6 より前なら、ここで止めます。
2. **平らな床で：** 前を 2 m 以上空けて、次を順に押します。出た `error_code` を書き留めます。
   1. 1（立つ）→ 1013 が出る。
   2. 3（階段の歩き方）→ 2010 が出るか。
   3. W を 2 秒ほど押して離す → 歩くか。歩いている間に別の値が出ないか。
   4. 4（ふだんの歩き方）→ 100 が出るか。
   5. W を少し押す → ふだんどおり歩くか。
3. **低い段 1 つ：** 高さ 5〜10 cm ほどの、ずれない台を、Go2 の前 0.5 m に置きます。A・D・Q・E でまっすぐ向けてから、3 → W を押し続けます。後ろ足まで台に乗ったら離します。次を書き留めます。
   - 台の前で止まって進まなかったか（本体の障害物よけが、`Move`（1008）でも効くのかは、わかっていません）。
   - 登れたか。そのときの `error_code`。
4. **本物の階段：** 段の高さが 16 cm 以下の階段で、いちばん下の段の前に、まっすぐ向けて立たせます。3 → W で、最初は 2〜3 段で離して止まり、様子を見ます。段の上で止まったまま立っていられるかは、わかっていません。うまくいったら、上の平らなところまで登って離し、4 で戻します。
5. **終わり：** 2 で伏せるか、Z で終了します。3 のまま終えると「階段も歩ける歩き方のままです」と出ます。そのときは、もう一度起動して 4 を押します。

## 実機で試す手順（ページから話す）

[real_robot.md](real_robot.md) の準備と「動かす前の確かめ」（`robot_check`、姿勢の値、LiDAR）を済ませてから始めます。以下のコマンドは Docker の形です。Docker なしで動かすときは `bash scripts/run_local.sh robot` でページを開き、`ros2` の道具は PC の ROS 2 で動かします。

**安全：** リモコンを持った人がそばにいて、すぐダンピング（Damp）にできるようにします（[real_robot.md](real_robot.md) の「安全の注意」）。階段では、転ぶと下まで落ちます。最初は低い段 1 つから試します。階段の下と横に人がいないようにします。降りる練習は、登るのがうまくいってからにします。

1. **版を確かめる：** Unitree のアプリで Go2 のソフトが V1.1.6 以上かを確かめ、版を書き留めます。V1.1.6 より前なら、ここで止めます（`ClassicWalk` がありません）。
2. **`error_code` を見られるようにする：** 別の端末で次を動かしたままにします。

   ```bash
   docker compose -f compose.robot.yaml run --rm --no-deps app ros2 topic echo /sportmodestate --field error_code
   ```

3. **平らな床で切り替える：** 前と後ろを 2 m 以上空けた平らな床で、ページから次を順に話します。
   1. 「立って」→ 1013 などの立ちの値になる。
   2. 「階段モードにして」→ 2010 になるか。
   3. 「1メートル進んで」→ 歩くか。歩いている間の値を書き留めます（2010 のままか）。
   4. 「普通の歩き方に戻して」→ 100 になるか（公式の説明では Agile に入る）。
   5. 「1メートル進んで」→ ふだんどおり歩くか。

   値が表と違ったら、書き留めて知らせてください。2010・100 以外の値になると、姿勢が「不明」になって止まります（`actions.py` の `_ERROR_CODE_TO_STATE` に足す必要があります）。
4. **低い段 1 つ：** 高さ 5〜10 cm ほどの、ずれない台を Go2 の前 0.5 m に置き、まっすぐ向けて「階段を1メートル登って」と言います。次を書き留めます。
   - 段の前で止まったか（ページに「一時停止」「前方に障害物があります」が出るか、15 秒後に中止になるか）。本体の障害物よけ（`GO2_OBSTACLE_AUTHORITY=go2`、既定）が段を障害物と見るかどうかは、まだわかっていません。
   - 登れたか。登っている間の `error_code`。
5. **本物の階段：** 段の高さが 16 cm 以下の階段で、いちばん下の段の前に、まっすぐ向けて立たせます。「階段を1メートル登って」から始め、うまくいったら距離を延ばします。登り切った先が平らで広いことを確かめておきます。
6. **降りる：** 登るのがうまくいってから、いちばん上の段の手前に、階段に向けて立たせ、「階段を1メートル降りて」から試します。

**わかっていないこと（実機で確かめる）：**

- 本体の障害物よけを通した Move（`/api/obstacles_avoid/request`）で、ClassicWalk の歩き方のまま歩くか。
- 本体の障害物よけが、段を障害物と見て止まるか。止まる場合、このアプリには障害物よけを切って登る道はありません（作っていません）。
- ClassicWalk の切り替えにかかる時間（このアプリは 2 秒待つ）。
- `ClassicWalk(false)` の後の Agile（100）で、ほかの動作（座る・挨拶など）がいつもどおりできるか。
- 伏せて立ち直した後も ClassicWalk のままか。偽の Go2 と Gazebo は、切り替えた歩き方を伏せても覚えています（実機に合わせたものではありません）。

**今のつくりの限り：**

- 監督器は、今どちらの歩き方かを知りません（姿勢だけを見ます）。そのため、
  - 「階段を2メートル登って」が途中で止まると（「止まって」、障害物で 15 秒待って中止など）、`normal_gait` は送られず、Go2 は ClassicWalk のままです。次の指示もその歩き方で動きます。止まったときは「普通の歩き方に戻して」と言います。
  - 階段の歩き方の間でも、式にあれば挨拶・座る・伏せるをします。階段の上ではそういう指示をしないでください。
  - 「普通の歩き方に戻して」は、ClassicWalk でないときにも `ClassicWalk(false)` を送ります。公式の説明では、そのとき Go2 は Agile（100）に入ります。
