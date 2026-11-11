# 準備の詳しい説明（Mac）

コマンドだけなら [README](../README.md#1-準備mac) にあります。

## 準備の手順

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

## 書き起こしのモデル

`env.txt` の `WHISPER_MODEL` は既定で `small` です。この Mac（CPU）では、1 秒前後に近いのは `small` だけでした。`medium` と `large-v3-turbo` は遅すぎます（比較は [docs/reference/whisper_models.md](reference/whisper_models.md)）。GPU のある PC では `medium` などに書き換えて試せます。

## テスト

```bash
docker compose run --rm --no-deps app python3 -m pytest -q -m "not slow" tests nl_control/tests
```

`run --rm` は、使い捨てのコンテナを 1 つ起動してコマンドを実行し、終わると消します。

- `-m "not slow"` は、遅いテストを除きます。普段はこれで足ります。
- 遅いテスト（`slow`）は、本物の Whisper のモデルを使います。初回はモデルを落とすので時間がかかります。流すときは `-m "not slow"` を外すか、`-m slow tests` にします。
