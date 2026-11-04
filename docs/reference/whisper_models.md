# Whisper モデルの比較（Task 11）

音声: `tests/fixtures/stand_hello.wav`（macOS `say -v Kyoko "立って挨拶して"`、約 1.5 秒、16 kHz）。
faster-whisper 1.2.1、int8、CPU（Docker Desktop arm64、8 CPU。ほかのコンテナが同時に動いている）。
モデル読み込み後の 2 回目の呼び出しの秒数。

| モデル | 書き起こし | 秒 | 「挨拶」 |
|---|---|---|---|
| small | だって挨拶して | 1.56 | 出た |
| medium | 語って挨拶して | 6.20 | 出た |
| large-v3-turbo | たって挨拶して | 8.67 | 出た |

3 つとも「挨拶」は出るが「立って」は正しく出ない（合成音声のため）。
基準（設計書 §11：5 秒の指示を 3 秒程度以内、この音声では 1 秒以内が目安）に最も近いのは small だけなので、
`env.txt.example` の `WHISPER_MODEL` の既定は `small` とする。
medium / large-v3-turbo はこの CPU では遅すぎる（実機の PC や GPU なら `WHISPER_MODEL` で切り替えられる）。
