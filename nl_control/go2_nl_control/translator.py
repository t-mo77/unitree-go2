"""LLM による翻訳：自然言語 → 動作計画（JSON）。

LLM の担当はここだけ。出力は構造化出力（JSON Schema）で形を固定し、
supervisor.check_plan() で検証し、不合格なら理由を添えて最大3回まで聞き直す。
LLM の誤りをコード側で勝手に直すことはしない。

OpenAI 互換の Chat Completions API なら何でも使える（Groq, Ollama, LM Studio, OpenRouter, vLLM など）。
"""

import json
import os
import re
import urllib.error
import urllib.request
from typing import Callable, Dict, List, Optional, Tuple

from .actions import LIMITS, MAX_STEPS, ActionSpec
from .supervisor import Step, check_plan

STOP_WORDS = ("止まれ", "止まって", "とまれ", "とまって", "ストップ", "停止", "stop", "やめて", "待って")


def is_stop_command(text: str) -> bool:
    """「止まって」系の指示は LLM を通さずに即停止する（遅延なし・誤訳なし）。"""
    t = text.strip().lower()
    return len(t) <= 12 and any(w in t for w in STOP_WORDS)


def plan_schema(cat: Dict[str, ActionSpec]) -> dict:
    return {
        "type": "object",
        "properties": {
            "steps": {
                "type": "array",
                "maxItems": MAX_STEPS,
                "items": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": sorted(cat)},
                        "value": {"type": "number"},
                    },
                    "required": ["action", "value"],
                    "additionalProperties": False,
                },
            },
        },
        "required": ["steps"],
        "additionalProperties": False,
    }


def build_messages(text: str, state: str, cat: Dict[str, ActionSpec],
                   rejected: Optional[Tuple[str, str]] = None) -> List[dict]:
    def line(a: ActionSpec) -> str:
        if a.param:
            lo, hi = LIMITS[a.param]
            v = f"value = {a.param}（{lo}〜{hi}）"
        else:
            v = "value = 0"
        return f"- {a.name}: {a.description}。{v}。実行できる状態: {', '.join(sorted(a.transitions))}"

    vocab = "\n".join(line(a) for a in cat.values())
    system = f"""あなたは四足歩行ロボット Unitree Go2 の動作計画係です。
人の指示を、下の語彙だけを使った動作の列（JSON）に翻訳してください。

# Goal
指示どおりの動作を、実行できる順番で並べる。

# Context
- ロボットの今の状態: {state}
- 状態は STANDING（歩ける立ち方）/ SITTING（おすわり）/ LYING（伏せ）/ UNKNOWN（不明）。
- 移動やジェスチャーは STANDING でしかできない。LYING や UNKNOWN なら先に stand、SITTING なら先に rise_sit。
- 前方の障害物はロボット側が見張っていて、近づけば自動で止まり、なくなれば続きから動く。計画で気にしなくてよい。

# Vocabulary（使ってよい action はこれだけ）
{vocab}

# Constraints
- 語彙に無い動作を作らない。できない指示は、できる範囲で最も近い動作にする。危ない動作を勝手に足さない。
- 距離の指定が無ければ 0.5 m、角度の指定が無ければ 90 度とする。「振り向いて」は 180 度。
- 動作は最大 {MAX_STEPS} 個。

# Done when
JSON {{"steps": [...]}} だけを出力する。説明文は書かない。

# Examples
指示: 立って1メートル前に進んで
{{"steps":[{{"action":"stand","value":0}},{{"action":"forward","value":1.0}}]}}
指示: 左を向いてから挨拶して、最後に座って
{{"steps":[{{"action":"turn_left","value":90}},{{"action":"hello","value":0}},{{"action":"sit","value":0}}]}}"""
    user = f"指示: {text}"
    if rejected:
        prev, reason = rejected
        user += f"\n\n前回の出力 {prev} は次の理由で却下されました: {reason}\n直した JSON を出力してください。"
    return [{"role": "system", "content": system}, {"role": "user", "content": user}]


def extract_json(text: str) -> object:
    """返答から JSON を取り出す。<think> とコードブロックだけ剥がし、中身は直さない。"""
    t = re.sub(r"<think>.*?</think>", "", text, flags=re.S).strip()
    m = re.search(r"```(?:json)?\s*(.*?)```", t, flags=re.S)
    if m:
        t = m.group(1).strip()
    if not t.startswith("{"):
        i, j = t.find("{"), t.rfind("}")
        if i >= 0 and j > i:
            t = t[i:j + 1]
    return json.loads(t)


def load_env_file(path: str) -> None:
    """env.txt / .env の KEY=VALUE を環境変数に入れる（既にある値は上書きしない）。"""
    if not path or not os.path.isfile(path):
        return
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


class OpenAICompatLLM:
    """OpenAI 互換 API の最小クライアント（標準ライブラリのみ）。"""

    def __init__(self, base_url: str, api_key: str, model: str, timeout: float = 20.0):
        self.url = base_url.rstrip("/") + "/chat/completions"
        self.api_key = api_key
        self.model = model
        self.timeout = timeout
        self._use_schema = True

    def _post(self, body: dict) -> dict:
        req = urllib.request.Request(
            self.url, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {self.api_key}"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read().decode())

    def __call__(self, messages: List[dict], schema: dict) -> str:
        body = {"model": self.model, "messages": messages, "temperature": 0}
        if self._use_schema:
            body["response_format"] = {"type": "json_schema", "json_schema": {
                "name": "go2_plan", "schema": schema, "strict": True}}
        else:
            body["response_format"] = {"type": "json_object"}
        try:
            data = self._post(body)
        except urllib.error.HTTPError as e:
            if e.code == 400 and self._use_schema:
                # 構造化出力に対応していないサーバー → JSON モードで再試行（形は check_plan で守る）
                self._use_schema = False
                return self(messages, schema)
            raise
        return data["choices"][0]["message"]["content"] or ""


class Translator:
    def __init__(self, llm: Callable[[List[dict], dict], str], max_tries: int = 3):
        self.llm = llm
        self.max_tries = max_tries

    def translate(self, text: str, state: str,
                  cat: Dict[str, ActionSpec]) -> Tuple[Optional[List[Step]], List[str]]:
        """戻り値: (合格した計画 or None, 試行ログ)"""
        log: List[str] = []
        schema = plan_schema(cat)
        rejected = None
        for k in range(1, self.max_tries + 1):
            try:
                reply = self.llm(build_messages(text, state, cat, rejected), schema)
            except Exception as e:  # 通信エラーなどは再試行しない（ロボットは止まったまま）
                log.append(f"[{k}] LLM 呼び出し失敗: {e}")
                return None, log
            try:
                raw = extract_json(reply)
            except Exception:
                reason = "JSON として読めません"
                log.append(f"[{k}] 却下: {reason}")
                rejected = (reply[:300], reason)
                continue
            steps, reason = check_plan(raw, state, cat)
            if steps is not None:
                log.append(f"[{k}] 合格: {json.dumps(raw, ensure_ascii=False)}")
                return steps, log
            log.append(f"[{k}] 却下: {reason}")
            rejected = (json.dumps(raw, ensure_ascii=False)[:300], reason)
        return None, log
