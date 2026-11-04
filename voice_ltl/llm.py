"""LLM の窓口。Claude CLI / Codex CLI / テスト用の偽物。

翻訳役の CLI に、トークンや作業のファイルを触らせない（SEC-3、Ruling R37）:
- 空の一時フォルダ（/work の外）を作業場所にして起動する。
- 子のプロセスの環境変数から、もう一方の LLM の秘密を除く。
- Codex には道具（シェル・画像の読み込み・Web 検索）を使わせない。Claude は `--tools ""`。
- compose は /work/env.txt を空のもの（/dev/null）で覆う（compose.yaml・compose.robot.yaml）。
"""
import json
import logging
import os
import re
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Callable, Mapping

log = logging.getLogger(__name__)

LLM = Callable[[str, dict], dict]

MSG_LOGIN = "LLM にログインできていません（README の手順でログインしてください）"
MSG_UNREADABLE = "LLM の返事が読めませんでした"


class LLMError(Exception):
    """利用者に見せる日本語の短い文。"""


def _msg_timeout(timeout_s: float) -> str:
    t = int(timeout_s) if float(timeout_s).is_integer() else timeout_s
    return f"LLM の返事が {t} 秒で返ってきませんでした"


_SECRET_WORDS = ("sk-", "oauth", "token", "api_key", "api-key", "apikey",
                 "secret", "password", "bearer ", "authorization")
_LONG_VALUE = re.compile(r"=\s*\S{20,}")
MSG_REDACTED = "[伏せました]"


def _is_secretish(line: str) -> bool:
    low = line.lower()
    return any(w in low for w in _SECRET_WORDS) or bool(_LONG_VALUE.search(line))


def _redacted_head(text: str | None) -> str:
    """先頭 300 文字だけ返す（資格情報らしき行は伏せる）。"""
    if not text:
        return ""
    lines = [MSG_REDACTED if _is_secretish(l) else l for l in text.splitlines()]
    return "\n".join(lines)[:300]


def _log_stderr(stderr: str | None) -> None:
    text = _redacted_head(stderr)
    if text:
        log.warning("LLM stderr: %s", text)


def _looks_like_login_problem(proc) -> bool:
    blob = f"{proc.stderr or ''}\n{proc.stdout or ''}".lower()
    return any(k in blob for k in ("login", "auth", "api key"))


def _matches_schema(value, schema: dict) -> bool:
    """schema の形（type / required / properties / items / null 許容）だけ確かめる。"""
    types = schema.get("type")
    if types is not None:
        types = [types] if isinstance(types, str) else types
        checks = {
            "object": lambda v: isinstance(v, dict),
            "array": lambda v: isinstance(v, list),
            "string": lambda v: isinstance(v, str),
            "null": lambda v: v is None,
            "boolean": lambda v: isinstance(v, bool),
            "integer": lambda v: isinstance(v, int) and not isinstance(v, bool),
            "number": lambda v: isinstance(v, (int, float)) and not isinstance(v, bool),
        }
        if not any(checks.get(t, lambda v: True)(value) for t in types):
            return False
    if isinstance(value, dict):
        if any(k not in value for k in schema.get("required", [])):
            return False
        for k, sub in schema.get("properties", {}).items():
            if k in value and not _matches_schema(value[k], sub):
                return False
    if isinstance(value, list) and "items" in schema:
        return all(_matches_schema(v, schema["items"]) for v in value)
    return True


def _validated(value, schema: dict) -> dict:
    if not isinstance(value, dict) or not _matches_schema(value, schema):
        raise LLMError(MSG_UNREADABLE)
    return value


# 子のプロセスに渡さない、もう一方の LLM の秘密（環境変数の名前の頭）
_CLAUDE_SECRETS = ("CLAUDE_CODE_OAUTH_TOKEN", "ANTHROPIC_")   # Codex に渡さない
_CODEX_SECRETS = ("OPENAI_", "CODEX_")                         # Claude に渡さない

# Codex に道具を使わせない（codex-cli 0.166.0 の `codex features list` と
# `codex debug prompt-input` で確かめた）。shell_tool・unified_exec を切るとシェル
# （exec_command・write_stdin）が、view_image を切ると画像の読み込みが消える。
# Web 検索は切る。ユーザーの config.toml（MCP サーバーなど）は読まない（ログインは ~/.codex のまま）。
_CODEX_NO_TOOLS = ("--ignore-user-config",
                   "--disable", "shell_tool", "--disable", "unified_exec", "--disable", "view_image",
                   "-c", 'web_search="disabled"')


def _child_env(drop: tuple[str, ...]) -> dict[str, str]:
    """今の環境変数の写しから、名前が drop のどれかで始まるものを除く。"""
    return {k: v for k, v in os.environ.items() if not k.startswith(drop)}


def _run(runner, argv, prompt: str, timeout_s: float, cwd: str, env: dict[str, str]):
    try:
        return runner(argv, input=prompt, capture_output=True, text=True,
                      encoding="utf-8", timeout=timeout_s, cwd=cwd, env=env)
    except subprocess.TimeoutExpired:
        raise LLMError(_msg_timeout(timeout_s)) from None
    except FileNotFoundError:
        raise LLMError(f"{argv[0]} コマンドが見つかりません") from None


def _check_exit(proc) -> None:
    if proc.returncode != 0:
        _log_stderr(proc.stderr)
        if _looks_like_login_problem(proc):
            raise LLMError(MSG_LOGIN)
        raise LLMError(MSG_UNREADABLE)


class ClaudeCLI:
    def __init__(self, model: str | None = None, timeout_s: float = 60.0, runner=subprocess.run):
        self.model, self.timeout_s, self.runner = model, timeout_s, runner

    def __call__(self, prompt: str, schema: dict) -> dict:
        argv = ["claude", "-p", "--output-format", "json",
                "--json-schema", json.dumps(schema),
                "--tools", "", "--no-session-persistence"]
        if self.model:
            argv += ["--model", self.model]
        with tempfile.TemporaryDirectory(prefix="go2-llm-") as cwd:   # 空のフォルダ（/work の外）
            proc = _run(self.runner, argv, prompt, self.timeout_s, cwd, _child_env(_CODEX_SECRETS))
        _check_exit(proc)
        try:
            data = json.loads(proc.stdout)
        except (TypeError, ValueError):
            raise LLMError(MSG_UNREADABLE) from None
        if not isinstance(data, dict):
            raise LLMError(MSG_UNREADABLE)
        if data.get("is_error"):
            if _looks_like_login_problem(proc) or any(
                    k in str(data.get("result", "")).lower() for k in ("login", "auth", "api key")):
                raise LLMError(MSG_LOGIN)
            _log_stderr(proc.stderr)
            result = _redacted_head(str(data.get("result", "")))
            if result:
                log.warning("LLM result: %s", result)
            raise LLMError(MSG_UNREADABLE)
        out = data.get("structured_output")
        if out is None:
            try:
                out = json.loads(data.get("result"))
            except (TypeError, ValueError):
                raise LLMError(MSG_UNREADABLE) from None
        return _validated(out, schema)


class CodexCLI:
    def __init__(self, model: str | None = None, timeout_s: float = 60.0, runner=subprocess.run):
        self.model, self.timeout_s, self.runner = model, timeout_s, runner

    def __call__(self, prompt: str, schema: dict) -> dict:
        with tempfile.TemporaryDirectory(prefix="go2-llm-") as d:
            # Codex の作業場所は空のフォルダ（/work の外）。schema と答えのファイルはその外に置く
            cwd = Path(d) / "cwd"
            cwd.mkdir()
            schema_path, out_path = Path(d) / "schema.json", Path(d) / "out.json"
            schema_path.write_text(json.dumps(schema), encoding="utf-8")
            argv = ["codex", "exec", "--skip-git-repo-check", "--ephemeral",
                    "--sandbox", "read-only", *_CODEX_NO_TOOLS, "-C", str(cwd),
                    "--output-schema", str(schema_path), "-o", str(out_path)]
            if self.model:
                argv += ["-m", self.model]
            argv.append("-")  # プロンプトは標準入力から読む
            proc = _run(self.runner, argv, prompt, self.timeout_s, str(cwd),
                        _child_env(_CLAUDE_SECRETS))
            _check_exit(proc)
            try:
                out = json.loads(out_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                raise LLMError(MSG_UNREADABLE) from None
        return _validated(out, schema)


class FakeLLM:
    """テスト用。replies を順に返す（Exception は raise する）。"""

    def __init__(self, replies: list, delay_s: float = 0.0):
        self.replies = list(replies)
        self.delay_s = delay_s
        self.prompts: list[str] = []

    def __call__(self, prompt: str, schema: dict) -> dict:
        self.prompts.append(prompt)
        if self.delay_s:
            time.sleep(self.delay_s)
        if len(self.prompts) > len(self.replies):
            raise LLMError("FakeLLM: 返事が足りません")
        reply = self.replies[len(self.prompts) - 1]
        if isinstance(reply, Exception):
            raise reply
        return reply


def make_llm(env: Mapping[str, str]) -> LLM:
    backend = (env.get("LLM_BACKEND") or "").strip().lower() or "claude"
    model = (env.get("LLM_MODEL") or "").strip() or None
    if backend == "claude":
        return ClaudeCLI(model=model)
    if backend == "codex":
        return CodexCLI(model=model)
    raise ValueError(f"LLM_BACKEND が不正です: {backend}（claude か codex）")
