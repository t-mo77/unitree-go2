"""LLM の窓口。Claude CLI / Codex CLI / テスト用の偽物。"""
import json
import logging
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


def _log_stderr(stderr: str | None) -> None:
    """標準エラーの先頭 300 文字だけをログに残す（トークンらしき行は除く）。"""
    if not stderr:
        return
    lines = [l for l in stderr.splitlines()
             if "sk-" not in l and "oauth" not in l.lower()]
    text = "\n".join(lines)[:300]
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


def _run(runner, argv, prompt: str, timeout_s: float):
    try:
        return runner(argv, input=prompt, capture_output=True, text=True, timeout=timeout_s)
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
        proc = _run(self.runner, argv, prompt, self.timeout_s)
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
        with tempfile.TemporaryDirectory() as d:
            schema_path, out_path = Path(d) / "schema.json", Path(d) / "out.json"
            schema_path.write_text(json.dumps(schema))
            argv = ["codex", "exec", "--skip-git-repo-check", "--ephemeral",
                    "--sandbox", "read-only",
                    "--output-schema", str(schema_path), "-o", str(out_path)]
            if self.model:
                argv += ["-m", self.model]
            argv.append("-")  # プロンプトは標準入力から読む
            proc = _run(self.runner, argv, prompt, self.timeout_s)
            _check_exit(proc)
            try:
                out = json.loads(out_path.read_text())
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
        reply = self.replies[len(self.prompts) - 1]
        if isinstance(reply, Exception):
            raise reply
        return reply


def make_llm(env: Mapping[str, str]) -> LLM:
    backend = (env.get("LLM_BACKEND") or "claude").strip().lower()
    model = (env.get("LLM_MODEL") or "").strip() or None
    if backend == "claude":
        return ClaudeCLI(model=model)
    if backend == "codex":
        return CodexCLI(model=model)
    raise ValueError(f"LLM_BACKEND が不正です: {backend}（claude か codex）")
