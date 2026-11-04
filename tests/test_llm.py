import json
import os
import subprocess
from pathlib import Path

import pytest

from voice_ltl import llm as llm_module
from voice_ltl.llm import ClaudeCLI, CodexCLI, FakeLLM, LLMError, make_llm

REPO = Path(__file__).resolve().parents[1]   # /work in the container
# test-only fake values (never real secrets)
FAKE_ENV = {
    "CLAUDE_CODE_OAUTH_TOKEN": "fake-claude-token",
    "ANTHROPIC_API_KEY": "fake-anthropic-key",
    "ANTHROPIC_BASE_URL": "http://claude.invalid",
    "OPENAI_API_KEY": "fake-openai-key",
    "OPENAI_BASE_URL": "http://openai.invalid",
    "CODEX_API_KEY": "fake-codex-key",
    "CODEX_HOME": "/fake/codex-home",
}

SCHEMA = {
    "type": "object",
    "properties": {"formula": {"type": ["string", "null"]},
                   "unsupported": {"type": "array", "items": {"type": "string"}}},
    "required": ["formula", "unsupported"],
}
GOOD = {"formula": "F hello", "unsupported": [], "substitutions": []}


def fake_run(stdout="", returncode=0, stderr=""):
    calls = []

    def run(argv, **kw):
        calls.append((argv, kw))
        return subprocess.CompletedProcess(argv, returncode, stdout, stderr)
    return run, calls


def test_claude_argv_and_parse():
    run, calls = fake_run(json.dumps({"is_error": False, "structured_output": GOOD}))
    out = ClaudeCLI(runner=run)("PROMPT", SCHEMA)
    argv, kw = calls[0]
    assert argv[:2] == ["claude", "-p"] and "--json-schema" in argv and "--bare" not in argv
    assert argv[argv.index("--tools") + 1] == "" and kw["input"] == "PROMPT"
    assert out == GOOD


def test_claude_model_flag_and_result_fallback():
    run, calls = fake_run(json.dumps({"is_error": False, "result": json.dumps(GOOD)}))
    assert ClaudeCLI(model="sonnet", runner=run)("p", SCHEMA) == GOOD
    argv = calls[0][0]
    assert argv[argv.index("--model") + 1] == "sonnet"


def test_claude_cli_failure_raises_llm_error():
    run, _ = fake_run(returncode=1, stderr="Invalid API key · Please run /login")
    with pytest.raises(LLMError, match="ログイン"):
        ClaudeCLI(runner=run)("p", SCHEMA)
    run, _ = fake_run(stdout="not json")
    with pytest.raises(LLMError, match="読めません"):
        ClaudeCLI(runner=run)("p", SCHEMA)
    run, _ = fake_run(json.dumps({"is_error": False, "structured_output": {"formula": 1}}))
    with pytest.raises(LLMError, match="読めません"):
        ClaudeCLI(runner=run)("p", SCHEMA)


def test_timeout_is_llm_error():
    def run(argv, **kw):
        raise subprocess.TimeoutExpired(argv, 60)
    with pytest.raises(LLMError, match="60"):
        ClaudeCLI(runner=run)("p", SCHEMA)


def test_stderr_log_drops_token_lines(monkeypatch):
    logged = []
    monkeypatch.setattr(llm_module.log, "warning", lambda fmt, *a: logged.append(fmt % a))
    run, _ = fake_run(returncode=2, stderr="boom\nsk-secret123\nOAuth token abc")
    with pytest.raises(LLMError):
        ClaudeCLI(runner=run)("p", SCHEMA)
    text = "\n".join(logged)
    assert "boom" in text and "sk-" not in text and "abc" not in text


def test_codex_argv(tmp_path):
    seen = {}

    def run(argv, **kw):
        seen["argv"], seen["kw"] = argv, kw
        Path(argv[argv.index("-o") + 1]).write_text(json.dumps(GOOD))
        return subprocess.CompletedProcess(argv, 0, "", "")
    assert CodexCLI(model="gpt-x", runner=run)("p", SCHEMA)["formula"] == "F hello"
    argv = seen["argv"]
    assert argv[:2] == ["codex", "exec"] and "--output-schema" in argv
    assert argv[argv.index("-m") + 1] == "gpt-x" and seen["kw"]["input"] == "p"


def test_codex_failure_and_bad_output():
    run, _ = fake_run(returncode=1, stderr="not logged in: please login")
    with pytest.raises(LLMError, match="ログイン"):
        CodexCLI(runner=run)("p", SCHEMA)
    run, _ = fake_run()  # out.json が書かれない
    with pytest.raises(LLMError, match="読めません"):
        CodexCLI(runner=run)("p", SCHEMA)


def test_fake_llm_records_and_raises():
    llm = FakeLLM([GOOD, LLMError("x")])
    assert llm("a", SCHEMA) == GOOD
    with pytest.raises(LLMError):
        llm("b", SCHEMA)
    assert llm.prompts == ["a", "b"]


def test_make_llm():
    assert isinstance(make_llm({}), ClaudeCLI)
    assert isinstance(make_llm({"LLM_BACKEND": "codex"}), CodexCLI)
    assert make_llm({"LLM_MODEL": "opus"}).model == "opus"
    with pytest.raises(ValueError):
        make_llm({"LLM_BACKEND": "gpt"})


def test_claude_is_error_payload_with_exit_zero():
    # 本物の claude が期限切れログインで返した形（終了コード 0、is_error 真、result にメッセージ）
    payload = {"type": "result", "is_error": True, "api_error_status": None,
               "result": "Failed to authenticate: OAuth session expired and could not be refreshed"}
    run, _ = fake_run(json.dumps(payload))
    with pytest.raises(LLMError, match="ログイン"):
        ClaudeCLI(runner=run)("p", SCHEMA)


def test_cli_not_found_is_llm_error():
    def run(argv, **kw):
        raise FileNotFoundError(argv[0])
    with pytest.raises(LLMError, match="見つかりません"):
        ClaudeCLI(runner=run)("p", SCHEMA)
    with pytest.raises(LLMError, match="見つかりません"):
        CodexCLI(runner=run)("p", SCHEMA)


def test_codex_timeout_is_llm_error():
    def run(argv, **kw):
        raise subprocess.TimeoutExpired(argv, 30)
    with pytest.raises(LLMError, match="30"):
        CodexCLI(timeout_s=30, runner=run)("p", SCHEMA)


def test_subprocess_and_files_use_utf8(monkeypatch):
    run, calls = fake_run(json.dumps({"is_error": False, "structured_output": GOOD}))
    ClaudeCLI(runner=run)("日本語", SCHEMA)
    assert calls[0][1]["encoding"] == "utf-8"

    seen = {}

    def run2(argv, **kw):
        seen["kw"] = kw
        reply = {"formula": "F hello", "unsupported": ["キッチン"]}
        Path(argv[argv.index("-o") + 1]).write_bytes(json.dumps(reply, ensure_ascii=False).encode("utf-8"))
        return subprocess.CompletedProcess(argv, 0, "", "")
    out = CodexCLI(runner=run2)("p", SCHEMA)
    assert seen["kw"]["encoding"] == "utf-8" and out["unsupported"] == ["キッチン"]


def test_is_error_exit_zero_non_login_logs_stderr_and_result_redacted(monkeypatch):
    logged = []
    monkeypatch.setattr(llm_module.log, "warning", lambda fmt, *a: logged.append(fmt % a))
    payload = {"is_error": True,
               "result": "Overloaded, try later\nsk-leaky999\nTOKEN=zzz\n" + "x" * 500}
    run, _ = fake_run(json.dumps(payload), stderr="stderr-note\nBearer token abc")
    with pytest.raises(LLMError, match="読めません"):
        ClaudeCLI(runner=run)("p", SCHEMA)
    text = "\n".join(logged)
    assert "stderr-note" in text and "Overloaded" in text
    assert "sk-" not in text and "token" not in text.lower()
    assert len(text) < 700


def test_fake_llm_exhausted_raises_llm_error():
    llm = FakeLLM([GOOD])
    llm("a", SCHEMA)
    with pytest.raises(LLMError, match="FakeLLM: 返事が足りません"):
        llm("b", SCHEMA)


# ---- SEC-3 (Ruling R37): an empty temp dir outside /work, no secrets of the other backend ----

def _empty_dir_outside_repo(kw) -> Path:
    """The cwd the runner got: an empty folder outside the repo (/work), checked at call time."""
    cwd = Path(kw["cwd"]).resolve()
    assert cwd.is_dir() and list(cwd.iterdir()) == []
    assert cwd != REPO and REPO not in cwd.parents
    assert Path("/work") not in cwd.parents
    return cwd


def _pairs(argv):
    return list(zip(argv, argv[1:]))


def test_claude_runs_in_an_empty_temp_dir_without_codex_secrets(monkeypatch):
    for k, v in FAKE_ENV.items():
        monkeypatch.setenv(k, v)
    seen = {}

    def run(argv, **kw):
        seen["cwd"], seen["env"] = _empty_dir_outside_repo(kw), kw["env"]
        return subprocess.CompletedProcess(
            argv, 0, json.dumps({"is_error": False, "structured_output": GOOD}), "")
    assert ClaudeCLI(runner=run)("p", SCHEMA) == GOOD
    env = seen["env"]
    assert env["CLAUDE_CODE_OAUTH_TOKEN"] == "fake-claude-token"   # its own login stays
    assert env["ANTHROPIC_BASE_URL"] == "http://claude.invalid"
    assert env["PATH"] == os.environ["PATH"] and env["HOME"] == os.environ["HOME"]
    assert [k for k in env if k.startswith(("OPENAI_", "CODEX_"))] == []
    assert not seen["cwd"].exists()   # removed after the call


def test_codex_runs_in_an_empty_temp_dir_without_claude_secrets_or_tools(monkeypatch):
    for k, v in FAKE_ENV.items():
        monkeypatch.setenv(k, v)
    seen = {}

    def run(argv, **kw):
        seen["cwd"], seen["env"], seen["argv"] = _empty_dir_outside_repo(kw), kw["env"], argv
        Path(argv[argv.index("-o") + 1]).write_text(json.dumps(GOOD))
        return subprocess.CompletedProcess(argv, 0, "", "")
    assert CodexCLI(runner=run)("p", SCHEMA) == GOOD
    argv, env, cwd = seen["argv"], seen["env"], seen["cwd"]
    # its working root is that empty folder; the schema and answer files sit outside it
    assert Path(argv[argv.index("-C") + 1]).resolve() == cwd
    for flag in ("--output-schema", "-o"):
        assert Path(argv[argv.index(flag) + 1]).resolve().parent != cwd
    # no tools: no shell (exec_command / write_stdin), no image reading, no web search,
    # no user config.toml (MCP servers), read-only sandbox
    pairs = _pairs(argv)
    for feature in ("shell_tool", "unified_exec", "view_image"):
        assert ("--disable", feature) in pairs
    assert ("-c", 'web_search="disabled"') in pairs
    assert "--ignore-user-config" in argv and ("--sandbox", "read-only") in pairs
    assert argv[-1] == "-"   # the prompt still comes on stdin
    # its own login stays, Claude's does not
    assert env["OPENAI_API_KEY"] == "fake-openai-key" and env["CODEX_API_KEY"] == "fake-codex-key"
    assert env["CODEX_HOME"] == "/fake/codex-home" and env["PATH"] == os.environ["PATH"]
    assert "CLAUDE_CODE_OAUTH_TOKEN" not in env
    assert [k for k in env if k.startswith("ANTHROPIC_")] == []
    assert not cwd.exists()


def test_child_env_is_read_when_called(monkeypatch):
    run, calls = fake_run(json.dumps({"is_error": False, "structured_output": GOOD}))
    llm = ClaudeCLI(runner=run)
    monkeypatch.setenv("OPENAI_API_KEY", "fake-late")
    monkeypatch.setenv("GO2_TEST_MARK", "1")
    llm("p", SCHEMA)
    env = calls[0][1]["env"]
    assert "OPENAI_API_KEY" not in env and env["GO2_TEST_MARK"] == "1"


@pytest.mark.parametrize("reply", [
    {"formula": "F hello", "unsupported": []},                                   # no substitutions
    {"formula": "F hello", "unsupported": [], "substitutions": None},
    {"formula": "F hello", "unsupported": [], "substitutions": [{"said": "走って"}]},
])
def test_a_reply_without_well_formed_substitutions_is_unreadable(reply):
    # design §7.4.1: all three keys are required, so a replacement is never acted on unseen
    from voice_ltl.prompt import SCHEMA as PROMPT_SCHEMA
    for stdout in (json.dumps({"is_error": False, "structured_output": reply}),
                   json.dumps({"is_error": False, "result": json.dumps(reply)})):
        run, _ = fake_run(stdout)
        with pytest.raises(LLMError):
            ClaudeCLI(runner=run)("p", PROMPT_SCHEMA)
    good = {**reply, "substitutions": [{"said": "走って", "as": "前へ歩く"}]}
    run, _ = fake_run(json.dumps({"is_error": False, "structured_output": good}))
    assert ClaudeCLI(runner=run)("p", PROMPT_SCHEMA) == good
