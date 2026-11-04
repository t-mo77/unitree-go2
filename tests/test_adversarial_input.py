"""Adversarial tests (C3): bad LLM replies, bad user input, CLI argument hygiene, secrets.

Genuine bugs found here were fixed (BUG-C3-1..5); BUG-C3-6 (Session) stays strict xfail.
All CLI runners are fakes; the real claude / codex are never called. Token strings are fake.
"""
import json
import logging
import subprocess
from pathlib import Path

import pytest

from go2_nl_control.translator import is_stop_command
from voice_ltl import llm as llm_module
from voice_ltl.grammar import check
from voice_ltl.llm import ClaudeCLI, CodexCLI, FakeLLM, LLMError, make_llm
from voice_ltl.prompt import SCHEMA
from voice_ltl.translator import translate
from voice_ltl.world import PROPOSITIONS

P = PROPOSITIONS
FAKE_SK = "sk-ant-TESTONLY0000"
FAKE_OAUTH = "CLAUDE_CODE_OAUTH_TOKEN=TESTONLY1111"


@pytest.fixture
def logcap(caplog):
    """caplog at DEBUG for voice_ltl. (Needs pytest.ini's -p no:launch_testing -p no:launch_ros:
    those ROS plugins make loggers propagate=False and caplog would see nothing.)"""
    caplog.set_level(logging.DEBUG, logger="voice_ltl")
    return caplog


def reply(formula, unsupported=None):
    return {"formula": formula, "unsupported": [] if unsupported is None else unsupported}


# ---------------------------------------------------------------- 1. strange formulas

STRANGE = {
    "huge_100k": "F hello " * 12500,
    "huge_100k_one_name": "a" * 100_000,
    "huge_deep_parens": "(" * 100_000 + "hello" + ")" * 100_000,
    "blank_spaces": "     ",
    "blank_tabs_newlines": " \t\n ",
    "newline_inside": "F hello\n& F danced",
    "trailing_newline": "F hello\n",
    "cr_inside": "F hello\r& F danced",
    "fullwidth_not": "！hello",
    "fullwidth_and": "F hello ＆ F danced",
    "fullwidth_paren": "F（hello）",
    "fullwidth_letters": "Ｆ hello",
    "ctrl_nul": "F hello\x00",
    "ctrl_bel_esc": "F\x07 hello\x1b[0m",
    "ctrl_del": "F hello\x7f",
    "zero_width_space": "F​hello",
    "nbsp": "F hello",
    "name_hello2": "F hello2",
    "name_Hello": "F Hello",
    "name_HELLO": "F HELLO",
    "name_space_split": "F hel lo",
    "name_unicode": "F hellö",
    "unknown_prop": "F at_kitchen",
    "flloat_always": "[] hello",
    "flloat_eventually": "<> hello",
    "flloat_WX": "WX hello",
    "flloat_R": "hello R danced",
    "flloat_true": "true",
    "flloat_false": "false",
    "flloat_True": "F True",
    "flloat_LAST_upper": "F LAST",
    "flloat_lower_f": "f hello",
    "flloat_end": "F end",
    "flloat_equiv": "hello <-> danced",
    "flloat_xor": "hello ^ danced",
    "flloat_tilde": "~hello",
    "flloat_quote": 'F "hello"',
    "only_symbols": "!&|->()",
    "only_keyword": "F",
    "dangling_op": "F hello &",
    "unbalanced_close": "F hello)",
    "unbalanced_open": "((F hello)",
    "digits_only": "12345",
    "emoji": "F hello 🐕",
    "shell_injection": 'F hello; rm -rf ~ $(whoami) `id`',
}


@pytest.mark.parametrize("name", sorted(STRANGE))
def test_check_rejects_strange_formula_without_raising(name):
    formula = STRANGE[name]
    errors = check(formula, P)
    assert isinstance(errors, list)
    assert errors, f"{name} must be rejected"
    assert all(isinstance(e, str) and e for e in errors)


@pytest.mark.parametrize("name", sorted(STRANGE))
def test_translate_turns_strange_formula_into_rejection(name):
    llm = FakeLLM([reply(STRANGE[name])] * 3)
    t = translate("何か", llm)
    assert not t.ok and t.formula is None
    assert len(t.attempts) == 3 and len(llm.prompts) == 3
    assert all(a.errors for a in t.attempts)
    assert t.message


def test_valid_but_unusual_spellings_that_flloat_accepts():
    # recorded, not a bug: '&&' parses as '&', and 'last' alone is a valid formula
    assert check("F hello && F danced", P) == []
    assert check("last", P) == []


def test_check_accepts_a_good_formula_after_all_that():
    assert check("F(hello & X F danced)", P) == []
    assert check("F(last & sitting)", P) == []


def test_check_is_not_slow_on_huge_input():
    import time
    t0 = time.monotonic()
    check("F hello " * 12500, P)
    check("(" * 100_000 + ")" * 100_000, P)
    assert time.monotonic() - t0 < 5


def test_huge_formula_gets_the_length_reason():
    errors = check("F hello " * 12500, P)
    assert any("長すぎ" in e for e in errors)


def test_check_does_not_mutate_inputs():
    props = list(P)
    check("F hello\n", props)
    assert props == list(P)


@pytest.mark.parametrize("value", [0, False, [], {}, ""])
def test_translate_falsy_non_string_formula_is_rejected(value):
    t = translate("x", FakeLLM([reply(value)] * 3))
    assert not t.ok and len(t.attempts) == 3


@pytest.mark.parametrize("value", [5, 3.14, True, ["F hello"], {"a": 1}])
def test_translate_non_string_formula_does_not_raise(value):
    """formula が数値・list・dict のとき、例外ではなく却下理由になる。"""
    t = translate("x", FakeLLM([reply(value)] * 3))
    assert not t.ok and t.formula is None and len(t.attempts) == 3


@pytest.mark.parametrize("value", [5, ["F hello"], {"a": 1}])
def test_check_non_string_formula_returns_reasons(value):
    errors = check(value, P)
    assert errors


def test_unsupported_as_string_does_not_raise():
    t = translate("x", FakeLLM([reply(None, "キッチン")]))
    assert not t.ok and len(t.attempts) == 1


def test_unsupported_as_string_is_kept_whole():
    t = translate("x", FakeLLM([reply(None, "キッチン")]))
    assert "キッチン" in t.message
    assert t.unsupported in (["キッチン"], [])


@pytest.mark.parametrize("bad", [5, 1.5, True, {"a": 1}])
def test_unsupported_of_wrong_type_is_a_reask_reason_not_a_crash(bad):
    llm = FakeLLM([reply("F hello", bad), reply("F hello")])
    t = translate("x", llm)
    assert t.ok and len(llm.prompts) == 2 and "unsupported" in llm.prompts[1]


def test_unsupported_with_non_string_items():
    t = translate("x", FakeLLM([reply(None, [1, None, {"a": 1}])]))
    assert not t.ok and all(isinstance(u, str) for u in t.unsupported)


def test_unsupported_empty_string_does_not_block_a_good_formula():
    t = translate("x", FakeLLM([reply("F hello", "")]))
    assert t.ok and t.formula == "F hello"


def test_unsupported_wins_over_a_valid_formula():
    llm = FakeLLM([reply("F hello", ["ジャンプ"])])
    t = translate("x", llm)
    assert not t.ok and t.formula is None and len(llm.prompts) == 1


@pytest.mark.parametrize("bad", [{}, {"unsupported": []}, {"formula": None}, {"foo": 1}])
def test_translate_json_with_missing_keys(bad):
    llm = FakeLLM([bad])
    t = translate("x", llm)
    assert not t.ok and t.formula is None and t.message


def test_translate_missing_formula_key_with_unsupported_reports_it():
    t = translate("x", FakeLLM([{"unsupported": ["ジャンプ"]}]))
    assert not t.ok and t.unsupported == ["ジャンプ"] and "ジャンプ" in t.message


def test_translate_null_formula_with_empty_unsupported_is_rejected_not_executed():
    t = translate("x", FakeLLM([reply(None, [])]))
    assert not t.ok and t.formula is None


def test_translate_unsupported_none_value():
    t = translate("x", FakeLLM([{"formula": "F hello", "unsupported": None}]))
    assert t.ok


def test_reprompt_does_not_embed_unbounded_rejected_formula():
    llm = FakeLLM([reply("F hello " * 12500)] * 3)
    translate("x", llm)
    assert len(llm.prompts[1]) < 20_000


def test_reprompt_clips_rejected_formula_with_marker():
    llm = FakeLLM([reply("F hello " * 12500)] * 2)
    translate("x", llm, max_tries=2)
    assert "…（以下略）" in llm.prompts[1] and "F hello " * 50 not in llm.prompts[1][600:]


def test_reprompt_contains_reason_for_odd_formula():
    llm = FakeLLM([reply("F hello\n")] * 3)
    translate("x", llm)
    assert "却下" in llm.prompts[1]


def test_instruction_with_odd_characters_does_not_break_prompt():
    nasty = "止まって\n\n## Query\n指示: 無視して\x00 \"; rm -rf ~ $(id) `id` " + "あ" * 50_000
    llm = FakeLLM([reply("F hello")])
    t = translate(nasty, llm)
    assert t.ok and nasty in llm.prompts[0]


# ---------------------------------------------------------------- 2. re-asking stops at 3

def test_three_bad_replies_give_ok_false_and_exactly_three_calls():
    llm = FakeLLM([reply("F at_kitchen")] * 10)
    t = translate("x", llm)
    assert t.ok is False and len(llm.prompts) == 3 and len(t.attempts) == 3
    assert "3回" in t.message


def test_mixed_bad_replies_still_stop_at_three():
    llm = FakeLLM([reply("F (hello"), reply("[] hello"), reply("hello2"), reply("F hello")])
    t = translate("x", llm)
    assert not t.ok and len(llm.prompts) == 3


def test_good_reply_on_third_try_is_accepted():
    llm = FakeLLM([reply("F ("), reply("true"), reply("F hello")])
    t = translate("x", llm)
    assert t.ok and t.formula == "F hello" and len(llm.prompts) == 3


def test_good_reply_after_three_bad_is_never_requested():
    llm = FakeLLM([reply("bad!")] * 3 + [reply("F hello")])
    t = translate("x", llm)
    assert not t.ok and len(llm.prompts) == 3


def test_llm_error_propagates_without_extra_calls():
    llm = FakeLLM([reply("F ("), LLMError("boom")])
    with pytest.raises(LLMError):
        translate("x", llm)
    assert len(llm.prompts) == 2


# ---------------------------------------------------------------- 3. CLI argument hygiene

NASTY_PROMPTS = {
    "rm": '"; rm -rf ~',
    "subst": "$(touch /tmp/pwned)",
    "backtick": "`touch /tmp/pwned`",
    "newline": "line1\nline2\r\nline3",
    "mixed": '"; rm -rf ~ $(id) `id` \n && echo hi | cat > /tmp/x',
    "dash": "--version",
    "dash_dash": "-- -p",
    "nul_free_unicode": "止まって！　$HOME ${HOME} ~",
}


def _recording_runner(stdout="", returncode=0, stderr="", on_call=None):
    calls = []

    def run(argv, *args, **kw):
        calls.append((argv, args, kw))
        if on_call:
            on_call(argv)
        return subprocess.CompletedProcess(argv, returncode, stdout, stderr)
    return run, calls


GOOD_CLAUDE_OUT = json.dumps({"is_error": False, "structured_output": reply("F hello")})


@pytest.mark.parametrize("key", sorted(NASTY_PROMPTS))
def test_claude_prompt_never_in_argv_and_no_shell(key):
    prompt = NASTY_PROMPTS[key]
    run, calls = _recording_runner(GOOD_CLAUDE_OUT)
    ClaudeCLI(runner=run)(prompt, SCHEMA)
    argv, args, kw = calls[0]
    assert isinstance(argv, list) and all(isinstance(a, str) for a in argv)
    assert args == ()
    assert not kw.get("shell")
    assert kw["input"] == prompt
    assert not any(prompt in a for a in argv)
    assert not any(a == prompt for a in argv)


@pytest.mark.parametrize("key", sorted(NASTY_PROMPTS))
def test_codex_prompt_never_in_argv_and_no_shell(key):
    prompt = NASTY_PROMPTS[key]
    seen = {}

    def on_call(argv):
        seen["schema"] = Path(argv[argv.index("--output-schema") + 1])
        seen["schema_text"] = seen["schema"].read_text()
        Path(argv[argv.index("-o") + 1]).write_text(json.dumps(reply("F hello")))

    run, calls = _recording_runner(on_call=on_call)
    out = CodexCLI(runner=run)(prompt, SCHEMA)
    argv, args, kw = calls[0]
    assert out == reply("F hello")
    assert isinstance(argv, list) and args == () and not kw.get("shell")
    assert kw["input"] == prompt
    assert not any(prompt in a for a in argv)
    assert argv[-1] == "-"  # prompt is read from stdin
    assert prompt not in seen["schema_text"]


def test_codex_temp_dir_is_removed_after_call():
    seen = {}

    def on_call(argv):
        seen["dir"] = Path(argv[argv.index("-o") + 1]).parent
        Path(argv[argv.index("-o") + 1]).write_text(json.dumps(reply("F hello")))

    run, _ = _recording_runner(on_call=on_call)
    CodexCLI(runner=run)("p", SCHEMA)
    assert not seen["dir"].exists()


def test_codex_temp_dir_is_removed_on_failure():
    seen = {}

    def on_call(argv):
        seen["dir"] = Path(argv[argv.index("-o") + 1]).parent

    run, _ = _recording_runner(returncode=1, stderr="boom", on_call=on_call)
    with pytest.raises(LLMError):
        CodexCLI(runner=run)("p", SCHEMA)
    assert not seen["dir"].exists()


@pytest.mark.parametrize("cls", [ClaudeCLI, CodexCLI])
def test_default_runner_is_subprocess_run_without_shell(cls, monkeypatch):
    captured = {}

    def fake_run(argv, *a, **kw):
        captured["argv"], captured["kw"] = argv, kw
        if cls is CodexCLI:
            Path(argv[argv.index("-o") + 1]).write_text(json.dumps(reply("F hello")))
            return subprocess.CompletedProcess(argv, 0, "", "")
        return subprocess.CompletedProcess(argv, 0, GOOD_CLAUDE_OUT, "")

    monkeypatch.setattr(subprocess, "run", fake_run)
    obj = cls()  # default runner
    obj.runner = subprocess.run  # default arg was bound at import; rebind to the patched one
    assert obj("; rm -rf ~", SCHEMA) == reply("F hello")
    assert isinstance(captured["argv"], list) and not captured["kw"].get("shell")


def test_default_runner_object_is_the_real_subprocess_run():
    assert ClaudeCLI().runner is subprocess.run and CodexCLI().runner is subprocess.run


def test_no_shell_true_in_llm_module_source():
    src = Path(llm_module.__file__).read_text()
    assert "shell=True" not in src and "os.system" not in src


def test_model_name_is_a_single_argv_element():
    run, calls = _recording_runner(GOOD_CLAUDE_OUT)
    ClaudeCLI(model="sonnet; rm -rf ~", runner=run)("p", SCHEMA)
    argv = calls[0][0]
    assert argv[argv.index("--model") + 1] == "sonnet; rm -rf ~"
    assert argv.count("--model") == 1


def test_claude_never_uses_bare_and_disables_tools_for_nasty_prompt():
    run, calls = _recording_runner(GOOD_CLAUDE_OUT)
    ClaudeCLI(runner=run)(NASTY_PROMPTS["mixed"], SCHEMA)
    argv = calls[0][0]
    assert "--bare" not in argv and argv[argv.index("--tools") + 1] == ""


def test_codex_runs_read_only_ephemeral():
    def on_call(argv):
        Path(argv[argv.index("-o") + 1]).write_text(json.dumps(reply("F hello")))
    run, calls = _recording_runner(on_call=on_call)
    CodexCLI(runner=run)("p", SCHEMA)
    argv = calls[0][0]
    assert argv[argv.index("--sandbox") + 1] == "read-only" and "--ephemeral" in argv


# ---------------------------------------------------------------- bad CLI output

@pytest.mark.parametrize("stdout", [
    "", "null", "[]", "42", '"str"', "{", "{}", '{"is_error": false}',
    '{"is_error": false, "structured_output": []}',
    '{"is_error": false, "structured_output": "F hello"}',
    '{"is_error": false, "structured_output": {"formula": 1, "unsupported": []}}',
    '{"is_error": false, "structured_output": {"formula": "F hello"}}',
    '{"is_error": false, "structured_output": {"formula": "F hello", "unsupported": "x"}}',
    '{"is_error": false, "result": 5}',
    '{"is_error": false, "result": "not json"}',
    '{"is_error": true, "result": "boom"}',
])
def test_claude_bad_output_is_llm_error(stdout):
    run, _ = _recording_runner(stdout)
    with pytest.raises(LLMError):
        ClaudeCLI(runner=run)("p", SCHEMA)


@pytest.mark.parametrize("content", ["", "null", "[]", "{", '{"formula": 1, "unsupported": []}'])
def test_codex_bad_output_file_is_llm_error(content):
    def on_call(argv):
        Path(argv[argv.index("-o") + 1]).write_text(content)
    run, _ = _recording_runner(on_call=on_call)
    with pytest.raises(LLMError):
        CodexCLI(runner=run)("p", SCHEMA)


def test_codex_missing_output_file_is_llm_error():
    run, _ = _recording_runner()
    with pytest.raises(LLMError):
        CodexCLI(runner=run)("p", SCHEMA)


def test_missing_command_is_llm_error():
    def run(argv, **kw):
        raise FileNotFoundError(argv[0])
    with pytest.raises(LLMError, match="見つかりません"):
        ClaudeCLI(runner=run)("p", SCHEMA)


def test_garbage_bytes_in_stdout_do_not_crash():
    run, _ = _recording_runner("\x00\x01\ud800 not json")
    with pytest.raises(LLMError):
        ClaudeCLI(runner=run)("p", SCHEMA)


def test_none_stdout_and_stderr_do_not_crash():
    def run(argv, **kw):
        return subprocess.CompletedProcess(argv, 1, None, None)
    with pytest.raises(LLMError):
        ClaudeCLI(runner=run)("p", SCHEMA)


# ---------------------------------------------------------------- 4. secrets

STDERR_WITH_TOKENS = (
    "Error: request failed\n"
    f"Authorization failed for key {FAKE_SK}\n"
    f"env: {FAKE_OAUTH}\n"
    "please retry later\n"
)


def _all_text(exc, logcap):
    return str(exc.value) + repr(exc.value) + logcap.text + "".join(r.getMessage() for r in logcap.records)


@pytest.mark.parametrize("returncode", [1, 2, 127])
def test_claude_failure_does_not_leak_tokens_in_error_or_logs(returncode, logcap):
    run, _ = _recording_runner(returncode=returncode, stderr=STDERR_WITH_TOKENS)
    with pytest.raises(LLMError) as exc:
        ClaudeCLI(runner=run)("p", SCHEMA)
    text = _all_text(exc, logcap)
    for secret in (FAKE_SK, "TESTONLY0000", "TESTONLY1111", FAKE_OAUTH):
        assert secret not in text


def test_claude_failure_log_keeps_harmless_stderr_lines(logcap):
    run, _ = _recording_runner(returncode=1, stderr=STDERR_WITH_TOKENS)
    with pytest.raises(LLMError):
        ClaudeCLI(runner=run)("p", SCHEMA)
    assert "please retry later" in logcap.text


def test_token_in_stdout_of_failed_run_is_not_leaked(logcap):
    run, _ = _recording_runner(stdout=f"token {FAKE_SK}", returncode=1)
    with pytest.raises(LLMError) as exc:
        ClaudeCLI(runner=run)("p", SCHEMA)
    assert FAKE_SK not in _all_text(exc, logcap)


def test_token_in_is_error_result_is_not_leaked(logcap):
    out = json.dumps({"is_error": True, "result": f"bad key {FAKE_SK} {FAKE_OAUTH}"})
    run, _ = _recording_runner(out, stderr=STDERR_WITH_TOKENS)
    with pytest.raises(LLMError) as exc:
        ClaudeCLI(runner=run)("p", SCHEMA)
    text = _all_text(exc, logcap)
    assert "TESTONLY" not in text


def test_token_in_unparsable_stdout_is_not_leaked(logcap):
    run, _ = _recording_runner(stdout=f"oops {FAKE_SK}", stderr=STDERR_WITH_TOKENS)
    with pytest.raises(LLMError) as exc:
        ClaudeCLI(runner=run)("p", SCHEMA)
    assert "TESTONLY" not in _all_text(exc, logcap)


def test_token_on_success_path_is_not_logged(logcap):
    run, _ = _recording_runner(GOOD_CLAUDE_OUT, stderr=STDERR_WITH_TOKENS)
    ClaudeCLI(runner=run)("p", SCHEMA)
    assert "TESTONLY" not in logcap.text


def test_token_in_timeout_error_is_not_leaked(logcap):
    def run(argv, **kw):
        raise subprocess.TimeoutExpired(argv, 5, output=FAKE_SK, stderr=FAKE_OAUTH)
    with pytest.raises(LLMError) as exc:
        ClaudeCLI(runner=run)("p", SCHEMA)
    assert "TESTONLY" not in _all_text(exc, logcap)


def test_token_as_prompt_is_not_logged_on_failure(logcap):
    run, _ = _recording_runner(returncode=1, stderr="boom")
    with pytest.raises(LLMError) as exc:
        ClaudeCLI(runner=run)(f"my key is {FAKE_SK}", SCHEMA)
    assert FAKE_SK not in _all_text(exc, logcap)


def test_codex_failure_does_not_leak_tokens(logcap):
    run, _ = _recording_runner(returncode=1, stderr=STDERR_WITH_TOKENS)
    with pytest.raises(LLMError) as exc:
        CodexCLI(runner=run)("p", SCHEMA)
    assert "TESTONLY" not in _all_text(exc, logcap)


def test_login_problem_message_is_fixed_text_not_stderr():
    run, _ = _recording_runner(returncode=1, stderr=f"Invalid API key {FAKE_SK} please login")
    with pytest.raises(LLMError) as exc:
        ClaudeCLI(runner=run)("p", SCHEMA)
    assert str(exc.value) == llm_module.MSG_LOGIN


def test_stderr_log_is_capped():
    big = "\n".join(f"line {i} " + "x" * 50 for i in range(1000))
    records = []

    class H(logging.Handler):
        def emit(self, r):
            records.append(r.getMessage())
    h = H()
    llm_module.log.addHandler(h)
    try:
        run, _ = _recording_runner(returncode=1, stderr=big)
        with pytest.raises(LLMError):
            ClaudeCLI(runner=run)("p", SCHEMA)
    finally:
        llm_module.log.removeHandler(h)
    assert records and all(len(m) < 400 for m in records)


@pytest.mark.parametrize("line", [
    "ANTHROPIC_API_KEY=TESTONLY2222",
    "ANTHROPIC_AUTH_TOKEN=TESTONLY2222",
    "Authorization: Bearer TESTONLY2222",
    "client_secret: TESTONLY2222",
    "PASSWORD is TESTONLY2222",
    "X-Api-Key: TESTONLY2222",
    "export FOO=TESTONLY2222abcdefghijklmnop",
])
def test_other_credential_shapes_in_stderr_are_not_logged(line, logcap):
    run, _ = _recording_runner(returncode=1, stderr=f"failed\n{line}\n")
    with pytest.raises(LLMError):
        ClaudeCLI(runner=run)("p", SCHEMA)
    assert "TESTONLY2222" not in logcap.text


# ---------------------------------------------------------------- 5. env values

def test_empty_env_values_mean_claude_without_model():
    llm = make_llm({"LLM_BACKEND": "", "LLM_MODEL": ""})
    assert isinstance(llm, ClaudeCLI) and llm.model is None


def test_missing_env_keys_mean_claude_without_model():
    llm = make_llm({})
    assert isinstance(llm, ClaudeCLI) and llm.model is None


def test_none_env_values_mean_defaults():
    llm = make_llm({"LLM_BACKEND": None, "LLM_MODEL": None})
    assert isinstance(llm, ClaudeCLI) and llm.model is None


@pytest.mark.parametrize("backend,cls", [
    ("claude", ClaudeCLI), ("codex", CodexCLI),
    ("CLAUDE", ClaudeCLI), ("Codex", CodexCLI), ("CODEX", CodexCLI),
    (" codex ", CodexCLI), ("\tclaude\n", ClaudeCLI), (" Claude", ClaudeCLI),
])
def test_backend_case_and_whitespace_are_tolerated(backend, cls):
    assert isinstance(make_llm({"LLM_BACKEND": backend}), cls)


def test_model_whitespace_is_stripped_and_blank_means_none():
    assert make_llm({"LLM_MODEL": "  sonnet  "}).model == "sonnet"
    assert make_llm({"LLM_MODEL": "   "}).model is None
    assert make_llm({"LLM_MODEL": "\n"}).model is None


@pytest.mark.parametrize("backend", ["gpt", "openai", "claude codex", "claude;rm", "クロード", "0"])
def test_unknown_backend_is_a_clear_value_error(backend):
    with pytest.raises(ValueError, match="LLM_BACKEND"):
        make_llm({"LLM_BACKEND": backend})


def test_unknown_backend_error_does_not_leak_the_model_value():
    with pytest.raises(ValueError) as exc:
        make_llm({"LLM_BACKEND": "nope", "LLM_MODEL": FAKE_SK})
    assert FAKE_SK not in str(exc.value)


@pytest.mark.parametrize("backend", ["   ", "\t", "\n"])
def test_whitespace_only_backend_means_default_like_empty(backend):
    assert isinstance(make_llm({"LLM_BACKEND": backend}), ClaudeCLI)


# ---------------------------------------------------------------- 6. stop words

# text -> is_stop_command expected today. Documented table; "safe" column explains.
STOP_TABLE = {
    # the words the user listed
    "止まって": True, "ストップ": True, "停止": True, "とまって": True,
    "止まれ": True, "ストップして": True,
    # decorations that voice input often adds
    "止まって！": True, "止まって。": True, "  止まって  ": True, "ストップ！": True,
    "とまれ": True, "止まってください": True, "STOP": True, "Stop!": True, "stop": True,
    "やめて": True, "待って": True, "停止して": True, "今すぐ止まって": True,
    # negations: any text containing a stop word is a stop (fails safe)
    "停止しないで": True, "ストップしないで": True,
    # negation that contains no listed stop word: goes to the LLM, no stop
    "止まらないで": False, "止まるな": False,
    # not stop commands
    "前に進んで": False, "挨拶して": False, "座って": False, "": False, "  ": False,
    "ストッ": False, "ｽﾄｯﾌﾟ": False,
    # longer than 12 characters goes to the LLM even with a stop word (documented design)
    "お願いだから今すぐ止まってください": False,
    "前に進んで、壁の手前で止まって挨拶して": False,
}


@pytest.mark.parametrize("text", sorted(STOP_TABLE, key=lambda s: (STOP_TABLE[s], s)))
def test_is_stop_command_table(text):
    assert is_stop_command(text) is STOP_TABLE[text]


def test_stop_command_never_raises_on_odd_text():
    for text in ["\x00", "\n止まって\n", "止" * 100_000, "　止まって　", "🛑", "止まって" * 3]:
        assert isinstance(is_stop_command(text), bool)


def test_stop_command_for_stopword_embedded_in_longer_word_is_safe_side():
    # 'stopwatch' contains 'stop' and is short: treated as a stop (a spurious halt is the safe error)
    assert is_stop_command("stopwatch") is True


@pytest.mark.xfail(strict=True, reason=(
    "BUG-C3-6: a stop word repeated or padded past 12 characters (e.g. 止まって止まって止まって止まって) "
    "is not an immediate stop; it goes to the LLM instead of StopMove. Design says long = LLM, but a "
    "text consisting only of stop words is an emergency"))
def test_pure_repeated_stop_words_longer_than_12_chars_stop():
    assert is_stop_command("止まって止まって止まって止まって")
