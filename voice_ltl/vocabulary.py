"""物の種類の単語帳 voice_ltl/vocabulary.json を読み、確かめる（open-objects spec §7・§8・§9.4・§9.5）。

標準ライブラリだけを使う（Jazzy の Gazebo ブリッジと検出器も、objects.py を通してこれを読み込む）。
import のときに 1 回読み、§7.3 の決まりに 1 つでも外れたら VocabularyError を出す（fail closed）。
決まり 2 のうち world.py の名前の区切りと、決まり 6（扱う動詞）は、world.py と translator.py が
import のときに check_reserved_segments / check_words で確かめる（このモジュールはそれらを読み込まない）。
エラーの文は「<ファイル>: rule <番号>: <label か ->: <値>: <わけ>」の 1 行（JSON の誤りは行と桁も）。
"""

from __future__ import annotations

import json
import logging
import math
import re
import unicodedata
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType

VOCABULARY_PATH = Path(__file__).with_name("vocabulary.json")
MAX_KINDS = 200
MAX_SIZE = 3.0
DETECT_WARN = 100
LABEL_SHAPE = re.compile(r"(?!(?:cw|sim)(?:_|$))[a-z][a-z0-9]*(?:_[a-z][a-z0-9]*){0,3}")   # fullmatch, len <= 24
MAX_LABEL = 24
FIXED_RESERVED = frozenset({"near", "around", "cw", "to", "last", "true", "false", "end"})
_ONE_CHAR = "箱棚机籠鞄"
ONE_CHAR_WORDS = frozenset(_ONE_CHAR)
SUFFIXES = ("", "まで", "の周り", "のところ", "の前", "を", "に", "へ")        # rule 5
MAX_JA = 12
MAX_WORD = 20
MAX_WORDS = 20
MAX_DETECT = 3
MAX_EN = 8

_LOG = logging.getLogger("voice_ltl.vocabulary")
_ROOT = Path(__file__).resolve().parent.parent
_TOP_KEYS = ("version", "labels", "not_words", "detect_ignore")
_KIND_KEYS = ("label", "ja", "words", "detect", "en", "size", "height")
_KIND_REQUIRED = ("label", "ja", "words", "detect", "size", "height")
_WORD_SHAPE = re.compile(r"[ぁ-ゖァ-ヺー々一-鿿]+")              # kana, kanji, ー and 々 (not the middle dot ・)
_PROMPT_SHAPE = re.compile(r"[a-z][a-z -]{0,39}")


class VocabularyError(ValueError):
    """str(e) == "<file name>: rule <k>: <label or '-'>: <value>: <reason>"."""

    def __init__(self, source: str, rule: int, label: str, value: object, reason: str):
        self.source, self.rule, self.label, self.value, self.reason = source, rule, label, value, reason
        super().__init__(f"{source}: rule {rule}: {_show(label)}: {_show(value)}: {reason}")


@dataclass(frozen=True)
class Kind:
    label: str
    word: str           # in the vocabulary's spelling (チェアー -> チェア)
    start: int          # offsets on the NFKC, masked text
    end: int


@dataclass(frozen=True)
class Entry:
    label: str
    ja: str
    words: tuple[str, ...]
    detect: tuple[str, ...]
    en: tuple[str, ...]
    size: tuple[float, float]       # [long, short] m
    height: float


@dataclass(frozen=True)
class Vocabulary:
    entries: tuple[Entry, ...]
    not_words: tuple[str, ...]
    detect_ignore: tuple[str, ...]
    labels: frozenset[str]
    detect: Mapping[str, str]                       # detect prompt -> label
    source: str = field(default="vocabulary.json", compare=False)     # the file named in errors
    _by_label: Mapping[str, Entry] = field(init=False, repr=False, compare=False)
    _aliases: Mapping[str, str] = field(init=False, repr=False, compare=False)
    _matcher: _Matcher = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        aliases: dict[str, str] = {}
        for e in self.entries:
            for name in (e.label, *e.detect, *e.en):
                aliases.setdefault(normalise_alias(name), e.label)
        object.__setattr__(self, "_by_label", MappingProxyType({e.label: e for e in self.entries}))
        object.__setattr__(self, "_aliases", MappingProxyType(aliases))
        object.__setattr__(self, "_matcher", _Matcher({w: e.label for e in self.entries for w in e.words},
                                                      self.not_words))

    def entry(self, label: str) -> Entry | None:
        return self._by_label.get(label) if isinstance(label, str) else None

    def name_of(self, label: str) -> str:
        """The vocabulary's ja, else the label with spaces for "_". Never raises."""
        e = self.entry(label)
        return e.ja if e is not None else str(label).replace("_", " ")

    def lookup(self, name: object) -> str | None:
        """An English name (Claude's) -> a label of this vocabulary, or None (spec §9.4). Never raises."""
        if not isinstance(name, str):
            return None
        key = normalise_alias(name)
        tries = [key]
        if key.endswith("es"):
            tries.append(key[:-2])
        if key.endswith("s"):
            tries.append(key[:-1])
        return next((self._aliases[k] for k in tries if k in self._aliases), None)

    def kinds_in(self, text: str) -> tuple[Kind, ...]:
        """The vocabulary words in text, in order (spec §9.5): longest at one position, not_words masked."""
        return self._matcher.kinds_in(text) if isinstance(text, str) else ()

    def named_in(self, text: str) -> tuple[Kind, ...]:
        """kinds_in, and also a word that ends in kanji when one of GOING_STEMS follows it, the particle dropped
        (ゴミ箱行って, 机前まで). Only the kind guard's refusal set uses it (translator.named_kinds)."""
        return self._matcher.kinds_in(text, loose=True) if isinstance(text, str) else ()


def normalise_alias(name: str) -> str:
    """NFKC, lower case, strip; runs of white space and "-" become "_"; "_" at either end go."""
    return re.sub(r"[\s-]+", "_", unicodedata.normalize("NFKC", name).lower().strip()).strip("_")


def parse(data: object, source: str = "vocabulary.json") -> Vocabulary:
    """Check spec §7.3 rules 1-5 and 7-11 (in that order, the first failure raises) and build the vocabulary.
    Rule 4 owns "a word is not a not_word"; rule 10 checks only that each not_word holds a word."""
    c = _Checker(source)
    kinds = c.rule1(data)
    if len(kinds) > MAX_KINDS:
        c.fail(11, "-", len(kinds), f"more than {MAX_KINDS} kinds")
    not_words = tuple(data.get("not_words", ()))
    detect_ignore = tuple(data.get("detect_ignore", ()))
    c.rule2(kinds)
    c.rule3(kinds)
    c.rule4(kinds, not_words)
    c.rule5(kinds, not_words)
    c.rule7(kinds, detect_ignore)
    c.rule8(kinds, detect_ignore)
    c.rule9(kinds)
    c.rule10(kinds, not_words)
    entries = tuple(Entry(k["label"], k["ja"], tuple(k["words"]), tuple(k["detect"]), tuple(k.get("en", ())),
                          (float(k["size"][0]), float(k["size"][1])), float(k["height"])) for k in kinds)
    n_detect = sum(len(e.detect) for e in entries)
    if n_detect > DETECT_WARN:
        _LOG.warning("%d detect prompts (more than %d)", n_detect, DETECT_WARN)
    return Vocabulary(entries, not_words, detect_ignore, frozenset(e.label for e in entries),
                      MappingProxyType({p: e.label for e in entries for p in e.detect}), source)


def load(path: Path = VOCABULARY_PATH) -> Vocabulary:
    """Read and parse the file. A file that cannot be read or is not JSON is rule 1 (with line and column)."""
    path = Path(path)
    source = _display(path)
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as e:
        raise VocabularyError(source, 1, "-", path.name, f"cannot read the file ({type(e).__name__})") from None
    try:
        data = json.loads(text, object_pairs_hook=_object_without_repeats)
    except json.JSONDecodeError as e:
        raise VocabularyError(source, 1, "-", f"line {e.lineno} column {e.colno}",
                              f"not valid JSON ({e.msg})") from None
    except _RepeatedKey as e:
        raise VocabularyError(source, 1, "-", e.key, "a key appears twice in one object") from None
    return parse(data, source)


def check_reserved_segments(vocab: Vocabulary, segments: Iterable[str], source: str) -> None:
    """Rule 2, the part from the code: no label segment is one of `segments` (the names in `source`)."""
    reserved = frozenset(segments)
    for e in vocab.entries:
        for seg in e.label.split("_"):
            if seg in reserved:
                raise VocabularyError(vocab.source, 2, e.label, seg, f"a label segment is a name in {source}")


def check_words(vocab: Vocabulary, ok: Callable[[str], bool], rule: int, reason: str) -> None:
    """Rule 6 (and any rule like it): every word passes ok."""
    for e in vocab.entries:
        for w in e.words:
            if not ok(w):
                raise VocabularyError(vocab.source, rule, e.label, w, reason)


# ---- 語の境目（objects.py から移した。objects.py と translator.py がここから使う） ----

_KATAKANA = "ァ-ヺー"                     # the long-vowel mark too, but not the middle dot ・
_HIRAGANA = "ぁ-ゖ"
_KANJI = "々一-鿿"
_PARTICLES_BEFORE = "のとやもをにへがはで"
_PARTICLES_AFTER = "まで|から|[のとやもをにへがはで]"


def _script(ch: str) -> str | None:
    return next((name for name, chars in (("kata", _KATAKANA), ("hira", _HIRAGANA), ("kanji", _KANJI))
                 if re.fullmatch(f"[{chars}]", ch)), None)


# 語の境目は word_start / word_end の一か所だけで決める（助詞の一覧もここだけ）。

def word_start(word: str) -> str:
    """word の前に置く正規表現：word がそこで語として始まるときだけ当たる（長さ 0）。
    カタカナで始まる語はカタカナの後ろでは始まらない。ひらがなで始まる語は、ひらがなでない字の後ろか
    助詞の後ろで始まる（このいす は可、かわいすぎる は不可）。漢字で始まる語はどこでも始まる。
    物の名前（label_in）と、ひらがなの「持つ・押す…」（translator の OBJECT_ACTION_KANA）が使う。"""
    return {"kata": f"(?<![{_KATAKANA}])",
            "hira": f"(?:(?<![{_HIRAGANA}])|(?<=[{_PARTICLES_BEFORE}]))"}.get(_script(word[0]), "")


# 助詞を落として漢字の動詞が続くときの、行き先の語の後ろの漢字（ゴミ箱行って・机前まで。pre-final review の
# 2 回目、指摘 1）。物の種類の見張り（translator の named_kinds）の断る側だけが named_in で使う。kinds_in と
# word_end の既定は変えない（箱根 は 箱 ではない）。
GOING_STEMS = ("目指", "行", "向", "寄", "移", "進", "歩", "近", "回", "来", "前", "横", "周")


def word_end(word: str, after: str = _PARTICLES_AFTER, kanji_after: str | None = None) -> str:
    """word の後ろに置く正規表現：word がそこで語として終わるときだけ当たる。カタカナで終わる語は
    カタカナと続かない（チェアー は可）。ひらがなで終わる語は、ひらがなでない字か after（正規表現、
    既定は助詞）と続く（いすまで は可、いすわる は不可）。漢字で終わる語は漢字と続かない（箱根 は不可）。
    ただし kanji_after（正規表現）があれば、漢字で終わる語はそれとも続く（named_in の GOING_STEMS）。"""
    kanji = f"(?![{_KANJI}])" if kanji_after is None else f"(?:(?![{_KANJI}])|(?={kanji_after}))"
    return {"kata": f"ー?(?![{_KATAKANA}])",
            "hira": f"(?:(?={after})|(?![{_HIRAGANA}]))",
            "kanji": kanji}.get(_script(word[-1]), "")


def _word_pattern(word: str, kanji_after: str | None = None) -> str:
    """word が長い語の一部でないときだけ当たる正規表現。"""
    return word_start(word) + re.escape(word) + word_end(word, kanji_after=kanji_after)


# ---- 内部 ----

class _Matcher:
    """All words in one alternation, longest first (spec §9.5). finditer left to right on the NFKC text with
    every not_word masked by as many 「・」 (a boundary, so the offsets do not move)."""

    def __init__(self, word_label: Mapping[str, str], not_words: Iterable[str]):
        self.word_label = dict(word_label)
        # longest first; one length keeps the file order (dict, not set: no PYTHONHASHSEED dependence)
        self.not_words = sorted(dict.fromkeys(w for w in not_words if isinstance(w, str) and w), key=len,
                                reverse=True)
        words = sorted(self.word_label, key=len, reverse=True)
        self.pattern = re.compile("|".join(_word_pattern(w) for w in words) if words else r"(?!)")
        stems = "|".join(GOING_STEMS)
        self.loose = re.compile("|".join(_word_pattern(w, stems) for w in words) if words else r"(?!)")

    def kinds_in(self, text: str, loose: bool = False) -> tuple[Kind, ...]:
        """loose: the named_in pattern (a kanji word may end before one of GOING_STEMS)."""
        masked = unicodedata.normalize("NFKC", text)
        for w in self.not_words:
            masked = masked.replace(w, "・" * len(w))
        out = []
        for m in (self.loose if loose else self.pattern).finditer(masked):
            word = m.group()
            if word not in self.word_label and word.endswith("ー"):
                word = word[:-1]                     # チェアー -> チェア (word_end lets a ー follow)
            label = self.word_label.get(word)
            if label is not None:
                out.append(Kind(label, word, m.start(), m.end()))
        return tuple(out)

    def search(self, text: str) -> bool:
        return self.pattern.search(text) is not None


class _RepeatedKey(Exception):
    def __init__(self, key: str):
        super().__init__(key)
        self.key = key


def _object_without_repeats(pairs: list[tuple[str, object]]) -> dict:
    out: dict = {}
    for k, v in pairs:
        if k in out:
            raise _RepeatedKey(k)
        out[k] = v
    return out


def _display(path: Path) -> str:
    """The file as errors name it: relative to the repository when inside it."""
    try:
        return path.resolve().relative_to(_ROOT).as_posix()
    except (OSError, ValueError):
        return str(path)


def _show(value: object) -> str:
    """A value on one line: strings as they are, others as JSON; control characters escaped; at most 80."""
    if isinstance(value, str):
        text = value
    else:
        try:
            text = json.dumps(value, ensure_ascii=False)
        except (TypeError, ValueError):
            text = repr(value)
    text = "".join(ch if unicodedata.category(ch)[0] != "C" else ch.encode("unicode_escape").decode("ascii")
                   for ch in text)
    return text if len(text) <= 80 else text[:79] + "…"


def _number(v: object) -> bool:
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return False
    try:
        return math.isfinite(v)
    except OverflowError:
        return False


def _is_word_shaped(w: object, longest: int) -> bool:
    return (isinstance(w, str) and 1 <= len(w) <= longest and _WORD_SHAPE.fullmatch(w) is not None
            and unicodedata.normalize("NFKC", w) == w)


class _Checker:
    """The rules of spec §7.3, one method each; fail() raises the first failure."""

    def __init__(self, source: str):
        self.source = source

    def fail(self, rule: int, label: object, value: object, reason: str) -> None:
        raise VocabularyError(self.source, rule, label if isinstance(label, str) else "-", value, reason)

    @staticmethod
    def _label(kind: dict) -> object:
        return kind.get("label", "-")

    def rule1(self, data: object) -> list[dict]:
        """version 1, no unknown or missing keys, and the containers have the right types."""
        if not isinstance(data, dict):
            self.fail(1, "-", type(data).__name__, "the file must hold one JSON object")
        for key in data:
            if key not in _TOP_KEYS:
                self.fail(1, "-", key, f"unknown key (the keys are {', '.join(_TOP_KEYS)})")
        for key in ("version", "labels"):
            if key not in data:
                self.fail(1, "-", key, "missing key")
        version = data["version"]
        if not (isinstance(version, int) and not isinstance(version, bool) and version == 1):
            self.fail(1, "-", version, "version must be 1")
        for key in ("labels", "not_words", "detect_ignore"):
            if key in data and not isinstance(data[key], list):
                self.fail(1, "-", key, "must be a list")
        for key in ("not_words", "detect_ignore"):
            for item in data.get(key, ()):
                if not isinstance(item, str):
                    self.fail(1, "-", item, f"{key} must hold strings")
        for kind in data["labels"]:
            if not isinstance(kind, dict):
                self.fail(1, "-", kind, "each kind must be a JSON object")
            for key in kind:
                if key not in _KIND_KEYS:
                    self.fail(1, self._label(kind), key, f"unknown key (the keys are {', '.join(_KIND_KEYS)})")
            for key in _KIND_REQUIRED:
                if key not in kind:
                    self.fail(1, self._label(kind), key, "missing key")
            for key in ("words", "detect", "en", "size"):
                if key in kind and not isinstance(kind[key], list):
                    self.fail(1, self._label(kind), key, "must be a list")
        return data["labels"]

    def rule2(self, kinds: list[dict]) -> None:
        seen: set[str] = set()
        for kind in kinds:
            label = kind["label"]
            if not isinstance(label, str) or LABEL_SHAPE.fullmatch(label) is None:
                self.fail(2, label, label, "a label is 1-4 segments [a-z][a-z0-9]* joined by _, "
                                           "the first not cw or sim")
            if len(label) > MAX_LABEL:
                self.fail(2, label, label, f"longer than {MAX_LABEL} characters")
            for seg in label.split("_"):
                if seg in FIXED_RESERVED:
                    self.fail(2, label, seg, "a label segment is a reserved word")
            if label in seen:
                self.fail(2, label, label, "a second kind with this label")
            seen.add(label)
        for label in ("box", "chair"):
            if label not in seen:
                self.fail(2, label, label, "this kind must be in the vocabulary")

    def rule3(self, kinds: list[dict]) -> None:
        seen: dict[str, str] = {}
        for kind in kinds:
            label, ja = kind["label"], kind["ja"]
            if not _is_word_shaped(ja, MAX_JA):
                self.fail(3, label, ja, f"ja is 1-{MAX_JA} kana or kanji (with ー and 々), unchanged by NFKC")
            if ja in seen:
                self.fail(3, label, ja, f"also the ja of {seen[ja]}")
            seen[ja] = label
            if ja not in kind["words"]:
                self.fail(3, label, ja, "ja must be one of the kind's words")

    def rule4(self, kinds: list[dict], not_words: tuple[str, ...]) -> None:
        owner: dict[str, str] = {}
        masks = set(not_words)
        for kind in kinds:
            label, words = kind["label"], kind["words"]
            if not 1 <= len(words) <= MAX_WORDS:
                self.fail(4, label, len(words), f"a kind has 1-{MAX_WORDS} words")
            for w in words:
                if not _is_word_shaped(w, MAX_WORD):
                    self.fail(4, label, w, f"a word is 1-{MAX_WORD} kana or kanji (with ー and 々), "
                                           "unchanged by NFKC")
                if len(w) == 1 and w not in ONE_CHAR_WORDS:
                    self.fail(4, label, w, f"one-character words are only {'・'.join(_ONE_CHAR)}")
                if w in owner:
                    self.fail(4, label, w, "a second time in this kind" if owner[w] == label
                              else f"also a word of {owner[w]}")
                if w in masks:
                    self.fail(4, label, w, "also a not_word")
                owner[w] = label

    def rule5(self, kinds: list[dict], not_words: tuple[str, ...]) -> None:
        matcher = _Matcher({w: kind["label"] for kind in kinds for w in kind["words"]}, not_words)
        for kind in kinds:
            for w in kind["words"]:
                for suffix in SUFFIXES:
                    found = matcher.kinds_in(w + suffix)
                    got = found[-1].label if found else None
                    if got != kind["label"]:
                        self.fail(5, kind["label"], w + suffix, f"resolves to {got or 'no kind'}")

    def rule7(self, kinds: list[dict], detect_ignore: tuple[str, ...]) -> None:
        if len(set(detect_ignore)) != len(detect_ignore):
            self.fail(7, "-", list(detect_ignore), "detect_ignore holds a prompt twice")
        for p in detect_ignore:
            if _PROMPT_SHAPE.fullmatch(p) is None:
                self.fail(7, "-", p, "a prompt is [a-z][a-z -]{0,39}")
        owner: dict[str, str] = {}
        for kind in kinds:
            label, prompts = kind["label"], kind["detect"]
            if not 1 <= len(prompts) <= MAX_DETECT:
                self.fail(7, label, len(prompts), f"a kind has 1-{MAX_DETECT} detect prompts")
            for p in prompts:
                if not isinstance(p, str) or _PROMPT_SHAPE.fullmatch(p) is None:
                    self.fail(7, label, p, "a prompt is [a-z][a-z -]{0,39}")
                if p in owner:
                    self.fail(7, label, p, "a second time in this kind" if owner[p] == label
                              else f"also a detect prompt of {owner[p]}")
                if p in detect_ignore:
                    self.fail(7, label, p, "also in detect_ignore")
                owner[p] = label

    def rule8(self, kinds: list[dict], detect_ignore: tuple[str, ...]) -> None:
        ignored = {normalise_alias(p) for p in detect_ignore}
        owner: dict[str, str] = {}
        for kind in kinds:
            label, en = kind["label"], kind.get("en", [])
            if len(en) > MAX_EN:
                self.fail(8, label, len(en), f"a kind has at most {MAX_EN} en names")
            for i, name in enumerate(en):
                if not isinstance(name, str) or _PROMPT_SHAPE.fullmatch(name) is None:
                    self.fail(8, label, name, "an en name is [a-z][a-z -]{0,39}")
                if name in en[:i]:
                    self.fail(8, label, name, "a second time in this kind")
            for name in (label, *kind["detect"], *en):
                alias = normalise_alias(name)
                if alias in owner and owner[alias] != label:
                    self.fail(8, label, name, f"also an alias of {owner[alias]}")
                if alias in ignored:
                    self.fail(8, label, name, "an alias in detect_ignore")
                owner[alias] = label

    def rule9(self, kinds: list[dict]) -> None:
        for kind in kinds:
            label, size, height = kind["label"], kind["size"], kind["height"]
            if not (len(size) == 2 and all(_number(v) and 0 < v <= MAX_SIZE for v in size)):
                self.fail(9, label, size, f"size is [long, short], each in (0, {MAX_SIZE:g}] m")
            if size[0] < size[1]:
                self.fail(9, label, size, "size is [long, short]: the long side first")
            if not (_number(height) and 0 < height <= MAX_SIZE):
                self.fail(9, label, height, f"height is in (0, {MAX_SIZE:g}] m")

    def rule10(self, kinds: list[dict], not_words: tuple[str, ...]) -> None:
        matcher = _Matcher({w: kind["label"] for kind in kinds for w in kind["words"]}, ())
        for i, w in enumerate(not_words):
            if not _is_word_shaped(w, MAX_WORD):
                self.fail(10, "-", w, f"a not_word is 1-{MAX_WORD} kana or kanji (with ー and 々), "
                                      "unchanged by NFKC")
            if w in not_words[:i]:
                self.fail(10, "-", w, "a second time in not_words")
            if not matcher.search(w):
                self.fail(10, "-", w, "holds no word, so it hides nothing")


VOCABULARY: Vocabulary = load()                                # at import: fail closed
LABELS = VOCABULARY.labels
DETECT = VOCABULARY.detect
DETECT_IGNORE = frozenset(VOCABULARY.detect_ignore)
name_of = VOCABULARY.name_of
lookup = VOCABULARY.lookup
kinds_in = VOCABULARY.kinds_in
named_in = VOCABULARY.named_in


def size_of(label: str) -> tuple[float, float] | None:
    e = VOCABULARY.entry(label)
    return e.size if e is not None else None


def height_of(label: str) -> float | None:
    e = VOCABULARY.entry(label)
    return e.height if e is not None else None
