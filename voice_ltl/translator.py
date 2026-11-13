"""Instruction -> LTLf formula, with up to max_tries re-asks on rejected formulas."""

import dataclasses
import math
import re
import unicodedata
from collections.abc import Sequence
from dataclasses import dataclass, field

from voice_ltl import vocabulary
from voice_ltl.grammar import check
from voice_ltl.llm import LLM
from voice_ltl.objects import (ID_RE, MAX_OBJECT_DISTANCE, Obj, Scene, direction, formula_object_props,
                               kinds_in, labels_in, msg_ambiguous, msg_not_found, msg_not_listed, msg_too_far,
                               split_prop, word_end, word_start)
from voice_ltl.prompt import SCHEMA, build_prompt
from voice_ltl.world import MAX_REPEAT, PROPOSITIONS, STEP_ANGLE, STEP_DISTANCE

# An unsupported part that is a number with a unit (0メートル, -2度): design §7.4.0. The prompt
# rounds off-unit numbers and caps big ones, so this is for what is still left (0, minus) or for an
# LLM that does not follow it. Digits without a unit (2階, 3番目の部屋) are a word not in the vocabulary.
_NUMBER = re.compile(r"(?:マイナス|[-−－])?\s*(?:\d+(?:[.．]\d+)?|[〇一二三四五六七八九十百千]+|ゼロ|零)\s*"
                     r"(?:メートル|センチ|ミリ|cm|mm|m|ｍ|度|°|回|歩|周|秒間|秒|分間|分|時間)(?![A-Za-z])")


@dataclass(frozen=True)
class Entry:
    """One accepted item of the reply's unresolved_objects (open-objects spec §9.3)."""
    said: str                   # words of the instruction
    label: str | None           # the vocabulary label vocabulary.lookup accepted, or None


@dataclass
class Attempt:
    formula: str | None
    unsupported: list[str]
    errors: list[str]
    unresolved: list[dict] = field(default_factory=list)       # [{"said": ..., "label": ... or None}]


@dataclass
class Translation:
    ok: bool
    formula: str | None
    unsupported: list[str]
    attempts: list[Attempt] = field(default_factory=list)
    message: str = ""
    # what was read differently from the words, for the page: [{"said": ..., "as": ...}]
    substitutions: list[dict[str, str]] = field(default_factory=list)
    # the objects and pose frozen for this instruction (design §6.3); None for a link without objects
    scene: Scene | None = None


MAX_SUBSTITUTIONS = 10
MAX_UNRESOLVED = 10
MAX_UNRESOLVED_LABEL = 64


def _plain(text: str) -> str:
    return re.sub(r"\s+", "", unicodedata.normalize("NFKC", text))


_TRAILING_NOTE = re.compile(r"\s*[（(][^（()）]*[）)]\s*$")


def _substitutions(raw: object, instruction: str) -> list[dict[str, str]]:
    """The reply's substitutions as {said, as} strings. The CLI's JSON Schema already requires
    them (a reply without the key is unreadable, design §7.4.1); this guards replies that come
    another way (FakeLLM, run_formula). `said` must be words of the instruction: a note the LLM
    added in brackets is cut off, and a said copied from the prompt's examples is dropped.
    The page clips them after hiding secrets (session.py)."""
    if not isinstance(raw, list):
        return []
    words = _plain(instruction)
    out = []
    for item in raw[:MAX_SUBSTITUTIONS]:
        if isinstance(item, dict) and isinstance(item.get("said"), str) and isinstance(item.get("as"), str):
            said, as_ = _TRAILING_NOTE.sub("", item["said"]).strip(), item["as"].strip()
            if said and as_ and _plain(said) in words:
                out.append({"said": said, "as": as_})
    return out


def _unresolved(raw: object, instruction: str) -> list[Entry]:
    """The reply's unresolved_objects (open-objects spec §9.3). The CLIs' JSON Schema requires the key;
    this guards replies that come another way (FakeLLM: a missing key reads as []). Only the first
    MAX_UNRESOLVED items are read, and a bad item is dropped (never re-asked). `said` must be words of the
    instruction (a note in brackets is cut off, a copy of an example is dropped), once each. `label` is
    Claude's English: only vocabulary.lookup's answer is kept (None for no kind, or for more than
    MAX_UNRESOLVED_LABEL characters), so Claude's text never reaches the page, the prompt or a message."""
    if not isinstance(raw, list):
        return []
    words = _plain(instruction)
    out: list[Entry] = []
    seen: set[str] = set()
    for item in raw[:MAX_UNRESOLVED]:
        if not (isinstance(item, dict) and isinstance(item.get("said"), str) and isinstance(item.get("label"), str)):
            continue
        said = _TRAILING_NOTE.sub("", item["said"]).strip()
        key = _plain(said)
        if not key or key not in words or key in seen:
            continue
        seen.add(key)
        label = item["label"]
        out.append(Entry(said, None if len(label) > MAX_UNRESOLVED_LABEL else vocabulary.lookup(label)))
    return out


def _shown(entries: Sequence[Entry]) -> list[dict]:
    return [{"said": e.said, "label": e.label} for e in entries]


def _is_number(item: str) -> bool:
    # Only an item that IS a number with a unit (0.7メートル, 約45度) gets the number message.
    # 「障害物の周りを一周する」 contains 一周 but is a behavior the vocabulary lacks.
    return re.fullmatch(r"\s*(?:約|およそ)?\s*(?:" + _NUMBER.pattern + r")\s*(?:だけ)?\s*", item) is not None


# Handling an object (design §3: not this step). An item with one of these is never an object's name,
# so 「箱を持つ」 keeps the 単語帳 message however many boxes there are (R16).
OBJECT_ACTION_WORDS = ("持", "押", "運", "拾", "動か", "どけ", "どか")
# The same verbs in hiragana, by their stems:
#   (plain, te-form, i-stem, a-stem, imperative)
OBJECT_ACTION_KANA = (
    ("もつ", "もって", "もち", "もた", "もて"),
    ("おす", "おして", "おし", "おさ", "おせ"),
    ("はこぶ", "はこんで", "はこび", "はこば", "はこべ"),
    ("ひろう", "ひろって", "ひろい", "ひろわ", "ひろえ"),
    ("うごかす", "うごかして", "うごかし", "うごかさ", "うごかせ"),
    ("どける", "どけて", "どけ", "どけ", "どけろ"),
    ("どかす", "どかして", "どかし", "どかさ", "どかせ"),
)
# Each form starts a word as a label word does (objects.word_start: after a non-hiragana character or a
# particle, so おもって, なおして and おもて are not hits).
# - These go on with anything: the te-form (もってきて, おしてください), the polite form (おします,
#   もちません), the negative (おさないで, もたない) and 〜上げる (もちあげて). An i-stem alone (もちろん,
#   おしえて, おしまい, ひろい) is not a hit.
# - The plain form ends there or goes on with な or の (おすな, もつの), and the imperative ends there or
#   goes on with よ (はこべ, どけろよ): おすわり, おすすめ, もてる and もてなし are not hits. もっと never
#   holds もつ or もって.
_KANA_GO_ON = tuple(f for _, te, i, a, _ in OBJECT_ACTION_KANA
                    for f in (te, i + "ます", i + "まし", i + "ませ", i + "あげ", a + "ない", a + "なく", a + "なかっ"))
_KANA_ACTION = re.compile("|".join(
    [word_start(f) + re.escape(f) for f in _KANA_GO_ON]
    + [word_start(p) + re.escape(p) + word_end(p, after="[なの]") for p, *_ in OBJECT_ACTION_KANA]
    + [word_start(m) + re.escape(m) + word_end(m, after="よ") for *_, m in OBJECT_ACTION_KANA]))
# Words that hold one of those forms after a particle-like letter but are other words: でもって (and
# then), 通す (とおす, とおして, ...) and 荷物 (にもつ). They are taken out before the search, so 「手で
# もって」 is not a hit either (only the refusal message differs, R16; 「手で持って」 is).
_NOT_HANDLING = re.compile("でもって|とお[さしすせそ]|にもつ")


def _handles_object(item: str) -> bool:
    return (any(w in item for w in OBJECT_ACTION_WORDS)
            or _KANA_ACTION.search(_NOT_HANDLING.sub("・", item)) is not None)


# No vocabulary word reads as a handling verb (open-objects spec §7.3 rule 6): checked at import, fail closed.
vocabulary.check_words(vocabulary.VOCABULARY, lambda w: not _handles_object(w), 6, "reads as a handling verb")


# ---- object words (open-objects spec §9.6-§9.9: R16 rewritten, O34; the kind guard, O35) ----

# The going and circling phrases after an object word (a closed list: the Go2 has only near and around).
# fullmatch on the stripped rest of an item; .match on an instruction clause. Every part is optional, so it
# also matches "": a clause counts only when the match holds more than a bare particle (_going).
_PLACE_WORDS = ("ところ", "所", "前", "そば", "横", "近く", "方", "周り", "まわり", "周囲")
_GOING_VERBS = ("歩く", "歩いて", "あるく", "あるいて", "行く", "行って", "いく", "いって", "進む", "進んで", "近づく",
                "近づいて", "ちかづく", "ちかづいて", "向かう", "向かって", "回る", "回って", "まわる", "まわって")
# The kind guard's exemption (O35) only: more going phrases (pre-final review round 2, finding 3). §9.6's list for
# the count messages stays as it is, so 「ゴミ箱に行け」 still keeps the 単語帳 message.
_GUARD_PLACE_WORDS = _PLACE_WORDS + ("ほう", "とこ")
_GUARD_VERBS = _GOING_VERBS + ("行け", "行きなさい", "進め", "向かえ", "近づけ", "歩け", "回れ", "移動して", "移動する",
                               "移動", "寄って", "寄れ", "近寄って", "近寄れ", "来て", "目指して", "目指せ", "向けて")


def _tail(places: Sequence[str], verbs: Sequence[str]) -> re.Pattern:
    def alternation(words: Sequence[str]) -> str:     # longest first: .match keeps the first that fits (とこ, ところ)
        return "|".join(sorted(words, key=len, reverse=True))
    return re.compile(rf"(?:の?(?:{alternation(places)}))?(?:まで|へ|に|を)?"
                      rf"(?:(?:ぐるっと|ぐるりと)?(?:{alternation(verbs)}"
                      rf"|(?:[0-9]+|[一二三四五六七八九十]+)周(?:する|して)?)){{0,2}}")


_GOING_TAIL = _tail(_PLACE_WORDS, _GOING_VERBS)                # spec §9.6
_GUARD_TAIL = _tail(_GUARD_PLACE_WORDS, _GUARD_VERBS)          # O35's exemption only
_BARE = ("", "を", "に", "へ")                  # spec §9.6 rule 3: a bare particle is no going phrase
_CLAUSE_END = re.compile(r"[、。,.，．！!？?]")
_COORDINATORS = "とや、・"
_DESCRIBED = re.compile(r"[^\s、。,.，．！!？?の]{1,6}\s*の\s*")   # P の in X の P の Y (named_kinds)
MAX_COORDINATION = 4


def _squeezed(text: str) -> str:
    return re.sub(r"\s+", "", text)


def _nfkc(text: str) -> str:
    return unicodedata.normalize("NFKC", text)


def _loose(said: str) -> re.Pattern:
    """said's letters (NFKC, no white space) with any white space between them: the said in a text."""
    return re.compile(r"\s*".join(re.escape(ch) for ch in _plain(said)))


def _going_phrase(said: str) -> bool:
    """A said that is itself a going phrase (前, ところ, 前まで行って) is no object word (pre-final review round 2,
    finding 4): else 「ゴミ箱の前」 with {前, box} would read the trash can as a describer of 前."""
    return _GUARD_TAIL.fullmatch(_squeezed(_nfkc(said))) is not None


class _Clauses:
    """spec §9.6 rule 3 over one instruction. `text` is its NFKC form with the white space kept, so positions
    match `kinds` (find of that text: kinds_in, or vocabulary.named_in for the guard's refusal set). The object
    words are those vocabulary words and the entries' saids, except a said that is a going phrase. `tail` is
    the going phrases (§9.6's list, or the guard's wider one). `passes` is memoised on (end, depth): a long
    chain of coordinated words or nested saids looks at each word end once per step, not once per path
    (fix round 1)."""

    def __init__(self, instruction: str, entries: Sequence[Entry] = (), tail: re.Pattern | None = None,
                 find=kinds_in):
        self.text = _nfkc(instruction)
        self.kinds = find(self.text)
        self._tail = _GOING_TAIL if tail is None else tail
        self._saids = [_loose(e.said) for e in entries if not _going_phrase(e.said)]
        self._starts: dict[int, list[int]] = {}
        for k in self.kinds:
            self._starts.setdefault(k.start, []).append(k.end)
        self._memo: dict[tuple[int, int], bool] = {}

    def _after_spaces(self, pos: int) -> int:
        while pos < len(self.text) and self.text[pos].isspace():
            pos += 1
        return pos

    def _words_at(self, pos: int) -> list[int]:
        """The ends of the object words that start at pos."""
        ends = list(self._starts.get(pos, ()))
        for said in self._saids:
            m = said.match(self.text, pos)
            if m is not None and m.group():
                ends.append(m.end())
        return ends

    def _going(self, end: int) -> bool:
        """The clause after `end` (up to the next 、。,.，．！!？?, without white space) starts with a going or
        circling phrase that is more than a bare particle, and that phrase is not followed by の and another
        object word: in X の P の Y (「冷蔵庫の横の椅子まで」) the place word belongs to Y, so X is no target
        (spec §9.8)."""
        stop = _CLAUSE_END.search(self.text, end)
        going = self._tail.match(_squeezed(self.text[end:stop.start() if stop else len(self.text)])).group()
        if going in _BARE:
            return False
        pos = end
        for _ in going:                               # step over the matched letters and the spaces among them
            pos = self._after_spaces(pos) + 1
        pos = self._after_spaces(pos)
        return not (self.text.startswith("の", pos) and self._words_at(self._after_spaces(pos + 1)))

    def passes(self, end: int, depth: int = 0) -> bool:
        """At one occurrence (the object word ends at `end`): its clause starts with a going or circling phrase,
        or with と・や・、・・ and another object word that passes the same test (MAX_COORDINATION steps at most)."""
        key = (end, depth)
        if key not in self._memo:
            self._memo[key] = self._going(end) or (depth < MAX_COORDINATION and self._coordinated(end, depth))
        return self._memo[key]

    def _coordinated(self, end: int, depth: int) -> bool:
        pos = self._after_spaces(end)
        if pos >= len(self.text) or self.text[pos] not in _COORDINATORS:
            return False
        return any(self.passes(e, depth + 1) for e in self._words_at(self._after_spaces(pos + 1)))

    def describes(self, end: int) -> bool:
        """The object word that ends at `end` is followed by の and another object word, or by の, a short word
        P (no clause end, no の) and の and another object word: X の Y, X の P の Y (「椅子の上の箱」)."""
        pos = self._after_spaces(end)
        if not self.text.startswith("の", pos):
            return False
        pos = self._after_spaces(pos + 1)
        if self._words_at(pos):
            return True
        between = _DESCRIBED.match(self.text, pos)
        return between is not None and bool(self._words_at(between.end()))


def _targets(clauses: _Clauses, entries: Sequence[Entry]) -> dict[str, str]:
    """label -> word for each object word whose clause passes, in instruction order (the first word per kind).
    A vocabulary word gives its kind and word; an entry's said (D3) gives its kind by §9.7 (_said_kind) and its
    head word, else the said (pre-final review round 2, finding 2)."""
    found = [(k.end, k.label, k.word) for k in clauses.kinds if clauses.passes(k.end)]
    for e in entries:
        kind = _said_kind(e)
        if kind is None or _going_phrase(e.said):
            continue
        ends = [m.end() for m in _loose(e.said).finditer(clauses.text) if m.group() and clauses.passes(m.end())]
        if ends:
            found.append((ends[0], kind[0], kind[1] or e.said))
    out: dict[str, str] = {}
    for _, label, word in sorted(found, key=lambda f: f[0]):
        out.setdefault(label, word)
    return out


def going_targets(instruction: str, entries: Sequence[Entry] = ()) -> dict[str, str]:
    """The kinds the instruction names as a place to go to or to circle (spec §9.6 rule 3, §9.8): label ->
    the user's vocabulary word (the first that passes, in the vocabulary's spelling), in instruction order.
    An entry whose said passes is a target too (D3), by §9.7's kind."""
    return _targets(_Clauses(instruction, entries), entries)


def _guard_targets(instruction: str, entries: Sequence[Entry] = ()) -> dict[str, str]:
    """going_targets with the guard's wider list of going phrases (_GUARD_TAIL): O35's exemption only."""
    return _targets(_Clauses(instruction, entries, tail=_GUARD_TAIL), entries)


def named_kinds(instruction: str, entries: Sequence[Entry] = ()) -> dict[str, str]:
    """Every kind the instruction names, whatever the phrasing (the kind guard fails closed; pre-final review
    F1): label -> the user's first such word, in instruction order. The words are vocabulary.named_in's, so a
    word that ends in kanji counts before a kanji verb with the particle dropped (ゴミ箱行って; round 2,
    finding 1). Only a word that describes another object word (X の Y, X の P の Y: 「冷蔵庫の横の椅子」 names
    the chair) is left out."""
    clauses = _Clauses(instruction, entries, find=vocabulary.named_in)
    out: dict[str, str] = {}
    for k in clauses.kinds:
        if k.label not in out and not clauses.describes(k.end):
            out[k.label] = k.word
    return out


def _said_kind(e: Entry) -> tuple[str, str | None] | None:
    """spec §9.7 for an entry: the said's own head word when it ends the said or is followed by a going tail
    (that word's kind and word), else Claude's accepted label (named ja), else no kind."""
    said = _nfkc(e.said)
    heads = kinds_in(said)
    if heads and _GOING_TAIL.fullmatch(_squeezed(said[heads[-1].end:])):
        return heads[-1].label, heads[-1].word
    return (e.label, None) if e.label is not None else None


def _object_kind(item: str, entries: Sequence[Entry], clauses: _Clauses) -> tuple[str, str | None] | None:
    """spec §9.6 and §9.7: (label, the user's word or None for ja) when the item names an object to go to or to
    circle, else None (the item keeps the 単語帳 message). `clauses` is the instruction's, with the same entries."""
    if _handles_object(item):
        return None
    text = _nfkc(item)
    kinds = kinds_in(text)
    end, word, said = (kinds[-1].end, kinds[-1], None) if kinds else (-1, None, None)
    for e in entries:                                 # the last object word; on a tie the said wins
        found = [m for m in _loose(e.said).finditer(text) if m.group()]
        if found and found[-1].end() >= end:
            end, word, said = found[-1].end(), None, e
    if word is None and said is None:
        return None
    rest = _squeezed(text[end:])
    if not _GOING_TAIL.fullmatch(rest):
        return None
    if not rest:                                      # a name alone: its clause in the instruction decides
        if said is not None:
            ends = [m.end() for m in _loose(said.said).finditer(clauses.text) if m.group()]
        else:
            ends = [k.end for k in clauses.kinds if k.word == word.word]
        if ends and not any(clauses.passes(e) for e in ends):
            return None                               # a word that is not in the instruction is trusted
    return _said_kind(said) if said is not None else (word.label, word.word)


def _distance(obj: Obj, scene: Scene) -> float:
    return math.hypot(obj.x - scene.pose[0], obj.y - scene.pose[1])


def _count_message(label: str, word: str | None, scene: Scene, listed: Sequence[Obj]) -> str | None:
    """spec §9.9: the message for one kind, from the whole Scene, the listed objects and the frozen pose;
    None when exactly one of the kind is listed (the item keeps the 単語帳 message)."""
    of_kind = [o for o in scene.objects if o.label == label]
    if not of_kind:
        return msg_not_found(label, word)
    if scene.pose is not None and all(_distance(o, scene) > MAX_OBJECT_DISTANCE for o in of_kind):
        return msg_too_far(label, word)
    shown = [o for o in listed if o.label == label]
    if len(shown) >= 2:
        return msg_ambiguous(label, len(of_kind), [direction(o, scene.pose) for o in shown], word)
    if not shown:
        return msg_not_listed(label, word)
    return None


def _object_messages(items: list[str], view: Scene | None, entries: Sequence[Entry],
                     instruction: str) -> tuple[list[str], list[str]]:
    """R16 (rewritten, spec §9.6-§9.9): the count message of each item that names an object to go to or to
    circle, each kind told once. Returns (those messages, the other items)."""
    scene = view if view is not None else Scene()
    listed = scene.listed()
    messages: list[str] = []
    rest: list[str] = []
    told: set[str] = set()
    clauses = _Clauses(instruction, entries)          # one memo for every item
    for item in items:
        kind = _object_kind(item, entries, clauses)
        if kind is None:
            rest.append(item)
            continue
        label, word = kind
        if label in told:
            continue
        message = _count_message(label, word, scene, listed)
        if message is None:
            rest.append(item)
        else:
            told.add(label)
            messages.append(message)
    return messages, rest


def _unsupported_message(items: list[str], view: Scene | None = None, entries: Sequence[Entry] = (),
                         instruction: str = "") -> str:
    parts, unsupported = _object_messages(items, view, entries, instruction)
    numbers = [u for u in unsupported if _is_number(u)]
    words = [u for u in unsupported if u not in numbers]
    if words or not (numbers or parts):
        what = "、".join(words) if words else "この指示"
        parts.append(f"「{what}」は単語帳にないので、まだできません。")
    if numbers:
        parts.append(f"「{'、'.join(numbers)}」：この数は指定できません"
                     f"（0・マイナスの数と、秒・分などの時間は使えません。"
                     f"{STEP_DISTANCE:g} m・{STEP_ANGLE:g} 度・1 回を単位に、{MAX_REPEAT} 回まで）。")
    return "".join(parts)


def _formula_labels(formula: str) -> set[str]:
    """The labels of the objects the formula uses."""
    out = set()
    for prop in formula_object_props(formula):
        m = ID_RE.fullmatch(split_prop(prop)[1])
        if m is not None:
            out.add(m.group(1))
    return out


def _kind_guard(formula: str, view: Scene | None, instruction: str, entries: Sequence[Entry] = ()) -> list[str]:
    """O35 (spec §9.8): when the formula uses an object whose kind is not a named target (_guard_targets: §9.6's
    closed list and the guard's extra going phrases, vocabulary words and entries) and some kind the
    instruction names (named_kinds, any phrasing) has no object in the Scene, the 見つかりません message of each
    such kind, once; [] if fine. A phrasing off both lists therefore refuses rather than walks to another kind
    (fails closed; pre-final review F1 and round 2)."""
    if view is None:
        return []
    named = named_kinds(instruction, entries)
    if not named:
        return []
    targets = _guard_targets(instruction, entries)
    if all(label in targets for label in _formula_labels(formula)):
        return []
    return [msg_not_found(label, word) for label, word in named.items()
            if not any(o.label == label for o in view.objects)]


def _users_word(label: str, instruction: str) -> str | None:
    """The user's vocabulary word for that kind (the first, in the vocabulary's spelling), else None (ja)."""
    return next((k.word for k in kinds_in(instruction) if k.label == label), None)


def _too_far(formula: str, scene: Scene | None) -> str | None:
    """P5: the label of the first object in the formula whose centre is more than MAX_OBJECT_DISTANCE
    from the frozen pose, so the Go2 does not stand up first. The grid checks again from its anchor."""
    if scene is None or scene.pose is None:
        return None
    for name in re.findall(r"\w+", formula):
        prop = split_prop(name)
        obj = scene.find(prop[1]) if prop else None
        if obj is not None and math.hypot(obj.x - scene.pose[0], obj.y - scene.pose[1]) > MAX_OBJECT_DISTANCE:
            return obj.label
    return None


def translate(instruction: str, llm: LLM, max_tries: int = 3, scene: Scene | None = None) -> Translation:
    """LLMError is not caught here; the caller handles it. `scene` is the object list frozen for this
    instruction: the kinds the instruction names are listed first (focus, spec §9.1), the listed objects'
    propositions join the vocabulary, and the frozen Scene is returned on every Translation.
    Order (spec §9.8): a refusal gets its messages from the items and the reply's unresolved_objects; a formula
    that passes the check is refused for an entry of a kind it does not use (O8), then by the kind guard (O35),
    then for a far object (P5)."""
    labels = labels_in(instruction)
    view = scene if scene is None or not labels else dataclasses.replace(scene, focus=labels)
    propositions = PROPOSITIONS + (view.propositions() if view is not None else ())
    attempts: list[Attempt] = []
    rejected: str | None = None
    error: str | None = None
    earlier_subs: list[dict[str, str]] = []   # from a rejected answer: kept if the fix drops them
    for _ in range(max_tries):
        reply = llm(build_prompt(instruction, rejected, error, scene=view), SCHEMA)
        entries = _unresolved(reply.get("unresolved_objects"), instruction)
        formula = reply.get("formula")
        raw = reply.get("unsupported")
        if raw is None or raw == "":
            raw = []
        elif isinstance(raw, str):
            raw = [raw]
        if not isinstance(raw, list):
            errors = [f"unsupported は文字列の配列で渡してください（{type(raw).__name__} が渡されました）。"]
            attempts.append(Attempt(formula if isinstance(formula, str) else None, [], errors, _shown(entries)))
            rejected, error = str(formula), " ".join(errors)
            continue
        unsupported = [str(u) for u in raw]
        if unsupported or formula is None:
            items = unsupported + [e.said for e in entries
                                   if not any(_plain(e.said) in _plain(u) for u in unsupported)]
            attempts.append(Attempt(formula, items, [], _shown(entries)))
            return Translation(False, None, items, attempts,
                               _unsupported_message(items, view, entries, instruction), scene=scene)
        errors = check(formula, propositions)
        subs = _substitutions(reply.get("substitutions"), instruction)
        if not errors:
            used = _formula_labels(formula)
            # O8, the entry's kind by §9.7 (its said's head word first; pre-final review F5)
            open_items = [e.said for e in entries if (kind := _said_kind(e)) is None or kind[0] not in used]
            attempts.append(Attempt(formula, open_items, [], _shown(entries)))
            if open_items:
                return Translation(False, None, open_items, attempts,
                                   _unsupported_message(open_items, view, entries, instruction), scene=scene)
            missing = _kind_guard(formula, view, instruction, entries)                           # O35
            if missing:
                return Translation(False, None, [], attempts, "".join(missing), scene=scene)
            far = _too_far(formula, view)
            if far is not None:
                return Translation(False, None, [], attempts, msg_too_far(far, _users_word(far, instruction)),
                                   scene=scene)
            return Translation(True, formula, [], attempts, "", subs or earlier_subs, scene=scene)
        attempts.append(Attempt(formula, [], errors, _shown(entries)))
        earlier_subs = subs or earlier_subs
        rejected, error = formula, " ".join(errors)
    if attempts and any("長すぎ" in e for e in attempts[-1].errors):
        return Translation(False, None, [], attempts,
                           "指示が長すぎて式にできませんでした。いくつかに分けて言ってください。", scene=scene)
    return Translation(
        False, None, [], attempts,
        f"指示を式に直せませんでした（{max_tries}回試しました）。言い方を変えてもう一度お願いします。",
        scene=scene,
    )
