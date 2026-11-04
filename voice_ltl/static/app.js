"use strict";
// The page (design: docs/reference/ui-design): push to talk (Space, or holding the talk
// button), the WebSocket view of Session.snapshot(), the messages, the diagrams, the top
// view and the buttons. Texts for the user are Japanese.

const MIME = "audio/webm;codecs=opus";
const STOP_DISTANCE = 0.5;   // [m] perception.py stop_distance
const HALF_WIDTH = 0.3;      // [m] perception.py half_width
const BODY = { length: 0.7, width: 0.31 };  // Go2 [m]
const GRID = 0.2;            // [m] one square of the top view
const VIEW_HALF = 1.6;       // [m] the top view shows this far to the left and right of the Go2

const POSTURE_JA = { LYING: "伏せ", STANDING: "立ち", SITTING: "おすわり", UNKNOWN: "不明" };
const SENSE_JA = { CLEAR: "なし", BLOCKED: "あり", SENSOR_LOST: "途切れ" };
const SENSE_TONE = { CLEAR: "ok", BLOCKED: "stop", SENSOR_LOST: "warn" };
const STATUS = {  // Session status -> the pill under the talk button
  idle: ["指示を待っています", "neutral"], transcribing: ["書き起こし中", "accent"],
  thinking: ["考えています", "accent"], executing: ["実行中", "accent"],
  waiting: ["待っています", "warn"], done: ["終わりました", "ok"], error: ["できませんでした", "stop"],
};
const TAB_JA = { FOCUS: "計画の道", SUP: "監督器", TRIMMED: "trim 後", SPEC: "要求", PLANT: "環境" };
// snapshot link_kind -> [the mode (hidden on a phone), what the page drives]
const LINK_JA = { fake: ["シミュレーション", "にせ Go2"], gazebo: ["シミュレーション", "Gazebo"], robot: [null, "実機"] };
const LLM_JA = { claude: "Claude", codex: "Codex" };  // snapshot llm_backend
const STEP_STATE = { done: "done", current: "now", pending: "todo" };  // snapshot steps -> CSS
const LOGIN_TITLE = "LLM にログインできていません";  // the notice's title (its message is longer)
// A recording shorter than this, or a smaller one (a webm of its header alone), holds no
// instruction: it is not sent, so a tap leaves what the session does as it was (UI-3).
const MIN_SECONDS = 0.3;
const MIN_BYTES = 2048;

const ICON = {  // static markup only; texts always go in with textContent
  check: '<svg width="18" height="18" viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M2.5 7.5l3 3 6-7"/></svg>',
  clock: '<svg width="18" height="18" viewBox="0 0 18 18" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><circle cx="9" cy="9" r="7"/><path d="M9 5v4.5l3 1.5"/></svg>',
  bang: '<svg width="18" height="18" viewBox="0 0 18 18" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><circle cx="9" cy="9" r="7"/><path d="M9 5.5v4"/><path d="M9 12.5h.01"/></svg>',
  cross: '<svg width="18" height="18" viewBox="0 0 18 18" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><circle cx="9" cy="9" r="7"/><path d="M6 6l6 6"/></svg>',
  square: '<svg width="16" height="16" viewBox="0 0 14 14"><rect x="2" y="2" width="10" height="10" rx="2" fill="currentColor"/></svg>',
  plus: '<svg width="18" height="18" viewBox="0 0 18 18" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><path d="M4 9h10"/><path d="M9 4v10"/></svg>',
  stepDone: '<svg width="16" height="16" viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M2.5 7.5l3 3 6-7"/></svg>',
  stepNow: '<svg width="16" height="16" viewBox="0 0 16 16" aria-hidden="true"><circle cx="8" cy="8" r="6" fill="none" stroke="currentColor" stroke-width="2" stroke-dasharray="22 16"/></svg>',
};

const $ = (id) => document.getElementById(id);

function setText(id, text) {
  const node = $(id);
  if (node.textContent !== text) node.textContent = text;
}

const shown = {};  // what each part last drew, so an unchanged part is not redrawn
function changed(key, value) {
  const s = JSON.stringify(value);
  if (shown[key] === s) return false;
  shown[key] = s;
  return true;
}

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

const meters = (d) => (d === null || d === undefined ? "—" : `${d.toFixed(2)} m`);

// ---------------- push to talk ----------------
// ptt.recording is true while a recording runs; ptt.starts counts the recordings started;
// ptt.take is the latest recording: { seconds } it lasted, set when it stops.
const ptt = { recording: false, starts: 0, stream: null, recorder: null, asking: false, since: 0, take: null };
window.ptt = ptt;

let serverStatus = "idle";  // the last snapshot's status
let recordTimer = null;

const TALK = {
  idle: {
    label: "押している間、話せます", title: "スペースを押しながら話す",
    sub: "離すと、そこまでを1つの指示として送ります",
  },
  recording: { label: "録音中。離すと送ります", title: "聞いています" },
  thinking: { label: "考えています。押している間、新しい指示を話せます", title: "考えています" },
};

function talkState() {
  if (ptt.recording) return "recording";
  return serverStatus === "transcribing" || serverStatus === "thinking" ? "thinking" : "idle";
}

function renderTalk() {
  const state = talkState();
  const talk = TALK[state];
  setData("talk", "state", state);
  const button = $("talk-button");
  if (button.getAttribute("aria-label") !== talk.label) button.setAttribute("aria-label", talk.label);
  setText("talk-title", talk.title);
  if (state === "recording") {
    const s = Math.floor((Date.now() - ptt.since) / 1000);
    setText("talk-sub", `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")} · 離すと送ります`);
  } else if (state === "idle") {
    setText("talk-sub", talk.sub);
  }
  $("talk-sub").hidden = state === "thinking";
  $("steps").hidden = state !== "thinking";
  $("status").hidden = state === "thinking";  // the title already says it
  if (state === "thinking") {
    // transcribing: the Whisper part; thinking: translation and synthesis (one step to Session)
    const now = serverStatus === "transcribing" ? 0 : 1;
    document.querySelectorAll("#steps li").forEach((li, i) => {
      li.dataset.state = i < now ? "done" : i === now ? "now" : "todo";
    });
  }
}

let hintTimer = null;
function showHint(text) {
  $("hint").textContent = text;
  clearTimeout(hintTimer);
  hintTimer = setTimeout(() => { $("hint").textContent = ""; }, 6000);
}

async function askMicrophone() {
  if (ptt.stream || ptt.asking) return;
  if (!navigator.mediaDevices || !window.MediaRecorder) {
    showHint("このブラウザではマイクが使えません（http://localhost で開いてください）");
    return;
  }
  ptt.asking = true;
  try {
    ptt.stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    $("mic-text").textContent = "マイク：使えます";
    $("mic").disabled = true;
  } catch (e) {
    showHint("マイクが使えません（ブラウザの許可を確かめてください）");
  } finally {
    ptt.asking = false;
  }
}

function startRecording() {
  if (ptt.recording) return;
  if (!ptt.stream) {
    showHint("先に「マイクを使う」を押してください");
    return;
  }
  let rec;
  try {
    rec = new MediaRecorder(ptt.stream, MediaRecorder.isTypeSupported(MIME) ? { mimeType: MIME } : {});
    rec.start();
  } catch (e) {  // e.g. the microphone was unplugged
    showHint("録音を始められませんでした（マイクを確かめて、ページを読み直してください）");
    return;
  }
  const chunks = [];
  const take = { seconds: 0 };
  rec.ondataavailable = (e) => { if (e.data.size) chunks.push(e.data); };
  rec.onstop = () => {
    const blob = new Blob(chunks, { type: rec.mimeType || MIME });
    if (take.seconds < MIN_SECONDS || blob.size < MIN_BYTES) {  // a tap: say so, send nothing
      showHint("短すぎて聞き取れませんでした");
      return;
    }
    sendAudio(blob);
  };
  ptt.recorder = rec;
  ptt.take = take;
  ptt.recording = true;
  ptt.starts += 1;
  ptt.since = Date.now();
  recordTimer = setInterval(renderTalk, 250);
  renderTalk();
}

function stopRecording() {
  if (!ptt.recording) return;
  ptt.recording = false;
  ptt.take.seconds = (Date.now() - ptt.since) / 1000;
  ptt.recorder.stop();  // onstop sends what was recorded
  ptt.recorder = null;
  clearInterval(recordTimer);
  renderTalk();
}

async function sendAudio(blob) {
  try {
    const r = await fetch("/api/audio", {
      method: "POST", headers: { "content-type": blob.type || MIME }, body: blob,
    });
    if (r.status !== 202) showHint(`音声を送れませんでした（${r.status}）`);
  } catch (e) {
    showHint("音声を送れませんでした（サーバーとつながっていません）");
  }
}

function isTyping(target) {
  return target instanceof HTMLElement
    && (target.isContentEditable || ["INPUT", "TEXTAREA", "SELECT"].includes(target.tagName));
}

// Space talks everywhere but in a text field: it never presses a focused button or tab.
window.addEventListener("keydown", (e) => {
  if (e.code !== "Space" || isTyping(e.target)) return;
  e.preventDefault();             // no page scroll and no button press, key repeats too
  if (!e.repeat) startRecording();
});
window.addEventListener("keyup", (e) => {
  if (e.code !== "Space") return;
  if (!isTyping(e.target)) e.preventDefault();
  stopRecording();
});
window.addEventListener("blur", stopRecording);  // the key-up would never come

// Holding the talk button with the mouse or a finger does the same as holding Space.
const talkButton = $("talk-button");
talkButton.addEventListener("pointerdown", (e) => {
  if (e.button !== 0) return;
  e.preventDefault();
  talkButton.setPointerCapture?.(e.pointerId);
  startRecording();
});
for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
  talkButton.addEventListener(type, stopRecording);
}
talkButton.addEventListener("contextmenu", (e) => e.preventDefault());  // long press on a phone

// ---------------- buttons ----------------
async function post(path, body) {
  try {
    const r = await fetch(path, {
      method: "POST",
      headers: body === undefined ? {} : { "content-type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
    if (!r.ok) showHint(`送れませんでした（${r.status}）`);
  } catch (e) {
    showHint("送れませんでした（サーバーとつながっていません）");
  }
}

// A mouse click leaves no focus behind; Enter on a focused button keeps it (keyboard use).
function onClick(id, fn) {
  $(id).addEventListener("click", (e) => {
    if (e.detail > 0) e.currentTarget.blur();
    fn();
  });
}

onClick("mic", askMicrophone);
onClick("stop", () => post("/api/stop"));
onClick("box-front", () => post("/api/obstacle", { cmd: "front" }));
onClick("box-back", () => post("/api/obstacle", { cmd: "back" }));
onClick("box-clear", () => post("/api/obstacle", { cmd: "clear" }));

$("text-form").addEventListener("submit", (e) => {
  e.preventDefault();
  const input = $("text-input");
  const text = input.value.trim();
  if (!text) return;
  post("/api/text", { text });
  input.value = "";
  input.blur();  // so that Space talks again
});

// ---------------- diagrams ----------------
let diagramTab = "FOCUS";
let diagramVersion = null;
let diagramNames = [];  // the diagrams the session has drawn
const tabs = [...document.querySelectorAll('[role="tab"]')];

// The SVG is fetched as text and shown from a blob: URL, so that the same text tells where
// the current state (FOCUS, SUP) or the start state is: the box scrolls there, because a big
// diagram at full size would otherwise open on an empty corner.
let diagramTicket = 0;     // the newest request; an older answer is dropped
let diagramUrl = null;     // the blob: URL shown now
let diagramFocus = null;   // where to scroll once the picture is loaded
let diagramTouched = 0;    // when the user last scrolled the box (Date.now())

async function loadDiagram(follow = true) {
  const ticket = ++diagramTicket;
  const name = diagramTab;
  let text = null;
  if (diagramNames.includes(name)) {
    try {
      const r = await fetch(`/api/diagram/${name}.svg?v=${diagramVersion}`, { cache: "no-store" });
      if (r.ok) text = await r.text();
    } catch (e) { /* shown as no picture */ }
  }
  if (ticket !== diagramTicket) return;
  const img = $("diagram");
  if (text === null) {
    img.removeAttribute("src");
    img.hidden = true;
    $("diagram-empty").hidden = false;
    return;
  }
  diagramFocus = follow ? focusPoint(text) : null;
  const old = diagramUrl;
  diagramUrl = URL.createObjectURL(new Blob([text], { type: "image/svg+xml" }));
  img.alt = `状態遷移図（${TAB_JA[name]}）`;
  img.src = diagramUrl;
  if (old) URL.revokeObjectURL(old);
}

// The highlighted state (diagrams.py marks it class "hl"), else state 0, as fractions of the
// picture's width and height; null when the SVG does not have the graphviz layout looked for.
function focusPoint(text) {
  const doc = new DOMParser().parseFromString(text, "image/svg+xml");
  const vb = (doc.documentElement.getAttribute("viewBox") || "").trim().split(/[\s,]+/).map(Number);
  const m = /scale\(([-\d.]+)(?:[ ,]+([-\d.]+))?\).*translate\(([-\d.]+)[ ,]+([-\d.]+)\)/
    .exec(doc.querySelector("g.graph")?.getAttribute("transform") ?? "");
  const node = doc.querySelector("g.node.hl")
    ?? [...doc.querySelectorAll("g.node")].find((g) => g.querySelector("title")?.textContent === "0");
  const shape = node?.querySelector("ellipse");
  if (vb.length !== 4 || !vb[2] || !vb[3] || !m || !shape) return null;
  const kx = Number(m[1]), ky = Number(m[2] || m[1]);
  const x = kx * (Number(shape.getAttribute("cx")) + Number(m[3]));
  const y = ky * (Number(shape.getAttribute("cy")) + Number(m[4]));
  return { fx: (x - vb[0]) / vb[2], fy: (y - vb[1]) / vb[3] };
}

$("diagram").addEventListener("load", () => {
  const img = $("diagram"), box = $("diagram-panel");
  img.hidden = false;
  $("diagram-empty").hidden = true;
  if (diagramFocus) {
    box.scrollLeft = img.offsetLeft + diagramFocus.fx * img.clientWidth - box.clientWidth / 2;
    box.scrollTop = img.offsetTop + diagramFocus.fy * img.clientHeight - box.clientHeight / 2;
  }
});
$("diagram").addEventListener("error", () => {
  $("diagram").hidden = true;
  $("diagram-empty").hidden = false;
});
for (const type of ["wheel", "pointerdown", "touchstart", "keydown"]) {
  $("diagram-panel").addEventListener(type, () => { diagramTouched = Date.now(); }, { passive: true });
}

function selectTab(tab) {
  diagramTab = tab.dataset.name;
  for (const t of tabs) {
    t.setAttribute("aria-selected", String(t === tab));
    t.tabIndex = t === tab ? 0 : -1;
  }
  $("diagram-panel").setAttribute("aria-labelledby", tab.id);
  loadDiagram();
}
tabs.forEach((tab, i) => {
  tab.addEventListener("click", (e) => {
    if (e.detail > 0) tab.blur();
    selectTab(tab);
  });
  tab.addEventListener("keydown", (e) => {  // arrow keys move between the tabs (Space talks)
    const to = { ArrowRight: i + 1, ArrowLeft: i - 1, Home: 0, End: tabs.length - 1 }[e.key];
    if (to === undefined) return;
    e.preventDefault();
    const next = tabs[(to + tabs.length) % tabs.length];
    selectTab(next);
    next.focus();
  });
});
onClick("fit", () => {
  const on = $("fit").getAttribute("aria-pressed") !== "true";
  $("fit").setAttribute("aria-pressed", String(on));
  $("diagram-panel").classList.toggle("fit", on);
});

// ---------------- view ----------------
function renderStatus(text, tone) {
  setText("status-text", text);
  setData("status", "tone", tone);
}

function setData(id, key, value) {
  const node = $(id);
  if (node.dataset[key] !== value) node.dataset[key] = value;
}

function renderLink(kind) {
  const [mode, name] = LINK_JA[kind] ?? [null, String(kind)];
  if (changed("link", [mode, name])) {
    const node = $("link-text");
    node.replaceChildren();
    if (mode) node.append(el("span", "link-mode", `${mode} · `));
    node.append(name);
  }
  setData("link", "tone", "ok");
  // Design §5: on a real Go2 (or anything but a known simulator) the caution stays; a
  // simulator gets the one small line.
  const simulating = kind === "fake" || kind === "gazebo";
  $("caution").hidden = simulating;
  $("caution-sim").hidden = !simulating;
}

function renderInstruction(s) {
  const empty = !s.transcript;
  setData("transcript", "empty", String(empty));
  setText("transcript", empty ? "まだ指示はありません" : `「${s.transcript}」`);
  // stt: the Whisper model and seconds of this transcript; null when it was typed
  const caption = s.stt
    ? ["書き起こし", s.stt.model && `Whisper ${s.stt.model}`, `${s.stt.seconds.toFixed(1)} 秒`]
      .filter(Boolean).join(" · ") +
      (s.stt.load_seconds ? `（初回の読み込み ${s.stt.load_seconds.toFixed(1)} 秒）` : "")
    : empty ? "書き起こした文" : "文字で入力した文";
  setText("transcript-caption", caption);
}

function attemptOk(a) {
  return a.formula !== null && a.unsupported.length === 0 && a.errors.length === 0;
}

// What the LLM read as a nearby vocabulary action or number (design §7.4.1): 「走って」→ 前へ歩く（0.5 m）
function substitutionLines(s) {
  return s.formula ? (s.substitutions ?? []).map((x) => `「${x.said}」→ ${x.as}`) : [];
}

function renderLogic(s) {
  const attempts = s.attempts;
  const llm = LLM_JA[s.llm_backend] ?? "LLM";
  setText("formula", s.formula ?? "—");
  const lines = substitutionLines(s);
  $("substitutions-box").hidden = lines.length === 0;
  $("substitutions").replaceChildren(...lines.map((t) => el("li", "substitution", t)));
  const verdict = $("verdict");
  verdict.hidden = attempts.length === 0;
  if (s.formula && attempts.length) {
    verdict.dataset.tone = "ok";
    verdict.innerHTML = ICON.check;
    verdict.append(`${attempts.length} 回目で合格`);
    setText("formula-caption", `${llm} が翻訳 · 単語帳と文法の検査に合格`);
  } else if (attempts.length) {
    verdict.dataset.tone = "fail";
    verdict.innerHTML = ICON.cross;
    verdict.append("合格した式はありません");
    setText("formula-caption", `${llm} が翻訳 · 却下の理由は「LLM の試行」に`);
  } else {
    setText("formula-caption", `${llm} が指示を式に翻訳します`);
  }

  $("attempts-box").hidden = attempts.length === 0;
  const rejected = attempts.filter((a) => !attemptOk(a)).length;
  setText("attempts-count", attempts.length
    ? ` · ${attempts.length} 回${rejected ? `（却下 ${rejected}）` : ""}` : "");
  const list = $("attempts");
  list.replaceChildren();
  attempts.forEach((a, i) => {
    const ok = attemptOk(a);
    const reason = a.unsupported.length
      ? `式で表せない部分：${a.unsupported.join("、")}`
      : a.formula === null ? "式がありません" : a.errors.join(" ");
    const li = el("li", "attempt");
    li.append(el("span", "attempt-no", `${i + 1} 回目`), el("code", "attempt-formula", a.formula ?? "null"));
    const v = el("span", "attempt-verdict", ok ? "合格" : "不合格");
    if (!ok) v.dataset.tone = "fail";
    li.append(v);
    if (reason) li.append(el("span", "attempt-reason", reason));
    list.append(li);
  });
}

// steps: the snapshot's whole plan of this instruction, each done, current or pending.
function renderPlan(steps) {
  const ol = $("plan");
  ol.replaceChildren();
  for (const st of steps) {
    const state = STEP_STATE[st.status] ?? "todo";
    const li = el("li");
    const step = el("span", "step");
    step.dataset.state = state;
    step.title = st.action;
    if (state === "done") step.insertAdjacentHTML("beforeend", ICON.stepDone);
    if (state === "now") step.insertAdjacentHTML("beforeend", ICON.stepNow);
    step.append(st.label);
    if (state === "now") step.append(el("span", "step-note", "実行中"));
    if (state === "done") step.append(el("span", "visually-hidden", "（済み）"));
    if (state === "todo") step.append(el("span", "visually-hidden", "（まだ）"));
    li.append(step);
    ol.append(li);
  }
  $("plan-empty").hidden = steps.length > 0;
  const left = steps.filter((st) => st.status !== "done").length;
  setText("plan-meta", left ? `監督器の上の最短の道 · 残り ${left} 手`
    : steps.length ? `実行した動作 · ${steps.length} 手` : "");
}

// ---------------- messages ----------------
// {kind, role, title, body, code, before, after}: before/after go around the body and change
// every second (the distance, the time left); they are shown but kept out of the live region,
// so a screen reader does not read the notice again each time.

// The seconds left before the wait ends the plan: waiting_since and server_now (both the
// server's clock, so the browser's clock does not matter) and block_timeout come from the snapshot.
function secondsLeft(s) {
  const waited = s.server_now - s.waiting_since;
  return Math.min(s.block_timeout, Math.max(0, Math.ceil(s.block_timeout - waited)));
}

// The side that keeps the Go2 waiting: for a move the executor holds, the side it looks at
// (a turn is held only while a sensor is lost); while the session waits, a side that is not
// clear. Only standing counts (R24).
function blockedSide(s) {
  if (s.posture !== "STANDING") return null;
  if (s.current === "forward") return s.front !== "CLEAR" ? "front" : null;
  if (s.current === "backward") return s.back !== "CLEAR" ? "back" : null;
  if (s.current) return s.front === "SENSOR_LOST" ? "front" : s.back === "SENSOR_LOST" ? "back" : null;
  if (s.front !== "CLEAR") return "front";
  return s.back !== "CLEAR" ? "back" : null;
}

// The kind comes from the server (snapshot message_kind: server.py message_kind compares the
// message with Session's own MSG_*), so no message text is matched here.
function noticeFor(s) {
  const m = s.message;
  if (s.waiting_since !== null) {  // the session waits, or the executor holds a move
    const left = secondsLeft(s);
    const side = blockedSide(s);
    if (side === null) {
      return { kind: "waiting", role: "status", title: m || "待っています",
        body: "進める動作ができれば続きを進みます", after: `（あと ${left} 秒で中止）。` };
    }
    const where = side === "front" ? "前" : "後ろ";
    const dist = side === "front" ? s.front_dist : s.back_dist;
    const lost = (side === "front" ? s.front : s.back) === "SENSOR_LOST";
    return {
      kind: "waiting", role: "status",
      title: lost ? "センサーが途切れているので待っています" : `${where}に物があるので待っています`,
      body: lost ? "センサーが戻れば続きを進みます" : "物がなくなれば続きを進みます",
      before: `${side === "front" ? "前方" : "後方"} ${meters(dist)}。`,
      after: `（あと ${left} 秒で中止）。`,
    };
  }
  if (!m) return null;
  switch (s.message_kind) {
    case "stopped":
      return { kind: "stop", role: "alert", title: m,
        body: s.stop_word ? `「${s.stop_word}」を聞いて、すぐに StopMove を送りました。`
          : "StopMove を送りました。" };
    case "done": {
      const did = s.steps.filter((st) => st.status === "done").map((st) => st.label).join(" → ");
      return { kind: "done", role: "status", title: m,
        body: did ? `${did} を、計画どおりに実行しました。`
          : "指示はもう満たされていたので、動かずに終わりました。" };
    }
    case "unsat":
      return { kind: "unsat", role: "alert", title: m,
        body: "監督器が空になりました。Go2 は動いていません。", code: s.formula };
    case "login":
      return { kind: "login", role: "alert", title: LOGIN_TITLE,
        body: "README の手順でログインしてください。Go2 は動いていません。" };
    case "unheard":  // 「聞き取れませんでした」「音声を文字にできませんでした」: the plan, if any, goes on
      return { kind: "heard", role: "status", title: m, body: "もう一度、スペースを押しながら話してください。" };
    case "busy":  // R27/R28: a gesture or posture change ends first; nothing to say again
      return { kind: "waiting", role: "status", title: m,
        body: "そのあと、この指示を始めます。もう一度話す必要はありません。" };
    case "waiting":
      return { kind: "waiting", role: "status", title: m, body: "進める動作ができれば続きを進みます。" };
    case "error": {
      const sentence = /[。）)]$/.test(m) ? m : `${m}。`;
      return s.sup_state !== null  // a supervisor was made: the plan had started
        ? { kind: "cannot", role: "alert", title: "続けられませんでした", body: sentence }
        : { kind: "cannot", role: "alert", title: "この指示はできません",
          body: `${sentence}Go2 は動いていません。` };
    }
    default:  // "info": a message of no kind above
      return { kind: "info", role: "status", title: m, body: "" };
  }
}

const NOTICE_ICON = {
  done: ICON.check, waiting: ICON.clock, cannot: ICON.bang, unsat: ICON.cross,
  stop: ICON.square, login: ICON.plus, heard: ICON.bang, info: ICON.bang,
};

function renderNotice(n) {
  const fixed = n && [n.kind, n.role, n.title, n.body, n.code ?? null];
  if (changed("notice", fixed)) {
    $("notice-status").replaceChildren();
    $("notice-alert").replaceChildren();
    if (n) {
      const box = el("div", "notice");
      box.dataset.kind = n.kind;
      const icon = el("span", "notice-icon");
      icon.setAttribute("aria-hidden", "true");
      icon.innerHTML = NOTICE_ICON[n.kind];
      const text = el("div", "notice-text");
      text.append(el("span", "notice-title", n.title));
      const body = el("span", "notice-body");
      body.append(n.body);
      if (n.before !== undefined || n.after !== undefined) {
        for (const [where, cls] of [["prepend", "notice-before"], ["append", "notice-after"]]) {
          const part = el("span", cls);
          part.setAttribute("aria-hidden", "true");
          body[where](part);
        }
      }
      text.append(body);
      if (n.code) text.append(el("code", "notice-code", n.code));
      box.append(icon, text);
      $(n.role === "alert" ? "notice-alert" : "notice-status").append(box);
    }
  }
  for (const [cls, text] of [["notice-before", n?.before], ["notice-after", n?.after]]) {
    const part = document.querySelector(`.${cls}`);
    if (part && part.textContent !== (text ?? "")) part.textContent = text ?? "";
  }
}

// ---------------- log ----------------
// Two lists: "log" (the snapshot's log, in words) and, under 詳しく, "debug-log" (debug_log,
// the session's own lines). The snapshot has no times: each line gets the time this page
// first saw it (lines from before the page opened get none).
const logSeen = {};  // list id -> { lines, times } of its last render

function stampLog(id, lines) {
  const seen = logSeen[id];
  if (seen === undefined) return lines.map(() => null);
  const prev = seen.lines;
  let shift = 0;  // lines that fell off the front of the server's log since the last render
  while (shift < prev.length && !prev.slice(shift).every((line, i) => line === lines[i])) shift++;
  const kept = seen.times.slice(shift, shift + lines.length);
  const now = new Date();
  return lines.map((_, i) => (i < kept.length ? kept[i] : now));
}

function clock(t) {
  return [t.getHours(), t.getMinutes(), t.getSeconds()].map((n) => String(n).padStart(2, "0")).join(":");
}

function renderLog(id, lines) {
  const times = stampLog(id, lines);
  logSeen[id] = { lines, times };
  const ol = $(id);
  const atEnd = ol.scrollTop + ol.clientHeight >= ol.scrollHeight - 4;
  ol.replaceChildren(...lines.map((line, i) => {
    const li = el("li");
    if (times[i]) {
      const t = el("time", "", clock(times[i]));
      t.dateTime = times[i].toISOString();
      li.append(t);
    } else {
      li.append(el("span", "no-time", "--:--:--"));
    }
    li.append(el("span", "line", line));
    return li;
  }));
  if (atEnd) ol.scrollTop = ol.scrollHeight;
}
for (const id of ["log", "debug-log"]) {  // opened: show the newest lines
  $(id).closest("details").addEventListener("toggle", (e) => {
    if (e.currentTarget.open) $(id).scrollTop = $(id).scrollHeight;
  });
}

// ---------------- Go2 ----------------
function renderSense(id, state, dist) {
  setText(`${id}-dist`, meters(dist));
  setText(id, SENSE_JA[state] ?? state);
  setData(id, "tone", SENSE_TONE[state] ?? "off");
}

function renderGo2(s) {
  setText("posture", POSTURE_JA[s.posture] ?? s.posture);
  renderSense("front", s.front, s.front_dist);
  renderSense("back", s.back, s.back_dist);
  const now = s.steps.find((st) => st.status === "current");
  setText("current", now ? `${now.label}（${now.action}）` : "—");
  if (s.pose) {
    const [x, y, yaw] = s.pose;
    setText("position", `x ${x.toFixed(2)} m, y ${y.toFixed(2)} m`);
    setText("heading", `${(yaw * 180 / Math.PI).toFixed(0)} 度`);
  } else {
    setText("position", "—");
    setText("heading", "—");
  }
}

function render(s) {
  serverStatus = s.status;
  renderTalk();
  renderStatus(...(STATUS[s.status] ?? [s.status, "neutral"]));
  renderLink(s.link_kind);
  $("box-buttons").hidden = !("world" in s);  // the server sends the world only when simulating

  renderInstruction(s);
  if (changed("logic", [s.attempts, s.formula, s.llm_backend, s.substitutions])) renderLogic(s);
  if (changed("plan", s.steps)) renderPlan(s.steps);
  renderNotice(noticeFor(s));
  if (changed("log", s.log)) renderLog("log", s.log);
  if (changed("debug-log", s.debug_log)) renderLog("debug-log", s.debug_log);

  if (s.diagram_version !== diagramVersion) {
    diagramVersion = s.diagram_version;
    diagramNames = s.diagram_names ?? [];
    loadDiagram(Date.now() - diagramTouched > 10000);  // follow the state, unless the user is looking around
  }

  renderGo2(s);
  if (changed("map", [s.pose, s.world ?? null])) drawMap(s.pose, s.world ?? null);
}

// ---------------- top view ----------------
// World x to the right, world y up, centred on the Go2; a square is GRID m.
function drawMap(pose, world) {
  const canvas = $("map");
  const ratio = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 480;
  const h = canvas.clientHeight || 240;
  if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  const ctx = canvas.getContext("2d");
  const css = getComputedStyle(document.documentElement);
  const color = (name) => css.getPropertyValue(name).trim();
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.font = `11px ${color("--font")}`;

  if (!pose) {
    ctx.fillStyle = color("--muted");
    ctx.fillText("位置がまだ届いていません", 14, 24);
    canvas.setAttribute("aria-label", "上から見た図：位置がまだ届いていません");
    return;
  }
  const [gx, gy, yaw] = pose;
  const scale = w / 2 / VIEW_HALF;  // [px/m]
  const sx = (x) => w / 2 + (x - gx) * scale;
  const sy = (y) => h / 2 - (y - gy) * scale;

  ctx.strokeStyle = color("--map-grid");
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let k = Math.ceil((gx - w / 2 / scale) / GRID); k * GRID <= gx + w / 2 / scale; k++) {
    const x = Math.round(sx(k * GRID)) + 0.5;
    ctx.moveTo(x, 0); ctx.lineTo(x, h);
  }
  for (let k = Math.ceil((gy - h / 2 / scale) / GRID); k * GRID <= gy + h / 2 / scale; k++) {
    const y = Math.round(sy(k * GRID)) + 0.5;
    ctx.moveTo(0, y); ctx.lineTo(w, y);
  }
  ctx.stroke();

  const boxes = world?.boxes ?? [];
  for (const [bx, by, size] of boxes) {
    const half = size / 2;
    ctx.beginPath();
    roundRect(ctx, sx(bx - half), sy(by + half), size * scale, size * scale, 6);
    ctx.fillStyle = world.collided ? color("--stop-soft") : color("--box-fill");
    ctx.strokeStyle = world.collided ? color("--stop") : color("--box-line");
    ctx.lineWidth = 1.5;
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = color("--muted");
    ctx.textAlign = "center";
    ctx.fillText("箱", sx(bx), sy(by - half) + 15);
    ctx.textAlign = "start";
  }

  // The Go2 in its own frame (x forward, y left), drawn in pixels.
  const L = BODY.length * scale, W = BODY.width * scale;
  ctx.save();
  ctx.translate(sx(gx), sy(gy));
  ctx.rotate(-yaw);
  ctx.strokeStyle = color("--accent-text");
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 5]);
  ctx.beginPath();
  for (const d of [STOP_DISTANCE, -STOP_DISTANCE]) {  // the 0.5 m stop lines, front and back
    ctx.moveTo(d * scale, -HALF_WIDTH * scale); ctx.lineTo(d * scale, HALF_WIDTH * scale);
  }
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = color("--robot");
  ctx.beginPath();
  roundRect(ctx, -L / 2, -W / 2, L, W, Math.min(10, W / 2));
  ctx.fill();
  ctx.fillStyle = color("--accent");
  ctx.beginPath();  // the head, so the heading is visible
  ctx.moveTo(L / 2 + 4, 0);
  ctx.lineTo(L / 2 - 6, -W / 4);
  ctx.lineTo(L / 2 - 6, W / 4);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // 「止まる 0.5 m」 by the left end of the front stop line, upright.
  const lx = gx + Math.cos(yaw) * STOP_DISTANCE - Math.sin(yaw) * HALF_WIDTH;
  const ly = gy + Math.sin(yaw) * STOP_DISTANCE + Math.cos(yaw) * HALF_WIDTH;
  ctx.fillStyle = color("--accent-text");
  ctx.fillText(`止まる ${STOP_DISTANCE} m`, sx(lx) + 4, sy(ly) - 6);

  if (world?.collided) {
    ctx.fillStyle = color("--stop-text");
    ctx.font = `600 12px ${color("--font")}`;
    ctx.fillText("箱にぶつかりました", 14, 22);
  }
  canvas.setAttribute("aria-label", `上から見た図：箱 ${boxes.length} 個${world?.collided ? "。箱にぶつかりました" : ""}`);
}

function roundRect(ctx, x, y, w, h, r) {
  if (ctx.roundRect) ctx.roundRect(x, y, w, h, r);
  else ctx.rect(x, y, w, h);
}

window.addEventListener("resize", () => { delete shown.map; });  // redraw at the new size
window.matchMedia?.("(prefers-color-scheme: dark)").addEventListener?.("change", () => { delete shown.map; });

// ---------------- WebSocket ----------------
function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onmessage = (e) => render(JSON.parse(e.data));
  ws.onclose = () => {
    serverStatus = "idle";  // no thinking spinner while nothing comes
    renderTalk();
    renderStatus("サーバーとつながっていません", "neutral");
    setText("link-text", "つながっていません");
    delete shown.link;  // draw the link again once connected
    setData("link", "tone", "off");
    setTimeout(connect, 1000);
  };
}

renderTalk();
connect();
