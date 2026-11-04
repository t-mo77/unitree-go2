"use strict";
// The page: push to talk (Space), the WebSocket view of Session.snapshot(), the diagrams,
// the top view and the buttons. Texts for the user are Japanese.

const MIME = "audio/webm;codecs=opus";
const STOP_DISTANCE = 0.5;   // [m] perception.py stop_distance
const HALF_WIDTH = 0.3;      // [m] perception.py half_width
const BODY = { length: 0.7, width: 0.31 };  // Go2 [m]
const MAP_SCALE = 60;        // [px/m]

const POSTURE_JA = { LYING: "伏せ", STANDING: "立ち", SITTING: "おすわり", UNKNOWN: "不明" };
const SENSE_JA = { CLEAR: "なし", BLOCKED: "あり", SENSOR_LOST: "途切れ" };
const STATUS_JA = {
  idle: "指示を待っています", transcribing: "書き起こし中…", thinking: "翻訳と合成の途中…",
  executing: "実行中", waiting: "待機中", done: "完了", error: "できませんでした",
};
const ACTION_JA = {
  stand: "立つ", lie_down: "伏せる", sit: "おすわり", rise_sit: "立ち上がる",
  hello: "挨拶", stretch: "伸び", content: "喜ぶ", heart: "ハート", scrape: "おじぎ",
  dance: "ダンス", forward: "前進", backward: "後退", turn_left: "左に回る", turn_right: "右に回る",
};

const $ = (id) => document.getElementById(id);

// ---------------- push to talk ----------------
// ptt.recording is true while a recording runs; ptt.starts counts the recordings started.
const ptt = { recording: false, starts: 0, stream: null, recorder: null, asking: false };
window.ptt = ptt;

function renderTalk() {
  $("talk").classList.toggle("recording", ptt.recording);
  $("talk-label").textContent = ptt.recording ? "録音中…" : "スペースを押している間、話せます";
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
    $("mic").textContent = "マイク：使えます";
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
  rec.ondataavailable = (e) => { if (e.data.size) chunks.push(e.data); };
  rec.onstop = () => sendAudio(new Blob(chunks, { type: rec.mimeType || MIME }));
  ptt.recorder = rec;
  ptt.recording = true;
  ptt.starts += 1;
  renderTalk();
}

function stopRecording() {
  if (!ptt.recording) return;
  ptt.recording = false;
  ptt.recorder.stop();  // onstop sends what was recorded
  ptt.recorder = null;
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

function isTyping(el) {
  return el instanceof HTMLElement
    && (el.isContentEditable || ["INPUT", "TEXTAREA", "SELECT"].includes(el.tagName));
}

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

function onClick(id, fn) {
  $(id).addEventListener("click", (e) => {
    e.currentTarget.blur();  // a focused button would be pressed again by the Space key
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
let diagramTab = "SUP";
let diagramVersion = null;
let diagramNames = [];  // the diagrams the session has drawn

function loadDiagram() {
  const img = $("diagram");
  if (!diagramNames.includes(diagramTab)) {
    img.removeAttribute("src");
    img.hidden = true;
    $("diagram-empty").hidden = false;
    return;
  }
  img.src = `/api/diagram/${diagramTab}.svg?v=${diagramVersion}`;
}
$("diagram").addEventListener("load", () => {
  $("diagram").hidden = false;
  $("diagram-empty").hidden = true;
});
$("diagram").addEventListener("error", () => {
  $("diagram").hidden = true;
  $("diagram-empty").hidden = false;
});
for (const tab of document.querySelectorAll('[role="tab"]')) {
  tab.addEventListener("click", (e) => {
    e.currentTarget.blur();
    diagramTab = tab.dataset.name;
    for (const t of document.querySelectorAll('[role="tab"]')) {
      t.setAttribute("aria-selected", String(t === tab));
    }
    loadDiagram();
  });
}
$("fit").addEventListener("change", (e) => {
  $("diagram-box").classList.toggle("fit", e.target.checked);
  e.target.blur();
});

// ---------------- view ----------------
function setText(id, text) {
  const el = $(id);
  if (el.textContent !== text) el.textContent = text;
}

const shown = {};  // what each part last drew, so an unchanged part is not redrawn
function changed(key, value) {
  const s = JSON.stringify(value);
  if (shown[key] === s) return false;
  shown[key] = s;
  return true;
}

function actionJa(name) {
  return name ? `${ACTION_JA[name] ?? name}（${name}）` : "—";
}

function renderAttempts(attempts) {
  const body = $("attempts");
  body.replaceChildren();
  attempts.forEach((a, i) => {
    const tr = document.createElement("tr");
    const ok = a.formula !== null && a.unsupported.length === 0 && a.errors.length === 0;
    const reason = a.unsupported.length
      ? `式で表せない部分：${a.unsupported.join("、")}`
      : a.formula === null ? "式がありません" : a.errors.join(" ");
    const cells = [String(i + 1), a.formula ?? "null", ok ? "合格" : "不合格", reason];
    cells.forEach((text, k) => {
      const td = document.createElement("td");
      td.textContent = text;
      if (k === 2) td.className = ok ? "pass" : "fail";
      tr.append(td);
    });
    body.append(tr);
  });
}

function renderPlan(plan, current) {
  const ol = $("plan");
  ol.replaceChildren();
  plan.forEach((name, i) => {
    const li = document.createElement("li");
    li.textContent = actionJa(name);
    if (i === 0 && name === current) li.className = "now";
    ol.append(li);
  });
}

function renderLog(lines) {
  const pre = $("log");
  const atEnd = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 4;
  pre.textContent = lines.join("\n");
  if (atEnd) pre.scrollTop = pre.scrollHeight;
}

function sensorText(state, dist) {
  const d = dist === null || dist === undefined ? "—" : `${dist.toFixed(2)} m`;
  return `${d}　${SENSE_JA[state] ?? state}`;
}

function render(s) {
  setText("status", STATUS_JA[s.status] ?? s.status);
  setText("transcript", s.transcript || "—");
  if (changed("attempts", s.attempts)) renderAttempts(s.attempts);
  setText("formula", s.formula ?? "—");
  if (changed("plan", [s.plan, s.current])) renderPlan(s.plan, s.current);
  setText("message", s.message);
  if (changed("log", s.log)) renderLog(s.log);

  if (s.diagram_version !== diagramVersion) {
    diagramVersion = s.diagram_version;
    diagramNames = s.diagram_names ?? [];
    loadDiagram();
  }

  setText("posture", POSTURE_JA[s.posture] ?? s.posture);
  setText("current", actionJa(s.current));
  if (s.pose) {
    const [x, y, yaw] = s.pose;
    setText("position", `x ${x.toFixed(2)} m, y ${y.toFixed(2)} m`);
    setText("heading", `${(yaw * 180 / Math.PI).toFixed(0)} 度`);
  } else {
    setText("position", "—");
    setText("heading", "—");
  }
  setText("front", sensorText(s.front, s.front_dist));
  setText("back", sensorText(s.back, s.back_dist));
  $("front").classList.toggle("blocked", s.front !== "CLEAR");
  $("back").classList.toggle("blocked", s.back !== "CLEAR");

  const simulating = "world" in s;
  for (const b of document.querySelectorAll(".sim-only")) b.hidden = !simulating;
  if (changed("map", [s.pose, s.world ?? null])) drawMap(s.pose, s.world ?? null);
}

// ---------------- top view ----------------
// World x to the right, world y up, centred on the Go2; grid lines every 1 m.
function drawMap(pose, world) {
  const canvas = $("map");
  const ratio = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 320;
  const h = canvas.clientHeight || 240;
  if (canvas.width !== Math.round(w * ratio)) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  const ctx = canvas.getContext("2d");
  const css = getComputedStyle(document.documentElement);
  const color = (name) => css.getPropertyValue(name).trim();
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.font = "12px sans-serif";

  if (!pose) {
    ctx.fillStyle = color("--muted");
    ctx.fillText("位置がまだ届いていません", 12, 22);
    return;
  }
  const [gx, gy, yaw] = pose;
  const sx = (x) => w / 2 + (x - gx) * MAP_SCALE;
  const sy = (y) => h / 2 - (y - gy) * MAP_SCALE;

  ctx.strokeStyle = color("--line");
  ctx.lineWidth = 1;
  for (let x = Math.floor(gx - w / MAP_SCALE); x <= gx + w / MAP_SCALE; x++) {
    ctx.beginPath(); ctx.moveTo(sx(x), 0); ctx.lineTo(sx(x), h); ctx.stroke();
  }
  for (let y = Math.floor(gy - h / MAP_SCALE); y <= gy + h / MAP_SCALE; y++) {
    ctx.beginPath(); ctx.moveTo(0, sy(y)); ctx.lineTo(w, sy(y)); ctx.stroke();
  }

  for (const [bx, by, size] of world?.boxes ?? []) {
    const half = size / 2;
    ctx.fillStyle = world.collided ? color("--bad") : "#b07a3b";
    ctx.fillRect(sx(bx - half), sy(by + half), size * MAP_SCALE, size * MAP_SCALE);
  }

  // The Go2 in its own frame: x forward, y left.
  ctx.save();
  ctx.translate(sx(gx), sy(gy));
  ctx.rotate(-yaw);
  ctx.scale(MAP_SCALE, -MAP_SCALE);
  ctx.lineWidth = 1.5 / MAP_SCALE;
  ctx.setLineDash([0.06, 0.05]);
  ctx.strokeStyle = color("--bad");
  for (const d of [STOP_DISTANCE, -STOP_DISTANCE]) {  // the 0.5 m stop lines, front and back
    ctx.beginPath(); ctx.moveTo(d, -HALF_WIDTH); ctx.lineTo(d, HALF_WIDTH); ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.fillStyle = color("--accent");
  ctx.fillRect(-BODY.length / 2, -BODY.width / 2, BODY.length, BODY.width);
  ctx.fillStyle = color("--card");
  ctx.beginPath();  // the head, so the heading is visible
  ctx.moveTo(BODY.length / 2 - 0.02, 0);
  ctx.lineTo(BODY.length / 2 - 0.16, 0.09);
  ctx.lineTo(BODY.length / 2 - 0.16, -0.09);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  ctx.fillStyle = color("--muted");
  ctx.fillText("赤い点線：止まる距離 0.5 m　方眼：1 m", 8, h - 8);
  if (world?.collided) {
    ctx.fillStyle = color("--bad");
    ctx.fillText("箱にぶつかりました", 8, 18);
  }
}

window.addEventListener("resize", () => { delete shown.map; });  // redraw at the new size

// ---------------- WebSocket ----------------
function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onmessage = (e) => render(JSON.parse(e.data));
  ws.onclose = () => {
    setText("status", "サーバーとつながっていません");
    setTimeout(connect, 1000);
  };
}

renderTalk();
connect();
