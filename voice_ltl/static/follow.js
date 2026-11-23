const startButton = document.getElementById("follow-start");
const stopButton = document.getElementById("follow-stop");
const statusText = document.getElementById("follow-status");
const standButton = document.getElementById("sim-stand");
let currentSocket = null;
let simulating = false;

const CONTROL_JA = {
  utrack: ["Go2 本体の追尾（UTrack）", "Go2 を立たせ、ペアリング済みの UWB 追従用リモコンを身につけてください。周囲を空け、すぐにリモコンで停止できる状態で始めてください。"],
  own: ["このプログラムの制御（UWB の距離と方向から歩く）", "Go2 を立たせてください。Go2 はリモコンの方を向き、1 m の距離まで歩いて止まります。周囲を空け、すぐに停止できる状態で始めてください。"],
};

async function loadConfig() {
  try {
    const config = await (await fetch("/api/config")).json();
    const [mode, intro] = CONTROL_JA[config.control] ?? [config.control, null];
    document.getElementById("follow-mode").textContent = `追尾のやり方: ${mode}${config.sim ? "・シミュレーション" : ""}`;
    if (intro) document.getElementById("follow-intro").textContent = intro;
    simulating = config.sim;
    standButton.hidden = !simulating;
    document.getElementById("sim-hint").hidden = !simulating;
    map.classList.toggle("sim-drag", simulating);
    if (config.camera) startCamera(config.sim);
  } catch {
    document.getElementById("follow-mode").textContent = "追尾のやり方: 読み取れません";
  }
}

standButton.addEventListener("click", async () => {
  standButton.disabled = true;
  statusText.textContent = "Go2 を立たせています…";
  try {
    const result = await (await fetch("/api/sim/stand", { method: "POST" })).json();
    statusText.textContent = result.message;
  } catch {
    statusText.textContent = "立ち上がりを送れませんでした";
  } finally {
    standButton.disabled = false;
  }
});

startButton.addEventListener("click", () => {
  startButton.disabled = true;
  statusText.textContent = "Go2 の追尾サービスを確認しています…";
  const socket = new WebSocket(`ws://${location.host}/ws/follow`);
  currentSocket = socket;
  let finished = false;

  socket.addEventListener("open", () => {
    if (currentSocket === socket) socket.send("start");
  });
  socket.addEventListener("message", (event) => {
    if (currentSocket !== socket) return;
    const status = JSON.parse(event.data);
    statusText.textContent = status.message;
    finished = !status.active;
    if (status.active) {
      setTimeout(() => {
        if (currentSocket === socket && socket.readyState === WebSocket.OPEN) {
          socket.send("continue");
        }
      }, 500);
    }
  });
  socket.addEventListener("close", () => {
    if (currentSocket !== socket) return;
    currentSocket = null;
    startButton.disabled = false;
    if (!finished) {
      statusText.textContent = "接続が切れ、追尾の解除を確認できません。リモコンで停止してください。";
    }
  });
});

stopButton.addEventListener("click", async () => {
  const socket = currentSocket;
  currentSocket = null;
  socket?.close();
  startButton.disabled = true;
  stopButton.disabled = true;
  statusText.textContent = "追尾を解除しています…";
  try {
    const response = await fetch("/api/stop", { method: "POST" });
    const result = await response.json();
    statusText.textContent = result.message;
  } catch {
    statusText.textContent = "追尾の解除を確認できません。リモコンで停止してください。";
  } finally {
    startButton.disabled = false;
    stopButton.disabled = false;
  }
});

window.addEventListener("pagehide", () => currentSocket?.close());

const map = document.getElementById("follow-map");
const go2Position = document.getElementById("go2-position");
const uwbPosition = document.getElementById("uwb-position");
const uwbRelative = document.getElementById("uwb-relative");
const uwbChannel = document.getElementById("uwb-channel");
const degrees = (radians) => Math.round(radians * 180 / Math.PI);
const xy = ([x, y]) => `x ${x.toFixed(2)} m, y ${y.toFixed(2)} m`;

// 最後に描いた地図の座標変換（シミュレーションでクリック位置をオドメトリ座標に直す）
let view = null;

function drawPositions(go2, target, simTag = null, obstacles = null, heading = null) {
  const w = map.clientWidth, h = map.clientHeight;
  if (!w || !h) return;
  const ratio = window.devicePixelRatio || 1;
  const width = Math.round(w * ratio), height = Math.round(h * ratio);
  if (map.width !== width || map.height !== height) {
    map.width = width;
    map.height = height;
  }
  const ctx = map.getContext("2d");
  const css = getComputedStyle(map);
  const color = (name) => css.getPropertyValue(name).trim();
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  ctx.font = `12px ${color("--font")}`;
  if (!go2) {
    ctx.fillStyle = color("--muted");
    ctx.fillText("Go2 の位置が届いていません", 16, 26);
    return;
  }

  const [gx, gy, yaw] = go2;
  const [tx, ty] = target ?? go2;
  let scale, cx, cy;
  if (simulating) {
    // クリックで動かせるよう、Go2 を中心に半径 4 m 以上を見せる。見えている範囲に置いた
    // リモコンでは縮尺を変えず（点がクリックした所に出る）、はみ出したときだけ広げる。
    scale = Math.min(w - 32, h - 32) / 8;
    for (const [x, y] of [target, simTag].filter(Boolean)) {
      scale = Math.min(scale, (w / 2 - 8) / Math.max(0.01, Math.abs(x - gx)),
                              (h / 2 - 8) / Math.max(0.01, Math.abs(y - gy)));
    }
    cx = gx;
    cy = gy;
  } else {
    scale = Math.min(75, (w - 64) / Math.max(1, Math.abs(tx - gx)),
                         (h - 64) / Math.max(1, Math.abs(ty - gy)));
    cx = (gx + tx) / 2;
    cy = (gy + ty) / 2;
  }
  view = { w, h, scale, cx, cy };
  const point = (x, y) => [w / 2 + (x - cx) * scale, h / 2 - (y - cy) * scale];
  const origin = point(gx, gy);
  const tag = point(tx, ty);

  // 点群の障害物：Go2 に近いほど目立つ色
  if (obstacles?.points?.length) {
    const near = color("--stop"), mid = color("--warn"), far = color("--edge");
    const size = Math.max(3, Math.min(8, 0.1 * scale));
    for (const [x, y] of obstacles.points) {
      const [px, py] = point(x, y);
      if (px < -size || py < -size || px > w + size || py > h + size) continue;
      const d = Math.hypot(x - gx, y - gy);
      ctx.fillStyle = d < 0.9 ? near : d < 1.5 ? mid : far;
      ctx.fillRect(px - size / 2, py - size / 2, size, size);
    }
  }

  // 追尾中に進もうとしている向き（障害物をよけるときはリモコンの方向からそれる）
  if (heading != null) {
    const len = Math.max(36, scale);
    const [ox, oy] = origin;
    const ex = ox + len * Math.cos(heading), ey = oy - len * Math.sin(heading);
    ctx.strokeStyle = ctx.fillStyle = color("--ok");
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(ex, ey);
    ctx.stroke();
    ctx.lineWidth = 1;
    ctx.save();
    ctx.translate(ex, ey);
    ctx.rotate(-heading);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(-9, 5);
    ctx.lineTo(-9, -5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  if (target) {
    ctx.strokeStyle = color("--accent-text");
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(...origin);
    ctx.lineTo(...tag);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  ctx.save();
  ctx.translate(...origin);
  ctx.rotate(-yaw);
  ctx.fillStyle = color("--robot");
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(-8, 9);
  ctx.lineTo(-4, 0);
  ctx.lineTo(-8, -9);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
  ctx.fillStyle = color("--robot");
  ctx.fillText("Go2", origin[0] - 12, origin[1] - 16);

  if (simTag) {
    ctx.strokeStyle = color("--muted");
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(...point(...simTag), 8, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.lineWidth = 1;
  }

  if (target) {
    ctx.fillStyle = color("--accent-text");
    ctx.beginPath();
    ctx.arc(...tag, 5, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillText("リモコン", tag[0] - 24, tag[1] + 24);
  }

  const ruler = Math.max(1, Math.round(50 / scale));
  ctx.strokeStyle = ctx.fillStyle = color("--muted");
  ctx.beginPath();
  ctx.moveTo(16, h - 15);
  ctx.lineTo(16 + ruler * scale, h - 15);
  ctx.stroke();
  ctx.fillText(`${ruler} m`, 16, h - 22);
}

const obstacleRow = document.getElementById("obstacle-row");
const obstacleInfo = document.getElementById("obstacle-info");
const direction = (radians) => {
  const angle = degrees(radians);
  return Math.abs(angle) <= 10 ? "正面" : `${angle > 0 ? "左" : "右"} ${Math.abs(angle)}°`;
};

function renderObstacles(data, disconnected) {
  // utrack のときは点群を受けないので、項目ごと出さない
  const shown = !disconnected && "obstacles" in data;
  obstacleRow.hidden = !shown;
  document.getElementById("cloud-hint").hidden = !shown;
  if (!shown) return;
  const o = data.obstacles;
  obstacleInfo.style.color = "";
  if (!o) {
    obstacleInfo.textContent = "点群が届いていません";
    obstacleInfo.style.color = "var(--warn)";
    return;
  }
  const free = o.front_free;
  const front = free == null ? "前方に障害物なし"
    : free < 0.3 ? `前方 ${free.toFixed(2)} m に障害物あり`
    : `前方 ${free.toFixed(2)} m 先に障害物`;
  const nearest = o.nearest == null ? "周りに障害物なし"
    : `いちばん近い障害物 ${o.nearest.toFixed(2)} m（${direction(o.nearest_bearing)}）`;
  obstacleInfo.textContent = `${front}・${nearest}`;
  if (free != null && free < 0.3) obstacleInfo.style.color = "var(--stop-text)";
  else if (free != null && free < 1.0) obstacleInfo.style.color = "var(--warn)";
}

function renderPositions(data, disconnected = false) {
  const { go2 = null, uwb = null, sim_tag: simTag = null, obstacles = null, heading = null } = data;
  renderObstacles(data, disconnected);
  const target = uwb?.position ?? null;
  go2Position.textContent = go2 ? `${xy(go2)}、向き ${degrees(go2[2])}°` : "未受信";
  uwbPosition.textContent = disconnected ? "サーバーとの通信が途切れました"
    : target ? xy(target)
    : uwb?.error ? `UWB エラー（${uwb.error}）`
    : uwb?.distance != null ? "Go2 の位置待ち" : "測位データなし";
  const angle = uwb?.bearing != null ? degrees(uwb.bearing) : 0;
  uwbRelative.textContent = uwb?.distance != null
    ? `${uwb.distance.toFixed(2)} m、${angle === 0 ? "正面" : `${angle > 0 ? "左" : "右"} ${Math.abs(angle)}°`}`
    : "—";
  uwbChannel.textContent = uwb?.channel ?? "—";
  drawPositions(go2, target, simTag, obstacles, heading);
}

// シミュレーション：地図のクリック・ドラッグでにせ UWB リモコンを動かす（送るのは 10 Hz まで）
let dragging = false, lastSent = 0, pendingPlace = null;

function placeAt(event) {
  if (!simulating || !view) return;
  const rect = map.getBoundingClientRect();
  const px = event.clientX - rect.left, py = event.clientY - rect.top;
  pendingPlace = { x: view.cx + (px - view.w / 2) / view.scale, y: view.cy - (py - view.h / 2) / view.scale };
  const now = performance.now();
  if (now - lastSent < 100) return;
  lastSent = now;
  const body = JSON.stringify(pendingPlace);
  pendingPlace = null;
  fetch("/api/sim/uwb", { method: "POST", headers: { "Content-Type": "application/json" }, body })
    .catch(() => { statusText.textContent = "リモコンの位置を送れませんでした"; });
}

map.addEventListener("pointerdown", (event) => {
  if (!simulating) return;
  dragging = true;
  map.setPointerCapture(event.pointerId);
  placeAt(event);
});
map.addEventListener("pointermove", (event) => { if (dragging) placeAt(event); });
const endDrag = (event) => {
  if (!dragging) return;
  dragging = false;
  lastSent = 0;
  placeAt(event);   // 離した位置を必ず送る
};
map.addEventListener("pointerup", endDrag);
map.addEventListener("pointercancel", endDrag);

async function updatePositions() {
  try {
    const response = await fetch("/api/position", { signal: AbortSignal.timeout(1500) });
    if (!response.ok) throw new Error(response.status);
    renderPositions(await response.json());
  } catch {
    renderPositions({}, true);
  } finally {
    setTimeout(updatePositions, 250);
  }
}

// 前のカメラ：最新の 1 枚を 5 回/秒まで取りに行く（タブが隠れている間は休む）
const cameraView = document.getElementById("camera-view");
const cameraEmpty = document.getElementById("camera-empty");
let cameraUrl = null;

function showCameraMessage(text) {
  cameraView.hidden = true;
  cameraEmpty.hidden = false;
  cameraEmpty.textContent = text;
}

function startCamera(sim) {
  document.getElementById("camera-card").hidden = false;
  const missing = sim
    ? "映像がありません。Gazebo を GO2_SIM_CAMERA=1 で起動するとカメラが付きます（偽の Go2 には映像がありません）。"
    : "映像がありません。Go2 の動画サービスから画像が届いていません。";
  async function update() {
    if (document.hidden) {
      setTimeout(update, 1000);
      return;
    }
    try {
      const response = await fetch("/api/camera", { signal: AbortSignal.timeout(2000) });
      if (response.status === 204) {
        showCameraMessage(missing);
      } else if (response.ok) {
        const url = URL.createObjectURL(await response.blob());
        cameraView.src = url;
        cameraView.hidden = false;
        cameraEmpty.hidden = true;
        if (cameraUrl) URL.revokeObjectURL(cameraUrl);
        cameraUrl = url;
      } else {
        showCameraMessage(`映像を読み込めません（${response.status}）`);
      }
    } catch {
      showCameraMessage("サーバーとの通信が途切れました");
    } finally {
      setTimeout(update, 200);
    }
  }
  update();
}

loadConfig().finally(updatePositions);
