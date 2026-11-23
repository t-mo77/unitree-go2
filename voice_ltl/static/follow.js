const startButton = document.getElementById("follow-start");
const stopButton = document.getElementById("follow-stop");
const statusText = document.getElementById("follow-status");
let currentSocket = null;

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

function drawPositions(go2, target) {
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
  const scale = Math.min(75, (w - 64) / Math.max(1, Math.abs(tx - gx)),
                             (h - 64) / Math.max(1, Math.abs(ty - gy)));
  const cx = (gx + tx) / 2, cy = (gy + ty) / 2;
  const point = (x, y) => [w / 2 + (x - cx) * scale, h / 2 - (y - cy) * scale];
  const origin = point(gx, gy);
  const tag = point(tx, ty);

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

function renderPositions({ go2 = null, uwb = null }, disconnected = false) {
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
  drawPositions(go2, target);
}

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

updatePositions();
