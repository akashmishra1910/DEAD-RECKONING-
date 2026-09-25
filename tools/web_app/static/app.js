/**
 * web_app/static/app.js
 * High-Performance 60FPS Canvas Navigation Renderer & Telemetry Cockpit.
 */

// State variables
let ws = null;
let roadData = null;
let currentTelem = null;
let isPaused = false;

// History trails for rendering
const MAX_TRAIL = 250;
const historyProposed = [];
const historyNaive = [];
const historyGT = [];

// Canvas setup
const canvas = document.getElementById('nav-canvas');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = rect.width * window.devicePixelRatio;
  canvas.height = rect.height * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// --- 1. Fetch Road Network Metadata ---
async function fetchRoadNetwork() {
  try {
    const res = await fetch('/api/trajectory');
    roadData = await res.json();
    console.log('[Client] Road network loaded:', roadData.segments.length, 'segments');
  } catch (err) {
    console.error('[Client] Failed to load road metadata:', err);
  }
}

// --- 2. WebSocket Telemetry Connection ---
function initWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/navigation`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    console.log('[WS] Connected to IDR Navigation stream');
  };

  ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    currentTelem = data;
    updateTelemetryHUD(data);

    // Record trail positions [East, North]
    if (data.pos_proposed) {
      historyProposed.push([data.pos_proposed[0], data.pos_proposed[1]]);
      if (historyProposed.length > MAX_TRAIL) historyProposed.shift();
    }
    if (data.pos_naive) {
      historyNaive.push([data.pos_naive[0], data.pos_naive[1]]);
      if (historyNaive.length > MAX_TRAIL) historyNaive.shift();
    }
    if (data.pos_gt) {
      historyGT.push([data.pos_gt[0], data.pos_gt[1]]);
      if (historyGT.length > MAX_TRAIL) historyGT.shift();
    }
  };

  ws.onclose = () => {
    console.log('[WS] Connection closed. Reconnecting in 2s...');
    setTimeout(initWebSocket, 2000);
  };
}

// --- 3. Update HUD Instruments ---
function updateTelemetryHUD(data) {
  // Speedometer
  document.getElementById('speedo-val').innerText = Math.round(data.speed_kmh);

  // Mode Badge
  const modeBadge = document.getElementById('mode-badge');
  const modeText = document.getElementById('mode-text');

  modeBadge.className = 'mode-badge';
  if (data.mode === 'GNSS_AIDED_INS') {
    modeBadge.classList.add('gnss-active');
    modeText.innerText = 'GNSS-AIDED INS';
  } else if (data.mode === 'INTELLIGENT_DEAD_RECKONING') {
    modeBadge.classList.add('idr-active');
    modeText.innerText = 'INTELLIGENT DEAD RECKONING';
  } else if (data.mode === 'GNSS_RECOVERY') {
    modeBadge.classList.add('recovery-active');
    modeText.innerText = 'GNSS RECOVERY (CHI^2 GATED)';
  }

  // Top stats
  document.getElementById('stat-drift').innerText = `${data.drift_pct.toFixed(2)}%`;
  document.getElementById('stat-outage-time').innerText = `${data.outage_duration_s.toFixed(1)}s`;

  // Corridor Name
  document.getElementById('current-road-text').innerText = `Corridor: ${formatRoadName(data.road_type)}`;

  // Altitude
  const altM = data.pos_proposed[2] || 0.0;
  document.getElementById('alt-val').innerText = `${altM.toFixed(1)}m`;
  const fillPct = Math.min(100, Math.max(0, (altM / 8.5) * 100));
  document.getElementById('alt-bar-fill').style.height = `${fillPct}%`;

  // Distance
  document.getElementById('total-dist-txt').innerText = `${data.total_dist_m.toFixed(1)}m`;
  document.getElementById('outage-dist-val').innerText = `${data.outage_dist_m.toFixed(1)}m`;

  // Attitude
  document.getElementById('pitch-deg').innerText = `${data.pitch_deg.toFixed(1)}°`;
  document.getElementById('roll-deg').innerText = `${data.roll_deg.toFixed(1)}°`;
  const pitchLine = document.getElementById('pitch-line');
  if (pitchLine) {
    pitchLine.style.transform = `translateY(${data.pitch_deg * 1.5}px) rotate(${data.roll_deg}deg)`;
  }

  // Drift Comparison
  document.getElementById('drift-pct-val').innerText = `${data.drift_pct.toFixed(2)}%`;
  document.getElementById('err-proposed-m').innerText = `${data.err_proposed_m.toFixed(1)}m`;
  document.getElementById('err-naive-m').innerText = `${data.err_naive_m.toFixed(1)}m`;
  const naiveDrift = data.outage_dist_m > 5 ? ((data.err_naive_m / data.outage_dist_m) * 100).toFixed(1) : '0.0';
  document.getElementById('drift-naive-val').innerText = `${naiveDrift}%`;

  // Heading & Pothole
  document.getElementById('heading-val').innerText = `${data.heading_deg.toFixed(0)}°`;
  document.getElementById('pothole-count').innerText = data.pothole_count;

  // Alerts
  document.getElementById('tunnel-alert').style.display = data.in_outage ? 'flex' : 'none';
  document.getElementById('pothole-alert').style.display = data.pothole_active ? 'flex' : 'none';
  document.getElementById('idle-alert').style.display = data.is_idling ? 'flex' : 'none';
}

function formatRoadName(type) {
  switch (type) {
    case 'flyover': return 'Elevated Flyover Deck (+8.5m)';
    case 'service_road': return 'Frontage Service Road (Parallel)';
    case 'highway': return 'Main Expressway Corridor';
    default: return 'Ground Arterial Avenue';
  }
}

// --- 4. High-Performance Canvas Rendering Loop ---
function renderLoop() {
  requestAnimationFrame(renderLoop);
  if (!currentTelem) return;

  const w = canvas.parentElement.clientWidth;
  const h = canvas.parentElement.clientHeight;

  ctx.clearRect(0, 0, w, h);

  // World-to-Screen Transform:
  // Center camera on the proposed vehicle position
  const camX = currentTelem.pos_proposed[0];
  const camY = currentTelem.pos_proposed[1];
  const zoom = 1.3; // pixels per meter

  ctx.save();
  ctx.translate(w / 2, h / 2);
  ctx.scale(zoom, -zoom); // Y points North (up)
  ctx.translate(-camX, -camY);

  // A. Draw Grid & Background Corridors
  drawRoadNetwork();

  // B. Draw Trajectory Trails
  drawTrails();

  // C. Draw Vehicle Marker
  drawVehicleMarker(camX, camY, currentTelem.heading_deg);

  ctx.restore();
}

function drawRoadNetwork() {
  if (!roadData || !roadData.segments) return;

  for (const seg of roadData.segments) {
    const s = seg.start;
    const e = seg.end;

    // Road body
    ctx.lineWidth = seg.type === 'service_road' ? 14 : 22;
    if (seg.type === 'flyover') {
      ctx.strokeStyle = '#1e3a8a'; // Blue deck
      ctx.shadowColor = 'rgba(0, 176, 255, 0.4)';
      ctx.shadowBlur = 10;
    } else if (seg.type === 'service_road') {
      ctx.strokeStyle = '#27272a'; // Service lane
      ctx.shadowBlur = 0;
    } else {
      ctx.strokeStyle = '#1f2937'; // Ground road
      ctx.shadowBlur = 0;
    }

    ctx.beginPath();
    ctx.moveTo(s[0], s[1]);
    ctx.lineTo(e[0], e[1]);
    ctx.stroke();

    // Road centerline
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = seg.type === 'flyover' ? '#60a5fa' : '#4b5563';
    ctx.setLineDash([4, 6]);
    ctx.beginPath();
    ctx.moveTo(s[0], s[1]);
    ctx.lineTo(e[0], e[1]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.shadowBlur = 0;
  }
}

function drawTrails() {
  // Ground Truth (White)
  if (historyGT.length > 1) {
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
    ctx.beginPath();
    ctx.moveTo(historyGT[0][0], historyGT[0][1]);
    for (let i = 1; i < historyGT.length; i++) {
      ctx.lineTo(historyGT[i][0], historyGT[i][1]);
    }
    ctx.stroke();
  }

  // Naive Double Integration (Red Diverging)
  if (historyNaive.length > 1) {
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = '#ef4444';
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(historyNaive[0][0], historyNaive[0][1]);
    for (let i = 1; i < historyNaive.length; i++) {
      ctx.lineTo(historyNaive[i][0], historyNaive[i][1]);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Proposed AI+IDR (Bright Green)
  if (historyProposed.length > 1) {
    ctx.lineWidth = 3.5;
    ctx.strokeStyle = '#00e676';
    ctx.shadowColor = 'rgba(0, 230, 118, 0.6)';
    ctx.shadowBlur = 8;
    ctx.beginPath();
    ctx.moveTo(historyProposed[0][0], historyProposed[0][1]);
    for (let i = 1; i < historyProposed.length; i++) {
      ctx.lineTo(historyProposed[i][0], historyProposed[i][1]);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;
  }
}

function drawVehicleMarker(x, y, headingDeg) {
  ctx.save();
  ctx.translate(x, y);

  // Heading rotation: heading is azimuth from North (0 = North, 90 = East)
  // In canvas, North is +Y. Angle in math coords is (90 - headingDeg)
  const angleRad = ((90 - headingDeg) * Math.PI) / 180;
  ctx.rotate(angleRad);

  // Headlights beam
  const grad = ctx.createRadialGradient(0, 0, 4, 30, 0, 45);
  grad.addColorStop(0, 'rgba(0, 230, 118, 0.45)');
  grad.addColorStop(1, 'rgba(0, 230, 118, 0)');
  ctx.fillStyle = grad;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.arc(0, 0, 45, -Math.PI / 6, Math.PI / 6);
  ctx.fill();

  // Vehicle Body (Car shape)
  ctx.fillStyle = '#00e676';
  ctx.shadowColor = '#00e676';
  ctx.shadowBlur = 12;

  ctx.beginPath();
  ctx.moveTo(10, 0);       // Front tip
  ctx.lineTo(-8, 6);       // Left rear
  ctx.lineTo(-4, 0);       // Rear notch
  ctx.lineTo(-8, -6);      // Right rear
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}

// --- 5. User Control Event Handlers ---
function setupControls() {
  document.getElementById('btn-jam-gnss').onclick = () => {
    ws.send(JSON.stringify({ action: 'jam_gnss' }));
  };

  document.getElementById('btn-restore-gnss').onclick = () => {
    ws.send(JSON.stringify({ action: 'restore_gnss' }));
  };

  document.getElementById('btn-pothole').onclick = () => {
    ws.send(JSON.stringify({ action: 'trigger_pothole' }));
  };

  document.getElementById('btn-bump').onclick = () => {
    ws.send(JSON.stringify({ action: 'bump_phone' }));
  };

  document.getElementById('btn-play').onclick = (e) => {
    isPaused = !isPaused;
    e.target.innerText = isPaused ? '▶️ Resume' : '⏸️ Pause';
    ws.send(JSON.stringify({ action: 'toggle_pause' }));
  };

  document.getElementById('btn-reset').onclick = () => {
    historyProposed.length = 0;
    historyNaive.length = 0;
    historyGT.length = 0;
    ws.send(JSON.stringify({ action: 'reset' }));
  };

  // Speed controls
  document.querySelectorAll('.btn-speed').forEach((btn) => {
    btn.onclick = (e) => {
      document.querySelectorAll('.btn-speed').forEach((b) => b.classList.remove('active'));
      e.target.classList.add('active');
      const speedVal = parseFloat(e.target.dataset.speed);
      ws.send(JSON.stringify({ action: 'set_speed', value: speedVal }));
    };
  });
}

// Initialize
window.onload = async () => {
  await fetchRoadNetwork();
  initWebSocket();
  setupControls();
  requestAnimationFrame(renderLoop);
};
