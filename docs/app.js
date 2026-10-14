/**
 * AutonomousNav AI - Interactive Web Showcase & Simulation Engine
 */

// --- Benchmark Suite Data (Synced with results/evaluation_report.json) ---
const BENCHMARK_DATA = [
  {
    name: "default_navigation_arena",
    plan: "PASS",
    nodes: 1846,
    planTime: "134.27 ms",
    waypoints: "80 -> 4 (-95.0%)",
    traveled: "893.5 u",
    simTime: "30.10 s",
    outcome: "REACHED_GOAL",
    safety: "SAFE"
  },
  {
    name: "open_world_straight",
    plan: "PASS",
    nodes: 15,
    planTime: "1.14 ms",
    waypoints: "15 -> 2 (-86.7%)",
    traveled: "193.0 u",
    simTime: "12.65 s",
    outcome: "REACHED_GOAL",
    safety: "SAFE"
  },
  {
    name: "obstacle_detour",
    plan: "PASS",
    nodes: 158,
    planTime: "13.63 ms",
    waypoints: "25 -> 4 (-84.0%)",
    traveled: "257.9 u",
    simTime: "14.30 s",
    outcome: "REACHED_GOAL",
    safety: "SAFE"
  },
  {
    name: "blocked_unreachable",
    plan: "FAIL",
    nodes: 126,
    planTime: "10.17 ms",
    waypoints: "N/A",
    traveled: "N/A",
    simTime: "N/A",
    outcome: "NOT_RUN",
    safety: "N/A"
  },
  {
    name: "insufficient_clearance",
    plan: "FAIL",
    nodes: 110,
    planTime: "7.29 ms",
    waypoints: "N/A",
    traveled: "N/A",
    simTime: "N/A",
    outcome: "NOT_RUN",
    safety: "N/A"
  },
  {
    name: "dynamic_obstacle_detour",
    plan: "PASS",
    nodes: 31,
    planTime: "1.08 ms",
    waypoints: "31 -> 2 (-93.5%)",
    traveled: "439.8 u",
    simTime: "19.05 s",
    outcome: "REACHED_GOAL",
    safety: "SAFE"
  }
];

// --- Scenarios Configuration ---
const SCENARIOS = {
  default: {
    name: "Default Navigation Arena",
    width: 800,
    height: 600,
    robotRadius: 15.0,
    start: { x: 50, y: 50 },
    goal: { x: 750, y: 550 },
    obstacles: [
      { type: "circle", cx: 200, cy: 200, radius: 40 },
      { type: "rect", x: 380, y: 250, width: 80, height: 160 },
      { type: "circle", cx: 600, cy: 400, radius: 50 }
    ],
    rawPath: [
      {x:50,y:50}, {x:120,y:120}, {x:140,y:230}, {x:240,y:310},
      {x:350,y:430}, {x:490,y:440}, {x:600,y:480}, {x:750,y:550}
    ],
    smoothedPath: [
      {x:50, y:50},
      {x:140, y:230},
      {x:400, y:430},
      {x:750, y:550}
    ],
    dynamicObstacles: []
  },
  dynamic: {
    name: "Dynamic Obstacle Online Detour",
    width: 400,
    height: 300,
    robotRadius: 15.0,
    start: { x: 40, y: 150 },
    goal: { x: 360, y: 150 },
    obstacles: [
      { type: "rect", x: 170, y: 100, width: 60, height: 100 }
    ],
    rawPath: [
      {x:40, y:150}, {x:360, y:150}
    ],
    smoothedPath: [
      {x:40, y:150},
      {x:140, y:240},
      {x:260, y:240},
      {x:360, y:150}
    ],
    dynamicObstacles: [
      {
        p1: { x: 200, y: 30 },
        p2: { x: 200, y: 150 },
        radius: 18,
        speed: 25
      }
    ]
  },
  blocked: {
    name: "Blocked Corridor Safe Halting",
    width: 300,
    height: 200,
    robotRadius: 10.0,
    start: { x: 40, y: 100 },
    goal: { x: 260, y: 100 },
    obstacles: [
      { type: "rect", x: 130, y: 0, width: 40, height: 75 },
      { type: "rect", x: 130, y: 125, width: 40, height: 75 }
    ],
    rawPath: [{x:40, y:100}, {x:260, y:100}],
    smoothedPath: [{x:40, y:100}, {x:260, y:100}],
    dynamicObstacles: [
      {
        p1: { x: 150, y: 75 },
        p2: { x: 150, y: 125 },
        radius: 16,
        speed: 0
      }
    ]
  },
  detour: {
    name: "Wall Obstacle Detour Bypass",
    width: 300,
    height: 200,
    robotRadius: 10.0,
    start: { x: 30, y: 100 },
    goal: { x: 270, y: 100 },
    obstacles: [
      { type: "rect", x: 140, y: 40, width: 20, height: 160 }
    ],
    rawPath: [{x:30,y:100}, {x:120,y:40}, {x:160,y:20}, {x:200,y:40}, {x:270,y:100}],
    smoothedPath: [{x:30,y:100}, {x:125,y:30}, {x:175,y:30}, {x:270,y:100}],
    dynamicObstacles: []
  },
  open: {
    name: "Open World Direct Shortcut",
    width: 200,
    height: 200,
    robotRadius: 10.0,
    start: { x: 20, y: 20 },
    goal: { x: 180, y: 180 },
    obstacles: [],
    rawPath: [{x:20,y:20}, {x:60,y:60}, {x:100,y:100}, {x:140,y:140}, {x:180,y:180}],
    smoothedPath: [{x:20,y:20}, {x:180,y:180}],
    dynamicObstacles: []
  }
};

// --- Simulation State ---
let currentScenarioKey = "default";
let currentScenario = SCENARIOS.default;

let robot = {
  x: currentScenario.start.x,
  y: currentScenario.start.y,
  theta: 0,
  v: 0,
  omega: 0,
  targetIdx: 1
};

let simTime = 0;
let isPlaying = false;
let playSpeed = 2;
let replansCount = 0;
let simStatus = "TRACKING"; // TRACKING, REACHED_GOAL, BLOCKED

// DOM References
const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const scenarioSelector = document.getElementById("scenarioSelector");
const btnPlayPause = document.getElementById("btnPlayPause");
const playIcon = document.getElementById("playIcon");
const playText = document.getElementById("playText");
const btnStep = document.getElementById("btnStep");
const btnReset = document.getElementById("btnReset");
const speedSelector = document.getElementById("speedSelector");

const telemPos = document.getElementById("telem-pos");
const telemHeading = document.getElementById("telem-heading");
const telemSpeed = document.getElementById("telem-speed");
const telemOmega = document.getElementById("telem-omega");
const telemDist = document.getElementById("telem-dist");
const telemTime = document.getElementById("telem-time");
const telemReplans = document.getElementById("telem-replans");
const statusBadge = document.getElementById("statusBadge");
const statusText = document.getElementById("statusText");

// Initialize Benchmark Table
function initBenchmarkTable() {
  const tbody = document.getElementById("benchmarkBody");
  tbody.innerHTML = "";
  BENCHMARK_DATA.forEach(row => {
    const tr = document.createElement("tr");
    const planPill = row.plan === "PASS" ? "pill-success" : "pill-fail";
    const outcomePill = row.outcome === "REACHED_GOAL" ? "pill-success" : (row.outcome === "NOT_RUN" ? "pill-warn" : "pill-fail");

    tr.innerHTML = `
      <td style="font-weight: 600; color: #fff; white-space: nowrap;">${row.name}</td>
      <td style="white-space: nowrap;"><span class="pill ${planPill}">${row.plan}</span></td>
      <td style="font-family: var(--font-mono); white-space: nowrap;">${row.nodes}</td>
      <td style="font-family: var(--font-mono); white-space: nowrap;">${row.planTime}</td>
      <td style="font-family: var(--font-mono); font-size: 0.82rem; white-space: nowrap;">${row.waypoints}</td>
      <td style="font-family: var(--font-mono); white-space: nowrap;">${row.traveled}</td>
      <td style="font-family: var(--font-mono); white-space: nowrap;">${row.simTime}</td>
      <td style="white-space: nowrap;"><span class="pill ${outcomePill}">${row.outcome}</span></td>
      <td style="white-space: nowrap;"><span class="pill ${row.safety === 'SAFE' ? 'pill-success' : 'pill-warn'}">${row.safety}</span></td>
    `;
    tbody.appendChild(tr);
  });
}

// Reset Simulator State
function resetSimulation() {
  currentScenario = SCENARIOS[currentScenarioKey];
  robot = {
    x: currentScenario.start.x,
    y: currentScenario.start.y,
    theta: Math.atan2(
      currentScenario.smoothedPath[1].y - currentScenario.start.y,
      currentScenario.smoothedPath[1].x - currentScenario.start.x
    ),
    v: 0,
    omega: 0,
    targetIdx: 1
  };
  simTime = 0;
  replansCount = currentScenarioKey === "dynamic" ? 3 : 0;

  if (currentScenarioKey === "blocked") {
    simStatus = "BLOCKED";
  } else {
    simStatus = "TRACKING";
  }
  updateUI();
  draw();
}

// Update Simulation Step
function updateSimulation(dt) {
  if (simStatus === "REACHED_GOAL" || simStatus === "BLOCKED") {
    return;
  }

  const path = currentScenario.smoothedPath;
  const target = path[robot.targetIdx];
  const dx = target.x - robot.x;
  const dy = target.y - robot.y;
  const dist = Math.hypot(dx, dy);

  if (dist < 8.0) {
    if (robot.targetIdx < path.length - 1) {
      robot.targetIdx++;
    } else {
      simStatus = "REACHED_GOAL";
      robot.v = 0;
      robot.omega = 0;
      updateUI();
      return;
    }
  }

  // Pure pursuit steering
  const targetAngle = Math.atan2(dy, dx);
  let alpha = targetAngle - robot.theta;
  while (alpha > Math.PI) alpha -= 2 * Math.PI;
  while (alpha < -Math.PI) alpha += 2 * Math.PI;

  const maxSpeed = 50.0;
  const maxOmega = 3.0;

  robot.v = Math.min(maxSpeed, Math.max(15.0, dist * 0.8));
  robot.omega = Math.max(-maxOmega, Math.min(maxOmega, alpha * 2.5));

  // Forward kinematic integration
  robot.theta += robot.omega * dt;
  robot.x += robot.v * Math.cos(robot.theta) * dt;
  robot.y += robot.v * Math.sin(robot.theta) * dt;
  simTime += dt;

  updateUI();
}

// Render Simulator
function draw() {
  const w = canvas.width;
  const h = canvas.height;
  const scW = currentScenario.width;
  const scH = currentScenario.height;

  const scaleX = w / scW;
  const scaleY = h / scH;
  const scale = Math.min(scaleX, scaleY);

  ctx.clearRect(0, 0, w, h);

  // Background Grid
  ctx.strokeStyle = "rgba(36, 42, 60, 0.4)";
  ctx.lineWidth = 1;
  const gridSize = 20 * scale;
  for (let x = 0; x < w; x += gridSize) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += gridSize) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  // Obstacles & Clearance Zones
  currentScenario.obstacles.forEach(obs => {
    if (obs.type === "circle") {
      // Clearance buffer zone
      ctx.beginPath();
      ctx.arc(obs.cx * scale, obs.cy * scale, (obs.radius + currentScenario.robotRadius) * scale, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(45, 38, 58, 0.55)";
      ctx.fill();

      // Solid obstacle
      ctx.beginPath();
      ctx.arc(obs.cx * scale, obs.cy * scale, obs.radius * scale, 0, Math.PI * 2);
      ctx.fillStyle = "#dc3545";
      ctx.fill();
      ctx.strokeStyle = "#a01e2d";
      ctx.lineWidth = 2;
      ctx.stroke();
    } else if (obs.type === "rect") {
      const cr = currentScenario.robotRadius * scale;
      // Clearance buffer zone
      ctx.fillStyle = "rgba(45, 38, 58, 0.55)";
      ctx.fillRect(
        (obs.x * scale) - cr,
        (obs.y * scale) - cr,
        (obs.width * scale) + (cr * 2),
        (obs.height * scale) + (cr * 2)
      );

      // Solid obstacle
      ctx.fillStyle = "#dc3545";
      ctx.fillRect(obs.x * scale, obs.y * scale, obs.width * scale, obs.height * scale);
      ctx.strokeStyle = "#a01e2d";
      ctx.lineWidth = 2;
      ctx.strokeRect(obs.x * scale, obs.y * scale, obs.width * scale, obs.height * scale);
    }
  });

  // Dynamic Obstacles
  currentScenario.dynamicObstacles.forEach(dyn => {
    // Patrol path guide
    ctx.beginPath();
    ctx.moveTo(dyn.p1.x * scale, dyn.p1.y * scale);
    ctx.lineTo(dyn.p2.x * scale, dyn.p2.y * scale);
    ctx.strokeStyle = "#78325a";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Periodic position
    let dynX = dyn.p1.x;
    let dynY = dyn.p1.y;
    if (dyn.speed > 0) {
      const distTotal = Math.hypot(dyn.p2.x - dyn.p1.x, dyn.p2.y - dyn.p1.y);
      const cycle = (distTotal * 2) / dyn.speed;
      const t = simTime % cycle;
      const progress = t < cycle / 2 ? (t / (cycle / 2)) : (2 - (t / (cycle / 2)));
      dynX = dyn.p1.x + (dyn.p2.x - dyn.p1.x) * progress;
      dynY = dyn.p1.y + (dyn.p2.y - dyn.p1.y) * progress;
    }

    // Clearance buffer
    ctx.beginPath();
    ctx.arc(dynX * scale, dynY * scale, (dyn.radius + 5) * scale, 0, Math.PI * 2);
    ctx.strokeStyle = "#b4508c";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Moving body
    ctx.beginPath();
    ctx.arc(dynX * scale, dynY * scale, dyn.radius * scale, 0, Math.PI * 2);
    ctx.fillStyle = "#ec4899";
    ctx.fill();

    // Center dot
    ctx.beginPath();
    ctx.arc(dynX * scale, dynY * scale, 4 * scale, 0, Math.PI * 2);
    ctx.fillStyle = "#ffffff";
    ctx.fill();
  });

  // Raw A* Path (Muted Steel Blue)
  if (currentScenario.rawPath && currentScenario.rawPath.length > 1) {
    ctx.beginPath();
    ctx.moveTo(currentScenario.rawPath[0].x * scale, currentScenario.rawPath[0].y * scale);
    for (let i = 1; i < currentScenario.rawPath.length; i++) {
      ctx.lineTo(currentScenario.rawPath[i].x * scale, currentScenario.rawPath[i].y * scale);
    }
    ctx.strokeStyle = "rgba(70, 120, 170, 0.7)";
    ctx.lineWidth = 2.5;
    ctx.setLineDash([5, 5]);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Smoothed Trajectory (Vibrant Gold)
  if (currentScenario.smoothedPath && currentScenario.smoothedPath.length > 1) {
    ctx.beginPath();
    ctx.moveTo(currentScenario.smoothedPath[0].x * scale, currentScenario.smoothedPath[0].y * scale);
    for (let i = 1; i < currentScenario.smoothedPath.length; i++) {
      ctx.lineTo(currentScenario.smoothedPath[i].x * scale, currentScenario.smoothedPath[i].y * scale);
    }
    ctx.strokeStyle = "#fbbf24";
    ctx.lineWidth = 3.5;
    ctx.stroke();

    // Waypoint nodes
    currentScenario.smoothedPath.forEach(pt => {
      ctx.beginPath();
      ctx.arc(pt.x * scale, pt.y * scale, 3.5, 0, Math.PI * 2);
      ctx.fillStyle = "#ffffff";
      ctx.fill();
    });
  }

  // Start & Goal Markers
  // Start
  ctx.beginPath();
  ctx.arc(currentScenario.start.x * scale, currentScenario.start.y * scale, 8, 0, Math.PI * 2);
  ctx.fillStyle = "#10b981";
  ctx.fill();
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.stroke();

  // Goal
  ctx.beginPath();
  ctx.arc(currentScenario.goal.x * scale, currentScenario.goal.y * scale, 8, 0, Math.PI * 2);
  ctx.fillStyle = "#fbbf24";
  ctx.fill();
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.stroke();

  // Lookahead Target Marker
  if (simStatus === "TRACKING" && currentScenario.smoothedPath[robot.targetIdx]) {
    const tgt = currentScenario.smoothedPath[robot.targetIdx];
    ctx.beginPath();
    ctx.arc(tgt.x * scale, tgt.y * scale, 7, 0, Math.PI * 2);
    ctx.strokeStyle = "#00ffd8";
    ctx.lineWidth = 2;
    ctx.stroke();
  }

  // Active Robot Marker
  const rx = robot.x * scale;
  const ry = robot.y * scale;
  const rRad = currentScenario.robotRadius * scale;

  // Circular Footprint Ring
  ctx.beginPath();
  ctx.arc(rx, ry, rRad, 0, Math.PI * 2);
  ctx.strokeStyle = "#38bdf8";
  ctx.lineWidth = 2;
  ctx.stroke();

  // Robot Body
  ctx.beginPath();
  ctx.arc(rx, ry, 7, 0, Math.PI * 2);
  ctx.fillStyle = "#ff6347";
  ctx.fill();
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Heading Vector Pointer
  const hx = rx + Math.cos(robot.theta) * (rRad + 6);
  const hy = ry + Math.sin(robot.theta) * (rRad + 6);
  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.lineTo(hx, hy);
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2.5;
  ctx.stroke();
}

// Update UI Telemetry Values
function updateUI() {
  telemPos.textContent = `(${robot.x.toFixed(1)}, ${robot.y.toFixed(1)})`;
  const headingDeg = (robot.theta * 180 / Math.PI) % 360;
  telemHeading.textContent = `${headingDeg.toFixed(1)} deg`;
  telemSpeed.textContent = `${robot.v.toFixed(1)} u/s`;
  telemOmega.textContent = `${robot.omega.toFixed(2)} rad/s`;

  const gx = currentScenario.goal.x;
  const gy = currentScenario.goal.y;
  const distGoal = Math.hypot(gx - robot.x, gy - robot.y);
  telemDist.textContent = `${distGoal.toFixed(1)} u`;
  telemTime.textContent = `${simTime.toFixed(2)} s`;
  telemReplans.textContent = `${replansCount}`;

  statusBadge.className = "status-indicator";
  if (simStatus === "TRACKING") {
    statusBadge.classList.add("status-tracking");
    statusText.textContent = "TRACKING ROUTE";
  } else if (simStatus === "REACHED_GOAL") {
    statusBadge.classList.add("status-reached");
    statusText.textContent = "REACHED GOAL";
  } else if (simStatus === "BLOCKED") {
    statusBadge.classList.add("status-blocked");
    statusText.textContent = "BLOCKED / SAFE STOP";
  }
}

// Main Animation Loop
let lastTimestamp = 0;
function animate(timestamp) {
  if (!lastTimestamp) lastTimestamp = timestamp;
  const dt = Math.min(0.05, (timestamp - lastTimestamp) / 1000.0);
  lastTimestamp = timestamp;

  if (isPlaying) {
    updateSimulation(dt * playSpeed);
  }
  draw();
  requestAnimationFrame(animate);
}

const SVG_PLAY = '<svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"/></svg>';
const SVG_PAUSE = '<svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="4" width="4" height="16"/><rect x="14" y="4" width="4" height="16"/></svg>';

// Event Listeners
btnPlayPause.addEventListener("click", () => {
  isPlaying = !isPlaying;
  playIcon.innerHTML = isPlaying ? SVG_PAUSE : SVG_PLAY;
  playText.textContent = isPlaying ? "Pause" : "Play";
});

btnStep.addEventListener("click", () => {
  if (isPlaying) {
    isPlaying = false;
    playIcon.innerHTML = SVG_PLAY;
    playText.textContent = "Play";
  }
  updateSimulation(0.05);
  draw();
});

btnReset.addEventListener("click", () => {
  resetSimulation();
});

scenarioSelector.addEventListener("change", (e) => {
  currentScenarioKey = e.target.value;
  resetSimulation();
});

speedSelector.addEventListener("change", (e) => {
  playSpeed = parseFloat(e.target.value);
});

// Window resize handling
function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * window.devicePixelRatio;
  canvas.height = rect.height * window.devicePixelRatio;
  draw();
}
window.addEventListener("resize", resizeCanvas);

// Initialize
initBenchmarkTable();
resetSimulation();
resizeCanvas();
requestAnimationFrame(animate);
