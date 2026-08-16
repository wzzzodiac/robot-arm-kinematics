const canvas = document.getElementById('armCanvas');
const ctx = canvas.getContext('2d');

const theta1 = document.getElementById('theta1');
const theta2 = document.getElementById('theta2');
const link1 = document.getElementById('link1');
const link2 = document.getElementById('link2');
const theta1Value = document.getElementById('theta1Value');
const theta2Value = document.getElementById('theta2Value');
const link1Value = document.getElementById('link1Value');
const link2Value = document.getElementById('link2Value');
const fkMode = document.getElementById('fkMode');
const ikMode = document.getElementById('ikMode');
const homeButton = document.getElementById('homeButton');
const randomButton = document.getElementById('randomButton');
const gripperButton = document.getElementById('gripperButton');
const pickPlaceButton = document.getElementById('pickPlaceButton');
const modeReadout = document.getElementById('modeReadout');
const statusBox = document.getElementById('statusBox');

const teleTheta1 = document.getElementById('teleTheta1');
const teleTheta2 = document.getElementById('teleTheta2');
const teleX = document.getElementById('teleX');
const teleY = document.getElementById('teleY');
const teleError = document.getElementById('teleError');
const teleGripper = document.getElementById('teleGripper');
const telePayload = document.getElementById('telePayload');
const teleSolver = document.getElementById('teleSolver');

const state = {
  mode: 'fk',
  theta1: 30,
  theta2: 55,
  link1: 180,
  link2: 150,
  gripperClosed: false,
  target: null,
  heldPart: null,
  selectedPart: null,
  animating: false,
  autoRunning: false,
  parts: [
    { id: 'PART_01', x: 165, y: 95, r: 16, color: '#ff9b61' },
    { id: 'PART_02', x: 250, y: 55, r: 14, color: '#f3c969' },
    { id: 'PART_03', x: 95, y: 145, r: 13, color: '#d98cff' }
  ],
  dropZone: { x: 270, y: 150, w: 80, h: 70 }
};

function degToRad(deg) { return deg * Math.PI / 180; }
function radToDeg(rad) { return rad * 180 / Math.PI; }
function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

function getBase() {
  return { x: canvas.width * 0.24, y: canvas.height * 0.76 };
}

function worldToCanvas(x, y) {
  const base = getBase();
  return { x: base.x + x, y: base.y - y };
}

function canvasToWorld(x, y) {
  const base = getBase();
  return { x: x - base.x, y: base.y - y };
}

function forwardKinematics(t1 = state.theta1, t2 = state.theta2) {
  const a1 = degToRad(t1);
  const a2 = degToRad(t1 + t2);
  const elbow = {
    x: state.link1 * Math.cos(a1),
    y: state.link1 * Math.sin(a1)
  };
  const end = {
    x: elbow.x + state.link2 * Math.cos(a2),
    y: elbow.y + state.link2 * Math.sin(a2)
  };
  return { elbow, end };
}

function solveIK(x, y, elbowUp = true) {
  const l1 = state.link1;
  const l2 = state.link2;
  const r2 = x * x + y * y;
  const r = Math.sqrt(r2);
  const minReach = Math.abs(l1 - l2);
  const maxReach = l1 + l2;

  if (r > maxReach || r < minReach) return null;

  let c2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  c2 = clamp(c2, -1, 1);
  let s2 = Math.sqrt(Math.max(0, 1 - c2 * c2));
  if (!elbowUp) s2 *= -1;

  const t2 = Math.atan2(s2, c2);
  const k1 = l1 + l2 * c2;
  const k2 = l2 * s2;
  const t1 = Math.atan2(y, x) - Math.atan2(k2, k1);

  const d1 = radToDeg(t1);
  const d2 = radToDeg(t2);
  if (d1 < -170 || d1 > 170 || d2 < -170 || d2 > 170) return null;

  return { theta1: d1, theta2: d2 };
}

function updateControls() {
  theta1.value = state.theta1;
  theta2.value = state.theta2;
  link1.value = state.link1;
  link2.value = state.link2;
  theta1Value.textContent = `${state.theta1.toFixed(0)}°`;
  theta2Value.textContent = `${state.theta2.toFixed(0)}°`;
  link1Value.textContent = state.link1.toFixed(0);
  link2Value.textContent = state.link2.toFixed(0);
  const manual = state.mode === 'fk';
  theta1.disabled = !manual || state.animating;
  theta2.disabled = !manual || state.animating;
  link1.disabled = state.animating || state.autoRunning;
  link2.disabled = state.animating || state.autoRunning;
}

function setStatus(text) { statusBox.textContent = text; }

function setMode(mode) {
  if (state.autoRunning) return;
  state.mode = mode;
  fkMode.classList.toggle('active', mode === 'fk');
  ikMode.classList.toggle('active', mode === 'ik');
  modeReadout.textContent = `MODE: ${mode.toUpperCase()}`;
  teleSolver.textContent = mode === 'fk' ? 'MANUAL FK' : 'ANALYTIC IK';
  updateControls();
  setStatus(mode === 'fk'
    ? 'Manual forward kinematics active. Move the joint sliders.'
    : 'Inverse kinematics active. Click a target in the workspace.');
  draw();
}

function nearestPart(world) {
  let best = null;
  let bestDist = Infinity;
  for (const part of state.parts) {
    if (state.heldPart === part) continue;
    const d = Math.hypot(world.x - part.x, world.y - part.y);
    if (d < part.r + 18 && d < bestDist) {
      best = part;
      bestDist = d;
    }
  }
  return best;
}

function setTarget(x, y, selectedPart = null) {
  state.target = { x, y };
  state.selectedPart = selectedPart;
  const solution = solveIK(x, y, true) || solveIK(x, y, false);

  if (!solution) {
    teleError.textContent = 'OUT';
    teleSolver.textContent = 'NO SOLUTION';
    setStatus('TARGET OUTSIDE WORKSPACE // physics says no XD');
    draw();
    return false;
  }

  teleSolver.textContent = 'IK SOLVED';
  animateTo(solution.theta1, solution.theta2, 420);
  return true;
}

function animateTo(targetT1, targetT2, duration = 420) {
  if (state.animating) return Promise.resolve(false);
  state.animating = true;
  updateControls();

  const startT1 = state.theta1;
  const startT2 = state.theta2;
  const start = performance.now();

  return new Promise(resolve => {
    function step(now) {
      const p = clamp((now - start) / duration, 0, 1);
      const eased = 1 - Math.pow(1 - p, 3);
      state.theta1 = startT1 + (targetT1 - startT1) * eased;
      state.theta2 = startT2 + (targetT2 - startT2) * eased;
      updateHeldPart();
      updateControls();
      draw();
      if (p < 1) requestAnimationFrame(step);
      else {
        state.animating = false;
        updateControls();
        draw();
        resolve(true);
      }
    }
    requestAnimationFrame(step);
  });
}

function updateHeldPart() {
  if (!state.heldPart) return;
  const { end } = forwardKinematics();
  state.heldPart.x = end.x;
  state.heldPart.y = end.y;
}

function toggleGripper(force) {
  const shouldClose = typeof force === 'boolean' ? force : !state.gripperClosed;
  state.gripperClosed = shouldClose;

  if (shouldClose && !state.heldPart) {
    const { end } = forwardKinematics();
    const candidate = state.selectedPart || nearestPart(end);
    if (candidate && Math.hypot(candidate.x - end.x, candidate.y - end.y) <= candidate.r + 22) {
      state.heldPart = candidate;
      state.selectedPart = candidate;
      setStatus(`${candidate.id} captured. Tiny logistics empire established.`);
    } else {
      setStatus('Gripper closed. Payload acquisition failed with impressive confidence.');
    }
  }

  if (!shouldClose && state.heldPart) {
    const released = state.heldPart;
    state.heldPart = null;
    const dz = state.dropZone;
    const inside = released.x > dz.x - dz.w / 2 && released.x < dz.x + dz.w / 2 &&
                   released.y > dz.y - dz.h / 2 && released.y < dz.y + dz.h / 2;
    setStatus(inside ? `${released.id} placed successfully. Industrial automation achieved.` : `${released.id} released. Location quality is management's problem now.`);
  }

  gripperButton.textContent = state.gripperClosed ? 'OPEN GRIPPER' : 'CLOSE GRIPPER';
  draw();
}

async function autoPickPlace() {
  if (state.autoRunning || state.animating) return;
  const part = state.selectedPart || state.parts.find(p => p !== state.heldPart);
  if (!part) return;

  state.autoRunning = true;
  state.selectedPart = part;
  setMode('ik');
  pickPlaceButton.disabled = true;

  const abovePick = solveIK(part.x, part.y + 42, true) || solveIK(part.x, part.y + 42, false);
  const pick = solveIK(part.x, part.y, true) || solveIK(part.x, part.y, false);
  const dz = state.dropZone;
  const aboveDrop = solveIK(dz.x, dz.y + 50, true) || solveIK(dz.x, dz.y + 50, false);
  const drop = solveIK(dz.x, dz.y, true) || solveIK(dz.x, dz.y, false);

  if (!abovePick || !pick || !aboveDrop || !drop) {
    setStatus('Auto cycle aborted. One waypoint is outside the reachable workspace.');
    state.autoRunning = false;
    pickPlaceButton.disabled = false;
    return;
  }

  setStatus(`AUTO CYCLE // approaching ${part.id}`);
  await animateTo(abovePick.theta1, abovePick.theta2, 430);
  await animateTo(pick.theta1, pick.theta2, 330);
  toggleGripper(true);
  await new Promise(r => setTimeout(r, 220));
  await animateTo(abovePick.theta1, abovePick.theta2, 330);
  setStatus('AUTO CYCLE // transporting payload');
  await animateTo(aboveDrop.theta1, aboveDrop.theta2, 520);
  await animateTo(drop.theta1, drop.theta2, 300);
  toggleGripper(false);
  await new Promise(r => setTimeout(r, 180));
  await animateTo(aboveDrop.theta1, aboveDrop.theta2, 300);
  setStatus(`AUTO CYCLE COMPLETE // ${part.id} moved to drop zone`);

  state.autoRunning = false;
  pickPlaceButton.disabled = false;
  updateControls();
}

function drawGrid() {
  const step = 40;
  ctx.save();
  ctx.strokeStyle = 'rgba(105,240,193,.055)';
  ctx.lineWidth = 1;
  for (let x = 0; x <= canvas.width; x += step) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
  }
  for (let y = 0; y <= canvas.height; y += step) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
  }
  const base = getBase();
  ctx.strokeStyle = 'rgba(105,240,193,.18)';
  ctx.beginPath(); ctx.moveTo(0, base.y); ctx.lineTo(canvas.width, base.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(base.x, 0); ctx.lineTo(base.x, canvas.height); ctx.stroke();
  ctx.restore();
}

function drawWorkspaceLimits() {
  const base = getBase();
  ctx.save();
  ctx.setLineDash([7, 7]);
  ctx.strokeStyle = 'rgba(105,240,193,.13)';
  ctx.beginPath();
  ctx.arc(base.x, base.y, state.link1 + state.link2, 0, Math.PI * 2);
  ctx.stroke();
  const min = Math.abs(state.link1 - state.link2);
  if (min > 1) {
    ctx.beginPath(); ctx.arc(base.x, base.y, min, 0, Math.PI * 2); ctx.stroke();
  }
  ctx.restore();
}

function drawDropZone() {
  const dz = state.dropZone;
  const p = worldToCanvas(dz.x, dz.y);
  ctx.save();
  ctx.strokeStyle = '#7ab6ff';
  ctx.fillStyle = 'rgba(122,182,255,.06)';
  ctx.lineWidth = 2;
  ctx.setLineDash([8, 5]);
  ctx.fillRect(p.x - dz.w/2, p.y - dz.h/2, dz.w, dz.h);
  ctx.strokeRect(p.x - dz.w/2, p.y - dz.h/2, dz.w, dz.h);
  ctx.setLineDash([]);
  ctx.fillStyle = '#7ab6ff';
  ctx.font = '12px Courier New';
  ctx.fillText('DROP ZONE', p.x - dz.w/2, p.y - dz.h/2 - 8);
  ctx.restore();
}

function drawParts() {
  for (const part of state.parts) {
    const p = worldToCanvas(part.x, part.y);
    ctx.save();
    ctx.fillStyle = part.color;
    ctx.strokeStyle = state.selectedPart === part ? '#ffffff' : 'rgba(255,255,255,.28)';
    ctx.lineWidth = state.selectedPart === part ? 3 : 1;
    ctx.fillRect(p.x - part.r, p.y - part.r, part.r * 2, part.r * 2);
    ctx.strokeRect(p.x - part.r, p.y - part.r, part.r * 2, part.r * 2);
    ctx.fillStyle = '#dceae5';
    ctx.font = '11px Courier New';
    ctx.fillText(part.id, p.x - part.r, p.y + part.r + 15);
    ctx.restore();
  }
}

function drawTarget() {
  if (!state.target) return;
  const p = worldToCanvas(state.target.x, state.target.y);
  ctx.save();
  ctx.strokeStyle = '#e7d65e';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(p.x, p.y, 11, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(p.x - 17, p.y); ctx.lineTo(p.x + 17, p.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(p.x, p.y - 17); ctx.lineTo(p.x, p.y + 17); ctx.stroke();
  ctx.restore();
}

function drawArm() {
  const base = getBase();
  const { elbow, end } = forwardKinematics();
  const e = worldToCanvas(elbow.x, elbow.y);
  const tip = worldToCanvas(end.x, end.y);

  ctx.save();
  ctx.lineCap = 'round';
  ctx.lineWidth = 18;
  ctx.strokeStyle = '#386e62';
  ctx.beginPath(); ctx.moveTo(base.x, base.y); ctx.lineTo(e.x, e.y); ctx.stroke();
  ctx.strokeStyle = '#4cae98';
  ctx.beginPath(); ctx.moveTo(e.x, e.y); ctx.lineTo(tip.x, tip.y); ctx.stroke();

  for (const joint of [base, e]) {
    ctx.fillStyle = '#07100f';
    ctx.strokeStyle = '#69f0c1';
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(joint.x, joint.y, 13, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  }

  const totalAngle = degToRad(state.theta1 + state.theta2);
  const perpX = Math.sin(totalAngle);
  const perpY = Math.cos(totalAngle);
  const spread = state.gripperClosed ? 8 : 17;
  const fingerLen = 27;
  ctx.strokeStyle = '#dceae5';
  ctx.lineWidth = 5;
  ctx.lineCap = 'square';
  for (const side of [-1, 1]) {
    const sx = tip.x + perpX * spread * side;
    const sy = tip.y + perpY * spread * side;
    const ex = sx + Math.cos(totalAngle) * fingerLen;
    const ey = sy - Math.sin(totalAngle) * fingerLen;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
  }

  ctx.fillStyle = '#69f0c1';
  ctx.font = '12px Courier New';
  ctx.fillText('J1', base.x + 16, base.y + 4);
  ctx.fillText('J2', e.x + 16, e.y + 4);
  ctx.fillText('TCP', tip.x + 12, tip.y - 12);
  ctx.restore();
}

function updateTelemetry() {
  const { end } = forwardKinematics();
  teleTheta1.textContent = `${state.theta1.toFixed(1)}°`;
  teleTheta2.textContent = `${state.theta2.toFixed(1)}°`;
  teleX.textContent = end.x.toFixed(1);
  teleY.textContent = end.y.toFixed(1);
  teleGripper.textContent = state.gripperClosed ? 'CLOSED' : 'OPEN';
  telePayload.textContent = state.heldPart ? state.heldPart.id : 'NONE';
  if (state.target) teleError.textContent = Math.hypot(end.x - state.target.x, end.y - state.target.y).toFixed(2);
  else teleError.textContent = '—';
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  drawWorkspaceLimits();
  drawDropZone();
  drawTarget();
  drawParts();
  drawArm();
  updateTelemetry();
}

function getCanvasPointer(event) {
  const rect = canvas.getBoundingClientRect();
  const clientX = event.touches ? event.touches[0].clientX : event.clientX;
  const clientY = event.touches ? event.touches[0].clientY : event.clientY;
  return {
    x: (clientX - rect.left) * canvas.width / rect.width,
    y: (clientY - rect.top) * canvas.height / rect.height
  };
}

canvas.addEventListener('pointerdown', event => {
  if (state.animating || state.autoRunning) return;
  const p = getCanvasPointer(event);
  const world = canvasToWorld(p.x, p.y);
  const part = nearestPart(world);

  if (part) {
    state.selectedPart = part;
    setStatus(`${part.id} selected. Click AUTO PICK + PLACE or close the gripper near it.`);
    if (state.mode === 'ik') setTarget(part.x, part.y, part);
    draw();
    return;
  }

  if (state.mode === 'ik') setTarget(world.x, world.y);
});

theta1.addEventListener('input', () => {
  state.theta1 = Number(theta1.value);
  state.target = null;
  updateHeldPart(); updateControls(); draw();
});
theta2.addEventListener('input', () => {
  state.theta2 = Number(theta2.value);
  state.target = null;
  updateHeldPart(); updateControls(); draw();
});
link1.addEventListener('input', () => {
  state.link1 = Number(link1.value);
  state.target = null;
  updateHeldPart(); updateControls(); draw();
});
link2.addEventListener('input', () => {
  state.link2 = Number(link2.value);
  state.target = null;
  updateHeldPart(); updateControls(); draw();
});

fkMode.addEventListener('click', () => setMode('fk'));
ikMode.addEventListener('click', () => setMode('ik'));

gripperButton.addEventListener('click', () => toggleGripper());
homeButton.addEventListener('click', async () => {
  if (state.animating || state.autoRunning) return;
  state.target = null;
  await animateTo(30, 55, 420);
  setStatus('Home-ish position reached. Good enough for a browser robot.');
});

randomButton.addEventListener('click', () => {
  if (state.animating || state.autoRunning) return;
  setMode('ik');
  for (let i = 0; i < 50; i++) {
    const a = Math.random() * Math.PI * 1.35 - Math.PI * .15;
    const r = 80 + Math.random() * (state.link1 + state.link2 - 100);
    const x = Math.cos(a) * r;
    const y = Math.max(15, Math.sin(a) * r);
    if (solveIK(x, y, true) || solveIK(x, y, false)) {
      setTarget(x, y);
      setStatus('Random reachable target generated. Statistical competence achieved.');
      break;
    }
  }
});

pickPlaceButton.addEventListener('click', autoPickPlace);

updateControls();
setMode('fk');
draw();
