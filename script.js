const canvas = document.getElementById('armCanvas');
const ctx = canvas.getContext('2d');

const theta1 = document.getElementById('theta1');
const theta2 = document.getElementById('theta2');
const theta3 = document.getElementById('theta3');
const link1 = document.getElementById('link1');
const link2 = document.getElementById('link2');
const link3 = document.getElementById('link3');
const toolAngle = document.getElementById('toolAngle');

const theta1Value = document.getElementById('theta1Value');
const theta2Value = document.getElementById('theta2Value');
const theta3Value = document.getElementById('theta3Value');
const link1Value = document.getElementById('link1Value');
const link2Value = document.getElementById('link2Value');
const link3Value = document.getElementById('link3Value');
const toolAngleValue = document.getElementById('toolAngleValue');

const fkMode = document.getElementById('fkMode');
const ikMode = document.getElementById('ikMode');
const dof2Button = document.getElementById('dof2Button');
const dof3Button = document.getElementById('dof3Button');
const elbowUpButton = document.getElementById('elbowUpButton');
const elbowDownButton = document.getElementById('elbowDownButton');
const homeButton = document.getElementById('homeButton');
const randomButton = document.getElementById('randomButton');
const gripperButton = document.getElementById('gripperButton');
const clearTrailButton = document.getElementById('clearTrailButton');
const pickPlaceButton = document.getElementById('pickPlaceButton');
const modeReadout = document.getElementById('modeReadout');
const statusBox = document.getElementById('statusBox');
const orientationGroup = document.querySelector('.orientation-group');
const joint3Controls = [...document.querySelectorAll('.joint3-control')];

const teleTheta1 = document.getElementById('teleTheta1');
const teleTheta2 = document.getElementById('teleTheta2');
const teleTheta3 = document.getElementById('teleTheta3');
const telePhi = document.getElementById('telePhi');
const teleX = document.getElementById('teleX');
const teleY = document.getElementById('teleY');
const teleError = document.getElementById('teleError');
const teleBranch = document.getElementById('teleBranch');
const teleGripper = document.getElementById('teleGripper');
const telePayload = document.getElementById('telePayload');
const teleSolver = document.getElementById('teleSolver');

const LIMITS = {
  theta1: [-160, 160],
  theta2: [-145, 145],
  theta3: [-135, 135]
};

const state = {
  mode: 'fk',
  dof: 3,
  elbow: 'up',
  theta1: 30,
  theta2: 55,
  theta3: -35,
  link1: 180,
  link2: 150,
  link3: 75,
  toolAngle: 0,
  gripperClosed: false,
  target: null,
  heldPart: null,
  selectedPart: null,
  animating: false,
  autoRunning: false,
  trail: [],
  parts: [
    { id: 'PART_01', x: 165, y: 95, r: 16, color: '#ff9b61', orientation: 0 },
    { id: 'PART_02', x: 250, y: 55, r: 14, color: '#f3c969', orientation: 0 },
    { id: 'PART_03', x: 95, y: 145, r: 13, color: '#d98cff', orientation: 0 }
  ],
  dropZone: { x: 285, y: 155, w: 86, h: 74 }
};

function degToRad(deg) { return deg * Math.PI / 180; }
function radToDeg(rad) { return rad * 180 / Math.PI; }
function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }
function normalizeDeg(angle) {
  let a = angle;
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}
function within(value, limits) { return value >= limits[0] && value <= limits[1]; }

function getBase() {
  return { x: canvas.width * 0.24, y: canvas.height * 0.78 };
}
function worldToCanvas(x, y) {
  const base = getBase();
  return { x: base.x + x, y: base.y - y };
}
function canvasToWorld(x, y) {
  const base = getBase();
  return { x: x - base.x, y: base.y - y };
}

function forwardKinematics(t1 = state.theta1, t2 = state.theta2, t3 = state.theta3) {
  const a1 = degToRad(t1);
  const a2 = degToRad(t1 + t2);
  const a3 = degToRad(t1 + t2 + (state.dof === 3 ? t3 : 0));

  const elbow = {
    x: state.link1 * Math.cos(a1),
    y: state.link1 * Math.sin(a1)
  };
  const wrist = {
    x: elbow.x + state.link2 * Math.cos(a2),
    y: elbow.y + state.link2 * Math.sin(a2)
  };
  const end = state.dof === 3
    ? {
        x: wrist.x + state.link3 * Math.cos(a3),
        y: wrist.y + state.link3 * Math.sin(a3)
      }
    : wrist;

  return { elbow, wrist, end, phi: radToDeg(state.dof === 3 ? a3 : a2) };
}

function solve2LinkIK(x, y, elbowBranch = state.elbow) {
  const l1 = state.link1;
  const l2 = state.link2;
  const r2 = x * x + y * y;
  const r = Math.sqrt(r2);
  const minReach = Math.abs(l1 - l2);
  const maxReach = l1 + l2;
  if (r > maxReach + 0.001 || r < minReach - 0.001) return null;

  let c2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  c2 = clamp(c2, -1, 1);
  let s2 = Math.sqrt(Math.max(0, 1 - c2 * c2));
  if (elbowBranch === 'down') s2 *= -1;

  const t2 = Math.atan2(s2, c2);
  const t1 = Math.atan2(y, x) - Math.atan2(l2 * s2, l1 + l2 * c2);
  const d1 = normalizeDeg(radToDeg(t1));
  const d2 = normalizeDeg(radToDeg(t2));

  if (!within(d1, LIMITS.theta1) || !within(d2, LIMITS.theta2)) return null;
  return { theta1: d1, theta2: d2, theta3: state.theta3 };
}

function solveIK(x, y, branch = state.elbow, desiredPhi = state.toolAngle) {
  if (state.dof === 2) return solve2LinkIK(x, y, branch);

  const phi = degToRad(desiredPhi);
  const wristX = x - state.link3 * Math.cos(phi);
  const wristY = y - state.link3 * Math.sin(phi);
  const solution = solve2LinkIK(wristX, wristY, branch);
  if (!solution) return null;

  const t3 = normalizeDeg(desiredPhi - solution.theta1 - solution.theta2);
  if (!within(t3, LIMITS.theta3)) return null;
  return { theta1: solution.theta1, theta2: solution.theta2, theta3: t3 };
}

function alternativeSolution(x, y, desiredPhi = state.toolAngle) {
  const preferred = solveIK(x, y, state.elbow, desiredPhi);
  if (preferred) return preferred;
  const other = state.elbow === 'up' ? 'down' : 'up';
  return solveIK(x, y, other, desiredPhi);
}

function addTrailPoint(force = false) {
  const { end } = forwardKinematics();
  const last = state.trail[state.trail.length - 1];
  if (!force && last && Math.hypot(last.x - end.x, last.y - end.y) < 3) return;
  state.trail.push({ x: end.x, y: end.y });
  if (state.trail.length > 260) state.trail.shift();
}

function updateControls() {
  theta1.value = state.theta1;
  theta2.value = state.theta2;
  theta3.value = state.theta3;
  link1.value = state.link1;
  link2.value = state.link2;
  link3.value = state.link3;
  toolAngle.value = state.toolAngle;

  theta1Value.textContent = `${state.theta1.toFixed(0)}°`;
  theta2Value.textContent = `${state.theta2.toFixed(0)}°`;
  theta3Value.textContent = `${state.theta3.toFixed(0)}°`;
  link1Value.textContent = state.link1.toFixed(0);
  link2Value.textContent = state.link2.toFixed(0);
  link3Value.textContent = state.link3.toFixed(0);
  toolAngleValue.textContent = `${state.toolAngle.toFixed(0)}°`;

  const manual = state.mode === 'fk';
  theta1.disabled = !manual || state.animating;
  theta2.disabled = !manual || state.animating;
  theta3.disabled = state.dof !== 3 || !manual || state.animating;
  link1.disabled = state.animating || state.autoRunning;
  link2.disabled = state.animating || state.autoRunning;
  link3.disabled = state.dof !== 3 || state.animating || state.autoRunning;
  toolAngle.disabled = state.dof !== 3 || state.mode !== 'ik' || state.animating || state.autoRunning;

  joint3Controls.forEach(el => el.classList.toggle('hidden-control', state.dof !== 3));
  orientationGroup.classList.toggle('disabled-group', state.dof !== 3);

  dof2Button.classList.toggle('active', state.dof === 2);
  dof3Button.classList.toggle('active', state.dof === 3);
  elbowUpButton.classList.toggle('active', state.elbow === 'up');
  elbowDownButton.classList.toggle('active', state.elbow === 'down');

  modeReadout.textContent = `MODE: ${state.mode.toUpperCase()} // ${state.dof}-DOF`;
}

function setStatus(text) { statusBox.textContent = text; }

function setMode(mode) {
  if (state.autoRunning && mode !== 'ik') return;
  state.mode = mode;
  fkMode.classList.toggle('active', mode === 'fk');
  ikMode.classList.toggle('active', mode === 'ik');
  teleSolver.textContent = mode === 'fk' ? 'MANUAL FK' : `${state.dof}R ANALYTIC IK`;
  updateControls();
  setStatus(mode === 'fk'
    ? 'Manual forward kinematics active. Move the available joint sliders.'
    : 'Inverse kinematics active. Click a target in the workspace.');
  draw();
}

function setDof(dof) {
  if (state.animating || state.autoRunning) return;
  state.dof = dof;
  state.target = null;
  if (dof === 2) state.theta3 = 0;
  else if (Math.abs(state.theta3) < 1) state.theta3 = -35;
  state.trail = [];
  addTrailPoint(true);
  teleSolver.textContent = state.mode === 'fk' ? 'MANUAL FK' : `${dof}R ANALYTIC IK`;
  setStatus(`${dof}-DOF model loaded. ${dof === 3 ? 'Tool orientation control enabled.' : 'Third joint removed from the equation.'}`);
  updateControls();
  draw();
}

function setElbow(branch) {
  if (state.animating || state.autoRunning) return;
  state.elbow = branch;
  teleBranch.textContent = branch.toUpperCase();
  if (state.mode === 'ik' && state.target) {
    const solution = solveIK(state.target.x, state.target.y, branch, state.toolAngle);
    if (solution) animateTo(solution.theta1, solution.theta2, solution.theta3, 460);
    else setStatus(`ELBOW ${branch.toUpperCase()} has no valid solution inside the joint limits.`);
  }
  updateControls();
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

function setTarget(x, y, selectedPart = null, desiredPhi = state.toolAngle) {
  state.target = { x, y };
  state.selectedPart = selectedPart;
  const solution = solveIK(x, y, state.elbow, desiredPhi);

  if (!solution) {
    teleError.textContent = 'OUT';
    teleSolver.textContent = 'NO VALID SOLUTION';
    setStatus('TARGET / ORIENTATION OUTSIDE VALID WORKSPACE // joint limits say no XD');
    draw();
    return false;
  }

  teleSolver.textContent = `${state.dof}R IK SOLVED`;
  animateTo(solution.theta1, solution.theta2, solution.theta3, 460);
  return true;
}

function animateTo(targetT1, targetT2, targetT3 = state.theta3, duration = 460) {
  if (state.animating) return Promise.resolve(false);
  state.animating = true;
  updateControls();

  const startT1 = state.theta1;
  const startT2 = state.theta2;
  const startT3 = state.theta3;
  const start = performance.now();

  return new Promise(resolve => {
    function step(now) {
      const p = clamp((now - start) / duration, 0, 1);
      const eased = 1 - Math.pow(1 - p, 3);
      state.theta1 = startT1 + (targetT1 - startT1) * eased;
      state.theta2 = startT2 + (targetT2 - startT2) * eased;
      state.theta3 = startT3 + (targetT3 - startT3) * eased;
      updateHeldPart();
      addTrailPoint();
      updateControls();
      draw();
      if (p < 1) requestAnimationFrame(step);
      else {
        state.animating = false;
        addTrailPoint(true);
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
  const { end, phi } = forwardKinematics();
  state.heldPart.x = end.x;
  state.heldPart.y = end.y;
  state.heldPart.orientation = phi;
}

function angularDifference(a, b) {
  return Math.abs(normalizeDeg(a - b));
}

function toggleGripper(force) {
  const shouldClose = typeof force === 'boolean' ? force : !state.gripperClosed;
  state.gripperClosed = shouldClose;

  if (shouldClose && !state.heldPart) {
    const { end, phi } = forwardKinematics();
    const candidate = state.selectedPart || nearestPart(end);
    const positionOK = candidate && Math.hypot(candidate.x - end.x, candidate.y - end.y) <= candidate.r + 22;
    const orientationOK = state.dof === 2 || (candidate && angularDifference(phi, candidate.orientation || 0) <= 35);

    if (candidate && positionOK && orientationOK) {
      state.heldPart = candidate;
      state.selectedPart = candidate;
      setStatus(`${candidate.id} captured. Position and gripper orientation accepted.`);
    } else if (candidate && positionOK && !orientationOK) {
      setStatus('Gripper is close enough, but orientation is outside the ±35° grasp tolerance.');
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

function solveWaypoint(x, y, phi = 0) {
  return solveIK(x, y, state.elbow, phi) || solveIK(x, y, state.elbow === 'up' ? 'down' : 'up', phi);
}

async function autoPickPlace() {
  if (state.autoRunning || state.animating) return;
  const part = state.selectedPart || state.parts.find(p => p !== state.heldPart);
  if (!part) return;

  state.autoRunning = true;
  state.selectedPart = part;
  state.mode = 'ik';
  fkMode.classList.remove('active');
  ikMode.classList.add('active');
  pickPlaceButton.disabled = true;
  const cyclePhi = state.dof === 3 ? 0 : state.toolAngle;

  const abovePick = solveWaypoint(part.x, part.y + 45, cyclePhi);
  const pick = solveWaypoint(part.x, part.y, cyclePhi);
  const dz = state.dropZone;
  const aboveDrop = solveWaypoint(dz.x, dz.y + 55, cyclePhi);
  const drop = solveWaypoint(dz.x, dz.y, cyclePhi);

  if (!abovePick || !pick || !aboveDrop || !drop) {
    setStatus('Auto cycle aborted. A waypoint violates reachability or joint limits.');
    state.autoRunning = false;
    pickPlaceButton.disabled = false;
    updateControls();
    return;
  }

  if (state.dof === 3) state.toolAngle = 0;
  updateControls();
  setStatus(`AUTO CYCLE // approaching ${part.id}`);
  await animateTo(abovePick.theta1, abovePick.theta2, abovePick.theta3, 450);
  await animateTo(pick.theta1, pick.theta2, pick.theta3, 340);
  toggleGripper(true);
  await new Promise(r => setTimeout(r, 220));
  await animateTo(abovePick.theta1, abovePick.theta2, abovePick.theta3, 340);
  setStatus('AUTO CYCLE // transporting payload');
  await animateTo(aboveDrop.theta1, aboveDrop.theta2, aboveDrop.theta3, 540);
  await animateTo(drop.theta1, drop.theta2, drop.theta3, 320);
  toggleGripper(false);
  await new Promise(r => setTimeout(r, 180));
  await animateTo(aboveDrop.theta1, aboveDrop.theta2, aboveDrop.theta3, 320);
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
  ctx.strokeStyle = 'rgba(105,240,193,.20)';
  ctx.beginPath(); ctx.moveTo(0, base.y); ctx.lineTo(canvas.width, base.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(base.x, 0); ctx.lineTo(base.x, canvas.height); ctx.stroke();
  ctx.restore();
}

function drawWorkspaceLimits() {
  const base = getBase();
  const maxReach = state.link1 + state.link2 + (state.dof === 3 ? state.link3 : 0);
  const minCore = Math.max(0, Math.abs(state.link1 - state.link2) - (state.dof === 3 ? state.link3 : 0));

  ctx.save();
  ctx.fillStyle = 'rgba(105,240,193,.025)';
  ctx.strokeStyle = 'rgba(105,240,193,.16)';
  ctx.setLineDash([7,7]);
  ctx.beginPath();
  ctx.arc(base.x, base.y, maxReach, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  if (minCore > 2) {
    ctx.beginPath();
    ctx.arc(base.x, base.y, minCore, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.fillStyle = 'rgba(105,240,193,.38)';
  ctx.font = '13px Courier New';
  ctx.fillText(`MAX REACH ${maxReach.toFixed(0)}`, base.x + 14, Math.max(18, base.y - maxReach + 18));
  ctx.restore();
}

function drawTrail() {
  if (state.trail.length < 2) return;
  ctx.save();
  ctx.strokeStyle = 'rgba(217,140,255,.62)';
  ctx.lineWidth = 2;
  ctx.setLineDash([4,4]);
  ctx.beginPath();
  state.trail.forEach((pt, index) => {
    const p = worldToCanvas(pt.x, pt.y);
    if (index === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawDropZone() {
  const dz = state.dropZone;
  const p = worldToCanvas(dz.x, dz.y);
  ctx.save();
  ctx.strokeStyle = '#7ab6ff';
  ctx.fillStyle = 'rgba(122,182,255,.06)';
  ctx.lineWidth = 2;
  ctx.setLineDash([8,5]);
  ctx.fillRect(p.x - dz.w/2, p.y - dz.h/2, dz.w, dz.h);
  ctx.strokeRect(p.x - dz.w/2, p.y - dz.h/2, dz.w, dz.h);
  ctx.setLineDash([]);
  ctx.fillStyle = '#7ab6ff';
  ctx.font = '13px Courier New';
  ctx.fillText('DROP ZONE', p.x - dz.w/2, p.y - dz.h/2 - 8);
  ctx.restore();
}

function drawParts() {
  for (const part of state.parts) {
    const p = worldToCanvas(part.x, part.y);
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-degToRad(part.orientation || 0));
    ctx.fillStyle = part.color;
    ctx.strokeStyle = state.selectedPart === part ? '#ffffff' : 'rgba(255,255,255,.28)';
    ctx.lineWidth = state.selectedPart === part ? 3 : 1;
    ctx.fillRect(-part.r, -part.r, part.r * 2, part.r * 2);
    ctx.strokeRect(-part.r, -part.r, part.r * 2, part.r * 2);
    ctx.restore();
    ctx.save();
    ctx.fillStyle = '#dceae5';
    ctx.font = '12px Courier New';
    ctx.fillText(part.id, p.x - part.r, p.y + part.r + 16);
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
  if (state.dof === 3) {
    const a = degToRad(state.toolAngle);
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x + Math.cos(a) * 34, p.y - Math.sin(a) * 34);
    ctx.stroke();
  }
  ctx.restore();
}

function drawArm() {
  const base = getBase();
  const { elbow, wrist, end, phi } = forwardKinematics();
  const e = worldToCanvas(elbow.x, elbow.y);
  const w = worldToCanvas(wrist.x, wrist.y);
  const tip = worldToCanvas(end.x, end.y);

  ctx.save();
  ctx.lineCap = 'round';
  ctx.lineWidth = 18;
  ctx.strokeStyle = '#386e62';
  ctx.beginPath(); ctx.moveTo(base.x, base.y); ctx.lineTo(e.x, e.y); ctx.stroke();
  ctx.strokeStyle = '#4cae98';
  ctx.beginPath(); ctx.moveTo(e.x, e.y); ctx.lineTo(w.x, w.y); ctx.stroke();
  if (state.dof === 3) {
    ctx.strokeStyle = '#5fbfb0';
    ctx.lineWidth = 15;
    ctx.beginPath(); ctx.moveTo(w.x, w.y); ctx.lineTo(tip.x, tip.y); ctx.stroke();
  }

  const joints = state.dof === 3 ? [base, e, w] : [base, e];
  for (const joint of joints) {
    ctx.fillStyle = '#07100f';
    ctx.strokeStyle = '#69f0c1';
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(joint.x, joint.y, 13, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  }

  const totalAngle = degToRad(phi);
  const perpX = Math.sin(totalAngle);
  const perpY = Math.cos(totalAngle);
  const spread = state.gripperClosed ? 8 : 17;
  const fingerLen = 27;
  ctx.strokeStyle = '#dceae5';
  ctx.lineWidth = 5;
  ctx.lineCap = 'square';
  for (const side of [-1,1]) {
    const sx = tip.x + perpX * spread * side;
    const sy = tip.y + perpY * spread * side;
    const ex = sx + Math.cos(totalAngle) * fingerLen;
    const ey = sy - Math.sin(totalAngle) * fingerLen;
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
  }

  ctx.fillStyle = '#69f0c1';
  ctx.font = '13px Courier New';
  ctx.fillText('J1', base.x + 16, base.y + 4);
  ctx.fillText('J2', e.x + 16, e.y + 4);
  if (state.dof === 3) ctx.fillText('J3', w.x + 16, w.y + 4);
  ctx.fillText('TCP', tip.x + 12, tip.y - 12);
  ctx.restore();
}

function updateTelemetry() {
  const { end, phi } = forwardKinematics();
  teleTheta1.textContent = `${state.theta1.toFixed(1)}°`;
  teleTheta2.textContent = `${state.theta2.toFixed(1)}°`;
  teleTheta3.textContent = state.dof === 3 ? `${state.theta3.toFixed(1)}°` : 'N/A';
  telePhi.textContent = `${normalizeDeg(phi).toFixed(1)}°`;
  teleX.textContent = end.x.toFixed(1);
  teleY.textContent = end.y.toFixed(1);
  teleBranch.textContent = state.elbow.toUpperCase();
  teleGripper.textContent = state.gripperClosed ? 'CLOSED' : 'OPEN';
  telePayload.textContent = state.heldPart ? state.heldPart.id : 'NONE';
  if (state.target) teleError.textContent = Math.hypot(end.x - state.target.x, end.y - state.target.y).toFixed(2);
  else teleError.textContent = '—';
}

function draw() {
  ctx.clearRect(0,0,canvas.width,canvas.height);
  drawGrid();
  drawWorkspaceLimits();
  drawTrail();
  drawDropZone();
  drawTarget();
  drawParts();
  drawArm();
  updateTelemetry();
}

function getCanvasPointer(event) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * canvas.width / rect.width,
    y: (event.clientY - rect.top) * canvas.height / rect.height
  };
}

canvas.addEventListener('pointerdown', event => {
  if (state.animating || state.autoRunning) return;
  const p = getCanvasPointer(event);
  const world = canvasToWorld(p.x,p.y);
  const part = nearestPart(world);

  if (part) {
    state.selectedPart = part;
    setStatus(`${part.id} selected. Click AUTO PICK + PLACE or move the gripper near it.`);
    if (state.mode === 'ik') {
      if (state.dof === 3) state.toolAngle = part.orientation || 0;
      setTarget(part.x, part.y, part, state.toolAngle);
    }
    draw();
    return;
  }

  if (state.mode === 'ik') setTarget(world.x, world.y);
});

function manualJointInput() {
  state.theta1 = Number(theta1.value);
  state.theta2 = Number(theta2.value);
  if (state.dof === 3) state.theta3 = Number(theta3.value);
  state.target = null;
  updateHeldPart();
  addTrailPoint();
  updateControls();
  draw();
}

theta1.addEventListener('input', manualJointInput);
theta2.addEventListener('input', manualJointInput);
theta3.addEventListener('input', manualJointInput);

link1.addEventListener('input', () => {
  state.link1 = Number(link1.value);
  state.target = null;
  state.trail = [];
  updateHeldPart(); addTrailPoint(true); updateControls(); draw();
});
link2.addEventListener('input', () => {
  state.link2 = Number(link2.value);
  state.target = null;
  state.trail = [];
  updateHeldPart(); addTrailPoint(true); updateControls(); draw();
});
link3.addEventListener('input', () => {
  state.link3 = Number(link3.value);
  state.target = null;
  state.trail = [];
  updateHeldPart(); addTrailPoint(true); updateControls(); draw();
});

toolAngle.addEventListener('input', () => {
  state.toolAngle = Number(toolAngle.value);
  toolAngleValue.textContent = `${state.toolAngle.toFixed(0)}°`;
  if (state.mode === 'ik' && state.target && !state.animating) {
    const solution = solveIK(state.target.x, state.target.y, state.elbow, state.toolAngle);
    if (solution) animateTo(solution.theta1, solution.theta2, solution.theta3, 340);
    else {
      teleSolver.textContent = 'NO VALID SOLUTION';
      setStatus('Requested TCP orientation cannot be reached at this target within joint limits.');
      draw();
    }
  }
});

fkMode.addEventListener('click', () => setMode('fk'));
ikMode.addEventListener('click', () => setMode('ik'));
dof2Button.addEventListener('click', () => setDof(2));
dof3Button.addEventListener('click', () => setDof(3));
elbowUpButton.addEventListener('click', () => setElbow('up'));
elbowDownButton.addEventListener('click', () => setElbow('down'));
gripperButton.addEventListener('click', () => toggleGripper());
clearTrailButton.addEventListener('click', () => {
  state.trail = [];
  addTrailPoint(true);
  setStatus('TCP trajectory cleared. Evidence successfully destroyed.');
  draw();
});

homeButton.addEventListener('click', async () => {
  if (state.animating || state.autoRunning) return;
  state.target = null;
  const homeT3 = state.dof === 3 ? -35 : 0;
  await animateTo(30,55,homeT3,440);
  setStatus('Home-ish position reached. Good enough for a browser robot.');
});

randomButton.addEventListener('click', () => {
  if (state.animating || state.autoRunning) return;
  setMode('ik');
  for (let i = 0; i < 80; i++) {
    const maxReach = state.link1 + state.link2 + (state.dof === 3 ? state.link3 : 0);
    const a = Math.random() * Math.PI * 1.35 - Math.PI * .15;
    const r = 90 + Math.random() * (maxReach - 120);
    const x = Math.cos(a) * r;
    const y = Math.max(15, Math.sin(a) * r);
    const phi = state.dof === 3 ? Math.round(-75 + Math.random() * 150) : state.toolAngle;
    const solution = solveIK(x,y,state.elbow,phi);
    if (solution) {
      state.toolAngle = phi;
      updateControls();
      setTarget(x,y,null,phi);
      setStatus('Random valid target generated inside reachability and joint limits.');
      break;
    }
  }
});

pickPlaceButton.addEventListener('click', autoPickPlace);

addTrailPoint(true);
updateControls();
setMode('fk');
draw();
