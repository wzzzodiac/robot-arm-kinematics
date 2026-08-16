const canvas = document.getElementById('armCanvas');
const ctx = canvas.getContext('2d');

const $ = id => document.getElementById(id);
const theta1 = $('theta1'), theta2 = $('theta2'), theta3 = $('theta3');
const link1 = $('link1'), link2 = $('link2'), link3 = $('link3'), toolAngle = $('toolAngle');
const theta1Value = $('theta1Value'), theta2Value = $('theta2Value'), theta3Value = $('theta3Value');
const link1Value = $('link1Value'), link2Value = $('link2Value'), link3Value = $('link3Value'), toolAngleValue = $('toolAngleValue');
const fkMode = $('fkMode'), ikMode = $('ikMode'), dof2Button = $('dof2Button'), dof3Button = $('dof3Button');
const elbowUpButton = $('elbowUpButton'), elbowDownButton = $('elbowDownButton');
const homeButton = $('homeButton'), randomButton = $('randomButton'), gripperButton = $('gripperButton');
const clearTrailButton = $('clearTrailButton'), pickPlaceButton = $('pickPlaceButton');
const modeReadout = $('modeReadout'), statusBox = $('statusBox');
const orientationGroup = document.querySelector('.orientation-group');
const joint3Controls = [...document.querySelectorAll('.joint3-control')];

const teleTheta1 = $('teleTheta1'), teleTheta2 = $('teleTheta2'), teleTheta3 = $('teleTheta3'), telePhi = $('telePhi');
const teleX = $('teleX'), teleY = $('teleY'), teleError = $('teleError'), teleBranch = $('teleBranch');
const teleGripper = $('teleGripper'), telePayload = $('telePayload'), teleSolver = $('teleSolver');
const teleCollision = $('teleCollision'), teleProgram = $('teleProgram');

const toggleObstaclesButton = $('toggleObstaclesButton'), randomObstaclesButton = $('randomObstaclesButton'), resetObstaclesButton = $('resetObstaclesButton');
const obstacleCount = $('obstacleCount');
const savePoseButton = $('savePoseButton'), deletePoseButton = $('deletePoseButton'), clearProgramButton = $('clearProgramButton');
const runProgramButton = $('runProgramButton'), stopProgramButton = $('stopProgramButton'), programList = $('programList'), poseCount = $('poseCount');

const LIMITS = { theta1: [-160,160], theta2: [-145,145], theta3: [-135,135] };
const LINK_RADIUS = 10;
const MAX_POSES = 8;

const DEFAULT_OBSTACLES = [
  { id:'OBS_A', x:300, y:285, w:72, h:62 },
  { id:'OBS_B', x:395, y:205, w:82, h:70 },
  { id:'OBS_C', x:365, y:75, w:64, h:82 }
];

const state = {
  mode:'fk', dof:3, elbow:'up', theta1:30, theta2:55, theta3:-35,
  link1:180, link2:150, link3:75, toolAngle:0,
  gripperClosed:false, target:null, heldPart:null, selectedPart:null,
  animating:false, autoRunning:false, programRunning:false, stopRequested:false,
  trail:[], obstaclesVisible:true, obstacles:DEFAULT_OBSTACLES.map(o=>({...o})), taughtPoses:[], activePose:-1,
  parts:[
    {id:'PART_01',x:165,y:95,r:16,color:'#ff9b61',orientation:0},
    {id:'PART_02',x:250,y:55,r:14,color:'#f3c969',orientation:0},
    {id:'PART_03',x:95,y:145,r:13,color:'#d98cff',orientation:0}
  ],
  dropZone:{x:285,y:155,w:86,h:74}
};

function degToRad(v){return v*Math.PI/180} function radToDeg(v){return v*180/Math.PI}
function clamp(v,a,b){return Math.max(a,Math.min(b,v))}
function normalizeDeg(a){while(a>180)a-=360;while(a<-180)a+=360;return a}
function within(v,l){return v>=l[0]&&v<=l[1]}
function sleep(ms){return new Promise(r=>setTimeout(r,ms))}

function getBase(){return{x:canvas.width*.22,y:canvas.height*.80}}
function worldToCanvas(x,y){const b=getBase();return{x:b.x+x,y:b.y-y}}
function canvasToWorld(x,y){const b=getBase();return{x:x-b.x,y:b.y-y}}

function fkFor(t1,t2,t3=state.theta3,dof=state.dof){
  const a1=degToRad(t1), a2=degToRad(t1+t2), a3=degToRad(t1+t2+(dof===3?t3:0));
  const p0={x:0,y:0};
  const p1={x:state.link1*Math.cos(a1),y:state.link1*Math.sin(a1)};
  const p2={x:p1.x+state.link2*Math.cos(a2),y:p1.y+state.link2*Math.sin(a2)};
  const p3=dof===3?{x:p2.x+state.link3*Math.cos(a3),y:p2.y+state.link3*Math.sin(a3)}:p2;
  return {base:p0,elbow:p1,wrist:p2,end:p3,phi:radToDeg(dof===3?a3:a2)};
}
function forwardKinematics(){return fkFor(state.theta1,state.theta2,state.theta3,state.dof)}

function solve2LinkIK(x,y,branch=state.elbow){
  const l1=state.link1,l2=state.link2,r2=x*x+y*y,r=Math.sqrt(r2);
  if(r>l1+l2+.001||r<Math.abs(l1-l2)-.001)return null;
  let c2=clamp((r2-l1*l1-l2*l2)/(2*l1*l2),-1,1),s2=Math.sqrt(Math.max(0,1-c2*c2));
  if(branch==='down')s2*=-1;
  const t2=Math.atan2(s2,c2),t1=Math.atan2(y,x)-Math.atan2(l2*s2,l1+l2*c2);
  const d1=normalizeDeg(radToDeg(t1)),d2=normalizeDeg(radToDeg(t2));
  if(!within(d1,LIMITS.theta1)||!within(d2,LIMITS.theta2))return null;
  return{theta1:d1,theta2:d2,theta3:state.theta3,branch};
}
function solveIK(x,y,branch=state.elbow,phi=state.toolAngle){
  if(state.dof===2)return solve2LinkIK(x,y,branch);
  const a=degToRad(phi),wx=x-state.link3*Math.cos(a),wy=y-state.link3*Math.sin(a);
  const s=solve2LinkIK(wx,wy,branch);if(!s)return null;
  const t3=normalizeDeg(phi-s.theta1-s.theta2);if(!within(t3,LIMITS.theta3))return null;
  return{theta1:s.theta1,theta2:s.theta2,theta3:t3,branch};
}

function pointInRect(p,r,pad=0){return p.x>=r.x-r.w/2-pad&&p.x<=r.x+r.w/2+pad&&p.y>=r.y-r.h/2-pad&&p.y<=r.y+r.h/2+pad}
function segmentIntersectsRect(a,b,r,pad=0){
  const xmin=r.x-r.w/2-pad,xmax=r.x+r.w/2+pad,ymin=r.y-r.h/2-pad,ymax=r.y+r.h/2+pad;
  if(pointInRect(a,r,pad)||pointInRect(b,r,pad))return true;
  let t0=0,t1=1,dx=b.x-a.x,dy=b.y-a.y;
  const checks=[[-dx,a.x-xmin],[dx,xmax-a.x],[-dy,a.y-ymin],[dy,ymax-a.y]];
  for(const [p,q] of checks){if(p===0){if(q<0)return false}else{const t=q/p;if(p<0){if(t>t1)return false;if(t>t0)t0=t}else{if(t<t0)return false;if(t<t1)t1=t}}}
  return true;
}
function collisionForPose(t1,t2,t3=state.theta3,dof=state.dof){
  if(!state.obstaclesVisible)return null;
  const k=fkFor(t1,t2,t3,dof),pts=dof===3?[k.base,k.elbow,k.wrist,k.end]:[k.base,k.elbow,k.end];
  for(let i=0;i<pts.length-1;i++)for(const o of state.obstacles)if(segmentIntersectsRect(pts[i],pts[i+1],o,LINK_RADIUS))return{obstacle:o,link:i+1};
  return null;
}
function currentCollision(){return collisionForPose(state.theta1,state.theta2,state.theta3,state.dof)}
function pathCollision(a,b,steps=36){
  for(let i=1;i<=steps;i++){const p=i/steps;const t1=a.theta1+(b.theta1-a.theta1)*p,t2=a.theta2+(b.theta2-a.theta2)*p,t3=a.theta3+(b.theta3-a.theta3)*p;const hit=collisionForPose(t1,t2,t3,b.dof??state.dof);if(hit)return hit}
  return null;
}
function chooseIK(x,y,phi=state.toolAngle){
  const branches=[state.elbow,state.elbow==='up'?'down':'up'];
  for(const branch of branches){const s=solveIK(x,y,branch,phi);if(s&&!collisionForPose(s.theta1,s.theta2,s.theta3,state.dof))return s}
  return null;
}

function addTrailPoint(force=false){const{end}=forwardKinematics();const last=state.trail.at(-1);if(!force&&last&&Math.hypot(last.x-end.x,last.y-end.y)<3)return;state.trail.push({...end});if(state.trail.length>300)state.trail.shift()}
function updateHeldPart(){if(!state.heldPart)return;const{end,phi}=forwardKinematics();Object.assign(state.heldPart,{x:end.x,y:end.y,orientation:phi})}
function angularDifference(a,b){return Math.abs(normalizeDeg(a-b))}
function setStatus(t){statusBox.textContent=t}

function updateControls(){
  theta1.value=state.theta1;theta2.value=state.theta2;theta3.value=state.theta3;link1.value=state.link1;link2.value=state.link2;link3.value=state.link3;toolAngle.value=state.toolAngle;
  theta1Value.textContent=`${state.theta1.toFixed(0)}°`;theta2Value.textContent=`${state.theta2.toFixed(0)}°`;theta3Value.textContent=`${state.theta3.toFixed(0)}°`;
  link1Value.textContent=state.link1.toFixed(0);link2Value.textContent=state.link2.toFixed(0);link3Value.textContent=state.link3.toFixed(0);toolAngleValue.textContent=`${state.toolAngle.toFixed(0)}°`;
  const busy=state.animating||state.autoRunning||state.programRunning,manual=state.mode==='fk';
  theta1.disabled=!manual||busy;theta2.disabled=!manual||busy;theta3.disabled=state.dof!==3||!manual||busy;
  link1.disabled=busy;link2.disabled=busy;link3.disabled=state.dof!==3||busy;toolAngle.disabled=state.dof!==3||state.mode!=='ik'||busy;
  joint3Controls.forEach(el=>el.classList.toggle('hidden-control',state.dof!==3));orientationGroup.classList.toggle('disabled-group',state.dof!==3);
  dof2Button.classList.toggle('active',state.dof===2);dof3Button.classList.toggle('active',state.dof===3);elbowUpButton.classList.toggle('active',state.elbow==='up');elbowDownButton.classList.toggle('active',state.elbow==='down');
  fkMode.classList.toggle('active',state.mode==='fk');ikMode.classList.toggle('active',state.mode==='ik');modeReadout.textContent=`MODE: ${state.mode.toUpperCase()} // ${state.dof}-DOF`;
  pickPlaceButton.disabled=busy;savePoseButton.disabled=busy||state.taughtPoses.length>=MAX_POSES;deletePoseButton.disabled=busy||!state.taughtPoses.length;clearProgramButton.disabled=busy||!state.taughtPoses.length;runProgramButton.disabled=busy||!state.taughtPoses.length;stopProgramButton.disabled=!state.programRunning;
}
function setMode(mode){if(state.autoRunning||state.programRunning)return;state.mode=mode;teleSolver.textContent=mode==='fk'?'MANUAL FK':`${state.dof}R ANALYTIC IK`;updateControls();setStatus(mode==='fk'?'Manual FK active. Collision guard is watching every link.':'IK active. Click a target; collision-free solutions only.');draw()}
function setDof(dof){if(state.animating||state.autoRunning||state.programRunning)return;state.dof=dof;state.target=null;if(dof===2)state.theta3=0;else if(Math.abs(state.theta3)<1)state.theta3=-35;state.trail=[];addTrailPoint(true);updateControls();setStatus(`${dof}-DOF model loaded.`);draw()}
function setElbow(branch){if(state.animating||state.autoRunning||state.programRunning)return;state.elbow=branch;teleBranch.textContent=branch.toUpperCase();if(state.mode==='ik'&&state.target){const s=chooseIK(state.target.x,state.target.y,state.toolAngle);if(s)animateTo(s.theta1,s.theta2,s.theta3,460);else setStatus(`ELBOW ${branch.toUpperCase()} cannot reach that target safely.`)}updateControls();draw()}

function nearestPart(w){let best=null,bd=Infinity;for(const p of state.parts){if(state.heldPart===p)continue;const d=Math.hypot(w.x-p.x,w.y-p.y);if(d<p.r+18&&d<bd){best=p;bd=d}}return best}
function setTarget(x,y,part=null,phi=state.toolAngle){state.target={x,y};state.selectedPart=part;const s=chooseIK(x,y,phi);if(!s){teleError.textContent='OUT';teleSolver.textContent='NO SAFE SOLUTION';setStatus('TARGET REJECTED // unreachable, joint-limited or blocked by obstacle.');draw();return false}state.elbow=s.branch;teleSolver.textContent=`${state.dof}R IK SOLVED`;animateTo(s.theta1,s.theta2,s.theta3,460);return true}

function animateTo(t1,t2,t3=state.theta3,duration=460){
  if(state.animating)return Promise.resolve(false);
  const target={theta1:t1,theta2:t2,theta3:t3,dof:state.dof},startPose={theta1:state.theta1,theta2:state.theta2,theta3:state.theta3,dof:state.dof};
  const hit=pathCollision(startPose,target,Math.max(24,Math.round(duration/18)));
  if(hit){setStatus(`MOTION BLOCKED // LINK ${hit.link} would collide with ${hit.obstacle.id}.`);teleSolver.textContent='COLLISION BLOCK';draw();return Promise.resolve(false)}
  state.animating=true;updateControls();const started=performance.now();
  return new Promise(resolve=>{function step(now){if(state.stopRequested&&state.programRunning){state.animating=false;updateControls();resolve(false);return}const p=clamp((now-started)/duration,0,1),e=1-Math.pow(1-p,3);state.theta1=startPose.theta1+(t1-startPose.theta1)*e;state.theta2=startPose.theta2+(t2-startPose.theta2)*e;state.theta3=startPose.theta3+(t3-startPose.theta3)*e;updateHeldPart();addTrailPoint();updateControls();draw();if(p<1)requestAnimationFrame(step);else{state.animating=false;addTrailPoint(true);updateControls();draw();resolve(true)}}requestAnimationFrame(step)})
}

function toggleGripper(force){const close=typeof force==='boolean'?force:!state.gripperClosed;state.gripperClosed=close;if(close&&!state.heldPart){const{end,phi}=forwardKinematics(),c=state.selectedPart||nearestPart(end),pos=c&&Math.hypot(c.x-end.x,c.y-end.y)<=c.r+22,ori=state.dof===2||(c&&angularDifference(phi,c.orientation||0)<=35);if(c&&pos&&ori){state.heldPart=c;state.selectedPart=c;setStatus(`${c.id} captured.`)}else if(c&&pos)setStatus('Gripper position is valid, orientation is not.');else setStatus('Gripper closed. No payload acquired.')}if(!close&&state.heldPart){const r=state.heldPart;state.heldPart=null;const z=state.dropZone,inside=pointInRect(r,z,0);setStatus(inside?`${r.id} placed successfully.`:`${r.id} released outside drop zone.`)}gripperButton.textContent=state.gripperClosed?'OPEN GRIPPER':'CLOSE GRIPPER';draw()}

async function autoPickPlace(){
  if(state.autoRunning||state.animating||state.programRunning)return;const part=state.selectedPart||state.parts.find(p=>p!==state.heldPart);if(!part)return;
  state.autoRunning=true;state.mode='ik';state.selectedPart=part;updateControls();const phi=state.dof===3?0:state.toolAngle;const dz=state.dropZone;
  const waypoints=[[part.x,part.y+42],[part.x,part.y],[part.x,part.y+42],[dz.x,dz.y+55],[dz.x,dz.y],[dz.x,dz.y+55]];
  const sols=waypoints.map(([x,y])=>chooseIK(x,y,phi));if(sols.some(s=>!s)){setStatus('AUTO CYCLE ABORTED // waypoint blocked or unreachable.');state.autoRunning=false;updateControls();return}
  for(let i=0;i<sols.length;i++){const s=sols[i];state.elbow=s.branch;const ok=await animateTo(s.theta1,s.theta2,s.theta3,i===3?560:360);if(!ok){setStatus('AUTO CYCLE ABORTED // collision guard intervened.');state.autoRunning=false;updateControls();return}if(i===1){toggleGripper(true);await sleep(180)}if(i===4){toggleGripper(false);await sleep(160)}}
  setStatus(`AUTO CYCLE COMPLETE // ${part.id} moved.`);state.autoRunning=false;updateControls()
}

function savePose(){if(state.taughtPoses.length>=MAX_POSES||state.animating||state.autoRunning||state.programRunning)return;state.taughtPoses.push({theta1:state.theta1,theta2:state.theta2,theta3:state.theta3,dof:state.dof,gripperClosed:state.gripperClosed});renderProgram();setStatus(`POSE P${String(state.taughtPoses.length).padStart(2,'0')} taught.`);updateControls()}
function renderProgram(){poseCount.textContent=`${state.taughtPoses.length} / ${MAX_POSES} POSES`;if(!state.taughtPoses.length){programList.innerHTML='<div class="program-empty">NO TAUGHT POSITIONS</div>';return}programList.innerHTML=state.taughtPoses.map((p,i)=>`<div class="program-row ${i===state.activePose?'active':''}"><span class="program-index">P${String(i+1).padStart(2,'0')}</span><span class="program-pose">${p.dof}DOF // J1 ${p.theta1.toFixed(0)}° // J2 ${p.theta2.toFixed(0)}°${p.dof===3?` // J3 ${p.theta3.toFixed(0)}°`:''}</span><span class="program-grip">${p.gripperClosed?'GRIP CLOSED':'GRIP OPEN'}</span></div>`).join('')}
async function runProgram(){
  if(!state.taughtPoses.length||state.programRunning||state.autoRunning||state.animating)return;state.programRunning=true;state.stopRequested=false;teleProgram.textContent='RUNNING';setStatus('PROGRAM START // replaying taught poses.');updateControls();
  for(let i=0;i<state.taughtPoses.length;i++){if(state.stopRequested)break;state.activePose=i;renderProgram();const p=state.taughtPoses[i];if(state.dof!==p.dof){state.dof=p.dof;if(p.dof===2)state.theta3=0;updateControls()}const ok=await animateTo(p.theta1,p.theta2,p.theta3,520);if(!ok){if(!state.stopRequested)setStatus(`PROGRAM ABORTED AT P${String(i+1).padStart(2,'0')} // unsafe motion.`);break}if(state.gripperClosed!==p.gripperClosed){toggleGripper(p.gripperClosed);await sleep(160)}await sleep(120)}
  const stopped=state.stopRequested;state.programRunning=false;state.stopRequested=false;state.activePose=-1;teleProgram.textContent=stopped?'STOPPED':'IDLE';renderProgram();updateControls();if(stopped)setStatus('PROGRAM STOPPED BY OPERATOR.');else if(teleSolver.textContent!=='COLLISION BLOCK')setStatus('PROGRAM COMPLETE // sequence replay finished.')
}

function drawGrid(){ctx.save();ctx.strokeStyle='rgba(105,240,193,.055)';for(let x=0;x<=canvas.width;x+=40){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,canvas.height);ctx.stroke()}for(let y=0;y<=canvas.height;y+=40){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(canvas.width,y);ctx.stroke()}const b=getBase();ctx.strokeStyle='rgba(105,240,193,.18)';ctx.beginPath();ctx.moveTo(0,b.y);ctx.lineTo(canvas.width,b.y);ctx.stroke();ctx.beginPath();ctx.moveTo(b.x,0);ctx.lineTo(b.x,canvas.height);ctx.stroke();ctx.restore()}
function drawWorkspaceLimits(){const b=getBase(),max=state.link1+state.link2+(state.dof===3?state.link3:0);ctx.save();ctx.setLineDash([7,7]);ctx.strokeStyle='rgba(105,240,193,.12)';ctx.beginPath();ctx.arc(b.x,b.y,max,0,Math.PI*2);ctx.stroke();ctx.restore()}
function drawObstacles(){if(!state.obstaclesVisible)return;const hit=currentCollision();for(const o of state.obstacles){const p=worldToCanvas(o.x,o.y),isHit=hit&&hit.obstacle.id===o.id;ctx.save();ctx.fillStyle=isHit?'rgba(255,100,120,.24)':'rgba(70,91,85,.45)';ctx.strokeStyle=isHit?'#ff6478':'#60766f';ctx.lineWidth=isHit?3:1;ctx.fillRect(p.x-o.w/2,p.y-o.h/2,o.w,o.h);ctx.strokeRect(p.x-o.w/2,p.y-o.h/2,o.w,o.h);ctx.fillStyle=isHit?'#ff8998':'#829b92';ctx.font='12px Courier New';ctx.fillText(o.id,p.x-o.w/2+5,p.y-o.h/2+15);ctx.restore()}}
function drawDropZone(){const z=state.dropZone,p=worldToCanvas(z.x,z.y);ctx.save();ctx.strokeStyle='#7ab6ff';ctx.fillStyle='rgba(122,182,255,.06)';ctx.setLineDash([8,5]);ctx.fillRect(p.x-z.w/2,p.y-z.h/2,z.w,z.h);ctx.strokeRect(p.x-z.w/2,p.y-z.h/2,z.w,z.h);ctx.setLineDash([]);ctx.fillStyle='#7ab6ff';ctx.font='12px Courier New';ctx.fillText('DROP ZONE',p.x-z.w/2,p.y-z.h/2-8);ctx.restore()}
function drawParts(){for(const part of state.parts){const p=worldToCanvas(part.x,part.y);ctx.save();ctx.fillStyle=part.color;ctx.strokeStyle=state.selectedPart===part?'#fff':'rgba(255,255,255,.28)';ctx.lineWidth=state.selectedPart===part?3:1;ctx.fillRect(p.x-part.r,p.y-part.r,part.r*2,part.r*2);ctx.strokeRect(p.x-part.r,p.y-part.r,part.r*2,part.r*2);ctx.fillStyle='#dceae5';ctx.font='11px Courier New';ctx.fillText(part.id,p.x-part.r,p.y+part.r+15);ctx.restore()}}
function drawTarget(){if(!state.target)return;const p=worldToCanvas(state.target.x,state.target.y);ctx.save();ctx.strokeStyle='#e7d65e';ctx.lineWidth=2;ctx.beginPath();ctx.arc(p.x,p.y,11,0,Math.PI*2);ctx.stroke();ctx.beginPath();ctx.moveTo(p.x-17,p.y);ctx.lineTo(p.x+17,p.y);ctx.stroke();ctx.beginPath();ctx.moveTo(p.x,p.y-17);ctx.lineTo(p.x,p.y+17);ctx.stroke();ctx.restore()}
function drawTrail(){if(state.trail.length<2)return;ctx.save();ctx.strokeStyle='rgba(217,140,255,.72)';ctx.lineWidth=2;ctx.beginPath();state.trail.forEach((w,i)=>{const p=worldToCanvas(w.x,w.y);i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y)});ctx.stroke();ctx.restore()}
function drawArm(){const b=getBase(),k=forwardKinematics(),pts=[b,worldToCanvas(k.elbow.x,k.elbow.y),worldToCanvas(k.wrist.x,k.wrist.y)];if(state.dof===3)pts.push(worldToCanvas(k.end.x,k.end.y));const hit=currentCollision();ctx.save();ctx.lineCap='round';for(let i=0;i<pts.length-1;i++){ctx.lineWidth=18;ctx.strokeStyle=hit&&hit.link===i+1?'#ff6478':i===0?'#386e62':i===1?'#4cae98':'#5fc9af';ctx.beginPath();ctx.moveTo(pts[i].x,pts[i].y);ctx.lineTo(pts[i+1].x,pts[i+1].y);ctx.stroke()}for(let i=0;i<pts.length-1;i++){ctx.fillStyle='#07100f';ctx.strokeStyle='#69f0c1';ctx.lineWidth=3;ctx.beginPath();ctx.arc(pts[i].x,pts[i].y,13,0,Math.PI*2);ctx.fill();ctx.stroke()}const tip=worldToCanvas(k.end.x,k.end.y),a=degToRad(k.phi),px=Math.sin(a),py=Math.cos(a),spread=state.gripperClosed?8:17;ctx.strokeStyle='#dceae5';ctx.lineWidth=5;ctx.lineCap='square';for(const side of[-1,1]){const sx=tip.x+px*spread*side,sy=tip.y+py*spread*side,ex=sx+Math.cos(a)*27,ey=sy-Math.sin(a)*27;ctx.beginPath();ctx.moveTo(sx,sy);ctx.lineTo(ex,ey);ctx.stroke()}ctx.fillStyle='#69f0c1';ctx.font='12px Courier New';ctx.fillText('J1',b.x+16,b.y+4);ctx.fillText('J2',pts[1].x+16,pts[1].y+4);if(state.dof===3)ctx.fillText('J3',pts[2].x+16,pts[2].y+4);ctx.fillText('TCP',tip.x+12,tip.y-12);ctx.restore()}
function updateTelemetry(){const k=forwardKinematics(),hit=currentCollision();teleTheta1.textContent=`${state.theta1.toFixed(1)}°`;teleTheta2.textContent=`${state.theta2.toFixed(1)}°`;teleTheta3.textContent=state.dof===3?`${state.theta3.toFixed(1)}°`:'—';telePhi.textContent=`${normalizeDeg(k.phi).toFixed(1)}°`;teleX.textContent=k.end.x.toFixed(1);teleY.textContent=k.end.y.toFixed(1);teleBranch.textContent=state.elbow.toUpperCase();teleGripper.textContent=state.gripperClosed?'CLOSED':'OPEN';telePayload.textContent=state.heldPart?state.heldPart.id:'NONE';teleError.textContent=state.target?Math.hypot(k.end.x-state.target.x,k.end.y-state.target.y).toFixed(2):'—';teleCollision.textContent=hit?`${hit.obstacle.id} / L${hit.link}`:'CLEAR';teleCollision.classList.toggle('danger',!!hit);teleProgram.textContent=state.programRunning?'RUNNING':'IDLE';obstacleCount.textContent=state.obstaclesVisible?`${state.obstacles.length} ACTIVE`:'DISABLED'}
function draw(){ctx.clearRect(0,0,canvas.width,canvas.height);drawGrid();drawWorkspaceLimits();drawTrail();drawDropZone();drawObstacles();drawTarget();drawParts();drawArm();updateTelemetry()}

function getCanvasPointer(e){const r=canvas.getBoundingClientRect();return{x:(e.clientX-r.left)*canvas.width/r.width,y:(e.clientY-r.top)*canvas.height/r.height}}
canvas.addEventListener('pointerdown',e=>{if(state.animating||state.autoRunning||state.programRunning)return;const p=getCanvasPointer(e),w=canvasToWorld(p.x,p.y),part=nearestPart(w);if(part){state.selectedPart=part;setStatus(`${part.id} selected.`);if(state.mode==='ik')setTarget(part.x,part.y,part);draw();return}if(state.mode==='ik')setTarget(w.x,w.y)});

function manualJointChange(key,input){const old=state[key],next=Number(input.value);state[key]=next;if(currentCollision()){state[key]=old;setStatus(`MANUAL MOVE BLOCKED // ${key.toUpperCase()} would cause collision.`)}else{state.target=null;updateHeldPart();addTrailPoint()}updateControls();draw()}
theta1.addEventListener('input',()=>manualJointChange('theta1',theta1));theta2.addEventListener('input',()=>manualJointChange('theta2',theta2));theta3.addEventListener('input',()=>manualJointChange('theta3',theta3));
link1.addEventListener('input',()=>{const old=state.link1;state.link1=Number(link1.value);if(currentCollision())state.link1=old;state.target=null;updateControls();draw()});link2.addEventListener('input',()=>{const old=state.link2;state.link2=Number(link2.value);if(currentCollision())state.link2=old;state.target=null;updateControls();draw()});link3.addEventListener('input',()=>{const old=state.link3;state.link3=Number(link3.value);if(currentCollision())state.link3=old;state.target=null;updateControls();draw()});
toolAngle.addEventListener('input',()=>{state.toolAngle=Number(toolAngle.value);updateControls();if(state.mode==='ik'&&state.target)setTarget(state.target.x,state.target.y,state.selectedPart,state.toolAngle);else draw()});

fkMode.addEventListener('click',()=>setMode('fk'));ikMode.addEventListener('click',()=>setMode('ik'));dof2Button.addEventListener('click',()=>setDof(2));dof3Button.addEventListener('click',()=>setDof(3));elbowUpButton.addEventListener('click',()=>setElbow('up'));elbowDownButton.addEventListener('click',()=>setElbow('down'));
gripperButton.addEventListener('click',()=>toggleGripper());clearTrailButton.addEventListener('click',()=>{state.trail=[];addTrailPoint(true);setStatus('TCP trajectory cleared.');draw()});pickPlaceButton.addEventListener('click',autoPickPlace);
homeButton.addEventListener('click',async()=>{if(state.animating||state.autoRunning||state.programRunning)return;state.target=null;const t3=state.dof===3?-35:0;const ok=await animateTo(30,55,t3,440);setStatus(ok?'Home-ish position reached.':'HOME blocked by obstacle. Excellent factory layout.')});
randomButton.addEventListener('click',()=>{if(state.animating||state.autoRunning||state.programRunning)return;state.mode='ik';updateControls();for(let i=0;i<100;i++){const a=Math.random()*Math.PI*1.35-Math.PI*.15,r=90+Math.random()*(state.link1+state.link2+(state.dof===3?state.link3:0)-120),x=Math.cos(a)*r,y=Math.max(15,Math.sin(a)*r);if(chooseIK(x,y)){setTarget(x,y);setStatus('Random reachable and collision-free target generated.');break}}});

toggleObstaclesButton.addEventListener('click',()=>{state.obstaclesVisible=!state.obstaclesVisible;toggleObstaclesButton.textContent=state.obstaclesVisible?'HIDE OBSTACLES':'SHOW OBSTACLES';setStatus(state.obstaclesVisible?'Obstacle guard enabled.':'Obstacles disabled. Safety department has left the chat.');draw();updateControls()});
resetObstaclesButton.addEventListener('click',()=>{if(state.animating||state.autoRunning||state.programRunning)return;state.obstacles=DEFAULT_OBSTACLES.map(o=>({...o}));state.obstaclesVisible=true;toggleObstaclesButton.textContent='HIDE OBSTACLES';setStatus('Obstacle layout reset.');draw()});
randomObstaclesButton.addEventListener('click',()=>{if(state.animating||state.autoRunning||state.programRunning)return;const fresh=[];for(let i=0;i<3;i++){let candidate,tries=0;do{candidate={id:`OBS_${String.fromCharCode(65+i)}`,x:90+Math.random()*310,y:70+Math.random()*250,w:55+Math.random()*45,h:50+Math.random()*60};tries++}while((pointInRect({x:0,y:0},candidate,45)||pointInRect({x:state.dropZone.x,y:state.dropZone.y},candidate,65))&&tries<50);fresh.push(candidate)}state.obstacles=fresh;state.obstaclesVisible=true;toggleObstaclesButton.textContent='HIDE OBSTACLES';setStatus('Obstacle layout randomized. Chaos, but bounded.');draw()});

savePoseButton.addEventListener('click',savePose);deletePoseButton.addEventListener('click',()=>{if(state.programRunning)return;state.taughtPoses.pop();renderProgram();setStatus('Last taught pose deleted.');updateControls()});clearProgramButton.addEventListener('click',()=>{if(state.programRunning)return;state.taughtPoses=[];renderProgram();setStatus('Teach program cleared.');updateControls()});runProgramButton.addEventListener('click',runProgram);stopProgramButton.addEventListener('click',()=>{state.stopRequested=true;setStatus('STOP requested. Finishing current frame...')});

addTrailPoint(true);renderProgram();updateControls();setMode('fk');draw();
