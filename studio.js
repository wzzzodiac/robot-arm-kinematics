import * as THREE from 'three';
import { GLTFLoader } from './vendor/GLTFLoader.js';

// The solver owns XY. Presentation maps (x,y) to Three.js (x,height,-y).
const host = document.getElementById('sceneHost');
const get = id => document.getElementById(id);
const modelNodes = ['AxisRoot','J1','J2','J3','Body1','Body2','Body3','TCP','FingerL','FingerR'];
let studio;
function showPlan(message) {
  host.hidden = true; get('armCanvas').hidden = false;
  get('view2d').hidden = true; get('view3d').hidden = !studio || studio.failed;
  document.querySelector('.scene-caption').hidden = true;
  for (const id of ['orbitView','topView','zoomIn','zoomOut']) get(id).disabled = true;
  get('assetState').textContent = message || 'PLAN VIEW / SAME SOLVER';
  if (studio) studio.active = false;
  draw();
}

class AxisStudio {
  constructor() {
    this.active = true; this.pending = false; this.failed = false;
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color('#e4e8dd');
    this.renderer = new THREE.WebGLRenderer({antialias:true, alpha:false});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.75));
    this.renderer.shadowMap.enabled = true; this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping; this.renderer.toneMappingExposure = 1.35;
    host.append(this.renderer.domElement);
    this.camera = new THREE.OrthographicCamera(-400,400,250,-250,1,3000);
    this.focus = new THREE.Vector3(120,12,-125); this.yaw = .58; this.pitch = .88; this.zoom = 1;
    this.ray = new THREE.Raycaster(); this.plane = new THREE.Plane(new THREE.Vector3(0,1,0),0);
    this.scene.add(new THREE.HemisphereLight('#fbffec','#798176',2.8));
    const sun = new THREE.DirectionalLight('#fff2dd',4.2); sun.position.set(-200,650,180);
    sun.castShadow = true; sun.shadow.mapSize.set(2048,2048); sun.shadow.camera.left=-700;sun.shadow.camera.right=700;sun.shadow.camera.top=700;sun.shadow.camera.bottom=-700;sun.shadow.camera.far=1600;sun.shadow.normalBias=.8; sun.shadow.bias=-.0002;
    this.scene.add(sun); const fill=new THREE.DirectionalLight('#dfefff',1.3);fill.position.set(300,250,-450);this.scene.add(fill);
    this.materials = {}; this.makeTable(); this.makeDynamic();
    this.bind(); this.resize();
    this.observer = new ResizeObserver(()=>this.resize()); this.observer.observe(host);
    this.renderer.domElement.addEventListener('webglcontextlost',e=>{e.preventDefault();this.failed=true;showPlan('3D CONTEXT LOST / PLAN VIEW ACTIVE')});
  }
  mat(color,metalness=0,roughness=.65) { return new THREE.MeshStandardMaterial({color,metalness,roughness}); }
  box(size,pos,mat,parent=this.scene) {
    const mesh=new THREE.Mesh(new THREE.BoxGeometry(...size),mat);mesh.position.set(...pos);mesh.castShadow=true;mesh.receiveShadow=true;parent.add(mesh);return mesh;
  }
  label(text,x,y,z,size=12,color='#5c6d61',parent=this.scene) {
    const c=document.createElement('canvas');c.width=512;c.height=64;
    const ctx=c.getContext('2d');ctx.font='24px Consolas, monospace';ctx.fillStyle=color;ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(text,256,32);
    const texture=new THREE.CanvasTexture(c);texture.colorSpace=THREE.SRGBColorSpace;
    const m=new THREE.Mesh(new THREE.PlaneGeometry(size*8,size),new THREE.MeshBasicMaterial({map:texture,transparent:true,depthWrite:false}));m.rotation.x=-Math.PI/2;m.position.set(x,y,z);parent.add(m);return m;
  }
  makeTable() {
    const slab=this.mat('#aebaaa',.25,.75), rim=this.mat('#68786c',.4), edge=this.mat('#cfd6c6',.25);
    this.box([1040,24,960],[70,-20,-50],rim);
    this.box([1020,12,940],[70,-3,-50],slab);
    for(const z of [-524,424])this.box([1040,4,7],[70,-9,z],edge);
    for(const x of [-447,587])this.box([7,4,945],[x,-9,-50],edge);
    const points=[]; for(let i=-440;i<=580;i+=40){points.push(i,3,-500,i,3,400)}for(let z=-500;z<=420;z+=40){points.push(-440,3,z,580,3,z)}
    const lines=new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(points,3)),new THREE.LineBasicMaterial({color:'#879886',transparent:true,opacity:.32}));this.scene.add(lines);
    const holes=new THREE.InstancedMesh(new THREE.CylinderGeometry(2.1,2.1,.5,8),this.mat('#657662'),25*23);const obj=new THREE.Object3D();let n=0;
    for(let x=-420;x<=540;x+=40)for(let z=-480;z<=400;z+=40){obj.position.set(x,3.3,z);obj.updateMatrix();holes.setMatrixAt(n++,obj.matrix)}this.scene.add(holes);
    this.label('AXIS / PRECISION WORKCELL',130,4,365,17);
    this.label('01   —   PLANAR XY',-290,4,-460,12);
    for(let x=100;x<=400;x+=100)this.label(String(x),x,4,34,8);
    this.box([430,1,1],[205,4,0],this.mat('#d47943'));
    this.box([1,1,430],[0,4,-205],this.mat('#d47943'));
    const dash=new THREE.LineDashedMaterial({color:'#7c8e7b',dashSize:7,gapSize:6,transparent:true,opacity:.65});
    const pts=Array.from({length:129},(_,i)=>new THREE.Vector3(Math.cos(i/128*Math.PI*2),0,Math.sin(i/128*Math.PI*2)));
    this.reach=new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),dash);this.reach.position.y=4;this.scene.add(this.reach);
  }
  makeDynamic() {
    this.obstacleGroup=new THREE.Group();this.scene.add(this.obstacleGroup);this.obstacles=[];
    for(let i=0;i<3;i++){
      const group=new THREE.Group();this.obstacleGroup.add(group);
      const m=this.mat('#778477',.35,.5);this.box([1,1,1],[0,0,0],m,group);
      const cap=this.box([1,1,1],[0,0,0],this.mat('#c4ccb9',.4),group);
      const stripe=this.box([1,1,1],[0,0,0],this.mat('#cf7934'),group);
      this.obstacles.push({group,body:group.children[0],cap,stripe,mat:m});
    }
    const zone=new THREE.Mesh(new THREE.PlaneGeometry(1,1),new THREE.MeshBasicMaterial({color:'#659d9b',transparent:true,opacity:.26,depthWrite:false}));zone.rotation.x=-Math.PI/2;this.scene.add(zone);this.zone=zone;
    this.dropLabel=this.label('DROP / 01',285,4,-105,11,'#345e60');
    this.parts=state.parts.map(p=>{const g=new THREE.Group();this.scene.add(g);const body=this.box([p.r*2,40,p.r*2],[0,22,0],this.mat(p.color,.35,.4),g);this.box([p.r*1.6,2,p.r*1.6],[0,43,0],this.mat('#e7e7d8',.45),g);const ring=new THREE.Mesh(new THREE.RingGeometry(p.r+6,p.r+8,40),new THREE.MeshBasicMaterial({color:'#ce502b',side:THREE.DoubleSide}));ring.rotation.x=-Math.PI/2;ring.position.y=4;g.add(ring);return{g,body,ring}});
    const targetGeo=new THREE.RingGeometry(11,13,48);this.target=new THREE.Mesh(targetGeo,new THREE.MeshBasicMaterial({color:'#c84421',side:THREE.DoubleSide}));this.target.rotation.x=-Math.PI/2;this.scene.add(this.target);
    const trailGeo=new THREE.BufferGeometry();trailGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(300*3),3));trailGeo.setDrawRange(0,0);this.trail=new THREE.Line(trailGeo,new THREE.LineBasicMaterial({color:'#2d7371'}));this.trail.frustumCulled=false;this.scene.add(this.trail);
  }
  async load() {
    const gltf=await new GLTFLoader().loadAsync('./assets/axis-arm.glb');
    this.model=gltf.scene;this.nodes={};
    for(const name of modelNodes){const node=this.model.getObjectByName(name);if(!node)throw new Error(`Missing rig node: ${name}`);this.nodes[name]=node}
    this.model.traverse(o=>{if(o.isMesh){o.castShadow=true;o.receiveShadow=true}});this.scene.add(this.model);
    get('assetState').textContent='AX—03 / READY';this.sync();
  }
  sync() {
    const s=state, n=this.nodes;
    if(n){
      n.J1.rotation.y=degToRad(s.theta1);n.J2.rotation.y=degToRad(s.theta2);n.J3.rotation.y=degToRad(s.theta3);
      n.J2.position.x=s.link1;n.J3.position.x=s.link2;
      n.Body1.scale.x=s.link1/180;n.Body2.scale.x=s.link2/150;n.Body3.scale.x=s.link3/75;
      n.J3.visible=s.dof===3;
      const parent=s.dof===3?n.J3:n.J2;if(n.TCP.parent!==parent)parent.add(n.TCP);
      n.TCP.position.set(s.dof===3?s.link3:s.link2,s.dof===3?0:16,0);
      n.FingerL.position.z=-(s.gripperClosed?8:17);n.FingerR.position.z=s.gripperClosed?8:17;
      this.model.updateMatrixWorld(true);
    }
    const hit=currentCollision();this.obstacleGroup.visible=s.obstaclesVisible;
    s.obstacles.forEach((o,i)=>{const v=this.obstacles[i];v.group.position.set(o.x,0,-o.y);v.body.scale.set(o.w,94,o.h);v.body.position.y=49;v.cap.scale.set(o.w+2,4,o.h+2);v.cap.position.y=98;v.stripe.scale.set(o.w+1,7,o.h+1);v.stripe.position.y=78;v.mat.color.set(hit?.obstacle.id===o.id?'#bf5038':'#778477')});
    s.parts.forEach((p,i)=>{const v=this.parts[i];v.g.position.set(p.x,0,-p.y);v.g.rotation.y=degToRad(p.orientation||0);v.ring.visible=p===s.selectedPart;});
    const z=s.dropZone;this.zone.scale.set(z.w,z.h,1);this.zone.position.set(z.x,3.7,-z.y);
    this.target.visible=!!s.target;if(s.target)this.target.position.set(s.target.x,5,-s.target.y);
    const a=this.trail.geometry.attributes.position; s.trail.forEach((p,i)=>a.setXYZ(i,p.x,5,-p.y));a.needsUpdate=true;this.trail.geometry.setDrawRange(0,s.trail.length);
    this.reach.scale.setScalar(s.link1+s.link2+(s.dof===3?s.link3:0));this.reach.computeLineDistances();
    get('sceneHint').textContent=s.mode==='ik'?'Tap the table to target · Drag to orbit':'Drag to orbit · Scroll to zoom';
    this.request();
  }
  resize(){if(!this.active)return;const w=host.clientWidth,h=host.clientHeight;if(!w||!h)return;this.renderer.setSize(w,h,false);const aspect=w/h,half=Math.max(255,285/aspect)/this.zoom;this.camera.left=-half*aspect;this.camera.right=half*aspect;this.camera.top=half;this.camera.bottom=-half;this.camera.updateProjectionMatrix();this.positionCamera();this.request()}
  positionCamera(){this.camera.position.set(this.focus.x+Math.sin(this.yaw)*Math.cos(this.pitch)*1000,this.focus.y+Math.sin(this.pitch)*1000,this.focus.z+Math.cos(this.yaw)*Math.cos(this.pitch)*1000);this.camera.lookAt(this.focus);this.camera.updateMatrixWorld()}
  request(){if(this.pending||!this.active||document.hidden||this.failed)return;this.pending=true;this.frame=requestAnimationFrame(()=>{this.pending=false;if(this.active&&!document.hidden&&!this.failed)this.renderer.render(this.scene,this.camera)})}
  preset(top){this.pitch=top?Math.PI/2-.001:.88;this.yaw=top?0:.58;this.zoom=1;get('topView').classList.toggle('active',top);get('orbitView').classList.toggle('active',!top);this.resize()}
  bind(){
    const el=this.renderer.domElement;const pointers=new Map();let origin=null,moved=false,pinch=0;
    el.addEventListener('pointerdown',e=>{el.setPointerCapture(e.pointerId);pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});if(pointers.size===1){origin={x:e.clientX,y:e.clientY};moved=false}else{moved=true;const a=[...pointers.values()];pinch=Math.hypot(a[0].x-a[1].x,a[0].y-a[1].y)}});
    el.addEventListener('pointermove',e=>{const prev=pointers.get(e.pointerId);if(!prev)return;const dx=e.clientX-prev.x,dy=e.clientY-prev.y;pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});if(pointers.size===2){const a=[...pointers.values()],d=Math.hypot(a[0].x-a[1].x,a[0].y-a[1].y);if(pinch>0)this.zoom=THREE.MathUtils.clamp(this.zoom*d/pinch,.6,2.8);pinch=d;this.resize();return}if(origin&&Math.hypot(e.clientX-origin.x,e.clientY-origin.y)>5)moved=true;if(moved){this.yaw-=dx*.006;this.pitch=THREE.MathUtils.clamp(this.pitch+dy*.005,.2,1.56);this.positionCamera();this.request()}});
    el.addEventListener('pointerup',e=>{pointers.delete(e.pointerId);if(!moved&&origin&&!state.animating&&!state.autoRunning&&!state.programRunning){const rect=el.getBoundingClientRect();this.ray.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,-(e.clientY-rect.top)/rect.height*2+1),this.camera);const p=new THREE.Vector3();if(this.ray.ray.intersectPlane(this.plane,p)){const w={x:p.x,y:-p.z};const hit=this.ray.intersectObjects(this.parts.map(v=>v.body),false)[0];const part=hit?state.parts[this.parts.findIndex(v=>v.body===hit.object)]:nearestPart(w);if(part){state.selectedPart=part;setStatus(`${part.id} selected.`)}if(state.mode==='ik')setTarget(part?.x??w.x,part?.y??w.y,part);draw()}}if(!pointers.size)origin=null});
    el.addEventListener('pointercancel',e=>{pointers.delete(e.pointerId);moved=true;origin=null});
    el.addEventListener('wheel',e=>{e.preventDefault();this.zoom=THREE.MathUtils.clamp(this.zoom*Math.exp(-e.deltaY*.001),.6,2.8);this.resize()},{passive:false});
    document.addEventListener('visibilitychange',()=>{if(!document.hidden)this.request()});
    get('orbitView').onclick=()=>this.preset(false);get('topView').onclick=()=>this.preset(true);
    get('zoomIn').onclick=()=>{this.zoom=Math.min(2.8,this.zoom*1.2);this.resize()};get('zoomOut').onclick=()=>{this.zoom=Math.max(.6,this.zoom/1.2);this.resize()};
  }
  dispose(){cancelAnimationFrame(this.frame);this.observer.disconnect();const geometry=new Set(),materials=new Set(),textures=new Set();this.scene.traverse(o=>{if(o.geometry)geometry.add(o.geometry);for(const m of(Array.isArray(o.material)?o.material:o.material?[o.material]:[])){materials.add(m);if(m.map)textures.add(m.map)}});geometry.forEach(g=>g.dispose());materials.forEach(m=>m.dispose());textures.forEach(t=>t.dispose());this.renderer.dispose()}
}
get('view2d').onclick=()=>showPlan();
get('view3d').onclick=()=>{if(!studio||studio.failed)return;host.hidden=false;get('armCanvas').hidden=true;get('view3d').hidden=true;get('view2d').hidden=false;document.querySelector('.scene-caption').hidden=false;for(const id of ['orbitView','topView','zoomIn','zoomOut'])get(id).disabled=false;get('assetState').textContent='AX—03 / READY';studio.active=true;studio.resize();studio.sync()};
try {studio=new AxisStudio();window.axisStudio=studio;await studio.load()} catch(error){console.warn('AXIS 3D unavailable:',error.message);if(studio){studio.failed=true;studio.dispose()}showPlan('3D UNAVAILABLE / PLAN VIEW ACTIVE')}
window.addEventListener('pagehide',e=>{if(!e.persisted)studio?.dispose()});
