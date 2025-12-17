/* vr_app.js — full file (safe) with OrbitControls, throttled teleop, HUD hooks, and pose -> robotViz syncing.
   Replace existing vr_app.js with this file (no other files changed).
*/

import * as THREE from './node_modules/three/build/three.module.js';
import { OrbitControls } from './node_modules/three/examples/jsm/controls/OrbitControls.js';

/* ---------- DOM & Config ---------- */
const mapCanvas = document.getElementById('mapCanvas');
const mapCtx = mapCanvas.getContext('2d');
mapCanvas.width = 320; mapCanvas.height = 320;

const statusSpan = document.getElementById('status');
const poseSpan = document.getElementById('poseText');
const cmdSpan  = document.getElementById('cmdText');
const logDiv   = document.getElementById('log');

const MAP_RESOLUTION = 0.05;   // meters per pixel
const MAP_ORIGIN_X = -1.22;
const MAP_ORIGIN_Y = -2.4;
const MAP_IMAGE = './assets/map.png';

/* ---------- Map image ---------- */
let mapImage = new Image();
let MAP_IMAGE_W = 1, MAP_IMAGE_H = 1;
let mapLoaded = false;
mapImage.onload = () => { MAP_IMAGE_W = mapImage.width; MAP_IMAGE_H = mapImage.height; mapLoaded = true; };
mapImage.onerror = () => { console.error('map image load failed'); };
mapImage.src = MAP_IMAGE;

/* ---------- WebSocket ---------- */
const WS_URL = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host;
let ws = null;
let latestPose = null;
let lastGoal = null;

function logLine(s) {
  const t = new Date().toLocaleTimeString();
  if (logDiv) logDiv.innerText = `[${t}] ${s}\n` + logDiv.innerText;
  else console.log(`[${t}] ${s}`);
}

function connectWS(){
  ws = new WebSocket(WS_URL);
  ws.onopen = () => { logLine('WS connected'); statusSpan && (statusSpan.innerText = 'WS connected'); };
  ws.onclose = () => { logLine('WS closed'); statusSpan && (statusSpan.innerText = 'WS closed'); setTimeout(connectWS,1000); };
  ws.onerror = (e) => { logLine('WS error'); console.error(e); };
  ws.onmessage = (ev) => {
    let parsed = null;
    try { parsed = JSON.parse(ev.data); } catch(e){ return; }
    if (!parsed) return;
    if (parsed.type === 'pose' || parsed.type === 'amcl' || parsed.type === 'amcl_pose') {
      const px = Number(parsed.x);
      const py = Number(parsed.y);
      const th = Number(parsed.theta ?? parsed.yaw ?? 0);
      latestPose = { x: px, y: py, theta: th, ts: performance.now() };
      poseSpan && (poseSpan.innerText = `x=${px.toFixed(2)} y=${py.toFixed(2)} θ=${th.toFixed(2)}`);
      updateRobotTargetsFromPose(latestPose);
      logLine(`WS pose recv x=${px.toFixed(2)} y=${py.toFixed(2)} th=${th.toFixed(2)}`);
      return;
    }
    if (parsed.type === 'ack') {
      cmdSpan && (cmdSpan.innerText = `v=${Number(parsed.linear||0).toFixed(2)} w=${Number(parsed.angular||0).toFixed(2)}`);
    }
    if (parsed.type === 'nav_goal_ack') {
      lastGoal = { x: parsed.x, y: parsed.y, theta: parsed.theta };
      logLine(`nav_goal_ack x=${parsed.x.toFixed(2)} y=${parsed.y.toFixed(2)}`);
    }
  };
}
connectWS();

/* ---------- Send helpers ---------- */
function sendCmd(vx, wz){
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({type:'cmd_vel', linear: vx, angular: wz}));
}
function sendNavGoal(x,y,theta){
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({type:'nav_goal', x, y, theta}));
  lastGoal = {x,y,theta};
  logLine(`nav_goal sent x=${x.toFixed(2)} y=${y.toFixed(2)}`);
}

/* ---------- Map drawing ---------- */
function worldToPixelX(x){
  const px = (x - MAP_ORIGIN_X) / MAP_RESOLUTION;
  return (px / MAP_IMAGE_W) * mapCanvas.width;
}
function worldToPixelY(y){
  const py = (y - MAP_ORIGIN_Y) / MAP_RESOLUTION;
  const imgY = MAP_IMAGE_H - py;
  return (imgY / MAP_IMAGE_H) * mapCanvas.height;
}
function pixelToWorld(cx, cy){
  const u = (cx / mapCanvas.width) * MAP_IMAGE_W;
  const v = MAP_IMAGE_H - (cy / mapCanvas.height) * MAP_IMAGE_H;
  const x = MAP_ORIGIN_X + u * MAP_RESOLUTION;
  const y = MAP_ORIGIN_Y + v * MAP_RESOLUTION;
  return {x,y};
}

mapCanvas.addEventListener('click', (ev)=>{
  const r = mapCanvas.getBoundingClientRect();
  const cx = ev.clientX - r.left, cy = ev.clientY - r.top;
  const u = (cx / mapCanvas.width) * MAP_IMAGE_W;
  const v = MAP_IMAGE_H - (cy / mapCanvas.height) * MAP_IMAGE_H;
  const wx = MAP_ORIGIN_X + u * MAP_RESOLUTION;
  const wy = MAP_ORIGIN_Y + v * MAP_RESOLUTION;
  const th = latestPose ? latestPose.theta : 0.0;
  sendNavGoal(wx, wy, th);
  lastGoal = {x:wx,y:wy};
  logLine(`map click -> nav_goal x=${wx.toFixed(2)} y=${wy.toFixed(2)}`);
});

function drawMap(){
  if (!mapLoaded) {
    mapCtx.fillStyle='#111'; mapCtx.fillRect(0,0,mapCanvas.width,mapCanvas.height); return;
  }
  mapCtx.clearRect(0,0,mapCanvas.width,mapCanvas.height);
  mapCtx.drawImage(mapImage, 0, 0, mapCanvas.width, mapCanvas.height);

  if (lastGoal) {
    const gx = worldToPixelX(lastGoal.x), gy = worldToPixelY(lastGoal.y);
    mapCtx.strokeStyle = '#ff4444'; mapCtx.lineWidth = 2;
    mapCtx.beginPath(); mapCtx.moveTo(gx-8,gy); mapCtx.lineTo(gx+8,gy);
    mapCtx.moveTo(gx,gy-8); mapCtx.lineTo(gx,gy+8); mapCtx.stroke();
  }

  if (latestPose) {
    const rx = worldToPixelX(latestPose.x), ry = worldToPixelY(latestPose.y);
    mapCtx.save();
    mapCtx.fillStyle = '#00ff00';
    mapCtx.beginPath(); mapCtx.arc(rx, ry, 7, 0, Math.PI*2); mapCtx.fill();
    mapCtx.fillStyle = '#000'; mapCtx.font='bold 12px monospace'; mapCtx.textAlign='center'; mapCtx.textBaseline='middle';
    mapCtx.fillText('R', rx, ry);
    mapCtx.restore();
    if (lastGoal) {
      const gx = worldToPixelX(lastGoal.x), gy = worldToPixelY(lastGoal.y);
      mapCtx.strokeStyle = '#ffaa00'; mapCtx.lineWidth = 2;
      mapCtx.beginPath(); mapCtx.moveTo(rx,ry); mapCtx.lineTo(gx,gy); mapCtx.stroke();
    }
  }
}
setInterval(drawMap, 200);

/* ---------- Three.js RobotViz ---------- */
let scene, camera, renderer, robotViz, controls;
let robotTargetPos = {x:0,y:0,z:0};
let robotTargetQuat = {x:0,y:0,z:0,w:1};

const LERP_POS = 0.18, LERP_ROT = 0.18, TELEPORT_DIST = 1.5, POSE_STALE_MS = 1600;

initThree();

function initThree(){
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x202020);

  camera = new THREE.PerspectiveCamera(70, window.innerWidth/window.innerHeight, 0.01, 100);
  camera.position.set(0, 2.2, 3.5);

  const hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 1.0);
  scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 0.6);
  dir.position.set(3, 10, 5);
  scene.add(dir);

  const floorGeom = new THREE.PlaneGeometry(50, 50);
  const floorMat = new THREE.MeshStandardMaterial({ color: 0x333333, roughness: 1 });
  const floor = new THREE.Mesh(floorGeom, floorMat);
  floor.rotation.x = -Math.PI/2;
  floor.position.y = 0;
  scene.add(floor);

  const grid = new THREE.GridHelper(50, 50, 0x00ff00, 0x444444);
  grid.position.y = 0.01;
  scene.add(grid);

  const geom = new THREE.BoxGeometry(0.4, 0.18, 0.6);
  const mat = new THREE.MeshStandardMaterial({ color: 0x0088ff });
  robotViz = new THREE.Mesh(geom, mat);
  robotViz.position.set(0, 0.09, 0);
  scene.add(robotViz);

  renderer = new THREE.WebGLRenderer({ antialias:false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.xr.enabled = true;
  document.body.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(robotViz.position.x, robotViz.position.y, robotViz.position.z);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.screenSpacePanning = false;
  controls.enablePan = true;
  controls.panSpeed = 1.0;
  controls.enableZoom = true;
  controls.zoomSpeed = 1.2;
  controls.minDistance = 0.5;
  controls.maxDistance = 40;
  controls.maxPolarAngle = Math.PI * 0.49 + 1.0;

  renderer.domElement.addEventListener('dblclick', () => { focusOnRobot(); });
  window.addEventListener('keydown', (e) => { if (e.key === 'c' || e.key === 'C') focusOnRobot(); });

  const enterBtn = document.getElementById('enterVr');
  if (enterBtn) {
    enterBtn.onclick = () => {
      if (navigator.xr && navigator.xr.requestSession) {
        navigator.xr.requestSession('immersive-vr').then(sess => renderer.xr.setSession(sess)).catch(()=>{ logLine('XR request failed'); });
      }
    };
  }

  window.addEventListener('resize', onResize);
  renderer.setAnimationLoop(renderLoop);
}

function onResize(){ camera.aspect = window.innerWidth/window.innerHeight; camera.updateProjectionMatrix(); renderer.setSize(window.innerWidth, window.innerHeight); }

function worldToThreeVec(x,y){
  return { x: x, y: 0.05, z: -y };
}

function updateRobotTargetsFromPose(p){
  if (!p) return;
  const v = worldToThreeVec(p.x, p.y);
  robotTargetPos.x = v.x; robotTargetPos.y = v.y; robotTargetPos.z = v.z;
  const qy = -p.theta;
  const half = qy * 0.5;
  robotTargetQuat = { x: 0, y: Math.sin(half), z: 0, w: Math.cos(half) };
}

function focusOnRobot(){
  if (!robotViz || !controls) return;
  const target = robotViz.position.clone ? robotViz.position.clone() : new THREE.Vector3(robotViz.position.x, robotViz.position.y, robotViz.position.z);
  controls.target.copy(target);
  const dir = new THREE.Vector3().subVectors(camera.position, target).normalize();
  const desiredDistance = 3.0;
  camera.position.copy(new THREE.Vector3().addVectors(target, dir.multiplyScalar(desiredDistance)));
  controls.update();
}

/* ---------- Render loop ---------- */
function renderLoop(){
  const now = performance.now();
  if (latestPose && (now - latestPose.ts) < POSE_STALE_MS) {
    statusSpan && (statusSpan.innerText = 'pose live');
    const dx = robotViz.position.x - robotTargetPos.x;
    const dz = robotViz.position.z - robotTargetPos.z;
    const dist = Math.sqrt(dx*dx + dz*dz);
    if (dist > TELEPORT_DIST) {
      robotViz.position.set(robotTargetPos.x, robotTargetPos.y, robotTargetPos.z);
      robotViz.quaternion.set(robotTargetQuat.x, robotTargetQuat.y, robotTargetQuat.z, robotTargetQuat.w);
    } else {
      robotViz.position.x += (robotTargetPos.x - robotViz.position.x) * LERP_POS;
      robotViz.position.y += (robotTargetPos.y - robotViz.position.y) * LERP_POS;
      robotViz.position.z += (robotTargetPos.z - robotViz.position.z) * LERP_POS;
      robotViz.quaternion.x += (robotTargetQuat.x - robotViz.quaternion.x) * LERP_ROT;
      robotViz.quaternion.y += (robotTargetQuat.y - robotViz.quaternion.y) * LERP_ROT;
      robotViz.quaternion.z += (robotTargetQuat.z - robotViz.quaternion.z) * LERP_ROT;
      robotViz.quaternion.w += (robotTargetQuat.w - robotViz.quaternion.w) * LERP_ROT;
      robotViz.quaternion.normalize();
    }
  } else {
    statusSpan && (statusSpan.innerText = 'pose stale');
  }

  if (controls) controls.update();
  renderer.render(scene, camera);
}

/* ---------- Teleop + HUD + send throttling (zero-suppression) ---------- */
const SEND_EPS = 0.02;      // minimum change to consider
const SEND_RATE_MS = 100;   // max send rate (100ms => 10Hz)
let lastSent = { linear: 0, angular: 0, t: 0 };

function sendCmdThrottled(vx, wz) {
  const now = Date.now();
  const linearDiff = Math.abs(vx - lastSent.linear);
  const angularDiff = Math.abs(wz - lastSent.angular);
  if (linearDiff > SEND_EPS || angularDiff > SEND_EPS || (now - lastSent.t) > 1000) {
    if ((now - lastSent.t) >= SEND_RATE_MS) {
      sendCmd(vx, wz);
      lastSent = { linear: vx, angular: wz, t: now };
    }
  }
}

const keys = { w:false, a:false, s:false, d:false };
window.addEventListener('keydown', (e)=>{ if (e.key in keys) keys[e.key]=true; });
window.addEventListener('keyup', (e)=>{ if (e.key in keys) keys[e.key]=false; });

setInterval(()=>{
  let vx = 0, wz = 0;
  if (keys.w) vx = 0.25;
  if (keys.s) vx = -0.18;
  if (keys.a) wz = 0.8;
  if (keys.d) wz = -0.8;
  sendCmdThrottled(vx, wz);
}, 40);

/* ---------- HUD buttons hookup ---------- */
function hookHudButtons(){
  const btnMap = {
    btnForward: {v:0.25, w:0},
    btnBack:    {v:-0.18, w:0},
    btnLeft:    {v:0, w:0.8},
    btnRight:   {v:0, w:-0.8},
    btnStop:    {v:0, w:0}
  };
  Object.keys(btnMap).forEach(id=>{
    const el = document.getElementById(id);
    if (!el) return;
    el.addEventListener('pointerdown', ()=> {
      const cfg = btnMap[id];
      sendCmd(cfg.v, cfg.w);
      lastSent = { linear: cfg.v, angular: cfg.w, t: Date.now() };
    });
    el.addEventListener('pointerup', ()=> {
      if (id !== 'btnStop') {
        sendCmd(0,0);
        lastSent = { linear: 0, angular: 0, t: Date.now() };
      }
    });
  });
}
hookHudButtons();

/* ---------- Debug pose watcher ---------- */
setInterval(()=>{
  if (latestPose) {
    const age = performance.now() - latestPose.ts;
    if (age > 2000) logLine(`Pose stale age=${(age/1000).toFixed(2)}s`);
  }
}, 3000);

/* expose debug helpers */
window.__vrDebug = { getWS:()=>ws, getLatestPose:()=>latestPose };

