import * as THREE from '/node_modules/three/build/three.module.js';
import { OrbitControls } from '/node_modules/three/examples/jsm/controls/OrbitControls.js';
import { XRControllerModelFactory } from '/node_modules/three/examples/jsm/webxr/XRControllerModelFactory.js';

const WS_URL = 'ws://localhost:8080';
const ws = new WebSocket(WS_URL);

const statusSpan = document.getElementById('status');
const modeBtn = document.getElementById('modebtn');

let controlMode = "keyboard";
modeBtn.onclick = () => {
    controlMode = (controlMode === "keyboard") ? "vr" : "keyboard";
    modeBtn.innerText = `Mode: ${controlMode}`;
};

ws.onopen = () => statusSpan.innerText = 'WS connected';
ws.onmessage = (ev) => {
  try {
    const d = JSON.parse(ev.data);
    if (d.type === 'ack') {
      statusSpan.innerText = `ack v:${d.linear} w:${d.angular}`;
    }
  } catch {}
};

function sendCmd(vx, wz) {
  const msg = { type:'cmd_vel', linear:vx, angular:wz, ts:Date.now() };
  if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

let camera, scene, renderer;
init();
animate();

function init() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x111133);

  camera = new THREE.PerspectiveCamera(70, window.innerWidth/window.innerHeight, 0.01, 100);
  camera.position.set(0,1.6,2);

  const light = new THREE.HemisphereLight(0xffffff,0x444444,1);
  scene.add(light);

  const grid = new THREE.GridHelper(10,10);
  scene.add(grid);

  const robot = new THREE.Mesh(
    new THREE.BoxGeometry(0.4,0.2,0.6),
    new THREE.MeshStandardMaterial({color:0x00ffaa})
  );
  robot.position.y = 0.1;
  scene.add(robot);

  renderer = new THREE.WebGLRenderer({antialias:true});
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.xr.enabled = true;
  document.body.appendChild(renderer.domElement);

  document.body.appendChild(VRButton());

  const modelFactory = new XRControllerModelFactory();
  const ctrl1 = renderer.xr.getController(0);
  const grip1 = renderer.xr.getControllerGrip(0);
  grip1.add(modelFactory.createControllerModel(grip1));
  scene.add(ctrl1);
  scene.add(grip1);

  const ctrl2 = renderer.xr.getController(1);
  const grip2 = renderer.xr.getControllerGrip(1);
  grip2.add(modelFactory.createControllerModel(grip2));
  scene.add(ctrl2);
  scene.add(grip2);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0,0.2,0);

  window.addEventListener('resize', onWindowResize);
}

function VRButton() {
  const b = document.createElement('button');
  b.innerText = 'ENTER VR';
  b.style.cssText = `
    position:fixed;right:10px;bottom:10px;padding:12px 18px;
    background:#333;color:#fff;border:0;border-radius:6px;z-index:1000;
  `;
  b.onclick = async () => {
    if (navigator.xr) {
      try {
        let session = await navigator.xr.requestSession('immersive-vr', {optionalFeatures:['local-floor']});
        renderer.xr.setSession(session);
        controlMode = "vr";
        modeBtn.innerText = "Mode: vr";
      } catch(e) { alert("VR error " + e); }
    } else {
      alert("WebXR not supported! Use Chrome.");
    }
  };
  return b;
}

function onWindowResize() {
  camera.aspect = window.innerWidth/window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth,window.innerHeight);
}

window.addEventListener('keydown', (e)=>{
  if (controlMode !== "keyboard") return;
  if (e.key==='w') sendCmd(0.3,0);
  if (e.key==='s') sendCmd(-0.2,0);
  if (e.key==='a') sendCmd(0,0.7);
  if (e.key==='d') sendCmd(0,-0.7);
});

let lastSend = 0;
function handleVRControllers() {
  if (controlMode !== "vr") return;
  const session = renderer.xr.getSession();
  if (!session) return;

  const sources = session.inputSources;
  for (const src of sources) {
    if (!src.gamepad) continue;
    const gp = src.gamepad;

    const axX = gp.axes[2] || gp.axes[0] || 0;
    const axY = gp.axes[3] || gp.axes[1] || 0;

    const vx = (-axY) * 0.5;
    const wz = (axX) * 1.0;

    const now = performance.now();
    if (now - lastSend > 50) {
      sendCmd(vx, wz);
      lastSend = now;
    }
  }
}

function animate() {
  renderer.setAnimationLoop(()=>{
    renderer.render(scene,camera);
    handleVRControllers();
  });
}
