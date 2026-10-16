import * as THREE from 'three'
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js'

/* warn early if opened directly as a file:// page — ColladaLoader needs XHR,
   which browsers block on the file:// protocol. */
if (location.protocol === 'file:') {
  document.getElementById('loadingTxt').innerHTML =
    'Open this over a local server, not file://<br>e.g. <code>python3 -m http.server</code> in this folder';
  document.querySelector('#loading .spin').style.display = 'none';
}


// ─────────────────────────────────────────────────────────────────────────────
// Setting up scene
// ─────────────────────────────────────────────────────────────────────────────
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const app = document.getElementById('plot');
export const scene = new THREE.Scene();
scene.background = new THREE.Color('#3a3a3a');
scene.fog = new THREE.Fog('#3a3a3a', 3, 9);

export const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
export const renderer = new THREE.WebGLRenderer({ antialias:true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
app.appendChild(renderer.domElement);

export function resize(){
  const w = app.clientWidth, h = app.clientHeight;
  renderer.setSize(w,h);
  camera.aspect = w/h;
  camera.up.set(0, 0, 1);
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

/* lighting — a bit brighter than a PBR scene since Collada materials build as Phong */
scene.add(new THREE.HemisphereLight(0xaebdb4, 0x0a0a0a, 0.7));
const key = new THREE.DirectionalLight(0xffffff, 1.1);
key.position.set(1.6, 2.4, 1.2);
scene.add(key);
const fill = new THREE.DirectionalLight(0xbfd4ff, 0.4);
fill.position.set(-1.4, 1.0, 1.6);
scene.add(fill);
const rim = new THREE.DirectionalLight(0xff8a3d, 0.25);
rim.position.set(-1.5, 1.0, -1.8);
scene.add(rim);

/* floor grid */
// const grid = new THREE.GridHelper(3, 30, 0x2c3a32, 0x1a221d);
// scene.add(grid);
// grid.rotation.x = Math.PI / 2;
const worldAxes = new THREE.AxesHelper(0.25);
worldAxes.position.set(0,0,0.002);
scene.add(worldAxes);

// ─────────────────────────────────────────────────────────────────────────────
// 3D graphics controls
// ─────────────────────────────────────────────────────────────────────────────
let camDist = 1.7, camTheta = 0.6, camPhi = 1.15, camTarget = new THREE.Vector3(0,0.35,0);
let dragging = false, panning = false, lastX=0, lastY=0;

export function updateCamera(){
  const x = camDist * Math.sin(camPhi) * Math.cos(camTheta);
  const y = camDist * Math.sin(camPhi) * Math.sin(camTheta);
  const z = camDist * Math.cos(camPhi);
  camera.position.set(camTarget.x + x, camTarget.y + y, camTarget.z + z);
  camera.lookAt(camTarget);
}

renderer.domElement.addEventListener('mousedown', e=>{
  if (e.button === 2) panning = true; else dragging = true;
  lastX = e.clientX; lastY = e.clientY;
});
window.addEventListener('mouseup', ()=>{ dragging=false; panning=false; });
window.addEventListener('mousemove', e=>{
  const dx = e.clientX-lastX, dy = e.clientY-lastY;
  lastX = e.clientX; lastY = e.clientY;
  if (dragging){
    camTheta -= dx*0.006;
    camPhi = Math.min(Math.max(camPhi - dy*0.006, 0.15), Math.PI-0.1);
    updateCamera();
  } else if (panning){
    const right = new THREE.Vector3().setFromMatrixColumn(camera.matrix,0);
    const up = new THREE.Vector3().setFromMatrixColumn(camera.matrix,1);
    camTarget.addScaledVector(right, -dx*0.0015*camDist);
    camTarget.addScaledVector(up, dy*0.0015*camDist);
    updateCamera();
  }
});
renderer.domElement.addEventListener('contextmenu', e=>e.preventDefault());
renderer.domElement.addEventListener('wheel', e=>{
  e.preventDefault();
  camDist = Math.min(Math.max(camDist * (1 + e.deltaY*0.001), 0.5), 5);
  updateCamera();
}, { passive:false });