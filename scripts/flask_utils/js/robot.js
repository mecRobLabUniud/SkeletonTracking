import * as THREE from 'three'
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js'
import { scene, camera, renderer, resize , updateCamera} from './scene.js'

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────
const DH = [
  { a:0,       alpha:0,          d:0.333, name:'J1', mesh:'link1' },
  { a:0,       alpha:-Math.PI/2, d:0,     name:'J2', mesh:'link2' },
  { a:0,       alpha: Math.PI/2, d:0.316, name:'J3', mesh:'link3' },
  { a:0.0825,  alpha: Math.PI/2, d:0,     name:'J4', mesh:'link4' },
  { a:-0.0825, alpha:-Math.PI/2, d:0.384, name:'J5', mesh:'link5' },
  { a:0,       alpha: Math.PI/2, d:0,     name:'J6', mesh:'link6' },
  { a:0.088,   alpha: Math.PI/2, d:0,     name:'J7', mesh:'link7' },
  { a:0,       alpha:0,          d:0.107, name:'Flange', fixed:true, mesh:null }
];

const LIMITS = [
  [-2.8973, 2.8973],
  [-1.7628, 1.7628],
  [-2.8973, 2.8973],
  [-3.0718, -0.0698],
  [-2.8973, 2.8973],
  [-0.0175, 3.7525],
  [-2.8973, 2.8973]
];

const READY_POSE = [0, -Math.PI/4, 0, -3*Math.PI/4, 0, Math.PI/2, Math.PI/4];
let q = READY_POSE.slice();
let robot_base = [0.0, 0.0, 0.0];
let gripperOpening = 0.04;
let HAND = false;
const opacity = true;
const socket = io();


// ─────────────────────────────────────────────────────────────────────────────
// Robot group
// ─────────────────────────────────────────────────────────────────────────────
const robotGroup = new THREE.Group();
scene.add(robotGroup);

/* ---------- load real Franka visual meshes (.dae) ---------- */
var MESH_FILES = [];
if (HAND) {
  MESH_FILES = ['link0','link1','link2','link3','link4','link5','link6','link7','hand','finger'];
}
else {
  MESH_FILES = ['link0','link1','link2','link3','link4','link5','link6','link7'];
}

const loader = new ColladaLoader();
const loaded = {};

function loadOne(name){
  return new Promise((resolve, reject)=>{
    loader.load(
      `meshes/visual/${name}.dae`,
      (collada) => {
        const wrapper = new THREE.Group();
        collada.scene.quaternion.identity();
        collada.scene.position.set(0,0,0);
        collada.scene.scale.set(1,1,1);
        collada.scene.traverse(o=>{
          if (o.isMesh){
            o.castShadow = false;
            o.receiveShadow = false;
            if (opacity) {
                o.material.transparent = true;
                o.material.opacity = 0.5;
            }
          }
        });
        wrapper.add(collada.scene);
        loaded[name] = wrapper;
        robotGroup.add(wrapper);
        resolve();
      },
      undefined,
      (err) => reject(err)
    );
  });
}

const loadingTxt = document.getElementById('loadingTxt');
let doneCount = 0;
Promise.all(MESH_FILES.map(n =>
  loadOne(n).then(()=>{
    doneCount++;
    loadingTxt.textContent = `Loading Franka mesh library… (${doneCount}/${MESH_FILES.length})`;
  })
)).then(()=>{
  // build the two gripper fingers from the single finger.dae (as the real robot does)
  if (HAND) {
    const fingerTemplate = loaded['finger'];
    robotGroup.remove(fingerTemplate);
    const fingerL = fingerTemplate;
    const fingerR = fingerTemplate.clone(true);
    robotGroup.add(fingerL, fingerR);
    loaded['fingerL'] = fingerL;
    loaded['fingerR'] = fingerR;
  }

  document.getElementById('loading').style.opacity = '0';
  setTimeout(()=> document.getElementById('loading').style.display = 'none', 300);

  animate();
}).catch(err=>{
  loadingTxt.innerHTML = 'Failed to load meshes.<br>Check the browser console and that meshes/visual/*.dae exist.';
  document.querySelector('#loading .spin').style.display = 'none';
  console.error(err);
});


const frameHelpers = [];
for (let i=0;i<DH.length;i++){
  const f = new THREE.AxesHelper(0.09);
  f.visible = false;
  robotGroup.add(f);
  frameHelpers.push(f);
}
const flangeHelper = new THREE.AxesHelper(0.09);
flangeHelper.visible = false;
robotGroup.add(flangeHelper);

/* ---------- forward kinematics ---------- */
function dhMatrix(a, alpha, d, theta){
  const ca=Math.cos(alpha), sa=Math.sin(alpha);
  const ct=Math.cos(theta), st=Math.sin(theta);
  // modified DH (Craig convention): Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
  const m = new THREE.Matrix4();
  m.set(
    ct,      -st,      0,     a,
    st*ca,   ct*ca,   -sa,   -sa*d,
    st*sa,   ct*sa,    ca,    ca*d,
    0,        0,        0,     1
  );
  return m;
}

function updateKinematics(){
  const baseRot = new THREE.Matrix4();
  let T = baseRot.clone();
  // T.setPosition(0, 0, 0);
  T.setPosition(robot_base[0], robot_base[1], robot_base[2]);

  if (loaded['link0']){
    loaded['link0'].position.setFromMatrixPosition(T);
    loaded['link0'].quaternion.setFromRotationMatrix(T);
  }

  const mats = [T.clone()];
  for (let i=0;i<DH.length;i++){
    const theta = DH[i].fixed ? 0 : q[i];
    T = T.clone().multiply(dhMatrix(DH[i].a, DH[i].alpha, DH[i].d, theta));
    mats.push(T.clone());
    if (DH[i].mesh && loaded[DH[i].mesh]){
      const obj = loaded[DH[i].mesh];
      obj.position.setFromMatrixPosition(T);
      obj.quaternion.setFromRotationMatrix(T);
    }
    const fh = frameHelpers[i];
    fh.position.setFromMatrixPosition(T);
    fh.quaternion.setFromRotationMatrix(T);
  }

  // hand attaches to the flange with a fixed -45deg twist (franka_hand.xacro)
  const flangeMat = mats[mats.length-1];
  const handMat4 = flangeMat.clone().multiply(new THREE.Matrix4().makeRotationZ(-Math.PI/4));
  if (loaded['hand']){
    loaded['hand'].position.setFromMatrixPosition(handMat4);
    loaded['hand'].quaternion.setFromRotationMatrix(handMat4);
  }
  flangeHelper.position.setFromMatrixPosition(handMat4);
  flangeHelper.quaternion.setFromRotationMatrix(handMat4);

  const fingerBase = new THREE.Matrix4().makeTranslation(0, 0, 0.0584);
  const leftMat = handMat4.clone().multiply(fingerBase).multiply(new THREE.Matrix4().makeTranslation(0, gripperOpening, 0));
  const rightMat = handMat4.clone().multiply(fingerBase)
    .multiply(new THREE.Matrix4().makeTranslation(0, -gripperOpening, 0))
    .multiply(new THREE.Matrix4().makeRotationZ(Math.PI));
  if (loaded['fingerL']){
    loaded['fingerL'].position.setFromMatrixPosition(leftMat);
    loaded['fingerL'].quaternion.setFromRotationMatrix(leftMat);
  }
  if (loaded['fingerR']){
    loaded['fingerR'].position.setFromMatrixPosition(rightMat);
    loaded['fingerR'].quaternion.setFromRotationMatrix(rightMat);
  }

  // TCP readout — hand frame, 0.1034m along its z (franka_hand default tcp offset)
  const tcpMat = handMat4.clone().multiply(new THREE.Matrix4().makeTranslation(0,0,0.1034));
  const p = new THREE.Vector3().setFromMatrixPosition(tcpMat);
}

/* ---------- render loop ---------- */
function animate(){
  requestAnimationFrame(animate);
  // if (spin){ camTheta += 0.003; updateCamera(); }
  renderer.render(scene, camera);
}



resize();
updateCamera();


// ─────────────────────────────────────────────────────────────────────────────
// Update plot
// ─────────────────────────────────────────────────────────────────────────────
let do_once = true;
function update_plot() {
  socket.on('update_plot', function (point) {
        if (point.q_robot != undefined) {
            q = point.q_robot;
            if (do_once) {
                robot_base = [point.x_robot[0], point.y_robot[0], point.z_robot[0]];
                do_once = false;
            }
        }
        
        updateKinematics();
    }); 
}

setTimeout(update_plot, 1 / 30 * 1000);


