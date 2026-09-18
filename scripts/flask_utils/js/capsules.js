import * as THREE from 'three'
import { scene } from './scene.js'

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────
const MP_SKELETON = [[0, 9], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7], [6, 8], [9, 10], [11, 13], [12, 14], [13, 15], [14, 16], [17, 19], [18, 20]];
const r_sw_h = [0.16, 0.06, 0.06, 0.06, 0.06, 0.1, 0.1, 0.15, 0.1, 0.1, 0.08, 0.08, 0.05, 0.05];
const ROBOT_CONFIG = [[0, 1], [2, 3], [4, 5], [6, 7]];
const rv = [0.085, 0.085, 0.06, 0.065];
let rula_score = 0;
let rula = true;
const socket = io();

let human_caps = []
for (const _ of MP_SKELETON) {
    human_caps.push(createCapsule());
}
let robot_caps = []
for (const _ of MP_SKELETON) {
    robot_caps.push(createCapsule());
}


// ─────────────────────────────────────────────────────────────────────────────
// Plot capsules mesh
// ─────────────────────────────────────────────────────────────────────────────
function createCapsule() {
    const geometry = new THREE.CapsuleGeometry(0.01, 0.01, 4, 8);
    const material = new THREE.MeshBasicMaterial( { color: '#aaaaaa', transparent: true, opacity: 0.5 } );

    const mesh = new THREE.Mesh(geometry, material);
    mesh.visible = false;
    scene.add(mesh);

    return mesh;
}


function updateCapsule(capsule, p1, p2, radius = 0.05, rula = false) {
    const start = new THREE.Vector3(p1.x, p1.y, p1.z);
    const end   = new THREE.Vector3(p2.x, p2.y, p2.z);

    const length = start.distanceTo(end);
    const mid = start.clone().add(end).multiplyScalar(0.5);

    

    let caps_color = '#aaaaaa';
    if (rula) {
        switch (rula_score) {
            case 1:
                caps_color = '#9CCB3B';
                break;
            case 2:
                caps_color = '#9CCB3B';
                break;
            case 3:
                caps_color = '#FAF04C';
                break;
            case 4:
                caps_color = '#FAF04C';
                break;
            case 5:
                caps_color = '#f38a4d';
                break;
            case 6:
                caps_color = '#f38a4d';
                break;
            case 7:
                caps_color = '#F04C4E';
                break;
            default:
                caps_color = '#aaaaaa';
        }
    }

    capsule.geometry.dispose();
    capsule.geometry = new THREE.CapsuleGeometry(radius, length, 10, 20);
    capsule.position.copy(mid);
    capsule.material.color.set(caps_color);

    // Default capsule axis is Y; rotate to align with p1->p2
    const dir = end.clone().sub(start).normalize();
    const quaternion = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    capsule.quaternion.copy(quaternion);

    return capsule;
}


// ─────────────────────────────────────────────────────────────────────────────
// 3D plot update
// ─────────────────────────────────────────────────────────────────────────────
function update_plot() {
    socket.on('update_plot', function (point) {
        const r_sw_r = point.radius;


        if (point.x_robot && point.y_robot && point.z_robot && point.x_robot) {
            for (let i = 0; i < ROBOT_CONFIG.length; i++) {
                const a = ROBOT_CONFIG[i][0];
                const b = ROBOT_CONFIG[i][1];
                if (point.x_robot[a] != null && point.x_robot[b] != null) {
                    robot_caps[i] = updateCapsule(
                        robot_caps[i],
                        { x: point.x_robot[a], y: point.y_robot[a], z: point.z_robot[a] },
                        { x: point.x_robot[b], y: point.y_robot[b], z: point.z_robot[b] },
                        r_sw_r[i],
                        false
                    );
                    robot_caps[i].visible = true;
                }
                else {
                    robot_caps[i].visible = false;
                }
            }
        }

        // if (!plotInitialized) {
        //     Plotly.newPlot('plot', data, layout);
        //     plotInitialized = true;
        // } else {
        //     Plotly.react('plot', data, layout);
        // }
    });

    
}



// ─────────────────────────────────────────────────────────────────────────────
// RULA score update
// ─────────────────────────────────────────────────────────────────────────────
function update_rula() {
    socket.on('update_rula', function (score) {
        rula_score = Math.max(...score);
        const rulaDiv = document.getElementById('rula_score');
        rulaDiv.innerHTML = `RULA Score: ${rula_score}`;
    });
}


// ─────────────────────────────────────────────────────────────────────────────
// Launch functions @ 30Hz update rate
// ─────────────────────────────────────────────────────────────────────────────
setTimeout(update_plot, 1 / 30 * 1000);
setTimeout(update_rula, 1 / 30 * 1000);
