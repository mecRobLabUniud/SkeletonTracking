import * as THREE from 'three'
import { scene } from './scene.js'

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────
const socket = io();

let distance = []
distance.push(createCapsule());


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

    let caps_color = '#00ffaa';

    capsule.geometry.dispose();
    capsule.geometry = new THREE.CapsuleGeometry(radius, length, 4, 8);
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
        if (point.c_h != null && point.c_r != null) {
            distance[0] = updateCapsule(
                    distance[0],
                    { x: point.c_h[0], y: point.c_h[1], z: point.c_h[2]},
                    { x: point.c_r[0], y: point.c_r[1], z: point.c_r[2]},
                    0.002, 
                    true
                );
            distance[0].visible = true;
        }
        else {
            distance[0].visible = false;
        }
    });    
}


// ─────────────────────────────────────────────────────────────────────────────
// Launch functions @ 30Hz update rate
// ─────────────────────────────────────────────────────────────────────────────
setTimeout(update_plot, 1 / 30 * 1000);
