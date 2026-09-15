import * as THREE from 'three'
import { scene } from './scene.js'

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────
const socket = io();

let p_real = [];
let p_r = [];
const t_start = Date.now();


// ─────────────────────────────────────────────────────────────────────────────
// Plot capsules mesh
// ─────────────────────────────────────────────────────────────────────────────
function createPoint(p, color) {
    const point = new THREE.Vector3(p.x, p.y, p.z);

    // const geometry = new THREE.BufferGeometry(0.01, 0.01, 4, 8);
    // const material = new THREE.Material( { color: '#aaaaaa', transparent: true, opacity: 0.5 } );

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( point, 3 ) );
    const material = new THREE.PointsMaterial( { color: color, size: 0.03, sizeAttenuation: true, transparent: true, opacity: 0.5 - 0.1*(t_start - Date.now()) } );
    const points = new THREE.Points( geometry, material );
    scene.add( points );

    return points;
}


// ─────────────────────────────────────────────────────────────────────────────
// 3D plot update
// ─────────────────────────────────────────────────────────────────────────────
function update_plot() {
    socket.on('update_plot', function (point) {        
        if (point.p_real != null && point.p_r != null) {
            createPoint(
                { x: point.p_real[0], y: point.p_real[1], z: point.p_real[2]},
                '#0000aa'
            );
            createPoint(
                { x: point.p_r[0], y: point.p_r[1], z: point.p_r[2]},
                '#00aa00'
            );
        }
    });    
}


// ─────────────────────────────────────────────────────────────────────────────
// Launch functions @ 30Hz update rate
// ─────────────────────────────────────────────────────────────────────────────
setTimeout(update_plot, 1 / 30 * 1000);
