import * as THREE from 'three'
import { scene } from './scene.js'

// ─────────────────────────────────────────────────────────────────────────────
// Parameters
// ─────────────────────────────────────────────────────────────────────────────
const socket = io();

let traj_real = [];
let traj_r = [];
let curve_real = [];
let curve_r = [];
const t_start = Date.now();


// ─────────────────────────────────────────────────────────────────────────────
// Plot 3D point
// ─────────────────────────────────────────────────────────────────────────────
function createPoint(p, color) {
    const point = new THREE.Vector3(p.x, p.y, p.z);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute( 'position', new THREE.Float32BufferAttribute( point, 3 ) );
    const material = new THREE.PointsMaterial( { color: color, size: 0.01, sizeAttenuation: true, transparent: true, opacity: 0.5 - 0.1*(t_start - Date.now()) } );
    const points = new THREE.Points( geometry, material );
    scene.add( points );

    return points;
}


// ─────────────────────────────────────────────────────────────────────────────
// Plot interpolated curve through waypoints
// ─────────────────────────────────────────────────────────────────────────────
function createCurve(p, color) {
    const curve = new THREE.CatmullRomCurve3(p);
    const curvePoints = curve.getPoints(100);
    const geometry = new THREE.BufferGeometry().setFromPoints(curvePoints);
    const material = new THREE.LineBasicMaterial({ color: color });
    const curveLine = new THREE.Line(geometry, material);
    scene.add(curveLine);

    return curveLine;
}


// ─────────────────────────────────────────────────────────────────────────────
// 3D plot update
// ─────────────────────────────────────────────────────────────────────────────
function update_plot() {
    socket.on('update_plot', function (point) {    
        
        if (point.x != null) {
            createPoint(
                { x: point.x[0], y: point.y[0], z: point.z[0]},
                '#aa0000'
            );
        }

        if (point.p_real != null && point.p_r != null) {
            traj_real.push(new THREE.Vector3(point.p_real[0], point.p_real[1], point.p_real[2]));
            traj_r.push(new THREE.Vector3(point.p_r[0], point.p_r[1], point.p_r[2]));
        }

        if (traj_real.length >= 30) {
            traj_real.splice(0, 1);
        }
        if (traj_r.length >= 30) {
            traj_r.splice(0, 1);
        }

        if (traj_real.length >= 2 && traj_r.length >= 2) {
            if (curve_real != [] && curve_r != []) {
                scene.remove(curve_real);
                scene.remove(curve_r);
            }
            
            curve_real = createCurve(traj_real, '#0000aa');
            curve_r = createCurve(traj_r, '#00aa00');
        }
        
    });    
}


// ─────────────────────────────────────────────────────────────────────────────
// Launch functions @ 30Hz update rate
// ─────────────────────────────────────────────────────────────────────────────
setTimeout(update_plot, 1 / 30 * 1000);
