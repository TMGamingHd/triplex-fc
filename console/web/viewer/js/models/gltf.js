// A model that is not generated: a glTF file (.glb or .gltf) exported from CAD or a modelling tool, put in console/models/ and chosen in the viewer. It takes the place of the generated body; everything else (the engines
// of the vehicle file, the plumes, the lenses' overlays) stays, because it comes from the vehicle file and the simulator, not from the picture.
//
// What the viewer needs to know about a file, and how it is told:
//   * which way the vehicle points in the file (its axis, tail to nose): chosen in the panel (+Y by default, as most tools export a rocket standing up);
//   * its size: by default it is scaled so that its length along that axis is the vehicle file's length (so that the engines of the file, the centre of mass and the tanks are where the picture has them);
//   * its stages: nodes named `stage1`, `stage2`... (any case, with a space, dash or underscore: `Stage_2 interstage`) go with that stage and leave when it separates; every other node goes with the first stage.
// The origin of the picture is placed at the tail of the first stage, which is where the vehicle file measures from.
import * as THREE from '../../vendor/three.module.js';
import { GLTFLoader } from '../../vendor/addons/loaders/GLTFLoader.js';

export const AXES = [['+y', '+Y (up: most CAD exports)'], ['-y', '−Y'], ['+x', '+X'], ['-x', '−X'], ['+z', '+Z (up: most CAD tools)'], ['-z', '−Z']];
const AXIS_VEC = { '+y': [0, 1, 0], '-y': [0, -1, 0], '+x': [1, 0, 0], '-x': [-1, 0, 0], '+z': [0, 0, 1], '-z': [0, 0, -1] };

/** Fetch and parse a model file from the console. Returns the glTF scene (a THREE.Group). */
export async function loadGltf(name) {
  const tok = sessionStorage.getItem('tfc.token') || '';
  const r = await fetch('/api/viewer/model?name=' + encodeURIComponent(name), { headers: { 'X-TFC-Token': tok } });
  if (!r.ok) throw new Error(`${name}: ${r.status} ${r.statusText}`);
  const buf = await r.arrayBuffer();
  return new Promise((resolve, reject) => new GLTFLoader().parse(buf, '', (g) => resolve(g.scene), (e) => reject(new Error(`${name}: ${e.message || e}`))));
}

/** Turn a loaded scene into the vehicle's model frame: +Y along the axis, the tail at the origin, centred across, at the vehicle's length unless told otherwise. */
export function fit(scene, spec, { axis = '+y', fitLength = true, roll = 0, offset = 0, scale = 1 } = {}) {
  const root = new THREE.Group(); root.name = 'imported';
  const inner = new THREE.Group(); inner.add(scene); root.add(inner);
  const a = new THREE.Vector3(...AXIS_VEC[axis]);
  inner.quaternion.setFromUnitVectors(a, new THREE.Vector3(0, 1, 0));
  inner.rotateOnWorldAxis(new THREE.Vector3(0, 1, 0), roll * Math.PI / 180);
  inner.updateMatrixWorld(true);
  let box = new THREE.Box3().setFromObject(inner);
  const len = box.max.y - box.min.y || 1, k = (fitLength ? spec.length / len : 1) * scale;
  inner.scale.setScalar(k); inner.updateMatrixWorld(true);
  box = new THREE.Box3().setFromObject(inner);
  inner.position.set(-(box.min.x + box.max.x) / 2, -box.min.y + offset, -(box.min.z + box.max.z) / 2);
  inner.updateMatrixWorld(true);
  return { root, inner, scale: k, length: (box.max.y - box.min.y) };
}

/** Which stage a node belongs to by its name (0-based), or -1. */
export function stageOf(name) { const m = /^stage[\s_-]*(\d+)/i.exec(name || ''); return m ? +m[1] - 1 : -1; }

/** Move the nodes of a fitted model into the groups of the stages (each gets a wrapper that carries the fit). `groups[i]` is stage i's THREE.Group; the nodes with no stage in their name go to stage 0. */
export function distribute(fitted, groups) {
  const wraps = groups.map((g) => {
    const w = new THREE.Group(); w.name = 'imported';
    w.position.copy(fitted.inner.position); w.quaternion.copy(fitted.inner.quaternion); w.scale.copy(fitted.inner.scale);
    g.add(w); return w;
  });
  const top = fitted.inner.children[0];
  top.updateMatrix();
  for (const n of [...top.children]) {
    const i = Math.min(Math.max(stageOf(n.name), 0), groups.length - 1);
    n.applyMatrix4(top.matrix);                                                     // the scene's own transform (usually none) goes onto the node it was above
    wraps[i].add(n);
  }
}
