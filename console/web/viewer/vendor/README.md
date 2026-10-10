# Vendored third-party code

The viewer runs on the bench with no network, so the one library it needs is kept here, unmodified except for one line in each add-on.

| File | From | Licence |
|---|---|---|
| `three.module.js` | three.js r170 (`three@0.170.0`, `build/three.module.js`) | MIT, `LICENSE-three.txt` |
| `addons/loaders/GLTFLoader.js` | `three@0.170.0`, `examples/jsm/loaders/GLTFLoader.js` | MIT |
| `addons/utils/BufferGeometryUtils.js` | `three@0.170.0`, `examples/jsm/utils/BufferGeometryUtils.js` | MIT |

SHA-256 of the files as downloaded (before the edit below):
```
ce1fa418de16a19495a9f72495580e3015d7745c296d3ce0485897f902ddedfb  three.module.js
45139faddd5aaf48ed2d62203d976e5cbd703db1a592de40527f9f6cf58abd44  addons/loaders/GLTFLoader.js
c25b7930e570e9ec56173cd3b866ec8d2e10016630db3937efb439daf1cedbf6  addons/utils/BufferGeometryUtils.js
```
The edit: the add-ons say `from 'three'`, which a browser can only resolve with an import map (an inline script, which the console's content-security policy does not allow). Each is changed to `from '../../three.module.js'`.

To update: download the same three files of a newer release from `https://cdn.jsdelivr.net/npm/three@VERSION/`, repeat the one-line edit, and fly a model through the viewer.
