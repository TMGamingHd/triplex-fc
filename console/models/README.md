# Your own vehicle models

Put a glTF file here (`.glb` or `.gltf`, one file with everything inside it) and the 3D viewer lists it under **Model** in the Overview lens as `Imported: NAME`. This is how a vehicle designed in CAD is seen in the viewer; nothing in this folder is committed (see `.gitignore`).

* **Exporting.** Any tool that writes glTF: Blender (File > Export > glTF 2.0, binary `.glb`), Fusion 360 and Onshape through Blender or the Autodesk/Onshape glTF exporters, or STEP converted with FreeCAD or Blender. Keep it to a few hundred thousand triangles; the viewer draws it as it is.
* **Axis and size.** The viewer asks which way the vehicle points in the file (`+Y` by default; many CAD tools have `+Z` up) and by default scales it so that its length is the vehicle file's length, with the tail at the origin, so that the engines, the centre of mass and the tanks the lenses draw are where the picture has them. Both can be changed in the panel.
* **Stages.** Name the nodes of a stage `stage1`, `stage2`... (any case; `Stage_2 interstage` works): they leave when that stage separates. Nodes without such a name go with the first stage.
* **What stays generated.** The engines' bells are not drawn (the picture has its own); their positions, the plumes, the gimbal, the force arrows, the pressure map's overlay on every surface, the streamlines, the shock, the tanks and all numbers come from the vehicle file and the simulator. See `docs/design/VIEWER.md`.
* **Files are served only by name**, from this folder, to a page that has the session token: nothing outside it can be read through the viewer.
