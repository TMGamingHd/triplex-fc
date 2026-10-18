Flow fields for the 3D viewer's *CFD* view, made by the Euler solver (`tfc_cfd`, `docs/design/CFD.md`). They are not in the repository (a case is about 0.6 MB and a library of them is more):

    python3 tools/cfd/make_library.py vehicles/starship.json            # Mach 2, 3, 5, 8, 12 at 0, 4 and 10 degrees: about an hour on twelve cores
    build/host/tfc_cfd vehicles/starship.json --mach 3 --alpha 4 --field console/cfd/starship/m3_a4.cfd   # one case

`console/demo/` holds the few that are committed. The viewer picks the case nearest the flight's Mach number and angle of attack.
