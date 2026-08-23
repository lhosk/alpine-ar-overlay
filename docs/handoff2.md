# alpine-ar-overlay — handoff 2 (post first real session)

## state: working end-to-end pipeline with measured results

photo: iphone xr, linn cove viaduct overlook, blue ridge parkway nc, exif
stripped, 2880x2160. intrinsics from 26mm equiv: fx=fy=2080 px, cx=1440,
cy=1080, hfov 69.4. position prior 36.094976, -81.812135 (+/- 100 m,
hand-placed; a real system would have gps)

## key results (all in docs/results.md)

- yaw-only fit vs 29 hand-clicked far-ridge points: **193.3 deg,
  rms 8.45 px = 0.233 deg**, vs 212 deg eyeball guess. correction ~19 deg
- cost landscape swept 150-260 deg: unique minimum, but **asymmetric** —
  steep wall east (near hillside enters frame), near-flat plateau west
  (only 15-17 px). cause: clicked ridge has ~47 px relief over 1200 px.
  low-relief skylines weakly constrain heading
- **near-ridge mismatch fully diagnosed**: position freedom (4-param,
  moved 88 m) didn't help; roll converged to exactly 0 and didn't help;
  near-only fit wants +2.6 deg pitch = ~18 m extra height at 400 m =
  **tree canopy**. bare-earth dems fail for near-field registration in
  forest. this is the main finding
- perf: 11 min -> 39 s for a 2-param fit. wins: -O2 (cmake had NO build
  type = -O0 the whole time, 4x), openmp (4x), fov cull. -O3 -march=native
  was WORSE than -O2, reverted
- visual proof: check_solved.png vs check_magneto.png in docs/img/

## architecture

- include/dem.hpp + src/dem.cpp: gdal geotiff -> enu mesh (meters).
  enu origin ANCHORED at -81.812135, 36.094976 — never grid center.
  (grid-center origin silently moved the camera on every crop; that bug
  cost a session-half. any tool loading a dem passes the anchor)
- src/overlay.cpp: photo + dem + pose -> red tint + green skyline png
- src/pick.cpp: opencv gui, click ridge points, save x y pairs (wslg works)
- src/fit_yaw.cpp: THE fitter. yaw-only, pitch/roll/position fixed
  (honest sensor model: phone accel gives pitch/roll to ~1 deg, gps 5 m,
  only yaw is bad). full sweep + landscape dump to data/landscape.txt,
  openmp, edge-walk skyline rasterizer (no interior fill)
- src/export_web.cpp + web/viewer.html: decimated mesh -> three.js pose
  tuner, photo backdrop, live sliders, solved/eyeball preset buttons.
  serve with python3 -m http.server 8000 from web/
- src/mesh_info.cpp: diagnostic dump

## data

- data/raw/dem_view.tif: usgs 10m downsampled to 30m, cropped to view
  (-82.05..-81.55, 35.80..36.11). THE dem. bigger tiles: opentopography
  copernicus glo-30 or usgs 3dep (1m has no coverage here; 10m worked)
- data/photos/photo.jpg, data/ridge.txt (29 far pts), ridge_near.txt (15),
  ridge_both.txt, landscape.txt
- solved pose: east 0, north 0, agl 1.6, yaw 193.3, pitch 3.69, roll 0

## deleted as superseded

skyline.cpp/skyline2.cpp (color + gradient detectors both failed on hazy
autumn photo — road edges, canopy; detection remains open, do it
prediction-guided within +/-200 px of rendered line), fit_pose/fit_pose4
(superseded by fit_yaw), wireframe/contour (viewer.html replaces),
hillshade/export_obj/dem_info/render_view/calibrate (served their purpose)

## next steps, in order

1. hold-out validation: fit on half the clicks, error on the other half
   (current 8.45 px is training error)
2. a second photo with exif intact (airdrop/usb, not messaging) -> real
   gps + orientation, run pipeline with sensor-true priors end to end
3. prediction-guided skyline detection to replace hand clicking
4. canopy: try a canopy height model (meta/glad has global 1m chm) for
   the near field
5. ceres solver once cost eval is cheap enough to matter

## working style notes (learned the hard way)

- user pastes blocks into bash; heredocs > 50 lines get truncated by the
  terminal. keep patches small, verify with grep after every write
- ALWAYS pwd-check before cd build or cat >> CMakeLists.txt — half the
  session's failures were appends landing in build/ or double-cd
- user wants: one command block at a time, no long explanations, latex
  equations with one-line variable definitions, told exactly which file
  to open and look at
