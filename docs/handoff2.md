# alpine-ar-overlay — handoff 2

paste-into-new-claude-project context. read this whole file before doing anything.

## what this project is

c++ / linux (wsl2 ubuntu) computer vision project: register a rendered
dem skyline against a real photograph to solve for camera heading,
correcting the 5-15 deg magnetometer error that breaks gps+compass ar
overlays. target: meta reality labs cv/sensors postings. production c++,
real numbers, honest limitations. user is strong python, weak c++/linux
(actively fixing), physics + quantum + data science background.

## repo structure (exact, current)

    alpine-ar-overlay/
    |-- build/                  gitignored, cmake output, regenerates
    |-- data/
    |   |-- photos/photo.jpg    the test photo (2880x2160, exif stripped)
    |   |-- raw/dem_view.tif    THE dem: usgs 10m -> 30m, cropped
    |   |                       (-82.05..-81.55, 35.80..36.11)
    |   |-- landscape.txt       yaw sweep cost curve (yaw_deg rms_px)
    |   |-- ridge.txt           29 clicked far-ridge pts (x y, full-res px)
    |   |-- ridge_both.txt      ridge.txt + ridge_near.txt concatenated
    |   `-- ridge_near.txt      15 clicked near-hillside pts
    |-- docs/
    |   |-- img/
    |   |   |-- check_magneto.png   overlay at 212 deg (the failure)
    |   |   |-- check_solved.png    overlay at 193.3 deg (the result)
    |   |   `-- landscape.png       cost vs heading plot
    |   |-- handoff2.md         this file
    |   `-- results.md          full numbers, all experiments
    |-- include/dem.hpp         Dem struct + load_dem + build_mesh api
    |-- src/
    |   |-- dem.cpp             gdal geotiff -> enu mesh, ANCHORED origin
    |   |-- export_web.cpp      decimated mesh -> web/terrain.bin
    |   |-- fit_yaw.cpp         THE solver. yaw-only + landscape sweep
    |   |-- mesh_info.cpp       dem diagnostic dump
    |   |-- overlay.cpp         photo+dem+pose -> tint+skyline png
    |   `-- pick.cpp            opencv click tool -> x y pairs
    |-- web/
    |   |-- photo.jpg           copy for the viewer
    |   |-- terrain.bin         binary mesh (nv,nt,verts,tris)
    |   `-- viewer.html         three.js pose tuner, sliders, presets
    |-- .gitignore              build/, data/raw/, *.tif, *.mp4, data/*.png
    |-- CMakeLists.txt          -O2 release, openmp, dem as a library
    |-- LICENSE                 mit
    `-- README.md

## build & the commands that matter

    cd ~/dev/alpine-ar-overlay/build && cmake .. && make

    # solve heading (11 s):
    ./fit_yaw ../data/raw/dem_view.tif ../data/ridge.txt 2880 2160 0 0 1.6 26 3.69 0 150 260
    # args: dem ridge W H east north agl f35 pitch_fixed roll_fixed yaw_lo yaw_hi

    # render overlay at a pose:
    ./overlay ../data/raw/dem_view.tif ../data/photos/photo.jpg out.png 0 0 1.6 193.3 3.69 26 100
    # args: dem photo out east north agl yaw pitch f35 znear

    # click correspondences (wslg gui works):
    ./pick ../data/photos/photo.jpg ../data/ridge2.txt

    # 3d viewer:
    cd ../web && python3 -m http.server 8000   # -> localhost:8000/viewer.html

## solved state / ground facts

- camera: iphone xr, 26mm equiv -> fx=fy=2080 px at 2880 wide, c at center
- position prior: 36.094976, -81.812135 (+/-100 m hand guess, exif was
  stripped; real system would have gps to 5 m)
- enu origin ANCHORED at that lon/lat. dem.cpp takes anchor args. never
  let origin default to grid center — that bug silently moved the camera
  on every dem crop and burned half a session
- solved pose: east 0, north 0, agl 1.6 m, **yaw 193.3**, pitch 3.69
  (fixed, from earlier 2-param fit), roll 0
- rms 8.45 px = 0.233 deg on 29 far-ridge points. eyeball guess was 212.
  correction ~19 deg

## findings (the actual science)

1. far-ridge heading solve works: 0.233 deg residual, unique cost minimum
   over a 110 deg sweep
2. **bare-earth dems fail near-field in forest.** near hillside misfits by
   ~37 px no matter what. tested and RULED OUT: position freedom (4-param
   fit moved 88 m, no help), roll (converged to exactly 0). near-only fit
   wants +2.6 deg pitch = ~18 m at 400 m = tree canopy. conclusion: the
   terrain model is wrong near-field, not the camera model
3. **discrimination scales with ridge relief.** cost landscape is
   asymmetric: steep wall east of minimum, flat 15-17 px plateau west,
   because the clicked ridge has only ~47 px relief over 1200 px. alpine
   skylines are the favorable case; this is near the difficulty floor
4. perf: 11 min -> 39 s per fit. wins: -O2 (cmake had NO build type =
   -O0!, 4x), openmp 16 threads (4x), fov cull, edge-walk rasterizer.
   -O3 -march=native was SLOWER than -O2 here, reverted

## failed approaches (do not retry blindly)

- hsv color sky detection: broke on hazy sunset sky
- vertical-gradient skyline detection: locked onto the road, bright
  fields; autumn colors too noisy. detection remains OPEN — when retried,
  do it prediction-guided: search only +/-200 px around the rendered line
- rectangular lat/lon wireframe overlay: looks random against terrain.
  contours or the 3d viewer instead

## next steps, in priority order

1. **hold-out validation**: fit on ~15 of the 29 clicks, report error on
   the held-out 14. current 8.45 px is training error and overstates
   accuracy. cheap, do first
2. **second photo with exif intact** (airdrop/usb, never messaging/social
   — those strip it): real gps + real orientation -> run the whole
   pipeline with sensor-true priors, zero hand-placed inputs
3. prediction-guided automatic skyline detection (kills hand clicking)
4. canopy height model for near field (meta/glad global 1m chm exists)
5. ceres solver, but only after cost-eval is cheap; grid search at 11 s
   is currently fine

## working style (strict, learned the hard way)

- ONE command block at a time. extreme brevity. no menus of options
  unless the tradeoff genuinely needs user input — just decide
- user pastes into bash directly. heredocs >50 lines get truncated by
  the terminal — keep writes small, verify with grep/tail after EVERY
  file write before building
- pwd-check before any cd or append. most common failures all session:
  cd build from inside build, cat >> landing in build/CMakeLists.txt
- on any failure: ask for terminal output, never guess twice
- latex for equations, one-line variable definitions
- say exactly which file to open
