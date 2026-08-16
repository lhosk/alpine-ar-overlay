# alpine-ar-overlay

overlays crevasse and glacier data onto a live camera view of a mountain

the crevasses aren't detected from the image, they come from existing map data
and get lined up with whatever the camera is pointed at

not a nav tool, don't climb off it, crevasse data is coarse and moves

## the hard part

position is fine, gps gets you within ~5 m

heading is the problem, the magnetometer is off by 5-15 degrees and at 500 m out
5 degrees of heading error slides the overlay ~44 m sideways

fix is to render the skyline from the terrain model, find the skyline in the
camera frame, then push the pose around until they match, ridgelines are sharp
and effectively infinitely far away so they pin down orientation well, then fuse
that with the imu so it doesn't jitter frame to frame

comes down to solving for camera pose T

    T* = argmin_T  sum_i || u_i_detected - pi(K, T, X_i_DEM) ||

## layout

    alpine-ar-overlay/
    |-- CMakeLists.txt
    |-- README.md
    |-- .gitignore
    |-- src/          # the c++
    |   `-- main.cpp
    |-- include/      # headers
    |-- data/         # dems and calibration stuff, data/raw/ is gitignored
    |-- tools/        # python for data prep and plots
    |-- tests/
    |-- docs/         # error analysis, results
    `-- build/        # gitignored

## build

    mkdir -p build && cd build
    cmake ..
    make

c++17, cmake 3.16+, opencv 4.x

## data sources

- terrain: copernicus glo-30, usgs 3dep, swissalti3d
- glaciers: randolph glacier inventory, glims
- camera intrinsics: calibrated here from checkerboard photos

## results

nothing yet, numbers go here as they show up, reprojection error in px, heading
error in deg, overlay error in m at range, latency in ms
