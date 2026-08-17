# results

## setup

- **site**: linn cove viaduct overlook, blue ridge parkway, nc
- **camera**: iphone xr wide, 26 mm equivalent, exif stripped
- **image**: 2880 x 2160, downscaled from native 4032 x 3024
- **intrinsics**: fx = fy = 2080 px, cx = 1440, cy = 1080, hfov 69.4 deg
  - derived from 35mm-equivalent focal length, no checkerboard calibration
- **dem**: usgs 10 m, downsampled to 30 m, ~45 x 34 km around the site
- **position prior**: 36.094976, -81.812135, +/- ~100 m (hand-placed, no gps in exif)
- **correspondences**: hand-clicked ridgeline points

## far-ridge fit (29 points, columns 368-1567)

target is the distant skyline near linville gorge, ~25 km out

| quantity | value |
|---|---|
| initial heading (by eye) | 212 deg |
| solved yaw | 196 deg |
| solved pitch | 3.69 deg |
| rms residual | 10.1 px |
| rms residual (angular) | 0.279 deg |
| points used | 29 / 29 |

heading correction: **16 deg**

for scale, 16 deg of heading error at 500 m range puts an overlay marker
~143 m off target. a magnetometer is spec'd at 5-15 deg, so this is the
same order as the error the project exists to correct

## combined fit (44 points, far ridge + near hillside)

added ridge points along the near tree line on the right side of frame

| quantity | far only | far + near |
|---|---|---|
| yaw | 193.5 deg | 205.6 deg |
| pitch | 3.69 deg | 4.51 deg |
| rms | 7.08 px | 36.74 px |
| rms (angular) | 0.195 deg | 1.01 deg |

*(run on the uncropped tile; the cropped-tile far-only number is 10.1 px)*

**a 2-parameter model cannot fit both ridges at once.** residual grows 5x
and yaw shifts 12 deg. three candidate causes, not yet separated:

1. **position error** — near ridge is hypersensitive to it. angular error
   goes as `dtheta ~ dx / R`, so 100 m of position error is ~0.2 deg at
   25 km but 10-20 deg at 300-500 m
2. **canopy** — dem is bare earth, the near hillside carries 20-30 m of trees
3. **roll** — not currently modelled, phone was probably not level

this is the main open finding

## known issues

- enu origin was tied to grid center, so it moved whenever the dem was
  re-cropped and silently relocated the camera. now anchored to a fixed
  lon/lat. **any result predating this fix is suspect**
- pose search is brute-force grid search, ~720 ms per skyline render,
  ~8 min per 2-parameter fit. 4-parameter search is not tractable this way
- residual is a *fit* quality, not an accuracy. position is a fixed prior,
  so position error partially absorbs into yaw
- single photo, single site. n = 1

## next

1. speed: bearing cull + skyline-only rasterization, target <20 ms/render
2. replace grid search with ceres, solve yaw/pitch/roll/east/north jointly
3. more photos, more sites, report a spread rather than one number
4. automatic skyline detection — hsv threshold and vertical-gradient methods
   both failed on this image (hazy sky, road edges, autumn colour)

## near-ridge investigation (resolved)

three hypotheses for why yaw+pitch cannot fit near and far ridges together:

| hypothesis | test | rms | verdict |
|---|---|---|---|
| position error | 4-param fit, yaw/pitch/east/north | 38.7 px | ruled out |
| roll | 3-param fit, yaw/pitch/roll | 37.5 px, roll -> 0.0 deg | ruled out |
| canopy | near-only fit | wants +2.6 deg pitch vs far fit | supported |

adding position freedom let the camera move 88 m and did not help. adding
roll converged to exactly zero and did not help. no camera pose explains
the near ridge

near-only fit: yaw 209 deg, pitch 6.31 deg, rms 38.9 px — it cannot even
fit itself well with two free parameters, meaning the near skyline's
*shape* is wrong in the dem, not just its offset

**conclusion**: the mismatch is in the terrain model, not the camera model.
the dem is bare earth; the near hillside carries mature forest. at ~400 m,
25 m of canopy is ~3.6 deg ~ 130 px, the right order for the residual

implication for outdoor ar in forested terrain: bare-earth dems are
sufficient for far skylines (>10 km, 0.28 deg achieved) and insufficient
for near ones. a canopy height model is needed for near-field registration

## performance

| change | time (2-param fit, 29 pts) |
|---|---|
| initial | 11 min |
| -O2 (was building -O0) | 2 min 21 s |
| -O3 -march=native | 2 min 58 s (worse, reverted) |
| + openmp, 16 threads | **39 s** |

17x total. the single biggest win was noticing cmake had no build type set

## cost landscape (yaw-only fit)

fixed: position (gps prior), pitch 3.69, roll 0. free: yaw. full sweep
150-260 deg at 0.5 deg, 11 s wall clock with openmp

![cost vs heading](../data/landscape.png)

- global minimum 193.3 deg, rms 8.45 px = 0.233 deg, unique in the range
- **but discrimination is asymmetric**: cost rises steeply east of the
  minimum (near hillside enters frame) and stays nearly flat west of it
  (15-17 px plateau from 150-190 deg)
- cause: the clicked far ridge has only ~47 px of vertical relief across
  1200 px of image. low-relief skylines weakly constrain heading
- implication: method accuracy depends on ridge relief. high-relief
  alpine skylines are the favorable case; flat appalachian horizons are
  near the difficulty floor. convergence needs a prior good to roughly
  +/-20 deg on the east side, looser on the west
