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
