#include "dem.hpp"

#include <gdal_priv.h>
#include <cmath>
#include <iostream>

namespace {
constexpr double kEarthR = 6378137.0;
constexpr double kDeg2Rad = M_PI / 180.0;
}

bool load_dem(const std::string& path, Dem& out) {
    GDALAllRegister();

    GDALDataset* ds = (GDALDataset*)GDALOpen(path.c_str(), GA_ReadOnly);
    if (!ds) {
        std::cerr << "failed to open " << path << "\n";
        return false;
    }

    out.width  = ds->GetRasterXSize();
    out.height = ds->GetRasterYSize();
    ds->GetGeoTransform(out.gt);

    out.elev.resize((size_t)out.width * out.height);

    CPLErr err = ds->GetRasterBand(1)->RasterIO(
        GF_Read, 0, 0, out.width, out.height,
        out.elev.data(), out.width, out.height, GDT_Float32, 0, 0);

    GDALClose(ds);

    if (err != CE_None) {
        std::cerr << "raster read failed\n";
        return false;
    }

    out.elev_min = out.elev[0];
    out.elev_max = out.elev[0];
    for (float e : out.elev) {
        if (e < out.elev_min) out.elev_min = e;
        if (e > out.elev_max) out.elev_max = e;
    }

    // enu origin at grid center
    out.lon0 = out.gt[0] + ((out.width  - 1) * 0.5) * out.gt[1];
    out.lat0 = out.gt[3] + ((out.height - 1) * 0.5) * out.gt[5];

    return true;
}

void build_mesh(Dem& d) {
    const double mps_lat = kEarthR * kDeg2Rad;
    const double mps_lon = mps_lat * std::cos(d.lat0 * kDeg2Rad);

    d.east.resize((size_t)d.width * d.height);
    d.north.resize((size_t)d.width * d.height);

    for (int y = 0; y < d.height; ++y) {
        for (int x = 0; x < d.width; ++x) {
            double lon = d.gt[0] + x * d.gt[1];
            double lat = d.gt[3] + y * d.gt[5];

            size_t i = (size_t)y * d.width + x;
            d.east[i]  = (float)((lon - d.lon0) * mps_lon);
            d.north[i] = (float)((lat - d.lat0) * mps_lat);
        }
    }

    // two triangles per grid cell
    d.tris.clear();
    d.tris.reserve((size_t)(d.width - 1) * (d.height - 1) * 6);

    for (int y = 0; y < d.height - 1; ++y) {
        for (int x = 0; x < d.width - 1; ++x) {
            unsigned int tl = (unsigned int)(y * d.width + x);
            unsigned int tr = tl + 1;
            unsigned int bl = tl + d.width;
            unsigned int br = bl + 1;

            d.tris.push_back(tl); d.tris.push_back(bl); d.tris.push_back(tr);
            d.tris.push_back(tr); d.tris.push_back(bl); d.tris.push_back(br);
        }
    }
}
