#pragma once
#include <string>
#include <vector>

// a dem loaded into a local east-north-up frame, meters
struct Dem {
    int width  = 0;
    int height = 0;

    // geotransform, lon/lat degrees
    double gt[6] = {0, 0, 0, 0, 0, 0};

    // enu origin, degrees
    double lon0 = 0.0;
    double lat0 = 0.0;

    // elevation grid, row-major, meters
    std::vector<float> elev;

    // vertex positions in enu meters, one per grid post
    std::vector<float> east;
    std::vector<float> north;

    // triangle indices into the grid, 3 per triangle
    std::vector<unsigned int> tris;

    float elev_min = 0.f;
    float elev_max = 0.f;
};

bool load_dem(const std::string& path, Dem& out);
void build_mesh(Dem& d);
