#include "dem.hpp"
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: mesh_info <path.tif>\n";
        return 1;
    }

    Dem d;
    if (!load_dem(argv[1], d)) return 1;
    build_mesh(d);

    std::cout << "grid: " << d.width << " x " << d.height << "\n";
    std::cout << "enu origin lon/lat: " << d.lon0 << ", " << d.lat0 << "\n";
    std::cout << "east  range: " << d.east.front() << " .. " << d.east.back() << " m\n";
    std::cout << "north range: " << d.north.front() << " .. " << d.north.back() << " m\n";
    std::cout << "elev  range: " << d.elev_min << " .. " << d.elev_max << " m\n";
    std::cout << "vertices: " << d.east.size() << "\n";
    std::cout << "triangles: " << d.tris.size() / 3 << "\n";
    return 0;
}
