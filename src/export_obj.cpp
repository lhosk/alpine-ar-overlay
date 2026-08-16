#include "dem.hpp"
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: export_obj <path.tif> [out.obj]\n";
        return 1;
    }
    const std::string out_path = (argc > 2) ? argv[2] : "terrain.obj";

    Dem d;
    if (!load_dem(argv[1], d)) return 1;
    build_mesh(d);

    std::ofstream f(out_path);
    if (!f) {
        std::cerr << "cannot write " << out_path << "\n";
        return 1;
    }

    f << "# alpine-ar-overlay terrain mesh, enu meters\n";

    // obj is y-up, our frame is z-up: write (east, up, -north)
    for (size_t i = 0; i < d.east.size(); ++i) {
        f << "v " << d.east[i] << " " << d.elev[i] << " " << -d.north[i] << "\n";
    }

    // obj indices are 1-based
    for (size_t t = 0; t < d.tris.size(); t += 3) {
        f << "f " << d.tris[t] + 1 << " "
                  << d.tris[t + 1] + 1 << " "
                  << d.tris[t + 2] + 1 << "\n";
    }

    f.close();

    std::cout << "wrote " << out_path << "\n";
    std::cout << d.east.size() << " vertices, " << d.tris.size() / 3 << " triangles\n";
    return 0;
}
