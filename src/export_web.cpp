#include "dem.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: export_web <dem.tif> <out.bin> [stride] [maxdist_m]\n";
        return 1;
    }
    int stride = (argc > 3) ? atoi(argv[3]) : 4;
    double maxd = (argc > 4) ? atof(argv[4]) : 25000.0;

    Dem d;
    if (!load_dem(argv[1], d, -81.812135, 36.094976)) return 1;
    build_mesh(d);

    // subsampled grid
    int nx = (d.width  + stride - 1) / stride;
    int ny = (d.height + stride - 1) / stride;

    std::vector<float> verts;
    std::vector<int> idx((size_t)nx*ny, -1);
    verts.reserve((size_t)nx*ny*3);

    int n = 0;
    for (int gy = 0; gy < ny; ++gy) {
        for (int gx = 0; gx < nx; ++gx) {
            int x = gx*stride, y = gy*stride;
            if (x >= d.width || y >= d.height) continue;
            size_t i = (size_t)y*d.width + x;
            double e = d.east[i], nn = d.north[i];
            if (std::hypot(e, nn) > maxd) continue;
            idx[(size_t)gy*nx+gx] = n++;
            verts.push_back((float)e);
            verts.push_back(d.elev[i]);
            verts.push_back((float)-nn);
        }
    }

    std::vector<unsigned int> tris;
    for (int gy = 0; gy + 1 < ny; ++gy)
        for (int gx = 0; gx + 1 < nx; ++gx) {
            int a = idx[(size_t)gy*nx+gx],     b = idx[(size_t)gy*nx+gx+1];
            int c = idx[(size_t)(gy+1)*nx+gx], e = idx[(size_t)(gy+1)*nx+gx+1];
            if (a<0||b<0||c<0||e<0) continue;
            tris.push_back(a); tris.push_back(c); tris.push_back(b);
            tris.push_back(b); tris.push_back(c); tris.push_back(e);
        }

    FILE* f = fopen(argv[2], "wb");
    if (!f) { std::cerr << "cannot write\n"; return 1; }
    unsigned int nv = n, nt = (unsigned int)tris.size();
    fwrite(&nv, 4, 1, f);
    fwrite(&nt, 4, 1, f);
    fwrite(verts.data(), 4, verts.size(), f);
    fwrite(tris.data(), 4, tris.size(), f);
    fclose(f);

    std::cout << "wrote " << argv[2] << "\n";
    std::cout << nv << " vertices, " << nt/3 << " triangles\n";
    std::cout << "stride " << stride << ", max dist " << maxd << " m\n";
    return 0;
}
