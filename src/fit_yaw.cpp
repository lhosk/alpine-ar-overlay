#include "dem.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <omp.h>

struct Cam { double e, n, u, fx, fy, cx, cy, znear; int W, H; };

static void render_skyline(const Dem& d, const Cam& c, double yaw_d, double pitch_d,
                           double roll_d, std::vector<int>& line) {
    const double psi = yaw_d * M_PI / 180.0, th = pitch_d * M_PI / 180.0;
    const double B[3][3] = {
        { std::cos(psi),                -std::sin(psi),                0.0           },
        { std::sin(psi) * std::sin(th),  std::cos(psi) * std::sin(th), -std::cos(th) },
        { std::sin(psi) * std::cos(th),  std::cos(psi) * std::cos(th),  std::sin(th) }
    };
    const double phi = roll_d * M_PI / 180.0;
    const double cr = std::cos(phi), sr = std::sin(phi);
    const double R[3][3] = {
        {  cr*B[0][0]+sr*B[1][0],  cr*B[0][1]+sr*B[1][1],  cr*B[0][2]+sr*B[1][2] },
        { -sr*B[0][0]+cr*B[1][0], -sr*B[0][1]+cr*B[1][1], -sr*B[0][2]+cr*B[1][2] },
        {  B[2][0],                B[2][1],                B[2][2]               }
    };

    const size_t nv = d.east.size();
    std::vector<double> px(nv), py(nv), pz(nv, -1.0);
    const double halffov = std::atan(c.W * 0.5 / c.fx) * 1.6;
    const double cy_ = std::cos(psi), sy_ = std::sin(psi);

    for (size_t i = 0; i < nv; ++i) {
        double we = d.east[i]-c.e, wn = d.north[i]-c.n;
        double fwd = sy_*we + cy_*wn;
        if (fwd <= 0.0) continue;
        double side = cy_*we - sy_*wn;
        if (std::abs(side) > fwd * std::tan(halffov)) continue;
        double wu = d.elev[i]-c.u;
        double xc = R[0][0]*we + R[0][1]*wn + R[0][2]*wu;
        double yc = R[1][0]*we + R[1][1]*wn + R[1][2]*wu;
        double zc = R[2][0]*we + R[2][1]*wn + R[2][2]*wu;
        if (zc <= c.znear) continue;
        pz[i] = zc;
        px[i] = c.fx*xc/zc + c.cx;
        py[i] = c.fy*yc/zc + c.cy;
    }

    std::vector<int> top(c.W, c.H);
    for (size_t t = 0; t < d.tris.size(); t += 3) {
        unsigned int a = d.tris[t], b = d.tris[t+1], g = d.tris[t+2];
        if (pz[a] < 0 || pz[b] < 0 || pz[g] < 0) continue;
        double x0=px[a],y0=py[a],x1=px[b],y1=py[b],x2=px[g],y2=py[g];

        auto edge = [&](double ax, double ay, double bx, double by) {
            if (bx < ax) { std::swap(ax,bx); std::swap(ay,by); }
            int xa = std::max((int)std::ceil(ax), 0);
            int xb = std::min((int)std::floor(bx), c.W-1);
            if (xb < xa) return;
            double dxe = bx - ax;
            for (int x = xa; x <= xb; ++x) {
                double tt = (dxe > 1e-12) ? (x - ax) / dxe : 0.0;
                int y = (int)std::lround(ay + tt * (by - ay));
                if (y < 0) y = 0;
                if (y < top[x]) top[x] = y;
            }
        };
        edge(x0,y0,x1,y1); edge(x1,y1,x2,y2); edge(x2,y2,x0,y0);
    }
    line.assign(c.W, -1);
    for (int x = 0; x < c.W; ++x) if (top[x] < c.H) line[x] = top[x];
}

int main(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "usage: fit_yaw <dem.tif> <ridge.txt> <W> <H> <east> <north> <agl> "
                     "[f35] [pitch_fixed] [roll_fixed] [yaw_lo] [yaw_hi]\n";
        return 1;
    }

    Dem d;
    if (!load_dem(argv[1], d, -81.812135, 36.094976)) return 1;
    build_mesh(d);

    std::ifstream f(argv[2]);
    std::vector<std::pair<int,int>> obs;
    int ox, oy;
    while (f >> ox >> oy) obs.emplace_back(ox, oy);
    if (obs.empty()) { std::cerr << "no points\n"; return 1; }

    Cam c;
    c.W = atoi(argv[3]); c.H = atoi(argv[4]);
    c.e = atof(argv[5]); c.n = atof(argv[6]);
    double agl  = atof(argv[7]);
    double f35  = (argc > 8)  ? atof(argv[8])  : 26.0;
    double pit  = (argc > 9)  ? atof(argv[9])  : 3.69;
    double rol  = (argc > 10) ? atof(argv[10]) : 0.0;
    double ylo  = (argc > 11) ? atof(argv[11]) : 150.0;
    double yhi  = (argc > 12) ? atof(argv[12]) : 260.0;

    c.fx = (f35/36.0)*c.W; c.fy = c.fx;
    c.cx = c.W*0.5; c.cy = c.H*0.5;
    c.znear = 100.0;

    const float dx = d.east[1]-d.east[0];
    const float dy = d.north[(size_t)d.width]-d.north[0];
    int gx = std::clamp((int)std::lround((c.e-d.east[0])/dx), 0, d.width-1);
    int gy = std::clamp((int)std::lround((c.n-d.north[0])/dy), 0, d.height-1);
    c.u = d.elev[(size_t)gy*d.width+gx] + agl;

    auto cost = [&](double yaw, int* used) {
        std::vector<int> line;
        render_skyline(d, c, yaw, pit, rol, line);
        double s = 0.0; int n = 0;
        for (auto& p : obs) {
            if (p.first < 0 || p.first >= c.W) continue;
            int r = line[p.first];
            if (r < 0) continue;
            double e = (double)p.second - r;
            s += e*e; ++n;
        }
        if (used) *used = n;
        return n ? std::sqrt(s/n) : 1e9;
    };

    std::cout << "observations: " << obs.size() << "\n";
    std::cout << "cam enu: " << c.e << ", " << c.n << ", " << c.u
              << " m (fixed)\n";
    std::cout << "pitch " << pit << " deg, roll " << rol << " deg (fixed)\n";
    std::cout << "searching yaw " << ylo << " .. " << yhi << " deg\n\n";

    // full sweep at 0.5 deg so we see the whole cost landscape
    const double step0 = 0.5;
    int nsteps = (int)std::round((yhi - ylo)/step0) + 1;
    std::vector<double> costs(nsteps, 1e9);

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < nsteps; ++i) {
        int used = 0;
        double cst = cost(ylo + i*step0, &used);
        if (used >= (int)obs.size()/2) costs[i] = cst;
    }

    int besti = 0;
    for (int i = 1; i < nsteps; ++i) if (costs[i] < costs[besti]) besti = i;
    double by = ylo + besti*step0, bc = costs[besti];

    // report the landscape: local minima under 2x the best
    std::cout << "cost landscape (local minima):\n";
    for (int i = 1; i + 1 < nsteps; ++i)
        if (costs[i] < costs[i-1] && costs[i] < costs[i+1] && costs[i] < 2.5*bc)
            std::cout << "  yaw " << (ylo+i*step0) << " deg, rms "
                      << costs[i] << " px\n";

    // refine around the best
    for (double st = step0/4; st > 0.01; st /= 4) {
        double b0 = by;
        for (double y = b0 - st*4; y <= b0 + st*4 + 1e-9; y += st) {
            int used = 0;
            double cst = cost(y, &used);
            if (used < (int)obs.size()/2) continue;
            if (cst < bc) { bc = cst; by = y; }
        }
    }

    int used = 0;
    cost(by, &used);
    double ang = std::atan(bc/c.fy)*180.0/M_PI;

    std::cout << "\nbest yaw: " << by << " deg\n";
    std::cout << "rms residual: " << bc << " px = " << ang << " deg\n";
    std::cout << "points used: " << used << " / " << obs.size() << "\n";
    return 0;
}