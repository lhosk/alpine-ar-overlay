#include "dem.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <omp.h>

struct Cam { double e, n, u, fx, fy, cx, cy, znear; int W, H; };

// render skyline row per column for a given yaw/pitch; -1 where no terrain
static void render_skyline(const Dem& d, const Cam& c, double yaw_d, double pitch_d, double roll_d,
                           std::vector<int>& line) {
    const double psi = yaw_d * M_PI / 180.0, th = pitch_d * M_PI / 180.0;
    const double B[3][3] = {
        { std::cos(psi),                -std::sin(psi),                0.0           },
        { std::sin(psi) * std::sin(th),  std::cos(psi) * std::sin(th), -std::cos(th) },
        { std::sin(psi) * std::cos(th),  std::cos(psi) * std::cos(th),  std::sin(th) }
    };
    const double phi = roll_d * M_PI / 180.0;
    const double cr = std::cos(phi), sr = std::sin(phi);
    const double R[3][3] = {
        { cr*B[0][0]+sr*B[1][0], cr*B[0][1]+sr*B[1][1], cr*B[0][2]+sr*B[1][2] },
        { -sr*B[0][0]+cr*B[1][0], -sr*B[0][1]+cr*B[1][1], -sr*B[0][2]+cr*B[1][2] },
        { B[2][0], B[2][1], B[2][2] }
    };

    const size_t nv = d.east.size();
    std::vector<double> px, py, pz;
    px.assign(nv, 0.0); py.assign(nv, 0.0); pz.assign(nv, -1.0);

    // fov half-angle plus margin; skip vertices well outside the view cone
    const double halffov = std::atan(c.W * 0.5 / c.fx) * 1.6;
    const double cy_ = std::cos(psi), sy_ = std::sin(psi);

    for (size_t i = 0; i < nv; ++i) {
        double we = d.east[i]-c.e, wn = d.north[i]-c.n;
        // forward component in the horizontal plane
        double fwd = sy_*we + cy_*wn;
        if (fwd <= 0.0) continue;
        double side = cy_*we - sy_*wn;
        if (std::abs(side) > fwd * std::tan(halffov)) continue;

        double wu = d.elev[i]-c.u;
        double xc = R[0][0]*we + R[0][1]*wn + R[0][2]*wu;
        double yc = R[1][0]*we + R[1][1]*wn + R[1][2]*wu;
        double zc = R[2][0]*we + R[2][1]*wn + R[2][2]*wu;
        pz[i] = zc;
        if (zc > c.znear) { px[i] = c.fx*xc/zc + c.cx; py[i] = c.fy*yc/zc + c.cy; }
    }

    line.assign(c.W, -1);
    std::vector<int> top(c.W, c.H);

    for (size_t t = 0; t < d.tris.size(); t += 3) {
        unsigned int a = d.tris[t], b = d.tris[t+1], g = d.tris[t+2];
        if (pz[a] <= c.znear || pz[b] <= c.znear || pz[g] <= c.znear) continue;
        double x0=px[a],y0=py[a],x1=px[b],y1=py[b],x2=px[g],y2=py[g];
        int minx = std::max((int)std::floor(std::min({x0,x1,x2})), 0);
        int maxx = std::min((int)std::ceil (std::max({x0,x1,x2})), c.W-1);
        if (minx > maxx) continue;
        int miny = std::max((int)std::floor(std::min({y0,y1,y2})), 0);
        if (miny >= c.H) continue;
        double area = (x1-x0)*(y2-y0) - (x2-x0)*(y1-y0);
        if (std::abs(area) < 1e-12) continue;
        int maxy = std::min((int)std::ceil(std::max({y0,y1,y2})), c.H-1);

        auto edge = [&](double ax, double ay, double bx, double by) {
            if (bx < ax) { std::swap(ax,bx); std::swap(ay,by); }
            int xa = std::max((int)std::ceil(ax), 0);
            int xb = std::min((int)std::floor(bx), c.W-1);
            if (xb < xa) return;
            double dxe = bx - ax;
            for (int x = xa; x <= xb; ++x) {
                double t = (dxe > 1e-12) ? (x - ax) / dxe : 0.0;
                int y = (int)std::lround(ay + t * (by - ay));
                if (y < 0) y = 0;
                if (y < top[x]) top[x] = y;
            }
        };
        edge(x0,y0,x1,y1); edge(x1,y1,x2,y2); edge(x2,y2,x0,y0);
    }
    for (int x = 0; x < c.W; ++x) line[x] = (top[x] < c.H) ? top[x] : -1;
}

int main(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "usage: fit_pose <dem.tif> <ridge.txt> <W> <H> "
                     "<east> <north> <agl> [f35] [yaw0] [pitch0]\n";
        return 1;
    }

    Dem d;
    if (!load_dem(argv[1], d, -81.812135, 36.094976)) return 1;
    build_mesh(d);

    std::ifstream f(argv[2]);
    std::vector<std::pair<int,int>> obs;
    int ox, oy;
    while (f >> ox >> oy) obs.emplace_back(ox, oy);
    if (obs.empty()) { std::cerr << "no points in " << argv[2] << "\n"; return 1; }

    Cam c;
    c.W = atoi(argv[3]); c.H = atoi(argv[4]);
    c.e = atof(argv[5]); c.n = atof(argv[6]);
    double agl = atof(argv[7]);
    double f35 = (argc > 8) ? atof(argv[8]) : 26.0;
    double yaw0 = (argc > 9) ? atof(argv[9]) : 212.0;
    double pit0 = (argc > 10) ? atof(argv[10]) : 5.0;

    c.fx = (f35/36.0) * c.W; c.fy = c.fx;
    c.cx = c.W*0.5; c.cy = c.H*0.5;
    c.znear = 100.0;

    const float dx = d.east[1]-d.east[0];
    const float dy = d.north[(size_t)d.width]-d.north[0];
    int gx = std::clamp((int)std::lround((c.e-d.east[0])/dx), 0, d.width-1);
    int gy = std::clamp((int)std::lround((c.n-d.north[0])/dy), 0, d.height-1);
    c.u = d.elev[(size_t)gy*d.width+gx] + agl;

    std::cout << "observations: " << obs.size() << "\n";
    std::cout << "cam enu: " << c.e << ", " << c.n << ", " << c.u << " m\n";
    std::cout << "fx = " << c.fx << " px\n\n";

    auto cost = [&](double yaw, double pitch, double roll, int* used) {
        std::vector<int> line;
        render_skyline(d, c, yaw, pitch, roll, line);
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

    // coarse then fine
    double best_y = yaw0, best_p = pit0, best_r = 0.0, best_c = 1e9;
    double yr = 20.0, pr = 10.0, rr = 8.0, ys = 2.0, ps = 1.0, rs = 2.0;

    for (int pass = 0; pass < 3; ++pass) {
        double by = best_y, bp = best_p, br = best_r;
        int ny = (int)std::round(2*yr/ys) + 1;
        int np = (int)std::round(2*pr/ps) + 1;
        int nr = (int)std::round(2*rr/rs) + 1;

        #pragma omp parallel for collapse(3) schedule(dynamic)
        for (int iy = 0; iy < ny; ++iy)
            for (int ip = 0; ip < np; ++ip)
              for (int ir = 0; ir < nr; ++ir) {
                double y = by - yr + iy*ys;
                double p = bp - pr + ip*ps;
                double r = br - rr + ir*rs;
                int used = 0;
                double cst = cost(y, p, r, &used);
                if (used < (int)obs.size()/2) continue;
                #pragma omp critical
                if (cst < best_c) { best_c = cst; best_y = y; best_p = p; best_r = r; }
            }
        std::cout << "pass " << pass << ": yaw " << best_y << ", pitch " << best_p << ", roll " << best_r
                  << ", rms " << best_c << " px\n";
        yr = ys*2; pr = ps*2; rr = rs*2; ys /= 4; ps /= 4; rs /= 4;
    }

    int used = 0;
    cost(best_y, best_p, best_r, &used);

    double ang = std::atan(best_c / c.fy) * 180.0 / M_PI;
    std::cout << "\nbest yaw:   " << best_y << " deg\n";
    std::cout << "best pitch: " << best_p << " deg\n";
    std::cout << "rms residual: " << best_c << " px = " << ang << " deg\n";
    std::cout << "points used: " << used << " / " << obs.size() << "\n";
    return 0;
}