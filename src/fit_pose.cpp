#include "dem.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

struct Cam { double e, n, u, fx, fy, cx, cy, znear; int W, H; };

// render skyline row per column for a given yaw/pitch; -1 where no terrain
static void render_skyline(const Dem& d, const Cam& c, double yaw_d, double pitch_d,
                           std::vector<int>& line) {
    const double psi = yaw_d * M_PI / 180.0, th = pitch_d * M_PI / 180.0;
    const double R[3][3] = {
        { std::cos(psi),                -std::sin(psi),                0.0           },
        { std::sin(psi) * std::sin(th),  std::cos(psi) * std::sin(th), -std::cos(th) },
        { std::sin(psi) * std::cos(th),  std::cos(psi) * std::cos(th),  std::sin(th) }
    };

    const size_t nv = d.east.size();
    std::vector<double> px(nv), py(nv), pz(nv);
    for (size_t i = 0; i < nv; ++i) {
        double we = d.east[i]-c.e, wn = d.north[i]-c.n, wu = d.elev[i]-c.u;
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

        for (int x = minx; x <= maxx; ++x)
            for (int y = miny; y <= maxy && y < top[x]; ++y) {
                double fxp=x+0.5, fyp=y+0.5;
                double w0 = ((x1-fxp)*(y2-fyp)-(x2-fxp)*(y1-fyp))/area;
                double w1 = ((x2-fxp)*(y0-fyp)-(x0-fxp)*(y2-fyp))/area;
                double w2 = 1.0-w0-w1;
                if (w0<0||w1<0||w2<0) continue;
                if (y < top[x]) top[x] = y;
                break;
            }
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
    if (!load_dem(argv[1], d)) return 1;
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

    auto cost = [&](double yaw, double pitch, int* used) {
        std::vector<int> line;
        render_skyline(d, c, yaw, pitch, line);
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
    double best_y = yaw0, best_p = pit0, best_c = 1e9;
    double yr = 20.0, pr = 10.0, ys = 2.0, ps = 1.0;

    for (int pass = 0; pass < 3; ++pass) {
        double by = best_y, bp = best_p;
        for (double y = by - yr; y <= by + yr + 1e-9; y += ys)
            for (double p = bp - pr; p <= bp + pr + 1e-9; p += ps) {
                int used = 0;
                double cst = cost(y, p, &used);
                if (used < (int)obs.size()/2) continue;
                if (cst < best_c) { best_c = cst; best_y = y; best_p = p; }
            }
        std::cout << "pass " << pass << ": yaw " << best_y << ", pitch " << best_p
                  << ", rms " << best_c << " px\n";
        yr = ys*2; pr = ps*2; ys /= 4; ps /= 4;
    }

    int used = 0;
    cost(best_y, best_p, &used);

    double ang = std::atan(best_c / c.fy) * 180.0 / M_PI;
    std::cout << "\nbest yaw:   " << best_y << " deg\n";
    std::cout << "best pitch: " << best_p << " deg\n";
    std::cout << "rms residual: " << best_c << " px = " << ang << " deg\n";
    std::cout << "points used: " << used << " / " << obs.size() << "\n";
    return 0;
}