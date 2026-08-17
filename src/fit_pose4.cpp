#include "dem.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <omp.h>

struct Cam { double e, n, u, fx, fy, cx, cy, znear; int W, H; };

static void render_skyline(const Dem& d, const Cam& c, double yaw_d, double pitch_d,
                           int xlo, int xhi, std::vector<int>& line) {
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

    std::vector<int> top(c.W, c.H);
    for (size_t t = 0; t < d.tris.size(); t += 3) {
        unsigned int a = d.tris[t], b = d.tris[t+1], g = d.tris[t+2];
        if (pz[a] <= c.znear || pz[b] <= c.znear || pz[g] <= c.znear) continue;
        double x0=px[a],y0=py[a],x1=px[b],y1=py[b],x2=px[g],y2=py[g];
        int minx = std::max((int)std::floor(std::min({x0,x1,x2})), xlo);
        int maxx = std::min((int)std::ceil (std::max({x0,x1,x2})), xhi);
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
                top[x] = y;
                break;
            }
    }
    line.assign(c.W, -1);
    for (int x = xlo; x <= xhi; ++x) if (top[x] < c.H) line[x] = top[x];
}

int main(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "usage: fit_pose4 <dem.tif> <ridge.txt> <W> <H> "
                     "<east0> <north0> <agl> [f35] [yaw0] [pitch0] [pos_range_m]\n";
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
    double e0 = atof(argv[5]), n0 = atof(argv[6]);
    double agl = atof(argv[7]);
    double f35 = (argc > 8)  ? atof(argv[8])  : 26.0;
    double yaw0 = (argc > 9) ? atof(argv[9])  : 200.0;
    double pit0 = (argc > 10)? atof(argv[10]) : 4.0;
    double prange = (argc > 11)? atof(argv[11]) : 150.0;

    c.fx = (f35/36.0)*c.W; c.fy = c.fx;
    c.cx = c.W*0.5; c.cy = c.H*0.5;
    c.znear = 100.0;

    int xlo = c.W, xhi = 0;
    for (auto& p : obs) { xlo = std::min(xlo, p.first); xhi = std::max(xhi, p.first); }
    xlo = std::max(xlo-2, 0); xhi = std::min(xhi+2, c.W-1);

    const float dx = d.east[1]-d.east[0];
    const float dy = d.north[(size_t)d.width]-d.north[0];

    auto set_pos = [&](double e, double n) {
        c.e = e; c.n = n;
        int gx = std::clamp((int)std::lround((e-d.east[0])/dx), 0, d.width-1);
        int gy = std::clamp((int)std::lround((n-d.north[0])/dy), 0, d.height-1);
        c.u = d.elev[(size_t)gy*d.width+gx] + agl;
    };

    auto cost = [&](double e, double n, double yaw, double pitch, int* used) {
        set_pos(e, n);
        std::vector<int> line;
        render_skyline(d, c, yaw, pitch, xlo, xhi, line);
        double s = 0.0; int cnt = 0;
        for (auto& p : obs) {
            int r = line[p.first];
            if (r < 0) continue;
            double err = (double)p.second - r;
            s += err*err; ++cnt;
        }
        if (used) *used = cnt;
        return cnt ? std::sqrt(s/cnt) : 1e9;
    };

    std::cout << "observations: " << obs.size() << ", columns " << xlo << ".." << xhi << "\n";
    std::cout << "searching position +/- " << prange << " m\n\n";

    double be = e0, bn = n0, by = yaw0, bp = pit0, bc = 1e9;

    double pstep = prange/3.0, ystep = 3.0, pistep = 1.5;
    double prng = prange, yrng = 15.0, pirng = 6.0;

    for (int pass = 0; pass < 4; ++pass) {
        double ce = be, cn = bn, cy = by, cp = bp;
        for (double e = ce-prng; e <= ce+prng+1e-9; e += pstep)
        for (double n = cn-prng; n <= cn+prng+1e-9; n += pstep)
        for (double y = cy-yrng; y <= cy+yrng+1e-9; y += ystep)
        for (double p = cp-pirng; p <= cp+pirng+1e-9; p += pistep) {
            int used = 0;
            double cst = cost(e, n, y, p, &used);
            if (used < (int)obs.size()*3/4) continue;
            if (cst < bc) { bc = cst; be = e; bn = n; by = y; bp = p; }
        }
        std::cout << "pass " << pass << ": e " << be << ", n " << bn
                  << ", yaw " << by << ", pitch " << bp
                  << ", rms " << bc << " px\n";
        prng = pstep; yrng = ystep; pirng = pistep;
        pstep /= 3.0; ystep /= 3.0; pistep /= 3.0;
    }

    int used = 0;
    cost(be, bn, by, bp, &used);
    double ang = std::atan(bc/c.fy)*180.0/M_PI;
    double moved = std::hypot(be-e0, bn-n0);

    std::cout << "\n--- solved ---\n";
    std::cout << "east:  " << be << " m  (moved " << (be-e0) << ")\n";
    std::cout << "north: " << bn << " m  (moved " << (bn-n0) << ")\n";
    std::cout << "position shift: " << moved << " m from initial guess\n";
    std::cout << "yaw:   " << by << " deg\n";
    std::cout << "pitch: " << bp << " deg\n";
    std::cout << "cam elevation: " << c.u << " m\n";
    std::cout << "rms residual: " << bc << " px = " << ang << " deg\n";
    std::cout << "points used: " << used << " / " << obs.size() << "\n";
    return 0;
}
