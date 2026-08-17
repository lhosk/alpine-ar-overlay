#include "dem.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    if (argc < 9) {
        std::cerr << "usage: wireframe <dem.tif> <photo.jpg> <out.png> "
                     "<east> <north> <agl> <yaw> <pitch> [f35] [znear] [step]\n";
        return 1;
    }

    Dem d;
    if (!load_dem(argv[1], d, -81.812135, 36.094976)) return 1;
    build_mesh(d);

    cv::Mat photo = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (photo.empty()) { std::cerr << "cannot read photo\n"; return 1; }

    double cam_e = atof(argv[4]), cam_n = atof(argv[5]), agl = atof(argv[6]);
    double yaw_d = atof(argv[7]), pitch_d = atof(argv[8]);
    double f35   = (argc > 9)  ? atof(argv[9])  : 26.0;
    double znear = (argc > 10) ? atof(argv[10]) : 100.0;
    int    step  = (argc > 11) ? atoi(argv[11]) : 4;
    double maxd  = (argc > 12) ? atof(argv[12]) : 8000.0;
    bool   occl  = (argc > 13) ? (atoi(argv[13]) != 0) : true;

    const int W = photo.cols, H = photo.rows;
    const double fx = (f35 / 36.0) * W, fy = fx;
    const double cx = W * 0.5, cy = H * 0.5;

    const float dx = d.east[1] - d.east[0];
    const float dy = d.north[(size_t)d.width] - d.north[0];
    int gx = std::clamp((int)std::lround((cam_e - d.east[0]) / dx), 0, d.width - 1);
    int gy = std::clamp((int)std::lround((cam_n - d.north[0]) / dy), 0, d.height - 1);
    double cam_u = d.elev[(size_t)gy * d.width + gx] + agl;

    const double psi = yaw_d * M_PI / 180.0, th = pitch_d * M_PI / 180.0;
    const double R[3][3] = {
        { std::cos(psi),                -std::sin(psi),                0.0           },
        { std::sin(psi) * std::sin(th),  std::cos(psi) * std::sin(th), -std::cos(th) },
        { std::sin(psi) * std::cos(th),  std::cos(psi) * std::cos(th),  std::sin(th) }
    };

    const size_t nv = d.east.size();
    std::vector<double> px(nv), py(nv), pz(nv);
    for (size_t i = 0; i < nv; ++i) {
        double we = d.east[i]-cam_e, wn = d.north[i]-cam_n, wu = d.elev[i]-cam_u;
        double xc = R[0][0]*we + R[0][1]*wn + R[0][2]*wu;
        double yc = R[1][0]*we + R[1][1]*wn + R[1][2]*wu;
        double zc = R[2][0]*we + R[2][1]*wn + R[2][2]*wu;
        pz[i] = zc;
        if (zc > znear) { px[i] = fx*xc/zc + cx; py[i] = fy*yc/zc + cy; }
    }

    // depth buffer so hidden grid lines get culled
    std::vector<float> zbuf((size_t)W*H, std::numeric_limits<float>::max());
    for (size_t t = 0; t < d.tris.size(); t += 3) {
        unsigned int a = d.tris[t], b = d.tris[t+1], c = d.tris[t+2];
        if (pz[a] <= znear || pz[b] <= znear || pz[c] <= znear) continue;
        double x0=px[a],y0=py[a],x1=px[b],y1=py[b],x2=px[c],y2=py[c];
        int minx = std::max((int)std::floor(std::min({x0,x1,x2})), 0);
        int maxx = std::min((int)std::ceil (std::max({x0,x1,x2})), W-1);
        int miny = std::max((int)std::floor(std::min({y0,y1,y2})), 0);
        int maxy = std::min((int)std::ceil (std::max({y0,y1,y2})), H-1);
        if (minx > maxx || miny > maxy) continue;
        double area = (x1-x0)*(y2-y0) - (x2-x0)*(y1-y0);
        if (std::abs(area) < 1e-12) continue;
        for (int y = miny; y <= maxy; ++y)
            for (int x = minx; x <= maxx; ++x) {
                double fxp=x+0.5, fyp=y+0.5;
                double w0 = ((x1-fxp)*(y2-fyp)-(x2-fxp)*(y1-fyp))/area;
                double w1 = ((x2-fxp)*(y0-fyp)-(x0-fxp)*(y2-fyp))/area;
                double w2 = 1.0-w0-w1;
                if (w0<0||w1<0||w2<0) continue;
                float z = (float)(w0*pz[a]+w1*pz[b]+w2*pz[c]);
                size_t idx=(size_t)y*W+x;
                if (z < zbuf[idx]) zbuf[idx]=z;
            }
    }

    cv::Mat out = photo.clone();

    // draw a grid line segment only where it agrees with the depth buffer
    auto draw_seg = [&](size_t i, size_t j, cv::Scalar col) {
        if (pz[i] <= znear || pz[j] <= znear) return;
        if (maxd > 0 && pz[i] > maxd && pz[j] > maxd) return;
        double x0=px[i], y0=py[i], x1=px[j], y1=py[j];
        int n = (int)std::max(2.0, std::hypot(x1-x0, y1-y0));
        if (n > 4000) return;
        for (int s = 0; s <= n; ++s) {
            double a = (double)s/n;
            int x = (int)std::lround(x0 + a*(x1-x0));
            int y = (int)std::lround(y0 + a*(y1-y0));
            if (x < 0 || x >= W || y < 0 || y >= H) continue;
            double z = pz[i] + a*(pz[j]-pz[i]);
            if (occl && z > zbuf[(size_t)y*W+x] + 150.0f) continue;   // hidden
            cv::circle(out, cv::Point(x,y), 2, col, -1, cv::LINE_AA);
        }
    };

    // grid lines every `step` posts
    for (int y = 0; y < d.height; y += step)
        for (int x = 0; x + step < d.width; x += step)
            draw_seg((size_t)y*d.width+x, (size_t)y*d.width+x+step, cv::Scalar(0,255,255));

    for (int x = 0; x < d.width; x += step)
        for (int y = 0; y + step < d.height; y += step)
            draw_seg((size_t)y*d.width+x, (size_t)(y+step)*d.width+x, cv::Scalar(255,255,0));

    cv::imwrite(argv[3], out);
    std::cout << "wrote " << argv[3] << "\n";
    std::cout << "cam enu: " << cam_e << ", " << cam_n << ", " << cam_u << " m\n";
    std::cout << "yaw " << yaw_d << ", pitch " << pitch_d << ", grid step "
              << step << " posts (" << step*std::abs(dx) << " m)\n";
    return 0;
}