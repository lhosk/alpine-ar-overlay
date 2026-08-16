#include "dem.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: render_view <dem.tif> [out.png] "
                     "[east north agl yaw_deg pitch_deg hfov_deg]\n";
        return 1;
    }
    const std::string out_path = (argc > 2) ? argv[2] : "view.png";

    Dem d;
    if (!load_dem(argv[1], d)) return 1;
    build_mesh(d);

    // camera: default at south edge, 2 m above ground, looking north
    double cam_e   = (argc > 3) ? atof(argv[3]) : 0.0;
    double cam_n   = (argc > 4) ? atof(argv[4]) : -2000.0;
    double agl     = (argc > 5) ? atof(argv[5]) : 2.0;
    double yaw_d   = (argc > 6) ? atof(argv[6]) : 0.0;
    double pitch_d = (argc > 7) ? atof(argv[7]) : 5.0;
    double hfov_d  = (argc > 8) ? atof(argv[8]) : 65.0;
    double znear   = (argc > 9) ? atof(argv[9]) : 50.0;

    // ground height under the camera, nearest grid post
    const float dx = d.east[1] - d.east[0];
    const float dy = d.north[(size_t)d.width] - d.north[0];
    int gx = (int)std::lround((cam_e - d.east[0]) / dx);
    int gy = (int)std::lround((cam_n - d.north[0]) / dy);
    gx = std::clamp(gx, 0, d.width - 1);
    gy = std::clamp(gy, 0, d.height - 1);
    double cam_u = d.elev[(size_t)gy * d.width + gx] + agl;

    // image and intrinsics
    const int W = 1280, H = 720;
    const double hfov = hfov_d * M_PI / 180.0;
    const double fx = (W * 0.5) / std::tan(hfov * 0.5);
    const double fy = fx;
    const double cx = W * 0.5, cy = H * 0.5;

    // rotation from yaw/pitch: rows are right, down, forward
    const double psi = yaw_d * M_PI / 180.0;
    const double th  = pitch_d * M_PI / 180.0;
    const double R[3][3] = {
        { std::cos(psi),                -std::sin(psi),                0.0            },
        { std::sin(psi) * std::sin(th),  std::cos(psi) * std::sin(th), -std::cos(th)  },
        { std::sin(psi) * std::cos(th),  std::cos(psi) * std::cos(th),  std::sin(th)  }
    };

    // project every vertex once
    const size_t nv = d.east.size();
    std::vector<double> px(nv), py(nv), pz(nv);
    for (size_t i = 0; i < nv; ++i) {
        double we = d.east[i]  - cam_e;
        double wn = d.north[i] - cam_n;
        double wu = d.elev[i]  - cam_u;

        double xc = R[0][0]*we + R[0][1]*wn + R[0][2]*wu;
        double yc = R[1][0]*we + R[1][1]*wn + R[1][2]*wu;
        double zc = R[2][0]*we + R[2][1]*wn + R[2][2]*wu;

        pz[i] = zc;
        if (zc > znear) {
            px[i] = fx * xc / zc + cx;
            py[i] = fy * yc / zc + cy;
        }
    }

    cv::Mat img(H, W, CV_8UC1, cv::Scalar(20));
    std::vector<float> zbuf((size_t)W * H, std::numeric_limits<float>::max());

    // sun, enu
    const double az = 315.0 * M_PI / 180.0, alt = 45.0 * M_PI / 180.0;
    const double sx = std::cos(alt) * std::sin(az);
    const double sy = std::cos(alt) * std::cos(az);
    const double sz = std::sin(alt);

    long drawn = 0;
    for (size_t t = 0; t < d.tris.size(); t += 3) {
        unsigned int a = d.tris[t], b = d.tris[t+1], c = d.tris[t+2];
        if (pz[a] <= znear || pz[b] <= znear || pz[c] <= znear) continue;

        double x0 = px[a], y0 = py[a];
        double x1 = px[b], y1 = py[b];
        double x2 = px[c], y2 = py[c];

        int minx = (int)std::floor(std::min({x0, x1, x2}));
        int maxx = (int)std::ceil (std::max({x0, x1, x2}));
        int miny = (int)std::floor(std::min({y0, y1, y2}));
        int maxy = (int)std::ceil (std::max({y0, y1, y2}));
        if (maxx < 0 || minx >= W || maxy < 0 || miny >= H) continue;
        minx = std::max(minx, 0); maxx = std::min(maxx, W - 1);
        miny = std::max(miny, 0); maxy = std::min(maxy, H - 1);

        double area = (x1 - x0)*(y2 - y0) - (x2 - x0)*(y1 - y0);
        if (std::abs(area) < 1e-12) continue;

        // face normal in enu, for shading
        double ux = d.east[b]-d.east[a], uy = d.north[b]-d.north[a], uz = d.elev[b]-d.elev[a];
        double vx = d.east[c]-d.east[a], vy = d.north[c]-d.north[a], vz = d.elev[c]-d.elev[a];
        double nx = uy*vz - uz*vy, ny = uz*vx - ux*vz, nz = ux*vy - uy*vx;
        double nl = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (nl < 1e-12) continue;
        if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; }
        double dot = (nx*sx + ny*sy + nz*sz) / nl;
        double shade = 0.25 + 0.75 * std::max(0.0, dot);

        for (int y = miny; y <= maxy; ++y) {
            for (int x = minx; x <= maxx; ++x) {
                double fxp = x + 0.5, fyp = y + 0.5;
                double w0 = ((x1-fxp)*(y2-fyp) - (x2-fxp)*(y1-fyp)) / area;
                double w1 = ((x2-fxp)*(y0-fyp) - (x0-fxp)*(y2-fyp)) / area;
                double w2 = 1.0 - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0) continue;

                float z = (float)(w0*pz[a] + w1*pz[b] + w2*pz[c]);
                size_t idx = (size_t)y * W + x;
                if (z >= zbuf[idx]) continue;
                zbuf[idx] = z;
                img.at<unsigned char>(y, x) = (unsigned char)(shade * 255.0);
            }
        }
        ++drawn;
    }

    if (!cv::imwrite(out_path, img)) {
        std::cerr << "write failed\n";
        return 1;
    }

    std::cout << "wrote " << out_path << "\n";
    std::cout << "cam enu: " << cam_e << ", " << cam_n << ", " << cam_u << " m\n";
    std::cout << "yaw " << yaw_d << " deg, pitch " << pitch_d << " deg, hfov " << hfov_d << " deg\n";
    std::cout << "fx = fy = " << fx << " px\n";
    std::cout << "triangles rasterized: " << drawn << "\n";
    return 0;
}