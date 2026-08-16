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
        std::cerr << "usage: overlay <dem.tif> <photo.jpg> <out.png> "
                     "<east> <north> <agl> <yaw_deg> <pitch_deg> [f35_mm] [znear]\n";
        return 1;
    }

    Dem d;
    if (!load_dem(argv[1], d)) return 1;
    build_mesh(d);

    cv::Mat photo = cv::imread(argv[2], cv::IMREAD_COLOR);
    if (photo.empty()) {
        std::cerr << "cannot read photo " << argv[2] << "\n";
        return 1;
    }

    const std::string out_path = argv[3];
    double cam_e   = atof(argv[4]);
    double cam_n   = atof(argv[5]);
    double agl     = atof(argv[6]);
    double yaw_d   = atof(argv[7]);
    double pitch_d = atof(argv[8]);
    double f35     = (argc > 9)  ? atof(argv[9])  : 26.0;
    double znear   = (argc > 10) ? atof(argv[10]) : 100.0;

    const int W = photo.cols, H = photo.rows;

    // intrinsics from 35mm-equivalent focal length
    const double fx = (f35 / 36.0) * W;
    const double fy = fx;
    const double cx = W * 0.5, cy = H * 0.5;
    const double hfov_d = 2.0 * std::atan(W / (2.0 * fx)) * 180.0 / M_PI;

    // ground height under camera
    const float dx = d.east[1] - d.east[0];
    const float dy = d.north[(size_t)d.width] - d.north[0];
    int gx = std::clamp((int)std::lround((cam_e - d.east[0]) / dx), 0, d.width - 1);
    int gy = std::clamp((int)std::lround((cam_n - d.north[0]) / dy), 0, d.height - 1);
    double cam_u = d.elev[(size_t)gy * d.width + gx] + agl;

    // rotation: rows are right, down, forward
    const double psi = yaw_d * M_PI / 180.0;
    const double th  = pitch_d * M_PI / 180.0;
    const double R[3][3] = {
        { std::cos(psi),                -std::sin(psi),                0.0           },
        { std::sin(psi) * std::sin(th),  std::cos(psi) * std::sin(th), -std::cos(th) },
        { std::sin(psi) * std::cos(th),  std::cos(psi) * std::cos(th),  std::sin(th) }
    };

    const size_t nv = d.east.size();
    std::vector<double> px(nv), py(nv), pz(nv);
    for (size_t i = 0; i < nv; ++i) {
        double we = d.east[i] - cam_e, wn = d.north[i] - cam_n, wu = d.elev[i] - cam_u;
        double xc = R[0][0]*we + R[0][1]*wn + R[0][2]*wu;
        double yc = R[1][0]*we + R[1][1]*wn + R[1][2]*wu;
        double zc = R[2][0]*we + R[2][1]*wn + R[2][2]*wu;
        pz[i] = zc;
        if (zc > znear) { px[i] = fx * xc / zc + cx; py[i] = fy * yc / zc + cy; }
    }

    std::vector<float> zbuf((size_t)W * H, std::numeric_limits<float>::max());
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));

    for (size_t t = 0; t < d.tris.size(); t += 3) {
        unsigned int a = d.tris[t], b = d.tris[t+1], c = d.tris[t+2];
        if (pz[a] <= znear || pz[b] <= znear || pz[c] <= znear) continue;

        double x0 = px[a], y0 = py[a], x1 = px[b], y1 = py[b], x2 = px[c], y2 = py[c];
        int minx = std::max((int)std::floor(std::min({x0,x1,x2})), 0);
        int maxx = std::min((int)std::ceil (std::max({x0,x1,x2})), W - 1);
        int miny = std::max((int)std::floor(std::min({y0,y1,y2})), 0);
        int maxy = std::min((int)std::ceil (std::max({y0,y1,y2})), H - 1);
        if (minx > maxx || miny > maxy) continue;

        double area = (x1-x0)*(y2-y0) - (x2-x0)*(y1-y0);
        if (std::abs(area) < 1e-12) continue;

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
                mask.at<unsigned char>(y, x) = 255;
            }
        }
    }

    // tint terrain region red, 35% opacity
    cv::Mat out = photo.clone();
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (mask.at<unsigned char>(y, x)) {
                cv::Vec3b& p = out.at<cv::Vec3b>(y, x);
                p[2] = (unsigned char)std::min(255.0, p[2] * 0.65 + 255 * 0.35);
                p[0] = (unsigned char)(p[0] * 0.65);
                p[1] = (unsigned char)(p[1] * 0.65);
            }

    // rendered skyline: topmost terrain pixel per column, drawn green
    int sky_cols = 0;
    long sky_sum = 0;
    int sky_min = H, sky_max = 0;
    for (int x = 0; x < W; ++x) {
        for (int y = 0; y < H; ++y) {
            if (mask.at<unsigned char>(y, x)) {
                cv::circle(out, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
                ++sky_cols;
                sky_sum += y;
                if (y < sky_min) sky_min = y;
                if (y > sky_max) sky_max = y;
                break;
            }
        }
    }
    double sky_mean = sky_cols ? (double)sky_sum / sky_cols : -1.0;

    cv::imwrite(out_path, out);

    std::cout << "wrote " << out_path << "\n";
    std::cout << "photo: " << W << " x " << H << "\n";
    std::cout << "fx = fy = " << fx << " px, hfov = " << hfov_d << " deg\n";
    std::cout << "cam enu: " << cam_e << ", " << cam_n << ", " << cam_u << " m\n";
    std::cout << "yaw " << yaw_d << " deg, pitch " << pitch_d << " deg\n";
    std::cout << "skyline in " << sky_cols << " / " << W << " columns\n";
    std::cout << "skyline row: min " << sky_min << ", mean " << sky_mean
              << ", max " << sky_max << "  (0 = top, " << H << " = bottom)\n";
    std::cout << "sky fraction: " << (sky_mean / H) * 100.0 << " % of frame above skyline\n";
    return 0;
}