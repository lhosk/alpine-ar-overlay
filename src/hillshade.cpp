#include "dem.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: hillshade <path.tif> [out.png]\n";
        return 1;
    }
    const std::string out_path = (argc > 2) ? argv[2] : "hillshade.png";

    Dem d;
    if (!load_dem(argv[1], d)) return 1;
    build_mesh(d);

    // ground spacing in meters, from the enu grid
    const float dx = d.east[1] - d.east[0];
    const float dy = d.north[(size_t)d.width] - d.north[0];  // negative

    // sun: azimuth 315 deg (nw), altitude 45 deg
    const double az  = 315.0 * M_PI / 180.0;
    const double alt =  45.0 * M_PI / 180.0;
    const double sx = std::cos(alt) * std::sin(az);
    const double sy = std::cos(alt) * std::cos(az);
    const double sz = std::sin(alt);

    cv::Mat img(d.height, d.width, CV_8UC1, cv::Scalar(0));

    for (int y = 1; y < d.height - 1; ++y) {
        for (int x = 1; x < d.width - 1; ++x) {
            size_t i = (size_t)y * d.width + x;

            // central differences, meters of rise per meter of run
            float dhdE = (d.elev[i + 1] - d.elev[i - 1]) / (2.0f * dx);
            float dhdN = (d.elev[i + d.width] - d.elev[i - d.width]) / (2.0f * dy);

            // normal, unnormalized
            double nx = -dhdE, ny = -dhdN, nz = 1.0;
            double len = std::sqrt(nx*nx + ny*ny + nz*nz);

            double dot = (nx*sx + ny*sy + nz*sz) / len;
            if (dot < 0.0) dot = 0.0;

            img.at<unsigned char>(y, x) = (unsigned char)(dot * 255.0);
        }
    }

    if (!cv::imwrite(out_path, img)) {
        std::cerr << "write failed: " << out_path << "\n";
        return 1;
    }

    std::cout << "wrote " << out_path << " (" << d.width << " x " << d.height << ")\n";
    std::cout << "ground spacing: " << dx << " m east, " << dy << " m north\n";
    return 0;
}
