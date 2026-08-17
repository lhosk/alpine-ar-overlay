#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: skyline <photo.jpg> <out.png> [sky_thresh]\n";
        return 1;
    }
    double thresh = (argc > 3) ? atof(argv[3]) : 0.55;

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (img.empty()) { std::cerr << "cannot read photo\n"; return 1; }

    const int W = img.cols, H = img.rows;

    // sky score: blue-dominant and bright
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat sky(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            cv::Vec3b p = img.at<cv::Vec3b>(y, x);
            cv::Vec3b h = hsv.at<cv::Vec3b>(y, x);
            double b = p[0], g = p[1], r = p[2];
            double v = h[2] / 255.0;
            double s = h[1] / 255.0;
            // sky: bright, low-to-mid saturation, blue >= red
            double score = v * (b >= r ? 1.0 : 0.4) * (s < 0.6 ? 1.0 : 0.5);
            if (score > thresh) sky.at<unsigned char>(y, x) = 255;
        }
    }

    // clean up
    cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
    cv::morphologyEx(sky, sky, cv::MORPH_CLOSE, k);
    cv::morphologyEx(sky, sky, cv::MORPH_OPEN, k);

    // skyline: lowest sky pixel per column that has sky continuous from top
    std::vector<int> line(W, -1);
    int found = 0;
    for (int x = 0; x < W; ++x) {
        int y = 0;
        while (y < H && sky.at<unsigned char>(y, x)) ++y;
        if (y > 0 && y < H) { line[x] = y; ++found; }
    }

    cv::Mat out = img.clone();
    for (int x = 0; x < W; ++x)
        if (line[x] >= 0)
            cv::circle(out, cv::Point(x, line[x]), 3, cv::Scalar(255, 0, 255), -1);

    cv::imwrite(argv[2], out);

    long sum = 0; int mn = H, mx = 0;
    for (int x = 0; x < W; ++x) if (line[x] >= 0) {
        sum += line[x];
        if (line[x] < mn) mn = line[x];
        if (line[x] > mx) mx = line[x];
    }

    std::cout << "detected skyline in " << found << " / " << W << " columns\n";
    if (found) {
        std::cout << "row: min " << mn << ", mean " << (double)sum/found << ", max " << mx << "\n";
        std::cout << "sky fraction: " << ((double)sum/found)/H*100.0 << " %\n";
    }
    return 0;
}
