#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: skyline2 <photo.jpg> <out.png> [blur] [top_frac]\n";
        return 1;
    }
    int blur = (argc > 3) ? atoi(argv[3]) : 9;
    double top_frac = (argc > 4) ? atof(argv[4]) : 0.95;

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (img.empty()) { std::cerr << "cannot read photo\n"; return 1; }
    const int W = img.cols, H = img.rows;

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(blur | 1, blur | 1), 0);

    // vertical gradient: positive where bright above, dark below
    cv::Mat gy;
    cv::Sobel(gray, gy, CV_32F, 0, 1, 3);

    int limit = (int)(H * top_frac);

    std::vector<int> line(W, -1);
    for (int x = 0; x < W; ++x) {
        float best = 0.f; int besty = -1;
        for (int y = 1; y < limit; ++y) {
            float g = gy.at<float>(y, x);   // sky bright -> terrain dark = positive
            if (g > best) { best = g; besty = y; }
        }
        line[x] = besty;
    }

    // median filter to kill outliers
    std::vector<int> sm(W);
    const int win = 31;
    for (int x = 0; x < W; ++x) {
        std::vector<int> v;
        for (int d = -win/2; d <= win/2; ++d) {
            int xx = x + d;
            if (xx >= 0 && xx < W && line[xx] >= 0) v.push_back(line[xx]);
        }
        if (v.empty()) { sm[x] = -1; continue; }
        std::sort(v.begin(), v.end());
        sm[x] = v[v.size()/2];
    }

    cv::Mat out = img.clone();
    long sum = 0; int n = 0, mn = H, mx = 0;
    for (int x = 0; x < W; ++x) {
        if (sm[x] < 0) continue;
        cv::circle(out, cv::Point(x, sm[x]), 3, cv::Scalar(0, 255, 255), -1);
        sum += sm[x]; ++n;
        if (sm[x] < mn) mn = sm[x];
        if (sm[x] > mx) mx = sm[x];
    }
    cv::imwrite(argv[2], out);

    std::cout << "skyline in " << n << " / " << W << " columns\n";
    if (n) std::cout << "row: min " << mn << ", mean " << (double)sum/n
                     << ", max " << mx << "  (sky " << ((double)sum/n)/H*100 << " %)\n";
    return 0;
}
