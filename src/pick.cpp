#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <iostream>
#include <vector>

static std::vector<cv::Point> pts;
static cv::Mat disp, base;
static double scale = 1.0;

static void redraw() {
    disp = base.clone();
    for (size_t i = 0; i < pts.size(); ++i) {
        cv::Point d((int)(pts[i].x * scale), (int)(pts[i].y * scale));
        cv::circle(disp, d, 5, cv::Scalar(0, 255, 255), -1);
        if (i) {
            cv::Point p((int)(pts[i-1].x * scale), (int)(pts[i-1].y * scale));
            cv::line(disp, p, d, cv::Scalar(0, 255, 255), 2);
        }
    }
    cv::imshow("pick ridgeline", disp);
}

static void on_mouse(int ev, int x, int y, int, void*) {
    if (ev == cv::EVENT_LBUTTONDOWN) {
        pts.push_back(cv::Point((int)(x / scale), (int)(y / scale)));
        redraw();
    } else if (ev == cv::EVENT_RBUTTONDOWN && !pts.empty()) {
        pts.pop_back();
        redraw();
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: pick <photo.jpg> <out.txt>\n";
        return 1;
    }

    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (img.empty()) { std::cerr << "cannot read photo\n"; return 1; }

    const int maxw = 1400;
    scale = (img.cols > maxw) ? (double)maxw / img.cols : 1.0;
    cv::resize(img, base, cv::Size(), scale, scale, cv::INTER_AREA);

    std::cout << "left click = add point along the ridgeline (left to right)\n";
    std::cout << "right click = undo\n";
    std::cout << "press s = save and quit, q = quit without saving\n";

    cv::namedWindow("pick ridgeline", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("pick ridgeline", on_mouse);
    redraw();

    for (;;) {
        int k = cv::waitKey(20) & 0xFF;
        if (k == 'q') return 0;
        if (k == 's') break;
    }

    std::ofstream f(argv[2]);
    for (auto& p : pts) f << p.x << " " << p.y << "\n";
    std::cout << "saved " << pts.size() << " points to " << argv[2] << "\n";
    return 0;
}
