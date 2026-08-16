#include <iostream>
#include <opencv2/core.hpp>

int main() {
    std::cout << "alpine-ar-overlay: build OK\n";
    std::cout << "opencv version: " << CV_VERSION << "\n";

    cv::Mat m = cv::Mat::eye(3, 3, CV_64F);
    std::cout << "identity K:\n" << m << "\n";
    return 0;
}
