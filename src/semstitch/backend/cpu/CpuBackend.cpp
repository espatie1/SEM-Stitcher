#include "CpuBackend.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

namespace semstitch {

//------------------------------------------------------------------
// Drift (phase correlation via OpenCV)
//------------------------------------------------------------------
DriftVector CpuBackend::drift(const Frame& prev, const Frame& curr) {
    cv::Mat a(prev.height, prev.width, CV_8UC1, const_cast<unsigned char*>(prev.data.data()));
    cv::Mat b(curr.height, curr.width, CV_8UC1, const_cast<unsigned char*>(curr.data.data()));
    cv::Point2d shift = cv::phaseCorrelate(a, b);
    return {shift.x, shift.y};
}

//------------------------------------------------------------------
// Feature matching + RANSAC homography
//------------------------------------------------------------------
Homography CpuBackend::match(const Frame& prev, const Frame& curr) {
    cv::Mat img1(prev.height, prev.width, CV_8UC1, const_cast<unsigned char*>(prev.data.data()));
    cv::Mat img2(curr.height, curr.width, CV_8UC1, const_cast<unsigned char*>(curr.data.data()));

    auto orb = cv::ORB::create(2000);
    std::vector<cv::KeyPoint> k1, k2;
    cv::Mat d1, d2;
    orb->detectAndCompute(img1, cv::noArray(), k1, d1);
    orb->detectAndCompute(img2, cv::noArray(), k2, d2);

    cv::BFMatcher bf(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn;
    bf.knnMatch(d1, d2, knn, 2);

    std::vector<cv::DMatch> good;
    for (auto& v : knn)
        if (v.size() == 2 && v[0].distance < 0.75f * v[1].distance)
            good.push_back(v[0]);

    std::vector<cv::Point2f> p1, p2;
    for (auto& m : good) {
        p1.push_back(k1[m.queryIdx].pt);
        p2.push_back(k2[m.trainIdx].pt);
    }

    Homography Hwrap{};
    if (p1.size() >= 4) {
        cv::Mat H = cv::findHomography(p1, p2, cv::RANSAC);
        if (!H.empty()) {
            std::copy(reinterpret_cast<double*>(H.data),
                      reinterpret_cast<double*>(H.data) + 9,
                      Hwrap.h);
        }
    }
    return Hwrap;
}

} // namespace semstitch

