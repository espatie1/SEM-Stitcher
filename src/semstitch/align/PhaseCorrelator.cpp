#include "semstitch/align/PhaseCorrelator.hpp"
#include <opencv2/imgproc.hpp>

namespace semstitch {

static cv::Mat to32f(const cv::Mat& m8) {
    CV_Assert(m8.type() == CV_8UC1);
    cv::Mat f; m8.convertTo(f, CV_32F, 1.0/255.0);
    return f;
}

PairMatch phaseCorrelateGray8(const cv::Mat& a_gray8,
                              const cv::Mat& b_gray8,
                              const cv::Rect& roiA,
                              const cv::Rect& roiB,
                              int hanning_win)
{
    PairMatch out{};
    CV_Assert(a_gray8.type()==CV_8UC1 && b_gray8.type()==CV_8UC1);
    CV_Assert(roiA.width>0 && roiA.height>0);
    CV_Assert(roiA.size() == roiB.size());
    CV_Assert(roiA.x>=0 && roiA.y>=0 && roiA.x+roiA.width <= a_gray8.cols && roiA.y+roiA.height <= a_gray8.rows);
    CV_Assert(roiB.x>=0 && roiB.y>=0 && roiB.x+roiB.width <= b_gray8.cols && roiB.y+roiB.height <= b_gray8.rows);

    cv::Mat A = to32f(a_gray8(roiA));
    cv::Mat B = to32f(b_gray8(roiB));

    cv::Mat win;
    if (hanning_win > 0) {
        // полный Хэннинг по размеру ROI — это ок; отдельно "hanning_win" оставляем
        // как просто переключатель аподизации (тонкая настройка окна может быть позже)
        cv::createHanningWindow(win, A.size(), CV_32F);
    }

    double response = 0.0;
    cv::Point2d shift = cv::phaseCorrelate(A, B, win, &response);

    out.dx = shift.x;
    out.dy = shift.y;
    out.score = response;
    out.ok = std::isfinite(response);
    return out;
}

} // namespace semstitch
