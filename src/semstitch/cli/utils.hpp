#pragma once
#include <opencv2/core.hpp>
#include <string>

/*
  Helpers for viewing and saving images.
  These utilities do not change the original data on disk;
  they only prepare an image for display or write it to a file.
*/

/* Prepare an image for on-screen display.
   - If input is 8-bit grayscale, stretch contrast to [0..255].
   - For other types, return a clone without changes. */
cv::Mat displayize(const cv::Mat& src8u);

/* Save a mosaic image as PNG to 'path'.
   Prints a short message on success or failure. */
void save_mosaic_png(const cv::Mat& m, const std::string& path);
