#ifndef CONTOUR_EXTRACTOR_H
#define CONTOUR_EXTRACTOR_H

#include "opencv3/include/opencv2/opencv.hpp"
#include "basic_calc.h"

class contour_extractor
{
public:
    contour_extractor();

    // initializes background image for background subtraction
    void initBackground(cv::Mat *image);

    ///
    /// \brief extractContours      calculates the binary image from the input image and extracts as well as filters the contours
    /// \param image                input gray value image
    /// \param binary_image         result binary image
    /// \param min_area             minimum area enclosed by a contour to be considered a larva contour
    /// \param max_area             pendant to min_area
    /// \return                     contours extracted from the binary image
    ///
    std::vector<std::vector<cv::Point>> extractContours(cv::Mat *image, cv::Mat *binary_image, int min_area, int max_area);

private:
    cv::Mat current_background;
    //adds image region to the background image
    void addBackgroundObject(cv::Mat* bounding_rectangle, std::vector<cv::Point> contour, int x_pos, int y_pos);
    //background subtraction
    cv::Mat subtract(cv::Mat* source);
    //calculates histogramm
    std::vector<float> calcHist(cv::Mat* image, float size);
    //iterative flood fill algorithm
    void revFloodFill(cv::Mat* image, int a, int b);

};

#endif // CONTOUR_EXTRACTOR_H
