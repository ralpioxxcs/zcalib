#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
#include <opencv2/opencv.hpp>
extern "C" {
#endif

#ifdef __cplusplus
typedef cv::Mat* Mat;
typedef cv::TermCriteria* TermCriteria;
typedef cv::RNG* RNG;
typedef std::vector<float>* floatVector;
typedef std::vector<cv::Point>* PointVector;
typedef std::vector<std::vector<cv::Point>>* PointsVector;
typedef std::vector<cv::Point2f>* Point2fVector;
typedef std::vector<std::vector<cv::Point2f>>* Points2fVector;
#else
typedef void* Mat;
typedef void* TermCriteria;
typedef void* RNG;
typedef void* PointVector;
typedef void* PointsVector;
typedef void* Point2fVector;
#endif

#include "minpack.h"

#define real __minpack_real__
#define COLUMN_MAJOR_ARRAYS

typedef struct {
  int m;
  real* y;
} fcndata_t;

void test();

void curve_fit(Mat h, Point2fVector obj, Point2fVector img);

void curve_fit_all(floatVector p_init, Point2fVector obj, Points2fVector img);

void unpack_p(floatVector p, Mat& K, Mat& k, Mat& E, int curIdx);

#ifdef __cplusplus
}
#endif
