#ifdef __cplusplus
extern "C" {
#endif

#include <minpack.h>
#include <opencv4/opencv2/core.hpp>

#define real __minpack_real_
#define COLUMN_MAJOR_ARRAYS

typedef struct {
  int m;
  real* y;
} fcndata_t;

void print_test();

// homography parameter optimize
void curve_fit(cv::Mat h, std::vector<cv::Point_<double>> obj,
               std::vector<cv::Point_<double>> img);

// all parameter optimize
void curve_fit_all(std::vector<double>& p_init,
                   std::vector<cv::Point_<double>> obj,
                   std::vector<std::vector<cv::Point_<double>>> img);

static void unpack_p(std::vector<double> p, cv::Mat& K, cv::Mat& k, cv::Mat& E,
                     int curIdx);

void print();

#ifdef __cplusplus
}
#endif

