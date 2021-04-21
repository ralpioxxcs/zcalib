#include <stdbool.h>
#include <stdint.h>

typedef struct IntVector {
  int* val;
  int length;
} IntVector;

typedef struct FloatVector {
  float* val;
  int length;
} FloatVector;

#ifdef __cplusplus
#include <opencv2/opencv.hpp>
extern "C" {
#endif

// Wrapper for an individual cv::Point2f
typedef struct Point2f {
  float x;
  float y;
} Point2f;

// Wrapper for an individual cv::cvPoint
typedef struct Point {
  int x;
  int y;
} Point;

// Wrapper for the vector of Point structs aka std::vector<Point>
typedef struct Points {
  Point* points;
  int length;
} Points;

// Wrapper for the vector of Point2f structs aka std::vector<Point2f>
typedef struct Points2f {
  Point2f* points;
  int length;
} Points2f;

#ifdef __cplusplus
typedef cv::Mat* Mat;
typedef std::vector<cv::Point>* PointVector;
typedef std::vector<cv::Point2f>* Point2fVector;
#else
typedef void* Mat;
typedef void* PointVector;
typedef void* Point2fVector;
#endif

PointVector zPointVector_New();
Point2fVector zPoint2fVector_New();
PointVector zPointVector_NewFromPoints(Points points);
Point2fVector zPoint2fVector_NewFromPoints(Points2f points);
int zPointVector_Size(PointVector pv);
int zPoint2fVector_Size(Point2fVector pv);

#include "minpack.h"

#define real __minpack_real__
#define COLUMN_MAJOR_ARRAYS

typedef struct {
  int m;
  real* y;
} fcndata_t;

//struct IntVector test(struct IntVector elem);

//Point2fVector test_f(Point2fVector elem);

FloatVector curve_fit(FloatVector elem, Point2fVector obj, Point2fVector img);

// void curve_fit_all(floatVector p_init, Point2fVector obj, Points2fVector
// img);

// void unpack_p(floatVector p, Mat& K, Mat& k, Mat& E, int curIdx);

// void curve_fit(Mat h, Points2dVector obj, Points2dVector img);

// void curve_fit_all(floatVector p_init, Point2fVector obj, Points2fVector
// img);

// void unpack_p(floatVector p, Mat& K, Mat& k, Mat& E, int curIdx);

#ifdef __cplusplus
}
#endif
