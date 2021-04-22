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

// Wrapper for an individual cv::cvPoint
typedef struct Point {
  int x;
  int y;
} Point;

// Wrapper for an individual cv::Point2f
typedef struct Point2f {
  float x;
  float y;
} Point2f;

// Wrapper for the vector of Point structs
// aka std::vector<Point>
typedef struct Points {
  Point* points;
  int length;
} Points;

// Wrapper for the vector of Point2f structs
// aka std::vector<Point2f>
typedef struct Points2f {
  Point2f* points;
  int length;
} Points2f;

// Wrapper for the vector of PointVector structs
// aka std::vector<std::vector<Point>>
typedef struct PointsVectors {
  Points* vec;
  int length;
} PointsVectors;

// Wrapper for the vector of Point2fVector structs
// aka std::vector<std::vector<Point2f>>
typedef struct Points2fVectors {
  Points2f* vec;
  int length;
} Points2fVectors;

#ifdef __cplusplus
typedef cv::Mat* Mat;
typedef std::vector<cv::Point>* PointVector;
typedef std::vector<std::vector<cv::Point>>* PointVectorVector;
typedef std::vector<cv::Point2f>* Point2fVector;
typedef std::vector<std::vector<cv::Point2f>>* Point2fVectorVector;
#else
typedef void* Mat;
typedef void* PointVector;
typedef void* PointVectorVector;
typedef void* Point2fVector;
typedef void* Point2fVectorVector;
#endif

PointVector zPointVector_New();
Point2fVector zPoint2fVector_New();
PointVector zPointVector_NewFromPoints(Points points);
Point2fVector zPoint2fVector_NewFromPoints(Points2f points);
PointVectorVector zPointVectorVector_NewFromVector(PointsVectors vec);
Point2fVectorVector zPoint2fVectorVector_NewFromVector(Points2fVectors vec);

int zPointVector_Size(PointVector pv);
int zPoint2fVector_Size(Point2fVector pv);
int zPointVectorVector_Size(PointVectorVector pvv);
int zPoint2fVectorVector_Size(Point2fVectorVector pvv);

#include "minpack.h"

#define real __minpack_real__
#define COLUMN_MAJOR_ARRAYS

typedef struct {
  int m;
  real* y;
} fcndata_t;

FloatVector curve_fit(FloatVector elem, Point2fVector obj, Point2fVector img);
FloatVector curve_fit_all(FloatVector elem, Point2fVector obj,
                          Point2fVectorVector imgVec);

// void unpack_p(floatVector p, Mat& K, Mat& k, Mat& E, int curIdx);

#ifdef __cplusplus
}
#endif
