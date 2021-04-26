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

typedef struct DoubleVector {
  double* val;
  int length;
} DoubleVector;

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
  double x;
  double y;
} Point2f;

// Wrapper for an individual cv::Point2d
typedef struct Point2d {
  double x;
  double y;
} Point2d;

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

// Wrapper for the vector of Point2d structs
// aka std::vector<Point2d>
typedef struct Points2d {
  Point2d* points;
  int length;
} Points2d;

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

// Wrapper for the vector of Point2dVector structs
// aka std::vector<std::vector<Point2d>>
typedef struct Points2dVectors {
  Points2d* vec;
  int length;
} Points2dVectors;

#ifdef __cplusplus
typedef cv::Mat* Mat;
typedef std::vector<cv::Point>* PointVector;
typedef std::vector<std::vector<cv::Point>>* PointVectorVector;
typedef std::vector<cv::Point2f>* Point2fVector;
typedef std::vector<std::vector<cv::Point2f>>* Point2fVectorVector;
typedef std::vector<cv::Point2d>* Point2dVector;
typedef std::vector<std::vector<cv::Point2d>>* Point2dVectorVector;
#else
typedef void* Mat;
typedef void* PointVector;
typedef void* PointVectorVector;
typedef void* Point2fVector;
typedef void* Point2fVectorVector;
typedef void* Point2dVector;
typedef void* Point2dVectorVector;
#endif

PointVector zPointVector_New();
Point2fVector zPoint2fVector_New();
Point2dVector zPoint2dVector_New();

PointVector zPointVector_NewFromPoints(Points points);
Point2fVector zPoint2fVector_NewFromPoints(Points2f points);
Point2dVector zPoint2dVector_NewFromPoints(Points2d points);

PointVectorVector zPointVectorVector_NewFromVector(PointsVectors vec);
Point2fVectorVector zPoint2fVectorVector_NewFromVector(Points2fVectors vec);
Point2dVectorVector zPoint2dVectorVector_NewFromVector(Points2dVectors vec);

int zPointVector_Size(PointVector pv);
int zPoint2fVector_Size(Point2fVector pv);
int zPoint2dVector_Size(Point2dVector pv);
int zPointVectorVector_Size(PointVectorVector pvv);
int zPoint2fVectorVector_Size(Point2fVectorVector pvv);
int zPoint2dVectorVector_Size(Point2dVectorVector pvv);

#include "minpack.h"

#define real __minpack_real__
#define COLUMN_MAJOR_ARRAYS

typedef struct {
  int m;
  real* y;
} fcndata_t;

DoubleVector curve_fit(DoubleVector elem, Point2dVector obj, Point2dVector img);
DoubleVector curve_fit_all(DoubleVector elem, Point2dVector obj,
                           Point2dVectorVector imgVec);

#ifdef __cplusplus
}
#endif
