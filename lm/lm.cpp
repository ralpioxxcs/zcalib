#include "lm.h"

#include <math.h>

PointVector zPointVector_New() { return new std::vector<cv::Point>; }

Point2fVector zPoint2fVector_New() { return new std::vector<cv::Point2f>; }

PointVector zPointVector_NewFromPoints(Points points) {
  std::vector<cv::Point>* cntr = new std::vector<cv::Point>;
  for (size_t i = 0; i < points.length; i++) {
    cntr->push_back(cv::Point(points.points[i].x, points.points[i].y));
  }
  return cntr;
}

Point2fVector zPoint2fVector_NewFromPoints(Points2f points) {
  std::cout << "dsds" << std::endl;
  std::vector<cv::Point2f>* cntr = new std::vector<cv::Point2f>;
  for (size_t i = 0; i < points.length; i++) {
    cntr->push_back(cv::Point2f(points.points[i].x, points.points[i].y));
  }
  return cntr;
}

PointVectorVector zPointVectorVector_NewFromVector(PointsVectors vec) {
  std::vector<std::vector<cv::Point>>* cntr =
      new std::vector<std::vector<cv::Point>>;
  for (size_t i = 0; i < vec.length; i++) {
    std::vector<cv::Point>* tv = new std::vector<cv::Point>;
    for (size_t j = 0; j < vec.vec[i].length; j++) {
      tv->push_back(cv::Point(vec.vec[i].points[j].x, vec.vec[i].points[j].y));
    }
    cntr->push_back(*tv);
  }
  return cntr;
}

Point2fVectorVector zPoint2fVectorVector_NewFromVector(Points2fVectors vec) {
  std::cout << "t" << std::endl;
  std::vector<std::vector<cv::Point2f>>* cntr =
      new std::vector<std::vector<cv::Point2f>>;
  std::cout << "outer length : " << vec.length << std::endl;
  for (size_t i = 0; i < vec.length; i++) {
    std::vector<cv::Point2f>* tv = new std::vector<cv::Point2f>;
    std::cout << "inner length : " << vec.vec[i].length << std::endl;
    for (size_t j = 0; j < vec.vec[i].length; j++) {
      tv->push_back(
          cv::Point2f(vec.vec[i].points[j].x, vec.vec[i].points[j].y));
    }
    cntr->push_back(*tv);
  }
  return cntr;
}

int zPointVector_Size(PointVector pv) { return pv->size(); }

int zPoint2fVector_Size(Point2fVector pv) { return pv->size(); }

int zPointVectorVector_Size(PointVectorVector pvv) { return pvv->size(); }

int zPoint2fVectorVector_Size(Point2fVectorVector pvv) { return pvv->size(); }

//---------------------------------------------------------------------
//
void fcn(const int* m, const int* n, const real* x, real* fvec, real* fjac,
         const int* ldjac, int* iflag);
void fcn_all(const int* m, const int* n, const real* x, real* fvec, int* iflag);

// homography opt
Point2fVector g_objPtr;
Point2fVector g_imgPtr;
FloatVector curve_fit(FloatVector elem, Point2fVector obj, Point2fVector img) {
  const int objSize = obj->size();
  const int imgSize = img->size();

  g_objPtr = obj;
  g_imgPtr = img;

  const int m = objSize * 2;
  const int n = elem.length;
  int one = 1;
  real x[n];
  real fvec[m];
  real fjac[m * n];
  int ldjac = m;
  int maxfev, mode, nprint, nfev, njev;

  // following starting values provide a rough fit
  for (int i = 0; i < elem.length; i++) {
    x[i] = elem.val[i];
  }

  real xtol = std::sqrt(__minpack_func__(dpmpar)(&one));
  real ftol = std::sqrt(__minpack_func__(dpmpar)(&one));
  real gtol = 0.0;

  real fnorm, factor;
  int ipvt[n];
  real diag[n], qtf[n];
  real wa1[n], wa2[n], wa3[n], wa4[m];
  real covfac;
  int lwa = 2000;
  int info;

  maxfev = 100 * (n + 1);
  mode = 1;
  factor = 100;
  nprint = 0;

  __minpack_func__(lmder)(&fcn, &m, &n, x, fvec, fjac, &ldjac, &ftol, &xtol,
                          &gtol, &maxfev, diag, &mode, &factor, &nprint, &info,
                          &nfev, &njev, ipvt, qtf, wa1, wa2, wa3, wa4);
  fnorm = __minpack_func__(enorm)(&m, fvec);

  ftol = __minpack_func__(dpmpar)(&one);
  covfac = fnorm * fnorm / (m - n);
  __minpack_func__(covar)(&n, fjac, &ldjac, ipvt, &ftol, wa1);

  FloatVector refined_vector;
  refined_vector.length = 3 * 3;
  refined_vector.val = new float[refined_vector.length];
  for (auto i = 0; i < refined_vector.length; i++) {
    refined_vector.val[i] = x[i];
  }
  return refined_vector;
}

// refine opt
FloatVector g_ref_p;
Point2fVector g_ref_objPtr;
Point2fVectorVector g_ref_imgVecPtr;
FloatVector curve_fit_all(FloatVector elem, Point2fVector obj,
                          Point2fVectorVector imgVec) {
  const int objSize = obj->size();
  const int imgVecSize = imgVec->size();

  g_ref_p = elem;
  g_ref_objPtr = obj;
  g_ref_imgVecPtr = imgVec;

  //- m : 2*M*N rows
  //- n : solution

  int m = 2 * (imgVecSize) * (objSize);
  int n = elem.length;
  int one = 1;
  real x[n];
  real fvec[m];
  real fjac[m * n];
  int ldjac = m;
  int maxfev, mode, nprint, nfev, njev;

  // following starting values provide a rough fit
  for (int i = 0; i < elem.length; i++) {
    x[i] = elem.val[i];
  }
  real xtol = sqrt(__minpack_func__(dpmpar)(&one));
  real ftol = sqrt(__minpack_func__(dpmpar)(&one));
  real gtol = 0.0;
  real epsfcn = 2.2204460492503131e-16;

  real fnorm, factor;
  int ipvt[n];
  real diag[n], qtf[n];
  real wa1[n], wa2[n], wa3[n], wa4[m];
  real covfac;
  int lwa = 2000;
  int info;

  maxfev = 200 * (n + 1);
  mode = 1;
  factor = 100;
  nprint = 0;

  __minpack_func__(lmdif)(&fcn_all, &m, &n, x, fvec, &ftol, &xtol, &gtol,
                          &maxfev, &epsfcn, diag, &mode, &factor, &nprint,
                          &info, &nfev, fjac, &ldjac, ipvt, qtf, wa1, wa2, wa3,
                          wa4);
  fnorm = __minpack_func__(enorm)(&m, fvec);

  ftol = __minpack_func__(dpmpar)(&one);
  covfac = fnorm * fnorm / (m - n);
  __minpack_func__(covar)(&n, fjac, &ldjac, ipvt, &ftol, wa1);

  int xSize = sizeof(x) / sizeof(x[0]);
  FloatVector refined_vector;
  refined_vector.length = xSize;
  refined_vector.val = new float[refined_vector.length];
  for (auto i = 0; i < refined_vector.length; i++) {
    refined_vector.val[i] = x[i];
  }

  return refined_vector;
}

void unpack_p(std::vector<double> p, cv::Mat& K, cv::Mat& k, cv::Mat& E,
              int curIdx) {
  // first 7 elements of p vector;
  // double alpha = p[0];
  // double beta = p[1];
  // double gamma = p[2];
  // double uc = p[3];
  // double vc = p[4];
  // double k1 = p[5];
  // double k2 = p[6];

  // double K_data[3 * 3] = {alpha, gamma, uc, 0.0, beta, vc, 0.0, 0.0, 1.0};
  // double k_data[2] = {k1, k2};

  //// deep copy
  // cv::Mat K_(3, 3, CV_64F, K_data);
  // cv::Mat k_(2, 1, CV_64F, k_data);
  // K = K_.clone();
  // k = k_.clone();

  // const int offset = 7;
  // const int size = 6;

  // double t_data[3] = {p[(curIdx * size) + offset + 0],
  //                    p[(curIdx * size) + offset + 1],
  //                    p[(curIdx * size) + offset + 2]};

  // double r_data[3] = {p[(curIdx * size) + offset + 3],
  //                    p[(curIdx * size) + offset + 4],
  //                    p[(curIdx * size) + offset + 5]};

  // Mat R;
  // cv::Mat E_(3, 4, CV_64F);
  // cv::Rodrigues(cv::Mat(3, 1, CV_64F, r_data), R);
  // Mat t(3, 1, CV_64F, t_data);
  // cv::hconcat(R, t, E_);
  // E = E_.clone();

  return;
}

/**
 *
 * @function: fcn
 * @brief : 비선형 최소자승법 서브루틴 함수
 * @param :
 *   -m  : positive integer input variable set to number of functions
 *   -n  : positive integer input variable set to number of variables
 *   -x : array of lenth, final estimates of the solution vector
 *   -fvec : output array of length "m" which contains the functions evaluated
 *   -fjac : output "m" by "n" array
 *   -ldjac : positive integer variable not less than m which specifies dims
 *of fjac -iflag : user customed flag
 **/
void fcn(const int* m, const int* n, const real* x, real* fvec, real* fjac,
         const int* ldjac, int* iflag) {
  // # Select order of matrix -> column major OR row major
  int i;
#ifdef COLUMN_MAJOR_ARRAYS
  if (*iflag == 1) {
    // refine f vector [VAL(X,h)]
    for (int i = 0; i < *m; i += 2) {
      double cur_x = g_objPtr->at(i / 2).x;
      double cur_y = g_objPtr->at(i / 2).y;

      double w = (x[6] * cur_x + x[7] * cur_y + x[8]);
      double u = (x[0] * cur_x + x[1] * cur_y + x[2]) / w;
      double v = (x[3] * cur_x + x[4] * cur_y + x[5]) / w;

      fvec[i] = u - g_imgPtr->at(i / 2).x;
      fvec[i + 1] = v - g_imgPtr->at(i / 2).y;
    }
  } else {
    // refine f jacobian [JAC(X,h)]
    for (int i = 0; i < *m; i += 2) {
      // (Xj,Yj)
      double cur_x = g_objPtr->at(i / 2).x;
      double cur_y = g_objPtr->at(i / 2).y;

      // (Sx,Sy)
      double s_x = (x[0] * cur_x + x[1] * cur_y + x[2]);
      double s_y = (x[3] * cur_x + x[4] * cur_y + x[5]);
      double w = (x[6] * cur_x + x[7] * cur_y + x[8]);
      double w_2 = w * w;

      // J(2j,*)
      fjac[i + *ldjac * (1 - 1)] = cur_x / w;
      fjac[i + *ldjac * (2 - 1)] = cur_y / w;
      fjac[i + *ldjac * (3 - 1)] = 1.0 / w;
      fjac[i + *ldjac * (4 - 1)] = 0.0;
      fjac[i + *ldjac * (5 - 1)] = 0.0;
      fjac[i + *ldjac * (6 - 1)] = 0.0;
      fjac[i + *ldjac * (7 - 1)] = (-s_x * cur_x) / w_2;
      fjac[i + *ldjac * (8 - 1)] = (-s_x * cur_y) / w_2;
      fjac[i + *ldjac * (9 - 1)] = (-s_x) / w_2;

      // J(2j+1,*)
      fjac[i + 1 + *ldjac * (1 - 1)] = 0.0;
      fjac[i + 1 + *ldjac * (2 - 1)] = 0.0;
      fjac[i + 1 + *ldjac * (3 - 1)] = 0.0;
      fjac[i + 1 + *ldjac * (4 - 1)] = cur_x / w;
      fjac[i + 1 + *ldjac * (5 - 1)] = cur_y / w;
      fjac[i + 1 + *ldjac * (6 - 1)] = 1.0 / w;
      fjac[i + 1 + *ldjac * (7 - 1)] = (-s_y * cur_x) / w_2;
      fjac[i + 1 + *ldjac * (8 - 1)] = (-s_y * cur_y) / w_2;
      fjac[i + 1 + *ldjac * (9 - 1)] = (-s_y) / w_2;
    }
  }
#elif ROW_MAJOR_ARRAYS
  if (*iflag == 1) {
    // refine f vector [VAL(X,h)]
    for (int i = 0; i < *m; i += 2) {
      double cur_x = g_objPtr->at(i / 2).x;
      double cur_y = g_objPtr->at(i / 2).y;

      double w = (x[6] * cur_x + x[7] * cur_y + x[8]);
      double u = (x[0] * cur_x + x[1] * cur_y + x[2]) / w;
      double v = (x[3] * cur_x + x[4] * cur_y + x[5]) / w;

      fvec[i] = u - imgPtr->at(i / 2).x;
      fvec[i + 1] = v - imgPtr->at(i / 2).y;
    }
  } else {
    // refine f jacobian [JAC(X,h)]
    for (int i = 0; i < *m; i += 2) {
      // (Xj,Yj)
      double cur_x = g_objPtr->at(i / 2).x;
      double cur_y = g_objPtr->at(i / 2).y;

      // (Sx,Sy)
      double s_x = (x[0] * cur_x + x[1] * cur_y + x[2]);
      double s_y = (x[3] * cur_x + x[4] * cur_y + x[5]);
      double w = (x[6] * cur_x + x[7] * cur_y + x[8]);
      double w_2 = w * w;

      // J(2j,*)
      fjac[(i * 9)] = cur_x / w;
      fjac[(i * 9) + 1] = cur_y / w;
      fjac[(i * 9) + 2] = 1 / w;
      fjac[(i * 9) + 3] = 0.0;
      fjac[(i * 9) + 4] = 0.0;
      fjac[(i * 9) + 5] = 0.0;
      fjac[(i * 9) + 6] = (-s_x * cur_x) / w_2;
      fjac[(i * 9) + 7] = (-s_x * cur_y) / w_2;
      fjac[(i * 9) + 8] = (-s_x) / w_2;

      // J(2j+1,*)
      fjac[(i * 9) + 9] = 0.0;
      fjac[(i * 9) + 9 + 1] = 0.0;
      fjac[(i * 9) + 9 + 2] = 0.0;
      fjac[(i * 9) + 9 + 3] = cur_x / w;
      fjac[(i * 9) + 9 + 4] = cur_y / w;
      fjac[(i * 9) + 9 + 5] = 1 / w;
      fjac[(i * 9) + 9 + 6] = (-s_y * cur_x) / w_2;
      fjac[(i * 9) + 9 + 7] = (-s_y * cur_y) / w_2;
      fjac[(i * 9) + 9 + 8] = (-s_y) / w_2;
    }
  }
#endif
  return;
}

cv::Point2f distort(cv::Point2f pt, cv::Mat k) {
  cv::Point2f ret;

  float x = pt.x;
  float y = pt.y;

  float r = sqrt(x * x + y * y);
  float k0 = k.at<float>(0, 0);
  float k1 = k.at<float>(1, 0);

  // k0=-k0;
  // k1=-k1;
  float d = k0 * (r * r) + k1 * (r * r * r * r);

  ret.x = x * (1. + d);
  ret.y = y * (1. + d);

  return ret;
}

std::vector<cv::Vec<float, 4>> to_homogeneous3d(Point2fVector vec) {
  size_t sz = vec->size();
  std::vector<cv::Vec<float, 4>> homoVec(sz);
  for (int i = 0; i < sz; i++) {
    homoVec[i][0] = static_cast<float>(vec->at(i).x);
    homoVec[i][1] = static_cast<float>(vec->at(i).y);
    homoVec[i][2] = 0;
    homoVec[i][3] = 1.0;
  }
  return homoVec;
}

cv::Point2f projection(cv::Vec<float, 4> pt, cv::Mat K, cv::Mat k, cv::Mat E) {
  cv::Point2f ret;

  cv::Mat homoMat(pt, true);
  cv::Mat dst = E * homoMat;

  cv::Point3f pts;
  pts = cv::Point3f(dst.at<float>(0, 0), dst.at<float>(0, 1),
                    dst.at<float>(0, 2));
  cv::Point2f inhomoPnt(pts.x / pts.z, pts.y / pts.z);

  // apply distortion
  cv::Point2f distortedPnt;
  if (!k.empty()) {
    distortedPnt = distort(inhomoPnt, k);
  } else {
    distortedPnt = inhomoPnt;
  }

  cv::Point3f homoDistortedPnt;
  homoDistortedPnt.x = distortedPnt.x;
  homoDistortedPnt.y = distortedPnt.y;
  homoDistortedPnt.z = 1.0;

  cv::Mat homoDistortedMat(1, 3, CV_64F);
  homoDistortedMat.at<float>(0, 0) = homoDistortedPnt.x;
  homoDistortedMat.at<float>(0, 1) = homoDistortedPnt.y;
  homoDistortedMat.at<float>(0, 2) = homoDistortedPnt.z;

  cv::Mat KK;
  cv::transpose(K, KK);
  cv::Mat K_t(3, 2, CV_64F);
  K_t.at<float>(0, 0) = KK.at<float>(0, 0);
  K_t.at<float>(1, 0) = KK.at<float>(1, 0);
  K_t.at<float>(2, 0) = KK.at<float>(2, 0);
  K_t.at<float>(0, 1) = KK.at<float>(0, 1);
  K_t.at<float>(1, 1) = KK.at<float>(1, 1);
  K_t.at<float>(2, 1) = KK.at<float>(2, 1);

  cv::Mat dst_proj = homoDistortedMat * K_t;

  ret.x = dst_proj.at<float>(0, 0);
  ret.y = dst_proj.at<float>(0, 1);

  return ret;
}

void fcn_all(const int* m, const int* n, const real* x, real* fvec,
             int* iflag) {
  // value function
  // M loop
  // refine f vector [VAL(X,h)]
  auto homo_vec = to_homogeneous3d(g_ref_objPtr);
  for (int i = 0; i < g_ref_imgVecPtr->size(); i++) {
    cv::Mat K(3, 3, CV_64F);
    cv::Mat k(2, 1, CV_64F);
    cv::Mat E(3, 4, CV_64F);

    std::vector<double> xvec(*n);
    for (int k = 0; k < xvec.size(); k++) {
      xvec[k] = x[k];
    }
    unpack_p(xvec, K, k, E, i);

    for (int j = 0; j < g_ref_objPtr->size() * 2; j += 2) {
      auto proj = projection(homo_vec[j / 2], K, k, E);
      double xx = g_ref_imgVecPtr->at(i).at(j / 2).x;
      double yy = g_ref_imgVecPtr->at(i).at(j / 2).y;
      fvec[(i * g_ref_objPtr->size() * 2) + j] = proj.x - xx;
      fvec[(i * g_ref_objPtr->size() * 2) + j + 1] = proj.y - yy;
    }
  }
  return;
}

