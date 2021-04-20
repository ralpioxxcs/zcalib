#include "lm.h"

#include <iostream>

using namespace std;
using namespace cv;

// homography opt
Point2fVector objPtr;
Point2fVector imgPtr;
// refine opt
floatVector ref_p;
Point2fVector ref_objPtr;
Points2fVector ref_imgPtr;

// void fcn(const int* m, const int* n, const real* x, real* fvec, real* fjac,
//         const int* ldjac, int* iflag);

// void fcn_all(const int* m, const int* n, const real* x, real* fvec, int*
// iflag);
void test() { std::cout << "test" << std::endl; }

void curve_fit_all(floatVector& p_init, Point2fVector obj, Points2fVector img) {
  ref_p = p_init;
  ref_objPtr = obj;
  ref_imgPtr = img;

  /**
   *  - m : 2*M*N rows
   *  - n : solution
   * */
  int m = 2 * (img->size()) * (obj->size());
  int n = p_init.size();
  int one = 1;
  real x[n];
  real fvec[m];
  real fjac[m * n];
  int ldjac = m;
  int maxfev, mode, nprint, nfev, njev;

  /*
      following starting values provide a rough fit
  */
  for (int i = 0; i < p_init.size(); i++) {
    x[i] = p_init[i];
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

  //__minpack_func__(lmdif)(&fcn_all, &m, &n, x, fvec, &ftol, &xtol, &gtol,
  //                        &maxfev, &epsfcn, diag, &mode, &factor, &nprint,
  //                        &info, &nfev, fjac, &ldjac, ipvt, qtf, wa1, wa2,
  //                        wa3, wa4);
  fnorm = __minpack_func__(enorm)(&m, fvec);
  ftol = __minpack_func__(dpmpar)(&one);
  covfac = fnorm * fnorm / (m - n);
  __minpack_func__(covar)(&n, fjac, &ldjac, ipvt, &ftol, wa1);

  int xSize = sizeof(x) / sizeof(x[0]);
  for (int i = 0; i < xSize; i++) {
    p_init[i] = x[i];
  }

  return;
}

void curve_fit(Mat h, Point2fVector obj, Point2fVector img) {
  int objSize = obj.size();
  int imgSize = img.size();

  objPtr = &obj;
  imgPtr = &img;

  int m = objSize * 2;
  int n = h.rows;
  int one = 1;
  real x[n];
  real fvec[m];
  real fjac[m * n];
  int ldjac = m;
  int maxfev, mode, nprint, nfev, njev;

  /*
      following starting values provide a rough fit
  */
  x[0] = h.at<float>(0, 0);
  x[1] = h.at<float>(0, 1);
  x[2] = h.at<float>(0, 2);
  x[3] = h.at<float>(0, 3);
  x[4] = h.at<float>(0, 4);
  x[5] = h.at<float>(0, 5);
  x[6] = h.at<float>(0, 6);
  x[7] = h.at<float>(0, 7);
  x[8] = h.at<float>(0, 8);

  real xtol = sqrt(__minpack_func__(dpmpar)(&one));
  real ftol = sqrt(__minpack_func__(dpmpar)(&one));
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

  //__minpack_func__(lmder)(&fcn, &m, &n, x, fvec, fjac, &ldjac, &ftol, &xtol,
  //                        &gtol, &maxfev, diag, &mode, &factor, &nprint,
  //                        &info, &nfev, &njev, ipvt, qtf, wa1, wa2, wa3, wa4);
  fnorm = __minpack_func__(enorm)(&m, fvec);
  ftol = __minpack_func__(dpmpar)(&one);
  covfac = fnorm * fnorm / (m - n);
  __minpack_func__(covar)(&n, fjac, &ldjac, ipvt, &ftol, wa1);

  Mat refined_matrix = Mat(3, 3, CV_64F);
  refined_matrix.at<float>(0, 0) = x[0];
  refined_matrix.at<float>(0, 1) = x[1];
  refined_matrix.at<float>(0, 2) = x[2];
  refined_matrix.at<float>(1, 0) = x[3];
  refined_matrix.at<float>(1, 1) = x[4];
  refined_matrix.at<float>(1, 2) = x[5];
  refined_matrix.at<float>(2, 0) = x[6];
  refined_matrix.at<float>(2, 1) = x[7];
  refined_matrix.at<float>(2, 2) = x[8];

  return;
}

// void fcn_all(const int* m, const int* n, const real* x, real* fvec,
//             int* iflag) {
//  // value function
//  // M loop
//  // refine f vector [VAL(X,h)]
//  auto homo_vec = util::to_homogeneous3d(*ref_objPtr);
//  for (int i = 0; i < ref_imgPtr->size(); i++) {
//    Mat K(3, 3, CV_64F);
//    Mat k(2, 1, CV_64F);
//    Mat E(3, 4, CV_64F);

//    // optimization::unpack_p(*ref_p, K, k, E, i);
//    floatVector xvec(*n);
//    for (int k = 0; k < xvec.size(); k++) {
//      xvec[k] = x[k];
//    }
//    unpack_p(xvec, K, k, E, i);

//    for (int j = 0; j < ref_objPtr->size() * 2; j += 2) {
//      auto proj = util::projection(homo_vec[j / 2], K, k, E);
//      float xx = ref_imgPtr->at(i).at(j / 2).x;
//      float yy = ref_imgPtr->at(i).at(j / 2).y;
//      fvec[(i * ref_objPtr->size() * 2) + j] = proj.x - xx;
//      fvec[(i * ref_objPtr->size() * 2) + j + 1] = proj.y - yy;
//    }
//  }
//  return;
//}

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
// void fcn(const int* m, const int* n, const real* x, real* fvec, real* fjac,
//         const int* ldjac, int* iflag) {
//  // * select order of matrix
//  //   -> column major OR row major
//  int i;
//#ifdef COLUMN_MAJOR_ARRAYS
//  if (*iflag == 1) {
//    // refine f vector [VAL(X,h)]
//    for (int i = 0; i < *m; i += 2) {
//      float cur_x = objPtr->at(i / 2).x;
//      float cur_y = objPtr->at(i / 2).y;

//      float w = (x[6] * cur_x + x[7] * cur_y + x[8]);
//      float u = (x[0] * cur_x + x[1] * cur_y + x[2]) / w;
//      float v = (x[3] * cur_x + x[4] * cur_y + x[5]) / w;

//      fvec[i] = u - imgPtr->at(i / 2).x;
//      fvec[i + 1] = v - imgPtr->at(i / 2).y;
//    }
//  } else {
//    // refine f jacobian [JAC(X,h)]
//    for (int i = 0; i < *m; i += 2) {
//      // (Xj,Yj)
//      float cur_x = objPtr->at(i / 2).x;
//      float cur_y = objPtr->at(i / 2).y;

//      // (Sx,Sy)
//      float s_x = (x[0] * cur_x + x[1] * cur_y + x[2]);
//      float s_y = (x[3] * cur_x + x[4] * cur_y + x[5]);
//      float w = (x[6] * cur_x + x[7] * cur_y + x[8]);
//      float w_2 = w * w;

//      // J(2j,*)
//      fjac[i + *ldjac * (1 - 1)] = cur_x / w;
//      fjac[i + *ldjac * (2 - 1)] = cur_y / w;
//      fjac[i + *ldjac * (3 - 1)] = 1.0 / w;
//      fjac[i + *ldjac * (4 - 1)] = 0.0;
//      fjac[i + *ldjac * (5 - 1)] = 0.0;
//      fjac[i + *ldjac * (6 - 1)] = 0.0;
//      fjac[i + *ldjac * (7 - 1)] = (-s_x * cur_x) / w_2;
//      fjac[i + *ldjac * (8 - 1)] = (-s_x * cur_y) / w_2;
//      fjac[i + *ldjac * (9 - 1)] = (-s_x) / w_2;

//      // J(2j+1,*)
//      fjac[i + 1 + *ldjac * (1 - 1)] = 0.0;
//      fjac[i + 1 + *ldjac * (2 - 1)] = 0.0;
//      fjac[i + 1 + *ldjac * (3 - 1)] = 0.0;
//      fjac[i + 1 + *ldjac * (4 - 1)] = cur_x / w;
//      fjac[i + 1 + *ldjac * (5 - 1)] = cur_y / w;
//      fjac[i + 1 + *ldjac * (6 - 1)] = 1.0 / w;
//      fjac[i + 1 + *ldjac * (7 - 1)] = (-s_y * cur_x) / w_2;
//      fjac[i + 1 + *ldjac * (8 - 1)] = (-s_y * cur_y) / w_2;
//      fjac[i + 1 + *ldjac * (9 - 1)] = (-s_y) / w_2;
//    }
//  }
//#elif ROW_MAJOR_ARRAYS
//  if (*iflag == 1) {
//    // refine f vector [VAL(X,h)]
//    for (int i = 0; i < *m; i += 2) {
//      float cur_x = objPtr->at(i / 2).x;
//      float cur_y = objPtr->at(i / 2).y;

//      float w = (x[6] * cur_x + x[7] * cur_y + x[8]);
//      float u = (x[0] * cur_x + x[1] * cur_y + x[2]) / w;
//      float v = (x[3] * cur_x + x[4] * cur_y + x[5]) / w;

//      fvec[i] = u - imgPtr->at(i / 2).x;
//      fvec[i + 1] = v - imgPtr->at(i / 2).y;
//    }
//  } else {
//    // refine f jacobian [JAC(X,h)]
//    for (int i = 0; i < *m; i += 2) {
//      // (Xj,Yj)
//      float cur_x = objPtr->at(i / 2).x;
//      float cur_y = objPtr->at(i / 2).y;

//      // (Sx,Sy)
//      float s_x = (x[0] * cur_x + x[1] * cur_y + x[2]);
//      float s_y = (x[3] * cur_x + x[4] * cur_y + x[5]);
//      float w = (x[6] * cur_x + x[7] * cur_y + x[8]);
//      float w_2 = w * w;

//      // J(2j,*)
//      fjac[(i * 9)] = cur_x / w;
//      fjac[(i * 9) + 1] = cur_y / w;
//      fjac[(i * 9) + 2] = 1 / w;
//      fjac[(i * 9) + 3] = 0.0;
//      fjac[(i * 9) + 4] = 0.0;
//      fjac[(i * 9) + 5] = 0.0;
//      fjac[(i * 9) + 6] = (-s_x * cur_x) / w_2;
//      fjac[(i * 9) + 7] = (-s_x * cur_y) / w_2;
//      fjac[(i * 9) + 8] = (-s_x) / w_2;

//      // J(2j+1,*)
//      fjac[(i * 9) + 9] = 0.0;
//      fjac[(i * 9) + 9 + 1] = 0.0;
//      fjac[(i * 9) + 9 + 2] = 0.0;
//      fjac[(i * 9) + 9 + 3] = cur_x / w;
//      fjac[(i * 9) + 9 + 4] = cur_y / w;
//      fjac[(i * 9) + 9 + 5] = 1 / w;
//      fjac[(i * 9) + 9 + 6] = (-s_y * cur_x) / w_2;
//      fjac[(i * 9) + 9 + 7] = (-s_y * cur_y) / w_2;
//      fjac[(i * 9) + 9 + 8] = (-s_y) / w_2;
//    }
//  }
//#endif
//  return;
//}

// void unpack_p(floatVector p, Mat& K, Mat& k, Mat& E,
//              int curIdx) {
//  // first 7 elements of p vector;
//  float alpha = p[0];
//  float beta = p[1];
//  float gamma = p[2];
//  float uc = p[3];
//  float vc = p[4];
//  float k1 = p[5];
//  float k2 = p[6];

//  float K_data[3 * 3] = {alpha, gamma, uc, 0.0, beta, vc, 0.0, 0.0, 1.0};
//  float k_data[2] = {k1, k2};

//  // deep copy
//  Mat K_(3, 3, CV_64F, K_data);
//  Mat k_(2, 1, CV_64F, k_data);
//  K = K_.clone();
//  k = k_.clone();

//  const int offset = 7;
//  const int size = 6;

//  float t_data[3] = {p[(curIdx * size) + offset + 0],
//                     p[(curIdx * size) + offset + 1],
//                     p[(curIdx * size) + offset + 2]};

//  float r_data[3] = {p[(curIdx * size) + offset + 3],
//                     p[(curIdx * size) + offset + 4],
//                     p[(curIdx * size) + offset + 5]};

//  Mat R;
//  Mat E_(3, 4, CV_64F);
//  cv::Rodrigues(Mat(3, 1, CV_64F, r_data), R);
//  Mat t(3, 1, CV_64F, t_data);
//  cv::hconcat(R, t, E_);
//  E = E_.clone();

//  return;
//}

