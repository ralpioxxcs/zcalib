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
  std::vector<cv::Point2f>* cntr = new std::vector<cv::Point2f>;

  for (size_t i = 0; i < points.length; i++) {
    cntr->push_back(cv::Point2f(points.points[i].x, points.points[i].y));
  }

  return cntr;
}

int zPointVector_Size(PointVector pv) { return pv->size(); }

int zPoint2fVector_Size(Point2fVector pv) { return pv->size(); }

//---------------------------------------------------------------------
//
void fcn(const int* m, const int* n, const real* x, real* fvec, real* fjac,
         const int* ldjac, int* iflag);

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
//    std::vector<double> xvec(*n);
//    for (int k = 0; k < xvec.size(); k++) {
//      xvec[k] = x[k];
//    }
//    optimization::unpack_p(xvec, K, k, E, i);

//    for (int j = 0; j < ref_objPtr->size() * 2; j += 2) {
//      auto proj = util::projection(homo_vec[j / 2], K, k, E);
//      double xx = ref_imgPtr->at(i).at(j / 2).x;
//      double yy = ref_imgPtr->at(i).at(j / 2).y;
//      fvec[(i * ref_objPtr->size() * 2) + j] = proj.x - xx;
//      fvec[(i * ref_objPtr->size() * 2) + j + 1] = proj.y - yy;
//    }
//    // APR_LOGGER(GET_LOGGER, DEBUG) << "i:" << i;
//  }
//  // APR_LOGGER(GET_LOGGER, DEBUG) << "trace";
//  return;
//}

// struct IntVector test(struct IntVector elem) {
//  std::vector<int> test;

//  test.push_back(1);
//  test.push_back(3);
//  test.push_back(5);

//  IntVector ret;
//  ret.length = test.size();
//  ret.val = new int[ret.length];
//  for (int i = 0; i < ret.length; ++i) {
//    ret.val[i] = test[i];
//  }

//  return ret;
//}

// Point2fVector test_f(Point2fVector elem) {
//  std::vector<cv::Point2f>* test = new std::vector<cv::Point2f>;

//  test->emplace_back(cv::Point2f(1.1, 2.2));
//  test->emplace_back(cv::Point2f(5.1, 7.2));

//  for (const auto& v : *elem) {
//    test->emplace_back(v);
//  }

//  for (const auto& v : *test) {
//    std::cout << "test : " << v<< std::endl;
//  }

//  return test;
//}

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

  //std::cout << "final l2 norm of the residulas : " << (double)fnorm
  //          << std::endl;
  //std::cout << "number of function evaluations :" << nfev << std::endl;
  //std::cout << "number of Jacobian evaluations :" << njev << std::endl;
  //std::cout << "exit parameter : " << info << std::endl;
  //std::cout << "[final approximate solution]" << std::endl;
  //for (int j = 1; j <= n; j++) {
  //  std::cout << "[" << j - 1 << "] :" << (double)x[j - 1] << std::endl;
  //}

  FloatVector refined_vector;
  refined_vector.length = 3 * 3;
  refined_vector.val = new float[refined_vector.length];
  for (auto i = 0; i < refined_vector.length; i++) {
    refined_vector.val[i] = x[i];
  }
  return refined_vector;
}

// void curve_fit_all(std::vector<double>& p_init, std::vector<cv::Point2d> obj,
//                   std::vector<std::vector<cv::Point2d>> img) {
//  ref_p = &p_init;
//  ref_objPtr = &obj;
//  ref_imgPtr = &img;

/**
 *  - m : 2*M*N rows
 *  - n : solution
 * */
//  int m = 2 * (img.size()) * (obj.size());
//  int n = p_init.size();
//  int one = 1;
//  real x[n];
//  real fvec[m];
//  real fjac[m * n];
//  int ldjac = m;
//  int maxfev, mode, nprint, nfev, njev;

//  [>
//      following starting values provide a rough fit
//  */
//  for (int i = 0; i < p_init.size(); i++) {
//    x[i] = p_init[i];
//  }
//  real xtol = sqrt(__minpack_func__(dpmpar)(&one));
//  real ftol = sqrt(__minpack_func__(dpmpar)(&one));
//  real gtol = 0.0;
//  real epsfcn = 2.2204460492503131e-16;

//  real fnorm, factor;
//  int ipvt[n];
//  real diag[n], qtf[n];
//  real wa1[n], wa2[n], wa3[n], wa4[m];
//  real covfac;
//  int lwa = 2000;
//  int info;

//  maxfev = 200 * (n + 1);
//  mode = 1;
//  factor = 100;
//  nprint = 0;

/**
 *  [ EXCEPTION ]
 **/
//  // BOOST_ASSERT_MSG(m > n, "improper input: n>m");

//  __minpack_func__(lmdif)(&fcn_all, &m, &n, x, fvec, &ftol, &xtol, &gtol,
//                          &maxfev, &epsfcn, diag, &mode, &factor, &nprint,
//                          &info, &nfev, fjac, &ldjac, ipvt, qtf, wa1, wa2,
//                          wa3, wa4);
//  fnorm = __minpack_func__(enorm)(&m, fvec);

//  // APR_LOGGER(GET_LOGGER, DEBUG)
//  //<< "final l2 norm of the residulas : " << (double)fnorm;
//  // APR_LOGGER(GET_LOGGER, DEBUG) << "number of function evaluations :" <<
//  // nfev; APR_LOGGER(GET_LOGGER, DEBUG) << "exit parameter : " << info;
//  // APR_LOGGER(GET_LOGGER, DEBUG) << "[final approximate solution]";
//  // for (int j = 1; j <= n; j++) {
//  //   APR_LOGGER(GET_LOGGER, DEBUG) << "[" << j - 1 << "] :" << (double)x[j -
//  //   1];
//  // }

//  ftol = __minpack_func__(dpmpar)(&one);
//  covfac = fnorm * fnorm / (m - n);
//  __minpack_func__(covar)(&n, fjac, &ldjac, ipvt, &ftol, wa1);
//  // APR_LOGGER(GET_LOGGER, DEBUG) << "[covarinace]";
//  // for (int i = 1; i <= n; i++) {
//  //   for (int j = 1; j <= n; j++) {
//  //     APR_LOGGER(GET_LOGGER, DEBUG)
//  //         << "[" << i - 1 << "][" << j - 1 << "]"
//  //         << (double)(fjac[(i - 1) * ldjac + j - 1] * covfac);
//  //   }
//  // }

//  int xSize = sizeof(x) / sizeof(x[0]);
//  // BOOST_ASSERT_MSG(xSize == p_init.size(), "solution vector size is
//  // improper");
//  for (int i = 0; i < xSize; i++) {
//    p_init[i] = x[i];
//  }

//  return;
//}

// void unpack_p(std::vector<double> p, cv::Mat& K, cv::Mat& k, cv::Mat& E,
//              int curIdx) {
//  // first 7 elements of p vector;
//  double alpha = p[0];
//  double beta = p[1];
//  double gamma = p[2];
//  double uc = p[3];
//  double vc = p[4];
//  double k1 = p[5];
//  double k2 = p[6];

//  double K_data[3 * 3] = {alpha, gamma, uc, 0.0, beta, vc, 0.0, 0.0, 1.0};
//  double k_data[2] = {k1, k2};

//  // deep copy
//  cv::Mat K_(3, 3, CV_64F, K_data);
//  cv::Mat k_(2, 1, CV_64F, k_data);
//  K = K_.clone();
//  k = k_.clone();

//  const int offset = 7;
//  const int size = 6;

//  double t_data[3] = {p[(curIdx * size) + offset + 0],
//                      p[(curIdx * size) + offset + 1],
//                      p[(curIdx * size) + offset + 2]};

//  double r_data[3] = {p[(curIdx * size) + offset + 3],
//                      p[(curIdx * size) + offset + 4],
//                      p[(curIdx * size) + offset + 5]};

//  Mat R;
//  cv::Mat E_(3, 4, CV_64F);
//  cv::Rodrigues(cv::Mat(3, 1, CV_64F, r_data), R);
//  Mat t(3, 1, CV_64F, t_data);
//  cv::hconcat(R, t, E_);
//  E = E_.clone();

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
