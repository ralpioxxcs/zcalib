package zcalib

import (
	"gonum.org/v1/gonum/mat"
)

func solveE(H mat.Dense, K mat.Dense) *mat.Dense {
	K_inv := mat.NewDense(3, 3, nil)
	K_inv.Inverse(&K)
	h_col0 := K.ColView(0)
	h_col1 := K.ColView(1)
	h_col2 := K.ColView(2)

	temMat := mat.NewDense(3, 1, nil)
	temMat.Mul(K_inv, h_col0)
	lambda := mat.Norm(temMat, 2)

	var kh0, kh1, kh2 mat.Dense
	kh0.Mul(K_inv, h_col0)
	kh1.Mul(K_inv, h_col1)
	kh2.Mul(K_inv, h_col2)

	//mat.
	//lambda := 1 / (mat.
	//lambda := 1 / (K_inv.Mul)

	return mat.NewDense(3, 3, nil)
}

//Mat extrinsic::solveE(const Mat H, const Mat K) {
//  BOOST_ASSERT_MSG(K.cols == 3 && H.cols == 3, "input matrix column error");
//  BOOST_ASSERT_MSG(K.rows == 3 && H.rows == 3, "input matrix row error");

//  Mat K_inv = K.inv();
//  Mat h_col0 = H.col(0);
//  Mat h_col1 = H.col(1);
//  Mat h_col2 = H.col(2);

//  double lambda = 1 / cv::norm((K_inv * h_col0), cv::NORM_L2);

//  Mat r0 = lambda * K_inv * h_col0;
//  Mat r1 = lambda * K_inv * h_col1;
//  Mat r2 = r0.cross(r1);

//  Mat t = lambda * K_inv * h_col2;

//  Mat rStack;
//  hconcat(r0, r1, rStack);
//  hconcat(rStack, r2, rStack);

//  // APR_LOGGER(GET_LOGGER, DEBUG) << "stacked R :\n" << rStack;

//  // Reorthogonalize rotation matrix
//  // -> again based on singular-value decomposition
//  Mat U, Sigma, V_t;
//  SVD::compute(rStack, Sigma, U, V_t, SVD::FULL_UV);
//  Mat R = U * V_t;

//  // APR_LOGGER(GET_LOGGER, DEBUG) << "reorthogonalize matrix R :\n" << R;
//  // APR_LOGGER(GET_LOGGER, DEBUG) << "translation matrix T :\n" << t;

//  Mat E;
//  hconcat(R, t, E);

//  return E;
//}
