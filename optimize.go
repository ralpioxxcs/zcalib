package zcalib

import (
	cv "gocv.io/x/gocv"
)

func RefineAll(
	imgVec []cv.Point2fVector,
	obj cv.Point2fVector,
	K cv.Mat,
	extrinsics []cv.Mat,
	k1, k2 float32) (cv.Mat, []cv.Mat, float32, float32) {

	// compose_Pvec returns united vector composist of intrisic, extrinsic values
	p_init := func() []float32 {
		alpha := K.GetFloatAt(0, 0)
		beta := K.GetFloatAt(1, 1)
		gamma := K.GetFloatAt(0, 1)
		uc := K.GetFloatAt(0, 2)
		vc := K.GetFloatAt(1, 2)

		M := len(imgVec)

		pvec := make([]float32, 7)
		extVec := make([]float32, M*6)
		pvec[0] = alpha
		pvec[1] = beta
		pvec[2] = gamma
		pvec[3] = uc
		pvec[4] = vc
		pvec[5] = k1
		pvec[6] = k2

		// loop for view count ( 1, 2, 3, ... M-1, M )
		const offset = 6
		for i := 0; i < len(extVec); i += offset {
			// Decompose to rotation matrix
			R := cv.NewMatWithSize(3, 3, cv.MatTypeCV32F)
			r1 := R.RowRange(0, 1)
			r2 := R.RowRange(1, 2)
			r3 := R.RowRange(2, 3)

			ptrdst, _ := r1.DataPtrFloat32()
			extR1 := extrinsics[i/offset].RowRange(0, 1)
			ptrsrc, _ := extR1.DataPtrFloat32()
			copy(ptrdst, ptrsrc)

			ptrdst, _ = r2.DataPtrFloat32()
			extR2 := extrinsics[i/offset].RowRange(1, 2)
			ptrsrc, _ = extR2.DataPtrFloat32()
			copy(ptrdst, ptrsrc)

			ptrdst, _ = r3.DataPtrFloat32()
			extR3 := extrinsics[i/offset].RowRange(2, 3)
			ptrsrc, _ = extR3.DataPtrFloat32()
			copy(ptrdst, ptrsrc)

			// Convert extrinsics to Rodrigues form matrix
			Rod := toRodrigues(R)

			// translation
			extVec[i+0] = extrinsics[i/offset].GetFloatAt(0, 3)
			extVec[i+1] = extrinsics[i/offset].GetFloatAt(1, 3)
			extVec[i+2] = extrinsics[i/offset].GetFloatAt(2, 3)
			// rotation (rodrigues)a
			extVec[i+3] = Rod[0]
			extVec[i+4] = Rod[1]
			extVec[i+5] = Rod[2]
		}

		for _, v := range extVec {
			pvec = append(pvec, v)
		}
		return pvec
	}()

	logger.Infof("p_init size : %v", len(p_init))

	/** [최소자승 input data 설정]
	 *   -> M : 캘리브레이션 지그 촬영 갯수
	 *   -> N : 캘리브레이션 지그 포인트 갯수
	 *   -> X : M 뷰의 갯수를 갖는 캘리브레이션 지그 3D 좌표
	 *   -> Y : 코너점 측정 u,v좌표
	 **/
	M := len(imgVec)
	N := obj.Size()

	Xvec := make([]cv.Point2f, N)
	Yvec := make([][]cv.Point2f, M)

	for i, v := range obj.ToPoints() {
		Xvec[i] = v
		for j := 0; j < M; j++ {
			Yvec[i] = make([]cv.Point2f, len(imgVec[i].ToPoints()))
			for k, v2 := range imgVec[i].ToPoints() {
				Yvec[i][k] = v2
			}
		}
	}
	//lm.CurveFitting()

	// Decompose vector to matrix form
	return (func(p []float32) (cv.Mat, []cv.Mat, float32, float32) {

		//APR_LOGGER(GET_LOGGER, DEBUG) << "p size :" << p.size();

		//BOOST_ASSERT_MSG(m_intrinsic.cols == 3 && m_intrinsic.rows == 3,
		//                 "intrinsic matrix size is not initialized");
		//BOOST_ASSERT_MSG(m_extrinsic.size() != 0,
		//                 "extrinsic matrix vector size error");
		//BOOST_ASSERT_MSG(m_extrinsic[0].cols == 4 && m_extrinsic[0].rows == 3,
		//                 "extrinsic matrix size is not initialized");

		//int curIdx = 0;

		//// 1. intrinsic parameter
		//Mat K = m_intrinsic;
		//const double fx = p[curIdx++];
		//const double fy = p[curIdx++];
		//const double skew = p[curIdx++];
		//const double cx = p[curIdx++];
		//const double cy = p[curIdx++];
		//const double data[3][3] = {{fx, skew, cx}, {0, fy, cy}, {0, 0, 1}};
		//for (int i = 0; i < m_intrinsic.rows; i++) {
		//  for (int j = 0; j < m_intrinsic.cols; j++) {
		//    K.at<double>(i, j) = data[i][j];
		//  }
		//}

		//// 2. distortion coefficients
		//m_k1 = p[curIdx++];
		//m_k2 = p[curIdx++];

		//// 3. extrinsic paramater
		//int vecSize = m_extrinsic.size();
		//for (int i = 0; i < vecSize; i++) {
		//  double t_data[3] = {p[curIdx++], p[curIdx++], p[curIdx++]};
		//  double r_data[3] = {p[curIdx++], p[curIdx++], p[curIdx++]};

		//  Mat R;
		//  Rodrigues(Mat(3, 1, CV_64F, r_data), R);
		//  Mat t(3, 1, CV_64F, t_data);

		//  BOOST_ASSERT_MSG(R.cols == 3 && R.rows == 3, "R matrix is not 3x3");

		//  Mat E;
		//  hconcat(R, t, E);

		//  MatIterator_<double> it;
		//  MatIterator_<double> src = E.begin<double>();
		//  for (it = m_extrinsic[i].begin<double>();
		//       it != m_extrinsic[i].end<double>(); it++, src++) {
		//    *it = *src;
		//  }
		//}

		//APR_LOGGER(GET_LOGGER, DEBUG) << "curIdx :" << curIdx;

		return cv.NewMat(), []cv.Mat{}, 1, 2
	}(p_init))
}
