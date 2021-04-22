package zcalib

import (
	"github.com/ralpioxxcs/zcalib/lm"
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
		fx := K.GetFloatAt(0, 0)
		fy := K.GetFloatAt(1, 1)
		skew := K.GetFloatAt(0, 1)
		cx := K.GetFloatAt(0, 2)
		cy := K.GetFloatAt(1, 2)

		M := len(imgVec)

		pvec := make([]float32, 7)
		extVec := make([]float32, M*6)
		pvec[0] = fx
		pvec[1] = fy
		pvec[2] = skew
		pvec[3] = cx
		pvec[4] = cy
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
			Rod := toRodrigues33to31(R)
			logger.Debugf("Rodrigues output (3x3 -> 3x1) : %v", Rod)

			// translation
			extVec[i+0] = extrinsics[i/offset].GetFloatAt(0, 3)
			extVec[i+1] = extrinsics[i/offset].GetFloatAt(1, 3)
			extVec[i+2] = extrinsics[i/offset].GetFloatAt(2, 3)

			// rotation (rodrigues)
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
			Yvec[j] = make([]cv.Point2f, len(imgVec[j].ToPoints()))
			for k, v2 := range imgVec[j].ToPoints() {
				Yvec[j][k] = v2
			}
		}
	}
	refined_p := lm.CurveFittingAll(p_init, obj, imgVec)

	logger.Infof("refined_p : %v", refined_p)

	// Decompose vector to matrix form
	return (func(p []float32) (cv.Mat, []cv.Mat, float32, float32) {
		//logger.Debugf("p size : %v", len(p))

		//// Intrinsic parameter ( 0 ~ 4 )
		//K := NewMatWithSizeNElem(3, 3, cv.MatTypeCV32F, []float32{
		//  p[0], p[2], p[3],
		//  0, p[1], p[3],
		//  0, 0, 1,
		//})
		//// Distortion coefficients (5,6)
		//k1 := p[5]
		//k2 := p[6]

		//curIdx := 6
		//// Extrinsic paramater
		//t_data, r_data := make([]float32, len(extrinsics)), make([]float32, len(extrinsics))
		//for i, e := range extrinsics {
		//  t_data[i] = []float32{p[curIdx+i], p[curIdx+i+1], p[curIdx+i+2]}
		//  r_data[i] = []float32{p[curIdx+i+3], p[curIdx+i+4], p[curIdx+i+5]}

		//  R := toRodrigues(NewMatWithSizeNElem(3, 1, cv.MatTypeCV32F, r_data))
		//  t := NewMatWithSizeNElem(3, 1, cv.MatTypeCV32F, t_data)

		//  //E := cv.NewMat()
		//  //cv.Hconcat(R, t, &E)
		//  // extrinsics[i] = E

		//}

		return cv.NewMat(), []cv.Mat{}, 1, 2
	}(refined_p))
}
