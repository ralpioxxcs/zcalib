package zcalib

import (
	cv "gocv.io/x/gocv"
	"math"
)

func estimateLensDistortion(
	imgVec []cv.Point2fVector,
	obj cv.Point2fVector,
	K cv.Mat,
	extrinsics []cv.Mat) (k1, k2 float32) {

	/**
	 *  u(i,j)  : 투영된 uv 좌표 (3d->2d), projected sensor points
	 *  u'(i,j) : 실제로 관찰된 uv좌표, observed sensor points
	 *  d'(i,j) : observed 왜곡 벡터, observed distorted vector
	 *  d(i,j)  : model distortion vector
	 *
	 *  D(r,k)  : k0*r^2 + k1*r^4
	 **/

	uc := K.GetFloatAt(0, 2)
	vc := K.GetFloatAt(1, 2)

	M := len(imgVec) // number of views
	N := obj.Size()  // number of model points

	D := cv.NewMatWithSize(2*M*N, 2, cv.MatTypeCV32F)
	d := cv.NewMatWithSize(2*M*N, 1, cv.MatTypeCV32F)

	homogenObj := toHomogeneous3d(obj)
	// loop for view
	for i := 0; i < M; i++ {
		// loop for model points
		for j := i * (N * 2); j < ((i * (N * 2)) + (N * 2)); j += 2 {
			Drow_u := D.RowRange(j, j+1)
			Drow_v := D.RowRange(j+1, (j+1)+1)
			drow_u := d.RowRange(j, j+1)
			drow_v := d.RowRange(j+1, (j+1)+1)

			t := homogenObj[(j%(N*2))/2]
			npPt := normalize_projection(t, extrinsics[i])
			rad := math.Sqrt(float64((npPt.X * npPt.X) + (npPt.Y * npPt.Y)))
			projected := projection_simple(t, K, extrinsics[i])

			logger.Debugf("npPt : %v,%v", npPt.X, npPt.Y)
			logger.Debugf("rad : %v", rad)
			logger.Debugf("projected : %v,%v", projected.X, projected.Y)

			Du0 := (projected.X - uc) * (float32(math.Pow(rad, 2)))
			Du1 := (projected.X - uc) * (float32(math.Pow(rad, 4)))
			Dv0 := (projected.Y - vc) * (float32(math.Pow(rad, 2)))
			Dv1 := (projected.Y - vc) * (float32(math.Pow(rad, 4)))
			du := (projected.X - imgVec[i].At((j%(N*2))/2).X)
			dv := (projected.Y - imgVec[i].At((j%(N*2))/2).Y)

			Drow_u.SetFloatAt(0, 0, Du0)
			Drow_u.SetFloatAt(0, 1, Du1)
			Drow_v.SetFloatAt(0, 0, Dv0)
			Drow_v.SetFloatAt(0, 1, Dv1)
			drow_u.SetFloatAt(0, 0, du)
			drow_v.SetFloatAt(0, 0, dv)
		}
	}

	pInv := cv.NewMat()
	defer pInv.Close()

	cv.Invert(D, &pInv, cv.SolveDecompositionSvd)

	// 2x(2mn) * (2mn)x1 = 2x1
	k := pInv.MultiplyMatrix(d)

	logger.Debugf("k : \n%v", printFormattedMat(k))

	return k.GetFloatAt(0, 0), k.GetFloatAt(0, 1)
}
