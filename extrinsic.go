package zcalib

import (
	cv "gocv.io/x/gocv"
	_ "gonum.org/v1/gonum/mat"
)

func solveE(H cv.Mat, K cv.Mat) cv.Mat {

	K_inv := cv.NewMatWithSize(3, 3, cv.MatTypeCV32F)
	ret := cv.Invert(K, &K_inv, cv.SolveDecompositionLu)
	logger.Debugf("K_inv ret : %v", ret)

	Hcol0 := H.ColRange(0, 1)
	Hcol1 := H.ColRange(1, 2)
	Hcol2 := H.ColRange(2, 3)

	lambda := 1 / cv.Norm(K_inv.MultiplyMatrix(Hcol0), cv.NormL2)

	temp := K_inv.MultiplyMatrix(Hcol0)
	r0 := temp.Clone()
	r0.MultiplyFloat(float32(lambda))

	temp = K_inv.MultiplyMatrix(Hcol1)
	r1 := temp.Clone()
	r1.MultiplyFloat(float32(lambda))

	r2 := r1.MultiplyMatrix(r0.T())
	temp = K_inv.MultiplyMatrix(Hcol2)
	t := temp.Clone()
	t.MultiplyFloat(float32(lambda))

	rStack := cv.NewMat()
	cv.Hconcat(r0, r1, &rStack)
	cv.Hconcat(rStack, r2, &rStack)

	// Solve SVD
	r, c := rStack.Rows(), rStack.Cols()
	U := cv.NewMatWithSize(r, r, cv.MatTypeCV32F)
	Sigma := cv.NewMatWithSize(c, 1, cv.MatTypeCV32F)
	V_t := cv.NewMatWithSize(c, c, cv.MatTypeCV32F)
	cv.SVDCompute(rStack, &Sigma, &U, &V_t)

	R := U.MultiplyMatrix(V_t)

	E := cv.NewMat()
	cv.Hconcat(R, t, &E)

	return E
}
