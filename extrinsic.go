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

	// 3x3 * 3x1 = 3x1
	temp := K_inv.MultiplyMatrix(Hcol0)
	r0 := temp.Clone()
	r0.MultiplyFloat(float32(lambda))

	// 3x3 * 3x1 = 3x1
	temp = K_inv.MultiplyMatrix(Hcol1)
	r1 := temp.Clone()
	r1.MultiplyFloat(float32(lambda))

	// 3x1 * 3x1 =  3x1
	r2Vec := CrossProduct(
		cv.Vecf{
			r0.GetFloatAt(0, 0),
			r0.GetFloatAt(1, 0),
			r0.GetFloatAt(2, 0)},
		cv.Vecf{
			r1.GetFloatAt(0, 0),
			r1.GetFloatAt(1, 0),
			r1.GetFloatAt(2, 0)})
	r2 := NewMatWithSizeNElem(3, 1, cv.MatTypeCV32F, r2Vec)

	// translation matrix
	temp = K_inv.MultiplyMatrix(Hcol2)
	tMat := temp.Clone()
	tMat.MultiplyFloat(float32(lambda))

	// generate concated matrix
	rStack := cv.NewMat()
	cv.Hconcat(r0, r1, &rStack)
	cv.Hconcat(rStack, r2, &rStack)

	// Solve SVD
	r, c := rStack.Rows(), rStack.Cols()
	U := cv.NewMatWithSize(r, r, cv.MatTypeCV32F)
	Sigma := cv.NewMatWithSize(c, 1, cv.MatTypeCV32F)
	V_t := cv.NewMatWithSize(c, c, cv.MatTypeCV32F)
	cv.SVDCompute(rStack, &Sigma, &U, &V_t)

	// rotation matrix
	rMat := U.MultiplyMatrix(V_t)

	// E = R | t
	eMat := cv.NewMat()
	cv.Hconcat(rMat, tMat, &eMat)

	return eMat
}
