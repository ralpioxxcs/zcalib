package zcalib

import "gonum.org/v1/gonum/mat"

func solveE(H mat.Matrix, K mat.Matrix) (E mat.Matrix) {
	logger.Info("solveK")

	return mat.NewDense(2, 2, []float64{
		1, 0,
		1, 0,
	})
}
