package zcalib

import "gonum.org/v1/gonum/mat"

func solveK(homographies []mat.Matrix) (k mat.Matrix) {
	logger.Info("solveK")

	return mat.NewDense(2, 2, []float64{
		1, 0,
		1, 0,
	})
}
