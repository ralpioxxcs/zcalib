package zcalib

import "gonum.org/v1/gonum/mat"

func solveH(uvPts Points, xyzPts Point3s) (hmat mat.Matrix) {
	logger.Info("solveH")

	return mat.NewDense(2, 2, []float64{
		1, 0,
		1, 0,
	})
}

func optimize() {
}
