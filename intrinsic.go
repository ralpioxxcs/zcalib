package zcalib

import (
	"gonum.org/v1/gonum/mat"
	"math"
)

/*
   |   v(0,1)(H)             |       | 0 |
   |   v(0,0)(H) - v(1,1)(H) |       | 0 |
   ---------------------------       -----
   |           .             |       | 0 |
   |           .             |       | 0 |
   --------------------------- * b = -----   ===> V*b = 0
   |           .             |       | 0 |
   |           .             |       | 0 |
   ---------------------------       -----
   |   v(0,1)(H(m-1))        |       | 0 |
   |   v(0,0)(H(m-1))-v(1,1) |       | 0 |
   ---------------------------       -----
*/
func solveK(homographies []mat.Dense) *mat.Dense {
	N := len(homographies)

	// Generate stacked matrix
	V := mat.NewDense(2*N, 6, nil)
	for i := 0; i < 2*N; i += 2 {
		// contraints
		v00 := getVElements(&homographies[i/2], 0, 0)
		v01 := getVElements(&homographies[i/2], 0, 1)
		v11 := getVElements(&homographies[i/2], 1, 1)

		r, c := v00.Dims()
		row1 := mat.NewDense(r, c, nil)
		row2 := mat.NewDense(r, c, nil)
		row1 = mat.DenseCopyOf(v01)
		row2.Sub(v00, v11)

		V.SetRow(i, row1.RawRowView(0))
		V.SetRow(i+1, row2.RawRowView(0))
	}

	// Solve SVD
	logger.Info("Solve SVD")
	var svd mat.SVD
	ok := svd.Factorize(V, mat.SVDFull)
	if !ok {
		logger.Fatal("failed to factorize V")
	}
	// extract SVD
	m, n := V.Dims()
	u := mat.NewDense(m, m, nil)
	vt := mat.NewDense(n, n, nil)
	svd.UTo(u)
	svd.VTo(vt)
	s := svd.Values(nil)
	sminIdx := 0
	smin := s[sminIdx]
	for i, v := range s {
		if smin > v {
			smin = v
			sminIdx = i
		}
	}
	brow := vt.RawRowView(sminIdx)
	B := mat.NewDense(3, 3, []float64{
		brow[0], brow[1], brow[3],
		brow[1], brow[2], brow[4],
		brow[3], brow[4], brow[5],
	})

	//fb := mat.Formatted(B, mat.Prefix(""), mat.Squeeze())
	//logger.Infof("K matrix : \n%v", fb)

	v0 := (B.At(0, 1)*B.At(0, 2) - B.At(0, 0)*B.At(1, 2)) / (B.At(0, 0)*B.At(1, 1) - B.At(0, 1)*B.At(0, 1))
	lambda := B.At(2, 2) - ((B.At(0, 2)*B.At(0, 2))+v0*(B.At(0, 1)*B.At(0, 2)-B.At(0, 0)*B.At(1, 2)))/B.At(0, 0)
	alpha := math.Sqrt(lambda / B.At(0, 0))
	beta := math.Sqrt(lambda * B.At(0, 0) / (B.At(0, 0)*B.At(1, 1) - B.At(0, 1)*B.At(0, 1)))
	gamma := -(B.At(0, 1) * (alpha * alpha) * beta) / lambda
	u0 := ((gamma * v0) / beta) - (B.At(0, 2) * (alpha * alpha) / lambda)

	logger.Debugf("v0 : %v, lambda : %v , alpha : %v, beta : %v, gamma : %v, u0 : %v", v0, lambda, alpha, beta, gamma, u0)

	return mat.NewDense(3, 3, []float64{
		alpha, gamma, u0,
		0, beta, v0,
		0, 0, 1,
	})
}

func getVElements(h mat.Matrix, p int, q int) *mat.Dense {
	return mat.NewDense(1, 6, []float64{
		(h.At(0, p) * h.At(0, q)),
		(h.At(0, p) * h.At(1, q)) + (h.At(1, p) * h.At(0, q)),
		(h.At(1, p) * h.At(1, q)),
		(h.At(2, p) * h.At(0, q)) + (h.At(0, p) * h.At(2, q)),
		(h.At(2, p) * h.At(1, q)) + (h.At(1, p) * h.At(2, q)),
		(h.At(2, p) * h.At(2, q)),
	})
}
