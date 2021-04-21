package zcalib

import (
	cv "gocv.io/x/gocv"
	_ "gonum.org/v1/gonum/mat"
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
func solveK(homographies []cv.Mat) cv.Mat {
	N := len(homographies)

	// Generate stacked matrix
	V := cv.NewMatWithSize(2*N, 6, cv.MatTypeCV32F)
	for i := 0; i < 2*N; i += 2 {
		// contraints (1x6 matrix)
		v00 := getVElements(homographies[i/2], 0, 0)
		v01 := getVElements(homographies[i/2], 0, 1)
		v11 := getVElements(homographies[i/2], 1, 1)

		row1 := v01.Clone()
		row2 := cv.NewMat()
		cv.Subtract(v00, v11, &row2)

		vrow1 := V.RowRange(i, i+1)
		vrow2 := V.RowRange(i+1, (i+1)+1)

		for k := 0; k < row1.Cols(); k++ {
			vrow1.SetFloatAt(0, k, row1.GetFloatAt(0, k))
			vrow2.SetFloatAt(0, k, row2.GetFloatAt(0, k))
		}
	}

	logger.Debugf("V: \n%v", printFormattedMat(V))

	// SVD(Singular Vector Decomposition)
	// V = U*Sigma*V
	// V               : (2 * N) X 6
	// U 		 					 : (2 * N) X (2 * N)
	// Sigma(diagonal) : 6 X 1
	// V_t             : 6 X 6
	r, c := V.Rows(), V.Cols()
	U := cv.NewMatWithSize(r, r, cv.MatTypeCV32F)
	Sigma := cv.NewMatWithSize(c, 1, cv.MatTypeCV32F)
	V_t := cv.NewMatWithSize(c, c, cv.MatTypeCV32F)
	cv.SVDCompute(V, &Sigma, &U, &V_t)

	logger.Debugf("V_t: \n%v", printFormattedMat(V_t))

	ptr, _ := Sigma.DataPtrFloat32()
	minIdx := minIdx(ptr)
	b := V_t.RowRange(minIdx, minIdx+1)

	logger.Debugf("b: \n%v", printFormattedMat(b))

	B := cv.NewMatWithSize(3, 3, cv.MatTypeCV32F)
	B.SetFloatAt(0, 0, b.GetFloatAt(0, 0))
	B.SetFloatAt(0, 1, b.GetFloatAt(0, 1))
	B.SetFloatAt(0, 2, b.GetFloatAt(0, 3))
	B.SetFloatAt(1, 0, b.GetFloatAt(0, 1))
	B.SetFloatAt(1, 1, b.GetFloatAt(0, 2))
	B.SetFloatAt(1, 2, b.GetFloatAt(0, 4))
	B.SetFloatAt(2, 0, b.GetFloatAt(0, 3))
	B.SetFloatAt(2, 1, b.GetFloatAt(0, 4))
	B.SetFloatAt(2, 2, b.GetFloatAt(0, 5))

	v0 := (B.GetFloatAt(0, 1)*B.GetFloatAt(0, 2) - B.GetFloatAt(0, 0)*B.GetFloatAt(1, 2)) /
		(B.GetFloatAt(0, 0)*B.GetFloatAt(1, 1) - B.GetFloatAt(0, 1)*B.GetFloatAt(0, 1))

	lambda :=
		B.GetFloatAt(2, 2) - ((B.GetFloatAt(0, 2)*B.GetFloatAt(0, 2))+v0*(B.GetFloatAt(0, 1)*B.GetFloatAt(0, 2)-B.GetFloatAt(0, 0)*B.GetFloatAt(1, 2)))/
			B.GetFloatAt(0, 0)

	alpha := float32(math.Sqrt(float64((lambda / B.GetFloatAt(0, 0)))))

	beta := float32(math.Sqrt(float64(lambda * B.GetFloatAt(0, 0) / (B.GetFloatAt(0, 0)*B.GetFloatAt(1, 1) - B.GetFloatAt(0, 1)*B.GetFloatAt(0, 1)))))

	gamma := -(B.GetFloatAt(0, 1) * (alpha * alpha) * beta) / lambda

	u0 := ((gamma * v0) / beta) - (B.GetFloatAt(0, 2) * (alpha * alpha) / lambda)

	logger.Debugf("v0 : %v, lambda : %v , alpha : %v, beta : %v, gamma : %v, u0 : %v", v0, lambda, alpha, beta, gamma, u0)

	return NewMatWithSizeNElem(3, 3, cv.MatTypeCV32F, []float32{
		alpha, gamma, u0,
		0, beta, v0,
		0, 0, 1,
	})
}

func getVElements(h cv.Mat, p int, q int) cv.Mat {
	const (
		row = 1
		col = 6
	)
	m := cv.NewMatWithSize(row, col, cv.MatTypeCV32F)
	m.SetFloatAt(0, 0,
		h.GetFloatAt(0, p)*h.GetFloatAt(0, q))
	m.SetFloatAt(0, 1,
		(h.GetFloatAt(0, p)*h.GetFloatAt(1, q))+(h.GetFloatAt(1, p)*h.GetFloatAt(0, q)))
	m.SetFloatAt(0, 2,
		(h.GetFloatAt(1, p) * h.GetFloatAt(1, q)))
	m.SetFloatAt(0, 3,
		(h.GetFloatAt(2, p)*h.GetFloatAt(0, q))+(h.GetFloatAt(0, p)*h.GetFloatAt(2, q)))
	m.SetFloatAt(0, 4,
		(h.GetFloatAt(2, p)*h.GetFloatAt(1, q))+(h.GetFloatAt(1, p)*h.GetFloatAt(2, q)))
	m.SetFloatAt(0, 5,
		(h.GetFloatAt(2, p) * h.GetFloatAt(2, q)))

	return m
}
