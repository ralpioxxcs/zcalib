package zcalib

import (
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/stat"
	"math"
)

func solveH(img Points, obj Points) *mat.Dense {
	N := len(obj)
	logger.Infof("corner size : %v", N)

	if len(img) != len(obj) {
		logger.Fatalf("uv size != xyz size")
	}

	// Initialize normalized matrix, homogeneous points of uv, xyz points
	normObjMatrix := normalize(obj)
	normImgMatrix := normalize(img)
	homoObjPts := homogeneous(obj)
	homoImgPts := homogeneous(img)

	// Normalize all points (1x3)*(3x3)
	normedHomoObjPts := dotProduct(normObjMatrix, homoObjPts)
	normedHomoImgPts := dotProduct(normImgMatrix, homoImgPts)

	// Create M matrix
	M := mat.NewDense(2*N, 9, nil)
	for i := 0; i < N; i++ {
		row1 := M.RawRowView(i)
		row2 := M.RawRowView(i + N)

		data1 := []float64{
			-(normedHomoObjPts[i][0]),
			-(normedHomoObjPts[i][1]),
			-1,
			0,
			0,
			0,
			normedHomoImgPts[i][0] * normedHomoObjPts[i][0],
			normedHomoImgPts[i][0] * normedHomoObjPts[i][1],
			normedHomoImgPts[i][0]}
		data2 := []float64{
			0,
			0,
			0,
			-(normedHomoObjPts[i][0]),
			-(normedHomoObjPts[i][1]),
			-1,
			normedHomoImgPts[i][1] * normedHomoObjPts[i][0],
			normedHomoImgPts[i][1] * normedHomoObjPts[i][1],
			normedHomoImgPts[i][1]}

		if len(row1) != len(row2) {
			logger.Fatalf("row1(%v), row2(%v) size is not equal", len(row1), len(row2))
		}
		for i := 0; i < 9; i++ {
			row1 = append(row1, data1[i])
			row2 = append(row2, data2[i])
		}
	}
	//fm := mat.Formatted(M, mat.Prefix(""), mat.Squeeze())
	//logger.Infof("M matrix :\n%v", fm)

	// SVD(Singular Vector Decomposition)
	// M     : (2 * len) X 9
	// U 		 : (2 * len) X (2 * len)
	// Sigma : (2 * len) X 9
	// V_t   : 9 X 9
	logger.Info("Solve SVD")
	var svd mat.SVD
	ok := svd.Factorize(M, mat.SVDFull)
	if !ok {
		logger.Fatal("failed to factorize M")
	}
	// extract SVD
	m, n := M.Dims()
	u := mat.NewDense(m, m, nil)
	vt := mat.NewDense(n, n, nil)
	//sigma := mat.NewDense(m, n, nil)
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
	H_norm := mat.NewDense(3, 3, vt.RawRowView(sminIdx))

	// Denormalize homography matrix
	var normImgMatrixInv mat.Dense
	err := normImgMatrixInv.Inverse(normImgMatrix)
	if err != nil {
		logger.Fatalf("normImgMatrix is not invertible: %v", err)
	}
	var h1, H mat.Dense
	h1.Mul(&normImgMatrixInv, H_norm)
	H.Mul(&h1, normObjMatrix)

	return &H
}

func optimize() {
}

func normalize(pts Points) *mat.Dense {
	var xs = make([]float64, len(pts))
	var ys = make([]float64, len(pts))
	for i, s := range pts {
		xs[i] = s[0]
		ys[i] = s[1]
	}

	meanX := stat.Mean(xs, nil)
	meanY := stat.Mean(ys, nil)
	varianceX := stat.Variance(xs, nil)
	varianceY := stat.Mean(ys, nil)
	sX := math.Sqrt(varianceX)
	sY := math.Sqrt(varianceY)

	normMat := mat.NewDense(3, 3, []float64{
		sX, 0.0, -sX * meanX,
		0.0, sY, -sY * meanY,
		0.0, 0.0, 1,
	})

	return normMat
}

func homogeneous(pts Points) Point3s {
	homoPts := Point3s{}
	for _, s := range pts {
		homoPts = append(homoPts, Point3{s[0], s[1], 1})
	}
	return homoPts
}

func dotProduct(normMat mat.Matrix, homoPts Point3s) Point3s {
	var ret Point3s
	for _, s := range homoPts {
		lMat := mat.NewDense(1, 3, []float64{s[0], s[1], s[2]})
		lMat.Mul(lMat, normMat)
		ret = append(ret, Point3{lMat.At(0, 0), lMat.At(0, 1), lMat.At(0, 2)})
	}
	return ret
}
