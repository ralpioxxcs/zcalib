package zcalib

import (
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/stat"
	"math"
)

func solveH(img Points, obj Points) (hmat mat.Matrix) {
	logger.Info("solveH")

	size := len(obj)

	if len(img) != len(obj) {
		logger.Fatalf("uv size != xyz size")
	}

	// Initialize normalized matrix, homogeneous points of uv, xyz points
	normObjMatrix := normalize(obj)
	normImgMatrix := normalize(img)
	//fa := mat.Formatted(normObjMatrix, mat.Prefix(""), mat.Squeeze())
	//logger.Infof("\n%v", fa)
	homoObjPts := homogeneous(obj)
	homoImgPts := homogeneous(img)

	// Normalize all points (1x3)*(3x3)
	normedHomoObjPts := dotProduct(normObjMatrix, homoObjPts)
	normedHomoImgPts := dotProduct(normImgMatrix, homoImgPts)

	// Create M matrix
	M := mat.NewDense(2*len(obj), 9, nil)
	for i := 0; i < M.RawMatrix().Rows/2; i++ {
		row1 := M.RawRowView(i)
		row2 := M.RawRowView(i + size)

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
		for i := 0; i < len(row1); i++ {
			row1 = append(row1, data1[i])
			row2 = append(row2, data2[i])
		}
	}

	// SVD(Singular Vector Decomposition)
	logger.Info("Solve SVD")

	var svd mat.SVD
	ok := svd.Factorize(M, mat.SVDFull)
	if !ok {
		logger.Fatal("failed to factorize A")
	}

	//APR_LOGGER(GET_LOGGER, TRACE) << "solving SVD";
	//Mat U(2 * vecSize, 2 * vecSize, CV_64F);
	//Mat Sigma(9, 9, CV_64F);
	//Mat V_t(1, 9, CV_64F);
	//SVD::compute(M, Sigma, U, V_t, SVD::FULL_UV);
	//double min, max;
	//int minIdx, maxIdx;
	//minMaxIdx(Sigma, &min, &max, &minIdx, &maxIdx);

	//Mat H_norm = V_t.row(minIdx);
	//H_norm = H_norm.reshape(0, 3);
	//APR_LOGGER(GET_LOGGER, DEBUG) << "homography (normalized):\n" << H_norm;

	//// 6) 호모그래피 행렬 비정규화
	//Mat invMat = normImgMatrix.inv();
	//Mat h1 = invMat * H_norm;
	//Mat H = h1 * normObjMatrix;
	//APR_LOGGER(GET_LOGGER, DEBUG) << "homography matrix:\n" << H;

	return mat.NewDense(2, 2, []float64{
		1, 0,
		1, 0,
	})
}

func optimize() {
}

func normalize(pts Points) Mat {
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

	return mat.NewDense(3, 3, []float64{
		sX, 0.0, -sX * meanX,
		0.0, sY, -sY * meanY,
		0.0, 0.0, 1,
	})
}

func homogeneous(pts Points) Point3s {
	homoPts := Point3s{}
	for _, s := range pts {
		homoPts = append(homoPts, Point3{s[0], s[1], 1})
	}
	return homoPts
}

func dotProduct(normMat Mat, homoPts Point3s) Point3s {
	var ret Point3s
	for _, s := range homoPts {
		lMat := mat.NewDense(1, 3, []float64{s[0], s[1], s[2]})
		lMat.Mul(lMat, normMat)
		ret = append(ret, Point3{lMat.At(0, 0), lMat.At(0, 1), lMat.At(0, 2)})
	}
	return ret
}
