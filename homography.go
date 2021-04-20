package zcalib

import (
	"math"

	cv "gocv.io/x/gocv"
	_ "gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/stat"
)

func SolveH(img Points, obj Points) cv.Mat {
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
	M := cv.NewMatWithSize(2*N, 9, cv.MatTypeCV64F)
	for i := 0; i < N; i++ {
		row1 := M.RowRange(i, i+1)
		row2 := M.RowRange(i+N, (i+N)+1)

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

		for k := 0; k < M.Cols(); k++ {
			row1.SetDoubleAt(0, k, data1[k])
			row2.SetDoubleAt(0, k, data2[k])
		}
	}

	// SVD(Singular Vector Decomposition)
	// M = U*Sigma*V
	// M               : (2 * N) X 9
	// U 		 					 : (2 * N) X (2 * N)
	// Sigma(diagonal) : 9 X 1
	// V_t             : 9 X 9
	r, c := M.Rows(), M.Cols()
	U := cv.NewMatWithSize(r, r, cv.MatTypeCV64F)
	Sigma := cv.NewMatWithSize(c, 1, cv.MatTypeCV64F)
	V_t := cv.NewMatWithSize(c, c, cv.MatTypeCV64F)
	cv.SVDCompute(M, &Sigma, &U, &V_t)

	ptr, _ := Sigma.DataPtrFloat64()
	minIdx := minIdx(ptr)
	H_norm := V_t.RowRange(minIdx, minIdx+1)
	H_norm = H_norm.Reshape(0, 3)
	logger.Debugf("homography (normalized) : \n%v", printFormattedMat(H_norm))

	// Denormalize homography matrix
	normImgMatrixInv := cv.NewMat()
	cv.Invert(normImgMatrix, &normImgMatrixInv, cv.SolveDecompositionLu)
	temp := normImgMatrixInv.MultiplyMatrix(H_norm)
	H := temp.MultiplyMatrix(normObjMatrix)
	logger.Debugf("homography matrix : \n%v", printFormattedMat(H))

	return H
}

// RefineH returns refined linear homography matrix
// using nonlinear least sqaures
//
// Args :
//	H   : 3x3 homography matrix
//  obj : Nx2 world points
//  img : Nx2 detected corner points (uv)
// Return :
// 	 Refined 3x3 homography matrix
func RefineH(H cv.Mat, obj []Point, img []Point) cv.Mat {
	//Hclone := H.Clone()
	//Hopt := cv.NewMatWithSize(c, 1, cv.MatTypeCV64F)

	return cv.Mat{}
}

func normalize(pts []Point) cv.Mat {
	var xs = make([]float64, len(pts))
	var ys = make([]float64, len(pts))
	for i, s := range pts {
		xs[i] = s[0]
		ys[i] = s[1]
	}

	meanX := stat.Mean(xs, nil)
	meanY := stat.Mean(ys, nil)
	varianceX := stat.Variance(xs, nil)
	varianceY := stat.Variance(ys, nil)
	sX := math.Sqrt(2 / varianceX)
	sY := math.Sqrt(2 / varianceY)

	logger.Infof("mean X,Y : [%v,%v]", meanX, meanY)
	logger.Infof("variance X,Y : [%v,%v]", varianceX, varianceY)
	logger.Infof("sX,sY : [%v,%v]", sX, sY)

	srcElems := []float64{
		sX, 0.0, -sX * meanX,
		0.0, sY, -sY * meanY,
		0.0, 0.0, 1,
	}
	return NewMatWithSizeNElem(3, 3, cv.MatTypeCV64F, srcElems)
}

func homogeneous(pts Points) []Point3 {
	homoPts := []Point3{}
	for _, s := range pts {
		homoPts = append(homoPts, Point3{s[0], s[1], 1})
	}
	return homoPts
}

func dotProduct(normMat cv.Mat, homoPts []Point3) []Point3 {
	ret := []Point3{}
	for _, s := range homoPts {
		lMat := NewMatWithSizeNElem(
			1, 3, cv.MatTypeCV64F,
			[]float64{s[0], s[1], s[2]})
		dotMat := lMat.MultiplyMatrix(normMat.T())
		ret = append(ret, Point3{
			dotMat.GetDoubleAt(0, 0),
			dotMat.GetDoubleAt(0, 1),
			dotMat.GetDoubleAt(0, 2)})
	}
	return ret
}
