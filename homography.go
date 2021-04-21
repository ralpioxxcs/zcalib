package zcalib

import (
	"math"

	cv "gocv.io/x/gocv"
	_ "gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/stat"
)

func SolveH(img cv.Point2fVector, obj cv.Point2fVector) cv.Mat {
	N := obj.Size()
	logger.Debugf("corner size : %v", N)

	if img.Size() != obj.Size() {
		logger.Fatalf("uv size != xyz size")
	}

	// Initialize normalized matrix, homogeneous points of uv, xyz points
	normObjMatrix := normalize(obj)
	normImgMatrix := normalize(img)
	homoObjPts := toHomogeneous(obj)
	homoImgPts := toHomogeneous(img)

	// Normalize all points (1x3)*(3x3)
	normedHomoObjPts := dotProduct(normObjMatrix, homoObjPts)
	normedHomoImgPts := dotProduct(normImgMatrix, homoImgPts)

	// Create M matrix
	M := cv.NewMatWithSize(2*N, 9, cv.MatTypeCV32F)
	for i := 0; i < N; i++ {
		row1 := M.RowRange(i, i+1)
		row2 := M.RowRange(i+N, (i+N)+1)

		data1 := []float32{
			-(normedHomoObjPts[i].X),
			-(normedHomoObjPts[i].Y),
			-1,
			0,
			0,
			0,
			normedHomoImgPts[i].X * normedHomoObjPts[i].X,
			normedHomoImgPts[i].X * normedHomoObjPts[i].Y,
			normedHomoImgPts[i].X}
		data2 := []float32{
			0,
			0,
			0,
			-(normedHomoObjPts[i].X),
			-(normedHomoObjPts[i].Y),
			-1,
			normedHomoImgPts[i].Y * normedHomoObjPts[i].X,
			normedHomoImgPts[i].Y * normedHomoObjPts[i].Y,
			normedHomoImgPts[i].Y}

		for k := 0; k < M.Cols(); k++ {
			row1.SetFloatAt(0, k, data1[k])
			row2.SetFloatAt(0, k, data2[k])
		}
	}

	// SVD(Singular Vector Decomposition)
	// M = U*Sigma*V
	// M               : (2 * N) X 9
	// U 		 					 : (2 * N) X (2 * N)
	// Sigma(diagonal) : 9 X 1
	// V_t             : 9 X 9
	r, c := M.Rows(), M.Cols()
	U := cv.NewMatWithSize(r, r, cv.MatTypeCV32F)
	Sigma := cv.NewMatWithSize(c, 1, cv.MatTypeCV32F)
	V_t := cv.NewMatWithSize(c, c, cv.MatTypeCV32F)
	cv.SVDCompute(M, &Sigma, &U, &V_t)

	ptr, _ := Sigma.DataPtrFloat32()
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

func normalize(pts cv.Point2fVector) cv.Mat {
	gopts := pts.ToPoints()
	nXs := make([]float64, len(gopts))
	nYs := make([]float64, len(gopts))
	for i := 0; i < len(gopts); i++ {
		nXs[i] = float64(gopts[i].X)
		nYs[i] = float64(gopts[i].Y)
	}

	meanX := stat.Mean(nXs, nil)
	meanY := stat.Mean(nYs, nil)
	varianceX := stat.Variance(nXs, nil)
	varianceY := stat.Variance(nYs, nil)
	sX := math.Sqrt(2 / varianceX)
	sY := math.Sqrt(2 / varianceY)

	logger.Debugf("mean X,Y : [%v,%v]", meanX, meanY)
	logger.Debugf("variance X,Y : [%v,%v]", varianceX, varianceY)
	logger.Debugf("sX,sY : [%v,%v]", sX, sY)

	srcElems := []float32{
		float32(sX), 0.0, float32(-sX * meanX),
		0.0, float32(sY), float32(-sY * meanY),
		0.0, 0.0, 1,
	}
	return NewMatWithSizeNElem(3, 3, cv.MatTypeCV32F, srcElems)
}

func dotProduct(normMat cv.Mat, homoPts []Point3f) []Point3f {
	ret := []Point3f{}
	for _, s := range homoPts {
		lMat := NewMatWithSizeNElem(
			1, 3, cv.MatTypeCV32F,
			[]float32{s.X, s.Y, s.Z})
		dotMat := lMat.MultiplyMatrix(normMat.T())
		ret = append(ret, Point3f{
			X: dotMat.GetFloatAt(0, 0),
			Y: dotMat.GetFloatAt(0, 1),
			Z: dotMat.GetFloatAt(0, 2)})
	}
	return ret
}
