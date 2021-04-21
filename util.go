package zcalib

import (
	"fmt"
	cv "gocv.io/x/gocv"
	"gonum.org/v1/gonum/mat"
)

func printFormattedMat(m cv.Mat) fmt.Formatter {
	temp, _ := CvMatToMat(m)
	return mat.Formatted(temp, mat.Prefix(""), mat.Squeeze())
}

func NewMatWithSizeNElem(rows int, cols int, mt cv.MatType, srcElems []float32) cv.Mat {
	dstMat := cv.NewMatWithSize(rows, cols, mt)
	dstElems, _ := dstMat.DataPtrFloat32()
	copy(dstElems, srcElems)
	return dstMat
}

func CvMatToMat(m cv.Mat) (*mat.Dense, error) {
	convertedImg := cv.NewMat()
	m.ConvertTo(&convertedImg, cv.MatTypeCV64F)
	imgArray, err := convertedImg.DataPtrFloat64()
	if err != nil {
		return mat.NewDense(0, 0, nil), err
	}
	return mat.NewDense(m.Rows(), m.Cols(), imgArray), nil
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func minIdx(v []float32) int {
	minIdx := 0
	min := v[minIdx]
	for i, s := range v {
		if min > s {
			minIdx = i
			min = s
		}
	}
	return minIdx
}

func argmin(v mat.Dense) (int, int) {
	si, sj := 0, 0
	for i := 0; i < v.RawMatrix().Rows; i++ {
		for j := 0; j < v.RawMatrix().Cols; j++ {
			if v.At(si, sj) > v.At(i, j) {
				si, sj = i, j
			}
		}
	}
	return si, sj
}

func mean(slice []float32) float32 {
	if len(slice) == 1 {
		return slice[0]
	}
	sum := float32(0.0)
	for _, v := range slice {
		sum += v
	}
	return sum / float32(len(slice))
}
