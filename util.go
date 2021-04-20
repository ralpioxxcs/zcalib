package zcalib

import (
	"gocv.io/x/gocv"
	"gonum.org/v1/gonum/mat"
)

func CvMatToMat(m gocv.Mat) (*mat.Dense, error) {
	convertedImg := gocv.NewMat()
	m.ConvertTo(&convertedImg, gocv.MatTypeCV64F)
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

//func variance(slice []float32) float32 {
//  if len(slice) == 1 {
//    return slice[0]
//  }
//  //return
//}
