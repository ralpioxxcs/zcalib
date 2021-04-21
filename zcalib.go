package zcalib

import (
	"os"

	"github.com/ralpioxxcs/zcalib/lm"
	"github.com/sirupsen/logrus"
	cv "gocv.io/x/gocv"
	_ "gonum.org/v1/gonum/mat"
)

type Data struct {
	// Board is general information of checkerboard
	Board struct {
		Count      int     `yaml:"count"`
		Row        int     `yaml:"row"`
		Column     int     `yaml:"column"`
		SquareSize float32 `yaml:"squareSize"`
		Width      float32 `yaml:"width"`
		Height     float32 `yaml:"height"`
	} `yaml:"board"`
	//Coordinates []cv.Point2fVector `yaml:"coordinates"`
	Coordinates [][]Point2f `yaml:"coordinates"`
}

// CalibResults is set of calibration matrices
type CalibResults struct {
	Intrinsic   cv.Mat
	DistCoeffs  cv.Mat
	Homography  []cv.Mat
	Extrinsics  []cv.Mat
	Rotation    []cv.Mat
	Translation []cv.Mat
}

type Point2f []float32 // for only saving yaml

type Point3f struct {
	X float32
	Y float32
	Z float32
}

// logger context
var logger = logrus.New()

func Run(data Data) CalibResults {
	logger.SetFormatter(&logrus.TextFormatter{
		DisableColors: false,
		FullTimestamp: true,
	})
	logger.SetOutput(os.Stdout)
	logger.SetLevel(logrus.DebugLevel)
	logger.Info("Calculate homography matrix each boards")

	// initialize object point vector
	objPts := []cv.Point2f{}
	for i := 0; i < data.Board.Row; i++ {
		for j := 0; j < data.Board.Column; j++ {
			objPts = append(objPts,
				cv.Point2f{
					X: float32(j) * data.Board.SquareSize,
					Y: float32(i) * data.Board.SquareSize})
		}
	}
	obj := cv.NewPoint2fVectorFromPoints(objPts)

	// initialize image point vector
	imgVec := []cv.Point2fVector{}
	for _, s := range data.Coordinates {
		cvpt := []cv.Point2f{}
		for _, s2 := range s {
			cvpt = append(cvpt, cv.Point2f{X: s2[0], Y: s2[1]})
		}
		imgVec = append(imgVec, cv.NewPoint2fVectorFromPoints(cvpt))
	}

	// 1. Calculate homographies
	logger.Info("Solve H matrix (homography)")
	homographies := []cv.Mat{}
	// get each optimized homography matrix
	for i, imgPt := range imgVec {
		H := SolveH(imgPt, obj)
		Hary, _ := H.DataPtrFloat32()
		Hopt := lm.CurveFitting(Hary, obj, imgPt)
		homographies = append(homographies, Hopt)
		logger.Infof("[%v] Board refined homography matrix : \n%v", i, printFormattedMat(Hopt))
	}

	//// 2. Extract intrisic camera paramter from homography matrix
	//logger.Info("Solve K matrix (camera intrinsic)")
	//K := solveK(homographies)

	//fk := mat.Formatted(K, mat.Prefix(""), mat.Squeeze())
	//logger.Infof("K matrix : \n%v", fk)

	//// 3. Calculate extrinsic matrix each board angle
	//logger.Info("Solve E matrix (camera extrinsics)")
	//var extrinsics []mat.Dense
	//for i, h := range homographies {
	//  E := solveE(h, *K)
	//  extrinsics = append(extrinsics, *E)

	//  fe := mat.Formatted(E, mat.Prefix(""), mat.Squeeze())
	//  logger.Infof("[%v]Board extrinsic matrix : \n%v", i, fe)
	//}

	// 4. Estimate disotortion coefficients
	//k1, k2 := estimateLensDistortion(uvPts, xyzPt, K, extrinsics)
	//logger.Info("k1 : %v, k2 : %v", k1, k2)

	// 5. Refine all parameters
	//K, extrinsics, k1, k2 = refineAll(uvPts, xyzPt, K, extrinsics, k1, k2)

	return CalibResults{}
}
