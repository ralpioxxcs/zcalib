package zcalib

import (
	"os"

	"github.com/sirupsen/logrus"
	cv "gocv.io/x/gocv"
	"gonum.org/v1/gonum/mat"
)

type Data struct {
	// Board is general information of checkerboard
	Board struct {
		Count      int `yaml:"count"`
		Row        int `yaml:"row"`
		Column     int `yaml:"column"`
		SquareSize int `yaml:"squareSize"`
		Width      int `yaml:"width"`
		Height     int `yaml:"height"`
	} `yaml:"board"`
	Coordinates []Points `yaml:"coordinates"`
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

// Points is a set of Point
type Points []Point

// Point3s is a set of Point3
type Point3s []Point3

// Point is a [x,y]
type Point [2]float64

// Point is a [x,y,z]
type Point3 [3]float64

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

	// initialize 3d coordinates of board
	var xyzPt Point3s
	for i := 0; i < data.Board.Row; i++ {
		for j := 0; j < data.Board.Column; j++ {
			xyzPt = append(xyzPt,
				Point3{
					float64(i * data.Board.SquareSize),
					float64(j * data.Board.SquareSize),
					0})
		}
	}

	// 1. Calculate homographies
	var obj Points
	for i := 0; i < data.Board.Row; i++ {
		for j := 0; j < data.Board.Column; j++ {
			obj = append(obj,
				Point{
					float64(i * data.Board.SquareSize),
					float64(j * data.Board.SquareSize)})
		}
	}

	logger.Info("Solve H matrix (homography)")
	uvPts := data.Coordinates
	var homographies []mat.Dense
	// get each optimized homography matrix
	for i, uvPt := range uvPts {
		H := solveH(uvPt, obj)
		homographies = append(homographies, *H)

		fh := mat.Formatted(H, mat.Prefix(""), mat.Squeeze())
		logger.Infof("[%v]Board homography matrix : \n%v", i, fh)
	}

	// 2. Extract intrisic camera paramter from homography matrix
	logger.Info("Solve K matrix (camera intrinsic)")
	K := solveK(homographies)

	fk := mat.Formatted(K, mat.Prefix(""), mat.Squeeze())
	logger.Infof("K matrix : \n%v", fk)

	// 3. Calculate extrinsic matrix each board angle
	logger.Info("Solve E matrix (camera extrinsics)")
	var extrinsics []mat.Dense
	for i, h := range homographies {
		E := solveE(h, *K)
		extrinsics = append(extrinsics, *E)

		fe := mat.Formatted(E, mat.Prefix(""), mat.Squeeze())
		logger.Infof("[%v]Board extrinsic matrix : \n%v", i, fe)
	}

	// 4. Estimate disotortion coefficients
	//k1, k2 := estimateLensDistortion(uvPts, xyzPt, K, extrinsics)
	//logger.Info("k1 : %v, k2 : %v", k1, k2)

	// 5. Refine all parameters
	//K, extrinsics, k1, k2 = refineAll(uvPts, xyzPt, K, extrinsics, k1, k2)

	return CalibResults{}
}
