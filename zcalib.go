package zcalib

import (
	"os"

	"github.com/sirupsen/logrus"
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
	Intrinsic   Mat
	DistCoeffs  Mat
	Homography  Mats
	Extrinsics  Mats
	Rotation    Mats
	Translation Mats
}

// Points is a set of Point
type Points []Point

// Point3s is a set of Point3
type Point3s []Point3

// Point is a [x,y]
type Point [2]float32

// Point is a [x,y,z]
type Point3 [3]float32

// Mats is a set of Matrix
type Mats []mat.Matrix

// Mat is matrix
type Mat mat.Matrix

// logger context
var logger = logrus.New()

func Run(data Data) CalibResults {
	logger.SetFormatter(&logrus.TextFormatter{
		DisableColors: false,
		FullTimestamp: true,
	})
	logger.SetOutput(os.Stdout)
	logger.Info("Calculate homography matrix each boards")

	// 1. Calculate homographies
	// initialize 3d coordinates of board
	var xyzPt Point3s
	for i := 0; i < data.Board.Row; i++ {
		for j := 0; j < data.Board.Column; j++ {
			xyzPt = append(xyzPt,
				Point3{
					float32(i * data.Board.SquareSize),
					float32(j * data.Board.SquareSize),
					0})
		}
	}

	logger.Info("Solve H matrix (homography)")
	uvPts := data.Coordinates
	var homographies []mat.Matrix
	for i, uvPt := range uvPts {
		// get each optimized homography matrix
		homographies = append(homographies, solveH(uvPt, xyzPt))
		logger.Info("[%d]Board homography matrix : %v", i, homographies[i])
	}

	// 2. Extract intrisic camera paramter from homography matrix
	logger.Info("Solve K matrix (camera intrinsic)")
	K := solveK(homographies)
	logger.Info("K matrix : %v", K)

	// 3. Calculate extrinsic matrix each board angle
	logger.Info("Solve E matrix (camera extrinsics)")
	var extrinsics []mat.Matrix
	for i, h := range homographies {
		extrinsics = append(extrinsics, solveE(h, K))
		logger.Info("[%d]Board extrinsic matrix : %v", i, extrinsics[i])
	}

	// 4. Estimate disotortion coefficients
	k1, k2 := estimateLensDistortion(uvPts, xyzPt, K, extrinsics)
	logger.Info("k1 : %v, k2 : %v", k1, k2)

	// 5. Refine all parameters
	//K, extrinsics, k1, k2 = refineAll(uvPts, xyzPt, K, extrinsics, k1, k2)

	return CalibResults{}
}
