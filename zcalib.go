package zcalib

import (
	"github.com/sirupsen/logrus"
	"os"
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

// Points is a set of Point
type Points []Point

// Point is a [x,y]
type Point [2]float32

// Point is a [x,y,z]
type Point3 [3]float32

// logger context
var logger = logrus.New()

func Run(d Data) {
	logger.SetFormatter(&logrus.TextFormatter{
		DisableColors: false,
		FullTimestamp: true,
	})
	logger.SetOutput(os.Stdout)
	logger.Info("Calculate homography matrix each boards")

	solveH()
}
