package main

import (
	_ "bufio"
	"fmt"
	"image"
	"io/ioutil"
	"os"

	"github.com/ralpioxxcs/zcalib"
	cv "gocv.io/x/gocv"
	"gopkg.in/yaml.v2"
)

const (
	count      = 5
	row        = 8
	column     = 11
	squaresize = 0.060
	width      = 0.720
	height     = 0.540
)

//func FindCorner(filename string) (corrner []cv.Point2f) {
func FindCorner(filename string) (corrner cv.Point2fVector) {
	fmt.Println(filename)

	img := cv.IMRead(filename, cv.IMReadAnyColor)
	if img.Empty() {
		fmt.Printf("Error reading chessboard image (%v)\n", filename)
		return
	}
	defer img.Close()

	corners := cv.NewMat()
	defer corners.Close()
	sz := image.Point{X: column, Y: row}
	found := cv.FindChessboardCorners(img, sz, &corners, cv.CalibCBAdaptiveThresh)
	if found == false {
		fmt.Printf("chessboard pattern not found")
		return
	}
	if corners.Empty() {
		fmt.Printf("corners mat is empty")
		return
	}

	cv.CornerSubPix(img, &corners, image.Point{7, 7}, image.Point{-1, -1}, cv.NewTermCriteria(cv.Count+cv.EPS, 100, 0.01))

	fmt.Printf("Corners Found. Size: %+v Rows: %+v Cols: %+v\n", corners.Size(), corners.Rows(), corners.Cols())
	clone := img.Clone()
	cv.CvtColor(clone, &clone, cv.ColorGrayToBGR)
	defer clone.Close()
	cv.DrawChessboardCorners(&clone, sz, corners, found)
	if clone.Empty() {
		fmt.Printf("Error writing to chessboard image (%v)\n", filename)
		return
	}

	var pts []cv.Point2f
	for x := 0; x < corners.Cols(); x++ {
		for y := 0; y < corners.Rows(); y++ {
			p := corners.GetVecfAt(x, y)
			pts = append(pts, cv.Point2f{X: p[0], Y: p[1]})
		}
	}
	return cv.NewPoint2fVectorFromPoints(pts)
}

func main() {
	if len(os.Args) < 2 {
		fmt.Println("How to run:\n\tfind-chessboard [foldername]")
		return
	}

	folderpath := os.Args[1]
	files, err := ioutil.ReadDir(folderpath)
	if err != nil {
		panic(err)
	}

	cords := [][]zcalib.Point2f{}
	for _, s := range files {
		corner := FindCorner(folderpath + s.Name())

		elem := []zcalib.Point2f{}
		for _, s := range corner.ToPoints() {
			elem = append(elem, zcalib.Point2f{s.X, s.Y})
		}
		cords = append(cords, elem)
	}

	data := &zcalib.Data{
		Board: struct {
			Count      int     `yaml:"count"`
			Row        int     `yaml:"row"`
			Column     int     `yaml:"column"`
			SquareSize float32 `yaml:"squareSize"`
			Width      float32 `yaml:"width"`
			Height     float32 `yaml:"height"`
		}{
			Count:      count,
			Row:        row,
			Column:     column,
			SquareSize: squaresize,
			Width:      width,
			Height:     height,
		},
		Coordinates: cords,
	}

	d, err := yaml.Marshal(&data)
	if err != nil {
		panic(err)
	}
	err = ioutil.WriteFile("board_data.yaml", d, 0644)
	if err != nil {
		panic(err)
	}
	fmt.Println("write board_data.yaml file successfully!")
}
