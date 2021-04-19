package main

import (
	"bufio"
	"fmt"
	"image"
	"io/ioutil"
	"os"

	"github.com/ralpioxxcs/zcalib"
	"gocv.io/x/gocv"
	"gopkg.in/yaml.v2"
)

const (
	count      = 5
	row        = 8
	column     = 11
	squaresize = 550
	width      = 1000
	height     = 700
)

func FindCorner(filename string) (corner []image.Point) {
	img := gocv.IMRead(filename, gocv.IMReadAnyColor)
	if img.Empty() {
		fmt.Printf("Error reading chessboard image (%v)\n", filename)
		return
	}
	defer img.Close()

	corners := gocv.NewMat()
	defer corners.Close()
	sz := image.Point{X: column, Y: row}
	found := gocv.FindChessboardCorners(img, sz, &corners, gocv.CalibCBAdaptiveThresh)
	if found == false {
		fmt.Printf("chessboard pattern not found")
		return
	}
	if corners.Empty() {
		fmt.Printf("corners mat is empty")
		return
	}

	fmt.Printf("Corners Found. Size: %+v Rows: %+v Cols: %+v\n", corners.Size(), corners.Rows(), corners.Cols())
	clone := img.Clone()
	gocv.CvtColor(clone, &clone, gocv.ColorGrayToBGR)
	defer clone.Close()
	gocv.DrawChessboardCorners(&clone, sz, corners, found)
	if clone.Empty() {
		fmt.Printf("Error writing to chessboard image (%v)\n", filename)
		return
	}

	var pts []image.Point
	for x := 0; x < corners.Cols(); x++ {
		for y := 0; y < corners.Rows(); y++ {
			p := corners.GetVecfAt(x, y)
			pts = append(pts, image.Point{X: int(p[0]), Y: int(p[1])})
		}
	}
	return pts
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

	var cords []zcalib.Points
	for _, s := range files {
		corner := FindCorner(folderpath + s.Name())
		var cord zcalib.Points
		for i, ss := range corner {
			fmt.Printf("[%v] : %v\n", i, ss)
			cord = append(cord, zcalib.Point{float32(ss.X), float32(ss.Y)})
		}
		reader := bufio.NewReader(os.Stdin)
		reader.ReadString('\n')

		cords = append(cords, cord)
	}

	data := &zcalib.Data{
		Board: struct {
			Count      int `yaml:"count"`
			Row        int `yaml:"row"`
			Column     int `yaml:"column"`
			SquareSize int `yaml:"squareSize"`
			Width      int `yaml:"width"`
			Height     int `yaml:"height"`
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
}
