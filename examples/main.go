package main

import (
	"fmt"
	"io/ioutil"
	"os"

	"gopkg.in/yaml.v2"

	"github.com/ralpioxxcs/zcalib"
)

func main() {
	if len(os.Args) < 2 {
		fmt.Println("How to run:\n\tfind-chessboard [foldername]")
		return
	}
	filename := os.Args[1]

	file, err := ioutil.ReadFile(filename)
	if err != nil {
		panic(err)
	}

	data := zcalib.Data{}
	err = yaml.Unmarshal(file, &data)
	if err != nil {
		panic(err)
	}

	for i, s := range data.Coordinates {
		fmt.Printf("point[%v] : %v\n", i, s)
	}

	zcalib.Run(data)
}
