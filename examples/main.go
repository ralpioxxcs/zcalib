package main

import (
	"fmt"
	"gopkg.in/yaml.v2"
	"io/ioutil"

	"github.com/ralpioxxcs/zcalib"
)

func main() {
	file, err := ioutil.ReadFile("board_data.yaml")
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
