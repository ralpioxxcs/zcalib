package zcalib

// #cgo CXXFLAGS: -I.
// #cgo CFLAGS: -I.
// #cgo LDFLAGS: -lopencv_core
// #include "lm.h"
import "C"

func CurveFit() {
	C.print_test()
}
