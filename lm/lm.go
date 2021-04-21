package lm

/*
#cgo pkg-config: opencv
#cgo CFLAGS: -I${SRCDIR}/lm
#cgo LDFLAGS: -L${SRCDIR}/lm -Wl,-rpath=\${SRCDIR}/lm -llm
#include "lm.h"
*/
import "C"
import (
	"fmt"
	cv "gocv.io/x/gocv"
)

// CurveFitting return refined linear homography matrix
// Using Nonlinear Least Sqaures
// * [args]
//		elem : 3x3 serialized elements
//		obj  : Nx2 world points
//		img  : Nx2 detected corner image points
// * [return]
//		refined 3x3 homography matrix
func CurveFitting(elem []float32, obj []float32, img []float32) cv.Mat {

	elemArray := make([]C.int, len(elem))
	for i, s := range elem {
		elemArray[i] = C.int(s)
	}

	elemIntVector := C.IntVector{
		val:    (*C.int)(&elemArray[0]),
		length: C.int(len(elem)),
	}

	take := C.test(elemIntVector)
	fmt.Println("take : ", take)

	refined := cv.NewMatWithSize(3, 3, cv.MatTypeCV16S)
	row1 := refined.RowRange(0, 1)
	row1.SetIntAt(0, 1, 55)

	return refined
}
