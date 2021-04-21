package lm

/*
#cgo pkg-config: opencv4
#cgo CXXFLAGS: --std=c++11
#cgo LDFLAGS: -lcminpack
#include <stdlib.h>
#include "lm.h"
*/
import "C"
import (
	_ "fmt"
	cv "gocv.io/x/gocv"
	"reflect"
	"unsafe"
)

// PointVector is a wrapper around a std::vector< cv::Point >*
type PointVector struct {
	p C.PointVector
}

// Point2fVector is a wrapper around a std::vector< std::vector< cv::Point2f > >*
type Point2fVector struct {
	p C.Point2fVector
}

// CurveFitting return refined linear homography matrix
// Using Nonlinear Least Sqaures
// * [args]
//		elem : 3x3 serialized elements
//		obj  : Nx2 world points
//		img  : Nx2 detected corner image points
// * [return]
//		refined 3x3 homography matrix
func CurveFitting(elem []float32, obj cv.Point2fVector, img cv.Point2fVector) cv.Mat {

	elemArray := make([]C.float, len(elem))
	for i, s := range elem {
		elemArray[i] = C.float(s)
	}
	elemFloatVector := C.FloatVector{
		val:    (*C.float)(&elemArray[0]),
		length: C.int(len(elem)),
	}

	objArray := make([]C.struct_Point2f, obj.Size())
	for i := 0; i < obj.Size(); i++ {
		objArray[i] = C.struct_Point2f{
			x: C.float(obj.At(i).X),
			y: C.float(obj.At(i).Y),
		}
	}
	objPoint2fVector := C.Points2f{
		points: (*C.Point2f)(&objArray[0]),
		length: C.int(obj.Size()),
	}

	imgArray := make([]C.struct_Point2f, img.Size())
	for i := 0; i < img.Size(); i++ {
		imgArray[i] = C.struct_Point2f{
			x: C.float(img.At(i).X),
			y: C.float(img.At(i).Y),
		}
	}
	imgPoint2fVector := C.Points2f{
		points: (*C.Point2f)(&imgArray[0]),
		length: C.int(obj.Size()),
	}

	rv := C.curve_fit(
		elemFloatVector,
		C.zPoint2fVector_NewFromPoints(objPoint2fVector),
		C.zPoint2fVector_NewFromPoints(imgPoint2fVector))

	h := &reflect.SliceHeader{
		Data: uintptr(unsafe.Pointer(rv.val)),
		Len:  int(rv.length),
		Cap:  int(rv.length),
	}
	pa := *(*[]C.float)(unsafe.Pointer(h))

	sol := make([]float32, int(rv.length))
	for i := 0; i < len(sol); i++ {
		sol[i] = float32(pa[i])
	}

	refined := cv.NewMatWithSize(3, 3, cv.MatTypeCV32F)
	refined.SetFloatAt(0, 0, sol[0]/sol[8])
	refined.SetFloatAt(0, 1, sol[1]/sol[8])
	refined.SetFloatAt(0, 2, sol[2]/sol[8])
	refined.SetFloatAt(1, 0, sol[3]/sol[8])
	refined.SetFloatAt(1, 1, sol[4]/sol[8])
	refined.SetFloatAt(1, 2, sol[5]/sol[8])
	refined.SetFloatAt(2, 0, sol[6]/sol[8])
	refined.SetFloatAt(2, 1, sol[7]/sol[8])
	refined.SetFloatAt(2, 2, sol[8]/sol[8])

	return refined
}

//-------------------------------------------------------------

func NewPointVector() PointVector {
	return PointVector{p: C.zPointVector_New()}
}

func NewPoint2fVector() Point2fVector {
	return Point2fVector{p: C.zPoint2fVector_New()}
}

func NewPointVectorFromPoints(pts [][]int) PointVector {
	p := (*C.struct_Point)(C.malloc(C.size_t(C.sizeof_struct_Point * len(pts))))
	defer C.free(unsafe.Pointer(p))

	h := &reflect.SliceHeader{
		Data: uintptr(unsafe.Pointer(p)),
		Len:  len(pts),
		Cap:  len(pts),
	}
	pa := *(*[]C.Point)(unsafe.Pointer(h))

	for j, point := range pts {
		pa[j] = C.struct_Point{
			x: C.int(point[0]),
			y: C.int(point[1]),
		}
	}

	cpoints := C.struct_Points{
		points: (*C.Point)(p),
		length: C.int(len(pts)),
	}

	return PointVector{p: C.zPointVector_NewFromPoints(cpoints)}
}

func NewPoint2fVectorFromPoints(pts []cv.Point2f) Point2fVector {
	p := (*C.struct_Point2f)(C.malloc(C.size_t(C.sizeof_struct_Point2f * len(pts))))
	defer C.free(unsafe.Pointer(p))

	h := &reflect.SliceHeader{
		Data: uintptr(unsafe.Pointer(p)),
		Len:  len(pts),
		Cap:  len(pts),
	}
	pa := *(*[]C.Point2f)(unsafe.Pointer(h))

	for j, point := range pts {
		pa[j] = C.struct_Point2f{
			x: C.float(point.X),
			y: C.float(point.Y),
		}
	}

	cpoints := C.struct_Points2f{
		points: (*C.Point2f)(p),
		length: C.int(len(pts)),
	}

	return Point2fVector{p: C.zPoint2fVector_NewFromPoints(cpoints)}
}
