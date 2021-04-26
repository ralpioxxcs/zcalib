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

// PointVector is a wrapper around a std::vector<cv::Point>*
type PointVector struct {
	p C.PointVector
}

// Point2fVector is a wrapper around a std::vector<cv::Point2f>*
type Point2fVector struct {
	p C.Point2fVector
}

// Point2dVector is a wrapper around a std::vector<cv::Point2f>*
type Point2dVector struct {
	p C.Point2dVector
}

// PointVectorVector is a wrapper around a std::vector<std::vector<cv::Point>>*
type PointVectorVector struct {
	p C.PointVectorVector
}

// Point2fVectorVector is a wrapper around a std::vector<std::vector<cv::Point2f>>*
type Point2fVectorVector struct {
	p C.Point2fVectorVector
}

// Point2dVectorVector is a wrapper around a std::vector<std::vector<cv::Point2f>>*
type Point2dVectorVector struct {
	p C.Point2dVectorVector
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
	elemArray := make([]C.double, len(elem))
	for i, s := range elem {
		elemArray[i] = C.double(s)
	}
	elemDoubleVector := C.DoubleVector{
		val:    (*C.double)(&elemArray[0]),
		length: C.int(len(elem)),
	}

	objArray := make([]C.struct_Point2d, obj.Size())
	for i := 0; i < obj.Size(); i++ {
		objArray[i] = C.struct_Point2d{
			x: C.double(obj.At(i).X),
			y: C.double(obj.At(i).Y),
		}
	}
	objPoint2dVector := C.Points2d{
		points: (*C.Point2d)(&objArray[0]),
		length: C.int(obj.Size()),
	}

	imgArray := make([]C.struct_Point2d, img.Size())
	for i := 0; i < img.Size(); i++ {
		imgArray[i] = C.struct_Point2d{
			x: C.double(img.At(i).X),
			y: C.double(img.At(i).Y),
		}
	}
	imgPoint2dVector := C.Points2d{
		points: (*C.Point2d)(&imgArray[0]),
		length: C.int(img.Size()),
	}

	rv := C.curve_fit(
		elemDoubleVector,
		C.zPoint2dVector_NewFromPoints(objPoint2dVector),
		C.zPoint2dVector_NewFromPoints(imgPoint2dVector))

	h := &reflect.SliceHeader{
		Data: uintptr(unsafe.Pointer(rv.val)),
		Len:  int(rv.length),
		Cap:  int(rv.length),
	}
	pa := *(*[]C.double)(unsafe.Pointer(h))

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

func CurveFittingAll(elem []float32, obj cv.Point2fVector, imgVec []cv.Point2fVector) []float32 {
	// elements
	elemArray := make([]C.double, len(elem))
	for i, s := range elem {
		elemArray[i] = C.double(s)
	}
	elemDoubleVector := C.DoubleVector{
		val:    (*C.double)(&elemArray[0]),
		length: C.int(len(elem)),
	}

	// objects
	objArray := make([]C.struct_Point2d, obj.Size())
	for i := 0; i < obj.Size(); i++ {
		objArray[i] = C.struct_Point2d{
			x: C.double(obj.At(i).X),
			y: C.double(obj.At(i).Y),
		}
	}
	objPoint2dVector := C.Points2d{
		points: (*C.Point2d)(&objArray[0]),
		length: C.int(obj.Size()),
	}

	// image points vector
	imgVecArray := make([]C.struct_Points2d, len(imgVec))
	for i, pt := range imgVec {
		p := (*C.struct_Point2d)(C.malloc(C.size_t(C.sizeof_struct_Point2d * pt.Size())))
		defer C.free(unsafe.Pointer(p))

		h := &reflect.SliceHeader{
			Data: uintptr(unsafe.Pointer(p)),
			Len:  pt.Size(),
			Cap:  pt.Size(),
		}
		pa := *(*[]C.Point2d)(unsafe.Pointer(h))

		for j := 0; j < pt.Size(); j++ {
			pa[j] = C.struct_Point2d{
				x: C.double(imgVec[i].At(j).X),
				y: C.double(imgVec[i].At(j).Y),
			}
		}

		imgVecArray[i] = C.struct_Points2d{
			points: (*C.Point2d)(p),
			length: C.int(pt.Size()),
		}
	}

	imgVecPoint2dVector := C.struct_Points2dVectors{
		vec:    (*C.struct_Points2d)(&imgVecArray[0]),
		length: C.int(len(imgVec)),
	}

	rv := C.curve_fit_all(
		elemDoubleVector,
		C.zPoint2dVector_NewFromPoints(objPoint2dVector),
		C.zPoint2dVectorVector_NewFromVector(imgVecPoint2dVector))

	h := &reflect.SliceHeader{
		Data: uintptr(unsafe.Pointer(rv.val)),
		Len:  int(rv.length),
		Cap:  int(rv.length),
	}
	pa := *(*[]C.double)(unsafe.Pointer(h))

	sol := make([]float32, int(rv.length))
	for i := 0; i < len(sol); i++ {
		sol[i] = float32(pa[i])
	}
	return sol
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
	p := (*C.struct_Point2d)(C.malloc(C.size_t(C.sizeof_struct_Point2d * len(pts))))
	defer C.free(unsafe.Pointer(p))

	h := &reflect.SliceHeader{
		Data: uintptr(unsafe.Pointer(p)),
		Len:  len(pts),
		Cap:  len(pts),
	}
	pa := *(*[]C.Point2f)(unsafe.Pointer(h))

	for j, point := range pts {
		pa[j] = C.struct_Point2f{
			x: C.double(point.X),
			y: C.double(point.Y),
		}
	}

	cpoints := C.struct_Points2f{
		points: (*C.Point2f)(p),
		length: C.int(len(pts)),
	}

	return Point2fVector{p: C.zPoint2fVector_NewFromPoints(cpoints)}
}

func NewPointVectorVectorFromVectors(vec [][][]int) PointVectorVector {
	p := (*C.struct_Points)(C.malloc(C.size_t(C.sizeof_struct_Points * len(vec))))
	defer C.free(unsafe.Pointer(p))

	//h := &reflect.SliceHeader{
	//  Data: uintptr(unsafe.Pointer(p)),
	//  Len:  len(vec),
	//  Cap:  len(vec),
	//}
	//pa := *(*[]C.struct_Points)(unsafe.Pointer(h))

	//for j, point := range vec {
	//  pa[j] = NewPointVectorFromPoints(point)
	//}

	cpoints := C.struct_PointsVectors{
		vec:    (*C.struct_Points)(p),
		length: C.int(len(vec)),
	}

	return PointVectorVector{p: C.zPointVectorVector_NewFromVector(cpoints)}
}

func NewPoint2fVectorVectorFromVectors(vec [][]cv.Point2f) Point2fVectorVector {
	p := (*C.struct_Points2f)(C.malloc(C.size_t(C.sizeof_struct_Points2f * len(vec))))
	defer C.free(unsafe.Pointer(p))

	//h := &reflect.SliceHeader{
	//  Data: uintptr(unsafe.Pointer(p)),
	//  Len:  len(pts),
	//  Cap:  len(pts),
	//}
	//pa := *(*[]C.Point)(unsafe.Pointer(h))

	//for j, point := range pts {
	//  pa[j] = C.struct_Point{
	//    x: C.int(point[0]),
	//    y: C.int(point[1]),
	//  }
	//}

	cpoints := C.struct_Points2fVectors{
		vec:    (*C.struct_Points2f)(p),
		length: C.int(2),
	}

	return Point2fVectorVector{p: C.zPoint2fVectorVector_NewFromVector(cpoints)}
}
