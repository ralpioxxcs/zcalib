package zcalib

import (
	"fmt"
	cv "gocv.io/x/gocv"
	"gonum.org/v1/gonum/mat"
	"math"
)

func printFormattedMat(m cv.Mat) fmt.Formatter {
	temp, _ := CvMatToMat(m)
	return mat.Formatted(temp, mat.Prefix(""), mat.Squeeze())
}

func NewMatWithSizeNElem(rows int, cols int, mt cv.MatType, srcElems []float32) cv.Mat {
	dstMat := cv.NewMatWithSize(rows, cols, mt)
	dstElems, _ := dstMat.DataPtrFloat32()
	copy(dstElems, srcElems)
	return dstMat
}

func CvMatToMat(m cv.Mat) (*mat.Dense, error) {
	convertedImg := cv.NewMat()
	m.ConvertTo(&convertedImg, cv.MatTypeCV64F)
	imgArray, err := convertedImg.DataPtrFloat64()
	if err != nil {
		return mat.NewDense(0, 0, nil), err
	}
	return mat.NewDense(m.Rows(), m.Cols(), imgArray), nil
}

func CrossProduct(a cv.Vecf, b cv.Vecf) cv.Vecf {
	return cv.Vecf{
		a[1]*b[2] - a[2]*b[1],
		-(a[0]*b[2] - a[2]*b[0]),
		a[0]*b[1] - a[1]*b[0],
	}
}

func DotProduct(a cv.Vecf, b cv.Vecf) float32 {
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func minIdx(v []float32) int {
	minIdx := 0
	min := v[minIdx]
	for i, s := range v {
		if min > s {
			minIdx = i
			min = s
		}
	}
	return minIdx
}

func argmin(v mat.Dense) (int, int) {
	si, sj := 0, 0
	for i := 0; i < v.RawMatrix().Rows; i++ {
		for j := 0; j < v.RawMatrix().Cols; j++ {
			if v.At(si, sj) > v.At(i, j) {
				si, sj = i, j
			}
		}
	}
	return si, sj
}

func mean(slice []float32) float32 {
	if len(slice) == 1 {
		return slice[0]
	}
	sum := float32(0.0)
	for _, v := range slice {
		sum += v
	}
	return sum / float32(len(slice))
}

func normalize_projection(pt cv.Vecf, E cv.Mat) cv.Point2f {
	homoMat := NewMatWithSizeNElem(4, 1, cv.MatTypeCV32F, pt)
	dstMat := E.MultiplyMatrix(homoMat)

	return cv.Point2f{
		X: dstMat.GetFloatAt(0, 0) / dstMat.GetFloatAt(2, 0),
		Y: dstMat.GetFloatAt(1, 0) / dstMat.GetFloatAt(2, 0),
	}
}

func projection_simple(pt cv.Vecf, K cv.Mat, E cv.Mat) cv.Point2f {
	homoMat := NewMatWithSizeNElem(4, 1, cv.MatTypeCV32F, pt)
	dst := E.MultiplyMatrix(homoMat)

	pts := cv.Vecf{
		dst.GetFloatAt(0, 0),
		dst.GetFloatAt(0, 1),
		dst.GetFloatAt(0, 2)}

	inhomoPt := cv.Vecf{pts[0] / pts[2], pts[1] / pts[2], 1}

	k1 := K.RowRange(0, 1)
	k2 := K.RowRange(1, 2)
	kp := cv.NewMat()
	cv.Vconcat(k1, k2, &kp)

	tempMat := NewMatWithSizeNElem(3, 1, cv.MatTypeCV32F, inhomoPt)
	res := kp.MultiplyMatrix(tempMat)

	return cv.Point2f{X: res.GetFloatAt(0, 0), Y: res.GetFloatAt(0, 1)}
}

func toHomogeneous(pts cv.Point2fVector) []Point3f {
	homoPts := []Point3f{}
	gopts := pts.ToPoints()
	for _, s := range gopts {
		homoPts = append(homoPts, Point3f{
			X: s.X,
			Y: s.Y,
			Z: 1.0})
	}
	return homoPts
}

func toHomogeneous3d(pts cv.Point2fVector) []cv.Vecf {
	homo3dPts := []cv.Vecf{}
	gopts := pts.ToPoints()
	for _, s := range gopts {
		homo3dPts = append(homo3dPts, cv.Vecf{
			s.X,
			s.Y,
			0,
			1.0,
		})
	}
	return homo3dPts
}

// toRodrigues31to33 returns 3x3 matrix Rodrigues formed
func toRodrigues31to33(v cv.Vecf) cv.Mat {
	const (
		FLT_EPSILON = 1.1920929e-07
		DBL_EPSILON = 2.2204460492503131e-16
	)

	dst := cv.NewMat()

	r := v
	theta := cv.Norm(NewMatWithSizeNElem(3, 1, cv.MatTypeCV32F, r), cv.NormL2)
	if theta < DBL_EPSILON {
		R := NewMatWithSizeNElem(3, 3, cv.MatTypeCV32F, []float32{
			v[0], 0, 0,
			0, v[1], 0,
			0, 0, v[2],
		})
		R.CopyTo(&dst)
	} else {
		c := math.Cos(theta)
		s := math.Sin(theta)
		c1 := 1. - c
		var itheta float64
		if theta > 0 {
			itheta = 1. / theta
		} else {
			itheta = 0.
		}

		r[0] *= float32(itheta)
		r[1] *= float32(itheta)
		r[2] *= float32(itheta)

		rrt := NewMatWithSizeNElem(3, 3, cv.MatTypeCV32F, []float32{
			r[0] * r[0], r[0] * r[1], r[0] * r[2],
			r[0] * r[1], r[1] * r[1], r[1] * r[2],
			r[0] * r[2], r[1] * r[2], r[2] * r[2],
		})
		r_x := NewMatWithSizeNElem(3, 3, cv.MatTypeCV32F, []float32{
			0, -r[2], r[1],
			r[2], 0, -r[0],
			-r[1], r[0], 0,
		})

		eye := NewMatWithSizeNElem(3, 3, cv.MatTypeCV32F, []float32{
			1, 0, 0,
			0, 1, 0,
			0, 0, 1})
		eye.MultiplyFloat(float32(c))
		rrt.MultiplyFloat(float32(c1))
		r_x.MultiplyFloat(float32(s))

		cv.Add(eye, rrt, &rrt)
		cv.Add(rrt, r_x, &r_x)

		R := r_x.Clone()
		R.CopyTo(&dst)

	}

	return dst
}

// toRodrigues33to31 return 3x1 vector Rodrigues formed
func toRodrigues33to31(m cv.Mat) cv.Vecf {
	// m = 3x3 matrix
	// SVD
	R := m.Clone()
	U, Sigma, V_t := cv.NewMat(), cv.NewMat(), cv.NewMat()
	cv.SVDCompute(R, &Sigma, &U, &V_t)
	R = U.MultiplyMatrix(V_t)

	r := cv.Vecf{
		R.GetFloatAt(2, 1) - R.GetFloatAt(1, 2),
		R.GetFloatAt(0, 2) - R.GetFloatAt(2, 0),
		R.GetFloatAt(1, 0) - R.GetFloatAt(0, 1)}

	s := math.Sqrt(float64((r[0]*r[0] + r[1]*r[1] + r[2]*r[2]) * 0.25))
	c := (R.GetFloatAt(0, 0) + R.GetFloatAt(1, 1) + R.GetFloatAt(2, 2) - 1) * 0.5
	//c = c > 1. ? 1. : c < -1. ? -1. : c
	if c > 1. {
		c = 1.
	} else {
		if c < -1. {
			c = -1.
		}
	}
	theta := math.Acos(float64(c))

	if s < 1e-5 {
		if c > 0 {
			r = cv.Vecf{0, 0, 0}
		} else {
			t := (R.GetFloatAt(0, 0) + 1) * 0.5
			r[0] = float32(math.Sqrt(math.Max(float64(t), 0.)))

			t = (R.GetFloatAt(1, 1) + 1) * 0.5
			temp := math.Sqrt(math.Max(float64(t), 0.)) * float64(R.GetFloatAt(0, 1))
			if temp < 0 {
				temp = -1.
			} else {
				temp = 1.
			}
			r[1] = float32(temp)

			t = (R.GetFloatAt(2, 2) + 1) * 0.5
			temp = math.Sqrt(math.Max(float64(t), 0.)) * float64(R.GetFloatAt(0, 2))
			if temp < 0 {
				temp = -1.
			} else {
				temp = 1.
			}
			r[2] = float32(temp)

			if (math.Abs(float64(r[0])) < math.Abs(float64(r[1]))) &&
				(math.Abs(float64(r[0])) < math.Abs(float64(r[2]))) &&
				(R.GetFloatAt(1, 2) > 0) !=
					(r[1]*r[2] > 0) {
				r[2] = -r[2]
			}

			theta /= cv.Norm(NewMatWithSizeNElem(3, 1, cv.MatTypeCV32F, r), cv.NormL2)
			r[0] *= float32(theta)
			r[1] *= float32(theta)
			r[2] *= float32(theta)
		}
	} else {
		vth := 1 / (2 * s)
		vth *= theta
		rmat := NewMatWithSizeNElem(3, 1, cv.MatTypeCV32F, r)
		rmat.MultiplyFloat(float32(vth))

		r[0] = rmat.GetFloatAt(0, 0)
		r[1] = rmat.GetFloatAt(1, 0)
		r[2] = rmat.GetFloatAt(2, 0)
	}

	return cv.Vecf{r[0], r[1], r[2]}
}
