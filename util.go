package zcalib

import (
	"fmt"
	cv "gocv.io/x/gocv"
	"gonum.org/v1/gonum/mat"
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

//Matx33d U, Vt;
//       Vec3d W;
//       double theta, s, c;
//       int step = dst->rows > 1 ? dst->step / elem_size : 1;

//       if( (dst->rows != 1 || dst->cols*CV_MAT_CN(dst->type) != 3) &&
//           (dst->rows != 3 || dst->cols != 1 || CV_MAT_CN(dst->type) != 1))
//           CV_Error( CV_StsBadSize, "Output matrix must be 1x3 or 3x1" );

//       Matx33d R = cvarrToMat(src);

//       if( !checkRange(R, true, NULL, -100, 100) )
//       {
//           cvZero(dst);
//           if( jacobian )
//               cvZero(jacobian);
//           return 0;
//       }

//       SVD::compute(R, W, U, Vt);
//       R = U*Vt;

//       Point3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

//       s = std::sqrt((r.x*r.x + r.y*r.y + r.z*r.z)*0.25);
//       c = (R(0, 0) + R(1, 1) + R(2, 2) - 1)*0.5;
//       c = c > 1. ? 1. : c < -1. ? -1. : c;
//       theta = acos(c);

//       if( s < 1e-5 )
//       {
//           double t;

//           if( c > 0 )
//               r = Point3d(0, 0, 0);
//           else
//           {
//               t = (R(0, 0) + 1)*0.5;
//               r.x = std::sqrt(MAX(t,0.));
//               t = (R(1, 1) + 1)*0.5;
//               r.y = std::sqrt(MAX(t,0.))*(R(0, 1) < 0 ? -1. : 1.);
//               t = (R(2, 2) + 1)*0.5;
//               r.z = std::sqrt(MAX(t,0.))*(R(0, 2) < 0 ? -1. : 1.);
//               if( fabs(r.x) < fabs(r.y) && fabs(r.x) < fabs(r.z) && (R(1, 2) > 0) != (r.y*r.z > 0) )
//                   r.z = -r.z;
//               theta /= norm(r);
//               r *= theta;
//           }

//           if( jacobian )
//           {
//               memset( J, 0, sizeof(J) );
//               if( c > 0 )
//               {
//                   J[5] = J[15] = J[19] = -0.5;
//                   J[7] = J[11] = J[21] = 0.5;
//               }
//           }
func toRodrigues(m cv.Mat) cv.Vecf {

}
