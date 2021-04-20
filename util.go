package zcalib

import "gonum.org/v1/gonum/mat"

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
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

//import (
////"gonum.org/v1/gonum/stat"
//)

//func mean(slice []float32) float32 {
//  if len(slice) == 1 {
//    return slice[0]
//  }
//  sum := float32(0.0)
//  for _, v := range slice {
//    sum += v
//  }
//  return sum / float32(len(slice))

//  stat.Mean
//}

//func variance(slice []float32) float32 {
//  if len(slice) == 1 {
//    return slice[0]
//  }

//  return stat.Variance(slice, nil)
//}

//template <typename T>
//T variance(const std::vector<T>& vec) {
//  size_t sz = vec.size();
//  if (sz == 1) {
//    return 0.0;
//  }

//  T mean = std::accumulate(vec.begin(), vec.end(), 0.0) / sz;

//  auto variance_func = [&mean, &sz](T accumulator, const T& val) {
//    return accumulator + ((val - mean) * (val - mean) / (sz - 1));
//  };
//  return std::accumulate(vec.begin(), vec.end(), 0.0, variance_func);
//}
