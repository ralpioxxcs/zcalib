package zcalib

/*
#cgo CFLAGS: -I ${SRCDIR}/c
#cgo LDFLAGS: -L${SRCDIR}/c -Wl,-rpath=\${SRCDIR}/c -llm
#include "lm.h"
*/
import "C"

func CurveFit() {
	C.test()
}
