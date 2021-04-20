TARG=lm
CGOFILES=optimize.go
include $(GOROOT)/src/Make.$(GOARCH)
include $(GOROOT)/src/Make.pkg
lm.o:foo.cpp
    g++ $(_CGO_CFLAGS_$(GOARCH)) -fPIC -O2 -o $@ -c $(CGO_CFLAGS) $<
clm.o:cfoo.cpp
    g++ $(_CGO_CFLAGS_$(GOARCH)) -fPIC -O2 -o $@ -c $(CGO_CFLAGS) $<
CGO_LDFLAGS+=-lstdc++
$(elem)_lm.so: foo.cgo4.o foo.o cfoo.o
    gcc $(_CGO_CFLAGS_$(GOARCH)) $(_CGO_LDFLAGS_$(GOOS)) -o $@ $^ $(CGO_LDFLAGS)
