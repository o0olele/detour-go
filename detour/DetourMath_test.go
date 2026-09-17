package detour

import (
	"math"
	"testing"
)

// DtMathFloorf uses an int32 fast path that must agree with C's floorf()/Go's
// math.Floor for every input, including the boundaries of the fast path.
func TestDtMathFloorf(t *testing.T) {
	var vals []float32
	for i := -200; i <= 200; i++ {
		f := float32(i)
		vals = append(vals, f, f+0.5, f-0.25, f+0.75)
	}
	for e := -40; e <= 40; e++ {
		v := float32(math.Pow(2, float64(e)))
		vals = append(vals,
			v, -v,
			math.Nextafter32(v, 0), math.Nextafter32(-v, 0),
			math.Nextafter32(v, float32(math.Inf(1))), math.Nextafter32(-v, float32(math.Inf(-1))),
		)
	}
	vals = append(vals,
		-2147483648.0, -2147483649.0, 2147483647.0, 2147483520.0, 2147483648.0,
		4294967296.0, -4294967296.0,
		0.0, float32(math.Copysign(0, -1)), 0.5, -0.5, 1e-8, -1e-8,
		1e30, -1e30, 1e-45, -1e-45,
	)

	for _, v := range vals {
		want := float32(math.Floor(float64(v)))
		got := DtMathFloorf(v)
		if got != want {
			t.Fatalf("DtMathFloorf(%v) = %v, want %v", v, got, want)
		}
		if math.Signbit(float64(got)) != math.Signbit(float64(want)) {
			t.Fatalf("DtMathFloorf(%v) sign mismatch: got %v, want %v", v, got, want)
		}
	}

	if !math.IsNaN(float64(DtMathFloorf(float32(math.NaN())))) {
		t.Error("DtMathFloorf(NaN) is not NaN")
	}
	if got := DtMathFloorf(float32(math.Inf(1))); !math.IsInf(float64(got), 1) {
		t.Errorf("DtMathFloorf(+Inf) = %v", got)
	}
	if got := DtMathFloorf(float32(math.Inf(-1))); !math.IsInf(float64(got), -1) {
		t.Errorf("DtMathFloorf(-Inf) = %v", got)
	}
}

// DtMathFabsf must match C's fabsf(), including -0.0 -> +0.0 and NaN.
func TestDtMathFabsf(t *testing.T) {
	vals := []float32{
		0, float32(math.Copysign(0, -1)), 1, -1, 0.5, -0.5,
		float32(math.Inf(1)), float32(math.Inf(-1)),
		math.MaxFloat32, -math.MaxFloat32, math.SmallestNonzeroFloat32, -math.SmallestNonzeroFloat32,
	}
	for i := -100; i <= 100; i++ {
		vals = append(vals, float32(i)*0.25)
	}
	for _, v := range vals {
		want := float32(math.Abs(float64(v)))
		got := DtMathFabsf(v)
		if got != want || math.Signbit(float64(got)) != math.Signbit(float64(want)) {
			t.Fatalf("DtMathFabsf(%v) = %v, want %v", v, got, want)
		}
	}
	if !math.IsNaN(float64(DtMathFabsf(float32(math.NaN())))) {
		t.Error("DtMathFabsf(NaN) is not NaN")
	}
}
