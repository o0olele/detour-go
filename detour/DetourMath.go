/**
@defgroup detour Detour

Members in this module are wrappers around the standard math library
*/

package detour

import "math"

func DtMathFabsf(x float32) float32 {
	// Exact IEEE-754 fabs: clear the sign bit. Bit-for-bit identical to C's
	// fabsf() (including -0.0 -> +0.0 and NaN payloads), but branchless and
	// without the float32 -> float64 -> float32 round trip that
	// math.Abs(float64(x)) costs.
	return math.Float32frombits(math.Float32bits(x) &^ (1 << 31))
}

func DtMathSqrtf(x float32) float32 { return float32(math.Sqrt(float64(x))) }

func DtMathFloorf(x float32) float32 {
	// Fast path: for |x| < 2^31 the truncating conversion plus a single
	// correction gives exactly the same result as C's floorf(), without the
	// float32 -> float64 -> float32 round trip through math.Floor (a
	// non-inlinable Go function with significand bit twiddling).
	// Everything outside that range, plus NaN/Inf, falls through.
	if x >= -2147483648.0 && x < 2147483648.0 {
		i := int32(x)
		if float32(i) > x {
			i--
		}
		f := float32(i)
		if f == 0 && math.Float32bits(x)&(1<<31) != 0 {
			// floorf(-0.0) is -0.0, but the int32 round trip yields +0.0.
			// floor never changes the sign, so f == 0 together with a negative
			// input means x was exactly -0.0.
			return float32(math.Float32frombits(1 << 31))
		}
		return f
	}
	return float32(math.Floor(float64(x)))
}

func DtMathCeilf(x float32) float32             { return float32(math.Ceil(float64(x))) }
func DtMathCosf(x float32) float32              { return float32(math.Cos(float64(x))) }
func DtMathSinf(x float32) float32              { return float32(math.Sin(float64(x))) }
func DtMathAtan2f(y float32, x float32) float32 { return float32(math.Atan2(float64(y), float64(x))) }
