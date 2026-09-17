package detour

import (
	"math"
	"testing"
)

// xorshift is a tiny deterministic PRNG so this test is reproducible.
type xorshift struct{ s uint32 }

func (r *xorshift) next() uint32 {
	r.s ^= r.s << 13
	r.s ^= r.s >> 17
	r.s ^= r.s << 5
	return r.s
}

// f returns a float32 roughly in [-10, 10).
func (r *xorshift) f() float32 {
	return float32(r.next()>>8)/float32(1<<24)*20 - 10
}

// TestInternalHelperEquivalence asserts that the fixed-size internal helpers
// used by the hot query paths (closestPointOnPoly, closestPointOnPolyBoundary,
// getPolyHeight) produce bit-identical results to the exported slice-based
// implementations they mirror.
//
// The two implementations are separate code, so this test is what keeps them
// from silently diverging.
func TestInternalHelperEquivalence(t *testing.T) {
	r := &xorshift{s: 0x9E3779B9}

	for iter := 0; iter < 300000; iter++ {
		nverts := 3 + int(r.next()%4) // 3..6, i.e. <= DT_VERTS_PER_POLYGON
		var verts [DT_VERTS_PER_POLYGON * 3]float32
		for i := 0; i < nverts*3; i++ {
			verts[i] = r.f()
		}
		var pt [3]float32
		pt[0], pt[1], pt[2] = r.f(), r.f(), r.f()

		// --- DtDistancePtPolyEdgesSqr ---
		var edA, etA [DT_VERTS_PER_POLYGON]float32
		cA := DtDistancePtPolyEdgesSqr(pt[:], verts[:nverts*3], nverts, edA[:], etA[:])
		var edB, etB [DT_VERTS_PER_POLYGON]float32
		cB := dtDistancePtPolyEdgesSqr(pt[0], pt[2], &verts, nverts, &edB, &etB)
		if cA != cB {
			t.Fatalf("iter %d: inside flag exported=%v internal=%v", iter, cA, cB)
		}
		for j := 0; j < nverts; j++ {
			if math.Float32bits(edA[j]) != math.Float32bits(edB[j]) {
				t.Fatalf("iter %d edge %d: ed exported=%v internal=%v", iter, j, edA[j], edB[j])
			}
			if math.Float32bits(etA[j]) != math.Float32bits(etB[j]) {
				t.Fatalf("iter %d edge %d: et exported=%v internal=%v", iter, j, etA[j], etB[j])
			}
		}

		// --- DtDistancePtSegSqr2D ---
		var tA float32
		dA := DtDistancePtSegSqr2D(pt[:], verts[0:3], verts[3:6], &tA)
		dB, tB := dtDistancePtSegSqr2D(pt[0], pt[2], verts[0], verts[2], verts[3], verts[5])
		if math.Float32bits(dA) != math.Float32bits(dB) || math.Float32bits(tA) != math.Float32bits(tB) {
			t.Fatalf("iter %d: seg exported=(%v,%v) internal=(%v,%v)", iter, dA, tA, dB, tB)
		}

		// --- DtClosestHeightPointTriangle ---
		a := [3]float32{verts[0], verts[1], verts[2]}
		b := [3]float32{verts[3], verts[4], verts[5]}
		c := [3]float32{verts[6], verts[7], verts[8]}
		var hA float32
		okA := DtClosestHeightPointTriangle(pt[:], a[:], b[:], c[:], &hA)
		hB, okB := dtClosestHeightPointTriangle(pt, a, b, c)
		if okA != okB {
			t.Fatalf("iter %d: tri ok exported=%v internal=%v", iter, okA, okB)
		}
		if okA && math.Float32bits(hA) != math.Float32bits(hB) {
			t.Fatalf("iter %d: tri h exported=%v internal=%v", iter, hA, hB)
		}

		// --- projectPoly ---
		var aminA, amaxA float32
		projectPoly(pt[:], verts[:nverts*3], nverts, &aminA, &amaxA)
		aminB, amaxB := dtProjectPoly(pt[0], pt[2], &verts, nverts)
		if math.Float32bits(aminA) != math.Float32bits(aminB) || math.Float32bits(amaxA) != math.Float32bits(amaxB) {
			t.Fatalf("iter %d: projectPoly exported=(%v,%v) internal=(%v,%v)", iter, aminA, amaxA, aminB, amaxB)
		}

		// --- DtOverlapPolyPoly2D ---
		// Build a second polygon from the same value pool.
		npb := 3 + int(r.next()%4)
		var vertsB [DT_VERTS_PER_POLYGON * 3]float32
		// Bias B towards A so that both the "overlapping" and "separated"
		// outcomes occur regularly.
		bias := r.f()
		for i := 0; i < npb*3; i++ {
			vertsB[i] = verts[i%(nverts*3)] + bias
		}
		oA := DtOverlapPolyPoly2D(verts[:nverts*3], nverts, vertsB[:npb*3], npb)
		oB := dtOverlapPolyPoly2D(&verts, nverts, &vertsB, npb)
		if oA != oB {
			t.Fatalf("iter %d: overlapPoly exported=%v internal=%v", iter, oA, oB)
		}
	}
}
