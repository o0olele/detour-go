//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

package detour

// ---------------------------------------------------------------------------
// Internal fixed-size variants of the hottest geometry helpers.
//
// The exported versions in DetourCommon.go take []float32 vectors and a
// *float32 output. Every access needs a bounds check, each call passes several
// 3-word slice headers, and an out-parameter forces the result through memory
// because the compiler must assume it may alias the inputs. Their inline costs
// reflect that: DtClosestHeightPointTriangle 378, DtOverlapPolyPoly2D 476,
// DtDistancePtPolyEdgesSqr 209, DtDistancePtSegSqr2D 145 (budget is 80).
//
// These variants take scalars / fixed-size arrays by value instead, so they
// have no bounds checks and no slice headers, and they keep results in
// registers. Costs drop to 96..277, and dtProjectPoly (77) becomes inlinable.
// That matters because they sit in the inner loop of closestPointOnPoly() and
// findLocalNeighbourhood(), the two largest query hotspots.
//
// The exported functions are deliberately left untouched so the public API and
// its behaviour are unchanged; TestInternalHelperEquivalence asserts that both
// implementations agree bit-for-bit.
// ---------------------------------------------------------------------------

// dtDistancePtSegSqr2D mirrors DtDistancePtSegSqr2D but keeps t in a register.
func dtDistancePtSegSqr2D(px, pz, ax, az, bx, bz float32) (dsqr, t float32) {
	pqx := bx - ax
	pqz := bz - az
	dx := px - ax
	dz := pz - az
	d := pqx*pqx + pqz*pqz
	t = pqx*dx + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	dx = ax + t*pqx - px
	dz = az + t*pqz - pz
	return dx*dx + dz*dz, t
}

// dtDistancePtPolyEdgesSqr mirrors DtDistancePtPolyEdgesSqr over a fixed-size
// vertex buffer. verts holds nverts*3 floats and ed/et receive nverts entries.
func dtDistancePtPolyEdgesSqr(ptx, ptz float32,
	verts *[DT_VERTS_PER_POLYGON * 3]float32, nverts int,
	ed, et *[DT_VERTS_PER_POLYGON]float32) bool {
	// Clamping lets the compiler prove i*3+2 < len(verts) for every i, which
	// removes the per-edge bounds checks entirely.
	const maxVerts = int(DT_VERTS_PER_POLYGON)
	if nverts > maxVerts {
		nverts = maxVerts
	}
	c := false
	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		vix, viz := verts[i*3], verts[i*3+2]
		vjx, vjz := verts[j*3], verts[j*3+2]
		if (viz > ptz) != (vjz > ptz) {
			if ptx < (vjx-vix)*(ptz-viz)/(vjz-viz)+vix {
				c = !c
			}
		}
		ed[j], et[j] = dtDistancePtSegSqr2D(ptx, ptz, vjx, vjz, vix, viz)
	}
	return c
}

// dtClosestHeightPointTriangle mirrors DtClosestHeightPointTriangle, returning
// the height by value instead of through *float32.
func dtClosestHeightPointTriangle(p, a, b, c [3]float32) (h float32, ok bool) {
	var v0, v1, v2 [3]float32
	v0[0], v0[1], v0[2] = c[0]-a[0], c[1]-a[1], c[2]-a[2]
	v1[0], v1[1], v1[2] = b[0]-a[0], b[1]-a[1], b[2]-a[2]
	v2[0], v2[1], v2[2] = p[0]-a[0], p[1]-a[1], p[2]-a[2]

	dot00 := v0[0]*v0[0] + v0[2]*v0[2]
	dot01 := v0[0]*v1[0] + v0[2]*v1[2]
	dot02 := v0[0]*v2[0] + v0[2]*v2[2]
	dot11 := v1[0]*v1[0] + v1[2]*v1[2]
	dot12 := v1[0]*v2[0] + v1[2]*v2[2]

	invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	if u >= -EPS && v >= -EPS && (u+v) <= 1+EPS {
		return a[1] + v0[1]*u + v1[1]*v, true
	}
	return 0, false
}

// dtProjectPoly mirrors projectPoly over a fixed-size polygon. Keeping the
// running min/max in locals instead of behind *float32 out-parameters removes
// a store/load round trip per vertex.
func dtProjectPoly(ax, az float32, poly *[DT_VERTS_PER_POLYGON * 3]float32, npoly int) (rmin, rmax float32) {
	const maxVerts = int(DT_VERTS_PER_POLYGON)
	if npoly > maxVerts {
		npoly = maxVerts
	}
	rmax = ax*poly[0] + az*poly[2]
	rmin = rmax
	for i := 1; i < npoly; i++ {
		d := ax*poly[i*3] + az*poly[i*3+2]
		if d < rmin {
			rmin = d
		}
		if d > rmax {
			rmax = d
		}
	}
	return rmin, rmax
}

// dtOverlapPolyPoly2D mirrors DtOverlapPolyPoly2D over fixed-size polygons.
// This is the O(n^2) inner loop of findLocalNeighbourhood().
func dtOverlapPolyPoly2D(polya *[DT_VERTS_PER_POLYGON * 3]float32, npolya int,
	polyb *[DT_VERTS_PER_POLYGON * 3]float32, npolyb int) bool {
	const maxVerts = int(DT_VERTS_PER_POLYGON)
	if npolya > maxVerts {
		npolya = maxVerts
	}
	if npolyb > maxVerts {
		npolyb = maxVerts
	}
	for i, j := 0, npolya-1; i < npolya; j, i = i, i+1 {
		nx := polya[i*3+2] - polya[j*3+2]
		nz := -(polya[i*3] - polya[j*3])
		amin, amax := dtProjectPoly(nx, nz, polya, npolya)
		bmin, bmax := dtProjectPoly(nx, nz, polyb, npolyb)
		if !overlapRange(amin, amax, bmin, bmax, EPS) {
			return false
		}
	}
	for i, j := 0, npolyb-1; i < npolyb; j, i = i, i+1 {
		nx := polyb[i*3+2] - polyb[j*3+2]
		nz := -(polyb[i*3] - polyb[j*3])
		amin, amax := dtProjectPoly(nx, nz, polya, npolya)
		bmin, bmax := dtProjectPoly(nx, nz, polyb, npolyb)
		if !overlapRange(amin, amax, bmin, bmax, EPS) {
			return false
		}
	}
	return true
}
