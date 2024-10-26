// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//  1. The origin of this software must not be misrepresented; you must not
//     claim that you wrote the original software. If you use this software
//     in a product, an acknowledgment in the product documentation would be
//     appreciated but is not required.
//  2. Altered source versions must be plainly marked as such, and must not be
//     misrepresented as being the original software.
//  3. This notice may not be removed or altered from any source distribution.
package dtcrowd

import (
	"math"

	detour "github.com/o0olele/detour-go/Detour"
)

func (this *DtLocalBoundary) Reset() {
	detour.DtVset(this.m_center[:], math.MaxFloat32, math.MaxFloat32, math.MaxFloat32)
	this.m_npolys = 0
	this.m_nsegs = 0
}

func (this *DtLocalBoundary) addSegment(dist float32, s []float32) {
	// Insert neighbour based on the distance.
	var seg *BoundarySegment
	if this.m_nsegs <= 0 {
		// First, trivial accept.
		seg = &this.m_segs[0]
	} else if dist >= this.m_segs[this.m_nsegs-1].d {
		// Further than the last segment, skip.
		if this.m_nsegs >= MAX_LOCAL_SEGS {
			return
		}
		// Last, trivial accept.
		seg = &this.m_segs[this.m_nsegs]
	} else {
		// Insert inbetween.
		var i int
		for i = 0; i < this.m_nsegs; i += 1 {
			if dist <= this.m_segs[i].d {
				break
			}
		}

		tgt := i + 1
		n := detour.DtMinInt(this.m_nsegs-i, MAX_LOCAL_SEGS-tgt)
		detour.DtAssert(tgt+n <= MAX_LOCAL_SEGS)
		if n > 0 {
			copy(this.m_segs[tgt:], this.m_segs[i:i+n])
			//memmove(&m_segs[tgt], &m_segs[i], sizeof(Segment)*n);
		}

		seg = &this.m_segs[i]
	}

	seg.d = dist
	copy(seg.s[0:], s[:])

	if this.m_nsegs < MAX_LOCAL_SEGS {
		this.m_nsegs += 1
	}

}

func (this *DtLocalBoundary) Update(ref detour.DtPolyRef, pos []float32, collisionQueryRange float32,
	navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) {

	const MAX_SEGS_PER_POLY int32 = detour.DT_VERTS_PER_POLYGON * 3

	if ref <= 0 {
		this.Reset()
		return
	}

	detour.DtVcopy(this.m_center[:], pos)

	navquery.FindLocalNeighbourhood(ref, pos, collisionQueryRange, filter, this.m_polys[:], nil,
		&this.m_npolys, int(MAX_LOCAL_POLYS))

	this.m_nsegs = 0

	var segs [MAX_SEGS_PER_POLY * 6]float32
	var nsegs int
	for j := 0; j < this.m_npolys; j += 1 {
		navquery.GetPolyWallSegments(this.m_polys[j], filter, segs[:], nil, &nsegs, int(MAX_SEGS_PER_POLY))
		for k := 0; k < nsegs; k += 1 {
			s := segs[k*6:]
			var tseg float32
			var distSqr = detour.DtDistancePtSegSqr2D(pos, s, s[3:], &tseg)
			if distSqr > detour.DtSqrFloat32(collisionQueryRange) {
				continue
			}
			this.addSegment(distSqr, s)
		}
	}
}

func (this *DtLocalBoundary) IsValid(navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) bool {

	if this.m_npolys <= 0 {
		return false
	}

	for i := 0; i < this.m_npolys; i += 1 {
		if !navquery.IsValidPolyRef(this.m_polys[i], filter) {
			return false
		}
	}
	return true
}
