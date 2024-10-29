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

import detour "github.com/o0olele/detour-go/detour"

const MAX_LOCAL_SEGS int = 8
const MAX_LOCAL_POLYS int = 16

type BoundarySegment struct {
	s [6]float32 ///< Segment start/end
	d float32    ///< Distance for pruning.
}

type DtLocalBoundary struct {
	m_center [3]float32
	m_segs   [MAX_LOCAL_SEGS]BoundarySegment
	m_nsegs  int

	m_polys  [MAX_LOCAL_POLYS]detour.DtPolyRef
	m_npolys int
}

func (this *DtLocalBoundary) GetCenter() []float32 {
	return this.m_center[:]
}

func (this *DtLocalBoundary) GetSegmentCount() int {
	return this.m_nsegs
}

func (this *DtLocalBoundary) GetSegment(i int) []float32 {
	return this.m_segs[i].s[:]
}
