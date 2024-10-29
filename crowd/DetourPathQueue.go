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

const DT_PATHQ_INVALID DtPathQueueRef = 0

type DtPathQueueRef uint32

type PathQuery struct {
	ref DtPathQueueRef
	/// Path find start and end location.
	startPos [3]float32
	endPos   [3]float32
	startRef detour.DtPolyRef
	endRef   detour.DtPolyRef
	/// Result.
	path  []detour.DtPolyRef
	npath int
	/// State.
	status    detour.DtStatus
	keepAlive int
	filter    *detour.DtQueryFilter ///< TODO: This is potentially dangerous!
}

const MAX_QUEUE int = 8

type DtPathQueue struct {
	m_queue       [MAX_QUEUE]PathQuery
	m_nextHandle  DtPathQueueRef
	m_maxPathSize int
	m_queueHead   int
	m_navquery    *detour.DtNavMeshQuery
}

func (this *DtPathQueue) GetNavQuery() *detour.DtNavMeshQuery {
	return this.m_navquery
}
