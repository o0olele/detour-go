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

package dtcrowd

import detour "github.com/o0olele/detour-go/detour"

func (this *DtPathQueue) Init(maxPathSize, maxSearchNodeCount int, nav *detour.DtNavMesh) bool {

	this.purge()

	this.m_navquery = detour.DtAllocNavMeshQuery()
	if this.m_navquery == nil {
		return false
	}
	if detour.DtStatusFailed(this.m_navquery.Init(nav, maxSearchNodeCount)) {
		return false
	}

	this.m_maxPathSize = maxPathSize
	for i := 0; i < MAX_QUEUE; i += 1 {
		this.m_queue[i].ref = DtPathQueueRef(DT_PATHQ_INVALID)
		this.m_queue[i].path = make([]detour.DtPolyRef, this.m_maxPathSize)
		if len(this.m_queue[i].path) <= 0 {
			return false
		}
	}
	this.m_queueHead = 0
	return true
}

func (this *DtPathQueue) purge() {
	this.m_navquery = nil
	for i := 0; i < MAX_QUEUE; i += 1 {
		this.m_queue[i].filter = nil
	}
}

func (this *DtPathQueue) Update(maxIters int) {
	const MAX_KEEP_ALIVE = 2

	var iterCount = maxIters
	for i := 0; i < MAX_QUEUE; i += 1 {
		var q = &this.m_queue[this.m_queueHead%MAX_QUEUE]

		if q.ref == DtPathQueueRef(DT_PATHQ_INVALID) {
			this.m_queueHead += 1
			continue
		}

		if detour.DtStatusSucceed(q.status) || detour.DtStatusFailed(q.status) {
			q.keepAlive += 1
			if q.keepAlive > MAX_KEEP_ALIVE {
				q.ref = DtPathQueueRef(DT_PATHQ_INVALID)
				q.status = 0
			}

			this.m_queueHead += 1
			continue
		}

		if q.status == 0 {
			var option detour.DtFindPathOptions
			q.status = this.m_navquery.InitSlicedFindPath(q.startRef, q.endRef,
				q.startPos[:], q.endPos[:], q.filter, option)
		}

		if detour.DtStatusInProgress(q.status) {
			var iters int
			q.status = this.m_navquery.UpdateSlicedFindPath(iterCount, &iters)
			iterCount -= iters
		}
		if detour.DtStatusSucceed(q.status) {
			q.status = this.m_navquery.FinalizeSlicedFindPath(q.path, &q.npath, this.m_maxPathSize)
		}

		if iterCount <= 0 {
			break
		}

		this.m_queueHead += 1
	}
}

func (this *DtPathQueue) Request(startRef, endRef detour.DtPolyRef,
	startPos, endPos []float32,
	filter *detour.DtQueryFilter) DtPathQueueRef {

	var slot = -1
	for i := 0; i < MAX_QUEUE; i += 1 {
		if this.m_queue[i].ref == DtPathQueueRef(DT_PATHQ_INVALID) {
			slot = i
			break
		}
	}
	if slot < 0 {
		return DtPathQueueRef(DT_PATHQ_INVALID)
	}

	var ref = this.m_nextHandle
	this.m_nextHandle += 1
	if this.m_nextHandle == DtPathQueueRef(DT_PATHQ_INVALID) {
		this.m_nextHandle += 1
	}

	var q = &this.m_queue[slot]
	q.ref = ref
	detour.DtVcopy(q.startPos[:], startPos)
	q.startRef = startRef
	detour.DtVcopy(q.endPos[:], endPos)
	q.endRef = endRef

	q.status = 0
	q.npath = 0
	q.filter = filter
	q.keepAlive = 0

	return ref
}

func (this *DtPathQueue) GetRequestStatus(ref DtPathQueueRef) detour.DtStatus {
	for i := 0; i < MAX_QUEUE; i += 1 {
		if this.m_queue[i].ref == ref {
			return this.m_queue[i].status
		}
	}
	return detour.DT_FAILURE
}

func (this *DtPathQueue) GetPathResult(ref DtPathQueueRef, path []detour.DtPolyRef,
	pathSize *int, maxPath int) detour.DtStatus {
	for i := 0; i < MAX_QUEUE; i += 1 {
		if this.m_queue[i].ref == ref {
			var q = &this.m_queue[i]
			var details = q.status & detour.DT_STATUS_DETAIL_MASK
			// Free request for reuse.
			q.ref = DtPathQueueRef(DT_PATHQ_INVALID)
			q.status = 0
			// Copy path
			var n = detour.DtMinInt(q.npath, maxPath)
			copy(path[:], q.path[:n])
			*pathSize = n
			return details | detour.DT_SUCCESS
		}
	}
	return detour.DT_FAILURE
}
