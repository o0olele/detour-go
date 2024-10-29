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

import (
	detour "github.com/o0olele/detour-go/detour"
)

const MAX_ITERS_PER_UPDATE = 100

const MAX_PATHQUEUE_NODES = 4096
const MAX_COMMON_NODES = 512

func tween(t, t0, t1 float32) float32 {
	return detour.DtClampFloat32((t-t0)/(t1-t0), 0, 1)
}

func integrate(ag *DtCrowdAgent, dt float32) {
	// Fake dynamic constraint.
	var maxDelta = ag.params.maxAcceleration * dt
	var dv [3]float32
	detour.DtVsub(dv[:], ag.nvel[:], ag.vel[:])
	var ds = detour.DtVlen(dv[:])
	if ds > maxDelta {
		detour.DtVscale(dv[:], dv[:], maxDelta/ds)
	}
	detour.DtVadd(ag.vel[:], ag.vel[:], dv[:])

	// Integrate
	if detour.DtVlen(ag.vel[:]) > 0.0001 {
		detour.DtVmad(ag.npos[:], ag.npos[:], ag.vel[:], dt)
	} else {
		detour.DtVset(ag.vel[:], 0, 0, 0)
	}
}

func overOffmeshConnection(ag *DtCrowdAgent, radius float32) bool {
	if ag.ncorners <= 0 {
		return false
	}

	var offMeshConnection bool
	if ag.cornerFlags[ag.ncorners-1]&detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION > 0 {
		offMeshConnection = true
	}
	if offMeshConnection {
		var distSq = detour.DtVdist2DSqr(ag.npos[:], ag.cornerVerts[(ag.ncorners-1)*3:])
		if distSq < radius*radius {
			return true
		}
	}
	return false
}

func getDistanceToGoal(ag *DtCrowdAgent, ranged float32) float32 {
	if ag.ncorners <= 0 {
		return ranged
	}

	var endOfPath = ag.cornerFlags[ag.ncorners-1]&detour.DT_STRAIGHTPATH_END > 0
	if endOfPath {
		return detour.DtMinFloat32(detour.DtVdist2D(ag.npos[:], ag.cornerVerts[(ag.ncorners-1)*3:]), ranged)
	}
	return ranged
}

func calcSmoothSteerDirection(ag *DtCrowdAgent, dir []float32) {
	if ag.ncorners <= 0 {
		detour.DtVset(dir, 0, 0, 0)
		return
	}

	var ip0 int
	var ip1 = detour.DtMinInt(1, ag.ncorners-1)
	var p0 = ag.cornerVerts[ip0*3:]
	var p1 = ag.cornerVerts[ip1*3:]

	var dir0, dir1 [3]float32
	detour.DtVsub(dir0[:], p0, ag.npos[:])
	detour.DtVsub(dir1[:], p1, ag.npos[:])
	dir0[1] = 0
	dir1[1] = 0

	var len0 = detour.DtVlen(dir0[:])
	var len1 = detour.DtVlen(dir1[:])
	if len1 > 0.001 {
		detour.DtVscale(dir1[:], dir1[:], 1/len1)
	}

	dir[0] = dir0[0] - dir1[0]*len0*0.5
	dir[1] = 0
	dir[2] = dir0[2] - dir1[2]*len0*0.5

	detour.DtVnormalize(dir)
}

func calcStraightSteerDirection(ag *DtCrowdAgent, dir []float32) {
	if ag.ncorners <= 0 {
		detour.DtVset(dir, 0, 0, 0)
		return
	}
	detour.DtVsub(dir, ag.cornerVerts[:], ag.npos[:])
	dir[1] = 0
	detour.DtVnormalize(dir)
}

func addNeighbour(idx int, dist float32, neis []DtCrowdNeighbour, nneis int, maxNeis int) int {
	// Insert neighbour based on the distance.
	var nei *DtCrowdNeighbour
	if nneis <= 0 {
		nei = &neis[nneis]
	} else if dist >= neis[nneis-1].dist {
		if nneis >= maxNeis {
			return nneis
		}
		nei = &neis[nneis]
	} else {
		var i int
		for ; i < nneis; i += 1 {
			if dist <= neis[i].dist {
				break
			}
		}
		var tgt = i + 1
		var n = detour.DtMinInt(nneis-i, maxNeis-tgt)

		detour.DtAssert(tgt+n <= maxNeis)

		if n > 0 {
			copy(neis[tgt:], neis[i:i+n])
		}
		nei = &neis[i]
	}

	nei.dist = 0
	nei.idx = 0

	return detour.DtMinInt(nneis+1, maxNeis)
}

func getNeighbours(pos []float32, height, ranged float32, skip *DtCrowdAgent, result []DtCrowdNeighbour, maxResult int,
	agents []*DtCrowdAgent, _ int, grid *DtProximityGrid) int {
	const MAX_NEIS = 32

	var n int
	var ids [MAX_NEIS]uint16
	var nids = grid.queryItems(pos[0]-ranged, pos[2]-ranged,
		pos[0]+ranged, pos[2]+ranged,
		ids[:], MAX_NEIS)

	for i := 0; i < nids; i += 1 {
		var ag = agents[ids[i]]
		if ag == skip {
			continue
		}

		// Check for overlap.
		var diff [3]float32
		detour.DtVsub(diff[:], pos, ag.npos[:])
		if detour.DtMathFabsf(diff[1]) >= (height+ag.params.height)/2 {
			continue
		}
		diff[1] = 0
		var distSqr = detour.DtVlenSqr(diff[:])
		if distSqr > detour.DtSqrFloat32(ranged) {
			continue
		}

		n = addNeighbour(int(ids[i]), distSqr, result, n, maxResult)
	}
	return n
}

func addToOptQueue(newag *DtCrowdAgent, agents []*DtCrowdAgent, nagents int, maxAgents int) int {
	// Insert neighbour based on greatest time.
	var slot int
	if nagents <= 0 {
		slot = nagents
	} else if newag.topologyOptTime <= agents[nagents-1].topologyOptTime {
		if nagents >= maxAgents {
			return nagents
		}
		slot = nagents
	} else {
		var i int
		for ; i < nagents; i += 1 {
			if newag.topologyOptTime >= agents[i].topologyOptTime {
				break
			}
		}
		var tgt = i + 1
		var n = detour.DtMinInt(nagents-i, maxAgents-tgt)

		detour.DtAssert(tgt+n <= maxAgents)

		if n > 0 {
			copy(agents[tgt:], agents[i:i+n])
		}
		slot = i
	}
	agents[slot] = newag

	return detour.DtMinInt(nagents+1, maxAgents)
}

func addToPathQueue(newag *DtCrowdAgent, agents []*DtCrowdAgent, nagents int, maxAgents int) int {
	// Insert neighbour based on greatest time.
	var slot int
	if nagents <= 0 {
		slot = nagents
	} else if newag.targetReplanTime <= agents[nagents-1].targetReplanTime {
		if nagents >= maxAgents {
			return nagents
		}
		slot = nagents
	} else {
		var i int
		for ; i < nagents; i += 1 {
			if newag.targetReplanTime >= agents[i].targetReplanTime {
				break
			}
		}
		var tgt = i + 1
		var n = detour.DtMinInt(nagents-i, maxAgents-tgt)

		detour.DtAssert(tgt+n <= maxAgents)
		if n > 0 {
			copy(agents[tgt:], agents[i:i+n])
		}
		slot = i
	}
	agents[slot] = newag

	return detour.DtMinInt(nagents+1, maxAgents)
}

func (this *DtCrowd) purge() {
	this.m_agents = nil
	this.m_maxAgents = 0
	this.m_activeAgents = nil
	this.m_agentAnims = nil
	this.m_pathResult = nil
	this.m_grid = nil
	this.m_obstacleQuery = nil
	this.m_navquery = nil
}

func (this *DtCrowd) Init(maxAgents int, maxAgentRadius float32, nav *detour.DtNavMesh) bool {
	this.purge()

	this.m_maxAgents = maxAgents
	this.m_maxAgentRadius = maxAgentRadius

	// Larger than agent radius because it is also used for agent recovery.
	detour.DtVset(this.m_agentPlacementHalfExtents[:], this.m_maxAgentRadius*2, this.m_maxAgentRadius*1.5, this.m_maxAgentRadius*2)

	this.m_grid = DtAllocProximityGrid()
	if !this.m_grid.init(this.m_maxAgents*4, maxAgentRadius*3) {
		return false
	}

	this.m_obstacleQuery = DtAllocObstacleAvoidanceQuery()
	if !this.m_obstacleQuery.init(6, 8) {
		return false
	}

	// Init obstacle query params.
	for i := 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; i += 1 {
		var params = &this.m_obstacleQueryParams[i]
		params.velBias = 0.4
		params.weightDesVel = 2.0
		params.weightCurVel = 0.75
		params.weightSide = 0.75
		params.weightToi = 2.5
		params.horizTime = 2.5
		params.gridSize = 33
		params.adaptiveDivs = 7
		params.adaptiveRings = 2
		params.adaptiveDepth = 5
	}

	// Allocate temp buffer for merging paths.
	this.m_maxPathResult = 256
	this.m_pathResult = make([]detour.DtPolyRef, this.m_maxPathResult)

	if !this.m_pathq.Init(this.m_maxPathResult, MAX_PATHQUEUE_NODES, nav) {
		return false
	}

	this.m_agents = make([]DtCrowdAgent, this.m_maxAgents)
	this.m_activeAgents = make([]*DtCrowdAgent, this.m_maxAgents)
	this.m_agentAnims = make([]DtCrowdAgentAnimation, this.m_maxAgents)

	for i := 0; i < this.m_maxAgents; i += 1 {
		this.m_agents[i].active = false
		if !this.m_agents[i].corridor.Init(this.m_maxPathResult) {
			return false
		}
	}

	for i := 0; i < this.m_maxAgents; i += 1 {
		this.m_agentAnims[i].active = false
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	this.m_navquery = detour.DtAllocNavMeshQuery()
	if detour.DtStatusFailed(this.m_navquery.Init(nav, MAX_COMMON_NODES)) {
		return false
	}

	return true
}

func (this *DtCrowd) SetObstacleAvoidanceParams(idx int, params *DtObstacleAvoidanceParams) {
	if idx >= 0 && idx <= DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS {
		this.m_obstacleQueryParams[idx] = *params
	}
}

func (this *DtCrowd) GetObstacleAvoidanceParams(idx int) *DtObstacleAvoidanceParams {
	if idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS {
		return &this.m_obstacleQueryParams[idx]
	}
	return nil
}

func (this *DtCrowd) GetAgentCount() int {
	return this.m_maxAgents
}

// / @par
// /
// / Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
func (this *DtCrowd) GetAgent(idx int) *DtCrowdAgent {
	if idx < 0 || idx >= this.m_maxAgents {
		return nil
	}
	return &this.m_agents[idx]
}

// /
// / Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
func (this *DtCrowd) GetEditableAgent(idx int) *DtCrowdAgent {
	if idx < 0 || idx >= this.m_maxAgents {
		return nil
	}
	return &this.m_agents[idx]
}

func (this *DtCrowd) UpdateAgentParameters(idx int, params *DtCrowdAgentParams) {
	if idx < 0 || idx >= this.m_maxAgents {
		return
	}
	this.m_agents[idx].params = *params
}

// / @par
// /
// / The agent's position will be constrained to the surface of the navigation mesh.
func (this *DtCrowd) AddAgent(pos []float32, params *DtCrowdAgentParams) int {
	// Find empty slot.
	var idx = -1
	for i := 0; i < this.m_maxAgents; i += 1 {
		if !this.m_agents[i].active {
			idx = i
			break
		}
	}
	if idx == -1 {
		return -1
	}

	var ag = &this.m_agents[idx]

	this.UpdateAgentParameters(idx, params)

	// Find nearest position on navmesh and place the agent there.
	var nearest [3]float32
	var ref detour.DtPolyRef
	detour.DtVcopy(nearest[:], pos)
	var status = this.m_navquery.FindNearestPoly(pos, this.m_agentPlacementHalfExtents[:],
		&this.m_filters[ag.params.queryFilterType], &ref, nearest[:])
	if detour.DtStatusFailed(status) {
		detour.DtVcopy(nearest[:], pos)
		ref = 0
	}

	ag.corridor.Reset(ref, nearest[:])
	ag.boundary.Reset()
	ag.partial = false

	ag.topologyOptTime = 0
	ag.targetReplanTime = 0
	ag.nneis = 0

	detour.DtVset(ag.dvel[:], 0, 0, 0)
	detour.DtVset(ag.nvel[:], 0, 0, 0)
	detour.DtVset(ag.vel[:], 0, 0, 0)
	detour.DtVcopy(ag.npos[:], nearest[:])

	ag.desiredSpeed = 0

	if ref > 0 {
		ag.state = DT_CROWDAGENT_STATE_WALKING
	} else {
		ag.state = DT_CROWDAGENT_STATE_INVALID
	}

	ag.active = true

	return idx
}

// / @par
// /
// / The agent is deactivated and will no longer be processed.  Its #dtCrowdAgent object
// / is not removed from the pool.  It is marked as inactive so that it is available for reuse.
func (this *DtCrowd) RemoveAgent(idx int) {
	if idx >= 0 && idx < this.m_maxAgents {
		this.m_agents[idx].active = false
	}
}

func (this *DtCrowd) requestMoveTargetReplan(idx int, ref detour.DtPolyRef, pos []float32) bool {
	if idx < 0 || idx >= this.m_maxAgents {
		return false
	}

	var ag = &this.m_agents[idx]

	// Initialize request.
	ag.targetRef = ref
	detour.DtVcopy(ag.targetPos[:], pos)
	ag.targetPathqRef = DT_PATHQ_INVALID
	ag.targetReplan = true
	if ag.targetRef > 0 {
		ag.targetState = DT_CROWDAGENT_TARGET_REQUESTING
	} else {
		ag.targetState = DT_CROWDAGENT_TARGET_FAILED
	}

	return true
}

// / @par
// /
// / This method is used when a new target is set.
// /
// / The position will be constrained to the surface of the navigation mesh.
// /
// / The request will be processed during the next #update().
func (this *DtCrowd) RequestMoveTarget(idx int, ref detour.DtPolyRef, pos []float32) bool {
	if idx < 0 || idx >= this.m_maxAgents {
		return false
	}
	if ref <= 0 {
		return false
	}

	var ag = &this.m_agents[idx]

	// Initialize request.
	ag.targetRef = ref
	detour.DtVcopy(ag.targetPos[:], pos)
	ag.targetPathqRef = DT_PATHQ_INVALID
	ag.targetReplan = false
	if ag.targetRef > 0 {
		ag.targetState = DT_CROWDAGENT_TARGET_REQUESTING
	} else {
		ag.targetState = DT_CROWDAGENT_TARGET_FAILED
	}

	return true
}

func (this *DtCrowd) RequestMoveVelocity(idx int, vel []float32) bool {
	if idx < 0 || idx >= this.m_maxAgents {
		return false
	}

	var ag = &this.m_agents[idx]
	// Initialize request.
	ag.targetRef = 0
	detour.DtVcopy(ag.targetPos[:], vel)
	ag.targetPathqRef = DT_PATHQ_INVALID
	ag.targetReplan = false
	ag.targetState = DT_CROWDAGENT_TARGET_VELOCITY

	return true
}

func (this *DtCrowd) ResetMoveTargetIdx(idx int) bool {
	if idx < 0 || idx >= this.m_maxAgents {
		return false
	}

	var ag = &this.m_agents[idx]
	// Initialize request.
	ag.targetRef = 0
	detour.DtVset(ag.targetPos[:], 0, 0, 0)
	detour.DtVset(ag.dvel[:], 0, 0, 0)
	ag.targetPathqRef = DT_PATHQ_INVALID
	ag.targetReplan = false
	ag.targetState = DT_CROWDAGENT_TARGET_NONE

	return true
}

func (this *DtCrowd) GetActiveAgents(agents []*DtCrowdAgent, maxAgents int) int {
	var n int
	for i := 0; i < this.m_maxAgents; i += 1 {
		if !this.m_agents[i].active {
			continue
		}
		if n < maxAgents {
			agents[n] = &this.m_agents[i]
			n += 1
		}
	}
	return n
}

func (this *DtCrowd) updateMoveRequest(_ float32) {
	const PATH_MAX_AGENTS = 8
	var queue = make([]*DtCrowdAgent, PATH_MAX_AGENTS)
	var nqueue int

	for i := 0; i < this.m_maxAgents; i += 1 {
		var ag = &this.m_agents[i]
		if !ag.active {
			continue
		}
		if ag.state == DT_CROWDAGENT_STATE_INVALID {
			continue
		}
		if ag.targetState == DT_CROWDAGENT_TARGET_NONE || ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
			continue
		}

		if ag.targetState == DT_CROWDAGENT_TARGET_REQUESTING {
			var path = ag.corridor.GetPath()
			var npath = ag.corridor.GetPathCount()
			detour.DtAssert(npath > 0)

			const MAX_RES = 32
			var reqPos [3]float32
			var reqPath [MAX_RES]detour.DtPolyRef
			var reqPathCount int
			var doneIters int
			const MAX_ITER = 20
			this.m_navquery.InitSlicedFindPath(path[0], ag.targetRef, ag.npos[:], ag.targetPos[:], &this.m_filters[ag.params.queryFilterType], 0)
			this.m_navquery.UpdateSlicedFindPath(MAX_ITER, &doneIters)

			var status detour.DtStatus
			if ag.targetReplan {
				status = this.m_navquery.FinalizeSlicedFindPathPartial(path, npath, reqPath[:], &reqPathCount, MAX_RES)
			} else {
				status = this.m_navquery.FinalizeSlicedFindPath(reqPath[:], &reqPathCount, MAX_RES)
			}

			if !detour.DtStatusFailed(status) && reqPathCount > 0 {
				if reqPath[reqPathCount-1] != ag.targetRef {
					var posOverPoly bool
					status = this.m_navquery.ClosestPointOnPoly(reqPath[reqPathCount-1], ag.targetPos[:], reqPos[:], &posOverPoly)
					if detour.DtStatusFailed(status) {
						reqPathCount = 0
					}
				} else {
					detour.DtVcopy(reqPos[:], ag.targetPos[:])
				}
			} else {
				reqPathCount = 0
			}

			if reqPathCount <= 0 {
				detour.DtVcopy(reqPos[:], ag.npos[:])
				reqPath[0] = path[0]
				reqPathCount = 0
			}

			ag.corridor.SetCorridor(reqPos[:], reqPath[:], reqPathCount)
			ag.boundary.Reset()
			ag.partial = false

			if reqPath[reqPathCount-1] == ag.targetRef {
				ag.targetState = DT_CROWDAGENT_TARGET_VALID
				ag.targetReplanTime = 0
			} else {
				ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE
			}
		}

		if ag.targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE {
			nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS)
		}
	}

	for i := 0; i < nqueue; i += 1 {
		var ag = queue[i]
		ag.targetPathqRef = this.m_pathq.Request(ag.corridor.GetLastPoly(), ag.targetRef,
			ag.corridor.GetTarget(), ag.targetPos[:], &this.m_filters[ag.params.queryFilterType])
		if ag.targetPathqRef != DT_PATHQ_INVALID {
			ag.targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH
		}
	}

	this.m_pathq.Update(MAX_ITERS_PER_UPDATE)

	var status detour.DtStatus
	for i := 0; i < this.m_maxAgents; i++ {
		var ag = this.m_agents[i]
		if !ag.active {
			continue
		}
		if ag.targetState == DT_CROWDAGENT_TARGET_NONE || ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
			continue
		}

		if ag.targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH {
			status = this.m_pathq.GetRequestStatus(ag.targetPathqRef)
			if detour.DtStatusFailed(status) {
				ag.targetPathqRef = DT_PATHQ_INVALID
				if ag.targetRef > 0 {
					ag.targetState = DT_CROWDAGENT_TARGET_REQUESTING
				} else {
					ag.targetState = DT_CROWDAGENT_TARGET_FAILED
				}
				ag.targetReplanTime = 0
			} else if detour.DtStatusSucceed(status) {
				var path = ag.corridor.GetPath()
				var npath = ag.corridor.GetPathCount()
				detour.DtAssert(npath > 0)

				var targetPos [3]float32
				detour.DtVcopy(targetPos[:], ag.targetPos[:])

				var res = this.m_pathResult
				var valid = true
				var nres int
				status = this.m_pathq.GetPathResult(ag.targetPathqRef, res, &nres, this.m_maxPathResult)
				if detour.DtStatusFailed(status) || nres <= 0 {
					valid = false
				}

				if detour.DtStatusDetail(status, detour.DT_PARTIAL_RESULT) {
					ag.partial = true
				} else {
					ag.partial = false
				}

				if valid && path[npath-1] != res[0] {
					valid = false
				}

				if valid {
					if npath > 1 {
						if (npath-1)+nres > this.m_maxPathResult {
							nres = this.m_maxPathResult - (npath - 1)
						}

						copy(res[npath-1:], res[:nres])
						copy(res, path[:npath-1])
						nres += npath - 1

						for j := 0; j < nres; j += 1 {
							if j-1 >= 0 && j+1 < nres {
								if res[j-1] == res[j+1] {
									copy(res[j-1:], res[j+1:nres])
									nres -= 2
									j -= 2
								}
							}
						}
					}

					if res[nres-1] != ag.targetRef {
						var nearest [3]float32
						var posOverPoly bool
						status = this.m_navquery.ClosestPointOnPoly(res[nres-1], targetPos[:], nearest[:], &posOverPoly)
						if detour.DtStatusSucceed(status) {
							detour.DtVcopy(targetPos[:], nearest[:])
						} else {
							valid = false
						}
					}
				}

				if valid {
					ag.corridor.SetCorridor(targetPos[:], res, nres)
					ag.boundary.Reset()
					ag.targetState = DT_CROWDAGENT_TARGET_VALID
				} else {
					ag.targetState = DT_CROWDAGENT_TARGET_FAILED
				}

				ag.targetReplanTime = 0
			}
		}
	}
}

func (this *DtCrowd) updateTopologyOptimization(agents []*DtCrowdAgent, nagents int, dt float32) {
	if nagents <= 0 {
		return
	}

	const OPT_TIME_THR = 0.5 // seconds
	const OPT_MAX_AGENTS = 1

	var queue [OPT_MAX_AGENTS]*DtCrowdAgent
	var nqueue int

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]
		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}
		if ag.targetState == DT_CROWDAGENT_TARGET_NONE || ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
			continue
		}
		if ag.params.updateFlags&DT_CROWD_OPTIMIZE_TOPO == 0 {
			continue
		}
		ag.topologyOptTime += dt
		if ag.topologyOptTime >= OPT_TIME_THR {
			nqueue = addToOptQueue(ag, queue[:], nqueue, OPT_MAX_AGENTS)
		}
	}

	for i := 0; i < nqueue; i += 1 {
		var ag = queue[i]
		ag.corridor.OptimizePathTopology(this.m_navquery, &this.m_filters[ag.params.queryFilterType])
		ag.topologyOptTime = 0
	}
}

func (this *DtCrowd) checkPathValidity(agents []*DtCrowdAgent, nagents int, dt float32) {
	const CHECK_LOOKAHEAD = 10
	const TARGET_REPLAN_DELAY = 1.0 // seconds

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]
		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}

		ag.targetReplanTime += dt

		var replan = false

		var idx = this.getAgentIndex(ag)
		var agentPos [3]float32
		var agentRef = ag.corridor.GetFirstPoly()
		detour.DtVcopy(agentPos[:], ag.npos[:])
		if !this.m_navquery.IsValidPolyRef(agentRef, &this.m_filters[ag.params.queryFilterType]) {
			var nearest [3]float32
			detour.DtVcopy(nearest[:], agentPos[:])
			agentRef = 0
			this.m_navquery.FindNearestPoly(ag.npos[:], this.m_agentPlacementHalfExtents[:], &this.m_filters[ag.params.queryFilterType], &agentRef, nearest[:])
			detour.DtVcopy(agentPos[:], nearest[:])

			if agentRef <= 0 {
				ag.corridor.Reset(0, agentPos[:])
				ag.partial = false
				ag.boundary.Reset()
				ag.state = DT_CROWDAGENT_STATE_INVALID
				continue
			}

			ag.corridor.FixPathStart(agentRef, agentPos[:])
			// ag.corridor.trimInvalidPath(agentRef, agentPos[:], this.m_navquery, &this.m_filters[ag.params.queryFilterType])
			ag.boundary.Reset()
			detour.DtVcopy(ag.npos[:], agentPos[:])

			replan = true
		}

		if ag.targetState == DT_CROWDAGENT_TARGET_NONE || ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
			if !this.m_navquery.IsValidPolyRef(ag.targetRef, &this.m_filters[ag.params.queryFilterType]) {
				var nearest [3]float32
				detour.DtVcopy(nearest[:], ag.targetPos[:])
				ag.targetRef = 0
				this.m_navquery.FindNearestPoly(ag.targetPos[:], this.m_agentPlacementHalfExtents[:], &this.m_filters[ag.params.queryFilterType], &ag.targetRef, nearest[:])
				detour.DtVcopy(ag.targetPos[:], nearest[:])
				replan = true
			}
			if ag.targetRef <= 0 {
				ag.corridor.Reset(agentRef, agentPos[:])
				ag.partial = false
				ag.targetState = DT_CROWDAGENT_TARGET_NONE
			}
		}

		if !ag.corridor.IsValid(CHECK_LOOKAHEAD, this.m_navquery, &this.m_filters[ag.params.queryFilterType]) {
			replan = true
		}

		if ag.targetState == DT_CROWDAGENT_TARGET_VALID {
			if ag.targetReplanTime > TARGET_REPLAN_DELAY &&
				ag.corridor.GetPathCount() < CHECK_LOOKAHEAD &&
				ag.corridor.GetLastPoly() != ag.targetRef {
				replan = true
			}
		}

		if replan {
			if ag.targetState != DT_CROWDAGENT_TARGET_NONE {
				this.requestMoveTargetReplan(idx, ag.targetRef, ag.targetPos[:])
			}
		}
	}

}

func (this *DtCrowd) Update(dt float32, debug *DtCrowdAgentDebugInfo) {
	this.m_velocitySampleCount = 0

	var debugIdx = -1
	if debug != nil {
		debugIdx = debug.idx
	}

	var agents = this.m_activeAgents
	var nagents = this.GetActiveAgents(agents, this.m_maxAgents)

	this.checkPathValidity(agents, nagents, dt)

	this.updateMoveRequest(dt)

	this.updateTopologyOptimization(agents, nagents, dt)

	this.m_grid.clear()
	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]
		var p = ag.npos[0:]
		var r = ag.params.radius
		this.m_grid.addItem(uint16(i), p[0]-r, p[2]-r, p[0]+r, p[2]+r)
	}

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]
		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}

		var updateThr = ag.params.collisionQueryRange * 0.25
		if detour.DtVdist2DSqr(ag.npos[:], ag.boundary.GetCenter()) > detour.DtSqrFloat32(updateThr) ||
			!ag.boundary.IsValid(this.m_navquery, &this.m_filters[ag.params.queryFilterType]) {
			ag.boundary.Update(ag.corridor.GetFirstPoly(), ag.npos[:], ag.params.collisionQueryRange,
				this.m_navquery, &this.m_filters[ag.params.queryFilterType])
		}

		ag.nneis = getNeighbours(ag.npos[:], ag.params.height, ag.params.collisionQueryRange,
			ag, ag.neis[:], DT_CROWDAGENT_MAX_NEIGHBOURS,
			agents, nagents, this.m_grid)
		for j := 0; j < ag.nneis; j += 1 {
			ag.neis[j].idx = this.getAgentIndex(ag)
		}
	}

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]

		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}
		if ag.targetState == DT_CROWDAGENT_TARGET_NONE || ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
			continue
		}

		ag.ncorners = ag.corridor.FindCorners(ag.cornerVerts[:], ag.cornerFlags[:], ag.cornerPolys[:],
			DT_CROWDAGENT_MAX_CORNERS, this.m_navquery, &this.m_filters[ag.params.queryFilterType])

		if (ag.params.updateFlags&DT_CROWD_OPTIMIZE_VIS > 0) && ag.ncorners > 0 {
			var target = ag.cornerVerts[detour.DtMinInt(1, ag.ncorners-1)*3:]
			ag.corridor.OptimizePathVisibility(target, ag.params.pathOptimizationRange, this.m_navquery, &this.m_filters[ag.params.queryFilterType])

			if debugIdx == i {
				detour.DtVcopy(debug.optStart[:], ag.corridor.GetPos())
				detour.DtVcopy(debug.optEnd[:], target)
			}
		} else {
			if debugIdx == i {
				detour.DtVset(debug.optStart[:], 0, 0, 0)
				detour.DtVset(debug.optEnd[:], 0, 0, 0)
			}
		}
	}

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]

		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}
		if ag.targetState == DT_CROWDAGENT_TARGET_NONE || ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
			continue
		}

		var triggerRadius = ag.params.radius * 2.25
		if overOffmeshConnection(ag, triggerRadius) {
			var idx = this.getAgentIndex(ag)
			var anim = &this.m_agentAnims[idx]

			var refs [2]detour.DtPolyRef
			if ag.corridor.MoveOverOffmeshConnection(ag.cornerPolys[ag.ncorners-1], refs[:], anim.startPos[:], anim.endPos[:], this.m_navquery) {
				detour.DtVcopy(anim.initPos[:], ag.npos[:])
				anim.polyRef = refs[1]
				anim.active = true
				anim.t = 0
				anim.tmax = detour.DtVdist2D(anim.startPos[:], anim.endPos[:]) / ag.params.maxSpeed * 0.5

				ag.state = DT_CROWDAGENT_STATE_OFFMESH
				ag.ncorners = 0
				ag.nneis = 0
				continue
			} else {

			}
		}

	}

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]

		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}
		if ag.targetState == DT_CROWDAGENT_TARGET_NONE {
			continue
		}

		var dvel [3]float32

		if ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
			detour.DtVcopy(dvel[:], ag.targetPos[:])
			ag.desiredSpeed = detour.DtVlen(ag.targetPos[:])
		} else {
			if ag.params.updateFlags&DT_CROWD_ANTICIPATE_TURNS > 0 {
				calcSmoothSteerDirection(ag, dvel[:])
			} else {
				calcStraightSteerDirection(ag, dvel[:])
			}

			var slowDownRadius = ag.params.radius * 2
			var speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius

			ag.desiredSpeed = ag.params.maxSpeed
			detour.DtVscale(dvel[:], dvel[:], ag.desiredSpeed*speedScale)
		}

		if ag.params.updateFlags&DT_CROWD_SEPARATION > 0 {
			var separationDist = ag.params.collisionQueryRange
			var invSeparationDist = 1.0 / separationDist
			var separationWeight = ag.params.separationWeight

			var w float32
			var disp [3]float32

			for j := 0; j < ag.nneis; j += 1 {
				var nei = this.m_agents[ag.neis[j].idx]
				var diff [3]float32
				detour.DtVsub(diff[:], ag.npos[:], nei.npos[:])
				diff[1] = 0

				var distSqr = detour.DtVlenSqr(diff[:])
				if distSqr < 0.00001 {
					continue
				}
				if distSqr > detour.DtSqrFloat32(separationDist) {
					continue
				}
				var dist = detour.DtMathSqrtf(distSqr)
				var weight = separationWeight * (1 - detour.DtSqrFloat32(dist*invSeparationDist))

				detour.DtVmad(disp[:], disp[:], diff[:], weight/dist)
				w += 1
			}

			if w > 0.0001 {
				detour.DtVmad(dvel[:], dvel[:], disp[:], 1/w)

				var speedSqr = detour.DtVlenSqr(dvel[:])
				var desiredSqr = detour.DtSqrFloat32(ag.desiredSpeed)
				if speedSqr > desiredSqr {
					detour.DtVscale(dvel[:], dvel[:], desiredSqr/speedSqr)
				}
			}
		}

		detour.DtVcopy(ag.dvel[:], dvel[:])
	}

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]

		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}

		if ag.params.updateFlags&DT_CROWD_OBSTACLE_AVOIDANCE > 0 {
			this.m_obstacleQuery.reset()

			for j := 0; j < ag.nneis; j += 1 {
				var nei = &this.m_agents[ag.neis[j].idx]
				this.m_obstacleQuery.addCircle(nei.npos[:], nei.params.radius, nei.vel[:], nei.dvel[:])
			}

			for j := 0; j < ag.boundary.GetSegmentCount(); j += 1 {
				var s = ag.boundary.GetSegment(j)
				if detour.DtTriArea2D(ag.npos[:], s, s[3:]) < 0 {
					continue
				}
				this.m_obstacleQuery.addSegment(s, s[3:])
			}

			var vod *DtObstacleAvoidanceDebugData
			if debugIdx == i {
				vod = debug.vod
			}

			var adaptive = true
			var ns = 0
			var params = &this.m_obstacleQueryParams[ag.params.obstacleAvoidanceType]

			if adaptive {
				ns = this.m_obstacleQuery.sampleVelocityAdaptive(ag.npos[:], ag.params.radius, ag.desiredSpeed,
					ag.vel[:], ag.dvel[:], ag.nvel[:], params, vod)
			} else {
				ns = this.m_obstacleQuery.sampleVelocityGrid(ag.npos[:], ag.params.radius, ag.desiredSpeed,
					ag.vel[:], ag.dvel[:], ag.nvel[:], params, vod)
			}
			this.m_velocitySampleCount += ns
		} else {
			detour.DtVcopy(ag.nvel[:], ag.dvel[:])
		}
	}

	for i := 0; i < nagents; i += 1 {
		var ag = agents[i]
		if ag.state != DT_CROWDAGENT_STATE_WALKING {
			continue
		}
		integrate(ag, dt)
	}

	const COLLISION_RESOLVE_FACTOR = 0.7

	for iter := 0; iter < 4; iter += 1 {
		for i := 0; i < nagents; i += 1 {
			var ag = agents[i]
			if ag.state != DT_CROWDAGENT_STATE_WALKING {
				continue
			}

			var idx0 = this.getAgentIndex(ag)

			detour.DtVset(ag.disp[:], 0, 0, 0)

			var w float32
			for j := 0; j < ag.nneis; j += 1 {
				var nei = &this.m_agents[ag.neis[j].idx]
				var idx1 = this.getAgentIndex(nei)

				var diff [3]float32
				detour.DtVsub(diff[:], ag.npos[:], nei.npos[:])
				diff[1] = 0

				var dist = detour.DtVlenSqr(diff[:])
				if dist > detour.DtSqrFloat32(ag.params.radius+nei.params.radius) {
					continue
				}

				dist = detour.DtMathSqrtf(dist)
				var pen = (ag.params.radius + nei.params.radius) - dist
				if dist < 0.0001 {
					if idx0 > idx1 {
						detour.DtVset(diff[:], -ag.dvel[2], 0, ag.dvel[0])
					} else {
						detour.DtVset(diff[:], ag.dvel[2], 0, -ag.dvel[0])
					}
					pen = 0.01
				} else {
					pen = (1 / dist) * (pen * 0.5) * COLLISION_RESOLVE_FACTOR
				}

				detour.DtVmad(ag.disp[:], ag.disp[:], diff[:], pen)
				w += 1
			}

			if w > 0.0001 {
				var iw = 1 / w
				detour.DtVscale(ag.disp[:], ag.disp[:], iw)
			}
		}

		for i := 0; i < nagents; i += 1 {
			var ag = agents[i]
			if ag.state != DT_CROWDAGENT_STATE_WALKING {
				continue
			}

			ag.corridor.MovePosition(ag.npos[:], this.m_navquery, &this.m_filters[ag.params.queryFilterType])
			detour.DtVcopy(ag.npos[:], ag.corridor.GetPos())

			if ag.targetState == DT_CROWDAGENT_TARGET_NONE || ag.targetState == DT_CROWDAGENT_TARGET_VELOCITY {
				ag.corridor.Reset(ag.corridor.GetFirstPoly(), ag.npos[:])
				ag.partial = false
			}
		}

		for i := 0; i < nagents; i += 1 {
			var ag = agents[i]
			var idx = this.getAgentIndex(ag)
			var anim = &this.m_agentAnims[idx]
			if !anim.active {
				continue
			}

			anim.t += dt
			if anim.t > anim.tmax {
				anim.active = false
				ag.state = DT_CROWDAGENT_STATE_WALKING
				continue
			}

			var ta = anim.tmax * 0.15
			var tb = anim.tmax
			if anim.t < ta {
				var u = tween(anim.t, 0, ta)
				detour.DtVlerp(ag.npos[:], anim.initPos[:], anim.startPos[:], u)
			} else {
				var u = tween(anim.t, ta, tb)
				detour.DtVlerp(ag.npos[:], anim.startPos[:], anim.endPos[:], u)
			}

			detour.DtVset(ag.vel[:], 0, 0, 0)
			detour.DtVset(ag.dvel[:], 0, 0, 0)
		}
	}
}
