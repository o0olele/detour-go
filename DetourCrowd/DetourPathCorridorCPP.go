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

import detour "github.com/o0olele/detour-go/Detour"

func dtMergeCorridorStartMoved(path []detour.DtPolyRef, npath, maxPath int,
	visited []detour.DtPolyRef, nvisited int) int {
	var furthestPath = -1
	var furthestVisited = -1

	// Find furthest common polygon.
	for i := npath - 1; i >= 0; i -= 1 {
		var found = false
		for j := nvisited - 1; j >= 0; j -= 1 {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.

	// Adjust beginning of the buffer to include the visited.
	var req = nvisited - furthestVisited
	var orig = detour.DtMinInt(furthestPath+1, npath)
	var size = detour.DtMaxInt(0, npath-orig)
	if req+size > maxPath {
		size = maxPath - req
	}
	if size > 0 {
		// memmove(path+req, path+orig, size*sizeof(dtPolyRef));
		copy(path[req:], path[orig:orig+size])
	}

	// Store visited
	for i := 0; i < req; i += 1 {
		path[i] = visited[(nvisited-1)-i]
	}

	return req + size
}

func dtMergeCorridorEndMoved(path []detour.DtPolyRef, npath, maxPath int,
	visited []detour.DtPolyRef, nvisited int) int {
	var furthestPath = -1
	var furthestVisited = -1

	// Find furthest common polygon.
	for i := 0; i < npath; i += 1 {
		var found = false
		for j := nvisited - 1; j >= 0; j -= 1 {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path.
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.
	var ppos = furthestPath + 1
	var vpos = furthestVisited + 1
	var count = detour.DtMinInt(nvisited-vpos, maxPath-ppos)
	detour.DtAssert(ppos+count <= maxPath)
	if count > 0 {
		// memcpy(path+ppos, visited+vpos, sizeof(dtPolyRef)*count)
		copy(path[ppos:], visited[vpos:vpos+count])
	}

	return ppos + count
}

func dtMergeCorridorStartShortcut(path []detour.DtPolyRef, npath, maxPath int,
	visited []detour.DtPolyRef, nvisited int) int {
	var furthestPath = -1
	var furthestVisited = -1

	// Find furthest common polygon.'
	for i := npath - 1; i >= 0; i -= 1 {
		var found bool
		for j := nvisited - 1; j >= 0; j -= 1 {
			if path[i] == visited[j] {
				furthestPath = i
				furthestVisited = j
				found = true
			}
		}
		if found {
			break
		}
	}

	// If no intersection found just return current path. '
	if furthestPath == -1 || furthestVisited == -1 {
		return npath
	}

	// Concatenate paths.

	// Adjust beginning of the buffer to include the visited.
	var req = furthestVisited
	if req <= 0 {
		return npath
	}

	var orig = furthestPath
	var size = detour.DtMaxInt(0, npath-orig)
	if req+size > maxPath {
		size = maxPath - req
	}
	if size > 0 {
		// memmove(path+req, path+orig, size*sizeof(dtPolyRef));
		copy(path[req:], path[orig:orig+size])
	}

	// Store visited
	for i := 0; i < req; i += 1 {
		path[i] = visited[i]
	}

	return req + size
}

/**
@class dtPathCorridor
@par

The corridor is loaded with a path, usually obtained from a #dtNavMeshQuery::findPath() query. The corridor
is then used to plan local movement, with the corridor automatically updating as needed to deal with inaccurate
agent locomotion.

Example of a common use case:

-# Construct the corridor object and call #init() to allocate its path buffer.
-# Obtain a path from a #dtNavMeshQuery object.
-# Use #reset() to set the agent's current position. (At the beginning of the path.)
-# Use #setCorridor() to load the path and target.
-# Use #findCorners() to plan movement. (This handles dynamic path straightening.)
-# Use #movePosition() to feed agent movement back into the corridor. (The corridor will automatically adjust as needed.)
-# If the target is moving, use #moveTargetPosition() to update the end of the corridor.
   (The corridor will automatically adjust as needed.)
-# Repeat the previous 3 steps to continue to move the agent.

The corridor position and target are always constrained to the navigation mesh.

One of the difficulties in maintaining a path is that floating point errors, locomotion inaccuracies, and/or local
steering can result in the agent crossing the boundary of the path corridor, temporarily invalidating the path.
This class uses local mesh queries to detect and update the corridor as needed to handle these types of issues.

The fact that local mesh queries are used to move the position and target locations results in two beahviors that
need to be considered:

Every time a move function is used there is a chance that the path will become non-optimial. Basically, the further
the target is moved from its original location, and the further the position is moved outside the original corridor,
the more likely the path will become non-optimal. This issue can be addressed by periodically running the
#optimizePathTopology() and #optimizePathVisibility() methods.

All local mesh queries have distance limitations. (Review the #dtNavMeshQuery methods for details.) So the most accurate
use case is to move the position and target in small increments. If a large increment is used, then the corridor
may not be able to accurately find the new location.  Because of this limiation, if a position is moved in a large
increment, then compare the desired and resulting polygon references. If the two do not match, then path replanning
may be needed.  E.g. If you move the target, check #getLastPoly() to see if it is the expected polygon.

*/

// / @par
// /
// / @warning Cannot be called more than once.
func (this *DtPathCorridor) Init(maxPath int) bool {

	detour.DtAssert(len(this.m_path) <= 0)
	this.m_path = make([]detour.DtPolyRef, maxPath)

	this.m_npath = 0
	this.m_maxPath = maxPath

	return true
}

// / @par
// /
// / Essentially, the corridor is set of one polygon in size with the target
// / equal to the position.
func (this *DtPathCorridor) Reset(ref detour.DtPolyRef, pos []float32) {
	detour.DtAssert(len(this.m_path) > 0)

	detour.DtVcopy(this.m_pos[:], pos)
	detour.DtVcopy(this.m_target[:], pos)
	this.m_path[0] = ref
	this.m_npath = 1
}

/*
*
@par

This is the function used to plan local movement within the corridor. One or more corners can be
detected in order to plan movement. It performs essentially the same function as #dtNavMeshQuery::findStraightPath.

Due to internal optimizations, the maximum number of corners returned will be (@p maxCorners - 1)
For example: If the buffers are sized to hold 10 corners, the function will never return more than 9 corners.
So if 10 corners are needed, the buffers should be sized for 11 corners.

If the target is within range, it will be the last corner and have a polygon reference id of zero.
*/
func (this *DtPathCorridor) FindCorners(cornerVerts []float32, cornerFlags []detour.DtStraightPathFlags,
	cornerPolys []detour.DtPolyRef, maxCorners int,
	navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) int {

	detour.DtAssert(len(this.m_path) > 0)
	detour.DtAssert(this.m_npath > 0)

	const MIN_TARGET_DIST = 0.01

	var ncorners int
	var option detour.DtStraightPathOptions
	navquery.FindStraightPath(this.m_pos[:], this.m_target[:], this.m_path, this.m_npath,
		cornerVerts, cornerFlags, cornerPolys, &ncorners, maxCorners, option)

	// Prune points in the beginning of the path which are too close.
	for ncorners > 0 {
		if cornerFlags[0]&detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION > 0 ||
			detour.DtVdist2DSqr(cornerVerts[0:], this.m_pos[:]) > detour.DtSqrFloat32(MIN_TARGET_DIST) {
			break
		}
		ncorners -= 1
		if ncorners > 0 {
			copy(cornerFlags[0:], cornerFlags[1:1+ncorners])
			copy(cornerPolys[0:], cornerPolys[1:1+ncorners])
			copy(cornerVerts[0:], cornerVerts[3:3+3*ncorners])
		}
	}

	// Prune points after an off-mesh connection.
	for i := 0; i < ncorners; i += 1 {
		if cornerFlags[i]&detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION > 0 {
			ncorners = i + 1
			break
		}
	}

	return ncorners
}

/*
*
@par

Inaccurate locomotion or dynamic obstacle avoidance can force the argent position significantly outside the
original corridor. Over time this can result in the formation of a non-optimal corridor. Non-optimal paths can
also form near the corners of tiles.

This function uses an efficient local visibility search to try to optimize the corridor
between the current position and @p next.

The corridor will change only if @p next is visible from the current position and moving directly toward the point
is better than following the existing path.

The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency
of the call to match the needs to the agent.

This function is not suitable for long distance searches.
*/
func (this *DtPathCorridor) OptimizePathVisibility(next []float32, pathOptimizationRange float32,
	navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) {
	detour.DtAssert(len(this.m_path) > 0)

	// Clamp the ray to max distance.
	var goal [3]float32
	detour.DtVcopy(goal[:], next)
	var dist = detour.DtVdist2D(this.m_pos[:], goal[:])

	// If too close to the goal, do not try to optimize.
	if dist < 0.01 {
		return
	}

	// Overshoot a little. This helps to optimize open fields in tiled meshes.
	dist = detour.DtMinFloat32(dist+0.01, pathOptimizationRange)

	// Adjust ray length.
	var delta [3]float32
	detour.DtVsub(delta[:], goal[:], this.m_pos[:])
	detour.DtVmad(goal[:], this.m_pos[:], delta[:], pathOptimizationRange/dist)

	const MAX_RES = 32
	var res [MAX_RES]detour.DtPolyRef
	var t float32
	var norm [3]float32
	var nres int
	navquery.Raycast(this.m_path[0], this.m_pos[:], goal[:], filter, &t, norm[:], res[:], &nres, MAX_RES)
	if nres > 1 && t > 0.99 {
		this.m_npath = dtMergeCorridorStartShortcut(this.m_path, this.m_npath, this.m_maxPath, res[:], nres)
	}
}

/*
*
@par

Inaccurate locomotion or dynamic obstacle avoidance can force the agent position significantly outside the
original corridor. Over time this can result in the formation of a non-optimal corridor. This function will use a
local area path search to try to re-optimize the corridor.

The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency of
the call to match the needs to the agent.
*/
func (this *DtPathCorridor) OptimizePathTopology(navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) bool {

	detour.DtAssert(navquery != nil)
	detour.DtAssert(filter != nil)
	detour.DtAssert(len(this.m_path) > 0)

	if this.m_npath < 3 {
		return false
	}

	const MAX_ITER = 32
	const MAX_RES = 32

	var res [MAX_RES]detour.DtPolyRef
	var nres int
	var option detour.DtFindPathOptions
	var doneIter int
	navquery.InitSlicedFindPath(this.m_path[0], this.m_path[this.m_npath-1], this.m_pos[:], this.m_target[:], filter, option)
	navquery.UpdateSlicedFindPath(MAX_ITER, &doneIter)

	var status = navquery.FinalizeSlicedFindPathPartial(this.m_path, this.m_npath, res[:], &nres, MAX_RES)
	if detour.DtStatusSucceed(status) && nres > 0 {
		this.m_npath = dtMergeCorridorStartShortcut(this.m_path, this.m_npath, this.m_maxPath, res[:], nres)
		return true
	}

	return false
}

func (this *DtPathCorridor) MoveOverOffmeshConnection(offMeshConRef detour.DtPolyRef, refs []detour.DtPolyRef,
	startPos, endPos []float32,
	navquery *detour.DtNavMeshQuery) bool {
	detour.DtAssert(navquery != nil)
	detour.DtAssert(len(this.m_path) > 0)
	detour.DtAssert(this.m_npath > 0)

	// Advance the path up to and over the off-mesh connection.
	var prevRef detour.DtPolyRef
	var polyRef = this.m_path[0]
	var npos int
	for npos < this.m_npath && polyRef != offMeshConRef {
		prevRef = polyRef
		polyRef = this.m_path[npos]
		npos += 1
	}
	if npos == this.m_npath {
		// Could not find offMeshConRef
		return false
	}

	// Prune path
	for i := npos; i < this.m_npath; i += 1 {
		this.m_path[i-npos] = this.m_path[i]
	}
	this.m_npath -= npos

	refs[0] = prevRef
	refs[1] = polyRef

	var nav = navquery.GetAttachedNavMesh()
	detour.DtAssert(nav != nil)

	var status = nav.GetOffMeshConnectionPolyEndPoints(refs[0], refs[1], startPos, endPos)
	if detour.DtStatusSucceed(status) {
		detour.DtVcopy(this.m_pos[:], endPos)
		return true
	}

	return false
}

/*
*
@par

Behavior:

- The movement is constrained to the surface of the navigation mesh.
- The corridor is automatically adjusted (shorted or lengthened) in order to remain valid.
- The new position will be located in the adjusted corridor's first polygon.

The expected use case is that the desired position will be 'near' the current corridor. What is considered 'near'
depends on local polygon density, query search half extents, etc.

The resulting position will differ from the desired position if the desired position is not on the navigation mesh,
or it can't be reached using a local search.
*/
func (this *DtPathCorridor) MovePosition(npos []float32, navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) bool {

	detour.DtAssert(len(this.m_path) > 0)
	detour.DtAssert(this.m_npath > 0)

	const MAX_VISITED = 16
	// Move along navmesh and update new position.
	var result [3]float32
	var visited [MAX_VISITED]detour.DtPolyRef
	var nvisited int
	var bhit bool
	var status = navquery.MoveAlongSurface(this.m_path[0], this.m_pos[:], npos, filter, result[:], visited[:], &nvisited, MAX_VISITED, &bhit)
	if detour.DtStatusSucceed(status) {
		this.m_npath = dtMergeCorridorStartMoved(this.m_path, this.m_npath, this.m_maxPath, visited[:], nvisited)

		// Adjust the position to stay on top of the navmesh.
		var h = this.m_pos[1]
		navquery.GetPolyHeight(this.m_path[0], result[:], &h)
		result[1] = h
		detour.DtVcopy(this.m_pos[:], result[:])
		return true
	}

	return false
}

/*
*
@par

Behavior:

- The movement is constrained to the surface of the navigation mesh.
- The corridor is automatically adjusted (shorted or lengthened) in order to remain valid.
- The new target will be located in the adjusted corridor's last polygon.

The expected use case is that the desired target will be 'near' the current corridor. What is considered 'near' depends on local polygon density, query search half extents, etc.

The resulting target will differ from the desired target if the desired target is not on the navigation mesh, or it can't be reached using a local search.
*/
func (this *DtPathCorridor) MoveTargetPosition(npos []float32, navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) bool {
	detour.DtAssert(len(this.m_path) > 0)
	detour.DtAssert(this.m_npath > 0)

	const MAX_VISITED = 16
	// Move along navmesh and update new position.
	var result [3]float32
	var visited [MAX_VISITED]detour.DtPolyRef
	var nvisited int
	var bhit bool
	var status = navquery.MoveAlongSurface(this.m_path[this.m_npath-1], this.m_target[:], npos, filter,
		result[:], visited[:], &nvisited, MAX_VISITED, &bhit)
	if detour.DtStatusSucceed(status) {
		this.m_npath = dtMergeCorridorEndMoved(this.m_path, this.m_npath, this.m_maxPath, visited[:], nvisited)
		// TODO: should we do that?
		// Adjust the position to stay on top of the navmesh.
		/*	float h = m_target[1];
			navquery->getPolyHeight(m_path[m_npath-1], result, &h);
			result[1] = h;*/
		detour.DtVcopy(this.m_target[:], result[:])
		return true
	}

	return false
}

// / @par
// /
// / The current corridor position is expected to be within the first polygon in the path. The target
// / is expected to be in the last polygon.
// /
// / @warning The size of the path must not exceed the size of corridor's path buffer set during #init().
func (this *DtPathCorridor) SetCorridor(target []float32, path []detour.DtPolyRef, npath int) {
	detour.DtAssert(len(this.m_path) > 0)
	detour.DtAssert(npath > 0)
	detour.DtAssert(npath < this.m_maxPath)

	detour.DtVcopy(this.m_target[:], target)
	copy(this.m_path[0:], path[:npath])
	this.m_npath = npath
}

func (this *DtPathCorridor) FixPathStart(safeRef detour.DtPolyRef, safePos []float32) bool {
	detour.DtAssert(len(this.m_path) > 0)

	detour.DtVcopy(this.m_pos[:], safePos)
	if this.m_npath < 3 && this.m_npath > 0 {
		this.m_path[2] = this.m_path[this.m_npath-1]
		this.m_path[0] = safeRef
		this.m_path[1] = 0
		this.m_npath = 3
	} else {
		this.m_path[0] = safeRef
		this.m_path[1] = 0
	}

	return true
}

func (this *DtPathCorridor) trimInvalidPath(safeRef detour.DtPolyRef, safePos []float32,
	navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) bool {
	detour.DtAssert(len(this.m_path) > 0)
	detour.DtAssert(navquery != nil)
	detour.DtAssert(filter != nil)

	// Keep valid path as far as possible.
	var n int
	for n < this.m_npath && navquery.IsValidPolyRef(this.m_path[n], filter) {
		n += 1
	}

	if n == this.m_npath {
		// All valid, no need to fix.
		return true
	} else if n == 0 {
		// The first polyref is bad, use current safe values.
		detour.DtVcopy(this.m_pos[:], safePos)
		this.m_path[0] = safeRef
		this.m_npath = 1
	} else {
		// The path is partially usable.
		this.m_npath = n
	}

	var tgt [3]float32
	detour.DtVcopy(tgt[:], this.m_target[:])
	navquery.ClosestPointOnPolyBoundary(this.m_path[this.m_npath-1], tgt[:], this.m_target[:])

	return true
}

// / @par
// /
// / The path can be invalidated if there are structural changes to the underlying navigation mesh, or the state of
// / a polygon within the path changes resulting in it being filtered out. (E.g. An exclusion or inclusion flag changes.)
func (this *DtPathCorridor) IsValid(maxLookAhead int, navquery *detour.DtNavMeshQuery, filter *detour.DtQueryFilter) bool {
	// Check that all polygons still pass query filter.

	var n = detour.DtMinInt(this.m_npath, maxLookAhead)
	for i := 0; i < n; i += 1 {
		if !navquery.IsValidPolyRef(this.m_path[i], filter) {
			return false
		}
	}
	return true
}
