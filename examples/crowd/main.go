package main

import (
	"fmt"

	detour "github.com/o0olele/detour-go/Detour"
	dtcrowd "github.com/o0olele/detour-go/DetourCrowd"
	"github.com/o0olele/detour-go/examples/common"
)

func main() {
	var mesh = common.LoadTileMesh("../assets/navmesh.bin")
	if mesh == nil {
		panic("load mesh failed.")
	}

	var meshQuery = detour.DtAllocNavMeshQuery()
	var status = meshQuery.Init(mesh, 2048)
	if detour.DtStatusFailed(status) {
		panic("init mesh query failed.")
	}

	var meshFilter = detour.DtAllocDtQueryFilter()

	var crowd = dtcrowd.DtAllocCrowd()
	if !crowd.Init(1, 0.5, mesh) {
		panic("init crowd failed.")
	}

	var agentPos [3]float32
	var agentHalfExtents = [3]float32{1, 0.75, 1}
	var agentNearestPoly detour.DtPolyRef
	status = meshQuery.FindNearestPoly(agentPos[:], agentHalfExtents[:], meshFilter, &agentNearestPoly, agentPos[:])
	if detour.DtStatusFailed(status) {
		panic("find closest point failed.")
	}

	var agentParams = dtcrowd.DtAllocCrowdAgentParams().
		SetRadius(0.5).
		SetHeight(2).
		SetMaxAcceleration(6).
		SetMaxSpeed(10).
		SetCollisionQueryRange(0.3 * 12).
		SetPathOptimizationRange(0.3 * 30)
	var agentId = crowd.AddAgent(agentPos[:], agentParams)

	var agentTarget = [3]float32{1.1322085857391357, 10.197294235229492, -5.400757312774658}
	var agentTragetRef detour.DtPolyRef
	status = meshQuery.FindNearestPoly(agentTarget[:], agentHalfExtents[:], meshFilter, &agentTragetRef, agentTarget[:])
	if detour.DtStatusFailed(status) {
		panic("find agent target closest point failed.")
	}

	crowd.RequestMoveTarget(agentId, agentTragetRef, agentTarget[:])

	var path [256]detour.DtPolyRef
	var pathCount int
	meshQuery.FindPath(agentNearestPoly, agentTragetRef, agentPos[:], agentTarget[:], meshFilter, path[:], &pathCount, 256)

	var straightPath [256 * 3]float32
	var straightPathFlags [256]detour.DtStraightPathFlags
	var straightPathRef [256]detour.DtPolyRef
	var straightPathCount int
	meshQuery.FindStraightPath(agentPos[:], agentTarget[:], path[:], pathCount, straightPath[:], straightPathFlags[:], straightPathRef[:], &straightPathCount, 256, 0)
	fmt.Println(straightPath[:pathCount*3])

	var agent = crowd.GetAgent(agentId)
	fmt.Println(agent.GetCurrentPos())
	for i := 0; i < 100; i += 1 {
		crowd.Update(0.1, nil)
		fmt.Println(agent.GetCurrentPos())
	}
}
