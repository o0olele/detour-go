package debugger

import (
	"errors"

	dtcrowd "github.com/o0olele/detour-go/crowd"
	detour "github.com/o0olele/detour-go/detour"
	"github.com/o0olele/detour-go/loader"
	dtcache "github.com/o0olele/detour-go/tilecache"
)

func GetNavMeshPrimitives(navmesh *detour.DtNavMesh) []*DebugDrawerPrimitive {
	dispList := NewDisplayList(256)
	duDebugDrawNavMesh(dispList, navmesh, DU_DRAWNAVMESH_COLOR_TILES)
	return dispList.flush()
}

type NavItem struct {
	name        string
	dispList    *duDisplayList
	navmesh     *detour.DtNavMesh
	tilecache   *dtcache.DtTileCache
	crowd       *dtcrowd.DtCrowd
	agents      []*ServerAgent
	agentParams ServerAgentParams
}

func NewNavItem(name string) *NavItem {
	return &NavItem{
		name:     name,
		dispList: NewDisplayList(256),
		agentParams: ServerAgentParams{
			Radius:          0.3,
			Height:          2,
			MaxSpeed:        6,
			MaxAcceleration: 20,
		},
	}
}

func (i *NavItem) GetInfo(addMesh bool) *NavInfo {

	info := &NavInfo{
		Params: &i.agentParams,
	}

	for _, sa := range i.agents {
		agent := i.crowd.GetAgent(int(sa.Id))
		if agent == nil {
			continue
		}
		tmp := &ServerAgent{
			Id: sa.Id,
		}
		pos := agent.GetCurrentPos()
		if len(pos) == 3 {
			detour.DtVcopy(tmp.Pos[:], pos)
		}
		info.Agents = append(info.Agents, tmp)
	}

	if i.navmesh != nil && addMesh {
		duDebugDrawNavMesh(i.dispList, i.navmesh, DU_DRAWNAVMESH_COLOR_TILES)
		info.Primitives = i.dispList.flush()
	}

	return info
}

func (i *NavItem) Load(navType string, data []byte) error {

	switch navType {
	case "tilemesh":
		i.navmesh = loader.LoadTileMeshByBytes(data)
		if i.navmesh == nil {
			return errors.New("load tile mesh failed")
		}
		i.crowd = dtcrowd.DtAllocCrowd()
		i.crowd.Init(1, 10, i.navmesh)
	case "tmpobstacles":
		i.navmesh, i.tilecache = loader.LoadTempObstaclesByBytes(data)
		if i.navmesh == nil || i.tilecache == nil {
			return errors.New("load tmpobstacles failed")
		}
		i.crowd = dtcrowd.DtAllocCrowd()
		i.crowd.Init(1, 10, i.navmesh)
	}

	return nil
}

func (i *NavItem) AddAgent(x, y, z, r, h, speed, acc float32) int {

	if i.crowd == nil || i.navmesh == nil {
		return -1
	}

	var agentParams = dtcrowd.DtAllocCrowdAgentParams().
		SetRadius(r).
		SetHeight(h).
		SetMaxAcceleration(acc).
		SetMaxSpeed(speed).
		SetCollisionQueryRange(0.3 * 12).
		SetPathOptimizationRange(0.3 * 30)
	i.agentParams.Radius = r
	i.agentParams.Height = h
	i.agentParams.MaxSpeed = speed
	i.agentParams.MaxAcceleration = acc

	idx := i.crowd.AddAgent([]float32{x, y, z}, agentParams)
	if idx < 0 {
		return -1
	}

	agent := i.crowd.GetAgent(idx)
	if agent == nil {
		return -1
	}

	var serverAgent *ServerAgent
	for _, sa := range i.agents {
		if sa.Id == uint32(idx) {
			serverAgent = sa
			break
		}
	}

	if serverAgent == nil {
		serverAgent = &ServerAgent{Id: uint32(idx)}
		i.agents = append(i.agents, serverAgent)
	}

	pos := agent.GetCurrentPos()
	if len(pos) == 3 {
		detour.DtVcopy(serverAgent.Pos[:], pos)
	}

	return idx
}

func (i *NavItem) UpdateAgents() {

	if i.crowd == nil {
		return
	}
	// todo update once
	i.crowd.Update(0.025, nil)

	for _, sa := range i.agents {
		agent := i.crowd.GetAgent(int(sa.Id))
		if agent == nil {
			continue
		}
		pos := agent.GetCurrentPos()
		if len(pos) == 3 {
			detour.DtVcopy(sa.Pos[:], pos)
		}
	}
}

func (i *NavItem) ClearAgent() {
	if i.crowd == nil {
		return
	}

	for _, sa := range i.agents {
		i.crowd.RemoveAgent(int(sa.Id))
	}
}

func (i *NavItem) SetAgentTarget(x, y, z float32) {
	if i.crowd == nil {
		return
	}

	for _, sa := range i.agents {
		i.crowd.AgentGoto(int(sa.Id), []float32{x, y, z})
	}
}

func (i *NavItem) TeleportAgent(x, y, z float32) bool {
	if i.crowd == nil {
		return false
	}

	for _, sa := range i.agents {
		i.crowd.TeleportAgent(int(sa.Id), []float32{x, y, z})
	}
	return true
}
