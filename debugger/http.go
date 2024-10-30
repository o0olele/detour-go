package debugger

import (
	"fmt"
	"io"
	"net/http"
	"sync"

	"github.com/gin-gonic/gin"
	dtcrowd "github.com/o0olele/detour-go/crowd"
	detour "github.com/o0olele/detour-go/detour"
	"github.com/o0olele/detour-go/loader"
	dtcache "github.com/o0olele/detour-go/tilecache"
)

type ServerAgent struct {
	Id  uint32     `json:"id"`
	Pos [3]float32 `json:"pos"`
}

type ServerAgentParams struct {
	Radius          float32 `json:"radius"`
	Height          float32 `json:"height"`
	MaxSpeed        float32 `json:"max_speed"`
	MaxAcceleration float32 `json:"max_acceleration"`
}

type Server struct {
	mutex       sync.RWMutex
	dispList    *duDisplayList
	navmesh     *detour.DtNavMesh
	tilecache   *dtcache.DtTileCache
	crowd       *dtcrowd.DtCrowd
	agents      []*ServerAgent
	agentParams ServerAgentParams
}

func NewServer(r *gin.Engine) *Server {
	server := &Server{
		dispList: NewDisplayList(256),
		agentParams: ServerAgentParams{
			Radius:          0.3,
			Height:          2,
			MaxSpeed:        6,
			MaxAcceleration: 20,
		},
	}
	r.GET("/info", server.HandleInfo)
	r.POST("/load", server.HandleLoad)

	agentGroup := r.Group("/agent")
	{
		agentGroup.POST("/add", server.HandleAgentAdd)
		agentGroup.POST("/move", server.HandleAgentMove)
		agentGroup.POST("/update", server.HandleAgentUpdate)
		agentGroup.POST("/teleport", server.HandleAgentTeleport)
	}

	return server
}

type NavInfo struct {
	Primitives []*DebugDrawerPrimitive `json:"primitives"`
	Agents     []*ServerAgent          `json:"agents"`
	Params     *ServerAgentParams      `json:"agent_params"`
}

func (s *Server) AddAgent(x, y, z, r, h, speed, acc float32) int {
	s.mutex.Lock()
	defer s.mutex.Unlock()

	if s.crowd == nil || s.navmesh == nil {
		return -1
	}

	var agentParams = dtcrowd.DtAllocCrowdAgentParams().
		SetRadius(r).
		SetHeight(h).
		SetMaxAcceleration(acc).
		SetMaxSpeed(speed).
		SetCollisionQueryRange(0.3 * 12).
		SetPathOptimizationRange(0.3 * 30)
	s.agentParams.Radius = r
	s.agentParams.Height = h
	s.agentParams.MaxSpeed = speed
	s.agentParams.MaxAcceleration = acc

	idx := s.crowd.AddAgent([]float32{x, y, z}, agentParams)
	if idx < 0 {
		return -1
	}

	agent := s.crowd.GetAgent(idx)
	if agent == nil {
		return -1
	}

	var serverAgent *ServerAgent
	for _, sa := range s.agents {
		if sa.Id == uint32(idx) {
			serverAgent = sa
			break
		}
	}

	if serverAgent == nil {
		serverAgent = &ServerAgent{Id: uint32(idx)}
		s.agents = append(s.agents, serverAgent)
	}

	pos := agent.GetCurrentPos()
	if len(pos) == 3 {
		detour.DtVcopy(serverAgent.Pos[:], pos)
	}

	return idx
}

func (s *Server) UpdateAgents() {
	s.mutex.Lock()
	defer s.mutex.Unlock()

	if s.crowd == nil {
		return
	}
	// todo update once
	s.crowd.Update(0.025, nil)

	for _, sa := range s.agents {
		agent := s.crowd.GetAgent(int(sa.Id))
		if agent == nil {
			continue
		}
		pos := agent.GetCurrentPos()
		if len(pos) == 3 {
			detour.DtVcopy(sa.Pos[:], pos)
		}
	}
}

func (s *Server) ClearAgent() {
	if s.crowd == nil {
		return
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	for _, sa := range s.agents {
		s.crowd.RemoveAgent(int(sa.Id))
	}
}

func (s *Server) SetAgentTarget(x, y, z float32) {
	if s.crowd == nil {
		return
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	for _, sa := range s.agents {
		s.crowd.AgentGoto(int(sa.Id), []float32{x, y, z})
	}
}

func (s *Server) TeleportAgent(x, y, z float32) bool {
	if s.crowd == nil {
		return false
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	for _, sa := range s.agents {
		s.crowd.TeleportAgent(int(sa.Id), []float32{x, y, z})
	}
	return true
}

func (s *Server) GetInfo(addMesh bool, flags DrawNavMeshFlags) *NavInfo {
	s.mutex.RLock()
	defer s.mutex.RUnlock()

	info := &NavInfo{
		Params: &s.agentParams,
	}

	for _, sa := range s.agents {
		agent := s.crowd.GetAgent(int(sa.Id))
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

	if s.navmesh != nil && addMesh {
		duDebugDrawNavMesh(s.dispList, s.navmesh, DU_DRAWNAVMESH_COLOR_TILES)
		info.Primitives = s.dispList.flush()
	}

	return info
}

func (s *Server) Response(ctx *gin.Context, code int, msg string, data any) {
	ctx.JSON(http.StatusOK, gin.H{
		"code": code,
		"msg":  msg,
		"data": data,
	})
}

// HandleLoad handle upload binary navmesh data
func (s *Server) HandleLoad(ctx *gin.Context) {
	navType := ctx.PostForm("type")
	// single file
	file, _ := ctx.FormFile("file")

	fmt.Println(navType, file.Filename)
	f, err := file.Open()
	if err != nil {
		s.Response(ctx, 1, "read file failed.", nil)
		return
	}
	defer f.Close()

	data, err := io.ReadAll(f)
	if err != nil {
		s.Response(ctx, 1, "read binary data from file failed.", nil)
		return
	}

	s.mutex.Lock()

	switch navType {
	case "tilemesh":
		s.navmesh = loader.LoadTileMeshByBytes(data)
		if s.navmesh == nil {
			s.Response(ctx, 2, "load tile mesh failed.", nil)
			return
		}
		s.crowd = dtcrowd.DtAllocCrowd()
		s.crowd.Init(1, 10, s.navmesh)
	case "tmpobstacles":
		s.navmesh, s.tilecache = loader.LoadTempObstaclesByBytes(data)
		if s.navmesh == nil || s.tilecache == nil {
			s.Response(ctx, 2, "load tmpobstacles failed.", nil)
			return
		}
		s.crowd = dtcrowd.DtAllocCrowd()
		s.crowd.Init(1, 10, s.navmesh)
	}
	s.mutex.Unlock()

	s.Response(ctx, 0, "ok", s.GetInfo(true, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleInfo(ctx *gin.Context) {
	s.Response(ctx, 0, "ok", s.GetInfo(true, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentAdd(ctx *gin.Context) {
	type Params struct {
		Pos struct {
			X float32 `json:"x"`
			Y float32 `json:"y"`
			Z float32 `json:"z"`
		} `json:"pos"`
		Radius          float32 `json:"radius"`
		Height          float32 `json:"height"`
		MaxSpeed        float32 `json:"max_speed"`
		MaxAcceleration float32 `json:"max_acceleration"`
	}

	p := &Params{}
	if err := ctx.ShouldBindJSON(p); err != nil {
		s.Response(ctx, 3, "bad params.", nil)
		return
	}

	s.ClearAgent()
	if s.AddAgent(p.Pos.X, p.Pos.Y, p.Pos.Z, p.Radius, p.Height, p.MaxSpeed, p.MaxAcceleration) < 0 {
		s.Response(ctx, 4, "navmesh not init.", nil)
		return
	}
	s.Response(ctx, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentMove(ctx *gin.Context) {
	type Params struct {
		X float32 `json:"x"`
		Y float32 `json:"y"`
		Z float32 `json:"z"`
	}

	p := &Params{}
	if err := ctx.ShouldBindJSON(p); err != nil {
		s.Response(ctx, 3, "bad params.", nil)
		return
	}

	s.SetAgentTarget(p.X, p.Y, p.Z)
	s.Response(ctx, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentUpdate(ctx *gin.Context) {
	s.UpdateAgents()
	s.Response(ctx, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentTeleport(ctx *gin.Context) {
	type Params struct {
		X float32 `json:"x"`
		Y float32 `json:"y"`
		Z float32 `json:"z"`
	}

	p := &Params{}
	if err := ctx.ShouldBindJSON(p); err != nil {
		s.Response(ctx, 3, "bad params.", nil)
		return
	}

	s.TeleportAgent(p.X, p.Y, p.Z)
	s.Response(ctx, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}
