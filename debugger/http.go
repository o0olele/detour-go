package debugger

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"sync"
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
	mutex   sync.RWMutex
	navItem *NavItem
	mux     *http.ServeMux
}

func NewServer() *Server {
	server := &Server{
		navItem: NewNavItem("navmesh"),
		mux:     http.NewServeMux(),
	}

	// Register routes
	server.mux.HandleFunc("/info", server.HandleInfo)
	server.mux.HandleFunc("/load", server.HandleLoad)
	server.mux.HandleFunc("/agent/add", server.HandleAgentAdd)
	server.mux.HandleFunc("/agent/move", server.HandleAgentMove)
	server.mux.HandleFunc("/agent/update", server.HandleAgentUpdate)
	server.mux.HandleFunc("/agent/teleport", server.HandleAgentTeleport)

	return server
}

// ServeHTTP implements http.Handler interface
func (s *Server) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	s.mux.ServeHTTP(w, r)
}

type NavInfo struct {
	Primitives []*DebugDrawerPrimitive `json:"primitives"`
	Agents     []*ServerAgent          `json:"agents"`
	Params     *ServerAgentParams      `json:"agent_params"`
}

func (s *Server) AddAgent(x, y, z, r, h, speed, acc float32) int {
	if s.navItem == nil {
		return -1
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	return s.navItem.AddAgent(x, y, z, r, h, speed, acc)
}

func (s *Server) UpdateAgents() {
	if s.navItem == nil {
		return
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	s.navItem.UpdateAgents()
}

func (s *Server) ClearAgent() {
	if s.navItem == nil {
		return
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	s.navItem.ClearAgent()
}

func (s *Server) SetAgentTarget(x, y, z float32) {
	if s.navItem == nil {
		return
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	s.navItem.SetAgentTarget(x, y, z)
}

func (s *Server) TeleportAgent(x, y, z float32) bool {
	if s.navItem == nil {
		return false
	}

	s.mutex.Lock()
	defer s.mutex.Unlock()

	return s.navItem.TeleportAgent(x, y, z)
}

func (s *Server) GetInfo(addMesh bool, flags DrawNavMeshFlags) *NavInfo {
	if s.navItem == nil {
		return nil
	}

	s.mutex.RLock()
	defer s.mutex.RUnlock()

	return s.navItem.GetInfo(addMesh)
}

type APIResponse struct {
	Code int    `json:"code"`
	Msg  string `json:"msg"`
	Data any    `json:"data"`
}

func (s *Server) writeJSONResponse(w http.ResponseWriter, code int, msg string, data any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)

	response := APIResponse{
		Code: code,
		Msg:  msg,
		Data: data,
	}

	json.NewEncoder(w).Encode(response)
}

func (s *Server) parseJSONRequest(r *http.Request, v any) error {
	defer r.Body.Close()
	return json.NewDecoder(r.Body).Decode(v)
}

// HandleLoad handle upload binary navmesh data
func (s *Server) HandleLoad(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	if s.navItem == nil {
		s.writeJSONResponse(w, 1, "navmesh not init.", nil)
		return
	}

	// Parse multipart form
	err := r.ParseMultipartForm(32 << 20) // 32MB max memory
	if err != nil {
		s.writeJSONResponse(w, 1, "failed to parse form.", nil)
		return
	}

	navType := r.FormValue("type")
	file, header, err := r.FormFile("file")
	if err != nil {
		s.writeJSONResponse(w, 1, "read file failed.", nil)
		return
	}
	defer file.Close()

	fmt.Println(navType, header.Filename)

	data, err := io.ReadAll(file)
	if err != nil {
		s.writeJSONResponse(w, 1, "read binary data from file failed.", nil)
		return
	}

	s.mutex.Lock()
	err = s.navItem.Load(navType, data)
	s.mutex.Unlock()

	if err != nil {
		s.writeJSONResponse(w, 2, "load navmesh failed.", nil)
		return
	}

	s.writeJSONResponse(w, 0, "ok", s.GetInfo(true, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleInfo(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	s.writeJSONResponse(w, 0, "ok", s.GetInfo(true, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentAdd(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	if s.navItem == nil {
		s.writeJSONResponse(w, 1, "navmesh not init.", nil)
		return
	}

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
	if err := s.parseJSONRequest(r, p); err != nil {
		s.writeJSONResponse(w, 3, "bad params.", nil)
		return
	}

	s.ClearAgent()
	if s.AddAgent(p.Pos.X, p.Pos.Y, p.Pos.Z, p.Radius, p.Height, p.MaxSpeed, p.MaxAcceleration) < 0 {
		s.writeJSONResponse(w, 4, "navmesh not init.", nil)
		return
	}
	s.writeJSONResponse(w, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentMove(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	type Params struct {
		X float32 `json:"x"`
		Y float32 `json:"y"`
		Z float32 `json:"z"`
	}

	p := &Params{}
	if err := s.parseJSONRequest(r, p); err != nil {
		s.writeJSONResponse(w, 3, "bad params.", nil)
		return
	}

	s.SetAgentTarget(p.X, p.Y, p.Z)
	s.writeJSONResponse(w, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentUpdate(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	s.UpdateAgents()
	s.writeJSONResponse(w, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}

func (s *Server) HandleAgentTeleport(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	type Params struct {
		X float32 `json:"x"`
		Y float32 `json:"y"`
		Z float32 `json:"z"`
	}

	p := &Params{}
	if err := s.parseJSONRequest(r, p); err != nil {
		s.writeJSONResponse(w, 3, "bad params.", nil)
		return
	}

	s.TeleportAgent(p.X, p.Y, p.Z)
	s.writeJSONResponse(w, 0, "ok", s.GetInfo(false, DU_DRAWNAVMESH_COLOR_TILES))
}
