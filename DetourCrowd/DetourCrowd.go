package detourcrowd

import detour "github.com/o0olele/detour-go/Detour"

/// The maximum number of neighbors that a crowd agent can take into account
/// for steering decisions.
/// @ingroup crowd
const DT_CROWDAGENT_MAX_NEIGHBOURS int = 6

/// The maximum number of corners a crowd agent will look ahead in the path.
/// This value is used for sizing the crowd agent corner buffers.
/// Due to the behavior of the crowd manager, the actual number of useful
/// corners will be one less than this number.
/// @ingroup crowd
const DT_CROWDAGENT_MAX_CORNERS int = 4

/// The maximum number of crowd avoidance configurations supported by the
/// crowd manager.
/// @ingroup crowd
/// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
///		 dtCrowdAgentParams::obstacleAvoidanceType
const DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS int = 8

/// The maximum number of query filter types supported by the crowd manager.
/// @ingroup crowd
/// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
///		dtCrowdAgentParams::queryFilterType
const DT_CROWD_MAX_QUERY_FILTER_TYPE int = 16

/// Provides neighbor data for agents managed by the crowd.
/// @ingroup crowd
/// @see dtCrowdAgent::neis, dtCrowd
type DtCrowdNeighbour struct {
	idx  int     ///< The index of the neighbor in the crowd.
	dist float32 ///< The distance between the current agent and the neighbor.
}

/// The type of navigation mesh polygon the agent is currently traversing.
/// @ingroup crowd
type CrowdAgentState int

const DT_CROWDAGENT_STATE_INVALID CrowdAgentState = 0 ///< The agent is not in a valid state.
const DT_CROWDAGENT_STATE_WALKING CrowdAgentState = 1 ///< The agent is traversing a normal navigation mesh polygon.
const DT_CROWDAGENT_STATE_OFFMESH CrowdAgentState = 2 ///< The agent is traversing an off-mesh connection.

/// Configuration parameters for a crowd agent.
/// @ingroup crowd
type DtCrowdAgentParams struct {
	radius          float32 ///< Agent radius. [Limit: >= 0]
	height          float32 ///< Agent height. [Limit: > 0]
	maxAcceleration float32 ///< Maximum allowed acceleration. [Limit: >= 0]
	maxSpeed        float32 ///< Maximum allowed speed. [Limit: >= 0]

	/// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	collisionQueryRange float32

	pathOptimizationRange float32 ///< The path visibility optimization range. [Limit: > 0]

	/// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	separationWeight float32

	/// Flags that impact steering behavior. (See: #UpdateFlags)
	updateFlags UpdateFlags

	/// The index of the avoidance configuration to use for the agent.
	/// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	obstacleAvoidanceType uint8

	/// The index of the query filter used by this agent.
	queryFilterType uint8

	/// User defined data attached to the agent.
	userData []byte
}

type MoveRequestState int

const DT_CROWDAGENT_TARGET_NONE MoveRequestState = 0
const DT_CROWDAGENT_TARGET_FAILED MoveRequestState = 1
const DT_CROWDAGENT_TARGET_VALID MoveRequestState = 2
const DT_CROWDAGENT_TARGET_REQUESTING MoveRequestState = 3
const DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE MoveRequestState = 4
const DT_CROWDAGENT_TARGET_WAITING_FOR_PATH MoveRequestState = 5
const DT_CROWDAGENT_TARGET_VELOCITY MoveRequestState = 6

/// Represents an agent managed by a #dtCrowd object.
/// @ingroup crowd
type DtCrowdAgent struct {
	/// True if the agent is active, false if the agent is in an unused slot in the agent pool.
	active bool

	/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	state CrowdAgentState

	/// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	partial bool

	/// The path corridor the agent is using.
	corridor DtPathCorridor

	/// The local boundary data for the agent.
	boundary DtLocalBoundary

	/// Time since the agent's path corridor was optimized.
	topologyOptTime float32

	/// The known neighbors of the agent.
	neis [DT_CROWDAGENT_MAX_NEIGHBOURS]DtCrowdNeighbour

	/// The number of neighbors.
	nneis int

	/// The desired speed.
	desiredSpeed float32

	npos [3]float32 ///< The current agent position. [(x, y, z)]
	disp [3]float32 ///< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
	dvel [3]float32 ///< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
	nvel [3]float32 ///< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
	vel  [3]float32 ///< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]

	/// The agent's configuration parameters.
	params DtCrowdAgentParams

	/// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	cornerVerts [DT_CROWDAGENT_MAX_CORNERS * 3]float32

	/// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	cornerFlags [DT_CROWDAGENT_MAX_CORNERS]detour.DtStraightPathFlags

	/// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	cornerPolys [DT_CROWDAGENT_MAX_CORNERS]detour.DtPolyRef

	/// The number of corners.
	ncorners int

	targetState      MoveRequestState ///< State of the movement request.
	targetRef        detour.DtPolyRef ///< Target polyref of the movement request.
	targetPos        [3]float32       ///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	targetPathqRef   DtPathQueueRef   ///< Path finder ref.
	targetReplan     bool             ///< Flag indicating that the current path is being replanned.
	targetReplanTime float32          /// <Time since the agent's target was replanned.
}

type DtCrowdAgentAnimation struct {
	active   bool
	initPos  [3]float32
	startPos [3]float32
	endPos   [3]float32
	polyRef  detour.DtPolyRef
	t        float32
	tmax     float32
}

/// Crowd agent update flags.
/// @ingroup crowd
/// @see dtCrowdAgentParams::updateFlags
type UpdateFlags uint8

const (
	DT_CROWD_ANTICIPATE_TURNS   UpdateFlags = 1
	DT_CROWD_OBSTACLE_AVOIDANCE UpdateFlags = 2
	DT_CROWD_SEPARATION         UpdateFlags = 4
	DT_CROWD_OPTIMIZE_VIS       UpdateFlags = 8  ///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	DT_CROWD_OPTIMIZE_TOPO      UpdateFlags = 16 ///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
)

type DtCrowdAgentDebugInfo struct {
	idx      int
	optStart [3]float32
	optEnd   [3]float32
	vod      *DtObstacleAvoidanceDebugData
}

/// Provides local steering behaviors for a group of agents.
/// @ingroup crowd
type DtCrowd struct {
	m_maxAgents    int
	m_agents       []DtCrowdAgent
	m_activeAgents []*DtCrowdAgent
	m_agentAnims   []DtCrowdAgentAnimation

	m_pathq DtPathQueue

	m_obstacleQueryParams [DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]DtObstacleAvoidanceParams
	m_obstacleQuery       *DtObstacleAvoidanceQuery

	m_grid *DtProximityGrid

	m_pathResult    []detour.DtPolyRef
	m_maxPathResult int

	m_agentPlacementHalfExtents [3]float32

	m_filters [DT_CROWD_MAX_QUERY_FILTER_TYPE]detour.DtQueryFilter

	m_maxAgentRadius float32

	m_velocitySampleCount int

	m_navquery *detour.DtNavMeshQuery
}

func (this *DtCrowd) getAgentIndex(agent *DtCrowdAgent) int {
	for i := range this.m_agents {
		if &this.m_agents[i] == agent {
			return i
		}
	}
	return -1
}

func (this *DtCrowd) GetFilter(i int) *detour.DtQueryFilter {
	if i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE {
		return &this.m_filters[i]
	}
	return nil
}

func (this *DtCrowd) GetEditableFilter(i int) *detour.DtQueryFilter {
	if i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE {
		return &this.m_filters[i]
	}
	return nil
}

func (this *DtCrowd) GetQueryHalfExtents() []float32 {
	return this.m_agentPlacementHalfExtents[:]
}

func (this *DtCrowd) GetQueryExtents() []float32 {
	return this.m_agentPlacementHalfExtents[:]
}

func (this *DtCrowd) GetVelocitySampleCount() int {
	return this.m_velocitySampleCount
}

func (this *DtCrowd) GetGrid() *DtProximityGrid {
	return this.m_grid
}

func (this *DtCrowd) GetPathQueue() *DtPathQueue {
	return &this.m_pathq
}

func (this *DtCrowd) GetNavMeshQuery() *detour.DtNavMeshQuery {
	return this.m_navquery
}

func DtAllocCrowd() *DtCrowd {
	crowd := &DtCrowd{}
	return crowd
}

///////////////////////////////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@defgroup crowd Crowd

Members in this module implement local steering and dynamic avoidance features.

The crowd is the big beast of the navigation features. It not only handles a
lot of the path management for you, but also local steering and dynamic
avoidance between members of the crowd. I.e. It can keep your agents from
running into each other.

Main class: #dtCrowd

The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy
to use path planning features. But in the end they only give you points that
your navigation client should be moving toward. When it comes to deciding things
like agent velocity and steering to avoid other agents, that is up to you to
implement. Unless, of course, you decide to use #dtCrowd.

Basically, you add an agent to the crowd, providing various configuration
settings such as maximum speed and acceleration. You also provide a local
target to more toward. The crowd manager then provides, with every update, the
new agent position and velocity for the frame. The movement will be
constrained to the navigation mesh, and steering will be applied to ensure
agents managed by the crowd do not collide with each other.

This is very powerful feature set. But it comes with limitations.

The biggest limitation is that you must give control of the agent's position
completely over to the crowd manager. You can update things like maximum speed
and acceleration. But in order for the crowd manager to do its thing, it can't
allow you to constantly be giving it overrides to position and velocity. So
you give up direct control of the agent's movement. It belongs to the crowd.

The second biggest limitation revolves around the fact that the crowd manager
deals with local planning. So the agent's target should never be more than
256 polygons aways from its current position. If it is, you risk
your agent failing to reach its target. So you may still need to do long
distance planning and provide the crowd manager with intermediate targets.

Other significant limitations:

- All agents using the crowd manager will use the same #dtQueryFilter.
- Crowd management is relatively expensive. The maximum agents under crowd
  management at any one time is between 20 and 30.  A good place to start
  is a maximum of 25 agents for 0.5ms per frame.

@note This is a summary list of members.  Use the index or search
feature to find minor members.

@struct dtCrowdAgentParams
@see dtCrowdAgent, dtCrowd::addAgent(), dtCrowd::updateAgentParameters()

@var dtCrowdAgentParams::obstacleAvoidanceType
@par

#dtCrowd permits agents to use different avoidance configurations.  This value
is the index of the #dtObstacleAvoidanceParams within the crowd.

@see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(),
	 dtCrowd::getObstacleAvoidanceParams()

@var dtCrowdAgentParams::collisionQueryRange
@par

Collision elements include other agents and navigation mesh boundaries.

This value is often based on the agent radius and/or maximum speed. E.g. radius * 8

@var dtCrowdAgentParams::pathOptimizationRange
@par

Only applicalbe if #updateFlags includes the #DT_CROWD_OPTIMIZE_VIS flag.

This value is often based on the agent radius. E.g. radius * 30

@see dtPathCorridor::optimizePathVisibility()

@var dtCrowdAgentParams::separationWeight
@par

A higher value will result in agents trying to stay farther away from each other at
the cost of more difficult steering in tight spaces.

*/
