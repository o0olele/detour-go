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

type DtObstacleCircle struct {
	p    [3]float32 ///< Position of the obstacle
	vel  [3]float32 ///< Velocity of the obstacle
	dvel [3]float32 ///< Velocity of the obstacle
	rad  float32    ///< Radius of the obstacle
	dp   [3]float32
	np   [3]float32 ///< Use for side selection during sampling.
}

type DtObstacleSegment struct {
	p     [3]float32
	q     [3]float32 ///< End points of the obstacle segment
	touch bool
}

type DtObstacleAvoidanceDebugData struct {
	m_nsamples   int
	m_maxSamples int
	m_vel        []float32
	m_ssize      []float32
	m_pen        []float32
	m_vpen       []float32
	m_vcpen      []float32
	m_spen       []float32
	m_tpen       []float32
}

const DT_MAX_PATTERN_DIVS int = 32 ///< Max numver of adaptive divs.
const DT_MAX_PATTERN_RINGS int = 4 ///< Max number of adaptive rings.

type DtObstacleAvoidanceParams struct {
	velBias       float32
	weightDesVel  float32
	weightCurVel  float32
	weightSide    float32
	weightToi     float32
	horizTime     float32
	gridSize      uint8 ///< grid
	adaptiveDivs  uint8 ///< adaptive
	adaptiveRings uint8 ///< adaptive
	adaptiveDepth uint8 ///< adaptive
}

type DtObstacleAvoidanceQuery struct {
	m_params       DtObstacleAvoidanceParams
	m_invHorizTime float32
	m_vmax         float32
	m_invVmax      float32

	m_maxCircles int
	m_circles    []DtObstacleCircle
	m_ncircles   int

	m_maxSegments int
	m_segments    []DtObstacleSegment
	m_nsegments   int
}

func DtAllocObstacleAvoidanceQuery() *DtObstacleAvoidanceQuery {
	query := &DtObstacleAvoidanceQuery{}
	return query
}
