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

package detourcrowd

import (
	"math"

	detour "github.com/o0olele/detour-go/Detour"
)

const DT_PI float32 = 3.14159265

func sweepCircleCircle(c0 []float32, r0 float32, v []float32,
	c1 []float32, r1 float32,
	tmin *float32, tmax *float32) int {
	const EPS float32 = 0.0001
	var s [3]float32
	detour.DtVsub(s[:], c1, c0)
	var r = r0 + r1
	var c = detour.DtVdot2D(s[:], s[:]) - r*r
	var a = detour.DtVdot2D(v, v)
	if a < EPS {
		return 0
	} // not moving

	// Overlap, calc time to exit.
	var b = detour.DtVdot2D(v, s[:])
	var d = b*b - a*c
	if d < 0.0 {
		return 0 // no intersection.
	}
	a = 1.0 / a
	var rd = detour.DtMathSqrtf(d)
	*tmin = (b - rd) * a
	*tmax = (b + rd) * a
	return 1
}

func isectRaySeg(ap, u, bp, bq []float32, t *float32) int {
	var v, w [3]float32
	detour.DtVsub(v[:], bq, bp)
	detour.DtVsub(w[:], ap, bp)
	var d = detour.DtVperp2D(u, v[:])
	if detour.DtMathFabsf(d) < 1e-6 {
		return 0
	}
	d = 1.0 / d
	*t = detour.DtVperp2D(v[:], w[:]) * d
	if *t < 0 || *t > 1 {
		return 0
	}
	var s = detour.DtVperp2D(u, w[:]) * d
	if s < 0 || s > 1 {
		return 0
	}
	return 1
}

func (this *DtObstacleAvoidanceDebugData) init(maxSamples int) bool {

	detour.DtAssert(maxSamples > 0)
	this.m_maxSamples = maxSamples

	this.m_vel = make([]float32, 3*this.m_maxSamples)
	this.m_pen = make([]float32, this.m_maxSamples)
	this.m_ssize = make([]float32, this.m_maxSamples)
	this.m_vpen = make([]float32, this.m_maxSamples)
	this.m_vcpen = make([]float32, this.m_maxSamples)
	this.m_spen = make([]float32, this.m_maxSamples)
	this.m_tpen = make([]float32, this.m_maxSamples)

	return true
}

func (this *DtObstacleAvoidanceDebugData) reset() {
	this.m_nsamples = 0
}

func (this *DtObstacleAvoidanceDebugData) addSample(vel []float32, ssize, pen, vpen, vcpen, spen, tpen float32) {
	if this.m_nsamples >= this.m_maxSamples {
		return
	}
	detour.DtAssert(len(this.m_vel) > 0)
	detour.DtAssert(len(this.m_ssize) > 0)
	detour.DtAssert(len(this.m_pen) > 0)
	detour.DtAssert(len(this.m_vpen) > 0)
	detour.DtAssert(len(this.m_vcpen) > 0)
	detour.DtAssert(len(this.m_spen) > 0)
	detour.DtAssert(len(this.m_tpen) > 0)

	detour.DtVcopy(this.m_vel[this.m_nsamples*3:], vel)
	this.m_ssize[this.m_nsamples] = ssize
	this.m_pen[this.m_nsamples] = pen
	this.m_vpen[this.m_nsamples] = vpen
	this.m_vcpen[this.m_nsamples] = vcpen
	this.m_spen[this.m_nsamples] = spen
	this.m_tpen[this.m_nsamples] = tpen
	this.m_nsamples += 1
}

func normalizeArray(arr []float32, n int) {
	// Normalize penaly range.
	var minPen = float32(math.MaxFloat32)
	var maxPen = -float32(math.MaxFloat32)
	for i := 0; i < n; i++ {
		minPen = detour.DtMinFloat32(minPen, arr[i])
		maxPen = detour.DtMaxFloat32(maxPen, arr[i])
	}
	var penRange = maxPen - minPen
	var s = float32(1)
	if penRange > 0.001 {
		s = 1 / penRange
	}
	for i := 0; i < n; i += 1 {
		arr[i] = detour.DtClampFloat32((arr[i]-minPen)*s, 0, 1)
	}
}

func (this *DtObstacleAvoidanceDebugData) normalizeSamples() {
	normalizeArray(this.m_pen, this.m_nsamples)
	normalizeArray(this.m_vpen, this.m_nsamples)
	normalizeArray(this.m_vcpen, this.m_nsamples)
	normalizeArray(this.m_spen, this.m_nsamples)
	normalizeArray(this.m_tpen, this.m_nsamples)
}

func (this *DtObstacleAvoidanceQuery) init(maxCircles, maxSegments int) bool {

	this.m_maxCircles = maxCircles
	this.m_ncircles = 0
	this.m_circles = make([]DtObstacleCircle, this.m_maxCircles)

	this.m_maxSegments = maxSegments
	this.m_nsegments = 0
	this.m_segments = make([]DtObstacleSegment, this.m_maxSegments)

	return true
}

func (this *DtObstacleAvoidanceQuery) reset() {
	this.m_ncircles = 0
	this.m_nsegments = 0
}

func (this *DtObstacleAvoidanceQuery) addCircle(pos []float32, rad float32,
	vel []float32, dvel []float32) {
	if this.m_ncircles >= this.m_maxCircles {
		return
	}

	var cir = &this.m_circles[this.m_ncircles]
	this.m_ncircles += 1
	detour.DtVcopy(cir.p[:], pos)
	cir.rad = rad
	detour.DtVcopy(cir.vel[:], vel)
	detour.DtVcopy(cir.dvel[:], dvel)
}

func (this *DtObstacleAvoidanceQuery) addSegment(p []float32, q []float32) {
	if this.m_nsegments >= this.m_maxSegments {
		return
	}

	var seg = &this.m_segments[this.m_nsegments]
	this.m_nsegments += 1
	detour.DtVcopy(seg.p[:], p)
	detour.DtVcopy(seg.q[:], q)
}

func (this *DtObstacleAvoidanceQuery) prepare(pos, dvel []float32) {
	// Prepare obstacles
	for i := 0; i < this.m_ncircles; i += 1 {
		var cir = &this.m_circles[i]

		var pa = pos
		var pb = cir.p[:]

		var orig [3]float32
		var dv [3]float32
		detour.DtVsub(cir.dp[:], pb, pa)
		detour.DtVnormalize(cir.dp[:])
		detour.DtVsub(dv[:], cir.dvel[:], dvel)

		var a = detour.DtTriArea2D(orig[:], cir.dp[:], dv[:])
		if a < 0.01 {
			cir.np[0] = -cir.dp[2]
			cir.np[2] = cir.dp[0]
		} else {
			cir.np[0] = cir.dp[2]
			cir.np[2] = cir.dp[0]
		}
	}

	for i := 0; i < this.m_nsegments; i += 1 {
		var seg = &this.m_segments[i]

		// Precalc if the agent is really close to the segment.
		var r = float32(0.01)
		var t float32
		seg.touch = detour.DtDistancePtSegSqr2D(pos, seg.p[:], seg.q[:], &t) < detour.DtSqrFloat32(r)
	}
}

/* Calculate the collision penalty for a given velocity vector
 *
 * @param vcand sampled velocity
 * @param dvel desired velocity
 * @param minPenalty threshold penalty for early out
 */
func (this *DtObstacleAvoidanceQuery) processSample(vcand []float32, cs float32,
	pos []float32, rad float32,
	vel []float32, dvel []float32,
	minPenalty float32,
	debug *DtObstacleAvoidanceDebugData) float32 {
	// penalty for straying away from the desired and current velocities
	var vpen = this.m_params.weightDesVel * (detour.DtVdist2D(vcand, dvel) * this.m_invVmax)
	var vcpen = this.m_params.weightCurVel * (detour.DtVdist2D(vcand, vel) * this.m_invVmax)

	const FLT_EPSILON = 1.192092896e-07
	// find the threshold hit time to bail out based on the early out penalty
	// (see how the penalty is calculated below to understnad)
	var minPen = minPenalty - vpen - vcpen
	var tThresold = (this.m_params.weightToi/minPen - 0.1) * this.m_params.horizTime
	if tThresold-this.m_params.horizTime > -FLT_EPSILON {
		return minPenalty
	}

	// Find min time of impact and exit amongst all obstacles.
	var tmin = this.m_params.horizTime
	var side float32
	var nside int

	for i := 0; i < this.m_ncircles; i += 1 {
		var cir = &this.m_circles[i]

		// RVO
		var vab [3]float32
		detour.DtVscale(vab[:], vcand, 2)
		detour.DtVsub(vab[:], vab[:], vel)
		detour.DtVsub(vab[:], vab[:], cir.vel[:])

		// Side
		side += detour.DtClampFloat32(detour.DtMinFloat32(detour.DtVdot2D(cir.dp[:], vab[:])*0.5+0.5, detour.DtVdot2D(cir.np[:], vab[:])*2), 0, 1)
		nside += 1

		var htmin, htmax float32
		if sweepCircleCircle(pos, rad, vab[:], cir.p[:], cir.rad, &htmin, &htmax) <= 0 {
			continue
		}

		// Handle overlapping obstacles.
		if htmin < 0 && htmax > 0 {
			// Avoid more when overlapped.
			htmin = -htmin * 0.5
		}

		if htmin >= 0 {
			// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
			if htmin < tmin {
				tmin = htmin
				if tmin < tThresold {
					return minPenalty
				}
			}
		}
	}

	for i := 0; i < this.m_nsegments; i++ {
		var seg = &this.m_segments[i]
		var htmin float32

		if seg.touch {
			// Special case when the agent is very close to the segment.
			var sdir, snorm [3]float32
			detour.DtVsub(sdir[:], seg.q[:], seg.p[:])
			snorm[0] = -sdir[2]
			snorm[2] = sdir[0]
			// If the velocity is pointing towards the segment, no collision.
			if detour.DtVdot2D(snorm[:], vcand) < 0 {
				continue
			}
			htmin = 0
		} else {
			if isectRaySeg(pos, vcand, seg.p[:], seg.q[:], &htmin) <= 0 {
				continue
			}
		}

		// Avoid less when facing walls.
		htmin *= 2.0

		// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
		if htmin < tmin {
			tmin = htmin
			if tmin < tThresold {
				return minPenalty
			}
		}
	}

	// Normalize side bias, to prevent it dominating too much.
	if nside > 0 {
		side /= float32(nside)
	}

	var spen = this.m_params.weightSide * side
	var tpen = this.m_params.weightToi * (1 / (0.1 + tmin*this.m_invHorizTime))

	var penalty = vpen + vcpen + spen + tpen
	// Store different penalties for debug viewing
	if debug != nil {
		debug.addSample(vcand, cs, penalty, vpen, vcpen, spen, tpen)
	}

	return penalty
}

func (this *DtObstacleAvoidanceQuery) sampleVelocityGrid(pos []float32, rad float32, vmax float32,
	vel []float32, dvel []float32, nvel []float32,
	params *DtObstacleAvoidanceParams,
	debug *DtObstacleAvoidanceDebugData) int {

	this.prepare(pos, dvel)

	this.m_params = *params
	this.m_invHorizTime = 1 / this.m_params.horizTime
	this.m_vmax = vmax
	this.m_invVmax = math.MaxFloat32
	if vmax > 0 {
		this.m_invVmax = 1 / vmax
	}

	detour.DtVset(nvel, 0, 0, 0)

	if debug != nil {
		debug.reset()
	}

	var cvx = dvel[0] * this.m_params.velBias
	var cvz = dvel[2] * this.m_params.velBias
	var cs = vmax * 2 * (1 - this.m_params.velBias) / float32(this.m_params.gridSize-1)
	var half = (float32(this.m_params.gridSize) - 1) * cs * 0.5

	var minPenalty = float32(math.MaxFloat32)
	var ns int

	for y := 0; y < int(this.m_params.gridSize); y += 1 {
		for x := 0; x < int(this.m_params.gridSize); x += 1 {
			var vcand [3]float32
			vcand[0] = cvx + float32(x)*cs - half
			vcand[1] = 0
			vcand[2] = cvz + float32(y)*cs - half

			if detour.DtSqrFloat32(vcand[0])+detour.DtSqrFloat32(vcand[2]) > detour.DtSqrFloat32(vmax+cs/2) {
				continue
			}

			var penalty = this.processSample(vcand[:], cs, pos, rad, vel, dvel, minPenalty, debug)
			ns += 1
			if penalty < minPenalty {
				minPenalty = penalty
				detour.DtVcopy(nvel, vcand[:])
			}
		}
	}
	return ns
}

// vector normalization that ignores the y-component.
func dtNormalize2D(v []float32) {
	var d = detour.DtMathSqrtf(v[0]*v[0] + v[2]*v[2])
	if d == 0 {
		return
	}
	d = 1.0 / d
	v[0] *= d
	v[2] *= d
}

// vector normalization that ignores the y-component.
func dtRorate2D(dest []float32, v []float32, ang float32) {
	var c = float32(math.Cos(float64(ang)))
	var s = float32(math.Sin(float64(ang)))
	dest[0] = v[0]*c - v[2]*s
	dest[2] = v[0]*s + v[2]*c
	dest[1] = v[1]
}

func (this *DtObstacleAvoidanceQuery) sampleVelocityAdaptive(pos []float32, rad float32, vmax float32,
	vel []float32, dvel []float32, nvel []float32,
	params *DtObstacleAvoidanceParams,
	debug *DtObstacleAvoidanceDebugData) int {

	this.prepare(pos, dvel)
	this.m_params = *params
	this.m_invHorizTime = 1 / this.m_params.horizTime
	this.m_vmax = vmax
	this.m_invVmax = math.MaxFloat32
	if vmax > 0 {
		this.m_invVmax = 1 / vmax
	}

	detour.DtVset(nvel, 0, 0, 0)

	if debug != nil {
		debug.reset()
	}

	// Build sampling pattern aligned to desired velocity.
	var pat [(DT_MAX_PATTERN_DIVS*DT_MAX_PATTERN_RINGS + 1) * 2]float32
	var npat int

	var ndivs = int(this.m_params.adaptiveDivs)
	var nrings = int(this.m_params.adaptiveRings)
	var depth = int(this.m_params.adaptiveDepth)

	var nd = detour.DtClampInt(ndivs, 1, DT_MAX_PATTERN_DIVS)
	var nr = detour.DtClampInt(nrings, 1, DT_MAX_PATTERN_RINGS)
	var da = (1.0 / float32(nd)) * DT_PI * 2
	var ca = float32(math.Cos(float64(da)))
	var sa = float32(math.Sin(float64(da)))

	// desired direction
	var ddir [6]float32
	detour.DtVcopy(ddir[:], dvel)
	dtNormalize2D(ddir[:])
	dtRorate2D(ddir[3:], ddir[:], da*0.5)

	// Always add sample at zero
	pat[npat*2+0] = 0
	pat[npat*2+1] = 0
	npat += 1

	for j := 0; j < nr; j++ {
		var r = float32(nr-j) / float32(nr)
		pat[npat*2+0] = ddir[(j%2)*3] * r
		pat[npat*2+1] = ddir[(j%2)*3+2] * r
		var last1 = pat[npat*2:]
		var last2 = last1
		npat += 1

		for i := 1; i < nd-1; i += 2 {
			// get next point on the "right" (rotate CW)
			pat[npat*2+0] = last1[0]*ca + last1[1]*sa
			pat[npat*2+1] = -last1[0]*sa + last1[1]*ca
			// get next point on the "left" (rotate CCW)
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca

			last1 = pat[npat*2:]
			last2 = last1[2:]
			npat += 2
		}

		if nd&1 == 0 {
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca
			npat += 1
		}
	}

	// Start sampling.
	var cr = vmax * (1 - this.m_params.velBias)
	var res [3]float32
	detour.DtVset(res[:], dvel[0]*this.m_params.velBias, 0, dvel[2]*this.m_params.velBias)
	var ns int

	for k := 0; k < depth; k += 1 {
		var minPenalty = float32(math.MaxFloat32)
		var bvel [3]float32

		for i := 0; i < npat; i += 1 {
			var vcand [3]float32
			vcand[0] = res[0] + pat[i*2+0]*cr
			vcand[1] = 0
			vcand[2] = res[2] + pat[i*2+1]*cr

			if detour.DtSqrFloat32(vcand[0])+detour.DtSqrFloat32(vcand[2]) > detour.DtSqrFloat32(vmax+0.001) {
				continue
			}

			var penalty = this.processSample(vcand[:], cr/10, pos, rad, vel, dvel, minPenalty, debug)
			ns += 1
			if penalty < minPenalty {
				minPenalty = penalty
				detour.DtVcopy(bvel[:], vcand[:])
			}
		}

		detour.DtVcopy(res[:], bvel[:])
		cr *= 0.5
	}

	detour.DtVcopy(nvel, res[:])
	return ns
}
