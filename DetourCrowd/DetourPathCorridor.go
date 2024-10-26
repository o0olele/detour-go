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

// / Represents a dynamic polygon corridor used to plan agent movement.
// / @ingroup crowd, detour
package detourcrowd

import detour "github.com/o0olele/detour-go/Detour"

type DtPathCorridor struct {
	m_pos    [3]float32
	m_target [3]float32

	m_path    []detour.DtPolyRef
	m_npath   int
	m_maxPath int
}

func (this *DtPathCorridor) GetPos() []float32 {
	return this.m_pos[:]
}

func (this *DtPathCorridor) GetPath() []detour.DtPolyRef {
	return this.m_path
}

func (this *DtPathCorridor) GetTarget() []float32 {
	return this.m_target[:]
}

func (this *DtPathCorridor) GetFirstPoly() detour.DtPolyRef {
	if len(this.m_path) > 0 {
		return this.m_path[0]
	}
	return 0
}

func (this *DtPathCorridor) GetLastPoly() detour.DtPolyRef {
	if len(this.m_path) > 0 {
		return this.m_path[this.m_npath-1]
	}
	return 0
}

func (this *DtPathCorridor) GetPathCount() int {
	return this.m_npath
}
