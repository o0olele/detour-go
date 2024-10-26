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

func hashPos2(x int, y int, n int) int {
	return ((x * 73856093) ^ (y * 19349663)) & (n - 1)
}

func (this *DtProximityGrid) init(poolSize int, cellSize float32) bool {
	detour.DtAssert(poolSize > 0)
	detour.DtAssert(cellSize > 0)

	this.m_cellSize = cellSize
	this.m_invCellSize = 1.0 / this.m_cellSize

	this.m_bucketsSize = int(detour.DtNextPow2(uint32(poolSize)))
	this.m_buckets = make([]uint16, this.m_bucketsSize)
	if len(this.m_buckets) <= 0 {
		return false
	}

	this.m_poolSize = poolSize
	this.m_poolHead = 0
	this.m_pool = make([]DtProximityGridItem, this.m_poolSize)
	if len(this.m_pool) <= 0 {
		return false
	}

	this.clear()

	return true
}

func (this *DtProximityGrid) clear() {
	for i := range this.m_buckets {
		this.m_buckets[i] = 0
	}
	this.m_poolHead = 0
	this.m_bounds[0] = 0xffff
	this.m_bounds[1] = 0xffff
	this.m_bounds[2] = -0xffff
	this.m_bounds[3] = -0xffff
}

func (this *DtProximityGrid) addItem(id uint16, minx, miny, maxx, maxy float32) {
	var iminx = int(detour.DtMathFloorf(minx * this.m_invCellSize))
	var iminy = int(detour.DtMathFloorf(miny * this.m_invCellSize))
	var imaxx = int(detour.DtMathFloorf(maxx * this.m_invCellSize))
	var imaxy = int(detour.DtMathFloorf(maxy * this.m_invCellSize))

	this.m_bounds[0] = detour.DtMinInt(this.m_bounds[0], iminx)
	this.m_bounds[1] = detour.DtMinInt(this.m_bounds[1], iminy)
	this.m_bounds[2] = detour.DtMinInt(this.m_bounds[2], imaxx)
	this.m_bounds[3] = detour.DtMinInt(this.m_bounds[3], imaxy)

	var h int
	var idx uint16
	for y := iminy; y <= imaxy; y += 1 {
		for x := iminx; x <= imaxx; x += 1 {
			if this.m_poolHead < this.m_poolSize {
				h = hashPos2(x, y, this.m_bucketsSize)
				idx = uint16(this.m_poolHead)
				this.m_poolHead += 1

				var item = &this.m_pool[idx]
				item.x = int16(x)
				item.y = int16(y)
				item.id = id
				item.next = this.m_buckets[h]
				this.m_buckets[h] = idx
			}
		}
	}
}

func (this *DtProximityGrid) queryItems(minx, miny, maxx, maxy float32,
	ids []uint16, maxIds int) int {
	var iminx = int(detour.DtMathFloorf(minx * this.m_invCellSize))
	var iminy = int(detour.DtMathFloorf(miny * this.m_invCellSize))
	var imaxx = int(detour.DtMathFloorf(maxx * this.m_invCellSize))
	var imaxy = int(detour.DtMathFloorf(maxy * this.m_invCellSize))

	var n int
	var h int
	var idx uint16
	for y := iminy; y <= imaxy; y += 1 {
		for x := iminx; x <= imaxx; x += 1 {
			h = hashPos2(x, y, this.m_bucketsSize)
			idx = this.m_buckets[h]
			for idx != 0xffff {
				var item = &this.m_pool[idx]
				if int(item.x) == x && int(item.y) == y {
					var alreadyExist bool
					for i := 0; i < n; i++ {
						if ids[i] == item.id {
							alreadyExist = true
						}
					}
					if !alreadyExist {
						if n >= maxIds {
							return n
						}
						ids[n] = item.id
						n += 1
					}
				}
				idx = item.next
			}
		}
	}

	return n
}

func (this *DtProximityGrid) getItemCountAt(x, y int) int {
	var n int

	var h = hashPos2(x, y, this.m_bucketsSize)
	var idx = this.m_buckets[h]

	for idx != 0xffff {
		var item = &this.m_pool[idx]
		if int(item.x) == x && int(item.y) == y {
			n += 1
		}
		idx = item.next
	}
	return n
}
