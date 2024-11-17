package debugger

import detour "github.com/o0olele/detour-go/detour"

type DrawNavMeshFlags int

const (
	DU_DRAWNAVMESH_OFFMESHCONS DrawNavMeshFlags = 0x01
	DU_DRAWNAVMESH_CLOSEDLIST  DrawNavMeshFlags = 0x02
	DU_DRAWNAVMESH_COLOR_TILES DrawNavMeshFlags = 0x04
)

func distancePtLine2d(pt, p, q []float32) float32 {
	var pqx = q[0] - p[0]
	var pqz = q[2] - p[2]
	var dx = pt[0] - p[0]
	var dz = pt[2] - p[2]
	var d = pqx*pqx + pqz*pqz
	var t = pqx*dx + pqz*dz
	if d != 0 {
		t /= d
	}
	dx = p[0] + t*pqx - pt[0]
	dz = p[2] + t*pqz - pt[2]
	return dx*dx + dz*dz
}

func drawPolyBoundaries(dd duDebugDraw, tile *detour.DtMeshTile, col uint32, linew float32, inner bool) {
	const thr = 0.01 * 0.01
	dd.begin(DU_DRAW_LINES, linew)

	for i := 0; i < int(tile.Header.PolyCount); i += 1 {
		var p = &tile.Polys[i]
		if p.GetType() == detour.DT_POLYTYPE_OFFMESH_CONNECTION {
			continue
		}

		var pd = &tile.DetailMeshes[i]

		for j, nj := 0, p.VertCount; j < int(nj); j += 1 {
			var c = col
			if inner {
				if p.Neis[j] <= 0 {
					continue
				}
				if p.Neis[j]&detour.DT_EXT_LINK > 0 {
					var con bool
					for k := uint32(p.FirstLink); k != detour.DT_NULL_LINK; k = tile.Links[k].Next {
						if tile.Links[k].Edge == uint8(j) {
							con = true
							break
						}
					}
					if con {
						c = duRGBA(255, 255, 255, 48)
					} else {
						c = duRGBA(0, 0, 0, 48)
					}
				} else {
					c = duRGBA(0, 48, 64, 32)
				}
			} else {
				if p.Neis[j] != 0 {
					continue
				}
			}

			var v0 = tile.Verts[int(p.Verts[j])*3:]
			var v1 = tile.Verts[int(p.Verts[(j+1)%int(nj)])*3:]

			for k := 0; k < int(pd.TriCount); k += 1 {
				var t = tile.DetailTris[(int(pd.TriBase)+k)*4:]
				var tv [3][]float32
				for m := 0; m < 3; m += 1 {
					if t[m] < p.VertCount {
						tv[m] = tile.Verts[int(p.Verts[t[m]])*3:]
					} else {
						tv[m] = tile.DetailVerts[(int(pd.VertBase)+int(t[m])-int(p.VertCount))*3:]
					}
				}
				for m, n := 0, 2; m < 3; m, n = m+1, m {
					if detour.DtGetDetailTriEdgeFlags(t[3], n)&int(detour.DT_DETAIL_EDGE_BOUNDARY) == 0 {
						continue
					}
					if distancePtLine2d(tv[n], v0, v1) < thr && distancePtLine2d(tv[m], v0, v1) < thr {
						dd.vertex0(tv[n], c)
						dd.vertex0(tv[m], c)
					}
				}
			}

		}
	}

	dd.end()
}

func drawMeshTile(dd duDebugDraw, mesh *detour.DtNavMesh, query *detour.DtNavMeshQuery, tile *detour.DtMeshTile, flags DrawNavMeshFlags) {
	var base = mesh.GetPolyRefBase(tile)
	var tileNum = mesh.DecodePolyIdTile(base)
	var tileColor = duIntToCol(tileNum, 128)

	dd.depthMask(false)
	dd.begin(DU_DRAW_TRIS, 1)

	for i := 0; i < int(tile.Header.PolyCount); i += 1 {
		var p = &tile.Polys[i]
		if p.GetType() == detour.DT_POLYTYPE_OFFMESH_CONNECTION {
			continue
		}

		var pd = &tile.DetailMeshes[i]
		var col uint32
		if query != nil && query.IsInClosedList(base|detour.DtPolyRef(i)) {
			col = duRGBA(255, 196, 0, 64)
		} else {
			if flags&DU_DRAWNAVMESH_COLOR_TILES > 0 {
				col = tileColor
			} else {
				col = duTransCol(dd.areaToCol(uint32(p.GetArea())), 64)
			}
		}

		for j := 0; j < int(pd.TriCount); j += 1 {
			var t = tile.DetailTris[(int(pd.TriBase)+j)*4:]
			for k := 0; k < 3; k += 1 {
				if t[k] < p.VertCount {
					dd.vertex0(tile.Verts[int(p.Verts[t[k]])*3:], col)
				} else {
					dd.vertex0(tile.DetailVerts[(int(pd.VertBase)+int(t[k])-int(p.VertCount))*3:], col)
				}
			}
		}
	}
	dd.end()

	// Draw inter poly boundaries
	drawPolyBoundaries(dd, tile, duRGBA(0, 48, 64, 32), 1.5, true)

	// Draw outer poly boundaries
	drawPolyBoundaries(dd, tile, duRGBA(0, 48, 64, 220), 2.5, false)

	if flags&DU_DRAWNAVMESH_OFFMESHCONS > 0 {
		dd.begin(DU_DRAW_LINES, 2.0)
		for i := 0; i < int(tile.Header.PolyCount); i += 1 {
			var p = &tile.Polys[i]
			if p.GetType() == detour.DT_POLYTYPE_OFFMESH_CONNECTION {
				continue
			}

			var col1, col2 uint32
			if query != nil && query.IsInClosedList(base|detour.DtPolyRef(i)) {
				col1 = duRGBA(255, 196, 0, 220)
			} else {
				col1 = duDarkenCol(duTransCol(dd.areaToCol(uint32(p.GetArea())), 220))
			}

			var con = &tile.OffMeshCons[i-int(tile.Header.OffMeshBase)]
			var va = tile.Verts[int(p.Verts[0])*3:]
			var vb = tile.Verts[int(p.Verts[1])*3:]

			var startSet bool
			var endSet bool
			for k := p.FirstLink; k != detour.DT_NULL_LINK; k = tile.Links[k].Next {
				if tile.Links[k].Edge <= 0 {
					startSet = true
				}
				if tile.Links[k].Edge == 1 {
					endSet = true
				}
			}

			// End points and their on-mesh locations.
			dd.vertex1(va[0], va[1], va[2], col1)
			dd.vertex1(con.Pos[0], con.Pos[1], con.Pos[2], col1)
			col2 = duRGBA(220, 32, 16, 196)
			if startSet {
				col2 = col1
			}
			duAppendCircle(dd, con.Pos[0], con.Pos[1]+0.1, con.Pos[2], con.Rad, col2)

			dd.vertex1(vb[0], vb[1], vb[2], col1)
			dd.vertex1(con.Pos[3], con.Pos[4], con.Pos[5], col1)
			col2 = duRGBA(220, 32, 16, 196)
			if endSet {
				col2 = col1
			}
			duAppendCircle(dd, con.Pos[3], con.Pos[4]+0.1, con.Pos[5], con.Rad, col2)

			// End point vertices.
			dd.vertex1(con.Pos[0], con.Pos[1], con.Pos[2], duRGBA(0, 48, 64, 196))
			dd.vertex1(con.Pos[0], con.Pos[1]+0.2, con.Pos[2], duRGBA(0, 48, 64, 196))

			dd.vertex1(con.Pos[3], con.Pos[4], con.Pos[5], duRGBA(0, 48, 64, 196))
			dd.vertex1(con.Pos[3], con.Pos[4]+0.2, con.Pos[5], duRGBA(0, 48, 64, 196))

			// Connection arc.
			var as1 = float32(0)
			if con.Flags&1 > 0 {
				as1 = 0.6
			}
			duAppendArc(dd, con.Pos[0], con.Pos[1], con.Pos[2], con.Pos[3], con.Pos[4], con.Pos[5], 0.25, as1, 0.6, col1)
		}
		dd.end()
	}

	var vcol = duRGBA(0, 0, 0, 196)
	dd.begin(DU_DRAW_POINTS, 3.0)
	for i := 0; i < int(tile.Header.VertCount); i += 1 {
		var v = tile.Verts[i*3:]
		dd.vertex0(v, vcol)
	}
	dd.end()

	dd.depthMask(true)
}

func duDebugDrawNavMesh(dd duDebugDraw, mesh *detour.DtNavMesh, flags DrawNavMeshFlags) {
	if dd == nil {
		return
	}

	for i := 0; i < int(mesh.GetMaxTiles()); i += 1 {
		var tile = mesh.GetTile(i)
		if tile.Header == nil {
			continue
		}
		drawMeshTile(dd, mesh, nil, tile, flags)
	}
}

func duDebugDrawNavMeshWithClosedList(dd duDebugDraw, mesh *detour.DtNavMesh, query *detour.DtNavMeshQuery, flags DrawNavMeshFlags) {
	if dd == nil {
		return
	}
	var q *detour.DtNavMeshQuery
	if flags&DU_DRAWNAVMESH_CLOSEDLIST > 0 {
		q = query
	}

	for i := 0; i < int(mesh.GetMaxTiles()); i += 1 {
		var tile = mesh.GetTile(i)
		if tile.Header == nil {
			continue
		}
		drawMeshTile(dd, mesh, q, tile, flags)
	}
}
