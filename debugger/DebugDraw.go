package debugger

import (
	"math"

	detour "github.com/o0olele/detour-go/detour"
)

const DU_PI = 3.14159265

type duDebugDrawPrimitives int

const (
	DU_DRAW_POINTS duDebugDrawPrimitives = 0
	DU_DRAW_LINES  duDebugDrawPrimitives = 1
	DU_DRAW_TRIS   duDebugDrawPrimitives = 2
	DU_DRAW_QUADS  duDebugDrawPrimitives = 3
)

type duDebugDraw interface {
	depthMask(state bool)
	texture(state bool)
	begin(prim duDebugDrawPrimitives, size float32)
	vertex0(pos []float32, color uint32)
	vertex1(x, y, z float32, color uint32)
	vertex2(pos []float32, color uint32, uv []float32)
	vertex3(x, y, z float32, color uint32, u, v float32)
	end()
	areaToCol(area uint32) uint32
}

func duRGBA(r, g, b, a uint32) uint32 {
	return uint32(r) | uint32(g)<<8 | uint32(b)<<16 | uint32(a)<<24
}

func duRGBAf(fr, fg, fb, fa float32) uint32 {
	var r = uint32(fr * 255)
	var g = uint32(fg * 255)
	var b = uint32(fb * 255)
	var a = uint32(fa * 255)
	return duRGBA(r, g, b, a)
}

func duMultCol(col, d uint32) uint32 {
	var r = col & 0xff
	var g = (col >> 8) & 0xff
	var b = (col >> 16) & 0xff
	var a = (col >> 24) & 0xff
	return duRGBA((r*d)>>8, (g*d)>>8, (b*d)>>8, a)
}

func duDarkenCol(col uint32) uint32 {
	return ((col >> 1) & 0x007f7f7f) | (col & 0xff000000)
}

func duLerpCol(ca, cb, u uint32) uint32 {
	var ra = ca & 0xff
	var ga = (ca >> 8) & 0xff
	var ba = (ca >> 16) & 0xff
	var aa = (ca >> 24) & 0xff
	var rb = cb & 0xff
	var gb = (cb >> 8) & 0xff
	var bb = (cb >> 16) & 0xff
	var ab = (cb >> 24) & 0xff

	var r = (ra*(255-u) + rb*u) / 255
	var g = (ga*(255-u) + gb*u) / 255
	var b = (ba*(255-u) + bb*u) / 255
	var a = (aa*(255-u) + ab*u) / 255
	return duRGBA(r, g, b, a)
}

func duTransCol(c, a uint32) uint32 {
	return (a << 24) | (c & 0x00ffffff)
}

func bit(a, b uint32) uint32 {
	return (a & (1 << b)) >> b
}

func duIntToCol(i, a uint32) uint32 {
	var r = bit(i, 1) + bit(i, 3)*2 + 1
	var g = bit(i, 2) + bit(i, 4)*2 + 1
	var b = bit(i, 0) + bit(i, 5)*2 + 1
	return duRGBA(r*63, g*63, b*63, a)
}

func duIntToColFloat32(i uint32, col []float32) {
	var r = bit(i, 0) + bit(i, 3)*2 + 1
	var g = bit(i, 1) + bit(i, 4)*2 + 1
	var b = bit(i, 2) + bit(i, 5)*2 + 1
	col[0] = 1 - float32(r)*63.0/255.0
	col[1] = 1 - float32(g)*63.0/255.0
	col[2] = 1 - float32(b)*63.0/255.0
}

func duCalcBoxColors(colors []uint32, colTop, colSide uint32) {
	if len(colors) <= 0 {
		return
	}

	colors[0] = duMultCol(colTop, 250)
	colors[1] = duMultCol(colSide, 140)
	colors[2] = duMultCol(colSide, 165)
	colors[3] = duMultCol(colSide, 217)
	colors[4] = duMultCol(colSide, 165)
	colors[5] = duMultCol(colSide, 217)
}

type DebugDrawerPrimitive struct {
	PrimitiveType duDebugDrawPrimitives `json:"type"`
	Vertices      [][7]float32          `json:"vertices"`
}

type duDisplayList struct {
	m_pos   []float32
	m_color []uint32
	m_size  int
	m_cap   int

	m_prim      duDebugDrawPrimitives
	m_primSize  float32
	m_depthMask bool
	m_primList  []*DebugDrawerPrimitive
}

func (disp *duDisplayList) texture(state bool) {

}

func (disp *duDisplayList) resize(cap int) {
	newPos := make([]float32, cap*3)
	if len(disp.m_pos) > 0 {
		copy(newPos, disp.m_pos)
	}
	disp.m_pos = newPos

	newColor := make([]uint32, cap)
	if len(disp.m_color) > 0 {
		copy(newColor, disp.m_color)
	}
	disp.m_color = newColor

	disp.m_cap = cap
}

func (disp *duDisplayList) clear() {
	disp.m_size = 0
}

func (disp *duDisplayList) depthMask(state bool) {
	disp.m_depthMask = state
}

func (disp *duDisplayList) begin(prim duDebugDrawPrimitives, size float32) {
	disp.clear()
	disp.m_prim = prim
	disp.m_primSize = size
}

func (disp *duDisplayList) vertex0(pos []float32, color uint32) {
	disp.vertex1(pos[0], pos[1], pos[2], color)
}

func (disp *duDisplayList) vertex1(x, y, z float32, color uint32) {
	if disp.m_size+1 >= disp.m_cap {
		disp.resize(disp.m_cap * 2)
	}
	var p = disp.m_pos[disp.m_size*3:]
	p[0] = x
	p[1] = y
	p[2] = z
	disp.m_color[disp.m_size] = color
	disp.m_size += 1
}

func (disp *duDisplayList) vertex2(pos []float32, color uint32, uv []float32) {

}

func (disp *duDisplayList) vertex3(x, y, z float32, color uint32, u, v float32) {

}

func (disp *duDisplayList) end() {

	tmp := &DebugDrawerPrimitive{
		PrimitiveType: disp.m_prim,
	}
	for i := 0; i < disp.m_size; i += 1 {
		tmp.Vertices = append(tmp.Vertices,
			[7]float32{disp.m_pos[i*3],
				disp.m_pos[i*3+1],
				disp.m_pos[i*3+2],
				float32(disp.m_color[i]&0xff) / 255,
				float32((disp.m_color[i]>>8)&0xff) / 255,
				float32((disp.m_color[i]>>16)&0xff) / 255,
				float32((disp.m_color[i]>>24)&0xff) / 255,
			},
		)
	}
	disp.m_primList = append(disp.m_primList, tmp)
}

func (disp *duDisplayList) flush() []*DebugDrawerPrimitive {
	cur := disp.m_primList
	disp.m_primList = nil
	return cur
}

func (disp *duDisplayList) areaToCol(area uint32) uint32 {
	if area == 0 {
		// Treat zero area type as default.
		return duRGBA(0, 192, 255, 255)
	} else {
		return duIntToCol(area, 255)
	}
}

func (disp *duDisplayList) draw(dd duDebugDraw) {
	if dd == nil {
		return
	}
	if disp.m_size <= 0 {
		return
	}
	dd.depthMask(disp.m_depthMask)
	dd.begin(disp.m_prim, disp.m_primSize)
	for i := 0; i < disp.m_size; i += 1 {
		dd.vertex0(disp.m_pos[i*3:], disp.m_color[i])
	}
	dd.end()
}

func NewDisplayList(cap int) *duDisplayList {
	disp := &duDisplayList{
		m_prim:      DU_DRAW_LINES,
		m_primSize:  1,
		m_depthMask: true,
	}
	if cap < 8 {
		cap = 8
	}
	disp.resize(cap)
	return disp
}

func duDebugDrawCylinderWire(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, lineWidth)
	duAppendCylinderWire(dd, minx, miny, minz, maxx, maxy, maxz, col)
	dd.end()
}

func duDebugDrawCylinder(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, 1)
	duAppendCylinder(dd, minx, miny, minz, maxx, maxy, maxz, col)
	dd.end()
}

func duDebugDrawBoxWire(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, lineWidth)
	duAppendBoxWire(dd, minx, miny, minz, maxx, maxy, maxz, col)
	dd.end()
}

func duDebugDrawBox(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, fcol []uint32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, 1)
	duAppendBox(dd, minx, miny, minz, maxx, maxy, maxz, fcol)
	dd.end()
}

func duDebugDrawArc(dd duDebugDraw, x0, y0, z0, x1, y1, z1, h, as0, as1 float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, lineWidth)
	duAppendArc(dd, x0, y0, z0, x1, y1, z1, h, as0, as1, col)
	dd.end()
}

func duDebugDrawArrow(dd duDebugDraw, x0, y0, z0, x1, y1, z1, as0, as1 float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, lineWidth)
	duAppendArrow(dd, x0, y0, z0, x1, y1, z1, as0, as1, col)
	dd.end()
}

func duDebugDrawCircle(dd duDebugDraw, x, y, z, r float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, lineWidth)
	duAppendCircle(dd, x, y, z, r, col)
	dd.end()
}

func duDebugDrawCross(dd duDebugDraw, x, y, z, size float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, lineWidth)
	duAppendCross(dd, x, y, z, size, col)
	dd.end()
}

func duDebugDrawGridXZ(dd duDebugDraw, ox, oy, oz, w, h, size float32, col uint32, lineWidth float32) {
	if dd == nil {
		return
	}

	dd.begin(DU_DRAW_LINES, lineWidth)
	for i := float32(0); i <= h; i += 1 {
		dd.vertex1(ox, oy, oz+i*size, col)
		dd.vertex1(ox+w*size, oy, oz+i*size, col)
	}
	for i := float32(0); i <= w; i += 1 {
		dd.vertex1(ox+i*size, oy, oz, col)
		dd.vertex1(ox+i*size, oy, oz+h*size, col)
	}

	dd.end()
}

func duAppendCylinderWire(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}

	const NUM_SEG = 16
	var dir [NUM_SEG * 2]float32
	var init = false
	if !init {
		init = true
		for i := 0; i < NUM_SEG; i += 1 {
			var a = float32(i) / float32(NUM_SEG) * DU_PI * 2
			dir[i*2] = detour.DtMathCosf(a)
			dir[i*2+1] = detour.DtMathSinf(a)
		}
	}

	var cx = (maxx + minx) / 2
	var cz = (maxz + minz) / 2
	var rx = (maxx - minx) / 2
	var rz = (maxz - minz) / 2

	for i, j := 0, NUM_SEG-1; i < NUM_SEG; i += 1 {
		dd.vertex1(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col)
		dd.vertex1(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col)
		dd.vertex1(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col)
		dd.vertex1(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col)
		j = i
	}
	for i := 0; i < NUM_SEG; i += NUM_SEG / 4 {
		dd.vertex1(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col)
		dd.vertex1(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col)
	}
}

func duAppendCylinder(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	const NUM_SEG = 16
	var dir [NUM_SEG * 2]float32
	var init = false
	if !init {
		init = true
		for i := 0; i < NUM_SEG; i += 1 {
			var a = float32(i) / float32(NUM_SEG) * DU_PI * 2
			dir[i*2] = detour.DtMathCosf(a)
			dir[i*2+1] = detour.DtMathSinf(a)
		}
	}

	var col2 = duMultCol(col, 160)

	var cx = (maxx + minx) / 2
	var cz = (maxz + minz) / 2
	var rx = (maxx - minx) / 2
	var rz = (maxz - minz) / 2

	for i := 2; i < NUM_SEG; i += 1 {
		var a = 0
		var b = i - 1
		var c = i
		dd.vertex1(cx+dir[a*2+0]*rx, miny, cz+dir[a*2+1]*rz, col2)
		dd.vertex1(cx+dir[b*2+0]*rx, miny, cz+dir[b*2+1]*rz, col2)
		dd.vertex1(cx+dir[c*2+0]*rx, miny, cz+dir[c*2+1]*rz, col2)
	}
	for i := 2; i < NUM_SEG; i += 1 {
		var a = 0
		var b = i
		var c = i - 1
		dd.vertex1(cx+dir[a*2+0]*rx, maxy, cz+dir[a*2+1]*rz, col)
		dd.vertex1(cx+dir[b*2+0]*rx, maxy, cz+dir[b*2+1]*rz, col)
		dd.vertex1(cx+dir[c*2+0]*rx, maxy, cz+dir[c*2+1]*rz, col)
	}
	for i, j := 0, NUM_SEG-1; i < NUM_SEG; i = i + 1 {
		dd.vertex1(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2)
		dd.vertex1(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col2)
		dd.vertex1(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col)

		dd.vertex1(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2)
		dd.vertex1(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col)
		dd.vertex1(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col)

		j = i
	}
}

func duAppendBox(dd duDebugDraw, minx, miny, minz,
	maxx, maxy, maxz float32, fcol []uint32) {
	if dd == nil {
		return
	}
	var verts = [8 * 3]float32{
		minx, miny, minz,
		maxx, miny, minz,
		maxx, miny, maxz,
		minx, miny, maxz,
		minx, maxy, minz,
		maxx, maxy, minz,
		maxx, maxy, maxz,
		minx, maxy, maxz,
	}
	var inds = [6 * 4]uint8{
		7, 6, 5, 4,
		0, 1, 2, 3,
		1, 5, 6, 2,
		3, 7, 4, 0,
		2, 6, 7, 3,
		0, 4, 5, 1,
	}

	var in = 0
	for i := 0; i < 6; i += 1 {
		dd.vertex0(verts[inds[in]*3:], fcol[i])
		in++
		dd.vertex0(verts[inds[in]*3:], fcol[i])
		in++
		dd.vertex0(verts[inds[in]*3:], fcol[i])
		in++
		dd.vertex0(verts[inds[in]*3:], fcol[i])
		in++
	}
}

func duAppendBoxPoints(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	// Top
	dd.vertex1(minx, miny, minz, col)
	dd.vertex1(maxx, miny, minz, col)
	dd.vertex1(maxx, miny, minz, col)
	dd.vertex1(maxx, miny, maxz, col)
	dd.vertex1(maxx, miny, maxz, col)
	dd.vertex1(minx, miny, maxz, col)
	dd.vertex1(minx, miny, maxz, col)
	dd.vertex1(minx, miny, minz, col)

	// bottom
	dd.vertex1(minx, maxy, minz, col)
	dd.vertex1(maxx, maxy, minz, col)
	dd.vertex1(maxx, maxy, minz, col)
	dd.vertex1(maxx, maxy, maxz, col)
	dd.vertex1(maxx, maxy, maxz, col)
	dd.vertex1(minx, maxy, maxz, col)
	dd.vertex1(minx, maxy, maxz, col)
	dd.vertex1(minx, maxy, minz, col)
}

func duAppendBoxWire(dd duDebugDraw, minx, miny, minz, maxx, maxy, maxz float32, col uint32) {
	if dd == nil {
		return
	}
	// Top
	dd.vertex1(minx, miny, minz, col)
	dd.vertex1(maxx, miny, minz, col)
	dd.vertex1(maxx, miny, minz, col)
	dd.vertex1(maxx, miny, maxz, col)
	dd.vertex1(maxx, miny, maxz, col)
	dd.vertex1(minx, miny, maxz, col)
	dd.vertex1(minx, miny, maxz, col)
	dd.vertex1(minx, miny, minz, col)

	// bottom
	dd.vertex1(minx, maxy, minz, col)
	dd.vertex1(maxx, maxy, minz, col)
	dd.vertex1(maxx, maxy, minz, col)
	dd.vertex1(maxx, maxy, maxz, col)
	dd.vertex1(maxx, maxy, maxz, col)
	dd.vertex1(minx, maxy, maxz, col)
	dd.vertex1(minx, maxy, maxz, col)
	dd.vertex1(minx, maxy, minz, col)

	// Sides
	dd.vertex1(minx, miny, minz, col)
	dd.vertex1(minx, maxy, minz, col)
	dd.vertex1(maxx, miny, minz, col)
	dd.vertex1(maxx, maxy, minz, col)
	dd.vertex1(maxx, miny, maxz, col)
	dd.vertex1(maxx, maxy, maxz, col)
	dd.vertex1(minx, miny, maxz, col)
	dd.vertex1(minx, maxy, maxz, col)
}

func evalArc(x0, y0, z0, dx, dy, dz, h, u float32, res []float32) {
	res[0] = x0 + dx*u
	res[1] = y0 + dy*u + h*(1-(u*2-1)*(u*2-1))
	res[2] = z0 + dz*u
}

func vcross(dest, v1, v2 []float32) {
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1]
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2]
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]
}

func vnormalize(v []float32) {
	var d = 1.0 / float32(math.Sqrt(float64((v[0]*v[0] + v[1]*v[1] + v[2]*v[2]))))
	v[0] *= d
	v[1] *= d
	v[2] *= d
}

func vsub(dest, v1, v2 []float32) {
	dest[0] = v1[0] - v2[0]
	dest[1] = v1[1] - v2[1]
	dest[2] = v1[2] - v2[2]
}

func vdistSqr(v1, v2 []float32) float32 {
	var x = v1[0] - v2[0]
	var y = v1[1] - v2[1]
	var z = v1[2] - v2[2]
	return x*x + y*y + z*z
}

func appendArrowHead(dd duDebugDraw, p, q []float32, s float32, col uint32) {
	const eps = 0.001
	if dd == nil {
		return
	}
	if vdistSqr(p, q) < eps*eps {
		return
	}
	var ax = [3]float32{0, 1, 0}
	var ay = [3]float32{0, 1, 0}
	var az = [3]float32{0, 0, 0}
	vsub(az[:], q, p)
	vnormalize(az[:])
	vcross(ax[:], ay[:], az[:])
	vcross(ay[:], az[:], ax[:])
	vnormalize(ay[:])

	dd.vertex0(p, col)
	//	dd.vertex(p[0]+az[0]*s+ay[0]*s/2, p[1]+az[1]*s+ay[1]*s/2, p[2]+az[2]*s+ay[2]*s/2, col);
	dd.vertex1(p[0]+az[0]*s+ax[0]*s/3, p[1]+az[1]*s+ax[1]*s/3, p[2]+az[2]*s+ax[2]*s/3, col)

	dd.vertex0(p, col)
	//	dd.vertex(p[0]+az[0]*s-ay[0]*s/2, p[1]+az[1]*s-ay[1]*s/2, p[2]+az[2]*s-ay[2]*s/2, col);
	dd.vertex1(p[0]+az[0]*s-ax[0]*s/3, p[1]+az[1]*s-ax[1]*s/3, p[2]+az[2]*s-ax[2]*s/3, col)

}

func duAppendArc(dd duDebugDraw, x0, y0, z0, x1, y1, z1, h, as0, as1 float32, col uint32) {
	if dd == nil {
		return
	}
	const NUM_ARC_PTS = 8
	const PAD = 0.05
	const ARC_PTS_SCALE = (1.0 - PAD*2) / NUM_ARC_PTS
	var dx = x1 - x0
	var dy = y1 - y0
	var dz = z1 - z0
	var len = float32(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
	var prev [3]float32
	evalArc(x0, y0, z0, dx, dy, dz, len*h, PAD, prev[:])
	for i := 1; i <= NUM_ARC_PTS; i += 1 {
		var u = PAD + float32(i)*ARC_PTS_SCALE
		var pt [3]float32
		evalArc(x0, y0, z0, dx, dy, dz, len*h, u, pt[:])
		dd.vertex1(prev[0], prev[1], prev[2], col)
		dd.vertex1(pt[0], pt[1], pt[2], col)
		prev[0] = pt[0]
		prev[1] = pt[1]
		prev[2] = pt[2]
	}

	// End arrows
	if as0 > 0.001 {
		var p, q [3]float32
		evalArc(x0, y0, z0, dx, dy, dz, len*h, PAD, p[:])
		evalArc(x0, y0, z0, dx, dy, dz, len*h, PAD+0.05, q[:])
		appendArrowHead(dd, p[:], q[:], as0, col)
	}

	if as1 > 0.001 {
		var p, q [3]float32
		evalArc(x0, y0, z0, dx, dy, dz, len*h, 1-PAD, p[:])
		evalArc(x0, y0, z0, dx, dy, dz, len*h, 1-(PAD+0.05), q[:])
		appendArrowHead(dd, p[:], q[:], as1, col)
	}
}

func duAppendArrow(dd duDebugDraw, x0, y0, z0, x1, y1, z1, as0, as1 float32, col uint32) {
	if dd == nil {
		return
	}

	dd.vertex1(x0, y0, z0, col)
	dd.vertex1(x1, y1, z1, col)

	// End arrows
	var p = [3]float32{x0, y0, z0}
	var q = [3]float32{x1, y1, z1}
	if as0 > 0.001 {
		appendArrowHead(dd, p[:], q[:], as0, col)
	}
	if as1 > 0.001 {
		appendArrowHead(dd, q[:], p[:], as1, col)
	}
}

func duAppendCircle(dd duDebugDraw, x, y, z, r float32, col uint32) {
	if dd == nil {
		return
	}
	const NUM_SEG = 40
	var dir [40 * 2]float32
	var init = false
	if !init {
		init = true
		for i := 0; i < NUM_SEG; i += 1 {
			var a = float32(i) / float32(NUM_SEG) * DU_PI * 2
			dir[i*2] = detour.DtMathCosf(a)
			dir[i*2+1] = detour.DtMathCosf(a)
		}
	}

	for i, j := 0, NUM_SEG-1; i < NUM_SEG; i = i + 1 {
		dd.vertex1(x+dir[j*2+0]*r, y, z+dir[j*2+1]*r, col)
		dd.vertex1(x+dir[i*2+0]*r, y, z+dir[i*2+1]*r, col)
		j = i
	}
}

func duAppendCross(dd duDebugDraw, x, y, z, s float32, col uint32) {
	if dd == nil {
		return
	}
	dd.vertex1(x-s, y, z, col)
	dd.vertex1(x+s, y, z, col)
	dd.vertex1(x, y-s, z, col)
	dd.vertex1(x, y+s, z, col)
	dd.vertex1(x, y, z-s, col)
	dd.vertex1(x, y, z+s, col)
}
