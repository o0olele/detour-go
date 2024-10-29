package common

import (
	"os"
	"reflect"
	"unsafe"

	detour "github.com/o0olele/detour-go/detour"
	"github.com/o0olele/detour-go/fastlz"
	dtcache "github.com/o0olele/detour-go/tilecache"
)

const TILECACHESET_MAGIC = 'T'<<24 | 'S'<<16 | 'E'<<8 | 'T' //'TSET';
const TILECACHESET_VERSION = 1

type TileCacheSetHeader struct {
	magic       int32
	version     int32
	numTiles    int32
	params      detour.DtNavMeshParams
	cacheParams dtcache.DtTileCacheParams
}

type TileCacheTileHeader struct {
	tileRef  dtcache.DtCompressedTileRef
	dataSize int32
}

type FastLZCompressor struct{}

func (c *FastLZCompressor) MaxCompressedSize(bufferSize int32) int32 {
	return int32(float64(bufferSize) * 1.05)
}
func (c *FastLZCompressor) Compress(buffer []byte, bufferSize int32, compressed []byte, maxCompressedSize int32, compressedSize *int32) detour.DtStatus {
	*compressedSize = int32(fastlz.Fastlz_compress(buffer, int(bufferSize), compressed))
	return detour.DT_SUCCESS
}
func (c *FastLZCompressor) Decompress(compressed []byte, compressedSize int32, buffer []byte, maxBufferSize int32, bufferSize *int32) detour.DtStatus {
	*bufferSize = int32(fastlz.Fastlz_decompress(compressed, int(compressedSize), buffer, int(maxBufferSize)))
	if *bufferSize < 0 {
		return detour.DT_FAILURE
	} else {
		return detour.DT_SUCCESS
	}
}

const (
	POLYAREA_GROUND uint8 = 0
	POLYAREA_WATER  uint8 = 1
	POLYAREA_ROAD   uint8 = 2
	POLYAREA_DOOR   uint8 = 3
	POLYAREA_GRASS  uint8 = 4
	POLYAREA_JUMP   uint8 = 5
)

const (
	POLYFLAGS_WALK     uint16 = 0x01   // Ability to walk (ground, grass, road)
	POLYFLAGS_SWIM     uint16 = 0x02   // Ability to swim (water).
	POLYFLAGS_DOOR     uint16 = 0x04   // Ability to move through doors.
	POLYFLAGS_JUMP     uint16 = 0x08   // Ability to jump.
	POLYFLAGS_DISABLED uint16 = 0x10   // Disabled polygon
	POLYFLAGS_ALL      uint16 = 0xffff // All abilities.
)

type MeshProcess struct{}

func (p *MeshProcess) Process(params *detour.DtNavMeshCreateParams, polyAreas []uint8, polyFlags []uint16) {
	// Update poly flags from areas.
	for i := 0; i < int(params.PolyCount); i++ {
		if polyAreas[i] == dtcache.DT_TILECACHE_WALKABLE_AREA {
			polyAreas[i] = POLYAREA_GROUND
		}
		if polyAreas[i] == POLYAREA_GROUND ||
			polyAreas[i] == POLYAREA_GRASS ||
			polyAreas[i] == POLYAREA_ROAD {
			polyFlags[i] = POLYFLAGS_WALK
		} else if polyAreas[i] == POLYAREA_WATER {
			polyFlags[i] = POLYFLAGS_SWIM
		} else if polyAreas[i] == POLYAREA_DOOR {
			polyFlags[i] = POLYFLAGS_WALK | POLYFLAGS_DOOR
		}
	}
}

func LoadTempObstaclesByBytes(data []byte) (*detour.DtNavMesh, *dtcache.DtTileCache) {

	header := (*TileCacheSetHeader)(unsafe.Pointer(&(data[0])))
	if header.magic != TILECACHESET_MAGIC {
		return nil, nil
	}
	if header.version != TILECACHESET_VERSION {
		return nil, nil
	}

	var navMesh = detour.DtAllocNavMesh()
	if navMesh == nil {
		return nil, nil
	}

	var status = navMesh.Init(&header.params)
	if detour.DtStatusFailed(status) {
		return nil, nil
	}

	var tileCache = dtcache.DtAllocTileCache()
	if tileCache == nil {
		return nil, nil
	}

	status = tileCache.Init(&header.cacheParams, &FastLZCompressor{}, &MeshProcess{})
	if detour.DtStatusFailed(status) {
		return nil, nil
	}

	var offset = int(unsafe.Sizeof(*header))
	for i := 0; i < int(header.numTiles); i++ {
		tileHeader := (*TileCacheTileHeader)(unsafe.Pointer(&(data[offset])))
		offset += int(unsafe.Sizeof(*tileHeader))
		if tileHeader.tileRef == 0 || tileHeader.dataSize == 0 {
			break
		}

		var tempData []byte
		sliceHeader := (*reflect.SliceHeader)((unsafe.Pointer(&tempData)))
		sliceHeader.Cap = int(tileHeader.dataSize)
		sliceHeader.Len = int(tileHeader.dataSize)
		sliceHeader.Data = uintptr(unsafe.Pointer(&data[offset]))
		offset += int(tileHeader.dataSize)
		data := make([]byte, tileHeader.dataSize)
		copy(data, tempData)

		var tile dtcache.DtCompressedTileRef
		status = tileCache.AddTile(data, tileHeader.dataSize, dtcache.DT_COMPRESSEDTILE_FREE_DATA, &tile)
		detour.DtAssert(detour.DtStatusSucceed(status))

		if tile != 0 {
			tileCache.BuildNavMeshTile(tile, navMesh)
		} else {
			detour.DtAssert(false)
		}
	}
	return navMesh, tileCache
}

func LoadTempObstacles(path string) (*detour.DtNavMesh, *dtcache.DtTileCache) {
	meshData, err := os.ReadFile(path)
	detour.DtAssert(err == nil)

	return LoadTempObstaclesByBytes(meshData)
}
