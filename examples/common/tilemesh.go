package common

import (
	"os"
	"unsafe"

	detour "github.com/o0olele/detour-go/Detour"
)

const NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T' //'MSET';
const NAVMESHSET_VERSION = 1

type NavMeshSetHeader struct {
	magic    int32
	version  int32
	numTiles int32
	params   detour.DtNavMeshParams
}

type NavMeshTileHeader struct {
	tileRef  detour.DtTileRef
	dataSize int32
}

func LoadTileMeshByBytes(data []byte) *detour.DtNavMesh {
	header := (*NavMeshSetHeader)(unsafe.Pointer(&(data[0])))
	if header.magic != NAVMESHSET_MAGIC {
		return nil
	}
	if header.version != NAVMESHSET_VERSION {
		return nil
	}

	var navMesh = detour.DtAllocNavMesh()
	if navMesh == nil {
		return nil
	}

	var status = navMesh.Init(&header.params)
	if detour.DtStatusFailed(status) {
		return nil
	}

	var offset = int32(unsafe.Sizeof(*header))
	for i := 0; i < int(header.numTiles); i += 1 {
		var tileHeader = (*NavMeshTileHeader)(unsafe.Pointer(&(data[offset])))
		if tileHeader.tileRef <= 0 || tileHeader.dataSize <= 0 {
			break
		}
		offset += int32(unsafe.Sizeof(*tileHeader))

		tileData := data[offset : offset+tileHeader.dataSize]
		navMesh.AddTile(tileData, int(tileHeader.dataSize), detour.DT_TILE_FREE_DATA, tileHeader.tileRef, nil)
		offset += tileHeader.dataSize
	}

	return navMesh
}

func LoadTileMesh(path string) *detour.DtNavMesh {
	meshData, err := os.ReadFile(path)
	detour.DtAssert(err == nil)

	return LoadTileMeshByBytes(meshData)
}
