
#ifndef DETOURNAVMESHQUERY_NONPOINT_H
#define DETOURNAVMESHQUERY_NONPOINT_H

#include "DetourNavMeshQuery.h"
#include "DetourAssert.h"

// 处理带有半径的单位寻路

struct dtInternalPrimitive
{
	static dtInternalPrimitive INVALID;

	dtInternalPrimitive(const dtNavMesh* inNavmesh, const dtPolyRef& inPolyId, const int& inInnerIdx)
		: navmesh(inNavmesh), polyId(inPolyId), innerIdx(inInnerIdx)
	{
	}

	dtInternalPrimitive(const dtInternalPrimitive& inOther)
	{
		*this = inOther;
	}

	dtInternalPrimitive& operator=(const dtInternalPrimitive& inOther)
	{
		navmesh = inOther.navmesh;
		polyId = inOther.polyId;
		innerIdx = inOther.innerIdx;
		return *this;
	}

	bool isValid() const
	{
		return navmesh != nullptr && polyId != 0 && innerIdx != -1;
	}

	bool getTileAndPoly(const dtMeshTile** tile, const dtPoly** poly) const
	{
		if (isValid())
		{
			if (dtStatusSucceed(navmesh->getTileAndPolyByRef(polyId, tile, poly)))
			{
				return true;
			}
		}
		return false;
	}

	bool operator==(const dtInternalPrimitive& inOther)
	{
		return navmesh == inOther.navmesh &&
			polyId == inOther.polyId &&
			innerIdx == inOther.innerIdx;
	}

	const dtNavMesh* navmesh;
	dtPolyRef			polyId;
	int					innerIdx;
};

dtInternalPrimitive dtInternalPrimitive::INVALID(nullptr, 0, -1);

typedef dtInternalPrimitive dtInternalVertex;
typedef dtInternalPrimitive dtInternalEdge;
typedef dtInternalPrimitive dtInternalFace;


struct dtPolygonTraits
{
	/*
		内部边成对增加，表示正向和反向

		3边形：
				0 +----+ 1
				   \   |     内部边：
					 \ |     内部面：[0]: 0-1-2
					  \+ 2   面的边：[0]: 0-1-2

		4边形：
				0 +-----+ 1
				  | \   |    内部边：[0]&[1]: 2-0, 0-2
				  |   \ |    内部面：[0]: 0-1-2, [1]: 0-2-3
				3 +-----+ 2  面的边：[0]: 0-1-6, [1]: 7-2-3

		5边形：
				0  _____ 1
				  |\` . \
				  | \  `.\ 2   内部边：[0]&[1]: 2-0, 0-2, [2]&[3]: 3-0, 0-3
				  |   \  /     内部面：[0]: 0-1-2, [1]: 0-2-3, [2]: 0-3-4
				  |____\/      面的边：[0]: 0-1-6, [1]: 7-2-8, [2]：9-3-4
				 4       3

		6边形：
				0 ______ 1
				 /` .   \
			  5 / |\  ` .\ 2   内部边：[0]&[1]: 2-0, 0-2, [2]&[3]: 3-0, 0-3, [4]&[5]: 4-0, 0-4
				\ |  \   /     内部面：[0]: 0-1-2, [1]: 0-2-3, [2]: 0-3-4, [3]: 0-4-5
				 \|____\/      面的边：[0]: 0-1-6, [1]: 7-2-8, [2]：9-3-10, [3]: 10-4-5
				 4       3
	*/

	inline static dtInternalVertex edgeOriginVertex(const dtInternalEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				return dtInternalVertex(edge.navmesh, edge.polyId, edge.innerIdx);
			}
			else
			{
				// 新增内部边(起点)
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				return (idx & 0x1) == 1 ? dtInternalVertex(edge.navmesh, edge.polyId, 0) : dtInternalVertex(edge.navmesh, edge.polyId, idx + 2);
			}
		}
		return dtInternalVertex::INVALID;
	}

	inline static dtInternalVertex edgeDestinationVertex(const dtInternalEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				return dtInternalVertex(edge.navmesh, edge.polyId, (edge.innerIdx + 1) % poly->vertCount);
			}
			else
			{
				// 新增内部边(终点)
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				return (idx & 0x1) == 0 ? dtInternalVertex(edge.navmesh, edge.polyId, 0) : dtInternalVertex(edge.navmesh, edge.polyId, idx + 2);
			}
		}
		return dtInternalVertex::INVALID;
	}

	inline static int sharedEdgeIndex(const dtNavMesh* nav, const dtPolyRef from, const dtPolyRef to)
	{
		const dtMeshTile* fromTile = 0;
		const dtPoly* fromPoly = 0;
		nav->getTileAndPolyByRefUnsafe(from, &fromTile, &fromPoly);

		for (unsigned int k = fromPoly->firstLink; k != DT_NULL_LINK; k = fromTile->links[k].next)
		{
			const dtLink* link = &fromTile->links[k];
			if (link->ref == to)
			{
				return link->edge;
			}
		}
		return -1;
	}

	inline static dtInternalEdge edgeOppositeEdge(const dtInternalEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				unsigned char edgeIdx = edge.innerIdx;
				unsigned short nei = poly->neis[edgeIdx];

				if ((nei & DT_EXT_LINK) == 0)
				{
					// Tile border.
					for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
					{
						const dtLink* link = &tile->links[k];
						if (link->edge == edgeIdx)
						{
							if (link->ref != 0)
							{
								int neiEdgeIdx = sharedEdgeIndex(edge.navmesh, link->ref, edge.polyId);
								return dtInternalEdge(edge.navmesh, link->ref, neiEdgeIdx);
							}
						}
					}
				}
				else
				{
					// Portal edge
					const unsigned int idx = (unsigned int)(nei - 1);
					const dtPolyRef ref = edge.navmesh->getPolyRefBase(tile) | idx;

					int neiEdgeIdx = sharedEdgeIndex(edge.navmesh, ref, edge.polyId);
					return dtInternalEdge(edge.navmesh, ref, neiEdgeIdx);
				}
			}
			else
			{
				// 新增内部边
				auto idx = edge.innerIdx;
				return (idx & 0x1) == 0 ? dtInternalEdge(edge.navmesh, edge.polyId, idx + 1) : dtInternalVertex(edge.navmesh, edge.polyId, idx - 1);
			}
		}
		return dtInternalEdge::INVALID;
	}

	inline static dtInternalEdge edgeNextLeftEdge(const dtInternalEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			int edges[3];
			if (poly->vertCount == 3)
			{
				edges[0] = 0; edges[1] = 1; edges[2] = 2;
			}
			else
			{
				auto face = edgeLeftFace(edge);
				auto i = face.innerIdx;
				// N = poly->vertCount, 表示总边数
				// i == 0				=> [0, 1, 6]
				// i > 0 && i < N-3		=> [7 + (i-1)*2, i, 8 + (i-1)*2]
				// i == N-3				=> [7 + (i-1)*2, i+1, i+2]

				if (i == 0)
				{
					edges[0] = 0; edges[1] = 1; edges[2] = 6;
				}
				else if (i == poly->vertCount - 3)
				{
					edges[0] = 7 + (i - 1) * 2; edges[1] = i; edges[2] = 8 + (i - 1) * 2;
				}
				else
				{
					edges[0] = 7 + (i - 1) * 2; edges[1] = i + 1; edges[2] = i + 2;
				}
			}

			for (int k = 0; k < 3; ++k)
			{
				if (edges[k] == edge.innerIdx)
				{
					return dtInternalEdge(edge.navmesh, edge.polyId, edges[(k + 1) % 3]);
				}
			}
		}
		return dtInternalEdge::INVALID;
	}

	inline static dtInternalEdge edgePrevLeftEdge(const dtInternalEdge& edge)
	{
		do
		{
			auto currEdge = edgeNextLeftEdge(edge);
			if (!currEdge.isValid()) break;
			return edgeNextLeftEdge(currEdge);
		} while (false);
		return dtInternalEdge::INVALID;
	}

	inline static dtInternalEdge edgeNextRightEdge(const dtInternalEdge& edge)
	{
		do
		{
			auto currEdge = edgeOppositeEdge(edge);
			if (!currEdge.isValid()) break;
			currEdge = edgeNextLeftEdge(currEdge);
			if (!currEdge.isValid()) break;
			currEdge = edgeNextLeftEdge(currEdge);
			if (!currEdge.isValid()) break;
			return edgeOppositeEdge(currEdge);
		} while (false);
		return dtInternalEdge::INVALID;
	}

	inline static dtInternalEdge edgePrevRightEdge(const dtInternalEdge& edge)
	{
		do
		{
			auto currEdge = edgeOppositeEdge(edge);
			if (!currEdge.isValid()) break;
			currEdge = edgeNextLeftEdge(currEdge);
			return edgeOppositeEdge(currEdge);
		} while (false);
		return dtInternalEdge::INVALID;
	}

	inline static dtInternalFace edgeLeftFace(const dtInternalEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				if (edge.innerIdx < 2)
				{
					return dtInternalFace(edge.navmesh, edge.polyId, 0);
				}
				else if (edge.innerIdx > poly->vertCount - 3)
				{
					return dtInternalFace(edge.navmesh, edge.polyId, poly->vertCount - 3);
				}
				else
				{
					return dtInternalFace(edge.navmesh, edge.polyId, edge.innerIdx - 1);
				}
			}
			else
			{
				// 新增内部边
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				return (idx & 0x1) == 0 ? dtInternalFace(edge.navmesh, edge.polyId, idx / 2) : dtInternalFace(edge.navmesh, edge.polyId, idx / 2 + 1);
			}
		}
	}

	inline static dtInternalFace edgeRightFace(const dtInternalEdge& edge)
	{
		do
		{
			auto currEdge = edgeOppositeEdge(edge);
			if (!currEdge.isValid()) break;
			return edgeLeftFace(currEdge);
		} while (false);
		return dtInternalFace::INVALID;
	}

	inline static dtInternalEdge faceEdge(const dtInternalFace& face)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (face.getTileAndPoly(&tile, &poly))
		{
			auto edgeIndex = -1;
			if (poly->vertCount == 3)
			{
				edgeIndex = 0;
			}
			else
			{
				auto i = face.innerIdx;
				// N = poly->vertCount, 表示总边数
				// i == 0				=> [0, 1, 6]
				// i > 0 && i < N-3		=> [7 + (i-1)*2, i, 8 + (i-1)*2]
				// i == N-3				=> [7 + (i-1)*2, i+1, i+2]

				if (i == 0)
				{
					edgeIndex = 0;
				}
				else
				{
					edgeIndex = 7 + (i - 1) * 2;
				}
			}
			return dtInternalEdge(face.navmesh, face.polyId, edgeIndex);
		}
		return dtInternalEdge::INVALID;
	}
};
#endif // DETOURNAVMESHQUERY_NONPOINT_H