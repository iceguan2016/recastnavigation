
#ifndef DETOURNAVMESHQUERY_NONPOINT_H
#define DETOURNAVMESHQUERY_NONPOINT_H

#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"

// 处理带有半径的单位寻路
typedef unsigned short dtPrimIndex;
static const dtPrimIndex DT_INVALID_PRIM_INDEX = ~0;

struct dtPolyPrimitive
{
	static dtPolyPrimitive INVALID;

	dtPolyPrimitive()
		: navmesh(nullptr), polyId(0), innerIdx(DT_INVALID_PRIM_INDEX)
	{
	}

	dtPolyPrimitive(const dtNavMesh* inNavmesh, const dtPolyRef& inPolyId, const dtPrimIndex& inInnerIdx)
		: navmesh(inNavmesh), polyId(inPolyId), innerIdx(inInnerIdx)
	{
	}

	dtPolyPrimitive(const dtPolyPrimitive& inOther)
	{
		*this = inOther;
	}

	dtPolyPrimitive& operator=(const dtPolyPrimitive& inOther)
	{
		navmesh = inOther.navmesh;
		polyId = inOther.polyId;
		innerIdx = inOther.innerIdx;
		return *this;
	}

	explicit operator bool() const {
		return isValid();
	}

	bool isValid() const
	{
		return navmesh != nullptr && polyId != 0 && innerIdx != DT_INVALID_PRIM_INDEX;
	}

	void reset()
	{
		navmesh = nullptr;
		polyId = 0;
		innerIdx = -1;
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

	bool operator==(const dtPolyPrimitive& inOther) const
	{
		return navmesh == inOther.navmesh &&
			polyId == inOther.polyId &&
			innerIdx == inOther.innerIdx;
	}

	bool operator!=(const dtPolyPrimitive& inOther) const
	{
		return navmesh != inOther.navmesh ||
			polyId != inOther.polyId ||
			innerIdx != inOther.innerIdx;
	}

	const dtNavMesh*	navmesh;
	dtPolyRef			polyId;
	dtPrimIndex		innerIdx;
};

typedef dtPolyPrimitive dtPolyVertex;
typedef dtPolyPrimitive dtPolyEdge;
typedef dtPolyPrimitive dtPolyFace;


namespace queriers
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

	// Vertex
	inline static bool vertexPosition(const dtPolyVertex& vertex, float* pos) 
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (vertex.getTileAndPoly(&tile, &poly))
		{
			dtAssert(vertex.innerIdx < poly->vertCount);
			auto vi = poly->verts[vertex.innerIdx];
			auto v = &tile->verts[vi*3];
			dtVcopy(pos, v);
			return true;
		}
		return false;
	}

	// Edge
	inline static dtPolyVertex edgeOriginVertex(const dtPolyEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				return dtPolyVertex(edge.navmesh, edge.polyId, edge.innerIdx);
			}
			else
			{
				// 新增内部边(起点)
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				return (idx & 0x1) == 1 ? dtPolyVertex(edge.navmesh, edge.polyId, 0) : dtPolyVertex(edge.navmesh, edge.polyId, idx/2 + 2);
			}
		}
		return dtPolyVertex::INVALID;
	}

	inline static dtPolyVertex edgeDestinationVertex(const dtPolyEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				return dtPolyVertex(edge.navmesh, edge.polyId, (edge.innerIdx + 1) % poly->vertCount);
			}
			else
			{
				// 新增内部边(终点)
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				return (idx & 0x1) == 0 ? dtPolyVertex(edge.navmesh, edge.polyId, 0) : dtPolyVertex(edge.navmesh, edge.polyId, idx/2 + 2);
			}
		}
		return dtPolyVertex::INVALID;
	}

	inline static bool edgeOriginAndDestinationVertex(const dtPolyEdge& edge, dtPolyVertex* origin, dtPolyVertex* destination)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				if (origin)
					*origin = dtPolyVertex(edge.navmesh, edge.polyId, edge.innerIdx);
				if (destination)
					*destination = dtPolyVertex(edge.navmesh, edge.polyId, (edge.innerIdx + 1) % poly->vertCount);
				return true;
			}
			else
			{
				// 新增内部边(起点)
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				if ((idx & 0x1) == 1)
				{
					if (origin)
						*origin = dtPolyVertex(edge.navmesh, edge.polyId, 0);
					if (destination)
						*destination = dtPolyVertex(edge.navmesh, edge.polyId, idx / 2 + 2);
				} 
				else
				{
					if (origin)
						*origin = dtPolyVertex(edge.navmesh, edge.polyId, idx / 2 + 2);
					if (destination)
						*destination = dtPolyVertex(edge.navmesh, edge.polyId, 0);
				}
				return true;
			}
		}
		return false;
	}

	inline static dtPolyFace edgeLeftFace(const dtPolyEdge& edge)
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
					return dtPolyFace(edge.navmesh, edge.polyId, 0);
				}
				else if (edge.innerIdx > poly->vertCount - 3)
				{
					return dtPolyFace(edge.navmesh, edge.polyId, poly->vertCount - 3);
				}
				else
				{
					return dtPolyFace(edge.navmesh, edge.polyId, edge.innerIdx - 1);
				}
			}
			else
			{
				// 新增内部边
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				return (idx & 0x1) == 0 ? dtPolyFace(edge.navmesh, edge.polyId, idx / 2) : dtPolyFace(edge.navmesh, edge.polyId, idx / 2 + 1);
			}
		}
		return dtPolyFace::INVALID;
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

	inline static dtPolyEdge edgeOppositeEdge(const dtPolyEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				unsigned short edgeIdx = (unsigned short)edge.innerIdx;
				unsigned short nei = poly->neis[edgeIdx];

				if (nei & DT_EXT_LINK)
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
								return dtPolyEdge(edge.navmesh, link->ref, neiEdgeIdx);
							}
						}
					}
				}
				else if (nei > 0)
				{
					// Portal edge
					const unsigned int idx = (unsigned int)(nei - 1);
					const dtPolyRef ref = edge.navmesh->getPolyRefBase(tile) | idx;

					int neiEdgeIdx = sharedEdgeIndex(edge.navmesh, ref, edge.polyId);
					return dtPolyEdge(edge.navmesh, ref, neiEdgeIdx);
				}
			}
			else
			{
				// 新增内部边
				auto idx = edge.innerIdx;
				return (idx & 0x1) == 0 ? dtPolyEdge(edge.navmesh, edge.polyId, idx + 1) : dtPolyVertex(edge.navmesh, edge.polyId, idx - 1);
			}
		}
		return dtPolyEdge::INVALID;
	}

	inline static dtPolyEdge edgeNextLeftEdge(const dtPolyEdge& edge)
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
				// i > 0 && i < N-3		=> [7 + (i-1)*2, i+1, 8 + (i-1)*2]
				// i == N-3				=> [7 + (i-1)*2, i+1, i+2]

				if (i == 0)
				{
					edges[0] = 0; edges[1] = 1; edges[2] = 6;
				}
				else if (i == poly->vertCount - 3)
				{
					edges[0] = 7 + (i - 1) * 2; edges[1] = i + 1; edges[2] = i + 2;
				}
				else
				{
					edges[0] = 7 + (i - 1) * 2; edges[1] = i + 1; edges[2] = 8 + (i - 1) * 2;
				}
			}

			for (int k = 0; k < 3; ++k)
			{
				if (edges[k] == edge.innerIdx)
				{
					return dtPolyEdge(edge.navmesh, edge.polyId, edges[(k + 1) % 3]);
				}
			}
		}
		return dtPolyEdge::INVALID;
	}

	inline static dtPolyEdge edgePrevLeftEdge(const dtPolyEdge& edge)
	{
		do
		{
			auto currEdge = edgeNextLeftEdge(edge);
			if (!currEdge.isValid()) break;
			return edgeNextLeftEdge(currEdge);
		} while (false);
		return dtPolyEdge::INVALID;
	}

	inline static dtPolyEdge edgeNextRightEdge(const dtPolyEdge& edge)
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
		return dtPolyEdge::INVALID;
	}

	inline static dtPolyEdge edgePrevRightEdge(const dtPolyEdge& edge)
	{
		do
		{
			auto currEdge = edgeOppositeEdge(edge);
			if (!currEdge.isValid()) break;
			currEdge = edgeNextLeftEdge(currEdge);
			return edgeOppositeEdge(currEdge);
		} while (false);
		return dtPolyEdge::INVALID;
	}

	inline static dtPolyFace edgeRightFace(const dtPolyEdge& edge)
	{
		do
		{
			auto currEdge = edgeOppositeEdge(edge);
			if (!currEdge.isValid()) break;
			return edgeLeftFace(currEdge);
		} while (false);
		return dtPolyFace::INVALID;
	}

	inline static bool edgeIsBoundary(const dtPolyEdge& edge)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				unsigned short edgeIdx = (unsigned short)edge.innerIdx;
				unsigned short nei = poly->neis[edgeIdx];
				return nei == 0;
			}
			else
			{
				// 新增内部边(起点)
				return false;
			}
		}
		return false;
	}

	// Face
	inline static dtPolyEdge faceEdge(const dtPolyFace& face)
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
			return dtPolyEdge(face.navmesh, face.polyId, edgeIndex);
		}
		return dtPolyEdge::INVALID;
	}

	inline static void faceAllEdges(const dtPolyFace& face, dtPolyEdge outEdges[3])
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (face.getTileAndPoly(&tile, &poly))
		{
			int edges[3];
			if (poly->vertCount == 3)
			{
				edges[0] = 0; edges[1] = 1; edges[2] = 2;
			}
			else
			{
				auto i = face.innerIdx;
				// N = poly->vertCount, 表示总边数
				// i == 0				=> [0, 1, 6]
				// i > 0 && i < N-3		=> [7 + (i-1)*2, i+1, 8 + (i-1)*2]
				// i == N-3				=> [7 + (i-1)*2, i+1, i+2]

				if (i == 0)
				{
					edges[0] = 0; edges[1] = 1; edges[2] = 6;
				}
				else if (i == poly->vertCount - 3)
				{
					edges[0] = 7 + (i - 1) * 2; edges[1] = i + 1; edges[2] = i + 2;
				}
				else
				{
					edges[0] = 7 + (i - 1) * 2; edges[1] = i + 1; edges[2] = 8 + (i - 1) * 2;
				}
			}

			for (int k = 0; k < 3; ++k)
			{
				outEdges[k] = dtPolyEdge(face.navmesh, face.polyId, edges[(k + 1) % 3]);
			}
		}
	}
};

namespace iterations
{
	struct fromFaceToInnerEdges
	{
		fromFaceToInnerEdges(const dtPolyFace& fromFace)
			: _fromFace(fromFace)
		{
			_fromFaceEdge = queriers::faceEdge(fromFace);
			_nextEdge = _fromFaceEdge;
		}

		dtPolyEdge next()
		{
			if (_nextEdge.isValid())
			{
				_resultEdge = _nextEdge;
				_nextEdge = queriers::edgeNextLeftEdge(_nextEdge);
				if (_nextEdge == _fromFaceEdge)
				{
					_nextEdge = dtPolyEdge::INVALID;
				}
			}
			else 
			{
				_resultEdge = dtPolyEdge::INVALID;
			}

			return _resultEdge;
		}

		dtPolyFace _fromFace;
		dtPolyEdge _fromFaceEdge;
		dtPolyEdge _nextEdge;
		dtPolyFace _resultEdge;
	};

	struct fromFaceToVertices
	{
		fromFaceToVertices(const dtPolyFace& fromFace)
			: _fromFace(fromFace)
		{
			_fromFaceEdge = queriers::faceEdge(fromFace);
			_nextEdge = _fromFaceEdge;
		}

		dtPolyVertex next()
		{
			if (_nextEdge.isValid())
			{
				_resultVertex = queriers::edgeOriginVertex(_nextEdge);
				_nextEdge = queriers::edgeNextLeftEdge(_nextEdge);
				if (_nextEdge == _fromFaceEdge)
				{
					_nextEdge = dtPolyEdge::INVALID;
				}
			}
			else 
			{
				_resultVertex = dtPolyVertex::INVALID;
			}
			return _resultVertex;
		}

		int allVertices(dtPolyVertex *outVerts, int maxVertNum)
		{
			int vi = 0;
			do 
			{
				if (vi >= maxVertNum)
					break;

				auto v = next();
				if (!v.isValid()) 
					break;

				outVerts[vi] = v;
				++vi;
			} while (true);
			return vi;
		}

		dtPolyFace _fromFace;
		dtPolyEdge _fromFaceEdge;
		dtPolyEdge _nextEdge;
		dtPolyVertex _resultVertex;
	};

	struct fromFaceToNeighborFace
	{
		fromFaceToNeighborFace(const dtPolyFace& fromFace)
			: _fromFace(fromFace)
		{
			_fromFaceEdge = queriers::faceEdge(fromFace);
			_nextEdge = _fromFaceEdge;
		}

		dtPolyFace next()
		{
			if (_nextEdge.isValid()) 
			{
				do
				{
					_resultFace = queriers::edgeRightFace(_nextEdge);
					_nextEdge = queriers::edgeNextLeftEdge(_nextEdge);
					if (_nextEdge == _fromFaceEdge)
					{
						_nextEdge = dtPolyEdge::INVALID;
						break;
					}
					if (_resultFace.isValid())
					{
						break;
					}
				} while (true);
			}
			else
			{
				_resultFace = dtPolyFace::INVALID;
			}

			return _resultFace;
		}

		int allFaces(dtPolyFace* outFaces, int maxFaceNum)
		{
			int fi = 0;
			do
			{
				if (fi >= maxFaceNum)
					break;

				auto f = next();
				if (!f.isValid())
					break;
				
				outFaces[fi] = f;
				++fi;
			} while (true);
			return fi;
		}

		dtPolyFace _fromFace;
		dtPolyEdge _fromFaceEdge;
		dtPolyEdge _nextEdge;
		dtPolyFace _resultFace;
	};

	struct fromPolyToInternalFaces
	{
		fromPolyToInternalFaces(const dtNavMesh* nav, const dtPolyRef& ref)
			: _nav(nav), _ref(ref), _poly(0), _faceIdx(0), _resultFace()
		{
			if (_nav && _ref)
			{
				const dtMeshTile* tile = 0;
				const dtPoly* poly = 0;
				if (dtStatusSucceed(_nav->getTileAndPolyByRef(_ref, &tile, &poly)))
				{
					_poly = poly;
					_faceIdx = 0;
				}
			}
		}

		dtPolyFace next()
		{
			if (_poly && _faceIdx < _poly->vertCount)
			{
				_resultFace = dtPolyFace(_nav, _ref, _faceIdx);
				++_faceIdx;
			}
			else
			{
				_resultFace.reset();
			}
			return _resultFace;
		}

		const dtNavMesh*	_nav;
		dtPolyRef			_ref;
		const dtPoly*		_poly;	
		int					_faceIdx;
		dtPolyFace		_resultFace;
	};
}

namespace geom
{
	inline static bool projectPointOnEdge(const float* p, const dtPolyEdge& edge, float* proj)
	{
		dtPolyVertex v0, v1;
		if (queriers::edgeOriginAndDestinationVertex(edge, &v0, &v1))
		{
			float p0[3], p1[3];
			queriers::vertexPosition(v0, p0);
			queriers::vertexPosition(v1, p1);

			float p0p[3], n_p0p1[3];
			dtVsub(p0p, p, p0);
			dtVsub(n_p0p1, p1, p0);
			dtVnormalize(n_p0p1);
			float d = dtVdot(p0p, n_p0p1);
			dtVmad(proj, p0, n_p0p1, d);
			return true;
		}
		return false;
	}

	inline static float distanceSquaredPointToEdge(const float* p, const dtPolyEdge& edge)
	{
		dtPolyVertex v0, v1;
		if (queriers::edgeOriginAndDestinationVertex(edge, &v0, &v1))
		{
			float a[3], b[3];
			queriers::vertexPosition(v0, a);
			queriers::vertexPosition(v1, b);

			float ab[3], ap[3];
			dtVsub(ab, b, a);
			float abSqr = dtVlenSqr(ab);
			if (dtAbs(abSqr) < 0.01f)
			{
				return 0.0f;
			}
			else
			{
				dtVsub(ap, p, a);
				float d = dtVdot(ap, ab) / abSqr;
				if (d < 0.0f)
				{
					return dtVlenSqr(ap);
				}
				else if (d <= 1.0f)
				{
					auto paSqr = dtVlenSqr(ap);
					return paSqr - d * d * abSqr;
				}
				else
				{
					float bp[3];
					dtVsub(bp, p, a);
					return dtVlenSqr(bp);
				}
			}
		}

		return 0.0f;
	}

	inline static float distanceSquaredVertexToEdge(const dtPolyVertex& vertex, const dtPolyEdge& edge)
	{
		float p[3];
		if (queriers::vertexPosition(vertex, p))
		{
			return distanceSquaredPointToEdge(p, edge);
		}
		return 0.0f;
	}

	inline static bool closestPointToEdge(const float* p, const dtPolyEdge& edge, float* closest)
	{
		dtPolyVertex v0, v1;
		if (queriers::edgeOriginAndDestinationVertex(edge, &v0, &v1))
		{
			float a[3], b[3];
			queriers::vertexPosition(v0, a);
			queriers::vertexPosition(v1, b);

			float ab[3], ap[3];
			dtVsub(ab, b, a);
			dtVsub(ap, p, a);
			float d = dtVdot(ab, ap);
			float abSqr = dtVlenSqr(ab);
			if (dtAbs(abSqr) < 0.01f)
			{
				dtVcopy(closest, a);
				return true;
			}
			else
			{
				auto t = d / abSqr;
				if (t < 0.0f)
				{
					dtVcopy(closest, a);
				}
				else if (t > 1.0f)
				{
					dtVcopy(closest, b);
				}
				else
				{
					dtVmad(closest, a, ab, t);
				}
				return true;
			}
			return false;
		}
		return false;
	}
}

namespace astar
{
	// 检查通过throughFace从fromEdge到toEdge，半径为radius的单位是否能够通过
	bool isWalkableByRadius(float radius, const dtPolyEdge& fromEdge, const dtPolyFace& throughFace, const dtPolyEdge& toEdge);
}

namespace debug
{
}
#endif // DETOURNAVMESHQUERY_NONPOINT_H