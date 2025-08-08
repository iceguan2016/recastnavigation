
#ifndef DETOURNAVMESHQUERY_NONPOINT_H
#define DETOURNAVMESHQUERY_NONPOINT_H

#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"

// 处理带有半径的单位寻路

struct dtInternalPrimitive
{
	static dtInternalPrimitive INVALID;

	dtInternalPrimitive()
		: navmesh(nullptr), polyId(0), innerIdx(-1)
	{
	}

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

	explicit operator bool() const {
		return isValid();
	}

	bool isValid() const
	{
		return navmesh != nullptr && polyId != 0 && innerIdx != -1;
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

	bool operator==(const dtInternalPrimitive& inOther) const
	{
		return navmesh == inOther.navmesh &&
			polyId == inOther.polyId &&
			innerIdx == inOther.innerIdx;
	}

	bool operator!=(const dtInternalPrimitive& inOther) const
	{
		return navmesh != inOther.navmesh ||
			polyId != inOther.polyId ||
			innerIdx != inOther.innerIdx;
	}

	const dtNavMesh*	navmesh;
	dtPolyRef			polyId;
	int					innerIdx;
};

typedef dtInternalPrimitive dtInternalVertex;
typedef dtInternalPrimitive dtInternalEdge;
typedef dtInternalPrimitive dtInternalFace;


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
	inline static bool vertexPosition(const dtInternalVertex& vertex, float* pos) 
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
				return (idx & 0x1) == 1 ? dtInternalVertex(edge.navmesh, edge.polyId, 0) : dtInternalVertex(edge.navmesh, edge.polyId, idx/2 + 2);
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
				return (idx & 0x1) == 0 ? dtInternalVertex(edge.navmesh, edge.polyId, 0) : dtInternalVertex(edge.navmesh, edge.polyId, idx/2 + 2);
			}
		}
		return dtInternalVertex::INVALID;
	}

	inline static bool edgeOriginAndDestinationVertex(const dtInternalEdge& edge, dtInternalVertex* origin, dtInternalVertex* destination)
	{
		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		if (edge.getTileAndPoly(&tile, &poly))
		{
			if (edge.innerIdx < DT_VERTS_PER_POLYGON)
			{
				// 原始边
				if (origin)
					*origin = dtInternalVertex(edge.navmesh, edge.polyId, edge.innerIdx);
				if (destination)
					*destination = dtInternalVertex(edge.navmesh, edge.polyId, (edge.innerIdx + 1) % poly->vertCount);
				return true;
			}
			else
			{
				// 新增内部边(起点)
				auto idx = edge.innerIdx - DT_VERTS_PER_POLYGON;
				if ((idx & 0x1) == 1)
				{
					if (origin)
						*origin = dtInternalVertex(edge.navmesh, edge.polyId, 0);
					if (destination)
						*destination = dtInternalVertex(edge.navmesh, edge.polyId, idx / 2 + 2);
				} 
				else
				{
					if (origin)
						*origin = dtInternalVertex(edge.navmesh, edge.polyId, idx / 2 + 2);
					if (destination)
						*destination = dtInternalVertex(edge.navmesh, edge.polyId, 0);
				}
				return true;
			}
		}
		return false;
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
		return dtInternalFace::INVALID;
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
								return dtInternalEdge(edge.navmesh, link->ref, neiEdgeIdx);
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

	inline static bool edgeIsBoundary(const dtInternalEdge& edge)
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

	inline static void faceAllEdges(const dtInternalFace& face, dtInternalEdge outEdges[3])
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
				outEdges[k] = dtInternalEdge(face.navmesh, face.polyId, edges[(k + 1) % 3]);
			}
		}
	}
};

namespace iterations
{
	struct fromFaceToInnerEdges
	{
		fromFaceToInnerEdges(const dtInternalFace& fromFace)
			: _fromFace(fromFace)
		{
			_fromFaceEdge = queriers::faceEdge(fromFace);
			_nextEdge = _fromFaceEdge;
		}

		dtInternalEdge next()
		{
			if (_nextEdge.isValid())
			{
				_resultEdge = _nextEdge;
				_nextEdge = queriers::edgeNextLeftEdge(_nextEdge);
				if (_nextEdge == _fromFaceEdge)
				{
					_nextEdge = dtInternalEdge::INVALID;
				}
			}
			else 
			{
				_resultEdge = dtInternalEdge::INVALID;
			}

			return _resultEdge;
		}

		dtInternalFace _fromFace;
		dtInternalEdge _fromFaceEdge;
		dtInternalEdge _nextEdge;
		dtInternalFace _resultEdge;
	};

	struct fromFaceToVertices
	{
		fromFaceToVertices(const dtInternalFace& fromFace)
			: _fromFace(fromFace)
		{
			_fromFaceEdge = queriers::faceEdge(fromFace);
			_nextEdge = _fromFaceEdge;
		}

		dtInternalVertex next()
		{
			if (_nextEdge.isValid())
			{
				_resultVertex = queriers::edgeOriginVertex(_nextEdge);
				_nextEdge = queriers::edgeNextLeftEdge(_nextEdge);
				if (_nextEdge == _fromFaceEdge)
				{
					_nextEdge = dtInternalEdge::INVALID;
				}
			}
			else 
			{
				_resultVertex = dtInternalVertex::INVALID;
			}
			return _resultVertex;
		}

		int allVertices(dtInternalVertex *outVerts, int maxVertNum)
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

		dtInternalFace _fromFace;
		dtInternalEdge _fromFaceEdge;
		dtInternalEdge _nextEdge;
		dtInternalVertex _resultVertex;
	};

	struct fromFaceToNeighborFace
	{
		fromFaceToNeighborFace(const dtInternalFace& fromFace)
			: _fromFace(fromFace)
		{
			_fromFaceEdge = queriers::faceEdge(fromFace);
			_nextEdge = _fromFaceEdge;
		}

		dtInternalFace next()
		{
			if (_nextEdge.isValid()) 
			{
				do
				{
					_resultFace = queriers::edgeRightFace(_nextEdge);
					_nextEdge = queriers::edgeNextLeftEdge(_nextEdge);
					if (_nextEdge == _fromFaceEdge)
					{
						_nextEdge = dtInternalEdge::INVALID;
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
				_resultFace = dtInternalFace::INVALID;
			}

			return _resultFace;
		}

		int allFaces(dtInternalFace* outFaces, int maxFaceNum)
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

		dtInternalFace _fromFace;
		dtInternalEdge _fromFaceEdge;
		dtInternalEdge _nextEdge;
		dtInternalFace _resultFace;
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

		dtInternalFace next()
		{
			if (_poly && _faceIdx < _poly->vertCount)
			{
				_resultFace = dtInternalFace(_nav, _ref, _faceIdx);
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
		dtInternalFace		_resultFace;
	};
}

namespace geom
{
	inline static bool projectPointOnEdge(const float* p, const dtInternalEdge& edge, float* proj)
	{
		dtInternalVertex v0, v1;
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

	inline static float distanceSquaredPointToEdge(const float* p, const dtInternalEdge& edge)
	{
		dtInternalVertex v0, v1;
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

	inline static float distanceSquaredVertexToEdge(const dtInternalVertex& vertex, const dtInternalEdge& edge)
	{
		float p[3];
		if (queriers::vertexPosition(vertex, p))
		{
			return distanceSquaredPointToEdge(p, edge);
		}
		return 0.0f;
	}
}

namespace astar
{
	// 检查通过throughFace从fromEdge到toEdge，半径为radius的单位是否能够通过
	bool isWalkableByRedius(float radius, const dtInternalEdge& fromEdge, const dtInternalFace& throughFace, const dtInternalEdge& toEdge);
}

namespace debug
{
}
#endif // DETOURNAVMESHQUERY_NONPOINT_H