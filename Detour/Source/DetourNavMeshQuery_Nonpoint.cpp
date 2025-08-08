#include "DetourNavMeshQuery_Nonpoint.h"
#include "DetourNavMeshQuery.h"

#include <list>
#include <unordered_set>

dtInternalPrimitive dtInternalPrimitive::INVALID(nullptr, 0, -1);

dtStatus dtNavMeshQuery::findNearestFace(const float* center, const float* halfExtents,
	const dtQueryFilter* filter,
	dtInternalFace* nearestFace, float* nearestPt) const
{
	dtPolyRef nearestPoly = 0;
	float nearestPos[3];
	if (dtStatusSucceed(findNearestPoly(center, halfExtents, filter, &nearestPoly, nearestPos)))
	{
		// 遍历polygon内部face找到包含nearestPt点的face
		iterations::fromPolyToInternalFaces iterFaces(m_nav, nearestPoly);
		do
		{
			auto face = iterFaces.next();
			if (!face.isValid())
				break;

			dtInternalVertex verts[3];
			iterations::fromFaceToVertices iterFaceVerts(face);
			auto num_verts = iterFaceVerts.allVertices(verts, 3);
			dtAssert(num_verts == 3);
			float v0[3], v1[3], v2[3];
			queriers::vertexPosition(verts[0], v0);
			queriers::vertexPosition(verts[1], v1);
			queriers::vertexPosition(verts[2], v2);

			float s0 = dtTriArea2D(nearestPos, v0, v1);
			float s1 = dtTriArea2D(nearestPos, v1, v2);
			float s2 = dtTriArea2D(nearestPos, v2, v0);
			if ((s0 * s1) >= 0 && (s0 * s2) >= 0)
			{
				*nearestFace = face;
				if (nearestPt) dtVcopy(nearestPt, nearestPos);
				return DT_SUCCESS;
			}
		} while (true);
	}
	return DT_FAILURE;
}

namespace astar
{
	struct dtInternalPrimitiveHash
	{
		std::size_t operator()(const dtInternalPrimitive& p) const {
			return std::hash<dtPolyRef>()(p.polyId) ^ (std::hash<int>()(p.innerIdx) << 1);
		}
	};

	bool isWalkableByRedius(float radius, const dtInternalEdge& fromEdge, const dtInternalFace& throughFace, const dtInternalEdge& toEdge)
	{
		// we identify the points
		dtInternalVertex fromEdgeOrigin, fromEdgeDestination;
		if (!queriers::edgeOriginAndDestinationVertex(fromEdge, &fromEdgeOrigin, &fromEdgeDestination))
		{
			return false;
		}

		dtInternalVertex toEdgeOrigin, toEdgeDestination;
		if (!queriers::edgeOriginAndDestinationVertex(toEdge, &toEdgeOrigin, &toEdgeDestination))
		{
			return false;
		}

		dtInternalVertex vA;  // the vertex on fromEdge not on toEdge  
		dtInternalVertex vB;  // the vertex on toEdge not on fromEdge  
		dtInternalVertex vC;  // the common vertex of the 2 edges (pivot) 

		if (fromEdgeOrigin == toEdgeOrigin)
		{
			vA = fromEdgeDestination;
			vB = toEdgeDestination;
			vC = fromEdgeOrigin;
		}
		else if (fromEdgeDestination == toEdgeDestination)
		{
			vA = fromEdgeOrigin;
			vB = toEdgeOrigin;
			vC = fromEdgeDestination;
		}
		else if (fromEdgeOrigin == toEdgeDestination)
		{
			vA = fromEdgeDestination;
			vB = toEdgeOrigin;
			vC = fromEdgeOrigin;
		}
		else if (fromEdgeDestination == toEdgeOrigin)
		{
			vA = fromEdgeOrigin;
			vB = toEdgeDestination;
			vC = fromEdgeDestination;
		}

		float diameterSquared = radius * radius;

		float dot;
		//bool result;
		float distSquared;

		float a[3], b[3], c[3];
		queriers::vertexPosition(vA, a);
		queriers::vertexPosition(vB, b);
		queriers::vertexPosition(vC, c);

		// if we have a right or obtuse angle on CAB
		float ab[3], ac[3];
		dtVsub(ab, b, a);
		dtVsub(ab, c, a);
		dot = dtVdot(ab, ac);
		if (dot <= 0)
		{
			// we compare length of AC with radius
			distSquared = dtVlenSqr(ac);
			return distSquared >= diameterSquared;
		}

		// if we have a right or obtuse angle on CBA  
		float ba[3], bc[3];
		dtVsub(ba, a, b);
		dtVsub(bc, c, b);
		dot = dtVdot(ba, bc);
		if (dot <= 0)
		{
			// we compare length of BC with radius
			distSquared = dtVlenSqr(bc);
			return distSquared >= diameterSquared;
		}

		// we identify the adjacent edge (facing pivot vertex) 
		dtInternalEdge faceEdge = queriers::faceEdge(throughFace);
		if (!faceEdge) return false;
		dtInternalEdge oppositeEdge = queriers::edgeOppositeEdge(faceEdge);
		if (!oppositeEdge) return false;
		dtInternalEdge nextLeftEdge = queriers::edgeNextLeftEdge(faceEdge);
		if (!nextLeftEdge) return false;
		dtInternalEdge nextLeftEdge_Opp = queriers::edgeOppositeEdge(nextLeftEdge);

		dtInternalEdge adjEdge;
		if (faceEdge != fromEdge && oppositeEdge != fromEdge && faceEdge != toEdge && oppositeEdge != toEdge)
		{
			adjEdge = faceEdge;
		}
		else if (nextLeftEdge != fromEdge && nextLeftEdge_Opp != fromEdge && nextLeftEdge != toEdge && nextLeftEdge_Opp != toEdge)
		{
			adjEdge = nextLeftEdge;
		}
		else
		{
			adjEdge = queriers::edgePrevLeftEdge(faceEdge);
		}

		// if the adjacent edge is constrained, we check the distance of orthognaly projected
		if (queriers::edgeIsBoundary(adjEdge))
		{
			float proj[3];
			if (geom::projectPointOnEdge(c, adjEdge, proj))
			{
				float v[3];
				dtVsub(v, proj, c);
				distSquared = dtVlenSqr(v);
				return distSquared >= diameterSquared;
			}
			else
			{
				return false;
			}
		}
		else
		{
			// if the adjacent is not constrained
			float distSquaredA = dtVlenSqr(ac);
			float distSquaredB = dtVlenSqr(bc);

			if (distSquaredA < diameterSquared || distSquaredB < diameterSquared)
			{
				return false;
			}
			else
			{
				std::list<dtInternalFace> faceToCheck;
				std::list<dtInternalEdge> faceFromEdge;

				std::unordered_set<dtInternalFace, dtInternalPrimitiveHash> faceDone;

				faceFromEdge.push_back(adjEdge);
				auto leftFace = queriers::edgeLeftFace(adjEdge);
				if (leftFace == throughFace)
				{
					auto rightFace = queriers::edgeRightFace(adjEdge);
					faceToCheck.push_back(rightFace);
					faceDone.insert(rightFace);
				}
				else
				{
					faceToCheck.push_back(leftFace);
					faceDone.insert(leftFace);
				}

				while (faceToCheck.size() > 0)
				{
					auto currFace = faceToCheck.front();
					faceToCheck.pop_front();

					auto fromEdge = faceFromEdge.front();
					faceFromEdge.pop_front();

					auto faceEdge = queriers::faceEdge(currFace);
					auto faceEdge_Next = queriers::edgeNextLeftEdge(faceEdge);
					auto fromEdge_Opp = queriers::edgeOppositeEdge(fromEdge);

					dtInternalEdge currEdgeA, currEdgeB;
					if (faceEdge == fromEdge || faceEdge == fromEdge_Opp)
					{
						// we identify the 2 edges to evaluate
						currEdgeA = queriers::edgeNextLeftEdge(faceEdge);
						currEdgeB = queriers::edgeNextLeftEdge(faceEdge_Next);
					}
					else if (faceEdge_Next == fromEdge || faceEdge_Next == fromEdge_Opp)
					{
						// we identify the faces related to the 2 edges
						currEdgeA = faceEdge;
						currEdgeB = queriers::edgeNextLeftEdge(faceEdge_Next);
					}
					else
					{
						currEdgeA = faceEdge;
						currEdgeB = faceEdge_Next;
					}

					dtInternalFace nextFaceA;
					auto currEdgeA_LeftFace = queriers::edgeLeftFace(currEdgeA);
					if (currEdgeA_LeftFace == currFace)
					{
						nextFaceA = queriers::edgeRightFace(currEdgeA);
					}
					else
					{
						nextFaceA = currEdgeA_LeftFace;
					}

					dtInternalFace nextFaceB;
					auto currEdgeB_LeftFace = queriers::edgeLeftFace(currEdgeB);
					if (currEdgeB_LeftFace == currFace)
					{
						nextFaceB = queriers::edgeRightFace(currEdgeB);
					}
					else
					{
						nextFaceB = currEdgeB_LeftFace;
					}
					// we check if the next face is not already in pipe
					// and if the edge A is close to pivot vertex
					bool nextFaceA_not_done = faceDone.count(nextFaceA) == 0;
					if (nextFaceA_not_done && geom::distanceSquaredVertexToEdge(vC, currEdgeA) < diameterSquared)
					{
						// if the edge is constrained
						if (queriers::edgeIsBoundary(currEdgeA))
						{
							// so it is not walkable
							return false;
						}
						else
						{
							// if the edge is not constrained, we continue the search
							faceToCheck.push_back(nextFaceA);
							faceFromEdge.push_back(currEdgeA);
							faceDone.insert(nextFaceA);
						}
					} 

					// and if the edge B is close to pivot vertex    // we check if the next face is not already in pipe  
					bool nextFaceB_not_done = faceDone.count(nextFaceB) == 0;
					if (nextFaceB_not_done && geom::distanceSquaredVertexToEdge(vC, currEdgeB) < diameterSquared)
					{
						// if the edge is constrained
						if (queriers::edgeIsBoundary(currEdgeB))
						{
							// so it is not walkable
							return false;
						}
						else
						{
							// if the edge is not constrained, we continue the search
							faceToCheck.push_back(nextFaceB);
							faceFromEdge.push_back(currEdgeB);
							faceDone.insert(nextFaceB);
						}
					}
				}  
				
				// if we didn't previously meet a constrained edge  
				return true;
			}
		}

		return true;
	}
}