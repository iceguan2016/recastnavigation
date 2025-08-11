#include "DetourNavMeshQuery_Nonpoint.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"

#include <list>
#include <unordered_set>

static const float H_SCALE = 0.999f; // Search heuristic scale.

dtPolyPrimitive dtPolyPrimitive::INVALID(nullptr, 0, -1);

dtStatus dtNavMeshQuery::findNearestFace(const float* center, const float* halfExtents,
	const dtQueryFilter* filter,
	dtPolyFace* nearestFace, float* nearestPt) const
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

			dtPolyVertex verts[3];
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

dtStatus dtNavMeshQuery::findPathByRadius(const dtPolyFace& startRef, const dtPolyFace& endRef,
	const float* startPos, const float* endPos,
	const dtQueryFilter* filter,
	dtPolyFace* path, int* pathCount, const int maxPath,
	const float radius) const
{
	if (dtAbs(radius) < 0.01f)
	{
		return DT_FAILURE;
	}
	
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	if (!pathCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*pathCount = 0;

	// Validate input
	if (!startRef.isValid() || !endRef.isValid() ||
		startRef.navmesh != m_nav || endRef.navmesh != m_nav ||
		!startPos || !dtVisfinite(startPos) ||
		!endPos || !dtVisfinite(endPos) ||
		!filter || !path || maxPath <= 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	if (startRef == endRef)
	{
		path[0] = startRef;
		*pathCount = 1;
		return DT_SUCCESS;
	}

	m_nodePool->clear();
	m_openList->clear();

	dtNode* startNode = m_nodePool->getNode(startRef.polyId, 0, startRef.innerIdx);
	dtVcopy(startNode->pos, startPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef.polyId;
	startNode->primIdx = startRef.innerIdx;
	startNode->entryEdge.reset();
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);

	dtNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;

	bool outOfNodes = false;

	while (!m_openList->empty())
	{
		// Remove node from open list and put it in closed list.
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		dtPolyFace bestFace(m_nav, bestNode->id, bestNode->primIdx);
		auto entryEdge = bestNode->entryEdge;

		// Reached the goal, stop searching.
		if (bestFace == endRef)
		{
			lastBestNode = bestNode;
			break;
		}

		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		bestFace.getTileAndPoly(&bestTile, &bestPoly);

		// Get parent poly and tile.
		dtPolyFace parentFace;
		if (bestNode->pidx)
		{
			auto parentNode = m_nodePool->getNodeAtIdx(bestNode->pidx);
			parentFace = dtPolyFace(m_nav, parentNode->id, parentNode->primIdx);
		}

		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		parentFace.getTileAndPoly(&parentTile, &parentPoly);

		iterations::fromFaceToInnerEdges iterInnerEdges(bestFace);

		do 
		{
			auto innerEdge = iterInnerEdges.next();
			if (!innerEdge.isValid())
				break;

			auto neighbourFace = queriers::edgeRightFace(innerEdge);

			// Skip invalid ids and do not expand back to where we came from.
			if (!neighbourFace.isValid()
				|| neighbourFace == bestFace
				|| neighbourFace == parentFace)
			{
				continue;
			}

			// Get neighbour poly and tile.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			neighbourFace.getTileAndPoly(&neighbourTile, &neighbourPoly);

			if (!filter->passFilter(neighbourFace.polyId, neighbourTile, neighbourPoly))
				continue;

			// check radius
			if (bestFace != startRef
				&& radius > 0.0f
				&& !astar::isWalkableByRadius(radius, entryEdge, bestFace, innerEdge))
			{
				continue;
			}

			// Get neighbor node
			dtNode* neighbourNode = m_nodePool->getNode(neighbourFace.polyId, 0, neighbourFace.innerIdx);
			if (!neighbourNode)
			{
				outOfNodes = true;
				continue;
			}

			// If the node is visited the first time, calculate node position.
			if (neighbourNode->flags == 0)
			{
				if (!geom::closestPointToEdge(bestNode->pos, innerEdge, neighbourNode->pos))
				{
					break;
				}
			}

			// Calculate cost and heuristic.
			float cost = 0;
			float heuristic = 0;

			// Special case for last node.
			if (neighbourFace == endRef)
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
					parentFace.polyId, parentTile, parentPoly,
					bestFace.polyId, bestTile, bestPoly,
					neighbourFace.polyId, neighbourTile, neighbourPoly);
				const float endCost = filter->getCost(neighbourNode->pos, endPos,
					bestFace.polyId, bestTile, bestPoly,
					neighbourFace.polyId, neighbourTile, neighbourPoly,
					0, 0, 0);

				cost = bestNode->cost + curCost + endCost;
				heuristic = 0;
			}
			else
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
					parentFace.polyId, parentTile, parentPoly,
					bestFace.polyId, bestTile, bestPoly,
					neighbourFace.polyId, neighbourTile, neighbourPoly);
				cost = bestNode->cost + curCost;
				heuristic = dtVdist(neighbourNode->pos, endPos) * H_SCALE;
			}

			const float total = cost + heuristic;

			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			// The node is already visited and process, and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_CLOSED) && total >= neighbourNode->total)
				continue;

			// Add or update the node.
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->id = neighbourFace.polyId;
			neighbourNode->primIdx = neighbourFace.innerIdx;
			neighbourNode->entryEdge = innerEdge;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->cost = cost;
			neighbourNode->total = total;

			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				// Already in open, update node location.
				m_openList->modify(neighbourNode);
			}
			else
			{
				// Put the node in open list.
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}

			// Update nearest node to target so far.
			if (heuristic < lastBestNodeCost)
			{
				lastBestNodeCost = heuristic;
				lastBestNode = neighbourNode;
			}
		} while(true);
	}

	dtStatus status = getPathToNode(lastBestNode, path, pathCount, maxPath);

	if (lastBestNode->id != endRef.polyId &&
		lastBestNode->primIdx != endRef.innerIdx)
		status |= DT_PARTIAL_RESULT;

	if (outOfNodes)
		status |= DT_OUT_OF_NODES;

	return status;
}

dtStatus dtNavMeshQuery::getPathToNode(dtNode* endNode, dtPolyFace* path, int* pathCount, int maxPath) const
{
	// Find the length of the entire path.
	dtNode* curNode = endNode;
	int length = 0;
	do
	{
		length++;
		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	} while (curNode);

	// If the path cannot be fully stored then advance to the last node we will be able to store.
	curNode = endNode;
	int writeCount;
	for (writeCount = length; writeCount > maxPath; writeCount--)
	{
		dtAssert(curNode);

		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	}

	// Write path
	for (int i = writeCount - 1; i >= 0; i--)
	{
		dtAssert(curNode);

		path[i] = dtPolyFace(m_nav, curNode->id, curNode->primIdx);
		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	}

	dtAssert(!curNode);

	*pathCount = dtMin(length, maxPath);

	if (length > maxPath)
		return DT_SUCCESS | DT_BUFFER_TOO_SMALL;

	return DT_SUCCESS;
}

namespace astar
{
	struct dtInternalPrimitiveHash
	{
		std::size_t operator()(const dtPolyPrimitive& p) const {
			return std::hash<dtPolyRef>()(p.polyId) ^ (std::hash<int>()(p.innerIdx) << 1);
		}
	};

	bool isWalkableByRadius(float radius, const dtPolyEdge& fromEdge, const dtPolyFace& throughFace, const dtPolyEdge& toEdge)
	{
		// we identify the points
		dtPolyVertex fromEdgeOrigin, fromEdgeDestination;
		if (!queriers::edgeOriginAndDestinationVertex(fromEdge, &fromEdgeOrigin, &fromEdgeDestination))
		{
			return false;
		}

		dtPolyVertex toEdgeOrigin, toEdgeDestination;
		if (!queriers::edgeOriginAndDestinationVertex(toEdge, &toEdgeOrigin, &toEdgeDestination))
		{
			return false;
		}

		dtPolyVertex vA;  // the vertex on fromEdge not on toEdge  
		dtPolyVertex vB;  // the vertex on toEdge not on fromEdge  
		dtPolyVertex vC;  // the common vertex of the 2 edges (pivot) 

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
		dtPolyEdge faceEdge = queriers::faceEdge(throughFace);
		if (!faceEdge) return false;
		dtPolyEdge oppositeEdge = queriers::edgeOppositeEdge(faceEdge);
		if (!oppositeEdge) return false;
		dtPolyEdge nextLeftEdge = queriers::edgeNextLeftEdge(faceEdge);
		if (!nextLeftEdge) return false;
		dtPolyEdge nextLeftEdge_Opp = queriers::edgeOppositeEdge(nextLeftEdge);

		dtPolyEdge adjEdge;
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
				std::list<dtPolyFace> faceToCheck;
				std::list<dtPolyEdge> faceFromEdge;

				std::unordered_set<dtPolyFace, dtInternalPrimitiveHash> faceDone;

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

					dtPolyEdge currEdgeA, currEdgeB;
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

					dtPolyFace nextFaceA;
					auto currEdgeA_LeftFace = queriers::edgeLeftFace(currEdgeA);
					if (currEdgeA_LeftFace == currFace)
					{
						nextFaceA = queriers::edgeRightFace(currEdgeA);
					}
					else
					{
						nextFaceA = currEdgeA_LeftFace;
					}

					dtPolyFace nextFaceB;
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