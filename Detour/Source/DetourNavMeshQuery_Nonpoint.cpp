#include "DetourNavMeshQuery_Nonpoint.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"

#include <list>
#include <unordered_set>
#include <unordered_map>

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
	dtPolyEdge* portalEdges, int* portalEdgeCount, const int maxPortalEdge,
	const float radius
#if DT_DEBUG_ASTAR
	,
	int maxIters,
	astar::dtAstarNodeDebug* visitNodes, int& nVisitNodes, const int maxVisitNode
#endif
	) const
{
	if (dtAbs(radius) < 0.01f)
	{
		return DT_FAILURE;
	}
	
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	if (!pathCount || !portalEdgeCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*pathCount = 0;

#if DT_DEBUG_ASTAR
	int iterTimes = 0;
	nVisitNodes = 0;
#endif

	// Validate input
	if (!startRef.isValid() || !endRef.isValid() ||
		startRef.navmesh != m_nav || endRef.navmesh != m_nav ||
		!startPos || !dtVisfinite(startPos) ||
		!endPos || !dtVisfinite(endPos) ||
		!filter || !path || maxPath <= 0 || !portalEdges || maxPortalEdge <= 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	if (startRef == endRef)
	{
		path[0] = startRef;
		*pathCount = 1;
		return DT_SUCCESS;
	}

#if DT_DEBUG_ASTAR
	LOG_INFO("### findPathByRadius, startFace:%s", startRef.toString().c_str());
#endif

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

#if DT_DEBUG_ASTAR
		++iterTimes;
		if (iterTimes > maxIters)
		{
			break;
		}

		if (visitNodes && nVisitNodes < maxVisitNode)
		{
			auto& visit = visitNodes[nVisitNodes];
			visit.face = bestFace;
			dtVcopy(visit.entry_pos, bestNode->pos);

			++nVisitNodes;
		}
#endif

		// Reached the goal, stop searching.
		if (bestFace == endRef)
		{
			lastBestNode = bestNode;
			break;
		}

#if DT_DEBUG_ASTAR
		LOG_INFO("findPathByRadius, bestFace:%ld[%d]", bestFace.polyId, bestFace.innerIdx);
#endif

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
#if DT_DEBUG_ASTAR
				LOG_INFO("\t not walkable");
#endif
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

	dtStatus status = getPathToNode(lastBestNode, path, pathCount, maxPath, portalEdges, portalEdgeCount, maxPortalEdge);

	if (lastBestNode->id != endRef.polyId &&
		lastBestNode->primIdx != endRef.innerIdx)
		status |= DT_PARTIAL_RESULT;

	if (outOfNodes)
		status |= DT_OUT_OF_NODES;

	return status;
}

dtStatus dtNavMeshQuery::getPathToNode(
	dtNode* endNode, 
	dtPolyFace* path, int* pathCount, int maxPath,
	dtPolyEdge* portalEdges, int* portalEdgeCount, const int maxPortalEdge) const
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
		if (i != 0)
			portalEdges[i-1] = curNode->entryEdge;
		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	}

	dtAssert(!curNode);

	*pathCount = dtMin(length, maxPath);
	*portalEdgeCount = dtMin(length-1, maxPath-1);

	if (length > maxPath)
		return DT_SUCCESS | DT_BUFFER_TOO_SMALL;

	return DT_SUCCESS;
}

struct dtPolyPrimitiveHash
{
	std::size_t operator()(const dtPolyPrimitive& p) const {
		return std::hash<dtPolyRef>()(p.polyId) ^ (std::hash<int>()(p.innerIdx) << 1);
	}
};

namespace astar
{
	bool isWalkableByRadius(float radius, const dtPolyEdge& fromEdge, const dtPolyFace& throughFace, const dtPolyEdge& toEdge)
	{
#if DT_DEBUG_ASTAR
		LOG_INFO("\t isWalkableByRadius, fromEdge:%s throughFace:%s, toEdge:%s", 
			fromEdge.toString().c_str(), 
			throughFace.toString().c_str(),
			toEdge.toString().c_str());
#endif
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

#if DT_DEBUG_ASTAR
		LOG_INFO("\t isWalkableByRadius, vA:%s vB:%s, vC:%s",
			vA.toString().c_str(),
			vB.toString().c_str(),
			vC.toString().c_str());
#endif
		// Note: Diameter-based judgment is applied here.
		float diameter = radius * 2;
		float diameterSquared = diameter * diameter;

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
#if DT_DEBUG_ASTAR
			LOG_INFO("\t isWalkableByRadius, CAB is obutse, distSquared:%.2f, diameterSquared:%.2f", distSquared, diameterSquared);
#endif
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
#if DT_DEBUG_ASTAR
			LOG_INFO("\t isWalkableByRadius, CBA is obutse, distSquared:%.2f, diameterSquared:%.2f", distSquared, diameterSquared);
#endif
			return distSquared >= diameterSquared;
		}

		// we identify the adjacent edge (facing pivot vertex) 
		dtPolyEdge faceEdge = queriers::faceEdge(throughFace);
		if (!faceEdge) return false;
		dtPolyEdge oppositeEdge = queriers::edgeOppositeEdge(faceEdge);
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

#if DT_DEBUG_ASTAR
		LOG_INFO("\t isWalkableByRadius, adjEdge:%s, isBoundary:%d", adjEdge.toString().c_str(), queriers::edgeIsBoundary(adjEdge)); 
#endif

		// if the adjacent edge is constrained, we check the distance of orthognaly projected
		if (queriers::edgeIsBoundary(adjEdge))
		{
			float proj[3];
			if (geom::projectPointOnEdge(c, adjEdge, proj))
			{
				float v[3];
				dtVsub(v, proj, c);
				distSquared = dtVlenSqr(v);
#if DT_DEBUG_ASTAR
				LOG_INFO("\t isWalkableByRadius, isBoundary:1, distSquared:%.2f diameterSquared:%.2f", distSquared, diameterSquared);
#endif
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

				std::unordered_set<dtPolyFace, dtPolyPrimitiveHash> faceDone;

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

namespace funnel
{
	// Funnel Algorithm: https://blog.csdn.net/fengkeyleaf/article/details/118832924?utm_source=app&app_version=5.3.1
	// 算法步骤：
	//		1.每次处理一条导航线，直到处理完所有的导航线:
	//		2.如果新的顶点与apex形成的漏斗比原来的小，那么我们将对应左顶点或右顶点移动到该位置处; 反之，则不做更新，因为新漏斗变大了 (注意：变大了不更新)
	//		3.如果新的一侧顶点与apex形成的边界，越过了另一侧原顶点与apex形成的边界，我们将apex移动到该位置处，并将apex加入到最短路径中 (注意：最后将起点和终点分别添加到最短路径中，形成的路径点连线即为最短路径）
	dtStatus appendVertex(const float* pos, const unsigned char flags, const dtPolyFace ref,
		float* straightPath, unsigned char* straightPathFlags, dtPolyFace* straightPathRefs,
		int* straightPathCount, const int maxStraightPath)
	{
		if ((*straightPathCount) > 0 && dtVequal(&straightPath[((*straightPathCount) - 1) * 3], pos))
		{
			// The vertices are equal, update flags and poly.
			if (straightPathFlags)
				straightPathFlags[(*straightPathCount) - 1] = flags;
			if (straightPathRefs)
				straightPathRefs[(*straightPathCount) - 1] = ref;
		}
		else
		{
			// Append new vertex.
			dtVcopy(&straightPath[(*straightPathCount) * 3], pos);
			if (straightPathFlags)
				straightPathFlags[(*straightPathCount)] = flags;
			if (straightPathRefs)
				straightPathRefs[(*straightPathCount)] = ref;
			(*straightPathCount)++;

			// If there is no space to append more vertices, return.
			if ((*straightPathCount) >= maxStraightPath)
			{
				return DT_SUCCESS | DT_BUFFER_TOO_SMALL;
			}

			// If reached end of path, return.
			if (flags == DT_STRAIGHTPATH_END)
			{
				return DT_SUCCESS;
			}
		}
		return DT_IN_PROGRESS;
	}

	dtStatus straightPathByRadius(const float* startPos, const float* endPos,
		const dtPolyFace* path, const int pathSize, 
		const dtPolyEdge* portalEdges, const int portalEdgeCount,
		float* straightPath, unsigned char* straightPathFlags, dtPolyFace* straightPathRefs, 
		int* straightPathCount, const int maxStraightPath, 
		const float radius
#if DT_DEBUG_ASTAR
		,
		dtFunnelDebug* portalDebugs, int* portalDebugCount, const int maxPortalDebug
#endif
		)
	{
		if (!straightPathCount)
			return DT_FAILURE | DT_INVALID_PARAM;

		*straightPathCount = 0;

#if DT_DEBUG_ASTAR
		*portalDebugCount = 0;
#endif

		if (!startPos || !dtVisfinite(startPos) ||
			!endPos || !dtVisfinite(endPos) ||
			!path || pathSize <= 0 || !path[0] ||
			!portalEdges || portalEdgeCount <= 0 ||
			maxStraightPath <= 0)
		{
			return DT_FAILURE | DT_INVALID_PARAM;
		}

		dtStatus stat = 0;

		// Add start point.
		stat = appendVertex(startPos, DT_STRAIGHTPATH_START, path[0],
			straightPath, straightPathFlags, straightPathRefs,
			straightPathCount, maxStraightPath);
		if (stat != DT_IN_PROGRESS)
			return stat;

		if (portalEdgeCount > 1)
		{
			float portalApex[3], portalLeft[3], portalRight[3];
			dtVcopy(portalApex, startPos);
			dtVcopy(portalLeft, portalApex);
			dtVcopy(portalRight, portalApex);
			int apexIndex = 0;
			int leftIndex = 0;
			int rightIndex = 0;

			unsigned char leftPolyType = 0;
			unsigned char rightPolyType = 0;

			auto leftPolyFace = path[0];
			auto rightPolyFace = path[0];

			std::unordered_map<dtPolyVertex, int, dtPolyPrimitiveHash> vertexSideCache;

			dtPolyVertex fromVertex;
			dtPolyVertex fromFromVertex;
			float fromVertexPos[3], fromFromVertexPos[3];
			dtPolyVertex currVertex;
			for (int i = 0; i <= portalEdgeCount; ++i)
			{
				float left[3], right[3];
				if (i == portalEdgeCount)
				{
					// 注意一定要把终点也加入到漏斗中，不然最后一个拐点会丢失
					dtVcopy(right, endPos);
					dtVcopy(left, endPos);
				}
				else
				{
					// we identify the current vertex and his origin vertex
					auto currEdge = &portalEdges[i];

					int direction = 0;
					dtPolyVertex origin, destination;
					if (!queriers::edgeOriginAndDestinationVertex(*currEdge, &origin, &destination))
					{
						return DT_FAILURE;
					}

					float originPos[3], destinationPos[3];
					queriers::vertexPosition(origin, originPos);
					queriers::vertexPosition(destination, destinationPos);

					if (i == 0)
					{
						auto side = geom::relativeSide(startPos, originPos, destinationPos);
						if (side == geom::REL_SIDE_LEFT)
						{
							vertexSideCache[destination] = geom::REL_SIDE_LEFT;
							vertexSideCache[origin] = geom::REL_SIDE_RIGHT;
						}
						else
						{
							vertexSideCache[destination] = geom::REL_SIDE_RIGHT;
							vertexSideCache[origin] = geom::REL_SIDE_LEFT;
						}

						direction = -side;
						fromVertex = origin;
						fromFromVertex = destination;
						dtVcopy(fromVertexPos, originPos);
						dtVcopy(fromFromVertexPos, destinationPos);
					}
					else
					{
						if (dtVequal(originPos, fromVertexPos))
						{
							currVertex = destination;
						}
						else if (dtVequal(destinationPos, fromVertexPos))
						{
							currVertex = origin;
						}
						else if (dtVequal(originPos, fromFromVertexPos))
						{
							currVertex = destination;
							fromVertex = fromFromVertex;
							dtVcopy(fromVertexPos, fromFromVertexPos);
						}
						else if (dtVequal(destinationPos, fromFromVertexPos))
						{
							currVertex = origin;
							fromVertex = fromFromVertex;
							dtVcopy(fromVertexPos, fromFromVertexPos);
						}
						else
						{
							dtAssert(false && "IMPOSSIBLE TO IDENTIFY THE VERTEX !!!");
							//return DT_FAILURE;
						}

						auto find_iter = vertexSideCache.find(fromVertex);
						dtAssert(find_iter != vertexSideCache.end() && "straightPathByRadius, find fromVertex failed!!!");
						direction = -(*find_iter).second;

						fromFromVertex = fromVertex;
						fromVertex = currVertex;
						dtVcopy(fromFromVertexPos, fromVertexPos);
						queriers::vertexPosition(currVertex, fromVertexPos);

						vertexSideCache[fromVertex] = direction;
						vertexSideCache[fromFromVertex] = -direction;
					}

#if DT_DEBUG_ASTAR
					if (*portalDebugCount < maxPortalDebug)
					{
						auto portalDebug = &portalDebugs[*portalDebugCount];
						portalDebug->portalEdge = *currEdge;
						if (direction == geom::REL_SIDE_RIGHT)
						{
							portalDebug->portalRight = fromVertex;
							portalDebug->portalLeft = fromFromVertex;
						}
						else
						{
							portalDebug->portalRight = fromFromVertex;
							portalDebug->portalLeft = fromVertex;
						}

						(*portalDebugCount)++;
					}
#endif

					if (direction == geom::REL_SIDE_RIGHT)
					{
						// fromVertex on right
						dtVcopy(right, fromVertexPos);
						dtVcopy(left, fromFromVertexPos);
						//queriers::vertexPosition(fromVertex, right);
						//queriers::vertexPosition(fromFromVertex, left);
					}
					else
					{
						// fromVertex on left
						dtVcopy(right, fromFromVertexPos);
						dtVcopy(left, fromVertexPos);
						//queriers::vertexPosition(fromFromVertex, right);
						//queriers::vertexPosition(fromVertex, left);
					}
				}
				

				// Right vertex.
				if (dtTriArea2D(portalApex, portalRight, right) >= 0.0f)
				{
					if (dtVequal(portalApex, portalRight) || dtTriArea2D(portalApex, portalLeft, right) < 0.0f)
					{
						dtVcopy(portalRight, right);
						rightPolyFace = (i + 1 < pathSize) ? path[i + 1] : dtPolyFace::INVALID;
						rightIndex = i;
					}
					else
					{
						dtVcopy(portalApex, portalLeft);
						apexIndex = leftIndex;

						unsigned char flags = 0;
						if (!leftPolyFace)
							flags = DT_STRAIGHTPATH_END;
						else if (leftPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
							flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
						dtPolyFace ref = leftPolyFace;

						// Append or update vertex
						stat = appendVertex(portalApex, flags, ref,
							straightPath, straightPathFlags, straightPathRefs,
							straightPathCount, maxStraightPath);
						if (stat != DT_IN_PROGRESS)
							return stat;

						dtVcopy(portalLeft, portalApex);
						dtVcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;

						// Restart
						i = apexIndex;

						continue;
					}
				}

				// Left vertex.
				if (dtTriArea2D(portalApex, portalLeft, left) <= 0.0f)
				{
					if (dtVequal(portalApex, portalLeft) || dtTriArea2D(portalApex, portalRight, left) > 0.0f)
					{
						dtVcopy(portalLeft, left);
						leftPolyFace = (i + 1 < pathSize) ? path[i + 1] : dtPolyFace::INVALID;
						leftIndex = i;
					}
					else
					{
						dtVcopy(portalApex, portalRight);
						apexIndex = rightIndex;

						unsigned char flags = 0;
						if (!rightPolyFace)
							flags = DT_STRAIGHTPATH_END;
						else if (rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
							flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
						dtPolyFace ref = rightPolyFace;

						// Append or update vertex
						stat = appendVertex(portalApex, flags, ref,
							straightPath, straightPathFlags, straightPathRefs,
							straightPathCount, maxStraightPath);
						if (stat != DT_IN_PROGRESS)
							return stat;

						dtVcopy(portalLeft, portalApex);
						dtVcopy(portalRight, portalApex);
						leftIndex = apexIndex;
						rightIndex = apexIndex;

						// Restart
						i = apexIndex;

						continue;
					}
				}
			}
		}

		// Ignore status return value as we're just about to return anyway.
		appendVertex(endPos, DT_STRAIGHTPATH_END, dtPolyFace::INVALID,
			straightPath, straightPathFlags, straightPathRefs,
			straightPathCount, maxStraightPath);

		return DT_SUCCESS | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
	}
}
