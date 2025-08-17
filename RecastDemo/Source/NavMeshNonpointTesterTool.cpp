#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "NavMeshNonpointTesterTool.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Uncomment this to dump all the requests in stdout.
#define DUMP_REQS

// Returns a random number [0..1]
static float frand()
{
	//	return ((float)(rand() & 0xffff)/(float)0xffff);
	return (float)rand() / (float)RAND_MAX;
}

// debug draw
static float debugYOffset = 0.0f;
static bool  debugDrawNeiFaces = false;
static float debugFindPathRadius = 0.5f;
static float debugFindPathIters = 100;
static bool  debugDrawAstarVisitedFaces = false;
static bool  debugDrawAstarPathFaces = true;
static bool  debugDrawFunnelPathEdges = false;
static bool  debugDrawFunnelPortalDebug = true;
static float debugDrawFunnelPortalDebugIndex = 0;
static bool  debugDrawFunnelStraightPath = true;
static float debugDrawFunnelStepIndex = 0;

void duDebugDrawPoint(duDebugDraw* dd, const float p[3], unsigned int col, const float size /*= 3.0f*/)
{
	if (dd == nullptr) return;

	dd->begin(DU_DRAW_POINTS, size);
	float v[3];
	dtVcopy(v, p);
	v[1] += debugYOffset;
	dd->vertex(v, col);
	dd->end();
}

void duDebugDrawLine(duDebugDraw* dd, const float p0[3], const float p1[3], unsigned int col, const float size /*= 3.0f*/)
{
	if (dd == nullptr) return;

	dd->begin(DU_DRAW_LINES, size);
	float v[3];
	dtVcopy(v, p0);
	v[1] += debugYOffset;
	dd->vertex(v, col);

	dtVcopy(v, p1);
	v[1] += debugYOffset;
	dd->vertex(v, col);
	dd->end();
}

void duDebugDrawPolyVertex(duDebugDraw* dd, const dtPolyVertex& vertex, unsigned int col, const float size/* = 3.0f*/)
{
	if (dd == nullptr) return;
	if (!vertex.isValid()) return;

	dd->begin(DU_DRAW_POINTS, size);
	float v[3];
	queriers::vertexPosition(vertex, v);
	
	v[1] += debugYOffset;
	dd->vertex(v, col);
	dd->end();
}

void duDebugDrawPolyEdge(duDebugDraw* dd, const dtPolyEdge& edge, unsigned int col, const float lineWidth/* = 1.0f*/)
{
	if (dd == nullptr) return;
	if (!edge.isValid()) return;

	auto origin = queriers::edgeOriginVertex(edge);
	if (!origin.isValid()) return;
	auto destination = queriers::edgeDestinationVertex(edge);
	if (!destination.isValid()) return;

	float v0[3], v1[3];
	queriers::vertexPosition(origin, v0);
	queriers::vertexPosition(destination, v1);

	v0[1] += debugYOffset;
	v1[1] += debugYOffset;
	duDebugDrawArrow(dd, v0[0], v0[1], v0[2], v1[0], v1[1], v1[2], 0.0f, 0.4f, col, lineWidth);
}

void duDebugDrawPolyFace(duDebugDraw* dd, const dtPolyFace& face, unsigned int col)
{
	if (dd == nullptr) return;
	if (!face.isValid()) return;

	dtPolyVertex verts[3];
	iterations::fromFaceToVertices iterFaceVerts(face);
	auto num = iterFaceVerts.allVertices(verts, 3);
	dtAssert(num == 3);
	
	dd->depthMask(false);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < 3; ++i)
	{
		float v[3];
		queriers::vertexPosition(verts[i], v);

		v[1] += debugYOffset;
		dd->vertex(v, col);
	}
	dd->end();

	dd->depthMask(true);
}

NavMeshNonpointTesterTool::NavMeshNonpointTesterTool() :
	m_sample(0),
	m_navMesh(0),
	m_navQuery(0),
	m_pathFindStatus(DT_FAILURE),
	m_toolMode(TOOLMODE_PATHFIND_SET_GOAL),
	m_hitResult(false),
	m_sposSet(false),
	m_eposSet(false),
	m_dposSet(false),
	m_debugPolyRef(0),
	m_debugVertexIdx(0),
	m_debugEdgeIdx(0),
	m_debugFaceIdx(0),
	m_nPathFaces(0),
	m_nPathEdges(0),
	m_nModifiedStraightPath(0)
#if DT_DEBUG_ASTAR
	,
	m_nVisitedFaces(0),
	m_funnelStepCount(0),
	m_modifierDebugCount(0)
#endif
{
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;
}

void NavMeshNonpointTesterTool::init(Sample* sample)
{
	m_sample = sample;
	m_navMesh = sample->getNavMesh();
	m_navQuery = sample->getNavMeshQuery();
	recalc();

	if (m_navQuery)
	{
		// Change costs.
		m_filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
	}

	debug::dtLogger::Init(debug::LogLevel::INFO, "myapp", "./logs");
}

void NavMeshNonpointTesterTool::handleMenu()
{
	if (imguiCheck("Pathfind SetGoal", m_toolMode == TOOLMODE_PATHFIND_SET_GOAL))
	{
		m_toolMode = TOOLMODE_PATHFIND_SET_GOAL;
		recalc();
	}
	if (imguiCheck("Pathfind FindPath", m_toolMode == TOOLMODE_PATHFIND_FIND_PATH))
	{
		m_toolMode = TOOLMODE_PATHFIND_FIND_PATH;
		recalc();
	}
	if (imguiCheck("Debug DrawPrimitives", m_toolMode == TOOLMODE_DEBUG_DRAW_PRIMITIVES))
	{
		m_toolMode = TOOLMODE_DEBUG_DRAW_PRIMITIVES;
		recalc();
	}

	imguiSeparator();

	imguiLabel("Include Flags");

	imguiIndent();
	if (imguiCheck("Walk", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();
	imguiLabel("Exclude Flags");

	imguiIndent();
	if (imguiCheck("Walk", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();

	imguiSlider("Debug Draw YOffset", &debugYOffset, 0.0f, 2.0f, 0.1f);

	char buff[1024];
	if (m_toolMode == TOOLMODE_DEBUG_DRAW_PRIMITIVES)
	{
		if (m_navMesh && m_debugPolyRef)
		{
			const dtMeshTile* tile = 0;
			const dtPoly* poly = 0;
			if (dtStatusSucceed(m_navMesh->getTileAndPolyByRef(m_debugPolyRef, &tile, &poly)))
			{
				float vertCount = poly->vertCount;

				imguiSlider("Debug Vertex Index", &m_debugVertexIdx, 0.0f, vertCount - 1.0f, 1.0f);
				float edgeCount = DT_VERTS_PER_POLYGON + (vertCount - 3) * 2;
				imguiSlider("Debug Edge Index", &m_debugEdgeIdx, 0.0f, edgeCount - 1.0f, 1.0f);

				float faceCount = vertCount - 2;
				imguiSlider("Debug Face Index", &m_debugFaceIdx, 0.0f, faceCount - 1.0f, 1.0f);
				if (imguiCheck("Debug Draw NeiFaces", debugDrawNeiFaces))
				{
					debugDrawNeiFaces = !debugDrawNeiFaces;
				}

				clampValues();

				
				{
					dtPolyVertex face(m_navMesh, m_debugPolyRef, (int)m_debugFaceIdx);

					// 显示face的顶点索引
					dtPolyVertex verts[3];
					iterations::fromFaceToVertices iterFaceVerts(face);
					auto num_verts = iterFaceVerts.allVertices(verts, 3);
					dtAssert(num_verts == 3);
					sprintf_s(buff, 1024, "face:%ld[%d] verts:[%d, %d, %d]", face.polyId, face.innerIdx, verts[0].innerIdx, verts[1].innerIdx, verts[2].innerIdx);
					imguiValue(buff);

					// 显示face的邻接face
					dtPolyFace faces[3];
					iterations::fromFaceToNeighborFace iterNeiFaces(face);
					auto num_faces = iterNeiFaces.allFaces(faces, 3);
					sprintf_s(
					buff, 
					1024, 
					"nei faces:{%ld[%d], %ld[%d], %ld[%d]}", 
					faces[0].polyId, faces[0].innerIdx, faces[1].polyId, faces[1].innerIdx, faces[2].polyId, faces[2].innerIdx);
					imguiValue(buff);
				}
			}
		}
	}
	else if (m_toolMode == TOOLMODE_PATHFIND_FIND_PATH)
	{
		if (m_navMesh && m_navQuery &&
			m_startRef.isValid() && m_endRef.isValid())
		{
			imguiSlider("Debug Find Path Radius", &debugFindPathRadius, 0.1f, 3.0f, 0.1f);
			imguiSlider("Debug Find Path Iters", &debugFindPathIters, 0.0f, 100.0f, 1.0f);

			const float agentRadius = m_sample->getAgentRadius();
			float radius = debugFindPathRadius; //dtMax(debugFindPathRadius - agentRadius, 0.1f);

			bool isAstar = false;
			bool isFunnel = false;
			bool isModify = false;

			if (imguiButton("Find path"))
			{
				isAstar = true;
				isFunnel = true;
				isModify = true;
			}

			if (imguiButton("Find astar path"))
			{
				isAstar = true;
			}

			if (m_nPathEdges > 0 && imguiButton("Find straight path"))
			{
				isFunnel = true;
			}

			if (m_nStraightPath > 0 && imguiButton("Find modified straight path"))
			{
				isModify = true;
			}

			if (isAstar)
			{
				auto stat = m_navQuery->findPathByRadius(m_startRef, m_endRef, m_spos, m_epos, &m_filter, 
					m_pathFaces, &m_nPathFaces, MAX_POLYS,
					m_pathEdges, &m_nPathEdges, MAX_POLYS,
					radius
				#if DT_DEBUG_ASTAR
					,
					(int)debugFindPathIters,
					m_visitedFaces, m_nVisitedFaces, MAX_VISIT_FACES
				#endif	
					);
			}

			if (isFunnel)
			{
				funnel::straightPathByRadius(m_spos, m_epos,
					m_pathFaces, m_nPathFaces, m_pathEdges, m_nPathEdges,
					m_straightPath, 0, 0, &m_nStraightPath, MAX_POLYS,
					radius
#if DT_DEBUG_ASTAR
					,
					m_portalDebugs, &m_portalDebugCount, MAX_POLYS,
					m_funnelSteps, &m_funnelStepCount, MAX_FUNNEL_STEPS
#endif
				);
			}

			if (isModify)
			{
				funnel::dtRadiusModifier radiusModifier(radius, 10.0f);
				radiusModifier.applyModify(m_straightPath, m_nStraightPath, 
					m_modifiedStraightPath, &m_nModifiedStraightPath, MAX_POLYS*2
				#if DT_DEBUG_ASTAR
					,
					m_modifierDebugs, &m_modifierDebugCount, MAX_POLYS
				#endif
					);
			}

		#if DT_DEBUG_ASTAR
			if (m_nVisitedFaces > 0)
			{
				const auto& face = m_visitedFaces[m_nVisitedFaces-1].face;
				sprintf_s(
					buff,
					1024,
					"last face:%ld[%d]",
					face.polyId, face.innerIdx);
				imguiValue(buff);
			}

			if (m_portalDebugCount > 0)
			{
				imguiSlider("funnel portal debug index", &debugDrawFunnelPortalDebugIndex, 0.0f, (float)(m_portalDebugCount - 1), 1.0f);

				if (imguiButton("add portal debug index"))
				{
					debugDrawFunnelPortalDebugIndex = dtClamp(debugDrawFunnelPortalDebugIndex + 1.0f, 0.0f, (float)(m_portalDebugCount - 1));
				}
				if (imguiButton("sub portal debug index"))
				{
					debugDrawFunnelPortalDebugIndex = dtClamp(debugDrawFunnelPortalDebugIndex - 1.0f, 0.0f, (float)(m_portalDebugCount - 1));
				}

				imguiSeparator();
			}

			if (m_funnelStepCount > 0)
			{
				imguiSlider("funnel step index", &debugDrawFunnelStepIndex, 0.0f, (float)(m_funnelStepCount - 1), 1.0f);

				if (imguiButton("add funnel step index"))
				{
					debugDrawFunnelStepIndex = dtClamp(debugDrawFunnelStepIndex + 1.0f, 0.0f, (float)(m_funnelStepCount - 1));
				}
				if (imguiButton("sub funnel step index"))
				{
					debugDrawFunnelStepIndex = dtClamp(debugDrawFunnelStepIndex - 1.0f, 0.0f, (float)(m_funnelStepCount - 1));
				}

				imguiSeparator();
			}
		#endif

			if (imguiCheck("Draw astar visited faces", debugDrawAstarVisitedFaces))
			{
				debugDrawAstarVisitedFaces = !debugDrawAstarVisitedFaces;
			}
			if (imguiCheck("Draw astar path faces", debugDrawAstarPathFaces))
			{
				debugDrawAstarPathFaces = !debugDrawAstarPathFaces;
			}
			if (imguiCheck("Draw funnel path edges", debugDrawFunnelPathEdges))
			{
				debugDrawFunnelPathEdges = !debugDrawFunnelPathEdges;
			}
			if (imguiCheck("Draw funnel portal debug", debugDrawFunnelPortalDebug))
			{
				debugDrawFunnelPortalDebug = !debugDrawFunnelPortalDebug;
			}
			if (imguiCheck("Draw funnel straight path", debugDrawFunnelStraightPath))
			{
				debugDrawFunnelStraightPath = !debugDrawFunnelStraightPath;
			}
				
		}
		else
		{
			imguiValue("invalid params!");
		}
	}
}

void NavMeshNonpointTesterTool::handleClick(const float* /*s*/, const float* p, bool shift)
{
	if (m_toolMode == TOOLMODE_PATHFIND_SET_GOAL)
	{
		if (shift)
		{
			m_sposSet = true;
			dtVcopy(m_spos, p);
		}
		else
		{
			m_eposSet = true;
			dtVcopy(m_epos, p);
		}
		recalc();
	}
	else if(m_toolMode == TOOLMODE_DEBUG_DRAW_PRIMITIVES)
	{
		m_dposSet = true;
		dtVcopy(m_dpos, p);
		recalc();
	}
}

void NavMeshNonpointTesterTool::handleStep()
{
}

void NavMeshNonpointTesterTool::handleToggle()
{
	if (!m_sposSet || !m_eposSet || !m_startRef || !m_endRef)
		return;

	static const float STEP_SIZE = 0.5f;
	static const float SLOP = 0.01f;

}

void NavMeshNonpointTesterTool::handleUpdate(const float /*dt*/)
{
	
}

void NavMeshNonpointTesterTool::reset()
{
	m_startRef.reset();
	m_endRef.reset();
	memset(m_hitPos, 0, sizeof(m_hitPos));
	memset(m_hitNormal, 0, sizeof(m_hitNormal));
}


void NavMeshNonpointTesterTool::recalc()
{
	if (!m_navMesh)
		return;
	
	if (m_sposSet)
		m_navQuery->findNearestFace(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	else
		m_startRef.reset();

	if (m_eposSet)
		m_navQuery->findNearestFace(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	else
		m_endRef.reset();
	
	if (m_dposSet)
		m_navQuery->findNearestPoly(m_dpos, m_polyPickExt, &m_filter, &m_debugPolyRef, 0);
	else
		m_debugPolyRef = 0;

	m_pathFindStatus = DT_FAILURE;
}

void NavMeshNonpointTesterTool::handleRender()
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	static const unsigned int startCol = duRGBA(128, 25, 0, 192);
	static const unsigned int endCol = duRGBA(51, 102, 0, 129);
	static const unsigned int pathCol = duRGBA(0, 0, 0, 64);
	static const unsigned int faceCol = duRGBA(128, 128, 0, 64);
	static const unsigned int neiFaceCol = duRGBA(0, 128, 128, 64);

	static const unsigned int redCol = duRGBA(255, 0, 0, 255);
	static const unsigned int greenCol = duRGBA(0, 255, 0, 255);
	static const unsigned int blueCol = duRGBA(0, 0, 255, 255);
	static const unsigned int yellowCol = duRGBA(255, 255, 0, 255);

	const float agentRadius = m_sample->getAgentRadius();
	const float agentHeight = m_sample->getAgentHeight();
	const float agentClimb = m_sample->getAgentClimb();

	dd.depthMask(false);
	if (m_sposSet)
		drawAgent(m_spos, debugFindPathRadius, agentHeight, agentClimb, startCol);
	if (m_eposSet)
		drawAgent(m_epos, debugFindPathRadius, agentHeight, agentClimb, endCol);
	dd.depthMask(true);

	if (!m_navMesh)
	{
		return;
	}

	if (m_toolMode == TOOLMODE_PATHFIND_SET_GOAL)
	{
		if (m_startRef.isValid())
		{
			duDebugDrawPolyFace(&dd, m_startRef, faceCol);
		}

		if (m_endRef.isValid())
		{
			duDebugDrawPolyFace(&dd, m_endRef, faceCol);
		}
	}
	else if (m_toolMode == TOOLMODE_DEBUG_DRAW_PRIMITIVES)
	{
		if (m_debugPolyRef)
		{
			clampValues();

			//duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_debugPolyRef, faceCol);

			dtPolyVertex vertex(m_navMesh, m_debugPolyRef, (int)m_debugVertexIdx);
			duDebugDrawPolyVertex(&dd, vertex, startCol, 10.0f);

			dtPolyVertex edge(m_navMesh, m_debugPolyRef, (int)m_debugEdgeIdx);
			duDebugDrawPolyEdge(&dd, edge, startCol);

			dtPolyVertex face(m_navMesh, m_debugPolyRef, (int)m_debugFaceIdx);
			duDebugDrawPolyFace(&dd, face, endCol);

			if (debugDrawNeiFaces)
			{
				dtPolyFace faces[3];
				iterations::fromFaceToNeighborFace iterNeiFaces(face);
				auto num_faces = iterNeiFaces.allFaces(faces, 3);
				for (int i = 0; i < num_faces; ++i)
				{
					duDebugDrawPolyFace(&dd, faces[i], neiFaceCol);
				}
			}
		}
	}
	else if (m_toolMode == TOOLMODE_PATHFIND_FIND_PATH)
	{
#if DT_DEBUG_ASTAR
		static const unsigned int pointCol = duRGBA(255, 255, 0, 64);
		static const unsigned int faceCols[3] = { 
			duRGBA(255, 0, 0, 64),
			duRGBA(0, 255, 0, 64), 
			duRGBA(0, 0, 255, 64), 
		};

		if (debugDrawAstarVisitedFaces && m_nVisitedFaces > 0)
		{
			for (int i = 0; i < m_nVisitedFaces; ++i)
			{
				duDebugDrawPolyFace(&dd, m_visitedFaces[i].face, faceCols[i % 3]);
				duDebugDrawPoint(&dd, m_visitedFaces[i].entry_pos, pointCol, 10.0f);
			}
		}

		if (debugDrawFunnelPortalDebug && m_portalDebugCount > 0)
		{
			int debugIndex = (int)dtClamp(debugDrawFunnelPortalDebugIndex, 0.0f, (float)(m_portalDebugCount-1));
			for (int i = 0; i < m_portalDebugCount; ++i)
			{
				const auto& d = m_portalDebugs[i];
				duDebugDrawPolyEdge(&dd, d.portalEdge, redCol, 1.0f);

				if (i == debugIndex)
				{
					float left[3], right[3];
					queriers::vertexPosition(d.portalLeft, left);
					queriers::vertexPosition(d.portalRight, right);
					duDebugDrawPoint(&dd, left, greenCol, 10.0f);
					duDebugDrawPoint(&dd, right, blueCol, 10.0f);
				}
			}
		}
#endif

		if (debugDrawAstarPathFaces && m_nPathFaces > 0)
		{
			for (int i = 0; i < m_nPathFaces; ++i)
			{
				duDebugDrawPolyFace(&dd, m_pathFaces[i], faceCols[i % 3]);
			}
		}

		if (debugDrawFunnelPathEdges && m_nPathEdges > 0)
		{
			for (int i = 0; i < m_nPathEdges; ++i)
			{
				const auto& e = m_pathEdges[i];
				duDebugDrawPolyEdge(&dd, e, redCol, 5.0f);
			}
		}

		if (debugDrawFunnelStraightPath && m_nStraightPath > 0)
		{
			for (int i = 0; i < m_nStraightPath; ++i)
			{
				const float* p = &m_straightPath[i*3];
				//duDebugDrawPoint(&dd, p, redCol, 3.0f);

				if ((i+1) < m_nStraightPath)
				{
					const float* np = &m_straightPath[(i+1)*3];
					duDebugDrawLine(&dd, p, np, yellowCol, 3.0f);
				}
			}

			for (int i = 0; i < m_nModifiedStraightPath; ++i)
			{
				const float* p = &m_modifiedStraightPath[i * 3];
				//duDebugDrawPoint(&dd, p, redCol, 3.0f);

				if ((i + 1) < m_nModifiedStraightPath)
				{
					const float* np = &m_modifiedStraightPath[(i + 1) * 3];
					duDebugDrawLine(&dd, p, np, redCol, 3.0f);
				}
			}

			for (int i = 0; i < m_modifierDebugCount; ++i)
			{
				auto& d = m_modifierDebugs[i];
				duDebugDrawCircle(&dd, d.pos[0], d.pos[1], d.pos[2], d.radius, redCol, 1.0f);
			}
		}

		if (m_funnelStepCount > 0)
		{
			for (int i = 0; i < m_funnelStepCount; ++i)
			{
				const auto& d = m_funnelSteps[i];

				if (i == (int)debugDrawFunnelStepIndex)
				{
					duDebugDrawPoint(&dd, d.apexPos, redCol, 10.0f);
					duDebugDrawLine(&dd, d.apexPos, d.leftPos, greenCol, 3.0f);
					duDebugDrawLine(&dd, d.apexPos, d.rightPos, blueCol, 3.0f);

					if (d.currSide == 1)
						duDebugDrawPoint(&dd, d.currPos, greenCol, 10.0f);
					else
						duDebugDrawPoint(&dd, d.currPos, blueCol, 10.0f);
						
				}
			}
		}
	}
}

void NavMeshNonpointTesterTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;

	// Draw start and end point labels
	if (m_sposSet && gluProject((GLdouble)m_spos[0], (GLdouble)m_spos[1], (GLdouble)m_spos[2],
		model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0, 0, 0, 220));
	}
	if (m_eposSet && gluProject((GLdouble)m_epos[0], (GLdouble)m_epos[1], (GLdouble)m_epos[2],
		model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, "End", imguiRGBA(0, 0, 0, 220));
	}

	// Tool help
	const int h = view[3];
	imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB+SHIFT: Set start location  LMB: Set end location", imguiRGBA(255, 255, 255, 192));
}

void NavMeshNonpointTesterTool::drawAgent(const float* pos, float r, float h, float c, const unsigned int col)
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	dd.depthMask(false);

	// Agent dimensions.	
	duDebugDrawCylinderWire(&dd, pos[0] - r, pos[1] + 0.02f, pos[2] - r, pos[0] + r, pos[1] + h, pos[2] + r, col, 2.0f);

	duDebugDrawCircle(&dd, pos[0], pos[1] + c, pos[2], r, duRGBA(0, 0, 0, 64), 1.0f);

	unsigned int colb = duRGBA(0, 0, 0, 196);
	dd.begin(DU_DRAW_LINES);
	dd.vertex(pos[0], pos[1] - c, pos[2], colb);
	dd.vertex(pos[0], pos[1] + c, pos[2], colb);
	dd.vertex(pos[0] - r / 2, pos[1] + 0.02f, pos[2], colb);
	dd.vertex(pos[0] + r / 2, pos[1] + 0.02f, pos[2], colb);
	dd.vertex(pos[0], pos[1] + 0.02f, pos[2] - r / 2, colb);
	dd.vertex(pos[0], pos[1] + 0.02f, pos[2] + r / 2, colb);
	dd.end();

	dd.depthMask(true);
}

void NavMeshNonpointTesterTool::clampValues()
{
	if (!m_navMesh)
		return;

	if (m_toolMode == TOOLMODE_DEBUG_DRAW_PRIMITIVES)
	{
		if (m_navMesh && m_debugPolyRef)
		{
			const dtMeshTile* tile = 0;
			const dtPoly* poly = 0;
			if (dtStatusSucceed(m_navMesh->getTileAndPolyByRef(m_debugPolyRef, &tile, &poly)))
			{
				float vertCount = poly->vertCount;

				m_debugVertexIdx = dtClamp(m_debugVertexIdx, 0.0f, vertCount);

				float edgeCount = DT_VERTS_PER_POLYGON + (vertCount - 3) * 2;

				if (m_debugEdgeIdx < DT_VERTS_PER_POLYGON)
				{
					m_debugEdgeIdx = dtClamp(m_debugEdgeIdx, 0.0f, vertCount - 1.0f);
				}
				else
				{
					m_debugEdgeIdx = dtClamp(m_debugEdgeIdx, (float)DT_VERTS_PER_POLYGON, edgeCount - 1.0f);
				}

				float faceCount = vertCount - 2;
				m_debugFaceIdx = dtClamp(m_debugFaceIdx, 0.0f, faceCount);
			}
		}
	}
}
