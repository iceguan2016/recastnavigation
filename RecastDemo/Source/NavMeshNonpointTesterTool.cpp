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
void duDebugDrawInternalVertex(duDebugDraw* dd, const dtInternalVertex& vertex, unsigned int col, const float size/* = 3.0f*/)
{
	if (dd == nullptr) return;
	if (!vertex.isValid()) return;

	dd->begin(DU_DRAW_POINTS, size);
	float v[3];
	queriers::vertexPosition(vertex, v);
	dd->vertex(v, col);
	dd->end();
}

void duDebugDrawInternalEdge(duDebugDraw* dd, const dtInternalEdge& edge, unsigned int col, const float lineWidth/* = 1.0f*/)
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

	duDebugDrawArrow(dd, v0[0], v0[1], v0[2], v1[0], v1[1], v1[2], 0.0f, 0.4f, col, lineWidth);
}

void duDebugDrawInternalFace(duDebugDraw* dd, const dtInternalFace& face, unsigned int col)
{
	if (dd == nullptr) return;
	if (!face.isValid()) return;

	dtInternalVertex verts[3];
	iterations::fromFaceToVertices iterFaceVerts(face);
	auto num = iterFaceVerts.allVertices(verts, 3);
	dtAssert(num == 3);
	
	dd->depthMask(false);

	for (int i = 0; i < 3; ++i)
	{
		float v[3];
		queriers::vertexPosition(verts[i], v);
		dd->vertex(v, col);
		dd->end();
	}

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
	m_eposSet(false)
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
}

void NavMeshNonpointTesterTool::handleClick(const float* /*s*/, const float* p, bool shift)
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

	m_pathFindStatus = DT_FAILURE;
}

void NavMeshNonpointTesterTool::handleRender()
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	static const unsigned int startCol = duRGBA(128, 25, 0, 192);
	static const unsigned int endCol = duRGBA(51, 102, 0, 129);
	static const unsigned int pathCol = duRGBA(0, 0, 0, 64);
	static const unsigned int faceCol = duRGBA(128, 128, 0, 64);

	const float agentRadius = m_sample->getAgentRadius();
	const float agentHeight = m_sample->getAgentHeight();
	const float agentClimb = m_sample->getAgentClimb();

	dd.depthMask(false);
	if (m_sposSet)
		drawAgent(m_spos, agentRadius, agentHeight, agentClimb, startCol);
	if (m_eposSet)
		drawAgent(m_epos, agentRadius, agentHeight, agentClimb, endCol);
	dd.depthMask(true);

	if (!m_navMesh)
	{
		return;
	}

	if (m_startRef.isValid())
	{
		duDebugDrawInternalFace(&dd, m_startRef, faceCol);
	}

	if (m_endRef.isValid())
	{
		duDebugDrawInternalFace(&dd, m_endRef, faceCol);
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
