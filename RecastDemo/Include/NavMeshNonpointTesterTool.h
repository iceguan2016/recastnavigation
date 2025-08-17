#ifndef NAVMESH_NONPOINT_TESTERTOOL_H
#define NAVMESH_NONPOINT_TESTERTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

void duDebugDrawPoint(duDebugDraw* dd, const float p[3], unsigned int col, const float size = 3.0f);
void duDebugDrawPolyVertex(duDebugDraw* dd, const dtPolyVertex& vertex, unsigned int col, const float size=3.0f);
void duDebugDrawPolyEdge(duDebugDraw* dd, const dtPolyEdge& edge, unsigned int col, const float lineWidth=1.0f);
void duDebugDrawPolyFace(duDebugDraw* dd, const dtPolyFace& face, unsigned int col);

class NavMeshNonpointTesterTool : public SampleTool
{
	Sample* m_sample;

	dtNavMesh* m_navMesh;
	dtNavMeshQuery* m_navQuery;

	dtQueryFilter m_filter;

	dtStatus m_pathFindStatus;

	enum ToolMode
	{
		TOOLMODE_PATHFIND_SET_GOAL,
		TOOLMODE_PATHFIND_FIND_PATH,
		TOOLMODE_DEBUG_DRAW_PRIMITIVES,
	};

	ToolMode m_toolMode;


	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	static const int MAX_VISIT_FACES = 512;
	static const int MAX_FUNNEL_STEPS = 1500;

	dtPolyFace m_startRef;
	dtPolyFace m_endRef;
	float m_polyPickExt[3];

	float m_spos[3];
	float m_epos[3];
	float m_hitPos[3];
	float m_hitNormal[3];
	bool m_hitResult;
	bool m_sposSet;
	bool m_eposSet;

	float		m_dpos[3];
	bool		m_dposSet;
	dtPolyRef	m_debugPolyRef;
	float		m_debugVertexIdx;
	float		m_debugEdgeIdx;
	float		m_debugFaceIdx;

	dtPolyFace	m_pathFaces[MAX_POLYS];
	int			m_nPathFaces;
	dtPolyEdge	m_pathEdges[MAX_POLYS];
	int			m_nPathEdges;

	float		m_straightPath[MAX_POLYS*3];
	int			m_nStraightPath;
	float		m_modifiedStraightPath[MAX_POLYS * 3 * 2];
	int			m_nModifiedStraightPath;

#if DT_DEBUG_ASTAR
	astar::dtAstarNodeDebug m_visitedFaces[MAX_VISIT_FACES];
	int			m_nVisitedFaces;
	funnel::dtFunnelDebug m_portalDebugs[MAX_POLYS];
	int			m_portalDebugCount;
	funnel::dtFunnelStep m_funnelSteps[MAX_FUNNEL_STEPS];
	int			m_funnelStepCount;
	funnel::dtRadiusModifierDebug m_modifierDebugs[MAX_POLYS];
	int			m_modifierDebugCount;
#endif

public:
	NavMeshNonpointTesterTool();

	virtual int type() { return TOOL_NAVMESH_NONPOINT_TESTER; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

	void recalc();
	void drawAgent(const float* pos, float r, float h, float c, const unsigned int col);

	void clampValues();
};

#endif // NAVMESH_NONPOINT_TESTERTOOL_H