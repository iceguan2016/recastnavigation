#ifndef NAVMESH_NONPOINT_TESTERTOOL_H
#define NAVMESH_NONPOINT_TESTERTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

void duDebugDrawInternalVertex(duDebugDraw* dd, const dtInternalVertex& vertex, unsigned int col, const float size=3.0f);
void duDebugDrawInternalEdge(duDebugDraw* dd, const dtInternalEdge& edge, unsigned int col, const float lineWidth=1.0f);
void duDebugDrawInternalFace(duDebugDraw* dd, const dtInternalFace& face, unsigned int col);

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

	dtInternalFace m_startRef;
	dtInternalFace m_endRef;
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
};

#endif // NAVMESH_NONPOINT_TESTERTOOL_H