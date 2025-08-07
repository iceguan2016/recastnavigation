#include "DetourNavMeshQuery_Nonpoint.h"
#include "DetourNavMeshQuery.h"

dtInternalPrimitive dtInternalPrimitive::INVALID(nullptr, 0, -1);

dtStatus dtNavMeshQuery::findNearestFace(const float* center, const float* halfExtents,
	const dtQueryFilter* filter,
	dtInternalFace* nearestFace, float* nearestPt) const
{
	dtPolyRef nearestPoly = 0;
	if (dtStatusSucceed(findNearestPoly(center, halfExtents, filter, &nearestPoly, nearestPt)))
	{
		dtInternalFace face(m_nav, nearestPoly, 0);
		// 遍历polygon内部face找到包含nearestPt点的face
		iterations::fromFaceToNeighborFace iterNeiFaces(face);
		do 
		{
			auto f = iterNeiFaces.next();
			if (!f.isValid())
				break;
			// 不处理外部相连的face
			if (f.polyId != nearestPoly)
				continue;

			dtInternalVertex verts[3];
			iterations::fromFaceToVertices iterFaceVerts(face);
			auto num = iterFaceVerts.allVertices(verts, 3);
			dtAssert(num == 3);
			float v0[3], v1[3], v2[3];
			queriers::vertexPosition(verts[0], v0);
			queriers::vertexPosition(verts[1], v1);
			queriers::vertexPosition(verts[2], v2);

			float s0 = dtTriArea2D(nearestPt, v0, v1);
			float s1 = dtTriArea2D(nearestPt, v1, v2);
			float s2 = dtTriArea2D(nearestPt, v2, v0);
			if ((s0 * s1) >= 0 && (s0 * s2) >= 0)
			{
				*nearestFace = f;
				return DT_SUCCESS;
			}
		} while (true);
	}
	return DT_FAILURE;
}