#include "DetourNavMeshQuery_Nonpoint.h"
#include "DetourNavMeshQuery.h"

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