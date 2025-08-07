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
	}
	return DT_FAILURE;
}