#include "DetourAvoidanceQuery.h"

dtEdgeHandle dtEdgeHandle::INVALID;

int rayIntersectWithSegment(const float* ap, const float* u,
	const float* bp, const float* bq,
	float& t)
{
	float v[3], w[3];
	dtVsub(v, bq, bp);
	dtVsub(w, ap, bp);
	float d = dtVperp2D(u, v);
	if (dtMathFabsf(d) < 1e-6f) return 0;
	d = 1.0f / d;
	t = dtVperp2D(v, w) * d;
	if (t < 0 || t > 1) return 0;
	float s = dtVperp2D(u, w) * d;
	if (s < 0 || s > 1) return 0;
	return 1;
}
