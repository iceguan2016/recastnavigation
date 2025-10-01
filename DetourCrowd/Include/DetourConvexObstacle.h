
#ifndef DETOURCONVEXOBSTACLE_H
#define DETOURCONVEXOBSTACLE_H

#include <functional>

#include "DetourProximityDatabase.h"

class dtConvexObstacle
{
public:
	using TCallback = std::function<bool(const float* p0, const float* p1)>;

	virtual ~dtConvexObstacle() {}

	virtual bool Eval(const float* c, const float r) const = 0;
	virtual void Tick(float dt) const = 0;
	virtual int SegmentNum() const = 0;
	virtual int ForeachSegement(TCallback func) const = 0;
};

typedef dtConvexObstacle* TConvexObstaclePtr;
typedef dtProximityDatabase<TConvexObstaclePtr> TConvexObstacleProximityDatabase;

#endif//DETOURCONVEXOBSTACLE_H