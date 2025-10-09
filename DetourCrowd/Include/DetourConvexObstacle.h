
#ifndef DETOURCONVEXOBSTACLE_H
#define DETOURCONVEXOBSTACLE_H

#include <functional>

#include "DetourProximityDatabase.h"
#include "DetourCommon.h"

/// ```svgbob
///             +-----------+
///             | shape[1] |
///             |           |
/// +-----------+----+      |
/// | shape[0]  |    |      |
/// |           |    |      |
/// |     point |    |      |
/// |           *--->|      |
/// |           |  normal   |
/// |           |    |      |
/// |           |    |      |
/// +-----------+----+      |
///             |           |
///             |-----------+
///             .    .
///             :<-->:
///             :    :
///           separation
/// ```
struct dtContactInfo
{
	/// Contact point on the surface of second shape (world space).
	float point[3];
	/// Separation normal at the contact point on the second entity (world space).
	float normal[3];
	/// Separation between two shapes, positive for penetration.
	float separation;
};

class dtConvexObstacle
{
public:
	using TCallback = std::function<bool(const float* p0, const float* p1)>;

	dtConvexObstacle()
	{
		dtVset(worldCenter, 0.0f, 0.0f, 0.0f);

		dtVset(worldAxis[0], 1.0f, 0.0f, 0.0f);
		dtVset(worldAxis[1], 0.0f, 1.0f, 0.0f);
		dtVset(worldAxis[2], 0.0f, 0.0f, 1.0f);

		dtVset(worldAabb[0], 0.0f, 0.0f, 0.0f);
		dtVset(worldAabb[1], 0.0f, 0.0f, 0.0f);
	}

	virtual ~dtConvexObstacle() {}

	virtual bool	ContactEvaluateWithCircle(const float* c, const float r, const float h) const = 0;
	virtual bool	ContactResultWithCircle(const float* c, const float r, dtContactInfo& contact) const = 0;

	virtual void	Tick(float dt) = 0;
	virtual int		SegmentNum() const = 0;
	virtual void	ForeachSegement(TCallback func) const = 0;

	bool LocalToWorldPosition(float* dest, const float* p) const;
	bool WorldToLocalPosition(float* dest, const float* p) const;
	bool LocalToWorldDirection(float* dest, const float* v) const;
	bool WorldToLocalDirection(float* dest, const float* v) const;

public:
	float worldCenter[3];
	float worldAxis[3][3];
	float worldAabb[2][3];
};

class dtBoxObstacle : public dtConvexObstacle
{
public:
	static void box_local_vertices_and_normals(const float* extent, float out_vertices[4][3], float out_normals[4][3]);

	virtual bool	ContactEvaluateWithCircle(const float* c, const float r, const float h) const override;
	// shape0: Box, shape1: Circle
	virtual bool	ContactResultWithCircle(const float* c, const float r, dtContactInfo& contact) const override;

	virtual void	Tick(float dt) override;
	virtual int		SegmentNum() const override;
	virtual void	ForeachSegement(TCallback func) const override;

	void UpdateAabb();

public:
	float localExtent[3];
};

typedef dtConvexObstacle* TConvexObstaclePtr;
typedef dtLocalityProximityDatabase<TConvexObstaclePtr> TConvexObstacleProximityDatabase;

#endif//DETOURCONVEXOBSTACLE_H