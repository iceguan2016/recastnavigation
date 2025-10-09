//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef RECASTSAMPLETEMPOBSTACLE_H
#define RECASTSAMPLETEMPOBSTACLE_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "ChunkyTriMesh.h"

// add by iceguan
#include "DetourConvexObstacle.h"

#include <vector>
#include <list>
#include <map>
#include <memory>

class GizmosDrawable : public dtGizmosDrawable
{

public:
	GizmosDrawable(SampleDebugDraw* dd)
		: m_dd(dd)
	{
	}

	void DrawLine(const float* start, const float* end, const dtGizmosColor& color) override;

	void DrawAabb(const float* aabb_min, const float* aabb_max, const dtGizmosColor& color) override;

protected:
	SampleDebugDraw* m_dd;
};

class DynamicBoxObstacle : public dtBoxObstacle
{
public:
	typedef dtBoxObstacle Super;

	void RotateYAngle(const float yangle);
	void MoveDelta(const float* delta);

	void DrawGizmos(dtGizmosDrawable& drawable);

	void Tick(float dt) override;

public:
	float angular_speed = 0.0f;
	float move_start[3];
	float move_end[3];
	float move_dir = 1.0f;
	float move_speed = 0.0f;
};

class BoxObstacleManager : public dtLocalityProximityDatabase<TConvexObstaclePtr>
{
public:
	typedef dtLocalityProximityDatabase<TConvexObstaclePtr> Super;

	BoxObstacleManager(
		const float* origin,
		const float* size,
		const int divx,
		const int divy,
		const int divz)
		: Super(origin, size, divx, divy, divz)
	{

	}

	TTokenForProximityDatabase* AddBoxObstacle(
		const float* pos, 
		const float* extent, 
		const float yangle,
		const float angular_speed,
		const float* move_dir,
		const float move_speed);
	void RemoveBoxObstacle(TTokenForProximityDatabase* token);

	void DrawGizmos(dtGizmosDrawable& drawable) override;

protected:
	using TBoxObstaclePtr = std::shared_ptr<DynamicBoxObstacle>;

	std::vector<TBoxObstaclePtr> m_boxObstacles;
};
// end


class Sample_TempObstacles : public Sample
{
protected:
	bool m_keepInterResults;

	struct LinearAllocator* m_talloc;
	struct FastLZCompressor* m_tcomp;
	struct MeshProcess* m_tmproc;

	class dtTileCache* m_tileCache;
	
	float m_cacheBuildTimeMs;
	int m_cacheCompressedSize;
	int m_cacheRawSize;
	int m_cacheLayerCount;
	unsigned int m_cacheBuildMemUsage;
	
	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
		DRAWMODE_NAVMESH_PORTALS,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_CACHE_BOUNDS,
		MAX_DRAWMODE
	};
	
	DrawMode m_drawMode;
	
	int m_maxTiles;
	int m_maxPolysPerTile;
	float m_tileSize;

	// add by iceguan
	BoxObstacleManager* m_obstacles;
	// end
	
public:
	Sample_TempObstacles();
	virtual ~Sample_TempObstacles();
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	virtual void handleUpdate(const float dt);

	void getTilePos(const float* pos, int& tx, int& ty);
	
	void renderCachedTile(const int tx, const int ty, const int type);
	void renderCachedTileOverlay(const int tx, const int ty, double* proj, double* model, int* view);

	void addTempObstacle(const float* pos);
	void removeTempObstacle(const float* sp, const float* sq);
	void clearAllTempObstacles();

	void saveAll(const char* path);
	void loadAll(const char* path);

	void addBoxObstacle(const float* pos);
	void removeBoxObstacle(const float* sp, const float* sq);

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample_TempObstacles(const Sample_TempObstacles&);
	Sample_TempObstacles& operator=(const Sample_TempObstacles&);

	int rasterizeTileLayers(const int tx, const int ty, const rcConfig& cfg, struct TileCacheData* tiles, const int maxTiles);
};


#endif // RECASTSAMPLETEMPOBSTACLE_H
