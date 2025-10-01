
#ifndef DETOURPROXIMITYDATABASE_H
#define DETOURPROXIMITYDATABASE_H

#include <functional>

class dtGizmosDrawable
{
public:
	virtual ~dtGizmosDrawable() {}

	virtual void DrawLine(const float* start, const float* end) = 0;
	virtual void DrawAabb(const float* aabb_min, const float* aabb_max) = 0;
};

template <typename T>
class dtProximityDatabase;

template <typename T>
class dtTokenForProximityDatabase
{
public:
	using TProximityDatabase = dtProximityDatabase<T>;

	virtual ~dtTokenForProximityDatabase() {}

	void UpdateForNewPosition(
		TProximityDatabase& database,
		const float* pos);

	int FindNeighbors(
		TProximityDatabase& database,
		const float* center, 
		const float radius, 
		const T* neis, 
		const int maxneinum);

	T data;
};

template <typename T>
class dtProximityDatabase
{
public:
	using TTokenForProximityDatabase = dtTokenForProximityDatabase<T>;
	using TCallback = std::function<bool(const T&)>;
	using TCallbackMut = std::function<bool(T&)>;

	virtual ~dtProximityDatabase() {}

	virtual TTokenForProximityDatabase* AllocToken(const T data) = 0;

	virtual void FreeToken(TTokenForProximityDatabase *token) = 0;

	virtual void ForeachByRadius(const float* center, const float radius, TCallback	callback) = 0;
	virtual void ForeachAllMut(TCallbackMut	callback) = 0;

	virtual void UpdateForNewLocation(TTokenForProximityDatabase& token, const float* pos) = 0;

	virtual void DrawGizmos(dtGizmosDrawable& drawable) = 0;
};

template <typename T>
int dtTokenForProximityDatabase<T>::FindNeighbors(
	TProximityDatabase& database, 
	const float* center, 
	const float radius,
	const T* neis, 
	const int maxneinum)
{
	int neinum = 0;
	database.ForeachByRadius(center, radius, [](const T& data)->bool {
		if (neinum < maxneinum)
		{
			neis[neinum] = data;
			++neinum;
			return true;
		}
		return false;
	});
	return neinum;
}

template <typename T>
void dtTokenForProximityDatabase<T>::UpdateForNewPosition(
	TProximityDatabase& database, 
	const float* pos)
{
	database.UpdateForNewLocation(*this, pos);
}

#endif //DETOURPROXIMITYDATABASE_H