
#ifndef DETOURPROXIMITYDATABASE_H
#define DETOURPROXIMITYDATABASE_H

#include <functional>
#include <list>

#include "DetourAssert.h"

struct dtGizmosColor
{
	unsigned char r, g, b, a;

	dtGizmosColor(
		unsigned char r, 
		unsigned char g, 
		unsigned char b, 
		unsigned char a)
	{
		this->r = r;
		this->g = g;
		this->b = b;
		this->a = a;
	}
};

class dtGizmosDrawable
{
public:
	virtual ~dtGizmosDrawable() {}

	virtual void DrawLine(const float* start, const float* end, const dtGizmosColor& color) = 0;
	virtual void DrawAabb(const float* aabb_min, const float* aabb_max, const dtGizmosColor& color) = 0;
};

template <typename TDataType, typename TTokenPayload>
class dtProximityDatabase;

template <typename TDataType, typename TPayload>
class dtTokenForProximityDatabase
{
public:
	using TData = TDataType;
	using TProximityDatabase = dtProximityDatabase<TDataType, TPayload>;

	virtual ~dtTokenForProximityDatabase() {}

	void UpdateForNewPosition(
		TProximityDatabase& database,
		const float* pos);

	int FindNeighbors(
		TProximityDatabase& database,
		const float* center,
		const float radius,
		const TDataType* neis,
		const int maxneinum);

	TPayload proxy;
};

template <typename TDataType, typename TTokenPayload>
class dtProximityDatabase
{
public:
	using TTokenForProximityDatabase = dtTokenForProximityDatabase<TDataType, TTokenPayload>;
	using TDatabaseQueryFilter = std::function<bool(const TDataType&)>;
	using TDatabaseQueryFilterMut = std::function<bool(TDataType&)>;

	virtual ~dtProximityDatabase() {}

	virtual TTokenForProximityDatabase* AllocToken(const TDataType& data) = 0;

	virtual void FreeToken(TTokenForProximityDatabase* token) = 0;

	virtual void ForeachByRadius(const float* center, const float radius, TDatabaseQueryFilter filter) = 0;
	virtual void ForeachAllMut(TDatabaseQueryFilterMut	filter) = 0;

	virtual void UpdateForNewLocation(TTokenForProximityDatabase& token, const float* pos) = 0;

	virtual void DrawGizmos(dtGizmosDrawable& drawable) = 0;
};

// dtTokenForProximityDatabase
template <typename TDataType, typename TPayload>
int dtTokenForProximityDatabase<TDataType, TPayload>::FindNeighbors(
	TProximityDatabase& database,
	const float* center,
	const float radius,
	const TDataType* neis,
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

template <typename TDataType, typename TPayload>
void dtTokenForProximityDatabase<TDataType, TPayload>::UpdateForNewPosition(
	TProximityDatabase& database,
	const float* pos)
{
	database.UpdateForNewLocation(*this, pos);
}

// dtLocalityProximityDatabase
template<typename T>
struct dtLocalityClientProxy
{
	// previous obj in this bin, or null
	dtLocalityClientProxy* prev;

	// next obj in this bin, or null
	dtLocalityClientProxy* next;

	// bin ID (index into bin contents list)
	int bin;

	// pointer to client obj
	T obj;

	// the obj's location ("key point") used for spatial sorting
	float position[3];

	dtLocalityClientProxy(const T& obj)
	{
		prev = 0;
		next = 0;
		bin = -1;
		this->obj = obj;
		dtVset(position, 0.0f, 0.0f, 0.0f);
	}
};

template<typename T>
class dtLocalityProximityDatabase : public dtProximityDatabase<T, dtLocalityClientProxy<T>*>
{

public:
	using Self = dtLocalityProximityDatabase<T>;

	using TClentProxy = dtLocalityClientProxy<T>;
	using TClientProxyList = std::list<TClentProxy>;

	using TTokenList = std::list<TTokenForProximityDatabase>;

	TTokenForProximityDatabase* AllocToken(const T& data) override
	{
		client_proxies.emplace_back(TClentProxy(data));

		auto token = tokens.emplace(tokens.end(), TTokenForProximityDatabase());
		token->proxy = &client_proxies.back();
		return &tokens.back();
	}


	void FreeToken(TTokenForProximityDatabase* token) override
	{
		auto token_iter = std::find_if(
			tokens.begin(), 
			tokens.end(),
			[token](const TTokenForProximityDatabase& item) {
				return &item == token;
			});

		if (token_iter != tokens.end())
		{
			auto client_proxy = token_iter->proxy;
			auto client_proxy_iter = std::find_if(
				client_proxies.begin(),
				client_proxies.end(),
				[client_proxy](const TClentProxy & item) {
					return &item == client_proxy;
				});

			if (client_proxy_iter != client_proxies.end())
			{
				client_proxies.erase(client_proxy_iter);
			}

			tokens.erase(token_iter);
		}
	}


	void ForeachByRadius(const float* center, const float radius, TDatabaseQueryFilter filter) override
	{
		map_over_all_objects_in_locality(center, radius, filter);
	}

	void ForeachAllMut(TDatabaseQueryFilterMut filter) override
	{
		for (auto& proxy : client_proxies)
		{
			filter(proxy.obj);
		}
	}


	void UpdateForNewLocation(TTokenForProximityDatabase& token, const float* pos) override
	{
		/* find bin for new location */
		auto new_bin = bin_for_location(pos);

		/* store location in client obj, for future reference */
		int obj_bin = -1;
		if (token.proxy)
		{
			dtVcopy(token.proxy->position, pos);
			obj_bin = token.proxy->bin;
		}

		/* has obj moved into a new bin? */
		if (new_bin != obj_bin)
		{
			remove_from_bin(token.proxy);
			add_to_bin(token.proxy, new_bin);
		}
	}


	void DrawGizmos(dtGizmosDrawable& drawable) override
	{
		auto slab = div_y * div_z;
		auto row = div_z;

		float bin_size[3] = { size[0] / div_x, size[1] / div_y, size[2] / div_z };

		float bin_scale = 0.95f;
		float sacled_half_bin_size[3];
		dtVscale(sacled_half_bin_size, bin_size, bin_scale / 2);

		/* loop for x bins across diameter of sphere */
		int iindex = 0;
		for (int i = 0; i < div_x; ++i) 
		{
			/* loop for y bins across diameter of sphere */
			int jindex = 0;
			for (int j = 0; j < div_y; ++j) 
			{
				/* loop for z bins across diameter of sphere */
				int kindex = 0;
				for (int k = 0; k < div_z; ++k)
				{
					/* get current bin's client obj list */
					auto bin = bins[iindex + jindex + kindex];
					auto is_empty = bin == 0;

					float min[3] = { origin[0] + bin_size[0] * i, origin[1] + bin_size[1] * j, origin[2] + bin_size[2] * k };
					float max[3];
					dtVadd(max, min, bin_size);

					float center[3] = { (max[0] + min[0]) * 0.5f, (max[1] + min[1]) * 0.5f, (max[2] + min[2]) * 0.5f};
					dtVsub(min, center, sacled_half_bin_size);
					dtVadd(max, center, sacled_half_bin_size);

					auto color = is_empty? dtGizmosColor(255, 255, 0, 255) : dtGizmosColor(255, 0, 0, 255);
					drawable.DrawAabb(min, max, color);

					kindex += 1;
				}
				jindex += row;
			}
			iindex += slab;
		}
	}

	dtLocalityProximityDatabase(
		const float* origin,
		const float* size,
		const int divx,
		const int divy,
		const int divz)
	{
		dtAssert(divx > 0 && divy > 0 && divz > 0);

		dtVcopy(this->origin, origin);
		dtVcopy(this->size, size);
		div_x = divx;
		div_y = divy;
		div_z = divz;

		bin_count = divx * divy * divz + 1;
		bins = new TClentProxy * [bin_count];
		for (int i = 0; i < bin_count; ++i)
		{
			bins[i] = 0;
		}
	}

	static Self* Create(
		const float* origin,
		const float* size,
		const int divx,
		const int divy,
		const int divz)
	{
		return new Self(origin, size, divx, divy, divz);
	}

	dtLocalityProximityDatabase()
		: div_x(0)
		, div_y(0)
		, div_z(0)
		, bins(0)
		, bin_count(0)
	{

	}

	virtual ~dtLocalityProximityDatabase()
	{
		if (bins)
		{
			delete bins;
			bins = 0;
		}
	}

protected:
	/* Determine index into linear bin array given 3D bin indices */
	int bin_coords_to_bin_index(const int ix, const int iy, const int iz)
	{
		return ((ix * div_y * div_z) + (iy * div_z) + iz);
	}

	/* Find the bin ID for a location in space.  The location is given in
	terms of its XYZ coordinates.  The bin ID is a pointer to a pointer
	to the bin contents list.  */

	int bin_for_location(const float* position)
	{
		float min[3], max[3];
		dtVcopy(min, origin);
		dtVadd(max, origin, size);
		/* if point outside super-brick, return the "other" bin */
		if (position[0] < min[0]
			|| position[1] < min[1]
			|| position[2] < min[2]
			|| position[0] >= max[0]
			|| position[1] >= max[1]
			|| position[2] >= max[2])
		{
			return bin_count - 1;
		}

		/* if point inside super-brick, compute the bin coordinates */
		float disp[3];
		dtVsub(disp, position, origin);
		int ix = (int)floor(disp[0] / size[0] * div_x);
		int iy = (int)floor(disp[1] / size[1] * div_y);
		int iz = (int)floor(disp[2] / size[2] * div_z);
		dtAssert(ix >= 0 && iy >= 0 && iz >= 0);

		/* convert to linear bin number */
		auto i = bin_coords_to_bin_index(ix, iy, iz);
		return i;
	}

	/* Adds a given client obj to a given bin, linking it into the bin contents list. */
	void add_to_bin(TClentProxy* obj, const int bin_index)
	{
		if (!obj)
		{
			return;
		}

		/* if bin is currently empty */
		if (bins[bin_index] == 0) {
			obj->prev = 0;
			obj->next = 0;
		}
		else {
			obj->prev = 0;
			obj->next = bins[bin_index];
			bins[bin_index]->prev = obj;
		}

		bins[bin_index] = obj;

		/* record bin ID in proxy obj */
		obj->bin = bin_index;
	}

	/* Removes a given client obj from its current bin, unlinking it
	from the bin contents list. */
	void remove_from_bin(TClentProxy* obj)
	{
		if (!obj) 
		{
			return;
		}

		/* adjust pointers if obj is currently in a bin */
		if (obj->bin != -1)
		{
			/* If this obj is at the head of the list, move the bin
			   pointer to the next item in the list (might be null). */
			if (bins[obj->bin] == obj)
				bins[obj->bin] = obj->next;

			/* If there is a prev obj, link its "next" pointer to the
			   obj after this one. */
			if (obj->prev != 0)
				obj->prev->next = obj->next;

			/* If there is a next obj, link its "prev" pointer to the
			   obj before this one. */
			if (obj->next != 0)
				obj->next->prev = obj->prev;
		}

		/* Null out prev, next and bin pointers of this obj. */
		obj->prev = 0;
		obj->next = 0;
		obj->bin = -1;
	}

	void traverse_bin_client_object_list(
		TClentProxy* proxy,
		const float position[3],
		const float radius_squared,
		TDatabaseQueryFilter filter)
	{
		auto curr_handle = proxy;
		while (curr_handle != 0)
		{
			auto distance_sqaured = dtVdist(curr_handle->position, position);
			if (distance_sqaured < radius_squared)
			{
				filter(curr_handle->obj);
			}

			curr_handle = curr_handle->next;
		}
	}

	/// If the query region (sphere) extends outside of the "super-brick"
	/// we need to check for objects in the catch-all "other" bin which
	/// holds any object which are not inside the regular sub-bricks
	void map_over_all_outside_objects(
		const float center[3],
		const float radius,
		TDatabaseQueryFilter filter)
	{
		dtAssert(bin_count > 0);
		auto last = bins[bin_count - 1];
		if (last != 0)
		{
			auto radius_squared = radius * radius;
			// traverse the "other" bin's client object list
			traverse_bin_client_object_list(last, center, radius_squared, filter);
		}
	}

	/// This subroutine of lqMapOverAllObjectsInLocality efficiently
	/// traverses of subset of bins specified by max and min bin
	/// coordinates.
	void map_over_all_objects_in_locality_clipped(
		const float center[3],
		const float radius,
		TDatabaseQueryFilter filter,
		const int min_bin_x,
		const int min_bin_y,
		const int min_bin_z,
		const int max_bin_x,
		const int max_bin_y,
		const int max_bin_z)
	{
		dtAssert(min_bin_x >= 0 && min_bin_x < div_x);
		dtAssert(min_bin_y >= 0 && min_bin_y < div_x);
		dtAssert(min_bin_z >= 0 && min_bin_z < div_x);

		dtAssert(max_bin_x >= 0 && max_bin_x < div_x);
		dtAssert(max_bin_y >= 0 && max_bin_y < div_y);
		dtAssert(max_bin_z >= 0 && max_bin_z < div_z);

		auto slab = div_y * div_z;
		auto row = div_z;
		auto istart = min_bin_x * slab;
		auto jstart = min_bin_y * row;
		auto kstart = min_bin_z;
		auto radius_squared = radius * radius;

		/* loop for x bins across diameter of sphere */
		auto iindex = istart;
		for (int i = min_bin_x; i <= max_bin_x; ++i)
		{
			/* loop for y bins across diameter of sphere */
			auto jindex = jstart;
			for (int j = min_bin_y; j <= max_bin_y; ++j)
			{
				/* loop for z bins across diameter of sphere */
				auto kindex = kstart;
				for (int k = min_bin_z; k <= max_bin_z; ++k)
				{
					/* get current bin's client obj list */
					auto bin = bins[iindex + jindex + kindex];
					auto co = bin;

					/* traverse current bin's client obj list */
					traverse_bin_client_object_list(co, center, radius_squared, filter);
					kindex += 1;
				}
				jindex += row;
			}
			iindex += slab;
		}
	}

	void map_over_all_objects_in_locality(
		const float center[3],
		const float radius,
		TDatabaseQueryFilter filter)
	{
		auto partly_out = false;
		float min[3], max[3];
		float half_extent[3] = { radius, radius, radius };
		dtVsub(min, center, half_extent);
		dtVadd(max, center, half_extent);

		float bound_min[3], bound_max[3];
		dtVcopy(bound_min, origin);
		dtVadd(bound_max, origin, size);
		auto completely_outside = (max[0] < bound_min[0])
			|| (max[1] < bound_min[1])
			|| (max[2] < bound_min[2])
			|| (min[0] >= bound_max[0])
			|| (min[1] >= bound_max[1])
			|| (min[2] >= bound_max[2]);

		/* is the sphere completely outside the "super brick"? */
		if (completely_outside) {
			map_over_all_outside_objects(center, radius, filter);
			return;
		}

		/* compute min and max bin coordinates for each dimension */
		float min_disp[3], max_disp[3];
		dtVsub(min_disp, min, origin);
		int min_bin_x = (int)floor(min_disp[0] / size[0] * div_x);
		int min_bin_y = (int)floor(min_disp[1] / size[1] * div_y);
		int min_bin_z = (int)floor(min_disp[2] / size[2] * div_z);

		dtVsub(max_disp, max, origin);
		int max_bin_x = (int)floor(max_disp[0] / size[0] * div_x);
		int max_bin_y = (int)floor(max_disp[1] / size[1] * div_y);
		int max_bin_z = (int)floor(max_disp[2] / size[2] * div_z);

		dtAssert(div_x > 0 && div_y > 0 && div_z > 0);

		/* clip bin coordinates */
		if (min_bin_x < 0)
		{
			partly_out = true;
			min_bin_x = 0;
		}

		if (min_bin_y < 0)
		{
			partly_out = true;
			min_bin_y = 0;
		}

		if (min_bin_z < 0)
		{
			partly_out = true;
			min_bin_z = 0;
		}

		if (max_bin_x >= div_x)
		{
			partly_out = true;
			max_bin_x = div_x - 1;
		}
		if (max_bin_y >= div_y)
		{
			partly_out = true;
			max_bin_y = div_y - 1;
		}
		if (max_bin_z >= div_z)
		{
			partly_out = true;
			max_bin_z = div_z - 1;
		}

		/* map function over outside objects if necessary (if clipped) */
		if (partly_out)
		{
			map_over_all_outside_objects(center, radius, filter);
		}

		/* map function over objects in bins */
		map_over_all_objects_in_locality_clipped(
			center,
			radius,
			filter,
			min_bin_x,
			min_bin_y,
			min_bin_z,
			max_bin_x,
			max_bin_y,
			max_bin_z);
	}

protected:
	// the origin is the super-brick corner minimum coordinates
	float origin[3];
	// length of the edges of the super-brick
	float size[3];
	// number of sub-brick divisions in each direction
	int div_x;
	int div_y;
	int div_z;

	TTokenList tokens;

	TClientProxyList client_proxies;
	TClentProxy** bins;
	int bin_count;
};

#endif //DETOURPROXIMITYDATABASE_H