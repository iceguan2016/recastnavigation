#include "DetourConvexObstacle.h"

bool intersection_circles(
	const float* c0,
	const float r0,
	const float* c1,
	const float r1,
	float* out_pos, // position
	float* out_norm,	// normal
	float* out_len	// length
) {
	float d[3];
	dtVsub(d, c1, c0);
	const float dist_sqr = dtVdot2D(d, d);
	const float radius = r0 + r1;
	if (dist_sqr > radius* radius)
	{
		return false;
	}
	else 
	{
		float dist = 0.0f;
		if (dist_sqr > 0.0001f)
		{
			dist = sqrt(dist_sqr);
			dtVscale(out_norm, d, 1.0f / dist);
		}
		else
		{
			dtVset(out_norm, 1.0f, 0.0f, 0.0f);
		}
		
		dtVmad(out_pos, c1, out_norm, r1);
		*out_len = radius - dist;
		return true;
	}
}

void box_local_vertices_and_normals(const float* extent, float out_vertices[4][3], float out_normals[4][3])
{
	float halfExtent[3];

	dtVscale(halfExtent, extent, 0.5f);
	dtVset(out_vertices[0], -halfExtent[0], 0.0f, -halfExtent[2]);
	dtVset(out_vertices[1], halfExtent[0], 0.0f, -halfExtent[2]);
	dtVset(out_vertices[2], halfExtent[0], 0.0f, halfExtent[2]);
	dtVset(out_vertices[3], -halfExtent[0], 0.0f, halfExtent[2]);

	dtVset(out_normals[0], 0.0f, -1.0f, 0.0f);
	dtVset(out_normals[1], 1.0f, 0.0f, 0.0f);
	dtVset(out_normals[2], 0.0f, 1.0f, 0.0f);
	dtVset(out_normals[3], -1.0f, 0.0f, 0.0f);
}

bool dtConvexObstacle::LocalToWorldPosition(float* dest, const float* p) const
{
	dtVmad(dest, worldCenter, worldAxis[0], p[0]);
	dtVmad(dest, dest, worldAxis[1], p[1]);
	dtVmad(dest, dest, worldAxis[2], p[2]);
	return true;
}

bool dtConvexObstacle::WorldToLocalPosition(float* dest, const float* p) const
{
	float d[3];
	dtVsub(d, p, worldCenter);

	dest[0] = dtVdot(d, worldAxis[0]);
	dest[1] = dtVdot(d, worldAxis[1]);
	dest[2] = dtVdot(d, worldAxis[2]);
	return true;
}

bool dtConvexObstacle::LocalToWorldDirection(float* dest, const float* v) const
{
	dtVscale(dest, worldAxis[0], v[0]);
	dtVmad(dest, dest, worldAxis[1], v[1]);
	dtVmad(dest, dest, worldAxis[2], v[2]);
	dtVnormalize(dest);
	return true;
}

bool dtConvexObstacle::WorldToLocalDirection(float* dest, const float* v) const
{
	dest[0] = dtVdot(v, worldAxis[0]);
	dest[1] = dtVdot(v, worldAxis[1]);
	dest[2] = dtVdot(v, worldAxis[2]);
	dtVnormalize(dest);
	return true;
}

bool dtBoxObstacle::ContactEvaluateWithCircle(const float* c, const float r, const float h) const
{
	float min[3], max[3];
	dtVset(min, c[0] - r, c[1] - h / 2, c[2] - r);
	dtVset(max, c[0] + r, c[1] + h / 2, c[2] + r);

	return worldAabb[0][0] <= max[0]
		&& worldAabb[0][1] <= max[1]
		&& worldAabb[0][2] <= max[2]
		&& worldAabb[1][0] >= min[0]
		&& worldAabb[1][1] >= min[1]
		&& worldAabb[1][2] >= min[2];
}

bool dtBoxObstacle::ContactResultWithCircle(const float* c, const float r, dtContactInfo& contact) const
{
	float vertices[4][3], normals[4][3];
	box_local_vertices_and_normals(localExtent, vertices, normals);

	float d[3];
	dtVsub(d, c, worldCenter);

	float c_local[3];
	WorldToLocalPosition(c_local, c);

	int normal_index = 0;
	float min_separation = -FLT_MAX;
	float radius = r + 0.01f;

	int vertex_len = 4;
	for (int i = 0; i < vertex_len; ++i)
	{
		float d[3];
		dtVsub(d, c_local, vertices[i]);
		const float* n = normals[i];
		float s = dtVdot2D(n, d);

		if (s > radius)
		{
			// Early out.
			return false;
		}

		if (s > min_separation)
		{
			min_separation = s;
			normal_index = i;
		}
	}

	// Vertices that subtend the incident face.
	int vert_index1 = normal_index;
	int vert_index2 = (vert_index1 + 1 < vertex_len) ? vert_index1 + 1 : 0;
	const float* v1 = vertices[vert_index1];
	const float* v2 = vertices[vert_index2];

	// If the circle is past one of the two vertices, check it like
	// a circle-circle intersection where the vertex has radius 0
	if (min_separation > 0.0f)
	{
		float tv0[3], tv1[3];
		dtVsub(tv0, c_local, v1);
		dtVsub(tv1, v2, v1);
		const float u1 = dtVdot2D(tv0, tv1);

		dtVsub(tv0, c_local, v2);
		dtVsub(tv1, v1, v2);
		const float u2 = dtVdot2D(tv0, tv1);

		if (u1 <= 0.0f)
		{
			if (dtVdist2DSqr(c_local, v1) > radius * radius)
			{
				return false;
			}

			float world_v1[3];
			LocalToWorldPosition(world_v1, v1);

			if (intersection_circles(c, r, world_v1, 0.0f, contact.point, contact.normal, &contact.separation))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if (u2 <= 0.0f)
		{
			if (dtVdist2DSqr(c_local, v2) > radius * radius)
			{
				return false;
			}

			float world_v2[3];
			LocalToWorldPosition(world_v2, v2);

			if (intersection_circles(c, r, world_v2, 0.0f, contact.point, contact.normal, &contact.separation))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}

	LocalToWorldDirection(contact.normal, normals[normal_index]);
	dtVmad(contact.point, c, contact.normal, min_separation);
	contact.separation = r - min_separation;
	return true;
}

void dtBoxObstacle::Tick(float dt) const
{

}

int dtBoxObstacle::SegmentNum() const
{
	return 4;
}

void dtBoxObstacle::ForeachSegement(TCallback func) const
{
	float vertices[4][3], normals[4][3];
	box_local_vertices_and_normals(localExtent, vertices, normals);

	for (int i = 0; i < 4; ++i)
	{
		LocalToWorldPosition(vertices[i], vertices[i]);
	}

	for (int i = 0; i < 4; ++i)
	{
		int j = (i + 1) >= 4 ? 0 : i + 1;
		func(vertices[i], vertices[j]);
	}
}
