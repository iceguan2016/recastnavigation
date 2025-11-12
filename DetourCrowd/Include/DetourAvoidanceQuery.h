#ifndef DETOURAVOIDANCEQUERY_H
#define DETOURAVOIDANCEQUERY_H

#include "DetourCommon.h"

#include <list>

class dtAvoidanceUtils
{
public:
	static const float eplision;
	static const float zero[3];

	static bool isZeroVec(const float* v)
	{
		return !(dtAbs(v[0]) > eplision ||
			dtAbs(v[1]) > eplision ||
			dtAbs(v[2]) > eplision);
	}

	static int isectRaySeg(const float* ap, const float* u,
		const float* bp, const float* bq,
		float& t);
};

class dtEdgeHandle
{
public:
	static dtEdgeHandle INVALID;

	dtEdgeHandle()
	{
		dtVset(points[0], 0.0f, 0.0f, 0.0f);
		dtVset(points[1], 0.0f, 0.0f, 0.0f);
	}

	dtEdgeHandle(const float* start, const float* end)
	{
		dtVcopy(points[0], start);
		dtVcopy(points[1], end);
	}

	bool operator==(const dtEdgeHandle& other)
	{
		static const float eplision = 0.000001f;

		float d1 = dtVdistSqr(points[0], other.points[0]);
		float d2 = dtVdistSqr(points[1], other.points[1]);

		return d1 < eplision && d2 < eplision;
	}

private:
	float points[2][3];
};

template<typename TObstacleHandle>
class dtVO
{
public:
	static const int edgeLeftIndex = 0;
	static const int edgeRightIndex = 1;

	struct VOEdge
	{
		// Edge标识
		TObstacleHandle _handle;
		// Edge方向
		float _direction[3];
		// 角度[-Pi~Pi)
		float _angle;
		//
		float _distSq;

		VOEdge()
		{
			dtVset(_direction, 0.0f, 0.0f, 0.0f);
			_angle = 0.0f;
			_distSq = 0.0f;
		}

		VOEdge(const TObstacleHandle& handle, const float* dir, float distSq)
		{
			setEdge(handle, dir, distSq);
		}

		void setEdge(const TObstacleHandle& handle, const float* dir, float distSq)
		{
			_handle = handle;
			dtVcopy(_direction, dir);
			if (dtAbs(dir[0]) > dtAvoidanceUtils::eplision ||
				dtAbs(dir[2]) > dtAvoidanceUtils::eplision)
			{
				_angle = atan2f(dir[2], dir[0]); // (-PI, PI)
				// while (Angle < 0) Angle += FixMath.F64.Pi2;
				// while (Angle > FixMath.F64.Pi2) Angle -= FixMath.F64.Pi2;
			}
			else
			{
				_angle = 0.0f;
			}
			_distSq = distSq;
		}

		void setEdge(const TObstacleHandle& handle, const float* dir, float angle, float distSq)
		{
			_handle = handle;
			dtVcopy(_direction, dir);
			_angle = angle;
			_distSq = distSq;
		}
	};

	VOEdge _edges[2] = {
		{ TObstacleHandle::INVALID, dtAvoidanceUtils::zero, 0.0f },
		{ TObstacleHandle::INVALID, dtAvoidanceUtils::zero, 0.0f }
	};

	bool contains(const float angle)
	{
		const auto& left = _edges[edgeLeftIndex]._angle;
		const auto& right = _edges[edgeRightIndex]._angle;
		// 注意：这里left - right的范围表示从left顺时针转到right所扫过的范围
		// 比如当left为-170度(-2.96弧度)，right为-20度(-0.34弧度)就会出现left < right情况，需要分开处理
		return left >= right ? (left >= angle && angle >= right) : (angle <= left || angle >= right);
	}

	bool tryMergeWith(const dtVO& other)
	{
		auto& left = _edges[edgeLeftIndex];
		auto& right = _edges[edgeRightIndex];
		const auto& otherLeft = other._edges[edgeLeftIndex];
		const auto& otherRight = other._edges[edgeRightIndex];

		bool leftContain = contains(otherLeft._angle);
		bool rightContain = contains(otherRight._angle); /* attention */

		if (leftContain && rightContain)
		{
			return true;
		}

		if (leftContain)
		{
			right.setEdge(otherRight._handle, otherRight._direction, otherRight._angle, otherRight._distSq);
		}

		if (rightContain)
		{
			left.setEdge(otherLeft._handle, otherLeft._direction, otherLeft._angle, otherLeft._distSq);
		}

		return leftContain || rightContain;
	}

	const VOEdge& left() const { return _edges[edgeLeftIndex]; }
	const VOEdge& right() const { return _edges[edgeRightIndex]; }
};

template<typename TObstacleHandle>
class dtAvoidExtraInfo
{
public:
	enum EAvoidSide
	{
		NONE = 0,
		LEFT,
		RIGHT
	};

	// 最新更新时间
	float _time;
	// 当前避让的Entity
	TObstacleHandle _handle;
	// 当前选择的避让Obstacle的VO方向
	EAvoidSide _side;

	bool isValid() const 
	{
		return _handle.isValid() && _side != EAvoidSide::NONE;
	}

	void setAvoidInfo(float time, const TObstacleHandle& handle, EAvoidSide side)
	{
		_time = time;
		_handle = handle;
		_side = side;
	}

	void reset()
	{
		_time = 0;
		_handle = TObstacleHandle::INVALID;
		_side = EAvoidSide::NONE;
	}
};

template<typename TObstacleHandle>
class dtDetourAvoidanceQuery
{
public:
	using TVO = dtVO<TObstacleHandle>;
	using TAvoidExtraInfo = dtAvoidExtraInfo<TObstacleHandle>;
	using TVOList = std::list<TVO>;

	void init(const float* pos, const float radius, const float* dvel, const float* nvel, const float timeHorizon);
	bool addSegment(const TObstacleHandle& nei, const float* start, const float* end);
	bool queryAvoidDirection(float time, const float* nvel, TAvoidExtraInfo& info, float* outDir);
	const TVOList& vos() const { return _vos; }

private:
	float _pos[3];
	float _radius;
	float _vel[2][3];
	float _timeHorizon;
	TVOList _vos;
};

template<typename TObstacleHandle>
void dtDetourAvoidanceQuery<TObstacleHandle>::init(const float* pos, const float radius, const float* dvel, const float* nvel, const float timeHorizon)
{
	dtVcopy(_pos, pos);
	_radius = radius;
	dtVcopy(_vel[0], dvel);
	dtVcopy(_vel[1], nvel);
	_timeHorizon = timeHorizon;
	_vos.clear();
}

template<typename TObstacleHandle>
bool dtDetourAvoidanceQuery<TObstacleHandle>::addSegment(const TObstacleHandle& nei, const float* p, const float* q)
{
	// check will collision?
	// 
	// Precalc if the agent is really close to the segment.
	const float r = 0.01f;
	float t;
	bool touch = dtDistancePtSegSqr2D(_pos, p, q, t) < dtSqr(r);

	float htmin = 0;

	if (touch)
	{
		// Special case when the agent is very close to the segment.
		float sdir[3], snorm[3];
		dtVsub(sdir, q, p);
		snorm[0] = -sdir[2];
		snorm[2] = sdir[0];
		// If the velocity is pointing towards the segment, no collision.
		if (dtVdot2D(snorm, _vel[0]) < 0.0f &&
			dtVdot2D(snorm, _vel[1]) < 0.0f)
			return false;
		// Else immediate collision.
		htmin = 0.0f;
	}
	else
	{
		float htmin1 = 0, htmin2 = 0; 
		if (!dtAvoidanceUtils::isectRaySeg(_pos, _vel[0], p, q, htmin1) &&
			!dtAvoidanceUtils::isectRaySeg(_pos, _vel[1], p, q, htmin2))
			return false;

		htmin = dtMin(htmin1, htmin2);
	}

	// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
	if (htmin < _timeHorizon)
	{
		float nt;
		float distSq = dtDistancePtSegSqr2D(_pos, p, q, nt);

		float left[3], right[3];
		dtVsub(left, p, _pos);
		left[1] = 0.0f;
		if (dtVlenSqr(left) > 0.0f)
			dtVnormalize(left);

		dtVsub(right, q, _pos);
		right[1] = 0.0f;
		if (dtVlenSqr(right) > 0.0f)
			dtVnormalize(right);

		TVO tmpVO;
		tmpVO._edges[TVO::edgeLeftIndex].setEdge(nei, left, distSq);
		tmpVO._edges[TVO::edgeRightIndex].setEdge(nei, right, distSq);
		bool addTempVO = true;
		for (auto it = _vos.begin(); it != _vos.end(); )
		{
			if (tmpVO.tryMergeWith(*it))
			{
				it = _vos.erase(it);
			}
			else if (it->tryMergeWith(tmpVO))
			{
				addTempVO = false;
				break;
			}
			else
			{
				++it;
			}
		}
		if (addTempVO) 
			_vos.push_back(tmpVO);

		return true;
	}

	return false;
}

template<typename TObstacleHandle>
bool dtDetourAvoidanceQuery<TObstacleHandle>::queryAvoidDirection(float time, const float* nvel, TAvoidExtraInfo& info, float* dir)
{
	using EAvoidSide = typename TAvoidExtraInfo::EAvoidSide;

	TObstacleHandle handle = TObstacleHandle::INVALID;

	// choose best direction
	const int num = TVO::edgeRightIndex + 1;
	float bestDirs[num][3] = {0.0f};
	for (int i = 0; i < num; ++i)
	{
		dtVset(bestDirs[i], 0.0f, 0.0f, 0.0f);
	}

	for (int s = TVO::edgeLeftIndex; s <= TVO::edgeRightIndex; ++s)
	{
		float minDistSq = FLT_MAX;
		for (auto it = _vos.begin(); it != _vos.end(); ++it)
		{
			const auto& edge = it->_edges[s];

			// 1.检查是否碰撞obstacle
			// 2.考虑方向最接近的
			float diff[3];
			dtVsub(diff, edge._direction, nvel);
			float diffSq = dtVlenSqr(diff);
			if (diffSq < minDistSq)
			{
				handle = edge._handle;
				dtVcopy(bestDirs[s], edge._direction);
				minDistSq = diffSq;
			}
		}
	}

	EAvoidSide bestSide = EAvoidSide::NONE;
	float bestDir[3] = { 0.0f };
	int startIndex = info._side != EAvoidSide::RIGHT ? TVO::edgeLeftIndex : TVO::edgeRightIndex;
	int indexCount = TVO::edgeRightIndex + 1;
	for (int index = 0; index < indexCount; ++index)
	{
		int side = (startIndex + index) % indexCount;
		const float* currDir = bestDirs[side];
		if (!dtAvoidanceUtils::isZeroVec(currDir))
		{
			bestSide = side == TVO::edgeLeftIndex ? EAvoidSide::LEFT : EAvoidSide::RIGHT;
			dtVcopy(bestDir, currDir);
			break;
		}
	}

	if (bestSide != EAvoidSide::NONE)
	{
		info.setAvoidInfo(time, handle, bestSide);
		dtVcopy(dir, bestDir);
		return true;
	}
	return false;
}

#endif//DETOURAVOIDANCEQUERY_H