#include "StdAfx.h"
#include "Sphere.h"

namespace CibraryEngine
{
	Sphere::Sphere() : center(), radius(0.0f) { }
	Sphere::Sphere(Vec3 center_, float radius_) : center(center_), radius(radius_) { }

	void Sphere::Expand(Vec3 b)
	{
		Vec3 dif = b - center;
		float distance = dif.ComputeMagnitude();

		// degenerate cases... one sphere completely inside the other
		if (radius > distance)
			return;

		// otherwise some actual work must be done... not too bad though
		float oldradius = radius;
		radius = (distance + radius) * 0.5f;
		center += Vec3::Normalize(dif, radius - oldradius);
	}

	Sphere Sphere::Expand(Sphere a, Sphere b)
	{
		Vec3 dif = b.center - a.center;
		float distance = dif.ComputeMagnitude();

		// degenerate cases... one sphere completely inside the other
		if (a.radius > distance + b.radius)
			return Sphere(a.center, a.radius);
		if (b.radius > distance + a.radius)
			return Sphere(b.center, b.radius);

		// otherwise some actual work must be done... not too bad though
		float radius = (distance + a.radius + b.radius) * 0.5f;
		Vec3 center = a.center + Vec3::Normalize(dif, radius - a.radius);

		return Sphere(center, radius);
	}

	bool Sphere::ContainsPoint(const Vec3& point) const { return (point - center).ComputeMagnitudeSquared() <= radius * radius; }

	Sphere Sphere::Expand(Sphere a, Vec3 b)
	{
		Vec3 dif = b - a.center;
		float distance = dif.ComputeMagnitude();

		// degenerate cases... one sphere completely inside the other
		if (a.radius > distance)
			return Sphere(a.center, a.radius);

		// otherwise some actual work must be done... not too bad though
		float radius = (distance + a.radius) * 0.5f;
		Vec3 center = a.center + Vec3::Normalize(dif, radius - a.radius);

		return Sphere(center, radius);
	}

	bool Sphere::IntersectTest(Sphere a, Sphere b)
	{
		Vec3 dist = b.center - a.center;
		float radius = a.radius + b.radius;

		return (dist.ComputeMagnitudeSquared() < radius * radius);
	}

	bool Sphere::OcclusionTest(Sphere occluding, Sphere hidden, Vec3 viewer)
	{
		Vec3 to_occluding = occluding.center - viewer;
		Vec3 to_hidden = hidden.center - viewer;
		float occluding_mag = to_occluding.ComputeMagnitude(), hidden_mag = to_hidden.ComputeMagnitude();
		float occluding_subtended_angle = asin(occluding.radius / occluding_mag);
		float hidden_subtended_angle = asin(hidden.radius / hidden_mag);
		if (occluding_subtended_angle < hidden_subtended_angle)
			return false;
		else
			return (acos(Vec3::Dot(to_occluding, to_hidden) / (occluding_mag * hidden_mag))) < occluding_subtended_angle - hidden_subtended_angle;
	}
}
