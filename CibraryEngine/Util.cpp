#include "StdAfx.h"
#include "Util.h"

#include "Vector.h"
#include "Matrix.h"
#include "Random3D.h"

#include "Plane.h"
#include "Sphere.h"

namespace CibraryEngine
{
	float Util::LeadTime(const Vec3& dx, const Vec3& dv, float muzzle_speed)
	{
		float xmag_sq = dx.ComputeMagnitudeSquared();
		float vmag_sq = dv.ComputeMagnitudeSquared();
		float qa = muzzle_speed * muzzle_speed - vmag_sq;
		float qb = 2.0f * Vec3::Dot(dx, dv);
		float qc = -xmag_sq;
		float urad = qb * qb - 4.0f * qa * qc;
		if(urad < 0.0f)
			return -1.0f;
		float root = sqrtf(urad);
		float min = -qb - root, max = -qb + root;
		float use = min > 0.0f ? min : max;
		use /= 2.0f * qa;
		return use;
	}

	Mat3 Util::FindOrientationZEdge(const Vec3& dir)
	{
		Vec3 dir_n = Vec3::Normalize(dir);
		while(true)
		{
			Vec3 up = Random3D::RandomNormalizedVector(1);
			Vec3 right = Vec3::Cross(up, dir_n);
			float magsq = right.ComputeMagnitudeSquared();

			if(magsq == 0)
				continue;

			right /= sqrtf(magsq);
			up = Vec3::Cross(dir_n, right);

			return Mat3(right.x, right.y, right.z, up.x, up.y, up.z, dir_n.x, dir_n.y, dir_n.z);
		}
	}

	float Util::TriangleMinimumDistance(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& x)
	{
		Vec3 ab = b - a, bc = c - b, ca = a - c;					// edge vectors
		Vec3 ax = x - a, bx = x - b, cx = x - c;					// vectors from verts to X
		Vec3 normal = Vec3::Normalize(Vec3::Cross(ab, bc));			// unit normal vector

		// squares of distances from verts; these distances guaranteed to be valid...
		// we will take the square root of the minimum instead of the minimum of the square roots!
		float dist_a_x_sq = ax.ComputeMagnitudeSquared();
		float dist_b_x_sq = bx.ComputeMagnitudeSquared();
		float dist_c_x_sq = cx.ComputeMagnitudeSquared();

		// minimum value so far encountered...
		float min_d = sqrtf(min(dist_a_x_sq, min(dist_b_x_sq, dist_c_x_sq)));

		// inverses of lengths of the edges
		float inv_len_ab = 1.0f / ab.ComputeMagnitude();
		float inv_len_bc = 1.0f / bc.ComputeMagnitude();
		float inv_len_ca = 1.0f / ca.ComputeMagnitude();

		// unit vectors perpendicular to each edge, lying in the plane of the triangle
		Vec3 n_ab = Vec3::Cross(ab, normal) * inv_len_ab;
		Vec3 n_bc = Vec3::Cross(bc, normal) * inv_len_bc;
		Vec3 n_ca = Vec3::Cross(ca, normal) * inv_len_ca;

		// finding the positions of X along each of the edges (necessary to determine if the edge distances are valid)
		float part_x_ab = Vec3::Dot(ax, ab) * inv_len_ab * inv_len_ab;
		float part_x_bc = Vec3::Dot(bx, bc) * inv_len_bc * inv_len_bc;
		float part_x_ca = Vec3::Dot(cx, ca) * inv_len_ca * inv_len_ca;

		// determining whether or not the edge distances are valid
		if(part_x_ab >= 0 && part_x_ab <= 1)
			min_d = min(min_d, Vec3::Cross(ab, ax).ComputeMagnitude() * inv_len_ab);
		if(part_x_bc >= 0 && part_x_bc <= 1)
			min_d = min(min_d, Vec3::Cross(bc, bx).ComputeMagnitude() * inv_len_bc);
		if(part_x_ca >= 0 && part_x_ca <= 1)
			min_d = min(min_d, Vec3::Cross(ca, cx).ComputeMagnitude() * inv_len_ca);

		// finding the distance from the plane; valid under the least frequently satisfied conditions
		float dot_n_ab_a = Vec3::Dot(n_ab, a);													// storing it because it's used twice in the expression... it'd be dumb to calculate twice
		if((Vec3::Dot(n_ab, x) - dot_n_ab_a) * (Vec3::Dot(n_ab, c) - dot_n_ab_a) > 0)			// if they're on the same side, this product is positive
		{
			float dot_n_bc_b = Vec3::Dot(n_bc, b);
			if((Vec3::Dot(n_bc, x) - dot_n_bc_b) * (Vec3::Dot(n_bc, a) - dot_n_bc_b) > 0)
			{
				double dot_n_ca_c = Vec3::Dot(n_ca, c);
				if((Vec3::Dot(n_ca, x) - dot_n_ca_c) * (Vec3::Dot(n_ca, b) - dot_n_ca_c) > 0)
				{
					// too bad it's so much harder to find out if it's valid than it is to calculate the value itself
					min_d = min(min_d, fabs(Vec3::Dot(normal, ax)));
				}
			}
		}

		return min_d;
	}

	float Util::RayPlaneIntersect(const Ray& ray, const Plane& plane)
	{
		float dir_dot = Vec3::Dot(ray.direction, plane.normal);
		float origin_dot = Vec3::Dot(ray.origin, plane.normal);
		return (plane.offset - origin_dot) / dir_dot;
	}

	bool Util::RaySphereIntersect(const Ray& ray, const Sphere& sphere, float& first, float& second)
	{
		Vec3 dx = ray.origin - sphere.center;

		float vmag_sq = ray.direction.ComputeMagnitudeSquared();
		float dmag_sq = dx.ComputeMagnitudeSquared();

		return SolveQuadraticFormula(vmag_sq, 2.0f * Vec3::Dot(dx, ray.direction), dmag_sq - sphere.radius * sphere.radius, first, second);
	}

	bool Util::SolveQuadraticFormula(float A, float B, float C, float& first, float& second)
	{
		float under_root = B * B - 4.0f * A * C;

		// no solutions
		if(under_root < 0 || A == 0)
			return false;

		// one or two solutions
		float root = sqrtf(under_root);
		float inv_a = 0.5f / A;

		// we want first to be the lesser of the two, regardless of the sign of A
		if(A > 0)
		{
			first = (-B - root) * inv_a;
			second = (-B + root) * inv_a;
		}
		else
		{
			first = (-B + root) * inv_a;
			second = (-B - root) * inv_a;
		}
		
		return true;
	}
}
