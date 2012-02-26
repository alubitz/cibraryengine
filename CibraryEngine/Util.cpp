#include "StdAfx.h"
#include "Util.h"

#include "Vector.h"
#include "Matrix.h"
#include "Random3D.h"

#include "Plane.h"
#include "Sphere.h"

namespace CibraryEngine
{
	float Util::LeadTime(Vec3 dx, Vec3 dv, float muzzle_speed)
	{
		float xmag_sq = dx.ComputeMagnitudeSquared();
		float vmag_sq = dv.ComputeMagnitudeSquared();
		float qa = muzzle_speed * muzzle_speed - vmag_sq;
		float qb = 2.0f * Vec3::Dot(dx, dv);
		float qc = -xmag_sq;
		float urad = qb * qb - 4.0f * qa * qc;
		if (urad < 0.0f)
			return -1.0f;
		float root = sqrt(urad);
		float min = -qb - root, max = -qb + root;
		float use = min > 0.0f ? min : max;
		use /= 2.0f * qa;
		return use;
	}

	Mat3 Util::FindOrientationZEdge(Vec3 dir)
	{
		Vec3 dir_n = Vec3::Normalize(dir);
		while (true)
		{
			Vec3 up = Random3D::RandomNormalizedVector(1);
			Vec3 right = Vec3::Cross(up, dir_n);
			float magsq = right.ComputeMagnitudeSquared();

			if (magsq == 0)
				continue;

			right /= sqrt(magsq);
			up = Vec3::Cross(dir_n, right);

			float values[] = { right.x, right.y, right.z, up.x, up.y, up.z, dir_n.x, dir_n.y, dir_n.z };
			return Mat3(values);
		}
	}


	vector<vector<Intersection> > Util::RayTriangleListIntersect(const vector<Vec3>& verts, const vector<unsigned int>& a_vert, const vector<unsigned int>& b_vert, const vector<unsigned int>& c_vert, const vector<Ray>& rays)
	{
		size_t num_rays = rays.size();
		size_t num_verts = verts.size();
		size_t num_faces = a_vert.size();

		Vec3* A = new Vec3[num_faces];
		Vec3* B = new Vec3[num_faces];
		Vec3* C = new Vec3[num_faces];
        Vec3* AB = new Vec3[num_faces];
		Vec3* AC = new Vec3[num_faces];
		Plane* planes = new Plane[num_faces];
        Vec3* P = new Vec3[num_faces];
		Vec3* Q = new Vec3[num_faces];
        float* UOffset = new float[num_faces];
		float* VOffset = new float[num_faces];
		for(size_t face_index = 0; face_index < num_faces; ++face_index)
		{
			A[face_index] = verts[a_vert[face_index]];
			B[face_index] = verts[b_vert[face_index]];
			C[face_index] = verts[c_vert[face_index]];
			AB[face_index] = B[face_index] - A[face_index];
			AC[face_index] = C[face_index] - A[face_index];


			Vec3 normal = Vec3::Normalize(Vec3::Cross(AB[face_index], AC[face_index]));
			float offset = Vec3::Dot(normal, A[face_index]);

			planes[face_index] = Plane(normal, offset);

			P[face_index] = Vec3::Cross(AC[face_index], normal);
			P[face_index] = P[face_index] / Vec3::Dot(P[face_index], AB[face_index]);
			Q[face_index] = Vec3::Cross(AB[face_index], normal);
			Q[face_index] = Q[face_index] / Vec3::Dot(Q[face_index], AC[face_index]);
			UOffset[face_index] = Vec3::Dot(P[face_index], A[face_index]);
			VOffset[face_index] = Vec3::Dot(Q[face_index], A[face_index]);
		}

		vector<vector<Intersection> > results;
		for(size_t ray_index = 0; ray_index < num_rays; ++ray_index)
		{
			Ray ray = rays[ray_index];
			vector<Intersection> test;
			for(size_t face_index = 0; face_index < num_faces; face_index++)
			{
				Plane& plane = planes[face_index];
				float hit = Util::RayPlaneIntersect(ray, plane);

				// TODO: maybe put (optional) conditions here to cull 'backward' items, etc.
				if(hit > 0 || hit <= 0)						// for now just check that it's a real number
				{
					Vec3 pos = ray.origin + ray.direction * hit;
					float u = Vec3::Dot(P[face_index], pos) - UOffset[face_index];
					float v = Vec3::Dot(Q[face_index], pos) - VOffset[face_index];
					if(u >= 0 && v >= 0 && u + v <= 1)
					{
						Intersection intersection;
						intersection.face = face_index;
						intersection.i = u;
						intersection.j = v;
						intersection.position = pos;
						intersection.time = hit;
						intersection.normal = plane.normal;
						intersection.ray = ray;
						test.push_back(intersection);
					}
				}
			}
			results.push_back(test);
		}

		return results;
	}

	float Util::TriangleMinimumDistance(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& x)
	{
		Vec3 ab = b - a, bc = c - b, ca = a - c;					// edge vectors
		Vec3 ax = x - a, bx = x - b, cx = x - c;					// vectors from verts to X
		Vec3 normal = Vec3::Normalize(Vec3::Cross(ab, bc));			// unit normal vector

		// distances from verts; guaranteed to be valid
		float dist_a_x = ax.ComputeMagnitude();
		float dist_b_x = bx.ComputeMagnitude();
		float dist_c_x = cx.ComputeMagnitude();

		// minimum value so far encountered
		float min_d = min(dist_a_x, min(dist_b_x, dist_c_x));

		// lengths of the edges
		float len_ab = ab.ComputeMagnitude();
		float len_bc = bc.ComputeMagnitude();
		float len_ca = ca.ComputeMagnitude();

		// unit vectors perpendicular to each edge, lying in the plane of the triangle
		Vec3 n_ab = Vec3::Cross(ab, normal) / len_ab;
		Vec3 n_bc = Vec3::Cross(bc, normal) / len_bc;
		Vec3 n_ca = Vec3::Cross(ca, normal) / len_ca;

		// finding the positions of X along each of the edges (necessary to determine if the edge distances are valid)
		float part_x_ab = Vec3::Dot(ax, ab) / (len_ab * len_ab);
		float part_x_bc = Vec3::Dot(bx, bc) / (len_bc * len_bc);
		float part_x_ca = Vec3::Dot(cx, ca) / (len_ca * len_ca);

		// determining whether or not the edge distances are valid
		if (part_x_ab >= 0 && part_x_ab <= 1)
			min_d = min(min_d, Vec3::Cross(ab, ax).ComputeMagnitude() / len_ab);
		if (part_x_bc >= 0 && part_x_bc <= 1)
			min_d = min(min_d, Vec3::Cross(bc, bx).ComputeMagnitude() / len_bc);
		if (part_x_ca >= 0 && part_x_ca <= 1)
			min_d = min(min_d, Vec3::Cross(ca, cx).ComputeMagnitude() / len_ca);

		// finding the distance from the plane; valid under the least frequently satisfied conditions
		float dot_n_ab_a = Vec3::Dot(n_ab, a);													// storing it because it's used twice in the expression... it'd be dumb to calculate twice
		if ((Vec3::Dot(n_ab, x) - dot_n_ab_a) * (Vec3::Dot(n_ab, c) - dot_n_ab_a) > 0)			// if they're on the same side, this product is positive
		{
			float dot_n_bc_b = Vec3::Dot(n_bc, b);
			if ((Vec3::Dot(n_bc, x) - dot_n_bc_b) * (Vec3::Dot(n_bc, a) - dot_n_bc_b) > 0)
			{
				double dot_n_ca_c = Vec3::Dot(n_ca, c);
				if ((Vec3::Dot(n_ca, x) - dot_n_ca_c) * (Vec3::Dot(n_ca, b) - dot_n_ca_c) > 0)
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

		// quadratic formula here...
        float A = vmag_sq;
		float B = 2.0 * Vec3::Dot(dx, ray.direction);
		float C = dmag_sq - sphere.radius * sphere.radius;
        
		float under_root = B * B - 4.0 * A * C;

		// no solutions
        if (under_root < 0 || A == 0)
            return false;
        
		// one or two solutions
		float root = sqrtf(under_root);
		float inv_a = 0.5f / A;
        first = (-B - root) * inv_a;
		second = (-B + root) * inv_a;
		
		return true;

	}
}
