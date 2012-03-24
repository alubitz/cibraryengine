#include "StdAfx.h"
#include "MultiSphereShape.h"

#include "Physics.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

#include "Serialize.h"
#include "Util.h"

namespace CibraryEngine
{
	/*
	 * MultiSphereShape private implementation struct
	 */
	struct MultiSphereShape::Imp
	{
		struct Part
		{
			vector<Plane> planes;

			Part() : planes() { }
			virtual ~Part() { }

			bool IsPointRelevant(const Vec3& point)
			{
				for(vector<Plane>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
					if(iter->PointDistance(point) < 0)
						return false;
				return true;
			}

			virtual bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time) = 0;
		};

		struct SpherePart : Part
		{
			Sphere sphere;
			SpherePart(Sphere sphere) : Part(), sphere(sphere) { }

			bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time)
			{
				float t[2];

				if(Util::RaySphereIntersect(ray, sphere, t[0], t[1]))
				{
					for(unsigned char i = 0; i < 2; ++i)
					{
						Vec3 pos = ray.origin + ray.direction * t[i];
						if(t[i] >= 0.0f && t[i] <= 1.0f && IsPointRelevant(pos))
						{
							contact.pos = pos;
							contact.norm = Vec3::Normalize(pos - sphere.center);

							time = t[i];

							return true;
						}
					}
				}

				return false;
			}
		};

		struct TubePart : Part
		{
			Vec3 p1, p2;
			float r1, r2;

			Vec3 u;
			float inv_dmag;
			float cos_theta, sin_theta;

			float opzsq, trz;

			TubePart(Vec3 p1, Vec3 p2, float r1, float r2) : Part(), p1(p1), p2(p2), r1(r1), r2(r2)
			{
				Vec3 n = Vec3::Normalize(p2 - p1);

				planes.push_back(Plane(n, Vec3::Dot(n, p1)));
				planes.push_back(Plane(-n, -Vec3::Dot(n, p2)));
			}

			bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time)
			{
				Vec3 q = ray.origin - p1;
				Vec3 v = ray.direction;

				float vu = Vec3::Dot(ray.direction, u);
				float qu = Vec3::Dot(q, u);

				float A = v.ComputeMagnitudeSquared() - vu * vu * opzsq;
				float B = 2.0f * (Vec3::Dot(q, v) - qu * vu * opzsq) - trz * vu;
				float C = q.ComputeMagnitudeSquared() - qu * qu * opzsq - trz * qu - r1 * r1;

				float t[2];
				if(Util::SolveQuadraticFormula(A, B, C, t[0], t[1]))
				{
					for(unsigned char i = 0; i < 2; ++i)
					{
						Vec3 pos = ray.origin + ray.direction * t[i];
						if(t[i] >= 0.0f && t[i] <= 1.0f && IsPointRelevant(pos))
						{
							contact.pos = pos;

							Vec3 from_axis = pos - (p1 + u * Vec3::Dot(pos - p1, u));
							contact.norm = Vec3::Normalize(from_axis) * cos_theta + u * sin_theta;

							time = t[i];

							return true;
						}
					}
				}

				return false;
			}
		};

		struct PlanePart : Part
		{
			Plane plane;

			PlanePart(Plane plane) : Part(), plane(plane) { }

			bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time)
			{
				float t = Util::RayPlaneIntersect(ray, plane);
				Vec3 pos = ray.origin + ray.direction * t;

				if(t >= 0.0f && t <= 1.0f && IsPointRelevant(pos))
				{
					contact.pos = pos;
					contact.norm = plane.normal;

					time  = t;

					return true;
				}

				return false;
			}
		};

		vector<SpherePart> spheres;
		vector<TubePart> tubes;
		vector<PlanePart> planes;

		AABB aabb;

		Imp() : spheres(), tubes(), planes(), aabb() { }
		Imp(Vec3* centers, float* radii, unsigned int count) { Init(centers, radii, count); }

		~Imp() { }

		void Init(Vec3* centers, float* radii, unsigned int count)
		{
			// TODO: deal with unused spheres, duplicates, etc.

			spheres = vector<SpherePart>();
			tubes = vector<TubePart>();
			planes = vector<PlanePart>();

			if(count > 0)
			{
				for(unsigned int i = 0; i < count; ++i)
				{
					// TODO: detect and discard unworthy spheres

					Vec3 i_center = centers[i];
					float i_radius = radii[i];

					SpherePart sphere(Sphere(i_center, i_radius));
					spheres.push_back(sphere);

					if(i == 0)
						aabb = AABB(i_center, i_radius);
					else
						aabb.Expand(AABB(i_center, i_radius));
				}

				unsigned int num_spheres = spheres.size();
				for(unsigned int i = 0; i < num_spheres; ++i)
				{
					SpherePart& i_sphere = spheres[i];

					Vec3 i_center = i_sphere.sphere.center;
					float i_radius = i_sphere.sphere.radius;

					for(unsigned int j = i + 1; j < num_spheres; ++j)
					{
						// TODO: detect and discard unworthy tubes

						SpherePart& j_sphere = spheres[j];

						Vec3 j_center = j_sphere.sphere.center;
						float j_radius = j_sphere.sphere.radius;

						// finding a tube tangent to both spheres
						Vec3 d = j_center - i_center;
						float dmag = d.ComputeMagnitude(), inv_dmag = 1.0f / dmag;
						Vec3 ud = d * inv_dmag;

						// "theta" would be the angle from the axis vector to the point of tangency
						float cos_theta = (j_radius - i_radius) * inv_dmag, sin_theta = sqrtf(1.0f - cos_theta * cos_theta);

						Vec3 p1 = i_center + ud * (i_radius * cos_theta);					// center points of the circles where the tube touches the spheres
						Vec3 p2 = j_center + ud * (j_radius * cos_theta);
						float r1 = i_radius * sin_theta, r2 = j_radius * sin_theta;			// radii of those circles

						TubePart tube(p1, p2, r1, r2);

						// cache some parts of the ray-test formula
						tube.cos_theta = cos_theta;
						tube.sin_theta = sin_theta;
						tube.u = ud;
						tube.inv_dmag = 1.0f / (p2 - p1).ComputeMagnitude();				// careful! it's not the same as the inv_dmag used here!

						float z = (j_radius - i_radius) * inv_dmag;
						tube.opzsq = 1 + z * z;
						tube.trz = 2.0f * r1 * z;

						i_sphere.planes.push_back(Plane::Reverse(tube.planes[0]));
						j_sphere.planes.push_back(Plane::Reverse(tube.planes[1]));

						tubes.push_back(tube);
					}
				}

				unsigned int num_tubes = tubes.size();
				// TODO: maybe change how this is set up
				for(unsigned int i = 0; i < num_tubes; ++i)
				{
					TubePart& i_tube = tubes[i];

					for(unsigned int j = i + 1; j < num_tubes; ++j)
					{
						TubePart& j_tube = tubes[j];

						Vec3 n = Vec3::Normalize(Vec3::Cross(i_tube.u, j_tube.u));

						Vec3 abn = i_tube.sin_theta * i_tube.u - i_tube.cos_theta * n;			// point of tangency in plane of a, b, and n
						Vec3 acn = j_tube.sin_theta * j_tube.u - j_tube.cos_theta * n;			// point of tangency in plane of a, c, and n

						Vec3 plane_normal = Vec3::Normalize(Vec3::Cross(abn, acn));
						Vec3 other_plane_normal = plane_normal - n * (2.0f * Vec3::Dot(plane_normal, n));

						Plane plane = Plane::FromPositionNormal(i_tube.p1 + plane_normal * i_tube.r1, plane_normal);
						Plane other_plane = Plane::FromPositionNormal(i_tube.p1 + other_plane_normal * i_tube.r1, other_plane_normal);

						// TODO: create PlanePart, cut tubes, etc.
					}
				}
			}
			else
				aabb = AABB();
		}

		void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
		{
			static const Vec3 r(1, 0, 0), g(0, 1, 0), b(0, 0, 1), w(1, 1, 1);

			Mat3 rm = ori.ToMat3();

			Vec3 x = Vec3(rm[0], rm[1], rm[2]);
			Vec3 y = Vec3(rm[3], rm[4], rm[5]);
			Vec3 z = Vec3(rm[6], rm[7], rm[8]);

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				const Sphere& sphere = iter->sphere;
				
				float r = sphere.radius;
				Vec3 cen = sphere.center;
				
				Vec3 use_pos = pos + x * cen.x + y * cen.y + z * cen.z;
				Vec3 xr = x * r, yr = y * r, zr = z * r;

				renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(use_pos - xr, use_pos + xr, w), 1.0f));
				renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(use_pos - yr, use_pos + yr, w), 1.0f));
				renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(use_pos - zr, use_pos + zr, w), 1.0f));
			}
		}

		MassInfo ComputeMassInfo()
		{
			// TODO: implement this for real
			MassInfo temp;

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
				temp += MassInfo(iter->sphere.center, pow(iter->sphere.radius, 3.0f));

			return temp;
		}

		bool Contains(const Vec3& point)
		{
			// TODO: implement this for real

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				Vec3 i_cen = iter->sphere.center;
				Vec3 dif = point - i_cen;
				float i_radius = iter->sphere.radius;

				if(dif.ComputeMagnitudeSquared() < i_radius * i_radius)
					return true;
			}

			return false;
		}

		bool CollisionCheck(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody)
		{
			ContactPoint cp;

			cp.a.obj = ibody;
			cp.b.obj = jbody;

			float t;
			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
				if(iter->RayTest(ray, cp.b, t))
				{
					result = cp;
					time = t;

					return true;
				}

			for(vector<TubePart>::iterator iter = tubes.begin(); iter != tubes.end(); ++iter)
				if(iter->RayTest(ray, cp.b, t))
				{
					result = cp;
					time = t;

					return true;
				}

			for(vector<PlanePart>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
				if(iter->RayTest(ray, cp.b, t))
				{
					result = cp;
					time = t;

					return true;
				}

			return false;
		}

		bool CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return false;
		}

		bool CollisionCheck(const Mat4& my_xform, const Plane& plane, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: maybe fudge a bit to make face/edge collisions less chaotic?
			float min = 0.0f;

			Vec3 plane_norm = plane.normal;
			float plane_offset = plane.offset;

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				Vec3 sphere_pos = my_xform.TransformVec3(iter->sphere.center, 1.0f);
				float radius = iter->sphere.radius;

				float dist = Vec3::Dot(plane_norm, sphere_pos) - plane_offset - radius;

				if(dist < min)
				{
					min = dist;

					Vec3 a_pos = sphere_pos - plane_norm * radius;

					result = ContactPoint();
					result.a.obj = ibody;
					result.b.obj = jbody;
					result.a.pos = a_pos;
					result.b.pos = a_pos - plane_norm * (Vec3::Dot(a_pos, plane_norm) - plane_offset);
					result.b.norm = plane_norm;
					result.a.norm = -plane_norm;
				}
			}

			return min < 0.0f;
		}

		bool CollisionCheck(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return false;
		}

		void Write(ostream& stream)
		{
			WriteUInt32(spheres.size(), stream);
			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				WriteVec3(iter->sphere.center, stream);
				WriteSingle(iter->sphere.radius, stream);
			}
		}

		unsigned int Read(istream& stream)
		{
			unsigned int count = ReadUInt32(stream);

			if(count > 0)
			{
				Vec3* centers = new Vec3[count];
				float* radii = new float[count];

				for(unsigned int i = 0; i < count; ++i)
				{
					centers[i] = ReadVec3(stream);
					radii[i] = ReadSingle(stream);
				}

				Init(centers, radii, count);

				delete[] centers;
				delete[] radii;
			}

			return stream.fail() ? 1 : 0;
		}
	};




	/*
	 * MultiSphereShape methods
	 */
	MultiSphereShape::MultiSphereShape() : CollisionShape(ST_MultiSphere), imp(new Imp()) { }
	MultiSphereShape::MultiSphereShape(Vec3* centers, float* radii, unsigned int count) : CollisionShape(ST_MultiSphere), imp(new Imp(centers, radii, count)) { }

	void MultiSphereShape::InnerDispose() { delete imp; imp = NULL; }

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori) { imp->DebugDraw(renderer, pos, ori); }

	MassInfo MultiSphereShape::ComputeMassInfo() { return imp->ComputeMassInfo(); }

	bool MultiSphereShape::Contains(const Vec3& point) { return imp->Contains(point); }

	bool MultiSphereShape::CollisionCheck(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(ray, result, time, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& my_xform, const Plane& plane, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(my_xform, plane, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(sphere, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(xform, other, result, ibody, jbody); }

	void MultiSphereShape::Write(ostream& stream) { imp->Write(stream);}
	unsigned int MultiSphereShape::Read(istream& stream) { return imp->Read(stream); }	
}
