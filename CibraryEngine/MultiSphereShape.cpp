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
						float ti = t[i];

						Vec3 pos = ray.origin + ray.direction * ti;
						if(ti >= 0.0f && ti <= 1.0f)
						{
							if(IsPointRelevant(pos))
							{
								contact.pos = pos;
								contact.norm = Vec3::Normalize(pos - sphere.center);

								time = ti;

								return true;
							}
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
				// creating spheres
				vector<Sphere> spheres_temp;
				for(unsigned int i = 0; i < count; ++i)
				{
					Vec3 i_center = centers[i];
					float i_radius = radii[i];

					spheres_temp.push_back(Sphere(i_center, i_radius));

					if(i == 0)
						aabb = AABB(i_center, i_radius);
					else
						aabb.Expand(AABB(i_center, i_radius));
				}

				// add those spheres to the list, eliminating any that are completely contained inside another sphere
				for(unsigned int i = 0; i < count; ++i)
				{
					const Sphere& a = spheres_temp[i];
					
					bool ok = true;
					for(unsigned int j = 0; j < count; ++j)
					{					
						const Sphere& b = spheres_temp[j];
						if(a.radius < b.radius)
						{
							float dr = a.radius - b.radius;
							if((a.center - b.center).ComputeMagnitudeSquared() < dr * dr)
							{
								ok = false;
								break;
							}
						}
					}
					if(ok)
						spheres.push_back(SpherePart(a));
				}

				unsigned int num_spheres = spheres.size();					// may differ from count, if spheres got discarded
				unsigned int n_sq = num_spheres * num_spheres;
				unsigned int n_cubed = n_sq * num_spheres;

				// creating tubes
				TubePart** sphere_tubes = new TubePart* [n_sq];
				memset(sphere_tubes, 0, sizeof(TubePart*) * n_sq);

				for(unsigned int i = 0; i < num_spheres; ++i)
				{
					SpherePart& i_sphere = spheres[i];
					unsigned int i_n = i * num_spheres;

					Vec3 i_center = i_sphere.sphere.center;
					float i_radius = i_sphere.sphere.radius;

					for(unsigned int j = i + 1; j < num_spheres; ++j)
					{
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

						//i_sphere.planes.push_back(Plane::Reverse(tube.planes[0]));
						//j_sphere.planes.push_back(Plane::Reverse(tube.planes[1]));

						sphere_tubes[i_n + j] = new TubePart(tube);
					}
				}

				// creating planes
				PlanePart** sphere_planes = new PlanePart* [n_cubed * 2];
				memset(sphere_planes, 0, sizeof(PlanePart*) * n_cubed * 2);

				for(unsigned int i = 0; i < num_spheres; ++i)
				{
					unsigned int i_n = i * num_spheres;
					SpherePart& i_sphere = spheres[i];									// looking to make planes from this sphere...

					for(unsigned int j = i + 1; j < num_spheres; ++j)
					{
						if(TubePart* ij_tubep = sphere_tubes[i_n + j])
						{
							TubePart& ij_tube = *ij_tubep;								// found one tube connected to this sphere

							unsigned int j_n = j * num_spheres;
							SpherePart& j_sphere = spheres[j];

							for(unsigned int k = j + 1; k < num_spheres; ++k)
							{
								if(TubePart* ik_tubep = sphere_tubes[i_n + k])
								{
									TubePart& ik_tube = *ik_tubep;						// found a second tube; make planes between them
									SpherePart& k_sphere = spheres[k];

									TubePart* jk_tubep = sphere_tubes[j_n + k];

									Vec3 n = Vec3::Normalize(Vec3::Cross(ij_tube.u, ik_tube.u));

									Vec3 abn = ij_tube.sin_theta * ij_tube.u - ij_tube.cos_theta * n;			// point of tangency in plane of a, b, and n
									Vec3 acn = ik_tube.sin_theta * ik_tube.u - ik_tube.cos_theta * n;			// point of tangency in plane of a, c, and n

									Vec3 normal_1 = Vec3::Normalize(Vec3::Cross(abn, acn));						// normal vectors to first plane
									Vec3 normal_2 = normal_1 - n * (2.0f * Vec3::Dot(normal_1, n));				// reflect normal vector across the plane containing the centerpoints

									PlanePart plane_parts[] =
									{
										PlanePart(Plane::FromPositionNormal(ij_tube.p1 + normal_1 * ij_tube.r1, normal_1)),
										PlanePart(Plane::FromPositionNormal(ij_tube.p1 + normal_2 * ij_tube.r1, normal_2))
									};

									for(unsigned char m = 0; m < 2; ++m)
									{
										PlanePart& pp = plane_parts[m];

										pp.planes.push_back(Plane::FromTriangleVertices(i_sphere.sphere.center, j_sphere.sphere.center, i_sphere.sphere.center + pp.plane.normal));
										pp.planes.push_back(Plane::FromTriangleVertices(j_sphere.sphere.center, k_sphere.sphere.center, j_sphere.sphere.center + pp.plane.normal));
										pp.planes.push_back(Plane::FromTriangleVertices(k_sphere.sphere.center, i_sphere.sphere.center, k_sphere.sphere.center + pp.plane.normal));

										//ij_tube.planes.push_back(Plane::Reverse(pp.planes[0]));
										//if(jk_tubep != NULL)
										//	jk_tubep->planes.push_back(Plane::Reverse(pp.planes[1]));			// is there a third tube to form a triangle?
										//ik_tube.planes.push_back(pp.planes[2]);								// not reversed because the tube is oriented the opposite direction

										sphere_planes[(i * n_sq + j * num_spheres + k) * 2 + m] = new PlanePart(pp);
									}
								}
							}
						}
					}
				}

				//stringstream ss;
				for(unsigned int i = 0; i < num_spheres; ++i)
				{
					// TODO: detect and discard unworthy spheres

					const SpherePart& part = spheres[i];
					const Sphere& sphere = part.sphere;
					//ss << "Sphere, center = (" << sphere.center.x << ", " << sphere.center.y << ", " << sphere.center.z << "), radius = " << sphere.radius << endl;
					//for(unsigned int j = 0; j < part.planes.size(); ++j)
					//{
					//	const Plane& plane = part.planes[j];
					//	ss << "\tPlane, normal = (" << plane.normal.x << ", " << plane.normal.y << ", " << plane.normal.z << "), offset = " << plane.offset << endl;
					//}
				}

				for(unsigned int i = 0; i < n_sq; ++i)
				{
					if(TubePart* tube_p = sphere_tubes[i])
					{
						TubePart part(*tube_p);

						bool worthy = true;
						// TODO: detect and discard unworthy tubes

						if(worthy)
						{
							tubes.push_back(part);

							// TODO: add cutting planes to the spheres this tube connects

							//ss << "Tube, p1 = (" << part.p1.x << ", " << part.p1.y << ", " << part.p1.z << "), p2 = " << part.p2.x << ", " << part.p2.y << ", " << part.p2.z << "), r1 = " << part.r1 << ", r2 = " << part.r2 << endl;
							//for(unsigned int j = 0; j < part.planes.size(); ++j)
							//{
							//	const Plane& plane = part.planes[j];
							//	ss << "\tPlane, normal = (" << plane.normal.x << ", " << plane.normal.y << ", " << plane.normal.z << "), offset = " << plane.offset << endl;
							//}
						}

						delete tube_p;
					}
				}
				delete[] sphere_tubes;

				for(unsigned int i = 0; i < n_cubed * 2; ++i)
				{
					if(PlanePart* plane_p = sphere_planes[i])
					{
						PlanePart part(*plane_p);

						bool worthy = true;

						// if anything sticks out farther than this plane, this plane is unworthy
						float my_dist = part.plane.offset;
						const Vec3& normal = part.plane.normal;
						for(unsigned int j = 0; j < num_spheres; ++j)
						{
							const Sphere& sphere = spheres[j].sphere;
							float dist = Vec3::Dot(normal, sphere.center) + sphere.radius;
							if(dist > my_dist + 0.0000001f)
							{
								worthy = false;
								break;
							}
						}

						if(worthy)
						{
							// TODO: merge duplicate planes
						}

						if(worthy)
						{
							planes.push_back(part);

							// TODO: add cutting planes from worthy planes to the tubes this plane connects

							//ss << "Plane, normal = (" << part.plane.normal.x << ", " << part.plane.normal.y << ", " << part.plane.normal.z << "), offset = " << part.plane.offset << endl;
							//for(unsigned int j = 0; j < part.planes.size(); ++j)
							//{
							//	const Plane& plane = part.planes[j];
							//	ss << "\tPlane, normal = (" << plane.normal.x << ", " << plane.normal.y << ", " << plane.normal.z << "), offset = " << plane.offset << endl;
							//}
						}

						delete plane_p;
					}
				}
						
				delete[] sphere_planes;

				//Debug(ss.str());
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
			Vec3 a = ray.origin, b = ray.origin + ray.direction;
			AABB ray_aabb(Vec3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)), Vec3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)));

			if(!AABB::IntersectTest(ray_aabb, aabb))
				return false;

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
