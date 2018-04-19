#include "StdAfx.h"
#include "MultiSphereShape.h"

#include "Physics.h"
#include "ContactPoint.h"
#include "ContactDataCollector.h"
#include "RigidBody.h"
#include "RayCollider.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "TriangleMeshShape.h"

#include "SceneRenderer.h"
#include "CameraView.h"
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
		// members and member struct definitions
		struct Part
		{
			vector<Plane> planes;

			Part() : planes() { }
			virtual ~Part() { }

			bool IsPointRelevant(const Vec3& point) const
			{
				for(vector<Plane>::const_iterator iter = planes.begin(), planes_end = planes.end(); iter != planes_end; ++iter)
					if(iter->PointDistance(point) < 0)
						return false;
				return true;
			}

			virtual int RayTest(const Ray& ray) const = 0;
			virtual bool RayTest(const Ray& ray, RayResult& rr) const = 0;
		};

		struct SpherePart : Part
		{
			Sphere sphere;
			SpherePart(const Sphere& sphere) : Part(), sphere(sphere) { }

			int RayTest(const Ray& ray) const
			{
				int result = 0;
				float t[2];

				if(Util::RaySphereIntersect(ray, sphere, t[0], t[1]))
				{
					for(unsigned char i = 0; i < 2; ++i)
					{
						float ti = t[i];
						Vec3 pos = ray.origin + ray.direction * ti;
						if(ti >= 0.0f && ti <= 1.0f)
							if(IsPointRelevant(pos))
								++result;
					}
				}

				return result;
			}
			bool RayTest(const Ray& ray, RayResult& rr) const
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
								rr.pos = pos;
								rr.norm = Vec3::Normalize(pos - sphere.center);

								rr.t = ti;

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

			TubePart(const Vec3& p1, const Vec3& p2, float r1, float r2) : Part(), p1(p1), p2(p2), r1(r1), r2(r2) { }

			bool RayTestInfinite(const Ray& ray, float& t1, float& t2) const
			{
				Vec3 q = ray.origin - p1;
				Vec3 v = ray.direction;

				float vu = Vec3::Dot(ray.direction, u);
				float qu = Vec3::Dot(q, u);

				float A = v.ComputeMagnitudeSquared() - vu * vu * opzsq;
				float B = 2.0f * (Vec3::Dot(q, v) - qu * vu * opzsq) - trz * vu;
				float C = q.ComputeMagnitudeSquared() - qu * qu * opzsq - trz * qu - r1 * r1;

				return Util::SolveQuadraticFormula(A, B, C, t1, t2);
			}
			int RayTest(const Ray& ray) const
			{
				int result = 0;
				float t[2];
				if(RayTestInfinite(ray, t[0], t[1]))
				{
					for(unsigned char i = 0; i < 2; ++i)
					{
						Vec3 pos = ray.origin + ray.direction * t[i];
						if(t[i] >= 0.0f && t[i] <= 1.0f && IsPointRelevant(pos))
							++result;
					}
				}

				return result;
			}
			bool RayTest(const Ray& ray, RayResult& rr) const
			{
				float t[2];
				if(RayTestInfinite(ray, t[0], t[1]))
				{
					for(unsigned char i = 0; i < 2; ++i)
					{
						Vec3 pos = ray.origin + ray.direction * t[i];
						if(t[i] >= 0.0f && t[i] <= 1.0f && IsPointRelevant(pos))
						{
							rr.pos = pos;

							Vec3 from_axis = pos - (p1 + u * Vec3::Dot(pos - p1, u));
							rr.norm = Vec3::Normalize(from_axis) * sin_theta + u * cos_theta;

							rr.t = t[i];

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

			PlanePart(const Plane& plane) : Part(), plane(plane) { }

			int RayTest(const Ray& ray) const
			{
				int result = 0;
				float t = Util::RayPlaneIntersect(ray, plane);
				Vec3 pos = ray.origin + ray.direction * t;

				if(IsPointRelevant(pos))
					if(t >= 0.0f && t <= 1.0f)
						++result;
				return result;
			}
			bool RayTest(const Ray& ray, RayResult& rr) const
			{
				float t = Util::RayPlaneIntersect(ray, plane);
				Vec3 pos = ray.origin + ray.direction * t;

				if(IsPointRelevant(pos))
					if(t >= 0.0f && t <= 1.0f)
					{
						rr.pos = pos;
						rr.norm = plane.normal;

						rr.t  = t;

						return true;
					}

				return false;
			}

			vector<Vec3> GetVerts() const
			{
				vector<Vec3> result;

				for(unsigned int i = 0; i < planes.size(); ++i)
					for(unsigned int j = i + 1; j < planes.size(); ++j)
					{
						Line line;
						Plane::Intersect(planes[i], planes[j], line);

						float d = Util::RayPlaneIntersect(Ray(line), plane);
						if(_finite(d))			// TODO: come up with a solution that doesn't depend on msvc
						{
							Vec3 vert = line.origin + line.direction * d;

							result.push_back(vert);
						}
					}

				return result;
			}
		};

		vector<SpherePart> spheres;
		vector<TubePart> tubes;
		vector<PlanePart> planes;

		AABB aabb;


		// constructors, destructors, etc.
		Imp() : spheres(), tubes(), planes(), aabb() { }
		Imp(Sphere* spheres, unsigned int count) { Init(spheres, count); }

		~Imp() { }

		TubePart MakeTube(const Sphere& i_sphere, const Sphere& j_sphere)
		{
			const Vec3& i_center = i_sphere.center;
			float i_radius = i_sphere.radius;

			const Vec3& j_center = j_sphere.center;
			float j_radius = j_sphere.radius;

			// finding a tube tangent to both spheres
			Vec3 ij = j_center - i_center;
			float ijmag = ij.ComputeMagnitude(), inv_ijmag = 1.0f / ijmag;
			Vec3 uij = ij * inv_ijmag;

			// "theta" would be the angle from the axis vector to the point of tangency
			float cos_theta = (j_radius - i_radius) * inv_ijmag, sin_theta = sqrtf(1.0f - cos_theta * cos_theta);

			Vec3 p1 = i_center + uij * (i_radius * cos_theta);					// center points of the circles where the tube touches the spheres
			Vec3 p2 = j_center + uij * (j_radius * cos_theta);
			float r1 = i_radius * sin_theta, r2 = j_radius * sin_theta;			// radii of those circles

			TubePart tp(p1, p2, r1, r2);

			// cache some parts of the ray-test formula
			tp.cos_theta = cos_theta;
			tp.sin_theta = sin_theta;
			tp.u = uij;
			tp.inv_dmag = 1.0f / (p2 - p1).ComputeMagnitude();

			float z = (r2 - r1) * inv_ijmag;
			tp.opzsq = 1 + z * z;
			tp.trz = 2.0f * r1 * z;

			// add cutting planes for the spheres capping off the ends
			tp.planes.push_back(Plane( uij,  Vec3::Dot(uij, p1)));
			tp.planes.push_back(Plane(-uij, -Vec3::Dot(uij, p2)));

			return tp;
		}

		void Init(Sphere* input_spheres, unsigned int count)
		{
			spheres = vector<SpherePart>();
			tubes = vector<TubePart>();
			planes = vector<PlanePart>();

			if(count > 0)
			{
				const float plane_nonplanar_threshold		= 0.0001f;
				const float plane_noncontribution_threshold	= 0.000001f;
				const float plane_dup_normal_threshold		= 0.9999f;
				const float plane_dup_offset_threshold		= 0.000001f;
				const float cut_noncontribution_threshold	= 0.0000001f;

				unsigned int num_spheres = 0;
				Sphere* spheres_temp = new Sphere[count];

				// get a condensed list of spheres (discarding anything that is completely contained inside another sphere)
				for(unsigned int i = 0; i < count; ++i)
				{
					const Sphere& i_sphere = input_spheres[i];
					const Vec3& i_center = i_sphere.center;
					float i_radius = i_sphere.radius;

					unsigned int j;
					for(j = 0; j < num_spheres; ++j)
					{
						const Sphere& j_sphere = spheres_temp[j];

						float dr = i_radius - j_sphere.radius;
						float dif = (i_center - j_sphere.center).ComputeMagnitudeSquared() - dr * dr;
						if(dif <= 0)
						{
							if(dr <= 0)				// this sphere was eaten by a sphere that was already added
								break;
							else					// this sphere eats a sphere that has already been added
								swap(spheres_temp[j], spheres_temp[num_spheres--]);
						}
					}

					if(j >= num_spheres)			// need to check >= in case we end up decrementing num_spheres on the last action
					{
						spheres_temp[num_spheres++] = i_sphere;

						if(i == 0)
							aabb = AABB(i_center, i_radius);
						else
							aabb.Expand(AABB(i_center, i_radius));
					}
				}


				// construct planes (and adjoining tubes)
				Plane* planes_temp = NULL;
				bool* plane_spheres = NULL;			// for each possible plane, booleans for whether each sphere contributed to it
				
				unsigned int* twospheres_to_tube = new unsigned int[num_spheres * num_spheres];
				memset(twospheres_to_tube, 0, sizeof(unsigned int) * num_spheres * num_spheres);

				unsigned int num_planes = 0;

				if(num_spheres >= 3)
				{
					unsigned int max_planes = num_spheres * (num_spheres - 1) * (num_spheres - 2);
					planes_temp = new Plane[max_planes];
					plane_spheres = new bool[max_planes * num_spheres];
					memset(plane_spheres, 0, sizeof(bool) * max_planes * num_spheres);

					for(unsigned int i = 2; i < num_spheres; ++i)
					{
						const Sphere& i_sphere = spheres_temp[i];
						const Vec3& i_center = i_sphere.center;
						float i_radius = i_sphere.radius;

						for(unsigned int j = 1; j < i; ++j)
						{
							const Sphere& j_sphere = spheres_temp[j];
							const Vec3& j_center = j_sphere.center;
							float j_radius = j_sphere.radius;

							Vec3 ij = j_center - i_center;
							float ijmag = ij.ComputeMagnitude(), inv_ijmag = 1.0f / ijmag;
							float ij_costheta = (j_radius - i_radius) * inv_ijmag;

							Plane ij_tubenormals = Plane(ij * inv_ijmag, -ij_costheta);

							for(unsigned int k = 0;	k < j; ++k)
							{
								const Sphere& k_sphere = spheres_temp[k];
								const Vec3& k_center = k_sphere.center;
								float k_radius = k_sphere.radius;

								Vec3 jk = k_center - j_center;
								float jkmag = jk.ComputeMagnitude(), inv_jkmag = 1.0f / jkmag;
								float jk_costheta = (k_radius - j_radius) * inv_jkmag;

								// construct 0 or 2 planes for these 3 spheres
								Vec3 centerplane_normal = Vec3::Cross(ij, k_sphere.center - i_sphere.center);
								if(float cpn_magsq = centerplane_normal.ComputeMagnitudeSquared())
								{
									centerplane_normal /= sqrtf(cpn_magsq);

									Plane jk_tubenormals = Plane(jk * inv_jkmag, -jk_costheta);

									Line line;
									if(Plane::Intersect(ij_tubenormals, jk_tubenormals, line))
									{
										float intersections[2];			// intersecting a ray with the unit sphere of possible normal vectors
										if(Util::RaySphereIntersect(Ray(line), Sphere(Vec3(), 1.0f), intersections[0], intersections[1]))
										{
											for(unsigned char m = 0; m < 2; ++m)
											{
												Vec3 normal = line.origin + line.direction * intersections[m];
												float offset = Vec3::Dot(normal, i_center) + i_radius;

												Plane plane = Plane(normal, offset);

												// in some odd cases it may not actually be possible to place a plane tangent to 3 spheres
												if(fabs(fabs(plane.PointDistance(i_center)) - i_radius) > plane_nonplanar_threshold || fabs(fabs(plane.PointDistance(j_center)) - j_radius) > plane_nonplanar_threshold || fabs(fabs(plane.PointDistance(k_center)) - k_radius) > plane_nonplanar_threshold)
													continue;

												// go through existing planes and see if this is a duplicate
												unsigned int n;
												for(n = 0; n < num_planes; ++n)
													if(Vec3::Dot(normal, planes_temp[n].normal) > plane_dup_normal_threshold && fabs(offset - planes_temp[n].offset) < plane_dup_offset_threshold)
														break;

												if(n == num_planes)
													planes_temp[num_planes++] = plane;													

												// register these spheres with the appropriate plane
												bool* spheres_registry = &plane_spheres[n * num_spheres];
												spheres_registry[i] = spheres_registry[j] = spheres_registry[k] = true;
											}
										}
									}
								}
							}
						}
					}

					// now that we've got all our planes and we know which spheres are relevant to each of them, let's create the actual PlanePart objects
					Vec3* plane_verts = new Vec3[num_spheres];
					unsigned int* vert_ids = new unsigned int[num_spheres];

					for(unsigned int i = 0; i < num_planes; ++i)
					{
						const Plane& plane = planes_temp[i];
						const Vec3& normal = plane.normal;
						float offset = plane.offset;

						// see whether or not this plane actually contributes to the surface of the shape
						bool worthy = true;
						for(unsigned int n = 0; n < num_spheres; ++n)
						{
							float dist = Vec3::Dot(spheres_temp[n].center, normal) + spheres_temp[n].radius;
							if(dist > offset + plane_noncontribution_threshold)
							{
								worthy = false;
								break;
							}
						}

						if(worthy)
						{
							bool* spheres_registry = &plane_spheres[i * num_spheres];

							PlanePart pp = PlanePart(plane);

							// project relevant spheres' centers onto plane
							unsigned int num_verts = 0;
							for(unsigned int j = 0; j < num_spheres; ++j)
								if(spheres_registry[j])
								{
									const Vec3& sphere_center = spheres_temp[j].center;
									plane_verts[num_verts] = sphere_center - normal * (Vec3::Dot(sphere_center, normal) - offset);
									vert_ids[num_verts] = j;

									++num_verts;
								}

							// add cutting planes
							for(unsigned int j = 1; j < num_verts; ++j)
							{
								const Vec3& j_vert = plane_verts[j];
								for(unsigned int k = 0; k < j; ++k)
								{
									const Vec3& k_vert = plane_verts[k];

									Plane cps[] = { Plane::FromTriangleVertices(j_vert, k_vert, j_vert + normal), Plane::FromTriangleVertices(k_vert, j_vert, j_vert + normal) };
									for(unsigned char m = 0; m < 2; ++m)
									{
										const Plane& cp = cps[m];

										// see if all the verts are on the correct side of the proposed cutting plane
										unsigned int n;
										for(n = 0; n < num_verts; ++n)
											if(Vec3::Dot(cp.normal, plane_verts[n]) < cp.offset - cut_noncontribution_threshold)
												break;
								
										if(n == num_verts)
										{
											// add the cutting plane to this PlanePart
											pp.planes.push_back(cp);

											// and also add it to the appropriate TubePart (creating one if necessary)
											unsigned int sphere_i = vert_ids[j];
											unsigned int sphere_j = vert_ids[k];

											if(sphere_i < sphere_j)
												swap(sphere_i, sphere_j);

											unsigned int& tube_index = twospheres_to_tube[sphere_i * num_spheres + sphere_j];
											if(tube_index == 0)
											{
												tubes.push_back(MakeTube(spheres_temp[sphere_i], spheres_temp[sphere_j]));
												tube_index = tubes.size();
											}

											TubePart& tp = tubes[tube_index - 1];
											tp.planes.push_back(Plane::Reverse(cp));
										}
									}
								}
							}

							planes.push_back(pp);
						}
					}

					delete[] plane_verts;
					delete[] vert_ids;
				}

				// construct tubes (with no adjoining planes)
				unsigned int num_tubes = 0;
				if(num_spheres >= 2)
				{
					unsigned int max_tubes = num_spheres * (num_spheres - 1);

					for(unsigned int i = 1; i < num_spheres; ++i)
					{
						for(unsigned int j = 0; j < i; ++j)
						{
							// we only create a tube here if there weren't any attempted planes
							unsigned int k;
							for(k = 0; k < num_planes; ++k)
								if(plane_spheres[k * num_spheres + i] && plane_spheres[k * num_spheres + j])
									break;

							if(k == num_planes)
							{
								tubes.push_back(MakeTube(spheres_temp[i], spheres_temp[j]));
								twospheres_to_tube[i * num_spheres + j] = tubes.size();
							}
						}
					}
				}

				// construct final spheres
				for(unsigned int i = 0; i < num_spheres; ++i)
				{
					const Sphere& i_sphere = spheres_temp[i];
					const Vec3& i_center = i_sphere.center;
					float i_radius = i_sphere.radius;

					SpherePart* sp = NULL;

					if(!tubes.empty())
					{
						for(unsigned int j = 0; j < num_spheres; ++j)
						{
							if(unsigned int tube_index = twospheres_to_tube[max(i, j) * num_spheres + min(i, j)])
							{
								if(sp == NULL)
								{
									spheres.push_back(SpherePart(i_sphere));
									sp = &*spheres.rbegin();
								}

								TubePart& tp = tubes[tube_index - 1];
								if(i > j)
									sp->planes.push_back(Plane::Reverse(tp.planes[0]));
								else
									sp->planes.push_back(Plane::Reverse(tp.planes[1]));
							}
						}
					}
					else
						spheres.push_back(SpherePart(i_sphere));
				}

				// clean up temporary arrays
				delete[] spheres_temp;
				if(planes_temp) { delete[] planes_temp; }
				if(plane_spheres) { delete[] plane_spheres; }
				delete[] twospheres_to_tube;

				// OutputParts();
			}
			else
				aabb = AABB();
		}


		// misc. utility stuff
		void OutputParts()
		{
			stringstream ss;
			ss << "Shape contains " << spheres.size() << " spheres, " << tubes.size() << " tubes, and " << planes.size() << " planes" << endl;
			ss << endl << "Spheres:" << endl;
			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				Sphere& sphere = iter->sphere;
				ss << endl << "\tSphere: center = (" << sphere.center.x << ", " << sphere.center.y << ", " << sphere.center.z << "), radius = " << sphere.radius << endl;
				for(vector<Plane>::iterator jter = iter->planes.begin(); jter != iter->planes.end(); ++jter)
					ss << "\t\tPlane: normal = (" << jter->normal.x << ", " << jter->normal.y << ", " << jter->normal.z << "), offset = " << jter->offset << endl;
			}
			ss << endl << "Tubes:" << endl;
			for(vector<TubePart>::iterator iter = tubes.begin(); iter != tubes.end(); ++iter)
			{
				Vec3& p1 = iter->p1;
				Vec3& p2 = iter->p2;
				ss << endl << "\tTube: p1 = (" << p1.x << ", " << p1.y << ", " << p1.z << "), p2 = (" << p2.x << ", " << p2.y << ", " << p2.z << "), r1 = " << iter->r1 << ", r2 = " << iter->r2 << ", theta = " << atan2f(iter->sin_theta, iter->cos_theta) << endl;
				for(vector<Plane>::iterator jter = iter->planes.begin(); jter != iter->planes.end(); ++jter)
					ss << "\t\tPlane: normal = (" << jter->normal.x << ", " << jter->normal.y << ", " << jter->normal.z << "), offset = " << jter->offset << endl;
			}
			ss << endl << "Planes:" << endl;
			for(vector<PlanePart>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
			{
				ss << endl << "\tPlane: normal = (" << iter->plane.normal.x << ", " << iter->plane.normal.y << ", " << iter->plane.normal.z << "), offset = " << iter->plane.offset << endl;
				for(vector<Plane>::iterator jter = iter->planes.begin(); jter != iter->planes.end(); ++jter)
					ss << "\t\tPlane: normal = (" << jter->normal.x << ", " << jter->normal.y << ", " << jter->normal.z << "), offset = " << jter->offset << endl;
			}
			Debug(ss.str());
		}

		void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori, const Vec3& color)
		{
			Mat4 model_mat = Mat4::FromPositionAndOrientation(pos, ori);
			const Vec3& eye = renderer->camera->GetPosition();
			const Vec3& forward = renderer->camera->GetForward();

			struct Circle
			{
				Vec3 wc, wn, eye;
				float wr;

				Circle() { }
				Circle(const Vec3& eye, const Mat4& model_mat, const Sphere& sphere) : eye(eye)
				{
					Vec3 center = model_mat.TransformVec3_1(sphere.center);
					Vec3 d = center - eye;
					float dist_sq = d.ComputeMagnitudeSquared();

					float radius = sphere.radius, radius_sq = radius * radius;
					if(radius_sq <= dist_sq)
					{
						float inv_dist = 1.0f / sqrtf(dist_sq);
						float offset = radius_sq * inv_dist;

						wr = sqrtf(radius_sq - offset * offset);		// the expression under the radical is guaranteed to be positive if the above condition passed
						wn = d * inv_dist;
						wc = center + wn * offset;
					}
					else
						wr = -1;
				}

				void SetFarthestExtent(const Vec3& dir, const Vec3& eye, const Vec3& forward, float& farthest, Vec3& pos) const
				{
					Vec3 on_plane = dir - wn * Vec3::Dot(dir, wn);
					float mag = on_plane.ComputeMagnitude(), inv_mag = 1.0f / mag;

					pos = wc + on_plane * (wr * inv_mag);

					Vec3 from_eye = pos - eye;
					farthest = Vec3::Dot(from_eye, dir) / Vec3::Dot(from_eye, forward);
				}

				void MaybeSetFarthestExtent(const Vec3& dir, const Vec3& eye, const Vec3& forward, float& farthest, Vec3& pos) const
				{
					Vec3 on_plane = dir - wn * Vec3::Dot(dir, wn);
					float mag = on_plane.ComputeMagnitude(), inv_mag = 1.0f / mag;

					Vec3 temp_pos = wc + on_plane * (wr * inv_mag);

					Vec3 from_eye = temp_pos - eye;
					float extent = Vec3::Dot(from_eye, dir) / Vec3::Dot(from_eye, forward);
					if(extent > farthest)
					{
						pos = temp_pos;
						farthest = extent;
					}
				}
			};

			bool any = false;

			vector<Circle> circles;
			for(vector<SpherePart>::iterator iter = spheres.begin(), spheres_end = spheres.end(); iter != spheres_end; ++iter)
			{
				if(!any && renderer->camera->CheckSphereVisibility(Sphere(model_mat.TransformVec3_1(iter->sphere.center), iter->sphere.radius)))
					any = true;
				Circle circle = Circle(eye, model_mat, iter->sphere);
				if(circle.wr >= 0)
					circles.push_back(Circle(eye, model_mat, iter->sphere));
			}

			if(circles.empty() || !any)
				return;

			Vec3 left = -renderer->camera->GetRight(), up = renderer->camera->GetUp();

			// now get the points on the silhouette edge
			static const unsigned int num_edges = 64;
			static const float theta_coeff = 2.0f * float(M_PI) / num_edges;

			// cache a unit circle
			static float *unit_circle_x = NULL, *unit_circle_y = NULL;
			if(unit_circle_x == NULL)
			{
				unit_circle_x = new float[num_edges + 1];
				unit_circle_y = new float[num_edges + 1];

				for(unsigned int i = 0; i <= num_edges; ++i)
				{
					float theta = i * theta_coeff;

					unit_circle_x[i] = cosf(theta);
					unit_circle_y[i] = sinf(theta);
				}
			}

			Circle* circles_begin = circles.data();
			Circle* circles_end = circles_begin + circles.size();

			DebugDrawMaterial* material = DebugDrawMaterial::GetDebugDrawMaterial();

			Vec3 temp, cur;
			float *ucx_ptr = unit_circle_x, *ucy_ptr = unit_circle_y;
			for(unsigned int i = 0; i <= num_edges; ++i)
			{
				Vec3 world_dir = left * *(ucx_ptr++) + up * (*ucy_ptr++);

				float farthest;

				Circle* iter = circles_begin;
				iter->SetFarthestExtent(world_dir, eye, forward, farthest, cur);

				for(++iter; iter != circles_end; ++iter)
					iter->MaybeSetFarthestExtent(world_dir, eye, forward, farthest, cur);

				if(i != 0)
					renderer->objects.push_back(RenderNode(material, material->New(cur, temp, color), 1.0f));
				temp = cur;
			}
		}

		AABB GetTransformedAABB(const Mat4& xform)
		{
#if 1
			// this will produce a tighter fitting AABB than aabb.GetTransformedAABB(xform), but it may be slower (especially if there are more than 8 spheres!)
			AABB xformed_aabb;
			for(vector<SpherePart>::const_iterator iter = spheres.begin(), spheres_end = spheres.end(); iter != spheres_end; ++iter)
			{
				const Sphere& sphere = iter->sphere;
				Vec3 pos = xform.TransformVec3_1(sphere.center);

				if(iter == spheres.begin())
					xformed_aabb = AABB(pos, sphere.radius);
				else
					xformed_aabb.Expand(AABB(pos, sphere.radius));
			}
			return xformed_aabb;
#else
			// even faster computation, but the result is bigger than the results of both the above implementation and AABB::GetTransformedAABB

			Vec3 center = xform.TransformVec3_1((aabb.min + aabb.max) * 0.5f);
			float radius = (aabb.max - aabb.min).ComputeMagnitude() / 2.0f;

			return AABB(center, radius);
#endif
		}

		AABB ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache)
		{
			MultiSphereShapeInstanceCache* mssic = (MultiSphereShapeInstanceCache*)cache;
			if(!mssic)
				cache = mssic = new MultiSphereShapeInstanceCache();

			mssic->Update(xform, this);

			return mssic->aabb;
		}

		MassInfo ComputeMassInfo()
		{
			if(aabb.IsDegenerate())
				return MassInfo();
			else
			{
#if 1
				static const unsigned int resolution = 200;

				Vec3 dim = aabb.max - aabb.min;

				float fres = float(resolution);
				float xstep = dim.x / fres;
				float ystep = dim.y / fres;
				float hxstep = xstep * 0.5f;
				float hystep = ystep * 0.5f;

				vector<MassInfo> planes;
				for(unsigned int i = 0; i < resolution; ++i)
				{
					vector<MassInfo> lines;
					float px = aabb.min.x + (float(i) + 0.5f) * xstep;
					for(unsigned int j = 0; j < resolution; ++j)
					{
						float py = aabb.min.y + (float(j) + 0.5f) * ystep;

						RayResult rr1, rr2;
						rr1.t = rr2.t = 1.0f;
						if(CollideRay(Ray(Vec3(px, py, aabb.min.z), Vec3(0, 0, dim.z)), rr1, nullptr, nullptr))
							if(CollideRay(Ray(Vec3(px, py, aabb.max.z), Vec3(0, 0, -dim.z)), rr2, nullptr, nullptr))
								if(rr2.pos.z > rr1.pos.z)
									lines.push_back(MassInfoFromAABB(AABB(Vec3(px - hxstep, py - hystep, rr1.pos.z), Vec3(px + hxstep, py + hystep, rr2.pos.z))));
					}

					if(!lines.empty())
						planes.push_back(MassInfo::Sum(lines.data(), lines.size()));
				}

				if(planes.empty())
					return MassInfo();
				else
					return MassInfo::Sum(planes.data(), planes.size());

#else
				// approximate the MultiSphereShape as a box (same as how Bullet handles them)
				return MassInfoFromAABB(aabb);
#endif
			}
		}

		MassInfo MassInfoFromAABB(const AABB& aabb)
		{
			MassInfo temp;

			Vec3 dim = aabb.max - aabb.min;

			temp.mass = dim.x * dim.y * dim.z;							// assumes density = 1
			temp.com = (aabb.min + aabb.max) * 0.5f;

			float coeff = temp.mass / 12.0f;
			temp.moi[0] = coeff * (dim.y * dim.y + dim.z * dim.z);
			temp.moi[4] = coeff * (dim.x * dim.x + dim.z * dim.z);
			temp.moi[8] = coeff * (dim.x * dim.x + dim.y * dim.y);

			return temp;
		}




		// multisphere collision functions...
		bool CollideRay(const Ray& ray, RayResult& result, RayCollider* collider, RigidBody* body)
		{
			Vec3 a = ray.origin, b = ray.origin + ray.direction;
			AABB ray_aabb(Vec3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)), Vec3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)));

			if(!AABB::IntersectTest(ray_aabb, aabb))
				return false;

			RayResult rr;

			bool any = false;

			for(vector<SpherePart>::iterator iter = spheres.begin(), spheres_end = spheres.end(); iter != spheres_end; ++iter)
				if(iter->RayTest(ray, rr))
					if(!any || rr.t < result.t)
					{
						result = rr;
						any = true;
					}

			for(vector<TubePart>::iterator iter = tubes.begin(), tubes_end = tubes.end(); iter != tubes_end; ++iter)
				if(iter->RayTest(ray, rr))
					if(!any || rr.t < result.t)
					{
						result = rr;
						any = true;
					}

			for(vector<PlanePart>::iterator iter = planes.begin(), planes_end = planes.end(); iter != planes_end; ++iter)
				if(iter->RayTest(ray, rr))
					if(!any || rr.t < result.t)
					{
						result = rr;
						any = true;
					}

			if(any)
			{
				result.collider = collider;
				result.body = body;

				return true;
			}
			else
				return false;
		}

		ContactRegion* CollideSphere(const Sphere& sphere, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return NULL;
		}

		ContactRegion* CollidePlane(const Mat4& my_xform, const Plane& plane, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)
		{
			Vec3 plane_norm = plane.normal;
			float plane_offset = plane.offset;

			vector<Vec3> positions;
			for(vector<SpherePart>::iterator iter = spheres.begin(), spheres_end = spheres.end(); iter != spheres_end; ++iter)
			{
				Vec3 sphere_pos = my_xform.TransformVec3_1(iter->sphere.center);
				float radius = iter->sphere.radius;

				float dist = Vec3::Dot(plane_norm, sphere_pos) - plane_offset - radius;

				if(dist <= 0.0f)
					positions.push_back(sphere_pos - plane_norm * radius);
			}

			if(positions.empty())
				return NULL;
			else
				return collect->AddRegion(ibody, jbody, -plane.normal, positions.size(), positions.data());
		}

		ContactRegion* CollideMesh(const Mat4& my_xform, vector<Sphere>& my_spheres, const TriangleMeshShape::TriCache& tri, RigidBody* ibody, RigidBody* jbody, ContactDataCollector* collect)
		{
			// vector containing just the spheres (no extra stuff like cutting planes, etc.) transformed into the triangle's coordinate system
			if(my_spheres.empty())		// only compute this if it hasn't been computed already
			{
				my_spheres.reserve(spheres.size());

				for(vector<SpherePart>::iterator iter = spheres.begin(), spheres_end = spheres.end(); iter != spheres_end; ++iter)
					my_spheres.push_back(Sphere(my_xform.TransformVec3_1(iter->sphere.center), iter->sphere.radius));
			}

			// try to find a separating axis
			struct Scorer
			{
				const Sphere *begin, *end;
				const TriangleMeshShape::TriCache& tri;

				bool first;

				float least;
				Vec3 direction;

				Scorer(const vector<Sphere>& a, const TriangleMeshShape::TriCache& tri) : begin(a.data()), end(begin + a.size()), tri(tri), first(true) { }

				bool Score(const Vec3& dir)
				{
					// get max extent of multisphere shape
					const Sphere* iter = begin;

					float max_val = Vec3::Dot(dir, iter->center) + iter->radius;
					++iter;

					while(iter != end)
					{
						max_val = max(max_val, Vec3::Dot(dir, iter->center) + iter->radius);
						++iter;
					}

					// get min extent of triangle
					float min_val = min(Vec3::Dot(dir, tri.a), min(Vec3::Dot(dir, tri.b), Vec3::Dot(dir, tri.c)));

					// do stuff with the results
					float value = max_val - min_val;

					if(first)
					{
						least = value;
						direction = dir;
						first = false;
					}
					else if(value < least)
					{
						least = value;
						direction = dir;
					}

					return value <= 0;
				}
			} scorer(my_spheres, tri);



			// triangle's planes
			if(scorer.Score(-tri.plane.normal))	{ return NULL; }
			if(scorer.Score(tri.plane.normal))	{ return NULL; }

			// my spheres ...
			for(vector<Sphere>::iterator iter = my_spheres.begin(), spheres_end = my_spheres.end(); iter != spheres_end; ++iter)
			{
				const Vec3& s = iter->center;

				Vec3 sa = tri.a - s;
				Vec3 sb = tri.b - s;
				Vec3 sc = tri.c - s;

				// ... vs. triangle's verts
				if(scorer.Score(Vec3::Normalize(sa)))	{ return NULL; }
				if(scorer.Score(Vec3::Normalize(sb)))	{ return NULL; }
				if(scorer.Score(Vec3::Normalize(sc)))	{ return NULL; }

				// ... vs. triangle's edges
				Vec3 absnabn = Vec3::Normalize(Vec3::Cross(tri.ab, Vec3::Cross(sa, tri.ab)));
				if(scorer.Score(absnabn))				{ return NULL; }

				Vec3 bcsnbcn = Vec3::Normalize(Vec3::Cross(tri.bc, Vec3::Cross(sb, tri.bc)));
				if(scorer.Score(bcsnbcn))				{ return NULL; }

				Vec3 casncan = Vec3::Normalize(Vec3::Cross(tri.ac, Vec3::Cross(sc, tri.ac)));		// would be -ac, but there are two of them
				if(scorer.Score(casncan))				{ return NULL; }
			}

			// my tubes ...
			for(vector<TubePart>::iterator iter = tubes.begin(), tubes_end = tubes.end(); iter != tubes_end; ++iter)
			{
				Vec3 p1 = my_xform.TransformVec3_1(iter->p1);
				Vec3 p2 = my_xform.TransformVec3_1(iter->p2);
				Vec3 u = my_xform.TransformVec3_0(iter->u);

				float sin_theta = iter->sin_theta;
				Vec3 u_cos_theta = u * iter->cos_theta;
				Plane my_plane = Plane(u, sin_theta);

				// ... vs. triangle's verts
				Vec3 atntn = Vec3::Normalize(Vec3::Cross(u, Vec3::Cross(u, tri.a - p1)), sin_theta);
				if(scorer.Score(u_cos_theta - atntn))	{ return NULL; }

				Vec3 btntn = Vec3::Normalize(Vec3::Cross(u, Vec3::Cross(u, tri.b - p1)), sin_theta);
				if(scorer.Score(u_cos_theta - btntn))	{ return NULL; }

				Vec3 ctntn = Vec3::Normalize(Vec3::Cross(u, Vec3::Cross(u, tri.c - p1)), sin_theta);
				if(scorer.Score(u_cos_theta - ctntn))	{ return NULL; }
			}



			// if we get this far, it means the objects are intersecting
			float best;
			Vec3 pos;
			for(unsigned int i = 0; i < my_spheres.size(); ++i)
			{
				const Sphere& sphere = my_spheres[i];
				float dist = Vec3::Dot(sphere.center, scorer.direction) + sphere.radius;
				if(i == 0 || best < dist)
				{
					best = dist;
					pos = sphere.center + scorer.direction * sphere.radius;
				}
			}

			return collect->AddRegion(ibody, jbody, -tri.plane.normal, 1, &pos);;
		}


		// i/o functions
		void Write(ostream& stream)
		{
			WriteUInt32(spheres.size(), stream);
			for(vector<SpherePart>::iterator iter = spheres.begin(), spheres_end = spheres.end(); iter != spheres_end; ++iter)
			{
				WriteVec3(iter->sphere.center, stream);
				WriteSingle(iter->sphere.radius, stream);
			}
		}

		unsigned int Read(istream& stream)
		{
			if(unsigned int count = ReadUInt32(stream))
			{
				Sphere* spheres = new Sphere[count];

				for(unsigned int i = 0; i < count; ++i)
				{
					Vec3 center = ReadVec3(stream);
					float radius = ReadSingle(stream);

					spheres[i] = Sphere(center, radius);
				}

				Init(spheres, count);

				delete[] spheres;
			}

			return stream.fail() ? 1 : 0;
		}
	};




	/*
	 * MultiSphereShape methods
	 */
	MultiSphereShape::MultiSphereShape() : CollisionShape(ST_MultiSphere), imp(new Imp())													{ }
	MultiSphereShape::MultiSphereShape(Sphere* spheres, unsigned int count) : CollisionShape(ST_MultiSphere), imp(new Imp(spheres, count))	{ }

	void MultiSphereShape::InnerDispose()															{ delete imp; imp = NULL; }

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori, const Vec3& color) { imp->DebugDraw(renderer, pos, ori, color); }

	AABB MultiSphereShape::GetAABB()																{ return imp->aabb; }
	AABB MultiSphereShape::GetTransformedAABB(const Mat4& xform)									{ return imp->GetTransformedAABB(xform); }
	AABB MultiSphereShape::ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache)	{ return imp->ComputeCachedWorldAABB(xform, cache); }

	MassInfo MultiSphereShape::ComputeMassInfo()													{ return imp->ComputeMassInfo(); }

	bool MultiSphereShape::CollideRay(const Ray& ray, RayResult& result, RayCollider* collider, RigidBody* body)																								{ return imp->CollideRay(ray, result, collider, body); }
	ContactRegion* MultiSphereShape::CollidePlane(const Mat4& my_xform, const Plane& plane, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)													{ return imp->CollidePlane(my_xform, plane, collect, ibody, jbody); }
	ContactRegion* MultiSphereShape::CollideSphere(const Sphere& sphere, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)																		{ return imp->CollideSphere(sphere, collect, ibody, jbody); }
	ContactRegion* MultiSphereShape::CollideMesh(const Mat4& my_xform, vector<Sphere>& my_spheres, const TriangleMeshShape::TriCache& tri, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)	{ return imp->CollideMesh(my_xform, my_spheres, tri, ibody, jbody, collect); }

	void MultiSphereShape::Write(ostream& stream)													{ imp->Write(stream); }
	unsigned int MultiSphereShape::Read(istream& stream)											{ return imp->Read(stream); }




	/*
	 * MultiSphereShapeInstanceCache methods
	 */
	MultiSphereShapeInstanceCache::MultiSphereShapeInstanceCache() : spheres(), aabb() { }
	MultiSphereShapeInstanceCache::~MultiSphereShapeInstanceCache() { }

	void MultiSphereShapeInstanceCache::Update(const Mat4& xform, MultiSphereShape::Imp* imp)
	{
		spheres.clear();
		for(unsigned int i = 0; i < imp->spheres.size(); ++i)
		{
			Sphere s = imp->spheres[i].sphere;
			s = Sphere(xform.TransformVec3_1(s.center), s.radius);
			spheres.push_back(s);

			if(!i)
				aabb = AABB(s.center, s.radius);
			else
				aabb.Expand(AABB(s.center, s.radius));
		}
	}
}
