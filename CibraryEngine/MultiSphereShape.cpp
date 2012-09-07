#include "StdAfx.h"
#include "MultiSphereShape.h"

#include "Physics.h"

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
				for(vector<Plane>::const_iterator iter = planes.begin(); iter != planes.end(); ++iter)
					if(iter->PointDistance(point) < 0)
						return false;
				return true;
			}

			virtual int RayTest(const Ray& ray) const = 0;
			virtual bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time) const = 0;
		};

		struct SpherePart : Part
		{
			Sphere sphere;
			SpherePart(Sphere sphere) : Part(), sphere(sphere) { }

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
			bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time) const
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

			int sphere1, sphere2;

			TubePart(Vec3 p1, Vec3 p2, float r1, float r2, int sphere1, int sphere2) : Part(), p1(p1), p2(p2), r1(r1), r2(r2), sphere1(sphere1), sphere2(sphere2)
			{
				Vec3 n = Vec3::Normalize(p2 - p1);

				planes.push_back(Plane(n, Vec3::Dot(n, p1)));
				planes.push_back(Plane(-n, -Vec3::Dot(n, p2)));
			}

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
			bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time) const
			{
				float t[2];
				if(RayTestInfinite(ray, t[0], t[1]))
				{
					for(unsigned char i = 0; i < 2; ++i)
					{
						Vec3 pos = ray.origin + ray.direction * t[i];
						if(t[i] >= 0.0f && t[i] <= 1.0f && IsPointRelevant(pos))
						{
							contact.pos = pos;

							Vec3 from_axis = pos - (p1 + u * Vec3::Dot(pos - p1, u));
							contact.norm = Vec3::Normalize(from_axis) * sin_theta + u * cos_theta;

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
			bool RayTest(const Ray& ray, ContactPoint::Part& contact, float& time) const
			{
				float t = Util::RayPlaneIntersect(ray, plane);
				Vec3 pos = ray.origin + ray.direction * t;

				if(IsPointRelevant(pos))
					if(t >= 0.0f && t <= 1.0f)
					{
						contact.pos = pos;
						contact.norm = plane.normal;

						time  = t;

						return true;
					}

				return false;
			}

			vector<Vec3> GetVerts()
			{
				vector<Vec3> result;

				for(unsigned int i = 0; i < planes.size(); ++i)
					for(unsigned int j = i + 1; j < planes.size(); ++j)
					{
						Line line;
						Plane::Intersect(planes[i], planes[j], line);

						Ray ray;
						ray.origin = line.origin;
						ray.direction = line.direction;

						float d = Util::RayPlaneIntersect(ray, plane);
						if(_finite(d))			// TODO: come up with a solution that doesn't depend on msvc
						{
							Vec3 vert = ray.origin + ray.direction * d;

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

		void Init(Sphere* input_spheres, unsigned int count)
		{
			spheres = vector<SpherePart>();
			tubes = vector<TubePart>();
			planes = vector<PlanePart>();

			if(count > 0)
			{
				// creating spheres
				vector<Sphere> spheres_temp;
				for(unsigned int i = 0; i < count; ++i)
				{
					const Sphere& sphere = input_spheres[i];

					Vec3 i_center = sphere.center;
					float i_radius = sphere.radius;

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

					unsigned int j;
					for(j = 0; j < count; ++j)
						if(i != j)
						{
							const Sphere& b = spheres_temp[j];

							float dr = a.radius - b.radius;
							float dif = (a.center - b.center).ComputeMagnitudeSquared() - dr * dr;

							if(dif <= 0 && (dr < 0 || dr == 0 && i < j))
								break;
						}
					if(j == count)											// did we make it all the way through without a larger sphere eating this one?
						spheres.push_back(SpherePart(a));
				}

				unsigned int num_spheres = spheres.size();					// may differ from count, if spheres got discarded
				unsigned int n_sq = num_spheres * num_spheres;
				unsigned int n_cubed = n_sq * num_spheres;

				// creating tubes
				TubePart** sphere_tubes = new TubePart* [n_sq];
				memset(sphere_tubes, 0, sizeof(TubePart*) * n_sq);

				bool* tube_tried_planes = new bool[n_sq];
				memset(tube_tried_planes, 0, sizeof(bool) * n_sq);

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

						TubePart tube(p1, p2, r1, r2, i, j);

						// cache some parts of the ray-test formula
						tube.cos_theta = cos_theta;
						tube.sin_theta = sin_theta;
						tube.u = ud;
						tube.inv_dmag = 1.0f / (p2 - p1).ComputeMagnitude();				// careful! it's not the same as the inv_dmag used here!

						float z = (r2 - r1) * inv_dmag;
						tube.opzsq = 1 + z * z;
						tube.trz = 2.0f * r1 * z;

						sphere_tubes[i_n + j] = new TubePart(tube);
					}
				}

				// creating planes
				PlanePart** sphere_planes = new PlanePart* [n_cubed * 2];
				memset(sphere_planes, 0, sizeof(PlanePart*) * n_cubed * 2);

				bool* planes_valid = new bool[n_cubed * 2];
				memset(planes_valid, 0, sizeof(bool) * n_cubed * 2);

				for(unsigned int i = 0; i < num_spheres; ++i)
				{
					unsigned int i_n = i * num_spheres;
					SpherePart& i_sphere = spheres[i];									// looking to make planes from this sphere...

					Vec3 i_center = i_sphere.sphere.center;
					float i_radius = i_sphere.sphere.radius;

					for(unsigned int j = i + 1; j < num_spheres; ++j)
					{
						if(TubePart* ij_tubep = sphere_tubes[i_n + j])
						{
							TubePart& ij_tube = *ij_tubep;								// found one tube connected to this sphere

							unsigned int j_n = j * num_spheres;
							SpherePart& j_sphere = spheres[j];

							Vec3 j_center = j_sphere.sphere.center;
							float j_radius = j_sphere.sphere.radius;

							for(unsigned int k = j + 1; k < num_spheres; ++k)
							{
								if(TubePart* ik_tubep = sphere_tubes[i_n + k])
								{
									TubePart& ik_tube = *ik_tubep;											// found a second tube; make planes between them
									SpherePart& k_sphere = spheres[k];

									Vec3 k_center = k_sphere.sphere.center;
									float k_radius = k_sphere.sphere.radius;

									TubePart* jk_tubep = sphere_tubes[j_n + k];

									Vec3 cross = Vec3::Cross(ij_tube.u, ik_tube.u);
									float cross_magsq = cross.ComputeMagnitudeSquared();
									if(cross_magsq > 0)
									{
										Vec3 n = cross / sqrtf(cross_magsq);									// normal to plane containing the spheres' centers

										Vec3 abn = ij_tube.sin_theta * ij_tube.u - ij_tube.cos_theta * n;		// point of tangency in plane of a, b, and n
										Vec3 acn = ik_tube.sin_theta * ik_tube.u - ik_tube.cos_theta * n;		// point of tangency in plane of a, c, and n

										Vec3 normal_1 = Vec3::Normalize(Vec3::Cross(abn, acn));					// normal vectors to first plane
										Vec3 normal_2 = normal_1 - n * (2.0f * Vec3::Dot(normal_1, n));			// reflect normal vector across the plane containing the centerpoints

										PlanePart plane_parts[] =
										{
											PlanePart(Plane::FromPositionNormal(i_center + normal_1 * i_radius, normal_1)),
											PlanePart(Plane::FromPositionNormal(i_center + normal_2 * i_radius, normal_2))
										};

										for(unsigned char m = 0; m < 2; ++m)
										{
											PlanePart& pp = plane_parts[m];

											// in some odd cases it may not actually be possible to place a plane tangent to 3 spheres
											if(fabs(fabs(pp.plane.PointDistance(i_center)) - i_radius) > 0.0001f || fabs(fabs(pp.plane.PointDistance(j_center)) - j_radius) > 0.0001f || fabs(fabs(pp.plane.PointDistance(k_center)) - k_radius) > 0.0001f)
												continue;

											pp.planes.push_back(Plane::FromTriangleVertices(i_center, j_center, i_center + pp.plane.normal));
											pp.planes.push_back(Plane::FromTriangleVertices(j_center, k_center, j_center + pp.plane.normal));
											pp.planes.push_back(Plane::FromTriangleVertices(k_center, i_center, k_center + pp.plane.normal));

											Vec3 center = (i_center + j_center + k_center) / 3.0f;

											for(vector<Plane>::iterator iter = pp.planes.begin(); iter != pp.planes.end(); ++iter)
												if(iter->PointDistance(center) < 0)
													*iter = Plane::Reverse(*iter);

											tube_tried_planes[i_n + j] = true;
											tube_tried_planes[i_n + k] = true;
											tube_tried_planes[j_n + k] = true;

											unsigned int index = (i * n_sq + j * num_spheres + k) * 2 + m;

											sphere_planes[index] = new PlanePart(pp);
											planes_valid[index] = true;
										}
									}
								}
							}
						}
					}
				}

				// detect and discard unworthy planes (ones that do not contribute to the surface of the final shape)
				for(unsigned int i = 0; i < n_cubed * 2; ++i)
				{
					if(planes_valid[i])
					{
						if(PlanePart* part_p = sphere_planes[i])
						{
							PlanePart part(*part_p);

							bool worthy = true;

							// if anything sticks out farther than this plane, this plane is unworthy
							float my_dist = part.plane.offset;
							const Vec3& normal = part.plane.normal;
							for(unsigned int j = 0; j < num_spheres; ++j)
							{
								const Sphere& sphere = spheres[j].sphere;
								float dist = Vec3::Dot(normal, sphere.center) + sphere.radius;
								if(dist > my_dist + 0.000001f)
								{
									worthy = false;
									planes_valid[i] = false;

									break;
								}
							}

							if(worthy)
							{
								vector<Vec3> my_verts = part.GetVerts();

								// go through existing planes and see if this is a duplicate
								unsigned int j = 0;
								for(vector<PlanePart>::iterator jter = planes.begin(); jter != planes.end(); ++jter, ++j)
								{
									PlanePart part2(*jter);

									if(Vec3::Dot(normal, part2.plane.normal) > 0.9999f && fabs(my_dist - part2.plane.offset) < 0.000001f)
									{
										// merge duplicate planes
										vector<Vec3> verts = part2.GetVerts();
										for(vector<Vec3>::iterator iter = my_verts.begin(); iter != my_verts.end(); ++iter)
											verts.push_back(*iter);

										vector<Plane> possible_planes;
										for(vector<Vec3>::iterator iter = verts.begin(); iter != verts.end(); ++iter)
											for(vector<Vec3>::iterator kter = verts.begin(); kter != verts.end(); ++kter)
												if(kter != iter)
												{
													Vec3 dx = *kter - *iter;
													if(dx.ComputeMagnitudeSquared() > 0.0000001f)
														possible_planes.push_back(Plane::FromTriangleVertices(*iter, *kter, *iter + normal));
												}

										vector<Plane> nu_planes;

										for(vector<Plane>::iterator iter = possible_planes.begin(); iter != possible_planes.end(); ++iter)
										{
											const Plane& boundary = *iter;

											// eliminate boundary planes that aren't the farthest thing in their direction
											bool worthy2 = true;
											for(vector<Vec3>::iterator kter = verts.begin(); kter != verts.end(); ++kter)
												if(Vec3::Dot(boundary.normal, *kter) < boundary.offset - 0.0000001f)
												{
													worthy2 = false;
													break;
												}

											// also check that the boundary planes aren't equivalent
											if(worthy2)
												for(vector<Plane>::iterator kter = nu_planes.begin(); kter != nu_planes.end(); ++kter)
													if(Vec3::Dot(boundary.normal, kter->normal) > 0.9999f && fabs(boundary.offset - kter->offset) < 0.000001f)
													{
														worthy2 = false;
														break;
													}

											if(worthy2)
												nu_planes.push_back(boundary);
										}

										jter->planes = nu_planes;

										worthy = false;
										break;
									}
								}
							}

							if(worthy)
								planes.push_back(part);
						}
					}
				}

				// detect and discard unworthy tubes; if a tube generated any planes then it should have exactly 2 planes neighboring it or it is unworthy
				vector<Plane>* tube_planes_adjoining = new vector<Plane>[n_sq];
				for(unsigned int i = 0; i < n_sq; ++i)
					tube_planes_adjoining[i] = vector<Plane>();

				for(unsigned int i = 0; i < num_spheres; ++i)
					for(unsigned int j = i + 1; j < num_spheres; ++j)
						for(unsigned int k = j + 1; k < num_spheres; ++k)
							for(unsigned char m = 0; m < 2; ++m)
							{
								unsigned int plane_index = (i * n_sq + j * num_spheres + k) * 2 + m;
								if(planes_valid[plane_index])
								{
									unsigned int indices[] = { i * num_spheres + j, i * num_spheres + k, j * num_spheres + k };
									for(unsigned char n = 0; n < 3; ++n)
									{
										int index = indices[n];

										TubePart* tube = sphere_tubes[index];
										PlanePart* plane = sphere_planes[plane_index];

										tube_planes_adjoining[index].push_back(plane->plane);
									}
								}
							}

				for(unsigned int i = 0; i < n_sq; ++i)
				{
					if(TubePart* tube_p = sphere_tubes[i])
					{
						TubePart part(*tube_p);

						bool worthy = true;

						if(tube_tried_planes[i])
						{
							// remove duplicate planes before taking a count!
							vector<Plane> nu_planes_adjoining;
							for(vector<Plane>::iterator iter = tube_planes_adjoining[i].begin(); iter != tube_planes_adjoining[i].end(); ++iter)
							{
								vector<Plane>::iterator jter;
								for(jter = nu_planes_adjoining.begin(); jter != nu_planes_adjoining.end(); ++jter)
									if(Vec3::Dot(iter->normal, jter->normal) > 0.9999f && fabs(iter->offset - jter->offset) < 0.000001f)
										break;
								if(jter == nu_planes_adjoining.end())
									if(nu_planes_adjoining.size() < 2)					// if count of unique planes is ever > 2, something is fishy
										nu_planes_adjoining.push_back(*iter);
									else
									{
										DEBUG();

										worthy = false;
										break;
									}
							}

							if(worthy)
							{
								if(nu_planes_adjoining.size() == 2)
								{
									part.planes.resize(2);

									for(unsigned char j = 0; j < 2; ++j)
									{
										Plane plane = Plane::FromTriangleVertices(part.p1, part.p2, part.p1 + nu_planes_adjoining[j].normal);

										// make sure the plane is facing the correct direction
										if(Vec3::Dot(plane.normal, nu_planes_adjoining[1 - j].normal) < 0.0f)
											plane = Plane::Reverse(plane);

										part.planes.push_back(plane);
									}
								}
								else
									worthy = false;
							}
						}

						if(worthy)
						{
							tubes.push_back(part);

							spheres[i / num_spheres].planes.push_back(Plane::Reverse(part.planes[0]));
							spheres[i % num_spheres].planes.push_back(Plane::Reverse(part.planes[1]));
						}

						delete tube_p;
					}
				}

				// eliminate orphaned spheres
				if(num_spheres > 1)
				{
					vector<SpherePart> nu_spheres;

					for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
						if(!iter->planes.empty())
							nu_spheres.push_back(*iter);

					spheres = nu_spheres;
				}

				delete[] sphere_tubes;
				delete[] tube_planes_adjoining;
				delete[] tube_tried_planes;

				for(unsigned int i = 0; i < n_cubed * 2; ++i)
					if(PlanePart* p = sphere_planes[i])
						delete p;
				delete[] sphere_planes;
				delete[] planes_valid;

				//OutputParts();
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

		void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
		{
			static const Vec3 red(1, 0, 0), green(0, 1, 0), blue(0, 0, 1), white(1, 1, 1);

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
					farthest = Vec3::Dot(dir, from_eye) / Vec3::Dot(from_eye, forward);
				}

				void MaybeSetFarthestExtent(const Vec3& dir, const Vec3& eye, const Vec3& forward, float& farthest, Vec3& pos) const
				{
					Vec3 on_plane = dir - wn * Vec3::Dot(dir, wn);
					float mag = on_plane.ComputeMagnitude(), inv_mag = 1.0f / mag;
					
					Vec3 temp_pos = wc + on_plane * (wr * inv_mag);

					Vec3 from_eye = temp_pos - eye;
					float extent = Vec3::Dot(dir, from_eye) / Vec3::Dot(from_eye, forward);
					if(extent > farthest)
					{
						pos = temp_pos;
						farthest = extent;
					}
				}
			};

			bool any = false;

			vector<Circle> circles;
			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				if(!any && renderer->camera->CheckSphereVisibility(Sphere(model_mat.TransformVec3_1(iter->sphere.center), iter->sphere.radius)))
					any = true;
				Circle circle = Circle(eye, model_mat, iter->sphere);
				if(circle.wr >= 0)
					circles.push_back(Circle(eye, model_mat, iter->sphere));
			}

			if(circles.empty() || !any)
				return;

			Vec3 right = renderer->camera->GetRight(), up = renderer->camera->GetUp();

			// now get the points on the silhouette edge
			const int num_edges = 32;
			const float theta_coeff = 2.0f * float(M_PI) / num_edges;

			Vec3 temp;
			for(int i = 0; i <= num_edges; ++i)
			{
				float theta = i * theta_coeff;
				Vec3 world_dir = right * -cosf(theta) + up * sinf(theta);

				float farthest;
				Vec3 cur;

				vector<Circle>::iterator iter = circles.begin();
				iter->SetFarthestExtent(world_dir, eye, forward, farthest, cur);

				for(++iter; iter != circles.end(); ++iter)
					iter->MaybeSetFarthestExtent(world_dir, eye, forward, farthest, cur);

				if(i != 0)
					renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(cur, temp, white), 1.0f));
				temp = cur;
			}
		}

		AABB GetTransformedAABB(const Mat4& xform)
		{
#if 1
			// this will produce a tighter fitting AABB than aabb.GetTransformedAABB(xform), but it may be slower (especially if there are more than 8 spheres!)
			AABB xformed_aabb;
			for(vector<SpherePart>::const_iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
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

		MassInfo ComputeMassInfo()
		{
			if(aabb.IsDegenerate())
				return MassInfo();
			else
			{
				// approximate the MultiSphereShape as a box (same as how Bullet handles them)
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
		}




		// multisphere collision functions...
		bool CollideRay(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody)
		{
			Vec3 a = ray.origin, b = ray.origin + ray.direction;
			AABB ray_aabb(Vec3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)), Vec3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)));

			if(!AABB::IntersectTest(ray_aabb, aabb))
				return false;

			ContactPoint cp;

			cp.obj_a = ibody;
			cp.obj_b = jbody;

			bool any = false;

			float t;
			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
				if(iter->RayTest(ray, cp.b, t))
					if(!any || t < time)
					{
						result = cp;
						time = t;

						any = true;
					}

			for(vector<TubePart>::iterator iter = tubes.begin(); iter != tubes.end(); ++iter)
				if(iter->RayTest(ray, cp.b, t))
					if(!any || t < time)
					{
						result = cp;
						time = t;

						any = true;
					}

			for(vector<PlanePart>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
				if(iter->RayTest(ray, cp.b, t))
					if(!any || t < time)
					{
						result = cp;
						time = t;

						any = true;
					}

			return any;
		}

		bool CollideSphere(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return false;
		}

		bool CollidePlane(const Mat4& my_xform, const Plane& plane, vector<ContactPoint>& results, RigidBody* ibody, RigidBody* jbody)
		{
			Vec3 plane_norm = plane.normal;
			float plane_offset = plane.offset;

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				Vec3 sphere_pos = my_xform.TransformVec3_1(iter->sphere.center);
				float radius = iter->sphere.radius;

				float dist = Vec3::Dot(plane_norm, sphere_pos) - plane_offset - radius;

				if(dist < 0.0f)
				{
					ContactPoint result;

					result.obj_a = ibody;
					result.obj_b = jbody;
					result.a.pos = sphere_pos + plane_norm * radius;
					result.b.pos = sphere_pos + plane_norm * (radius + dist);
					result.b.norm = plane_norm;
					result.a.norm = -plane_norm;

					results.push_back(result);
				}
			}

			return !results.empty();
		}

		bool CollideMultisphere(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			Imp& other_imp = *other->imp;
			AABB other_aabb = other_imp.GetTransformedAABB(xform);

			AABB overlap;
			if(AABB::Intersect(aabb, other_aabb, overlap))
			{
				// vector containing just the spheres (no extra stuff like cutting planes, etc.)
				vector<Sphere> my_spheres;
				my_spheres.reserve(spheres.size());

				for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
					my_spheres.push_back(iter->sphere);

				// transform spheres from the other multisphereshape into our own coordinate system
				vector<Sphere> other_spheres;
				other_spheres.reserve(other_imp.spheres.size());

				for(vector<SpherePart>::iterator iter = other_imp.spheres.begin(); iter != other_imp.spheres.end(); ++iter)
					other_spheres.push_back(Sphere(xform.TransformVec3_1(iter->sphere.center), iter->sphere.radius));

				// try to find a separating axis
				Vec3 direction;
				float score = -1;
				float search_scale = 0.6f;

				char best_test;
				Vec3 test_dir[8];

				static const float x_offsets[] = {	-1,	-1,	-1, -1,	1,	1,	1,	1 };
				static const float y_offsets[] = {	-1,	-1,	1,	1,	-1,	-1,	1,	1 };
				static const float z_offsets[] = {	-1,	1,	-1,	1,	-1,	1,	-1,	1 };

				for(char i = 0; i < 20; ++i)
				{
					float best_score;

					for(char j = 0; j < 8; ++j)
					{
						Vec3& dir = test_dir[j] = Vec3::Normalize(Vec3(
							direction.x + x_offsets[j] * search_scale,
							direction.y + y_offsets[j] * search_scale,
							direction.z + z_offsets[j] * search_scale));

						float test_score = GetMaximumExtent(dir, my_spheres) - GetMinimumExtent(dir, other_spheres);

						if(test_score < 0)							// found a separating plane? go home early
							return false;
						else
						{
							if(j == 0 || test_score < best_score)
							{
								best_test = j;
								best_score = test_score;
							}
						}
					}

					if(i != 0 && best_score >= score)
						search_scale *= 0.5f;
					else
					{
						direction = test_dir[best_test];
						score = best_score;
					}
				}

				result = ContactPoint();
				result.obj_a = ibody;
				result.obj_b = jbody;

				result.a.norm = direction;
				result.b.norm = -direction;

				Vec3 pos = overlap.GetCenterPoint();					// TODO: do this better
				Vec3 offset = direction * (score * 0.5f);
				result.a.pos = pos - offset;
				result.b.pos = pos + offset;

				return true;
			}

			return false;
		}

		bool CollideMesh(const Mat4& my_xform, const TriangleMeshShape::TriCache& tri, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			static const Sphere unit_sphere(Vec3(), 1.0f);

			// vector containing just the spheres (no extra stuff like cutting planes, etc.) transformed into the triangle's coordinate system
			vector<Sphere> my_spheres;
			my_spheres.reserve(spheres.size());

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
				my_spheres.push_back(Sphere(my_xform.TransformVec3_1(iter->sphere.center), iter->sphere.radius));

			// create 0-radius spheres from the vertices of the triangle
			vector<Sphere> other_spheres;
			other_spheres.reserve(3);

			other_spheres.push_back(Sphere(tri.a, 0.0f));
			other_spheres.push_back(Sphere(tri.b, 0.0f));
			other_spheres.push_back(Sphere(tri.c, 0.0f));

			// try to find a separating axis
			struct Scorer
			{
				const vector<Sphere>& a;
				const vector<Sphere>& b;

				bool first;

				float least;
				Vec3 direction;

				Scorer(const vector<Sphere>& a, const vector<Sphere>& b) : a(a), b(b), first(true) { }

				bool Score(const Vec3& dir)
				{
					float value = GetMaximumExtent(dir, a) - GetMinimumExtent(dir, b);

					if(first)
					{
						least = value;
						direction = dir;
						first = false;
					}
					else
					{
						least = value;
						direction = dir;
					}

					return value <= 0;
				}
			} scorer(my_spheres, other_spheres);

			// my spheres ...
			for(vector<Sphere>::iterator iter = my_spheres.begin(); iter != my_spheres.end(); ++iter)
			{
				const Vec3& s = iter->center;

				Vec3 sa = s - tri.a;
				Vec3 sb = s - tri.b;
				Vec3 sc = s - tri.c;

				// ... vs. triangle's spheres (verts
				if(scorer.Score(-sa))	{ return false; }
				if(scorer.Score(sa))	{ return false; }
				if(scorer.Score(-sb))	{ return false; }
				if(scorer.Score(sb))	{ return false; }
				if(scorer.Score(-sc))	{ return false; }
				if(scorer.Score(sc))	{ return false; }

				// ... vs. triangle's tubes (edges)
				Vec3 absn = Vec3::Cross(tri.ab, sa);
				Vec3 absnabn = Vec3::Normalize(Vec3::Cross(tri.ab, absn));
				if(scorer.Score(absnabn))	{ return false; }
				if(scorer.Score(-absnabn))	{ return false; }

				Vec3 bcsn = Vec3::Cross(tri.bc, sb);
				Vec3 bcsnbcn = Vec3::Normalize(Vec3::Cross(tri.bc, bcsn));
				if(scorer.Score(bcsnbcn))	{ return false; }
				if(scorer.Score(-bcsnbcn))	{ return false; }

				Vec3 casn = Vec3::Cross(-tri.ac, sc);
				Vec3 casncan = Vec3::Normalize(Vec3::Cross(-tri.ac, casn));
				if(scorer.Score(casncan))	{ return false; }
				if(scorer.Score(-casncan))	{ return false; }
			}
			
			// my tubes ...
			for(vector<TubePart>::iterator iter = tubes.begin(); iter != tubes.end(); ++iter)
			{
				Vec3 p1 = my_xform.TransformVec3_1(iter->p1);
				Vec3 p2 = my_xform.TransformVec3_1(iter->p2);
				Vec3 u = my_xform.TransformVec3_0(iter->u);
				Vec3 u_cos_theta = u * iter->cos_theta;
				float sin_theta = iter->sin_theta;
				Plane my_plane = Plane(u, sin_theta);

				// ... vs. triangle's spheres (verts)
				Vec3 atntn = Vec3::Normalize(Vec3::Cross(u, Vec3::Cross(u, tri.a - p1)));
				if(scorer.Score(u_cos_theta + atntn * sin_theta)) { return false; }
				if(scorer.Score(u_cos_theta - atntn * sin_theta)) { return false; }

				Vec3 btntn = Vec3::Normalize(Vec3::Cross(u, Vec3::Cross(u, tri.b - p1)));
				if(scorer.Score(u_cos_theta + btntn * sin_theta)) { return false; }
				if(scorer.Score(u_cos_theta - btntn * sin_theta)) { return false; }

				Vec3 ctntn = Vec3::Normalize(Vec3::Cross(u, Vec3::Cross(u, tri.c - p1)));
				if(scorer.Score(u_cos_theta + ctntn * sin_theta)) { return false; }
				if(scorer.Score(u_cos_theta - ctntn * sin_theta)) { return false; }

				// ... vs. triangle's tubes (edges)				
				Line abtl;
				if(Plane::Intersect(Plane(tri.ab * tri.inv_len_ab, 0.0f), my_plane, abtl))
				{
					Ray ray; ray.origin = abtl.origin; ray.direction = abtl.direction;
					float first, second;
					if(Util::RaySphereIntersect(ray, unit_sphere, first, second))
					{
						if(scorer.Score(ray.origin + ray.direction * first))	{ return false; }
						if(scorer.Score(ray.origin + ray.direction * second))	{ return false; }
					}
				}
				
				Line bctl;
				if(Plane::Intersect(Plane(tri.bc * tri.inv_len_bc, 0.0f), my_plane, bctl))
				{
					Ray ray; ray.origin = bctl.origin; ray.direction = bctl.direction;
					float first, second;
					if(Util::RaySphereIntersect(ray, unit_sphere, first, second))
					{
						if(scorer.Score(ray.origin + ray.direction * first))	{ return false; }
						if(scorer.Score(ray.origin + ray.direction * second))	{ return false; }
					}
				}

				Line catl;
				if(Plane::Intersect(Plane(-tri.ac * tri.inv_len_ca, 0.0f), my_plane, catl))
				{
					Ray ray; ray.origin = catl.origin; ray.direction = catl.direction;
					float first, second;
					if(Util::RaySphereIntersect(ray, unit_sphere, first, second))
					{
						if(scorer.Score(ray.origin + ray.direction * first))	{ return false; }
						if(scorer.Score(ray.origin + ray.direction * second))	{ return false; }
					}
				}
			}

			// my planes
			for(vector<PlanePart>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
				if(scorer.Score(my_xform.TransformVec3_0(iter->plane.normal))) { return false; }

			// triangle's planes
			if(scorer.Score(tri.plane.normal))	{ return false; }
			if(scorer.Score(-tri.plane.normal))	{ return false; }

			

			result.obj_a = ibody;
			result.obj_b = jbody;
			result.b.norm = tri.plane.normal;
			result.a.norm = -result.b.norm;
			
			float best;
			for(unsigned int i = 0; i < my_spheres.size(); ++i)
			{
				const Sphere& sphere = my_spheres[i];
				float dist = Vec3::Dot(sphere.center, scorer.direction) + sphere.radius;
				if(i == 0 || best < dist)
				{
					best = dist;
					result.a.pos = sphere.center + scorer.direction * sphere.radius;
				}
			}

			result.b.pos = result.a.pos + result.b.norm * (Vec3::Dot(result.a.pos, result.b.norm) - tri.plane.offset);

			return true;
		}


		// i/o functions
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


		// static misc. utility stuff
		static float GetMaximumExtent(const Vec3& direction, const vector<Sphere>& spheres)
		{
			vector<Sphere>::const_iterator iter = spheres.begin();

			float maximum = Vec3::Dot(direction, iter->center) + iter->radius;
			++iter;

			while(iter != spheres.end())
			{
				maximum = max(maximum, Vec3::Dot(direction, iter->center) + iter->radius);
				++iter;
			}

			return maximum;
		}

		static float GetMinimumExtent(const Vec3& direction, const vector<Sphere>& spheres)
		{
			vector<Sphere>::const_iterator iter = spheres.begin();

			float minimum = Vec3::Dot(direction, iter->center) - iter->radius;
			++iter;

			while(iter != spheres.end())
			{
				minimum = min(minimum, Vec3::Dot(direction, iter->center) - iter->radius);
				++iter;
			}

			return minimum;
		}
	};




	/*
	 * MultiSphereShape methods
	 */
	MultiSphereShape::MultiSphereShape() : CollisionShape(ST_MultiSphere), imp(new Imp()) { }
	MultiSphereShape::MultiSphereShape(Sphere* spheres, unsigned int count) : CollisionShape(ST_MultiSphere), imp(new Imp(spheres, count)) { }

	void MultiSphereShape::InnerDispose() { delete imp; imp = NULL; }

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori) { imp->DebugDraw(renderer, pos, ori); }

	AABB MultiSphereShape::GetTransformedAABB(const Mat4& xform) { return imp->GetTransformedAABB(xform); }

	MassInfo MultiSphereShape::ComputeMassInfo() { return imp->ComputeMassInfo(); }

	bool MultiSphereShape::CollideRay(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody) { return imp->CollideRay(ray, result, time, ibody, jbody); }
	bool MultiSphereShape::CollidePlane(const Mat4& my_xform, const Plane& plane, vector<ContactPoint>& results, RigidBody* ibody, RigidBody* jbody) { return imp->CollidePlane(my_xform, plane, results, ibody, jbody); }
	bool MultiSphereShape::CollideSphere(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollideSphere(sphere, result, ibody, jbody); }
	bool MultiSphereShape::CollideMultisphere(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollideMultisphere(xform, other, result, ibody, jbody); }
	bool MultiSphereShape::CollideMesh(const Mat4& my_xform, const TriangleMeshShape::TriCache& tri, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollideMesh(my_xform, tri, result, ibody, jbody); }

	AABB MultiSphereShape::GetAABB() { return imp->aabb; }

	void MultiSphereShape::Write(ostream& stream) { imp->Write(stream); }
	unsigned int MultiSphereShape::Read(istream& stream) { return imp->Read(stream); }
}
