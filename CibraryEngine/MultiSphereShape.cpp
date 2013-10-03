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

										Plane ij_tubenormals = Plane(ij_tube.u, -ij_tube.cos_theta);
										Plane jk_tubenormals = Plane(jk_tubep->u, -jk_tubep->cos_theta);
										//Plane ki_tubenormals = Plane(ik_tube.u, -ik_tube.cos_theta);
										Line line;
										if(Plane::Intersect(ij_tubenormals, jk_tubenormals, line))
										{
											float first, second;
											if(Util::RaySphereIntersect(Ray(line), Sphere(Vec3(), 1.0f), first, second))
											{
												Vec3 normal_1 = line.origin + line.direction * first;
												Vec3 normal_2 = line.origin + line.direction * second;

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

#if 0
				stringstream ss;
				ss << "here's a shape:" << endl;

				for(unsigned int i = 0; i < num_spheres; ++i)
				{
					const Sphere& s = spheres[i].sphere;
					const Vec3& c = s.center;

					ss << "\tsphere[" << i << "] center = (" << c.x << ", " << c.y << ", " << c.z << "), radius = " << s.radius << endl;
				}

				for(unsigned int i = 0; i < num_spheres; ++i)
					for(unsigned int j = i + 1; j < num_spheres; ++j)
					{
						if(const TubePart* tube = sphere_tubes[i * num_spheres + j])
						{
							const Vec3& c1 = tube->p1;
							const Vec3& c2 = tube->p2;

							ss << "\ttube between spheres " << i << " and " << j << "; c1 = (" << c1.x << ", " << c1.y << ", " << c1.z << "), r1 = " << tube->r1 << "; c2 = (" << c2.x << ", " << c2.y << ", " << c2.z << "), r2 = " << tube->r2 << endl;
						}
					}

				for(unsigned int i = 0; i < num_spheres; ++i)
					for(unsigned int j = i + 1; j < num_spheres; ++j)
						for(unsigned int k = j + 1; k < num_spheres; ++k)
							for(unsigned char m = 0; m < 2; ++m)
							{
								unsigned int plane_index = (i * n_sq + j * num_spheres + k) * 2 + m;
								if(planes_valid[plane_index])
								{
									const PlanePart* pp = sphere_planes[plane_index];
									const Plane& plane = pp->plane;
									const Vec3& n = plane.normal;

									vector<Vec3> verts = pp->GetVerts();

									ss << "\tplane between spheres " << i << ", " << j << ", and " << k << ":" << endl;
									for(vector<Vec3>::iterator iter = verts.begin(); iter != verts.end(); ++iter)
										ss << "\t\tvertex = (" << iter->x << ", " << iter->y << ", " << iter->z << ")" << endl;
								}
							}

				Debug(ss.str());
#endif

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
									nu_planes_adjoining.push_back(*iter);
							}

							if(worthy)
							{
								unsigned int nu_planes_count = nu_planes_adjoining.size();
								if(nu_planes_count == 2 || nu_planes_count == 4)
								{
									part.planes.resize(2);		// remove any planes besides the ones from the spheres that created this tube

									Vec3 tube_center = (part.p1 + part.p2) * 0.5f;

									for(unsigned char j = 0; j < nu_planes_count; ++j)
									{
										Plane plane = Plane::FromTriangleVertices(part.p1, part.p2, part.p1 + nu_planes_adjoining[j].normal);

										// make sure the plane is facing the correct direction
										if(Vec3::Dot(plane.normal, tube_center) > plane.offset)			// TODO: double-check this
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
					renderer->objects.push_back(RenderNode(material, material->New(cur, temp), 1.0f));
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
					positions.push_back(sphere_pos + plane_norm * radius);
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

	void MultiSphereShape::InnerDispose()																{ delete imp; imp = NULL; }

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)	{ imp->DebugDraw(renderer, pos, ori); }

	AABB MultiSphereShape::GetAABB()																	{ return imp->aabb; }
	AABB MultiSphereShape::GetTransformedAABB(const Mat4& xform)										{ return imp->GetTransformedAABB(xform); }
	AABB MultiSphereShape::ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache)		{ return imp->ComputeCachedWorldAABB(xform, cache); }

	MassInfo MultiSphereShape::ComputeMassInfo()														{ return imp->ComputeMassInfo(); }

	bool MultiSphereShape::CollideRay(const Ray& ray, RayResult& result, RayCollider* collider, RigidBody* body)																								{ return imp->CollideRay(ray, result, collider, body); }
	ContactRegion* MultiSphereShape::CollidePlane(const Mat4& my_xform, const Plane& plane, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)													{ return imp->CollidePlane(my_xform, plane, collect, ibody, jbody); }
	ContactRegion* MultiSphereShape::CollideSphere(const Sphere& sphere, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)																		{ return imp->CollideSphere(sphere, collect, ibody, jbody); }
	ContactRegion* MultiSphereShape::CollideMesh(const Mat4& my_xform, vector<Sphere>& my_spheres, const TriangleMeshShape::TriCache& tri, ContactDataCollector* collect, RigidBody* ibody, RigidBody* jbody)	{ return imp->CollideMesh(my_xform, my_spheres, tri, ibody, jbody, collect); }

	void MultiSphereShape::Write(ostream& stream)														{ imp->Write(stream); }
	unsigned int MultiSphereShape::Read(istream& stream)												{ return imp->Read(stream); }




	/*
	 * MultiSphereShapeInstanceCache methods
	 */
	MultiSphereShapeInstanceCache::MultiSphereShapeInstanceCache() : spheres(), aabb() { }

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
