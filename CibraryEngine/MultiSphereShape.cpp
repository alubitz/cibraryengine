#include "StdAfx.h"
#include "MultiSphereShape.h"

#include "Physics.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "TriangleMeshShape.h"

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

			TubePart(Vec3 p1, Vec3 p2, float r1, float r2) : Part(), p1(p1), p2(p2), r1(r1), r2(r2)
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

		struct GridNode		// caches what points are inside/outside of the multisphereshape
		{
			bool solid;
			Vec3 normal;
		};
		GridNode* grid;
		unsigned int grid_sx, grid_sy, grid_sz, grid_x_span;
		Vec3 xyz_to_grid_scalers;

		Imp() : spheres(), tubes(), planes(), aabb(), grid(NULL) { }
		Imp(Sphere* spheres, unsigned int count) : grid(NULL) { Init(spheres, count); }

		~Imp() { if(grid != NULL) { delete[] grid; grid = NULL; } }

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

						TubePart tube(p1, p2, r1, r2);

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
								if(dist > my_dist + 0.0000001f)
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

				BuildGrid();
			}
			else
				aabb = AABB();
		}

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
				ss << endl << "\tTube: p1 = (" << p1.x << ", " << p1.y << ", " << p1.z << "), p2 = (" << p2.x << ", " << p2.y << ", " << p2.z << "), r1 = " << iter->r1 << ", r2 = " << iter->r2 << ", theta = " << atan2(iter->sin_theta, iter->cos_theta) << endl;
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

				renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(use_pos - xr, use_pos + xr, white), 1.0f));
				renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(use_pos - yr, use_pos + yr, white), 1.0f));
				renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(use_pos - zr, use_pos + zr, white), 1.0f));
			}
		}

		AABB GetTransformedAABB(const Mat4& xform)
		{
#if 0
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
			// faster computation, but the result is bigger than the results of both the above implementation and AABB::GetTransformedAABB

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

		void BuildGrid()
		{
			// TODO: make the way the dimensions are determined dynamic
			grid_sx = 40;
			grid_sy = 40;
			grid_sz = 40;
			grid_x_span = grid_sy * grid_sz;

			grid = new GridNode[grid_sx * grid_sy * grid_sz];

			Vec3 increment = (aabb.max - aabb.min);

			xyz_to_grid_scalers.x = grid_sx / increment.x;
			xyz_to_grid_scalers.y = grid_sy / increment.y;
			xyz_to_grid_scalers.z = grid_sz / increment.z;

			increment.x /= float(grid_sx - 1);
			increment.y /= float(grid_sy - 1);
			increment.z /= float(grid_sz - 1);

			unsigned int x_step = grid_sy * grid_sz;

			GridNode* grid_ptr = grid;
			for(unsigned int x = 0; x < grid_sx; ++x)
				for(unsigned int y = 0; y < grid_sy; ++y)
					for(unsigned int z = 0; z < grid_sz; ++z)
					{
						Vec3 pos = Vec3(aabb.min.x + x * increment.x, aabb.min.y + y * increment.y, aabb.min.z + z * increment.z);

						GridNode node = ComputeGridNode(pos);
						*(grid_ptr++) = node;
					}
		}

		GridNode LookUpGridNode(const Vec3& point) const
		{
			Vec3 xyz = point - aabb.min;

			xyz.x *= xyz_to_grid_scalers.x;
			xyz.y *= xyz_to_grid_scalers.y;
			xyz.z *= xyz_to_grid_scalers.z;

			int x = max(0, min(int(grid_sx) - 1, (int)floor(xyz.x)));
			int y = max(0, min(int(grid_sy) - 1, (int)floor(xyz.y)));
			int z = max(0, min(int(grid_sz) - 1, (int)floor(xyz.z)));

			return grid[x * grid_x_span + y * grid_sz + z];
		}

		GridNode ComputeGridNode(const Vec3& point) const
		{
			GridNode result;

			// find out if the point is inside any spheres
			for(vector<SpherePart>::const_iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				if(iter->IsPointRelevant(point))
				{
					const Sphere& sphere = iter->sphere;

					Vec3 dx = point - sphere.center;
					float dmag_sq = dx.ComputeMagnitudeSquared();
					if(dmag_sq <= sphere.radius * sphere.radius)
					{
						float dmag = sqrtf(dmag_sq);

						result.solid = true;
						result.normal = dx / dmag;

						return result;
					}
				}
			}

			// find out if the point is inside any tubes
			for(vector<TubePart>::const_iterator iter = tubes.begin(); iter != tubes.end(); ++iter)
			{
				if(iter->IsPointRelevant(point))
				{
					const Vec3& u = iter->u;
					const Vec3& p1 = iter->p1;

					Vec3 q = point - p1;

					float qu = Vec3::Dot(q, u);
					float dr = iter->r2 - iter->r1;
					float r = iter->r1 + qu * iter->inv_dmag * dr;

					Vec3 from_axis = q - u * qu;

					if(from_axis.ComputeMagnitudeSquared() <= r * r)
					{
						result.solid = true;
						result.normal = Vec3::Normalize(from_axis) * iter->sin_theta + u * iter->cos_theta;

						return result;
					}
				}
			}

			// to get the correct normal vector we want to find the plane this point is closest to
			float best_dist;
			result.solid = false;

			for(vector<PlanePart>::const_iterator iter = planes.begin(); iter != planes.end(); ++iter)
			{
				if(iter->IsPointRelevant(point))
				{
					float dist = iter->plane.PointDistance(point);
					if(dist <= 0.0f && (!result.solid || dist > best_dist))
					{
						best_dist = dist;
						result.solid = true;
						result.normal = iter->plane.normal;
					}
				}
			}

			// either it's behind a plane, or it's in space... either way, the data is ready
			return result;
		}

		bool ContainsPoint(const Vec3& point) const { return LookUpGridNode(point).solid; }

		bool CollisionCheck(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody)
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

		bool CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			// TODO: implement this
			return false;
		}

		bool CollisionCheck(const Mat4& my_xform, const Plane& plane, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			bool any = false;

			Vec3 center;
			float weight = 0.0f;

			Vec3 plane_norm = plane.normal;
			float plane_offset = plane.offset;

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				Vec3 sphere_pos = my_xform.TransformVec3_1(iter->sphere.center);
				float radius = iter->sphere.radius;

				float dist = Vec3::Dot(plane_norm, sphere_pos) - plane_offset - radius;

				if(dist < 0.0f)
				{
					float cur_w = -dist;
					weight += cur_w;
					center += (sphere_pos - plane_norm * (Vec3::Dot(sphere_pos, plane_norm) - plane_offset)) * cur_w;

					any = true;
				}
			}

			if(any)
			{
				center /= weight;

				result = ContactPoint();
				result.obj_a = ibody;
				result.obj_b = jbody;
				result.a.pos = center;
				result.b.pos = center;
				result.b.norm = plane_norm;
				result.a.norm = -plane_norm;

				return true;
			}
			return false;
		}

		bool CollisionCheck(const Mat4& xform, const Mat4& inv_xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			AABB other_aabb = other->imp->aabb.GetTransformedAABB(xform);

			AABB overlap;
			if(AABB::Intersect(aabb, other_aabb, overlap))
			{
				const int steps = 8;			// adjust this to change precision/speed of multisphere-multisphere collisions
				Vec3 increment = (overlap.max - overlap.min) / float(steps);
				Vec3 start = overlap.min + increment * 0.5f;

				Vec3 pos_accum, normal_accum;	
				unsigned int weight = 0;

				Vec3 pos = start;
				int x, y, z;
				for(x = 0; x < steps; ++x, pos.x += increment.x)
					for(y = 0, pos.y = start.y; y < steps; ++y, pos.y += increment.y)
						for(z = 0, pos.z = start.z; z < steps; ++z, pos.z += increment.z)
						{
							GridNode a = LookUpGridNode(pos);
							if(a.solid)
							{
								Vec3 inv_xformed = inv_xform.TransformVec3_1(pos);

								GridNode b = other->imp->LookUpGridNode(inv_xformed);
								if(b.solid)
								{
									pos_accum += pos;
									normal_accum += Vec3::Normalize(a.normal - xform.TransformVec3_0(b.normal));
									++weight;
								}
							}
						}

				if(weight)
				{
					pos = pos_accum / float(weight);
					Vec3 normal = Vec3::Normalize(normal_accum);

					result = ContactPoint();
					result.obj_a = ibody;
					result.obj_b = jbody;
					result.a.pos = pos;
					result.b.pos = pos;

					result.a.norm = normal;
					result.b.norm = -normal;

					return true;
				}
			}

			return false;
		}

		bool CollisionCheck(const Mat4& my_xform, const Mat4& inv_xform, const AABB& xformed_aabb, const TriangleMeshShape::TriCache& tri, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			Vec3 dim = xformed_aabb.max - xformed_aabb.min;

			static const int steps = 5;				// adjust this to change precision/speed of multisphere-mesh collisions
			static const float coeff = 1.0f / float(steps - 1);
			Vec3 increment = dim * coeff;

			unsigned int weight = 0;
			Vec3 center;

			// figure out which of the cardinal axes the normal vector most closely matches (x = 0, y = 1, z = 2)
			int n_axis = fabs(tri.plane.normal.x) > fabs(tri.plane.normal.y) ? fabs(tri.plane.normal.x) > fabs(tri.plane.normal.z) ? 0 : 2 : fabs(tri.plane.normal.y) > fabs(tri.plane.normal.z) ? 1 : 2;
			
			Ray ray;
			ray.direction = n_axis == 0 ? Vec3(dim.x, 0, 0) : n_axis == 1 ? Vec3(0, dim.y, 0) : Vec3(0, 0, dim.z);
			ray.origin = xformed_aabb.min;

			// iterate on the other two cardinal axes, and find the appropriate height on the normal-ish axis
			for(int i = 0; i < steps; ++i)
			{
				switch(n_axis)
				{
					case 0:
						ray.origin.y += increment.y;
						ray.origin.z = xformed_aabb.min.z;
						break;
					case 1:
						ray.origin.x += increment.x;
						ray.origin.z = xformed_aabb.min.z;
						break;
					case 2:
						ray.origin.x += increment.x;
						ray.origin.y = xformed_aabb.min.y;
						break;
				}

				for(int j = 0; j < steps; ++j)
				{
					switch(n_axis)
					{
						case 0:
							ray.origin.z += increment.z;
							break;
						case 1:
							ray.origin.z += increment.z;
							break;
						case 2:
							ray.origin.y += increment.y;
							break;
					}
					
					Vec3 pos = ray.origin + ray.direction * Util::RayPlaneIntersect(ray, tri.plane);

					float u = Vec3::Dot(tri.p, pos) - tri.u_offset;
					float v = Vec3::Dot(tri.q, pos) - tri.v_offset;
					if(u >= 0 && v >= 0 && u + v <= 1)
					{
						Vec3 my_pos = inv_xform.TransformVec3_1(pos);
						if(aabb.ContainsPoint(my_pos) && LookUpGridNode(my_pos).solid)
						{
							++weight;
							center += pos;
						}
					}
				}
			}

			if(weight)
			{
				center /= float(weight);

				result.obj_a = ibody;
				result.obj_b = jbody;
				result.a.pos = result.b.pos = center;
				result.b.norm = tri.plane.normal;
				result.a.norm = -result.b.norm;

				return true;
			}

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
	MultiSphereShape::MultiSphereShape() : CollisionShape(ST_MultiSphere), imp(new Imp()) { }
	MultiSphereShape::MultiSphereShape(Sphere* spheres, unsigned int count) : CollisionShape(ST_MultiSphere), imp(new Imp(spheres, count)) { }

	void MultiSphereShape::InnerDispose() { delete imp; imp = NULL; }

	void MultiSphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori) { imp->DebugDraw(renderer, pos, ori); }

	AABB MultiSphereShape::GetTransformedAABB(const Mat4& xform) { return imp->GetTransformedAABB(xform); }

	MassInfo MultiSphereShape::ComputeMassInfo() { return imp->ComputeMassInfo(); }

	bool MultiSphereShape::ContainsPoint(const Vec3& point) const { return imp->ContainsPoint(point); }

	bool MultiSphereShape::CollisionCheck(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(ray, result, time, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& my_xform, const Plane& plane, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(my_xform, plane, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(sphere, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& xform, const Mat4& inv_xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(xform, inv_xform, other, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& my_xform, const Mat4& inv_xform, const AABB& xformed_aabb, const TriangleMeshShape::TriCache& tri, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(my_xform, inv_xform, xformed_aabb, tri, result, ibody, jbody); }

	AABB MultiSphereShape::GetAABB() { return imp->aabb; }

	void MultiSphereShape::Write(ostream& stream) { imp->Write(stream); }
	unsigned int MultiSphereShape::Read(istream& stream) { return imp->Read(stream); }
}
