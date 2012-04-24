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
		struct SpherePart;
		struct TubePart;
		struct PlanePart;

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

			// TODO: revise the signatures of these functions as necessary
			virtual bool Intersect(const Part& part) const = 0;

			virtual bool IntersectSphere(const SpherePart& part) const = 0;
			virtual bool IntersectTube(const TubePart& part) const = 0;
			virtual bool IntersectPlane(const PlanePart& part) const = 0;

			// true if any part of the circle is on the correct side of all planes (from both parts)
			static bool IsCircleRelevant(const Vec3& center, const Vec3& normal, float radius, const Part& part_a, const Part& part_b)
			{
				// circle struct, because we like OOP
				struct Circle
				{
					struct Arc
					{
						float a, b;

						Arc() : a(), b() { }
						Arc(float a, float b) : a(a), b(b) { }
					};

					vector<Arc> arcs;

					Vec3 center, normal;
					float radius;

					Plane plane;

					Vec3 axis_a, axis_b;

					Circle(const Vec3& center, const Vec3& normal, float radius) : arcs(), center(center), normal(normal), radius(radius), plane(Plane::FromPositionNormal(center, normal))
					{
						arcs.push_back(Arc(float(-M_PI), float(M_PI)));

						Mat3 mat = Util::FindOrientationZEdge(normal).Transpose();
						axis_a = mat * Vec3(1, 0, 0);
						axis_b = mat * Vec3(0, 1, 0);
					}

					void Cut(const Plane& cut)
					{
						if(arcs.empty())			// nothing to do
							return;

						Line line;
						if(Plane::Intersect(plane, cut, line))
						{
							line.origin -= center;

							// transform line into the 2d coordinate system of the circle
							Vec2 origin =		Vec2(Vec3::Dot(axis_a, line.origin),	Vec3::Dot(axis_b, line.origin)		);
							Vec2 direction =	Vec2(Vec3::Dot(axis_a, line.direction),	Vec3::Dot(axis_b, line.direction)	);

							// also find out which is the "good" side
							Vec2 good_dir =		Vec2(Vec3::Dot(axis_a, cut.normal),		Vec3::Dot(axis_b, cut.normal)		);
							float good_offset = Vec2::Dot(good_dir, origin);
        
							// find where the cutting line intersects the circle
							float t1, t2;
							if(Util::SolveQuadraticFormula(direction.ComputeMagnitudeSquared(), 2.0f * Vec2::Dot(origin, direction), origin.ComputeMagnitudeSquared() - radius * radius, t1, t2))
							{
								// points of intersection
								Vec2 p1 = origin + direction * t1;
								Vec2 p2 = origin + direction * t2;

								// find the angles to those points
								float theta_1 = atan2f(p1.y, p1.x);
								float theta_2 = atan2f(p2.y, p2.x);

								if(theta_1 > theta_2)
									swap(theta_1, theta_2);

								// cut arcs that contain our angles
								vector<Arc> nu_arcs;
								for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
								{
									float a = iter->a, b = iter->b;

									bool contains_1 = a < theta_1 && b >= theta_1, contains_2 = a < theta_2 && b >= theta_2;

									if(contains_1)
									{
										nu_arcs.push_back(Arc(a, theta_1));
										
										if(contains_2)
										{
											nu_arcs.push_back(Arc(theta_1, theta_2));
											nu_arcs.push_back(Arc(theta_2, b));
										}
										else
											nu_arcs.push_back(Arc(theta_1, b));
									}
									else if(contains_2)
									{
										nu_arcs.push_back(Arc(a, theta_2));
										nu_arcs.push_back(Arc(theta_2, b));
									}
									else
										nu_arcs.push_back(*iter);
								}
								arcs = nu_arcs;
							}

							// remove arcs that are on the wrong side of the cutting plane
							vector<Arc> nu_arcs;
							for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
							{
								float theta = 0.5f * (iter->a + iter->b);
								Vec2 arc_center = Vec2(cosf(theta) * radius, sinf(theta) * radius);

								if(Vec2::Dot(arc_center, good_dir) > good_offset)
									nu_arcs.push_back(*iter);
							}
							arcs = nu_arcs;
						}
						else if(cut.PointDistance(center) < 0.0f)
							arcs.clear();
					}
				} circle(center, normal, radius);

				for(vector<Plane>::const_iterator iter = part_a.planes.begin(); iter != part_a.planes.end(); ++iter)
					circle.Cut(*iter);
				for(vector<Plane>::const_iterator iter = part_b.planes.begin(); iter != part_b.planes.end(); ++iter)
					circle.Cut(*iter);

				return !circle.arcs.empty();
			}

			// true if any part of the line is on the correct side of all planes (from both parts)
			static bool IsLineRelevant(const Line& line, const Part& part_a, const Part& part_b)
			{
				struct Arc
				{
					Ray ray;

					bool valid;

					float a, b;
					bool a_inf, b_inf;

					Arc(const Line& line) : valid(true), a_inf(true), b_inf(true)
					{
						ray.origin = line.origin;
						ray.direction = line.direction;
					}

					// assuming it's a line segment, sees if it's on the correct side of the plane
					bool IsValid(const Plane& plane) { return plane.PointDistance(ray.origin + ray.direction * (0.5f * (a + b))) > 0;}

					void Cut(const Plane& plane)
					{
						if(!valid)
							return;

						float hit = Util::RayPlaneIntersect(ray, plane);

						if(_finite(hit))		// TODO: come up with a solution that doesn't depend on msvc
						{
							if(a_inf && b_inf)
							{
								if(plane.PointDistance(ray.origin + ray.direction * (hit + 1.0f)) > 0)
								{
									a_inf = false;
									a = hit;
								}
								else
								{
									b_inf = false;
									b = hit;
								}
							}
							else if(a_inf)
							{
								if(hit <= b)
								{
									a = hit;

									if(plane.PointDistance(ray.origin + ray.direction * (0.5f * (a + b))) < 0)
										valid = false;
									else
										a_inf = false;
								}
							}
							else if(b_inf)
							{
								if(hit >= a)
								{
									b = hit;

									if(IsValid(plane))
										b_inf = false;
									else
										valid = false;
								}
							}
							else
							{
								if(hit < a || hit > b)
								{
									if(!IsValid(plane))
										valid = false;
								}
								else if(plane.PointDistance(ray.origin + ray.direction * (0.5f * (a + hit))) < 0)
									a = hit;
								else if(plane.PointDistance(ray.origin + ray.direction * (0.5f * (a + hit))) < 0)
									b = hit;
							}
						}
						else if(plane.PointDistance(ray.origin) < 0)
							valid = false;
					}
				} arc(line);

				for(vector<Plane>::const_iterator iter = part_a.planes.begin(); iter != part_a.planes.end(); ++iter)
					arc.Cut(*iter);

				for(vector<Plane>::const_iterator iter = part_b.planes.begin(); iter != part_b.planes.end(); ++iter)
					arc.Cut(*iter);

				return arc.valid;
			}
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

			bool Intersect(const Part& part) const { return part.IntersectSphere(*this); }

			bool IntersectSphere(const SpherePart& part) const
			{
				const Sphere& other_sphere = part.sphere;
				Vec3 dx = other_sphere.center - sphere.center;
				float dmag_sq = dx.ComputeMagnitudeSquared();

				float sr = sphere.radius + other_sphere.radius;
				if(dmag_sq < sr * sr)
				{
					float dmag = sqrtf(dmag_sq), inv_dmag = 1.0f / dmag;
					float rsq = sphere.radius * sphere.radius;
					float other_rsq = other_sphere.radius * other_sphere.radius;

					float x = 0.5f * (rsq - other_rsq + dmag_sq) * inv_dmag;

					Vec3 c_normal = dx * inv_dmag;
					Vec3 c_center = sphere.center + c_normal * x;
					float c_radius = sqrtf(rsq - x * x);

					return IsCircleRelevant(c_center, c_normal, c_radius, *this, part);
				}

				return false;
			}

			bool IntersectTube(const TubePart& part) const
			{
				struct Locus
				{
					struct Arc
					{
						float a, b;
						bool plus;

						Arc() : a(), b(), plus() { }
						Arc(float a, float b, bool plus) : a(a), b(b), plus(plus) { }
					};

					vector<Arc> arcs;

					const TubePart& tube;
					const Sphere& sphere;

					Vec3 dp;
					float dr;

					float dpsq, inv_dpsq;

					Vec3 axis_a, axis_b;

					Locus(const TubePart& tube, const Sphere& sphere) : arcs(), tube(tube), sphere(sphere), dp(tube.p2 - tube.p1), dr(tube.r2 - tube.r1), dpsq(dp.ComputeMagnitudeSquared()), inv_dpsq(1.0f / dpsq)
					{
						Vec3 dc = tube.p1 - sphere.center;

						Mat3 mat = Util::FindOrientationZEdge(dp).Transpose();
						axis_a = mat * Vec3(1, 0, 0);
						axis_b = mat * Vec3(0, 1, 0);

						vector<Arc> nu_arcs;			// temporarily stores arc (no sign information)

						// some things behave differently depending on whether its a cylinder or a cone...
						if(dr != 0.0f)
						{
							// it's a cone... find where the discriminant of the quadratic formula is zero (this in turn requires us to solve yet another quadratic!)

							float cdotp = Vec3::Dot(dc, dp);

							float rsq = sphere.radius * sphere.radius;

							// solving for (dc dot g) which we will later use to solve for theta
							float A = dr * dr;
							float B = 2.0f * (dr * cdotp + tube.r1 * (rsq - dpsq));
							float C = cdotp * cdotp + 2.0f * tube.r1 * dr * cdotp + (dc.ComputeMagnitudeSquared() + tube.r1 * tube.r1) * (rsq - A - dpsq);;

							float t[2];
							if(Util::SolveQuadraticFormula(A, B, C, t[0], t[1]))
							{
								list<float> angles;
								for(unsigned char i = 0; i < 2; ++i)
								{
									// solving for theta given (dc dot g), as promised
									float X = Vec3::Dot(dc, axis_a);
									float Y = Vec3::Dot(dc, axis_b);
									float Z = t[i];

									float cos_theta[2];
									if(Util::SolveQuadraticFormula(X * X + Y * Y, -2.0f * X * Z, Z * Z - Y * Y, cos_theta[0], cos_theta[1]))
									{
										angles.push_back(acosf(cos_theta[0]));
										angles.push_back(acosf(cos_theta[1]));
									}
								}
								angles.sort();

								angles.push_front(float(-M_PI));
								angles.push_back(float(M_PI));

								float prev = -100;
								for(list<float>::iterator iter = angles.begin(); iter != angles.end(); ++iter)
								{
									if(prev != -100)
										nu_arcs.push_back(Arc(prev, *iter, false));
									prev = *iter;
								}
							}
							else
								nu_arcs.push_back(Arc(float(-M_PI), float(M_PI), false));
						}
						else
						{
							// it's a cylinder
							Vec3 sphere_offset = sphere.center - tube.p1;

							Vec2 dx = Vec2(Vec3::Dot(axis_a, sphere_offset), Vec3::Dot(axis_b, sphere_offset));
							float dmag_sq = dx.ComputeMagnitudeSquared();

							float sr = sphere.radius + tube.r1;
							if(dmag_sq <= sr * sr)
							{
								float dmag = sqrtf(dmag_sq), inv_dmag = 1.0f / dmag;
								float rsq = sphere.radius * sphere.radius;
								float other_rsq = tube.r1 * tube.r1;

								float x = 0.5f * (other_rsq - rsq + dmag_sq) * inv_dmag;
								float y = sqrtf(other_rsq - x * x);
								float signed_y[] = { -y, y };

								// find where the discriminant of the quadratic formula is zero
								Vec2 u = dx * inv_dmag, r = Vec2(-u.y, u.x);
								float theta[2];
								for(unsigned char i = 0; i < 2; ++i)
								{
									Vec2 p = u * x + r * signed_y[i];
									theta[i] = atan2f(p.y, p.x);
								}

								if(theta[0] > theta[1])
									swap(theta[0], theta[1]);

								nu_arcs.push_back(Arc(float(-M_PI), theta[0], false));
								nu_arcs.push_back(Arc(theta[0], theta[1], false));
								nu_arcs.push_back(Arc(theta[1], float(M_PI), false));
							}
						}

						// initial sign check
						for(vector<Arc>::iterator iter = nu_arcs.begin(); iter != nu_arcs.end(); ++iter)
						{
							float mid_theta = 0.5f * (iter->a + iter->b);
							Vec3 g = axis_a * cosf(mid_theta) + axis_b * sinf(mid_theta);

							Vec3 dppgdr = dp + g * dr;
							Vec3 gr1pdc = g * tube.r1 + dc;

							float A = dppgdr.ComputeMagnitudeSquared();
							float B = 2.0f * Vec3::Dot(dppgdr, gr1pdc);
							float C = gr1pdc.ComputeMagnitudeSquared() - sphere.radius * sphere.radius;

							float discriminant = B * B - 4.0f * A * C;
							if(discriminant >= 0.0f)
							{
								arcs.push_back(Arc(iter->a, iter->b, false));
								arcs.push_back(Arc(iter->a, iter->b, true));
							}
						}
					}

					void Cut(const Plane& cut)
					{
						if(arcs.empty())
							return;

						vector<Arc> nu_arcs;

						// find the circle where the sphere and plane intersect
						float sphere_dist = cut.PointDistance(sphere.center);
						if(fabs(sphere_dist) < sphere.radius)
						{
							Vec3 center_c = sphere.center - cut.normal * sphere_dist;
							float radius_c = sqrtf(sphere.radius * sphere.radius - sphere_dist * sphere_dist);

							Mat3 mat = Util::FindOrientationZEdge(cut.normal).Transpose();
							Vec3 circle_a = mat * Vec3(1, 0, 0);
							Vec3 circle_b = mat * Vec3(0, 1, 0);

							Vec3 dp = tube.p2 - tube.p1;
							Vec3 dc = center_c - tube.p1;

							// solve for (g dot dc); note that g = (a cos theta + b sin theta) for the circle, not the tube!
							float temp = tube.r1 + dr * Vec3::Dot(dc, dp) * inv_dpsq;
							Vec3 dc_non_dp = dc - dp * Vec3::Dot(dc, dp) * inv_dpsq;
							float g_dot_dc = 0.5f * (temp * temp - radius_c * radius_c + dc_non_dp.ComputeMagnitudeSquared()) / radius_c;
							
							// now see what angles satisfy that
							float X = Vec3::Dot(dc, circle_a);
							float Y = Vec3::Dot(dc, circle_b);
							float Z = g_dot_dc;

							float cos_theta[2];
							if(Util::SolveQuadraticFormula(X * X + Y * Y, -2.0f * X * Z, Z * Z - Y * Y, cos_theta[0], cos_theta[1]))
							{
								float theta[2];
								for(unsigned char i = 0; i < 2; ++i)
									theta[i] = acosf(cos_theta[i]);
								if(theta[0] > theta[1])
									swap(theta[0], theta[1]);

								for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
								{
									bool contains_a = iter->a <= theta[0] && iter->b >= theta[0];
									bool contains_b = iter->a <= theta[1] && iter->b >= theta[1];

									if(contains_a)
									{
										if(contains_b)
										{
											nu_arcs.push_back(Arc(iter->a, theta[0], iter->plus));
											nu_arcs.push_back(Arc(theta[0], theta[1], iter->plus));
											nu_arcs.push_back(Arc(theta[1], iter->b, iter->plus));
										}
										else
										{
											nu_arcs.push_back(Arc(iter->a, theta[0], iter->plus));
											nu_arcs.push_back(Arc(theta[0], iter->b, iter->plus));
										}
									}
									else if(contains_b)
									{
										nu_arcs.push_back(Arc(iter->a, theta[1], iter->plus));
										nu_arcs.push_back(Arc(theta[1], iter->b, iter->plus));
									}
									else
										nu_arcs.push_back(*iter);
								}
								arcs = nu_arcs;
								nu_arcs = vector<Arc>();
							}
						}


						// go through and check whether arcs are on the correct side of the cutting plane
						for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
						{
							float mid_theta = 0.5f * (iter->a + iter->b);
							Vec3 g = axis_a * cosf(mid_theta) + axis_b * sinf(mid_theta);

							Vec3 dppgdr = dp + g * dr;
							Vec3 gr1pdc = g * tube.r1 + tube.p1 - sphere.center;
							
							float A = dppgdr.ComputeMagnitudeSquared();
							float B = 2.0f * Vec3::Dot(dppgdr, gr1pdc);
							float C = gr1pdc.ComputeMagnitudeSquared() - (sphere.radius * sphere.radius);

							float discriminant = B * B - 4.0f * A * C;
							assert(discriminant >= 0);
							
							float y = iter->plus ? 0.5f * (-B + sqrtf(discriminant)) / A : 0.5f * (-B - sqrtf(discriminant)) / A;
							Vec3 pos = (dp + g * dr) * y + tube.p1 + g * tube.r1;

							if(cut.PointDistance(pos) >= 0.0f)
								nu_arcs.push_back(*iter);							
						}
						arcs = nu_arcs;
					}
				} locus(part, sphere);

				for(vector<Plane>::const_iterator iter = planes.begin(); iter != planes.end(); ++iter)
					locus.Cut(*iter);
				for(vector<Plane>::const_iterator iter = part.planes.begin(); iter != part.planes.end(); ++iter)
					locus.Cut(*iter);

				return !locus.arcs.empty();
			}

			bool IntersectPlane(const PlanePart& part) const
			{
				const Plane& plane = part.plane;

				float dist = plane.PointDistance(sphere.center);
				if(fabs(dist) <= sphere.radius)
				{
					Vec3 c_normal = plane.normal;
					Vec3 c_center = sphere.center - c_normal * dist;
					float c_radius = sqrtf(sphere.radius * sphere.radius - dist * dist);

					return IsCircleRelevant(c_center, c_normal, c_radius, *this, part);
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
							contact.norm = Vec3::Normalize(from_axis) * cos_theta + u * sin_theta;

							time = t[i];

							return true;
						}
					}
				}

				return false;
			}

			bool Intersect(const Part& part) const { return part.IntersectTube(*this); }

			bool IntersectSphere(const SpherePart& part) const { return part.IntersectTube(*this); }

			bool IntersectTube(const TubePart& part) const
			{
				struct Locus
				{
					struct Arc
					{
						float a, b;
						bool minus, plus;

						Arc() : a(), b(), minus(), plus() { }
						Arc(float a, float b, bool minus, bool plus) : a(a), b(b), minus(minus), plus(plus) { }
					};

					vector<Arc> arcs;

					const TubePart& tube_1;
					const TubePart& tube_2;

					Vec3 axis_a, axis_b;

					Vec3 dp1, dp2, p1mp3;
					float dr1;

					float qa[3];
					float qb[4]; 
					float qc[4]; 

					Locus(const TubePart& tube_1, const TubePart& tube_2) : arcs(), tube_1(tube_1), tube_2(tube_2)
					{
						dp1 = tube_1.p2 - tube_1.p1;
						dp2 = tube_2.p2 - tube_2.p1;
						p1mp3 = tube_1.p1 - tube_2.p1;

						float dp1_magsq = dp1.ComputeMagnitudeSquared();
						float dp2_magsq = dp2.ComputeMagnitudeSquared(), inv_dp2sq = 1.0f / dp2_magsq, inv_dp2_4 = inv_dp2sq * inv_dp2sq;

						dr1 = tube_1.r2 - tube_1.r1;
						float dr1_sq = dr1 * dr1;
						float dr2 = tube_2.r2 - tube_2.r1, dr2_sq = dr2 * dr2;

						float dp1dp2 = Vec3::Dot(dp1, dp2);
						float dp2sqpdr2sq = dp2_magsq + dr2_sq;

						float dp2dp1mp3 = Vec3::Dot(dp2, p1mp3);

						qa[0] = -dr1_sq * dp2sqpdr2sq * inv_dp2_4;
						qa[1] = -2.0f * tube_1.r1 * dp1dp2 * inv_dp2_4 * dp2sqpdr2sq;
						qa[2] = dp1_magsq + dr1_sq - dp2sqpdr2sq * dp1dp2 * dp1dp2 * inv_dp2_4;

						qb[0] = -2.0f * tube_1.r1 * dr1 * dp2sqpdr2sq * inv_dp2_4;
						qb[1] = -2.0f * (dp2sqpdr2sq * tube_1.r2 * inv_dp2_4 + tube_2.r1 * dr1 * dr2 * inv_dp2sq);
						qb[2] = 2.0f * dr1;
						qb[3] = 2.0f * (Vec3::Dot(dp1, p1mp3) - inv_dp2_4 * (dp1dp2 + dp2dp1mp3) - tube_2.r1 * dr2 * dp1dp2 * inv_dp2sq);

						qc[0] = tube_1.r1 * tube_1.r1;
						qc[1] = 2.0f * (dp2sqpdr2sq * tube_1.r1 * dp2dp1mp3) * inv_dp2_4 - tube_2.r1 * dr2 * tube_1.r1 * inv_dp2sq;
						qc[2] = 2.0f * tube_1.r1;
						qc[3] = tube_1.r1 * tube_1.r1 + p1mp3.ComputeMagnitudeSquared() + dp2sqpdr2sq * dp2dp1mp3 * dp2dp1mp3 * inv_dp2_4 - 2.0f * tube_1.r1 * dr2 * dp2dp1mp3 * inv_dp2sq - tube_2.r1 * tube_2.r1;

						Mat3 rm = Util::FindOrientationZEdge(dp1).Transpose();
						axis_a = rm * Vec3(1, 0, 0);
						axis_b = rm * Vec3(0, 1, 0);

						int ops = 0;
						list<float> zeros;

						FindZeros(zeros);
						zeros.sort();

						float prev = -100;
						for(list<float>::iterator iter = zeros.begin(); iter != zeros.end(); ++iter)
						{
							float cur = *iter;
							if(prev != -100)
							{
								float mid_theta = 0.5f * (prev + cur);
								if(EvaluateDiscriminant(mid_theta) > 0.0f)
									arcs.push_back(Arc(prev, cur, true, true));
							}
							prev = cur;
						}
					}

					float EvaluateDiscriminant(float theta)
					{
						float A, B, C;
						GetQuadraticCoefficients(theta, A, B, C);

						return B * B - 4.0f * A * C;
					}

					void FindZeros(list<float>& zeros) { float val = EvaluateDiscriminant(float(-M_PI)); RecursiveFindZeros(float(-M_PI), float(M_PI), val, val, 0, zeros); }
					void RecursiveFindZeros(float x1, float x2, float y1, float y2, int depth, list<float>& zeros)
					{
						if(depth > 10)
							return;

						if(depth <= 3)
						{
							float mid = 0.5f * (x1 + x2);
							float value = EvaluateDiscriminant(mid);

							RecursiveFindZeros(x1, mid, y1, value, depth + 1, zeros);
							RecursiveFindZeros(mid, x2, value, y2, depth + 1, zeros);
						}
						else if(y1 * y2 < 0.0f)
						{
							float guess = (x1 * y2 - x2 * y1) / (y2 - y1);
							float value = EvaluateDiscriminant(guess);

							if(fabs(value) < 0.001f)
								zeros.push_back(guess);
							else
							{
								RecursiveFindZeros(x1, guess, y1, value, depth + 1, zeros);
								RecursiveFindZeros(guess, x2, value, y2, depth + 1, zeros);
							}
						}
					}

					void GetQuadraticCoefficients(float theta, float& A, float& B, float& C, Vec3& radius_vector)
					{
						radius_vector = GetRadiusVector(theta);

						float z1 = Vec3::Dot(radius_vector, dp2), z2 = Vec3::Dot(radius_vector, p1mp3);
						float z1sq = z1 * z1;

						A = qa[0] * z1sq + qa[1] * z1 + qa[2];
						B = qb[0] * z1sq + qb[1] * z1 + qb[2] * z2 + qb[3];
						C = qc[0] * z1sq + qc[1] * z1 + qc[2] * z2 + qc[3];
					}
					void GetQuadraticCoefficients(float theta, float& A, float& B, float& C) { Vec3 dummy_var; GetQuadraticCoefficients(theta, A, B, C, dummy_var); }

					Vec3 GetRadiusVector(float theta) { return axis_a * cosf(theta) + axis_b * sinf(theta); }

					void Cut(const Plane& cut)
					{
						if(arcs.empty())			// nothing to do here
							return;

						// TODO: find where the plane and two tubes meet

						vector<Arc> nu_arcs;
						if(false)					// if arcs need cutting...
						{
							for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
							{
								if(false)
								{
									// TODO: cut arc
								}
								else
									nu_arcs.push_back(*iter);
							}

							arcs = nu_arcs;
							nu_arcs = vector<Arc>();
						}

						// now discard arcs which are on the wrong side of the cutting plane
						for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
						{
							float mid_theta = 0.5f * (iter->a + iter->b);
						
							float A, B, C;
							Vec3 g;
							GetQuadraticCoefficients(mid_theta, A, B, C, g);

							float root = sqrtf(B * B - 4.0f * A * C);
							float a_coeff = 0.5f / A;
							
							Vec3 dp1pgdr1 = dp1 + g * dr1;
							Vec3 p1pgr1 = tube_1.p1 + g * tube_1.r1;

							if(iter->minus)
							{
								float y = (-B - root) * a_coeff;
								Vec3 pos = dp1pgdr1 * y + p1pgr1;
								
								if(cut.PointDistance(pos) < 0.0f)
									iter->minus = false;
							}

							if(iter->plus)
							{
								float y = (-B + root) * a_coeff;
								Vec3 pos = dp1pgdr1 * y + p1pgr1;
								
								if(cut.PointDistance(pos) < 0.0f)
									iter->plus = false;
							}

							if(iter->plus || iter->minus)
								nu_arcs.push_back(*iter);
						}
						arcs = nu_arcs;
					}
				} locus(*this, part);

				for(vector<Plane>::const_iterator iter = planes.begin(); iter != planes.end(); ++iter)
					locus.Cut(*iter);
				for(vector<Plane>::const_iterator iter = part.planes.begin(); iter != part.planes.end(); ++iter)
					locus.Cut(*iter);

				return !locus.arcs.empty();
			}
			bool IntersectPlane(const PlanePart& part) const
			{
				struct Locus
				{
					struct Arc
					{
						float a, b;
						Arc() : a(), b() { }
						Arc(float a, float b) : a(a), b(b) { }
					};
					vector<Arc> arcs;

					const Plane& plane;
					const TubePart& tube;

					Vec3 axis_a, axis_b;

					Locus(const Plane& plane, const TubePart& tube) : arcs(), plane(plane), tube(tube)
					{
						const Vec3& normal = plane.normal;

						float dr = tube.r2 - tube.r1;
						Vec3 dp = tube.p2 - tube.p1;

						Mat3 mat = Util::FindOrientationZEdge(dp).Transpose();
						axis_a = mat * Vec3(1, 0, 0);
						axis_b = mat * Vec3(0, 1, 0);

						// some stuff only applies to cones (i.e. tubes where the end radii are unequal)
						if(dr != 0.0f)
						{
							// find out if shape is a hyperbola
							float X = Vec3::Dot(axis_a, normal);
							float Y = Vec3::Dot(axis_b, normal);
							float Z = Vec3::Dot(dp, normal) / dr;

							float cos1, cos2;
							if(Util::SolveQuadraticFormula(X * X + Y * Y, -X * Z, Z * Z - Y * Y, cos1, cos2))
							{
								// shape is a hyperbola! asymptotes are at these angles...
								float theta1 = acosf(cos1);
								float theta2 = acosf(cos2);

								if(theta2 < theta1)
									swap(theta2, theta1);

								arcs.push_back(Arc(float(-M_PI), theta1));
								arcs.push_back(Arc(theta1, theta2));
								arcs.push_back(Arc(theta2, float(M_PI)));
							}
							else
								arcs.push_back(Arc(float(-M_PI), float(M_PI)));			// shape is not a hyperbola

							// TODO: check that parabolas are dealt with properly
						}
						else
						{
							if(Vec3::Dot(dp, normal) != 0)
								arcs.push_back(Arc(float(-M_PI), float(M_PI)));
							else
							{
								// TODO: maybe there are lines
							}
						}
					}

					void Cut(const Plane& cut)
					{
						if(arcs.empty())
							return;

						vector<Arc> nu_arcs;

						// find out how the cutting plane cuts the existing arcs
						Line line;
						if(Plane::Intersect(plane, cut, line))
						{
							Ray ray;
							ray.origin = line.origin;
							ray.direction = line.direction;

							float t[2];
							if(tube.RayTestInfinite(ray, t[0], t[1]))
							{
								float theta[2];
								for(unsigned char i = 0; i < 2; ++i)
								{
									Vec3 hit = ray.origin + ray.direction * t[i];
									hit -= tube.p1;

									theta[i] = atan2f(Vec3::Dot(hit, axis_b), Vec3::Dot(hit, axis_a));
								}

								if(theta[1] < theta[0])
									swap(theta[0], theta[1]);

								for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
								{
									bool contains_a = theta[0] >= iter->a && theta[0] <= iter->b, contains_b = theta[1] >= iter->a && theta[1] <= iter->b;

									if(contains_a)
									{
										if(contains_b)
										{
											nu_arcs.push_back(Arc(iter->a, theta[0]));
											nu_arcs.push_back(Arc(theta[0], theta[1]));
											nu_arcs.push_back(Arc(theta[1], iter->b));
										}
										else
										{
											nu_arcs.push_back(Arc(iter->a, theta[0]));
											nu_arcs.push_back(Arc(theta[0], iter->b));
										}
									}
									else if(contains_b)
									{
										nu_arcs.push_back(Arc(iter->a, theta[1]));
										nu_arcs.push_back(Arc(theta[1], iter->b));
									}
									else
										nu_arcs.push_back(*iter);
								}
								arcs = nu_arcs;
								nu_arcs = vector<Arc>();
							}
						}

						Vec3 dp = tube.p2 - tube.p1;
						float dr = tube.r2 - tube.r1;

						// discard arcs that are on the wrong side of the cutting plane
						for(vector<Arc>::iterator iter = arcs.begin(); iter != arcs.end(); ++iter)
						{
							float mid_theta = (iter->a + iter->b) * 0.5f;
							
							Vec3 g = axis_a * cosf(mid_theta) + axis_b * sinf(mid_theta);
							Vec3 gr1 = g * tube.r1, gdr = g * dr;
							float y = (plane.offset - Vec3::Dot(plane.normal, tube.p1 + gr1)) / Vec3::Dot(plane.normal, dp + gdr);
							Vec3 point = (dp + gdr) * y + tube.p1 + gr1;


							if(cut.PointDistance(point) > 0.0f)
								nu_arcs.push_back(*iter);
						}
						arcs = nu_arcs;
					}
				} locus(part.plane, *this);

				for(vector<Plane>::const_iterator iter = planes.begin(); iter != planes.end(); ++iter)
					locus.Cut(*iter);
				for(vector<Plane>::const_iterator iter = part.planes.begin(); iter != part.planes.end(); ++iter)
					locus.Cut(*iter);

				return !locus.arcs.empty();
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

			bool Intersect(const Part& part) const { return part.IntersectPlane(*this); }

			bool IntersectSphere(const SpherePart& part) const { return part.IntersectPlane(*this); }
			bool IntersectTube(const TubePart& part) const { return part.IntersectPlane(*this); }

			bool IntersectPlane(const PlanePart& part) const
			{
				Line line;
				if(Plane::Intersect(plane, part.plane, line))
					return IsLineRelevant(line, *this, part);

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

		bool ContainsPoint(const Vec3& point) const
		{
			int count = 0;

			Ray ray;
			ray.origin = point;
			ray.direction = Vec3(0, 10000000, 0);			// direction is arbitrary, magnitude is arbitrarily large

			for(vector<SpherePart>::const_iterator iter = spheres.begin(); iter != spheres.end() && count < 2; ++iter)
				count += iter->RayTest(ray);

			for(vector<TubePart>::const_iterator iter = tubes.begin(); iter != tubes.end() && count < 2; ++iter)
				count += iter->RayTest(ray);

			for(vector<PlanePart>::const_iterator iter = planes.begin(); iter != planes.end() && count < 2; ++iter)
				count += iter->RayTest(ray);

			return count == 1;
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
			bool any;

			Vec3 center;
			float weight = 0.0f;

			Vec3 plane_norm = plane.normal;
			float plane_offset = plane.offset;

			for(vector<SpherePart>::iterator iter = spheres.begin(); iter != spheres.end(); ++iter)
			{
				Vec3 sphere_pos = my_xform.TransformVec3(iter->sphere.center, 1.0f);
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
				result.a.obj = ibody;
				result.b.obj = jbody;
				result.a.pos = center;
				result.b.pos = center;
				result.b.norm = plane_norm;
				result.a.norm = -plane_norm;

				return true;
			}
			return false;
		}

		bool CollisionCheck(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody)
		{
			AABB other_aabb = other->imp->aabb.GetTransformedAABB(xform);

			AABB overlap;
			if(AABB::Intersect(aabb, other_aabb, overlap))
			{
				Mat4 inv_xform = Mat4::Invert(xform);

				const int steps = 5;
				Vec3 start = overlap.min;
				Vec3 increment = (overlap.max - start) / float(steps - 1);

				Vec3 accum;
				int weight = 0;

				Vec3 pos = start;
				for(int x = 0; x < steps; ++x)
				{
					pos.x += increment.x;
					for(int y = 0; y < steps; ++y)
					{
						pos.y += increment.y;
						for(int z = 0; z < steps; ++z)
						{
							pos.z += increment.z;
							
							if(ContainsPoint(pos))
							{
								Vec3 inv_xformed = inv_xform.TransformVec3(pos, 1.0f);
								if(other->ContainsPoint(inv_xformed))
								{
									++weight;
									accum += pos;
								}
							}
						}
						pos.z = start.z;
					}
					pos.y = start.y;
				}

				if(weight > 0)
				{
					pos = accum / float(weight);

					result = ContactPoint();
					result.a.obj = ibody;
					result.b.obj = jbody;
					result.a.pos = pos;
					result.b.pos = pos;

					Vec3 normal = Vec3::Normalize(Vec3::Normalize(pos - aabb.GetCenterPoint()) - Vec3::Normalize(inv_xform.TransformVec3(pos, 1.0f) - other->imp->aabb.GetCenterPoint()));
					result.a.norm = normal;
					result.b.norm = -normal;

					return true;
				}
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

	MassInfo MultiSphereShape::ComputeMassInfo() { return imp->ComputeMassInfo(); }

	bool MultiSphereShape::ContainsPoint(const Vec3& point) const { return imp->ContainsPoint(point); }

	bool MultiSphereShape::CollisionCheck(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(ray, result, time, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& my_xform, const Plane& plane, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(my_xform, plane, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(sphere, result, ibody, jbody); }
	bool MultiSphereShape::CollisionCheck(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody, RigidBody* jbody) { return imp->CollisionCheck(xform, other, result, ibody, jbody); }

	void MultiSphereShape::Write(ostream& stream) { imp->Write(stream); }
	unsigned int MultiSphereShape::Read(istream& stream) { return imp->Read(stream); }
}
