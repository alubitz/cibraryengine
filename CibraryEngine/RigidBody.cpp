#include "StdAfx.h"
#include "RigidBody.h"

#include "Line.h"
#include "Sphere.h"

#include "Physics.h"
#include "PhysicsRegion.h"

#include "ContactPoint.h"
#include "ContactRegion.h"
#include "ContactDataCollector.h"

#include "CollisionShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"
#include "ConvexMeshShape.h"

#include "CollisionGroup.h"

#define ENABLE_OBJECT_DEACTIVATION 0

namespace CibraryEngine
{
	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ContactDataCollector* collect);
	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ContactDataCollector* collect);
	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShapeInstanceCache* ishape, MultiSphereShapeInstanceCache* jshape, ContactDataCollector* collect);

	static void DoConvexMeshTriangleMesh(RigidBody* ibody, RigidBody* jbody, ConvexMeshShape* ishape, const Mat4& xform, ContactDataCollector* collect);

	


	/*
	 * RigidBody methods
	 */
	RigidBody::RigidBody() : DynamicsObject(NULL, COT_RigidBody, MassInfo()), constraints(), shape(NULL), shape_cache(NULL) { }
	RigidBody::RigidBody(Entity* user_entity, CollisionShape* shape, const MassInfo& mass_info, const Vec3& pos, const Quaternion& ori) :
		DynamicsObject(user_entity, COT_RigidBody, mass_info, pos),
		constraints(),
		ori(ori),
		rot(),
		torque(),
		applied_torque(),
		shape(shape),
		shape_cache(NULL),
		angular_damp(0.1f),
		can_rotate(false),
		contact_callback(NULL),
		collision_callback(NULL)
	{
		restitution = shape->CanMove() ? 0.2f : 1.0f;
		friction    = 1.0f;
		linear_damp = 0.1f;

		active = can_move = shape->CanMove() && mass_info.mass > 0;

		if(Mat3(mass_info.moi).Determinant() != 0.0f)
			can_rotate = true;

		ori_rm = ori.ToMat3();
		inv_moi = ComputeInvMoi();
	}

	void RigidBody::InnerDispose() { if(shape_cache) { delete shape_cache; shape_cache = NULL; } }

	void RigidBody::UpdateVel(float timestep)
	{
		if(active)
		{
			vel += force * (inv_mass * timestep);
			vel *= expf(-linear_damp * timestep);

			if(can_rotate)
			{
				inv_moi = ComputeInvMoi();
				rot += inv_moi * (torque * timestep);

				rot *= expf(-angular_damp * timestep);
			}
		}

		ResetToApplied();
	}

	void RigidBody::UpdatePos(float timestep, PhysicsRegionManager* region_man)
	{
		if(active)
		{
#if ENABLE_OBJECT_DEACTIVATION
			if(vel.ComputeMagnitudeSquared() > 0.01f || rot.ComputeMagnitudeSquared() > 0.1f)
			{
#endif
				pos += vel * timestep;
				pos += ori * mass_info.com;

				// this block equivalent to: ori = Quaternion::FromRVec(-rot * timestep) * ori
				if(float magsq = rot.ComputeMagnitudeSquared())
				{
					float mag = sqrtf(magsq), half = mag * timestep * 0.5f, coeff = -sinf(half) / mag;
					ori = Quaternion(cosf(half), rot.x * coeff, rot.y * coeff, rot.z * coeff) * ori;
				}
				pos -= ori * mass_info.com;

				xform_valid = false;

				region_man->OnObjectUpdate(this, regions, timestep);

#if ENABLE_OBJECT_DEACTIVATION
				deactivation_timer = 0.5f;
			}
			else
			{
				// TODO: become inactive; tell the physics regions we extend into that we're no longer active, etc.
				deactivation_timer -= timestep;
				if(deactivation_timer <= 0)
					active = false;
			}
#endif
		}
	}

	void RigidBody::ComputeXform()
	{
		ori_rm = ori.ToMat3();
		xform = Mat4::FromPositionAndOrientation(pos, ori_rm);
		inv_xform = Mat4::Invert(xform);

		cached_aabb = shape->ComputeCachedWorldAABB(xform, shape_cache);

		cached_com = xform.TransformVec3_1(mass_info.com);

		xform_valid = true;
	}

	Vec3 RigidBody::LocalForceToTorque(const Vec3& force, const Vec3& local_poi)
	{
		ComputeXformAsNeeded();
		Vec3 offset = local_poi - mass_info.com;
		Vec3 radius_vector(
			offset.x * ori_rm.values[0] + offset.y * ori_rm.values[1] + offset.z * ori_rm.values[2],
			offset.x * ori_rm.values[3] + offset.y * ori_rm.values[4] + offset.z * ori_rm.values[5],
			offset.x * ori_rm.values[6] + offset.y * ori_rm.values[7] + offset.z * ori_rm.values[8]
		);
		return Vec3::Cross(force, radius_vector);
	}

	void RigidBody::ApplyLocalImpulse(const Vec3& impulse, const Vec3& local_poi)
	{
		if(active)
		{
			if(can_rotate)
				rot += inv_moi * LocalForceToTorque(impulse, local_poi);
			vel += impulse * inv_mass;
		}
	}

	void RigidBody::ApplyWorldImpulse(const Vec3& impulse, const Vec3& poi)
	{
		if(active)
		{
			if(can_rotate)
				rot += inv_moi * Vec3::Cross(impulse, poi - cached_com);
			vel += impulse * inv_mass;
		}
	}

	void RigidBody::RemoveDisabledCollisions(RelevantObjectsQuery& eligible_bodies)
	{
		CollisionObject::RemoveDisabledCollisions(eligible_bodies);

		for(set<PhysicsConstraint*>::const_iterator iter = constraints.begin(), constraints_end = constraints.end(); iter != constraints_end; ++iter)
		{
			const PhysicsConstraint* c = *iter;
			if(RigidBody* other = c->obj_a == this ? c->obj_b : c->obj_a)
				eligible_bodies.Erase(other);
		}
	}

	void RigidBody::ApplyWorldForce(const Vec3& force, const Vec3& poi)
	{
		ComputeXformAsNeeded();

		applied_torque += Vec3::Cross(force, poi - cached_com);
		applied_force += force;
	}

	MassInfo RigidBody::GetTransformedMassInfo() const
	{
		MassInfo result;
		result.mass = mass_info.mass;
		result.com = ori_rm * mass_info.com + pos;

		Mat3& moi_data = *((Mat3*)((void*)result.moi));			// moi_data.values and result.moi occupy the same space in memory
		moi_data = Mat3::Invert(inv_moi);

		return result;
	}

	AABB RigidBody::GetAABB(float timestep)
	{
		switch(shape->GetShapeType())
		{
			case ST_Sphere:
			{
				float radius = ((SphereShape*)shape)->radius;
				AABB result(pos, radius);
				result.Expand(AABB(pos + vel * timestep, radius));
				return result;
			}

			case ST_TriangleMesh:
			case ST_MultiSphere:
			case ST_ConvexMesh:
				return GetCachedAABB();

			case ST_InfinitePlane:
				return AABB();

			case ST_Ray:
			{
				Debug("A RigidBody is using the unsupported ShapeType ST_Ray!\n");
				return AABB();
			}

			default:
				return AABB();
		}
	}

	void RigidBody::InitiateCollisions(float timestep, ContactDataCollector* collect)
	{
		switch(shape->GetShapeType())
		{
			case ST_MultiSphere:
				InitiateCollisionsForMultisphere(timestep, collect);
				break;

			case ST_ConvexMesh:
				InitiateCollisionsForConvexMesh(timestep, collect);
				break;

			default:
				DEBUG();
				break;
		}
	}

	void RigidBody::InitiateCollisionsForMultisphere(float timestep, ContactDataCollector* collect)
	{
		MultiSphereShape* shape = (MultiSphereShape*)this->shape;

		Mat4 xform = GetTransformationMatrix();
		Mat4 inv_xform = GetInvTransform();

		MultiSphereShapeInstanceCache* cache = (MultiSphereShapeInstanceCache*)shape_cache;
		AABB xformed_aabb = cache->aabb;

		RelevantObjectsQuery relevant_objects;

		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = regions.buckets[i];
			for(vector<PhysicsRegion*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				(*iter)->GetRelevantObjects(xformed_aabb, relevant_objects);
		}

		if(relevant_objects.count != 0)
		{
			RemoveDisabledCollisions(relevant_objects);

			// do collision detection with those objects
			for(unsigned int i = 0; i < RelevantObjectsQuery::hash_size; ++i)
			{
				vector<CollisionObject*>& bucket = relevant_objects.buckets[i];
				for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
					switch((*iter)->GetType())
					{
						case COT_RigidBody:
						{
							RigidBody* rigid_body = (RigidBody*)*iter;
							if(rigid_body->GetShapeType() != ST_MultiSphere || rigid_body < this)
								CollideRigidBody((RigidBody*)*iter, collect);
							break;
						}

						case COT_CollisionGroup:
						{
							CollisionGroup* cgroup = (CollisionGroup*)*iter;
							if(*iter < this)
								cgroup->CollideRigidBody(this, collect);

							break;
						}
					}
			}
		}
	}

	void RigidBody::InitiateCollisionsForConvexMesh(float timestep, ContactDataCollector* collect)
	{
		ConvexMeshShape* shape = (ConvexMeshShape*)this->shape;

		Mat4 xform = GetTransformationMatrix();
		Mat4 inv_xform = GetInvTransform();

		ConvexMeshShapeInstanceCache* cache = (ConvexMeshShapeInstanceCache*)shape_cache;
		AABB xformed_aabb = cache->aabb;

		RelevantObjectsQuery relevant_objects;

		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = regions.buckets[i];
			for(vector<PhysicsRegion*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				(*iter)->GetRelevantObjects(xformed_aabb, relevant_objects);
		}

		if(relevant_objects.count != 0)
		{
			RemoveDisabledCollisions(relevant_objects);

			// do collision detection with those objects
			for(unsigned int i = 0; i < RelevantObjectsQuery::hash_size; ++i)
			{
				vector<CollisionObject*>& bucket = relevant_objects.buckets[i];
				for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
					switch((*iter)->GetType())
					{
						case COT_RigidBody:
						{
							RigidBody* rigid_body = (RigidBody*)*iter;
							if(rigid_body->GetShapeType() != ST_ConvexMesh || rigid_body < this)
								CollideRigidBody((RigidBody*)*iter, collect);
							break;
						}

						case COT_CollisionGroup:
						{
							CollisionGroup* cgroup = (CollisionGroup*)*iter;
							if(*iter < this)
								cgroup->CollideRigidBody(this, collect);

							break;
						}
					}
			}
		}
	}

	void RigidBody::CollideRigidBody(RigidBody* other, ContactDataCollector* collect)
	{
		Mat4 xform = GetTransformationMatrix();

		switch(shape->GetShapeType())
		{
			case ST_MultiSphere:
			{
				switch(other->GetShapeType())
				{
					case ST_TriangleMesh:
						DoMultisphereMesh(this, other, (MultiSphereShape*)shape, xform, collect);
						return;

					case ST_InfinitePlane:
						DoMultispherePlane(this, other, (MultiSphereShape*)shape, xform, collect);
						return;

					case ST_MultiSphere:
						DoMultisphereMultisphere(this, other, (MultiSphereShapeInstanceCache*)shape_cache, (MultiSphereShapeInstanceCache*)other->shape_cache, collect);
						return;

					default:
						DEBUG();
						return;
				}
			}

			case ST_ConvexMesh:
			{
				ConvexMeshShape* ishape = (ConvexMeshShape*)shape;
				switch(other->GetShapeType())
				{
					case ST_TriangleMesh:
						DoConvexMeshTriangleMesh(this, other, ishape, xform, collect);
						return;

					case ST_InfinitePlane:
						ishape->CollidePlane((ConvexMeshShapeInstanceCache*)shape_cache, ((InfinitePlaneShape*)other->GetCollisionShape())->plane, collect, this, other);
						return;

					case ST_ConvexMesh:
						ishape->CollideConvexMesh((ConvexMeshShapeInstanceCache*)shape_cache, (ConvexMeshShapeInstanceCache*)other->shape_cache, collect, this, other);
						return;

					default:
						DEBUG();
						return;
				}
			}

			default:
			{
				DEBUG();
				return;
			}
		}		
	}




	/*
	 * MultiSphereShape collision functions
	 */
	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ContactDataCollector* collect)
	{
		TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();
		Mat4 j_xform = jbody->GetTransformationMatrix();
		Mat4 jinv = jbody->GetInvTransform();

		Mat4 inv_net_xform = jinv * xform;
		AABB xformed_aabb = ishape->GetTransformedAABB(inv_net_xform);						// the AABB of the multisphere in the coordinate system of the mesh

		vector<unsigned int> relevant_triangles;
		jshape->GetRelevantTriangles(xformed_aabb, relevant_triangles);
		if(relevant_triangles.empty())
			return;

		vector<Sphere> my_spheres;															// CollideMesh function will modify this if it's empty, otherwise use existing values

		for(vector<unsigned int>::iterator kter = relevant_triangles.begin(), triangles_end = relevant_triangles.end(); kter != triangles_end; ++kter)
		{
			const TriangleMeshShape::TriCache& tri = jshape->GetTriangleData(*kter);

			if(ContactRegion* r = ishape->CollideMesh(inv_net_xform, my_spheres, tri, collect, ibody, jbody))
			{
				for(vector<ContactPoint*>::iterator iter = r->points.begin(); iter != r->points.end(); ++iter)
				{
					ContactPoint* p = *iter;
					p->pos = j_xform.TransformVec3_1(p->pos);
					p->normal = j_xform.TransformVec3_0(p->normal);
				}
			}
		}
	}

	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ContactDataCollector* collect)
	{
		InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();
		ishape->CollidePlane(xform, jshape->plane, collect, ibody, jbody);
	}

	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShapeInstanceCache* ishape, MultiSphereShapeInstanceCache* jshape, ContactDataCollector* collect)
	{
		AABB overlap;
		if(AABB::Intersect(ishape->aabb, jshape->aabb, overlap))
		{
			vector<Sphere>& my_spheres = ishape->spheres;
			vector<Sphere>& other_spheres = jshape->spheres;

			// sort of like Separating Axis Theorem, but without a convenient list of prescribed axes to check
			struct MaxExtentGetter
			{
				const Sphere* spheres_begin;
				const Sphere* spheres_end;

				MaxExtentGetter(const vector<Sphere>& spheres) : spheres_begin(spheres.data()), spheres_end(spheres_begin + spheres.size()) { }

				float operator()(const Vec3& direction)
				{
					const Sphere* iter = spheres_begin;

					float maximum = Vec3::Dot(direction, iter->center) + iter->radius;
					++iter;

					while(iter != spheres_end)
					{
						maximum = max(maximum, Vec3::Dot(direction, iter->center) + iter->radius);
						++iter;
					}

					return maximum;
				}
			} GetMaximumExtent(my_spheres);

			struct MinExtentGetter
			{
				const Sphere* spheres_begin;
				const Sphere* spheres_end;

				MinExtentGetter(const vector<Sphere>& spheres) : spheres_begin(spheres.data()), spheres_end(spheres_begin + spheres.size()) { }

				float operator()(const Vec3& direction)
				{
					const Sphere* iter = spheres_begin;

					float minimum = Vec3::Dot(direction, iter->center) - iter->radius;
					++iter;

					while(iter != spheres_end)
					{
						minimum = min(minimum, Vec3::Dot(direction, iter->center) - iter->radius);
						++iter;
					}

					return minimum;
				}
			} GetMinimumExtent(other_spheres);

			// try to find a separating axis
			Vec3 direction;
			float score = -1;
			float search_scale = 0.6f;

			static const float x_offsets[8] = {	-1,	-1,	-1, -1,	1,	1,	1,	1 };
			static const float y_offsets[8] = {	-1,	-1,	1,	1,	-1,	-1,	1,	1 };
			static const float z_offsets[8] = {	-1,	1,	-1,	1,	-1,	1,	-1,	1 };

			for(char i = 0; i < 5; ++i)
			{
				float best_score;
				Vec3 best_test;

				for(char j = 0; j < 8; ++j)
				{
					Vec3 dir(Vec3::Normalize(
						direction.x + x_offsets[j] * search_scale,
						direction.y + y_offsets[j] * search_scale,
						direction.z + z_offsets[j] * search_scale));

					float test_score = GetMaximumExtent(dir) - GetMinimumExtent(dir);

					if(test_score < 0)							// found a separating plane? go home early
						return;
					else if(j == 0 || test_score < best_score)
					{
						best_test = dir;
						best_score = test_score;
					}
				}

				if(i != 0 && best_score >= score)
					search_scale *= 0.5f;
				else
				{
					direction = best_test;
					score = best_score;
				}
			}


			// the objects are colliding; find out which of each objects' spheres are relevant
			vector<Sphere*> my_relevant_spheres, other_relevant_spheres;
			my_relevant_spheres.reserve(my_spheres.size());
			other_relevant_spheres.reserve(other_spheres.size());

			float max_extent = GetMaximumExtent(direction);
			float min_extent = GetMinimumExtent(direction);

			for(unsigned char i = 0, count = my_spheres.size(); i < count; ++i)
			{
				Sphere& sphere = my_spheres[i];
				if(Vec3::Dot(sphere.center, direction) + sphere.radius >= min_extent)
					my_relevant_spheres.push_back(&sphere);
			}

			for(unsigned char i = 0, count = other_spheres.size(); i < count; ++i)
			{
				Sphere& sphere = other_spheres[i];
				if(Vec3::Dot(sphere.center, direction) - sphere.radius <= max_extent)
					other_relevant_spheres.push_back(&sphere);
			}

			if(my_relevant_spheres.empty() || other_relevant_spheres.empty())
				return;								// lolwut? silly float math



			// now generate one or more contact points for the collision
			float contact_plane_offset = (min_extent + max_extent) * 0.5f;

			struct ConvexPoly
			{
				struct PolyData
				{
					Vec2 pos;

					Vec2 normal;
					float offset;

					PolyData *next, *prev;

				} data[32], *start;

				ConvexPoly(vector<Sphere*>& spheres, const Vec3& direction, float contact_plane_offset, const Vec3& x_axis, const Vec3& y_axis) : start(&data[0])
				{
					unsigned int count = spheres.size();

					unsigned int max_plane_points = sizeof(data) / sizeof(PolyData);
					if(count >= max_plane_points)
					{
						DEBUG();
						count = max_plane_points;
					}

					for(unsigned int i = 0; i < count; ++i)
					{
						Vec3 in_plane = spheres[i]->center - direction * (Vec3::Dot(direction, spheres[i]->center) - contact_plane_offset);

						Vec2& pos = data[i].pos;
						pos.x = Vec3::Dot(in_plane, x_axis);
						pos.y = Vec3::Dot(in_plane, y_axis);
					}

					data[0].normal = Vec2::Normalize(data[0].pos.y - data[1].pos.y, data[1].pos.x - data[0].pos.x);
					data[0].offset = Vec2::Dot(data[0].normal, data[0].pos);
					data[0].next = data[0].prev = &data[1];
					data[1].normal = -data[0].normal;
					data[1].offset = -data[0].offset;
					data[1].next = data[1].prev = &data[0];

					for(unsigned int i = 2; i < count; ++i)
					{
						PolyData* noob = &data[i];
						PolyData* iter = start;

						do
						{
							if(Vec2::Dot(iter->normal, noob->pos) > iter->offset)
							{
								PolyData *first = iter, *last = iter;

								if(iter == start)
									while(Vec2::Dot(first->prev->normal, noob->pos) > first->prev->offset)
										first = first->prev;

								while(Vec2::Dot(last->next->normal, noob->pos) > last->next->offset)
									last = last->next;
								last = last->next;

								first->next = last->prev = noob;
								noob->prev = first;
								noob->next = last;

								first->normal = Vec2::Normalize(first->pos.y - noob->pos.y, noob->pos.x - first->pos.x);
								first->offset = Vec2::Dot(first->normal, first->pos);

								noob->normal = Vec2::Normalize(noob->pos.y - last->pos.y, last->pos.x - noob->pos.x);
								noob->offset = Vec2::Dot(noob->normal, noob->pos);

								start = noob;

								break;
							}

							iter = iter->next;
						} while(iter != start);
					}
				}
			};

			struct TubePlaneSolver
			{
				const Sphere& sphere_a;
				const Sphere& sphere_b;
				const Vec3& direction;
				vector<Sphere*>& poly_spheres;
				float contact_plane_offset;

				RigidBody *ibody, *jbody;

				TubePlaneSolver(const Sphere& sphere_a, const Sphere& sphere_b, const Vec3& direction, float contact_plane_offset, vector<Sphere*>& poly_spheres, RigidBody* ibody, RigidBody* jbody) : sphere_a(sphere_a), sphere_b(sphere_b), direction(direction), contact_plane_offset(contact_plane_offset), poly_spheres(poly_spheres), ibody(ibody), jbody(jbody) { }

				bool Solve(ContactDataCollector* collect)
				{
					Vec3 a = sphere_a.center - direction * (Vec3::Dot(direction, sphere_a.center) - contact_plane_offset);
					Vec3 b = sphere_b.center - direction * (Vec3::Dot(direction, sphere_b.center) - contact_plane_offset);
					Vec3 ab = b - a;

					float magsq = ab.ComputeMagnitudeSquared();
					if(magsq != 0.0f)
					{
						Vec3 u_ab = ab / sqrtf(magsq);
						Vec3 cross_ab = Vec3::Cross(direction, u_ab);

						float ay  = Vec3::Dot(a, cross_ab);
						float ax1 = Vec3::Dot(a, u_ab);
						float ax2 = Vec3::Dot(b, u_ab);

						Vec2 tube_p1(ax1, ay), tube_p2(ax2, ay);

						// flatten polygon points onto plane
						ConvexPoly my_convex_poly(poly_spheres, direction, contact_plane_offset, u_ab, cross_ab);

						// truncate the line segment where it extends beyond the bounds of the polygon
						ConvexPoly::PolyData *iter = my_convex_poly.start;
						do
						{
							const Vec2& normal = iter->normal;

							// TODO: do truncation more cleanly? there's still the potential for some weird behavior here
							float y_sq = normal.y * normal.y;
							if(y_sq > 0.001f)
							{
								float p1_offness = Vec2::Dot(tube_p1, normal) - iter->offset;
								if(p1_offness > 0)			// funky control flow branching to only compute offness_coeff once, and only if needed
								{
									float offness_coeff = sqrtf(1.0f + (normal.x * normal.x) / y_sq);

									tube_p1.x -= p1_offness * offness_coeff;

									float p2_offness = Vec2::Dot(tube_p2, normal) - iter->offset;
									if(p2_offness > 0)
										tube_p2.x -= p2_offness * offness_coeff;
								}
								else
								{
									float p2_offness = Vec2::Dot(tube_p2, normal) - iter->offset;
									if(p2_offness > 0)
									{
										float offness_coeff = sqrtf(1.0f + (normal.x * normal.x) / y_sq);
										tube_p2.x -= p2_offness * offness_coeff;
									}
								}
							}

							iter = iter->next;

						} while(iter != my_convex_poly.start);

						// produce contact points at each endpoint
						Vec3 origin = direction * contact_plane_offset + cross_ab * ay;

						Vec3 positions[2] = { origin + u_ab * tube_p1.x, origin + u_ab * tube_p2.x };
						collect->AddRegion(ibody, jbody, direction, 2, positions);

						return true;
					}

					return false;
				}
			};

			switch(my_relevant_spheres.size())
			{
				case 1:
				{
					Sphere& my_sphere = *my_relevant_spheres[0];

					switch(other_relevant_spheres.size())
					{
						case 1:						// sphere-sphere
						{
							Sphere& other_sphere = *other_relevant_spheres[0];

							Vec3 pos = (my_sphere.center + other_sphere.center) * 0.5f;
							pos -= direction * (Vec3::Dot(direction, pos) - contact_plane_offset);

							collect->AddRegion(ibody, jbody, direction, 1, &pos);
							return;
						}

						case 2:						// sphere-tube
						{
							Sphere& other_sphere_a = *other_relevant_spheres[0];
							Sphere& other_sphere_b = *other_relevant_spheres[1];

							Vec3 pos = my_sphere.center - direction * (Vec3::Dot(direction, my_sphere.center) - contact_plane_offset);

							collect->AddRegion(ibody, jbody, direction, 1, &pos);
							return;
						}

						default:					// sphere-plane
						{
							Vec3 pos = my_sphere.center - direction * (Vec3::Dot(direction, my_sphere.center) - contact_plane_offset);

							collect->AddRegion(ibody, jbody, direction, 1, &pos);
							return;
						}
					}
				}

				case 2:
				{
					Sphere& my_sphere_a = *my_relevant_spheres[0];
					Sphere& my_sphere_b = *my_relevant_spheres[1];

					switch(other_relevant_spheres.size())
					{
						case 1:						// tube-sphere
						{
							Sphere& other_sphere = *other_relevant_spheres[0];

							Vec3 pos = other_sphere.center - direction * (Vec3::Dot(direction, other_sphere.center) - contact_plane_offset);

							collect->AddRegion(ibody, jbody, direction, 1, &pos);
							return;
						}

						case 2:						// tube-tube
						{
							Sphere& other_sphere_a = *other_relevant_spheres[0];
							Sphere& other_sphere_b = *other_relevant_spheres[1];

							// flatten sphere centers onto contact plane
							Vec3 my_pa = my_sphere_a.center - direction * (Vec3::Dot(direction, my_sphere_a.center) - contact_plane_offset);
							Vec3 my_pb = my_sphere_b.center - direction * (Vec3::Dot(direction, my_sphere_b.center) - contact_plane_offset);
							Vec3 other_pa = other_sphere_a.center - direction * (Vec3::Dot(direction, other_sphere_a.center) - contact_plane_offset);
							Vec3 other_pb = other_sphere_b.center - direction * (Vec3::Dot(direction, other_sphere_b.center) - contact_plane_offset);

							Vec3 my_ab = my_pb - my_pa;
							Vec3 other_ab = other_pb - other_pa;

							float my_magsq = my_ab.ComputeMagnitudeSquared(), other_magsq = other_ab.ComputeMagnitudeSquared();
							if(my_magsq != 0.0f && other_magsq != 0.0f)
							{
								float my_mag = sqrtf(my_magsq), other_mag = sqrtf(other_magsq);
								float my_inv = 1.0f / my_mag, other_inv = 1.0f / other_mag;

								float abdot = Vec3::Dot(my_ab, other_ab);
								float cosine = abdot * my_inv * other_inv;
								if(cosine > 0.8f || cosine < -0.8f)			// tubes are nearly parallel/anti-parallel; try to produce two contact points
								{
									Vec3 axis = Vec3::Normalize(cosine > 0 ? my_ab + other_ab : my_ab - other_ab);

									float my_adot		= Vec3::Dot(axis, my_pa),		my_bdot		= Vec3::Dot(axis, my_pb);
									float other_adot	= Vec3::Dot(axis, other_pa),	other_bdot	= Vec3::Dot(axis, other_pb);
									float near_end	= max(min(my_adot, my_bdot), min(other_adot, other_bdot));
									float far_end	= min(max(my_adot, my_bdot), max(other_adot, other_bdot));

									Vec3 cross = Vec3::Normalize(Vec3::Cross(axis, direction));
									float avg_xdot = Vec3::Dot(my_pa + my_pb + other_pa + other_pb, cross) * 0.25f;

									Vec3 origin = direction * contact_plane_offset + cross * avg_xdot;

									Vec3 positions[2] = { origin + axis * near_end, origin + axis * far_end };

									collect->AddRegion(ibody, jbody, direction, 2, positions);
									return;
								}
								else										// tubes are crossed; try to produce a single contact point where they intersect
								{
									// TODO: do this more cleanly; check that the point is reasonable (i.e. somewhere within the extent of both tubes)?
									Vec3 u_other = other_ab * other_inv;
									Vec3 x_my = Vec3::Normalize(Vec3::Cross(direction, my_ab));

									float speed = Vec3::Dot(u_other, x_my);
									float tti = Vec3::Dot(x_my, other_pa - my_pa) / speed;

									Vec3 pos = other_pa + u_other * tti;

									collect->AddRegion(ibody, jbody, direction, 1, &pos);
									return;
								}
							}

							DEBUG();				// degenerate case... so far this has yet to ever happen

							break;
						}

						default:					// tube-plane
						{
							if(TubePlaneSolver(my_sphere_a, my_sphere_b, direction, contact_plane_offset, other_relevant_spheres, ibody, jbody).Solve(collect))
								return;

							break;
						}
					}

					break;
				}

				default:
				{
					switch(other_relevant_spheres.size())
					{
						case 1:						// plane-sphere
						{
							Sphere& other_sphere = *other_relevant_spheres[0];

							Vec3 pos = other_sphere.center - direction * (Vec3::Dot(direction, other_sphere.center) - contact_plane_offset);

							collect->AddRegion(ibody, jbody, direction, 1, &pos);
							return;
						}

						case 2:						// plane-tube
						{
							if(TubePlaneSolver(*other_relevant_spheres[0], *other_relevant_spheres[1], direction, contact_plane_offset, my_relevant_spheres, ibody, jbody).Solve(collect))
								return;

							break;
						}

						default:					// plane-plane
						{
							Vec3 x_axis = fabs(direction.x) < 0.8f ? Vec3::Normalize(Vec3::Cross(direction, Vec3(1.0f, 0.0f, 0.0f))) : fabs(direction.y) < 0.8f ? Vec3::Normalize(Vec3::Cross(direction, Vec3(0.0f, 1.0f, 0.0f))) : Vec3::Normalize(Vec3::Cross(direction, Vec3(0.0f, 0.0f, 1.0f)));
							Vec3 y_axis = Vec3::Cross(direction, x_axis);

							ConvexPoly my_poly(my_relevant_spheres, direction, contact_plane_offset, x_axis, y_axis);
							ConvexPoly other_poly(other_relevant_spheres, direction, contact_plane_offset, x_axis, y_axis);

							Vec2 result_points[64];
							unsigned int count = 0;

							// add verts from my object, and from edge intersections between the two objects
							ConvexPoly::PolyData *my_start = my_poly.start, *other_start = other_poly.start, *iter = my_start, *jter;
							do
							{
								bool ok = true;
								Vec2& pos = iter->pos;

								Vec2 edge_dir(iter->normal.y, -iter->normal.x);

								jter = other_start;
								do
								{
									float p1_jdist = Vec2::Dot(pos - jter->pos, jter->normal);
									float p2_jdist = Vec2::Dot(iter->next->pos - jter->pos, jter->normal);

									if(p1_jdist * p2_jdist < 0)										// if this edge intersects the other edge, create a point where they intersect
									{
										float p1_idist = Vec2::Dot(jter->pos - pos, iter->normal);
										float p2_idist = Vec2::Dot(jter->next->pos - pos, iter->normal);
										if(p1_idist * p2_idist < 0)
											result_points[count++] = pos + edge_dir * fabs(p1_jdist);
									}

									if(ok && Vec2::Dot(jter->normal, pos) > jter->offset)			// if vert i was on the wrong side of edge j, discard it
										ok = false;

									jter = jter->next;
								} while(jter != other_start);

								if(ok)
									result_points[count++] = iter->pos;

								iter = iter->next;
							} while(iter != my_start);

							// add verts from other object
							jter = other_start;
							do
							{
								Vec2& pos = jter->pos;
								bool ok = true;

								iter = my_start;
								do
								{
									if(Vec2::Dot(iter->normal, pos) > iter->offset)			// if vert j was on the wrong side of edge i, discard it
									{
										ok = false;
										break;
									}

									iter = iter->next;
								} while(iter != my_start);

								if(ok)
									result_points[count++] = jter->pos;

								jter = jter->next;
							} while(jter != other_start);

							// now create actual contact points from those
							if(count > 0)
							{
								vector<Vec3> positions;

								Vec3 origin = direction * contact_plane_offset;
								for(unsigned int i = 0; i < count; ++i)
									positions.push_back(origin + x_axis * result_points[i].x  + y_axis * result_points[i].y);

								collect->AddRegion(ibody, jbody, direction, count, positions.data());
								return;
							}

							break;
						}
					}

					break;
				}
			}



			// fallback contact point generation

			Vec3 pos = overlap.GetCenterPoint();
			pos -= direction * (Vec3::Dot(direction, pos) - contact_plane_offset);

			collect->AddRegion(ibody, jbody, direction, 1, &pos);
		}
	}




	/*
	 * ConvexMeshShape collision functions
	 */
	static void DoConvexMeshTriangleMesh(RigidBody* ibody, RigidBody* jbody, ConvexMeshShape* ishape, const Mat4& xform, ContactDataCollector* collect)
	{
		TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();
		Mat4 j_xform = jbody->GetTransformationMatrix();
		Mat4 jinv = jbody->GetInvTransform();

		Mat4 inv_net_xform = jinv * xform;
		AABB xformed_aabb = ishape->GetTransformedAABB(inv_net_xform);						// the AABB of the multisphere in the coordinate system of the mesh

		vector<unsigned int> relevant_triangles;
		jshape->GetRelevantTriangles(xformed_aabb, relevant_triangles);
		if(relevant_triangles.empty())
			return;

		ConvexMeshShapeInstanceCache cache;
		cache.Update(inv_net_xform, ishape);

		for(vector<unsigned int>::iterator kter = relevant_triangles.begin(), triangles_end = relevant_triangles.end(); kter != triangles_end; ++kter)
		{
			if(ContactRegion* r = ishape->CollideTri(&cache, jshape->GetTriangleData(*kter), collect, ibody, jbody))
			{
				for(vector<ContactPoint*>::iterator iter = r->points.begin(), results_end = r->points.end(); iter != results_end; ++iter)
				{
					ContactPoint* p = *iter;
					p->pos = j_xform.TransformVec3_1(p->pos);
					p->normal = j_xform.TransformVec3_0(p->normal);
				}
			}
		}
	}
}