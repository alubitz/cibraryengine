#include "StdAfx.h"
#include "RigidBody.h"

#include "AABB.h"

#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"

#include "Sphere.h"

#include "Physics.h"
#include "PhysicsRegion.h"

#include "CollisionShape.h"
#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#define ENABLE_OBJECT_DEACTIVATION 0

namespace CibraryEngine
{
	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, vector<ContactPoint>& contact_points);
	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, vector<ContactPoint>& contact_points);
	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShapeInstanceCache* ishape, MultiSphereShapeInstanceCache* jshape, vector<ContactPoint>& contact_points);





	/*
	 * RigidBody methods
	 */
	RigidBody::RigidBody() : DynamicsObject(NULL, COT_RigidBody, MassInfo()), constraints(), shape(NULL), shape_cache(NULL) { }
	RigidBody::RigidBody(Entity* user_entity, CollisionShape* shape, const MassInfo& mass_info_, Vec3 pos, Quaternion ori) :
		DynamicsObject(user_entity, COT_RigidBody, mass_info_, pos),
		constraints(),
		ori(ori),
		rot(),
		torque(),
		applied_torque(),
		shape(shape),
		shape_cache(NULL),
		angular_damp(0.1f),
		can_rotate(false),
		collision_callback(NULL)
	{
		bounciness = shape->CanMove() ? 0.2f : 1.0f;
		friction = 1.0f;
		linear_damp = 0.1f;

		active = can_move = shape->CanMove() && mass_info.mass > 0;

		Mat3 moi_rm(mass_info.moi);

		if(moi_rm.Determinant() != 0.0f)
			can_rotate = true;

		inv_moi = ComputeInvMoi();
	}

	void RigidBody::InnerDispose()
	{
		if(shape)
		{
			shape->Dispose();
			delete shape;

			shape = NULL;
		}

		if(shape_cache)
		{
			delete shape_cache;
			shape_cache = NULL;
		}
	}

	void RigidBody::DisposePreservingCollisionShape()					{ shape = NULL; Dispose(); }




	Mat3 RigidBody::ComputeInvMoi()										{ return ori_rm.Transpose() * Mat3::Invert(Mat3(mass_info.moi)) * ori_rm; }

	void RigidBody::UpdateVel(float timestep)
	{
		if(active)
		{
			vel += (force * timestep) / mass_info.mass;
			vel *= exp(-linear_damp * timestep);

			if(can_rotate)
			{
				inv_moi = ComputeInvMoi();
				rot += inv_moi * (torque * timestep);

				rot *= exp(-angular_damp * timestep);
			}
		}

		ResetToApplied();
	}

	void RigidBody::UpdatePos(float timestep, PhysicsRegionManager* region_man)
	{
		if(active)
		{
#if ENABLE_OBJECT_DEACTIVATION
			if(vel.ComputeMagnitudeSquared() > 0.01f || rot.ComputeMagnitudeSquared() > 0.1f)			// eventually these should be thresholds instead of straight 0s
			{
#endif
				pos += vel * timestep;

				pos += ori.ToMat3().Transpose() * mass_info.com;
				ori *= Quaternion::FromPYR(rot * timestep);
				pos -= ori.ToMat3().Transpose() * mass_info.com;

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

	void RigidBody::ComputeXformAsNeeded()								{ if(!xform_valid) { ComputeXform(); } }

	void RigidBody::ResetForces()										{ DynamicsObject::ResetForces(); applied_torque = Vec3(); }

	void RigidBody::ResetToApplied()									{ DynamicsObject::ResetToApplied(); torque = applied_torque; }

	Vec3 RigidBody::LocalForceToTorque(const Vec3& force, const Vec3& local_poi)
	{
		ComputeXformAsNeeded();
		Vec3 offset = local_poi - mass_info.com;
		Vec3 radius_vector(
			offset.x * ori_rm.values[0] + offset.y * ori_rm.values[3] + offset.z * ori_rm.values[6],
			offset.x * ori_rm.values[1] + offset.y * ori_rm.values[4] + offset.z * ori_rm.values[7],
			offset.x * ori_rm.values[2] + offset.y * ori_rm.values[5] + offset.z * ori_rm.values[8]
		);
		return Vec3::Cross(force, radius_vector);
	}

	Vec3 RigidBody::GetLocalVelocity(const Vec3& point)					{ ComputeXformAsNeeded(); return vel + Vec3::Cross(point - cached_com, rot); }

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

	void RigidBody::ApplyAngularImpulse(const Vec3& angular_impulse)	{ rot += inv_moi * angular_impulse; }

	void RigidBody::RemoveConstrainedBodies(RelevantObjectsQuery& eligible_bodies) const
	{
		for(set<PhysicsConstraint*>::const_iterator iter = constraints.begin(), constraints_end = constraints.end(); iter != constraints_end; ++iter)
		{
			const PhysicsConstraint* c = *iter;
			if(RigidBody* other = c->obj_a == this ? c->obj_b : c->obj_a)
				eligible_bodies.Erase(other);
		}

		for(set<CollisionObject*>::const_iterator iter = disabled_collisions.begin(), disabled_end = disabled_collisions.end(); iter != disabled_end; ++iter)
			eligible_bodies.Erase(*iter);
	}



	

	Quaternion RigidBody::GetOrientation()								{ return ori; }
	void RigidBody::SetOrientation(Quaternion ori_)						{ ori = ori_; xform_valid = false; }

	Mat4 RigidBody::GetTransformationMatrix()							{ ComputeXformAsNeeded(); return xform; }
	Mat4 RigidBody::GetInvTransform()									{ ComputeXformAsNeeded(); return inv_xform; }

	bool RigidBody::MergesSubgraphs()									{ return shape->CanMove() && shape->GetShapeType() != ST_Ray; }

	void RigidBody::ApplyForce(const Vec3& force, const Vec3& local_poi)
	{
		applied_torque += LocalForceToTorque(force, local_poi);
		applied_force += force;
	}

	Vec3 RigidBody::GetAngularVelocity()								{ return rot; }
	void RigidBody::SetAngularVelocity(const Vec3& vel)					{ rot = vel; }

	MassInfo RigidBody::GetTransformedMassInfo() const
	{
		MassInfo result;
		result.mass = mass_info.mass;
		result.com = ori_rm.Transpose() * mass_info.com + pos;
		Mat3 moi_data = Mat3::Invert(inv_moi);
		for(int i = 0; i < 9; ++i)
			result.moi[i] = moi_data[i];

		return result;
	}

	
	Vec3 RigidBody::GetCenterOfMass()									{ ComputeXformAsNeeded(); return cached_com; }

	Mat3 RigidBody::GetInvMoI()											{ return inv_moi; }

	void RigidBody::DebugDraw(SceneRenderer* renderer)					{ shape->DebugDraw(renderer, pos, ori); }

	CollisionShape* RigidBody::GetCollisionShape()						{ return shape; }
	ShapeType RigidBody::GetShapeType()									{ return shape->GetShapeType(); }

	void RigidBody::SetCollisionCallback(CollisionCallback* callback)	{ collision_callback = callback; }
	CollisionCallback* RigidBody::GetCollisionCallback() const			{ return collision_callback; }

	AABB RigidBody::GetAABB(float timestep)
	{
		switch(shape->GetShapeType())
		{
			case ST_Ray:
			{
				AABB result(pos);
				result.Expand(pos + vel * timestep);
				return result;
			}

			case ST_Sphere:
			{
				float radius = ((SphereShape*)shape)->radius;
				AABB result(pos, radius);
				result.Expand(AABB(pos + vel * timestep, radius));
				return result;
			}

			case ST_TriangleMesh:
			case ST_MultiSphere:
				return GetCachedAABB();

			case ST_InfinitePlane:
				return AABB();

			default:
				return AABB();
		}
	}

	AABB RigidBody::GetCachedAABB()										{ ComputeXformAsNeeded(); return cached_aabb; }




	void RigidBody::InitiateCollisions(float timestep, vector<ContactPoint>& contact_points)
	{
		switch(shape->GetShapeType())
		{
			case ST_MultiSphere:
				InitiateCollisionsForMultisphere(timestep, contact_points);
				break;

			default:
				DEBUG();
				break;
		}
	}

	void RigidBody::InitiateCollisionsForMultisphere(float timestep, vector<ContactPoint>& contact_points)
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

		RemoveConstrainedBodies(relevant_objects);

		// do collision detection with those objects
		for(unsigned int i = 0; i < RelevantObjectsQuery::hash_size; ++i)
		{
			vector<CollisionObject*>& bucket = relevant_objects.buckets[i];
			for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				if(RigidBody* other = (RigidBody*)*iter)					// TODO: account for the other kinds of CollisionObject which will eventually exist
				{
					switch(other->GetShapeType())
					{
						case ST_TriangleMesh:
							DoMultisphereMesh(this, other, shape, xform, contact_points);
							break;

						case ST_InfinitePlane:
							DoMultispherePlane(this, other, shape, xform, contact_points);
							break;

						case ST_MultiSphere:
							if(other < this)
							{
								MultiSphereShapeInstanceCache* other_cache = (MultiSphereShapeInstanceCache*)other->shape_cache;
								DoMultisphereMultisphere(this, other, cache, other_cache, contact_points);
							}
							break;
					}
				}
		}
	}

	/*
	 * MultiSphereShape collision functions
	 */
	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, vector<ContactPoint>& contact_points)
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

		ContactPoint p;
		for(vector<unsigned int>::iterator kter = relevant_triangles.begin(), triangles_end = relevant_triangles.end(); kter != triangles_end; ++kter)
		{
			const TriangleMeshShape::TriCache& tri = jshape->GetTriangleData(*kter);

			if(ishape->CollideMesh(inv_net_xform, my_spheres, tri, p, ibody, jbody))
			{
				p.a.pos = j_xform.TransformVec3_1(p.a.pos);
				p.b.pos = j_xform.TransformVec3_1(p.b.pos);
				p.b.norm = j_xform.TransformVec3_0(p.b.norm);
				p.a.norm = -p.b.norm;

				contact_points.push_back(p);
			}
		}
	}

	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, vector<ContactPoint>& contact_points)
	{
		InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();
		ishape->CollidePlane(xform, jshape->plane, contact_points, ibody, jbody);
	}

	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShapeInstanceCache* ishape, MultiSphereShapeInstanceCache* jshape, vector<ContactPoint>& contact_points)
	{
		struct MaxExtentGetter
		{
			float operator()(const Vec3& direction, const vector<Sphere>& spheres)
			{
				vector<Sphere>::const_iterator iter = spheres.begin(), spheres_end = spheres.end();

				float maximum = Vec3::Dot(direction, iter->center) + iter->radius;
				++iter;

				while(iter != spheres_end)
				{
					maximum = max(maximum, Vec3::Dot(direction, iter->center) + iter->radius);
					++iter;
				}

				return maximum;
			}
		} GetMaximumExtent;

		struct MinExtentGetter
		{
			float operator()(const Vec3& direction, const vector<Sphere>& spheres)
			{
				vector<Sphere>::const_iterator iter = spheres.begin(), spheres_end = spheres.end();

				float minimum = Vec3::Dot(direction, iter->center) - iter->radius;
				++iter;

				while(iter != spheres_end)
				{
					minimum = min(minimum, Vec3::Dot(direction, iter->center) - iter->radius);
					++iter;
				}

				return minimum;
			}
		} GetMinimumExtent;

		AABB overlap;
		if(AABB::Intersect(ishape->aabb, jshape->aabb, overlap))
		{
			vector<Sphere>& my_spheres = ishape->spheres;
			vector<Sphere>& other_spheres = jshape->spheres;

			// try to find a separating axis
			Vec3 direction;
			float score = -1;
			float search_scale = 0.6f;

			char best_test;
			Vec3 test_dir[8];

			static const float x_offsets[] = {	-1,	-1,	-1, -1,	1,	1,	1,	1 };
			static const float y_offsets[] = {	-1,	-1,	1,	1,	-1,	-1,	1,	1 };
			static const float z_offsets[] = {	-1,	1,	-1,	1,	-1,	1,	-1,	1 };

			for(char i = 0; i < 5; ++i)
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
						return;
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

			ContactPoint p;
			p.obj_a = ibody;
			p.obj_b = jbody;

			p.a.norm = direction;
			p.b.norm = -direction;

			Vec3 pos = overlap.GetCenterPoint();					// TODO: do this better
			Vec3 offset = direction * (score * 0.5f);
			p.a.pos = pos - offset;
			p.b.pos = pos + offset;

			contact_points.push_back(p);
		}
	}
}
