#include "StdAfx.h"
#include "RigidBody.h"

#include "AABB.h"

#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Physics.h"

#include "CollisionShape.h"
#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#define ENABLE_OBJECT_DEACTIVATION 0

namespace CibraryEngine
{
	/*
	 * RigidBody methods
	 */
	RigidBody::RigidBody() : regions(), constraints(), gravity(), mass_info(), shape(NULL), user_entity(NULL), collision_callback(NULL) { }
	RigidBody::RigidBody(CollisionShape* shape, MassInfo mass_info, Vec3 pos, Quaternion ori) :
		regions(),
		constraints(),
		pos(pos),
		vel(),
		ori(ori),
		rot(),
		force(),
		torque(),
		applied_force(),
		applied_torque(),
		gravity(),
		mass_info(mass_info),
		shape(shape),
		xform_valid(false),
		bounciness(!shape->CanMove() ? 1.0f : shape->GetShapeType() == ST_Ray ? 0.8f : 0.2f),
		friction(shape->GetShapeType() == ST_InfinitePlane ? 1.0f : shape->GetShapeType() == ST_Ray ? 0.0f : 1.0f),
		linear_damp(0.1f),
		angular_damp(0.1f),
		can_move(shape->CanMove() && mass_info.mass > 0),
		can_rotate(false),
		active(can_move),
		deactivation_timer(0.5f),
		user_entity(NULL),
		collision_callback(NULL)
	{
		Mat3 moi_rm(mass_info.moi);

		if(moi_rm.Determinant() != 0.0f)
			can_rotate = true;

		inv_mass = mass_info.mass = 0.0f ? 0.0f : 1.0f / mass_info.mass;
		inv_moi = ComputeInvMoi();
	}

	void RigidBody::InnerDispose()
	{
		if(shape != NULL)
		{
			shape->Dispose();
			delete shape;

			shape = NULL;
		}
	}

	void RigidBody::DisposePreservingCollisionShape() { shape = NULL; Dispose(); }




	Mat3 RigidBody::ComputeInvMoi() { return ori_rm.Transpose() * Mat3::Invert(Mat3(mass_info.moi)) * ori_rm; }

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

		cached_aabb = shape->GetTransformedAABB(xform);

		cached_com = xform.TransformVec3_1(mass_info.com);

		xform_valid = true;
	}

	void RigidBody::ComputeXformAsNeeded() { if(!xform_valid) { ComputeXform(); } }

	void RigidBody::ResetForces() 
	{
		applied_force = can_move ? gravity * mass_info.mass : Vec3();
		applied_torque = Vec3();
	}

	void RigidBody::ResetToApplied()
	{
		force = applied_force;
		torque = applied_torque;
	}

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

	Vec3 RigidBody::GetLocalVelocity(const Vec3& point) { ComputeXformAsNeeded(); return vel + Vec3::Cross(point - cached_com, rot); }

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

	void RigidBody::ApplyCentralImpulse(const Vec3& impulse) { if(active) { vel += impulse * inv_mass; } }

	void RigidBody::ApplyAngularImpulse(const Vec3& angular_impulse) { rot += inv_moi * angular_impulse; }

	void RigidBody::RemoveConstrainedBodies(unordered_set<RigidBody*>* eligible_bodies) const
	{
		for(set<PhysicsConstraint*>::const_iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			const PhysicsConstraint* c = *iter;
			if(RigidBody* other = c->obj_a == this ? c->obj_b : c->obj_a)
				eligible_bodies[other->GetCollisionShape()->GetShapeType()].erase(other);
		}

		for(set<RigidBody*>::const_iterator iter = disabled_collisions.begin(); iter != disabled_collisions.end(); ++iter)
		{
			RigidBody* other = *iter;
			eligible_bodies[other->GetCollisionShape()->GetShapeType()].erase(other);
		}
	}



	Vec3 RigidBody::GetPosition() { return pos; }
	void RigidBody::SetPosition(Vec3 pos_) { pos = pos_; xform_valid = false; }

	Quaternion RigidBody::GetOrientation() { return ori; }
	void RigidBody::SetOrientation(Quaternion ori_) { ori = ori_; xform_valid = false; }

	Mat4 RigidBody::GetTransformationMatrix() { ComputeXformAsNeeded(); return xform; }
	Mat4 RigidBody::GetInvTransform() { ComputeXformAsNeeded(); return inv_xform; }

	void RigidBody::SetBounciness(float bounciness_) { bounciness = bounciness_; }
	void RigidBody::SetFriction(float friction_) { friction = friction_; }
	float RigidBody::GetBounciness() { return bounciness; }
	float RigidBody::GetFriction() { return friction; }

	bool RigidBody::MergesSubgraphs() { return shape->CanMove() && shape->GetShapeType() != ST_Ray; }

	void RigidBody::ApplyForce(const Vec3& force, const Vec3& local_poi)
	{
		applied_torque += LocalForceToTorque(force, local_poi);
		applied_force += force;
	}

	void RigidBody::ApplyCentralForce(const Vec3& force) { applied_force += force; }

	Vec3 RigidBody::GetLinearVelocity() { return vel; }
	void RigidBody::SetLinearVelocity(const Vec3& vel_) { vel = vel_; }

	Vec3 RigidBody::GetAngularVelocity() { return rot; }
	void RigidBody::SetAngularVelocity(const Vec3& vel) { rot = vel; }

	void RigidBody::SetGravity(const Vec3& grav) { gravity = grav; }
	void RigidBody::SetDamp(float damp) { linear_damp = damp; }

	MassInfo RigidBody::GetMassInfo() const { return mass_info; }
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

	float RigidBody::GetMass() const { return mass_info.mass; }
	Vec3 RigidBody::GetCenterOfMass() { ComputeXformAsNeeded(); return cached_com; }

	Mat3 RigidBody::GetInvMoI() { return inv_moi; }

	void RigidBody::DebugDraw(SceneRenderer* renderer) { shape->DebugDraw(renderer, pos, ori); }

	void RigidBody::SetCollisionCallback(CollisionCallback* callback) { collision_callback = callback; }
	CollisionCallback* RigidBody::GetCollisionCallback() { return collision_callback; }

	CollisionShape* RigidBody::GetCollisionShape() { return shape; }

	void RigidBody::SetCollisionEnabled(RigidBody* other, bool enabled)
	{
		if(enabled)
		{
			disabled_collisions.erase(other);
			other->disabled_collisions.erase(this);
		}
		else
		{
			disabled_collisions.insert(other);
			other->disabled_collisions.insert(this);
		}
	}

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
				return shape->GetTransformedAABB(GetTransformationMatrix());

			case ST_InfinitePlane:
				return AABB();

			default:
				return AABB();
		}
	}

	AABB RigidBody::GetCachedAABB() { ComputeXformAsNeeded(); return cached_aabb; }

	Entity* RigidBody::GetUserEntity() { return user_entity; }
	void RigidBody::SetUserEntity(Entity* entity) { user_entity = entity; }
}
