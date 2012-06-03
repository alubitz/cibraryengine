#include "StdAfx.h"
#include "RigidBody.h"

#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"
#include "Physics.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	/*
	 * RigidBody methods
	 */
	RigidBody::RigidBody() : gravity(), mass_info(), shape(NULL), user_entity(NULL), collision_callback(NULL) { }
	RigidBody::RigidBody(CollisionShape* shape, MassInfo mass_info, Vec3 pos, Quaternion ori) :
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
		linear_damp(0.2f),
		angular_damp(0.6f),
		can_move(shape->CanMove() && mass_info.mass > 0),
		can_rotate(false),
		active(can_move),
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




	Mat3 RigidBody::ComputeInvMoi() { Mat3 rm(ori.ToMat3()); return rm.Transpose() * Mat3::Invert(Mat3(mass_info.moi)) * rm; }

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

	void RigidBody::UpdatePos(float timestep)
	{
		if(active)
		{
			pos += ori.ToMat3().Transpose() * mass_info.com;
			pos += vel * timestep;

			ori *= Quaternion::FromPYR(rot * timestep);

			pos -= ori.ToMat3().Transpose() * mass_info.com;

			xform_valid = false;
		}
	}

	void RigidBody::ComputeXform()
	{
		xform = Mat4::FromPositionAndOrientation(pos, ori);
		inv_xform = Mat4::Invert(xform);

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

	Vec3 RigidBody::LocalForceToTorque(const Vec3& force, const Vec3& local_poi) { return Vec3::Cross(force, ori.ToMat3().Transpose() * (local_poi - mass_info.com)); }

	Vec3 RigidBody::GetLocalVelocity(const Vec3& point) { return vel + Vec3::Cross(point - (pos + ori.ToMat3().Transpose() * mass_info.com), rot); }

	void RigidBody::ApplyImpulse(const Vec3& impulse, const Vec3& local_poi)
	{
		if(active)
		{
			if(can_rotate)
				rot += inv_moi * LocalForceToTorque(impulse, local_poi);
			vel += impulse * inv_mass;
		}
	}

	void RigidBody::ApplyCentralImpulse(const Vec3& impulse) { if(active) { vel += impulse * inv_mass; } }



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

	MassInfo RigidBody::GetMassInfo() { return mass_info; }

	void RigidBody::DebugDraw(SceneRenderer* renderer) { shape->DebugDraw(renderer, pos, ori); }

	void RigidBody::SetCollisionCallback(CollisionCallback* callback) { collision_callback = callback; }
	CollisionCallback* RigidBody::GetCollisionCallback() { return collision_callback; }

	CollisionShape* RigidBody::GetCollisionShape() { return shape; }

	Entity* RigidBody::GetUserEntity() { return user_entity; }
	void RigidBody::SetUserEntity(Entity* entity) { user_entity = entity; }
}
