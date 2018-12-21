#include "StdAfx.h"
#include "DynamicsObject.h"

#include "Physics.h"

namespace CibraryEngine
{
	/*
	 * DynamicsObject methods
	 */
	DynamicsObject::DynamicsObject(Entity* user_entity, CollisionObjectType type, const MassInfo& mass_info, const Vec3& pos) :
		CollisionObject(user_entity, type),
		pos(pos),
		vel(),
		force(),
		applied_force(),
		gravity(),
		mass_info(mass_info),
		inv_mass(mass_info.mass > 0.0f ? 1.0f / mass_info.mass : 0.0f),
		xform_valid(false),
		restitution(1.0f),
		friction(1.0f),
		linear_damp(0.1f),
		deactivation_timer(0.5f)
	{
		active = can_move = mass_info.mass > 0.0f;
	}

	void DynamicsObject::ResetForces() { applied_force = can_move ? gravity * mass_info.mass : Vec3(); }

	void DynamicsObject::ResetToApplied() { force = applied_force; }

	void DynamicsObject::UpdateVel(float timestep)
	{
		if(active)
		{
			vel += force * (inv_mass * timestep);
			vel *= expf(-linear_damp * timestep);
		}

		ResetToApplied();
	}

	void DynamicsObject::UpdatePos(float timestep, PhysicsRegionManager* region_man)
	{
		if(active)
		{
			pos += vel * timestep;
			xform_valid = false;
			region_man->OnObjectUpdate(this, regions, timestep);
		}
	}

	void DynamicsObject::ApplyCentralImpulse(const Vec3& impulse)			{ if(active) { vel += impulse * inv_mass; } }

	Vec3 DynamicsObject::GetPosition() const								{ return pos; }
	void DynamicsObject::SetPosition(const Vec3& pos_)						{ pos = pos_; xform_valid = false; }

	void DynamicsObject::SetRestitution(float restitution_)					{ restitution = restitution_; }
	void DynamicsObject::SetFriction(float friction_)						{ friction = friction_; }
	float DynamicsObject::GetRestitution() const							{ return restitution; }
	float DynamicsObject::GetFriction() const								{ return friction; }

	void DynamicsObject::ApplyCentralForce(const Vec3& force)				{ applied_force += force; }

	Vec3 DynamicsObject::GetLinearVelocity() const							{ return vel; }
	void DynamicsObject::SetLinearVelocity(const Vec3& vel_)				{ vel = vel_; }

	void DynamicsObject::SetGravity(const Vec3& grav)						{ gravity = grav; }
	void DynamicsObject::SetDamp(float damp)								{ linear_damp = damp; }

	Vec3 DynamicsObject::GetGravity() const									{ return gravity; }

	MassInfo DynamicsObject::GetMassInfo() const							{ return mass_info; }
	float DynamicsObject::GetMass() const									{ return mass_info.mass; }
}
