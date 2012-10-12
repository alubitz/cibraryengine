#pragma once
#include "StdAfx.h"

#include "CollisionObject.h"

#include "Vector.h"
#include "MassInfo.h"

namespace CibraryEngine
{
	class CollisionCallback;

	class DynamicsObject : public CollisionObject
	{
		protected:

			Vec3 pos, vel;

			Vec3 force, applied_force;
			Vec3 gravity;

			MassInfo mass_info;							// a float mass would be redundant if there's a full MassInfo in a derived class
			float inv_mass;

			bool xform_valid;

			float bounciness;
			float friction;
			float linear_damp;

			bool active;								// TODO: support deactivation and related stuffs
			float deactivation_timer;

			virtual void ResetToApplied();

		public:

			DynamicsObject(Entity* user_entity, CollisionObjectType type, const MassInfo& mass_info, const Vec3& pos = Vec3());

			virtual void UpdateVel(float timestep);
			virtual void UpdatePos(float timestep, PhysicsRegionManager* region_man);

			/** Gets the position of this rigid body */
			Vec3 GetPosition() const;
			/** Sets the position of this rigid body */
			void SetPosition(const Vec3& pos);

			Vec3 GetLinearVelocity() const;
			void SetLinearVelocity(const Vec3& vel);

			void SetGravity(const Vec3& grav);			// overrides method in CollisionObject
			void SetDamp(float damp);

			MassInfo GetMassInfo() const;
			float GetMass() const;

			void SetBounciness(float bounciness);
			void SetFriction(float friction);
			float GetBounciness() const;
			float GetFriction() const;

			void Activate() { active = true; deactivation_timer = 0.5f; }

			void ApplyCentralForce(const Vec3& force);
			void ApplyCentralImpulse(const Vec3& impulse);

			virtual void ResetForces();					// overrides method in CollisionObject
	};
}