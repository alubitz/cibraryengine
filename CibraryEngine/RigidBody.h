#pragma once

#include "StdAfx.h"

#include "Disposable.h"

#include "Physics.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"

namespace CibraryEngine
{
	struct AABB;

	class PhysicsRegionManager;

	/** Class representing a rigid body */
	class RigidBody : public Disposable
	{
		friend class PhysicsWorld;
		friend class PhysicsRegion;

		private:

			set<PhysicsRegion*> regions;

			Vec3 pos;
			Vec3 vel;
			Quaternion ori;
			Vec3 rot;

			Vec3 force, torque;
			Vec3 applied_force, applied_torque;

			Vec3 gravity;

			MassInfo mass_info;
			CollisionShape* shape;

			// cached values for inverses of stuff
			float inv_mass;
			Mat3 inv_moi;

			bool xform_valid;
			Mat4 xform, inv_xform;

			float bounciness;
			float friction;
			float linear_damp, angular_damp;

			bool can_move, can_rotate;
			bool active;								// TODO: support deactivation and related stuffs

			Entity* user_entity;

			CollisionCallback* collision_callback;

			Mat3 ComputeInvMoi();

			void UpdateVel(float timestep);
			void UpdatePos(float timestep, PhysicsRegionManager* region_man);

			void ComputeXform();

			void ComputeXformAsNeeded();

			void ResetToApplied();

			// force is a world-space direction
			// local_poi is in the coordinate system of the object
			// returns a world-space direction
			Vec3 LocalForceToTorque(const Vec3& force, const Vec3& local_poi);

		protected:

			void InnerDispose();

		public:

			/** Default constructor for a RigidBody; the constructed RigidBody will not work without setting the fields manually */
			RigidBody();
			/** Initializes a rigid body with the specified collision shape, mass properties, and optional position and orientation */
			RigidBody(CollisionShape* shape, MassInfo mass_info, Vec3 pos = Vec3(), Quaternion ori = Quaternion::Identity());

			/** Disposes of this rigid body without disposing of and deleting the collision shape; by default RigidBody::Dispose will dispose of and delete the collision shape! */
			void DisposePreservingCollisionShape();

			/** Gets the position of this rigid body */
			Vec3 GetPosition();
			/** Sets the position of this rigid body */
			void SetPosition(Vec3 pos);

			/** Gets the orientation of this rigid body */
			Quaternion GetOrientation();
			/** Sets the orientation of this rigid body */
			void SetOrientation(Quaternion ori);

			Vec3 GetLinearVelocity();
			void SetLinearVelocity(const Vec3& vel);

			Vec3 GetAngularVelocity();
			void SetAngularVelocity(const Vec3& vel);

			// point is in world-space
			// returns a world-space velocity
			Vec3 GetLocalVelocity(const Vec3& point);

			/** Gets a 4x4 transformation matrix representing the position and orientation of this rigid body */
			Mat4 GetTransformationMatrix();			
			Mat4 GetInvTransform();

			MassInfo GetMassInfo();

			void SetBounciness(float bounciness);
			void SetFriction(float friction);
			float GetBounciness();
			float GetFriction();

			bool MergesSubgraphs();

			void ApplyForce(const Vec3& force, const Vec3& local_poi);

			// impulse is a world-space direction
			// local_poi is in the coordinate system of the object
			void ApplyImpulse(const Vec3& impulse, const Vec3& local_poi);
			void ApplyCentralForce(const Vec3& force);
			void ApplyCentralImpulse(const Vec3& impulse);

			void ResetForces();

			void DebugDraw(SceneRenderer* renderer);

			void SetCollisionCallback(CollisionCallback* callback);
			CollisionCallback* GetCollisionCallback();

			CollisionShape* GetCollisionShape();

			/** Gets AABB to be used for collision detection for this object. Timestep is only really relevant for rays and spheres */
			AABB GetAABB(float timestep);

			Entity* GetUserEntity();
			void SetUserEntity(Entity* entity);
	};
}
