#pragma once

#include "StdAfx.h"

#include "DynamicsObject.h"

#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"

#include "AABB.h"

#include "CollisionShape.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	class Entity;

	class PhysicsConstraint;
	class PhysicsRegionManager;

	class ContactCallback;



	/** Class representing a rigid body */
	class RigidBody : public DynamicsObject
	{
		friend class RayCollider;
		friend class PhysicsWorld;
		friend class PhysicsRegion;
		friend struct ContactPoint;
		friend class SkeletalJointConstraint;
		friend class FixedJointConstraint;

		private:

			set<PhysicsConstraint*> constraints;

			Quaternion ori;
			Vec3 rot;

			Vec3 torque, applied_torque;

			CollisionShape* shape;
			ShapeInstanceCache* shape_cache;

			// cached values for inverses of stuff
			Mat3 inv_moi;

			Mat3 ori_rm;
			Mat4 xform, inv_xform;
			AABB cached_aabb;
			Vec3 cached_com;

			float angular_damp;

			bool can_rotate;

			ContactCallback* contact_callback;
			CollisionCallback* collision_callback;

			Mat3 ComputeInvMoi()  { return ori_rm * Mat3::Invert(Mat3(mass_info.moi)) * ori_rm.Transpose(); }

			void ComputeXform();

			void ComputeXformAsNeeded() { if(!xform_valid) { ComputeXform(); } }

			void ResetToApplied() { DynamicsObject::ResetToApplied(); torque = applied_torque; }

			// force is a world-space direction
			// local_poi is in the coordinate system of the object
			// returns a world-space direction
			Vec3 LocalForceToTorque(const Vec3& force, const Vec3& local_poi);

			// removes rigid bodies which are contrained with this one from the collection of eligible bodies
			void RemoveDisabledCollisions(RelevantObjectsQuery& eligible_bodies);

			void InitiateCollisionsForMultisphere(float timestep, ContactDataCollector* collect);

			void InitiateCollisionsForConvexMesh(float timestep, ContactDataCollector* collect);

		protected:

			void InnerDispose();

		public:

			/** Default constructor for a RigidBody; the constructed RigidBody will not work without setting the fields manually */
			RigidBody();
			/** Initializes a rigid body with the specified collision shape, mass properties, and optional position and orientation */
			RigidBody(Entity* user_entity, CollisionShape* shape, const MassInfo& mass_info, const Vec3& pos = Vec3(), const Quaternion& ori = Quaternion::Identity());

			
			void UpdateVel(float timestep);
			void UpdatePos(float timestep, PhysicsRegionManager* region_man);

			void InitiateCollisions(float timestep, ContactDataCollector* collect);

			

			/** Gets the orientation of this rigid body */
			Quaternion GetOrientation() const           { return ori; }
			/** Sets the orientation of this rigid body */
			void SetOrientation(const Quaternion& ori_) { ori = ori_; xform_valid = false; }

			

			Vec3 GetAngularVelocity() const             { return rot; }
			void SetAngularVelocity(const Vec3& vel)    { rot = vel; }

			
			// point is in world-space
			// returns a world-space velocity
			Vec3 GetLocalVelocity(const Vec3& point)    { ComputeXformAsNeeded(); return vel + Vec3::Cross(point - cached_com, rot); }

			/** Gets a 4x4 transformation matrix representing the position and orientation of this rigid body */
			Mat4 GetTransformationMatrix() { ComputeXformAsNeeded(); return xform; }
			Mat4 GetInvTransform()         { ComputeXformAsNeeded(); return inv_xform; }

			MassInfo GetTransformedMassInfo() const;

			Vec3 GetCenterOfMass()         { ComputeXformAsNeeded(); return cached_com; }

			/** Gets the inverse of the moment of inertia matrix, in the world coordinate system; assumes nothing has modified the orientation or mass info since the object was created or UpdateVel was called */
			Mat3 GetInvMoI() const         { return inv_moi; }

			

			bool MergesSubgraphs() const   { return shape->CanMove(); }



			void ApplyWorldForce(const Vec3& force, const Vec3& poi);

			// impulse is a world-space direction
			// local_poi is in the coordinate system of the object
			void ApplyLocalImpulse(const Vec3& impulse, const Vec3& local_poi);

			// like the above, but the position is in world coords
			void ApplyWorldImpulse(const Vec3& impulse, const Vec3& poi);

			
			void ApplyAngularImpulse(const Vec3& angular_impulse)  { rot += inv_moi * angular_impulse; }

			void DebugDraw(SceneRenderer* renderer) const          { shape->DebugDraw(renderer, pos, ori); }

			CollisionShape* GetCollisionShape() const              { return shape; };
			ShapeType GetShapeType() const                         { return shape->GetShapeType(); };

			void SetContactCallback(ContactCallback* callback)     { contact_callback = callback; }
			ContactCallback* GetContactCallback() const            { return contact_callback; }

			void SetCollisionCallback(CollisionCallback* callback) { collision_callback = callback; }
			CollisionCallback* GetCollisionCallback() const        { return collision_callback; }


			/** Gets AABB to be used for collision detection for this object. Timestep is only really relevant for rays and spheres */
			AABB GetAABB(float timestep);

			/** Get AABB for this object, recomputing if necessary; not for rays; may be iffy for spheres */
			AABB GetCachedAABB()                                   { ComputeXformAsNeeded(); return cached_aabb; }



			void CollideRigidBody(RigidBody* other, ContactDataCollector* collect);

			void ResetForces()                                     { DynamicsObject::ResetForces(); applied_torque = Vec3(); }
	};
}
