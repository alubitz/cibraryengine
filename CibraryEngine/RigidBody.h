#pragma once

#include "StdAfx.h"

#include "DynamicsObject.h"

#include "Physics.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Matrix.h"

#include "AABB.h"

namespace CibraryEngine
{
	class Entity;

	class PhysicsRegionManager;



	/** Class representing a rigid body */
	class RigidBody : public DynamicsObject
	{
		friend class RayCollider;
		friend class PhysicsWorld;
		friend class PhysicsRegion;
		friend struct ContactPoint;
		friend class JointConstraint;

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

			CollisionCallback* collision_callback;

			Mat3 ComputeInvMoi();

			void ComputeXform();

			void ComputeXformAsNeeded();

			void ResetToApplied();

			// force is a world-space direction
			// local_poi is in the coordinate system of the object
			// returns a world-space direction
			Vec3 LocalForceToTorque(const Vec3& force, const Vec3& local_poi);

			// removes rigid bodies which are contrained with this one from the collection of eligible bodies
			void RemoveDisabledCollisions(RelevantObjectsQuery& eligible_bodies);

			void InitiateCollisionsForMultisphere(float timestep, ContactPointAllocator* alloc, vector<ContactPoint*>& contact_points);

			void InitiateCollisionsForConvexMesh(float timestep, ContactPointAllocator* alloc, vector<ContactPoint*>& contact_points);

		protected:

			void InnerDispose();

		public:

			/** Default constructor for a RigidBody; the constructed RigidBody will not work without setting the fields manually */
			RigidBody();
			/** Initializes a rigid body with the specified collision shape, mass properties, and optional position and orientation */
			RigidBody(Entity* user_entity, CollisionShape* shape, const MassInfo& mass_info, const Vec3& pos = Vec3(), const Quaternion& ori = Quaternion::Identity());

			
			void UpdateVel(float timestep);
			void UpdatePos(float timestep, PhysicsRegionManager* region_man);

			void InitiateCollisions(float timestep, ContactPointAllocator* alloc, vector<ContactPoint*>& contact_points);

			

			/** Gets the orientation of this rigid body */
			Quaternion GetOrientation();
			/** Sets the orientation of this rigid body */
			void SetOrientation(Quaternion ori);

			

			Vec3 GetAngularVelocity();
			void SetAngularVelocity(const Vec3& vel);

			
			// point is in world-space
			// returns a world-space velocity
			Vec3 GetLocalVelocity(const Vec3& point);

			/** Gets a 4x4 transformation matrix representing the position and orientation of this rigid body */
			Mat4 GetTransformationMatrix();			
			Mat4 GetInvTransform();

			MassInfo GetTransformedMassInfo() const;

			Vec3 GetCenterOfMass();

			/** Gets the inverse of the moment of inertia matrix, in the world coordinate system; assumes nothing has modified the orientation or mass info since the object was created or UpdateVel was called */
			Mat3 GetInvMoI();

			

			bool MergesSubgraphs();



			void ApplyWorldForce(const Vec3& force, const Vec3& poi);

			// impulse is a world-space direction
			// local_poi is in the coordinate system of the object
			void ApplyLocalImpulse(const Vec3& impulse, const Vec3& local_poi);

			// like the above, but the position is in world coords
			void ApplyWorldImpulse(const Vec3& impulse, const Vec3& poi);

			
			void ApplyAngularImpulse(const Vec3& angular_impulse);

			void DebugDraw(SceneRenderer* renderer);

			

			CollisionShape* GetCollisionShape();
			ShapeType GetShapeType();

			void SetCollisionCallback(CollisionCallback* callback);
			CollisionCallback* GetCollisionCallback() const;


			/** Gets AABB to be used for collision detection for this object. Timestep is only really relevant for rays and spheres */
			AABB GetAABB(float timestep);

			/** Get AABB for this object, recomputing if necessary; not for rays; may be iffy for spheres */
			AABB GetCachedAABB();



			void CollideRigidBody(RigidBody* other, ContactPointAllocator* alloc, vector<ContactPoint*>& contact_points);

			void ResetForces();
	};
}
