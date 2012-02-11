#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Vector.h"
#include "Quaternion.h"
#include "TimingInfo.h"
#include "Events.h"

#include "btBulletDynamicsCommon.h"

namespace CibraryEngine
{
	using namespace std;

	class RigidBodyInfo;
	class ConeTwistConstraint;

	struct Mat4;
	struct Quaternion;

	/** Class for a physical simulation */
	class PhysicsWorld : public Disposable
	{
		private:

			struct Imp;
			Imp* imp;

			// no copying!
			void operator=(PhysicsWorld& other) { }
			PhysicsWorld(PhysicsWorld& other) { }

		protected:

			void InnerDispose();

		public:

			/** Initializes a PhysicsWorld */
			PhysicsWorld();

			/** Adds a rigid body to the simulation */
			void AddRigidBody(RigidBodyInfo* r);
			/** Removes a rigid body from the simulation */
			bool RemoveRigidBody(RigidBodyInfo* r);

			void AddConstraint(ConeTwistConstraint* constraint, bool disable_collision = false);
			void RemoveConstraint(ConeTwistConstraint* constraint);

			/** Steps the simulation */
			void Update(TimingInfo time);

			void SetDebugDrawer(btIDebugDraw* d);
			void DebugDrawWorld();

			Vec3 GetGravity();
			void SetGravity(const Vec3& gravity);

			void RayTest(Vec3 from, Vec3 to, btCollisionWorld::RayResultCallback& callback);
			void ContactTest(RigidBodyInfo* object, btCollisionWorld::ContactResultCallback& callback);
	};

	struct MassInfo;

	/** Class representing a rigid body */
	class RigidBodyInfo : public Disposable
	{
		friend class PhysicsWorld;
		friend class ConeTwistConstraint;

		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			/** Default constructor for a RigidBodyInfo; the constructed RigidBodyInfo will not work without setting the fields manually */
			RigidBodyInfo();
			/** Initializes a rigid body with the specified collision shape, mass properties, and optional position and orientation */
			RigidBodyInfo(btCollisionShape* shape, MassInfo mass_info, Vec3 pos = Vec3(), Quaternion ori = Quaternion::Identity());

			/** Disposes of this rigid body without disposing of and deleting the collision shape; by default RigidBodyInfo::Dispose will dispose of and delete the collision shape! */
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

			/** Gets a 4x4 transformation matrix representing the position and orientation of this rigid body */
			Mat4 GetTransformationMatrix();

			void Activate();
			void ApplyImpulse(const Vec3& impulse, const Vec3& local_poi);
			void ApplyCentralImpulse(const Vec3& impulse);
			void ApplyCentralForce(const Vec3& force);

			void ClearForces();

			void SetFriction(float friction);
			void SetDamping(float linear, float angular);
			void SetRestitution(float restitution);

			void SetDeactivationTime(float time);
			void SetSleepingThresholds(float linear, float angular);

			void SetCustomCollisionEnabled(void* user_object);
	};

	/** Class representing the mass properties of an object */
	struct MassInfo
	{
		/** The mass of the object */
		float mass;

		/** The object's center of mass */
		Vec3 com;

		/** The object's 3D moment of inertia about an axis passing through its center of mass */
		float moi[9];

		/** Initializes a zero mass */
		MassInfo();
		/** Initializes a MassInfo representing a point mass, at the specified location */
		MassInfo(Vec3 pos, float mass);

		/** Gets the 3-component vector format of MoI used by Bullet */
		Vec3 GetDiagonalMoI();

		/** Adds two MassInfo objects */
		void operator +=(MassInfo other);
		/** Adds two MassInfo objects */
		MassInfo operator +(MassInfo other);

		/** Computes the moment of inertia about a parallel axis, with pivot point translated by the given vector (i.e. parallel axis theorem in 3 dimensions) */
		static void GetAlternatePivotMoI(Vec3 a, float* I, float m, float* result);

		static MassInfo FromCollisionShape(btCollisionShape* shape, float mass);
	};

	class ConeTwistConstraint : public Disposable
	{
		friend class PhysicsWorld;

		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			ConeTwistConstraint(RigidBodyInfo* body_a, RigidBodyInfo* body_b, Quaternion a_ori, Vec3 a_pos, Quaternion b_ori, Vec3 b_pos);

			void SetLimit(const Vec3& limits);
			void SetDamping(float damp);

			void SetDesiredOrientation(const Vec3& vec);

			// because motors aren't working
			void Update(TimingInfo time);
	};

	void WriteCollisionShape(btCollisionShape* shape, ostream& stream);
	btCollisionShape* ReadCollisionShape(istream& stream);

}
