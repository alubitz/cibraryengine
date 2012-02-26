#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Vector.h"
#include "Quaternion.h"
#include "TimingInfo.h"
#include "Events.h"

namespace CibraryEngine
{
	using namespace std;

	class RigidBody;

	struct Mat4;
	struct Quaternion;

	class SceneRenderer;

	class Entity;

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
			void AddRigidBody(RigidBody* r);
			/** Removes a rigid body from the simulation */
			bool RemoveRigidBody(RigidBody* r);

			/** Steps the simulation */
			void Update(TimingInfo time);

			void DebugDrawWorld(SceneRenderer* renderer);

			Vec3 GetGravity();
			void SetGravity(const Vec3& gravity);
	};

	struct MassInfo;
	class CollisionShape;
	class CollisionCallback;

	/** Class representing a rigid body */
	class RigidBody : public Disposable
	{
		friend class PhysicsWorld;

		private:

			struct Imp;
			Imp* imp;

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

			/** Gets a 4x4 transformation matrix representing the position and orientation of this rigid body */
			Mat4 GetTransformationMatrix();			

			void ApplyForce(const Vec3& force, const Vec3& local_poi);
			void ApplyImpulse(const Vec3& impulse, const Vec3& local_poi);
			void ApplyCentralForce(const Vec3& force);
			void ApplyCentralImpulse(const Vec3& impulse);

			void ResetForces();

			void Update(TimingInfo time);			// to be called by the PhysicsWorld

			void DebugDraw(SceneRenderer* renderer);

			void SetCollisionCallback(CollisionCallback* callback);
			CollisionCallback* GetCollisionCallback();

			CollisionShape* GetCollisionShape();

			Entity* GetUserEntity();
			void SetUserEntity(Entity* entity);
	};

	/** A point of contact between two physics objects */
	struct ContactPoint
	{
		RigidBody* obj_a;
		RigidBody* obj_b;

		Vec3 pos_a, pos_b;
		Vec3 norm_a, norm_b;
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

		/** Adds two MassInfo objects */
		void operator +=(MassInfo other);
		/** Adds two MassInfo objects */
		MassInfo operator +(MassInfo other);

		/** Scales the mass and MoI components of a MassInfo, leaving the CoM unchanged */
		void operator *=(float coeff);
		/** Scales the mass and MoI components of a MassInfo, leaving the CoM unchanged */
		MassInfo operator *(float coeff);

		/** Computes the moment of inertia about a parallel axis, with pivot point translated by the given vector (i.e. parallel axis theorem in 3 dimensions) */
		static void GetAlternatePivotMoI(Vec3 a, float* I, float m, float* result);

		static MassInfo FromCollisionShape(CollisionShape* shape, float mass);

		// serialization and deserialization functions
		static MassInfo ReadMassInfo(istream& stream);
		void Write(ostream& stream);
	};

	class CollisionCallback
	{
		public:

			/** A collision has occurred! Return whether or not to do the normal collision response behavior */
			virtual bool OnCollision(const ContactPoint& collision) = 0;
	};
}
