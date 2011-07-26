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
	class Mat4;
	class Quaternion;

	/** Class for a physical simulation */
	class PhysicsWorld : public Disposable
	{
		private:

			// no copying!
			void operator=(PhysicsWorld& other) { }
			PhysicsWorld(PhysicsWorld& other) { }

		protected:

			void InnerDispose();

		public:

			/** Bullet Physics Library stuff */
			btBroadphaseInterface* broadphase;
			/** Bullet Physics Library stuff */
			btDefaultCollisionConfiguration* collision_configuration;
			/** Bullet Physics Library stuff */
			btCollisionDispatcher* dispatcher;
			/** Bullet Physics Library stuff */
			btSequentialImpulseConstraintSolver* solver;
			/** Bullet Physics Library stuff */
			btDynamicsWorld* dynamics_world;

			/** List of all of the rigid bodies in the physical simulation */
			list<RigidBodyInfo*> rigid_bodies;

			/** Initializes a PhysicsWorld */
			PhysicsWorld(); 

			/** Adds a rigid body to the simulation */
			void AddRigidBody(RigidBodyInfo* r);
			/** Removes a rigid body from the simulation */
			bool RemoveRigidBody(RigidBodyInfo* r);

			/** Steps the simulation */
			void Update(TimingInfo time);
	};

	struct MassInfo;

	/** Class representing a rigid body */
	class RigidBodyInfo : public Disposable
	{
		protected:

			void InnerDispose();

		public:

			/** The shape of the object */
			btCollisionShape* shape;
			/** Bullet Physics Library's rigid body */
			btRigidBody* body;

			/** The position and orientation of this object */
			btMotionState* motion_state;

			/** Default constructor for a RigidBodyInfo; the constructed RigidBodyInfo will not work without setting the fields manually */
			RigidBodyInfo() : shape(NULL), body(NULL), motion_state(NULL) { }
			/** Initializes a rigid body with the specified collision shape, mass properties, and optional position and orientation */
			RigidBodyInfo(btCollisionShape* shape, MassInfo mass_info, Vec3 pos = Vec3(), Quaternion ori = Quaternion::Identity());

			/** Disposes of this rigid body without disposing of and deleting the collision shape; by default RigidBodyInfo::Dispose will dispose of and delete the collision shape! */
			void DisposePreservingCollisionShape();

			/** Gets the position of this rigid body */
			Vec3 GetPosition();
			/** Sets the position of this rigid body */
			void SetPosition(Vec3 pos);
			/** Sets the orientation of this rigid body */
			void SetOrientation(Quaternion ori);

			/** Gets a 4x4 transformation matrix representing the position and orientation of this rigid body */
			Mat4 GetTransformationMatrix();
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
	};

	void WriteCollisionShape(btCollisionShape* shape, ostream& stream);
	btCollisionShape* ReadCollisionShape(istream& stream);

}
