#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Vector.h"
#include "Quaternion.h"
#include "TimingInfo.h"
#include "Events.h"

#include "MassInfo.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;
	using boost::unordered_set;

	struct Mat4;

	class RigidBody;
	class CollisionCallback;

	class SceneRenderer;

	class Entity;

	/** Class for a physical simulation */
	class PhysicsWorld : public Disposable
	{
		friend class RigidBody;

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
			void RemoveRigidBody(RigidBody* r);

			/** Steps the simulation */
			void Update(TimingInfo time);

			void DebugDrawWorld(SceneRenderer* renderer);

			Vec3 GetGravity();
			void SetGravity(const Vec3& gravity);

			void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback);
	};

	class RigidBody;

	/** A point of contact between two physics objects */
	struct ContactPoint
	{
		struct Part
		{
			RigidBody* obj;
			Vec3 pos, norm;					// both are world coords

			Part() : obj(NULL), pos(), norm() { }
		} a, b;
	};

	struct RayResult
	{
		float t;
		ContactPoint p;

		RayResult(float t, const ContactPoint& p) : t(t), p(p) { }
		bool operator <(const RayResult& h) { return t < h.t; }
	};

	class CollisionCallback
	{
		public:

			/** A collision has occurred! Return whether or not to do the normal collision response behavior */
			virtual bool OnCollision(const ContactPoint& collision) = 0;
	};
}
