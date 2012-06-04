#pragma once

#include "StdAfx.h"

#include "Disposable.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	using namespace std;
	using boost::unordered_set;

	class RigidBody;
	struct CollisionGraph;
	class CollisionCallback;
	class SceneRenderer;

	class PhysicsRegion : public Disposable
	{
		protected:

			unordered_set<RigidBody*> rigid_bodies;
			unordered_set<RigidBody*> shape_bodies[ST_ShapeTypeMax];

			void InnerDispose();

		public:

			PhysicsRegion();



			// create a new rigid body within this region
			void AddRigidBody(RigidBody* body);

			// remove a rigid body from the simulation (true if the body was present)
			bool RemoveRigidBody(RigidBody* body);

			// take ownership of a rigid body from another region
			void TakeOwnership(RigidBody* body);

			// relinquish ownership of a rigid body (for when another region takes ownership)
			void Disown(RigidBody* body);


			void DebugDrawRegion(SceneRenderer* renderer);


			void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL);

			void UpdateVel(float timestep);
			void UpdatePos(float timestep);

			void ResetForces();
			void SetGravity(const Vec3& gravity);


			// conveniently doesn't need to know about any of that collision graph business
			void DoRayUpdates(float timestep, CollisionCallback& callback);

			void AddCollisions(float timestep, CollisionGraph& collision_graph);
	};
}
