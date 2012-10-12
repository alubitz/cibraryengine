#pragma once

#include "StdAfx.h"
#include "CollisionObject.h"

#include "AABB.h"

#include "Physics.h"
#include "PhysicsRegion.h"

namespace CibraryEngine
{
	class RigidBody;

	class CollisionGroup : public CollisionObject
	{
		friend class RayCollider;

		protected:

			class DummyRegionManager : public PhysicsRegionManager
			{
				public:

					DummyRegionManager();

					void OnObjectAdded(		CollisionObject* object, RegionSet& object_regions) { }
					void OnObjectUpdate(	CollisionObject* object, RegionSet& object_regions, float timestep) { }
					void OnObjectRemoved(	CollisionObject* object, RegionSet& object_regions) { }
					PhysicsRegion* GetRegion(const Vec3& point) { return NULL; }
					void GetRegionsOnRay(const Vec3& from, const Vec3& to, set<PhysicsRegion*>& results) { }
			};
			DummyRegionManager* dummy_region_man;

			boost::unordered_set<RigidBody*> children;
			bool collide_within;

			Vec3 gravity;

			void InnerDispose();

		public:

			CollisionGroup(Entity* entity);


			void UpdateVel(float timestep);
			void UpdatePos(float timestep, PhysicsRegionManager* region_man);

			void InitiateCollisions(float timestep, vector<ContactPoint>& contact_points);

			void DebugDraw(SceneRenderer* renderer);

			AABB GetAABB(float timestep);

			void AddChild(RigidBody* child);
			void RemoveChild(RigidBody* child);

			bool AreInternalCollisionsEnabled()			{ return collide_within; }
			void SetInternalCollisionsEnabled(bool yn)	{ collide_within = yn; }

			void SetGravity(const Vec3& gravity);

			void CollideRigidBody(RigidBody* body, vector<ContactPoint>& contact_points);
			void CollideCollisionGroup(CollisionGroup* other, vector<ContactPoint>& contact_points);

			void ResetForces();
	};
}
