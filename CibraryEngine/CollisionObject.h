#pragma once

#include "StdAfx.h"

#include "Disposable.h"

#include "SmartHashSet.h"

namespace CibraryEngine
{
	struct AABB;
	struct Vec3;
	class SceneRenderer;

	class PhysicsRegion;
	class PhysicsRegionManager;

	struct ConstraintGraph;
	struct ContactPoint;

	class Entity;

	class CollisionObject;

	using namespace std;

	typedef SmartHashSet<PhysicsRegion, 7> RegionSet;
	typedef SmartHashSet<CollisionObject, 17> RelevantObjectsQuery;

	enum CollisionObjectType
	{
		COT_RigidBody = 1,
		COT_RayCollider = 2,
		COT_CollisionGroup = 3,

		COT_CollisionObjectTypeMax
	};

	class CollisionObject : public Disposable
	{
		friend class PhysicsWorld;
		friend class PhysicsRegion;

		private:

			CollisionObjectType type;

		protected:

			bool can_move;

			RegionSet regions;
			set<CollisionObject*> disabled_collisions;

			Entity* user_entity;

			virtual void RemoveDisabledCollisions(RelevantObjectsQuery& eligible_bodies);

		public:

			CollisionObject(Entity* user_entity, CollisionObjectType type);

			virtual void UpdateVel(float timestep) { }
			virtual void UpdatePos(float timestep, PhysicsRegionManager* region_man) { }
			virtual void InitiateCollisions(float timestep, vector<ContactPoint*>& contact_points) { };

			virtual void DebugDraw(SceneRenderer* renderer) { }

			void SetCollisionEnabled(CollisionObject* other, bool enabled);

			virtual AABB GetAABB(float timestep) = 0;

			CollisionObjectType GetType() { return type; }

			Entity* GetUserEntity();
			void SetUserEntity(Entity* entity);

			virtual void SetGravity(const Vec3& gravity) { }

			virtual void ResetForces() { }
	};
}
