#pragma once

#include "StdAfx.h"

#include "Disposable.h"

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

	using namespace std;

	enum CollisionObjectType
	{
		COT_RigidBody = 1,
		COT_RayCollider = 2,
		COT_CollisionGroup = 3,

		COT_CollisionObjectTypeMax
	};

	struct RegionSet
	{
		static const unsigned int hash_size = 7;
				
		vector<PhysicsRegion*> buckets[hash_size];
		unsigned int count;

		RegionSet();

		void Insert(PhysicsRegion* region);
		void Erase(PhysicsRegion* region);

		void Clear();
	};

	class CollisionObject : public Disposable
	{
		friend class PhysicsWorld;
		friend class PhysicsRegion;
		friend struct RelevantObjectsQuery;

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
			virtual void InitiateCollisions(float timestep, vector<ContactPoint>& contact_points) { };

			virtual void DebugDraw(SceneRenderer* renderer) { }

			void SetCollisionEnabled(CollisionObject* other, bool enabled);

			virtual AABB GetAABB(float timestep) = 0;

			CollisionObjectType GetType() { return type; }

			Entity* GetUserEntity();
			void SetUserEntity(Entity* entity);

			virtual void SetGravity(const Vec3& gravity) { }

			virtual void ResetForces() { }
	};

	struct RelevantObjectsQuery
	{
		static const unsigned int hash_size = 17;

		vector<CollisionObject*> buckets[hash_size];
		unsigned int count;

		RelevantObjectsQuery();
		~RelevantObjectsQuery();

		void Insert(CollisionObject* object);
		void Erase(CollisionObject* object);

		void Clear();
	};
}
