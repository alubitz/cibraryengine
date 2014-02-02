#pragma once

#include "StdAfx.h"

#include "Damage.h"

namespace Test
{
	using namespace CibraryEngine;

	class Shootable;
	class Dood;

	class Shot : public Entity
	{
		protected:

			void InnerDispose();

		public:

			VertexBuffer* model;
			BillboardMaterial* material;

			PhysicsWorld* physics;

			Vec3 origin;						// for determining from what direction you're taking damage!
			Vec3 initial_vel;

			float mass;
			RayCollider* collider;

			void* causer;						// was IDamageBlame in the C# version
			Dood* firer;

			Shot(GameState* gs, VertexBuffer* model, BillboardMaterial* material, const Vec3& origin, const Vec3& vel, Dood* firer);

			void Spawned();
			void DeSpawned();

			virtual void Update(const TimingInfo& time);

			virtual Damage GetDamage();
			virtual void GetMomentumInfo(Vec3& vel_out, float& mass_out);

			struct TrailHead : public BillboardTrail::TrailHead
			{ 
				Shot* shot;

				Vec3 end_pos;
				bool ended;

				TrailHead(Shot* shot);
				bool operator()(BillboardTrail::TrailNode& node);
			};
			TrailHead* trail_head;

			struct MyImpactCallback : public RayCallback
			{
				Shot* shot;
				MyImpactCallback(Shot* shot);

				bool OnCollision(RayResult& rr);
			} impact_callback;
	};
}