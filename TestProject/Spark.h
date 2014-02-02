#pragma once
#include "StdAfx.h"

#include "Particle.h"

namespace Test
{
	using namespace CibraryEngine;

	class Spark : public Entity
	{
		protected:

			RayCollider* collider;
			float mass;

			void InnerDispose();

			struct Collider : public RayCallback
			{
				Spark* spark;
				Collider(Spark* spark) : spark(spark) { }

				bool OnCollision(RayResult& rr)
				{
					if(spark)
					{
						spark->age = spark->max_age - 0.9f * (spark->max_age - spark->age);
						return true;
					}
					return false;
				}
			};
			Collider* callback;

		public:
			
			Vec3 pos, vel;
			float age, max_age;

			Spark(GameState* gs, const Vec3& pos, BillboardMaterial* billboard_mat);

			void Update(const TimingInfo& time);

			virtual void Spawned();
			void DeSpawned();

			struct TrailHead : public BillboardTrail::TrailHead
			{
				Spark* spark;

				TrailHead(Spark* spark);
				bool operator()(BillboardTrail::TrailNode& node);
			};
			TrailHead* trailhead;

			BillboardMaterial* billboard_mat;
	};
}
