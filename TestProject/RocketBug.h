#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class Dood;

	class RocketBug : public Entity
	{
		protected:

			void InnerDispose();

		public:

			vector<Material*> materials;

			UberModel* model;
			ModelPhysics* mphys;

			PhysicsWorld* physics;
			RigidBody* rigid_body;

			Dood* firer;

			Quaternion ori;
			Vec3 pos, vel;

			float arm_time, detonation_time;

			RocketBug(GameState* game_state, Dood* firer, UberModel* model, ModelPhysics* mphys, const Vec3& pos, const Vec3& vel, const Quaternion& ori);

			void Update(const TimingInfo& time);

			void Spawned();
			void DeSpawned();

			void Vis(SceneRenderer* renderer);


			void Explode();


			struct ImpactCallback : public CollisionCallback
			{
				RocketBug* bug;
				void OnCollision(const ContactPoint& collision);
			} impact_callback;
	};
};
