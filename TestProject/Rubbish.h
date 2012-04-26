#pragma once

#include "StdAfx.h"

#include "Shootable.h"

namespace Test
{
	class Rubbish : public Entity, public Shootable
	{
		protected:

			void InnerDispose();

		public:

			UberModel* model;
			vector<Material*> materials;
			ParticleMaterial* dirt_particle;

			Mat4 xform;

			Sphere bs;

			ModelPhysics* model_phys;

			RigidBody* rigid_body;
			PhysicsWorld* physics;

			Rubbish(GameState* gs, UberModel* model, ModelPhysics* model_phys, Vec3 pos, Quaternion ori, ParticleMaterial* dirt_particle);

			void Vis(SceneRenderer* renderer);

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);

			void Update(TimingInfo time);
	};
}
