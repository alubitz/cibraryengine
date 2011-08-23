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

			Mat4 xform;

			Sphere bs;

			RigidBodyInfo* rigid_body;
			PhysicsWorld* physics;

			Rubbish(GameState* gs, UberModel* model, Vec3 pos, Quaternion ori);

			void Vis(SceneRenderer* renderer);
			void VisCleanup();

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);

			void Update(TimingInfo time);
	};
}
