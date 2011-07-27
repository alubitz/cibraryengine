#pragma once

#include "../CibraryEngine/CibraryEngine.h"

#include "Damage.h"

namespace Test
{
	using namespace CibraryEngine;

	class Shootable;
	class Dood;
	class GlowyModelMaterial;
	class GlowyModelMaterialNodeData;

	class Shot : public Entity
	{
		protected:

			void InnerDispose();

		public:

			Sphere bs;
			VTNModel* model;
			GlowyModelMaterial* material;

			PhysicsWorld* physics;

			Vec3 origin;						// for determining from what direction you're taking damage!

			Vec3 pos;
			Vec3 vel;

			Quaternion ori;
			Mat4 draw_xform;

			void* causer;						// was IDamageBlame in the C# version
			Dood* firer;

			float mass;

			Shot(GameState* gs, VTNModel* model, GlowyModelMaterial* material, Vec3 origin, Vec3 vel, Quaternion ori, Dood* firer);

			void Vis(SceneRenderer* renderer);
			void VisCleanup();
			void Spawned();

			virtual void Update(TimingInfo time);

			virtual Damage GetDamage();
			virtual Vec3 GetMomentum();

			struct HitObject
			{
				Shootable* obj;
				float time;

				HitObject(Shootable* obj, float time) : obj(obj), time(time) { }
			};

			struct MyRayResultCallback : public btCollisionWorld::RayResultCallback
			{
				Shot* shot;
				vector<HitObject> hits;

				MyRayResultCallback (Shot* shot);
				btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace);
			};
	};
}