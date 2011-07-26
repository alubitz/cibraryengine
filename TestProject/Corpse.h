#pragma once

#include "../CibraryEngine/CibraryEngine.h"

#include "Shootable.h"
#include "Dood.h"

namespace Test
{
	using namespace std;

	class Corpse : public Entity, public Shootable
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			Corpse(GameState* gs, Dood* dood);

			void Update(TimingInfo time);

			void Spawned();
			void DeSpawned();

			void Vis(SceneRenderer* renderer);

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);

			Vec3 GetPosition();;
	};
}
