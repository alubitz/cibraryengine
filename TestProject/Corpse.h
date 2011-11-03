#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	using namespace std;

	class Corpse : public Entity
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

			Vec3 GetPosition();
	};
}
