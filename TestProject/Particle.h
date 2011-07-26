#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace CibraryEngine;

	class Particle : public Entity
	{
		public:

			ParticleMaterial* material;

			Vec3 pos, vel;
			float radius, angle;
			float age, max_age;

			float gravity, damp;

			Particle(GameState* gs, Vec3 pos, Vec3 vel, ParticleMaterial* mat, float radius, float lifetime);

			void Update(TimingInfo time);

			void Vis(SceneRenderer* scene);
			void VisCleanup(SceneRenderer* scene);
	};
}
