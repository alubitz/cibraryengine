#pragma once

#include "StdAfx.h"

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

			Particle(GameState* gs, Vec3 pos, Vec3 vel, ParticleMaterial* mat, BillboardMaterial* billboard_mat, float radius, float lifetime);

			void Update(TimingInfo time);

			void Spawned();

			void Vis(SceneRenderer* scene);
			void VisCleanup(SceneRenderer* scene);

			struct TrailHead : public BillboardTrail::TrailHead
			{
				Particle* particle;

				TrailHead(Particle* particle);
				bool operator()(BillboardTrail::TrailNode& node);
			};

			TrailHead* trailhead;
			BillboardMaterial* billboard_mat;
	};
}
