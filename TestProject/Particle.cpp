#include "StdAfx.h"
#include "Particle.h"

namespace Test
{
	/*
	 * Particle methods
	 */
	Particle::Particle(GameState* gs, Vec3 pos, Vec3 vel, ParticleMaterial* mat, BillboardMaterial* billboard_mat, float radius, float lifetime) : Entity(gs), material(mat), pos(pos), vel(vel), radius(radius), angle(Random3D::Rand(float(2 * M_PI))), age(0), max_age(lifetime), trailhead(NULL), billboard_mat(billboard_mat) { }

	void Particle::Spawned()
	{
		if(billboard_mat != NULL)
		{
			trailhead = new TrailHead(this);
			game_state->Spawn(new BillboardTrail(game_state, trailhead, billboard_mat, radius));
		}
	}

	void Particle::Update(TimingInfo time)
	{
		float timestep = time.elapsed;

		age += timestep;
		if (age >= max_age)
		{
			if(trailhead != NULL)
				trailhead->particle = NULL;

			is_valid = false;
			return;
		}

		pos += vel * timestep;

		vel.y -= gravity * timestep;
		vel *= exp(-damp * timestep);
	}

	void Particle::Vis(SceneRenderer* scene)
	{
		if(material != NULL)
			if (age < max_age && age >= 0)
				if (scene->camera->CheckSphereVisibility(Sphere(pos, radius)))
				{
					ParticleMaterialNodeData* node_data = new ParticleMaterialNodeData(pos, radius, angle, scene->camera);		// deleted by ParticleMaterial::Cleanup
					node_data->third_coord = age / max_age;
					scene->objects.push_back(RenderNode(material, node_data, Vec3::Dot(scene->camera->GetForward(), pos)));
				}
	}

	void Particle::VisCleanup(SceneRenderer* scene) { }




	/* 
	 * Particle::TrailHead methods
	 */
	Particle::TrailHead::TrailHead(Particle* particle) : particle(particle) { }

	bool Particle::TrailHead::operator()(BillboardTrail::TrailNode& node)
	{
		if(particle == NULL)
			return false;

		node = BillboardTrail::TrailNode(particle->pos, 0, particle->max_age);
		return true;
	}
}
