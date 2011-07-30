#include "StdAfx.h"
#include "Particle.h"

namespace Test
{
	/*
	 * Particle methods
	 */
	Particle::Particle(GameState* gs, Vec3 pos, Vec3 vel, ParticleMaterial* mat, float radius, float lifetime) : Entity(gs), material(mat), pos(pos), vel(vel), radius(radius), angle(Random3D::Rand(2 * M_PI)), age(0), max_age(lifetime) { }

	void Particle::Update(TimingInfo time)
	{
		float timestep = time.elapsed;

		age += timestep;
		if (age >= max_age)
		{
			is_valid = false;
			return;
		}

		pos += vel * timestep;

		vel.y -= gravity * timestep;
		vel *= exp(-damp * timestep);
	}

	void Particle::Vis(SceneRenderer* scene)
	{
		if (age < max_age && age >= 0)
			if (scene->camera->CheckSphereVisibility(Sphere(pos, radius)))
			{
				ParticleMaterialNodeData* node_data = new ParticleMaterialNodeData(pos, radius, angle, scene->camera);		// deleted by ParticleMaterial::Cleanup
				node_data->third_coord = age / max_age;
				scene->objects.push_back(RenderNode(material, node_data, Vec3::Dot(scene->camera->GetForward(), pos)));
			}
	}

	void Particle::VisCleanup(SceneRenderer* scene) { }
}
