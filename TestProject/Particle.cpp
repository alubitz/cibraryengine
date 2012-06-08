#include "StdAfx.h"
#include "Particle.h"

namespace Test
{
	/*
	 * Particle methods
	 */
	Particle::Particle(GameState* gs, Vec3 pos, Vec3 vel, ParticleMaterial* mat, BillboardMaterial* billboard_mat, float radius, float lifetime) : Entity(gs), body(NULL), material(mat), pos(pos), vel(vel), radius(radius), angle(Random3D::Rand(float(2 * M_PI))), age(0), max_age(lifetime), trailhead(NULL), billboard_mat(billboard_mat) { }

	void Particle::InnerDispose()
	{
		if(body != NULL)
		{
			body->Dispose();
			delete body;
			body = NULL;
		}

		if(trailhead != NULL)
		{
			trailhead->head_free = true;
			if(trailhead->trail_free)
			{
				delete trailhead;
				trailhead = NULL;
			}
		}
	}

	void Particle::Spawned()
	{
		body = new RigidBody(new RayShape(), MassInfo(Vec3(), 0.000001f), pos);
		body->SetLinearVelocity(vel);
		body->SetUserEntity(this);
		body->SetDamp(damp);

		game_state->physics_world->AddRigidBody(body);

		body->SetGravity(Vec3(0, -gravity, 0));			// have to set this after adding to world, or world gravity will override it

		if(billboard_mat != NULL)
		{
			trailhead = new TrailHead(this);
			game_state->Spawn(new BillboardTrail(game_state, trailhead, billboard_mat, radius));
		}
	}

	void Particle::DeSpawned()
	{
		if(body != NULL)
			game_state->physics_world->RemoveRigidBody(body);
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

		pos = body->GetPosition();
		vel = body->GetLinearVelocity();
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
