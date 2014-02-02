#include "StdAfx.h"
#include "Particle.h"

namespace Test
{
	/*
	 * Particle methods
	 */
	Particle::Particle(GameState* gs, const Vec3& pos, const Vec3& vel, ParticleMaterial* mat, BillboardMaterial* billboard_mat, float radius, float lifetime) : Entity(gs), collider(NULL), material(mat), pos(pos), vel(vel), radius(radius), angle(Random3D::Rand(float(2 * M_PI))), age(0), max_age(lifetime), trailhead(NULL), billboard_mat(billboard_mat) { }

	void Particle::InnerDispose()
	{
		if(collider) { collider->Dispose(); delete collider; collider = NULL; }

		if(trailhead)
		{
			trailhead->head_free = true;

			if(trailhead->trail_free) { delete trailhead; trailhead = NULL; }
		}
	}

	void Particle::Spawned()
	{
		collider = new RayCollider(this, pos, vel, 0.000001f);
		collider->SetDamp(damp);

		game_state->physics_world->AddCollisionObject(collider);

		collider->SetGravity(Vec3(0, -gravity, 0));			// have to set this after adding to world, or world gravity will override it

		if(billboard_mat)
		{
			trailhead = new TrailHead(this);
			game_state->Spawn(new BillboardTrail(game_state, trailhead, billboard_mat, radius));
		}
	}

	void Particle::DeSpawned()
	{
		if(collider)
			game_state->physics_world->RemoveCollisionObject(collider);
	}

	void Particle::Update(const TimingInfo& time)
	{
		float timestep = time.elapsed;

		age += timestep;
		if(age >= max_age)
		{
			if(trailhead)
				trailhead->particle = NULL;

			is_valid = false;
			return;
		}

		pos = collider->GetPosition();
		vel = collider->GetLinearVelocity();
	}

	void Particle::Vis(SceneRenderer* scene)
	{
		if(material)
			if(age < max_age && age >= 0)
				if(scene->camera->CheckSphereVisibility(Sphere(pos, radius)))
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
