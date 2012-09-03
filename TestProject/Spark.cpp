#include "StdAfx.h"
#include "Spark.h"

namespace Test
{
	/*
	 * Spark methods
	 */
	Spark::Spark(GameState* gs, Vec3 pos, BillboardMaterial* billboard_mat) :
		Entity(gs),
		body(NULL),
		mass(Random3D::Rand(0.001f, 0.01f)),
		collider(NULL),
		pos(pos),
		vel(Random3D::RandomNormalizedVector(Random3D::Rand(2, 5) + Random3D::Rand() * Random3D::Rand(5)) * 0.005f / mass),
		age(0),
		max_age(0.2f + Random3D::Rand(3)),
		trailhead(NULL),
		billboard_mat(billboard_mat)
	{
	}

	void Spark::InnerDispose()
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

		if(collider)
		{
			delete collider;
			collider = NULL;
		}
	}

	void Spark::Spawned()
	{
		body = new RigidBody(new RayShape(), MassInfo(Vec3(), 0.000001f), pos);
		body->SetLinearVelocity(vel);
		body->SetUserEntity(this);

		game_state->physics_world->AddRigidBody(body);

		trailhead = new TrailHead(this);
		game_state->Spawn(new BillboardTrail(game_state, trailhead, billboard_mat, 0.01f));

		collider = new Collider(this);
		body->SetCollisionCallback(collider);
	}

	void Spark::DeSpawned()
	{
		if(body != NULL)
			game_state->physics_world->RemoveRigidBody(body);
	}

	void Spark::Update(TimingInfo time)
	{
		float timestep = time.elapsed;

		age += timestep;
		if(age >= max_age)
		{
			if(trailhead != NULL)
				trailhead->spark = NULL;

			is_valid = false;
			return;
		}

		pos = body->GetPosition();
		vel = body->GetLinearVelocity();
	}




	/* 
	 * Spark::TrailHead methods
	 */
	Spark::TrailHead::TrailHead(Spark* spark) : spark(spark) { }

	bool Spark::TrailHead::operator()(BillboardTrail::TrailNode& node)
	{
		if(spark == NULL)
			return false;

		float age_frac = spark->age / spark->max_age;
		node = BillboardTrail::TrailNode(spark->pos, age_frac * 0.1f, 0.1f);
		return true;
	}
}
