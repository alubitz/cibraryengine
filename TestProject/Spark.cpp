#include "StdAfx.h"
#include "Spark.h"

namespace Test
{
	/*
	 * Spark methods
	 */
	Spark::Spark(GameState* gs, const Vec3& pos, BillboardMaterial* billboard_mat) :
		Entity(gs),
		collider(NULL),
		mass(Random3D::Rand(0.001f, 0.01f)),
		callback(NULL),
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
		if(collider) { collider->Dispose(); delete collider; collider = NULL; }

		if(trailhead)
		{
			trailhead->head_free = true;
			if(trailhead->trail_free) { delete trailhead; trailhead = NULL; }
		}

		if(callback) { delete callback; callback = NULL; }
	}

	void Spark::Spawned()
	{
		collider = new RayCollider(this, pos, vel, 0.000001f);
		game_state->physics_world->AddCollisionObject(collider);

		trailhead = new TrailHead(this);
		game_state->Spawn(new BillboardTrail(game_state, trailhead, billboard_mat, 0.01f));

		callback = new Collider(this);
		collider->SetRayCallback(callback);
	}

	void Spark::DeSpawned() { if(collider != NULL) { game_state->physics_world->RemoveCollisionObject(collider); } }

	void Spark::Update(const TimingInfo& time)
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

		pos = collider->GetPosition();
		vel = collider->GetLinearVelocity();
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
