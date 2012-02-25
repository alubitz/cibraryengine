#include "StdAfx.h"
#include "Shot.h"

#include "GlowyModelMaterial.h"
#include "Damage.h"
#include "Shootable.h"
#include "Dood.h"

namespace Test
{
	/*
	 * Shot methods
	 */
	Shot::Shot(GameState* gs, VertexBuffer* model, BillboardMaterial* material, Vec3 origin, Vec3 initial_vel, Dood* firer) :
		Entity(gs),
		bs(origin, -1),
		model(model),
		material(material),
		physics(NULL),
		origin(origin),
		initial_vel(initial_vel),
		mass(0.05f),
		body(NULL),
		causer(firer),
		firer(firer),
		trail_head(NULL)
	{
	}

	void Shot::InnerDispose()
	{
		Entity::InnerDispose();

		body->Dispose();
		delete body;
		body = NULL;
	}

	void Shot::Spawned()
	{
		physics = game_state->physics_world;

		body = new RigidBody(new RayShape(), MassInfo(Vec3(), mass), origin);
		body->SetLinearVelocity(initial_vel);
		physics->AddRigidBody(body);

		trail_head = new TrailHead(this);
		game_state->Spawn(new BillboardTrail(game_state, trail_head, material, 0.03f));
	}

	void Shot::DeSpawned()
	{
		if(body != NULL)
			physics->RemoveRigidBody(body);
	}

	void Shot::Update(TimingInfo time)
	{
		Entity::Update(time);

		/*
		TODO: add this to hit callback function

		Shootable* hit = callback.hits[i].obj;
		if(hit->GetShot(this, poi, GetMomentum()))
		{
			if(trail_head != NULL)
			{
				trail_head->end_pos = poi;
				trail_head->shot = NULL;
			}

			is_valid = false;
			return;
		}
		*/

		bs = Sphere(body->GetPosition(), 10.0f);
	}

	Damage Shot::GetDamage() { return Damage(firer, 0.09f); }			// was .03 in C# version, but it took too many shots to do 1 damage
	Vec3 Shot::GetMomentum() { return body->GetLinearVelocity() * mass; }




	/*
	 * Shot::TrailHead methods
	 */
	Shot::TrailHead::TrailHead(Shot* shot) : shot(shot), ended(false) { }
	bool Shot::TrailHead::operator ()(BillboardTrail::TrailNode& node)
	{
		if(shot == NULL)
		{
			if(ended)
				return false;
			else
			{
				node = BillboardTrail::TrailNode(end_pos, 0.0f, 0.1f);
				ended = true;
				return true;
			}
		}
		else
		{
			node = BillboardTrail::TrailNode(shot->body->GetPosition(), 0.0f, 0.1f);
			return true;
		}
	}
}
