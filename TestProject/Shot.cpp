#include "StdAfx.h"
#include "Shot.h"

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
		model(model),
		material(material),
		physics(NULL),
		origin(origin),
		initial_vel(initial_vel),
		mass(1.05f),
		body(NULL),
		causer(firer),
		firer(firer),
		trail_head(NULL),
		impact_callback(this)
	{
	}

	void Shot::InnerDispose()
	{
		Entity::InnerDispose();

		body->Dispose();
		delete body;
		body = NULL;

		if(trail_head != NULL)
		{
			trail_head->head_free = true;
			if(trail_head->trail_free)
			{
				delete trail_head;
				trail_head = NULL;
			}
		}
	}

	void Shot::Spawned()
	{
		physics = game_state->physics_world;

		body = new RigidBody(new RayShape(), MassInfo(Vec3(), mass), origin);
		body->SetLinearVelocity(initial_vel);
		body->SetCollisionCallback(&impact_callback);
		body->SetUserEntity(this);
		physics->AddRigidBody(body);

		trail_head = new TrailHead(this);
		game_state->Spawn(new BillboardTrail(game_state, trail_head, material, 0.03f));
	}

	void Shot::DeSpawned()
	{
		if(body != NULL)
			physics->RemoveRigidBody(body);
	}

	void Shot::Update(TimingInfo time) { Entity::Update(time); }

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




	/*
	 * Shot::MyImpactCallback methods
	 */
	Shot::MyImpactCallback::MyImpactCallback(Shot* shot) : shot(shot) { }

	bool Shot::MyImpactCallback::OnCollision(const ContactPoint& cp)
	{
		if(shot->is_valid)
		{
			Shootable* hit = dynamic_cast<Shootable*>(cp.obj_b->GetUserEntity());
			if(hit != NULL && hit->GetShot(shot, cp.a.pos, shot->GetMomentum()))
			{
				if(shot->trail_head != NULL)
				{
					shot->trail_head->end_pos = cp.a.pos;
					shot->trail_head->shot = NULL;
				}

				shot->is_valid = false;

				return false;
			}
		}

		return false;
	}
}
