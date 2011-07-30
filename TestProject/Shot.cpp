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
	Shot::Shot(GameState* gs, VTNModel* model, GlowyModelMaterial* material, Vec3 origin, Vec3 initial_vel, Quaternion ori, Dood* firer) :
		Entity(gs),
		bs(origin, -1),
		model(model),
		material(material),
		physics(NULL),
		origin(origin),
		pos(origin),
		vel(initial_vel),
		ori(ori),
		draw_xform(Mat4::FromPositionAndOrientation(origin, ori)),
		causer(firer),
		firer(firer),
		mass(0.2f)
	{
	}

	void Shot::InnerDispose()
	{
		VisCleanup();

		Entity::InnerDispose();
	}

	void Shot::Vis(SceneRenderer* renderer)
	{
		if (renderer->camera->CheckSphereVisibility(bs))
		{
			GlowyModelMaterialNodeData* datum = new GlowyModelMaterialNodeData(model->GetVBO(), draw_xform * Mat4::Scale(0.02f, 0.02f, 5.0f));
			renderer->objects.push_back(RenderNode(material, datum, Vec3::Dot(renderer->camera->GetPosition(), bs.center)));
		}
	}

	void Shot::VisCleanup() { }

	void Shot::Spawned()
	{
		physics = game_state->physics_world;
	}

	void Shot::Update(TimingInfo time)
	{
		Entity::Update(time);

		Vec3 end_pos = pos + vel * time.elapsed;

		MyRayResultCallback callback = MyRayResultCallback(this);
		physics->dynamics_world->rayTest(btVector3(pos.x, pos.y, pos.z), btVector3(end_pos.x, end_pos.y, end_pos.z), callback);

		// sort hit objects from front to back (bubble sort... hopefully not too many objects need sorting)
		unsigned int hits_count = callback.hits.size(); 
		if(hits_count > 0)
		{
			for(unsigned int i = 0; i < hits_count; i++)
				for(unsigned int j = 0; j < hits_count - i - 1; j++)
				{
					HitObject temp = callback.hits[j];
					if(callback.hits[j + 1].time < temp.time)
					{
						callback.hits[j] = callback.hits[j + 1];
						callback.hits[j + 1] = temp;
					}
				}
			for(unsigned int i = 0; i < hits_count; i++)
			{
				float fraction = callback.hits[i].time;
				Vec3 poi = pos + (end_pos - pos) * fraction;
				Shootable* hit = callback.hits[i].obj;
				if(hit->GetShot(this, poi, GetMomentum()))
				{
					is_valid = false;
					return;
				}
			}
		}

		pos = end_pos;

		btVector3 gravity_vector = physics->dynamics_world->getGravity();
		vel += Vec3(gravity_vector.getX(), gravity_vector.getY(), gravity_vector.getZ()) * time.elapsed;

		draw_xform = Mat4::FromPositionAndOrientation(pos, ori);
		bs = Sphere(pos, 10.0);
	}

	Damage Shot::GetDamage() { return Damage(firer, 0.09f); }			// was .03 in C# version, but it took too many shots to do 1 damage
	Vec3 Shot::GetMomentum() { return vel * mass; }




	/*
	 * Shot::MyRayResultCallback methods
	 */
	Shot::MyRayResultCallback::MyRayResultCallback(Shot* shot) : shot(shot), hits() { }
	btScalar Shot::MyRayResultCallback::addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		if (shot->is_valid)
		{
			void* void_pointer = rayResult.m_collisionObject->getUserPointer();
			if(void_pointer != NULL)
			{
				Shootable* hit = dynamic_cast<Shootable*>((Entity*)void_pointer);
				if (hit != NULL && hit != shot->firer)
				{
					float fraction = rayResult.m_hitFraction;
					if(fraction >= 0 && fraction < m_closestHitFraction)
					{
						m_closestHitFraction = fraction;
						m_collisionObject = rayResult.m_collisionObject;
						hits.push_back(HitObject(hit, fraction));
					}
				}
			}
		}
		return 1;
	}
}
