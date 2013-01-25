#include "StdAfx.h"
#include "ContactPoint.h"

#include "RigidBody.h"

#define ENABLE_ANTI_PENETRATION_DISPLACEMENT 0
#define ENABLE_ANGULAR_FRICTION 0

namespace CibraryEngine
{
	/*
	 * ContactPoint methods
	 */
	void ContactPoint::BuildCache()
	{
		if(!cache_valid)
		{
			use_pos = (a.pos + b.pos) * 0.5f;
			normal = Vec3::Normalize(a.norm - b.norm);

			bounce_coeff = 1.0f + obj_a->bounciness * obj_b->bounciness;
			fric_coeff = obj_a->friction * obj_b->friction;			// kfric would be * 0.9f but in practice the sim treats everything as kinetic anyway
#if ENABLE_ANGULAR_FRICTION
			moi_n = Mat3::Invert(obj_a->inv_moi + obj_b->inv_moi) * normal;
#endif			

			use_mass = PhysicsWorld::GetUseMass(obj_a, obj_b, use_pos, normal);
			r1 = use_pos - obj_a->cached_com;
			r2 = use_pos - obj_b->cached_com;
			nr1 = Vec3::Cross(normal, r1);
			nr2 = Vec3::Cross(normal, r2);

			cache_valid = true;
		}
	}

	Vec3 ContactPoint::GetRelativeLocalVelocity() const { return obj_b->vel - obj_a->vel + Vec3::Cross(r2, obj_b->rot) - Vec3::Cross(r1, obj_a->rot); }

	void ContactPoint::ApplyImpulse(const Vec3& impulse) const
	{
		if(obj_a->active)
		{
			obj_a->vel += impulse * obj_a->inv_mass;
			if(obj_a->can_rotate)
				obj_a->rot += obj_a->inv_moi * Vec3::Cross(impulse, r1);
		}

		if(obj_b->active && obj_b->can_move)
		{
			obj_b->vel -= impulse * obj_b->inv_mass;
			if(obj_b->can_rotate)
				obj_b->rot -= obj_b->inv_moi * Vec3::Cross(impulse, r2);
		}
	}

	bool ContactPoint::DoCollisionResponse() const
	{
		assert(cache_valid);

		RigidBody* ibody = obj_a;
		RigidBody* jbody = obj_b;

		bool j_can_move = jbody->can_move;

		Vec3 dv = GetRelativeLocalVelocity();
		float nvdot = Vec3::Dot(normal, dv);
		if(nvdot < 0.0f)
		{
			float use_bounce_coeff = nvdot < bounce_threshold ? bounce_coeff : 1.0f;
			float impulse_mag = use_bounce_coeff * nvdot * use_mass;

			if(impulse_mag < 0)
			{
				Vec3 impulse = normal * impulse_mag;

				// normal force
				if(impulse.ComputeMagnitudeSquared() != 0)
				{
					ApplyImpulse(impulse);

					// applying this impulse means we need to recompute dv and nvdot!
					dv = GetRelativeLocalVelocity();
					nvdot = Vec3::Dot(normal, dv);
				}

				Vec3 t_dv = dv - normal * nvdot;
				float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

				// linear friction
				if(t_dv_magsq > 0)								// object is moving; apply kinetic friction
				{
					float t_dv_mag = sqrtf(t_dv_magsq), inv_tdmag = 1.0f / t_dv_mag;
					Vec3 u_tdv = t_dv * inv_tdmag;

					float use_mass2 = PhysicsWorld::GetUseMass(ibody, jbody, use_pos, u_tdv);
					ApplyImpulse(t_dv * min(use_mass2, fabs(impulse_mag * fric_coeff * inv_tdmag)));
				}

				// TODO: make angular friction less wrong
#if ENABLE_ANGULAR_FRICTION
				// angular friction (wip; currently completely undoes angular velocity around the normal vector)
				float angular_dv = Vec3::Dot(normal, obj_b->rot - obj_a->rot);
				if(fabs(angular_dv) > 0)
				{
					Vec3 angular_impulse = moi_n * angular_dv;

					ibody->ApplyAngularImpulse(angular_impulse);
					if(j_can_move && jbody->can_rotate)
						jbody->ApplyAngularImpulse(-angular_impulse);
				}
#endif

				return true;
			}
		}

		return false;
	}

	void ContactPoint::DoConstraintAction()
	{
		BuildCache();

		if(DoCollisionResponse())
		{
			if(obj_a->collision_callback)
				obj_a->collision_callback->OnCollision(*this);
			if(obj_b->collision_callback)
				obj_b->collision_callback->OnCollision(*this);
		}
	}

	void ContactPoint::DoUpdateAction(float timestep)
	{
		this->timestep = timestep;

		bounce_threshold = -9.8f * 5.0f * timestep;			// minus sign is for normal vector direction, not downwardness of gravity!

#if ENABLE_ANTI_PENETRATION_DISPLACEMENT

		// magical anti-penetration displacement! directly modifies position, instead of working with velocity
		Vec3 dx = b.pos - a.pos;

		if(dx.x || dx.y || dx.z)
		{
			// make sure we don't displace the same object multiple times in the same direction
			Vec3 cx = obj_a->moved_from_pos - obj_b->moved_from_pos;
			if(cx.x || cx.y || cx.z)
			{
				float cmag = cx.ComputeMagnitude(), dmag = dx.ComputeMagnitude();
				float dot = Vec3::Dot(cx, dx), comp = dot / cmag;
				if(comp <= cmag)
					return;
				else
					dx *= (comp - cmag) / dmag;
			}

			if(!obj_b->can_move)
			{
				obj_a->pos -= dx;
				obj_a->moved_from_pos -= dx;

				obj_a->xform_valid = false;
			}
			else
			{
				float total = obj_a->inv_mass + obj_b->inv_mass, inv_total = 1.0f / total;

				Vec3 ax = -dx * (inv_total * obj_a->inv_mass);
				Vec3 bx = dx * (inv_total * obj_b->inv_mass);

				obj_a->pos += ax;
				obj_b->pos += bx;
				obj_a->moved_from_pos += ax;
				obj_b->moved_from_pos += bx;

				obj_a->xform_valid = false;
				obj_b->xform_valid = false;
			}
		}
#endif
	}




	/*
	 * ContactPoint allocator class; create one of these for each thread you want to be simultaneously creating create contact points in
	 */
	struct ContactPointAllocator::Chunk
	{
		static const unsigned int SIZE = 64;

		ContactPoint	points		[SIZE];
		ContactPoint*	available	[SIZE];
		unsigned int available_count;

		Chunk() : available_count(SIZE)
		{
			for(unsigned int i = 0; i < SIZE; ++i)
				available[i] = points + i;
		}

		ContactPoint* New()
		{
			if(available_count != 0)
			{
				ContactPoint* cp = available[--available_count];
				return new (cp) ContactPoint();
			}
			else
				return NULL;
		}

		bool Delete(ContactPoint* cp)
		{
			if(cp >= points && cp < points + SIZE)
			{
				assert(available_count < SIZE);

				available[available_count++] = cp;
				return true;
			}
			else
				return false;
		}
	};

	static vector<ContactPointAllocator*> cp_allocators = vector<ContactPointAllocator*>();



	ContactPointAllocator::~ContactPointAllocator()
	{
		for(vector<Chunk*>::iterator iter = chunks.begin(); iter != chunks.end(); ++iter)
			delete *iter;
		chunks.clear();
	}

	ContactPointAllocator* ContactPointAllocator::NewAllocator()
	{
		ContactPointAllocator* alloc = new ContactPointAllocator();
		cp_allocators.push_back(alloc);

		return alloc;
	}

	void ContactPointAllocator::DeleteAllocator(ContactPointAllocator* alloc)
	{
		for(unsigned int i = 0, size = cp_allocators.size(); i < size; ++i)
			if(cp_allocators[i] == alloc)
			{
				cp_allocators[i] = cp_allocators[size - 1];
				cp_allocators.pop_back();

				break;
			}

		delete alloc;
	}

	ContactPoint* ContactPointAllocator::New()
	{
		for(unsigned int i = 0, size = chunks.size(); i < size; ++i)
			if(ContactPoint* cp = chunks[i]->New())
				return cp;

		Chunk* chunk = new Chunk();
		chunks.push_back(chunk);

		return chunk->New();
	}

	bool ContactPointAllocator::Delete(ContactPoint* cp)
	{
		cp->~ContactPoint();

		for(unsigned int i = 0, size = chunks.size(); i < size; ++i)
			if(chunks[i]->Delete(cp))
				return true;

		return false;
	}

	void ContactPoint::Delete(ContactPoint* cp)
	{
		for(vector<ContactPointAllocator*>::iterator iter = cp_allocators.begin(); iter != cp_allocators.end(); ++iter)
			if((*iter)->Delete(cp))
				break;
	}
}
