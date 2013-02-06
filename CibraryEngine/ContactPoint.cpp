#include "StdAfx.h"
#include "ContactPoint.h"

#include "RigidBody.h"

#include "DebugLog.h"

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

			restitution_coeff = 1.0f + obj_a->restitution * obj_b->restitution;
			fric_coeff = obj_a->friction * obj_b->friction;

			r1 = use_pos - obj_a->cached_com;
			r2 = use_pos - obj_b->cached_com;

			// computing rlv-to-impulse matrix
			Mat3 xr1(
					0,	  r1.z,	  -r1.y,
				-r1.z,	     0,	   r1.x,
				 r1.y,	 -r1.x,	      0		);
			Mat3 xr2(
					0,	  r2.z,	  -r2.y,
				-r2.z,	     0,	   r2.x,
				 r2.y,	 -r2.x,	      0		);

			float invmasses = -(obj_a->inv_mass + obj_b->inv_mass);
			Mat3 impulse_to_rlv = Mat3(invmasses, 0, 0, 0, invmasses, 0, 0, 0, invmasses)
				+ xr1 * obj_a->inv_moi * xr1
				+ xr2 * obj_b->inv_moi * xr2;

			rlv_to_impulse = Mat3::Invert(impulse_to_rlv);


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
		static const float adhesion_threshold = 0.04f;
		static const float impulse_sq_threshold = 0.1f;

		assert(cache_valid);

		Vec3 dv = GetRelativeLocalVelocity();
		float nvdot = Vec3::Dot(normal, dv);
		if(nvdot < adhesion_threshold)						// TODO: deal with icky stuff when there's a negative normal forces
		{
			Vec3 normal_nvdot = normal * nvdot;

			// normal force aka restitution
			float use_restitution_coeff = nvdot < bounce_threshold ? restitution_coeff : 1.0f;
			Vec3 restitution_impulse = rlv_to_impulse * normal_nvdot * -use_restitution_coeff;


			// friction
			Vec3 full_friction_impulse = rlv_to_impulse * (normal_nvdot - dv);
			float fric_fraction = min(1.0f, fric_coeff * sqrtf(restitution_impulse.ComputeMagnitudeSquared() / full_friction_impulse.ComputeMagnitudeSquared()));


			// apply computed impulses
			Vec3 apply_impulse = restitution_impulse + full_friction_impulse * fric_fraction;
			ApplyImpulse(apply_impulse);

			return apply_impulse.ComputeMagnitudeSquared() > impulse_sq_threshold;
		}

		return false;
	}

	bool ContactPoint::DoConstraintAction()
	{
		BuildCache();

		if(DoCollisionResponse())
		{
			if(obj_a->collision_callback)
				obj_a->collision_callback->OnCollision(*this);
			if(obj_b->collision_callback)
				obj_b->collision_callback->OnCollision(*this);

			return true;
		}
		else
			return false;
	}

	void ContactPoint::DoUpdateAction(float timestep)
	{
		this->timestep = timestep;

		bounce_threshold = -9.8f * 5.0f * timestep;			// minus sign is for normal vector direction, not downwardness of gravity!
	}

	
	static vector<ContactPointAllocator*> cp_allocators = vector<ContactPointAllocator*>();
	void ContactPoint::Delete(ContactPoint* cp)
	{
		for(vector<ContactPointAllocator*>::iterator iter = cp_allocators.begin(); iter != cp_allocators.end(); ++iter)
			if((*iter)->Delete(cp))
				return;

		DEBUG();			// unable to delete contact point; the allocator that created it could not be found
	}




	/*
	 * ContactPointAllocator::Chunk struct private implementation
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




	/*
	 * ContactPointAllocator methods (and global instance tracker)
	 */
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
}
