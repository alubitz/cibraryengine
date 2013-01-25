#pragma once

#include "StdAfx.h"

#include "Vector.h"
#include "Physics.h"

namespace CibraryEngine
{
	/** A point of contact between two physics objects */
	struct ContactPoint : public PhysicsConstraint
	{
		struct Part
		{
			Vec3 pos, norm;					// both are world coords

			Part() : pos(), norm() { }
		} a, b;

		bool cache_valid;
		// cached values (must be computed if cache_valid is false)
		Vec3 use_pos;
		Vec3 normal;
		float bounce_coeff, fric_coeff;
		Vec3 moi_n;

		// cached values relating to GetUseMass
		float use_mass;
		Vec3 r1, r2, nr1, nr2;

		// more cached values, but these are computed in DoUpdateAction instead of BuildCache
		float timestep;
		float bounce_threshold;


		ContactPoint() : cache_valid(false) { }
		~ContactPoint() { }


		void BuildCache();

		Vec3 GetRelativeLocalVelocity() const;
		void ApplyImpulse(const Vec3& impulse) const;

		void DoConstraintAction();
		void DoUpdateAction(float timestep);

		bool DoCollisionResponse() const;



		static void Delete(ContactPoint* cp);
	};

	struct ContactPointAllocator
	{
		friend struct ContactPoint;

		private:

			struct Chunk;
			vector<Chunk*> chunks;

			bool Delete(ContactPoint* cp);

			ContactPointAllocator() : chunks() { }
			~ContactPointAllocator();

		public:

			static ContactPointAllocator* NewAllocator();
			static void DeleteAllocator(ContactPointAllocator* alloc);

			ContactPoint* New();
	};
}
