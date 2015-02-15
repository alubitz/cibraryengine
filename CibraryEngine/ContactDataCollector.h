#pragma once

#include "StdAfx.h"

#include "Vector.h"

namespace CibraryEngine
{
	using namespace std;

	struct ContactPoint;
	class RigidBody;

	class ContactDataCollector
	{
		friend class ContactRegion;

		private:

			struct Imp;
			Imp* imp;

			bool Delete(ContactRegion* region);

			ContactDataCollector(Imp* imp);
			~ContactDataCollector();

		public:

			vector<ContactRegion*> results;

			static ContactDataCollector* NewCollector();
			static void DeleteCollector(ContactDataCollector* collector);

			ContactRegion* AddRegion(RigidBody* obj_a, RigidBody* obj_b, const Vec3& normal, unsigned int n_points, const Vec3* points);

			void RunPostResolutionCallbacks();
			void ClearResults();
	};
}
