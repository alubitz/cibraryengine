#include "StdAfx.h"
#include "ContactDataCollector.h"

#include "ContactPoint.h"
#include "ContactRegion.h"

namespace CibraryEngine
{
	/*
	 * ContactDataCollector private implementation struct
	 */
	struct ContactDataCollector::Imp
	{
		Imp() { }
		~Imp() { }
	};



	/*
	 * ContactDataCollector methods
	 */
	ContactDataCollector::ContactDataCollector(Imp* imp) : imp(imp), results() { }
	ContactDataCollector::~ContactDataCollector() { imp->~Imp(); }

	ContactDataCollector* ContactDataCollector::NewCollector()
	{
		char* bytes = (char*)malloc(sizeof(ContactDataCollector) + sizeof(ContactDataCollector::Imp));
		return new(bytes) ContactDataCollector(new(bytes + sizeof(ContactDataCollector)) ContactDataCollector::Imp());
	}

	void ContactDataCollector::DeleteCollector(ContactDataCollector* collector)
	{
		collector->~ContactDataCollector();
		free(collector);
	}



	ContactRegion* ContactDataCollector::AddRegion(RigidBody* obj_a, RigidBody* obj_b, const Vec3& normal, unsigned int n_points, const Vec3* points)
	{
		ContactRegion* result = new ContactRegion(obj_a, obj_b);

		for(; n_points > 0; --n_points, ++points)
		{
			ContactPoint* p = new ContactPoint(obj_a, obj_b);
			p->normal = normal;
			p->pos = *points;
			result->points.push_back(p);
		}

		results.push_back(result);

		return result;
	}

	void ContactDataCollector::ClearResults()
	{
		for(vector<ContactRegion*>::iterator iter = results.begin(), results_end = results.end(); iter != results_end; ++iter)
		{
			ContactRegion* region = *iter;

			for(vector<ContactPoint*>::iterator jter = region->points.begin(), points_end = region->points.end(); jter != points_end; ++jter)
				delete *jter;

			region->points.clear();
			delete region;
		}

		results.clear();
	}
}
