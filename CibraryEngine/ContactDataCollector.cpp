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
		vector<ContactPoint*>	cp_bin;
		vector<ContactRegion*>	cr_bin;

		Imp() : cp_bin(), cr_bin() { }

		~Imp()
		{
			for(vector<ContactPoint*>::iterator iter = cp_bin.begin(); iter != cp_bin.end(); ++iter)
				delete *iter;
			cp_bin.clear();

			for(vector<ContactRegion*>::iterator iter = cr_bin.begin(); iter != cr_bin.end(); ++iter)
				delete *iter;
			cr_bin.clear();
		}

		ContactPoint* NewContactPoint(RigidBody* obj_a, RigidBody* obj_b)
		{
			vector<ContactPoint*>::reverse_iterator any = cp_bin.rbegin();
			if(any == cp_bin.rend())
				return new ContactPoint(obj_a, obj_b);
			else
			{
				ContactPoint* result = new(*any) ContactPoint(obj_a, obj_b);
				cp_bin.pop_back();
				return result;
			}
		}

		void DeleteContactPoint(ContactPoint* cp)
		{
			cp->~ContactPoint();
			cp_bin.push_back(cp);
		}

		ContactRegion* NewContactRegion(RigidBody* obj_a, RigidBody* obj_b)
		{
			vector<ContactRegion*>::reverse_iterator any = cr_bin.rbegin();
			if(any == cr_bin.rend())
				return new ContactRegion(obj_a, obj_b);
			else
			{
				ContactRegion* result = new(*any) ContactRegion(obj_a, obj_b);
				cr_bin.pop_back();
				return result;
			}
		}

		void DeleteContactRegion(ContactRegion* cr)
		{
			cr->~ContactRegion();
			cr_bin.push_back(cr);
		}
	};



	/*
	 * ContactDataCollector methods
	 */
	ContactDataCollector::ContactDataCollector(Imp* imp) : imp(imp), results()	{ }
	ContactDataCollector::~ContactDataCollector()								{ imp->~Imp(); }

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
		ContactRegion* result = imp->NewContactRegion(obj_a, obj_b);

		for(; n_points > 0; --n_points, ++points)
		{
			ContactPoint* p = imp->NewContactPoint(obj_a, obj_b);
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
				imp->DeleteContactPoint(*jter);
			region->points.clear();

			imp->DeleteContactRegion(region);
		}

		results.clear();
	}
}
