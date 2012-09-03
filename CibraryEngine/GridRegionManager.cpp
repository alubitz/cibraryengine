#include "StdAfx.h"
#include "GridRegionManager.h"

#include "RigidBody.h"
#include "PhysicsRegion.h"

#include "AABB.h"

#include "InfinitePlaneShape.h"

namespace CibraryEngine
{
	using boost::unordered_set;

	/*
	 * GridRegionManager methods
	 */
	GridRegionManager::GridRegionManager(unordered_set<PhysicsRegion*>* all_regions, ObjectOrphanedCallback* orphan_callback) :
		PhysicsRegionManager(all_regions),
		orphan_callback(orphan_callback),
		cell_dim(8.0f),
		x0(-16),
		y0(-4),
		z0(-16),
		dx(32),
		dy(44),
		dz(32),
		region_array(),
		planes()
	{
		for(int x = 0; x < dx; ++x)
		{
			region_array.push_back(vector<vector<PhysicsRegion*> >());
			for(int y = 0; y < dy; ++y)
			{
				region_array[x].push_back(vector<PhysicsRegion*>());
				for(int z = 0; z < dz; ++z)
				{
					region_array[x][y].push_back(CreateRegion(x + x0, y + y0, z + z0));
				}
			}
		}
	}
	GridRegionManager::~GridRegionManager() { }


	void GridRegionManager::OnObjectAdded(RigidBody* object, set<PhysicsRegion*>& object_regions)
	{
		if(object->GetShapeType() != ST_InfinitePlane)
		{
			int x1, y1, z1, x2, y2, z2;
			AABBToCells(object->GetAABB(0), x1, y1, z1, x2, y2, z2);

			x1 = max(x0, x1);
			y1 = max(y0, y1);
			z1 = max(z0, z1);
			x2 = min(x0 + dx - 1, x2);
			y2 = min(y0 + dy - 1, y2);
			z2 = min(z0 + dz - 1, z2);

			for(int x = x1; x < x2; ++x)
				for(int y = y1; y < y2; ++y)
					for(int z = z1; z < z2; ++z)
						region_array[x - x0][y - y0][z - z0]->TakeOwnership(object);
		}
		else
		{
			InfinitePlaneShape* plane_shape = (InfinitePlaneShape*)object->GetCollisionShape();

			for(unsigned int x = 0; x < region_array.size(); ++x)
				for(unsigned int y = 0; y < region_array[x].size(); ++y)
					for(unsigned int z = 0; z < region_array[x][y].size(); ++z)
						if(IsPlaneRelevantToRegion(plane_shape->plane, x + x0, y + y0, z + z0))
							region_array[x][y][z]->TakeOwnership(object);

			planes.insert(object);
		}
	}

	void GridRegionManager::OnObjectUpdate(RigidBody* object, set<PhysicsRegion*>& object_regions, float timestep)
	{
		int x1, y1, z1, x2, y2, z2;
		AABBToCells(object->GetAABB(0), x1, y1, z1, x2, y2, z2);

		x1 = max(x0, x1);
		y1 = max(y0, y1);
		z1 = max(z0, z1);
		x2 = min(x0 + dx - 1, x2);
		y2 = min(y0 + dy - 1, y2);
		z2 = min(z0 + dz - 1, z2);

		set<PhysicsRegion*> ditch(object_regions.begin(), object_regions.end());
		set<PhysicsRegion*> add;

		for(int x = x1; x < x2; ++x)
			for(int y = y1; y < y2; ++y)
				for(int z = z1; z < z2; ++z)
				{
					PhysicsRegion* region = region_array[x - x0][y - y0][z - z0];

					set<PhysicsRegion*>::iterator found = ditch.find(region);
					if(found != ditch.end())
						ditch.erase(found);
					else
						add.insert(region);
				}

		for(set<PhysicsRegion*>::iterator iter = add.begin(); iter != add.end(); ++iter)
		{
			object_regions.insert(*iter);
			(*iter)->AddRigidBody(object);
		}

		for(set<PhysicsRegion*>::iterator iter = ditch.begin(); iter != ditch.end(); ++iter)
		{
			object_regions.erase(*iter);
			(*iter)->RemoveRigidBody(object);
		}

		if(object_regions.empty() && orphan_callback)
			orphan_callback->OnObjectOrphaned(object);
	}

	void GridRegionManager::OnObjectRemoved(RigidBody* object, set<PhysicsRegion*>& object_regions)
	{
		for(set<PhysicsRegion*>::iterator iter = object_regions.begin(); iter != object_regions.end(); ++iter)
		{
			PhysicsRegion* region = *iter;
			region->RemoveRigidBody(object);
		}

		if(object->GetShapeType() == ST_InfinitePlane)
			planes.erase(object);
	}

	PhysicsRegion* GridRegionManager::GetRegion(const Vec3& point)
	{
		int x1, y1, z1, x2, y2, z2;
		AABBToCells(AABB(point), x1, y1, z1, x2, y2, z2);

		return region_array[x1 - x0][y1 - y0][z1 - z0];
	}

	void GridRegionManager::GetRegionsOnRay(const Vec3& from, const Vec3& to, set<PhysicsRegion*>& results)
	{
		AABB aabb(from);
		aabb.Expand(to);

		int x1, y1, z1, x2, y2, z2;
		AABBToCells(aabb, x1, y1, z1, x2, y2, z2);

		Vec3 cell_diagonal(cell_dim, cell_dim, cell_dim);

		x1 = max(x0, x1);
		y1 = max(y0, y1);
		z1 = max(z0, z1);
		x2 = min(x0 + dx - 1, x2);
		y2 = min(y0 + dy - 1, y2);
		z2 = min(z0 + dz - 1, z2);

		for(int x = x1; x < x2; ++x)
			for(int y = y1; y < y2; ++y)
				for(int z = z1; z < z2; ++z)
				{
					Vec3 xyz(x * cell_dim, y * cell_dim, z * cell_dim);

					if(AABB(xyz, xyz + cell_diagonal).IntersectLineSegment(from, to))
						results.insert(region_array[x - x0][y - y0][z - z0]);
				}
	}



	// now for the protected methods...
	void GridRegionManager::AABBToCells(const AABB& aabb, int& x1, int& y1, int& z1, int& x2, int& y2, int& z2)
	{
		x1 = (int)floor(aabb.min.x / cell_dim);
		y1 = (int)floor(aabb.min.y / cell_dim);
		z1 = (int)floor(aabb.min.z / cell_dim);
		x2 = (int)ceil(aabb.max.x / cell_dim);
		y2 = (int)ceil(aabb.max.y / cell_dim);
		z2 = (int)ceil(aabb.max.z / cell_dim);
	}

	bool GridRegionManager::IsPlaneRelevantToRegion(const Plane& plane, int x, int y, int z)
	{
		float xa[] = { x * cell_dim, 0 };
		float ya[] = { y * cell_dim, 0 };
		float za[] = { z * cell_dim, 0 };
		float* vars[] = { xa, ya, za };

		for(int i = 0; i < 3; ++i)
			vars[i][1] = vars[i][0] + cell_dim;

		Vec3 xyz;

		bool any_plus = false;
		bool any_minus = false;

		for(int i = 0; i < 2; ++i)
		{
			xyz.x = xa[i];
			for(int j = 0; j < 2; ++j)
			{
				xyz.y = ya[j];
				for(int k = 0; k < 2; ++k)
				{
					xyz.z = za[k];

					float d = plane.PointDistance(xyz);
					if(d == 0)
						return true;
					if(d > 0)
						if(any_minus)
							return true;
						else
							any_plus = true;
					if(d < 0)
						if(any_plus)
							return true;
						else
							any_minus = true;
				}
			}
		}

		return false;
	}

	PhysicsRegion* GridRegionManager::CreateRegion(int x, int y, int z)
	{
		PhysicsRegion* result = new PhysicsRegion(orphan_callback);

		all_regions->insert(result);

		for(unordered_set<RigidBody*>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
			if(IsPlaneRelevantToRegion(((InfinitePlaneShape*)(*iter)->GetCollisionShape())->plane, x, y, z))
				result->TakeOwnership(*iter);

		return result;
	}
}
