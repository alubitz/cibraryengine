#include "StdAfx.h"
#include "GridRegionManager.h"

#include "RigidBody.h"
#include "PhysicsRegion.h"

#include "AABB.h"

#include "InfinitePlaneShape.h"

namespace CibraryEngine
{
	/*
	 * GridRegionManager methods
	 */
	GridRegionManager::GridRegionManager(unordered_set<PhysicsRegion*>* all_regions, ObjectOrphanedCallback* orphan_callback) :
		PhysicsRegionManager(all_regions),
		orphan_callback(orphan_callback),
		cell_dim(8.0f),
		inv_cell_dim(1.0f / cell_dim),
		x0(-16),
		y0(-4),
		z0(-16),
		dx(32),
		dy(44),
		dz(32),
		max_x(x0 + dx - 1),
		max_y(y0 + dy - 1),
		max_z(z0 + dz - 1),
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


	void GridRegionManager::OnObjectAdded(CollisionObject* object, RegionSet& object_regions)
	{
		if(object->GetType() != COT_RigidBody || ((RigidBody*)object)->GetShapeType() != ST_InfinitePlane)
		{
			int x1, y1, z1, x2, y2, z2;
			AABBToCells(object->GetAABB(0), x1, y1, z1, x2, y2, z2);

			x1 = max(x0,	x1) - x0;
			y1 = max(y0,	y1) - y0;
			z1 = max(z0,	z1) - z0;
			x2 = min(max_x,	x2) - x0;
			y2 = min(max_y,	y2) - y0;
			z2 = min(max_z,	z2) - z0;

			for(int x = x1; x < x2; ++x)
				for(int y = y1; y < y2; ++y)
					for(int z = z1; z < z2; ++z)
						region_array[x][y][z]->TakeOwnership(object);
		}
		else
		{
			InfinitePlaneShape* plane_shape = (InfinitePlaneShape*)((RigidBody*)object)->GetCollisionShape();

			for(unsigned int x = 0; x < region_array.size(); ++x)
				for(unsigned int y = 0; y < region_array[x].size(); ++y)
					for(unsigned int z = 0; z < region_array[x][y].size(); ++z)
						if(IsPlaneRelevantToRegion(plane_shape->plane, x + x0, y + y0, z + z0))
							region_array[x][y][z]->TakeOwnership(object);

			planes.insert((RigidBody*)object);
		}
	}

	void GridRegionManager::OnObjectUpdate(CollisionObject* object, RegionSet& object_regions, float timestep)
	{
		int x1, y1, z1, x2, y2, z2;
		AABBToCells(object->GetAABB(0), x1, y1, z1, x2, y2, z2);

		x1 = max(x0,	x1) - x0;
		y1 = max(y0,	y1) - y0;
		z1 = max(z0,	z1) - z0;
		x2 = min(max_x,	x2) - x0;
		y2 = min(max_y,	y2) - y0;
		z2 = min(max_z,	z2) - z0;

		set<PhysicsRegion*> ditch;
		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = object_regions.buckets[i];
			ditch.insert(bucket.begin(), bucket.end());
		}

		for(int x = x1; x < x2; ++x)
			for(int y = y1; y < y2; ++y)
				for(int z = z1; z < z2; ++z)
				{
					PhysicsRegion* region = region_array[x][y][z];

					set<PhysicsRegion*>::iterator found = ditch.find(region);
					if(found != ditch.end())
						ditch.erase(found);
					else
					{
						object_regions.Insert(region);
						region->AddCollisionObject(object);
					}
				}

		for(set<PhysicsRegion*>::iterator iter = ditch.begin(); iter != ditch.end(); ++iter)
		{
			object_regions.Erase(*iter);
			(*iter)->RemoveCollisionObject(object);
		}

		if(!object_regions.count && orphan_callback)
			orphan_callback->OnObjectOrphaned(object);
	}

	void GridRegionManager::OnObjectRemoved(CollisionObject* object, RegionSet& object_regions)
	{
		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = object_regions.buckets[i];
			for(vector<PhysicsRegion*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				(*iter)->RemoveCollisionObject(object);
		}

		if(object->GetType() == COT_RigidBody && ((RigidBody*)object)->GetShapeType() == ST_InfinitePlane)
			planes.erase((RigidBody*)object);
	}

	PhysicsRegion* GridRegionManager::GetRegion(const Vec3& point)
	{
		int x1, y1, z1, x2, y2, z2;
		AABBToCells(AABB(point), x1, y1, z1, x2, y2, z2);

		return region_array[x1 - x0][y1 - y0][z1 - z0];
	}

	void GridRegionManager::GetRegionsOnRay(const Vec3& from, const Vec3& to, RegionSet& results)
	{
		AABB aabb(from);
		aabb.Expand(to);

		int x1, y1, z1, x2, y2, z2;
		AABBToCells(aabb, x1, y1, z1, x2, y2, z2);

		Vec3 cell_diagonal(cell_dim, cell_dim, cell_dim);

		x1 = max(x0,	x1);
		y1 = max(y0,	y1);
		z1 = max(z0,	z1);
		x2 = min(max_x,	x2);
		y2 = min(max_y,	y2);
		z2 = min(max_z,	z2);

		for(int x = x1; x < x2; ++x)
			for(int y = y1; y < y2; ++y)
				for(int z = z1; z < z2; ++z)
				{
					Vec3 xyz(x * cell_dim, y * cell_dim, z * cell_dim);

					if(AABB(xyz, xyz + cell_diagonal).IntersectLineSegment(from, to))
						results.Insert(region_array[x - x0][y - y0][z - z0]);
				}
	}



	// now for the protected methods...
	void GridRegionManager::AABBToCells(const AABB& aabb, int& x1, int& y1, int& z1, int& x2, int& y2, int& z2)
	{
		x1 = (int)floor(aabb.min.x * inv_cell_dim);
		y1 = (int)floor(aabb.min.y * inv_cell_dim);
		z1 = (int)floor(aabb.min.z * inv_cell_dim);
		x2 = (int)ceil (aabb.max.x * inv_cell_dim);
		y2 = (int)ceil (aabb.max.y * inv_cell_dim);
		z2 = (int)ceil (aabb.max.z * inv_cell_dim);
	}

	bool GridRegionManager::IsPlaneRelevantToRegion(const Plane& plane, int x, int y, int z)
	{
		float xa[] = { x * cell_dim, 0 };
		float ya[] = { y * cell_dim, 0 };
		float za[] = { z * cell_dim, 0 };
		xa[1] = xa[0] + cell_dim;
		ya[1] = ya[0] + cell_dim;
		za[1] = za[0] + cell_dim;

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
