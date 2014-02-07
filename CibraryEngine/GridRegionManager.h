#pragma once

#include "StdAfx.h"
#include "Physics.h"

namespace CibraryEngine
{
	struct Plane;
	class ObjectOrphanedCallback;

	class GridRegionManager : public PhysicsRegionManager
	{
		protected:

			void AABBToCells(const AABB& aabb, int& x1, int& y1, int& z1, int& x2, int& y2, int& z2);

			bool IsPlaneRelevantToRegion(const Plane& plane, int x, int y, int z);

			PhysicsRegion* CreateRegion(int x, int y, int z);

		public:

			ObjectOrphanedCallback* orphan_callback;

			// size of each cell
			float cell_dim;
			float inv_cell_dim;

			// offset of the minimum cell
			int x0, y0, z0;
			int dx, dy, dz;
			int max_x, max_y, max_z;

			vector<vector<vector<PhysicsRegion*> > > region_array;
			unordered_set<RigidBody*> planes;

			GridRegionManager(unordered_set<PhysicsRegion*>* all_regions, ObjectOrphanedCallback* orphan_callback);
			~GridRegionManager();


			void OnObjectAdded(CollisionObject* object, RegionSet& object_regions);
			void OnObjectUpdate(CollisionObject* object, RegionSet& object_regions, float timestep);
			void OnObjectRemoved(CollisionObject* object, RegionSet& object_regions);

			PhysicsRegion* GetRegion(const Vec3& point);
			void GetRegionsOnRay(const Vec3& from, const Vec3& to, RegionSet& results);
	};
}
