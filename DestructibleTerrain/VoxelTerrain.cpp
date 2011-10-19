#include "StdAfx.h"

#include "VoxelTerrain.h"

namespace DestructibleTerrain
{
	/*
	 * VoxelTerrain methods
	 */
	VoxelTerrain::VoxelTerrain() :
		octree(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f))
	{
		octree.Split();

		float total_volume = 0.0f;
		
		struct : Octree<unsigned char>::OctreeAction
		{
			float* total_volume;

			void operator()(Octree<unsigned char>* node)
			{
				const float sphere_radius = 1.0f;
				Vec3 center = (node->min_xyz + node->max_xyz) * 0.5f;

				Vec3 diameter = node->max_xyz - node->min_xyz;
				float radius = diameter.ComputeMagnitude() * 0.5f;
				float dist = center.ComputeMagnitude();

				if(dist - radius > sphere_radius)
				{
					Debug("Definitely outside sphere\n");
					node->contents = 0;
				}
				else if(dist + radius < sphere_radius)
				{
					float volume = diameter.x * diameter.y * diameter.z;

					stringstream ss;
					ss << "Definitely inside sphere; increasing volume by " << volume << endl;
					Debug(ss.str());

					node->contents = 1;
					*total_volume += volume;
				}
				else
				{
					if(node->GetDepth() < 4)
					{
						Debug("Not clear whether inside or outside; splitting\n");
						node->Split();
						node->ForEach(*this);
					}
					else
					{
						Debug("Too deep without conclusive results; defaulting to outside\n");
						node->contents = 0;
					}
				}
			}
		} action;

		action.total_volume = &total_volume;

		octree.ForEach(action);


		stringstream ss;
		ss << "Total volume is " << total_volume << endl;
		Debug(ss.str());
	}

	void VoxelTerrain::Vis(SceneRenderer *renderer)
	{
		// TODO: push render node here
	}
}
