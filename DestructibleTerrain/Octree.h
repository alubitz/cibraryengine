#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	/**
	 * Octrees
	 * 3-dimensional space partitioning tree data structure
	 */
	template<class T> struct Octree
	{
		Octree* parent;
		Octree* children[8];
		T contents;					// if the node has children, this variable is ignored

		Vec3 min_xyz, max_xyz;

		Octree(Vec3 min, Vec3 max) : 
			parent(NULL), 
			contents(), 
			min_xyz(min), 
			max_xyz(max) 
		{ 
			for(int i = 0; i < 8; ++i)
				children[i] = NULL;
		}
		
		Octree(Octree& parent, Vec3 min, Vec3 max) : 
			parent(&parent), 
			contents(parent.contents), 
			min_xyz(min), 
			max_xyz(max) 
		{
			for(int i = 0; i < 8; ++i)
				children[i] = NULL;
		}
		
		Octree(Octree& parent, T contents, Vec3 min, Vec3 max) : 
			parent(&parent), 
			contents(contents), 
			min_xyz(min), 
			max_xyz(max) 
		{
			for(int i = 0; i < 8; ++i)
				children[i] = NULL;
		}

		~Octree()
		{
			for(int i = 0; i < 8; ++i)
			if(children[i] != NULL)
			{
				delete children[i];
				children[i] = NULL;
			}
		}

		/** Determine whether this node is a leaf node (i.e. it has no children) */
		bool IsLeaf()
		{
			for(int i = 0; i < 8; ++i)
				if(children[i] != NULL)
					return false;
			return true;
		}

		/**
		 * If the node is a leaf node, gives it 8 children (making it no longer a leaf node) and returns true
		 * Otherwise, returns false and does anything
		 *
		 * Copies contents variable to newly created children
		 */
		bool Split(int levels = 1)
		{
			if(!IsLeaf())
				return false;
			
			Vec3 cen_xyz = (min_xyz + max_xyz) * 0.5f;

			children[0] = new Octree(*this, Vec3(min_xyz.x, min_xyz.y, min_xyz.z), Vec3(cen_xyz.x, cen_xyz.y, cen_xyz.z));
			children[1] = new Octree(*this, Vec3(min_xyz.x, min_xyz.y, cen_xyz.z), Vec3(cen_xyz.x, cen_xyz.y, max_xyz.z));
			children[2] = new Octree(*this, Vec3(min_xyz.x, cen_xyz.y, min_xyz.z), Vec3(cen_xyz.x, max_xyz.y, cen_xyz.z));
			children[3] = new Octree(*this, Vec3(min_xyz.x, cen_xyz.y, cen_xyz.z), Vec3(cen_xyz.x, max_xyz.y, max_xyz.z));
			children[4] = new Octree(*this, Vec3(cen_xyz.x, min_xyz.y, min_xyz.z), Vec3(max_xyz.x, cen_xyz.y, cen_xyz.z));
			children[5] = new Octree(*this, Vec3(cen_xyz.x, min_xyz.y, cen_xyz.z), Vec3(max_xyz.x, cen_xyz.y, max_xyz.z));
			children[6] = new Octree(*this, Vec3(cen_xyz.x, cen_xyz.y, min_xyz.z), Vec3(max_xyz.x, max_xyz.y, cen_xyz.z));
			children[7] = new Octree(*this, Vec3(cen_xyz.x, cen_xyz.y, cen_xyz.z), Vec3(max_xyz.x, max_xyz.y, max_xyz.z));

			if(levels > 1)
			{
				for(int i = 0; i < 8; ++i)
					children[i]->Split(levels - 1);
			}

			return true;
		}

		/** Gets the depth of this node within the octree; the root node has depth 0 */
		int GetDepth()
		{
			int total = 0;

			Octree* temp = this;
			while(temp->parent != NULL)
			{
				total++;
				temp = temp->parent;
			}

			return total;
		}

		/** Returns the root node of the octree containing this node */
		Octree* GetRootNode()
		{
			Octree* temp = this;
			
			while(temp->parent != NULL)
				temp = temp->parent;

			return temp;
		}

		/** Gets the max depth of any child of this node, relative to this node */
		int GetMaxChildDepth()
		{
			int max = 0;

			for(int i = 0; i < 8; ++i)
				if(children[i] != NULL)
					max = max(max, children[i]->GetMaxChildDepth());

			return max;
		}
		
		/** True if ANY branches below this node reach the specified relative depth (or beyond) */
		bool AnyChildrenAtDepth(int depth)
		{
			if(depth <= 0)
				return true;

			for(int i = 0; i < 8; ++i)
				if(children[i] != NULL)
					if(children[i]->AnyChildrenAtDepth(depth - 1))
						return true;

			return false;
		}

		/** True if ALL branches below this node reach the specified relative depth (or beyond) */
		bool AllChildrenAtDepth(int depth)
		{
			if(depth <= 0)
				return true;

			for(int i = 0; i < 8; ++i)
				if(children[i] == NULL)
					return false;
				else if(!children[i]->AllChildrenAtDepth(depth - 1))
					return false;

			return true;
		}

		template <class U> void ForEach(U& action)
		{
			for(int i = 0; i < 8; ++i)
				if(children[i] != NULL)
					action(children[i]);
		}
	};
}
