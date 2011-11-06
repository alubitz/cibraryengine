#pragma once

#include "StdAfx.h"

#include "Octree.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class VoxelMaterial;

	struct TerrainLeaf
	{
		unsigned char types[4], weights[4];
		unsigned char solidity;

		TerrainLeaf();
		TerrainLeaf(unsigned char type);

		void ClearMaterials();
		unsigned char GetMaterialAmount(unsigned char mat);
		void SetMaterialAmount(unsigned char mat, unsigned char amount);
		int GetTotalNonzero();

		float GetScalarValue();
		bool IsSolid();
	};

	class VoxelTerrain
	{
		private:

			vector<TerrainLeaf> data;
			int dim[3];
			int x_span;

			Mat4 scale;
			Mat4 xform;

			VoxelMaterial* material;
			VertexBuffer* model;

			TerrainLeaf& Element(int x, int y, int z);

			VertexBuffer* CreateVBO();
			void InvalidateVBO();

		public:

			VoxelTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z);
			~VoxelTerrain();

			void Solidify();

			void Explode();
			void Erode();

			void Vis(SceneRenderer* renderer);
	};
}
