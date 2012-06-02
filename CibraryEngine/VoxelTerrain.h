#pragma once

#include "StdAfx.h"

#include "Disposable.h"
#include "Matrix.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Content.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class VoxelMaterial;
	
	struct TerrainNode;
	class TerrainChunk;

	struct TerrainAction
	{
		virtual void AffectNode(TerrainChunk* chunk, TerrainNode& node, int x, int y, int z, unsigned char amount) = 0;
	};

	class VoxelTerrain : public Disposable
	{
		friend struct VoxelTerrainLoader;

		private:

			vector<TerrainChunk*> chunks;
			int dim[3];
			int x_span;

			Mat4 scale;
			Mat4 xform;

			VoxelMaterial* material;

		protected:

			void InnerDispose();

		public:

			VoxelTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z);
			~VoxelTerrain();

			TerrainChunk* Chunk(int x, int y, int z);
			bool PosToNode(Vec3 pos, TerrainChunk*& chunk, int& x, int& y, int& z);

			int GetXDim() { return dim[0]; }
			int GetYDim() { return dim[1]; }
			int GetZDim() { return dim[2]; }
			Mat4 GetTransform() { return scale * xform; }

			void Vis(SceneRenderer* renderer);

			void Solidify();

			void ModifySphere(Vec3 center, float inner_radius, float outer_radius, TerrainAction& action);
	};

	struct VoxelTerrainLoader : public ContentTypeHandler<VoxelTerrain>
	{
		private:

			VoxelMaterial* default_material;				// pointer to material created elsewhere	

		public:

			VoxelTerrainLoader(ContentMan* man, VoxelMaterial* default_material);

			VoxelTerrain* Load(ContentMetadata& what);
			void Unload(VoxelTerrain* content, ContentMetadata& meta);

			static unsigned int SaveVVV(VoxelTerrain* terrain, const string& filename);
			static unsigned int LoadVVV(VoxelTerrain*& terrain, VoxelMaterial* material, const string& filename);

			static VoxelTerrain* GenerateTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z);
	};
}
