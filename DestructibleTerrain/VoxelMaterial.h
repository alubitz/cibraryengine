#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct VoxelMaterialNodeData
	{
		VertexBuffer* model;
		Vec3 chunk_pos;
		Mat4 xform;

		unsigned char materials[4];

		VoxelMaterialNodeData(VertexBuffer* model, Vec3 chunk_pos, Mat4 xform);

		void Draw(ShaderProgram* shader);
	};

	class VoxelMaterial : public Material
	{
		public:

			struct TerrainTexture
			{
				Texture2D* texture;
				int material_index;
				string name;

				TerrainTexture() : texture(NULL), name(), material_index(0) { }
				TerrainTexture(Texture2D* texture, string name, int material_index) : texture(texture), name(name), material_index(material_index) { }
			};

			ShaderProgram* shader;
			vector<TerrainTexture> textures;

			VoxelMaterial(ContentMan* content);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* material);
	};
}
