#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct TerrainTexture
	{
		Texture2D* texture;
		int material_index;
		string name;

		TerrainTexture() : texture(NULL), name(), material_index(0) { }
		TerrainTexture(Texture2D* texture, string name, int material_index) : texture(texture), name(name), material_index(material_index) { }
	};

	struct VoxelMaterialNodeData
	{
		VertexBuffer* model;
		Vec3 chunk_pos;
		Mat4 xform;

		unsigned char materials[4];

		VoxelMaterialNodeData(VertexBuffer* model, Vec3 chunk_pos, Mat4 xform, unsigned char materials[4]);

		void Draw(ShaderProgram* shader, Texture2D** textures);
	};

	class VoxelMaterial : public Material
	{
		public:

			ShaderProgram* shader;
			boost::unordered_map<unsigned char, TerrainTexture> textures;

			VoxelMaterial(ContentMan* content);

			void LoadTexture(Cache<Texture2D>* tex_cache, unsigned char material_index, string filename, string display_name);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* material);
	};
}
