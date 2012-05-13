#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	class VoxelMaterial;

	struct TerrainTexture
	{
		Texture2D* texture;
		int material_index;
		string name;

		TerrainTexture() : texture(NULL), name(), material_index(0) { }
		TerrainTexture(Texture2D* texture, string name, int material_index) : texture(texture), name(name), material_index(material_index) { }
	};

	struct VoxelMaterialVBO
	{
		unsigned char material;
		VertexBuffer* vbo;

		VoxelMaterialVBO() : vbo(NULL) { }
		VoxelMaterialVBO(unsigned char material, VertexBuffer* vbo) : material(material), vbo(vbo) { }
	};

	struct VoxelMaterialNodeData
	{
		boost::unordered_map<unsigned char, VoxelMaterialVBO> vbos;
		VertexBuffer* depth_vbo;

		Vec3 chunk_pos;
		Mat4 xform;

		VoxelMaterialNodeData(boost::unordered_map<unsigned char, VoxelMaterialVBO> vbos, VertexBuffer* depth_vbo, Vec3 chunk_pos, Mat4 xform);

		void Draw(ShaderProgram* shader, ShaderProgram* depth_shader, VoxelMaterial* material);
	};

	class VoxelMaterial : public Material
	{
		public:

			ShaderProgram* shader;
			ShaderProgram* depth_shader;
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
