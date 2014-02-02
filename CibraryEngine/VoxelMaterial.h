#pragma once

#include "StdAfx.h"
#include "Texture2D.h"
#include "Material.h"
#include "VertexBuffer.h"

namespace CibraryEngine
{
	class VoxelMaterial;

	struct TerrainTexture
	{
		Texture2D* texture;
		int material_index;
		string name;

		TerrainTexture() : texture(NULL), name(), material_index(0) { }
		TerrainTexture(Texture2D* texture, const string& name, int material_index) : texture(texture), name(name), material_index(material_index) { }
	};

	struct VoxelMaterialVBO
	{
		unsigned char material;
		VertexBuffer* vbo;

		VoxelMaterialVBO() : vbo(NULL) { }
		VoxelMaterialVBO(unsigned char material, VertexBuffer* vbo) : material(material), vbo(vbo) { }		
	};

	struct VoxelMaterialVBOBuilder
	{
		VoxelMaterialVBO* vbo;

		float* vert_ptr;
		float* normal_ptr;
		float* mat_ptr;

		unsigned int num_verts;

		VoxelMaterialVBOBuilder() : vbo(NULL), normal_ptr(NULL), mat_ptr(NULL), num_verts(0) { }
		VoxelMaterialVBOBuilder(VoxelMaterialVBO* vbo) :
			vbo(vbo),
			vert_ptr  (vbo->vbo->GetFloatPointer( "gl_Vertex"       )),
			normal_ptr(vbo->vbo->GetFloatPointer( "gl_Normal"       )),
			mat_ptr   (vbo->vbo->GetFloatPointer( "material_weight" )),
			num_verts(0)
		{
		}

		void AddVert(const Vec3& pos, const Vec3& normal, float mat_weight)
		{
			*(vert_ptr++)   = pos.x;
			*(vert_ptr++)   = pos.y;
			*(vert_ptr++)   = pos.z;

			*(normal_ptr++) = normal.x;
			*(normal_ptr++) = normal.y;
			*(normal_ptr++) = normal.z;

			*(mat_ptr++)    = mat_weight;

			++num_verts;
		}
	};

	struct VoxelMaterialNodeData
	{
		boost::unordered_map<unsigned char, VoxelMaterialVBO> vbos;
		VertexBuffer* depth_vbo;

		Vec3 chunk_pos;
		Mat4 xform;

		VoxelMaterialNodeData(boost::unordered_map<unsigned char, VoxelMaterialVBO> vbos, VertexBuffer* depth_vbo, const Vec3& chunk_pos, const Mat4& xform);
	};

	class VoxelMaterial : public Material
	{
		private:

			struct DrawCache;				// private data stored only between calls to BeginDraw and EndDraw
			DrawCache* draw_cache;

		public:

			ShaderProgram* shader;
			ShaderProgram* depth_shader;
			boost::unordered_map<unsigned char, TerrainTexture> textures;

			VoxelMaterial(ContentMan* content);

			void LoadTexture(Cache<Texture2D>* tex_cache, unsigned char material_index, const string& filename, const string& display_name);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(const RenderNode& node);

			void Cleanup(const RenderNode& node);

			bool Equals(const Material* material) const;
	};
}
