#include "StdAfx.h"

#include "PerlinNoise.h"
#include "VoxelMaterial.h"

#include "Shader.h"
#include "RenderNode.h"
#include "UniformVariables.h"

#define DEBUG_N_CHUNKS_VBOS_TRIANGLES 0

namespace DestructibleTerrain
{
	/*
	 * VoxelMaterial implementation struct for data cached between calls to BeginDraw and EndDraw
	 */
	struct VoxelMaterial::DrawCache
	{
		struct ChunkVBO
		{
			Mat4 xform;
			Vec3 offset;
			VertexBuffer* vbo;

			ChunkVBO(const Mat4& xform, const Vec3& offset, VertexBuffer* vbo) : xform(xform), offset(offset), vbo(vbo) { }
		};

		vector<ChunkVBO> depth_vbos;
		boost::unordered_map<unsigned char, vector<ChunkVBO> > material_vbos;

		DrawCache() : depth_vbos(), material_vbos() { }

		~DrawCache() { }

		void Draw(ShaderProgram* shader, ShaderProgram* depth_shader, VoxelMaterial* material)
		{
#if DEBUG_N_CHUNKS_VBOS_TRIANGLES
			unsigned int num_chunks = 0;
			unsigned int num_vbos = 0;
			unsigned int num_triangles = 0;
#endif

			ShaderProgram::SetActiveProgram(depth_shader);

			glMatrixMode(GL_MODELVIEW);

			glDisable(GL_BLEND);
			glEnable(GL_DEPTH_TEST);
			glDepthMask(true);
			glDepthFunc(GL_LESS);

			for(vector<ChunkVBO>::iterator iter = depth_vbos.begin(); iter != depth_vbos.end(); ++iter)
			{
				glPushMatrix();
				glMultMatrixf(iter->xform.Transpose().values);
		
				iter->vbo->Draw();

				glPopMatrix();

#if DEBUG_N_CHUNKS_VBOS_TRIANGLES
				++num_chunks;
				++num_vbos;
				num_triangles += iter->vbo->GetNumVerts() / 3;
#endif
			}

			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE);
			glDepthMask(false);
			glDepthFunc(GL_EQUAL);

			for(boost::unordered_map<unsigned char, vector<ChunkVBO> >::const_iterator iter = material_vbos.begin(); iter != material_vbos.end(); ++iter)
			{
				const vector<ChunkVBO>& vbos = iter->second;

				Texture2D* texture = material->textures[iter->first].texture;
				shader->SetUniform<Texture2D>("texture", texture);

				for(vector<ChunkVBO>::const_iterator jter = vbos.begin(); jter != vbos.end(); ++jter)
				{
					glPushMatrix();
					glMultMatrixf(jter->xform.Transpose().values);
		
					Vec3 chunk_pos = jter->offset;
					shader->SetUniform<Vec3>("chunk_pos", &chunk_pos);

					ShaderProgram::SetActiveProgram(shader);

					jter->vbo->Draw();

					glPopMatrix();

#if DEBUG_N_CHUNKS_VBOS_TRIANGLES
					++num_vbos;
					num_triangles += jter->vbo->GetNumVerts() / 3;
#endif
				}
			}

#if DEBUG_N_CHUNKS_VBOS_TRIANGLES
			Debug(((stringstream&)(stringstream() << "chunks = " << num_chunks << "; vbos = " << num_vbos << "; triangles = " << num_triangles << endl)).str());
#endif
		}
	};




	/*
	 * VoxelMaterialNodeData methods
	 */
	VoxelMaterialNodeData::VoxelMaterialNodeData(boost::unordered_map<unsigned char, VoxelMaterialVBO> vbos, VertexBuffer* depth_vbo, Vec3 chunk_pos, Mat4 xform) : vbos(vbos), depth_vbo(depth_vbo), chunk_pos(chunk_pos), xform(xform) { }




	/*
	 * VoxelMaterial methods
	 */
	VoxelMaterial::VoxelMaterial(ContentMan* content) : Material(4, Opaque, false), draw_cache(NULL)
	{
		Cache<Texture2D>* tex_cache = content->GetCache<Texture2D>();

		unsigned char next_mat = 1;

		LoadTexture(tex_cache, next_mat++, "rock1",			"Rock 1");
		LoadTexture(tex_cache, next_mat++, "rock2",			"Rock 2");
		LoadTexture(tex_cache, next_mat++, "rock3",			"Rock 3");
		LoadTexture(tex_cache, next_mat++, "sand1",			"Sand");
		LoadTexture(tex_cache, next_mat++, "grass1",		"Grass 1");
		LoadTexture(tex_cache, next_mat++, "grass2",		"Grass 2");
		LoadTexture(tex_cache, next_mat++, "dirt1",			"Dirt");
		LoadTexture(tex_cache, next_mat++, "gravel1",		"Gravel 1");
		LoadTexture(tex_cache, next_mat++, "gravel2",		"Gravel 2");

		// single-material shader
		Shader* vs = content->GetCache<Shader>()->Load("terrain-v");
		Shader* fs = content->GetCache<Shader>()->Load("terrain-f");

		shader = new ShaderProgram(vs, fs);
		shader->AddUniform<Texture2D>(new UniformTexture2D("texture", 0));

		shader->AddUniform<Vec3>(new UniformVector3("chunk_pos"));

		// depth shader
		Shader* depth_vs = content->GetCache<Shader>()->Load("terrain_depth-v");
		Shader* depth_fs = content->GetCache<Shader>()->Load("terrain_depth-f");

		depth_shader = new ShaderProgram(depth_vs, depth_fs);
	}

	void VoxelMaterial::LoadTexture(Cache<Texture2D>* tex_cache, unsigned char material_index, string filename, string display_name) { textures[material_index] = TerrainTexture(tex_cache->Load(filename), display_name, material_index); }

	void VoxelMaterial::BeginDraw(SceneRenderer* renderer)
	{
		assert(draw_cache == NULL);
		draw_cache = new DrawCache();

		GLDEBUG();

		glEnable(GL_CULL_FACE);

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		glEnable(GL_RESCALE_NORMAL);

		GLDEBUG();
	}

	void VoxelMaterial::Draw(RenderNode node)
	{
		assert(draw_cache != NULL);

		VoxelMaterialNodeData* vmnd = (VoxelMaterialNodeData*)node.data;

		const Vec3& chunk_pos = vmnd->chunk_pos;
		const Mat4& xform = vmnd->xform;

		draw_cache->depth_vbos.push_back(DrawCache::ChunkVBO(xform, chunk_pos, vmnd->depth_vbo));

		for(boost::unordered_map<unsigned char, VoxelMaterialVBO>::iterator iter = vmnd->vbos.begin(); iter != vmnd->vbos.end(); ++iter)
		{
			unsigned char material = iter->first;

			boost::unordered_map<unsigned char, vector<DrawCache::ChunkVBO> >::iterator found = draw_cache->material_vbos.find(material);

			if(found == draw_cache->material_vbos.end())
			{
				vector<DrawCache::ChunkVBO>& vbos = draw_cache->material_vbos[material] = vector<DrawCache::ChunkVBO>();
				vbos.push_back(DrawCache::ChunkVBO(xform, chunk_pos, iter->second.vbo));
			}
			else
				found->second.push_back(DrawCache::ChunkVBO(xform, chunk_pos, iter->second.vbo));
		}
	}

	void VoxelMaterial::EndDraw()
	{
		assert(draw_cache != NULL);

		draw_cache->Draw(shader, depth_shader, this);

		ShaderProgram::SetActiveProgram(NULL);
		glEnable(GL_CULL_FACE);
		glDepthMask(true);
		glDepthFunc(GL_LEQUAL);
		
		GLDEBUG();

		delete draw_cache;
		draw_cache = NULL;
	}

	void VoxelMaterial::Cleanup(RenderNode node) { delete (VoxelMaterialNodeData*)node.data; }

	bool VoxelMaterial::Equals(Material* material) { return mclass_id == material->mclass_id; }
}

