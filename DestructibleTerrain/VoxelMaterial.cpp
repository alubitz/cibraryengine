#include "StdAfx.h"

#include "PerlinNoise.h"
#include "VoxelMaterial.h"

namespace DestructibleTerrain
{
	/*
	 * VoxelMaterialNodeData methods
	 */
	VoxelMaterialNodeData::VoxelMaterialNodeData(boost::unordered_map<unsigned char, VoxelMaterialVBO> vbos, VertexBuffer* depth_vbo, Vec3 chunk_pos, Mat4 xform) : vbos(vbos), depth_vbo(depth_vbo), chunk_pos(chunk_pos), xform(xform) { }

	void VoxelMaterialNodeData::Draw(ShaderProgram* shader, ShaderProgram* depth_shader, VoxelMaterial* material)
	{
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glMultMatrixf(xform.Transpose().values);

		shader->SetUniform<Vec3>("chunk_pos", &chunk_pos);

		glDisable(GL_BLEND);
		glDepthMask(true);

		depth_shader->UpdateUniforms();
		ShaderProgram::SetActiveProgram(depth_shader);
		
		depth_vbo->Draw();

		glEnable(GL_BLEND);
		glBlendFunc(GL_ONE, GL_ONE);
		glDepthMask(false);

		for(boost::unordered_map<unsigned char, VoxelMaterialVBO>::iterator iter = vbos.begin(); iter != vbos.end(); ++iter)
		{
			if(iter->first != 0)
			{
				Texture2D* texture = material->textures[iter->first].texture;
				shader->SetUniform<Texture2D>("texture", texture);

				ShaderProgram::SetActiveProgram(shader);

				iter->second.vbo->Draw();
			}
		}

		glPopMatrix();
	}



	/*
	 * VoxelMaterial methods
	 */
	VoxelMaterial::VoxelMaterial(ContentMan* content) : Material(4, Opaque, false)
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
		GLDEBUG();

		glEnable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);

		glDisable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		glEnable(GL_RESCALE_NORMAL);
		glDisable(GL_CULL_FACE);

		glColor4f(1.0, 1.0, 1.0, 1.0);

		Vec3 v;

		Texture2D* default_tex = textures[1].texture;
		shader->SetUniform<Texture2D>("texture", default_tex);

		shader->SetUniform<Vec3>("chunk_pos", &v);

		ShaderProgram::SetActiveProgram(shader);

		GLDEBUG();
	}
	void VoxelMaterial::Draw(RenderNode node) { ((VoxelMaterialNodeData*)node.data)->Draw(shader, depth_shader, this); }
	void VoxelMaterial::EndDraw()
	{
		ShaderProgram::SetActiveProgram(NULL);
		glDepthMask(true);
		
		GLDEBUG();
	}

	void VoxelMaterial::Cleanup(RenderNode node) { delete (VoxelMaterialNodeData*)node.data; }

	bool VoxelMaterial::Equals(Material* material) { return mclass_id == material->mclass_id; }
}

