#include "StdAfx.h"

#include "PerlinNoise.h"
#include "VoxelMaterial.h"

namespace DestructibleTerrain
{
	/*
	 * VoxelMaterialNodeData methods
	 */
	VoxelMaterialNodeData::VoxelMaterialNodeData(VertexBuffer* model, Vec3 chunk_pos, Mat4 xform, unsigned char materials_[4]) :
		model(model),
		chunk_pos(chunk_pos),
		xform(xform)
	{
		materials[0] = materials_[0];
		materials[1] = materials_[1];
		materials[2] = materials_[2];
		materials[3] = materials_[3];
	}

	void VoxelMaterialNodeData::Draw(ShaderProgram* shader, Texture2D** textures)
	{
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glMultMatrixf(xform.Transpose().values);

		shader->SetUniform<Vec3>("chunk_pos", &chunk_pos);

		if(textures[0]) { shader->SetUniform<Texture2D>("texture_a", textures[0]); }
		if(textures[1]) { shader->SetUniform<Texture2D>("texture_b", textures[1]); }
		if(textures[2]) { shader->SetUniform<Texture2D>("texture_c", textures[2]); }
		if(textures[3]) { shader->SetUniform<Texture2D>("texture_d", textures[3]); }

		shader->UpdateUniforms();

		model->Draw();

		glPopMatrix();
	}




	/*
	 * VoxelMaterial methods
	 */
	VoxelMaterial::VoxelMaterial(ContentMan* content) : Material(4, Opaque, false)
	{
		Cache<Texture2D>* tex_cache = content->GetCache<Texture2D>();

		LoadTexture(tex_cache, 1, "rock1", "Rock 1");
		LoadTexture(tex_cache, 2, "rock2", "Rock 2");
		LoadTexture(tex_cache, 3, "rock3", "Rock 3");
		LoadTexture(tex_cache, 4, "sand1", "Sand");

		Shader* vs = content->GetCache<Shader>()->Load("terrain-v");
		Shader* fs = content->GetCache<Shader>()->Load("terrain-f");

		shader = new ShaderProgram(vs, fs);
		shader->AddUniform<Texture2D>(new UniformTexture2D("texture_a", 0));
		shader->AddUniform<Texture2D>(new UniformTexture2D("texture_b", 1));
		shader->AddUniform<Texture2D>(new UniformTexture2D("texture_c", 2));
		shader->AddUniform<Texture2D>(new UniformTexture2D("texture_d", 3));

		shader->AddUniform<Vec3>(new UniformVector3("chunk_pos"));
	}

	void VoxelMaterial::LoadTexture(Cache<Texture2D>* tex_cache, unsigned char material_index, string filename, string display_name)
	{
		textures[material_index] = TerrainTexture(tex_cache->Load(filename), display_name, material_index);
	}

	void VoxelMaterial::BeginDraw(SceneRenderer* renderer)
	{ 
		GLDEBUG();

		glEnable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(true);
		glDisable(GL_BLEND);

		glDisable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		glEnable(GL_RESCALE_NORMAL);
		glDisable(GL_CULL_FACE);

		glColor4f(1.0, 1.0, 1.0, 1.0);

		Vec3 v;

		Texture2D* default_tex = textures[1].texture;

		shader->SetUniform<Texture2D>("texture_a", default_tex);
		shader->SetUniform<Texture2D>("texture_b", default_tex);
		shader->SetUniform<Texture2D>("texture_c", default_tex);
		shader->SetUniform<Texture2D>("texture_d", default_tex);

		shader->SetUniform<Vec3>("chunk_pos", &v);

		ShaderProgram::SetActiveProgram(shader);

		GLDEBUG();
	}
	void VoxelMaterial::Draw(RenderNode node)
	{
		Texture2D* use_textures[] = { NULL, NULL, NULL, NULL };
		VoxelMaterialNodeData& vmnd = *((VoxelMaterialNodeData*)node.data);

		for(int i = 0; i < 4; ++i)
		{
			boost::unordered_map<unsigned char, TerrainTexture>::iterator found = textures.find(vmnd.materials[i]);
			if(found != textures.end())
				use_textures[i] = found->second.texture;
		}

		((VoxelMaterialNodeData*)node.data)->Draw(shader, use_textures);
	}
	void VoxelMaterial::EndDraw()
	{ 
		ShaderProgram::SetActiveProgram(NULL);	
		GLDEBUG(); 
	}

	void VoxelMaterial::Cleanup(RenderNode node) { delete (VoxelMaterialNodeData*)node.data; }

	bool VoxelMaterial::Equals(Material* material) { return mclass_id == material->mclass_id; }
}

