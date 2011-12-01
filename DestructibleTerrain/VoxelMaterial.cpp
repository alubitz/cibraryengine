#include "StdAfx.h"

#include "PerlinNoise.h"
#include "VoxelMaterial.h"

namespace DestructibleTerrain
{
	/*
	 * VoxelMaterialNodeData methods
	 */
	VoxelMaterialNodeData::VoxelMaterialNodeData(VertexBuffer* model, Vec3 chunk_pos, Mat4 xform) :
		model(model),
		chunk_pos(chunk_pos),
		xform(xform)
	{
	}

	void VoxelMaterialNodeData::Draw(ShaderProgram* shader)
	{
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glMultMatrixf(xform.Transpose().values);

		shader->SetUniform<Vec3>("chunk_pos", &chunk_pos);
		shader->UpdateUniforms();

		model->Draw();

		glPopMatrix();
	}




	/*
	 * VoxelMaterial methods
	 */
	VoxelMaterial::VoxelMaterial(ContentMan* content) : Material(4, Opaque, false)
	{
		Texture2D* rock = content->GetCache<Texture2D>()->Load("rock3d");
		int texture_res = min(rock->width, rock->height);

		texture = new Texture3D(texture_res, texture_res, texture_res, rock->byte_data, true, false);

		Shader* vs = content->GetCache<Shader>()->Load("pass-v");
		Shader* fs = content->GetCache<Shader>()->Load("terrain-f");

		shader = new ShaderProgram(vs, fs);
		shader->AddUniform<Texture3D>(new UniformTexture3D("diffuse", 0));
		shader->AddUniform<Vec3>(new UniformVector3("chunk_pos"));
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

		shader->SetUniform<Texture3D>("diffuse", texture);
		shader->SetUniform<Vec3>("chunk_pos", &v);
		ShaderProgram::SetActiveProgram(shader);

		GLDEBUG();
	}
	void VoxelMaterial::Draw(RenderNode node) { ((VoxelMaterialNodeData*)node.data)->Draw(shader); }
	void VoxelMaterial::EndDraw()
	{ 
		ShaderProgram::SetActiveProgram(NULL);	
		GLDEBUG(); 
	}

	void VoxelMaterial::Cleanup(RenderNode node) { delete (VoxelMaterialNodeData*)node.data; }

	bool VoxelMaterial::Equals(Material* material) { return mclass_id == material->mclass_id; }
}

