#include "DSNMaterial.h"

namespace Test
{
	/*
	 * Static stuff...
	 */
	Texture1D* CreateDefaultBoneMatrices()
	{
		vector<Mat4> matrices = vector<Mat4>();
		matrices.push_back(Mat4::Identity());

		return SkinnedCharacter::MatricesToTexture1D(matrices);
	}
	Texture1D* DSNMaterial::default_bone_matrices = NULL;




	/*
	 * DSNMaterial methods
	 */
	DSNMaterial::DSNMaterial(ShaderProgram* shader, Texture2D* diffuse, Texture2D* specular, Texture2D* normal, TextureCube* ambient) :
		Material(2, Opaque, true),
		shader(shader),
		diffuse(diffuse),
		specular(specular),
		normal(normal),
		ambient(ambient)
	{
		// now would be a good time to init the default bone matrices (we have an OpenGL context, nay?)
		if(default_bone_matrices == NULL)
			default_bone_matrices = CreateDefaultBoneMatrices();
	}

	void DSNMaterial::InnerDispose()
	{
		Material::InnerDispose();
	}

	void DSNMaterial::BeginDraw(SceneRenderer* renderer)
	{
		scene = renderer;
		node_data = vector<DSNMaterialNodeData*>();

		GLErrorDebug(__LINE__, __FILE__);
	}

	void DSNMaterial::Draw(RenderNode node) { node_data.push_back((DSNMaterialNodeData*)node.data); }

	void DSNMaterial::EndDraw()
	{
		GLErrorDebug(__LINE__, __FILE__);
		glMatrixMode(GL_MODELVIEW);

		glEnable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_LIGHTING);
		glEnable(GL_RESCALE_NORMAL);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		GLErrorDebug(__LINE__, __FILE__);

		map<LightSource*, vector<DSNMaterialNodeData*> > light_effects = map<LightSource*, vector<DSNMaterialNodeData*> >();
		for (vector<LightSource*>::iterator iter = scene->lights.begin(); iter != scene->lights.end(); iter++)
		{
			light_effects[*iter] = vector<DSNMaterialNodeData*>();
			for(vector<DSNMaterialNodeData*>::iterator jter = node_data.begin(); jter != node_data.end(); jter++)
				if ((*iter)->IsWithinLightingRange((*jter)->bs))
					light_effects[*iter].push_back(*jter);
		}

		shader->SetUniform<Texture2D>("diffuse", diffuse);
		shader->SetUniform<Texture2D>("specular", specular);
		shader->SetUniform<Texture2D>("normal_map", normal);
		shader->SetUniform<TextureCube>("ambient_cubemap", ambient);

		GLErrorDebug(__LINE__, __FILE__);

		Mat4 view_matrix = scene->camera->GetViewMatrix();
		shader->SetUniform<Mat4>("inv_view", &view_matrix);

		shader->SetUniform<Texture1D>("bone_matrices", default_bone_matrices);
		int bone_count = 1;
		shader->SetUniform<int>("bone_count", &bone_count);

		ShaderProgram::SetActiveProgram(shader);

		for (vector<LightSource*>::iterator iter = scene->lights.begin(); iter != scene->lights.end(); iter++)
		{
			glPushMatrix();
			glLoadIdentity();

			(*iter)->SetLight(0);

			glPopMatrix();
			GLErrorDebug(__LINE__, __FILE__);

			for(vector<DSNMaterialNodeData*>::iterator jter = light_effects[*iter].begin(); jter != light_effects[*iter].end(); jter++)
			{
				DSNMaterialNodeData* node_data = *jter;
				shader->SetUniform<Texture1D>("bone_matrices", node_data->bone_matrices);

				bone_count = node_data->bone_count;
				shader->SetUniform<int>("bone_count", &bone_count);

				shader->UpdateUniforms();

				GLErrorDebug(__LINE__, __FILE__);
				node_data->Draw();
				GLErrorDebug(__LINE__, __FILE__);
			}

			(*iter)->UnsetLight(0);
		}

		ShaderProgram::SetActiveProgram(NULL);

		glDepthFunc(GL_LEQUAL);
		glDisable(GL_RESCALE_NORMAL);

		GLErrorDebug(__LINE__, __FILE__);
	}

	void DSNMaterial::Cleanup(RenderNode node) { delete (DSNMaterialNodeData*)node.data; }

	bool DSNMaterial::Equals(Material* material)
	{
		return mclass_id == material->mclass_id && diffuse->GetGLName() == ((DSNMaterial*)material)->diffuse->GetGLName()  && specular->GetGLName() == ((DSNMaterial*)material)->specular->GetGLName() && normal->GetGLName() == ((DSNMaterial*)material)->normal->GetGLName();
	}




	/*
	 * DSNMaterialNodeData methods
	 */
	DSNMaterialNodeData::DSNMaterialNodeData(SkinVInfoVertexBuffer* model, Mat4 xform, Sphere bs) :
		model(model),
		xform(xform),
		bs(bs),
		bone_matrices(DSNMaterial::default_bone_matrices),
		bone_count(1)
	{
	}

	DSNMaterialNodeData::DSNMaterialNodeData(SkinVInfoVertexBuffer* model, Mat4 xform, Sphere bs, Texture1D* bone_matrices, int bone_count) :
		model(model),
		xform(xform),
		bs(bs),
		bone_matrices(bone_matrices),
		bone_count(bone_count)
	{
	}

	void DSNMaterialNodeData::Draw()
	{
		GLErrorDebug(__LINE__, __FILE__);
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glMultMatrixf(xform.Transpose().values);

		model->Draw();

		GLErrorDebug(__LINE__, __FILE__);
		glPopMatrix();
	}
}
