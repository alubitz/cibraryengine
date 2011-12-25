#include "StdAfx.h"
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
	DSNMaterial::DSNMaterial(ShaderProgram* shader, ShaderProgram* shadow_shader, Texture2D* diffuse, Texture2D* specular, Texture2D* normal, BlendStyle blend_style) :
		Material(2, Opaque, true),
		shader(shader),
		shadow_shader(shadow_shader),
		diffuse(diffuse),
		specular(specular),
		normal(normal),
		blend_style(blend_style)
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

		GLDEBUG();
	}

	void DSNMaterial::Draw(RenderNode node) { node_data.push_back((DSNMaterialNodeData*)node.data); }

	void DrawNodeData(DSNMaterialNodeData* data, ShaderProgram* use_shader);

	void DSNMaterial::EndDraw()
	{
		bool shadow = scene->DrawingDepth();
		ShaderProgram* use_shader = shadow ? shadow_shader : shader;

		GLDEBUG();

		GLboolean color_mask[4];
		glGetBooleanv(GL_COLOR_WRITEMASK, color_mask);

		bool shadow_color = color_mask[0] || color_mask[1] || color_mask[2] || color_mask[3];

		glEnable(GL_LIGHTING);
		if(shadow_color)
			glColorMask(true, true, true, true);
		glDepthMask(true);

		glMatrixMode(GL_MODELVIEW);

		glEnable(GL_CULL_FACE);
		glCullFace(shadow ? GL_FRONT : GL_BACK);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_RESCALE_NORMAL);

		glDisable(GL_BLEND);
		
		GLDEBUG();

		map<LightSource*, vector<DSNMaterialNodeData*> > light_effects = map<LightSource*, vector<DSNMaterialNodeData*> >();
		if(!shadow)
		{
			for (vector<LightSource*>::iterator iter = scene->lights.begin(); iter != scene->lights.end(); ++iter)
			{
				light_effects[*iter] = vector<DSNMaterialNodeData*>();
				for(vector<DSNMaterialNodeData*>::iterator jter = node_data.begin(); jter != node_data.end(); ++jter)
					if ((*iter)->IsWithinLightingRange((*jter)->bs))
						light_effects[*iter].push_back(*jter);
			}

			use_shader->SetUniform<Texture2D>("diffuse", diffuse);
			use_shader->SetUniform<Texture2D>("specular", specular);
			use_shader->SetUniform<Texture2D>("normal_map", normal);
		}

		GLDEBUG();

		use_shader->SetUniform<Texture1D>("bone_matrices", default_bone_matrices);

		int bone_count = 1;
		use_shader->SetUniform<int>("bone_count", &bone_count);

		ShaderProgram::SetActiveProgram(use_shader);

		if(shadow)
		{
			for(vector<DSNMaterialNodeData*>::iterator jter = node_data.begin(); jter != node_data.end(); ++jter)
				DrawNodeData(*jter, use_shader);
		}
		else
		{
			for (vector<LightSource*>::iterator iter = scene->lights.begin(); iter != scene->lights.end(); ++iter)
			{
				glPushMatrix();
				glLoadIdentity();

				(*iter)->SetLight(0);

				glPopMatrix();
				GLDEBUG();

				for(vector<DSNMaterialNodeData*>::iterator jter = light_effects[*iter].begin(); jter != light_effects[*iter].end(); ++jter)
					DrawNodeData(*jter, use_shader);

				(*iter)->UnsetLight(0);
			}
		}

		ShaderProgram::SetActiveProgram(NULL);

		glDepthFunc(GL_LEQUAL);
		glDisable(GL_RESCALE_NORMAL);
		glCullFace(GL_BACK);

		glColorMask(color_mask[0], color_mask[1], color_mask[2], color_mask[3]);

		GLDEBUG();
	}

	void DrawNodeData(DSNMaterialNodeData* data, ShaderProgram* use_shader)
	{
		use_shader->SetUniform<Texture1D>("bone_matrices", data->bone_matrices);

		int bone_count = data->bone_count;
		use_shader->SetUniform<int>("bone_count", &bone_count);

		use_shader->UpdateUniforms();

		GLDEBUG();
		data->Draw();
		GLDEBUG();
	}

	void DSNMaterial::Cleanup(RenderNode node) { delete (DSNMaterialNodeData*)node.data; }

	bool DSNMaterial::Equals(Material* material)
	{
		return mclass_id == material->mclass_id && diffuse->GetGLName() == ((DSNMaterial*)material)->diffuse->GetGLName()  && specular->GetGLName() == ((DSNMaterial*)material)->specular->GetGLName() && normal->GetGLName() == ((DSNMaterial*)material)->normal->GetGLName();
	}




	/*
	 * DSNMaterialNodeData methods
	 */
	DSNMaterialNodeData::DSNMaterialNodeData(VertexBuffer* model, Mat4 xform, Sphere bs) :
		model(model),
		xform(xform),
		bs(bs),
		bone_matrices(DSNMaterial::default_bone_matrices),
		bone_count(1)
	{
	}

	DSNMaterialNodeData::DSNMaterialNodeData(VertexBuffer* model, Mat4 xform, Sphere bs, Texture1D* bone_matrices, int bone_count) :
		model(model),
		xform(xform),
		bs(bs),
		bone_matrices(bone_matrices),
		bone_count(bone_count)
	{
	}

	void DSNMaterialNodeData::Draw()
	{
		GLDEBUG();
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glMultMatrixf(xform.Transpose().values);

		model->Draw();

		GLDEBUG();
		glPopMatrix();
	}
}
