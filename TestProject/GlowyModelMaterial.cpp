#include "GlowyModelMaterial.h"

namespace Test
{
	/*
	 * GlowyModelMaterialNodeData methods
	 */
	GlowyModelMaterialNodeData::GlowyModelMaterialNodeData(VertexBufferI* model, Mat4 xform) : model(model), xform(xform) { }

	void GlowyModelMaterialNodeData::Draw()
	{
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glMultMatrixf(xform.Transpose().values);

		model->Draw();

		glPopMatrix();
	}




	/*
	 * GlowyModelMaterial implementation
	 */
	struct GlowyModelMaterial::Imp
	{
		Texture* tex;
		ShaderProgram* shader;
		bool is_3d;

		Imp(Texture2D* tex, ShaderProgram* shader) : tex(tex), shader(shader), is_3d(false) { }
		Imp(Texture3D* tex, ShaderProgram* shader) : tex(tex), shader(shader), is_3d(true) { }
	};

	GlowyModelMaterial::GlowyModelMaterial(Texture2D* texture, ShaderProgram* shader) : Material(3, Alpha, false), imp(new Imp(texture, shader)) { }
	GlowyModelMaterial::GlowyModelMaterial(Texture3D* texture, ShaderProgram* shader) : Material(3, Alpha, false), imp(new Imp(texture, shader)) { }

	void GlowyModelMaterial::InnerDispose() { delete imp; imp = NULL; }

	void GlowyModelMaterial::BeginDraw(SceneRenderer* renderer)
	{
		GLDEBUG();

		glDisable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(false);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_COLOR);

		glColor4f(1.0, 1.0, 1.0, 1.0);

		if(imp->is_3d)
			imp->shader->SetUniform<Texture3D>("texture", (Texture3D*)imp->tex);
		else
			imp->shader->SetUniform<Texture2D>("texture", (Texture2D*)imp->tex);

		ShaderProgram::SetActiveProgram(imp->shader);

		GLDEBUG();
	}

	void GlowyModelMaterial::EndDraw()
	{
		ShaderProgram::SetActiveProgram(NULL);

		glDisable(GL_BLEND);
		glDisable(GL_RESCALE_NORMAL);
		glDepthMask(true);

		GLDEBUG();
	}

	void GlowyModelMaterial::Draw(RenderNode node) { ((GlowyModelMaterialNodeData*)node.data)->Draw(); }

	void GlowyModelMaterial::Cleanup(RenderNode node) { delete (GlowyModelMaterialNodeData*)node.data; }

	bool GlowyModelMaterial::Equals(Material* material)
	{
		return mclass_id == material->mclass_id && imp->is_3d == ((GlowyModelMaterial*)material)->imp->is_3d && imp->tex->GetGLName() == ((GlowyModelMaterial*)material)->imp->tex->GetGLName();
	}
}
