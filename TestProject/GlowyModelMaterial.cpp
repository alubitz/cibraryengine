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
		bool is_3d;

		Imp(Texture2D* tex) : tex(tex), is_3d(false) { }
		Imp(Texture3D* tex) : tex(tex), is_3d(true) { }
	};

	GlowyModelMaterial::GlowyModelMaterial(Texture2D* texture) : Material(3, Alpha, false), imp(new Imp(texture)) { }
	GlowyModelMaterial::GlowyModelMaterial(Texture3D* texture) : Material(3, Alpha, false), imp(new Imp(texture)) { }

	void GlowyModelMaterial::InnerDispose() { delete imp; imp = NULL; }

	void GlowyModelMaterial::BeginDraw(SceneRenderer* renderer)
	{
		GLErrorDebug(__LINE__, __FILE__);

		glDisable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(false);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_COLOR);

		glDisable(GL_LIGHTING);
		glColor4f(1.0, 1.0, 1.0, 1.0);

		if(imp->is_3d)
		{
			glDisable(GL_TEXTURE_2D);

			glEnable(GL_TEXTURE_3D);
			unsigned int gl_name = imp->tex->GetGLName();
			glBindTexture(GL_TEXTURE_3D, gl_name);
		}
		else
		{
			glEnable(GL_TEXTURE_2D);
			unsigned int gl_name = imp->tex->GetGLName();
			glBindTexture(GL_TEXTURE_2D, gl_name);
		}

		GLErrorDebug(__LINE__, __FILE__);
	}

	void GlowyModelMaterial::EndDraw()
	{
		if (imp->is_3d)
			glDisable(GL_TEXTURE_3D);

		glDisable(GL_BLEND);
		glDisable(GL_RESCALE_NORMAL);
		glDepthMask(true);

		GLErrorDebug(__LINE__, __FILE__);
	}

	void GlowyModelMaterial::Draw(RenderNode node) { ((GlowyModelMaterialNodeData*)node.data)->Draw(); }

	void GlowyModelMaterial::Cleanup(RenderNode node) { delete (GlowyModelMaterialNodeData*)node.data; }

	bool GlowyModelMaterial::Equals(Material* material)
	{
		return mclass_id == material->mclass_id && imp->is_3d == ((GlowyModelMaterial*)material)->imp->is_3d && imp->tex->GetGLName() == ((GlowyModelMaterial*)material)->imp->tex->GetGLName();
	}
}
