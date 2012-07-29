#include "StdAfx.h"
#include "ParticleMaterial.h"

#include "Texture2D.h"
#include "Texture3D.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Shader.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * ParticleMaterial implementation
	 */
	struct ParticleMaterial::Imp
	{
		Texture* tex;
		bool is_3d;

		Imp(Texture2D* tex) : tex(tex), is_3d(false) { }
		Imp(Texture3D* tex) : tex(tex), is_3d(true) { }
	};




	/*
	 * ParticleMaterial methods
	 */
	ParticleMaterial::ParticleMaterial(Texture2D* tex, BlendStyle bs) : Material(1, bs, false), imp(new Imp(tex)) { }
	ParticleMaterial::ParticleMaterial(Texture3D* tex, BlendStyle bs) : Material(1, bs, false), imp(new Imp(tex)) { }

	void ParticleMaterial::InnerDispose()
	{
		delete imp;
		imp = NULL;
	}

	Texture* ParticleMaterial::GetTexture() { return imp->tex; }
	bool ParticleMaterial::IsTexture3D() { return imp->is_3d; }

	void ParticleMaterial::SetTexture(Texture2D* tex)
	{
		imp->tex = tex;
		imp->is_3d = false;
	}

	void ParticleMaterial::SetTexture(Texture3D* tex)
	{
		imp->tex = tex;
		imp->is_3d = true;
	}

	bool ParticleMaterial::Equals(Material* other) { return other != NULL && other->mclass_id == mclass_id && GetTexture() == ((ParticleMaterial*)other)->GetTexture(); }

	void ParticleMaterial::BeginDraw(SceneRenderer* renderer)
	{
		ShaderProgram::SetActiveProgram(NULL);

		switch (blend_style)
		{
			case Additive:
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE);
				break;

			case Alpha:
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				break;

			default:
				glDisable(GL_BLEND);
				break;
		}

		glDisable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glDisable(GL_LIGHTING);

		if(IsTexture3D())
		{
			glDisable(GL_TEXTURE_2D);

			glEnable(GL_TEXTURE_3D);
			Texture3D* tex = (Texture3D*)GetTexture();
			unsigned int gl_name = tex->GetGLName();
			glBindTexture(GL_TEXTURE_3D, gl_name);
		}
		else
		{
			glEnable(GL_TEXTURE_2D);
			Texture2D* tex = (Texture2D*)GetTexture();
			unsigned int gl_name = tex->GetGLName();
			glBindTexture(GL_TEXTURE_2D, gl_name);
		}

		glDepthMask(false);

		glMatrixMode(GL_TEXTURE);
		glPushMatrix();
		glLoadIdentity();

		glBegin(GL_QUADS);
	}

	void ParticleMaterial::EndDraw()
	{
		glEnd();

		if (IsTexture3D())
			glDisable(GL_TEXTURE_3D);

		glMatrixMode(GL_TEXTURE);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);

		glDepthMask(true);

		GLDEBUG();
	}
	void ParticleMaterial::Draw(RenderNode node) { ((ParticleMaterialNodeData*)node.data)->Execute(); }

	void ParticleMaterial::Cleanup(RenderNode node) { delete (ParticleMaterialNodeData*)node.data; }




	/*
	 * ParticleMaterialNodeData methods
	 */
	ParticleMaterialNodeData::ParticleMaterialNodeData(Vec3 pos, float radius, float angle, CameraView* camera) : radius(radius), pos(pos), angle(angle), red(1), green(1), blue(1), alpha(1), third_coord(0), camera_view(camera) { }

	void PMNDVert(Vec3 vert) { glVertex3f(vert.x, vert.y, vert.z); }

	void ParticleMaterialNodeData::Execute()
	{
		glColor4f(red, green, blue, alpha);

		Vec3 cam_right = camera_view->GetRight() * radius;
		Vec3 cam_up = camera_view->GetUp() * radius;
		float cos_val = cosf(angle), sin_val = sinf(angle);
		Vec3 use_right = cam_right * cos_val + cam_up * sin_val;
		Vec3 use_up = cam_right * -sin_val + cam_up * cos_val;

		glTexCoord3f(0, 0, third_coord);
		PMNDVert(pos - use_right - use_up);
		glTexCoord3f(0, 1, third_coord);
		PMNDVert(pos - use_right + use_up);
		glTexCoord3f(1, 1, third_coord);
		PMNDVert(pos + use_right + use_up);
		glTexCoord3f(1, 0, third_coord);
		PMNDVert(pos + use_right - use_up);
	}
}
