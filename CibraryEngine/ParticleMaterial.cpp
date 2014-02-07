#include "StdAfx.h"
#include "ParticleMaterial.h"

#include "Texture2D.h"
#include "Texture3D.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Shader.h"
#include "VertexBuffer.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * ParticleMaterial private implementation struct
	 */
	struct ParticleMaterial::Imp
	{
		Texture* tex;
		bool is_3d;

		CameraView* camera;

		VertexBuffer* vbo;
		vector<ParticleMaterialNodeData*> node_data;

		Imp(Texture2D* tex) : tex(tex), is_3d(false), camera(NULL), vbo(NULL) { }
		Imp(Texture3D* tex) : tex(tex), is_3d(true),  camera(NULL), vbo(NULL) { }

		~Imp() { if(vbo) { vbo->Dispose(); delete vbo; vbo = NULL; } }

		void EndDraw()
		{
			if(vbo == NULL)
			{
				vbo = new VertexBuffer(Quads);
				vbo->AddAttribute("gl_Vertex",         Float, 3);
				vbo->AddAttribute("gl_MultiTexCoord0", Float, 3);
				vbo->AddAttribute("gl_Color",          Float, 4);
			}

			vbo->SetNumVerts(0);									// keep old data from being copied if a resize is needed
			vbo->SetNumVerts(node_data.size() * 4);

			float* vert_ptr  = vbo->GetFloatPointer("gl_Vertex");
			float* uvw_ptr   = vbo->GetFloatPointer("gl_MultiTexCoord0");
			float* color_ptr = vbo->GetFloatPointer("gl_Color");

			Vec3 camera_up    = camera->GetUp();
			Vec3 camera_right = camera->GetRight();

			for(vector<ParticleMaterialNodeData*>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
				(*iter)->PutQuad(vert_ptr, uvw_ptr, color_ptr, camera_up, camera_right);

			vbo->InvalidateVBO();

			vbo->Draw();
		}
	};




	/*
	 * ParticleMaterial methods
	 */
	ParticleMaterial::ParticleMaterial(Texture2D* tex, BlendStyle bs) : Material(1, bs, false), imp(new Imp(tex)) { }
	ParticleMaterial::ParticleMaterial(Texture3D* tex, BlendStyle bs) : Material(1, bs, false), imp(new Imp(tex)) { }

	void ParticleMaterial::InnerDispose()         { if(imp) { delete imp; imp = NULL; } }

	Texture* ParticleMaterial::GetTexture() const { return imp->tex;   }
	bool ParticleMaterial::IsTexture3D()    const { return imp->is_3d; }

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

	bool ParticleMaterial::Equals(const Material* other) const
	{
		if(other != NULL && other->mclass_id == mclass_id)
		{
			Imp* other_imp = ((ParticleMaterial*)other)->imp;
			return imp->is_3d == other_imp->is_3d && imp->tex == other_imp->tex;
		}
		else
			return false;
	}

	void ParticleMaterial::BeginDraw(SceneRenderer* renderer) { imp->node_data.clear(); imp->camera = renderer->camera; }

	void ParticleMaterial::EndDraw()
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

		imp->EndDraw();

		if(IsTexture3D())
			glDisable(GL_TEXTURE_3D);

		glMatrixMode(GL_TEXTURE);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);

		glDepthMask(true);

		GLDEBUG();

		imp->camera = NULL;
	}
	void ParticleMaterial::Draw(const RenderNode& node)    { imp->node_data.push_back((ParticleMaterialNodeData*)node.data); }

	void ParticleMaterial::Cleanup(const RenderNode& node) { delete (ParticleMaterialNodeData*)node.data; }




	/*
	 * ParticleMaterialNodeData methods
	 */
	ParticleMaterialNodeData::ParticleMaterialNodeData(const Vec3& pos, float radius, float angle) : radius(radius), pos(pos), angle(angle), red(1), green(1), blue(1), alpha(1), third_coord(0) { }

	
	void ParticleMaterialNodeData::PutColor(float*& color_ptr)
	{
		*(color_ptr++) = red;
		*(color_ptr++) = green;
		*(color_ptr++) = blue;
		*(color_ptr++) = alpha;
	}

	void ParticleMaterialNodeData::PutUVW(float*& uvw_ptr, float u, float v, float w)
	{
		*(uvw_ptr++) = u;
		*(uvw_ptr++) = v;
		*(uvw_ptr++) = w;
	}

	void ParticleMaterialNodeData::PutVertex(float*& vert_ptr, const Vec3& vert)
	{
		*(vert_ptr++) = vert.x;
		*(vert_ptr++) = vert.y;
		*(vert_ptr++) = vert.z;
	}

	void ParticleMaterialNodeData::PutQuad(float*& vert_ptr, float*& uvw_ptr, float*& color_ptr, const Vec3& camera_up, const Vec3& camera_right)
	{
		Vec3 cam_right = camera_right * radius;
		Vec3 cam_up    = camera_up    * radius;
		float cos_val  = cosf(angle);
		float sin_val  = sinf(angle);
		Vec3 use_right = cam_right *  cos_val + cam_up * sin_val;
		Vec3 use_up    = cam_right * -sin_val + cam_up * cos_val;

		PutColor (color_ptr);
		PutUVW   (uvw_ptr,  0, 0, third_coord);
		PutVertex(vert_ptr, pos - use_right - use_up);
	
		PutColor (color_ptr);
		PutUVW   (uvw_ptr,  0, 1, third_coord);
		PutVertex(vert_ptr, pos - use_right + use_up);

		PutColor (color_ptr);
		PutUVW   (uvw_ptr,  1, 1, third_coord);
		PutVertex(vert_ptr, pos + use_right + use_up);
		
		PutColor (color_ptr);
		PutUVW   (uvw_ptr,  1, 0, third_coord);
		PutVertex(vert_ptr, pos + use_right - use_up);
	}
}
