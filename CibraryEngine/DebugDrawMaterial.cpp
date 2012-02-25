#include "StdAfx.h"
#include "DebugDrawMaterial.h"

#include "VertexBuffer.h"
#include "SceneRenderer.h"
#include "RenderNode.h"
#include "Shader.h"

namespace CibraryEngine
{
	/*
	 * DebugDrawMaterial private implementation struct
	 */
	struct DebugDrawMaterial::Imp
	{
		VertexBuffer* vbo;
		SceneRenderer* renderer;

		Imp() : vbo(NULL), renderer(NULL) { }

		~Imp() { KillVBO(); }

		void KillVBO()
		{
			if(vbo != NULL)
			{
				vbo->Dispose();
				delete vbo;
				vbo = NULL;
			}
		}

		void BeginDraw(SceneRenderer* renderer)
		{
			assert(vbo == NULL);

			this->renderer = renderer;

			vbo = new VertexBuffer(Lines);

			vbo->AddAttribute("gl_Vertex", Float, 3);
			vbo->AddAttribute("gl_Color", Float, 3);
		}

		void EndDraw()
		{
			ShaderProgram::SetActiveProgram(NULL);

			glDisable(GL_BLEND);
			glDisable(GL_CULL_FACE);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LEQUAL);
			glDisable(GL_LIGHTING);
			glDisable(GL_TEXTURE_2D);

			glDisable(GL_LINE_SMOOTH);
			glLineWidth(1.0f);

			glDepthMask(true);

			vbo->Draw();

			KillVBO();
			renderer = NULL;
		}

		void Draw(RenderNode node)
		{
			unsigned int cur_vert = vbo->GetNumVerts();
			vbo->SetNumVerts(cur_vert + 2);

			float* vert_ptr = &vbo->GetFloatPointer("gl_Vertex")[cur_vert * 3];
			float* color_ptr = &vbo->GetFloatPointer("gl_Color")[cur_vert * 3];

			DebugDrawMaterialNodeData& data = *(DebugDrawMaterialNodeData*)node.data;

			// set vertex positions
			*(vert_ptr++) = data.p1.x;
			*(vert_ptr++) = data.p1.y;
			*(vert_ptr++) = data.p1.z;

			*(vert_ptr++) = data.p2.x;
			*(vert_ptr++) = data.p2.y;
			*(vert_ptr++) = data.p2.z;

			// set vertex colors
			*(color_ptr++) = data.color.x;
			*(color_ptr++) = data.color.y;
			*(color_ptr++) = data.color.z;

			*(color_ptr++) = data.color.x;
			*(color_ptr++) = data.color.y;
			*(color_ptr++) = data.color.z;
		}

		void Cleanup(RenderNode node) { delete (DebugDrawMaterialNodeData*)node.data; }
	};




	/*
	 * DebugDrawMaterial methods
	 */
	DebugDrawMaterial::DebugDrawMaterial() : Material(6, Opaque, false), imp(new Imp()) { }

	void DebugDrawMaterial::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void DebugDrawMaterial::BeginDraw(SceneRenderer* renderer) { imp->BeginDraw(renderer); }
	void DebugDrawMaterial::EndDraw() { imp->EndDraw(); }
	void DebugDrawMaterial::Draw(RenderNode node) { imp->Draw(node); }
	void DebugDrawMaterial::Cleanup(RenderNode node) { imp->Cleanup(node); }

	bool DebugDrawMaterial::Equals(Material* other) { return other->mclass_id == mclass_id; }
}
