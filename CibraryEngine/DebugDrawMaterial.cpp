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

		unsigned int num_verts;

		Imp() : vbo(NULL), renderer(NULL) { }

		~Imp() { KillVBO(); }

		void KillVBO() { if(vbo) { vbo->Dispose(); delete vbo; vbo = NULL; } }

		void BeginDraw(SceneRenderer* renderer)
		{
			assert(vbo == NULL);

			this->renderer = renderer;

			vbo = new VertexBuffer(Lines);
			vbo->SetSizeIncrement(4000);

			num_verts = 0;

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

		void Draw(const RenderNode& node)
		{
			vbo->SetNumVerts(num_verts + 2);

			float* vert_ptr = &vbo->GetFloatPointer("gl_Vertex")[num_verts * 3];
			float* color_ptr = &vbo->GetFloatPointer("gl_Color")[num_verts * 3];

			DebugDrawMaterialNodeData& data = *(DebugDrawMaterialNodeData*)node.data;

			// set vertex positions
			*(vert_ptr++)	= data.p1.x;
			*(vert_ptr++)	= data.p1.y;
			*(vert_ptr++)	= data.p1.z;

			*(vert_ptr++)	= data.p2.x;
			*(vert_ptr++)	= data.p2.y;
			*vert_ptr		= data.p2.z;

			// set vertex colors
			*(color_ptr++)	= data.color.x;
			*(color_ptr++)	= data.color.y;
			*(color_ptr++)	= data.color.z;

			*(color_ptr++)	= data.color.x;
			*(color_ptr++)	= data.color.y;
			*color_ptr		= data.color.z;

			num_verts += 2;
		}
	};




	/*
	 * DebugDrawMaterial methods
	 */
	DebugDrawMaterial::DebugDrawMaterial() : Material(6, Opaque, false), imp(new Imp()) { }

	void DebugDrawMaterial::InnerDispose()						{ if(imp) { delete imp; imp = NULL; } }

	void DebugDrawMaterial::BeginDraw(SceneRenderer* renderer)	{ imp->BeginDraw(renderer); }
	void DebugDrawMaterial::EndDraw()							{ imp->EndDraw(); }
	void DebugDrawMaterial::Draw(const RenderNode& node)		{ imp->Draw(node); }

	bool DebugDrawMaterial::Equals(const Material* other) const	{ return other->mclass_id == mclass_id; }

	DebugDrawMaterial* DebugDrawMaterial::GetDebugDrawMaterial()
	{
		static DebugDrawMaterial* mat = new DebugDrawMaterial();
		return mat;
	}


	/*
	 * DebugDrawMaterial dynamic allocation recycle bin stuffs
	 */
	static vector<DebugDrawMaterialNodeData*> ddmnd_recycle_bin;

	DebugDrawMaterialNodeData* DebugDrawMaterial::New(const Vec3& p1, const Vec3& p2)
	{
		if(ddmnd_recycle_bin.empty())
			return new DebugDrawMaterialNodeData(p1, p2);
		else
		{
			DebugDrawMaterialNodeData* result = *ddmnd_recycle_bin.rbegin();
			ddmnd_recycle_bin.pop_back();
			return new (result) DebugDrawMaterialNodeData(p1, p2);
		}
	}

	DebugDrawMaterialNodeData* DebugDrawMaterial::New(const Vec3& p1, const Vec3& p2, const Vec3& color)
	{
		if(ddmnd_recycle_bin.empty())
			return new DebugDrawMaterialNodeData(p1, p2, color);
		else
		{
			DebugDrawMaterialNodeData* result = *ddmnd_recycle_bin.rbegin();
			ddmnd_recycle_bin.pop_back();
			return new (result) DebugDrawMaterialNodeData(p1, p2, color);
		}
	}

	void DebugDrawMaterial::EmptyRecycleBin()
	{
		for(vector<DebugDrawMaterialNodeData*>::iterator iter = ddmnd_recycle_bin.begin(); iter != ddmnd_recycle_bin.end(); ++iter)
			delete *iter;
		ddmnd_recycle_bin.clear();
	}


	void DebugDrawMaterial::Cleanup(const RenderNode& node)		{ ddmnd_recycle_bin.push_back((DebugDrawMaterialNodeData*)node.data); }
}
