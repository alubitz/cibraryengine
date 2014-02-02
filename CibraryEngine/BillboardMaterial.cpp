#include "StdAfx.h"
#include "BillboardMaterial.h"

#include "CameraView.h"
#include "RenderNode.h"
#include "SceneRenderer.h"

#include "VertexBuffer.h"

namespace CibraryEngine
{
	/*
	 * BillboardMaterial methods
	 */
	BillboardMaterial::BillboardMaterial(Texture2D* texture, BlendStyle mode) : Material(5, mode, false), texture(texture), nodes_vbo(NULL), recycle_bin() { texture->clamp = true; }

	void BillboardMaterial::InnerDispose()
	{
		Material::InnerDispose();

		for(vector<NodeData*>::iterator iter = recycle_bin.begin(); iter != recycle_bin.end(); ++iter)
			delete *iter;
		recycle_bin.clear();
	}

	void BillboardMaterial::BeginDraw(SceneRenderer* renderer)
	{
		camera_position = renderer->camera->GetPosition();

		node_data.clear();
	}

	void BillboardMaterial::EndDraw()
	{
		if(nodes_vbo == NULL)
		{
			nodes_vbo = new VertexBuffer(Quads);
			nodes_vbo->AddAttribute("gl_Vertex", Float, 3);
			nodes_vbo->AddAttribute("gl_MultiTexCoord0", Float, 2);
			nodes_vbo->AddAttribute("gl_Color", Float, 4);
		}

		nodes_vbo->SetNumVerts(0);							// so we don't bother copying old verts that'll just end up getting overwritten later
		nodes_vbo->SetNumVerts(8 * node_data.size());

		float* vert_ptr  = nodes_vbo->GetFloatPointer("gl_Vertex");
		float* uv_ptr    = nodes_vbo->GetFloatPointer("gl_MultiTexCoord0");
		float* color_ptr = nodes_vbo->GetFloatPointer("gl_Color");

		for(NodeData **iter = node_data.data(), **nodes_end = iter + node_data.size(); iter != nodes_end; ++iter)
			(**iter).PutQuad(vert_ptr, uv_ptr, color_ptr, camera_position);

		GLDEBUG();

		switch(blend_style)
		{
			case Additive: { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE);                 break; }
			case Alpha:    { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); break; }
			default:       { glDisable(GL_BLEND); break; }
		}

		glDisable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glDisable(GL_LIGHTING);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texture->GetGLName());

		glDepthMask(false);

		GLDEBUG();

		nodes_vbo->Draw();

		nodes_vbo->Dispose();
		delete nodes_vbo;
		nodes_vbo = NULL;

		node_data.clear();
	}

	void BillboardMaterial::Draw(const RenderNode& node) { node_data.push_back((NodeData*)node.data); }

	void BillboardMaterial::Cleanup(const RenderNode& node)
	{
		NodeData* nd = (NodeData*)node.data;

		nd->~NodeData();
		recycle_bin.push_back(nd);
	}

	BillboardMaterial::NodeData* BillboardMaterial::NewNodeData(const Vec3& front, const Vec3& back, float width)
	{
		vector<NodeData*>::reverse_iterator found = recycle_bin.rbegin();
		if(found != recycle_bin.rend())
		{
			NodeData* nub = *found;
			recycle_bin.pop_back();
			return new (nub) NodeData(front, back, width);
		}
		else
			return new NodeData(front, back, width);
	}

	bool BillboardMaterial::Equals(const Material* other) const
	{ 
		if(other->mclass_id != mclass_id)
			return false;
		
		BillboardMaterial* bother = (BillboardMaterial*)other;
		return bother->texture->GetGLName() == texture->GetGLName() && bother->blend_style == blend_style;
	}




	/*
	 * BillboardMaterial::NodeData methods
	 */
	BillboardMaterial::NodeData::NodeData(const Vec3& front, const Vec3& back, float width) :
		front(front),
		back(back),
		width(width),
		red(1.0f),
		green(1.0f),
		blue(1.0f),
		alpha(1.0f),
		front_u(0.0f),
		back_u(1.0f)
	{
	}

	void BillboardMaterial::NodeData::PutUV(float*& uv_ptr, float u, float v)
	{
		*(uv_ptr++) = u;
		*(uv_ptr++) = v;
	}

	void BillboardMaterial::NodeData::PutColor(float*& color_ptr, const Vec4& color)
	{
		*(color_ptr++) = color.x;
		*(color_ptr++) = color.y;
		*(color_ptr++) = color.z;
		*(color_ptr++) = color.w;
	}

	void BillboardMaterial::NodeData::PutVertex(float*& vert_ptr, const Vec3& xyz)
	{
		*(vert_ptr++) = xyz.x;
		*(vert_ptr++) = xyz.y;
		*(vert_ptr++) = xyz.z;
	}

	void BillboardMaterial::NodeData::PutQuad(float*& vert_ptr, float*& uv_ptr, float*& color_ptr, const Vec3& camera_position)
	{
		Vec4 color(red, green, blue, alpha);

		Vec3 normal, pos;
		pos = front - back;
		normal = Vec3::Normalize(front - camera_position);            // temporarily uses the same variable to store it
		normal = Vec3::Cross(normal, pos);
		normal *= width * 0.5f / normal.ComputeMagnitude();

		Vec3 points[6] = { front + normal, front, front - normal, back + normal, back, back - normal };

		PutUV(uv_ptr, front_u, 0.0f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[0]);
		PutUV(uv_ptr, front_u, 0.5f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[1]);
		PutUV(uv_ptr, back_u,  0.5f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[4]);
		PutUV(uv_ptr, back_u,  0.0f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[3]);

		PutUV(uv_ptr, front_u, 0.5f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[1]);
		PutUV(uv_ptr, front_u, 1.0f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[2]);
		PutUV(uv_ptr, back_u,  1.0f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[5]);
		PutUV(uv_ptr, back_u,  0.5f); PutColor(color_ptr, color); PutVertex(vert_ptr, points[4]);
	}
}
