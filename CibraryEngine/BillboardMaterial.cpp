#include "StdAfx.h"
#include "BillboardMaterial.h"

#include "CameraView.h"
#include "RenderNode.h"
#include "SceneRenderer.h"

namespace CibraryEngine
{
	/*
	 * BillboardMaterial methods
	 */
	BillboardMaterial::BillboardMaterial(Texture2D* texture, BlendStyle mode) : Material(5, mode, false), texture(texture) { texture->clamp = true; }

	void BillboardMaterial::BeginDraw(SceneRenderer* renderer)
	{
		GLDEBUG();

		camera_position = renderer->camera->GetPosition();

		switch(blend_style)
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

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texture->GetGLName());

		glDepthMask(false);

		GLDEBUG();
		glBegin(GL_QUADS);
	}

	void BillboardMaterial::EndDraw() { glEnd(); }

	void BillboardMaterial::Draw(RenderNode node) { ((NodeData*)node.data)->Execute(camera_position); }
	void BillboardMaterial::Cleanup(RenderNode node) { delete node.data; }

	bool BillboardMaterial::Equals(Material* other)
	{ 
		if(other->mclass_id != mclass_id)
			return false;
		
		BillboardMaterial* bother = (BillboardMaterial*)other;
		return bother->texture->GetGLName() == texture->GetGLName() && bother->blend_style == blend_style;
	}




	/*
	 * BillboardMaterial::NodeData methods
	 */
	BillboardMaterial::NodeData::NodeData(Vec3 position, Vec3 back, float width) :
		front(position),
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

	void BillboardMaterial::NodeData::Execute(Vec3 camera_position)
	{
		glColor4f(red, green, blue, alpha);

		Vec3 normal, pos, anti_normal;
		pos = front - back;
		normal = Vec3::Normalize(front - camera_position);            // temporarily uses the same variable to store it
		normal = Vec3::Cross(normal, pos);
		normal *= width * 0.5f / normal.ComputeMagnitude();
		anti_normal = -normal;

		Vec3 points[6];
		points[0] = front + normal;
		points[1] = front;
		points[2] = front + anti_normal;
		points[3] = back + normal;
		points[4] = back;
		points[5] = back + anti_normal;

		glTexCoord3f(front_u,	0.0f, 0);
		glVertex3f(points[0].x, points[0].y, points[0].z);
		glTexCoord3f(front_u,	0.5f, 0);
		glVertex3f(points[1].x, points[1].y, points[1].z);
		glTexCoord3f(back_u,	0.5f, 0);
		glVertex3f(points[4].x, points[4].y, points[4].z);
		glTexCoord3f(back_u,	0.0f, 0);
		glVertex3f(points[3].x, points[3].y, points[3].z);

		glTexCoord3f(front_u,	0.5f, 0);
		glVertex3f(points[1].x, points[1].y, points[1].z);
		glTexCoord3f(front_u,	1.0f, 0);
		glVertex3f(points[2].x, points[2].y, points[2].z);
		glTexCoord3f(back_u,	1.0f, 0);
		glVertex3f(points[5].x, points[5].y, points[5].z);
		glTexCoord3f(back_u,	0.5f, 0);
		glVertex3f(points[4].x, points[4].y, points[4].z);
	}
}
