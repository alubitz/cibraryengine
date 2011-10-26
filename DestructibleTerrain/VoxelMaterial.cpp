#include "StdAfx.h"

#include "VoxelMaterial.h"

namespace DestructibleTerrain
{
	/*
	 * VoxelMaterialNodeData methods
	 */
	VoxelMaterialNodeData::VoxelMaterialNodeData(VertexBuffer* model, Mat4 xform) :
		model(model),
		xform(xform)
	{
	}

	void VoxelMaterialNodeData::Draw()
	{
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
		glMultMatrixf(xform.Transpose().values);

		model->Draw();

		glPopMatrix();
	}




	/*
	 * VoxelMaterial methods
	 */
	VoxelMaterial::VoxelMaterial() : Material(4, Opaque, false) { }

	void VoxelMaterial::BeginDraw(SceneRenderer* renderer)
	{ 
		GLDEBUG();

		glEnable(GL_CULL_FACE);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(true);
		glDisable(GL_BLEND);

		glDisable(GL_TEXTURE_2D);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		float light_pos[] = { -1, 1, 1, 0 };
		glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

		glPopMatrix();

		glEnable(GL_RESCALE_NORMAL);

		glColor4f(1.0, 1.0, 1.0, 1.0);

		ShaderProgram::SetActiveProgram(NULL);

		GLDEBUG();
	}
	void VoxelMaterial::Draw(RenderNode node) { ((VoxelMaterialNodeData*)node.data)->Draw(); }
	void VoxelMaterial::EndDraw() { GLDEBUG(); }

	void VoxelMaterial::Cleanup(RenderNode node) { delete (VoxelMaterialNodeData*)node.data; }

	bool VoxelMaterial::Equals(Material* material) { return mclass_id == material->mclass_id; }
}
