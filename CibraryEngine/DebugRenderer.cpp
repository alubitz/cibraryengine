#include "StdAfx.h"
#include "DebugRenderer.h"

#include "SceneRenderer.h"
#include "CameraView.h"
#include "RenderNode.h"

namespace CibraryEngine
{
	/*
	 * DebugRenderer methods
	 */
	DebugRenderer::DebugRenderer() : btIDebugDraw() { }

	void DebugRenderer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		glColor4f(color.getX(), color.getY(), color.getZ(), 1);

		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(true);
		glDisable(GL_TEXTURE_2D);
		glDisable(GL_LIGHTING);

		glLineWidth(1.0f);

		glBegin(GL_LINES);
		glVertex3f(from.getX(), from.getY(), from.getZ());
		glVertex3f(to.getX(), to.getY(), to.getZ());
		glEnd();
	}
}
