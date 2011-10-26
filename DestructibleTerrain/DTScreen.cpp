#include "StdAfx.h"

#include "DTScreen.h"
#include "VoxelTerrain.h"
#include "VoxelMaterial.h"

namespace DestructibleTerrain
{
	/*
	 * DTScreen private implementation struct
	 */
	struct DTScreen::Imp
	{
		VoxelMaterial material;
		VoxelTerrain terrain;

		float rotation;

		Imp() : material(), terrain(&material, 128, 128, 128), rotation() { }

		void Draw(int width, int height)
		{
			glViewport(0, 0, width, height);

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			Mat3 rm(Mat3::FromScaledAxis(0, rotation,0));
			Vec3 forward(rm * Vec3(0, 0, -1));
			CameraView camera(-3 * forward, forward, Vec3(0, 1, 0), 3.0f, (float)width / (float)height);

			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera.GetProjectionMatrix().Transpose().values);			
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(camera.GetViewMatrix().Transpose().values);

			SceneRenderer renderer(&camera);
			
			terrain.Vis(&renderer);

			renderer.Render();
		}
	};




	/*
	 * DTScreen methods
	 */
	DTScreen::DTScreen(ProgramWindow* win) :
		ProgramScreen(win),
		imp(NULL)
	{
	}

	DTScreen::~DTScreen()
	{ 
		delete imp; 
		imp = NULL;
	}

	void DTScreen::Activate()
	{
		if(imp == NULL)
			imp = new Imp();
	}

	ProgramScreen* DTScreen::Update(TimingInfo time)
	{
		if(input_state->keys[VK_ESCAPE])
			return NULL;
		else
		{
			imp->rotation += time.elapsed;

			return this;
		}
	}

	void DTScreen::Draw(int width, int height) { imp->Draw(width, height); }
}
