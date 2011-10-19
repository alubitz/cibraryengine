#include "StdAfx.h"

#include "DTScreen.h"
#include "VoxelTerrain.h"

namespace DestructibleTerrain
{
	/*
	 * DTScreen private implementation struct
	 */
	struct DTScreen::Imp
	{
		VoxelTerrain terrain;

		void Draw(int width, int height)
		{
			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			CameraView camera(Vec3(), Vec3(0, 0, -1), Vec3(0, 1, 0), 3.0f, (float)width / (float)height);

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
		imp(new Imp())
	{
	}

	DTScreen::~DTScreen() { delete imp; imp = NULL; }

	ProgramScreen* DTScreen::Update(TimingInfo time)
	{
		if(input_state->keys[VK_ESCAPE])
			return NULL;
		else
			return this;
	}

	void DTScreen::Draw(int width, int height) { imp->Draw(width, height); }
}
