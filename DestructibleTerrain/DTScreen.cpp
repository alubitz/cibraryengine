#include "StdAfx.h"

#include "DTScreen.h"
#include "VoxelTerrain.h"
#include "TerrainChunk.h"
#include "VoxelMaterial.h"

#define TERRAIN_RESOLUTION 8

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

		Imp(ContentMan* content) : material(content), terrain(&material, TERRAIN_RESOLUTION, TERRAIN_RESOLUTION, TERRAIN_RESOLUTION), rotation(), explode_handler(&terrain) { }

		void Draw(int width, int height)
		{
			glViewport(0, 0, width, height);

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			Mat3 rm(Mat3::FromScaledAxis(0, rotation,0));
			Vec3 forward(Vec3::Normalize(rm * Vec3(0, -1, -1)));
			Vec3 up = Vec3::Normalize(Vec3::Cross(Vec3::Cross(forward, Vec3(0, 1, 0)), forward));
			CameraView camera(-25 * forward, forward, up, 3.0f, (float)width / (float)height);

			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera.GetProjectionMatrix().Transpose().values);			
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(camera.GetViewMatrix().Transpose().values);

			SceneRenderer renderer(&camera);
			
			terrain.Vis(&renderer);

			renderer.Render();
		}

		struct ExplodeHandler : public EventHandler
		{
			VoxelTerrain* terrain;
			ExplodeHandler(VoxelTerrain* terrain) : terrain(terrain) { }

			void HandleEvent(Event* evt)
			{
				MouseButtonStateEvent* mbse = (MouseButtonStateEvent*)evt;

				if(mbse->state)
					if(mbse->button == 0)
						terrain->Explode();
			}
		} explode_handler;
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
			imp = new Imp(content);

		window->input_state->MouseButtonStateChanged += &imp->explode_handler;
	}

	void DTScreen::Deactivate()
	{
		window->input_state->MouseButtonStateChanged -= &imp->explode_handler;
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
