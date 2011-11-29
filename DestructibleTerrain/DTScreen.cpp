#include "StdAfx.h"

#include "DTScreen.h"
#include "VoxelTerrain.h"
#include "TerrainNode.h"
#include "TerrainChunk.h"
#include "VoxelMaterial.h"

#define TERRAIN_RESOLUTION 16

namespace DestructibleTerrain
{
	/*
	 * DTScreen private implementation struct
	 */
	struct DTScreen::Imp
	{
		ProgramWindow* window;

		VoxelMaterial material;
		VoxelTerrain terrain;

		Vec3 camera_pos;
		Vec3 camera_vel;
		float yaw, pitch;
		Quaternion camera_ori;

		Imp(ProgramWindow* window) : window(window), material(window->content), terrain(&material, TERRAIN_RESOLUTION, TERRAIN_RESOLUTION, TERRAIN_RESOLUTION), camera_pos(0, TERRAIN_RESOLUTION * TerrainChunk::ChunkSize * 0.125f, 0), yaw(), pitch(), camera_ori(Quaternion::Identity()), mouse_button_handler(this), mouse_motion_handler(&yaw, &pitch) { }

		void Draw(int width, int height)
		{
			glViewport(0, 0, width, height);

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			camera_ori = Quaternion::FromPYR(0, -yaw, 0) * Quaternion::FromPYR(pitch, 0, 0) * Quaternion::FromPYR(0, M_PI, 0);
			
			Mat3 rm = camera_ori.ToMat3();
			Vec3 forward(Vec3::Normalize(rm * Vec3(0, 0, -1)));
			Vec3 up = Vec3::Normalize(Vec3::Cross(Vec3::Cross(forward, Vec3(0, 1, 0)), forward));

			CameraView camera(camera_pos, forward, up, 3.0f, (float)width / (float)height);

			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera.GetProjectionMatrix().Transpose().values);			
			
			glMatrixMode(GL_MODELVIEW);		
			glLoadMatrixf(camera.GetViewMatrix().Transpose().values);

			float light_pos[] = {0, 1, 0, 0};
			glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

			SceneRenderer renderer(&camera);
			
			terrain.Vis(&renderer);

			renderer.Render();
			renderer.Cleanup();
		}

		void Update(TimingInfo time)
		{
			if(window->input_state->mb[0])
				Explode();

			Mat3 camera_rm = camera_ori.ToMat3();

			if(window->input_state->keys['W'])
				camera_vel += camera_rm * Vec3(0, 0, -50.0f * time.elapsed);
			if(window->input_state->keys['S'])
				camera_vel += camera_rm * Vec3(0, 0, 50.0f * time.elapsed);
			if(window->input_state->keys['A'])
				camera_vel += camera_rm * Vec3(-50.0f * time.elapsed, 0, 0);
			if(window->input_state->keys['D'])
				camera_vel += camera_rm * Vec3(50.0f * time.elapsed, 0, 0);
			if(window->input_state->keys[VK_SPACE])
				camera_vel += Vec3(0, 50 * time.elapsed, 0);
			if(window->input_state->keys['C'])
				camera_vel += Vec3(0, -50 * time.elapsed, 0);


			camera_vel *= exp(-10.0f * time.elapsed);
			camera_pos += camera_vel * time.elapsed;
		}

		void Explode()
		{
			Vec3 pos = Mat4::Invert(terrain.GetTransform()).TransformVec3(camera_pos, 1);
			Vec3 forward = camera_ori.ToMat3() * Vec3(0, 0, -0.5f);

			float center_xyz = TERRAIN_RESOLUTION * TerrainChunk::ChunkSize * 0.5f;
			Vec3 center = Vec3(center_xyz, center_xyz, center_xyz);

			float radius_squared = center_xyz * center_xyz * 3.0f;

			int steps = 0;
			while(steps < 1000)
			{
				int x, y, z;
				TerrainChunk* chunk;
				if(terrain.PosToNode(pos, chunk, x, y, z))
				{
					if(chunk->GetNode(x, y, z)->IsSolid())
					{
						terrain.Explode(pos, 2, 3);
						break;
					}
				}
				else
				{
					Vec3 offset = pos - center;
					if(Vec3::Dot(offset, forward) > 0 && offset.ComputeMagnitudeSquared() > radius_squared)
						break;
				}

				pos += forward;
				steps++;				
			}
		}



		struct MouseButtonHandler : public EventHandler
		{
			Imp* imp;
			MouseButtonHandler(Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				/*
				MouseButtonStateEvent* mbse = (MouseButtonStateEvent*)evt;

				if(mbse->state)
					if(mbse->button == 0)
						imp->Explode();
				*/
			}
		} mouse_button_handler;

		struct MouseMotionHandler : public EventHandler
		{
			float* yaw;
			float* pitch;
			MouseMotionHandler(float* yaw, float* pitch) : yaw(yaw), pitch(pitch) { }

			void HandleEvent(Event* evt)
			{
				MouseMotionEvent* mme = (MouseMotionEvent*)evt;

				*yaw += mme->dx * 0.001f;
				*pitch = max(-1.5f, min(1.5f, *pitch + mme->dy * 0.001f));
			}
		} mouse_motion_handler;
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
			imp = new Imp(window);

		window->input_state->MouseButtonStateChanged += &imp->mouse_button_handler;
		window->input_state->MouseMoved += &imp->mouse_motion_handler;
	}

	void DTScreen::Deactivate()
	{
		window->input_state->MouseButtonStateChanged -= &imp->mouse_button_handler;
		window->input_state->MouseMoved -= &imp->mouse_motion_handler;
	}

	ProgramScreen* DTScreen::Update(TimingInfo time)
	{
		if(input_state->keys[VK_ESCAPE])
			return NULL;
		else
		{
			imp->Update(time);
			return this;
		}
	}

	void DTScreen::Draw(int width, int height) { imp->Draw(width, height); }
}
