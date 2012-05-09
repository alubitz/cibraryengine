#include "StdAfx.h"

#include "DTScreen.h"
#include "VoxelTerrain.h"
#include "TerrainNode.h"
#include "TerrainChunk.h"
#include "VoxelMaterial.h"

#define TERRAIN_RESOLUTION 16

namespace DestructibleTerrain
{
	struct SphereSmoother : public TerrainAction
	{
		void AffectNode(TerrainChunk* chunk, TerrainNode& node, int x, int y, int z, unsigned char amount)
		{
			if(amount < 128)
				return;

			unsigned int total_solidity = node.solidity * 255;
			unsigned int total_weight = 255;

			unordered_map<unsigned char, unsigned int> total_materials;

			// copy this node's materials
			for(unsigned char i = 0; i < 4; ++i)
				if(unsigned char material = node.material.types[i])
					if(unsigned char mat_weight = node.material.weights[i])
					{
						unordered_map<unsigned char, unsigned int>::iterator found = total_materials.find(material);

						if(found == total_materials.end())
							total_materials[material] = mat_weight;
						else
							total_materials[material] += mat_weight;
					}

			for(int xx = x - 1; xx <= x + 1; ++xx)
				for(int yy = y - 1; yy <= y + 1; ++yy)
					for(int zz = z - 1; zz <= z + 1; ++zz)
						if(xx != x || yy != y || zz != z)			
							if(TerrainNode* neighbor_node = chunk->GetNodeRelative(xx, yy, zz))
							{
								int weight = 255;

								total_weight += weight;
								total_solidity += int(neighbor_node->solidity) * weight;

								if(neighbor_node->solidity >= 128)
								{
									// add neighboring node's materials
									for(int i = 0; i < 4; ++i)
										if(unsigned char material = neighbor_node->material.types[i])
											if(unsigned char mat_weight = neighbor_node->material.weights[i])
											{
												unordered_map<unsigned char, unsigned int>::iterator found = total_materials.find(material);

												if(found == total_materials.end())
													total_materials[material] = mat_weight;
												else
													total_materials[material] += mat_weight;
											}
								}
							}

			bool changed = false;					// only invalidate the node if it is changed by one (or more) of these two blocks of code...

			unsigned char nu_value = (unsigned char)floor(max(0.0f, min(255.0f, float(total_solidity) / total_weight + 0.5f)));
			if(nu_value != node.solidity)
			{
				node.solidity = nu_value;
				changed = true;
			}

			// if there are somehow no materials in or around this node, don't bother with these steps
			if(unsigned int count = min(4u, (unsigned int)total_materials.size()))
			{
				unsigned char mat_types[4];
				unsigned int mat_weights[4];
				for(unsigned char i = 0; i < count; ++i)
				{
					unordered_map<unsigned char, unsigned int>::iterator iter = total_materials.begin();
					unordered_map<unsigned char, unsigned int>::iterator best = iter++;

					for(; iter != total_materials.end(); ++iter)
						if(iter->second > best->second)
							best = iter;

					mat_types[i] = best->first;
					mat_weights[i] = best->second;

					total_materials.erase(best);
				}

				for(unsigned char i = 0; i < 4; ++i)
				{
					if(i < count)
					{
						node.material.types[i] = mat_types[i];
						node.material.weights[i] = mat_weights[i] * 255 / mat_weights[0];
					}
					else
						node.material.weights[i] = 0;
				}

				// TODO: make a MultiMaterial::operator== or something comparable, and only set this if it's not equal
				changed = true;
			}

			if(changed)
				chunk->InvalidateNode(x, y, z);
		};
	};

	struct SphereMaterialSetter : public TerrainAction
	{
		unsigned char material;
		SphereMaterialSetter(unsigned char material) : material(material) { }

		void AffectNode(TerrainChunk* chunk, TerrainNode& node, int x, int y, int z, unsigned char amount)
		{
			if(material == 0)						// subtract mode
			{	
				amount = 255 - amount;

				if(amount < node.solidity)
				{
					node.solidity = amount;
					chunk->InvalidateNode(x, y, z);
				}
			}
			else									// add/change material mode
			{
				bool changed = false;				// only invalidate the node if it is changed by one (or more) of these two blocks of code...

				if(amount > node.solidity)
				{
					node.solidity = amount;			// should this affect the node's material somehow?
					changed = true;
				}

				if(amount >= 128 && node.IsSolid())
				{
					MultiMaterial mat;
					mat.types[0] = material;
					mat.weights[0] = 255;
					node.material = mat;

					changed = true;
				}

				if(changed)
					chunk->InvalidateNode(x, y, z);
			}
		}
	};




	/*
	 * DTScreen private implementation struct
	 */
	struct DTScreen::Imp
	{
		struct EditorBrush
		{
		public:

			string name;

			EditorBrush(string name) : name(name) { }			
			virtual void DoAction(Imp* imp) = 0;
		};

		ProgramWindow* window;

		SceneRenderer renderer;

		VoxelMaterial* material;
		VoxelTerrain* terrain;

		VertexBuffer* editor_orb;

		BitmapFont* font;

		Vec3 camera_pos;
		Vec3 camera_vel;
		float yaw, pitch;
		Quaternion camera_ori;

		EditorBrush* current_brush;
		float brush_distance;
		float brush_radius;

		Imp(ProgramWindow* window) :
			window(window),
			renderer(NULL),
			material(NULL), 
			terrain(NULL), 
			font(NULL),
			camera_pos(0, TERRAIN_RESOLUTION * TerrainChunk::ChunkSize * 0.125f, 0), 
			yaw(), 
			pitch(), 
			camera_ori(Quaternion::Identity()), 
			current_brush(NULL), 
			brush_distance(40.0f),
			brush_radius(5.0f),
			mouse_button_handler(this), 
			mouse_motion_handler(&yaw, &pitch),
			subtract_brush(),
			stone_brush(),
			sand_brush(),
			smooth_brush()
		{
			current_brush = &subtract_brush;
			font = window->content->GetCache<BitmapFont>()->Load("../Font");

			editor_orb = window->content->GetCache<VertexBuffer>()->Load("terrain_edit_orb");
		}

		~Imp()
		{
			if(material != NULL) { delete material; material = NULL; }
			if(terrain != NULL) { delete terrain; terrain = NULL; }
		}

		void MakeTerrainAsNeeded()
		{
			if(material == NULL)
				material = new VoxelMaterial(window->content);

			if(terrain == NULL)
			{
				if(unsigned int terrain_load_error = VoxelTerrainLoader::LoadVVV(terrain, material, "Files/Levels/VoxelWorld.vvv"))
				{
					stringstream load_err_ss;
					load_err_ss << "LoadVVV returned with status " << terrain_load_error << "! Generating random terrain instead..." << endl;
					Debug(load_err_ss.str());

					terrain = VoxelTerrainLoader::GenerateTerrain(material, TERRAIN_RESOLUTION, TERRAIN_RESOLUTION, TERRAIN_RESOLUTION);

					if(unsigned int terrain_save_error = VoxelTerrainLoader::SaveVVV(terrain, "Files/Levels/VoxelWorld.vvv"))
					{
						stringstream save_err_ss;
						save_err_ss << "SaveVVV returned with status " << terrain_save_error << "!" << endl;
						Debug(save_err_ss.str());
					}
				}
			}
		}

		void Draw(int width, int height)
		{
			GLDEBUG();

			MakeTerrainAsNeeded();

			glViewport(0, 0, width, height);

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			camera_ori = Quaternion::FromPYR(0, -yaw, 0) * Quaternion::FromPYR(pitch, 0, 0) * Quaternion::FromPYR(0, float(M_PI), 0);
			
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

			GLDEBUG();

			renderer.camera = &camera;
			
			terrain->Vis(&renderer);

			renderer.Render();
			renderer.Cleanup();

			GLDEBUG();

			if(current_brush != NULL)
			{
				Vec3 ori, dir;
				GetCameraRay(ori, dir);

				Vec3 orb_pos = ori + dir * brush_distance;

				glPushMatrix();

				glMultMatrixf(terrain->GetTransform().Transpose().values);
				glTranslatef(orb_pos.x, orb_pos.y, orb_pos.z);
				glScalef(brush_radius, brush_radius, brush_radius);

				ShaderProgram::SetActiveProgram(NULL);
				
				glEnable(GL_LIGHTING);
				glDisable(GL_TEXTURE_2D);
				glDisable(GL_CULL_FACE);

				glDepthMask(false);

				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE);
				
				glColor4f(1.0f, 1.0f, 1.0f, 0.25f);

				editor_orb->Draw();

				glPopMatrix();
			}

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, width, height, 0, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

			if(current_brush != NULL)
			{
				stringstream ss;
				ss << current_brush->name << " - radius " << brush_radius;
				
				font->Print(ss.str(), 0, 0);
			}

			GLDEBUG();
		}

		void Update(TimingInfo time)
		{
			if(window->input_state->mb[0] && current_brush != NULL)
				current_brush->DoAction(this);

			Mat3 camera_rm = camera_ori.ToMat3();

			float dv = 50.0f * time.elapsed;

			if(window->input_state->keys['W'])
				camera_vel += camera_rm * Vec3(	0,		0,		-dv);
			if(window->input_state->keys['S'])
				camera_vel += camera_rm * Vec3(	0,		0,		dv);
			if(window->input_state->keys['A'])
				camera_vel += camera_rm * Vec3(	-dv,	0,		0);
			if(window->input_state->keys['D'])
				camera_vel += camera_rm * Vec3(	dv,		0,		0);
			if(window->input_state->keys[VK_SPACE])
				camera_vel += camera_rm * Vec3(	0,		dv,		0);
			if(window->input_state->keys['C'])
				camera_vel += camera_rm * Vec3(	0,		-dv,	0);


			camera_vel *= exp(-10.0f * time.elapsed);
			camera_pos += camera_vel * time.elapsed;
		}


		

		void ApplyBrush(TerrainAction& action)
		{
			Vec3 origin, direction;
			GetCameraRay(origin, direction);

			Vec3 pos = origin + direction * brush_distance;
			terrain->ModifySphere(pos, brush_radius - 1.0f, brush_radius + 1.0f, action);
		}

		void GetCameraRay(Vec3& origin, Vec3& direction)
		{ 
			origin = Mat4::Invert(terrain->GetTransform()).TransformVec3(camera_pos, 1); 
			direction = camera_ori.ToMat3() * Vec3(0, 0, -1); 
		}

		bool RayTrace(Vec3 origin, Vec3 direction, Vec3& result)
		{
			Vec3 pos = origin;
			direction = Vec3::Normalize(direction, 0.5f);

			float center_xyz = TERRAIN_RESOLUTION * TerrainChunk::ChunkSize * 0.5f;
			Vec3 center = Vec3(center_xyz, center_xyz, center_xyz);

			float radius_squared = center_xyz * center_xyz * 3.0f;

			int steps = 0;
			while(steps < 1000)
			{
				int x, y, z;
				TerrainChunk* chunk;
				if(terrain->PosToNode(pos, chunk, x, y, z))
				{
					if(chunk->GetNode(x, y, z)->IsSolid())
					{
						result = pos;
						return true;
					}
				}
				else
				{
					Vec3 offset = pos - center;
					if(Vec3::Dot(offset, direction) > 0 && offset.ComputeMagnitudeSquared() > radius_squared)
						break;
				}

				pos += direction;
				steps++;				
			}

			return false;
		}




		struct MouseButtonHandler : public EventHandler
		{
			Imp* imp;
			MouseButtonHandler(Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				MouseButtonStateEvent* mbse = (MouseButtonStateEvent*)evt;
				if(mbse->button == 2 && mbse->state)
				{
					if(imp->current_brush == &imp->subtract_brush)
						imp->current_brush = &imp->stone_brush;
					else if(imp->current_brush == &imp->stone_brush)
						imp->current_brush = &imp->sand_brush;
					else if(imp->current_brush == &imp->sand_brush)
						imp->current_brush = &imp->smooth_brush;
					else
						imp->current_brush = &imp->subtract_brush;
				}
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

				const float rotation_coeff = 0.001f;

				*yaw += mme->dx * rotation_coeff;
				*pitch = max(-1.5f, min(1.5f, *pitch + mme->dy * rotation_coeff));
			}
		} mouse_motion_handler;




		struct SubtractBrush : public EditorBrush
		{
			SubtractBrush() : EditorBrush("Subtract") { }
			void DoAction(Imp* imp) { imp->ApplyBrush(SphereMaterialSetter(0)); }
		} subtract_brush;

		struct StoneBrush : public EditorBrush
		{
			StoneBrush() : EditorBrush("Add Stone") { }
			void DoAction(Imp* imp) { imp->ApplyBrush(SphereMaterialSetter(1)); }
		} stone_brush;

		struct SandBrush : public EditorBrush
		{
			SandBrush() : EditorBrush("Add Sand") { }
			void DoAction(Imp* imp) { imp->ApplyBrush(SphereMaterialSetter(2)); }
		} sand_brush;

		struct SmoothBrush : public EditorBrush
		{
			SmoothBrush() : EditorBrush("Smooth") { }
			void DoAction(Imp* imp) { imp->ApplyBrush(SphereSmoother()); }
		} smooth_brush;
	};




	/*
	 * DTScreen methods
	 */
	DTScreen::DTScreen(ProgramWindow* win) :
		ProgramScreen(win),
		imp(NULL)
	{
	}

	DTScreen::~DTScreen() { delete imp; imp = NULL; }

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
