#include "StdAfx.h"

#include "DTScreen.h"
#include "../CibraryEngine/DestructibleTerrain.h"

#define TERRAIN_DIM_HORIZONTAL 16
#define TERRAIN_DIM_VERTICAL 8

namespace DestructibleTerrain
{
	struct SphereSmoother : public TerrainAction
	{
		void AffectNode(TerrainChunk* chunk, TerrainNode& node, int x, int y, int z, unsigned char amount)
		{
			if(amount < 128)
				return;

			unsigned int total_weight = 0;
			unsigned int total_solidity = 0;
			unordered_map<unsigned char, unsigned int> total_materials;

			for(int xx = x - 1; xx <= x + 1; ++xx)
				for(int yy = y - 1; yy <= y + 1; ++yy)
					for(int zz = z - 1; zz <= z + 1; ++zz)
						if(TerrainNode* neighbor_node = chunk->GetNodeRelative(xx, yy, zz))
						{
							int weight = 255;
							total_weight += weight;

							int solidity = neighbor_node->solidity;
							total_solidity += solidity * weight;

							if(solidity >= 128)
							{
								// add neighboring node's materials
								for(int i = 0; i < 4; ++i)
									if(unsigned char material = neighbor_node->material.types[i])
										if(unsigned int mat_weight = neighbor_node->material.weights[i])
										{
											unordered_map<unsigned char, unsigned int>::iterator found = total_materials.find(material);

											if(found == total_materials.end())
												total_materials[material] = mat_weight * (solidity - 128);
											else
												total_materials[material] += mat_weight * (solidity - 128);
										}
							}
						}

			bool changed = false;					// only invalidate the node if it is changed by one (or more) of these two blocks of code...

			unsigned char nu_value = (unsigned char)floor(max(0.0f, min(255.0f, float(total_solidity) / total_weight)));

			if(nu_value != node.solidity)
			{
				node.solidity = nu_value;
				changed = true;
			}

			if(nu_value >= 128)
			{
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
				if(amount >= 128 || amount > node.solidity)
				{
					if(amount > node.solidity)
						node.solidity = amount;

					MultiMaterial mat;
					mat.types[0] = material;
					mat.weights[0] = 255;
					node.material = mat;

					chunk->InvalidateNode(x, y, z);
				}
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
		bool enable_editing;

		float time_since_fps;
		int frames_since_fps;
		int fps;

		boost::unordered_map<unsigned char, TerrainTexture>::iterator add_tex;

		Imp(ProgramWindow* window) :
			window(window),
			renderer(NULL),
			material(NULL),
			terrain(NULL),
			font(NULL),
			camera_pos(0, TERRAIN_DIM_VERTICAL * TerrainChunk::ChunkSize * 0.125f, 0),
			yaw(),
			pitch(),
			camera_ori(Quaternion::Identity()), 
			current_brush(NULL),
			brush_distance(60.0f),
			brush_radius(7.5f),
			enable_editing(true),
			mouse_button_handler(this),
			mouse_motion_handler(&yaw, &pitch),
			key_press_handler(&add_tex, &add_brush, &enable_editing),
			subtract_brush(),
			add_brush(&add_tex),
			time_since_fps(0.0f),
			frames_since_fps(0),
			fps(0),
			smooth_brush()
		{
			current_brush = &subtract_brush;
			font = window->content->GetCache<BitmapFont>()->Load("../Font");

			editor_orb = window->content->GetCache<VertexBuffer>()->Load("terrain_edit_orb");
		}

		~Imp()
		{
			if(material)	{ delete material;	material = NULL; }
			if(terrain)		{ delete terrain;	terrain = NULL; }
		}

		void MakeTerrainAsNeeded()
		{
			if(material == NULL)
			{
				material = new VoxelMaterial(window->content);
				
				add_brush.material = key_press_handler.material = material;

				add_tex = material->textures.begin();
				add_brush.UpdateName();
			}

			if(terrain == NULL)
			{
				if(unsigned int terrain_load_error = VoxelTerrainLoader::LoadVVV(terrain, material, "Files/Levels/VoxelWorld.vvv"))
				{
					stringstream load_err_ss;
					load_err_ss << "LoadVVV returned with status " << terrain_load_error << "! Generating random terrain instead..." << endl;
					Debug(load_err_ss.str());

					terrain = VoxelTerrainLoader::GenerateTerrain(material, TERRAIN_DIM_HORIZONTAL, TERRAIN_DIM_VERTICAL, TERRAIN_DIM_HORIZONTAL);

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

			if(width <= 0 || height <= 0)
				return;

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			camera_ori = Quaternion::FromRVec(0, -yaw, 0) * Quaternion::FromRVec(pitch, 0, 0) * Quaternion::FromRVec(0, float(M_PI), 0);
			
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
			glEnable(GL_LIGHT0);

			GLDEBUG();

			renderer.camera = &camera;
			
			terrain->Vis(&renderer);

			renderer.Render();
			renderer.Cleanup();

			GLDEBUG();

			if(enable_editing && current_brush != NULL)
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

			if(enable_editing && current_brush != NULL)
			{
				font->Print(current_brush->name, 0, 0);
				font->Print(((stringstream&)(stringstream() << "Radius : " << brush_radius)).str(), 0, font->font_height);
			}
			font->Print(((stringstream&)(stringstream() << "FPS : " << fps)).str(), width - font->font_spacing * 10, 0);

			GLDEBUG();

			++frames_since_fps;
		}

		void Update(TimingInfo time)
		{
			time_since_fps += time.elapsed;

			if(frames_since_fps && time_since_fps > 0.1f)
			{
				fps = (int)(frames_since_fps / time_since_fps);

				frames_since_fps = 0;
				time_since_fps = 0.0f;
			}

			if(window->input_state->mb[0] && current_brush != NULL && enable_editing)
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


			camera_vel *= expf(-10.0f * time.elapsed);
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
			origin = Mat4::Invert(terrain->GetTransform()).TransformVec3_1(camera_pos); 
			direction = camera_ori.ToMat3() * Vec3(0, 0, -1); 
		}

		bool RayTrace(Vec3 origin, Vec3 direction, Vec3& result)
		{
			Vec3 pos = origin;
			direction = Vec3::Normalize(direction, 0.5f);

			float center_xz = TERRAIN_DIM_HORIZONTAL * TerrainChunk::ChunkSize * 0.5f;
			Vec3 center = Vec3(center_xz, TERRAIN_DIM_VERTICAL * TerrainChunk::ChunkSize * 0.5f, center_xz);

			float radius_squared = center.ComputeMagnitudeSquared() * 3.0f;

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
						imp->current_brush = &imp->add_brush;
					else if(imp->current_brush == &imp->add_brush)
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

		struct AddBrush;
		struct KeyPressHandler : public EventHandler
		{
			VoxelMaterial* material;
			boost::unordered_map<unsigned char, TerrainTexture>::iterator* which_ptr;
			AddBrush* add_brush;
			bool* enable_editing;

			KeyPressHandler(boost::unordered_map<unsigned char, TerrainTexture>::iterator* which_ptr, AddBrush* add_brush, bool* enable_editing) : material(NULL), which_ptr(which_ptr), add_brush(add_brush), enable_editing(enable_editing) { }

			void HandleEvent(Event* evt)
			{
				KeyStateEvent* kse = (KeyStateEvent*)evt;

				switch(kse->key)
				{
					case 'M':

						if(kse->state && material != NULL)
						{
							++(*which_ptr);
							if(*which_ptr == material->textures.end())
								*which_ptr = material->textures.begin();

							add_brush->UpdateName();
						}

						break;

					case VK_F1:

						if(kse->state)
							*enable_editing = !*enable_editing;

						break;
				}
			}
		} key_press_handler;


		struct SubtractBrush : public EditorBrush
		{
			SubtractBrush() : EditorBrush("Subtract") { }
			void DoAction(Imp* imp) { imp->ApplyBrush(SphereMaterialSetter(0)); imp->terrain->Solidify(); }
		} subtract_brush;

		struct AddBrush : public EditorBrush
		{
			VoxelMaterial* material;
			boost::unordered_map<unsigned char, TerrainTexture>::iterator* which_ptr;

			AddBrush(boost::unordered_map<unsigned char, TerrainTexture>::iterator* which_ptr) : EditorBrush("Add"), material(NULL), which_ptr(which_ptr) { }

			void UpdateName()		{ if(material != NULL) { name = ((stringstream&)(stringstream() << "Add \"" << (*which_ptr)->second.name << "\"")).str(); } }

			void DoAction(Imp* imp)	{ if(material != NULL) { imp->ApplyBrush(SphereMaterialSetter((*which_ptr)->second.material_index)); imp->terrain->Solidify(); } }
		} add_brush;

		struct SmoothBrush : public EditorBrush
		{
			SmoothBrush() : EditorBrush("Smooth") { }
			void DoAction(Imp* imp) { imp->ApplyBrush(SphereSmoother()); imp->terrain->Solidify(); }
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
		window->input_state->KeyStateChanged += &imp->key_press_handler;
	}

	void DTScreen::Deactivate()
	{
		window->input_state->MouseButtonStateChanged -= &imp->mouse_button_handler;
		window->input_state->MouseMoved -= &imp->mouse_motion_handler;
		window->input_state->KeyStateChanged -= &imp->key_press_handler;
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
