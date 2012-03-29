#include "StdAfx.h"

#include "TestGame.h"
#include "TestScreen.h"
#include "HUD.h"
#include "Dood.h"
#include "Soldier.h"
#include "CrabBug.h"
#include "ArtilleryBug.h"
#include "DSNMaterial.h"
#include "GlowyModelMaterial.h"
#include "Sun.h"
#include "Weapon.h"

#include "ConverterWhiz.h"

#include "CrabWeapon.h"
#include "DefaultWeapon.h"
#include "WorldBoundary.h"

#include "LevelLoad.h"

#include "MaterialLoader.h"

#include "Corpse.h"
#include "StaticLevelGeometry.h"
#include "Rubbish.h"

using namespace std;

#define ENABLE_SHADOWS 1

namespace Test
{
	/*
	 * TestGame::Loader private implementation struct
	 */
	struct TestGame::Loader::Imp
	{
		boost::mutex mutex;
		
		bool stopped;
		bool abort;

		Imp() : mutex(), stopped(false), abort(false) { }

		bool HasStopped()	{ boost::mutex::scoped_lock lock(mutex); return stopped; }
		bool HasAborted()	{ boost::mutex::scoped_lock lock(mutex); return abort; }
		void Stop()			{ boost::mutex::scoped_lock lock(mutex); stopped = true; }
		void Abort()		{ boost::mutex::scoped_lock lock(mutex); abort = true; }
	};




	/*
	 * TestGame::Loader methods
	 */
	TestGame::Loader::Loader(TestGame* game) : imp(new Imp()), game(game), task("") { }
	void TestGame::Loader::InnerDispose() { delete imp; imp = NULL; }

	void TestGame::Loader::operator ()() { game->Load(); }

	bool TestGame::Loader::HasStopped() { return imp->HasStopped(); }
	bool TestGame::Loader::HasAborted() { return imp->HasAborted(); }
	void TestGame::Loader::Stop() { imp->Stop(); }
	void TestGame::Loader::Abort() { imp->Abort(); }




	/*
	 * Struct to get all of the bugs on a level
	 */
	struct BugGetter : public EntityQualifier
	{
		bool Accept(Entity* ent)
		{
			Dood* dood = dynamic_cast<Dood*>(ent);
			if(dood != NULL && dood->team == TestGame::bug_team)
				return true;
			else
				return false;
		}
	};




	/*
	 * TestGame private implementation struct
	 */
	struct TestGame::Imp
	{
		class DoodDeathHandler : public EventHandler
		{
			public:
				TestGame* game;
				DoodDeathHandler() : game(NULL) { }

				void HandleEvent(Event* evt) { Dood::DeathEvent* de = (Dood::DeathEvent*)evt; if(de->dood == game->player_pawn) { game->imp->alive = false; } }
		} bot_death_handler, player_death_handler;

		class PlayerDamageHandler : public EventHandler
		{
			public:
				TestGame* game;
				PlayerDamageHandler() : game(NULL) { }

				void HandleEvent(Event* evt)
				{
					Dood::DamageTakenEvent* d_evt = (Dood::DamageTakenEvent*)evt;
					if(game->god_mode)
						d_evt->cancel = true;
				}
		} player_damage_handler;

		bool alive;

		UberModel* soldier_model;
		UberModel* crab_bug_model;
		UberModel* artillery_bug_model;

		UberModel* gun_model;
		VertexBuffer* mflash_model;
		VertexBuffer* shot_model;
		GlowyModelMaterial* mflash_material;
		BillboardMaterial* shot_material;
		
		BillboardMaterial* blood_red;
		BillboardMaterial* blood_blue;

		ParticleMaterial* dirt_particle;

		VertexBuffer* sky_sphere;
		TextureCube* sky_texture;
		ShaderProgram* sky_shader;

		SceneRenderer renderer;

		Sun* sun;

		TextureCube* ambient_cubemap;
		RenderTarget* render_target;
		RenderTarget* shadow_render_target;

		ShaderProgram* deferred_ambient;
		ShaderProgram* deferred_lighting;

		SoundBuffer* fire_sound;
		SoundBuffer* chamber_click_sound;
		SoundBuffer* reload_sound;

		Imp() :
			bot_death_handler(),
			player_death_handler(),
			player_damage_handler(),
			alive(true),
			renderer(NULL),
			render_target(NULL),
			shadow_render_target(NULL)
		{
		}

		~Imp()
		{
			if(render_target != NULL)
			{
				render_target->Dispose();
				delete render_target;
				render_target = NULL;
			}

			if(shadow_render_target != NULL)
			{
				shadow_render_target->Dispose();
				delete shadow_render_target;
				shadow_render_target = NULL;
			}
		}

		void DrawBackground(Mat4 view_matrix)
		{
			glDisable(GL_BLEND);
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);
			glDepthMask(false);

			GLDEBUG();

			sky_shader->SetUniform<TextureCube>	(	"sky_texture",	sky_texture		);
			sky_shader->SetUniform<Mat4>		(	"view_matrix",	&view_matrix	);
			ShaderProgram::SetActiveProgram(sky_shader);

			GLDEBUG();

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();

			sky_sphere->Draw();

			glPopMatrix();

			ShaderProgram::SetActiveProgram(NULL);

			sun->Draw();

			glDepthMask(true);			// otherwise depth testing breaks
		}

		void DrawNavEditorInfo(SceneRenderer* renderer, unsigned int nav_graph)
		{
			vector<unsigned int> nodes = NavGraph::GetAllNodes(nav_graph);
			vector<unsigned int> nu_nodes;

			glColor4f(1, 1, 1, 1);

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE);
			glEnable(GL_DEPTH_TEST);
			glDepthMask(true);
			glDisable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);

			glPointSize(4.0f);

			glBegin(GL_POINTS);

			Vec3 camera_fwd = renderer->camera->GetForward();
			float camera_dot = Vec3::Dot(camera_fwd, renderer->camera->GetPosition());
			// skip nodes behind the camera
			for(vector<unsigned int>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
			{
				unsigned int node = *iter;
				Vec3 pos = NavGraph::GetNodePosition(nav_graph, node);
				if(Vec3::Dot(pos, camera_fwd) > camera_dot)
				{
					glVertex3f(pos.x, pos.y, pos.z);
					nu_nodes.push_back(*iter);
				}
			}
			nodes = nu_nodes;

			glEnd();

			glPointSize(1.0f);
			glLineWidth(1.0f);

			glBegin(GL_LINES);
			for(vector<unsigned int>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
			{
				unsigned int node = *iter;
				Vec3 pos = NavGraph::GetNodePosition(nav_graph, node);
				vector<unsigned int> edges = NavGraph::GetNodeEdges(nav_graph, node, NavGraph::PD_OUT);

				for(vector<unsigned int>::iterator jter = edges.begin(); jter != edges.end(); ++jter)
				{
					Vec3 other = NavGraph::GetNodePosition(nav_graph, *jter);

					glColor4f(1, 0, 0, 1);
					glVertex3f(pos.x, pos.y, pos.z);
					glColor4f(1, 0, 0, 0);
					glVertex3f(other.x, other.y, other.z);
				}
			}
			glEnd();
		}
	};




	Team TestGame::human_team = Team(1);
	Team TestGame::bug_team = Team(2);

	unsigned int BuildNavGraph(TestGame* test_game);
	/*
	 * TestGame methods
	 */
	TestGame::TestGame(TestScreen* screen, SoundSystem* sound_system) :
		imp(new Imp()),
		screen(screen),
		nav_editor(false),
		god_mode(false),
		debug_draw(false),
		hud(NULL),
		player_controller(NULL),
		player_pawn(NULL),
		debug_text(""),
		chapter_text_start(0),
		chapter_text_end(0),
		chapter_text(),
		chapter_sub_text(),
		nav_graph(0),
		tex2d_cache(screen->window->content->GetCache<Texture2D>()),
		vtn_cache(screen->window->content->GetCache<VertexBuffer>()),
		ubermodel_cache(screen->window->content->GetCache<UberModel>()),
		mat_cache(NULL),
		load_status(this)
	{
		imp->bot_death_handler.game = this;
		imp->player_death_handler.game = this;
		imp->player_damage_handler.game = this;

		this->sound_system = sound_system;
		content = screen->window->content;
	}

	void TestGame::Load()
	{
		srand((unsigned int)time(NULL));

		sound_system->TryToEnable();

		font = content->GetCache<BitmapFont>()->Load("../Font");

		if(load_status.HasAborted())
		{
			load_status.Stop();
			return;
		}
		load_status.task = "dsn shader";

		// setting the content loader for materials if it hasn't been set already
		mat_cache = content->GetCache<Material>();
		if(mat_cache == NULL)
			mat_cache = content->CreateCache<Material>(new MaterialLoader(content));

		ScriptSystem::SetGS(this);

		ScriptingState thread_script = ScriptSystem::GetGlobalState().NewThread();

		ContentReqList content_req_list(content);

		ScriptSystem::SetContentReqList(&content_req_list);
		thread_script.DoFile("Files/Scripts/load.lua");
		ScriptSystem::SetContentReqList(NULL);
		content_req_list.LoadContent(&load_status.task);

		if(ubermodel_cache->Load("nbridge") == NULL)
		{
			load_status.task = "terrain mesh";

			// create terrain zzz file
			vector<MaterialModelPair> pairs;
			vector<string> material_names;
			MaterialModelPair pair;

			pair.material_index = 0;
			pair.vbo = vtn_cache->Load("nbridge");
			pairs.push_back(pair);
			material_names.push_back("nbridge");

			SkinnedModel* terrain_skinny = new SkinnedModel(pairs, material_names, new Skeleton());
			UberModel* terrain_model = UberModelLoader::CopySkinnedModel(terrain_skinny);
//			terrain_model->bone_physics.push_back(UberModel::BonePhysics());
//			terrain_model->bone_physics[0].shape = ShapeFromSkinnedModel(terrain_skinny);

			UberModelLoader::SaveZZZ(terrain_model, "Files/Models/nbridge.zzz");

			ubermodel_cache->GetMetadata(ubermodel_cache->GetHandle("nbridge").id).fail = false;
		}

		Cache<ModelPhysics>* mphys_cache = content->GetCache<ModelPhysics>();
		if(mphys_cache->Load("nbridge") == NULL)
		{
			load_status.task = "terrain mesh";

			ModelPhysics m;
			ModelPhysics::BonePhysics phys;

			TriangleMeshShape shape((*ubermodel_cache->Load("nbridge")->lods[0]->GetVBOs())[0].vbo);

			phys.bone_name = 0;
			phys.collision_shape = &shape;
			phys.mass_info = MassInfo();

			m.bones.push_back(phys);

			ModelPhysicsLoader::SaveZZP(&m, "Files/Physics/nbridge.zzp");
			
			mphys_cache->GetMetadata(mphys_cache->GetHandle("nbridge").id).fail = false;
		}

		load_status.task = "sky";

		// creating sky
		Cache<Shader>* shader_cache = content->GetCache<Shader>();
		Shader* sky_vertex_shader = shader_cache->Load("sky-v");
		Shader* sky_fragment_shader = shader_cache->Load("sky-f");

		ShaderProgram* sky_shader = imp->sky_shader = new ShaderProgram(sky_vertex_shader, sky_fragment_shader);
		sky_shader->AddUniform<TextureCube>(new UniformTextureCube("sky_texture", 0));
		sky_shader->AddUniform<Mat4>(new UniformMatrix4("view_matrix", false));

		imp->sky_texture = content->GetCache<TextureCube>()->Load("sky_cubemap");
		imp->sky_sphere = vtn_cache->Load("sky_sphere");

		if(load_status.HasAborted())
		{
			load_status.Stop();
			return;
		}
		load_status.task = "deferred lighting shader";

		imp->ambient_cubemap = content->GetCache<TextureCube>()->Load("ambient_cubemap");

		Shader* ds_vertex = shader_cache->Load("ds-v");
		Shader* ds_lighting = shader_cache->Load("ds_light-f");
		Shader* ds_ambient = shader_cache->Load("ds_ambient-f");

		ShaderProgram* deferred_lighting = imp->deferred_lighting = new ShaderProgram(ds_vertex, ds_lighting);
		deferred_lighting->AddUniform<Texture2D>(new UniformTexture2D("diffuse", 0));
		deferred_lighting->AddUniform<Texture2D>(new UniformTexture2D("normal", 1));
		deferred_lighting->AddUniform<Texture2D>(new UniformTexture2D("specular", 2));
		deferred_lighting->AddUniform<Texture2D>(new UniformTexture2D("depth", 3));	
#if ENABLE_SHADOWS
		deferred_lighting->AddUniform<Texture2D>(new UniformTexture2D("shadow_depth", 4));
		deferred_lighting->AddUniform<Mat4>(new UniformMatrix4("shadow_matrix", false));
		deferred_lighting->AddUniform<Mat4>(new UniformMatrix4("inv_shadow_matrix", false));
#endif
		deferred_lighting->AddUniform<Mat4>(new UniformMatrix4("inv_view_matrix", false));
		deferred_lighting->AddUniform<float>(new UniformFloat("aspect_ratio"));
		deferred_lighting->AddUniform<float>(new UniformFloat("zoom"));

		ShaderProgram* deferred_ambient = imp->deferred_ambient = new ShaderProgram(ds_vertex, ds_ambient);
		deferred_ambient->AddUniform<Texture2D>(new UniformTexture2D("diffuse", 0));
		deferred_ambient->AddUniform<Texture2D>(new UniformTexture2D("normal", 1));
		deferred_ambient->AddUniform<Texture2D>(new UniformTexture2D("specular", 2));
		deferred_ambient->AddUniform<Texture2D>(new UniformTexture2D("depth", 3));
		deferred_ambient->AddUniform<TextureCube>(new UniformTextureCube("ambient_cubemap", 4));
		deferred_ambient->AddUniform<TextureCube>(new UniformTextureCube("env_cubemap", 5));
		deferred_ambient->AddUniform<Mat4>(new UniformMatrix4("inv_view_matrix", false));
		deferred_ambient->AddUniform<float>(new UniformFloat("aspect_ratio"));
		deferred_ambient->AddUniform<float>(new UniformFloat("zoom"));

		if(load_status.HasAborted())
		{
			load_status.Stop();
			return;
		}
		load_status.task = "soldier";

		// Dood's model
		imp->soldier_model = ubermodel_cache->Load("soldier");

		imp->mflash_material = (GlowyModelMaterial*)mat_cache->Load("mflash");
		imp->shot_material = (BillboardMaterial*)mat_cache->Load("shot");
		imp->gun_model = ubermodel_cache->Load("gun");
		imp->mflash_model = vtn_cache->Load("mflash");
		imp->shot_model = vtn_cache->Load("shot");
		imp->blood_red = (BillboardMaterial*)mat_cache->Load("blood");
		imp->blood_blue = (BillboardMaterial*)mat_cache->Load("bug_blood");
		imp->dirt_particle = (ParticleMaterial*)mat_cache->Load("dirt_impact");

		if(load_status.HasAborted())
		{
			load_status.Stop();
			return;
		}
		load_status.task = "crab bug";

		imp->crab_bug_model = ubermodel_cache->Load("crab_bug");

		imp->artillery_bug_model = ubermodel_cache->Load("flea");
		if(imp->artillery_bug_model == NULL)
		{
			load_status.task = "artillery bug";

			vector<BoneEntry> bone_entries;
			ArtilleryBug::GetBoneEntries(bone_entries);

			UberModel* flea_model = AutoSkinUberModel(content, "flea", "flea", bone_entries);

			SetUberModelSkeleton(flea_model, bone_entries);
//			SetUberModelBonePhysics(flea_model, bone_entries);

			UberModelLoader::SaveZZZ(flea_model, "Files/Models/flea.zzz");

			ubermodel_cache->GetMetadata(ubermodel_cache->GetHandle("flea").id).fail = false;
		}

		if(ubermodel_cache->Load("pill") == NULL)
		{
			load_status.task = "pill";

			UberModel* pill_model = UberModelLoader::CopySkinnedModel(SkinnedModel::WrapVertexBuffer(vtn_cache->Load("pill"), "pill"));
			UberModelLoader::SaveZZZ(pill_model, "Files/Models/pill.zzz");

			ubermodel_cache->GetMetadata(ubermodel_cache->GetHandle("pill").id).fail = false;
		}

		if(load_status.HasAborted())
		{
			load_status.Stop();
			return;
		}
		load_status.task = "misc";

		// loading weapon sounds
		imp->fire_sound = content->GetCache<SoundBuffer>()->Load("shot");
		imp->chamber_click_sound = NULL;
		imp->reload_sound = NULL;

		if(load_status.HasAborted())
		{
			load_status.Stop();
			return;
		}
		load_status.task = "level";

		LoadLevel(this, "TestLevel");

		nav_graph = LoadNavGraph(this, "Files/Levels/TestNav.nav");
		if(nav_graph == 0)
		{
			nav_graph = BuildNavGraph(this);
			SaveNavGraph(nav_graph, "Files/Levels/TestNav.nav");
		}

		if(load_status.HasAborted())
		{
			load_status.Stop();
			return;
		}
		load_status.task = "starting game";

		imp->sun = new Sun(Vec3(2.4f, 4, 0), Vec3(1, 1, 1), NULL, NULL);

		hud = new HUD(this, screen->content);

		// dofile caused so much trouble D:<
		thread_script.DoFile("Files/Scripts/goals.lua");

		// if your game start script doesn't init the player, there will be trouble
		thread_script.DoFile("Files/Scripts/game_start.lua");
		hud->SetPlayer(player_pawn);

#if 1
		// spawn some rubbish
		for(int i = 0; i < 100; ++i)
		{
			Vec3 pos = Vec3(Random3D::Rand(-80, 80), 0, Random3D::Rand(-80, 80));
			pos.y = GetTerrainHeight(pos.x, pos.z) + 10;
			
			Quaternion ori = Random3D::RandomQuaternionRotation();

			Rubbish* rubbish = new Rubbish(this, ubermodel_cache->Load("dummycube"), pos, ori, imp->dirt_particle);
			Spawn(rubbish);
		}
#endif

		thread_script.Dispose();

		load_status.Stop();
	}

	Soldier* TestGame::SpawnPlayer(Vec3 pos)
	{
		if(player_pawn != NULL)
			player_pawn->is_valid = false;
		if(player_controller != NULL)
			player_controller->is_valid = false;

		player_pawn = new Soldier(this, imp->soldier_model, pos, human_team);
		Spawn(player_pawn);

		Spawn(player_pawn->equipped_weapon = new DefaultWeapon(this, player_pawn, imp->gun_model, imp->mflash_model, imp->shot_model, imp->mflash_material, imp->shot_material, imp->fire_sound, imp->chamber_click_sound, imp->reload_sound));

		player_pawn->blood_material = imp->blood_red;

		player_pawn->OnDeath += &imp->player_death_handler;
		player_pawn->OnDamageTaken += &imp->player_damage_handler;

		player_controller = new ScriptedController(this, "player_ai");
		player_controller->Possess(player_pawn);
		player_controller->ctrl_update_interval = 0;

		Spawn(player_controller);

		return player_pawn;
	}

	Dood* TestGame::SpawnBot(Vec3 pos)
	{
		Dood* dood = new CrabBug(this, imp->crab_bug_model, pos, bug_team);
		Spawn(dood);

		dood->blood_material = imp->blood_blue;

		Spawn(dood->intrinsic_weapon = new CrabWeapon(this, dood));

		ScriptedController* ai_controller = new ScriptedController(this, "crab_bug_ai");
		ai_controller->Possess(dood);

		Spawn(ai_controller);

		dood->OnDeath += &imp->bot_death_handler;

		return dood;
	}

	Dood* TestGame::SpawnArtilleryBug(Vec3 pos)
	{
		Dood* dood = new ArtilleryBug(this, imp->artillery_bug_model, pos, bug_team);
		Spawn(dood);

		dood->blood_material = imp->blood_blue;

		Spawn(dood->intrinsic_weapon = new CrabWeapon(this, dood));

		ScriptedController* ai_controller = new ScriptedController(this, "crab_bug_ai");
		ai_controller->Possess(dood);

		Spawn(ai_controller);

		dood->OnDeath += &imp->bot_death_handler;

		return dood;
	}

	unsigned int TestGame::GetNumberOfBugs()
	{
		BugGetter getter;
		EntityList bugs = GetQualifyingEntities(getter);

		return bugs.Count();
	}

	TestGame::~TestGame() { Dispose(); ScriptSystem::SetGS(NULL); }

	void DebugNavGraphError(unsigned int line, string file)
	{
		if(NavGraph::ErrorCode error = NavGraph::GetError())
		{
			stringstream msg;
			msg << "NavGraph error " << error << " (" << NavGraph::GetErrorString(error) << ") before line " << line << " of " << file << endl;
			Debug(msg.str());
		}
	}

#define NGDEBUG() DebugNavGraphError(__LINE__, __FILE__)

	string script_string;
	void TestGame::Update(TimingInfo time)
	{
		NGDEBUG();

		float elapsed = min((float)time.elapsed, 0.1f);
		total_game_time += elapsed;
		elapsed_game_time = elapsed;

		TimingInfo clamped_time = TimingInfo(elapsed, total_game_time);

		hud->UpdateHUDGauges(clamped_time);

		if(elapsed > 0)
		{
			stringstream fps_counter_ss;
			fps_counter_ss << "FPS = " << (int)(1.0 / time.elapsed);
			debug_text = fps_counter_ss.str();
		}

		if(script_string.empty())
			GetFileString("Files/Scripts/update.lua", &script_string);
		ScriptSystem::GetGlobalState().DoString(script_string);

		GameState::Update(clamped_time);
		ik_solver->Update(clamped_time);

		NGDEBUG();
	}

	// forward-declare a few functions which we'll be using in TestGame::Draw
	void DrawScreenQuad(ShaderProgram* shader, float sw, float sh, float tw, float th);
	void ClearDepthAndColor();
#if ENABLE_SHADOWS
	Texture2D* RenderShadowTexture(RenderTarget* target, Mat4& shadow_matrix, Sun* sun, SceneRenderer& renderer);
#endif

	void TestGame::Draw(int width_, int height_)
	{
		GLDEBUG();

		width = width_;
		height = height_;

		glViewport(0, 0, width, height);

		float zoom = 2.0f;
		float aspect_ratio = (float)width / height;

		static CameraView camera(Mat4::Identity(), 1.0f, 1.0f);
		if(imp->alive)
			camera = CameraView(((Dood*)player_controller->GetControlledPawn())->GetViewMatrix(), zoom, aspect_ratio);
		Mat4 proj_t = camera.GetProjectionMatrix().Transpose();
		Mat4 view_t = camera.GetViewMatrix().Transpose();

		// TODO: find a better place for this sound-related code?
		sound_system->SetListenerPos(-camera.GetViewMatrix().TransformVec3(0, 0, 0, 1));
		sound_system->SetListenerUp(camera.GetViewMatrix().TransformVec3(0, 1, 0, 0));
		sound_system->SetListenerForward(camera.GetViewMatrix().TransformVec3(0, 0, 1, 0));

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(&proj_t.values[0]);

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(&view_t.values[0]);

		GLDEBUG();

		if(debug_draw)
		{
			ClearDepthAndColor();

			imp->renderer.camera = &camera;
			physics_world->DebugDrawWorld(&imp->renderer);
		}
		else if(nav_editor)
		{
			ClearDepthAndColor();

			imp->renderer.camera = &camera;
			imp->DrawNavEditorInfo(&imp->renderer, nav_graph);
		}
		else
		{
			if(imp->render_target == NULL || imp->render_target->GetWidth() != width || imp->render_target->GetHeight() != height)
			{
				if(imp->render_target != NULL)
				{
					imp->render_target->Dispose();
					delete imp->render_target;
				}
				imp->render_target = new RenderTarget(width, height, 0, 4);
			}

#if ENABLE_SHADOWS
			if(imp->shadow_render_target == NULL)
				imp->shadow_render_target = new RenderTarget(4096, 4096, 0, 1);
#endif

			RenderTarget::Bind(imp->render_target);

			ClearDepthAndColor();

			glViewport(0, 0, width, height);

			imp->sun->view_matrix = camera.GetViewMatrix();
			imp->DrawBackground(camera.GetViewMatrix().Transpose());

			imp->renderer.camera = &camera;

			for(list<Entity*>::iterator iter = entities.begin(); iter != entities.end(); ++iter)
				(*iter)->Vis(&imp->renderer);
			imp->renderer.lights.push_back(imp->sun);

			imp->renderer.BeginRender();

			GLDEBUG();
			imp->renderer.RenderOpaque();
			GLDEBUG();

			glDepthMask(false);

			RenderTarget::Bind(NULL);

			glDepthMask(false);
			glColorMask(true, true, true, false);

			glEnable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);

			Mat4 inv_view_matrix = Mat4::Invert(imp->renderer.camera->GetViewMatrix());

			ShaderProgram* deferred_ambient = imp->deferred_ambient;
			RenderTarget* render_target = imp->render_target;

			deferred_ambient->SetUniform<Texture2D>		(	"diffuse",			render_target->GetColorBufferTex(0)	);
			deferred_ambient->SetUniform<Texture2D>		(	"normal",			render_target->GetColorBufferTex(1)	);
			deferred_ambient->SetUniform<Texture2D>		(	"specular",			render_target->GetColorBufferTex(2)	);
			deferred_ambient->SetUniform<Texture2D>		(	"depth",			render_target->GetColorBufferTex(3)	);
			deferred_ambient->SetUniform<TextureCube>	(	"ambient_cubemap",	imp->ambient_cubemap		);
			deferred_ambient->SetUniform<TextureCube>	(	"env_cubemap",		imp->sky_texture			);
			deferred_ambient->SetUniform<Mat4>			(	"inv_view_matrix",	&inv_view_matrix	);
			deferred_ambient->SetUniform<float>			(	"aspect_ratio",		&aspect_ratio		);
			deferred_ambient->SetUniform<float>			(	"zoom",				&zoom				);

			DrawScreenQuad(deferred_ambient, (float)width, (float)height, (float)render_target->GetWidth(), (float)render_target->GetHeight());

#if ENABLE_SHADOWS
			Mat4 shadow_matrix;				// passed by reference to RenderShadowTexture to give this a value
			Texture2D* shadow_texture = RenderShadowTexture(imp->shadow_render_target, shadow_matrix, imp->sun, imp->renderer);

			glBindTexture(GL_TEXTURE_2D, shadow_texture->GetGLName());
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

			Mat4 inv_shadow_matrix = Mat4::Invert(shadow_matrix);
#endif

			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE);

			glViewport(0, 0, width, height);
			glColorMask(true, true, true, false);

			ShaderProgram* deferred_lighting = imp->deferred_lighting;
			deferred_lighting->SetUniform<Texture2D>("diffuse", render_target->GetColorBufferTex(0));
			deferred_lighting->SetUniform<Texture2D>("normal", render_target->GetColorBufferTex(1));
			deferred_lighting->SetUniform<Texture2D>("specular", render_target->GetColorBufferTex(2));
			deferred_lighting->SetUniform<Texture2D>("depth", render_target->GetColorBufferTex(3));
#if ENABLE_SHADOWS
			deferred_lighting->SetUniform<Texture2D>("shadow_depth", shadow_texture);
			deferred_lighting->SetUniform<Mat4>("shadow_matrix", &shadow_matrix);
			deferred_lighting->SetUniform<Mat4>("inv_shadow_matrix", &inv_shadow_matrix);
#endif
			deferred_lighting->SetUniform<Mat4>("inv_view_matrix", &inv_view_matrix);
			deferred_lighting->SetUniform<float>("aspect_ratio", &aspect_ratio);
			deferred_lighting->SetUniform<float>("zoom", &zoom);

			DrawScreenQuad(deferred_lighting, (float)width, (float)height, (float)render_target->GetWidth(), (float)render_target->GetHeight());

			// re-draw the depth buffer (previous draw was on a different RenderTarget)
			glDepthMask(true);
			glClear(GL_DEPTH_BUFFER_BIT);

			imp->renderer.RenderDepth(false, false);

			GLDEBUG();
			glColorMask(true, true, true, false);
			imp->renderer.RenderTranslucent();
			GLDEBUG();

			imp->renderer.Cleanup();
		}

		hud->Draw((float)width, (float)height);

		GLDEBUG();
	}

	void ClearDepthAndColor()
	{
		glDepthMask(true);
		glColorMask(true, true, true, true);

		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	}

	void DrawScreenQuad(ShaderProgram* shader, float sw, float sh, float tw, float th)
	{
		sw--;
		sh--;

		ShaderProgram::SetActiveProgram(shader);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, sw, 0, sh, -1, 1);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glColor4f(1, 1, 1, 1);
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex2f(0, 0);
		glTexCoord2f(sw / (tw - 1), 0);
		glVertex2f(sw, 0);
		glTexCoord2f(sw / (tw - 1), sh / (th - 1));
		glVertex2f(sw, sh);
		glTexCoord2f(0, sh / (th - 1));
		glVertex2f(0, sh);
		glEnd();

		ShaderProgram::SetActiveProgram(NULL);

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

#if ENABLE_SHADOWS
	Texture2D* RenderShadowTexture(RenderTarget* target, Mat4& shadow_matrix, Sun* sun, SceneRenderer& renderer)
	{
		shadow_matrix = sun->GenerateShadowMatrix(*renderer.camera);

		RenderTarget* previous_render_target = RenderTarget::GetBoundRenderTarget();
		RenderTarget::Bind(target);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(-1, 1, -1, 1, 0, 1000);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixf(shadow_matrix.Transpose().values);

		glViewport(0, 0, target->GetWidth(), target->GetHeight());
		
		// not using "ClearDepthAndColor" because clear color must represent "far"
		glColorMask(true, true, true, true);
		glDepthMask(true);
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
		renderer.RenderDepth(true, true);

		// in case render functions changed the matrix mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		RenderTarget::Bind(previous_render_target);

		return target->GetColorBufferTex(0);
	}
#endif

	void TestGame::ShowChapterText(string text, string subtitle, float fade_time)
	{
		chapter_text = text;
		chapter_sub_text = subtitle;
		chapter_text_start = total_game_time;
		chapter_text_end = total_game_time + fade_time;
	}

	float TestGame::GetTerrainHeight(float x, float z)
	{
		struct RayCallback : public CollisionCallback
		{
			float max_y;
			RayCallback() : max_y(0.0f) { }

			bool OnCollision(const ContactPoint& cp)
			{
				if(Entity* entity = cp.b.obj->GetUserEntity())
					if(StaticLevelGeometry* slg = dynamic_cast<StaticLevelGeometry*>(entity))
					{
						float y = cp.a.pos.y;
						if(y > max_y)
							max_y = y;

						return true;
					}

				return false;
			}
		} ray_callback;

		physics_world->RayTest(Vec3(x, 0, z), Vec3(x, 1000, z), ray_callback);

		return ray_callback.max_y;
	}

	void TestGame::InnerDispose()
	{
		load_status.Dispose();

		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}

		if(hud != NULL)
		{
			delete hud;
			hud = NULL;
		}

		if(nav_graph != 0)
		{
			NavGraph::DeleteNavGraph(nav_graph);
			nav_graph = 0;
		}

		sound_system->StopAll();

		GameState::InnerDispose();

		if(ik_solver != NULL)
		{
			ik_solver->Dispose();
			delete ik_solver;
			ik_solver = NULL;
		}
	}

	void TestGame::VisUberModel(SceneRenderer* renderer, UberModel* model, int lod, Mat4 xform, SkinnedCharacter* character, vector<Material*>* materials)
	{
		if(model == NULL)
			return;

		vector<Material*> use_materials;
		if(materials != NULL)
			use_materials = *materials;
		else
		{
			for(unsigned int i = 0; i < model->materials.size(); ++i)
				use_materials.push_back(mat_cache->Load(model->materials[i]));
		}

		int num_lods = model->lods.size();
		if(lod >= num_lods)
			lod = num_lods - 1;
		UberModel::LOD* use_lod = model->lods[lod];

		Sphere bs = model->GetBoundingSphere();
		bs.center = xform.TransformVec3(bs.center, 1.0);

		vector<MaterialModelPair>* mmps = use_lod->GetVBOs();

		for(vector<MaterialModelPair>::iterator iter = mmps->begin(); iter != mmps->end(); ++iter)
		{
			MaterialModelPair& mmp = *iter;

			DSNMaterial* material = (DSNMaterial*)use_materials[mmp.material_index];
			VertexBuffer* vbo = mmp.vbo;

			if(character != NULL)
			{
				DSNMaterialNodeData* node_data = new DSNMaterialNodeData(vbo, xform, bs, character->GetBoneMatrices(), character->skeleton->bones.size());
				renderer->objects.push_back(RenderNode(material, node_data, Vec3::Dot(renderer->camera->GetForward(), bs.center)));
			}
			else
			{
				DSNMaterialNodeData* node_data = new DSNMaterialNodeData(vbo, xform, bs);
				renderer->objects.push_back(RenderNode(material, node_data, Vec3::Dot(renderer->camera->GetForward(), bs.center)));
			}
		}
	}

	int gs_spawnBot(lua_State* L);
	int gs_spawnArtilleryBug(lua_State* L);
	int gs_spawnPlayer(lua_State* L);
	int gs_getTerrainHeight(lua_State* L);
	int gs_getNumberOfBugs(lua_State* L);
	int gs_getDoodsList(lua_State* L);
	int gs_getNearestNavNode(lua_State* L);
	int gs_showChapterText(lua_State* L);
	int gs_setGodMode(lua_State* L);
	int gs_setNavEditMode(lua_State* L);
	int gs_setDebugDrawMode(lua_State* L);
	int gs_newPathSearch(lua_State* L);
	int gs_checkLineOfSight(lua_State* L);

	void TestGame::SetupScripting(ScriptingState& state)
	{
		GameState::SetupScripting(state);

		lua_State* L = state.GetLuaState();

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_spawnBot, 1);
		lua_setfield(L, 1, "spawnBot");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_spawnArtilleryBug, 1);
		lua_setfield(L, 1, "spawnArtilleryBug");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_spawnPlayer, 1);
		lua_setfield(L, 1, "spawnPlayer");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_getTerrainHeight, 1);
		lua_setfield(L, 1, "getTerrainHeight");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_getNumberOfBugs, 1);
		lua_setfield(L, 1, "getNumberOfBugs");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_getDoodsList, 1);
		lua_setfield(L, 1, "getDoodsList");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_getNearestNavNode, 1);
		lua_setfield(L, 1, "getNearestNav");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_showChapterText, 1);
		lua_setfield(L, 1, "showChapterText");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_setGodMode, 1);
		lua_setfield(L, 1, "setGodMode");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_setNavEditMode, 1);
		lua_setfield(L, 1, "setNavEditMode");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_setDebugDrawMode, 1);
		lua_setfield(L, 1, "setDebugDrawMode");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_newPathSearch, 1);
		lua_setfield(L, 1, "newPathSearch");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_checkLineOfSight, 1);
		lua_setfield(L, 1, "checkLineOfSight");
	}




	/*
	 * TestGame scripting stuff
	 */
	int gs_spawnBot(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1 && lua_isuserdata(L, 1))
		{
			void* ptr = lua_touserdata(L, 1);
			Vec3* vec = dynamic_cast<Vec3*>((Vec3*)ptr);

			lua_settop(L, 0);

			if(vec != NULL)
			{
				lua_pushvalue(L, lua_upvalueindex(1));
				TestGame* gs = (TestGame*)lua_touserdata(L, 1);
				Dood* dood = gs->SpawnBot(*vec);
				lua_pop(L, 1);

				PushDoodHandle(L, dood);

				return 1;
			}
		}

		Debug("gs.spawnBot takes exactly 1 argument, a position vector; returning nil\n");
		return 0;
	}

	int gs_spawnArtilleryBug(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1 && lua_isuserdata(L, 1))
		{
			void* ptr = lua_touserdata(L, 1);
			Vec3* vec = dynamic_cast<Vec3*>((Vec3*)ptr);

			lua_settop(L, 0);

			if(vec != NULL)
			{
				lua_pushvalue(L, lua_upvalueindex(1));
				TestGame* gs = (TestGame*)lua_touserdata(L, 1);
				Dood* dood = gs->SpawnArtilleryBug(*vec);
				lua_pop(L, 1);

				PushDoodHandle(L, dood);

				return 1;
			}
		}

		Debug("gs.spawnArtilleryBug takes exactly 1 argument, a position vector; returning nil\n");
		return 0;
	}

	int gs_spawnPlayer(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1 && lua_isuserdata(L, 1))
		{
			void* ptr = lua_touserdata(L, 1);
			Vec3* vec = dynamic_cast<Vec3*>((Vec3*)ptr);

			lua_settop(L, 0);

			if(vec != NULL)
			{
				lua_pushvalue(L, lua_upvalueindex(1));
				TestGame* gs = (TestGame*)lua_touserdata(L, 1);
				lua_pop(L, 1);

				PushDoodHandle(L, gs->SpawnPlayer(*vec));

				return 1;
			}
		}

		Debug("gs.spawnPlayer takes exactly 1 argument, a position vector; returning nil\n");
		return 0;
	}

	int gs_getTerrainHeight(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 2 && lua_isnumber(L, 1) && lua_isnumber(L, 2))
		{
			float x = (float)lua_tonumber(L, 1);
			float z = (float)lua_tonumber(L, 2);

			lua_settop(L, 0);
			lua_pushvalue(L, lua_upvalueindex(1));
			TestGame* gs = (TestGame*)lua_touserdata(L, 1);
			lua_pop(L, 1);

			lua_pushnumber(L, gs->GetTerrainHeight(x, z));

			return 1;
		}

		Debug("gs.getTerrainHeight takes exactly 2 arguments, an x and z coordinate; returning nil\n");
		return 0;
	}

	int gs_getNumberOfBugs(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 0)
		{
			lua_settop(L, 0);
			lua_pushvalue(L, lua_upvalueindex(1));
			TestGame* gs = (TestGame*)lua_touserdata(L, 1);
			lua_pop(L, 1);

			lua_pushnumber(L, gs->GetNumberOfBugs());

			return 1;
		}

		Debug("gs.getTerrainHeight doesn't take any arguments; returning nil\n");
		return 0;
	}

	int gs_getDoodsList(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 0)
		{
			lua_settop(L, 0);
			lua_pushvalue(L, lua_upvalueindex(1));
			TestGame* gs = (TestGame*)lua_touserdata(L, 1);
			lua_pop(L, 1);

			struct : EntityQualifier { bool Accept(Entity* ent) { return dynamic_cast<Dood*>(ent) != NULL; } } doods_only;
			EntityList e = gs->GetQualifyingEntities(doods_only);

			// begin table
			lua_newtable(L);							// push; top = 1

			for(unsigned int i = 0; i < e.Count(); ++i)
			{
				lua_pushnumber(L, i + 1);
				PushDoodHandle(L, (Dood*)e[i]);
				lua_settable(L, 1);
			}

			return 1;
		}

		Debug("gs.getDoodsList doesn't take any arguments; returning nil\n");
		return 0;
	}

	int gs_getNearestNavNode(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1)
		{
			TestGame* gs = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));

			Vec3 pos = *((Vec3*)lua_touserdata(L, 1));
			unsigned int node = NavGraph::GetNearestNode(gs->nav_graph, pos);

			lua_settop(L, 0);
			return PushNavNodeHandle(L, gs->nav_graph, node);
		}

		Debug("gs.getNearestNav takes exactly one parameter, a position vector; returning nil\n");
		return 0;
	}

	int gs_showChapterText(lua_State* L)
	{
		int n = lua_gettop(L);

		if(n >= 1 && n <= 3 && lua_isstring(L, 1))
		{
			string title = lua_tostring(L, 1);
			string subtitle = "";
			float duration = 2.0f;

			bool good = n == 1;

			if(n == 2 && (lua_isstring(L, 2) || lua_isnumber(L, 2)))
			{
				good = true;

				if(lua_isstring(L, 2))
					subtitle = lua_tostring(L, 2);
				else
					duration = (float)lua_tonumber(L, 2);
			}
			else if(n == 3 && lua_isstring(L, 2) && lua_isnumber(L, 3))
			{
				good = true;

				subtitle = lua_tostring(L, 2);
				duration = (float)lua_tonumber(L, 3);
			}

			if(good)
			{
				TestGame* gs = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));
				gs->ShowChapterText(title, subtitle, duration);

				return 0;
			}
		}

		Debug("Bad parameters for gs.showChapterText\n");
		return 0;
	}

	int gs_setGodMode(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1)
		{
			TestGame* gs = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));

			gs->god_mode = (bool)lua_toboolean(L, 1);

			lua_settop(L, 0);
			return 0;
		}

		Debug("gs.setGodMode takes exactly one parameter, a boolean\n");
		return 0;
	}

	int gs_setNavEditMode(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1)
		{
			TestGame* gs = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));

			gs->nav_editor = (bool)lua_toboolean(L, 1);

			lua_settop(L, 0);
			return 0;
		}

		Debug("gs.setNavEditMode takes exactly one parameter, a boolean\n");
		return 0;
	}

	int gs_setDebugDrawMode(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1)
		{
			TestGame* gs = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));

			gs->debug_draw = (bool)lua_toboolean(L, 1);

			lua_settop(L, 0);
			return 0;
		}

		Debug("gs.setDebugDrawMode takes exactly one parameter, a boolean\n");
		return 0;
	}

	int gs_checkLineOfSight(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 2)
		{
			TestGame* gs = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));

			Vec3* from = (Vec3*)lua_touserdata(L, 1);
			Vec3* to = (Vec3*)lua_touserdata(L, 2);

			bool result = VisionBlocker::CheckLineOfSight(gs->physics_world, *from, *to);

			lua_settop(L, 0);
			lua_pushboolean(L, result);

			return 1;
		}

		Debug("gs.checkLineOfSight takes two parameters, the points to check line of sight between\n");
		return 0;
	}

	bool MaybeCreateEdge(Vec3 delta, unsigned int graph, unsigned int my_node, unsigned int other_node)
	{
		float dy = delta.y;
		float dxz = sqrtf(delta.x * delta.x + delta.z * delta.z);
		float slope = dy / dxz;
		float cost = dxz + dy;
		if(slope < 0.5f && dy > -8.0f)
		{
			NavGraph::NewEdge(graph, my_node, other_node, cost);
			return true;
		}
		else
			return false;
	}

	void MaybeCreateEdge(TestGame* game, Vec3& my_pos, unsigned int graph, unsigned int my_node, vector<unsigned int> other_column)
	{
		for(vector<unsigned int>::iterator iter = other_column.begin(); iter != other_column.end(); ++iter)
		{
			unsigned int other_node = *iter;

			Vec3 other_pos = NavGraph::GetNodePosition(graph, other_node);
			MaybeCreateEdge(other_pos - my_pos, graph, my_node, other_node);
		}
	}

	const float min_ceiling_height = 2.0f;
	vector<float> GetNavGraphHeights(TestGame* game, float x, float z)
	{
		// define a callback for when a ray intersects an object
		list<float> results;

		struct RayCallback : public CollisionCallback
		{
			list<float>& results;
			RayCallback(list<float>& results) : results(results) { }

			bool OnCollision(const ContactPoint& cp)
			{
				if(Entity* entity = cp.b.obj->GetUserEntity())
					if(StaticLevelGeometry* slg = dynamic_cast<StaticLevelGeometry*>(entity))
						results.push_back(cp.a.pos.y);
				return false;
			}
		} ray_callback(results);

		float top = 1000;
		game->physics_world->RayTest(Vec3(x, 0, z), Vec3(x, top, z), ray_callback);
		
		results.sort();
		results.reverse();

		vector<float> valid_floors;
		bool floor = true;
		float y = top + min_ceiling_height;
		for(list<float>::iterator iter = results.begin(); iter != results.end(); ++iter)
		{
			float new_y = *iter;
			
			if(floor && new_y + min_ceiling_height <= y)
				valid_floors.push_back(new_y);

			y = new_y;
			floor = !floor;
		}
		if(valid_floors.size() > 1)
			return valid_floors;
		else
			return valid_floors;
	}

	template<class T> bool list_contains(list<T>& l, T& item)
	{
		for(list<T>::iterator iter = l.begin(); iter != l.end(); ++iter)
			if(*iter == item)
				return true;
		return false;
	}

	unsigned int BuildNavGraph(TestGame* test_game)
	{
		unsigned int graph = NavGraph::NewNavGraph(test_game);

		const unsigned int grid_res = 50;
		const unsigned int grm1 = grid_res - 1;

		vector<unsigned int>* nodes = new vector<unsigned int>[grid_res * grid_res];

		const float grid_min = -98.0f, grid_max = 98.0f, grid_size = grid_max - grid_min;
		const float grid_coeff = grid_size / grid_res;

		// place all of the nodes
		for(unsigned int i = 0; i < grid_res; ++i)
		{
			float x = grid_min + (float)i * grid_coeff;
			for(unsigned int j = 0; j < grid_res; ++j)
			{
				float z = grid_min + (float)j * grid_coeff;
				vector<unsigned int> column = vector<unsigned int>();

				vector<float> floors = GetNavGraphHeights(test_game, x, z);
				for(vector<float>::iterator iter = floors.begin(); iter != floors.end(); ++iter)
					column.push_back(NavGraph::NewNode(graph, Vec3(x, *iter, z)));

				nodes[i * grid_res + j] = column;
			}
		}

		// find out what nodes are directly connected to one another
		for(unsigned int i = 0; i < grid_res; ++i)
		{
			for(unsigned int j = 0; j < grid_res; ++j)
			{
				vector<unsigned int> my_column = nodes[i * grid_res + j];

				for(vector<unsigned int>::iterator iter = my_column.begin(); iter != my_column.end(); ++iter)
				{
					unsigned int my_node = *iter;
					Vec3 my_pos = NavGraph::GetNodePosition(graph, my_node);
					
					if(i > 0)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[(i - 1) * grid_res + j]);
					if(i < grm1)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[(i + 1) * grid_res + j]);
					if(j > 0)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[i * grid_res + j - 1]);
					if(j < grm1)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[i * grid_res + j + 1]);
					if(i > 0 && j > 0)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[(i - 1) * grid_res + j - 1]);
					if(i > 0 && j < grm1)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[(i - 1) * grid_res + j + 1]);
					if(i < grm1 && j > 0)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[(i + 1) * grid_res + j - 1]);
					if(i < grm1 && j < grm1)
						MaybeCreateEdge(test_game, my_pos, graph, my_node, nodes[(i + 1) * grid_res + j + 1]);
				}
			}
		}



		delete[] nodes;

		return graph;
	}

	list<unsigned int> FindPath(unsigned int graph, unsigned int source, unsigned int target);
	int gs_newPathSearch(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 2)
		{
			TestGame* game_state = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));
			LuaNavNode* a = (LuaNavNode*)lua_touserdata(L, 1);
			LuaNavNode* b = (LuaNavNode*)lua_touserdata(L, 2);

			if(a->graph == b->graph)
			{
				unsigned int graph = a->graph;
				PushPathSearchHandle(L, graph, a->node, b->node);

				return 1;
			}
		}
		
		Debug("gs.newPathSearch takes exactly 2 arguments, nav points on the same graph; returning nil\n");
		return 0;
	}

	/*
	 * NavGraph Pathfinding function! Uses A* (in theory)
	 */
	list<unsigned int> FindPath(unsigned int graph, unsigned int source, unsigned int target)
	{
		PathSearch search(graph, source, target);
		search.Solve();
		return search.GetSolution();
	}
}
