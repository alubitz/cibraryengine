#include "StdAfx.h"

#include "TestGame.h"
#include "TestScreen.h"
#include "HUD.h"
#include "Dood.h"
#include "Soldier.h"
#include "CrabBug.h"
#include "ArtilleryBug.h"
#include "Pendulum.h"
#include "Sun.h"
#include "Weapon.h"
#include "GhostCamera.h"

#include "CrabWeapon.h"
#include "DefaultWeapon.h"
#include "ArtilleryWeapon.h"

#include "Brambles.h"

#include "WorldBoundary.h"

#include "../CibraryEngine/DebugDrawMaterial.h"
#include "../CibraryEngine/TaskThread.h"

#include "LevelLoad.h"

#include "StaticLevelGeometry.h"
#include "Rubbish.h"

#include "GAExperiment.h"

#define ENABLE_SHADOWS                   1
#define SHADOW_MAP_SIZE                  1024
#define N_SHADOW_MAPS                    3

#define ENABLE_ENVIRONMENT_MAPPING       0


#define CPHFT_THREAD_COUNT               4

#define ENABLE_FPS_COUNTER               0
#define USE_GUN_AS_RUBBISH               0
#define PLAY_AS_CRAB_BUG                 1

#define DO_RAPID_UPDATE_TESTING          0
#define RAPID_UPDATE_COUNT               20
#define SPAWN_PLAYER_REPEATEDLY          0		// use newly spawned doods, or just re-init the existing dood?

#define DISALLOW_MULTIPLE_PLAYERS        0

namespace Test
{
	using namespace std;

	/*
	 * TestGame::Loader private implementation struct
	 */
	struct TestGame::Loader::Imp
	{
		mutex mutex;

		bool stopped;
		bool abort;

		string task;

		Imp() : mutex(), stopped(false), abort(false), task("") { }

		bool HasStopped()				{ unique_lock<std::mutex> lock(mutex); return stopped; }
		bool HasAborted()				{ unique_lock<std::mutex> lock(mutex); return abort; }
		void Stop()						{ unique_lock<std::mutex> lock(mutex); stopped = true; }
		void Abort()					{ unique_lock<std::mutex> lock(mutex); abort = true; }
		void SetTask(const string& str)	{ unique_lock<std::mutex> lock(mutex); task = str; }
		void GetTask(string& str)		{ unique_lock<std::mutex> lock(mutex); str = task; }
	};




	/*
	 * TestGame::Loader methods
	 */
	TestGame::Loader::Loader() : imp(new Imp()), game(NULL)	{ }
	void TestGame::Loader::InnerDispose()								{ delete imp; imp = NULL; }

	void TestGame::Loader::operator ()()								{ game->Load(); }

	bool TestGame::Loader::HasStopped()									{ return imp->HasStopped(); }
	bool TestGame::Loader::HasAborted()									{ return imp->HasAborted(); }
	void TestGame::Loader::Stop()										{ imp->Stop(); }
	void TestGame::Loader::Abort()										{ imp->Abort(); }
	void TestGame::Loader::SetTask(const string& str)					{ imp->SetTask(str); }
	void TestGame::Loader::GetTask(string& str)							{ imp->GetTask(str); }




	/*
	 * Struct to get all of the bugs on a level
	 */
	struct BugGetter : public EntityQualifier
	{
		bool Accept(Entity* ent)
		{
			Dood* dood = dynamic_cast<Dood*>(ent);
			if(dood != NULL && dood->team == TestGame::bug_team && dood->alive)
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

		class CharacterPhysicsHappyFunTime : public PhysicsStepCallback
		{
			public:

				struct CPHFTTask : public ThreadTask
				{
					float timestep;

					vector<Dood*>* doods;
					unsigned int from, to;

					void SetTaskParams(float timestep_, vector<Dood*>* doods_, unsigned int from_, unsigned int to_) { timestep = timestep_; doods = doods_; from = from_; to = to_; }

					void DoTask()
					{
						for(Dood **data = doods->data(), **iter = data + from, **end = data + to; iter != end; ++iter)
							(*iter)->PoseToPhysics(timestep);
					}
				} tasks[CPHFT_THREAD_COUNT];

				TestGame* game;

				TaskThread** threads;

				void OnPhysicsStep(PhysicsWorld* physics, float timestep)
				{
					struct DoodGetter : public EntityQualifier { bool Accept(Entity* ent) { return dynamic_cast<Dood*>(ent) != NULL; } } dood_getter;
					EntityList doods_elist = game->GetQualifyingEntities(dood_getter);

					unsigned int count = doods_elist.Count();
					vector<Dood*> doods;
					doods.reserve(count);
					for(unsigned int i = 0; i < count; ++i)
						doods.push_back((Dood*)doods_elist[i]);

					for(Dood **iter = doods.data(), **doods_end = iter + doods.size(); iter != doods_end; ++iter)
						(*iter)->PreCPHFT(timestep);

					// the actual posey stuff can be multithreaded though!
					unsigned int use_threads = CPHFT_THREAD_COUNT;
					for(unsigned int i = 0; i < use_threads; ++i)
					{
						tasks[i].SetTaskParams(timestep, &doods, i * count / use_threads, (i + 1) * count / use_threads);
						threads[i]->StartTask(&tasks[i]);
					}

					for(unsigned int i = 0; i < use_threads; ++i)
						threads[i]->WaitForCompletion();

					for(Dood **iter = doods.data(), **doods_end = iter + doods.size(); iter != doods_end; ++iter)
						(*iter)->PostCPHFT(timestep);
				}
		} character_physics_happy_fun_time;

		TaskThread* threads[CPHFT_THREAD_COUNT];

		bool alive;

		UberModel* soldier_model;
		UberModel* crab_bug_model;
		UberModel* artillery_bug_model;
		UberModel* rubbish_model;
		UberModel* pendulum_model;

		ModelPhysics* soldier_physics;
		ModelPhysics* crab_bug_physics;
		ModelPhysics* artillery_bug_physics;
		ModelPhysics* rubbish_physics;
		ModelPhysics* pendulum_physics;

		UberModel* gun_model;
		ModelPhysics* gun_physics;
		VertexBuffer* mflash_model;
		VertexBuffer* shot_model;
		GlowyModelMaterial* mflash_material;
		BillboardMaterial* shot_material;

		BillboardMaterial* blood_red;
		BillboardMaterial* blood_blue;

		ParticleMaterial* dirt_particle;

		TextureCube* sky_texture;
		TextureCube* ambient_cubemap;
		VertexBuffer* sky_sphere;
		ShaderProgram* sky_shader;

		SceneRenderer renderer;

		Sun* sun;

		RenderTarget* render_target;
		RenderTarget* shadow_render_targets[N_SHADOW_MAPS];

		ShaderProgram* deferred_ambient;
		ShaderProgram* deferred_lighting;

		SoundBuffer* fire_sound;
		SoundBuffer* chamber_click_sound;
		SoundBuffer* reload_sound;

		bool physics_content_init;

		GAExperiment* experiment;

		Imp() :
			bot_death_handler(),
			player_death_handler(),
			player_damage_handler(),
			character_physics_happy_fun_time(),
			alive(true),
			sky_shader(NULL),
			renderer(NULL),
			sun(NULL),
			render_target(NULL),
			shadow_render_targets(),
			deferred_ambient(NULL),
			deferred_lighting(NULL),
			physics_content_init(false),
			experiment(NULL)
		{
			for(unsigned int i = 0; i < CPHFT_THREAD_COUNT; ++i)
				threads[i] = new TaskThread();

			character_physics_happy_fun_time.threads = threads;
		}

		~Imp()
		{
			if(sun)						{ delete sun; sun = NULL; }

			if(render_target)			{ render_target->Dispose();			delete render_target;			render_target = NULL;			}

			for(unsigned int i = 0; i < N_SHADOW_MAPS; ++i)
				if(shadow_render_targets[i]) { shadow_render_targets[i]->Dispose(); delete shadow_render_targets[i]; shadow_render_targets[i] = NULL; }

			if(sky_shader)				{ sky_shader->Dispose();			delete sky_shader;				sky_shader = NULL;				}
			if(deferred_ambient)		{ deferred_ambient->Dispose();		delete deferred_ambient;		deferred_ambient = NULL;		}
			if(deferred_lighting)		{ deferred_lighting->Dispose();		delete deferred_lighting;		deferred_lighting = NULL;		}

			for(unsigned int i = 0; i < CPHFT_THREAD_COUNT; ++i)
			{
				threads[i]->Shutdown();
				delete threads[i];
				threads[i] = NULL;
			}

			if(experiment)				{ delete experiment; experiment = NULL; }
		}

		void DrawBackground(Mat4& view_matrix)
		{
			glDisable(GL_BLEND);
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);
			glDepthMask(false);

			GLDEBUG();

			sky_shader->SetUniform<TextureCube> ( "sky_texture", sky_texture  );
			sky_shader->SetUniform<Mat4>        ( "view_matrix", &view_matrix );
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
	TestGame::TestGame(TestScreen* screen, SoundSystem* sound_system_) :
		imp(new Imp()),
		screen(screen),
		nav_editor(false),
		god_mode(false),
		debug_draw(false),
		sim_speed(1.0f),
		quit(false),
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
		mphys_cache(screen->window->content->GetCache<ModelPhysics>()),
		load_status()
	{
		//physics_world->SetGravity(Vec3());

		imp->bot_death_handler.game					= this;
		imp->player_death_handler.game				= this;
		imp->player_damage_handler.game				= this;
		imp->character_physics_happy_fun_time.game	= this;

		load_status.game							= this;

		sound_system = sound_system_;
		content = screen->window->content;
	}

	void TestGame::Load()
	{
		srand((unsigned int)time(NULL));

		sound_system->TryToEnable();

		font = content->GetCache<BitmapFont>()->Load("../Font");

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("dsn shader"); }

		// setting the content loader for materials if it hasn't been set already
		mat_cache = content->GetCache<Material>();

		ScriptSystem::SetGS(this);

		ScriptingState thread_script = ScriptSystem::GetGlobalState().NewThread();

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("Models..."); }
		ContentReqList content_req_list(content);

		ScriptSystem::SetContentReqList(&content_req_list);
		thread_script.DoFile("Files/Scripts/load.lua");
		ScriptSystem::SetContentReqList(NULL);

		class StatusUpdater : public ContentReqList::StatusUpdater
		{
			public:
				Loader& loader;
				StatusUpdater(Loader& loader) : loader(loader) { }
				void SetString(const string& str) { loader.SetTask(str); }
		} crl_status_updater(load_status);

		content_req_list.LoadContent(&crl_status_updater);

		// creating sky
		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("sky"); }

		Cache<TextureCube>* cubemap_cache = content->GetCache<TextureCube>();
		Cache<Shader     >* shader_cache  = content->GetCache<Shader>();
		Cache<SoundBuffer>* sound_cache   = content->GetCache<SoundBuffer>();

		Shader* sky_vertex_shader	= shader_cache->Load("sky-v");
		Shader* sky_fragment_shader	= shader_cache->Load("sky-f");

		ShaderProgram* sky_shader = imp->sky_shader = new ShaderProgram(sky_vertex_shader, sky_fragment_shader);
		sky_shader->AddUniform<TextureCube>        ( new UniformTextureCube ( "sky_texture",         0     ));
		sky_shader->AddUniform<Mat4>               ( new UniformMatrix4(      "view_matrix",         false ));

		imp->sky_texture = cubemap_cache->Load("sky_cubemap");
		imp->sky_sphere  = vtn_cache->Load("sky_sphere");

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("deferred lighting shader"); }

		imp->ambient_cubemap = cubemap_cache->Load("ambient_cubemap");

		Shader* ds_vertex   = shader_cache->Load("ds-v");
		Shader* ds_lighting = shader_cache->Load("ds_light-f");
		Shader* ds_ambient  = shader_cache->Load("ds_ambient-f");

		ShaderProgram* deferred_lighting = imp->deferred_lighting = new ShaderProgram(ds_vertex, ds_lighting);
		deferred_lighting->AddUniform<Texture2D>   ( new UniformTexture2D   ( "diffuse",             0     ));
		deferred_lighting->AddUniform<Texture2D>   ( new UniformTexture2D   ( "normal",              1     ));
		deferred_lighting->AddUniform<Texture2D>   ( new UniformTexture2D   ( "specular",            2     ));
		deferred_lighting->AddUniform<Texture2D>   ( new UniformTexture2D   ( "depth",               3     ));
		deferred_lighting->AddUniform<float>       ( new UniformFloat       ( "camera_near"                ));
		deferred_lighting->AddUniform<float>       ( new UniformFloat       ( "camera_far"                 ));
#if ENABLE_SHADOWS
		deferred_lighting->AddUniform<Texture2D>   ( new UniformTexture2D   ( "shadow_depths[0]",    4     ));
		deferred_lighting->AddUniform<Texture2D>   ( new UniformTexture2D   ( "shadow_depths[1]",    5     ));
		deferred_lighting->AddUniform<Texture2D>   ( new UniformTexture2D   ( "shadow_depths[2]",    6     ));
		deferred_lighting->AddUniform<vector<Mat4>>( new UniformMatrix4Array( "shadow_matrices",     false ));
		deferred_lighting->AddUniform<vector<Mat4>>( new UniformMatrix4Array( "inv_shadow_matrices", false ));
		deferred_lighting->AddUniform<float>       ( new UniformFloat       ( "shadow_near"                ));
		deferred_lighting->AddUniform<float>       ( new UniformFloat       ( "shadow_far"                 ));
#endif
		deferred_lighting->AddUniform<Mat4>        ( new UniformMatrix4     ( "inv_view_matrix",     false ));
		deferred_lighting->AddUniform<float>       ( new UniformFloat       ( "aspect_ratio"               ));
		deferred_lighting->AddUniform<float>       ( new UniformFloat       ( "zoom"                       ));


		ShaderProgram* deferred_ambient = imp->deferred_ambient = new ShaderProgram(ds_vertex, ds_ambient);
		deferred_ambient->AddUniform<Texture2D>    ( new UniformTexture2D   ( "diffuse",             0     ));
		deferred_ambient->AddUniform<Texture2D>    ( new UniformTexture2D   ( "normal",              1     ));
		deferred_ambient->AddUniform<Texture2D>    ( new UniformTexture2D   ( "specular",            2     ));
#if ENABLE_ENVIRONMENT_MAPPING
		deferred_ambient->AddUniform<Texture2D>    ( new UniformTexture2D   ( "depth",               3     ));
		deferred_ambient->AddUniform<float>        ( new UniformFloat       ( "camera_near"                ));
		deferred_ambient->AddUniform<float>        ( new UniformFloat       ( "camera_far"                 ));
		deferred_ambient->AddUniform<TextureCube>  ( new UniformTextureCube ( "env_cubemap",         5     ));
#endif
		deferred_ambient->AddUniform<TextureCube>  ( new UniformTextureCube ( "ambient_cubemap",     4     ));
		deferred_ambient->AddUniform<Mat4>         ( new UniformMatrix4     ( "view_matrix",         false ));
		deferred_ambient->AddUniform<float>        ( new UniformFloat       ( "aspect_ratio"               ));
		deferred_ambient->AddUniform<float>        ( new UniformFloat       ( "zoom"                       ));

		// Dood's model
		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("soldier"); }

		imp->soldier_model   = ubermodel_cache->Load("soldier");
		imp->soldier_physics = mphys_cache->Load("soldier");

		imp->mflash_material = (GlowyModelMaterial*)mat_cache->Load("mflash");
		imp->shot_material   = (BillboardMaterial*)mat_cache->Load("shot");
		imp->gun_model       = ubermodel_cache->Load("gun");
		imp->gun_physics     = mphys_cache->Load("gun");
		imp->mflash_model    = vtn_cache->Load("mflash");
		imp->shot_model      = vtn_cache->Load("shot");
		imp->blood_red       = (BillboardMaterial*)mat_cache->Load("blood");
		imp->blood_blue      = (BillboardMaterial*)mat_cache->Load("bug_blood");
		imp->dirt_particle   = (ParticleMaterial*)mat_cache->Load("dirt_impact");

#if USE_GUN_AS_RUBBISH
		imp->rubbish_model         = imp->gun_model;
		imp->rubbish_physics       = imp->gun_physics;
#else
		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("rubbish"); }

		imp->rubbish_model         = ubermodel_cache->Load("dummycube");
		imp->rubbish_physics       = mphys_cache->Load("dummycube");

		imp->rubbish_model->LoadCachedMaterials(mat_cache);
#endif

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("crab bug"); }

		imp->crab_bug_model        = ubermodel_cache->Load("crab_bug");
		imp->crab_bug_physics      = mphys_cache->Load("crab_bug");

		imp->pendulum_model          = ubermodel_cache->Load("pendulum");
		imp->pendulum_physics        = mphys_cache->Load("pendulum");

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("artillery bug"); }

		imp->artillery_bug_model   = ubermodel_cache->Load("flea");
		imp->artillery_bug_physics = mphys_cache->Load("flea");

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("misc"); }

		// loading weapon sounds
		imp->fire_sound          = sound_cache->Load("shot");
		imp->chamber_click_sound = NULL;
		imp->reload_sound        = NULL;

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("level"); }

		LoadLevel(this, "TestLevel");

		//Spawn(new Bramble(this, Vec3(0, 0, 20)));

		nav_graph = LoadNavGraph(this, "Files/Levels/TestNav.nav");
		if(nav_graph == 0)
		{
			nav_graph = BuildNavGraph(this);
			SaveNavGraph(nav_graph, "Files/Levels/TestNav.nav");
		}

		if(load_status.HasAborted()) { load_status.Stop(); return; } else { load_status.SetTask("starting game"); }

		imp->sun = new Sun(Vec3(2.4f, 4, 0), Vec3(1, 0.95f, 0.8f), NULL, NULL);

		hud = new HUD(this, content);

		imp->experiment = new GAExperiment("Files/brains/genepool");

		// dofile caused so much trouble D:<
		thread_script.DoFile("Files/Scripts/goals.lua");

		// if your game_start script doesn't init the player, there will be trouble
		thread_script.DoFile("Files/Scripts/game_start.lua");

		SpawnGhostCamera(Vec3(0, 5, 0));

		thread_script.Dispose();

		load_status.Stop();
	}

	Dood* TestGame::SpawnPlayer(const Vec3& pos)
	{
		if(player_controller != NULL)
			player_controller->Exorcise();

#if DISALLOW_MULTIPLE_PLAYERS
		if(player_pawn != NULL)
		{
			player_pawn->is_valid = false;
			if(player_pawn->equipped_weapon != NULL)
				player_pawn->equipped_weapon->is_valid = false;
		}
#endif

#if PLAY_AS_CRAB_BUG
		//player_pawn = new CrabBug(this, imp->experiment, imp->crab_bug_model, imp->crab_bug_physics, pos, human_team);
		player_pawn = new Pendulum(this, imp->experiment, imp->pendulum_model, imp->pendulum_physics, pos, human_team);
#else
		player_pawn = new Soldier(this, imp->soldier_model, imp->soldier_physics, pos, human_team);
#endif
		player_pawn->blood_material = imp->blood_red;
		Spawn(player_pawn);

#if !PLAY_AS_CRAB_BUG
		WeaponEquip* player_weapon = new DefaultWeapon(this, player_pawn, imp->gun_model, imp->mflash_model, imp->mflash_material, imp->gun_physics, imp->shot_model, imp->shot_material, imp->fire_sound, imp->chamber_click_sound, imp->reload_sound);
		Spawn(player_weapon);

		player_weapon->Equip(player_pawn);
#endif

		player_pawn->OnDeath += &imp->player_death_handler;
		player_pawn->OnDamageTaken += &imp->player_damage_handler;

		if(player_controller == NULL)
		{
			player_controller = new ScriptedController(this, "player_ai");
			player_controller->Possess(player_pawn);
			player_controller->ctrl_update_interval = 0;
			Spawn(player_controller);
		}
		else
			player_controller->Possess(player_pawn);

		hud->SetPlayer(player_pawn);

		return player_pawn;
	}

	Dood* TestGame::SpawnBot(const Vec3& pos)
	{
		Dood* dood = new CrabBug(this, imp->experiment, imp->crab_bug_model, imp->crab_bug_physics, pos, bug_team);
		Spawn(dood);

		dood->blood_material = imp->blood_blue;

		Spawn(dood->intrinsic_weapon = new CrabWeapon(this, dood));

		ScriptedController* ai_controller = new ScriptedController(this, "crab_bug_ai");
		ai_controller->Possess(dood);

		Spawn(ai_controller);

		dood->OnDeath += &imp->bot_death_handler;

		return dood;
	}

	Dood* TestGame::SpawnArtilleryBug(const Vec3& pos)
	{
		Dood* dood = new ArtilleryBug(this, imp->artillery_bug_model, imp->artillery_bug_physics, pos, bug_team);
		Spawn(dood);

		dood->blood_material = imp->blood_blue;

		Spawn(dood->intrinsic_weapon = new ArtilleryWeapon(this, dood, dood->character->skeleton->GetNamedBone("carapace"), Vec3(0, 10.0f, 8.0f)));

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

	Pawn* TestGame::SpawnGhostCamera(const Vec3& pos)
	{
		player_pawn = nullptr;
		if(player_controller != NULL)
			player_controller->Exorcise();
		GhostCamera* camera = new GhostCamera(this);
		camera->pos = pos;
		Spawn(camera);
		if(player_controller == NULL)
		{
			player_controller = new ScriptedController(this, "player_ai");
			player_controller->Possess(camera);
			player_controller->ctrl_update_interval = 0;
			Spawn(player_controller);
		}
		else
			player_controller->Possess(camera);
		hud->SetPlayer(nullptr);
		return camera;
	}


	Rubbish* TestGame::SpawnRubbish(const Vec3& pos)
	{
		UberModel* rubbish_model = imp->rubbish_model;
		ModelPhysics* rubbish_phys = imp->rubbish_physics;

		if(rubbish_model && rubbish_phys)
		{
			Quaternion ori = Random3D::RandomQuaternionRotation();

			Rubbish* rubbish = new Rubbish(this, rubbish_model, rubbish_phys, pos, ori, imp->dirt_particle);
			Spawn(rubbish);

			return rubbish;
		}

		return NULL;
	}

	TestGame::~TestGame() { Dispose(); ScriptSystem::SetGS(NULL); }

	void DebugNavGraphError(unsigned int line, const string& file)
	{
		if(NavGraph::ErrorCode error = NavGraph::GetError())
		{
			stringstream msg;
			msg << "NavGraph error " << error << " (" << NavGraph::GetErrorString(error) << ") before line " << line << " of " << file << endl;
			Debug(msg.str());
		}
	}

#define NGDEBUG() DebugNavGraphError(__LINE__, __FILE__)

	static string script_string;
	void TestGame::Update(const TimingInfo& time)
	{
		NGDEBUG();

		if(!imp->physics_content_init)
		{
			physics_world->InitConstraintGraphSolver(content);
			physics_world->SetStepCallback(&imp->character_physics_happy_fun_time);

			imp->physics_content_init = true;
		}

#if DO_RAPID_UPDATE_TESTING
		for(unsigned int i = 0; i < RAPID_UPDATE_COUNT; ++i)
		{
			float elapsed = 1.0f / 60.0f;
#else
			float elapsed = elapsed_game_time = min((float)time.elapsed, 1.0f / 60.0f);
			elapsed *= sim_speed;
#endif

			total_game_time += elapsed;

#if SPAWN_PLAYER_REPEATEDLY
			if(((Soldier*)player_pawn)->IsExperimentDone())
				SpawnPlayer(Vec3());
#endif
			TimingInfo clamped_time(elapsed, total_game_time);

			hud->UpdateHUDGauges(clamped_time);

#if ENABLE_FPS_COUNTER
			if(elapsed > 0)
				debug_text = ((stringstream&)(stringstream() << "FPS = " << (int)(1.0 / time.elapsed))).str();
#endif

			if(script_string.empty())
				GetFileString("Files/Scripts/update.lua", &script_string);
			ScriptSystem::GetGlobalState().DoString(script_string);

			GameState::Update(clamped_time);

			if(imp->experiment != nullptr)
				debug_text = imp->experiment->GetDebugText();

			NGDEBUG();

#if DO_RAPID_UPDATE_TESTING
		}
#endif
	}

	// forward-declare a few functions which we'll be using in TestGame::Draw
	void DrawScreenQuad(ShaderProgram* shader, float sw, float sh, float tw, float th);
	void ClearDepthAndColor();

#if ENABLE_SHADOWS
	Texture2D* RenderShadowTexture(RenderTarget* target, const Mat4& shadow_matrix, float near_plane, float far_plane, Sun* sun, SceneRenderer& renderer)
	{
		RenderTarget* previous_render_target = RenderTarget::GetBoundRenderTarget();
		RenderTarget::Bind(target);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();

		glOrtho(-1, 1, -1, 1, near_plane, far_plane);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glMultMatrixf(shadow_matrix.Transpose().values);

		glViewport(0, 0, target->GetWidth(), target->GetHeight());

		ClearDepthAndColor();
		renderer.RenderDepth(true, true);

		// in case render functions changed the matrix mode
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		RenderTarget::Bind(previous_render_target);

		return target->GetDepthBufferTex();
	}
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
		{
			Pawn* controlled_pawn = player_controller->GetControlledPawn();
			GhostCamera* player_cam = dynamic_cast<GhostCamera*>(controlled_pawn);
			Mat4 view_matrix = player_cam != nullptr ? player_cam->GetViewMatrix() : ((Dood*)controlled_pawn)->GetViewMatrix();
			camera = CameraView(view_matrix, zoom, aspect_ratio);
		}

		Mat4 camera_proj   = camera.GetProjectionMatrix();
		Mat4 camera_view   = camera.GetViewMatrix();
		Mat4 camera_proj_t = camera_proj.Transpose();
		Mat4 camera_view_t = camera_view.Transpose();
		Vec3 camera_pos    = camera.GetPosition();

		// TODO: find a better place for this sound-related code?
		sound_system->SetListenerPos    (-camera_view.TransformVec3_1(0, 0, 0));
		sound_system->SetListenerUp     ( camera_view.TransformVec3_0(0, 1, 0));
		sound_system->SetListenerForward( camera_view.TransformVec3_0(0, 0, 1));

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(camera_proj_t.values);

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(camera_view_t.values);

		GLDEBUG();

		if(debug_draw)
		{
			ClearDepthAndColor();

			imp->renderer.camera = &camera;
			physics_world->DebugDrawWorld(&imp->renderer);

			GLDEBUG();
		}
		else if(nav_editor)
		{
			ClearDepthAndColor();

			imp->renderer.camera = &camera;
			imp->DrawNavEditorInfo(&imp->renderer, nav_graph);

			GLDEBUG();
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
				imp->render_target = new RenderTarget(width, height, 0, 3);
			}

#if ENABLE_SHADOWS
			for(unsigned int i = 0; i < N_SHADOW_MAPS; ++i)
				if(imp->shadow_render_targets[i] == NULL)
					imp->shadow_render_targets[i] = new RenderTarget(SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, 0, 0);
#endif

			GLDEBUG();
			RenderTarget::Bind(imp->render_target);
			GLDEBUG();

			ClearDepthAndColor();

			glViewport(0, 0, width, height);

			// draw opaque stuff to buffers for use in deferred lighting shaders
			imp->sun->view_matrix = camera_view;
			imp->DrawBackground(camera_view_t);

			imp->renderer.camera = &camera;

			for(list<Entity*>::iterator iter = entities.begin(); iter != entities.end(); ++iter)
				(*iter)->Vis(&imp->renderer);
			imp->renderer.lights.push_back(imp->sun);

			imp->renderer.BeginRender();

			GLDEBUG();
			imp->renderer.RenderOpaque();
			GLDEBUG();

			// do ambient and emissive lighting (deferred)
			glDepthMask(false);

			RenderTarget::Bind(NULL);

			glDepthMask(false);
			glColorMask(true, true, true, false);

			glEnable(GL_TEXTURE_2D);
			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);

			Mat4 inv_view_matrix = Mat4::Invert(camera_view);

			float camera_near = -imp->renderer.camera->GetNearPlane().PointDistance(camera_pos);
			float camera_far  = imp->renderer.camera->GetFarPlane().PointDistance(camera_pos);

			ShaderProgram* deferred_ambient = imp->deferred_ambient;
			RenderTarget* render_target = imp->render_target;

			deferred_ambient->SetUniform<Texture2D>  ( "diffuse",         render_target->GetColorBufferTex(0));
			deferred_ambient->SetUniform<Texture2D>  ( "normal",          render_target->GetColorBufferTex(1));
			deferred_ambient->SetUniform<Texture2D>  ( "specular",        render_target->GetColorBufferTex(2));
#if ENABLE_ENVIRONMENT_MAPPING
			deferred_ambient->SetUniform<Texture2D>  ( "depth",           render_target->GetDepthBufferTex());
			deferred_ambient->SetUniform<float>      ( "camera_near",     &camera_near         );
			deferred_ambient->SetUniform<float>      ( "camera_far",      &camera_far          );
			deferred_ambient->SetUniform<TextureCube>( "env_cubemap",     imp->sky_texture     );
#endif
			deferred_ambient->SetUniform<TextureCube>( "ambient_cubemap", imp->ambient_cubemap );
			deferred_ambient->SetUniform<Mat4>       ( "view_matrix",     &camera_view         );
			deferred_ambient->SetUniform<float>      ( "aspect_ratio",    &aspect_ratio        );
			deferred_ambient->SetUniform<float>      ( "zoom",            &zoom                );

			DrawScreenQuad(deferred_ambient, (float)width, (float)height, (float)render_target->GetWidth(), (float)render_target->GetHeight());

			// do lighting from the sun (deferred)
#if ENABLE_SHADOWS

			float shadow_near = 0, shadow_far = 1000;

			float        shadow_region_radii[N_SHADOW_MAPS] = { 8.0f, 32.0f, 100.0f };
			vector<Mat4>     shadow_matrices(N_SHADOW_MAPS);
			vector<Mat4> inv_shadow_matrices(N_SHADOW_MAPS);
			Texture2D*       shadow_textures[N_SHADOW_MAPS];

			imp->sun->GenerateShadowMatrices(*imp->renderer.camera, N_SHADOW_MAPS, shadow_region_radii, shadow_matrices.data());

			for(int i = 0; i < N_SHADOW_MAPS; ++i)
			{
				Texture2D* shadow_tex = shadow_textures[i] = RenderShadowTexture(imp->shadow_render_targets[i], shadow_matrices[i], shadow_near, shadow_far, imp->sun, imp->renderer);

				glBindTexture(GL_TEXTURE_2D, shadow_tex->GetGLName());
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

				inv_shadow_matrices[i] = Mat4::Invert(shadow_matrices[i]);
			}
#endif

			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE);

			glViewport(0, 0, width, height);
			glColorMask(true, true, true, false);

			imp->sun->SetLight(0);

			ShaderProgram* deferred_lighting = imp->deferred_lighting;
			deferred_lighting->SetUniform<Texture2D>    ( "diffuse",             render_target->GetColorBufferTex(0));
			deferred_lighting->SetUniform<Texture2D>    ( "normal",              render_target->GetColorBufferTex(1));
			deferred_lighting->SetUniform<Texture2D>    ( "specular",            render_target->GetColorBufferTex(2));
			deferred_lighting->SetUniform<Texture2D>    ( "depth",               render_target->GetDepthBufferTex() );
			deferred_lighting->SetUniform<float>        ( "camera_near",         &camera_near         );
			deferred_lighting->SetUniform<float>        ( "camera_far",          &camera_far          );
#if ENABLE_SHADOWS
			deferred_lighting->SetUniform<Texture2D>    ( "shadow_depths[0]",    shadow_textures[0]   );
			deferred_lighting->SetUniform<Texture2D>    ( "shadow_depths[1]",    shadow_textures[1]   );
			deferred_lighting->SetUniform<Texture2D>    ( "shadow_depths[2]",    shadow_textures[2]   );
			deferred_lighting->SetUniform<vector<Mat4>> ( "shadow_matrices",     &shadow_matrices     );
			deferred_lighting->SetUniform<vector<Mat4>> ( "inv_shadow_matrices", &inv_shadow_matrices );
			deferred_lighting->SetUniform<float>        ( "shadow_near",         &shadow_near         );
			deferred_lighting->SetUniform<float>        ( "shadow_far",          &shadow_far          );
#endif
			deferred_lighting->SetUniform<Mat4>         ( "inv_view_matrix",     &inv_view_matrix     );
			deferred_lighting->SetUniform<float>        ( "aspect_ratio",        &aspect_ratio        );
			deferred_lighting->SetUniform<float>        ( "zoom",                &zoom                );

			DrawScreenQuad(deferred_lighting, (float)width, (float)height, (float)render_target->GetWidth(), (float)render_target->GetHeight());

			imp->sun->UnsetLight(0);

			// draw translucent stuff on top of our opaque stuff
			// we need to re-draw the depth buffer because the previous draw was on a different RenderTarget
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

		if(debug_draw)
			DrawCompass(camera.GetViewMatrix());

		GLDEBUG();
	}

	void TestGame::DrawCompass(const Mat4& view_matrix) const
	{
		// draw world coordinate axes
		Vec3 x_axis = view_matrix.TransformVec3_0(40, 0, 0);
		Vec3 y_axis = view_matrix.TransformVec3_0(0, 40, 0);
		Vec3 z_axis = view_matrix.TransformVec3_0(0, 0, 40);

		int compass_x = width - 50;
		int compass_y = height - 50;

		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_CULL_FACE);
		glDisable(GL_BLEND);
		glDisable(GL_TEXTURE_2D);

		glBegin(GL_LINES);

		glColor3f(1, 0, 0);
		glVertex2i(compass_x, compass_y);
		glVertex2i(compass_x + (int)x_axis.x, compass_y - (int)x_axis.y);

		glColor3f(0, 1, 0);
		glVertex2i(compass_x, compass_y);
		glVertex2i(compass_x + (int)y_axis.x, compass_y - (int)y_axis.y);

		glColor3f(0, 0, 1);
		glVertex2i(compass_x, compass_y);
		glVertex2i(compass_x + (int)z_axis.x, compass_y - (int)z_axis.y);

		glEnd();
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

	void TestGame::ShowChapterText(const string& text, const string& subtitle, float fade_time)
	{
		chapter_text = text;
		chapter_sub_text = subtitle;
		chapter_text_start = total_game_time;
		chapter_text_end = total_game_time + fade_time;
	}

	float TestGame::GetTerrainHeight(float x, float z)
	{
		struct MyRayCallback : public RayCallback
		{
			float max_y;
			MyRayCallback() : max_y(0.0f) { }

			bool OnCollision(RayResult& rr)
			{
				if(Entity* entity = rr.body->GetUserEntity())
					if(StaticLevelGeometry* slg = dynamic_cast<StaticLevelGeometry*>(entity))
					{
						float y = rr.pos.y;
						if(y > max_y)
							max_y = y;

						return true;
					}

				return false;
			}
		} ray_callback;

		physics_world->RayTest(Vec3(x, 1000, z), Vec3(x, 0, z), ray_callback);

		return ray_callback.max_y;
	}

	void TestGame::InnerDispose()
	{
		load_status.Dispose();

		if(imp) { delete imp; imp = NULL; }

		if(hud) { delete hud; hud = NULL; }

		if(nav_graph) { NavGraph::DeleteNavGraph(nav_graph); nav_graph = 0; }

		sound_system->StopAll();

		DebugDrawMaterial::GetDebugDrawMaterial()->EmptyRecycleBin();

		GameState::InnerDispose();
	}

	int gs_spawnBot(lua_State* L);
	int gs_spawnArtilleryBug(lua_State* L);
	int gs_spawnPlayer(lua_State* L);
	int gs_spawnRubbish(lua_State* L);
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
	int gs_setSimulationSpeed(lua_State* L);

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
		lua_pushcclosure(L, gs_spawnRubbish, 1);
		lua_setfield(L, 1, "spawnRubbish");

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

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_setSimulationSpeed, 1);
		lua_setfield(L, 1, "setSimulationSpeed");
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

	int gs_spawnRubbish(lua_State* L)
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

				gs->SpawnRubbish(*vec);

				return 0;
			}
		}

		Debug("gs.spawnRubbish takes exactly 1 argument, a position vector; returning nil\n");
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

			struct : EntityQualifier { bool Accept(Entity* ent) { return dynamic_cast<Dood*>(ent) != NULL && ((Dood*)ent)->alive; } } doods_only;
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

			gs->god_mode = lua_toboolean(L, 1) != 0;

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

			gs->nav_editor = lua_toboolean(L, 1) != 0;

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

			gs->debug_draw = lua_toboolean(L, 1) != 0;

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

	bool MaybeCreateEdge(const Vec3& delta, unsigned int graph, unsigned int my_node, unsigned int other_node)
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

	void MaybeCreateEdge(TestGame* game, Vec3& my_pos, unsigned int graph, unsigned int my_node, const vector<unsigned int>& other_column)
	{
		for(vector<unsigned int>::const_iterator iter = other_column.begin(); iter != other_column.end(); ++iter)
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

		struct MyRayCallback : public RayCallback
		{
			list<float>& results;
			MyRayCallback(list<float>& results) : results(results) { }

			bool OnCollision(RayResult& rr)
			{
				if(Entity* entity = rr.body->GetUserEntity())
					if(StaticLevelGeometry* slg = dynamic_cast<StaticLevelGeometry*>(entity))
						results.push_back(rr.pos.y);
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

	int gs_setSimulationSpeed(lua_State* L)
	{
		int n = lua_gettop(L);
		if(n == 1)
		{
			TestGame* gs = (TestGame*)lua_touserdata(L, lua_upvalueindex(1));

			gs->sim_speed = (float)lua_tonumber(L, 1);

			lua_settop(L, 0);
			return 0;
		}

		Debug("gs.setSimulationSpeed takes exactly one parameter, a number\n");
		return 0;
	}
}
