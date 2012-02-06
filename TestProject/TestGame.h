#pragma once

#include "StdAfx.h"

#include "Team.h"

namespace Test
{
	using namespace CibraryEngine;

	class TestScreen;
	class HUD;
	class Dood;
	class Soldier;
	class DSNMaterial;
	class GlowyModelMaterial;
	class Sun;

	class TestGame : public GameState
	{
		private:

			TestScreen* screen;

			class DoodDeathHandler : public EventHandler
			{
				public:
					TestGame* game;
					DoodDeathHandler(TestGame* game);

					void HandleEvent(Event* evt);
			} bot_death_handler, player_death_handler;

			class PlayerDamageHandler : public EventHandler
			{
				public:
					TestGame* game;
					PlayerDamageHandler(TestGame* game);

					void HandleEvent(Event* evt);
			} player_damage_handler;

		protected:

			void InnerDispose();

		public:

			static Team human_team;
			static Team bug_team;

			float total_game_time;

			float chapter_text_start, chapter_text_end;
			string chapter_text, chapter_sub_text;

			bool nav_editor;
			bool god_mode;
			bool debug_draw;
			bool alive;

			BitmapFont* font;
			UberModel* model;
			UberModel* enemy;

			UberModel* gun_model;
			VertexBuffer* mflash_model;
			VertexBuffer* shot_model;
			GlowyModelMaterial* mflash_material;
			BillboardMaterial* shot_material;
			BillboardMaterial* blood_billboard;

			ParticleMaterial* dirt_particle;

			VertexBuffer* sky_sphere;
			TextureCube* sky_texture;
			ShaderProgram* sky_shader;

			TextureCube* ambient_cubemap;
			RenderTarget* render_target;
			RenderTarget* shadow_render_target;

			ShaderProgram* deferred_ambient;
			ShaderProgram* deferred_lighting;

			SoundBuffer* fire_sound;
			SoundBuffer* chamber_click_sound;
			SoundBuffer* reload_sound;

			int width, height;

			HUD* hud;
			Sun* sun;

			ScriptedController* player_controller;
			Soldier* player_pawn;

			string debug_text;

			unsigned int nav_graph;

			Cache<Texture2D>* tex2d_cache;
			Cache<VertexBuffer>* vtn_cache;
			Cache<UberModel>* ubermodel_cache;
			Cache<Material>* mat_cache;

			struct Loader
			{
				private:

					bool abort;
					bool stopped;

					boost::mutex* mutex;

				public:

					TestGame* game;

					string task;

					Loader(TestGame* game) : abort(false), stopped(false), mutex(new boost::mutex()), game(game), task() { }
					~Loader() { delete mutex; mutex = NULL; }

					void operator ()();

					bool HasStopped();
					bool HasAborted();
					void Stop();
					void Abort();
			} load_status;

			TestGame(TestScreen* screen, SoundSystem* sound_system);
			~TestGame();

			void Load();

			Soldier* SpawnPlayer(Vec3 pos);
			Dood* SpawnBot(Vec3 pos);
			Dood* SpawnArtilleryBug(Vec3 pos);
			unsigned int GetNumberOfBugs();

			void Update(TimingInfo time);

			// Drawing-related functions...
			void Draw(int width, int height);
			void DrawBackground(Mat4 view_matrix);
			// Functions for drawing alternate display modes
			void DrawPhysicsDebuggingInfo(SceneRenderer* renderer);
			void DrawNavEditorInfo(SceneRenderer* renderer);
			// Call this from within an object's Vis function to draw an UberModel
			void VisUberModel(SceneRenderer* renderer, UberModel* model, int lod, Mat4 xform, SkinnedCharacter* character = NULL,  vector<Material*>* materials = NULL);

			void ShowChapterText(string title, string subtitle, float duration);

			float GetTerrainHeight(float x, float z);

			void SetupScripting(ScriptingState& state);
	};
}