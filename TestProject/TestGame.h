#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class TestScreen;
	class HUD;
	class Dood;
	class DSNMaterial;
	class GlowyModelMaterial;
	class Sun;

	class TestGame : public GameState
	{
		private:

			TestScreen* screen;

			Dood* SpawnDood(Vec3 pos, UberModel* model);

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
			GlowyModelMaterial* shot_material;

			ParticleMaterial* blood_particle;
			ParticleMaterial* dirt_particle;

			VertexBuffer* sky_sphere;
			TextureCube* sky_texture;
			ShaderProgram* sky_shader;

			TextureCube* ambient_cubemap;
			RenderTarget* render_target;
			Texture2D* rtt_diffuse;
			Texture2D* rtt_normal;
			Texture2D* rtt_specular;
			Texture2D* rtt_depth;
			ShaderProgram* deferred_ambient;
			ShaderProgram* deferred_lighting;

			SoundBuffer* fire_sound;
			SoundBuffer* chamber_click_sound;
			SoundBuffer* reload_sound;

			int width, height;

			HUD* hud;
			Sun* sun;

			PlayerController* player_controller;
			Dood* player_pawn;
			IKSolver* ik_solver;

			string debug_text;

			unsigned int nav_graph;

			Cache<Texture2D>* tex2d_cache;
			Cache<VertexBuffer>* vtn_cache;
			Cache<UberModel>* ubermodel_cache;
			Cache<Material>* mat_cache;

			struct Loader
			{
				TestGame* game;

				unsigned int task_id;

				bool abort;
				bool stopped;

				string task;
				float progress;

				Loader(TestGame* game) : game(game), task_id(0), abort(false), stopped(false), task(), progress(0) { }

				void operator ()();
			} load_status;

			TestGame(TestScreen* screen, SoundSystem* sound_system);
			~TestGame();

			void Load();

			Dood* SpawnPlayer(Vec3 pos);
			Dood* SpawnBot(Vec3 pos);
			unsigned int GetNumberOfBots();

			void Update(TimingInfo time);
			void Draw(int width, int height);

			void DrawBackground(Mat4 view_matrix);
			void DrawPhysicsDebuggingInfo(SceneRenderer* renderer);
			void DrawNavEditorInfo(SceneRenderer* renderer);
			void VisUberModel(SceneRenderer* renderer, UberModel* model, int lod, Mat4 xform, SkinnedCharacter* character = NULL,  vector<Material*>* materials = NULL);

			void ShowChapterText(string title, string subtitle, float duration);

			float GetTerrainHeight(float x, float z);

			void SetupScripting(ScriptingState& state);
	};
}