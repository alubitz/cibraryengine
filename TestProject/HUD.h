#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class TestGame;
	class Dood;

	class HUD : public Disposable
	{
		private:

			TestGame* game;
			Dood* player;

			bool enable_radar;
			bool disable_all;

			Texture2D* reticle_tex;
			Texture2D* hud_splat_tex;

			Texture2D* healthbar_tex;
			Texture2D* ammogauge_tex;
			Texture2D* jumpbar_tex;

			float hud_flash_timer;
			float no_jump_flash_timer;
			float no_ammo_flash_timer;

			bool low_hp_dim;
			bool no_ammo_dim;
			bool low_ammo_dim;
			bool no_jump_dim;
			bool low_jump_dim;

			RenderTarget* render_target;

			vector<float> directional_damage;

			void SetOrtho(float w, float h);

			void Print(float x, float y, const string& str);
			void Print(float x, float y, const string& str, const Vec4& color);

			void DrawHealthGauge(float w, float h);
			void DrawAmmoGauge(float w, float h);
			void DrawJumpGauge(float w, float h);
			void DrawRadar(float w, float h);
			void DrawDamageIndicator(float w, float h);

			class AmmoFailureCallback : public EventHandler
			{
				public:
					HUD* hud;

					AmmoFailureCallback() : hud(NULL) { }

					void HandleEvent(Event* evt);
			} OnAmmoFailure;

			class DamageTakenCallback : public EventHandler
			{
				public:
					HUD* hud;

					DamageTakenCallback() : hud(NULL) { }

					void HandleEvent(Event* evt);
			} OnDamageTaken;

			class JumpFailureCallback : public EventHandler
			{
				public:
					HUD* hud;

					JumpFailureCallback() : hud(NULL) { }

					void HandleEvent(Event* evt);
			} OnJumpFailure;

		protected:

			void InnerDispose();

		public:

			HUD(TestGame* game, ContentMan* content);
			~HUD();

			void UpdateHUDGauges(const TimingInfo& time);
			void Draw(float w, float h);

			void SetPlayer(Dood* dood);
	};
}
