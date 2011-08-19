#include "StdAfx.h"
#include "HUD.h"

#include "Dood.h"
#include "WeaponEquip.h"

#include "TestGame.h"

namespace Test
{
	/*
	 * HUD methods
	 */
	HUD::HUD(TestGame* game, ContentMan* content) :
		game(game),
		player(NULL),
		enable_radar(true),
		hud_flash_timer(0),
		no_jump_flash_timer(0),
		no_ammo_flash_timer(0),
		low_hp_dim(false),
		no_ammo_dim(false),
		low_ammo_dim(false),
		no_jump_dim(false),
		low_jump_dim(false),
		render_target(NULL),
		directional_damage(),
		OnAmmoFailure(this),
		OnDamageTaken(this),
		OnJumpFailure(this)
	{
		reticle_tex = game->tex2d_cache->Load("reticle");
		hud_splat_tex = game->tex2d_cache->Load("hud_splat");
		healthbar_tex = game->tex2d_cache->Load("healthbar");
		ammogauge_tex = game->tex2d_cache->Load("ammogauge");
		jumpbar_tex = game->tex2d_cache->Load("jumpbar");

		for(int i = 0; i < 8; i++)
			directional_damage.push_back(0.0);
	}

	HUD::~HUD()
	{
		Dispose();
	}

	void HUD::InnerDispose()
	{
		if(player != NULL)
		{
			player->OnAmmoFailure -= &OnAmmoFailure;
			player->OnDamageTaken -= &OnDamageTaken;
			player->OnJumpFailure -= &OnJumpFailure;
		}

		if(render_target != NULL)
		{
			render_target->Dispose();
			delete render_target;
		}
	}

	void HUD::SetOrtho(float w, float h)
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, w, h, 0, -1, 1);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	void HUD::Print(float x, float y, string str) { Print(x, y, str, Vec4(1.0, 1.0, 1.0, 1.0)); }
	void HUD::Print(float x, float y, string str, Vec4 color)
	{
		glColor4f(color.x, color.y, color.z, color.w);
		game->font->Print(str, x, y);
	}

	void HUD::DrawHealthGauge(float w, float h)
	{
		glPushMatrix();

		glTranslatef(50, 50, 0);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, healthbar_tex->GetGLName());

		float health = player->hp;																// already is a fraction out of 1.0
		float unhealth = 1.0f - health;

		float red = min(2.0f - 2.0f * health, 1.0f);
		float green = min(2.0f * health, 1.0f);

		int dx = (int)(unhealth * (191));

		int min_x = 4;
		int max_x = 200 - dx;

		float min_u = (dx + 2) / 256.0f;
		float max_u = 198.0f / 256.0f;

		glBegin(GL_QUADS);

		// the bar itself
		if(low_hp_dim)
			glColor4f(1, 0, 0, 1);
		else
			glColor4f(red * 0.5f + 0.5f, green * 0.5f + 0.5f, 0, 1);
		glTexCoord2f(min_u, 0.5f);
		glVertex2f((float)min_x, 0);
		glTexCoord2f(min_u, 1);
		glVertex2f((float)min_x, 32);
		glTexCoord2f(max_u, 1);
		glVertex2f((float)max_x, 32);
		glTexCoord2f(max_u, 0.5f);
		glVertex2f((float)max_x, 0);

		// the frame and label
		glColor4f(0.5f, 0.5f, 1.0f, 1.0f);
		glTexCoord2f(0, 0);
		glVertex2f(2, 0);
		glTexCoord2f(0, 0.5f);
		glVertex2f(2, 32);
		glTexCoord2f(1, 0.5f);
		glVertex2f(258, 32);
		glTexCoord2f(1, 0);
		glVertex2f(258, 0);

		glEnd();

		glPopMatrix();
	}

	void HUD::DrawAmmoGauge(float w, float h)
	{
		// don't try to draw this stuff if the player hasn't got a weapon
		float ammo_frac;
		if(!player->GetAmmoFraction(ammo_frac))
			return;

		glPushMatrix();

		glTranslatef(50, 82, 0);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, ammogauge_tex->GetGLName());

		float unammo = 1.0f - ammo_frac;

		float red = min(2.0f - 2.0f * ammo_frac, 1.0f);
		float green = min(2.0f * ammo_frac, 1.0f);

		int dx = (int)(unammo * (148));

		int min_x = 47;
		int max_x = 200 - dx;

		float min_u = (dx + 45) / 256.0f;
		float max_u = 198.0f / 256.0f;

		glBegin(GL_QUADS);

		if (ammo_frac > 0)
		{
			no_ammo_dim = false;
			no_ammo_flash_timer = 0.0;

			// the bar itself
			if(low_ammo_dim)
				glColor4f(1, 0, 0, 1);
			else
				glColor4f(red * 0.5f + 0.5f, green * 0.5f + 0.5f, 0, 1);
			glTexCoord2f(min_u, 0.5f);
			glVertex2f((float)min_x, 0);
			glTexCoord2f(min_u, 1);
			glVertex2f((float)min_x, 32);
			glTexCoord2f(max_u, 1);
			glVertex2f((float)max_x, 32);
			glTexCoord2f(max_u, 0.5f);
			glVertex2f((float)max_x, 0);
		}

		// the frame and label
		if(no_ammo_dim)
			glColor4f(1, 0, 0, 1);
		else
			glColor4f(0.5f, 0.5f, 1, 1);
		glTexCoord2f(0, 0);
		glVertex2f(2, 0);
		glTexCoord2f(0, 0.5f);
		glVertex2f(2, 32);
		glTexCoord2f(1, 0.5F);
		glVertex2f(258, 32);
		glTexCoord2f(1, 0);
		glVertex2f(258, 0);

		glEnd();

		int ammo;
		if(player->GetAmmoCount(ammo))
		{
			stringstream ss;
			ss << ammo;
			string ammo_string = ss.str();
			while(ammo_string.length() < 3)
				ammo_string = "0" + ammo_string;
			Print(50 + 4, 82 + 4, ammo_string, no_ammo_dim ? Vec4(1, 0, 0, 1) : Vec4(1, 1, 1, 1)); 
		}

		glPopMatrix();
	}

	void HUD::DrawJumpGauge(float w, float h)
	{
		glPushMatrix();

		glTranslatef(50, 114, 0);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, jumpbar_tex->GetGLName());

		float jump_frac = player->jump_fuel;
		float unjump = 1.0f - jump_frac;

		float red = min(2.0f - 2.0f * jump_frac, 1.0f);
		float green = min(2.0f * jump_frac, 1.0f);

		int dx = (int)(unjump * (191));

		int min_x = 4;
		int max_x = 200 - dx;

		float min_u = (dx + 2) / 256.0f;
		float max_u = 198.0f / 256.0f;


		glBegin(GL_QUADS);

		if (player->jump_fuel > 0)
		{
			no_jump_dim = false;
			no_jump_flash_timer = 0.0f;

			// the bar itself
			if(low_jump_dim)
				glColor4f(1, 0, 0, 1);
			else
				glColor4f(red * 0.5f + 0.5f, green * 0.5f + 0.5f, 0, 1);
			glTexCoord2f(min_u, 0.5f);
			glVertex2f((float)min_x, 0);
			glTexCoord2f(min_u, 1);
			glVertex2f((float)min_x, 32);
			glTexCoord2f(max_u, 1);
			glVertex2f((float)max_x, 32);
			glTexCoord2f(max_u, 0.5f);
			glVertex2f((float)max_x, 0);
		}

		// the frame and label
		if(no_jump_dim)
			glColor4f(1, 0, 0, 1);
		else
			glColor4f(0.5f, 0.5f, 1, 1);
		glTexCoord2f(0, 0);
		glVertex2f(2, 0);
		glTexCoord2f(0, 0.5f);
		glVertex2f(2, 32);
		glTexCoord2f(1, 0.5f);
		glVertex2f(258, 32);
		glTexCoord2f(1, 0);
		glVertex2f(258, 0);

		glEnd();

		glPopMatrix();
	}

	void HUD::DrawRadar(float w, float h)
	{
		// Transform to radar coords

		glTranslatef(w - 200, 200, 0);
		glScalef(195, 195, 1);

		glDisable(GL_TEXTURE_2D);

		// Draw radar backdrop

		glColor4f(0.0f, 0.0f, 0.0f, 0.35f);

		glBegin(GL_TRIANGLE_FAN);

		glVertex2f(0, 0);

		int ring_steps = 128;
		float ring_coeff = M_PI * 2.0f / ring_steps;
		for (int i = 0; i <= ring_steps; i++)
		{
			float theta = i * ring_coeff;
			glVertex2f(cos(theta), sin(theta));
		}
		glEnd();

		// Shade the view cone

		glColor4f(1.0f, 1.0f, 1.0f, 0.1f);
		glBegin(GL_TRIANGLE_FAN);

		glVertex2f(0, 0);

		int fan_steps = 32;
		float fan_coeff = M_PI * 0.25f / fan_steps;
		for (int i = -fan_steps; i <= fan_steps; i++)
		{
			float theta = i * fan_coeff + player->yaw + M_PI * 0.5f;
			glVertex2f(cosf(theta), sinf(theta));
		}

		glEnd();

		// Draw target blips

		float radar_size = 50, inv_radar_scale = 1.0f / radar_size;

		glEnable(GL_POINT_SMOOTH);
		glPointSize(10);
		glBegin(GL_POINTS);

		struct : EntityQualifier { bool Accept(Entity* ent) { return (dynamic_cast<Dood*>(ent)) != NULL; } } predicate;
		EntityList doods = game->GetQualifyingEntities(predicate);
		for(unsigned int i = 0; i < doods.Count(); i++)
		{
			Dood* dood = (Dood*)doods[i];

			if (dood != player)
				glColor4f(1, 0, 0, 0.5);
			else
				glColor4f(0.5, 0.5, 1, 0.5);

			float dx = dood->pos.x - player->pos.x, dy = dood->pos.z - player->pos.z;
			dx *= inv_radar_scale;
			dy *= inv_radar_scale;
			float dist = sqrtf(dx * dx + dy * dy);

			if (dist > 1.0f)
			{
				float inv_dist = 1.0f / dist;
				dx *= inv_dist;
				dy *= inv_dist;
			}

			glVertex2f(dx, dy);
		}

		glEnd();

		// Draw radar hoop

		glLineWidth(2);

		glColor4f(0.5, 0.5, 1.0, 0.5);

		glBegin(GL_LINE_LOOP);
		for (int i = 0; i < ring_steps; i++)
		{
			float theta = i * ring_coeff;
			glVertex2f(cos(theta), sin(theta));
		}
		glEnd();
	}

	void HUD::DrawDamageIndicator(float w, float h)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, hud_splat_tex->GetGLName());

		glBegin(GL_TRIANGLE_FAN);

		float br = 0;

		// center
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(0.5, 0.5);
		glVertex2f(w / 2, h / 2);

		// now starting with top left, going clockwise
		br = directional_damage[0];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(0, 0);
		glVertex2f(0, 0);

		br = directional_damage[1];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(0.5, 0);
		glVertex2f(w / 2, 0);

		br = directional_damage[2];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(1, 0);
		glVertex2f(w, 0);

		br = directional_damage[3];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(1, 0.5);
		glVertex2f(w, h / 2);

		br = directional_damage[4];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(1, 1);
		glVertex2f(w, h);

		br = directional_damage[5];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(0.5, 1);
		glVertex2f(w / 2, h);

		br = directional_damage[6];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(0.0, 1);
		glVertex2f(0, h);

		br = directional_damage[7];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(0.0, 0.5);
		glVertex2f(0, h / 2);

		br = directional_damage[0];
		glColor4f(1.0, 1.0, 1.0, br);
		glTexCoord2f(0, 0);
		glVertex2f(0, 0);

		glEnd();
	}

	void HUD::Draw(float w, float h)
	{
		/*
		if(render_target != NULL && (render_target->GetWidth() != w || render_target->GetHeight() != h))
		{
			render_target->Dispose();
			delete render_target;
			render_target = NULL;
		}
		if(render_target == NULL)
			render_target = new RenderTarget(w, h, 0);

		RenderTarget::Bind(render_target);
		*/

		glColor4f(1.0, 1.0, 1.0, 1.0);
		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_CULL_FACE);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		SetOrtho(w, h);
		glEnable(GL_TEXTURE_2D);

		// Drawing the reticle

		if (player->hp > 0)
		{
			glBindTexture(GL_TEXTURE_2D, reticle_tex->GetGLName());
			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);
			glVertex2f(w / 2 - 64, h / 2 - 64);
			glTexCoord2f(0, 1);
			glVertex2f(w / 2 - 64, h / 2 + 63);
			glTexCoord2f(1, 1);
			glVertex2f(w / 2 + 64, h / 2 + 63);
			glTexCoord2f(1, 0);
			glVertex2f(w / 2 + 64, h / 2 - 64);
			glEnd();

			DrawHealthGauge(w, h);
			DrawAmmoGauge(w, h);
			DrawJumpGauge(w, h);

			if(enable_radar)
				DrawRadar(w, h);                    // radar draw is too complicated to just stick in loose with the rest of the HUD code
		}

		SetOrtho(w, h);

		DrawDamageIndicator(w, h);

		SetOrtho(w, h);

		if (game->debug_text != "")
			Print(0, 0, game->debug_text);

		if(game->total_game_time < game->chapter_text_end || game->chapter_text_end < game->chapter_text_start)
		{
			float frac = game->chapter_text_end < game->chapter_text_start ? 1.0f : (game->chapter_text_end - game->total_game_time) / (game->chapter_text_end - game->chapter_text_start);
			SetOrtho(w / 4, h / 4);
			Print((w / 4 - game->chapter_text.length() * game->font->font_spacing) / 2, h / 8 - game->font->font_height, game->chapter_text, Vec4(1, 1, 1, frac));
			SetOrtho(w, h);

			string lines = game->chapter_sub_text;
			string line;
			int row = 0;
			while(lines.length() > 0)
			{
				int index = lines.find('\n');
				if(index == -1)
				{
					line = lines;
					lines = "";
				}
				else
				{
					line = lines.substr(0, index);
					lines = lines.substr(index + 1);
				}
				if(line.length() > 0)
					Print((w - line.length() * game->font->font_spacing) / 2, h / 2 + row * game->font->font_height, line);

				row++;
			};
		}

		RenderTarget::Bind(NULL);
	}

	void HUD::SetPlayer(Dood* dood)
	{
		if(player != NULL)
		{
			player->OnAmmoFailure -= &OnAmmoFailure;
			player->OnDamageTaken -= &OnDamageTaken;
			player->OnJumpFailure -= &OnJumpFailure;
		}

		player = dood;

		if(player != NULL)
		{
			player->OnAmmoFailure += &OnAmmoFailure;
			player->OnDamageTaken += &OnDamageTaken;
			player->OnJumpFailure += &OnJumpFailure;
		}
	}

	void HUD::UpdateHUDGauges(TimingInfo time)
	{
		float timestep = time.elapsed;
		hud_flash_timer -= timestep;
		if (hud_flash_timer < 0)
		{
			no_ammo_flash_timer -= timestep;
			no_jump_flash_timer -= timestep;

			if (player->hp < 0.1)
				low_hp_dim = !low_hp_dim;
			else
				low_hp_dim = false;

			if (no_ammo_flash_timer > 0)
				no_ammo_dim = !no_ammo_dim;
			else
				no_ammo_dim = false;

			if (no_jump_flash_timer > 0)
				no_jump_dim = !no_jump_dim;
			else
				no_jump_dim = false;

			float ammo_frac;
			if (player->GetAmmoFraction(ammo_frac) && ammo_frac < 0.2)
				low_ammo_dim = !low_ammo_dim;
			else
				low_ammo_dim = false;

			if (player->jump_fuel < 0.25f)
				low_jump_dim = !low_jump_dim;
			else
				low_jump_dim = false;

			hud_flash_timer = 0.08f;
		}

		for (unsigned int i = 0; i < directional_damage.size(); i++)
			directional_damage[i] = max(0.0f, directional_damage[i] - timestep);
	}




	/*
	 * HUD::AmmoFailureCallback methods
	 */
	void HUD::AmmoFailureCallback::HandleEvent(Event* evt) { hud->no_ammo_flash_timer = 0.1f; }




	/*
	 * HUD::DamageTakenCallback methods
	 */
	void HUD::DamageTakenCallback::HandleEvent(Event* evt)
	{
		//((Dood::DamageTakenEvent*)evt)->cancel = true;					// uncomment for god mode

		Vec3 from_direction = ((Dood::DamageTakenEvent*)evt)->from_dir;

		float mag = from_direction.ComputeMagnitude();
		if (mag == 0)
			for (int i = 0; i < 8; i++)
				hud->directional_damage[i] = 1.0;
		else
		{
			from_direction = from_direction / mag;

			Vec3 forward = Vec3(-sin(hud->player->yaw), 0, cos(hud->player->yaw));
			Vec3 rightward = Vec3(-forward.z, 0, forward.x);

			for (int i = 0; i < 8; i++)
			{
				float theta = (i + 5) * M_PI * 2.0f / 8.0f;           // 8 is the number of steps in our "circle"
				Vec3 vec = rightward * cosf(theta) - forward * sinf(theta);
				float dot = Vec3::Dot(vec, from_direction);
				hud->directional_damage[i] = max(hud->directional_damage[i], dot);
			}
		}
	}




	/*
	 * HUD::JumpFailureCallback methods
	 */
	void HUD::JumpFailureCallback ::HandleEvent(Event* evt) { hud->no_jump_flash_timer = 0.2f; }
}
