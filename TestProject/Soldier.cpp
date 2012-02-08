#include "StdAfx.h"
#include "Soldier.h"

#include "PoseAimingGun.h"
#include "WeaponEquip.h"

namespace Test
{
	/*
	 * Soldier constants
	 */ 
	float jump_pack_accel = 15.0f;

	float jump_to_fly_delay = 0.3f;
	float jump_speed = 4.0f;

	float jump_fuel_spend_rate = 0.5f, jump_fuel_refill_rate = 0.4f;
	float flying_accel = 8.0f;



	
	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, Vec3 pos, Team& team) :
		Dood(game_state, model, pos, team),
		gun_hand_bone(NULL),
		p_ag(NULL),
		jump_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
		p_ag = new PoseAimingGun();
		character->active_poses.push_back(p_ag);

		// figure out which bones is the gun-holding bone
		for(vector<Bone*>::iterator iter = character->skeleton->bones.begin(); iter != character->skeleton->bones.end(); ++iter)
			if((*iter)->name == Bone::string_table["r grip"])
				gun_hand_bone = *iter;

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound = sound_cache->Load("jet_loop");
	}

	void Soldier::InnerDispose()
	{
		delete p_ag;
		p_ag = NULL;

		Dood::InnerDispose();
	}

	void Soldier::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		float timestep = time.elapsed;

		bool can_recharge = true;
		bool jetted = false;
		if (control_state->GetBoolControl("jump"))
		{
			if (standing > 0)
			{
				//jump off the ground
				rigid_body->ApplyCentralImpulse(Vec3(0, jump_speed * mass, 0));
				jump_start_timer = time.total + jump_to_fly_delay;
			}
			else
			{
				can_recharge = false;

				if (jump_fuel > 0)
				{
					// jetpacking
					if (time.total > jump_start_timer)
					{
						jetted = true;

						if(jet_loop == NULL)
						{
							PlayDoodSound(jet_start_sound, 5.0f, false);
							jet_loop = PlayDoodSound(jet_loop_sound, 1.0f, true);
						}
						else
						{
							jet_loop->pos = pos;
							jet_loop->vel = vel;
						}

						jump_fuel -= timestep * (jump_fuel_spend_rate);

						Vec3 jump_accel_vec = Vec3(0, jump_pack_accel, 0);
						Vec3 lateral_accel = forward * max(-1.0f, min(1.0f, control_state->GetFloatControl("forward"))) + rightward * max(-1.0f, min(1.0f, control_state->GetFloatControl("sidestep")));
						jump_accel_vec += lateral_accel * (flying_accel);

						Vec3 jump_force = jump_accel_vec * mass;

						rigid_body->ApplyCentralForce(jump_force);
					}
				}
				else
				{
					// out of fuel! flash hud gauge if it's relevant
					JumpFailureEvent evt = JumpFailureEvent(this);
					OnJumpFailure(&evt);
				}
			}
		}
		
		if(!jetted && jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		if (can_recharge)
			jump_fuel = min(jump_fuel + jump_fuel_refill_rate * timestep, 1.0f);
	}

	void Soldier::DoWeaponControls(TimingInfo time)
	{
		p_ag->pos = pos;
		p_ag->yaw = yaw;
		p_ag->pitch = pitch;

		Dood::DoWeaponControls(time);
	}

	void Soldier::PreUpdatePoses(TimingInfo time)
	{
		p_ag->pos = pos;
		p_ag->yaw = yaw;
		p_ag->pitch = pitch;
	}

	void Soldier::PostUpdatePoses(TimingInfo time)
	{
		if(equipped_weapon != NULL && gun_hand_bone != NULL)
		{
			equipped_weapon->gun_xform = Mat4::Translation(pos) * gun_hand_bone->GetTransformationMatrix() * Mat4::Translation(gun_hand_bone->rest_pos) /* * Mat4::Translation(-0.3, 1.3, 0.08) * Mat4::FromQuaternion(Quaternion::FromPYR(0, 0, -0.75) * Quaternion::FromPYR(1.5, 0.0, 0.0) * Quaternion::FromPYR(0, 0.1, 0) * Quaternion::FromPYR(0.1, 0, 0)) * Mat4::Translation(0, 0.05, 0.35) */;
			equipped_weapon->sound_pos = equipped_weapon->pos = equipped_weapon->gun_xform.TransformVec3(0, 0, 0, 1);
			equipped_weapon->sound_vel = equipped_weapon->vel = vel;
		}
	}

	void Soldier::Update(TimingInfo time) { Dood::Update(time); }
}
