#include "StdAfx.h"
#include "AIController.h"

#include "Dood.h"
#include "WeaponEquip.h"

namespace Test
{
	

	/*
	 * AIController methods
	 */
	AIController::AIController(GameState* gs) : Controller(gs) { }

	void AIController::Update(TimingInfo time)
	{
		Dood* dood = (Dood*)GetControlledPawn();
		if(dood == NULL || !dood->is_valid)
		{
			is_valid = false;
			return;
		}

		/*
			ControlState& control_state = *GetControlState();

			struct : public EntityQualifier { bool Accept(Entity* ent) { return dynamic_cast<Dood*>(ent) != NULL; } } dood_getter;
			EntityList doods = game_state->GetQualifyingEntities(dood_getter);

			struct : public EntityQualifier { bool Accept(Entity* ent) { return dynamic_cast<PlayerController*>(((Dood*)ent)->controller) != NULL; } } enemy_getter; 
			EntityList opponents = doods.FindAll(enemy_getter);

			Vec3 forward = Vec3(-sin(dood->yaw), 0, cos(dood->yaw));
			Vec3 rightward = Vec3(-forward.z, 0, forward.x);

			Dood* target = NULL;
			float closest_dist = 0;

			for(unsigned int i = 0; i < opponents.Count(); i++)
			{
				Dood* opponent = (Dood*)opponents[i];
				float dist = (opponent->pos - dood->pos).ComputeMagnitude();
				if (target == NULL || dist < closest_dist)
				{
					target = opponent;
					closest_dist = dist;
				}
			}

			control_state[WeaponEquip::PrimaryFire] = false;
			control_state[Dood::Forward] = 0;
			if (target != NULL)
			{
				Vec3 to_target = target->pos - dood->pos;

				float dist = to_target.ComputeMagnitude();
				if (dist > 1.0)
				{
					float inv_dist = 1.0 / dist;
					Vec3 target_vel = target->vel - dood->vel;
					float lead_time = max(0.0f, Util::LeadTime(to_target, target_vel, 200));

					Vec3 target_dir = Vec3::Normalize(to_target + target_vel * lead_time);

					float rdot = Vec3::Dot(target_dir, rightward);
					float fdot = Vec3::Dot(target_dir, forward);
					control_state[Dood::Yaw] = control_state[Dood::Yaw] + atan2(rdot, fdot) ;
					control_state[Dood::Pitch] = control_state[Dood::Pitch] + asin(-to_target.y * inv_dist) - dood->pitch;

					bool can_see = CanSee(target);
					if(can_see && dist < 2)
						control_state[WeaponEquip::PrimaryFire] = can_see;

					control_state[Dood::Forward] = can_see ? 1 : 0;
				}
				else
					control_state[Dood::Forward] = -1;
			}
		*/

		Controller::Update(time);
	}

	bool AIController::CanSee(Dood* target)
	{
		// TODO: base this on actual line-of-sight visibility
		return true;
	}
}
