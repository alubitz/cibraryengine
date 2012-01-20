#include "StdAfx.h"
#include "CrabBug.h"

namespace Test
{
	using namespace std;

	/*
	 * CrabBug constants
	 */
	float bug_leap_duration = 0.5f;
	float bug_leap_speed = 8.0f;




	static bool GetBoolControl(Dood* dood, string control_name) { return dood->control_state->GetBoolControl(control_name); }
	static float GetFloatControl(Dood* dood, string control_name) { return dood->control_state->GetFloatControl(control_name); }
	static void SetBoolControl(Dood* dood, string control_name, bool value) { dood->control_state->SetBoolControl(control_name, value); }
	static void SetFloatControl(Dood* dood, string control_name, float value) { dood->control_state->SetFloatControl(control_name, value); }

	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, UberModel* model, Vec3 pos, Team& team) :
		Dood(game_state, model, pos, team),
		walk_pose()
	{
		hp *= 0.3f;

		character->active_poses.push_back(&walk_pose);
	}

	void CrabBug::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		if (standing > 0 && GetBoolControl(this, "leap") && time.total > jump_start_timer)
		{
			// crab bug leaps forward
			float leap_angle = 0.4f;
			Vec3 leap_vector = (forward * (cosf(leap_angle)) + Vec3(0, sinf(leap_angle), 0)) * (mass * bug_leap_speed);
			rigid_body->body->applyCentralImpulse(btVector3(leap_vector.x, leap_vector.y, leap_vector.z));

			jump_start_timer = time.total + bug_leap_duration;
		}
	}

	void CrabBug::PreUpdatePoses(TimingInfo time)
	{
		walk_pose.pos = pos;
		walk_pose.yaw = yaw;
	}
}
