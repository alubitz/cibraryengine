#include "StdAfx.h"
#include "WalkPose.h"

#include "Dood.h"

namespace Test
{
	/*
	 * WalkPose methods
	 */
	WalkPose::WalkPose(Dood* dood, const KeyframeAnimation* rest_anim, const KeyframeAnimation* forward_anim, const KeyframeAnimation* backward_anim, const KeyframeAnimation* left_anim, const KeyframeAnimation* right_anim, const KeyframeAnimation* l_turn_anim, const KeyframeAnimation* r_turn_anim) :
		Pose(),
		dood(dood),
		anim_timer(0),
		forward_v (0),
		leftward_v(0),
		yaw_v     (0),
		rest_anim(     rest_anim     == NULL ? NULL : new KeyframeAnimation(*rest_anim    )),
		forward_anim(  forward_anim  == NULL ? NULL : new KeyframeAnimation(*forward_anim )),
		backward_anim( backward_anim == NULL ? NULL : new KeyframeAnimation(*backward_anim)),
		left_anim(     left_anim     == NULL ? NULL : new KeyframeAnimation(*left_anim    )),
		right_anim(    right_anim    == NULL ? NULL : new KeyframeAnimation(*right_anim   )),
		l_turn_anim(   l_turn_anim   == NULL ? NULL : new KeyframeAnimation(*l_turn_anim  )),
		r_turn_anim(   r_turn_anim   == NULL ? NULL : new KeyframeAnimation(*r_turn_anim  )),
		fwd_anim_rate     (1.0f),
		side_anim_rate    (1.0f),
		turn_anim_rate    (4.0f),
		desired_yaw_offset(0.0f),
		yaw_bone(0),
		yaw       (dood->yaw),
		target_yaw(dood->yaw),
		terrain_normal(0, 1, 0)
	{
	}

	WalkPose::~WalkPose()
	{
		if(rest_anim)     { delete rest_anim;     rest_anim     = NULL; }
		if(forward_anim)  { delete forward_anim;  forward_anim  = NULL; }
		if(backward_anim) { delete backward_anim; backward_anim = NULL; }
		if(left_anim)     { delete left_anim;     left_anim     = NULL; }
		if(right_anim)    { delete right_anim;    right_anim    = NULL; }
		if(l_turn_anim)   { delete l_turn_anim;   l_turn_anim   = NULL; }
		if(r_turn_anim)   { delete r_turn_anim;   r_turn_anim   = NULL; }
	}

	// TODO: prevent accidentally stepping when moving to rest animation
	void WalkPose::UpdatePose(const TimingInfo& time)
	{
		if(time.total < 0.1f)
			return;

		static const float vsmooth_exp_coeff    = 4.0f;
		static const float min_speed            = 0.9f;
		static const float playback_exp_coeff   = 0.25f;

		static const float max_standing_yaw     = 0.41f;						// soldier dood minimum horizontal range / 2; old anim system used 1.0f here
		static const float max_moving_yaw       = 0.1f;

		bones.clear();															// make sure we reset to the rest pose if we aren't doing anything

		Vec3 forward  = Vec3(-sinf(yaw),  0,  cosf(yaw));
		Vec3 leftward = Vec3( forward.z,  0, -forward.x);

		// find speeds along each of the dood's axes of movement
		Vec3 vel = dood->vel;
		float m_forward_v  = Vec3::Dot(vel, forward );							// measured speeds
		float m_leftward_v = Vec3::Dot(vel, leftward);


		// do yaw stuff
		float desired_yaw = dood->yaw + desired_yaw_offset;
		Vec3 desired_fwd  = Vec3(-sinf(desired_yaw),  0,  cosf(desired_yaw) );
		Vec3 desired_left = Vec3( desired_fwd.z,      0, -desired_fwd.x     );

		Vec3 target_fwd   = Vec3(-sinf(target_yaw),   0,  cosf(target_yaw)  );

		float desired_from_target = atan2f(Vec3::Dot(target_fwd, desired_left), Vec3::Dot(target_fwd, desired_fwd));
		if(fabs(desired_from_target) > max_standing_yaw || max(fabs(m_forward_v), fabs(m_leftward_v)) >= min_speed)
		{
			target_yaw += desired_from_target;
			target_fwd = Vec3(-sinf(target_yaw), 0, cosf(target_yaw));			// recompute!
		}
		float target_from_yaw = atan2f(Vec3::Dot(target_fwd, leftward), Vec3::Dot(target_fwd, forward));

		float m_yaw_v;															// measured (computed?) yaw speed
		if(fabs(target_from_yaw) > max_moving_yaw)
		{
			if(target_from_yaw > 0)
				m_yaw_v =  1.0f;
			else
				m_yaw_v = -1.0f;
		}
		else
			m_yaw_v = 0.0f;

		// rather than use instantaneous velocity values, approach the velocities gradually over time
		float old_coeff = expf(-vsmooth_exp_coeff * time.elapsed);
		float nu_coeff  = 1.0f - old_coeff;
		forward_v  = old_coeff * forward_v  + nu_coeff * m_forward_v;
		leftward_v = old_coeff * leftward_v + nu_coeff * m_leftward_v;
		yaw_v      = old_coeff * yaw_v      + nu_coeff * m_yaw_v;

		// figure out the fastest playback speed we have an animation for (used as master playback speed)
		float use_speed = 0.0f;
		if(forward_v  > 0 && forward_anim) { use_speed =     forward_v  * fwd_anim_rate;              } else if(forward_v  < 0 && backward_anim) { use_speed =     -forward_v  * fwd_anim_rate;              }
		if(leftward_v > 0 && left_anim   ) { use_speed = max(leftward_v * side_anim_rate, use_speed); } else if(leftward_v < 0 && right_anim   ) { use_speed = max(-leftward_v * side_anim_rate, use_speed); }
		if(yaw_v      > 0 && l_turn_anim ) { use_speed = max(yaw_v      * turn_anim_rate, use_speed); } else if(yaw_v      < 0 && r_turn_anim  ) { use_speed = max(-yaw_v      * turn_anim_rate, use_speed); }

		float forward_speed, leftward_speed, yaw_speed;
		if(use_speed < min_speed)			// if none of the animations are moving fast enough to warrant stepping, set all speeds to zero and reset the timer
		{
			use_speed = forward_speed = leftward_speed = yaw_speed = 0.0f;
			anim_timer = 0.0f;
		}
		else
		{
			forward_speed  = forward_v	* fwd_anim_rate;
			leftward_speed = leftward_v	* side_anim_rate;
			yaw_speed      = yaw_v		* turn_anim_rate;

			float dt = 1.0f - expf(-use_speed * playback_exp_coeff * time.elapsed);
			anim_timer += dt;
		}

		// negative coefficients are ignored (set to 0)
		struct AnimWithCoeff { KeyframeAnimation* anim; float coeff; AnimWithCoeff(KeyframeAnimation* anim, float coeff) : anim(anim), coeff(coeff) { } };
		AnimWithCoeff anims[] =
		{
			AnimWithCoeff(rest_anim,      1.0f - use_speed),
			AnimWithCoeff(forward_anim,   forward_speed   ),
			AnimWithCoeff(backward_anim, -forward_speed   ),
			AnimWithCoeff(left_anim,      leftward_speed  ),
			AnimWithCoeff(right_anim,    -leftward_speed  ),
			AnimWithCoeff(l_turn_anim,    yaw_speed       ),
			AnimWithCoeff(r_turn_anim,   -yaw_speed       )
		};
		int num_anims = sizeof(anims) / sizeof(AnimWithCoeff);

		float coeffs_total = 0.0f;
		for(int i = 0; i < num_anims; ++i)
		{
			float& coeff = anims[i].coeff;
			if(coeff > 0)
				coeffs_total += coeff;
			else
				coeff = 0;
		}

		// if the total is somehow zero, default to the rest anim
		if(coeffs_total == 0)
			coeffs_total = anims[0].coeff = 1.0f;
		float inv_total = 1.0f / coeffs_total;

		// doing a weighted average of the various animations
		boost::unordered_map<unsigned int, BoneInfluence> all_bones;

		for(int i = 0; i < num_anims; ++i)
			if(KeyframeAnimation* anim = anims[i].anim)
				if(float coeff = anims[i].coeff * inv_total)
				{
					anim->JumpToTime(anim_timer);
					anim->UpdatePose(TimingInfo(0, anim_timer));

					for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = anim->bones.begin(); iter != anim->bones.end(); ++iter)
					{
						boost::unordered_map<unsigned int, BoneInfluence>::iterator found = all_bones.find(iter->first);
						if(found != all_bones.end())
							found->second         += BoneInfluence(iter->second.ori * coeff, iter->second.pos * coeff);
						else
							all_bones[iter->first] = BoneInfluence(iter->second.ori * coeff, iter->second.pos * coeff);
					}
				}

		// TODO: apply adjustments for approximated value terrain_normal
		// TODO: adjust foot positions to match actual terrain(?)
		// TODO: soldier-specific pelvis adjustments: assist with aim yaw; move to balance aim pitch
		// TODO: do the actual IK and generate the new pose bone influences

		// apply the pose we computed
		for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
			SetBonePose(iter->first, iter->second.ori, iter->second.pos);

		// apply yaw changes
		float delta_yaw = yaw_speed * time.elapsed;
		yaw -= delta_yaw;

		if(yaw_bone != 0)
			SetBonePose(yaw_bone, Vec3(0, -yaw, 0), Vec3());

		// update terrain normal (rotate according to yaw change and move approximation closer to recent values)
		Vec3 current_normal(0, 1, 0);																									// TODO: compute this for real

		float lerp_a = expf(-10.0f * time.elapsed);
		float lerp_b = 1.0f - lerp_a;

		terrain_normal = Vec3::Normalize(lerp_a * (Quaternion::FromRVec(0, delta_yaw, 0) * terrain_normal) + lerp_b * current_normal);	// TODO: check quat direction

		// TODO: animation-driven PFC breakage (if we're still using PFCs)
	}

	void WalkPose::InitialSetYawOffset(float value) { desired_yaw_offset = value; yaw = target_yaw = dood->yaw + value; }
}
