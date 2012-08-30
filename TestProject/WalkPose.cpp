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
		anim_timer(0),
		dood(dood),
		rest_anim(		rest_anim == NULL ?		NULL : new KeyframeAnimation(*rest_anim)),
		forward_anim(	forward_anim == NULL ?	NULL : new KeyframeAnimation(*forward_anim)),
		backward_anim(	backward_anim == NULL ?	NULL : new KeyframeAnimation(*backward_anim)),
		left_anim(		left_anim == NULL ?		NULL : new KeyframeAnimation(*left_anim)),
		right_anim(		right_anim == NULL ?	NULL : new KeyframeAnimation(*right_anim)),
		l_turn_anim(	l_turn_anim == NULL ?	NULL : new KeyframeAnimation(*l_turn_anim)),
		r_turn_anim(	r_turn_anim == NULL ?	NULL : new KeyframeAnimation(*r_turn_anim))
	{
	}

	WalkPose::~WalkPose()
	{
		if(rest_anim)		{ delete rest_anim;		rest_anim = NULL; }
		if(forward_anim)	{ delete forward_anim;	forward_anim = NULL; }
		if(backward_anim)	{ delete backward_anim;	backward_anim = NULL; }
		if(left_anim)		{ delete left_anim;		left_anim = NULL; }
		if(right_anim)		{ delete right_anim;	right_anim = NULL; }
		if(l_turn_anim)		{ delete l_turn_anim;	l_turn_anim = NULL; }
		if(r_turn_anim)		{ delete r_turn_anim;	r_turn_anim = NULL; }
	}

	// TODO: prevent accidentally stepping when moving to rest animation
	void WalkPose::UpdatePose(TimingInfo time)
	{
		// make sure we reset to the rest pose if we aren't doing anything
		bones.clear();

		// find speeds along each of the dood's axes of movement
		Vec3 forward = Vec3(-sinf(dood->yaw), 0, cosf(dood->yaw));
		Vec3 rightward = Vec3(-forward.z, 0, forward.x);

		Vec3 vel = dood->vel;

		float forward_speed = Vec3::Dot(vel, forward);
		float rightward_speed = Vec3::Dot(vel, rightward) * 2.5f;			// TODO: do this more elegantly
		float turn_right = 0.0f;											// TODO: compute this for real somehow

		// figure out the fastest axial speed we have an animation for (used to control animation speed)
		float use_speed = 0.0f;
		if(forward_speed > 0		&& forward_anim)	{ use_speed = forward_speed; }						else if(forward_speed < 0	&& backward_anim)	{ use_speed = -forward_speed; }
		if(rightward_speed > 0		&& right_anim)		{ use_speed = max(rightward_speed,	use_speed); }	else if(rightward_speed < 0	&& left_anim)		{ use_speed = max(-rightward_speed,	use_speed); }
		if(use_speed < 0.2f)
			use_speed = forward_speed = rightward_speed = turn_right = 0.0f;

		float dt = 1.0f - exp(-use_speed * 0.25f * time.elapsed);
		anim_timer += dt;

		// negative coefficients are ignored (set to 0)
		struct AnimWithCoeff { KeyframeAnimation* anim; float coeff; AnimWithCoeff(KeyframeAnimation* anim, float coeff) : anim(anim), coeff(coeff) { } };
		AnimWithCoeff anims[] =
		{
			AnimWithCoeff(rest_anim,		2.0f * (0.5f - use_speed)),
			AnimWithCoeff(forward_anim,		forward_speed),
			AnimWithCoeff(backward_anim,	-forward_speed),
			AnimWithCoeff(left_anim,		-rightward_speed),
			AnimWithCoeff(right_anim,		rightward_speed),
			AnimWithCoeff(l_turn_anim,		-turn_right),
			AnimWithCoeff(r_turn_anim,		turn_right)
		};
		int num_anims = sizeof(anims) / sizeof(AnimWithCoeff);

		float coeffs_total = 0.0f;
		for(int i = 0; i < num_anims; ++i)
		{
			float& coeff = anims[i].coeff;
			if(coeff < 0)
				coeff = 0;

			// TODO: don't sum coeffs for NULL animations?

			coeffs_total += coeff;
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
						if(all_bones.find(iter->first) == all_bones.end())
							all_bones[iter->first] = BoneInfluence(iter->second.ori * coeff, iter->second.pos * coeff);
						else
							all_bones[iter->first] += BoneInfluence(iter->second.ori * coeff, iter->second.pos * coeff);
					}
				}

		// apply the pose we computed
		for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
			SetBonePose(iter->first, iter->second.ori, iter->second.pos);
	}
}
