#include "StdAfx.h"

#include "KeyframeAnimation.h"
#include "IKPose.h"
#include "IKSolver.h"
#include "GameState.h"

namespace CibraryEngine
{
	// a couple of util functions
	Quaternion PYToQuaternion(float pitch, float yaw) { return Quaternion::FromPYR(0, yaw, 0) * Quaternion::FromPYR(pitch, 0, 0); }
	void QuaternionToPY(Quaternion q, float& pitch, float &yaw)
	{
		Mat3 rm = q.ToMat3();
		Vec3 fwd = Vec3::Normalize(rm * Vec3(0, 0, 1));
		Vec3 left = Vec3::Normalize(rm * Vec3(1, 0, 0));

		pitch = asin(-fwd.y);
		yaw = atan2(left.x, left.z) - 0.5f * M_PI;
	}




	/*
	 * IKPose::EndEffector methods
	 */
	IKPose::EndEffector::EndEffector(string bone_name, Vec3 lcs_pos, bool set) : bone_name(bone_name), lcs_pos(lcs_pos), set(set), grounded(false) { }




	/*
	 * IKPose methods
	 */
	IKPose::IKPose(GameState* game_state, Skeleton* skeleton, Vec3 pos, float pitch, float yaw) :
		pos(pos),
		pitch(pitch),
		yaw(yaw),
		end_effectors(),
		game_state(game_state),
		ik_skeleton(new Skeleton(skeleton))
	{
		game_state->ik_solver->AddObject((void*)this, ik_skeleton, pos, PYToQuaternion(pitch, yaw));
		keyframe_animation = new KeyframeAnimation();
	}

	IKPose::~IKPose()
	{
		if(ik_skeleton != NULL)
		{
			ik_skeleton->Dispose();
			delete ik_skeleton;
			ik_skeleton = NULL;
		}

		if(keyframe_animation != NULL)
		{
			delete keyframe_animation;
			keyframe_animation = NULL;
		}

		game_state->ik_solver->DeleteObject((void*)this);
	}

	void IKPose::UpdatePose(TimingInfo time)
	{
		Quaternion py_ori;
		game_state->ik_solver->GetResultState((void*)this, pos, py_ori);
		QuaternionToPY(py_ori, pitch, yaw);

		keyframe_animation->UpdatePose(time);
		for(map<string, BoneInfluence>::iterator iter = keyframe_animation->bones.begin(); iter != keyframe_animation->bones.end(); iter++)
		{
			BoneInfluence& bi = iter->second;
			SetBonePose(iter->first, bi.ori, bi.pos, bi.div);
		}
	}

	void IKPose::SetDesiredState(Vec3 pos_, float pitch_, float yaw_)
	{
		pos = pos_;
		pitch = pitch_;
		yaw = yaw_;
	}

	void IKPose::AddEndEffector(string bone_name, Vec3 lcs_pos, bool set)
	{
		end_effectors.push_back(EndEffector(bone_name, lcs_pos, set));
	}
}
