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
		yaw = atan2(left.x, left.z) - 0.5f * float(M_PI);
	}




	/*
	 * IKPose private implementation struct
	 */
	struct IKPose::Imp
	{
		TimingInfo time;

		Imp() : time() { }
	};




	/*
	 * IKPose::EndEffector methods
	 */
	IKPose::EndEffector::EndEffector(string bone_name, Vec3 lcs_pos, bool set) : bone_name(bone_name), lcs_pos(lcs_pos), set(set), grounded(false) { }




	/*
	 * IKPose methods
	 */
	IKPose::IKPose(GameState* game_state, Skeleton* skeleton, Vec3 pos, float pitch, float yaw) :
		imp(new Imp()),
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
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}

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

		if(time.elapsed > 0)
		{
			// here we are using the internal animation timer, not the main game timer
			TimingInfo& imp_time = imp->time;

			keyframe_animation->UpdatePose(imp_time);
			imp_time.total += imp_time.elapsed;
			imp_time.elapsed = 0;
		}

		for(unordered_map<unsigned int, BoneInfluence>::iterator iter = keyframe_animation->bones.begin(); iter != keyframe_animation->bones.end(); ++iter)
		{
			BoneInfluence& binf = iter->second;
			SetBonePose(iter->first, binf.ori, binf.pos, binf.div);
		}
	}

	void IKPose::SetDesiredState(IKSolver* solver, Vec3 nuPos, float nuPitch, float nuYaw)
	{
		imp->time.elapsed += (nuPos - pos).ComputeMagnitude() * 0.25f;

		pos = nuPos;
		pitch = nuPitch;
		yaw = nuYaw;

		solver->SetDesiredState((void*)this, pos, PYToQuaternion(pitch, yaw));
	}

	void IKPose::AddEndEffector(string bone_name, Vec3 lcs_pos, bool set)
	{
		end_effectors.push_back(EndEffector(bone_name, lcs_pos, set));
	}
}
