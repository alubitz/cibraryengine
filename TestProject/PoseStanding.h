#pragma once
#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class PoseStanding : public Pose
	{
		public:

			Dood* dood;

			PoseStanding(Dood* dood) : Pose(), dood(dood) { }

			void UpdatePose(TimingInfo time)
			{
				Vec3 vel = dood->vel;

				Mat4 p_xform = dood->character->skeleton->GetNamedBone("pelvis")->GetTransformationMatrix();
				Mat4 l_xform = dood->character->skeleton->GetNamedBone("l foot")->GetTransformationMatrix();
				Mat4 r_xform = dood->character->skeleton->GetNamedBone("r foot")->GetTransformationMatrix();

				Vec3 p_pos =		p_xform.TransformVec3_1(0, 0, 0);
				Vec3 p_left =		p_xform.TransformVec3_0(1, 0, 0);
				Vec3 p_up =			p_xform.TransformVec3_0(0, 1, 0);
				Vec3 p_backward =	p_xform.TransformVec3_0(0, 0, 1);

				vel -= p_up * Vec3::Dot(p_up, vel);
				vel -= p_left * Vec3::Dot(p_left, vel);

				Vec3 lfoot_pos =	l_xform.TransformVec3_1(0, 0, 0) - p_pos;
				Vec3 rfoot_pos =	r_xform.TransformVec3_1(0, 0, 0) - p_pos;

				float v_ldot = Vec3::Dot(vel, lfoot_pos);
				float v_rdot = Vec3::Dot(vel, rfoot_pos);

				SetBonePose(Bone::string_table["l foot"], Vec3(v_ldot, 0, 0), Vec3());
				SetBonePose(Bone::string_table["r foot"], Vec3(v_rdot, 0, 0), Vec3());
			}
	};
}
