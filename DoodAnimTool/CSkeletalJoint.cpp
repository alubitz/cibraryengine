#include "StdAfx.h"
#include "CSkeletalJoint.h"

#include "DATJoint.h"
#include "DATBone.h"
#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * CSkeletalJoint methods
	 */
	CSkeletalJoint::CSkeletalJoint(const DATJoint& joint, const vector<DATBone>& bones) :
		bone_a(joint.joint->bone_a - 1),
		bone_b(joint.joint->bone_b - 1),
		joint_pos(joint.joint->pos),
		lcenter_a(bones[bone_a].center),
		lcenter_b(bones[bone_b].center),
		joint(joint.joint),
		enforce_rotation_limits(true)
	{ }

	CSkeletalJoint::~CSkeletalJoint() { }



	void CSkeletalJoint::InitCachedStuff(PoseSolverState& pose)
	{		
		initial_a = &pose.initial.data[bone_a];
		initial_b = &pose.initial.data[bone_b];

		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];

		nexta = &pose.next.data[bone_a];
		nextb = &pose.next.data[bone_b];
	}

	bool CSkeletalJoint::ApplyConstraint(PoseSolverState& pose)
	{
		static const float rotation_threshold    = 0.001f;
		static const float translation_threshold = 0.000001f;

		bool did_stuff = false;

		Quaternion a_ori = obja->ori, b_ori = objb->ori;
		Quaternion aori_inv = Quaternion::Reverse(a_ori);//, bori_inv = Quaternion::Reverse(b_ori);

		Quaternion nextori_a = a_ori, nextori_b = b_ori;
		Vec3 nextpos_a = obja->pos, nextpos_b = objb->pos;

		// bones rotating to stay in their sockets
		Mat4 amat = Mat4::FromPositionAndOrientation(nextpos_a, Quaternion::Reverse(nextori_a));
		Mat4 bmat = Mat4::FromPositionAndOrientation(nextpos_b, Quaternion::Reverse(nextori_b));
		Vec3 apos = amat.TransformVec3_1(joint_pos);
		Vec3 bpos = bmat.TransformVec3_1(joint_pos);
		Vec3 dx = bpos - apos;
		if(float err = dx.ComputeMagnitudeSquared())
		{
			pose.errors[0] += err;

			if(err > rotation_threshold)
			{
				static const float rotation_coeff = 0.01f;

				Vec3 acen = amat.TransformVec3_1(lcenter_a), bcen = bmat.TransformVec3_1(lcenter_b);
				Vec3 midpoint = (apos + bpos) * 0.5f;
				Vec3 aa = acen - apos, ab = acen - midpoint;
				Vec3 ba = bcen - apos, bb = bcen - midpoint;
				Vec3 ax = Vec3::Cross(aa, ab);
				Vec3 bx = Vec3::Cross(ba, bb);

				if(float axmagsq = ax.ComputeMagnitudeSquared())
				{
					float axmag = sqrtf(axmagsq);
					float acoeff = rotation_coeff * asinf(axmag / sqrtf(aa.ComputeMagnitudeSquared() * ab.ComputeMagnitudeSquared())) / axmag;
					nextori_a = Quaternion::FromPYR(ax * acoeff) * nextori_a;
				
					did_stuff = true;
				}

				if(float bxmagsq = bx.ComputeMagnitudeSquared())
				{
					float bxmag = sqrtf(bxmagsq);
					float bcoeff = rotation_coeff * asinf(bxmag / sqrtf(ba.ComputeMagnitudeSquared() * bb.ComputeMagnitudeSquared())) / bxmag;		
					nextori_b = Quaternion::FromPYR(bx * bcoeff) * nextori_b;

					did_stuff = true;
				}
			}
		}


		// joint staying within its rotation limits
		if(enforce_rotation_limits)
		{
			Quaternion a_to_b = Quaternion::Reverse(nextori_a) * nextori_b;
			Mat3 oriented_axes = joint->axes * nextori_a.ToMat3();

			Vec3 proposed_pyr = oriented_axes * -a_to_b.ToPYR();
			Vec3 nupyr = joint->GetClampedAngles(proposed_pyr);

			if(float err = (proposed_pyr - nupyr).ComputeMagnitudeSquared())
			{
				pose.errors[1] += err;

				Quaternion actual_ori = Quaternion::FromPYR(oriented_axes.Transpose() * -nupyr);
				Vec3 av = (Quaternion::Reverse(a_to_b) * actual_ori).ToPYR();

				av *= 0.5f;

				Quaternion delta_quat = Quaternion::FromPYR(av);
				nextori_a = Quaternion::Reverse(delta_quat) * nextori_a;
				nextori_b = delta_quat * nextori_b;

				did_stuff = true;
			}
		}

		

		// bones translating to stay in their sockets
		amat = Mat4::FromPositionAndOrientation(nextpos_a, Quaternion::Reverse(nextori_a));		// recompute these values using the updated bone orientations
		bmat = Mat4::FromPositionAndOrientation(nextpos_b, Quaternion::Reverse(nextori_b));
		apos = amat.TransformVec3_1(joint_pos);
		bpos = bmat.TransformVec3_1(joint_pos);

		dx = bpos - apos;
		if(float err = dx.ComputeMagnitudeSquared())
		{
			pose.errors[2] += err;

			if(err > translation_threshold)
			{
				dx *= 0.5f;

				nextpos_a += dx;
				nextpos_b -= dx;

				did_stuff = true;
			}
		}
		
		if(did_stuff)
		{
			nexta->pos += nextpos_a;
			nexta->ori += nextori_a;
			nextb->pos += nextpos_b;
			nextb->ori += nextori_b;

			++pose.contrib_count[bone_a];
			++pose.contrib_count[bone_b];

			return true;
		}
		else
			return false;
	}

	void CSkeletalJoint::OnAnyChanges(PoseSolverState& pose)
	{
		int bone_a = joint->bone_a - 1, bone_b = joint->bone_b - 1;

		obja = &pose.current.data[bone_a];
		objb = &pose.current.data[bone_b];
	}
}
