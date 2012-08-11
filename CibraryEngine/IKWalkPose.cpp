#include "StdAfx.h"
#include "IKWalkPose.h"

#include "Random3D.h"

#include "Physics.h"

#include "RigidBody.h"
#include "JointConstraint.h"

#include "Util.h"

namespace CibraryEngine
{
	/*
	 * IKWalkPose::EndEffector methods
	 */
	IKWalkPose::EndEffector::EndEffector(JointConstraint* hip, JointConstraint* knee, JointConstraint* ankle) :
		pelvis(hip->obj_a),
		upper_leg(knee->obj_a),
		lower_leg(knee->obj_b),
		foot(ankle->obj_b),
		hip(hip),
		knee(knee),
		ankle(ankle),
		upper_leg_length((hip->pos - knee->pos).ComputeMagnitude()),
		lower_leg_length((ankle->pos - knee->pos).ComputeMagnitude()),
		asq(upper_leg_length * upper_leg_length),
		bsq(lower_leg_length * lower_leg_length),
		asqmbsq(asq - bsq),
		hip_values_index(-1),
		knee_values_index(-1),
		ankle_values_index(-1),
		arrived(false)
	{	
	}


	bool IKWalkPose::EndEffector::Extend(const Mat4& pelvis_xform, const Mat4& foot_xform, float* values)
	{
		Vec3 hip_pos = pelvis_xform.TransformVec3_1(hip->pos);
		Vec3 ankle_pos = foot_xform.TransformVec3_1(ankle->pos);

		Vec3 hip_to_ankle = ankle_pos - hip_pos;
		float dist_sq = hip_to_ankle.ComputeMagnitudeSquared();
		if(dist_sq == 0.0f)
			return false;													// NOTE: if the leg lengths are equal this is still solvable!

		float dist = sqrtf(dist_sq), inv_dist = 1.0f / dist;

		float y = 0.5f * (dist_sq + asqmbsq) * inv_dist;					// distance along vector from ankle to hip to reach knee joint

		float r_sq = asq - y * y;
		if(r_sq < 0.0f)
		{
			Debug(((stringstream&)(stringstream() << "c = " << dist << "; a + b = " << upper_leg_length + lower_leg_length << "; |a - b| = " << fabs(upper_leg_length - lower_leg_length) << endl)).str());
			return false;
		}

		Vec3 n = hip_to_ankle * inv_dist;									// normal to plane of solution circle (locus on which the knee joint must lie)
		Vec3 c = hip_pos + y * n;											// center of solution circle
		float r = sqrtf(r_sq);												// radius of solution circle

		// figure out where things are now so we can pick a solution that is changed minimally
		Mat4 cur_lleg_xform = lower_leg->GetTransformationMatrix();			// this will be used twice, so might as well cache it
		Vec3 current_hip_pos =		pelvis->GetTransformationMatrix().TransformVec3_1(hip->pos);
		Vec3 current_knee_pos =		cur_lleg_xform.TransformVec3_1(knee->pos);
		Vec3 current_ankle_pos =	cur_lleg_xform.TransformVec3_1(ankle->pos);

		// find point on circle closest to current position of knee
		Vec3 knee_pos = current_knee_pos - c;
		knee_pos -= n * Vec3::Dot(n, knee_pos);
		knee_pos = c + Vec3::Normalize(knee_pos, r);

		Quaternion uleg_ori = upper_leg->GetOrientation();
		Quaternion lleg_ori = lower_leg->GetOrientation();

		Vec3 cur_uleg_vec = current_hip_pos - current_knee_pos, uleg_vec = hip_pos - knee_pos;
		Vec3 uleg_cross = Vec3::Cross(cur_uleg_vec, uleg_vec);
		if(float cross_sq = uleg_cross.ComputeMagnitudeSquared())
		{
			float angle = asinf(sqrtf(cross_sq / (cur_uleg_vec.ComputeMagnitudeSquared() * uleg_vec.ComputeMagnitudeSquared())));
			uleg_cross /= sqrtf(cross_sq);

			uleg_ori *= Quaternion::FromAxisAngle(uleg_cross.x, uleg_cross.y, uleg_cross.z, -angle);
		}

		Vec3 cur_lleg_vec = current_knee_pos - current_ankle_pos, lleg_vec = knee_pos - ankle_pos;
		Vec3 lleg_cross = Vec3::Cross(cur_lleg_vec, lleg_vec);
		if(float cross_sq = lleg_cross.ComputeMagnitudeSquared())
		{
			float angle = asinf(sqrtf(cross_sq / (cur_lleg_vec.ComputeMagnitudeSquared() * lleg_vec.ComputeMagnitudeSquared())));
			lleg_cross /= sqrtf(cross_sq);

			lleg_ori *= Quaternion::FromAxisAngle(lleg_cross.x, lleg_cross.y, lleg_cross.z, -angle);
		}

		Quaternion pelvis_ori;
		Quaternion foot_ori;
		Vec3 dummy;
		pelvis_xform.Decompose(	dummy, pelvis_ori);
		foot_xform.Decompose(	dummy, foot_ori);

		Quaternion hip_ori =	Quaternion::Reverse(pelvis_ori)	* uleg_ori;
		Quaternion knee_ori =	Quaternion::Reverse(uleg_ori)	* lleg_ori;
		Quaternion ankle_ori =	Quaternion::Reverse(lleg_ori)	* foot_ori;

		if(hip_values_index >= 0)
		{
			float* val_ptr = values + hip_values_index * 3;

			Vec3 pyr = hip_ori.ToPYR();
			*(val_ptr++) =	pyr.x;
			*(val_ptr++) =	pyr.y;
			*val_ptr =		pyr.z;
		}

		if(knee_values_index >= 0)
		{
			float* val_ptr = values + knee_values_index * 3;
		
			Vec3 pyr = knee_ori.ToPYR();
			*(val_ptr++) =	pyr.x;
			*(val_ptr++) =	pyr.y;
			*val_ptr =		pyr.z;
		}

		if(ankle_values_index >= 0)
		{
			float* val_ptr = values + ankle_values_index * 3;
			
			Vec3 pyr = ankle_ori.ToPYR();
			*(val_ptr++) =	pyr.x;
			*(val_ptr++) =	pyr.y;
			*val_ptr =		pyr.z;
		}

		return true;
	}




	/*
	 * IKWalkPose methods
	 */
	IKWalkPose::IKWalkPose(PhysicsWorld* physics, const vector<RigidBody*>& rigid_bodies, const vector<JointConstraint*>& all_joints, const vector<JointConstraint*>& constraints, const vector<Bone*>& rbody_to_posey) :
		Pose(),
		physics(physics),
		rigid_bodies(rigid_bodies),
		all_joints(all_joints),
		joints(),
		end_effectors(),
		arrive_time(-1)
	{
		for(vector<JointConstraint*>::const_iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			JointConstraint* constraint = *iter;

			Bone *bone_a = NULL, *bone_b = NULL;

			unsigned int i = 0;
			for(vector<RigidBody*>::const_iterator jter = rigid_bodies.begin(); jter != rigid_bodies.end(); ++jter, ++i)
			{
				if(*jter == constraint->obj_a)
					bone_a = rbody_to_posey[i];
				else if(*jter == constraint->obj_b)
					bone_b = rbody_to_posey[i];
			}

			if(bone_a && bone_b)
			{
				if(bone_a == bone_b->parent)
					joints.push_back(Joint(constraint, constraint->desired_ori, bone_b->name, false));
				else if(bone_b == bone_a->parent)
					joints.push_back(Joint(constraint, constraint->desired_ori, bone_a->name, true));
			}
		}
	}

	IKWalkPose::~IKWalkPose() { }

	void IKWalkPose::UpdatePose(TimingInfo time)
	{
		float timestep = time.elapsed;
		if(timestep > 0)
		{
			float now = time.total;
			float next_arrival = arrive_time;

			Seek(timestep, max(timestep, next_arrival - now));
		}
	}

	void IKWalkPose::AddEndEffector(RigidBody* foot)
	{
		JointConstraint *ankle = NULL, *knee = NULL, *hip = NULL;

		for(unsigned int i = 0; i < joints.size(); ++i)
		{
			JointConstraint* jc = joints[i].constraint;
			if(jc->obj_b == foot)
			{
				ankle = jc;
				for(unsigned int j = 0; j < joints.size(); ++j)
				{
					JointConstraint* jc = joints[j].constraint;
					if(jc->obj_b == ankle->obj_a)
					{
						knee = jc;
						for(unsigned int k = 0; k < joints.size(); ++k)
						{
							JointConstraint* jc = joints[k].constraint;
							if(jc->obj_b == knee->obj_a)
							{
								hip = jc;

								EndEffector ee(hip, knee, ankle);

								ee.hip_values_index = k;
								ee.knee_values_index = j;
								ee.ankle_values_index = i;

								end_effectors.push_back(ee);

								return;
							}
						}
					}
				}
				break;
			}
		}
	}

	void IKWalkPose::Seek(float timestep, float foresight)
	{
		// compute position and velocity of the dood's center of mass
		Vec3 com, com_vel;
		float total_weight = 0.0f;

		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			RigidBody& body = **iter;
			MassInfo mass_info = body.GetMassInfo();

			float weight = mass_info.mass;
			
			com += body.GetCenterOfMass() * weight;
			com_vel += body.GetLinearVelocity() * weight;

			total_weight += weight;
		}

		com /= total_weight;
		com_vel /= total_weight;

		// TODO: use the stuff we just computed somehow



		vector<float> joint_values(joints.size() * 3);

		bool first = true;

		Mat4 pelvis_xform = rigid_bodies[0]->GetTransformationMatrix();
		for(vector<EndEffector>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
		{
			Mat4 foot_xform = pelvis_xform;

			if(first)
			{
				foot_xform *= Mat4::Translation(0, 0.0f, 0);
				first = false;
			}

			if(!iter->Extend(pelvis_xform, foot_xform, joint_values.data()))
				return;
		}




		// apply the solution we came up with
		float* val_iter = joint_values.data();
		for(vector<Joint>::iterator iter = joints.begin(); iter != joints.end(); ++iter)
		{
			Joint& joint = *iter;
			JointConstraint& constraint = *joint.constraint;

			float x = *(val_iter++), y = *(val_iter++), z = *(val_iter++);
			SetBonePose(joint.set_pose_id, Vec3(x, y, z), Vec3());
		}
	}
}
