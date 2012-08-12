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

		Vec3 dummy;
		Quaternion pelvis_ori, foot_ori;
		pelvis_xform.Decompose(	dummy, pelvis_ori);
		foot_xform.Decompose(	dummy, foot_ori);

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

		// select position for the knee
		Vec3 knee_forward = pelvis_xform.TransformVec3_0(0, 0, 1) + foot_xform.TransformVec3_0(0, 0, 1);
		Vec3 knee_pos = knee_forward - n * Vec3::Dot(n, knee_forward);
		knee_pos = c + Vec3::Normalize(knee_pos, r);

		Vec3 uleg_length = knee_pos - hip_pos;
		Vec3 lleg_length = ankle_pos - knee_pos;

		Vec3 rest_uleg_vec = Vec3::Normalize(pelvis_xform.TransformVec3_0(knee->pos - hip->pos));
		Vec3 rest_lleg_vec = Vec3::Normalize(pelvis_xform.TransformVec3_0(ankle->pos - knee->pos));

		// uleg_ori * knee_x = knee_axis
		// uleg_ori * rest_uleg_vec = uleg_length
		// third component = cross product of the first two

		Vec3 knee_x = Vec3::Normalize(Vec3::Cross(rest_lleg_vec,	rest_uleg_vec));
		Vec3 knee_axis = Vec3::Normalize(Vec3::Cross(lleg_length,	uleg_length));

		Vec3 a_cross = Vec3::Normalize(Vec3::Cross(knee_x,		rest_uleg_vec));
		Vec3 b_cross = Vec3::Normalize(Vec3::Cross(knee_axis,	uleg_length));
		Mat3 ma = Mat3(knee_x.x,	knee_x.y,		knee_x.z,		rest_uleg_vec.x,	rest_uleg_vec.y,	rest_uleg_vec.z,	a_cross.x, a_cross.y, a_cross.z);
		Mat3 mb = Mat3(knee_axis.x,	knee_axis.y,	knee_axis.z,	uleg_length.x,		uleg_length.y,		uleg_length.z,		b_cross.x, b_cross.y, b_cross.z);
		Quaternion uleg_ori = Quaternion::FromRotationMatrix(mb * Mat3::Invert(ma));

		Vec3 c_cross = Vec3::Normalize(Vec3::Cross(knee_x,		rest_lleg_vec));
		Vec3 d_cross = Vec3::Normalize(Vec3::Cross(knee_axis,	lleg_length));
		Mat3 mc = Mat3(knee_x.x,	knee_x.y,		knee_x.z,		rest_lleg_vec.x,	rest_lleg_vec.y,	rest_lleg_vec.z,	c_cross.x, c_cross.y, c_cross.z);
		Mat3 md = Mat3(knee_axis.x,	knee_axis.y,	knee_axis.z,	lleg_length.x,		lleg_length.y,		lleg_length.z,		d_cross.x, d_cross.y, d_cross.z);
		Quaternion lleg_ori = Quaternion::FromRotationMatrix(md * Mat3::Invert(mc));

		// presence/absence of "Quaternion::Reverse(pelvis_ori) *" is due to weird business with transforming earlier
		Quaternion hip_ori =	uleg_ori;
		Quaternion knee_ori =	Quaternion::Reverse(uleg_ori) * lleg_ori;
		Quaternion ankle_ori =	Quaternion::Reverse(pelvis_ori * lleg_ori) * foot_ori;

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

		Mat4 pelvis_xform = rigid_bodies[0]->GetTransformationMatrix();
		for(vector<EndEffector>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
		{
			Mat4 foot_xform = pelvis_xform;

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
