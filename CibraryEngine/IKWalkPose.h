#pragma once
#include "StdAfx.h"

#include "SkeletalAnimation.h"
#include "IKChain.h"

namespace CibraryEngine
{
	using namespace std;

	class PhysicsWorld;
	class RigidBody;
	class JointConstraint;

	class IKWalkPose : public Pose
	{
		public:

			PhysicsWorld* physics;

			vector<RigidBody*> rigid_bodies;
			vector<JointConstraint*> all_joints;

			struct Joint
			{
				JointConstraint* constraint;

				Quaternion target_ori;
				unsigned int set_pose_id;
				bool invert;

				Joint(JointConstraint* constraint, const Quaternion& target_ori, unsigned int set_pose_id, bool invert) : constraint(constraint), target_ori(target_ori), set_pose_id(set_pose_id), invert(invert) { }
			};
			vector<Joint> joints;

			struct EndEffector
			{
				RigidBody* pelvis;
				RigidBody* upper_leg;
				RigidBody* lower_leg;
				RigidBody* foot;

				JointConstraint* hip;
				JointConstraint* knee;
				JointConstraint* ankle;

				float upper_leg_length, lower_leg_length;
				float asq, bsq;			// leg lengths squared
				float asqmbsq;			// upper_leg_length squared - lower_leg_length squared

				int hip_values_index;
				int knee_values_index;
				int ankle_values_index;

				bool arrived;

				EndEffector(JointConstraint* hip, JointConstraint* knee, JointConstraint* ankle);

				// false = no solution!
				// assigns to appropriate indices of joint values array the proper orientations for those joints
				bool Extend(const Mat4& pelvis_xform, const Mat4& foot_xform, float* values);
			};
			vector<EndEffector> end_effectors;

			// root bone desired state info
			Vec3 desired_pos;
			Quaternion desired_ori;

			Vec3 arrive_vel;			// velocity to maintain once arrived

			float arrive_time;

			IKWalkPose(PhysicsWorld* physics, const vector<RigidBody*>& rigid_bodies, const vector<JointConstraint*>& all_joints, const vector<JointConstraint*>& constraints, const vector<Bone*>& rbody_to_posey);
			~IKWalkPose();

			void UpdatePose(TimingInfo time);

			void AddEndEffector(RigidBody* foot);

			void Seek(float timestep, float foresight);
	};
}
