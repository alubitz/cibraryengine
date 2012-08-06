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
	class PlacedFootConstraint;

	class IKWalkPose : public Pose
	{
		public:

			PhysicsWorld* physics;

			ModelPhysics* mphys;

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
				PhysicsWorld* physics;

				RigidBody* foot;
				PlacedFootConstraint* placed;

				IKChain* chain;

				Vec3 desired_pos;
				Quaternion desired_ori;

				float arrive_time;

				bool arrived;			// might want more than two states?

				EndEffector(PhysicsWorld* physics, ModelPhysics* mphys, Bone* from, Bone* to, RigidBody* foot);
				~EndEffector();

				void LockPlacedFoot(RigidBody* base);
				void UnlockPlacedFoot();
			};
			vector<EndEffector*> end_effectors;

			vector<Bone*> rbody_to_posey;

			// root bone desired state info
			Vec3 desired_pos;
			Quaternion desired_ori;

			Vec3 arrive_vel;			// velocity to maintain once arrived

			float arrive_time;

			IKWalkPose(PhysicsWorld* physics, ModelPhysics* mphys, const vector<RigidBody*>& rigid_bodies, const vector<JointConstraint*>& all_joints, const vector<JointConstraint*>& constraints, const vector<Bone*>& rbody_to_posey);
			~IKWalkPose();

			void UpdatePose(TimingInfo time);

			void AddEndEffector(RigidBody* foot);

			void Seek(float timestep, float foresight);
			float EvaluateSolution(const vector<float>& values);
	};
}
