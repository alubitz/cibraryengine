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

			vector<RigidBody*> rigid_bodies;

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

				Vec3 desired_pos;
				Quaternion desired_ori;

				float arrive_time;

				bool arrived;			// might want more than two states?

				EndEffector(PhysicsWorld* physics, RigidBody* foot);
				~EndEffector();

				void LockPlacedFoot(RigidBody* base);
				void UnlockPlacedFoot();
			};
			vector<EndEffector*> end_effectors;

			// root bone desired state info
			Vec3 desired_pos;
			Quaternion desired_ori;

			Vec3 arrive_vel;			// velocity to maintain once arrived

			float arrive_time;

			IKWalkPose(PhysicsWorld* physics, const vector<RigidBody*>& rigid_bodies, const vector<JointConstraint*>& constraints, const vector<Bone*>& rbody_to_posey);
			~IKWalkPose();

			void UpdatePose(TimingInfo time);

			void AddEndEffector(RigidBody* foot);

			void Seek(float timestep, float foresight);
			float EvaluateSolution(const vector<float>& values);
	};
}
