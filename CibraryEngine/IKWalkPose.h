#pragma once
#include "StdAfx.h"

#include "SkeletalAnimation.h"
#include "IKChain.h"

namespace CibraryEngine
{
	using namespace std;

	struct ModelPhysics;

	class IKWalkPose : public Pose
	{
		public:

			ModelPhysics* mphys;

			Skeleton* pose_skel;
			Skeleton* physics_skel;

			unsigned int root_bone;

			struct EndEffector
			{
				IKChain* chain;

				Vec3 desired_pos;
				Quaternion desired_ori;

				float arrive_time;

				bool arrived;			// might want more than two states?

				EndEffector(Bone* from, Bone* to, ModelPhysics* mphys) : chain(new IKChain(from, to, mphys)) { }
				~EndEffector() { delete chain; chain = NULL; }
			};
			vector<EndEffector*> end_effectors;

			// root bone desired state info
			Vec3 desired_pos;
			Quaternion desired_ori;

			Vec3 arrive_vel;			// velocity to maintain once arrived

			float arrive_time;

			IKWalkPose(Skeleton* pose_skel, Skeleton* physics_skel, unsigned int root_bone, ModelPhysics* mphys);
			~IKWalkPose();

			void UpdatePose(TimingInfo time);

			void Seek(float timestep, float foresight);

			void AddEndEffector(unsigned int from, unsigned int to);
			void AddEndEffector(unsigned int to) { AddEndEffector(root_bone, to); }

			float EvaluateSolution(const vector<float>& values);
	};
}
