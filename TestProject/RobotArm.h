#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class RobotArm : public Dood
	{
		public:

			struct JointEntry
			{
				JointConstraint* constraint;
				Quaternion desired_ori;

				JointEntry(JointConstraint* constraint, const Quaternion& desired_ori = Quaternion::Identity()) : constraint(constraint), desired_ori(desired_ori) { }
			};

			vector<JointEntry> joints;

			float claw_timer;

			RobotArm(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void PoseToPhysics(float timestep);

			void Spawned();
	};
}
