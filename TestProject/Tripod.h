#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class GAExperiment;

	class Tripod : public Dood
	{
		private:

			struct Imp;
			Imp* imp;

			friend struct Imp;

		protected:

			void InnerDispose();

			void PreUpdatePoses(const TimingInfo& time);

			void MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi);

			virtual void InitBoneHelpers();
			virtual void InitJointHelpers();

		public:

			Tripod(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

			void Update(const TimingInfo& time);

			void RegisterFeet();
	};
}
