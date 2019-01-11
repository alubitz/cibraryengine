#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class GAExperiment;

	/**
	 * @class Represents a Pendulum object
	 */
	class Pendulum : public Dood
	{
	private:

		class Imp;
		Imp* imp;

		friend class Imp;

	protected:

		// Deletes the implementation and calls Dood::InnerDispose()
		void InnerDispose();

		// Called before pose updates occurs. Tells the implementation to update according to current timestamp
		void PreUpdatePoses(const TimingInfo& time);

		// Has a chance to nullify or cheat a velocity. Currently just nullifies the cheaty_rot
		void MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi);

		virtual void InitBoneHelpers();
		virtual void InitJointHelpers();

		void DoInitialPose();

	public:

		Pendulum(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

		void Update(const TimingInfo& time);

		void PrePhysicsStep(float timestep);

		void RegisterFeet();
	};
}
