#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class GhostCamera : public Pawn
	{
		public:
			Vec3 pos, vel;
			float yaw, pitch;
			Quaternion ori;

			GhostCamera(GameState* gs) : Pawn(gs), pos(), vel(), yaw(0), pitch(0), ori(Quaternion::Identity()) { }

			Mat4 GetViewMatrix();		// also computes ori as a function of yaw and pitch

			void Update(const TimingInfo& time);
	};
}