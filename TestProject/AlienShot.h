#pragma once

#include "StdAfx.h"

#include "Damage.h"
#include "Shot.h"

namespace Test
{
	class AlienShot : public Shot
	{
		public:

			AlienShot(GameState* gs, VTNModel* model, GlowyModelMaterial* material, Vec3 origin, Vec3 initial_vel, Quaternion ori, Dood* firer) : Shot(gs, model, material, origin, initial_vel, ori, firer) { }

			Damage GetDamage() { return Damage(firer, 0.03f); }
	};
}
