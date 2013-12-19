#include "StdAfx.h"
#include "CrabWeapon.h"

#include "Dood.h"
#include "Shot.h"

namespace Test
{
	struct CrabShot : Shot
	{
		float damage;

		CrabShot(GameState* gs, Dood* firer, Vec3 pos, Vec3 forward, float damage) : Shot(gs, NULL, NULL, pos, forward, firer), damage(damage) { }

		Damage GetDamage() { return Damage(firer, damage); }
	};




	/*
	 * CrabWeapon methods
	 */
	CrabWeapon::CrabWeapon(TestGame* test, Dood* owner) :
		WeaponIntrinsic(test, owner),
		attack_ready_time(-1.0f),
		attack_interval(1.0f)
	{
	}

	void CrabWeapon::OwnerUpdate(TimingInfo time)
	{
		if(IsFiring(1) && time.total >= attack_ready_time)
			ClawAttackNormal(time.total);
	}

	void CrabWeapon::ClawAttackNormal(float now)
	{
		Mat4 mat = Mat4::FromPositionAndOrientation(owner->pos, Quaternion::FromRVec(0, -owner->yaw, 0));

		Vec3 damage_location = mat.TransformVec3_1(0, 0, 0.8f);

		struct : public EntityQualifier { bool Accept(Entity* ent) { return dynamic_cast<Dood*>(ent) != NULL; } } qualifier;

		EntityList elist = game_state->GetQualifyingEntities(qualifier);
		for(unsigned int i = 0; i < elist.Count(); ++i)
		{
			Entity* ent = elist[i];
			Dood* dood = (Dood*)ent;
			if(dood == owner)
				continue;

			Vec3 dx = damage_location - dood->pos;
			float dist = dx.ComputeMagnitude();

			const float outer_r = 1.0;

			if(dist > outer_r)
				continue;

			const float inner_r = 0.25f;
			const float full_damage = 0.15f;
			const float coeff = full_damage / (outer_r - inner_r);

			float damage_amount = dist < inner_r ? full_damage : (outer_r - dist) * coeff;
			CrabShot shot(game_state, owner, damage_location, Vec3(), damage_amount);

			dood->shootables[0]->GetShot(&shot, damage_location, Vec3(), 0.0f);
		}

		attack_ready_time = now + attack_interval;

		FireOnce(1);
	}
}
