#include "StdAfx.h"
#include "DefaultWeapon.h"

#include "Dood.h"
#include "Shot.h"

#include "TestGame.h"

namespace Test
{
	using namespace CibraryEngine;

	DefaultWeapon::DefaultWeapon(TestGame* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, GlowyModelMaterial* mflash_material, ModelPhysics* mphys, VertexBuffer* shot_model, BillboardMaterial* shot_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound) :
		Gun(game_state, owner, gun_model, mflash_model, mflash_material, mphys, fire_sound, chamber_click_sound, reload_sound),
		shot_model(shot_model),
		shot_material(shot_material)
	{
		clip = clip_size = 150;
	}

	Shot* DefaultWeapon::CreateShot(const Vec3& origin, const Vec3& weapon_vel, const Vec3& direction)
	{
		Vec3 vel = Vec3::Normalize(direction, 300) + weapon_vel;
		return new Shot(game_state, shot_model, shot_material, origin, vel, owner);
	}

	bool DefaultWeapon::GetAmmoCount(int& result)	{ result = clip; return true; }

	Mat4 DefaultWeapon::GetInitialXform()			{ return owner->RigidBodyForNamedBone("r hand")->GetTransformationMatrix() * Mat4::FromPositionAndOrientation(Vec3(-0.959f,  1.098f,  0.077f), Quaternion::FromRVec(-Vec3(-1.27667f, 0.336123f, 0.64284f))) * Mat4::Translation(-Vec3(0.000f, -0.063f, -0.152f)); }
};
