#include "StdAfx.h"
#include "KinematicCharacter.h"

#include "Physics.h"

namespace CibraryEngine
{
	/*
	 * KinematicCharacterRig methods
	 */
	KinematicCharacterRig::KinematicCharacterRig(KinematicCharacterShape* shape, const MassInfo& mass_info, const Vec3& pos, const Quaternion& ori) :
		RigidBody(shape, mass_info, pos, ori),
		bones()
	{
		shape->rig = this;
	}

	KinematicCharacterRig* KinematicCharacterRig::FromModelPhysics(ModelPhysics* mphys)
	{
		KinematicCharacterShape* shape = new KinematicCharacterShape();

		// TODO: initialize these...
		MassInfo mass_info;
		Vec3 pos;
		Quaternion ori(Quaternion::Identity());

		KinematicCharacterRig* rig = new KinematicCharacterRig(shape, mass_info, pos, ori);

		// TODO:
		//
		//	for each bone in mphys:
		//		add that bone to rig->bones
		//	update mass info

		return rig;
	}




	/*
	 * KinematicCharacterBone methods
	 */
	KinematicCharacterBone::KinematicCharacterBone(KinematicCharacterRig* rig, CollisionShape* shape, const MassInfo& mass_info, const Vec3& pos, const Quaternion& ori) :
		RigidBody(shape, mass_info, pos, ori),
		rig(rig)
	{
		SetCollisionProxy(rig);
	}




	/*
	 * KinematicCharacterShape methods
	 */
	KinematicCharacterShape::KinematicCharacterShape() :
		CollisionShape(ST_KinematicCharacter),
		rig(NULL)
	{
	}

	MassInfo KinematicCharacterShape::ComputeMassInfo()
	{
		// TODO: implement this
		return CollisionShape::ComputeMassInfo();
	}

	AABB KinematicCharacterShape::GetTransformedAABB(const Mat4& xform)
	{
		// TODO: implement this
		return CollisionShape::GetTransformedAABB(xform);
	}

	void KinematicCharacterShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
	{
		// TODO: draw the shape
	}

	unsigned int KinematicCharacterShape::Read(istream& stream)
	{
		// TODO: load the shape from the stream
		return 0;
	}

	void KinematicCharacterShape::Write(ostream& stream)
	{
		// TODO: write the shape to the stream
	}
}
