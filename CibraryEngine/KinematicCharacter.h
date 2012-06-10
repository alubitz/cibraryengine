#pragma once
#include "StdAfx.h"

#include "RigidBody.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	using namespace std;

	struct MassInfo;
	struct ModelPhysics;

	class KinematicCharacterShape;
	class KinematicCharacterBone;

	class KinematicCharacterRig : public RigidBody
	{
		private:

			vector<KinematicCharacterBone*> bones;

			KinematicCharacterRig(KinematicCharacterShape* shape, const MassInfo& mass_info, const Vec3& pos, const Quaternion& ori);

		public:

			static KinematicCharacterRig* FromModelPhysics(ModelPhysics* mphys);	// "constructor" ish
	};

	class KinematicCharacterBone : public RigidBody
	{
		friend class KinematicCharacterRig;

			KinematicCharacterRig* rig;

		public:

			KinematicCharacterBone(KinematicCharacterRig* rig, CollisionShape* shape, const MassInfo& mass_info, const Vec3& pos, const Quaternion& ori);
	};

	class KinematicCharacterShape : public CollisionShape
	{
		friend class KinematicCharacterRig;

		private:

			KinematicCharacterRig* rig;

		public:

			KinematicCharacterShape();

			MassInfo ComputeMassInfo();

			AABB GetTransformedAABB(const Mat4& xform);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			unsigned int Read(istream& stream);
			void Write(ostream& stream);
	};
}
