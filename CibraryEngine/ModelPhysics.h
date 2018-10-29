#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "ContentTypeHandler.h"
#include "Physics.h"
#include "MassInfo.h"

#include "Matrix.h"

namespace CibraryEngine
{
	using namespace std;

	/** Physics data to go alongside a model */
	struct ModelPhysics : public Disposable
	{
		protected:

			void InnerDispose();

		public:

			struct BonePhysics
			{
				string bone_name;				// renderer will attempt to match this with a name in the UberModel's skeleton

				CollisionShape* collision_shape;
				MassInfo mass_info;
			};

			struct JointPhysics
			{
				string joint_name;					// only particularly relevant for combination with other objects (i.e. when bone_b is 0)

				/** Index into the bones vector + 1; 0 means no bone (joint specification exists for combination with other objects) */
				unsigned int bone_a;
				unsigned int bone_b;

				Vec3 pos;
				Mat3 axes;

				Vec3 min_extents, max_extents;
				Vec3 min_torque, max_torque;
				Vec3 angular_damp;

				void ClampAngles(Vec3& ori) const;
				Vec3 GetClampedAngles(const Vec3& ori) const;
			};

			vector<BonePhysics> bones;
			vector<JointPhysics> joints;
		
			ModelPhysics();
	};

	struct ModelPhysicsLoader : public ContentTypeHandler<ModelPhysics>
	{
		ModelPhysicsLoader(ContentMan* man);

		ModelPhysics* Load(ContentMetadata& what);
		void Unload(ModelPhysics* content, ContentMetadata& what);

		static unsigned int LoadZZP(ModelPhysics*& phys, const string& filename);
		static unsigned int SaveZZP(ModelPhysics* phys, const string& filename);
	};

	int ba_saveModelPhysics(lua_State* L);
}
