#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "ContentTypeHandler.h"
#include "Physics.h"

namespace CibraryEngine
{
	using namespace std;

	class Skeleton;

	/** Physics data to go alongside a model */
	struct ModelPhysics : public Disposable
	{
		protected:

			void InnerDispose();

		public:

			struct BonePhysics
			{
				unsigned int bone_name;

				CollisionShape* collision_shape;
				MassInfo mass_info;
			};

			struct JointPhysics { };

			Skeleton* skeleton;

			vector<BonePhysics> bones;
			vector<JointPhysics> joints;
		
			ModelPhysics();
	};

	struct ModelPhysicsLoader : public ContentTypeHandler<ModelPhysics>
	{
		ModelPhysicsLoader(ContentMan* man);

		ModelPhysics* Load(ContentMetadata& what);
		void Unload(ModelPhysics* content, ContentMetadata& what);

		static unsigned int LoadZZP(ModelPhysics*& phys, string filename);
		static unsigned int SaveZZP(ModelPhysics* phys, string filename);
	};
}
