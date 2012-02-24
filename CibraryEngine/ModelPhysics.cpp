#include "StdAfx.h"
#include "ModelPhysics.h"

#include "Content.h"

#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	/*
	 * ModelPhysics methods
	 */
	ModelPhysics::ModelPhysics() :
		skeleton(new Skeleton()),
		bones(),
		joints()
	{
	}

	void ModelPhysics::InnerDispose()
	{
		if(skeleton != NULL)
		{
			skeleton->Dispose();
			delete skeleton;
			skeleton = NULL;
		}
	}




	/*
	 * CollisionShapeLoader methods
	 */
	ModelPhysicsLoader::ModelPhysicsLoader(ContentMan* man) : ContentTypeHandler<ModelPhysics>(man) { }

	ModelPhysics* ModelPhysicsLoader::Load(ContentMetadata& what)
	{
		string filename = "Files/Physics/" + what.name + ".zzp";

		ModelPhysics* shape = NULL;
		if(unsigned int zzp_result = ModelPhysicsLoader::LoadZZP(shape, filename))
		{
			stringstream zzp_msg;
			zzp_msg << "LoadZZP (" << what.name << ") returned with status " << zzp_result << "!" << endl;
			Debug(zzp_msg.str());
		}

		return shape;
	}
	void ModelPhysicsLoader::Unload(ModelPhysics* content, ContentMetadata& what)
	{
		content->Dispose();
		delete content;
	}

	unsigned int ModelPhysicsLoader::LoadZZP(ModelPhysics*& shape, string filename)
	{
		// TODO: implement this
		return 1;
	}

	unsigned int ModelPhysicsLoader::SaveZZP(ModelPhysics* shape, string filename)
	{
		// TODO: implement this
		return 1;
	}
}
