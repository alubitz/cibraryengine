#include "StdAfx.h"
#include "ModelPhysics.h"

#include "CollisionShape.h"

#include "Content.h"
#include "SkeletalAnimation.h"

#include "Serialize.h"
#include "BinaryChunk.h"

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
			Debug(((stringstream&)(stringstream() << "LoadZZP (" << what.name << ") returned with status " << zzp_result << "!" << endl)).str());

		return shape;
	}

	void ModelPhysicsLoader::Unload(ModelPhysics* content, ContentMetadata& what)
	{
		content->Dispose();
		delete content;
	}

	struct UnrecognizedChunkHandler : public ChunkTypeFunction { void HandleChunk(BinaryChunk& chunk) { Debug("Unrecognized chunk: " + chunk.GetName() + "\n"); } };

	struct SkelChunkHandler : public ChunkTypeFunction
	{
		ModelPhysics* phys;
		SkelChunkHandler(ModelPhysics* phys) : phys(phys) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			Skeleton::ReadSkeleton(ss, &phys->skeleton);
		}
	};

	struct BoneChunkHandler : public ChunkTypeFunction
	{
		ModelPhysics* phys;
		BoneChunkHandler(ModelPhysics* phys) : phys(phys) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			ModelPhysics::BonePhysics bone;

			bone.bone_name = ReadUInt32(ss);

			bone.collision_shape = NULL;
			CollisionShape::ReadCollisionShape(bone.collision_shape, ss);

			bone.mass_info = MassInfo::ReadMassInfo(ss);
			
			phys->bones.push_back(bone);
		}
	};

	unsigned int ModelPhysicsLoader::LoadZZP(ModelPhysics*& phys, const string& filename)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		BinaryChunk whole;
		whole.Read(file);

		if(whole.GetName() != "MPHYS___")
			return 2;

		ModelPhysics* temp = new ModelPhysics();

		ChunkTypeIndexer indexer;

		UnrecognizedChunkHandler badchunk;
		BoneChunkHandler bonechunk(temp);
		SkelChunkHandler skelchunk(temp);

		indexer.SetDefaultHandler(&badchunk);
		indexer.SetHandler("BONE____", &bonechunk);
		indexer.SetHandler("SKEL____", &skelchunk);

		// TODO: read joints here

		indexer.HandleChunk(whole);
		file.close();

		// TODO: process bone names after reading them?

		phys = temp;

		return 0;
	}

	unsigned int ModelPhysicsLoader::SaveZZP(ModelPhysics* phys, const string& filename)
	{
		if(phys == NULL)
			return 1;

		BinaryChunk whole;
		whole.SetName("MPHYS___");

		stringstream ss;

		if(phys->skeleton != NULL)
		{
			BinaryChunk skel_chunk("SKEL____");
			stringstream skel_ss;

			if(unsigned int skel_result = Skeleton::WriteSkeleton(skel_ss, phys->skeleton))
				return 3;

			skel_chunk.data = skel_ss.str();
			skel_chunk.Write(ss);
		}

		if(!phys->bones.empty())
		{
			for(vector<ModelPhysics::BonePhysics>::iterator iter = phys->bones.begin(); iter != phys->bones.end(); ++iter)
			{
				BinaryChunk bone_chunk("BONE____");
				stringstream bone_ss;

				ModelPhysics::BonePhysics& bone = *iter;

				// TODO: process bone name before writing it?
				WriteUInt32(bone.bone_name, bone_ss);

				CollisionShape::WriteCollisionShape(bone.collision_shape, bone_ss);
				bone.mass_info.Write(bone_ss);
				
				bone_chunk.data = bone_ss.str();
				bone_chunk.Write(ss);
			}
		}

		// TODO: write joints here

		whole.data = ss.str();

		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 2;

		whole.Write(file);

		return 0;
	}
}
