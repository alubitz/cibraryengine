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
		bones(),
		joints()
	{
	}

	void ModelPhysics::InnerDispose() { }




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
		string filename;

		ModelPhysics* phys;
		SkelChunkHandler(ModelPhysics* phys, const string& filename) : phys(phys), filename(filename) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			Debug(((stringstream&)(stringstream() << "Found deprecated chunk ModelPhysics::Skeleton in file \"" << filename << "\"" << endl)).str());

			Skeleton* ptr;
			Skeleton::ReadSkeleton(ss, &ptr);
			delete ptr;
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

			bone.bone_name = ReadString1(ss);

			bone.collision_shape = NULL;
			CollisionShape::ReadCollisionShape(bone.collision_shape, ss);

			bone.mass_info = MassInfo::ReadMassInfo(ss);
			
			phys->bones.push_back(bone);
		}
	};

	struct JointChunkHandler : public ChunkTypeFunction
	{
		vector<BinaryChunk>& joint_chunks;
		JointChunkHandler(vector<BinaryChunk>& joint_chunks) : joint_chunks(joint_chunks) { }

		void HandleChunk(BinaryChunk& chunk) { joint_chunks.push_back(chunk); }
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

		vector<BinaryChunk> joint_chunks;

		UnrecognizedChunkHandler badchunk;
		BoneChunkHandler bonechunk(temp);
		SkelChunkHandler skelchunk(temp, filename);
		JointChunkHandler jointchunk(joint_chunks);				// this handler will put all of the joint chunks into a list to be processed at the end

		indexer.SetDefaultHandler(&badchunk);
		indexer.SetHandler("BONE____", &bonechunk);
		indexer.SetHandler("SKEL____", &skelchunk);
		indexer.SetHandler("JOINT___", &jointchunk);

		indexer.HandleChunk(whole);
		file.close();

		for(vector<BinaryChunk>::iterator iter = joint_chunks.begin(); iter != joint_chunks.end(); ++iter)
		{
			BinaryChunk& joint_chunk = *iter;

			istringstream ss(joint_chunk.data);

			ModelPhysics::JointPhysics joint;

			joint.joint_name = ReadString1(ss);
			joint.bone_a = ReadUInt32(ss);
			joint.bone_b = ReadUInt32(ss);
			joint.pos = ReadVec3(ss);
			joint.axes = ReadMat3(ss);
			joint.max_extents = ReadVec3(ss);
			joint.angular_damp = ReadVec3(ss);

			temp->joints.push_back(joint);
		}

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

		if(!phys->bones.empty())
		{
			for(vector<ModelPhysics::BonePhysics>::iterator iter = phys->bones.begin(); iter != phys->bones.end(); ++iter)
			{
				BinaryChunk bone_chunk("BONE____");
				stringstream bone_ss;

				ModelPhysics::BonePhysics& bone = *iter;

				WriteString1(bone.bone_name, bone_ss);

				CollisionShape::WriteCollisionShape(bone.collision_shape, bone_ss);
				bone.mass_info.Write(bone_ss);
				
				bone_chunk.data = bone_ss.str();
				bone_chunk.Write(ss);
			}
		}

		if(!phys->joints.empty())
		{
			for(vector<ModelPhysics::JointPhysics>::iterator iter = phys->joints.begin(); iter != phys->joints.end(); ++iter)
			{
				BinaryChunk joint_chunk("JOINT___");
				stringstream joint_ss;

				ModelPhysics::JointPhysics& joint = *iter;

				WriteString1(joint.joint_name, joint_ss);
				WriteUInt32(joint.bone_a, joint_ss);
				WriteUInt32(joint.bone_b, joint_ss);
				WriteVec3(joint.pos, joint_ss);
				WriteMat3(joint.axes, joint_ss);
				WriteVec3(joint.max_extents, joint_ss);
				WriteVec3(joint.angular_damp, joint_ss);

				joint_chunk.data = joint_ss.str();
				joint_chunk.Write(ss);
			}
		}

		whole.data = ss.str();

		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 2;

		whole.Write(file);

		return 0;
	}
}
