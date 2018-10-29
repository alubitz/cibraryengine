#include "StdAfx.h"
#include "ModelPhysics.h"

#include "CollisionShape.h"

#include "Content.h"
#include "SkeletalAnimation.h"

#include "Serialize.h"
#include "BinaryChunk.h"

// for lua-to-file converter function
#include "Sphere.h"
#include "MultiSphereShape.h"

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
	 * ModelPhysics::JointPhysics methods
	 */
	void ModelPhysics::JointPhysics::ClampAngles(Vec3& ori) const
	{
		ori.x = max(min_extents.x, min(max_extents.x, ori.x));
		ori.y = max(min_extents.y, min(max_extents.y, ori.y));
		ori.z = max(min_extents.z, min(max_extents.z, ori.z));
	}

	Vec3 ModelPhysics::JointPhysics::GetClampedAngles(const Vec3& ori) const
	{
		return Vec3(
			max(min_extents.x, min(max_extents.x, ori.x)),
			max(min_extents.y, min(max_extents.y, ori.y)),
			max(min_extents.z, min(max_extents.z, ori.z))
		);
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
			if(unsigned int error = CollisionShape::ReadCollisionShape(bone.collision_shape, ss))
				Debug(((stringstream&)(stringstream() << "Failed to read collision shape from stream (error code = " << error << ")" << endl)).str());

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
		indexer.SetHandler("JOINT2__", &jointchunk);

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
			joint.min_extents = ReadVec3(ss);
			joint.max_extents = ReadVec3(ss);
			if(joint_chunk.GetName() == "JOINT2__")
			{
				joint.min_torque = ReadVec3(ss);
				joint.max_torque = ReadVec3(ss);
			}
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
				BinaryChunk joint_chunk("JOINT2__");
				stringstream joint_ss;

				ModelPhysics::JointPhysics& joint = *iter;

				WriteString1(joint.joint_name, joint_ss);
				WriteUInt32(joint.bone_a, joint_ss);
				WriteUInt32(joint.bone_b, joint_ss);
				WriteVec3(joint.pos, joint_ss);
				WriteMat3(joint.axes, joint_ss);
				WriteVec3(joint.min_extents, joint_ss);
				WriteVec3(joint.max_extents, joint_ss);
				WriteVec3(joint.min_torque, joint_ss);
				WriteVec3(joint.max_torque, joint_ss);
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




	/*
	 * ModelPhysics scripting stuff
	 */
	static void DebugMassInfo(const MassInfo& minfo)
	{
		stringstream ss;
		ss << "mass = " << minfo.mass << endl;
		ss << "com = (" << minfo.com.x << ", " << minfo.com.y << ", " << minfo.com.z << ")" << endl;
		ss << "moi = { ";
		for(unsigned int i = 0; i < 9; ++i)
		{
			if(i != 0)
				ss << ", ";
			ss << minfo.moi[i];
		}
		ss << " }" << endl;
		Debug(ss.str());
	}

	int ba_saveModelPhysics(lua_State* L)
	{
		int n = lua_gettop(L);

		if(n == 3 && lua_istable(L, 1) && lua_istable(L, 2) && lua_isstring(L, 3))
		{
			vector<CollisionShape*> shapes_created;
			ModelPhysics mphys;

			string short_name = lua_tostring(L, 3);
			string filename = "Files/Physics/" + short_name + ".zzp";

			// adding bones...
			unsigned int num_bones = lua_objlen(L, 1);
			for(unsigned int i = 1; i <= num_bones; ++i)
			{
				lua_pushinteger(L, i);
				lua_gettable(L, 1);				// puts the i-th element of the bones table on top of the stack

				if(lua_istable(L, -1))
				{
					ModelPhysics::BonePhysics bone;
					bone.collision_shape = NULL;

					lua_pushstring(L, "name");
					lua_gettable(L, -2);
					if(lua_isstring(L, -1))
						bone.bone_name = lua_tostring(L, -1);
					lua_pop(L, 1);

					lua_pushstring(L, "shape");
					lua_gettable(L, -2);

					if(lua_istable(L, -1))
					{
						// load a multisphereshape
						unsigned int num_spheres = lua_objlen(L, -1);

						Sphere* spheres = new Sphere[num_spheres];
						for(unsigned int j = 0; j < num_spheres; ++j)
						{
							lua_pushinteger(L, j + 1);
							lua_gettable(L, -2);

							if(lua_istable(L, -1))
							{
								// load a sphere

								lua_pushstring(L, "center");
								lua_gettable(L, -2);
								if(lua_isuserdata(L, -1))
									spheres[j].center = *(Vec3*)lua_touserdata(L, -1);
								lua_pop(L, 1);

								lua_pushstring(L, "radius");
								lua_gettable(L, -2);
								if(lua_isnumber(L, -1))
									spheres[j].radius = (float)lua_tonumber(L, -1);
								lua_pop(L, 1);
							}

							lua_pop(L, 1);
						}

						bone.collision_shape = new MultiSphereShape(spheres, num_spheres);
						shapes_created.push_back(bone.collision_shape);

						delete[] spheres;

					}
					lua_pop(L, 1);

					bool mass_info = false;				// if we don't find a mass_info we will look for a mass, and generate a mass_info from the collision shape if possible
					lua_pushstring(L, "mass_info");
					lua_gettable(L, -2);
					if(lua_istable(L, -1))
					{
						mass_info = true;

						lua_pushstring(L, "mass");
						lua_gettable(L, -2);
						if(lua_isnumber(L, -1))
							bone.mass_info.mass = (float)lua_tonumber(L, -1);
						lua_pop(L, 1);

						lua_pushstring(L, "com");
						lua_gettable(L, -2);
						if(lua_isuserdata(L, -1))
							bone.mass_info.com = *(Vec3*)lua_touserdata(L, -1);
						lua_pop(L, 1);

						lua_pushstring(L, "moi");
						lua_gettable(L, -2);
						if(lua_istable(L, -1))
						{
							unsigned int array_len = lua_objlen(L, -1);
							if(array_len >= 9)
								for(unsigned int j = 0; j < 9; ++j)
								{
									lua_pushinteger(L, j + 1);
									lua_gettable(L, -2);
									if(lua_isnumber(L, -1))
										bone.mass_info.moi[j] = (float)lua_tonumber(L, -1);
									lua_pop(L, 1);
								}
						}
						lua_pop(L, 1);
					}
					lua_pop(L, 1);

					if(!mass_info)
					{
						lua_pushstring(L, "mass");
						lua_gettable(L, -2);
						if(lua_isnumber(L, -1))
						{
							float mass = (float)lua_tonumber(L, -1);

							if(bone.collision_shape != NULL)
							{
								bone.mass_info = bone.collision_shape->ComputeMassInfo();
								bone.mass_info *= mass / bone.mass_info.mass;
								//DebugMassInfo(bone.mass_info);
							}
							else
								bone.mass_info.mass = mass;
						}
						lua_pop(L, 1);
					}

					mphys.bones.push_back(bone);
				}

				lua_pop(L, 1);
			}

			// adding joints...
			unsigned int num_joints = lua_objlen(L, 2);
			for(unsigned int i = 1; i <= num_joints; ++i)
			{
				lua_pushinteger(L, i);
				lua_gettable(L, 2);				// puts the i-th element of the joints table on top of the stack

				if(lua_istable(L, -1))
				{
					ModelPhysics::JointPhysics joint;
					joint.min_extents = Vec3(-1, -1, -1);
					joint.max_extents = Vec3(1, 1, 1);
					joint.angular_damp = Vec3(1, 1, 1);

					lua_pushstring(L, "name");
					lua_gettable(L, -2);
					if(lua_isstring(L, -1))
						joint.joint_name = lua_tostring(L, -1);
					lua_pop(L, 1);

					lua_pushstring(L, "bone_a");
					lua_gettable(L, -2);
					if(lua_isnumber(L, -1))
						joint.bone_a = (unsigned int)lua_tointeger(L, -1);
					lua_pop(L, 1);

					lua_pushstring(L, "bone_b");
					lua_gettable(L, -2);
					if(lua_isnumber(L, -1))
						joint.bone_b = (unsigned int)lua_tointeger(L, -1);
					lua_pop(L, 1);

					lua_pushstring(L, "pos");
					lua_gettable(L, -2);
					if(lua_isuserdata(L, -1))
					{
						Vec3* vec = (Vec3*)lua_touserdata(L, -1);
						joint.pos = *vec;
					}
					lua_pop(L, 1);

					lua_pushstring(L, "axes");
					lua_gettable(L, -2);
					if(lua_istable(L, -1))
					{
						// get up to 3 axis vectors and store them in a Mat3
						unsigned int count = min(3u, lua_objlen(L, -1));
						for(unsigned int i = 0; i < count; ++i)
						{
							lua_pushinteger(L, i + 1);
							lua_gettable(L, -2);
							if(lua_isuserdata(L, -1))
							{
								Vec3 axis = *(Vec3*)lua_touserdata(L, -1);

								float magsq = axis.ComputeMagnitudeSquared();
								if(magsq > 0)
								{
									axis /= sqrtf(magsq);
									joint.axes[i * 3    ] = axis.x;
									joint.axes[i * 3 + 1] = axis.y;
									joint.axes[i * 3 + 2] = axis.z;
								}
							}
							lua_pop(L, 1);
						}

						// special case where quaternion-and-back may break down
						if(count == 2)
						{
							Vec3 cross = Vec3::Cross(Vec3(joint.axes[0], joint.axes[1], joint.axes[2]), Vec3(joint.axes[3], joint.axes[4], joint.axes[5]));

							float magsq = cross.ComputeMagnitudeSquared();
							if(magsq > 0.0f)
								cross /= sqrtf(magsq);

							joint.axes[6] = cross.x;
							joint.axes[7] = cross.y;
							joint.axes[8] = cross.z;
						}

						joint.axes = Quaternion::FromRotationMatrix(joint.axes).ToMat3();			// handles degenerate matrices just fine
					}
					else
						joint.axes = Mat3::Identity();
					lua_pop(L, 1);

					lua_pushstring(L, "min_extents");
					lua_gettable(L, -2);
					if(lua_isuserdata(L, -1))
					{
						Vec3* vec = (Vec3*)lua_touserdata(L, -1);
						joint.min_extents = *vec;
					}
					lua_pop(L, 1);

					lua_pushstring(L, "max_extents");
					lua_gettable(L, -2);
					if(lua_isuserdata(L, -1))
					{
						Vec3* vec = (Vec3*)lua_touserdata(L, -1);
						joint.max_extents = *vec;
					}
					lua_pop(L, 1);

					lua_pushstring(L, "min_torque");
					lua_gettable(L, -2);
					if(lua_isuserdata(L, -1))
					{
						Vec3* vec = (Vec3*)lua_touserdata(L, -1);
						joint.min_torque = *vec;
					}
					lua_pop(L, 1);

					lua_pushstring(L, "max_torque");
					lua_gettable(L, -2);
					if(lua_isuserdata(L, -1))
					{
						Vec3* vec = (Vec3*)lua_touserdata(L, -1);
						joint.max_torque = *vec;
					}
					lua_pop(L, 1);

					mphys.joints.push_back(joint);
				}

				lua_pop(L, 1);
			}

			if(unsigned int error = ModelPhysicsLoader::SaveZZP(&mphys, filename))
				Debug(((stringstream&)(stringstream() << "SaveZZP returned status " << error << "!" << endl)).str());
			else
				Debug("Successfully saved ModelPhysics as \"" + filename + "\"\n");

			for(vector<CollisionShape*>::iterator iter = shapes_created.begin(); iter != shapes_created.end(); ++iter)
				delete *iter;
			shapes_created.clear();

			mphys.Dispose();
			return 0;
		}

		Debug("ba.saveModelPhysics takes 3 parameters: a table of bones, a table of joints, and a string (the \"short name\" of the object to save)\n");
		return 0;
	}
}
