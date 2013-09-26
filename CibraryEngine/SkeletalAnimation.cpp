#include "StdAfx.h"
#include "SkeletalAnimation.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "Texture1D.h"
#include "Model.h"

namespace CibraryEngine
{
	using boost::unordered_map;

	vector<Bone*> bone_recycle_bin;

	/*
	 * Bone methods
	 */
	Bone::Bone(unsigned int name, Bone* parent, Quaternion ori_, Vec3 pos) : name(name), parent(parent), ori(Quaternion::Identity()), pos(), rest_ori(ori_), rest_pos(pos), cache_valid(false) { }

	Mat4 Bone::GetTransformationMatrix()
	{
		if(!cache_valid)
		{
			Mat3 rm = (ori * rest_ori).ToMat3();
			float* arr = rm.values;

			if(parent == NULL)
			{
				cached_xform = Mat4(
					arr[0],	arr[1],	arr[2],	pos.x,
					arr[3],	arr[4],	arr[5],	pos.y,
					arr[6],	arr[7],	arr[8],	pos.z,
					0,		0,		0,		1
				);
			}
			else
			{
				Mat4 temp(
						arr[0],	arr[1],	arr[2],	rest_pos.x,
						arr[3],	arr[4],	arr[5],	rest_pos.y,
						arr[6],	arr[7],	arr[8],	rest_pos.z,
						0,		0,		0,		1
					);
				cached_xform = parent->GetTransformationMatrix() * temp * Mat4::Translation(pos - rest_pos);
			}

			cache_valid = true;
		}

		return cached_xform;
	}

	StringTable Bone::string_table = StringTable();




	/*
	 * Skeleton methods
	 */
	Skeleton::Skeleton() : bone_matrices(NULL), bones() { }

	Skeleton::Skeleton(Skeleton* prototype)			// hope that prototype is never NULL
	{
		unsigned int bone_count = prototype->bones.size();
		for(unsigned int i = 0; i < bone_count; ++i)
		{
			Bone* proto_bone = prototype->bones[i];
			AddBone(proto_bone->name, proto_bone->rest_ori, proto_bone->rest_pos);
		}

		for(unsigned int i = 0; i < bone_count; ++i)
			if(prototype->bones[i]->parent != NULL)
				for(unsigned int j = 0; j < bone_count; ++j)
					if(prototype->bones[i]->parent == prototype->bones[j])
					{
						bones[i]->parent = bones[j];
						break;
					}
	}

	void Skeleton::InnerDispose()
	{
		for(vector<Bone*>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
			bone_recycle_bin.push_back(*iter);

		bones.clear();
	}

	Bone* Skeleton::AddBone(unsigned int bone_name, Quaternion ori, Vec3 attach) { return AddBone(bone_name, NULL, ori, attach); }

	Bone* Skeleton::AddBone(unsigned int bone_name, Bone* parent, Quaternion ori, Vec3 attach)
	{
		Bone* bone;
		if(bone_recycle_bin.empty())
			bone = new Bone(bone_name, parent, ori, attach);
		else
		{
			bone = bone_recycle_bin[bone_recycle_bin.size() - 1];
			bone_recycle_bin.pop_back();

			*bone = Bone(bone_name, parent, ori, attach);
		}

		bones.push_back(bone);
		return bone;
	}

	Bone* Skeleton::GetNamedBone(string bone_name) { return GetNamedBone(Bone::string_table[bone_name]); }
	Bone* Skeleton::GetNamedBone(unsigned int bone_name)
	{
		for(vector<Bone*>::iterator iter = bones.begin(); iter != bones.end(); iter++)
			if((*iter)->name == bone_name)
				return *iter;
		return NULL;
	}

	void Skeleton::GetBoneMatrices(vector<Mat4>& results)
	{
		results.clear();
		for(vector<Bone*>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
			results.push_back((*iter)->GetTransformationMatrix());
	}

	void Skeleton::InvalidateCachedBoneXforms()
	{
		for(vector<Bone*>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
			(*iter)->cache_valid = false;
	}

	int Skeleton::ReadSkeleton(istream& file, Skeleton** skeleton)
	{
		Skeleton* temp = new Skeleton();

		unsigned int bone_count = ReadUInt32(file);

		unsigned int* parent_indices = new unsigned int[bone_count];

		for(unsigned int i = 0; i < bone_count; ++i)
		{
			unsigned char name_len = ReadByte(file);

			if(name_len == 0)
			{
				delete temp;
				return 2;
			}

			string name = "";
			for(unsigned char j = 0; j < name_len; ++j)
				name += ReadByte(file);

			parent_indices[i] = ReadUInt32(file);

			Vec3 rest_pos = ReadVec3(file);
			Quaternion rest_ori = ReadQuaternion(file);
			temp->AddBone(Bone::string_table[name], rest_ori, rest_pos);
		}

		for(unsigned int i = 0; i < bone_count; ++i)
		{
			unsigned int parent_index = parent_indices[i];
			temp->bones[i]->parent = parent_index == 0 ? NULL : temp->bones[parent_index - 1];
		}

		delete[] parent_indices;

		*skeleton = temp;
		return 0;
	}

	int Skeleton::WriteSkeleton(ostream& file, Skeleton* skeleton)
	{
		WriteUInt32(skeleton->bones.size(), file);
		for(unsigned int i = 0; i < skeleton->bones.size(); ++i)
		{
			Bone* bone = skeleton->bones[i];
			string name = Bone::string_table[bone->name];

			WriteByte((unsigned char)name.length(), file);
			for(unsigned int j = 0; j < name.length(); ++j)
				WriteByte((unsigned char)name.at(j), file);

			unsigned int parent_index = 0;

			Bone* parent = bone->parent;
			if(parent != NULL)
				for(unsigned int j = 0; j < skeleton->bones.size(); ++j)
					if(skeleton->bones[j] == parent)
					{
						parent_index = j + 1;
						break;
					}
			WriteUInt32(parent_index, file);

			WriteVec3(bone->rest_pos, file);
			WriteQuaternion(bone->rest_ori, file);
		}

		return 0;
	}




	/*
	 * PosedCharacter methods
	 */
	PosedCharacter::PosedCharacter(Skeleton* skeleton) : skeleton(skeleton), active_poses() { }
	void PosedCharacter::InnerDispose() { if(skeleton) { skeleton->Dispose(); delete skeleton; skeleton = NULL; } }

	void PosedCharacter::UpdatePoses(TimingInfo time)
	{
		skeleton->InvalidateCachedBoneXforms();

		// update poses and throw out ones which are no longer active
		for(list<Pose*>::iterator iter = active_poses.begin(); iter != active_poses.end();)
		{
			Pose* pose = *iter;
			pose->UpdatePose(time);

			if(pose->IsActive())
				++iter;
			else
				iter = active_poses.erase(iter);
		}

		// add up bone influences
		unordered_map<unsigned int, BoneInfluence> bone_states;

		for(list<Pose*>::iterator iter = active_poses.begin(); iter != active_poses.end(); ++iter)
		{
			Pose* pose = *iter;
			for(unordered_map<unsigned int, BoneInfluence>::iterator jter = pose->bones.begin(); jter != pose->bones.end(); ++jter)
			{
				unsigned int name = jter->first;

				if(bone_states.find(name) == bone_states.end())
					bone_states[name] = BoneInfluence();

				bone_states[name] += jter->second;
			}
		}

		// done adding; apply them to the bones
		int i = 0;
		for(vector<Bone*>::iterator iter = skeleton->bones.begin(); iter != skeleton->bones.end(); ++iter, ++i)
		{
			Bone* bone = *iter;

			unsigned int name = bone->name;
			unordered_map<unsigned int, BoneInfluence>::iterator found = bone_states.find(name);

			if(found != bone_states.end())
			{
				BoneInfluence state = found->second;
				bone->ori = Quaternion::FromPYR(state.ori);
				bone->pos = state.pos;
			}
		}

		// force recomputation of bone transformation matrices
		skeleton->InvalidateCachedBoneXforms();
	}




	/*
	 * SkinnedCharacter methods
	 */
	SkinnedCharacter::SkinnedCharacter(Skeleton* skeleton) : render_info(), mat_tex_precision(4096.0f), skeleton(skeleton) { }

	void SkinnedCharacter::InnerDispose()
	{
		if(skeleton)					{ skeleton->Dispose();					delete skeleton;					skeleton = NULL; }
		if(render_info.bone_matrices)	{ render_info.bone_matrices->Dispose();	delete render_info.bone_matrices;	render_info.bone_matrices = NULL; }
	}

	SkinnedCharacterRenderInfo SkinnedCharacter::GetRenderInfo()
	{
		if(!render_info.valid)
		{
			static vector<Mat4> matrices;
			skeleton->GetBoneMatrices(matrices);

			render_info.bone_matrices = SkinnedCharacter::MatricesToTexture1D(matrices, render_info.bone_matrices, mat_tex_precision);
			render_info.mat_tex_precision = mat_tex_precision;
			render_info.num_bones = matrices.size();

			render_info.valid = true;
		}

		return render_info;
	}

	Texture1D* SkinnedCharacter::MatricesToTexture1D(vector<Mat4>& matrices, Texture1D* existing_texture, float precision)
	{
		unsigned int matrix_count = matrices.size();
		const unsigned int max_bones = 128;

		matrix_count = matrix_count > max_bones ? max_bones : matrix_count;

		unsigned int size = matrix_count * 24;
		unsigned int use_size = 4;
		while(use_size < size)
			use_size <<= 1;				// multiply by 2, lol

		if(existing_texture != NULL && existing_texture->size != use_size / 4)
			Debug("Existing texture's size doesn't match required size!\n");

		unsigned char* bytes = existing_texture != NULL ? existing_texture->byte_data : new unsigned char[use_size];
		unsigned char* target = bytes;

		for(unsigned int i = 0; i < matrix_count; ++i)
		{
			Mat4& mat = matrices[i];
			for(unsigned int j = 0; j < 12; ++j)
			{
				float val = mat[j];								// get value from the matrix
				float expanded = val * precision + 32768.0f;	// scale to reasonable range

				unsigned int int_val = (unsigned int)(max(0.0f, min(65535.0f, expanded + 0.5f)));	// offset by 0.5 so it rounds nicely

				for(int k = 0; k < 2; ++k)
					*(target++) = (unsigned char)((int_val & (0xFF << (k * 8))) >> (k * 8));;
			}
		}

		if(existing_texture != NULL)
		{
			existing_texture->UpdateTextureData();
			return existing_texture;
		}
		else
			return new Texture1D(use_size / 4, bytes);
	}
}
