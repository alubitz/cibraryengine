#include "StdAfx.h"
#pragma pack(push, 4)

#include "UnrealImport.h"

#include "UnclampedVertexBoneWeightInfo.h"

#include "Vector.h"
#include "Model.h"
#include "VertexInfo.h"
#include "Serialize.h"

#include "SkeletalAnimation.h"
#include "KeyframeAnimation.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * Importer for unreal's PSK skinned mesh and skeleton format
	 */
	int LoadPSK(string filename, SkinnedModel* model, float scale)
	{
		// open the file
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		// load the stuff from the file directly into memory
		VChunkHeader general_header;
		file.read((char*)&general_header, sizeof(VChunkHeader));

		VChunkHeader points_header;
		file.read((char*)&points_header, sizeof(VChunkHeader));
		VPoint* points = new VPoint[points_header.DataCount];
		for(int i = 0; i < points_header.DataCount; i++)
			file.read((char*)&points[i], sizeof(VPoint));

		VChunkHeader wedge_header;
		file.read((char*)&wedge_header, sizeof(VChunkHeader));
		VVertex* wedges = new VVertex[wedge_header.DataCount];
		for(int i = 0; i < wedge_header.DataCount; i++)
			file.read((char*)&wedges[i], sizeof(VVertex));

		VChunkHeader faces_header;
		file.read((char*)&faces_header, sizeof(VChunkHeader));
		VTriangle* faces = new VTriangle[faces_header.DataCount];
		for(int i = 0; i < faces_header.DataCount; i++)
			file.read((char*)&faces[i], sizeof(VTriangle));

		VChunkHeader mats_header;
		file.read((char*)&mats_header, sizeof(VChunkHeader));
		VMaterial* mats = new VMaterial[mats_header.DataCount];
		for(int i = 0; i < mats_header.DataCount; i++)
			file.read((char*)&mats[i], sizeof(VMaterial));

		VChunkHeader bones_header;
		file.read((char*)&bones_header, sizeof(VChunkHeader));
		VBone* bones = new VBone[bones_header.DataCount];
		for(int i = 0; i < bones_header.DataCount; i++)
			file.read((char*)&bones[i], sizeof(VBone));

		VChunkHeader influence_header;
		file.read((char*)&influence_header, sizeof(VChunkHeader));
		VRawBoneInfluence* influences = new VRawBoneInfluence[influence_header.DataCount];
		for(int i = 0; i < influence_header.DataCount; i++)
			file.read((char*)&influences[i], sizeof(VRawBoneInfluence));

		file.close();				// done with the file

		Vec3* face_normals = new Vec3[faces_header.DataCount];
		SkinVInfo* vertex_infos = new SkinVInfo[faces_header.DataCount * 3];

		// shove position and texture coordinates into the vertex info array
		// face normals will also be computed here
		for(int i = 0; i < faces_header.DataCount; i++)
		{
			VTriangle& tri = faces[i];

			Vec3 xyz[3];
			Vec3 uvw[3];

			for(int j = 0; j < 3; j++)
			{
				VVertex& wedge = wedges[tri.WedgeIndex[j]];
				FVector& point = points[wedge.PointIndex].Point;

				//xyz[j] = Vec3(point.X, point.Y, point.Z) * scale;
				xyz[j] = Vec3(-point.X, point.Z, point.Y) * scale;
				uvw[j] = Vec3(wedge.U, 1.0f - wedge.V, 0.0f);
				vertex_infos[i * 3 + j] = SkinVInfo(xyz[j], uvw[j], Vec3());
			}

			face_normals[i] = Vec3::Normalize(Vec3::Cross(xyz[1] - xyz[0], xyz[2] - xyz[0]));
		}

		// process the bone influences list
		VertexBoneWeightInfo* weights = new VertexBoneWeightInfo[points_header.DataCount];
		for(int i = 0; i < points_header.DataCount; i++)
			weights[i].point_index = i;

		for(int i = 0; i < influence_header.DataCount; i ++)
		{
			VRawBoneInfluence& influence = influences[i];

			weights[influence.PointIndex].AddValue(influence.BoneIndex, influence.Weight);
		}

		// assign vertex normals based on face normals and smoothing groups
		for(int i = 0; i < faces_header.DataCount; i++)
		{
			VTriangle& tri = faces[i];
			DWORD smoothing_groups = tri.SmoothingGroups;
			for(int j = 0; j < 3; j++)
			{
				int point_index = wedges[tri.WedgeIndex[j]].PointIndex;
				SkinVInfo& vertex_info = vertex_infos[i * 3 + j];

				Vec3 normal_total = Vec3();
				for(int k = 0; k < faces_header.DataCount; k++)
				{
					VTriangle& other_tri = faces[k];
					for(int l = 0; l < 3; l++)
						if(wedges[other_tri.WedgeIndex[l]].PointIndex == point_index)
							if((smoothing_groups & other_tri.SmoothingGroups) != 0)
								normal_total += face_normals[k];
				}

				float mag_sq = normal_total.ComputeMagnitudeSquared();
				vertex_info.n = mag_sq > 0.0f ? normal_total / sqrtf(mag_sq) : face_normals[i];

				// match it with a vertex weight info
				weights[point_index].GetByteValues(vertex_info.indices, vertex_info.weights);
			}
		}

		// shove mmps into skinned model
		for(int i = 0; i < mats_header.DataCount; i++)
		{
			VMaterial& mat = mats[i];
			string name = mat.MaterialName;

			model->material_names.push_back(name);

			MaterialModelPair mmp = MaterialModelPair();
			mmp.material_index = i;
			SkinVInfoVertexBuffer* vbo = new SkinVInfoVertexBuffer();
			mmp.vbo = vbo;

			for(int j = 0; j < faces_header.DataCount; j++)
				if(faces[i].MatIndex == i)
					vbo->AddTriangleVertexInfo(vertex_infos[j * 3 + 0], vertex_infos[j * 3 + 2], vertex_infos[j * 3 + 1]);

			model->material_model_pairs.push_back(mmp);
		}

		// finally, load the skeleton
		model->skeleton = new Skeleton();
		model->skeleton->bones.push_back(new Bone("root", NULL, Quaternion::Identity(), Vec3()));			// PSK does not have a root bone, it's bone 0 is our bone 1
		for(int i = 0; i < bones_header.DataCount; i++)
		{
			VBone& bone = bones[i];
			VJointPos& bone_pos = bone.BonePos;

			string bone_name = bone.Name;
			bone_name.erase(bone_name.find_last_not_of(" ") + 1);			// trim trailing spaces

			//model->skeleton->bones.push_back(new Bone(bone_name, NULL, Quaternion(bone_pos.Orientation.X, bone_pos.Orientation.Y, bone_pos.Orientation.Z, bone_pos.Orientation.W), Vec3(bone_pos.Position.X, bone_pos.Position.Y, bone_pos.Position.Z) * scale));
			model->skeleton->bones.push_back(new Bone(bone_name, NULL, Quaternion(-bone_pos.Orientation.X, bone_pos.Orientation.Z, bone_pos.Orientation.Y, bone_pos.Orientation.W), Vec3(-bone_pos.Position.X, bone_pos.Position.Z, bone_pos.Position.Y) * scale));

			Debug("Bone \"" + bone_name + "\"\n");
		}
		for(int i = 0; i < bones_header.DataCount; i++)
		{
			VBone& bone = bones[i];
			INT parent_index = bone.ParentIndex;
			model->skeleton->bones[i + 1]->parent = model->skeleton->bones[parent_index];
		}

		delete[] points;
		delete[] wedges;
		delete[] faces;
		delete[] mats;
		delete[] bones;
		delete[] influences;

		delete[] face_normals;
		delete[] vertex_infos;
		delete[] weights;

		return 0;
	}




	/*
	 * Importer for unreal's PSA animation format
	 */
	int LoadPSA(string filename, vector<KeyframeAnimation>& animations, float scale)
	{
		// open the file
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		// load the stuff from the file directly into memory
		VChunkHeader general_header;
		file.read((char*)&general_header, sizeof(VChunkHeader));

		VChunkHeader bones_header;
		file.read((char*)&bones_header, sizeof(VChunkHeader));
		FNamedBoneBinary* bones = new FNamedBoneBinary[bones_header.DataCount];
		for(int i = 0; i < bones_header.DataCount; i++)
			file.read((char*)&bones[i], bones_header.DataSize);

		VChunkHeader anims_header;
		file.read((char*)&anims_header, sizeof(VChunkHeader));
		AnimInfoBinary* anims = new AnimInfoBinary[anims_header.DataCount];
		for(int i = 0; i < anims_header.DataCount; i++)
			file.read((char*)&anims[i], anims_header.DataSize);

		VChunkHeader raw_keys_header;
		file.read((char*)&raw_keys_header, sizeof(VChunkHeader));
		VQuatAnimKey* raw_keys = new VQuatAnimKey[raw_keys_header.DataCount];
		for(int i = 0; i < raw_keys_header.DataCount; i++)
			file.read((char*)&raw_keys[i], raw_keys_header.DataSize);

		VChunkHeader scale_keys_header;
		file.read((char*)&scale_keys_header, sizeof(VChunkHeader));
		VScaleAnimKey* scale_keys = new VScaleAnimKey[scale_keys_header.DataCount];
		for(int i = 0; i < scale_keys_header.DataCount; i++)
			file.read((char*)&scale_keys[i], scale_keys_header.DataSize);

		// ignoring curve keys because i have no idea wtf those are

		VQuatAnimKey* key_pointer = &raw_keys[0];
		for(int i = 0; i < anims_header.DataCount; i++)
		{
			AnimInfoBinary& anim = anims[i];

			int total_bones = anim.TotalBones;
			int frame_count = anim.NumRawFrames;

			float frame_length = anim.AnimRate == 0.0f ? 0.0f : 1.0f / anim.AnimRate;

			string anim_name = anim.Name;
			anim_name.erase(anim_name.find_last_not_of(" ") + 1);			// trim trailing spaces

			KeyframeAnimation keyframe_animation = KeyframeAnimation(anim_name);
			for(int j = 0; j < frame_count; j++)
			{
				Keyframe frame = Keyframe(frame_length);

				frame.next = j + 1;
				if(frame.next == frame_count)
					frame.next = 0;

				keyframe_animation.frames.push_back(frame);
			}

			for(int j = 0; j < total_bones; j++)
				for(int k = 0; k < frame_count; k++)
				{
					VQuatAnimKey key = *key_pointer;

					Vec3 ori = Quaternion(-key.Orientation.X, key.Orientation.Y, key.Orientation.Z, key.Orientation.W).ToPYR();
					//Vec3 pos = Vec3(key.Position.X, key.Position.Y, key.Position.Z) * scale;
					Vec3 pos = Vec3(-key.Position.X, key.Position.Z, key.Position.Y) * scale;

					string bone_name = bones[j].Name;
					bone_name.erase(bone_name.find_last_not_of(" ") + 1);			// trim trailing spaces

					keyframe_animation.frames[k].values[bone_name] = BoneInfluence(ori, pos, 1.0f);

					key_pointer++;
				}

			animations.push_back(keyframe_animation);
		}

		delete[] bones;
		delete[] anims;
		delete[] raw_keys;
		delete[] scale_keys;

		return 0;
	}
}
