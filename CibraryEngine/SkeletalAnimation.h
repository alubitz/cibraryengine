#pragma once

#include "StdAfx.h"

#include "TimingInfo.h"
#include "Disposable.h"

#include "Matrix.h"
#include "Vector.h"
#include "Quaternion.h"

#include "Content.h"
#include "StringTable.h"

namespace CibraryEngine
{
	using namespace std;

	class Texture1D;

	/** Class representing a bone of a skeleton */
	class Bone
	{
		public:

			/** The name of this bone */
			unsigned int name;

			/** The bone to which this bone is attached */
			Bone* parent;

			/** The orientation of this bone relative to its parent, or in the world coordinate system if it has none */
			Quaternion ori;
			/** The position of this bone relative to its point of attachment, or in the world if it has none */
			Vec3 pos;

			/** The default orientation of this bone */
			Quaternion rest_ori;
			/** The default position of this bone */
			Vec3 rest_pos;

			Bone(unsigned int, Bone* parent, Quaternion ori, Vec3 pos);

			/** Gets a 4x4 transformation matrix representing the position and orientation of this bone, computing the transformation matrices of its ancestors in the parent hierarchy as necessary */
			Mat4 GetTransformationMatrix();




			static StringTable string_table;
	};

	/** Class representing an arrangement of bones */
	class Skeleton : public Disposable
	{
		protected:

			void InnerDispose();

		public:

			Texture1D* bone_matrices;

			/** All of the bones in this skeleton */
			vector<Bone*> bones;

			Skeleton();
			/** Initializes a skeleton as a copy of an existing skeleton */
			Skeleton(Skeleton* prototype);

			/** Adds a bone with the specified name, default orientation, and default point of attachment */
			Bone* AddBone(unsigned int, Quaternion ori, Vec3 attach);
			/** Adds a bone with the specified name, parent bone, default orientation, and default point of attachment */
			Bone* AddBone(unsigned int, Bone* parent, Quaternion ori, Vec3 attach);

			/** Reads a skeleton from a stream, and returns 0 if ok or an int error code */
			static int ReadSkeleton(ifstream& file, Skeleton** skeleton);
			/** Writes a skeleton from a stream, and returns 0 if ok or an int error code */
			static int WriteSkeleton(ofstream& file, Skeleton* skeleton);

			vector<Mat4> GetBoneMatrices();
	};

	class SkinnedModel;
	class Material;

	class Pose;

	/** Class containing a skinned model, a skeleton, and a collection of active poses affecting it */
	class SkinnedCharacter : public Disposable
	{
		protected:

			void InnerDispose();

		public:

			Texture1D* bone_matrices;

			/** The model for this character */
			SkinnedModel* skin;
			/** A copy of the Skeleton from the SkinnedModel, with orientations and offsets specified by the character's active poses */
			Skeleton* skeleton;

			/** The poses affecting this character's skeleton */
			list<Pose*> active_poses;

			/** Initializes a new SkinnedCharacter with the given skin */
			SkinnedCharacter(SkinnedModel* skin);

			/** Initializess a new SkinnedCharacter, passing the skeleton as parameter; no copying here */
			SkinnedCharacter(Skeleton* skeleton);

			/** Updates the poses of this character */
			void UpdatePoses(TimingInfo time);

			Texture1D* GetBoneMatrices();

			/** Static utility function to generate a 1-dimensional texture which encodes several transformation matrices, for use by my awesome vertex shader; remember to dispose of and delete the result! */
			static Texture1D* MatricesToTexture1D(vector<Mat4>& matrices);
	};

	/** Class representing how a Pose affects a certain Bone */
	struct BoneInfluence
	{
		/** The rotation to apply to the bone */
		Vec3 ori;
		/** How to translate the bone relative to its attachment position */
		Vec3 pos;
		/** I don't remember? */
		float div;

		/** Initializes a default BoneInfluence which does nothing */
		BoneInfluence() : ori(), pos(), div(0) { }
		/** Initializes a BoneInfluence, specifying the rotation, translation, and idr */
		BoneInfluence(Vec3 ori, Vec3 pos, float div) : ori(ori), pos(pos), div(div) { }

		/** Adds two bone influences */
		void operator +=(BoneInfluence& other) { ori += other.ori; pos += other.pos; div += other.div; }
		/** Adds two bone influences */
		BoneInfluence operator +(BoneInfluence& other) { BoneInfluence temp = *this; temp += other; return temp; }
	};

	/** Abstract class representing a pose or animation of a character */
	class Pose
	{
		private:

			bool active;

		public:

			/** For these bone names, a BoneInfluence may be specified */
			map<unsigned int, BoneInfluence> bones;

			Pose() : active(true), bones() { }

			/** Sets how this Pose will affect the specified bone, by modifying the bones map */
			void SetBonePose(unsigned int which, Vec3 ori, Vec3 pos, float weight) { bones[which] = BoneInfluence(ori, pos, weight); }

			/** Abstract function where you can call SetBonePose */
			virtual void UpdatePose(TimingInfo time) = 0;

			/** Query whether this Pose is active or not; if not, it will be removed from the active poses of the SkinnedCharacter */
			bool IsActive() { return active; }
			/** Set this Pose as active or inactive; use this to remove a pose from the list of active poses of a SkinnedCharacter */
			void SetActive(bool yn) { active = yn; }
	};
}
