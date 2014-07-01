#pragma once

#include "StdAfx.h"

#include "Shootable.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class BrambleNode;
	class Shot;
	class TestGame;

	class Bramble : public Entity, public Shootable
	{
		protected:

			void InnerDispose();

		public:

			BrambleNode* root;
			Sphere bs;

			vector<UberModel*> uber_models;

			RigidBody* rigid_body;
			TriangleMeshShape* collision_shape;

			Bramble(GameState* gs, const Vec3& pos);

			void Vis(SceneRenderer* renderer);

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, const Vec3& poi, const Vec3& vel, float mass);
	};




	class BrambleNode
	{
		private:

			VTNTT TransformVertexInfo(Vec3 pos, Vec3 normal, const Vec3& uv);

		public:

			Vec3 pos;
			Quaternion ori;
			float scale;

			float split_ready;

			BrambleNode* parent;
			vector<BrambleNode*> children;

			Mat4 draw_xform;

			BrambleNode(const Vec3& parent_pos, const Vec3& up, const Quaternion& ori, float scale);
			~BrambleNode();

			bool Grow();

			void AppendVertexData(UberModel::LOD* lod, unsigned int n_verts, float* pos, float* uv, float* normal);
			void AppendCollisionData(vector<float>& vec, unsigned int n_verts, float* pos);

			void GetUberModels(vector<UberModel*>& uber_models, unsigned int n_verts, float* pos, float* uv, float* normal);
	};
}
