#pragma once

#include "../CibraryEngine/CibraryEngine.h"

#include "Shootable.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class DSNMaterial;
	class BrambleNode;
	class Shot;
	class TestGame;

	class Bramble : public Entity, public Shootable
	{
		protected:

			void InnerDispose();

		public:

			BrambleNode* root;

			vector<BrambleNode*> nodes;
			Sphere bs;
			vector<Material*> materials;
			vector<UberModel*> uber_models;

			Bramble(GameState* gs, Vec3 pos);

			void Vis(SceneRenderer* renderer);

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);
	};




	class BrambleNode
	{
		private:

			VUVNTTC TransformVertexInfo(Vec3 x, Vec3 n, Vec3 uvw);

		public:

			Vec3 pos;
			Quaternion ori;
			float scale;

			float split_ready;
			float v_coord;

			BrambleNode* parent;
			vector<BrambleNode*> children;

			Mat4 draw_xform;

			vector<VTNModel*> node_models;
			VTNModel* model;
			VTNModel* collision_model;

			BrambleNode(Vec3 parent_pos, Vec3 up, Quaternion ori, float scale, float parent_vcoord, vector<VTNModel*> node_models, VTNModel* collision_model);
			~BrambleNode();

			bool Grow();

			void AppendVertexData(UberModel::LOD* lod);
			void AppendCollisionData(VUVNTTCVertexBuffer* collision_verts, VTNModel* collision_shape);

			void GetUberModels(vector<UberModel*>& uber_models);
	};
}
