#pragma once

#include "../CibraryEngine/CibraryEngine.h"
#include "Heightfield.h"

#include "Shootable.h"

namespace Test
{
	class DSNMaterial;
	class DSNMaterialNodeData;

	class TerrainChunk : public Entity, public Shootable
	{
		private:
			DSNMaterial* material;
			DSNMaterialNodeData* node_data;

		protected:
			void InnerDispose();

		public:

			Heightfield* heightfield;
			SkinVInfoVertexBuffer* model;

			btTriangleMesh* mesh;
			RigidBodyInfo* rigid_body;
			PhysicsWorld* physics;

			TerrainChunk(GameState* gs, Heightfield* heightfield, SkinVInfoVertexBuffer* model, DSNMaterial* material);

			void Vis(SceneRenderer* renderer);
			void VisCleanup();

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);
	};
}
