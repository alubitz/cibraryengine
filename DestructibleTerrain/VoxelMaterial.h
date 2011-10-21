#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct VoxelMaterialNodeData
	{
		VertexBuffer* model;
		Mat4 xform;

		VoxelMaterialNodeData(VertexBuffer* model, Mat4 xform);

		void Draw();
	};

	class VoxelMaterial : public Material
	{
		public:

			VoxelMaterial();

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* material);
	};
}
