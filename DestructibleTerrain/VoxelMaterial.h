#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct VoxelMaterialNodeData
	{
		VertexBuffer* model;
		Vec3 chunk_pos;
		Mat4 xform;

		VoxelMaterialNodeData(VertexBuffer* model, Vec3 chunk_pos, Mat4 xform);

		void Draw(ShaderProgram* shader);
	};

	class VoxelMaterial : public Material
	{
		public:

			ShaderProgram* shader;
			Texture3D* texture;

			VoxelMaterial(ContentMan* content);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* material);
	};
}
