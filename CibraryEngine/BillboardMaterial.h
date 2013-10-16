#pragma once
#include "StdAfx.h"

#include "Material.h"
#include "Texture2D.h"

#include "Vector.h"

namespace CibraryEngine
{
	struct VertexBuffer;

	class BillboardMaterial : public Material
	{
		public:

			struct NodeData
			{
				Vec3 front, back;
				float width;
				float red, green, blue, alpha;
				float front_u, back_u;

				NodeData(const Vec3& front, const Vec3& back, float width);

				void PutUV(float*& uv_ptr, float u, float v);
				void PutColor(float*& color_ptr, const Vec4& color);
				void PutVertex(float*& vert_ptr, const Vec3& xyz);
				void PutQuad(float*& vert_ptr, float*& uv_ptr, float*& color_ptr, const Vec3& camera_position);
			};

		private:

			Texture2D* texture;

			// temporary stuff for between calls to BeginDraw and EndDraw
			Vec3 camera_position;
			vector<NodeData*> node_data;
			VertexBuffer* nodes_vbo;

			// NodeData recycling stuff
			vector<NodeData*> recycle_bin;

		protected:

			void InnerDispose();

		public:

			BillboardMaterial(Texture2D* texture, BlendStyle mode);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);
			void Cleanup(RenderNode node);
			bool Equals(Material* other);

			NodeData* NewNodeData(const Vec3& front, const Vec3& rear, float width);
	};
}
