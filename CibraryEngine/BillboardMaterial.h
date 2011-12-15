#pragma once
#include "StdAfx.h"

#include "Material.h"
#include "Texture2D.h"

#include "Vector.h"

namespace CibraryEngine
{
	class BillboardMaterial : public Material
	{
		private:

			Vec3 camera_position;

			Texture2D* texture;

		public:

			BillboardMaterial(Texture2D* texture, BlendStyle mode);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);
			void Cleanup(RenderNode node);
			bool Equals(Material* other);

			struct NodeData
			{
				Vec3 front, back;
				float width;
				float red, green, blue, alpha;
				float front_u, back_u;

				NodeData(Vec3 position, Vec3 back, float width);

				void Execute(Vec3 camera_position);
			};
	};
}
