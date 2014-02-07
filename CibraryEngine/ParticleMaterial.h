#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "Material.h"

#include "CameraView.h"

namespace CibraryEngine
{
	class Texture;
	class Texture2D;
	class Texture3D;

	class ParticleMaterial : public Material
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			ParticleMaterial(Texture2D* tex, BlendStyle bs);
			ParticleMaterial(Texture3D* tex, BlendStyle bs);

			Texture* GetTexture() const;
			bool IsTexture3D()    const;

			void SetTexture(Texture2D* tex);
			void SetTexture(Texture3D* tex);

			bool Equals(const Material* other) const;

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(const RenderNode& node);

			void Cleanup(const RenderNode& node);
	};

	struct ParticleMaterialNodeData
	{
		float radius;
		Vec3 pos;
		float angle;
		float red, green, blue, alpha;

		float third_coord;

		ParticleMaterialNodeData(const Vec3& pos, float radius, float angle);

		void PutUVW(float*& uvw_ptr, float u, float v, float w);
		void PutColor(float*& color_ptr);
		void PutVertex(float*& vert_ptr, const Vec3& vert);
		void PutQuad(float*& vert_ptr, float*& uvw_ptr, float*& color_ptr, const Vec3& camera_up, const Vec3& camera_right);
	};
}
