#pragma once

#include "MathTypes.h"

namespace CibraryEngine
{
	/** VertexInfo class with position, texture coordinates, tangent space, and color */
	class VUVNTTC
	{
		public:

			/** Vertex position */
			Vec3 x;
			/** Texture coordinate */
			Vec3 uvw;
			/** Normal vector */
			Vec3 n;
			/** First tangent vector */
			Vec3 tan_1; 
			/** Second tangent vector */
			Vec3 tan_2;
			/** Color */
			Vec4 rgba;

			/** Initialize a default VUVNTTC */
			VUVNTTC() : x(0.0f, 0.0f, 0.0f), rgba(1.0f, 1.0f, 1.0f, 1.0f) { }
			/** Initialize a VUVNTTC soecifying everything except tangent vectors and color */
			VUVNTTC(Vec3 x, Vec3 uvw, Vec3 n) : x(x), uvw(uvw), n(n), rgba(1.0, 1.0, 1.0, 1.0) { }
	};

	/** VertexInfo class with everything VUVNTTC has, plus 4 bone indices and weights */
	class SkinVInfo : public VUVNTTC
	{
		public:

			/** Bone indices */
			unsigned char indices[4];
			/** Bone weights */
			unsigned char weights[4];

			/** Initialize a default SkinVInfo */
			SkinVInfo() : VUVNTTC(), indices(), weights() { weights[0] = 255; }
			/** Initialize a SkinVInfo specifying only position, texture coordinates, and normal vector */
			SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n) : VUVNTTC(x, uvw, n), indices(), weights() { weights[0] = 255; }
			/** Initialize a SkinVInfo copying data from a VUVNTTC */
			SkinVInfo(VUVNTTC original) : VUVNTTC(original), indices(), weights() { weights[0] = 255; }
			/** Initialize a SkinVInfo, specifying everything except color and tangent vectors */
			SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n, unsigned char* indices_, unsigned char* weights_) : VUVNTTC(x, uvw, n)
			{
				for(int i = 0; i < 4; i++)
				{
					indices[i] = indices_[i];
					weights[i] = weights_[i];
				}
			}
	};

	/** Vertex buffer class comprised of VUVNTTC vertex infos */
	class VUVNTTCVertexBuffer : public VertexBuffer<VUVNTTC>
	{
		private:

			unsigned int vbo_id[2];

		protected:

			void Build();
			void GLCleanup();

		public:

			VUVNTTCVertexBuffer();

			void Draw();

			Sphere GetBoundingSphere();

			/** Add the three vertices of a triangle, computing their tangent vectors in the process */
			void AddTriangleVertexInfo(VUVNTTC a, VUVNTTC b, VUVNTTC c);

			void AddVertexInfo(VUVNTTC info);

	};

	/** Vertex buffer class comprised of SkinVInfo vertex infos */
	class SkinVInfoVertexBuffer : public VertexBuffer<SkinVInfo>
	{
		private:

			unsigned int vbo_id[2];

		protected:

			void Build();
			void GLCleanup();

		public:

			SkinVInfoVertexBuffer();

			/** Initialize a SkinVInfoVertexBuffer as a copy of a VUVNTTCVertexBuffer, putting all vertices as solely within the first bone */
			SkinVInfoVertexBuffer(VUVNTTCVertexBuffer& other);

			void Draw();

			Sphere GetBoundingSphere();

			/** Add the three vertices of a triangle, computing their tangent vectors in the process */
			void AddTriangleVertexInfo(SkinVInfo a, SkinVInfo b, SkinVInfo c);

			void AddVertexInfo(SkinVInfo info);
	};

};
