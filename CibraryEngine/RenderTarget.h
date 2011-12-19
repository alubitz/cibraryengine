#pragma once

#include "StdAfx.h"
#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	class Texture2D;

	class RenderTarget : public Disposable
	{
		private:

			vector<Texture2D*> my_textures;
			vector<GLuint> my_color_buffers;
			GLuint my_depth_buffer;

			GLuint my_fbo;

			GLsizei my_samples;
			GLsizei my_width, my_height;

		protected:

			void InnerDispose();

		public:

			/** Construct a RenderTarget, with 0 for all fields */
			RenderTarget();
			/** Construct a RenderTarget with the given width, height, and number of samples per pixel; internally this just calls Init with the given params */
			RenderTarget(GLsizei width, GLsizei height, GLsizei samples, int n_buffers);

			/** Initializes a RenderTarget with the given width, height, and number of samples per pixel */
			void Init(GLsizei width, GLsizei height, GLsizei samples, int n_buffers);

			GLsizei GetWidth();
			GLsizei GetHeight();
			GLsizei GetSampleCount();

			Texture2D* GetColorBufferTex(int which);




			/** If the parameter is NULL, subsequent draw operations affect the screen as normal; otherwise subsequent draw operations affect the specified RenderTarget */
			static void Bind(RenderTarget* target);

			/** Gets the currently bound RenderTarget */
			static RenderTarget* GetBoundRenderTarget();
	};

}
