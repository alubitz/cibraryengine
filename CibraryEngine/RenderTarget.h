#pragma once

#include "StdAfx.h"
#include "Disposable.h"

namespace CibraryEngine
{
	class RenderTarget : public Disposable
	{
		private:

			GLuint my_color_buffer;
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
			RenderTarget(GLsizei width, GLsizei height, GLsizei samples);

			/** Initializes a RenderTarget with the given width, height, and number of samples per pixel */
			void Init(GLsizei width, GLsizei height, GLsizei samples);

			GLsizei GetWidth();
			GLsizei GetHeight();
			GLsizei GetSampleCount();



			/** If the parameter is NULL, subsequent draw operations affect the screen as normal; otherwise subsequent draw operations affect the specified RenderTarget */
			static void Bind(RenderTarget* target);
	};

}
