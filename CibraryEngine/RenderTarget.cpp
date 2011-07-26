#include "RenderTarget.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * RenderTarget methods
	 */
	RenderTarget::RenderTarget() :
		Disposable(),
		my_color_buffer(0),
		my_depth_buffer(0),
		my_fbo(0),
		my_samples(0),
		my_width(0),
		my_height(0)
	{
	}

	RenderTarget::RenderTarget(GLsizei width, GLsizei height, GLsizei samples) :
		Disposable(),
		my_color_buffer(0),
		my_depth_buffer(0),
		my_fbo(0),
		my_samples(0),
		my_width(0),
		my_height(0)
	{
		Init(width, height, samples);
	}

	void RenderTarget::InnerDispose()
	{
		glDeleteFramebuffers(1, &my_fbo);
		glDeleteRenderbuffers(1, &my_color_buffer);
		glDeleteRenderbuffers(1, &my_depth_buffer);
	}

	void RenderTarget::Init(GLsizei width, GLsizei height, GLsizei samples)
	{
		GLErrorDebug(__LINE__, __FILE__);

		// generate color and depth renderbuffers
		glGenRenderbuffers(1, &my_color_buffer);
		glGenRenderbuffers(1, &my_depth_buffer);

		// generate an fbo
		glGenFramebuffers(1, &my_fbo);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, my_fbo);

		// setting up the color buffer
		glBindRenderbuffer(GL_RENDERBUFFER, my_color_buffer);
		glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, width, height);

		// and the depth buffer too
		glBindRenderbuffer(GL_RENDERBUFFER, my_depth_buffer);
		glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH24_STENCIL8, width, height);


		// now that we have set those options, unbind
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		// fill in the fbo's data
		unsigned int i = 0;						// color attachment index; must be between 0 and GL_MAX_COLOR_ATTACHMENTS - 1
		glFramebufferRenderbuffer(	GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i,	GL_RENDERBUFFER,	my_color_buffer );
		glFramebufferRenderbuffer(	GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,		GL_RENDERBUFFER,	my_depth_buffer );

		// unbind stuff
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

		my_width = width;
		my_height = height;
		my_samples = samples;

		GLErrorDebug(__LINE__, __FILE__);
	}

	GLsizei RenderTarget::GetWidth() { return my_width; }
	GLsizei RenderTarget::GetHeight() { return my_height; }
	GLsizei RenderTarget::GetSampleCount() { return my_samples; }

	void RenderTarget::Bind(RenderTarget* target)
	{
		if(target != NULL)
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target->my_fbo);
		else
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	}
}