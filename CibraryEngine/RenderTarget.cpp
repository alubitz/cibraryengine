#include "StdAfx.h"
#include "RenderTarget.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * RenderTarget methods
	 */
	RenderTarget::RenderTarget() :
		Disposable(),
		my_color_buffers(),
		my_depth_buffer(0),
		my_fbo(0),
		my_samples(0),
		my_width(0),
		my_height(0)
	{
	}

	RenderTarget::RenderTarget(GLsizei width, GLsizei height, GLsizei samples, int n_buffers) :
		Disposable(),
		my_color_buffers(),
		my_depth_buffer(0),
		my_fbo(0),
		my_samples(0),
		my_width(0),
		my_height(0)
	{
		Init(width, height, samples, n_buffers);
	}

	void RenderTarget::InnerDispose()
	{
		glDeleteFramebuffers(1, &my_fbo);

		if(my_color_buffers.size() > 0)
		{
			glDeleteRenderbuffers(my_color_buffers.size(), &my_color_buffers[0]);
			my_color_buffers.clear();
		}

		glDeleteRenderbuffers(1, &my_depth_buffer);
	}

	void RenderTarget::Init(GLsizei width, GLsizei height, GLsizei samples, int n_buffers)
	{
		GLDEBUG();

		// generate color and depth renderbuffers
		if(n_buffers > 0)
		{
			my_color_buffers.resize(n_buffers);
			glGenRenderbuffers(my_color_buffers.size(), &my_color_buffers[0]);
		}
		glGenRenderbuffers(1, &my_depth_buffer);

		// generate an fbo
		glGenFramebuffers(1, &my_fbo);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, my_fbo);

		// setting up the color buffer
		for(unsigned int i = 0; i < my_color_buffers.size(); i++)
		{
			glBindRenderbuffer(GL_RENDERBUFFER, my_color_buffers[i]);
			glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, width, height);
		}

		// and the depth buffer too
		glBindRenderbuffer(GL_RENDERBUFFER, my_depth_buffer);
		//glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH24_STENCIL8, width, height);
		glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT24, width, height);


		// now that we have set those options, unbind
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		// fill in the fbo's data
		for(unsigned int i = 0; i < my_color_buffers.size(); i++)
			glFramebufferRenderbuffer(	GL_DRAW_FRAMEBUFFER,	GL_COLOR_ATTACHMENT0 + i,	GL_RENDERBUFFER,	my_color_buffers[i] );

		glFramebufferRenderbuffer(		GL_DRAW_FRAMEBUFFER,	GL_DEPTH_ATTACHMENT,		GL_RENDERBUFFER,	my_depth_buffer );

		// unbind stuff
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

		my_width = width;
		my_height = height;
		my_samples = samples;

		GLDEBUG();
	}

	GLsizei RenderTarget::GetWidth() { return my_width; }
	GLsizei RenderTarget::GetHeight() { return my_height; }
	GLsizei RenderTarget::GetSampleCount() { return my_samples; }

	void RenderTarget::GetColorBufferTex(int which, GLuint tex)
	{
		GLDEBUG();

		RenderTarget* prev_bound = GetBoundRenderTarget();
		Bind(NULL);

		GLuint temp_fbo;
		glGenFramebuffers(1, &temp_fbo);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, temp_fbo);
		glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);

		glBindFramebuffer(GL_READ_FRAMEBUFFER, my_fbo);

		glReadBuffer(GL_COLOR_ATTACHMENT0 + which);

		glBlitFramebuffer(0, 0, my_width, my_height, 0, 0, my_width, my_height, GL_COLOR_BUFFER_BIT, GL_NEAREST);

		glDeleteFramebuffers(1, &temp_fbo);

		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

		glReadBuffer(GL_BACK);

		Bind(prev_bound);

		GLDEBUG();
	}

	RenderTarget* bound_rt = NULL;

	void RenderTarget::Bind(RenderTarget* target)
	{
		GLDEBUG();

		if(target != NULL)
		{
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target->my_fbo);

			unsigned int n = target->my_color_buffers.size();
			GLenum* draw_buffers = new GLenum[n];
			for(unsigned int i = 0; i < n; i++)
				draw_buffers[i] = GL_COLOR_ATTACHMENT0 + i;
			glDrawBuffers(n, draw_buffers);
			delete[] draw_buffers;
		}
		else
		{
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

			glDrawBuffer(GL_BACK);
		}

		bound_rt = target;

		GLDEBUG();
	}

	RenderTarget* RenderTarget::GetBoundRenderTarget() { return bound_rt; }
}