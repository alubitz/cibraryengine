#include "StdAfx.h"
#include "RenderTarget.h"

#include "DebugLog.h"

#include "Texture2D.h"

namespace CibraryEngine
{
	/*
	 * RenderTarget methods
	 */
	RenderTarget::RenderTarget() :
		Disposable(),
		my_textures(),
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
		my_textures(),
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

		for(vector<Texture2D*>::iterator iter = my_textures.begin(); iter != my_textures.end(); ++iter)
			delete *iter;
		my_textures.clear();

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
			my_textures.resize(n_buffers);
			glGenRenderbuffers(my_color_buffers.size(), &my_color_buffers[0]);
		}
		glGenRenderbuffers(1, &my_depth_buffer);

		// generate an fbo
		glGenFramebuffers(1, &my_fbo);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, my_fbo);

		// setting up the color buffer
		for(unsigned int i = 0; i < my_color_buffers.size(); ++i)
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
		for(unsigned int i = 0; i < my_color_buffers.size(); ++i)
			glFramebufferRenderbuffer(	GL_DRAW_FRAMEBUFFER,	GL_COLOR_ATTACHMENT0 + i,	GL_RENDERBUFFER,	my_color_buffers[i] );

		glFramebufferRenderbuffer(		GL_DRAW_FRAMEBUFFER,	GL_DEPTH_ATTACHMENT,		GL_RENDERBUFFER,	my_depth_buffer );

		// create and attach textures
		bool were_textures_enabled = glIsEnabled(GL_TEXTURE_2D) == GL_TRUE;
		
		glEnable(GL_TEXTURE_2D);
		for(int i = 0; i < n_buffers; ++i)
		{
			my_textures[i] = new Texture2D(width, height, NULL, false, true);
			unsigned int id = my_textures[i]->GetGLName();

			glBindTexture(GL_TEXTURE_2D, id);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

			glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, id, 0);
		}
		if(!were_textures_enabled)
			glDisable(GL_TEXTURE_2D);

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

	Texture2D* RenderTarget::GetColorBufferTex(int which) { return my_textures[which]; }


	// global variable!!!
	RenderTarget* bound_rt = NULL;

	void RenderTarget::Bind(RenderTarget* target)
	{
		GLDEBUG();

		if(target != NULL)
		{
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target->my_fbo);

			unsigned int n = target->my_color_buffers.size();
			GLenum* draw_buffers = new GLenum[n];
			for(unsigned int i = 0; i < n; ++i)
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