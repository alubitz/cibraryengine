#pragma once

#include "StdAfx.h"

#include "Content.h"

namespace CibraryEngine
{
	using namespace std;

	class Texture : public Disposable
	{
		protected:
			unsigned int gl_name;
		public:

			Texture() : gl_name(0) { }

			virtual unsigned int GetGLName() = 0;
	};
}
