#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	class DATKeyframe;

	class DATConstraint
	{
		public:

			virtual ~DATConstraint() { }

			virtual bool ApplyConstraint(DATKeyframe& pose) { return false; }
	};
}
