#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	/** Table of external strings referred to internally by unsigned integers; 0 is not a valid value */
	struct StringTable
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			StringTable();
			~StringTable();

			bool StringExists(string str);
			bool IntExists(unsigned int i);

			/** Creates if not already present */
			unsigned int StringToInt(string str);

			string IntToString(unsigned int i);

			// wrap the above two functions
			unsigned int operator[](string str);
			string operator[](unsigned int i);
	};
}
