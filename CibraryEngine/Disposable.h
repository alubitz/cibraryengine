#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	/** Interface for something which may be disposed */
	class Disposable
	{
		private:
			bool disposed;

		protected:

			/** Virtual function called when the object is disposed, to deallocate resources; this should usually call the parent class' implementation of InnerDispose */
			virtual void InnerDispose();

		public:
			/** Default constructor for a disposable object */
			Disposable();
			/** Destructor... does this need to exist? */
			virtual ~Disposable();

			/** If the object has not yet been disposed, this will call InnerDispose */
			void Dispose();
	};
}