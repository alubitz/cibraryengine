#include "StdAfx.h"
#include "Disposable.h"

namespace CibraryEngine
{
	Disposable::Disposable() { disposed = false; }
	Disposable::~Disposable() { Dispose(); }

	void Disposable::Dispose()
	{
		if(!disposed)
		{
			InnerDispose();
			disposed = true;
		}
	}
	void Disposable::InnerDispose() { }

}