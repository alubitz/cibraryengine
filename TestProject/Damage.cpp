#include "Damage.h"

namespace Test
{
	/*
	 * Damage methods
	 */
	Damage::Damage() : causer(NULL), amount(0) { }
	Damage::Damage(void* causer, float amount) : causer(causer), amount(amount) { }
}
