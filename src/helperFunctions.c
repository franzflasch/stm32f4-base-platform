#include <helperFunctions.h>


void Delay(volatile uint32_t delayCount)
{
	while (delayCount > 0)
	{
		delayCount--;
	}
}


