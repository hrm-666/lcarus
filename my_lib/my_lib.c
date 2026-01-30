#include "my_lib.h"

void StructCopy(uint8_t *input, uint8_t *output, uint16_t len)
{
	uint16_t i = 0;
	
	for(i = 0; i < len; i++){
		output[i] = input[i]; 
	}
}

