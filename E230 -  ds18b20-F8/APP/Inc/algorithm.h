#ifndef _ALGORITHM_H
#define _ALGORITHM_H


#include "gd32e230.h"

//ÂË²¨
int Get_filter(int PCap_buf);

int GetDelExtremeAndAverage(int Array[], const uint32_t ArraySize, const uint32_t SortHeadSize, const uint32_t SortTailSize);

#endif