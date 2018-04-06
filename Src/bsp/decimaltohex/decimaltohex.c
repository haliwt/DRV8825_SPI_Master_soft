#include "decimaltohex/decimaltohex.h"
//#include <stdio.h>

uint32_t Decimal_TO_Hex(uint32_t decn)
{
  char u16[10];
  uint8_t w=0,x,y;

  if(decn==0)
	{
	   u16[0]='0';
	   w++;

	}
  else
	{

	   while(decn)
		{
		   y=x%16;
		    if(y<10)
			{
			  u16[w]='0'+y;
			}
		    else
			{
			   u16[w]='A'+y-10;
			}
			w++;
			x=x/16;

		}

	}
	//printf("\n");
	//printf("%d(10)to dec:  ",decn);
	for(w--;w>=0;w--)
    {
	 // printf("%c",u16[w]);
	  return  u16[w];
    }
	

  }