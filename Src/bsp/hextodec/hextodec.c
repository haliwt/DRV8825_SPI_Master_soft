#include "hextodec/hextodec.h"
#include "usart/bsp_usartx.h"
const uint8_t hex[16]={0,1,2,3,4,5,6,7,8,9,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
const uint8_t dec[16]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};



/*************************************************
*
*16进制�?0进制，最高字�?第四个字节�?
*
*********************************************/
uint32_t  Hex2oct_MSB(uint32_t msb)
{
        uint8_t q,r;
        uint8_t temp7,temp8;
        uint32_t dest;

        temp8=(msb >> 4 & 0x0F);  ////*src =0xabcd7869 8位数
        for(q=0; q<16; q++)
        {
            if(dec[ q ]  == temp8)
            break;
        }

        temp7 = (msb  & 0x0F);
        for(r=0; r<16; r++)
        {
            if(dec[r] == temp7)
            break;
        }


        dest = dec[q]*268435456 +dec[r]*16777216;


        return dest;
}
/*************************************************
*
*16进制�?0进制，中间字�?，第3个字节�?
*
*********************************************/
uint32_t  Hex2oct_MD1(uint32_t md1)
{
        uint8_t o,p;
        uint8_t temp5,temp6;
        uint32_t dest;



        temp6=(md1 >> 4 & 0x0F);  ////*src =0xabcd78 6位数
        for(o=0; o<16; o++)
        {
                if(dec[ o ]  == temp6)
                break;
        }

        temp5 = (md1  & 0x0F);
        for(p=0; p<16; p++)
        {
                if(dec[p] == temp5)
                break;
        }




       dest = dec[o]*1048576+dec[p]*65536 ;


        return dest;
}
/*************************************************
*
*16进制�?0进制，中间字�?，第2个字节�?
*
*********************************************/
uint32_t  Hex2oct_MD2(uint32_t md2)
{
        uint8_t m,n;
        uint8_t temp3,temp4;
        uint32_t dest;

       temp4 = ((md2 >> 4)& 0x0F); //*src =0xabcd  2个字�?
        for(m=0; m<16; m++)
        {
                if(dec[ m ]  == temp4)
                break;
        }

        temp3 = (md2  & 0x0F);
        for(n=0; n<16; n++)
        {
                if(dec[n] == temp3)
                break;
        }



       dest =dec[ m ] * 4096+ dec[n]*256 ;

        return dest;
}
/*************************************************
*
*16进制�?0进制，最低字�?，第1个字节�?
*
*********************************************/
uint32_t  Hex2oct_LSB(uint32_t lsb)
{
        uint8_t i,j;
        uint8_t temp1,temp2;
        uint32_t dest;
        temp2=(lsb >> 4 & 0x0F);   //0xab
        for(i=0; i<16; i++)
        {
                if(dec[ i ]  == temp2)
                break;
        }

        temp1 = (lsb & 0x0F);
        for(j=0; j<16; j++)
        {
                if(dec[j] == temp1)
                break;
        }


       dest =  dec[i]*16 + dec[j];  //oct[i]*16^1


        return dest;
}
/******************************************
   *
   *�������ƣ�
   *�������ܣ�10������ת16������
   *���������Ҫת���ĸ���
   *����ֵ����
   *
*******************************************/
void Dec_To_Hex(float decnum)
{
    char b[]="0123456789ABCDEF";
    long int x;
	uint8_t temp;//uint8_t dec_to_hex[3];
	uint8_t sendbuffer[6]={0xa1,0x02,00,00,00,0x0b};
    int i=3,a[40];
    //printf("/n Please input n=");
    //scanf("%ld\n",&n);
   // printf("decum = %f\n",decnum);
    x=decnum * 10;
    printf("X= %d\n",x);
    i=0;
    while(x>0)
    {
     a[i]=x%16;
     x=x/16;
     i++;
    }
    while(i>0)
    {
      if(i==3)
        {
            sendbuffer[2]=b[a[i-1]]-0x30;
            printf("sendbuffer[2]=%c\n",b[a[i-1]]);
            printf("sendbuffer[2] = %#x \n",b[a[i-1]]);
         }
        else if(i==2)
        {
         sendbuffer[3]=b[a[i-1]]-0x30;
         printf("sendbuffer[3]=%c\n",b[a[i-1]]);
         printf("sendbuffer[3] = %#x \n",b[a[i-1]]);
        }
        else
        {
            
            printf("temp1=%c\n",b[a[i-1]]);
            temp=b[a[i-1]]-0x30;
            if(temp<9)sendbuffer[4]=temp;
            else sendbuffer[4]=temp-1;
           // printf("sendbuffer[4]=%c\n",b[a[i-1]]);
            printf("temp = %#x \n",b[a[i-1]]);
            printf("temp= %d \n",b[a[i-1]]);
        }
       i--;
	  
      }
     
     
     HAL_UART_Transmit(&husartx,sendbuffer,6,12);		

}
/********************************************
   *
   *
   *
   *
********************************************/
uint8_t Dec_Hex_Fun(uint8_t hexdata)
{
    uint8_t num=0;
    uint8_t a[100];
    uint8_t i=0,m=0,yushu;
    char hex[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    num=hexdata;
    while(num>0)
    {
    yushu=num%16;
    a[i++]=yushu;
    num=num/16;

    }
    printf("??????????:0x");
    for(i=i-1;i>=0;i--)//????
    {
     m=a[i];
     printf("%c",hex[m]);
     return hex[m];
    }
    printf("\n");		
}

