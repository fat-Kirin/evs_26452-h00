/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include "prot_fx.h"
#include "stl.h"

void vlpc_2st_dec(
    Word16 *lsfq,    /* i/o:    i:1st stage   o:1st+2nd stage   */
    Word16 *indx,      /* input:  index[] (4 bits per words)      */
    Word16 mode,       /* input:  0=abs, >0=rel                   */
    Word32 sr_core)
{
    Word16 i;
    Word16 w[M];
    Word16 xq[M];
    Word16  gap;



    /* weighting from the 1st stage */
    lsf_weight_2st(lsfq, w, mode);

    /* quantize */
    AVQ_dec_lpc(indx, xq, 2);

    /* quantized lsf */

    FOR (i=0; i<M; i++)
    {
        lsfq[i] = add(lsfq[i], shl(mult_r(w[i], shl(xq[i],10)),2));                         /*14Q1*1.28*/ 	move16();
    }

    /* reorder */
    sort_fx(lsfq, 0, M-1);
    IF ( EQ_32(sr_core,16000))
    {
        gap = 102;
    }
    ELSE IF ( EQ_32(sr_core,25600))
    {
        gap = 64;
    }
    ELSE IF ( EQ_32(sr_core,32000))
    {
        gap = 51;
    }
    ELSE
    {
        gap = 34;
    }
    reorder_lsf_fx(lsfq, gap, M, INT_FS_FX);



    return;
}
