/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"
#include "basop_mpy.h"

/*========================================================================*/
/* FUNCTION : FEC_pitch_estim_fx()										  */
/*------------------------------------------------------------------------*/
/* PURPOSE :  Estimation of pitch for FEC								  */
/*																		  */
/*------------------------------------------------------------------------*/
/* INPUT ARGUMENTS :													  */
/* _ (Word16) st_fx->Opt_AMR_WB: flag indicating AMR-WB IO mode 		  */
/* _ (Word16) st_fx->L_frame_fx:  length of the frame					  */
/* _ (Word16) st_fx->clas_dec: frame classification						  */
/* _ (Word16) st_fx->last_good_fx: last good clas information     		  */
/* _ (Word16[])  pitch	  : pitch values for each subframe   	    Q6	  */
/* _ (Word16[])  old_pitch_buf:pitch values for each subframe   	Q6    */
/*------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :												  */
/*------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :													  */
/*------------------------------------------------------------------------*/

/* _ (Word16[]) st_fx->bfi_pitch	  : initial synthesis filter states   */
/* _ (Word16) st_fx->bfi_pitch_frame: LP filter E of last                 */
/* _ (Word16) st_fx->upd_cnt_fx: update counter                           */
/*------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :													  */
/* _ None																  */
/*========================================================================*/
void FEC_pitch_estim_fx(
    const Word16 Opt_AMR_WB,                 /* i  : flag indicating AMR-WB IO mode             */
    const Word16 last_core,                  /* i  : last core                                  */
    const Word16 L_frame,                    /* i  : length of the frame                        */
    const Word16 clas,                       /* i  : current frame classification               */
    const Word16 last_good,                  /* i  : last good clas information                 */
    const Word16 pitch_buf[],                /* i  : Floating pitch   for each subframe         */
    const Word32 old_pitch_buf[],            /* i  : buffer of old subframe pitch values 15Q16  */
    Word16 *bfi_pitch,                 /* i/o: update of the estimated pitch for FEC      */
    Word16 *bfi_pitch_frame,           /* o  : frame length when pitch was updated        */
    Word16 *upd_cnt                    /* i/o: update counter                             */
    ,const Word16 coder_type                  /* i  : coder_type                                 */
)
{
    Word16 tmp,tmp1,tmp2,tmp3;
    Word16 tmp16k1,tmp16k2;

    tmp = mult_r(pitch_buf[1],22938);  /*Q6( 0.7f * pitch_buf[1] 0.7 in Q15)*/
    tmp1 = shl(tmp,1);                           /*Q6 (1.4f * pitch_buf[1])*/
    tmp2 = round_fx(L_shl(Mpy_32_16_1(old_pitch_buf[2*NB_SUBFR-1],22938), 6)); /*Q6 (0.7f * old_pitch_buf[2*NB_SUBFR-1])*/
    tmp3 = shl(tmp2,1);								  /*Q6 (1.4f * old_pitch_buf[2*NB_SUBFR-1])*/

    tmp16k1 = round_fx(L_shl(Mpy_32_16_1(old_pitch_buf[2*NB_SUBFR16k-1],22938), 6));  /*Q6 0.7f * old_pitch_buf[2*NB_SUBFR16k-1]*/
    tmp16k2 = shl(tmp16k1,1);                               /*Q6 1.4f * old_pitch_buf[2*NB_SUBFR16k-1]*/

    IF( EQ_16(last_core,HQ_CORE))
    {
        *bfi_pitch = pitch_buf[shr(L_frame,6)-1];
        move16();
        *bfi_pitch_frame = L_frame;
        move16();
        *upd_cnt = MAX_UPD_CNT;
        move16();
    }

    test();
    test();
    test();
    IF( (EQ_16(clas,VOICED_CLAS)&&GE_16(last_good,VOICED_TRANSITION))||(Opt_AMR_WB&&EQ_16(clas,VOICED_CLAS)))
    {
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        test();
        IF( ( (LT_16(pitch_buf[3],tmp1))&&(GT_16(pitch_buf[3],tmp))&&
              (LT_16(pitch_buf[1],tmp3) ) && (GT_16(pitch_buf[1],tmp2) ) &&
              (EQ_16(L_frame,L_FRAME) ) ) ||
            ( (LT_16(pitch_buf[3],tmp1) ) && (GT_16(pitch_buf[3],tmp) ) &&
              (LT_16(pitch_buf[1],tmp16k2) ) && (GT_16(pitch_buf[1],tmp16k1) ) &&
              (EQ_16(L_frame,L_FRAME16k) ) )
            || (EQ_16(coder_type, TRANSITION) ) )
        {
            *bfi_pitch = pitch_buf[shr(L_frame,6)-1];
            move16();
            *bfi_pitch_frame = L_frame;
            move16();
            *upd_cnt = 0;
            move16();
        }
    }
}

