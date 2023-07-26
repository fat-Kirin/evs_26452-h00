/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/


#include <stdlib.h>

#include "stl.h"
#include "cnst_fx.h"
#include "stat_enc_fx.h"
#include "prot_fx.h"

#define NBITS_GACELP 5

extern const Word16 lsf_init[16];


void open_PLC_ENC_EVS(
    HANDLE_PLC_ENC_EVS hPlcExt,
    Word32 sampleRate           /* core coder SR */
)
{
    Word16 itr;


    hPlcExt->enableGplc = 0;
    move16();
    hPlcExt->calcOnlylsf = 1;
    move16();
    hPlcExt->nBits = NBITS_GACELP;
    move16();

    hPlcExt->Q_exp = 0;
    move16();
    hPlcExt->Q_new = 0;
    move16();

    set16_fx(hPlcExt->mem_MA_14Q1,0,M);
    set16_fx(hPlcExt->mem_AR,0,M);
    set16_fx(hPlcExt->lsfold_14Q1,0,M);
    set16_fx(hPlcExt->lspold_Q15,0,M);

    set16_fx(hPlcExt->old_exc_Qold,0,8);

    set16_fx(hPlcExt->lsfoldbfi0_14Q1,0,M);
    set16_fx(hPlcExt->lsfoldbfi1_14Q1,0,M);
    set16_fx(hPlcExt->lsf_adaptive_mean_14Q1,0,M);
    hPlcExt->stab_fac_Q15 = 0;
    move16();
    IF( EQ_32(sampleRate,12800))
    {
        hPlcExt->T0_4th = L_SUBFR;
        move16();
        hPlcExt->T0 = L_SUBFR;
        move16();
        FOR( itr=0; itr<M; itr++ )
        {
            hPlcExt->lsf_con[itr] = lsf_init[itr];
            move16();
            hPlcExt->last_lsf_ref[itr] = lsf_init[itr];
            move16();
            hPlcExt->last_lsf_con[itr] = lsf_init[itr];
            move16();
        }
    }
    ELSE
    {
        hPlcExt->T0_4th = L_SUBFR;
        move16();
        hPlcExt->T0 = L_SUBFR;
        move16();
        FOR( itr=0; itr<M; itr++ )
        {
            hPlcExt->lsf_con[itr] = add(lsf_init[itr],shr(lsf_init[itr],2));
            hPlcExt->last_lsf_ref[itr] = add(lsf_init[itr],shr(lsf_init[itr],2));
            hPlcExt->last_lsf_con[itr] = add(lsf_init[itr],shr(lsf_init[itr],2));
        }
    }

    return;
}


/*
  function to extract and write guided information
*/
void gPLC_encInfo (HANDLE_PLC_ENC_EVS self,
                   Word32 modeBitrate,
                   Word16 modeBandwidth,
                   Word16 old_clas,
                   Word16 coder_type
                  )
{


    IF (self)
    {
        self->calcOnlylsf = 1;
        move16();
        test();
        test();
        test();
        test();
        test();
        IF ( ( EQ_16(modeBandwidth, WB)&&EQ_32(modeBitrate,24400))||
             ( EQ_16(modeBandwidth, SWB)  && EQ_32(modeBitrate, 24400)  ) ||
             ( EQ_16(modeBandwidth, FB)  && EQ_32(modeBitrate, 24400)  ) )
        {
            self->enableGplc = 1;
            move16();
            self->nBits = 1;
            move16();
            test();
            test();
            test();
            IF ( (EQ_16(old_clas, VOICED_CLAS)||EQ_16(old_clas,ONSET))&&
                 (EQ_16(coder_type, VOICED) || EQ_16(coder_type, GENERIC) ) )
            {
                self->nBits = NBITS_GACELP;
                move16();
                self->calcOnlylsf = 0;
                move16();
            }
        }
        ELSE
        {
            self->enableGplc = 0;
            move16();
            self->nBits = NBITS_GACELP;
            move16();
        }

    }

}

