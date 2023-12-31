/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "options.h"
#include "cnst_fx.h"
#include "stl.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"

/*-----------------------------------------------------------------*
 * Funtion  core_coder_reconfig                                    *
 *          ~~~~~~~~~~~~~~~~~~~                                    *
 * - reconfig core coder when switching to another frame type      *
 *-----------------------------------------------------------------*/
void core_coder_reconfig(Encoder_State_fx *st)
{
    Word16 i;



    /*Configuration of ACELP*/
    BITS_ALLOC_init_config_acelp(st->total_brate_fx, st->narrowBand, st->nb_subfr, &(st->acelp_cfg));

    /*Configuration of partial copy*/
    st->acelp_cfg_rf.mode_index = 1;
    st->acelp_cfg_rf.midLpc = 0;
    st->acelp_cfg_rf.midLpc_enable = 0;
    st->acelp_cfg_rf.pre_emphasis = 0;
    st->acelp_cfg_rf.formant_enh = 1;
    st->acelp_cfg_rf.formant_tilt = 1;
    st->acelp_cfg_rf.voice_tilt = 1;
    st->acelp_cfg_rf.formant_enh_num = FORMANT_SHARPENING_G1;
    st->acelp_cfg_rf.formant_enh_den = FORMANT_SHARPENING_G2;

    st->nb_bits_header_tcx = 2;  /*Mode(1) + Last_mode(1)*/                                                   move16();
    IF (st->tcxonly == 0)
    {
        move16();
        st->nb_bits_header_ace = 4;
        st->nb_bits_header_tcx = st->nb_bits_header_ace;
        move16();
        if ( st->tcx_cfg.lfacNext<=0 )
        {
            /*st->nb_bits_header_ace--;*/ /*No Last_mode*/
            st->nb_bits_header_ace = sub(st->nb_bits_header_ace,1);
        }
    }

    if ( st->tcxonly)
    {
        st->nb_bits_header_tcx = add(st->nb_bits_header_tcx,2);
    }


    /*Switch off  TCX or ACELP?*/
    IF( EQ_32(st->sr_core,12800))
    {
        st->acelpEnabled = 0;
        move16();

        if( s_and(st->restrictedMode,1) != 0)
        {
            st->acelpEnabled = 1;
            move16();
        }

        st->tcx20Enabled = 0;
        move16();
        if( s_and(st->restrictedMode,2) != 0)
        {

            st->tcx20Enabled = 1;
            move16();
        }
    }
    move32();
    move32();
    move16();
    st->prevEnergyHF_fx = st->currEnergyHF_fx = 1073725440l/*65535.0f Q14*/; /* prevent block switch */
    st->currEnergyHF_e_fx = 17;

    /*Sanity check : don't need to be instrumented*/
    if(st->tcxonly==0)
    {
        assert(st->acelpEnabled ||  st->tcx20Enabled || st->frame_size_index==0);
    }
    else
    {
        assert(st->tcx10Enabled ||  st->tcx20Enabled || st->frame_size_index==0);
    }

    /* TCX-LTP */
    st->tcxltp = getTcxLtp(st->sr_core);

    /*Use for 12.8 kHz sampling rate and low bitrates, the conventional pulse search->better SNR*/

    st->acelp_autocorr = 1;
    move16();

    test();
    if( (LE_32(st->total_brate_fx, 9600))&&(EQ_32(st->sr_core,12800)))
    {
        st->acelp_autocorr = 0;
        move16();
    }

    {
        Word16 bandwidth_mode;

        /*Get bandwidth mode*/
        IF(st->narrowBand)
        {
            bandwidth_mode=0;
        }
        ELSE IF( LE_32(st->sr_core, 16000))
        {
            move16();
            bandwidth_mode=1;
        }
        ELSE
        {
            move16();
            bandwidth_mode=2;
        }

        /*Scale TCX for non-active frames to adjust loudness with ACELP*/
        st->tcx_cfg.na_scale=32767/*1.0f Q15*/;

        test();
        IF( LT_16(bandwidth_mode,2)&&(st->tcxonly==0))
        {
            const Word16 scaleTableSize = sizeof (scaleTcxTable) / sizeof (scaleTcxTable[0]);

            FOR (i = 0 ; i < scaleTableSize ; i++)
            {

                test();
                test();
                IF ( EQ_16(bandwidth_mode, scaleTcxTable[i].bwmode)&&
                     GE_32(st->total_brate_fx, scaleTcxTable[i].bitrateFrom)  &&
                     LT_32(st->total_brate_fx, scaleTcxTable[i].bitrateTo)  )
                {
                    if( st->rf_mode )
                    {
                        i--;
                    }
                    move16();
                    st->tcx_cfg.na_scale = scaleTcxTable[i].scale;
                    BREAK;
                }
            }
        }
    }

    st->enableTcxLpc = 0;
    move16();

    test();
    if ( EQ_16(st->lpcQuantization, 1)&&(LE_32(st->total_brate_fx,LOWRATE_TCXLPC_MAX_BR)||st->rf_mode!=0))
    {
        st->enableTcxLpc = 1;
        move16();
    }

    IF ( st->ini_frame_fx == 0 || EQ_16(st->last_codec_mode, MODE1))
    {
        st->envWeighted = 0;
        move16();
    }

    test();
    test();
    IF( EQ_16(st->bwidth_fx, SWB)&&
        (EQ_32(st->total_brate_fx, ACELP_16k40) || EQ_32(st->total_brate_fx, ACELP_24k40)) )
    {
        IF(st->tec_tfa == 0)
        {
            FOR (i = 0; i < MAX_TEC_SMOOTHING_DEG; i++)
            {
                st->tecEnc.loBuffer[i] = 0;
                move16();
            }
        }
        st->tec_tfa = 1;
        move16();
    }
    ELSE
    {
        st->tec_tfa = 0;
        move16();
    }

    st->enablePlcWaveadjust = 0;
    move16();
    IF (GE_32(st->total_brate_fx, 48000))
    {
        st->enablePlcWaveadjust = 1;
        move16();
    }


    move16();
    st->glr = 0;
    test();
    test();
    test();
    if( (EQ_32(st->total_brate_fx, 9600))||(EQ_32(st->total_brate_fx,16400))||
            (EQ_32(st->total_brate_fx, 24400)))
    {
        move16();
        st->glr = 1;
    }

    if(st->glr)
    {

        st->nb_bits_header_ace = add(st->nb_bits_header_ace, G_LPC_RECOVERY_BITS);
    }

    test();
    IF (EQ_16(st->bwidth_fx, NB)||EQ_16(st->bwidth_fx,WB))
    {
        test();
        IF (st->rf_mode==0)
        {
            st->nmStartLine = startLineWB[s_min((sizeof(startLineWB)/sizeof(startLineWB[0])-1), s_max(3, st->frame_size_index))];
            move16();
        }
        ELSE
        {
            st->nmStartLine = startLineWB[s_min((sizeof(startLineWB)/sizeof(startLineWB[0])-1), s_max(3, st->frame_size_index-1))];
            move16();
        }
    }
    ELSE /* (st->bwidth_fx == SWB || st->bwidth_fx == FB) */
    {
        IF (st->rf_mode==0)
        {
            st->nmStartLine = startLineSWB[s_min((sizeof(startLineSWB)/sizeof(startLineSWB[0])-1), sub(s_max(3, st->frame_size_index), 3))];
            move16();
        }
        ELSE {
            st->nmStartLine = startLineSWB[s_min((sizeof(startLineSWB)/sizeof(startLineSWB[0])-1), sub(s_max(3, st->frame_size_index-1), 3))];
            move16();
        }
    }

    test();
    test();
    test();
    test();
    test();
    IF ( (LT_32(st->total_brate_fx, ACELP_24k40))&&((GT_32(st->total_brate_fx,st->last_total_brate_fx))||(EQ_16(st->last_codec_mode,MODE1))))
    {
        /* low-freq memQuantZeros must be reset partially if bitrate increased */
        FOR (i = 0; i < st->nmStartLine; i++)
        {
            st->memQuantZeros[i] = 0;
            move16();
        }
    }
    ELSE IF ( (GE_32(st->total_brate_fx, ACELP_24k40))&&(LE_32(st->total_brate_fx,ACELP_32k))&&(GE_32(st->last_total_brate_fx,ACELP_13k20))&&(LT_32(st->last_total_brate_fx,ACELP_24k40)))
    {
        FOR (i = 0; i < st->L_frame_fx; i++)    /* memQuantZeros won't be updated */
        {
            st->memQuantZeros[i] = 0;
            move16();
        }
    }

}

