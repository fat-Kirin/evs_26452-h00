/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/



#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "rom_com_fx.h"
#include "stl.h"
#include "prot_fx.h"
#include "basop_util.h"

static void dec_prm_hm(
    Decoder_State_fx *st,
    Word16 *prm_hm,
    Word16 L_frame
)
{
    Word16 tmp;

    /* Disable HM for non-GC,VC modes */
    test();
    IF (NE_16(st->tcx_cfg.coder_type, VOICED)&&NE_16(st->tcx_cfg.coder_type,GENERIC))
    {
        prm_hm[0] = 0;
        move16();

        return;
    }

    prm_hm[1] = -1;
    move16();
    prm_hm[2] = 0;
    move16();

    /* Flag */
    prm_hm[0] = get_next_indice_fx(st, 1);
    move16();

    IF (prm_hm[0] != 0)
    {
        tmp = 0;
        move16();
        if (GE_16(L_frame, 256))
        {
            tmp = 1;
            move16();
        }
        /* Periodicity index */
        DecodeIndex(st, tmp, &prm_hm[1]);

        /* Gain index */
        IF (EQ_16(st->tcx_cfg.coder_type, VOICED))
        {
            prm_hm[2] = get_next_indice_fx(st, kTcxHmNumGainBits);
            move16();
        }
    }
}

/*-----------------------------------------------------------------*
 * Funtion  dec_prm()                                              *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                                   *
 *
 * SQ is used for TCX modes
 *
 * decode parameters according to selected mode                    *
 *-----------------------------------------------------------------*/
void dec_prm(
    Word16 *coder_type,
    Word16 param[],       /* (o) : decoded parameters               */
    Word16 param_lpc[],   /* (o) : LPC parameters                   */
    Word16 *total_nbbits, /* i/o : number of bits / decoded bits    */
    Decoder_State_fx *st,
    Word16 L_frame,
    Word16 *bitsRead
)
{
    Word16 j, k, n, sfr, *prm;
    Word16 lg;
    Word16 lgFB;
    Word16 start_bit_pos;
    Word16 acelp_target_bits=-1;
    Word16 tmp;
    Word16 nTnsParams;
    Word16 nTnsBits;
    Word16 nb_subfr;
    Word16 nbits_tcx;
    Word16 ix, j_old, wordcnt, bitcnt;
    Word16 hm_size;
    Word16 *prms;
    CONTEXT_HM_CONFIG hm_cfg;
    CONTEXT_HM_CONFIG * phm_cfg;
    Word16 indexBuffer[N_MAX+1];
    Word8 flag_ctx_hm;
    Word32 L_tmp;
    Word16 num_bits;
    UWord16 lsb;
    Word16 ltp_mode, gains_mode;
    Word16 ind;
    Word16 prm_ltp[LTPSIZE];

    /*--------------------------------------------------------------------------------*
     * INIT
     *--------------------------------------------------------------------------------*/

    hm_cfg.indexBuffer = indexBuffer;
    move16();

    IF (EQ_16(st->mdct_sw, MODE1))
    {
        start_bit_pos = 0; /* count from frame start */                                             move16();

        /* Adjust st->bits_frame_core not to subtract MODE2 bandwidth signaling */
        FOR (n=0; n<FRAME_SIZE_NB; n++)
        {
            Mpy_32_16_ss(st->total_brate_fx, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
            num_bits = extract_l(L_shr(L_tmp, 3)); /* Q0 */
            assert(num_bits == st->total_brate_fx/50);
            IF (EQ_16(FrameSizeConfig[n].frame_bits, num_bits))
            {
                st->bits_frame_core = add(st->bits_frame_core, FrameSizeConfig[n].bandwidth_bits);
                BREAK;
            }
        }
    }
    ELSE
    {
        IF( EQ_16(st->rf_flag,1))
        {
            /*Inherent adjustment to accommodate the compact packing used in the RF mode*/
            start_bit_pos = sub(st->next_bit_pos_fx,2);
        }
        ELSE
        {
            start_bit_pos = st->next_bit_pos_fx;
        }
    }

    /* Framing parameters */
    nb_subfr = st->nb_subfr;
    move16();

    /* Initialize pointers */
    prm = param;

    /* Init counters */
    j = 0;
    move16();

    /* Init LTP data */
    st->tcx_hm_LtpPitchLag = -1;
    move16();
    st->tcxltp_gain = 0;
    move16();

    /*--------------------------------------------------------------------------------*
     * HEADER
     *--------------------------------------------------------------------------------*/

    /* Modes (ACE_GC, ACE_UC, TCX20, TCX10...) */
    IF ( st->tcxonly )
    {
        st->core_fx = add(get_next_indice_fx(st, 1), 1);
        move16();
            ind = get_next_indice_fx(st, 2);
            st->clas_dec = ONSET;
            move16();
            IF( ind == 0 )
            {
                st->clas_dec = UNVOICED_CLAS;
                move16();
            }
            ELSE IF( EQ_16(ind, 1))
            {
                st->clas_dec = UNVOICED_TRANSITION;
                move16();
                if( GE_16(st->last_good_fx, VOICED_TRANSITION))
                {
                    st->clas_dec = VOICED_TRANSITION;
                    move16();
                }
            }
            ELSE if( EQ_16(ind, 2))
            {
                st->clas_dec = VOICED_CLAS;
                move16();
            }
        *coder_type = INACTIVE;
        st->VAD = 0;
        move16();
    }
    ELSE
    {
        IF (EQ_16(st->mdct_sw, MODE1))
        {
            /* 2 bits instead of 3 as TCX is already signaled */
            st->core_fx = TCX_20_CORE;
            move16();
            st->tcx_cfg.coder_type = get_next_indice_fx(st, 2);
            move16();
            *coder_type = st->tcx_cfg.coder_type;
            move16();
        }
        ELSE
        {
            IF (EQ_16(st->mdct_sw_enable, MODE2))
            {
                IF (get_next_indice_1_fx(st) != 0) /* TCX */
                {
                    tmp = get_next_indice_fx(st, 3);
                    assert(!(tmp & 4) || !"HQ_CORE encountered in dec_prm");
                    st->core_fx = TCX_20_CORE;
                    move16();
                    st->tcx_cfg.coder_type = tmp;
                    move16();
                    *coder_type = st->tcx_cfg.coder_type;
                    move16();
                }
                ELSE /* ACELP */
                {
                    st->core_fx = ACELP_CORE;
                    move16();
                    *coder_type = get_next_indice_fx(st, 2);
                    move16();
                }
            }
            ELSE
            {
                IF(EQ_16(st->rf_flag,1))
                {
                    IF( !( st->use_partial_copy ) )
                    {
                        tmp = get_next_indice_fx(st, 1);
                        IF(tmp == 0)
                        {
                            st->core_fx = ACELP_CORE;
                            move16();
                        }
                        ELSE
                        {
                            st->core_fx = TCX_20_CORE;
                            move16();
                            st->tcx_cfg.coder_type = *coder_type;
                            move16();
                        }
                    }
                }
                ELSE
                {
                    st->core_fx = ACELP_CORE;
                    move16();
                    *coder_type = get_next_indice_fx(st, 3);
                    move16();
                    IF ( GE_16(*coder_type, ACELP_MODE_MAX))
                    {
                        st->core_fx = TCX_20_CORE;
                        move16();
                        *coder_type=sub(*coder_type,ACELP_MODE_MAX);
                        st->tcx_cfg.coder_type = *coder_type;
                        move16();
                    }
                }
            }
        }

        test();
        IF ( st->igf != 0 && EQ_16(st->core_fx, ACELP_CORE) )
        {
            st->bits_frame_core = sub( st->bits_frame_core, get_tbe_bits_fx(st->total_brate_fx, st->bwidth_fx, st->rf_flag) );
        }

        IF( EQ_16(st->rf_flag,1))
        {
            st->bits_frame_core = sub(st->bits_frame_core, add(st->rf_target_bits, 1)); /* +1 as flag-bit not considered in rf_target_bits */
        }

        /* Inactive frame detection on non-DTX mode */
        st->VAD = 1;
        move16();
        if( EQ_16(*coder_type, INACTIVE))
        {
            st->VAD = 0;
            move16();
        }
    }

    /*Core extended mode mapping for correct PLC classification*/
    st->core_ext_mode = *coder_type;
    move16();

    if( EQ_16(*coder_type, INACTIVE))
    {
        st->core_ext_mode = UNVOICED;
        move16();
    }

    /* Decode previous mode for error concealment */
    tmp = 0;
    move16();
    test();
    test();
    IF( ( NE_16(st->core_fx, ACELP_CORE) || st->tcx_cfg.lfacNext > 0 ) && st->use_partial_copy == 0 )
    {
        st->last_core_bs_fx = get_next_indice_fx(st, 1);
        move16();

        /* 
           need to introduce special error handling for lost transition frames from CNG:
           in such cases, the bitstream reader continues with CNG, setting bfi = 0, total_brate = 0
           this might result in a not matching last_core transmitted in the BS - we should use this 
           only for interpreting the bitstream and re-use the internal state for the proper 
           transition handling; still, for voiced onsets rather stick to wrong windowing...
        */
        if( (!((st->last_total_brate_fx == 0) && (sub(st->clas_dec, VOICED_CLAS) != 0))) && (sub(st->last_core_bs_fx, st->last_core_fx) != 0) )
        {
            st->last_core_fx = st->last_core_bs_fx;
        } 
        /*for TCX 10 force last_core to be TCX since ACELP as previous core is forbidden*/
        IF( EQ_16(st->core_fx, TCX_10_CORE))
        {
            st->last_core_fx = TCX_20_CORE;
            st->last_core_bs_fx = TCX_20_CORE;            
            move16();
            move16();
        }
    }

    test();
    test();
    IF(EQ_16(st->rf_flag,1)&&EQ_16(st->use_partial_copy,1)&&!st->tcxonly)
    {
        st->bits_frame_core = st->rf_target_bits;

        /* offset the indices to read the acelp partial copy */
        Mpy_32_16_ss(st->total_brate_fx, 5243, &L_tmp, &lsb); /* 5243 is 1/50 in Q18. (0+18-15=3) */
        num_bits = extract_l(L_shr(L_tmp, 3)); /* Q0 */

        get_next_indice_tmp_fx(st, start_bit_pos + num_bits - st->rf_target_bits - 3 - st->next_bit_pos_fx);
    }

    IF( st->use_partial_copy == 0 )
    {
        /* Set the last overlap mode based on the previous and current frame type and coded overlap mode */
        test();
        IF ((EQ_16(st->last_core_fx, ACELP_CORE))||(EQ_16(st->last_core_fx,AMR_WB_CORE)))
        {
            st->tcx_cfg.tcx_last_overlap_mode = TRANSITION_OVERLAP;
            move16();
        }
        ELSE
        {
            test();
            IF ((EQ_16(st->core_fx, TCX_10_CORE))&&(EQ_16(st->tcx_cfg.tcx_curr_overlap_mode,ALDO_WINDOW)))
            {
                st->tcx_cfg.tcx_last_overlap_mode = FULL_OVERLAP;
                move16();
            }
            ELSE
            {
                st->tcx_cfg.tcx_last_overlap_mode = st->tcx_cfg.tcx_curr_overlap_mode;
                move16();
                test();
                if ((NE_16(st->core_fx, TCX_10_CORE))&&(EQ_16(st->tcx_cfg.tcx_curr_overlap_mode,FULL_OVERLAP)))
                {
                    st->tcx_cfg.tcx_last_overlap_mode = ALDO_WINDOW;
                    move16();
                }
            }
        }
        /* Set the current overlap mode based on the current frame type and coded overlap mode */
        st->tcx_cfg.tcx_curr_overlap_mode = ALDO_WINDOW;
        move16();

        IF (NE_16(st->core_fx, ACELP_CORE))
        {
            tmp = 0;
            move16();
            /* if current TCX mode is not 0 (full overlap), read another bit */
            IF (get_next_indice_fx(st, 1))
            {
                tmp = add(2, get_next_indice_fx(st, 1));
            }
            st->tcx_cfg.tcx_curr_overlap_mode = tmp;
            move16();

            /* TCX10 : always symmetric windows */
            test();
            test();
            test();
            if ((EQ_16(st->core_fx, TCX_20_CORE))&&(tmp==0)&&(NE_16(st->last_core_fx,ACELP_CORE))&&(NE_16(st->last_core_fx,AMR_WB_CORE)))
            {
                st->tcx_cfg.tcx_curr_overlap_mode = ALDO_WINDOW;
                move16();
            }
        }

        /* SIDE INFO. DECODING */
        IF(st->enableGplc)
        {
            Word16 pitchDiff;
            Word16 bits_per_subfr, search_range;
            bits_per_subfr = 4;
            move16();
            search_range = 8;
            move16();

            st->flagGuidedAcelp = get_next_indice_fx(st, 1);
            move16();

            pitchDiff = 0;
            move16();
            IF(st->flagGuidedAcelp)
            {
                pitchDiff = get_next_indice_fx(st, bits_per_subfr);
                move16();
                st->guidedT0 = sub(pitchDiff, search_range);
                move16();
            }
            test();
            if( (pitchDiff == 0) && st->flagGuidedAcelp)
            {
                st->flagGuidedAcelp = 0;
                move16();
            }
        }
        ELSE
        {
            st->flagGuidedAcelp = 0;
            move16();
        }

        IF( st->dec_glr )
        {
            move16();
            st->dec_glr_idx = -1;
            IF( EQ_16(st->core_fx, ACELP_CORE) )
            {
                st->dec_glr_idx = get_next_indice_fx(st, G_LPC_RECOVERY_BITS);
            }
        }
    }
    /*--------------------------------------------------------------------------------*
     * LPC PARAMETERS
     *--------------------------------------------------------------------------------*/

    /*Initialization of LPC Mid flag*/
    st->acelp_cfg.midLpc = st->acelp_cfg.midLpc_enable;
    move16();
    test();
    test();
    IF( (EQ_16(st->lpcQuantization, 1)&&(EQ_16(*coder_type,VOICED)))||(st->use_partial_copy))
    {
        st->acelp_cfg.midLpc = 0;
        move16();
    }

    IF( st->use_partial_copy == 0 )
    {
        /* Number of sets of LPC parameters (does not include mid-lpc) */
        st->numlpc = 2;
        move16();
        test();
        if ( st->tcxonly==0 || LT_16(st->core_fx, TCX_10_CORE))
        {
            st->numlpc = 1;
            move16();
        }

        /* Decode LPC parameters */
        test();
        IF (st->enableTcxLpc && NE_16(st->core_fx, ACELP_CORE))
        {
            Word16 tcx_lpc_cdk;
            tcx_lpc_cdk = tcxlpc_get_cdk(*coder_type);
            dec_lsf_tcxlpc(st, &param_lpc, st->narrowBand, tcx_lpc_cdk);
        }
        ELSE
        {
            IF (st->lpcQuantization==0)
            {
                decode_lpc_avq( st, st->numlpc, param_lpc );
                move16();
            }
            ELSE IF (EQ_16(st->lpcQuantization, 1))
            {
                test();
                test();
                IF(EQ_32(st->sr_core, 16000)&&EQ_16(*coder_type,VOICED)&&EQ_16(st->core_fx,ACELP_CORE))
                {
                    lsf_bctcvq_decprm(st, param_lpc);
                }
                ELSE
                {
                    lsf_msvq_ma_decprm( st, param_lpc, st->core_fx, *coder_type, st->acelp_cfg.midLpc, st->narrowBand, st->sr_core );
                }

            }
            ELSE
            {
                assert(0 && "LPC quant not supported!");
            }
        }
    }
    ELSE
    {
        st->numlpc = 1;
        move16();

        test();
        IF( EQ_16(st->rf_frame_type, RF_TCXFD))
        {
            param_lpc[0] = 0;
            move16();
            param_lpc[1] = get_next_indice_fx(st, lsf_numbits[0]);  /* VQ 1 */
            param_lpc[2] = get_next_indice_fx(st, lsf_numbits[1]);  /* VQ 2 */
            param_lpc[3] = get_next_indice_fx(st, lsf_numbits[2]);  /* VQ 3 */
        }
        ELSE IF( GE_16(st->rf_frame_type, RF_ALLPRED)&&LE_16(st->rf_frame_type,RF_NELP))
        {
            /* LSF indices */
            param_lpc[0] = get_next_indice_fx(st, 8);  /* VQ 1 */
            param_lpc[1] = get_next_indice_fx(st, 8);  /* VQ 2 */
        }
    }

    st->bits_common = sub(st->next_bit_pos_fx, start_bit_pos);
    move16();


    /*--------------------------------------------------------------------------------*
     * ACELP
     *--------------------------------------------------------------------------------*/
    test();
    test();
    IF( EQ_16(st->core_fx,ACELP_CORE)&&st->use_partial_copy==0)
    {
        /* Target Bits */
        acelp_target_bits = sub(st->bits_frame_core, st->bits_common);

        {
            Word16 acelp_bits;
            move16();
            acelp_bits = BITS_ALLOC_config_acelp( acelp_target_bits, *coder_type, &(st->acelp_cfg), st->narrowBand, st->nb_subfr );

            if ( acelp_bits < 0 )
            {
                st->BER_detect = 1;
                move16();
            }
        }

        /* Adaptive BPF (2 bits)*/
        n=ACELP_BPF_BITS[st->acelp_cfg.bpf_mode];
        move16();

        st->bpf_gain_param = shl(st->acelp_cfg.bpf_mode, 1);
        IF( n != 0)
        {
            st->bpf_gain_param = get_next_indice_fx(st, n);
            move16();
        }

        /* Mean energy (2 or 3 bits) */
        n = ACELP_NRG_BITS[st->acelp_cfg.nrg_mode];
        move16();
        IF( n != 0 )
        {
            prm[j++] = get_next_indice_fx(st, n);
            move16();
        }

        /* Subframe parameters */
        FOR (sfr=0; sfr<nb_subfr; sfr++)
        {
            /* Pitch lag (4, 5, 6, 8 or 9 bits) */
            n = ACELP_LTP_BITS_SFR[st->acelp_cfg.ltp_mode][sfr];
            move16();

            IF(n!=0)
            {
                prm[j++] = get_next_indice_fx(st, n);
                move16();
            }

            /* Adaptive codebook filtering (1 bit) */
            IF ( EQ_16(st->acelp_cfg.ltf_mode, 2))
            {
                prm[j++] = get_next_indice_fx(st, 1);
                move16();
            }

            /* Innovative codebook */
            {
                /* Decode pulse positions. */
                j_old = j;
                move16();
                wordcnt = shr(ACELP_FIXED_CDK_BITS(st->acelp_cfg.fixed_cdk_index[sfr]), 4);
                bitcnt = s_and(ACELP_FIXED_CDK_BITS(st->acelp_cfg.fixed_cdk_index[sfr]), 0xF);

                /* sanity check for testing - not instrumented */
                test();
                if ( GE_16(st->acelp_cfg.fixed_cdk_index[sfr], ACELP_FIXED_CDK_NB)||(st->acelp_cfg.fixed_cdk_index[sfr]<0))
                {
                    st->acelp_cfg.fixed_cdk_index[sfr] = 0;
                    move16();
                    st->BER_detect = 1;
                    move16();
                }

                FOR (ix = 0; ix < wordcnt; ix++)
                {
                    prm[j++] = get_next_indice_fx(st, 16);
                    move16();
                }
                IF (bitcnt)
                {
                    prm[j] = get_next_indice_fx(st, bitcnt);
                    move16();
                }

                j = add(j_old, 8);

            }

            /* Gains (5b, 6b or 7b / subfr) */
            n = ACELP_GAINS_BITS[st->acelp_cfg.gains_mode[sfr]];
            move16();

            prm[j++] = get_next_indice_fx(st, n);
            move16();
        }/*end of subfr loop*/
    }

    ELSE IF( GE_16(st->rf_frame_type,RF_ALLPRED)&&st->use_partial_copy)
    {
        Word16 acelp_bits = 
        BITS_ALLOC_config_acelp( st->rf_target_bits,         /* target bits ranges from 56 to 72 depending on rf_type */
                                 st->rf_frame_type,          /* already offset by 4 to parse the config elements for partial copy */
                                 &(st->acelp_cfg),           /* acelp_cfg_rf*/
                                 0,                          /* is narrowBand */
                                 st->nb_subfr );

        if ( acelp_bits < 0 )
        {
            st->BER_detect = 1;
            move16();
        }

        /* rf_frame_type NELP: 7 */
        IF(EQ_16(st->rf_frame_type,RF_NELP))
        {
            /* NELP gain indices */
            st->rf_indx_nelp_iG1 = get_next_indice_fx( st, 5 );
            st->rf_indx_nelp_iG2[0] = get_next_indice_fx( st, 6 );
            st->rf_indx_nelp_iG2[1] = get_next_indice_fx( st, 6 );

            /* NELP filter selection index */
            st->rf_indx_nelp_fid = get_next_indice_fx( st, 2 );

            /* tbe gainFr */
            st->rf_indx_tbeGainFr = get_next_indice_fx( st, 5 );
        }
        ELSE
        {
            /* rf_frame_type ALL_PRED: 4, NO_PRED: 5, GEN_PRED: 6*/
            /* ES pred */
            prm[j++] = get_next_indice_fx(st, 3);

            ltp_mode = ACELP_LTP_MODE[1][1][st->rf_frame_type];
            gains_mode = ACELP_GAINS_MODE[1][1][st->rf_frame_type];

            /* Subframe parameters */
            FOR( sfr = 0; sfr < nb_subfr; sfr++ )
            {
                /* Pitch lag (5, or 8 bits) */
                n = ACELP_LTP_BITS_SFR[ltp_mode][sfr];
                IF (n != 0)
                {
                    prm[j++] = get_next_indice_fx(st, n);
                }

                /*Innovative codebook*/
                test();
                test();
                test();
                IF( EQ_16(st->rf_frame_type,RF_NOPRED)
                    || ( EQ_16(st->rf_frame_type,RF_GENPRED)  && (sfr == 0 || EQ_16(sfr,2) )) )
                {
                    /* NOTE: FCB actual bits need to be backed up as well */
                    /*n = ACELP_FIXED_CDK_BITS(st->rf_indx_fcb[fec_offset][sfr]) & 15;*/
                    prm[j] = get_next_indice_fx(st, 7);
                    j = add(j,8);
                }

                /* Gains (5b, 6b or 7b / subfr) */
                test();
                IF( sfr == 0 || EQ_16(sfr,2))
                {
                    n = ACELP_GAINS_BITS[gains_mode];
                    prm[j++] = get_next_indice_fx(st, n);
                }
            }

            st->rf_indx_tbeGainFr = get_next_indice_fx( st, 2 );
        }
    }

    /*--------------------------------------------------------------------------------*
     * TCX20
     *--------------------------------------------------------------------------------*/
    test();
    IF( EQ_16(st->core_fx, TCX_20_CORE)&&st->use_partial_copy==0)
    {
        flag_ctx_hm = 0;
        move16();

        IF (st->enablePlcWaveadjust)
        {
            st->tonality_flag = get_next_indice_fx(st, 1);
            move16();
        }

        /* TCX Gain = 7 bits */
        prm[j++] = get_next_indice_fx(st, 7);
        move16();

        /* TCX Noise Filling = NBITS_NOISE_FILL_LEVEL bits */
        prm[j++] = get_next_indice_fx(st, NBITS_NOISE_FILL_LEVEL);
        move16();

        /* LTP data */
        /* PLC pitch info for HB */
        test();
        IF (st->tcxltp != 0 || GT_32(st->sr_core, 25600))
        {

            prm[j] = get_next_indice_fx(st, 1);
            move16();

            IF ( prm[j] )
            {
                prm[j+1] = get_next_indice_fx(st, 9);
                move16();
                prm[j+2] = get_next_indice_fx(st, 2);
                move16();
            }

            st->BER_detect = st->BER_detect |
                             tcx_ltp_decode_params(&prm[j],
                                                   &(st->tcxltp_pitch_int),
                                                   &(st->tcxltp_pitch_fr),
                                                   &(st->tcxltp_gain),
                                                   st->pit_min,
                                                   st->pit_fr1,
                                                   st->pit_fr2,
                                                   st->pit_max,
                                                   st->pit_res_max);

            st->tcx_hm_LtpPitchLag = -1;
            move16();
            st->tcxltp_last_gain_unmodified = st->tcxltp_gain;
            move16();

            test();
            IF ((st->tcxonly == 0) && (LT_16(st->tcxltp_pitch_int, L_frame)))
            {
                Word32 tmp32 = L_shl(L_mult0(st->L_frame_fx, st->pit_res_max), 1+kLtpHmFractionalResolution+1);
                Word16 tmp1 = add(imult1616(st->tcxltp_pitch_int, st->pit_res_max), st->tcxltp_pitch_fr);
                st->tcx_hm_LtpPitchLag = div_l(tmp32, tmp1);
            }
        }

        j = add(j, 3);

        /* TCX spectral data */
        lg = L_frame;
        move16();
        lgFB = st->tcx_cfg.tcx_coded_lines;
        move16();

        IF (st->last_core_bs_fx == ACELP_CORE )
        {
            /* ACE->TCX transition */
            lg = add(lg, st->tcx_cfg.tcx_offset);
            if(st->tcx_cfg.lfacNext < 0)
            {
                lg = sub(lg,st->tcx_cfg.lfacNext);
            }

            lgFB = add(lgFB, shr(lgFB, 2));
        }

        /* TNS data */
        nTnsParams = 0;
        move16();
        nTnsBits = 0;
        move16();

        IF (st->tcx_cfg.fIsTNSAllowed)
        {
            SetTnsConfig(&st->tcx_cfg, 1, st->last_core_bs_fx == ACELP_CORE);
            ReadTnsData(st->tcx_cfg.pCurrentTnsConfig, st, &nTnsBits, prm+j, &nTnsParams);

            j = add(j, nTnsParams);
        }
        hm_size = shl(mult(st->TcxBandwidth, lg), 1);

        test();
        IF (st->tcx_lpc_shaped_ari != 0 && NE_16(st->last_core_bs_fx, ACELP_CORE))
        {
            dec_prm_hm(st, &prm[j], hm_size);
        }

        nbits_tcx = sub(st->bits_frame_core, sub(st->next_bit_pos_fx, start_bit_pos));
        if (st->enableGplc != 0)
        {
            nbits_tcx = sub(nbits_tcx, 7);
        }

        /*Context HM flag*/
        test();
        IF ( st->tcx_cfg.ctx_hm && NE_16(st->last_core_bs_fx, ACELP_CORE))
        {
            prm[j] = get_next_indice_fx(st, 1);
            move16();
            nbits_tcx = sub(nbits_tcx, 1);

            IF (prm[j])
            {
                Word16 NumIndexBits;

                tmp = 0;
                move16();
                if(GE_16(hm_size, 256))
                {
                    tmp = 1;
                    move16();
                }

                NumIndexBits = DecodeIndex(st,
                                           tmp,
                                           prm+j+1);

                flag_ctx_hm = 1;
                move16();

                ConfigureContextHm(
                    lgFB,
                    nbits_tcx,
                    *(prm+j+1),
                    st->tcx_hm_LtpPitchLag,
                    &hm_cfg);

                nbits_tcx = sub(nbits_tcx, NumIndexBits);
            }
        }
        j = add(j, NPRM_CTX_HM);

        /* read IGF payload */
        IF (st->igf)
        {

            n = st->next_bit_pos_fx;
            move16();
            IF (EQ_16(st->last_core_bs_fx, ACELP_CORE))
            {
                IGFDecReadLevel( &st->hIGFDec, st, IGF_GRID_LB_TRAN, 1 );
                IGFDecReadData( &st->hIGFDec, st, IGF_GRID_LB_TRAN, 1 );

            }
            ELSE
            {
                IGFDecReadLevel( &st->hIGFDec, st, IGF_GRID_LB_NORM, 1 );
                IGFDecReadData( &st->hIGFDec, st, IGF_GRID_LB_NORM, 1 );
            }

            nbits_tcx = sub(nbits_tcx, sub(st->next_bit_pos_fx, n));

        }
        nbits_tcx = sub(st->bits_frame_core, sub(st->next_bit_pos_fx, start_bit_pos));
        IF (st->tcx_lpc_shaped_ari != 0)
        {
            prm[j++] = nbits_tcx;  /* store length of buffer */     move16();
            prms = &prm[j];
            FOR (ix = 0; ix < nbits_tcx; ix++)
            {
                prms[ix] = get_next_indice_1_fx(st);
                move16();
            }
            set16_fx(prms+nbits_tcx, 1, 32);
            j = add(j, nbits_tcx);
        }
        ELSE
        {
            phm_cfg = NULL;
            move16();
            if (flag_ctx_hm)
            {
                phm_cfg = &hm_cfg;
                move16();
            }
            st->resQBits[0] = ACcontextMapping_decode2_no_mem_s17_LC(st,
            prm+j,
            lgFB,
            nbits_tcx,
            NPRM_RESQ*st->tcx_cfg.resq,
            phm_cfg);
            move16();
            j = add(j, lg);
        }
    }

    test();
    test();
    IF( GE_16(st->rf_frame_type,RF_TCXFD)&&LE_16(st->rf_frame_type,RF_TCXTD2)&&EQ_16(st->use_partial_copy,1))
    {
        /* classification */
        ind = get_next_indice_fx(st, 2);
        st->clas_dec = ONSET;
        move16();

        IF( ind == 0 )
        {
            st->clas_dec = UNVOICED_CLAS;
            move16();
        }
        ELSE IF( EQ_16(ind, 1))
        {
            IF( GE_16(st->last_good_fx, VOICED_TRANSITION))
            {
                st->clas_dec = VOICED_TRANSITION;
                move16();
            }
            ELSE
            {
                st->clas_dec = UNVOICED_TRANSITION;
                move16();
            }
        }
        ELSE IF( EQ_16(ind, 2))
        {
            st->clas_dec = VOICED_CLAS;
            move16();
        }

        IF( EQ_16(st->rf_frame_type, RF_TCXFD))
        {
            /* TCX Gain = 7 bits */
            st->old_gaintcx_bfi = get_next_indice_fx(st, 7);
        }
        ELSE
        {
            /* LTP data */
            IF( st->tcxltp != 0 )
            {
                IF( EQ_16(st->rf_frame_type, RF_TCXTD2)||EQ_16(st->rf_frame_type,RF_TCXTD1))
                {
                    prm_ltp[0] = 1;
                    move16(); /* LTP active*/
                    prm_ltp[1] = get_next_indice_fx(st, 9);
                    prm_ltp[2] = 3;
                    move16(); /* max ampl. quantizer output (2bits), anyway not used later*/

                    IF( st->prev_bfi_fx == 0 )
                    {
                        st->BER_detect = st->BER_detect |
                        tcx_ltp_decode_params(&prm_ltp[0], &(st->tcxltp_pitch_int), &(st->tcxltp_pitch_fr), &(st->tcxltp_gain),
                        st->pit_min, st->pit_fr1, st->pit_fr2, st->pit_max, st->pit_res_max );

                        st->tcxltp_last_gain_unmodified = st->tcxltp_gain;
                        move16();
                    }
                }
            }
        }
    }


    /*--------------------------------------------------------------------------------*
     * TCX10
     *--------------------------------------------------------------------------------*/
    IF ( EQ_16(st->core_fx, TCX_10_CORE))
    {
        Word16 tcxltp_prm_0 = 0;
        Word16 tcxltp_prm_1 = 0;
        Word16 tcxltp_prm_2 = 0;
        Word16 nbits_igf    = 0;
        move16();
        move16();
        move16();
        move16();
        move16();
        /* read IGF payload */
        IF (st->igf)
        {

            n = st->next_bit_pos_fx;
            move16();

            IGFDecReadLevel( &st->hIGFDec, st, IGF_GRID_LB_SHORT, 1);
            IGFDecReadData( &st->hIGFDec, st, IGF_GRID_LB_SHORT, 1);
            IGFDecStoreTCX10SubFrameData( &st->hIGFDec, 0 );

            IGFDecReadLevel( &st->hIGFDec, st, IGF_GRID_LB_SHORT, 0);
            IGFDecReadData( &st->hIGFDec, st, IGF_GRID_LB_SHORT, 0);
            IGFDecStoreTCX10SubFrameData( &st->hIGFDec, 1 );

            nbits_igf  = sub(st->next_bit_pos_fx, n);

        }
        FOR (k = 0; k < 2; k++)
        {
            flag_ctx_hm = 0;
            move16();

            prm = param + (k*DEC_NPRM_DIV);
            j = 0;
            move16();

            nbits_tcx = sub(st->next_bit_pos_fx, start_bit_pos);

            test();
            IF (st->enablePlcWaveadjust && k)
            {
                st->tonality_flag = get_next_indice_fx(st, 1);
                move16();
            }
            /* TCX Gain = 7 bits */
            prm[j++] = get_next_indice_fx(st, 7);
            move16();

            /* TCX Noise Filling = NBITS_NOISE_FILL_LEVEL bits */
            prm[j++] = get_next_indice_fx(st, NBITS_NOISE_FILL_LEVEL);
            move16();

            /* LTP data */
            test();
            test();
            IF ( (k == 0) && ((st->tcxltp != 0) || (GT_32(st->sr_core, 25600))))
            {
                prm[j] = get_next_indice_fx(st, 1);
                move16();

                IF ( prm[j] )
                {

                    prm[j+1] = get_next_indice_fx(st, 9);
                    move16();
                    prm[j+2] = get_next_indice_fx(st, 2);
                    move16();

                    tcxltp_prm_0 = prm[j];
                    move16();
                    tcxltp_prm_1 = prm[j+1];
                    move16();
                    tcxltp_prm_2 = prm[j+2];
                    move16();
                }

                st->BER_detect = st->BER_detect |
                                 tcx_ltp_decode_params(&prm[j],
                                                       &(st->tcxltp_pitch_int),
                                                       &(st->tcxltp_pitch_fr),
                                                       &(st->tcxltp_gain),
                                                       st->pit_min,
                                                       st->pit_fr1,
                                                       st->pit_fr2,
                                                       st->pit_max,
                                                       st->pit_res_max);

                st->tcxltp_last_gain_unmodified = st->tcxltp_gain;
                move16();
                st->tcx_hm_LtpPitchLag = -1;
                move16();

                j = add(j, 3);
            }
            ELSE
            {
                prm[j++] = tcxltp_prm_0;
                move16();
                prm[j++] = tcxltp_prm_1;
                move16();
                prm[j++] = tcxltp_prm_2;
                move16();
            }

            /* TCX spectral data */
            lg = shr(L_frame, 1);
            lgFB = shr(st->tcx_cfg.tcx_coded_lines, 1);

            test();
            IF ( k == 0 && st->last_core_bs_fx == ACELP_CORE )
            {
                /* ACE->TCX transition */
                lg = add(lg, st->tcx_cfg.tcx_offset);
                if(st->tcx_cfg.lfacNext<0)
                {
                    lg = sub(lg,st->tcx_cfg.lfacNext);
                }

                lgFB = add(lgFB, shr(lgFB, 1));
            }

            /* TNS data */
            nTnsParams = 0;
            move16();
            nTnsBits = 0;
            move16();

            IF (st->tcx_cfg.fIsTNSAllowed)
            {
                IF( EQ_16(st->last_core_bs_fx, ACELP_CORE)&&(k==0))
                {
                    st->BER_detect = 1;
                    st->last_core_fx = TCX_20_CORE;     move16();
                    st->last_core_bs_fx = TCX_20_CORE;  move16();
                }
                test();
                test();
                SetTnsConfig(&st->tcx_cfg, 0, (st->last_core_bs_fx == ACELP_CORE) && (k == 0));

                ReadTnsData(st->tcx_cfg.pCurrentTnsConfig, st, &nTnsBits, prm+j, &nTnsParams);

                j = add(j, nTnsParams);
            }

            hm_size = shl(mult(st->TcxBandwidth, lgFB), 1);

            /*compute target bits*/
            nbits_tcx = sub(shr(sub(add(sub(sub(st->bits_frame_core, st->bits_common), nbits_igf), 1), k), 1), sub(sub(st->next_bit_pos_fx, start_bit_pos), nbits_tcx));

            /*Context HM flag*/
            test();
            test();
            IF ( st->tcx_cfg.ctx_hm && !(st->last_core_bs_fx == ACELP_CORE && k == 0) )
            {
                prm[j] = get_next_indice_fx(st, 1);
                move16();
                nbits_tcx = sub(nbits_tcx, 1);
                move16();

                IF (prm[j])  /* Read PeriodicityIndex */
                {
                    Word16 NumIndexBits = DecodeIndex(st,
                                                      hm_size >= 256,
                                                      prm+j+1);

                    flag_ctx_hm = 1;
                    move16();

                    ConfigureContextHm(
                        lgFB,
                        nbits_tcx,
                        *(prm+j+1),
                        -1,
                        &hm_cfg);

                    nbits_tcx = sub(nbits_tcx, NumIndexBits);
                }
            }
            j = add(j, NPRM_CTX_HM);
            phm_cfg = NULL;
            move16();
            if (flag_ctx_hm)
            {
                phm_cfg = &hm_cfg;
                move16();
            }
            st->resQBits[k] = ACcontextMapping_decode2_no_mem_s17_LC(st, prm+j,
                              lgFB,
                              nbits_tcx,
                              NPRM_RESQ*st->tcx_cfg.resq,
                              phm_cfg);
            move16();
            j = add(j, lgFB);

        } /* k, window index */
    }

    IF(!st->use_partial_copy)
    {
        IF (LT_16(sub(*total_nbbits, bitsRead[0]), sub(st->next_bit_pos_fx, start_bit_pos)))
        {
            st->BER_detect = 1;
            move16();
            st->next_bit_pos_fx = add(start_bit_pos, sub(*total_nbbits, bitsRead[0]));
        }
        bitsRead[0] = sub(st->next_bit_pos_fx, start_bit_pos);
        move16();
    }

    return;
}

