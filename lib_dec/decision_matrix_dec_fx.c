/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <stdlib.h>
#include <assert.h>
#include "options.h"
#include "prot_fx.h"
#include "stat_dec_fx.h"
#include "rom_com_fx.h"
#include "stl.h"


/*-----------------------------------------------------------------*
 * decision_matrix_dec()
 *
 * ACELP/HQ core selection
 * Read ACELP signalling bits from the bitstream
 * Set extension layers
 *-----------------------------------------------------------------*/

void decision_matrix_dec_fx(
    Decoder_State_fx *st,              /* i/o: decoder state structure                   */
    Word16 *coder_type,        /* o  : coder type                                */
    Word16 *sharpFlag,         /* o  : formant sharpening flag                   */
    Word16 *hq_core_type,      /* o  : HQ core type                              */
    Word16 *core_switching_flag/* o  : ACELP->HQ switching frame flag            */
)
{
    Word16 start_idx;
    Word16 ppp_nelp_mode;
    Word32 ind;
    Word16 nBits;
    Word16 tmp16, temp_core;

    st->core_fx = -1;
    move16();
    st->core_brate_fx = L_deposit_l(0);
    st->extl_fx = -1;
    move16();
    st->extl_brate_fx = 0;
    move16();
    st->ppp_mode_dec_fx = 0;
    move16();
    st->nelp_mode_dec_fx = 0;
    move16();
    st->igf = 0;
    move16();

    if( GT_32(st->total_brate_fx,ACELP_8k00))
    {
        st->vbr_hw_BWE_disable_dec_fx = 0;
        move16();
    }

    IF (EQ_16(st->mdct_sw, MODE2))
    {
        st->core_fx = HQ_CORE;
        move16();
    }
    ELSE
    {
        test();
        IF( EQ_32(st->total_brate_fx,FRAME_NO_DATA)||EQ_32(st->total_brate_fx,SID_2k40))
        {
            st->core_fx = ACELP_CORE;
            move16();
            st->core_brate_fx = st->total_brate_fx;
            move32();

            IF( NE_32(st->total_brate_fx,FRAME_NO_DATA))
            {
                st->cng_type_fx = get_next_indice_fx( st, 1 );

                IF( EQ_16(st->cng_type_fx,LP_CNG))
                {
                    st->L_frame_fx = L_FRAME;
                    move16();

                    tmp16 = get_next_indice_fx( st, 1 );
                    if( EQ_16(tmp16,1))
                    {
                        st->L_frame_fx = L_FRAME16k;
                        move16();
                    }
                }
                ELSE
                {
                    st->bwidth_fx = get_next_indice_fx(st, 2);

                    tmp16 = get_next_indice_fx(st, 1);
                    move16();

                    st->L_frame_fx = L_FRAME16k;
                    move16();
                    if( tmp16 == 0 )
                    {
                        st->L_frame_fx = L_FRAME;
                        move16();
                    }
                }
            }

            test();
            if( GE_32(st->output_Fs_fx,32000)&&GE_16(st->bwidth_fx,SWB))
            {
                st->extl_fx = SWB_CNG;
                move16();
            }

            test();
            test();
            test();
            if( EQ_32(st->total_brate_fx,FRAME_NO_DATA)&&st->prev_bfi_fx&&!st->bfi_fx&&GT_16(st->L_frame_fx,L_FRAME16k))
            {
                st->L_frame_fx = st->last_CNG_L_frame_fx;
                move16();
            }

            return;
        }

        /* SC-VBR */
        ELSE IF( EQ_32(st->total_brate_fx,PPP_NELP_2k80))
        {
            st->core_fx = ACELP_CORE;
            move16();
            st->core_brate_fx = PPP_NELP_2k80;
            move32();
            st->L_frame_fx = L_FRAME;
            move16();
            st->fscale = sr2fscale(INT_FS_FX);
            move16();

            IF ( st->ini_frame_fx == 0 )
            {
                /* avoid switching of internal ACELP Fs in the very first frame */
                st->last_L_frame_fx = st->L_frame_fx;
                st->last_core_fx = st->core_fx;
                st->last_core_brate_fx = st->core_brate_fx;
                st->last_extl_fx = st->extl_fx;
            }

            st->vbr_hw_BWE_disable_dec_fx = 1;
            move16();
            get_next_indice_fx( st, 1 );

            ppp_nelp_mode = get_next_indice_fx( st, 2 );

            /* 0 - PPP_NB, 1 - PPP_WB, 2 - NELP_NB, 3 - NELP_WB */
            IF( ppp_nelp_mode == 0 )
            {
                st->ppp_mode_dec_fx = 1;
                move16();
                *coder_type = VOICED;
                move16();
                st->bwidth_fx = NB;
                move16();
            }
            ELSE IF( EQ_16(ppp_nelp_mode,1))
            {
                st->ppp_mode_dec_fx = 1;
                move16();
                *coder_type = VOICED;
                move16();
                st->bwidth_fx = WB;
                move16();
            }
            ELSE IF( EQ_16(ppp_nelp_mode,2))
            {
                st->nelp_mode_dec_fx = 1;
                move16();
                *coder_type = UNVOICED;
                move16();
                st->bwidth_fx = NB;
                move16();
            }
            ELSE IF( EQ_16(ppp_nelp_mode,3))
            {
                st->nelp_mode_dec_fx = 1;
                move16();
                *coder_type = UNVOICED;
                move16();
                st->bwidth_fx = WB;
                move16();
            }


            return;
        }

        /*---------------------------------------------------------------------*
         * ACELP/HQ core selection
         *---------------------------------------------------------------------*/

        test();
        IF( LT_32(st->total_brate_fx,ACELP_24k40))
        {
            st->core_fx = ACELP_CORE;
            move16();
        }
        ELSE IF( GE_32(st->total_brate_fx,ACELP_24k40)&&LE_32(st->total_brate_fx,ACELP_64k))
        {
            /* read the ACELP/HQ core selection bit */
            temp_core = get_next_indice_fx( st, 1 );

            st->core_fx = HQ_CORE;
            move16();
            if( temp_core == 0 )
            {
                st->core_fx = ACELP_CORE;
                move16();
            }
        }
    }

    /*-----------------------------------------------------------------*
     * Read ACELP signalling bits from the bitstream
     *-----------------------------------------------------------------*/

    IF( EQ_16(st->core_fx,ACELP_CORE))
    {
        /* find the section in the ACELP signalling table corresponding to bitrate */
        start_idx = 0;
        move16();
        WHILE( NE_32(acelp_sig_tbl[start_idx],st->total_brate_fx))
        {
            start_idx = add(start_idx,1);
            IF( GE_16(start_idx,MAX_ACELP_SIG))
            {
                st->BER_detect = 1;
                move16();
                start_idx = 0;
                move16();
                break;
            }
        }

        /* skip the bitrate */
        start_idx = add(start_idx,1);

        /* retrieve the number of bits */
        nBits = extract_l(acelp_sig_tbl[start_idx]);
        start_idx = add(start_idx,1);

        start_idx = add(start_idx,get_next_indice_fx( st, nBits ));
        IF( start_idx >= MAX_ACELP_SIG )
        {
            ind = 0;
            move16();
            st->BER_detect = 1;
            move16();
        }
        ELSE
        {
            /* retrieve the signalling indice */
            ind = acelp_sig_tbl[start_idx];

            /* convert signalling indice into signalling information */
            *coder_type = extract_l(L_and(ind,0x7L));

            IF( EQ_16(*coder_type,LR_MDCT))
            {
                st->core_fx = HQ_CORE;
                move16();
                st->bwidth_fx = extract_l(L_shr(ind,3) & 0x7L);
            }
            ELSE
            {
                st->bwidth_fx = extract_l(L_and(L_shr(ind,3),0x7L));
                *sharpFlag = extract_l(L_and(L_shr(ind,6),0x1L));

            }
        }

        /* detect corrupted signalling (due to bit errors) */
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
        test();
        test();
        test();
        test();
        test();
        test();
        IF( ( st->BER_detect ) ||
            ( GE_32(ind,1<<7)  ) ||
            ( LE_32(st->total_brate_fx,ACELP_13k20)  && EQ_16(st->bwidth_fx,FB)  ) ||
            ( GE_32(st->total_brate_fx,ACELP_32k)    && EQ_16(st->bwidth_fx,NB)  ) ||
            ( GE_32(st->total_brate_fx,ACELP_32k)    && !(EQ_16(*coder_type,GENERIC)  || EQ_16(*coder_type,TRANSITION)  || EQ_16(*coder_type,INACTIVE)  ) ) ||
            ( LT_32(st->total_brate_fx,ACELP_13k20)  && NE_16(st->bwidth_fx,NB)  && EQ_16(*coder_type,LR_MDCT)  ) ||
            ( GE_32(st->total_brate_fx,ACELP_13k20)  && EQ_16(*coder_type,UNVOICED)  ) ||
            ( GE_32(st->total_brate_fx,ACELP_13k20)  && EQ_16(*coder_type,AUDIO)  && EQ_16(st->bwidth_fx,NB)  )
          )
        {
            st->BER_detect = 0;
            move16();
            st->bfi_fx = 1;
            move16();

            IF( st->ini_frame_fx == 0 )
            {
                st->core_fx = ACELP_CORE;
                move16();
                st->L_frame_fx = L_FRAME;
                move16();
                st->last_core_fx = st->core_fx;
                move16();
                st->last_core_brate_fx = st->core_brate_fx;
                move32();
            }
            ELSE IF( EQ_32(st->last_total_brate_fx, -1)) /* can happen in case of BER when no good frame was received before */
            {
                *coder_type = st->last_coder_type_fx;
                move16();
                st->bwidth_fx = st->last_bwidth_fx;
                move16();
                st->total_brate_fx = st->last_total_brate_ber_fx;
                move32();
                test();
                IF( EQ_16(st->last_core_fx, AMR_WB_CORE))
                {
                    st->core_fx = ACELP_CORE;
                    move16();
                    st->codec_mode = MODE1;
                    move16();
                }
                ELSE IF( EQ_16(st->last_core_bfi, TCX_20_CORE)||EQ_16(st->last_core_bfi,TCX_10_CORE))
                {
                    st->core_fx = st->last_core_bfi;
                    move16();
                    st->codec_mode = MODE2;
                    move16();
                }
                ELSE
                {
                    st->core_fx = st->last_core_fx;
                    move16();
                    st->codec_mode = MODE1;
                    move16();
                }
                st->core_brate_fx = st->last_core_brate_fx;
                move32();
                st->extl_fx = st->last_extl_fx;
                move16();
                st->extl_brate_fx = L_sub(st->total_brate_fx, st->core_brate_fx);
                move32();
            }
            ELSE
            {
                *coder_type = st->last_coder_type_fx;
                move16();
                st->bwidth_fx = st->last_bwidth_fx;
                move16();
                st->total_brate_fx = st->last_total_brate_fx;
                move16();

                test();
                IF( EQ_16(st->last_core_fx,AMR_WB_CORE))
                {
                    st->core_fx = ACELP_CORE;
                    move16();
                    st->codec_mode = MODE1;
                    move16();
                }
                ELSE IF( EQ_16(st->last_core_fx,TCX_20_CORE)||EQ_16(st->last_core_fx,TCX_10_CORE))
                {
                    st->core_fx = st->last_core_fx;
                    move16();
                    st->codec_mode = MODE2;
                    move16();
                }
                ELSE
                {
                    st->core_fx = st->last_core_fx;
                    move16();
                    st->codec_mode = MODE1;
                    move16();
                }
                st->core_brate_fx = st->last_core_brate_fx;
                move32();
                st->extl_fx = st->last_extl_fx;
                move16();
                st->extl_brate_fx = L_sub(st->total_brate_fx,st->core_brate_fx);
            }

            return;
        }
    }

    /*-----------------------------------------------------------------*
     * Set extension layers
     *-----------------------------------------------------------------*/

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
    test();
    test();
    test();
    test();
    IF( EQ_16(st->core_fx,ACELP_CORE)&&EQ_16(st->bwidth_fx,WB)&&LT_32(st->total_brate_fx,ACELP_9k60))
    {
        if( st->vbr_hw_BWE_disable_dec_fx == 0 )
        {
            st->extl_fx = WB_BWE;
            move16();
        }
    }
    ELSE IF( EQ_16(st->core_fx,ACELP_CORE)&&EQ_16(st->bwidth_fx,WB)&&GE_32(st->total_brate_fx,ACELP_9k60)&&LE_32(st->total_brate_fx,ACELP_16k40))
    {
        /* read the WB TBE/BWE selection bit */
        tmp16 = get_next_indice_fx( st, 1 );
        IF( EQ_16(tmp16,1))
        {
            st->extl_fx = WB_BWE;
            move16();
            st->extl_brate_fx = WB_BWE_0k35;
            move32();
        }
        ELSE
        {
            st->extl_fx = WB_TBE;
            move16();
            st->extl_brate_fx = WB_TBE_1k05;
            move32();
        }
    }
    ELSE IF( EQ_16(st->core_fx,ACELP_CORE)&&(EQ_16(st->bwidth_fx,SWB)||EQ_16(st->bwidth_fx,FB))&&GE_32(st->total_brate_fx,ACELP_13k20))
    {
        IF( GE_32(st->total_brate_fx,ACELP_48k))
        {
            st->extl_fx = SWB_BWE_HIGHRATE;
            move16();
            if( EQ_16(st->bwidth_fx,FB))
            {
                st->extl_fx = FB_BWE_HIGHRATE;
                move16();
            }

            st->extl_brate_fx = SWB_BWE_16k;
            move32();
        }

        /* read the SWB TBE/BWE selection bit */
        ELSE
        {
            tmp16 = get_next_indice_fx( st, 1 );
            IF( tmp16 )
            {
                st->extl_fx = SWB_BWE;
                move16();
                st->extl_brate_fx = SWB_BWE_1k6;
                move32();
            }
            ELSE
            {
                st->extl_fx = SWB_TBE;
                move16();
                st->extl_brate_fx = SWB_TBE_1k6;
                move32();
                if( GE_32(st->total_brate_fx,ACELP_24k40))
                {
                    st->extl_brate_fx = SWB_TBE_2k8;
                    move32();
                }
            }
        }

        /* set FB TBE and FB BWE extension layers */
        test();
        IF( EQ_16(st->bwidth_fx,FB)&&GE_32(st->total_brate_fx,ACELP_24k40))
        {
            IF( EQ_16(st->extl_fx,SWB_BWE))
            {
                st->extl_fx = FB_BWE;
                move16();
                st->extl_brate_fx = FB_BWE_1k8;
                move32();
            }
            ELSE IF( EQ_16(st->extl_fx,SWB_TBE))
            {
                st->extl_fx = FB_TBE;
                move16();
                {
                    st->extl_brate_fx = FB_TBE_3k0;
                    move32();
                }
            }
        }
    }

    /* set core bitrate */
    st->core_brate_fx = L_sub(st->total_brate_fx,st->extl_brate_fx);

    /*-----------------------------------------------------------------*
     * Read HQ signalling bits from the bitstream
     * Set HQ core type
     *-----------------------------------------------------------------*/


    IF( EQ_16(st->core_fx,HQ_CORE))
    {
        IF( NE_16(st->mdct_sw, MODE2))
        {
            /* skip the HQ/TCX core switching flag */
            get_next_indice_tmp_fx( st, 1 );
        }

        /* read ACELP->HQ core switching flag */
        *core_switching_flag = get_next_indice_fx( st, 1 );

        IF( EQ_16(*core_switching_flag,1))
        {
            st->last_L_frame_ori_fx = st->last_L_frame_fx;
            move16();

            /* read ACELP L_frame info */
            st->last_L_frame_fx = L_FRAME16k;
            move16();
            tmp16 = get_next_indice_fx( st, 1 );
            if( tmp16 == 0 )
            {
                st->last_L_frame_fx = L_FRAME;
                move16();
            }
        }

        IF( NE_16(st->mdct_sw, MODE2))
        {

            /* read/set band-width (needed for different I/O sampling rate support) */
            IF( GT_32(st->total_brate_fx,ACELP_16k40))
            {
                tmp16 = get_next_indice_fx( st, 2 );

                IF( tmp16 == 0 )
                {
                    st->bwidth_fx = NB;
                    move16();
                }
                ELSE IF( EQ_16(tmp16,1))
                {
                    st->bwidth_fx = WB;
                    move16();
                }
                ELSE IF( EQ_16(tmp16,2))
                {
                    st->bwidth_fx = SWB;
                    move16();
                }
                ELSE
                {
                    st->bwidth_fx = FB;
                    move16();
                }
            }
        }

        /* detect bit errors in signalling */
        test();
        test();
        test();
        test();
        IF( ( GE_32(st->total_brate_fx,ACELP_24k40)&&EQ_16(st->bwidth_fx,NB))||
            ( EQ_16(st->core_fx,HQ_CORE)  && LE_32(st->total_brate_fx,LRMDCT_CROSSOVER_POINT)  && EQ_16(st->bwidth_fx,FB) )
          )
        {
            st->bfi_fx = 1;
            move16();

            st->core_brate_fx = st->total_brate_fx;
            move32();
            st->extl_fx = -1;
            move16();
            st->extl_brate_fx = 0;
            move32();
            IF( EQ_16(st->last_core_fx,AMR_WB_CORE))
            {
                st->core_fx = ACELP_CORE;
                move16();
                st->L_frame_fx = L_FRAME;
                move16();
                st->codec_mode = MODE1;
                move16();
                st->last_L_frame_fx = L_FRAME;
                move16();

                IF( GE_32(st->total_brate_fx,ACELP_16k40))
                {
                    st->total_brate_fx = ACELP_13k20;
                    move32();
                    st->core_brate_fx = st->total_brate_fx;
                    move32();
                }
            }
            ELSE
            {
                /* make sure, we are in a valid configuration wrt to bandwidth */
                st->bwidth_fx = WB;
                move16();
            }
        }

        /* set HQ core type */
        *hq_core_type = NORMAL_HQ_CORE;
        move16();

        test();
        test();
        IF( (EQ_16(st->bwidth_fx,SWB)||EQ_16(st->bwidth_fx,WB))&&LE_32(st->total_brate_fx,LRMDCT_CROSSOVER_POINT))
        {
            *hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
        ELSE IF( EQ_16(st->bwidth_fx,NB))
        {
            *hq_core_type = LOW_RATE_HQ_CORE;
            move16();
        }
    }

    /*-----------------------------------------------------------------*
     * Set ACELP frame lnegth
     *-----------------------------------------------------------------*/

    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();
    IF( EQ_32(st->core_brate_fx,FRAME_NO_DATA))
    {
        /* prevent "L_frame" changes in CNG segments */
        st->L_frame_fx = st->last_L_frame_fx;
        move16();
    }
    ELSE IF( EQ_32(st->core_brate_fx,SID_2k40)&&EQ_16(st->bwidth_fx,WB)&&st->first_CNG_fx&&LT_16(st->act_cnt2_fx,MIN_ACT_CNG_UPD))
    {
        /* prevent "L_frame" changes in SID frame after short segment of active frames */
        st->L_frame_fx = st->last_CNG_L_frame_fx;
        move16();
    }
    ELSE IF( ( EQ_32(st->core_brate_fx,SID_2k40)&&GE_32(st->total_brate_fx,ACELP_9k60)&&EQ_16(st->bwidth_fx,WB))||
             ( GT_32(st->total_brate_fx,ACELP_24k40)  && LT_32(st->total_brate_fx,HQ_96k)  ) || ( EQ_32(st->total_brate_fx,ACELP_24k40)  && GE_16(st->bwidth_fx,WB)  ) )
    {
        st->L_frame_fx = L_FRAME16k;
        move16();
    }
    ELSE
    {
        st->L_frame_fx = L_FRAME;
        move16();
    }

    st->nb_subfr = NB_SUBFR;
    move16();
    if ( EQ_16(st->L_frame_fx,L_FRAME16k))
    {
        st->nb_subfr = NB_SUBFR16k;
        move16();
    }

    test();
    IF( EQ_32(st->output_Fs_fx,8000))
    {
        st->extl_fx = -1;
        move16();
    }
    ELSE IF( EQ_32(st->output_Fs_fx,16000)&&EQ_16(st->L_frame_fx,L_FRAME16k))
    {
        st->extl_fx = -1;
        move16();
        st->extl_brate_fx = L_deposit_l(0);
    }

    IF( st->ini_frame_fx == 0 )
    {
        /* avoid switching of internal ACELP Fs in the very first frame */
        st->last_L_frame_fx = st->L_frame_fx;
        move16();
        st->last_core_fx = st->core_fx;
        move16();
        st->last_core_brate_fx = st->core_brate_fx;
        move32();
        st->last_extl_fx = st->extl_fx;
        move16();
    }

    return;
}
