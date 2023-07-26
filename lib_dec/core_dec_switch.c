/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "basop_util.h"
#include "prot_fx.h"
#include "stl.h"
#include "options.h"
#include "rom_com_fx.h"


void mode_switch_decoder_LPD( Decoder_State_fx *st, Word16 bandwidth, Word32 bitrate, Word16 frame_size_index
                            )
{
    Word16 fscale, switchWB;
    Word32 sr_core;
    Word8 bSwitchFromAmrwbIO;
    Word16 frame_size;


    switchWB = 0;
    move16();
    bSwitchFromAmrwbIO = 0;
    move16();

    if (EQ_16(st->last_core_fx,AMR_WB_CORE))
    {
        bSwitchFromAmrwbIO = 1;
        move16();
    }

    sr_core = getCoreSamplerateMode2(bitrate, bandwidth, st->rf_flag);

    fscale = sr2fscale(sr_core);
    move16();

    /* set number of coded lines */
    st->tcx_cfg.tcx_coded_lines = getNumTcxCodedLines(bandwidth);

    test();
    test();
    IF (( (GE_16(bandwidth, WB))&&(EQ_16(fscale,(FSCALE_DENOM*16000)/12800))&&(EQ_16(fscale,st->fscale))))
    {
        test();
        test();
        test();
        if ( ((GT_32(bitrate, 32000))&&(st->tcxonly==0))||((LE_32(bitrate,32000))&&(st->tcxonly!=0)))
        {
            switchWB = 1;
            move16();
        }
    }

    test();
    if( GT_16(st->last_L_frame_fx,L_FRAME16k)&&LE_32(st->total_brate_fx,ACELP_32k))
    {
        switchWB = 1;
        move16();
    }


    st->igf = getIgfPresent(bitrate, bandwidth, st->rf_flag);

    st->hIGFDec.infoIGFStopFreq = -1;
    move16();
    IF( st->igf )
    {
        /* switch IGF configuration */
        IGFDecSetMode( &st->hIGFDec, st->total_brate_fx, bandwidth, -1, -1, st->rf_flag);

    }

    test();
    test();
    test();
    test();
    IF (  NE_16(fscale, st->fscale)||switchWB!=0||bSwitchFromAmrwbIO!=0||EQ_16(st->last_codec_mode,MODE1)||st->force_lpd_reset)
    {
        open_decoder_LPD( st, bitrate, bandwidth );
    }
    ELSE
    {
        assert(fscale > (FSCALE_DENOM/2));

        st->fscale_old  = st->fscale;
        move16();
        st->fscale      = fscale;
        move16();
        st->L_frame_fx  = extract_l(Mult_32_16(st->sr_core , 0x0290));
        st->L_frameTCX  = extract_l(Mult_32_16(st->output_Fs_fx , 0x0290));

        st->tcx_cfg.ctx_hm = getCtxHm(bitrate, st->rf_flag);
        st->tcx_cfg.resq   = getResq(bitrate);
        move16();

        st->tcx_lpc_shaped_ari = getTcxLpcShapedAri(
            bitrate,
            bandwidth
            ,st->rf_flag
        );

        st->narrowBand = 0;
        move16();
        if ( EQ_16(bandwidth, NB))
        {
            st->narrowBand = 1;
            move16();
        }
        st->TcxBandwidth = getTcxBandwidth(bandwidth);

        st->tcx_cfg.pCurrentTnsConfig = NULL;
        st->tcx_cfg.fIsTNSAllowed = getTnsAllowed(bitrate, st->igf );
        move16();

        IF (st->tcx_cfg.fIsTNSAllowed != 0)
        {
            InitTnsConfigs(bwMode2fs[bandwidth], st->tcx_cfg.tcx_coded_lines, st->tcx_cfg.tnsConfig, st->hIGFDec.infoIGFStopFreq, st->total_brate_fx);
        }
    }

    frame_size = FrameSizeConfig[frame_size_index].frame_net_bits;
    move16();

    reconfig_decoder_LPD( st, frame_size, bandwidth, bitrate, st->last_L_frame_fx );

    test();
    IF (st->envWeighted != 0 && st->enableTcxLpc == 0)
    {
        Copy(st->lspold_uw, st->lsp_old_fx, M);
        Copy(st->lsfold_uw, st->lsf_old_fx, M);
        st->envWeighted = 0;
        move16();
    }

    /* update PLC LSF memories */
    IF( st->tcxonly == 0 )
    {
        lsp2lsf_fx( st->lsp_old_fx, st->lsfoldbfi1_fx, M, extract_l(st->sr_core) );
    }
    ELSE
    {
        E_LPC_lsp_lsf_conversion( st->lsp_old_fx, st->lsfoldbfi1_fx, M );
    }
    mvr2r_Word16(st->lsfoldbfi1_fx, st->lsfoldbfi0_fx,M);
    mvr2r_Word16(st->lsfoldbfi1_fx, st->lsf_adaptive_mean_fx,M);

    IF( st->igf != 0 )
    {
        test();
        test();
        test();
        test();
        test();
        IF( (EQ_16(st->bwidth_fx, WB)&&NE_16(st->last_extl_fx,WB_TBE))||
            (EQ_16(st->bwidth_fx, SWB) && NE_16(st->last_extl_fx, SWB_TBE)) ||
            (EQ_16(st->bwidth_fx, FB) && NE_16(st->last_extl_fx,FB_TBE)) )
        {
            TBEreset_dec_fx(st, st->bwidth_fx);
        }
        ELSE
        {
            set16_fx( st->state_lpc_syn_fx, 0, LPC_SHB_ORDER );
            set16_fx( st->state_syn_shbexc_fx, 0, L_SHB_LAHEAD );
            set16_fx( st->mem_stp_swb_fx, 0, LPC_SHB_ORDER );
            set16_fx( st->mem_zero_swb_fx, 0, LPC_SHB_ORDER );
            st->gain_prec_swb_fx = 16384;
            move16(); /*Q14 = 1 */
        }
    }

    test();
    test();
    IF( (EQ_16(bandwidth, SWB))&&
        (EQ_32(st->total_brate_fx, ACELP_16k40) || EQ_32(st->total_brate_fx, ACELP_24k40))
      )
    {
        IF (st->tec_tfa == 0)
        {
            set16_fx(st->tecDec_fx.loBuffer, 0, MAX_TEC_SMOOTHING_DEG);
        }
        st->tec_tfa = 1;
        move16();
    }
    ELSE
    {
        st->tec_tfa = 0;
        move16();
    }

    st->tec_flag = 0;
    move16();
    st->tfa_flag = 0;
    move16();

    /* needed in decoder to read the bitstream */
    st->enableGplc = 0;
    move16();
    test();
    test();
    test();
    test();
    test();
    if( (EQ_16(bandwidth, WB )&&EQ_32(bitrate,24400))||
            (sub(bandwidth, SWB) == 0 && L_sub(bitrate, 24400) == 0 )||
            (sub(bandwidth, FB) == 0 && L_sub(bitrate, 24400) == 0 ) )
    {
        st->enableGplc = 1;
        move16();
    }
    move16();
    st->dec_glr = 0;
    test();
    test();
    if( (EQ_32(st->total_brate_fx, 9600))||(EQ_32(st->total_brate_fx,16400))||
            (EQ_32(st->total_brate_fx, 24400)))
    {
        st->dec_glr = 1;
        move16();
    }
    move16();
    st->dec_glr_idx = 0;

}

