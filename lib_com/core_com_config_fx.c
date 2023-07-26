/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "options.h"
#include "stl.h"
#include "basop_util.h"
#include "prot_fx.h"
#include "rom_com_fx.h"


Word8 getTcxonly(const Word32 bitrate)
{

    Word8 tcxonly;

    tcxonly = 0;
    move16();
    if( GT_32(bitrate,32000))
    {
        tcxonly = 1;
        move16();
    }

    return tcxonly;
}

Word8 getCtxHm(const Word32 bitrate, const Word16 rf_flag)
{

    Word8 ctx_hm;

    ctx_hm = 0;
    move16();
    test();
    if( (bitrate > LPC_SHAPED_ARI_MAX_RATE) && (bitrate <= 64000) && !rf_flag)
    {

        ctx_hm = 1;
        move16();
    }


    return ctx_hm;
}


Word8 getResq(const Word32 bitrate)
{

    Word8 resq;

    resq = 0;
    move16();
    if (LE_32(bitrate,64000))
    {
        resq = 1;
        move16();
    }

    return resq;
}


Word8 getTnsAllowed(const Word32 bitrate
                    ,const Word16 igf
                   )
{
    Word8 tnsAllowed;

    tnsAllowed = 0;
    move16();
    IF ( igf != 0 )
    {
        if( GT_32(bitrate, HQ_16k40))
        {
            tnsAllowed = 1;
            move16();
        }
    }
    ELSE
    {
        if( GT_32(bitrate, HQ_32k))
        {
            tnsAllowed = 1;
            move16();
        }
    }

    return tnsAllowed;
}


Word8 getRestrictedMode(const Word32 bitrate, const Word16 Opt_AMR_WB)
{

    Word8 restrictedMode;

    restrictedMode = 3;
    move16();

    test();
    IF ( (Opt_AMR_WB == 0) && (GT_32(bitrate,32000)))
    {
        restrictedMode = 6;
        move16();
    }
    ELSE IF( Opt_AMR_WB )
    {
        restrictedMode = 1;
        move16();
    }

    return restrictedMode;
}


Word16 sr2fscale(const Word32 sr)
{
    Word16 fscale;

    SWITCH(sr)
    {
    case 8000:
        fscale = (FSCALE_DENOM*8000)/12800;
        move16();
        BREAK;

    case 12800:
        fscale = FSCALE_DENOM;
        move16();
        BREAK;

    case 16000:
        fscale = (FSCALE_DENOM*16000)/12800;
        move16();
        BREAK;

    case 25600:
        fscale = (FSCALE_DENOM*25600)/12800;
        move16();
        BREAK;

    case 32000:
        fscale = (FSCALE_DENOM*32000)/12800;
        move16();
        BREAK;

    case 48000:
        fscale = (FSCALE_DENOM*48000)/12800;
        move16();
        BREAK;

    default:
        assert(0);
        fscale = 0; /* just to avoid compiler warning */
        BREAK;
    }
    return fscale;
}

Word32 getCoreSamplerateMode2(const Word32 bitrate, const Word16 bandwidth, const Word16 rf_mode)
{

    Word32 sr_core;
    sr_core = -1;       /* to suppress MSVC warning */ move32();

    test();
    test();
    test();
    test();
    test();
    test();
    test();
    test();

    IF( EQ_32( bandwidth,NB))
    {
        sr_core = 12800;
        move32();
    }

    ELSE IF (  L_and(EQ_32(bandwidth,WB),LT_32(bitrate,13200))||
               L_and(EQ_32(bandwidth,SWB), LE_32(bitrate,13200)) || EQ_16(rf_mode,1)  )

    {
        sr_core = 12800;
        move32();
    }
    ELSE IF (EQ_32(bandwidth,WB)||((LE_32(bitrate,32000))&&((EQ_32(bandwidth,SWB))||(EQ_32(bandwidth,FB)))))
    {
        sr_core = 16000;
        move32();
    }
    ELSE IF ( ((EQ_32(bandwidth,SWB))||(EQ_32(bandwidth,FB)))&&(LE_32(bitrate,64000)))
    {
        sr_core = 25600;
        move32();
    }
    ELSE IF (EQ_32(bandwidth,SWB)||EQ_32(bandwidth,FB))
    {
        sr_core = 32000;
        move32();
    }
    ELSE
    {
        assert(0);
    }

    return sr_core;
}

Word16 getTcxBandwidth(const Word16 bandwidth)
{

    Word16 tcxBandwidth;

    tcxBandwidth = 16384/*0.5f Q15*/;
    move16();
    if(EQ_16(bandwidth, NB))
    {
        tcxBandwidth = 10240/*0.3125f Q15*/;
        move16();

    }

    return tcxBandwidth;
}


Word8 getIgfPresent(
    const Word32 bitrate,
    const Word16 bandwidth
    ,const Word16 rf_mode
)
{
    Word8 igfPresent;

    igfPresent = 0;
    move16();

    test();
    test();
    if( (EQ_16(bandwidth, SWB))&&(GE_32(bitrate,ACELP_9k60))&&(LT_32(bitrate,HQ_64k)))
    {
        igfPresent = 1;
        move16();
    }

    test();
    if( EQ_16(bandwidth, FB)&&(GE_32(bitrate,ACELP_16k40)))
    {
        igfPresent = 1;
        move16();
    }

    test();
    if( (EQ_16(bandwidth, WB))&&(EQ_32(bitrate,ACELP_9k60)))
    {
        igfPresent = 1;
        move16();
    }

    test();
    test();
    test();
    if( ((EQ_16(bandwidth, WB))||(EQ_16(bandwidth,SWB)))&&(EQ_16(rf_mode,1))&&(EQ_32(bitrate,ACELP_13k20)))
    {
        igfPresent = 1;
        move16();
    }

    return igfPresent;
}


Word8 getCnaPresent(
    const Word32 bitrate,
    const Word16 bandwidth
)
{
    Word8 flag_cna = 0;

    flag_cna = 0;
    move16();
    test();
    if( EQ_16(bandwidth, NB)&&(LE_32(bitrate,ACELP_13k20)))
    {
        flag_cna = 1;
        move16();
    }

    test();
    if( (EQ_16(bandwidth, WB))&&(LE_32(bitrate,ACELP_13k20)))
    {
        flag_cna = 1;
        move16();
    }

    test();
    if( (EQ_16(bandwidth, SWB))&&(LE_32(bitrate,ACELP_13k20)))
    {
        flag_cna = 1;
        move16();
    }

    return flag_cna;
}

Word8 getTcxLtp(const Word32 sr_core)
{

    Word8 tcxltp = 0;

    tcxltp = 0;
    move16();
    test();
    if ( (LE_32(sr_core, 25600)))
    {
        tcxltp = 1;
        move16();
    }

    return tcxltp;
}


Word16 initPitchLagParameters(
    const Word32 sr_core,
    Word16 *pit_min,
    Word16 *pit_fr1,
    Word16 *pit_fr1b,
    Word16 *pit_fr2,
    Word16 *pit_max
)
{

    Word16 pit_res_max;

    IF (EQ_32(sr_core, 12800))
    {

        *pit_min = PIT_MIN_12k8;
        move16();
        *pit_max = PIT_MAX_12k8;
        move16();
        *pit_fr2 = PIT_FR2_12k8;
        move16();
        *pit_fr1 = PIT_FR1_12k8;
        move16();
        *pit_fr1b = PIT_FR1_8b_12k8;
        move16();
        pit_res_max = 4;
        move16();

    }
    ELSE IF (EQ_32(sr_core, 16000))
    {

        *pit_min = PIT_MIN_16k;
        move16();
        *pit_max = PIT_MAX_16k;
        move16();
        *pit_fr2 = PIT_FR2_16k;
        move16();
        *pit_fr1 = PIT_FR1_16k;
        move16();
        *pit_fr1b = PIT_FR1_8b_16k;
        move16();
        pit_res_max = 6;
        move16();

    }
    ELSE IF (EQ_32(sr_core, 25600))
    {

        *pit_min = PIT_MIN_25k6;
        move16();
        *pit_max = PIT_MAX_25k6;
        move16();
        *pit_fr2 = PIT_FR2_25k6;
        move16();
        *pit_fr1 = PIT_FR1_25k6;
        move16();
        *pit_fr1b = PIT_FR1_8b_25k6;
        move16();
        pit_res_max = 4;
        move16();
    }
    ELSE /* sr_core==32000 */
    {

        *pit_min = PIT_MIN_32k;
        move16();
        *pit_max = PIT_MAX_32k;
        move16();
        *pit_fr2 = PIT_FR2_32k;
        move16();
        *pit_fr1 = PIT_FR1_32k;
        move16();
        *pit_fr1b = PIT_FR1_8b_32k;
        move16();
        pit_res_max = 6;
        move16();

    }

    return pit_res_max;
}

Word16 getNumTcxCodedLines(const Word16 bwidth)
{

    Word16 tcx_coded_lines;

    tcx_coded_lines = 0;
    move16();

    if(EQ_16(bwidth, NB))
    {
        tcx_coded_lines = 160;
        move16();
    }

    if(EQ_16(bwidth, WB))
    {
        tcx_coded_lines = 320;
        move16();
    }

    if(EQ_16(bwidth, SWB))
    {
        tcx_coded_lines = 640;
        move16();
    }

    if(EQ_16(bwidth, FB))
    {
        tcx_coded_lines = 960;
        move16();
    }

    return tcx_coded_lines;
}

Word16 getTcxLpcShapedAri(
    const Word32 total_brate,
    const Word16 bwidth
    ,const Word16 rf_mode
)
{
    Word16 tcx_lpc_shaped_ari = 0;
    move16();

    (void) bwidth;

    test();
    if( (LE_32(total_brate, LPC_SHAPED_ARI_MAX_RATE))||rf_mode)
    {
        tcx_lpc_shaped_ari = 1;
        move16();
    }


    return tcx_lpc_shaped_ari;
}
