/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/


#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "prot_fx.h"
#include "stl.h"

/*-------------------------------------------------------------------*
 * gauss_L2:
 *
 * encode an additional Gaussian excitation for unvoiced subframes and compute
 * associated xcorrelations for gains computation
 *
 * Gaussian excitation is generated by a white noise and shapes it with LPC-derived filter
 *-------------------------------------------------------------------*/
void gauss_L2(
    const Word16 h[],         /* i  : weighted LP filter impulse response   Q14+s */
    Word16 code[],            /* o  : gaussian excitation                     Q9  */
    const Word16 y2[],        /* i  : zero-memory filtered code. excitation   Q9  */
    Word16 y11[],             /* o  : zero-memory filtered gauss. excitation  Q9  */
    Word32 *gain,             /* o  : excitation gain                         Q16 */
    ACELP_CbkCorr *g_corr,    /*i/o : correlation structure for gain coding       */
    const Word16 gain_pit,    /* i  : unquantized gain of code                Q14 */
    const Word16 tilt_code,   /* i  : tilt of code                            Q15 */
    const Word16 *Aq,         /* i  : quantized LPCs                          Q12 */
    const Word16 formant_enh_num, /* i  : formant enhancement numerator factor              Q15 */
    Word16 *seed_acelp,       /*i/o : random seed                             Q0  */
    const Word16 shift
)
{
    Word16 i, tmp16;
    Word32 tmp32, tmp32_2;


    assert(gain_pit==0);

    /*-----------------------------------------------------------------*
     * Find new target for the Gaussian codebook
     *-----------------------------------------------------------------*/

    /*Generate white gaussian noise using central limit theorem method (N only 4 as E_util_random is not purely uniform)*/
    FOR (i = 0; i < L_SUBFR; i++)
    {
        *seed_acelp = own_random2_fx(*seed_acelp);
        tmp32 = L_mac(0, *seed_acelp, 1<<9);

        *seed_acelp = own_random2_fx(*seed_acelp);
        tmp32 = L_mac(tmp32, *seed_acelp, 1<<9);

        *seed_acelp = own_random2_fx(*seed_acelp);
        code[i] = mac_r(tmp32, *seed_acelp, 1<<9);
        move16();
    }

    /*Shape the gaussian excitation*/
    cb_shape_fx( 1, 0, 0, 1, 0, formant_enh_num, FORMANT_SHARPENING_G2, Aq, code, tilt_code, 0, 1 );

    /*compute 0s memory weighted synthesis contribution and find gain*/
    E_UTIL_f_convolve(code, h, y11, L_SUBFR); /* y11: Q8+shift */
    Scale_sig(y11, L_SUBFR, sub(1, shift)); /* Q9 */
    *gain = L_deposit_l(0);

    /*Update correlations for gains coding */
    tmp32 = L_shr(21474836l/*0.01f Q31*/, 31-18); /* Q18 */
    tmp32_2 = L_shr(21474836l/*0.01f Q31*/, 31-18); /* Q18 */

    FOR (i = 0; i < L_SUBFR; i++)
    {
        tmp32 = L_mac0(tmp32, y11[i], y11[i]); /* Q18 */
        tmp32_2 = L_mac0(tmp32_2, y11[i], y2[i]); /* Q18 */
    }

    tmp16 = norm_l(tmp32);
    g_corr->y1y1 = round_fx(L_shl(tmp32, tmp16));
    g_corr->y1y1_e = sub(31-18, tmp16);
    move16();

    tmp16 = norm_l(tmp32_2);
    g_corr->y1y2 = round_fx(L_shl(tmp32_2, tmp16));
    g_corr->y1y2_e = sub(31-18, tmp16);
    move16();

}

