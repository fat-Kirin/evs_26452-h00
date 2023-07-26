/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "stl.h"        /* required by wmc_tool                       */

#include "rom_com_fx.h"

#include "prot_fx.h"    /* Function prototypes                    */
#include "cnst_fx.h"    /* Common constants                       */

/*--------------------------------------------------------------------------*/
/*  Function  hvq_pvq_bitalloc                                              */
/*  ~~~~~~~~~~~~~~~~~~~~~~~~                                                */
/*                                                                          */
/*  Calculate the number of PVQ bands to code and allocate bits based on    */
/*  the number of available bits.                                           */
/*--------------------------------------------------------------------------*/

Word16 hvq_pvq_bitalloc_fx(
    Word16 num_bits,     /* i/o: Number of available bits (including gain bits) */
    const Word32 brate,        /* i  : bitrate                     */
    const Word16 bwidth_fx,    /* i  : Encoded bandwidth           */
    const Word16 *ynrm,        /* i  : Envelope coefficients       */
    const Word32 manE_peak,    /* i  : Peak energy mantissa        */
    const Word16 expE_peak,    /* i  : Peak energy exponent        */
    Word16 *Rk,          /* o  : bit allocation for concatenated vector */
    Word16 *R,           /* i/o: Global bit allocation       */
    Word16 *sel_bands,   /* o  : Selected bands for encoding */
    Word16 *n_sel_bands  /* o  : No. of selected bands for encoding */
)
{
    Word16 num_bands, band_max_bits;
    Word16 one_over_band_max_bits;
    Word16 k;
    Word16 reciprocal, envSum, expo, align, m, n, indx;
    Word16 k_max;
    Word16 k_start;
    Word32 E_max, E_max5;
    Word32 tmp, acc;
    Word32 env_mean;
    UWord16 lsb;
    Word16 num_sfm;

    IF (EQ_16(bwidth_fx, FB))
    {
        num_sfm = SFM_N_HARM_FB;
    }
    ELSE
    {
        num_sfm = SFM_N_HARM;
    }

    IF ( EQ_32(brate, HQ_24k40))
    {
        band_max_bits = HVQ_BAND_MAX_BITS_24k;
        move16();
        one_over_band_max_bits = ONE_OVER_HVQ_BAND_MAX_BITS_24k_FX;
        move16();
        k_start = HVQ_THRES_SFM_24k;
        move16();
        IF (EQ_16(bwidth_fx, FB))
        {
            reciprocal = 2731;  /* Q15, 1/(SFM_N_HARM_FB + 1 - k_start) */     move16();
        }
        ELSE
        {
            reciprocal = 3277;  /* Q15, 1/(SFM_N_HARM + 1 - k_start) */     move16();
        }
    }
    ELSE
    {
        band_max_bits = HVQ_BAND_MAX_BITS_32k;
        move16();
        one_over_band_max_bits = ONE_OVER_HVQ_BAND_MAX_BITS_32k_FX;
        move16();
        k_start = HVQ_THRES_SFM_32k;
        move16();
        IF (EQ_16(bwidth_fx, FB))
        {
            reciprocal = 3641;  /* Q15, 1/(SFM_N_HARM_FB + 1 - k_start) */     move16();
        }
        ELSE
        {
            reciprocal = 4681;  /* Q15, 1/(SFM_N_HARM + 1 - k_start) */     move16();
        }
    }

    num_bands = mult( num_bits, one_over_band_max_bits );              /* Q0 */
    num_bits  = sub( num_bits, i_mult(num_bands, band_max_bits) );     /* Q0 */

    IF ( GE_16(num_bits, HVQ_NEW_BAND_BIT_THR))
    {
        num_bands   = add(num_bands, 1);
    }
    ELSE
    {
        num_bits    = add(num_bits, band_max_bits);
    }

    /* safety check in case of bit errors */
    if (LT_16(num_bands, 1))
    {
        return 0;
    }

    *n_sel_bands = 0;
    move16();
    envSum = 0;
    move16();
    E_max = L_deposit_l(0);
    k_max = k_start;
    move16();
    FOR ( k = k_start; k < num_sfm; k++ )
    {
        indx = ynrm[k];
        move16();
        tmp = dicn_fx[indx];  /* Q14 */
		move32();
        envSum = add(envSum, indx); /* Since the size of dicn_fx = 40, ynrm[k] must be less than 41. 16 bits are enough for envSum.*/
        IF (GT_32(tmp, E_max))
        {
            E_max = tmp;
			move32();
            k_max = k;
            move16();
        }
    }
    env_mean = L_mult(envSum, reciprocal);  /* env_mean in Q16 */
    IF (GT_32(L_sub(env_mean, L_deposit_h(ynrm[k_max])), 0x30000L))   /* condition: env_mean - ynrm[k_max] > 3 */
    {
        expo = norm_l(E_max);
        E_max = L_shl(E_max, expo);
        Mpy_32_16_ss(E_max, 0x7a12, &E_max5, &lsb); /* NB: 5.0e5 = 0x7a12(Q15) x 2^19. */
        /* True floating point value of E_max*5e5 = E_max5 x 2^(19 - expo - 14).
         *    In this context, the 32-bit E_max5 is in Q0, and
         *    -14 is due to Emax in Q14.
         * True floating point value of E_peak = manE_peak x 2^(31 - expE_peak - 2*12). See peak_vq_enc_fx().
         */

        /* Align the Q-points of the floating point Emax*5e5 and E_peak. */
        align = sub(expo, expE_peak);
        align = add(align, (19 - 14) - (31 - 2*12));
        IF (align < 0)
        {
            acc = L_sub(E_max5, L_shl(manE_peak, align));
        }
        ELSE
        {
            acc = L_sub(L_shr(E_max5, align), manE_peak);
        }

        IF (acc > 0)  /* condition: E_max*5.e5 > E_peak */
        {
            IF ( EQ_16(band_len_harm[k_max], 96))
            {
                n = 61;
            }
            ELSE
            {
                QuantaPerDsDirac_fx(band_len_harm[k_max], 1, hBitsN, &n);
            }
            m = shl(sub(num_bits, HVQ_PVQ_GAIN_BITS), 3);
            IF (GE_16(m, n))
            {
                IF (GT_16(num_bands, 1))  /* condition: num_bands > 1 */
                {
                    sel_bands[*n_sel_bands] = k_max;
                    move16();
                    *n_sel_bands = add(*n_sel_bands, 1);
                    R[k_max] = 1; /* Mark that the band has been encoded for fill_spectrum */ move16();
                }
            }
        }
    }

    /* Allocate bits */
    tmp = sub(num_bands,1);
    FOR (k = 0; k < tmp; k++)
    {
        Rk[k] = shl(sub(band_max_bits, HVQ_PVQ_GAIN_BITS), 3);
        move16();
    }
    /* NB: When it exits the above loop, k = num_bands - 1. */
    Rk[k] = shl(sub(num_bits, HVQ_PVQ_GAIN_BITS), 3);
    move16();

    return num_bands;
}

