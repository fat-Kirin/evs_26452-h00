/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include "options.h"    /* Compilation switches                   */
#include "stl.h"        /* required for wmc_tool */
#include "prot_fx.h"    /* Function prototypes                    */
#include "cnst_fx.h"    /* Common constants                       */

/*--------------------------------------------------------------------------*
 * hq_bit_allocation_fx()
 *
 * Assign bits for HQ fine structure coding with PVQ
 *--------------------------------------------------------------------------*/

void hq_bit_allocation_fx(
    const Word32 core_brate,     /* i  : Core bit-rate                    Q0  */
    const Word16 length,         /* i  : Frame length                     Q0  */
    const Word16 hqswb_clas,     /* i  : HQ class                         Q0  */
    Word16 *num_bits,      /* i/o: Remaining bit budget             Q0  */
    const Word16 *normqlg2,      /* i  : Quantized norms                  Q0  */
    const Word16 nb_sfm,         /* i  : Number sub bands to be encoded   Q0  */
    const Word16 *sfmsize,       /* i  : Sub band bandwidths              Q0  */
    Word16 *noise_level,   /* o  : HVQ noise level                    */
    Word16 *R,             /* o  : Bit allocation per sub band      Q0  */
    Word16 *Rsubband,      /* o  : Fractional bit allocation        Q3  */
    Word16 *sum,           /* o  : Sum of allocated shape bits      Q0  */
    Word16 *core_sfm,      /* o  : Last coded band in core          Q0  */
    const Word16 num_env_bands   /* i  : Number sub bands to be encoded for HQ_GEN Q0  */
)
{
    Word16 i;
    Word16 idx[NB_SFM];
    Word16 wnorm[NB_SFM];
    Word16 avrg_wnorm;
    Word16 tmp, tmp2;
    Word16 E_low;
    Word16 E_hb_mean;
    Word16 E_max;
    Word16 i_max;
    /* Temp */

    Word16 sfm_limit = nb_sfm;
    move16();

    set16_fx( R, 0, NB_SFM);
    FOR( i = 0; i < nb_sfm; i++ )
    {
        idx[i] = i;
        move16();
    }
    test();
    test();
    test();
    if( NE_16(hqswb_clas, HQ_TRANSIENT)&&NE_16(hqswb_clas,HQ_HVQ)&&!(EQ_16(length,L_FRAME16k)&&EQ_32(core_brate,HQ_32k)))
    {
        /* 'nf_idx' 2-bits index written later */
        *num_bits = sub(*num_bits, 2);
    }

    test();
    IF ( EQ_16(hqswb_clas, HQ_GEN_SWB)||EQ_16(hqswb_clas,HQ_GEN_FB))
    {
        IF ( EQ_32(core_brate, HQ_32k))
        {
            *num_bits = sub(*num_bits, HQ_GENERIC_SWB_NBITS2 );
        }
        ELSE
        {
            *num_bits = sub(*num_bits, HQ_GENERIC_SWB_NBITS );
        }

        if ( EQ_16(hqswb_clas, HQ_GEN_FB))
        {
            *num_bits = sub(*num_bits, HQ_GENERIC_FB_NBITS );
        }
    }

    IF( ( EQ_16(length, L_FRAME48k))&&(NE_16(hqswb_clas,HQ_HARMONIC))&&(NE_16(hqswb_clas,HQ_HVQ)))
    {
        tmp = 0;
        move16();
        if( EQ_16(hqswb_clas,HQ_TRANSIENT))
        {
            tmp = 1;
            move16();
        }
        map_quant_weight_fx( normqlg2, wnorm, tmp );
    }
    ELSE
    {
        Copy( normqlg2, wnorm, nb_sfm );
    }

    IF( EQ_16(hqswb_clas, HQ_HARMONIC))
    {
        /* classification and limit bandwidth for bit allocation */
        sfm_limit = sub(sfm_limit, 2);
        limit_band_noise_level_calc_fx( wnorm, &sfm_limit, core_brate, noise_level );

        /* Detect important band in high frequency region */
        E_low = sum16_fx(wnorm, SFM_G1);
        i_max = 0;
        move16();
        E_max = MIN16B;
        move16();
        E_hb_mean = 0;
        move16();
        FOR( i = SFM_G1; i < nb_sfm; i++)
        {
            E_hb_mean = add(E_hb_mean, wnorm[i]);
            IF( GT_16(wnorm[i], E_max))
            {
                E_max = wnorm[i];
                move16();
                i_max = i;
                move16();
            }
        }
        E_hb_mean = shr(E_hb_mean, 4); /* Truncated division by SFM_G1 */
        set16_fx( wnorm + sfm_limit, -20, sub(nb_sfm, sfm_limit) );
        IF (L_msu0(L_deposit_l(E_low), E_max, 15) <= 0)
        {
            IF (L_msu(L_deposit_h(E_hb_mean), E_max, 21955) <= 0)   /* 21955 = 0.67 (Q15) */
            {
                if (GE_16(i_max, sfm_limit))
                {
                    wnorm[i_max] = E_max;
                    move16();
                }
            }
        }
    }
    test();
    test();
    test();
    test();
    IF( EQ_16(hqswb_clas, HQ_HVQ))
    {
        *sum = 0;
        move16();
    }
    ELSE IF ( EQ_16(hqswb_clas, HQ_GEN_SWB)||(EQ_16(hqswb_clas,HQ_TRANSIENT)&&EQ_16(length,L_FRAME32k)&&LE_32(core_brate,HQ_32k)))
    {
        *sum = BitAllocF_fx( wnorm, core_brate, *num_bits, nb_sfm, R, Rsubband, hqswb_clas, num_env_bands );
    }
    ELSE IF( EQ_16(length, L_FRAME16k)&&EQ_32(core_brate,HQ_32k))
    {
        IF( NE_16(hqswb_clas, HQ_TRANSIENT))
        {
            avrg_wnorm = wnorm[10];
            move16();
            FOR( i=11; i<18; i++ )
            {
                avrg_wnorm = add(avrg_wnorm, wnorm[i]);
            }

            avrg_wnorm  = shr(avrg_wnorm, 3);
            FOR( i=0; i<4; i++ )
            {
                if( LT_16(wnorm[i], avrg_wnorm))
                {
                    wnorm[i] = avrg_wnorm;
                    move16();
                }
            }

            /* Estimate number of bits per band */
            *sum = BitAllocWB_fx( wnorm, *num_bits, nb_sfm, R, Rsubband );
        }
        ELSE
        {
            reordvct_fx(wnorm, nb_sfm, idx);
            bitalloc_fx( wnorm, idx, *num_bits, nb_sfm, QBIT_MAX2, R, sfmsize, hqswb_clas );
            bitallocsum_fx( R, nb_sfm, sum, Rsubband, *num_bits, length, sfmsize );
        }
    }
    ELSE
    {
        reordvct_fx(wnorm, nb_sfm, idx);

        /* enlarge the wnorm value so that more bits can be allocated to (sfm_limit/2 ~ sfm_limit) range */
        IF( EQ_16(hqswb_clas, HQ_HARMONIC))
        {
            tmp = shr(sfm_limit,1);
            tmp2 = sub(tmp,1);
            FOR( i=tmp; i<sfm_limit; i++ )
            {
                wnorm[i] = wnorm[tmp2];
                move16();
            }
        }
        bitalloc_fx( wnorm, idx, *num_bits, nb_sfm, QBIT_MAX2, R, sfmsize, hqswb_clas );
        bitallocsum_fx( R, nb_sfm, sum, Rsubband, *num_bits, length, sfmsize );
    }

    /* Find last coded core band */
    *core_sfm = sub(nb_sfm, 1);
    test();
    test();
    IF( hqswb_clas == HQ_NORMAL || EQ_16(hqswb_clas, HQ_HARMONIC))
    {
        *core_sfm = find_last_band_fx(R, nb_sfm );
    }
    ELSE IF ( EQ_16(hqswb_clas, HQ_GEN_SWB)||EQ_16(hqswb_clas,HQ_GEN_FB))
    {
        *core_sfm = find_last_band_fx( R, nb_sfm );
        IF ( LT_16(*core_sfm ,num_env_bands))
        {
            *core_sfm = sub(num_env_bands,1);
        }
    }

    *num_bits = sub(*num_bits, *sum);

    return;
}
