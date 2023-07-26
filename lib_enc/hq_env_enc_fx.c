/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <stdlib.h>
#include "options.h"        /* Compilation switches                   */
#include "prot_fx.h"        /* Function prototypes                    */
#include "rom_enc_fx.h"
#include "stl.h"

/*--------------------------------------------------------------------------------------*
 * encode_envelope_indices_fx()
 *
 * Encode envelope indices
 *--------------------------------------------------------------------------------------*/

Word16 encode_envelope_indices_fx(   /* o  : Number of bits if flag_pack=0,0 if flag_pack=1  Q0  */
    Encoder_State_fx *st_fx,             /* i/o: encoder state structure                             */
    const Word16 num_sfm,            /* i  : Number of subbands                              Q0  */
    const Word16 numnrmibits,        /* i  : Bitrate of fall-back coding mode                Q0  */
    Word16 *difidx,            /* i/o: Diff indices/encoded diff indices               Q0  */
    Word16 *LCmode,            /* o  : Coding mode if flag_pack=0, i : if flag_pack=1  Q0  */
    const Word16 flag_pack,          /* i  : indicator of packing or estimating bits         Q0  */
    const Word16 flag_HQ2            /* i  : indicator of HQ2 core                           Q0  */
    ,const Word16 is_transient        /* i  : indicator of HQ_TRANSIENT                       Q0  */
)
{
    Word16 bits;
    Word16 prevj;
    Word16 hcode_l;
    Word16 i,j;
    Word16 difidx_flag;
    Word16 index_max, index_min, index_rad;
    Word16 difidx_org[NB_SFM];      /* lenght of this buffer is max(BANDS_MAX,NB_SFM) */
    Word16 m, r;
    Word16 v, k;

    set16_fx( difidx_org, 0, NB_SFM );
    difidx_flag = 0;
    move16();

    /*------------------------------------------------------------------*
     * Check Huffman encoding for QNorm indices
     *------------------------------------------------------------------*/

    /* LC mode index is changed to synchronize LR_MDCT signaling    */
    /* LC mode 0 = Context based coding                             */
    /* LC mode 1 = resized huffman coding                           */
    /* LC mode 2 = normal Huffman Coding                            */
    /* LC mode 3 = bit packing                                      */
    IF ( flag_pack == 0 )
    {
        test();
        IF( is_transient && EQ_16(flag_HQ2, LOW_RATE_HQ_CORE_TRAN))
        {
            bits = 0;
            move16();
            index_max = 0;
            move16();
            index_min = 31;
            move16();
            FOR( i = 0; i< num_sfm; i++ )
            {
                IF( GT_16(difidx[i], index_max))
                {
                    index_max = difidx[i];
                    move16();
                }
                IF( LT_16(difidx[i], index_min))
                {
                    index_min = difidx[i];
                    move16();
                }
            }
            test();
            IF(GT_16(index_min, 10)&&LT_16(index_max,22))
            {
                FOR( i = 1; i < num_sfm; i++ )
                {
                    j = difidx[i];
                    move16();
                    bits = add(bits, huffsizn_tran[j]);
                }
            }
            hcode_l= 0;
            move16();
            *LCmode = 0;
            move16();
            prevj = add(difidx[0], OFFSET_NORM);
            /* LC mode 0 = Context based coding        */
            FOR( i = 1; i < num_sfm; i++ )
            {
                j = difidx[i];
                move16();
                IF( GT_16(prevj, HTH_NORM))
                {
                    /* above */
                    hcode_l = add(hcode_l, huffsizn_n_fx[31-j]);
                }
                ELSE
                {
                    IF( LT_16(prevj, LTH_NORM))
                    {
                        /* less */
                        hcode_l = add(hcode_l, huffsizn_n_fx[j]);
                    }
                    ELSE
                    {
                        /* equal */
                        hcode_l = add(hcode_l, huffsizn_e_fx[j]);
                    }
                }
                prevj = j;
                move16();
            }
            test();
            IF( GE_16(hcode_l, bits)&&bits!=0)
            {
                /* LC mode 1 Transient Huffman Coding   */
                *LCmode = 1;
                move16();
                hcode_l = bits;
                move16();
            }
        }
        ELSE
        {
            /* Check bits if LC mode == 3 -> Check bits if LC mode == 0 */
            hcode_l= 0;
            move16();
            prevj = add(difidx[0], OFFSET_NORM);
            FOR( i = 1; i < num_sfm; i++ )
            {
                j = difidx[i];
                move16();
                IF( GT_16(prevj, HTH_NORM))
                {
                    /* above */
                    hcode_l = add(hcode_l, huffsizn_n_fx[sub(31,j)]);
                }
                ELSE
                {
                    IF( LT_16(prevj, LTH_NORM))
                    {
                        /* less */
                        hcode_l = add(hcode_l, huffsizn_n_fx[j]);
                    }
                    ELSE
                    {
                        /* equal */
                        hcode_l = add(hcode_l, huffsizn_e_fx[j]);
                    }
                }
                prevj = j;
                move16();
            }

            *LCmode = 0;
            move16();

            /* LR-MDCT core doesn't have coding mode 2 and 3 */
            IF( flag_HQ2 == NORMAL_HQ_CORE )
            {
                /* Check bits if LC mode == 1 -> Check bits if LC mode == 2 */
                bits = 0;
                move16();
                FOR( i = 1; i < num_sfm; i++ )
                {
                    j = difidx[i];
                    move16();
                    bits = add(bits, huffsizn[j]);
                }

                /*------------------------------------------------------------------------------*
                 * comparing bit expenses of coding mode 2 with that of the optimal coding mode
                 *------------------------------------------------------------------------------*/

                if( GT_16(hcode_l, bits))
                {
                    *LCmode = 2;
                    move16();
                }
                hcode_l = s_min(hcode_l, bits);
            }

            /* Check bits if LC mode == 2 -> Check bits if LC mode == 1  */
            bits = 0;
            move16();
            index_max = 0;
            move16();
            index_min = 31;
            move16();
            FOR( i = 1; i < num_sfm; i++ )
            {
                difidx_org[i] = difidx[i];
                move16();
            }

            difidx_flag = 0;
            move16();
            FOR( i = 2; i < num_sfm; i++ )
            {
                IF( GT_16(difidx_org[i-1], 17))
                {
                    difidx[i] = add(difidx_org[i], s_min(sub(difidx_org[i-1],17),3));
                    move16();
                    IF( GT_16(difidx[i], 31))
                    {
                        difidx_flag = 1;
                        move16();
                        BREAK;
                    }
                }

                IF( LT_16(difidx_org[i-1], 13))
                {
                    difidx[i] = add(difidx_org[i], s_max(sub(difidx_org[i-1],13),-3));
                    move16();
                    IF( difidx[i] < 0 )
                    {
                        difidx_flag = 1;
                        move16();
                        BREAK;
                    }
                }
            }

            index_rad = 0;
            move16();
            IF( difidx_flag == 0 )
            {
                FOR( i = 1; i< num_sfm; i++ )
                {
                    index_max = s_max(index_max, difidx[i]);
                    index_min = s_min(index_min, difidx[i]);
                }

                index_rad = s_max(sub(15, index_min),sub(index_max, 15));

                IF( LE_16(index_rad, HUFF_THR))
                {
                    FOR( i = 1; i < num_sfm; i++ )
                    {
                        j = difidx[i];
                        move16();
                        bits  = add(bits, resize_huffsizn[j]);
                    }

                    /*------------------------------------------------------------------*
                     * comparing bit expenses of coding mode 1 with that of coding mode 0
                     *------------------------------------------------------------------*/

                    if( GT_16(hcode_l, bits))
                    {
                        *LCmode = 1;
                        move16();
                    }
                    hcode_l = s_min(hcode_l, bits);

                }
            }

            /* LR-MDCT core doesn't have coding mode 2 and 3 */
            IF( flag_HQ2 == NORMAL_HQ_CORE )
            {
                /*------------------------------------------------------------------------------*
                 * comparing bit expenses of coding mode 3 with that of the optimal coding mode
                 *------------------------------------------------------------------------------*/

                if( GE_16(hcode_l, numnrmibits))
                {
                    *LCmode = 3;
                    move16();
                }
                hcode_l = s_min(hcode_l, numnrmibits);
            }

            test();
            test();
            IF( (NE_16(*LCmode, 1)&&flag_HQ2==NORMAL_HQ_CORE)||EQ_16(flag_HQ2,LOW_RATE_HQ_CORE))
            {
                FOR(i = 2; i< num_sfm; i++)
                {
                    difidx[i] = difidx_org[i];
                    move16();
                }
            }
        }
    }
    ELSE
    {
        test();
        IF( EQ_16(flag_HQ2, LOW_RATE_HQ_CORE_TRAN)||EQ_16(flag_HQ2,LOW_RATE_HQ_CORE))
        {
            push_indice_fx( st_fx, IND_HQ2_DENG_HMODE, *LCmode, BITS_DE_HMODE);
            push_indice_fx( st_fx, IND_HQ2_DIFF_ENERGY, difidx[0], BITS_DE_FCOMP);
        }
        ELSE
        {
            push_indice_fx( st_fx, IND_LC_MODE, *LCmode, 2 );
            push_indice_fx( st_fx, IND_YNRM, difidx[0], NORM0_BITS );
        }

        test();
        IF(is_transient && EQ_16(flag_HQ2, LOW_RATE_HQ_CORE_TRAN))
        {
            hcode_l = 0;
            move16();
            IF ( EQ_16(*LCmode, 1))
            {
                /* LC mode 0 Transient Huffman Coding   */
                FOR( i = 1; i < num_sfm; i++ )
                {
                    j = difidx[i];
                    move16();
                    m = huffnorm_tran[j];
                    move16();
                    r = huffsizn_tran[j];
                    move16();
                    v = 0;
                    move16();

                    /* Bit reverse */
                    FOR( k = 0; k < r; k++ )
                    {
                        v = lshl(v, 1);
                        v = s_or(v, s_and(m, 1));
                        m = lshr(m, 1);
                    }

                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, v, r);
                }
            }
            ELSE
            {
                /* LC mode 1 context based Coding   */
                prevj = add(difidx[0], OFFSET_NORM);
                FOR( i = 1; i < num_sfm; i++ )
                {
                    j = difidx[i];
                    move16();

                    IF( GT_16(prevj, HTH_NORM))
                    {
                        /* above */
                        r = huffsizn_n_fx[sub(31,j)];
                        move16();
                        m = huffnorm_n_fx[sub(31,j)];
                        move16();
                    }
                    ELSE
                    {
                        IF( LT_16(prevj, LTH_NORM))
                        {
                            /* less */
                            r = huffsizn_n_fx[j];
                            move16();
                            m = huffnorm_n_fx[j];
                            move16();
                        }
                        ELSE
                        {
                            /* equal */
                            r = huffsizn_e_fx[j];
                            move16();
                            m = huffnorm_e_fx[j];
                            move16();
                        }
                    }
                    push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, m, r);
                    prevj = j;
                    move16();
                }
            }
        }
        ELSE
        {
            hcode_l = 0;
            move16();
            IF ( *LCmode == 0 )
            {
                /* LC mode 3 -> LC mode 0 */
                prevj = add(difidx[0], OFFSET_NORM);
                FOR( i = 1; i < num_sfm; i++ )
                {
                    j = difidx[i];
                    move16();

                    IF( GT_16(prevj, HTH_NORM))
                    {
                        /* above */
                        r = huffsizn_n_fx[sub(31,j)];
                        move16();
                        m = huffnorm_n_fx[sub(31,j)];
                        move16();
                    }
                    ELSE
                    {
                        IF( LT_16(prevj, LTH_NORM))
                        {
                            /* less */
                            r = huffsizn_n_fx[j];
                            move16();
                            m = huffnorm_n_fx[j];
                            move16();
                        }
                        ELSE
                        {
                            /* equal */
                            r = huffsizn_e_fx[j];
                            move16();
                            m = huffnorm_e_fx[j];
                            move16();
                        }
                    }

                    IF( EQ_16(flag_HQ2, LOW_RATE_HQ_CORE))
                    {
                        push_indice_fx(st_fx, IND_HQ2_DIFF_ENERGY, m, r);
                    }
                    ELSE
                    {
                        push_indice_fx( st_fx, IND_YNRM, m, r );
                    }

                    prevj = j;
                    move16();
                }
            }
            ELSE IF( EQ_16(*LCmode, 1))
            {
                IF ( EQ_16(flag_HQ2, 1))
                {
                    index_max = 0;
                    move16();
                    index_min = 31;
                    move16();
                    FOR(i = 1; i< num_sfm; i++)
                    {
                        difidx_org[i] = difidx[i];
                        move16();
                    }

                    FOR(i = 2; i< num_sfm; i++)
                    {
                        IF(GT_16(difidx_org[i-1], 17))
                        {
                            difidx[i] = add(difidx_org[i], s_min(sub(difidx_org[i-1],17),3));
                            move16();
                            IF(GT_16(difidx[i], 31))
                            {
                                difidx_flag = 1;
                                move16();
                                BREAK;
                            }
                        }

                        IF(LT_16(difidx_org[i-1], 13))
                        {
                            difidx[i] = add(difidx_org[i], s_max(sub(difidx_org[i-1],13),-3));
                            move16();
                            IF(difidx[i] < 0)
                            {
                                difidx_flag = 1;
                                move16();
                                BREAK;
                            }
                        }
                    }

                    IF( difidx_flag == 0 )
                    {
                        FOR(i = 1; i< num_sfm; i++)
                        {
                            index_max = s_max(index_max, difidx[i]);
                            index_min = s_min(index_min, difidx[i]);
                        }

                        index_rad = s_max(sub(15, index_min),sub(index_max, 15));

                        IF(LE_16(index_rad, HUFF_THR))
                        {
                            FOR (i = 1; i < num_sfm; i++)
                            {
                                j = difidx[i];
                                move16();
                            }
                        }
                    }
                }

                /* LC mode 2 -> LC mode 1 */
                FOR( i = 1; i < num_sfm; i++ )
                {
                    j = difidx[i];
                    move16();

                    m = resize_huffnorm_fx[j];
                    move16();
                    r = resize_huffsizn[j];
                    move16();
                    v = 0;
                    move16();

                    /* Bit reverse */
                    FOR( k = 0; k < r; k++ )
                    {
                        v = lshl(v, 1);
                        v = s_or(v,s_and(m, 1));
                        m = lshr(m, 1);
                    }

                    IF ( flag_HQ2 == 0 )
                    {
                        push_indice_fx( st_fx, IND_YNRM, v, r );
                    }
                    ELSE
                    {
                        push_indice_fx( st_fx, IND_HQ2_DIFF_ENERGY, v, r);
                    }
                }
            }
            ELSE IF( EQ_16(*LCmode, 2))
            {
                /* LC mode 1 -> LC mode 2 */
                FOR( i = 1; i < num_sfm; i++ )
                {
                    j = difidx[i];
                    move16();

                    m = huffnorm_fx[j];
                    move16();
                    r = huffsizn[j];
                    move16();

                    push_indice_fx( st_fx, IND_YNRM, m, r );
                }
            }
            ELSE
            {
                FOR( i = 1; i < num_sfm; i++ )
                {
                    push_indice_fx( st_fx, IND_YNRM, difidx[i], NORMI_BITS );
                }
            }
        }
    }

    return hcode_l;
}

/*--------------------------------------------------------------------------*
 * diff_envelope_coding_fx()
 *
 * Differential envelope coding
 *--------------------------------------------------------------------------*/

void diff_envelope_coding_fx(
    const Word16 is_transient,       /* i  : transient indicator               Q0  */
    const Word16 num_env_bands,      /* i  : number of envelope bands to code  Q0  */
    const Word16 start_norm,         /* i  : start of envelope coding          Q0  */
    Word16 *ynrm,              /* i/o: quantization indices for norms    Q0  */
    Word16 *normqlg2,          /* i/o: quantized norms                   Q0  */
    Word16 *difidx             /* o  : differential code                 Q0  */
)
{
    Word16 i, tmp;
    Word16 idxbuf[NB_SFM];
    Word16 normbuf[NB_SFM];

    /* Differential coding for indices of quantized norms */
    IF( is_transient != 0 )
    {
        /* Reorder quantization indices and quantized norms */
        reordernorm_fx( ynrm, normqlg2, idxbuf, normbuf, num_env_bands );
        diffcod_fx( num_env_bands, idxbuf, &difidx[1] );
        difidx[0] = idxbuf[0];
        move16();
        recovernorm_fx( idxbuf, ynrm, normqlg2, num_env_bands );
    }
    ELSE
    {
        diffcod_fx( num_env_bands, &ynrm[start_norm], &difidx[1] );
        difidx[0] = ynrm[start_norm];
        move16();

        tmp = add(start_norm, num_env_bands);
        FOR( i = start_norm; i < tmp; i++ )
        {
            normqlg2[i] = dicnlg2[ynrm[i]];
            move16();
            move16();
        }
    }

    return;
}
