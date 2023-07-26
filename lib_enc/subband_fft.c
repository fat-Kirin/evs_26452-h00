/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/


#include "basop_util.h"
#include "stl.h"
#include "vad_basop.h"
#include "prot_fx.h"
#include "rom_enc_fx.h"

#define RE(A) A.r
#define IM(A) A.i

typedef struct
{
    complex_16 work[32];
    complex_16 const* tab;
} cfft_info_16;

static  void ComplexMult_16(Word16 *y1, Word16 *y2, Word16 x1, Word16 x2, Word16 c1, Word16 c2)
{
    *y1 = add(mult(x1, c1) , mult(x2, c2));
    move16();
    *y2 = sub(mult(x2, c1) , mult(x1, c2));
    move16();
}

Word16 ffr_getSfWord32(Word32 *vector, /*!< Pointer to input vector */
                       Word16 len)           /*!< Length of input vector */
{
    Word32 maxVal;
    Word16 i;
    Word16 resu;


    maxVal = 0;
	move32();
    FOR(i=0; i<len; i++)
    {
        maxVal = L_max(maxVal,L_abs(vector[i]));
    }

    resu = 31;
    move16();
    if(maxVal)
        resu = norm_l(maxVal);


    return resu;
}

static
void cgetpreSfWord16( Word16 *vector, /*!< Pointer to input vector */
                      Word16 len,Word16 preshr,Word16* num)
{
    Word16 i;


    *num = sub(*num,preshr);
    move16();
    FOR (i=0; i<len; i++)
    {
        vector[i] = shr(vector[i],preshr);
        move16();
    }

}

static void passf4_1_16( const cmplx_s *cc,
						 cmplx_s *ch,
                         const cmplx_s *wa1,
                         const cmplx_s *wa2,
                         const cmplx_s *wa3)
{
    UWord16 i;


    for (i = 0; i < 4; i++)
    {
        cmplx_s c2, c3, c4, t1, t2, t3, t4;

        t2 = C_add(cc[i], cc[i+8]);
        t1 = C_sub(cc[i], cc[i+8]);
        t3 = C_add(cc[i+4], cc[i+12]);
        t4 = C_sub(cc[i+4], cc[i+12]);
        t4 = C_mul_j(t4);

        c2 = C_add(t1, t4);
        c4 = C_sub(t1, t4);
        ch[i] = C_add(t2, t3);
        c3 = C_sub(t2, t3);

        ch[i+4] = C_multr(c2, wa1[i]);
        ch[i+8] = C_multr(c3, wa2[i]);
        ch[i+12] = C_multr(c4, wa3[i]);
#if (WMOPS)
    multiCounter[currCounter].CL_move += 4;
#endif	
    }


}

static void passf4_2_16(const cmplx_s *cc,
                        cmplx_s *ch)
{

    Word16  k;


    for (k = 0; k < 4; k++)
    {
        cmplx_s t1, t2, t3, t4;

        t2 = C_add(cc[4*k], cc[4*k+2]);
        t1 = C_sub(cc[4*k], cc[4*k+2]);
        t3 = C_add(cc[4*k+3], cc[4*k+1]);
        t4 = C_sub(cc[4*k+1], cc[4*k+3]);
        t4 = C_mul_j(t4);


        ch[k] = C_add(t2,t3);
        ch[k+8] = C_sub(t2,t3);
        ch[k+4] = C_add(t1,t4);
        ch[k+12] = C_sub(t1,t4);
#if (WMOPS)
    multiCounter[currCounter].CL_move += 4;
#endif	
    }

}


static
void cfftf_16(Word16* scale, complex_16 *c, complex_16 *ch, const complex_16 *wa)
{

    cgetpreSfWord16((Word16*)c, 32,3,scale);
	passf4_1_16((const cmplx_s *)c, (cmplx_s *)ch, (const cmplx_s *)&wa[0], (const cmplx_s *)&wa[4], (const cmplx_s *)&wa[8]);
    cgetpreSfWord16((Word16*)ch, 32,2,scale);
	passf4_2_16((const cmplx_s*)ch, (cmplx_s*)c);

}

static
void fft16_fix_4_16(
    Word32 **Sr,
    Word32 **Si,
    Word32 Offset,
    Word16 i,
    cfft_info_16 cfft,
    Word16 in_specamp_Q,
    Word16 tmpQ,
    Word32 * spec_amp
)
{
    Word32 n;

    Word32  tmpr,tmpi,ptmpn,ptmp15_n,tmpspec;
    Word16  specamp_Q,tmpr_16,tmpi_16;
    Word16  resu,scalefactor1;
    complex_16 f_int2[16];
    Word16 *count2= &resu;
    Word16 Sr16, Si16;
    Word32 maxVal;


    maxVal = L_deposit_l(0);
    FOR (n=0; n<16; n++)
    {
        maxVal = L_max(maxVal,L_abs(Sr[Offset+n][i]));
        maxVal = L_max(maxVal,L_abs(Si[Offset+n][i]));
    }

    resu = 30;
    move16();
    IF ( maxVal )
    {
        resu = sub(norm_l(maxVal),1);
    }

    FOR (n = 0; n < 16; n++)
    {
        Sr16 = round_fx(L_shl(Sr[Offset+n][i],resu));
        Si16 = round_fx(L_shl(Si[Offset+n][i],resu));
        ComplexMult_16(&IM(f_int2[n]), &RE(f_int2[n]), Si16, Sr16, RE(M_in_fix16[n]), IM(M_in_fix16[n]));/*q+16*/
    }

    cfftf_16(count2, f_int2,cfft.work,  (const complex_16*)cfft.tab);
    cgetpreSfWord16((Word16*)f_int2, 32,1,count2);
    scalefactor1 = add(*count2,DATAFFT_Q);

    in_specamp_Q = add(in_specamp_Q,shl(scalefactor1,1));

    FOR (n = 0; n < 8; n++)
    {
        tmpi = L_mac(L_mult(IM(f_int2[n]),  M_Wr_fix16[n]) , RE(f_int2[n]), M_Wi_fix16[n]);
        tmpr = L_msu(L_mult(RE(f_int2[n]),  M_Wr_fix16[n]) , IM(f_int2[n]), M_Wi_fix16[n]);
        tmpi_16 = extract_h(tmpi);
        tmpr_16 = extract_h(tmpr);

        ptmpn = L_mac0(L_mult0(tmpi_16,tmpi_16), tmpr_16,tmpr_16);

        tmpi = L_mac(L_mult(IM(f_int2[15-n]),  M_Wr_fix16[15-n]) , RE(f_int2[15-n]), M_Wi_fix16[15-n]);
        tmpr = L_msu(L_mult(RE(f_int2[15-n]),  M_Wr_fix16[15-n]) , IM(f_int2[15-n]), M_Wi_fix16[15-n]);
        tmpi_16 = extract_h(tmpi);
        tmpr_16 = extract_h(tmpr);

        ptmp15_n = L_add((L_mult0(tmpi_16,tmpi_16)), (L_mult0(tmpr_16,tmpr_16)));
        tmpspec= L_add(ptmpn, ptmp15_n) ;

        tmpspec = fft_vad_Sqrt_l(tmpspec,in_specamp_Q,&specamp_Q);
        spec_amp[i*8+n]  = L_shr(tmpspec,  limitScale32(sub(specamp_Q,tmpQ)));
        move32();
    }

}

static
void fft16_fix_5_16(
    Word32 **Sr,
    Word32 **Si,
    Word32 Offset,
    Word16 i,
    cfft_info_16 cfft,
    Word16 in_specamp_Q,
    Word16 tmpQ,
    Word32 * spec_amp
)
{
    Word32 n;

    Word32  tmpr,tmpi,ptmpn,ptmp15_n,tmpspec;
    Word16  specamp_Q,tmpr_16,tmpi_16;

    Word16  resu,scalefactor1;
    complex_16 f_int2[16];
    Word16 *count2= &resu;
    Word16 Sr16, Si16;
    Word32 maxVal;


    maxVal = L_deposit_l(0);
    FOR (n=0; n<16; n++)
    {
        maxVal = L_max(maxVal,L_abs(Sr[Offset+n][i]));
        maxVal = L_max(maxVal,L_abs(Si[Offset+n][i]));
    }

    resu = 30;
    move16();
    IF ( maxVal )
    {
        resu = sub(norm_l(maxVal),1);
    }

    FOR (n = 0; n < 16; n++)
    {
        Sr16 = round_fx(L_shl(Sr[Offset+n][i],resu));
        Si16 = round_fx(L_shl(Si[Offset+n][i],resu));
        ComplexMult_16(&IM(f_int2[n]), &RE(f_int2[n]), Si16, Sr16, RE(M_in_fix16[n]), IM(M_in_fix16[n]));/*q+16*/
    }

    cfftf_16(count2, f_int2,cfft.work,  (const complex_16*)cfft.tab);
    cgetpreSfWord16((Word16*)f_int2, 32,1,count2);

    scalefactor1 = add(*count2,DATAFFT_Q);

    in_specamp_Q = add(in_specamp_Q,shl(scalefactor1,1));

    FOR (n = 0; n < 8; n++)
    {
        tmpi = L_mac(L_mult(IM(f_int2[n]),  M_Wr_fix16[n]), RE(f_int2[n]), M_Wi_fix16[n]);
        tmpr = L_msu(L_mult(RE(f_int2[n]),  M_Wr_fix16[n]) , IM(f_int2[n]), M_Wi_fix16[n]);
        tmpi_16 = extract_h(tmpi);
        tmpr_16 = extract_h(tmpr);

        ptmpn = L_add((L_mult0(tmpi_16,tmpi_16)) , (L_mult0(tmpr_16,tmpr_16)));

        tmpi = L_mac(L_mult(IM(f_int2[15-n]),  M_Wr_fix16[15-n]) , RE(f_int2[15-n]), M_Wi_fix16[15-n]);
        tmpr = L_msu(L_mult(RE(f_int2[15-n]),  M_Wr_fix16[15-n]) , IM(f_int2[15-n]), M_Wi_fix16[15-n]);
        tmpi_16 = extract_h(tmpi);
        tmpr_16 = extract_h(tmpr);

        ptmp15_n = L_mac0(L_mult0(tmpi_16,tmpi_16), tmpr_16,tmpr_16);
        tmpspec= L_add( ptmpn, ptmp15_n) ;

        tmpspec = fft_vad_Sqrt_l(tmpspec,in_specamp_Q,&specamp_Q);
        spec_amp[i*8+7-n]  =  L_shr(tmpspec,  limitScale32(sub(specamp_Q,tmpQ)));
        move32();
    }

}

void subband_FFT(
    Word32 ** Sr,     /*(i) real part of the CLDFB*/
    Word32 ** Si,     /*(i) imag part of the CLDFB*/
    Word32 *spec_amp, /*(o) spectral amplitude*/
    Word32	Offset,    /*(i) offset of the CLDFB*/
    Word16 *fftoQ     /*(o) the Scaling */
)
{
    Word16 i;
    Word16 tmpQ,in_specamp_Q;
    cfft_info_16 cfft;


    cfft.tab = wnk_table_16;
    in_specamp_Q = shl(sub(*fftoQ,DATAFFT_Q),1);
    tmpQ = add(*fftoQ,8);

    FOR(i=0; i<10; i=i+2)
    {
        fft16_fix_4_16(Sr,Si,Offset,i,cfft,in_specamp_Q,tmpQ,spec_amp);
    }

    FOR(i=1; i<10; i=i+2)
    {
        fft16_fix_5_16(Sr,Si,Offset,i,cfft,in_specamp_Q,tmpQ,spec_amp);
    }

}
