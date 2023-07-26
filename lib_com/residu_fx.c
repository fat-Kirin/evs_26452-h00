/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include "options.h"     /* Compilation switches                   */
#include "cnst_fx.h"       /* Common constants                       */
#include "rom_com_fx.h"       /* Function prototypes                    */
#include "prot_fx.h"       /* Function prototypes                    */
#include "stl.h"

/*--------------------------------------------------------------------*
 * Residu3_lc_fx:
 *
 * Compute the LP residual by filtering the input speech through A(z)
 * Output is in Qx
 *
 * Optimized Version: Use when Past[0..m-1] is 0 & a[0] is 1 (in Q12)
 *--------------------------------------------------------------------*/
void Residu3_lc_fx(
    const Word16 a[],   /* i  :   prediction coefficients                 Q12 */
    const Word16 m,     /* i  :   order of LP filter                      Q0  */
    const Word16 x[],   /* i  :   input signal (usually speech)           Qx  */
    Word16 y[],   /* o  :   output signal (usually residual)        Qx  */
    const Word16 lg,    /* i  :   vector size                             Q0  */
    const Word16 shift
)
{
    Word16 i, j;
    Word32 s;
    Word16 q;

    q = add( norm_s(a[0]), 1 );
    if (shift > 0)
        q = add(q, shift);
    *y++ = shl(x[0], shift);
    move16();

    FOR (i = 1; i < m; i++)
    {
        s = L_mult(x[i], a[0]);
        /* Stop at i to Avoid Mults with Zeros */
        FOR (j = 1; j <= i; j++)
        {
            s = L_mac(s, x[i-j], a[j]);
        }

        s = L_shl(s, q);
        *y++ = round_fx(s);
    }

    FOR (; i < lg; i++)
    {
        s = L_mult(x[i], a[0]);
        FOR (j = 1; j <= m; j++)
        {
            s = L_mac(s, x[i-j], a[j]);
        }

        s = L_shl(s, q);
        *y++ = round_fx(s);
    }
}

/*--------------------------------------------------------------------*
 * Residu3_10_fx:
 *
 * Compute the LP residual by filtering the input speech through A(z)
 * Output is in Qx
 *--------------------------------------------------------------------*/
void Residu3_10_fx(
    const Word16 a[],   /* i :  prediction coefficients                 Q12 */
    const Word16 x[],   /* i :  input signal (usually speech)           Qx  */
    /*      (note that values x[-10..-1] are needed)    */
    Word16 y[],   /* o :  output signal (usually residual)        Qx  */
    const Word16 lg,    /* i :  vector size                             Q0  */    const Word16 shift
)
{
    Word16 i,j;
    Word32 s;
	Word64 s64;
    Word16 q;
    q = add( norm_s(a[0]), 1 );
    if (shift != 0)
        q = add(q, shift);
    FOR (i = 0; i < lg; i++)
    {
        s64 = 0;
		FOR (j = 0; j <= 10; j++)
		{
			s64 = W_mac_16_16(s64, x[i-j], a[j]);
		}		
		s = W_shl_sat_l(s64, q);
        y[i] = round_fx(s);
    }
}
/*--------------------------------------------------------------------*
 * Residu3_fx:
 *
 * Compute the LP residual by filtering the input speech through A(z)
 * Output is in Qx
 *--------------------------------------------------------------------*/
void Residu3_fx(
    const Word16 a[],   /* i :  prediction coefficients                 Q12 */
    const Word16 x[],   /* i :  input signal (usually speech)           Qx  */
    /*      (note that values x[-M..-1] are needed)     */
    Word16 y[],   /* o :  output signal (usually residual)        Qx  */
    const Word16 lg,    /* i :  vector size                             Q0  */
    const Word16 shift
)
{
    Word16 i, j;
    Word64 s64;
	Word32 s32;
    Word16 q;
    q = add( norm_s(a[0]), 1 );
    if (shift != 0)
        q = add(q, shift);
    FOR (i = 0; i < lg; i++)
    {
        s64 = 0;
		FOR (j = 0; j <= 15; j++)
		{
			s64 = W_mac_16_16(s64, x[i-j], a[j]);
		}	
		s64 = W_mac_16_16(s64, x[i-16], a[16]);
        s32 = W_shl_sat_l(s64, q);		
        y[i] = round_fx( s32 );
    }
}
/*==========================================================================*/
/* FUNCTION      : 	void calc_residu()									    */
/*--------------------------------------------------------------------------*/
/* PURPOSE       : Compute the LP residual by filtering the input through	*/
/*													A(z) in all subframes	*/
/*--------------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :														*/
/* Word16 *speech          i  : weighted speech signal                  Qx  */
/* Word16 L_frame          i  : order of LP filter						Q0  */
/* Word16 *p_Aq            i  : quantized LP filter coefficients        Q12 */
/*--------------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :														*/
/* Word16 *res             o  : residual signal                        Qx+1 */
/*--------------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :													*/
/*--------------------------------------------------------------------------*/
/* RETURN ARGUMENTS :														*/
/*					 _ None													*/
/*--------------------------------------------------------------------------*/
/* CALLED FROM : 															*/
/*==========================================================================*/

void calc_residu_fx(
    Encoder_State_fx *st,          /* i/o: state structure                           */
    const Word16 *speech,      /* i  : weighted speech signal                    */
    Word16 *res,         /* o  : residual signal                           */
    const Word16 *p_Aq,        /* i  : quantized LP filter coefficients          */
    const Word16 vad_hover_flag,
    const Word16 vad_flag_dtx
)
{
    Word16 i_subfr;
    Word16 i;
    Word16 att;
    Word16 offset;

    FOR( i_subfr = 0; i_subfr < st->L_frame_fx; i_subfr += L_SUBFR )
    {
        /* calculate the residual signal */
        Residu3_fx( p_Aq, &speech[i_subfr], &res[i_subfr], L_SUBFR, 1 );

        /* next subframe */
        p_Aq += (M+1);
    }
    /* smoothing in case of CNG */
    test();
    IF( (st->Opt_DTX_ON_fx != 0 ) && (vad_hover_flag != 0) )   /* corresponds to line 504 in FLT acelp_core_enc.c */
    {
        st->burst_ho_cnt_fx = add(st->burst_ho_cnt_fx,1);
        st->burst_ho_cnt_fx = s_min(st->burst_ho_cnt_fx, HO_HIST_SIZE);
        IF( NE_16(st->bwidth_fx, NB))
        {
            offset = 5;
            test();
            if( EQ_16(st->bwidth_fx, WB)&&st->CNG_mode_fx>=0)
            {
                offset = st->CNG_mode_fx;
                move16();
            }

            att = CNG_burst_att_fx[offset][sub(st->burst_ho_cnt_fx,1)];  /*Q15*/

            FOR( i = 0; i < st->L_frame_fx; i++ )
            {
                res[i] = mult_r(res[i], att);
                move16();
            }
        }
    }
    ELSE
    {
        test();
        IF( (st->Opt_DTX_ON_fx != 0) && (vad_flag_dtx != 0) )
        {
            st->burst_ho_cnt_fx = 0;
            move16();
        }
    }
    return;
}

