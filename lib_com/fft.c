/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <assert.h>
#include "prot_fx.h"
#include "basop_util.h"
#include "rom_basop_util.h"
#include "options.h"
#include "stl.h"

void fft16_with_cmplx_data( cmplx *pInp, Word16 bsacle);

/**
 * \brief Profiling / Precision results
 *
 *        Profiling / Precision of complex valued FFTs: BASOP_cfft()
 *
 *                       WOPS BASOP  Precision BASOP
 *        FFT5                   87     16.96
 *        FFT8                  108     17.04
 *        FFT10                 194     16.70
 *        FFT15                 354     16.97
 *        FFT16                 288     16.62
 *        FFT20                 368     16.06
 *        FFT30                 828     16.80
 *        FFT32                 752     15.45   (cplx mult mit 3 mult und 3 add)
 *        FFT32                 824     16.07   (cplx mult mit 4 mult und 2 add)
 *        FFT64  ( 8x 8)      3.129     15.16
 *        FFT80  (10x 8)      4.385     15.55
 *        FFT100 (20x 5)      6.518     15.65
 *        FFT120 (15x 8)      7.029     15.38
 *        FFT128 (16x 8)      6.777     15.28
 *        FFT160 (20x 8)      9.033     14.95
 *        FFT240 (30x 8)     14.961     15.49
 *        FFT256 (32x 8)     14.905     14.61   (cplx mult mit 3 mult und 3 add)
 *        FFT256 (32x 8)     15.265     15.04   (cplx mult mit 4 mult und 2 add)
 *        FFT320 (20x16)     21.517     15.21
 *
 *
 *        Profiling / Precision of real valued FFTs / iFFTs: BASOP_rfft()
 *
 *                       WOPS BASOP  Precision BASOP
 *        rFFT40                955     15.68
 *        rFFT64               1635     16.17
 *
 *        irFFT40              1116     15.36
 *        irFFT64              1759     15.18
 *
 */


#define  Mpy_32_xx    Mpy_32_16_1

#define FFTC(x)       WORD322WORD16((Word32)x)

#define C31   (FFTC(0x91261468))      /* FL2WORD32( -0.86602540) -sqrt(3)/2 */

#define C51   (FFTC(0x79bc3854))      /* FL2WORD32( 0.95105652)   */
#define C52   (FFTC(0x9d839db0))      /* FL2WORD32(-1.53884180/2) */
#define C53   (FFTC(0xd18053ce))      /* FL2WORD32(-0.36327126)   */
#define C54   (FFTC(0x478dde64))      /* FL2WORD32( 0.55901699)   */
#define C55   (FFTC(0xb0000001))      /* FL2WORD32(-1.25/2)       */

#define C81   (FFTC(0x5a82799a))      /* FL2WORD32( 7.071067811865475e-1) */
#define C82   (FFTC(0xa57d8666))      /* FL2WORD32(-7.071067811865475e-1) */

#define C161  (FFTC(0x5a82799a))      /* FL2WORD32( 7.071067811865475e-1)  INV_SQRT2    */
#define C162  (FFTC(0xa57d8666))      /* FL2WORD32(-7.071067811865475e-1) -INV_SQRT2    */

#define C163  (FFTC(0x7641af3d))      /* FL2WORD32( 9.238795325112867e-1)  COS_PI_DIV8  */
#define C164  (FFTC(0x89be50c3))      /* FL2WORD32(-9.238795325112867e-1) -COS_PI_DIV8  */

#define C165  (FFTC(0x30fbc54d))      /* FL2WORD32( 3.826834323650898e-1)  COS_3PI_DIV8 */
#define C166  (FFTC(0xcf043ab3))      /* FL2WORD32(-3.826834323650898e-1) -COS_3PI_DIV8 */


#define cplxMpy4_8_0(re,im,a,b,c,d)   re = L_shr(L_sub(Mpy_32_xx(a,c),Mpy_32_xx(b,d)),1); \
                                      im = L_shr(L_add(Mpy_32_xx(a,d),Mpy_32_xx(b,c)),1);

#define cplxMpy4_8_1(re,im,a,b)       re = L_shr(a,1); \
                                      im = L_shr(b,1);




/**
 * \brief    Function performs a complex 5-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR5 bits.
 *
 *           WOPS with 32x16 bit multiplications:  88 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft5_with_cmplx_data(cmplx *inp)
{
    cmplx x0,x1,x2,x3,x4;	
    cmplx y1,y2,y3,y4;
    cmplx t;

    x0   = CL_shr(inp[0],SCALEFACTOR5);
    x1   = CL_shr(inp[1],SCALEFACTOR5);
    x2   = CL_shr(inp[2],SCALEFACTOR5);
    x3   = CL_shr(inp[3],SCALEFACTOR5);
    x4   = CL_shr(inp[4],SCALEFACTOR5);

    y1   = CL_add(x1,x4);						
    y4   = CL_sub(x1,x4);                        
    y3   = CL_add(x2,x3);                        
    y2   = CL_sub(x2,x3);                        
    t    = CL_scale_t(CL_sub(y1,y3),C54);     
    y1   = CL_add(y1,y3);                        
    inp[0] = CL_add(x0,y1);                      

    /* Bit shift left because of the constant C55 which was scaled with the factor 0.5 because of the representation of
    the values as fracts */
    y1   = CL_add(inp[0],(CL_shl(CL_scale_t(y1,C55),1)));
    y3   = CL_sub(y1,t); 
    y1   = CL_add(y1,t);

    t    = CL_scale_t(CL_add(y4,y2),C51);
    /* Bit shift left because of the constant C55 which was scaled with the factor 0.5 because of the representation of
    the values as fracts */
    y4   = CL_add(t,CL_shl(CL_scale_t(y4, C52),1));
    y2   = CL_add(t,CL_scale_t(y2,C53));


    /* combination */
    inp[1] = CL_msu_j(y1,y2);	
    inp[4] = CL_mac_j(y1,y2);

    inp[2] = CL_mac_j(y3,y4);        
    inp[3] = CL_msu_j(y3,y4);
	
#if (WMOPS)
    multiCounter[currCounter].CL_move += 5;
#endif	

}

/**
 * \brief    Function performs a complex 8-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR8 bits.
 *
 *           WOPS with 32x16 bit multiplications: 108 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft8_with_cmplx_data(cmplx *inp)
{
    cmplx x0, x1, x2, x3, x4, x5, x6, x7;
    cmplx s0, s1, s2, s3, s4, s5, s6, s7;
    cmplx t0, t1, t2, t3, t4, t5, t6, t7;

    /* Pre-additions */
    x0 = CL_shr(inp[0], SCALEFACTOR8);
    x1 = CL_shr(inp[1], SCALEFACTOR8);
    x2 = CL_shr(inp[2], SCALEFACTOR8);
    x3 = CL_shr(inp[3], SCALEFACTOR8);
    x4 = CL_shr(inp[4], SCALEFACTOR8);
    x5 = CL_shr(inp[5], SCALEFACTOR8);
    x6 = CL_shr(inp[6], SCALEFACTOR8);
    x7 = CL_shr(inp[7], SCALEFACTOR8);

    /* loops are unrolled */
    {
        t0 = CL_add(x0,x4);
        t1 = CL_sub(x0,x4);

        t2 = CL_add(x1,x5);
        t3 = CL_sub(x1,x5);

        t4 = CL_add(x2,x6);
        t5 = CL_sub(x2,x6);

        t6 = CL_add(x3,x7);
        t7 = CL_sub(x3,x7);
    }

    /* Pre-additions and core multiplications */

    s0 = CL_add(t0, t4);
    s2 = CL_sub(t0, t4);

    s4 = CL_mac_j(t1, t5);
    s5 = CL_msu_j(t1, t5);

    s1 = CL_add(t2, t6);
    s3 = CL_sub(t2, t6);
    s3 = CL_mul_j(s3);

    t0 = CL_add(t3, t7);
    t1 = CL_sub(t3, t7);

    s6 = CL_scale_t(CL_msu_j(t1, t0), C81);
    s7 = CL_dscale_t(CL_swap_real_imag(CL_msu_j(t0, t1)), C81, C82);

    /* Post-additions */

    inp[0] = CL_add(s0, s1);
    inp[4] = CL_sub(s0, s1);

    inp[2] = CL_sub(s2, s3);
    inp[6] = CL_add(s2, s3);

    inp[3] = CL_add(s4, s7);
    inp[7] = CL_sub(s4, s7);

    inp[1] = CL_add(s5, s6);
    inp[5] = CL_sub(s5, s6);
#if (WMOPS)
    multiCounter[currCounter].CL_move += 8;
#endif	
	

}


/**
 * \brief    Function performs a complex 10-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR10 bits.
 *
 *           WOPS with 32x16 bit multiplications:  196 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */

static void fft10_with_cmplx_data(cmplx *inp_data)
{
    cmplx r1,r2,r3,r4;
    cmplx x0,x1,x2,x3,x4,t;        
    cmplx y[10];

    /* FOR i=0 */
    {
        x0  = CL_shr(inp_data[0],SCALEFACTOR10);
        x1  = CL_shr(inp_data[2],SCALEFACTOR10);
        x2  = CL_shr(inp_data[4],SCALEFACTOR10);
        x3  = CL_shr(inp_data[6],SCALEFACTOR10);
        x4  = CL_shr(inp_data[8],SCALEFACTOR10);

        r1  = CL_add(x3,x2);
        r4  = CL_sub(x3,x2);
        r3  = CL_add(x1,x4);
        r2  = CL_sub(x1,x4);
        t   = CL_scale_t(CL_sub(r1,r3),C54);
        r1  = CL_add(r1,r3);
        y[0] = CL_add(x0,r1);
        r1  = CL_add(y[0],(CL_shl(CL_scale_t(r1,C55),1)));
        r3  = CL_sub(r1,t);
        r1  = CL_add(r1,t);
        t   = CL_scale_t((CL_add(r4,r2)),C51);
        r4  = CL_add(t,CL_shl(CL_scale_t(r4, C52),1));
        r2  = CL_add(t,CL_scale_t(r2,C53));


        y[2] = CL_msu_j(r1,r2);
        y[8] = CL_mac_j(r1,r2);
        y[4] = CL_mac_j(r3,r4);
        y[6] = CL_msu_j(r3,r4);
    }
    /* FOR i=1 */
    {
        x0  = CL_shr(inp_data[5],SCALEFACTOR10);
        x1  = CL_shr(inp_data[1],SCALEFACTOR10);
        x2  = CL_shr(inp_data[3],SCALEFACTOR10);
        x3  = CL_shr(inp_data[7],SCALEFACTOR10);
        x4  = CL_shr(inp_data[9],SCALEFACTOR10);

        r1  = CL_add(x1,x4);
        r4  = CL_sub(x1,x4);
        r3  = CL_add(x3,x2);
        r2  = CL_sub(x3,x2);
        t   = CL_scale_t(CL_sub(r1,r3),C54);
        r1  = CL_add(r1,r3);
        y[1] = CL_add(x0,r1);
        r1  = CL_add(y[1],(CL_shl(CL_scale_t(r1,C55),1)));
        r3  = CL_sub(r1,t);
        r1  = CL_add(r1,t);
        t   = CL_scale_t((CL_add(r4,r2)),C51);
        r4  = CL_add(t,CL_shl(CL_scale_t(r4, C52),1));
        r2  = CL_add(t,CL_scale_t(r2,C53));


        y[3] = CL_msu_j(r1,r2);
        y[9] = CL_mac_j(r1,r2);
        y[5] = CL_mac_j(r3,r4);
        y[7] = CL_msu_j(r3,r4);
    }

    /* FOR i=0 */
    {
        inp_data[0] = CL_add(y[0],y[1]);
        inp_data[5] = CL_sub(y[0],y[1]);
    }
    /* FOR i=2 */
    {
        inp_data[2] = CL_add(y[2],y[3]);
        inp_data[7] = CL_sub(y[2],y[3]);
    }
    /* FOR i=4 */
    {
        inp_data[4] = CL_add(y[4],y[5]);
        inp_data[9] = CL_sub(y[4],y[5]);
    }
    /* FOR i=6 */
    {
        inp_data[6] = CL_add(y[6],y[7]);
        inp_data[1] = CL_sub(y[6],y[7]);
    }
    /* FOR i=8 */
    {
        inp_data[8] = CL_add(y[8],y[9]);
        inp_data[3] = CL_sub(y[8],y[9]);
    }
	
#if (WMOPS)
    multiCounter[currCounter].CL_move += 10;
#endif		

}


/**
 * \brief    Function performs a complex 15-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR15 bits.
 *
 *           WOPS with 32x16 bit multiplications:  354 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */

static void fft15_with_cmplx_data(cmplx *inp_data)
{
    cmplx  c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14; 
    cmplx  c_z0,c_z1, c_z2, c_z3, c_z4, c_z5, c_z6, c_z7, c_z8, c_z9, c_z10, c_z11, c_z12, c_z13, c_z14;
    cmplx  c_y1,c_y2,c_y3,c_y4;
    cmplx  c_t;

    c0 = CL_shr(inp_data[ 0],SCALEFACTOR15);    
    c1 = CL_shr(inp_data[ 3],SCALEFACTOR15);    
    c2 = CL_shr(inp_data[ 6],SCALEFACTOR15);    
    c3 = CL_shr(inp_data[ 9],SCALEFACTOR15);    
    c4 = CL_shr(inp_data[12],SCALEFACTOR15);
    c5 = CL_shr(inp_data[ 5],SCALEFACTOR15);    
    c6 = CL_shr(inp_data[ 8],SCALEFACTOR15);    
    c7 = CL_shr(inp_data[11],SCALEFACTOR15);    
    c8 = CL_shr(inp_data[14],SCALEFACTOR15);    
    c9 = CL_shr(inp_data[ 2],SCALEFACTOR15);
    c10 = CL_shr(inp_data[10],SCALEFACTOR15);    
    c11 = CL_shr(inp_data[13],SCALEFACTOR15);    
    c12 = CL_shr(inp_data[ 1],SCALEFACTOR15);    
    c13 = CL_shr(inp_data[ 4],SCALEFACTOR15);    
    c14 = CL_shr(inp_data[ 7],SCALEFACTOR15);

    /* 1. FFT5 stage */	
    c_y1  = CL_add(c1,c4);                                       
    c_y4  = CL_sub(c1,c4);	
    c_y3  = CL_add(c2,c3);
    c_y2  = CL_sub(c2,c3); 	
    c_t   = CL_scale_t(CL_sub(c_y1,c_y3),C54);                        		
    c_y1  = CL_add(c_y1,c_y3); 	
    c_z0  = CL_add(c0,c_y1);  
    c_y1  = CL_add(c_z0,(CL_shl(CL_scale_t(c_y1,C55),1)));   	
    c_y3  = CL_sub(c_y1,c_t); 
    c_y1  = CL_add(c_y1,c_t); 
    c_t   = CL_scale_t(CL_add(c_y4,c_y2),C51);   
    c_y4  = CL_add(c_t,CL_shl(CL_scale_t(c_y4,C52),1)); 		
    c_y2  = CL_add(c_t,CL_scale_t(c_y2,C53));         

    /* combination */
    c_z1 = CL_msu_j(c_y1,c_y2);   
    c_z2 = CL_mac_j(c_y3,c_y4);   
    c_z3 = CL_msu_j(c_y3,c_y4);   
    c_z4 = CL_mac_j(c_y1,c_y2);      


    /* 2. FFT5 stage */
    c_y1  = CL_add(c6,c9);                                       
    c_y4  = CL_sub(c6,c9);		
    c_y3  = CL_add(c7,c8);
    c_y2  = CL_sub(c7,c8); 		
    c_t   = CL_scale_t(CL_sub(c_y1,c_y3),C54);                        			
    c_y1  = CL_add(c_y1,c_y3); 		
    c_z5  = CL_add(c5,c_y1);  
    c_y1  = CL_add(c_z5,(CL_shl(CL_scale_t(c_y1,C55),1)));   	
    c_y3  = CL_sub(c_y1,c_t); 
    c_y1  = CL_add(c_y1,c_t); 
    c_t   = CL_scale_t(CL_add(c_y4,c_y2),C51);   
    c_y4  = CL_add(c_t,CL_shl(CL_scale_t(c_y4,C52),1)); 		
    c_y2  = CL_add(c_t,CL_scale_t(c_y2,C53));        
    /* combination */
    c_z6 = CL_msu_j(c_y1,c_y2);   
    c_z7 = CL_mac_j(c_y3,c_y4);   
    c_z8 = CL_msu_j(c_y3,c_y4);   
    c_z9 = CL_mac_j(c_y1,c_y2);    


    /* 3. FFT5 stage */

    c_y1  = CL_add(c11,c14);
    c_y4  = CL_sub(c11,c14);
    c_y3  = CL_add(c12,c13);
    c_y2  = CL_sub(c12,c13);
    c_t   = CL_scale_t(CL_sub(c_y1,c_y3),C54);
    c_y1  = CL_add(c_y1,c_y3);
    c_z10  = CL_add(c10,c_y1);
    c_y1  = CL_add(c_z10,(CL_shl(CL_scale_t(c_y1,C55),1)));
    c_y3  = CL_sub(c_y1,c_t);
    c_y1  = CL_add(c_y1,c_t); 
    c_t   = CL_scale_t(CL_add(c_y4,c_y2),C51);
    c_y4  = CL_add(c_t,CL_shl(CL_scale_t(c_y4,C52),1));
    c_y2  = CL_add(c_t,CL_scale_t(c_y2,C53));
    /* combination */
    c_z11 = CL_msu_j(c_y1,c_y2);
    c_z12 = CL_mac_j(c_y3,c_y4);
    c_z13 = CL_msu_j(c_y3,c_y4);
    c_z14 = CL_mac_j(c_y1,c_y2);



    /* 1. FFT3 stage */

    c_y1 = CL_add(c_z5,c_z10);
    c_y2 = CL_scale_t(CL_sub(c_z5,c_z10),C31);
    inp_data[0] = CL_add(c_z0,c_y1);
    c_y1 = CL_sub(c_z0,CL_shr(c_y1,1));
    inp_data[10] = CL_mac_j(c_y1,c_y2);
    inp_data[5]  = CL_msu_j(c_y1,c_y2);

    /* 2. FFT3 stage */
    c_y1 = CL_add(c_z6,c_z11);
    c_y2 = CL_scale_t(CL_sub(c_z6,c_z11),C31);
    inp_data[6] = CL_add(c_z1,c_y1);
    c_y1 = CL_sub(c_z1,CL_shr(c_y1,1));
    inp_data[1] = CL_mac_j(c_y1,c_y2);
    inp_data[11]  = CL_msu_j(c_y1,c_y2);

    /* 3. FFT3 stage */
    c_y1 = CL_add(c_z7,c_z12);		
    c_y2 = CL_scale_t(CL_sub(c_z7,c_z12),C31);		
    inp_data[12] = CL_add(c_z2,c_y1);	
    c_y1 = CL_sub(c_z2,CL_shr(c_y1,1));	
    inp_data[7] = CL_mac_j(c_y1,c_y2);    	
    inp_data[2]  = CL_msu_j(c_y1,c_y2);


    /* 4. FFT3 stage */
    c_y1 = CL_add(c_z8,c_z13);		
    c_y2 = CL_scale_t(CL_sub(c_z8,c_z13),C31);		
    inp_data[3] = CL_add(c_z3,c_y1);	
    c_y1 = CL_sub(c_z3,CL_shr(c_y1,1));	
    inp_data[13] = CL_mac_j(c_y1,c_y2);    	
    inp_data[8]  = CL_msu_j(c_y1,c_y2);


    /* 5. FFT3 stage */
    c_y1 = CL_add(c_z9,c_z14);
    c_y2 = CL_scale_t(CL_sub(c_z9,c_z14),C31);
    inp_data[9] = CL_add(c_z4,c_y1);
    c_y1 = CL_sub(c_z4,CL_shr(c_y1,1));
    inp_data[4] = CL_mac_j(c_y1,c_y2);
    inp_data[14]  = CL_msu_j(c_y1,c_y2);
	
#if (WMOPS)
    multiCounter[currCounter].CL_move += 15;
#endif			

}


/**
 * \brief    Function performs a complex 16-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR16 bits.
 *
 *           WOPS with 32x16 bit multiplications (scale on ):  288 cycles
 *           WOPS with 32x16 bit multiplications (scale off):  256 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
void fft16(Word32 *re, Word32 *im, Word16 s, Word16 bScale)
{
	int i;
    if ( s == 2 )
    {
        fft16_with_cmplx_data( (cmplx *) re,  bScale );
    }
    else
    {
        cmplx inp_data[16];
        FOR(i=0; i<16; i++)
	{
		inp_data[i] = CL_form(re[s*i], im[s*i]);
	}
	fft16_with_cmplx_data(inp_data, bScale );
	FOR(i=0; i<16; i++)
	{
		re[s*i] = CL_Extract_real(inp_data[i]);
		im[s*i] = CL_Extract_imag(inp_data[i]);
	}
    }
}

void fft16_with_cmplx_data(cmplx *input, Word16 bScale )
{
    cmplx x0, x1, x2, x3, temp;
    cmplx t0,t2,t4,t6,t7;
    cmplx y[16];

    IF (bScale)
    {
        {
            x0 = CL_shr(input[0],SCALEFACTOR16);
            x1 = CL_shr(input[4],SCALEFACTOR16);
            x2 = CL_shr(input[8],SCALEFACTOR16);
            x3 = CL_shr(input[12],SCALEFACTOR16);
            t0 = CL_add(x0,x2);
            t2 = CL_sub(x0,x2);
            t4 = CL_add(x1,x3);
            t6 = CL_sub(x1,x3);
            t6 = CL_mul_j(t6);
            y[0] = CL_add(t0,t4);
            y[1] = CL_sub(t2,t6);
            y[2] = CL_sub(t0,t4);
            y[3] = CL_add(t2,t6);


            x0 = CL_shr(input[1],SCALEFACTOR16);
            x1 = CL_shr(input[5],SCALEFACTOR16);
            x2 = CL_shr(input[9],SCALEFACTOR16);
            x3 = CL_shr(input[13],SCALEFACTOR16);
            t0 = CL_add(x0,x2);
            t2 = CL_sub(x0,x2);
            t4 = CL_add(x1,x3);
            t6 = CL_sub(x1,x3);
            t6 = CL_mul_j(t6);
            y[4] = CL_add(t0,t4);
            y[5] = CL_sub(t2,t6);
            y[6] = CL_sub(t0,t4);
            y[7] = CL_add(t2,t6);


            x0 = CL_shr(input[2],SCALEFACTOR16);
            x1 = CL_shr(input[6],SCALEFACTOR16);
            x2 = CL_shr(input[10],SCALEFACTOR16);
            x3 = CL_shr(input[14],SCALEFACTOR16);
            t0 = CL_add(x0,x2);
            t2 = CL_sub(x0,x2);
            t4 = CL_add(x1,x3);
            t6 = CL_sub(x1,x3);
            t6 = CL_mul_j(t6);
            y[8] = CL_add(t0,t4);
            y[9] = CL_sub(t2,t6);
            y[10] = CL_sub(t4,t0);
            y[10] = CL_mul_j(y[10]);
            y[11] = CL_add(t2,t6);


            x0 = CL_shr(input[3],SCALEFACTOR16);
            x1 = CL_shr(input[7],SCALEFACTOR16);
            x2 = CL_shr(input[11],SCALEFACTOR16);
            x3 = CL_shr(input[15],SCALEFACTOR16);
            t0 = CL_add(x0,x2);
            t2 = CL_sub(x0,x2);
            t4 = CL_add(x1,x3);
            t6 = CL_sub(x1,x3);
            t6 = CL_mul_j(t6);
            y[12] = CL_add(t0,t4);
            y[13] = CL_sub(t2,t6);
            y[14] = CL_sub(t0,t4);
            y[15] = CL_add(t2,t6);
        }
    }
    else
    {
        {
            t0 = CL_add(input[ 0],input[ 8]);
            t2 = CL_sub(input[ 0],input[ 8]);
            t4 = CL_add(input[ 4],input[12]);
            t7 = CL_sub(input[ 4],input[12]);

            y[0] = CL_add(t0,t4);
            y[1] = CL_msu_j(t2,t7);
            y[2] = CL_sub(t0,t4);
            y[3] = CL_mac_j(t2,t7);
        }
        /* i=1 */
        {
            t0 = CL_add(input[ 1],input[ 9]);
            t2 = CL_sub(input[ 1],input[ 9]);
            t4 = CL_add(input[ 5],input[13]);
            t7 = CL_sub(input[ 5],input[13]);

            y[4] = CL_add(t0,t4);
            y[5] = CL_msu_j(t2,t7);
            y[6] = CL_sub(t0,t4);
            y[7] = CL_mac_j(t2,t7);
        }
        /* i=2 */
        {
            t0 = CL_add(input[ 2],input[ 10]);
            t2 = CL_sub(input[ 2],input[ 10]);
            t4 = CL_add(input[ 6],input[14]);
            t7 = CL_sub(input[ 6],input[14]);

            y[8] = CL_add(t0,t4);
            y[9] = CL_msu_j(t2,t7);
            temp = CL_sub(t0,t4);
            y[10] = CL_negate(CL_mul_j(temp));
            y[11] = CL_mac_j(t2,t7);
        }
        /* i=3 */
        {
            t0 = CL_add(input[ 3],input[ 11]);
            t2 = CL_sub(input[ 3],input[ 11]);
            t4 = CL_add(input[ 7],input[15]);
            t7 = CL_sub(input[ 7],input[15]);

            y[12] = CL_add(t0,t4);
            y[13] = CL_msu_j(t2,t7);
            y[14] = CL_sub(t0,t4);
            y[15] = CL_mac_j(t2,t7);
        }

    }

    x0  = CL_scale_t(y[11],C162);
    y[11] = CL_mac_j(x0,x0);

    x0  = CL_scale_t(y[14],C162);
    y[14] = CL_mac_j(x0,x0);

    x0  = CL_scale_t(y[6],C161);
    y[6] = CL_msu_j(x0,x0);

    x0  = CL_scale_t(y[9],C161);
    y[9] = CL_msu_j(x0,x0);

    y[5] = CL_mac_j(CL_scale_t(y[5], C163), CL_scale_t(y[5], C166));
    y[7] = CL_mac_j(CL_scale_t(y[7], C165), CL_scale_t(y[7], C164));
    y[13] = CL_mac_j(CL_scale_t(y[13], C165), CL_scale_t(y[13], C164));
    y[15] = CL_mac_j(CL_scale_t(y[15], C164), CL_scale_t(y[15], C165));


    /* i=0 */
    {
        t0 = CL_add(y[ 0],y[ 8]);
        t2 = CL_sub(y[ 0],y[ 8]);
        t4 = CL_add(y[ 4],y[12]);
        t7 = CL_sub(y[ 4],y[12]);

        input[0] = CL_add(t0,t4);
        input[4] = CL_msu_j(t2,t7);
        input[8] = CL_sub(t0,t4);
        input[12] = CL_mac_j(t2,t7);
    }
    /* i=1 */
    {
        t0 = CL_add(y[ 1],y[ 9]);
        t2 = CL_sub(y[ 1],y[ 9]);
        t4 = CL_add(y[ 5],y[13]);
        t7 = CL_sub(y[ 5],y[13]);

        input[1] = CL_add(t0,t4);
        input[5] = CL_msu_j(t2,t7);
        input[9] = CL_sub(t0,t4);
        input[13] = CL_mac_j(t2,t7);
    }
    /* i=2 */
    {
        t0 = CL_add(y[ 2],y[ 10]);
        t2 = CL_sub(y[ 2],y[ 10]);
        t4 = CL_add(y[ 6],y[14]);
        t7 = CL_sub(y[ 6],y[14]);

        input[2] = CL_add(t0,t4);
        input[6] = CL_msu_j(t2,t7);
        input[10] = CL_sub(t0,t4);
        input[14] = CL_mac_j(t2,t7);
    }
    /* i=3 */
    {
        t0 = CL_add(y[ 3],y[ 11]);
        t2 = CL_sub(y[ 3],y[ 11]);
        t4 = CL_add(y[ 7],y[15]);
        t7 = CL_sub(y[ 7],y[15]);

        input[3] = CL_add(t0,t4);
        input[7] = CL_msu_j(t2,t7);
        input[11] = CL_sub(t0,t4);
        input[15] = CL_mac_j(t2,t7);
    }
#if (WMOPS)
    multiCounter[currCounter].CL_move += 16;
#endif			

}


/**
 * \brief    Function performs a complex 20-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR20 bits.
 *
 *           WOPS with 32x16 bit multiplications:  432 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */
static void fft20_with_cmplx_data(cmplx *inp_data)
{
    cmplx r1,r2,r3,r4;
    cmplx x0,x1,x2,x3,x4;
    cmplx t,t0,t1,t2,t3;
    cmplx y[20];
    cmplx *y0, *y1,*y2,*y3,*y4;

    y0 =  y;
    y1 = &y[4];
    y2 = &y[16];
    y3 = &y[8];
    y4 = &y[12];

    {
        x0  = CL_shr(inp_data[0],SCALEFACTOR20);
        x1  = CL_shr(inp_data[16],SCALEFACTOR20);
        x2  = CL_shr(inp_data[12],SCALEFACTOR20);
        x3  = CL_shr(inp_data[8],SCALEFACTOR20);
        x4  = CL_shr(inp_data[4],SCALEFACTOR20);

        r4  = CL_sub(x1,x4);
        r2  = CL_sub(x2,x3);
        r1  = CL_add(x1,x4);
        r3  = CL_add(x2,x3);
        t   = CL_scale_t(CL_sub(r1,r3),C54);
        r1  = CL_add(r1,r3);
        y0[0] = CL_add(x0,r1);
        r1  = CL_add(y0[0],(CL_shl(CL_scale_t(r1,C55),1)));
        r3  = CL_sub(r1,t);
        r1  = CL_add(r1,t);
        t   = CL_scale_t((CL_add(r4,r2)),C51);
        r4  = CL_add(t,CL_shl(CL_scale_t(r4, C52),1));
        r2  = CL_add(t,CL_scale_t(r2,C53));


        y1[0] = CL_msu_j(r1,r2);
        y2[0] = CL_mac_j(r1,r2);
        y3[0] = CL_mac_j(r3,r4);
        y4[0] = CL_msu_j(r3,r4);
    }
    {
        x0  = CL_shr(inp_data[5],SCALEFACTOR20);
        x1  = CL_shr(inp_data[1],SCALEFACTOR20);
        x2  = CL_shr(inp_data[17],SCALEFACTOR20);
        x3  = CL_shr(inp_data[13],SCALEFACTOR20);
        x4  = CL_shr(inp_data[9],SCALEFACTOR20);

        r4  = CL_sub(x1,x4);
        r2  = CL_sub(x2,x3);
        r1  = CL_add(x1,x4);
        r3  = CL_add(x2,x3);
        t   = CL_scale_t(CL_sub(r1,r3),C54);
        r1  = CL_add(r1,r3);
        y0[1] = CL_add(x0,r1);
        r1  = CL_add(y0[1],(CL_shl(CL_scale_t(r1,C55),1)));
        r3  = CL_sub(r1,t);
        r1  = CL_add(r1,t);
        t   = CL_scale_t((CL_add(r4,r2)),C51);
        r4  = CL_add(t,CL_shl(CL_scale_t(r4, C52),1));
        r2  = CL_add(t,CL_scale_t(r2,C53));


        y1[1] = CL_msu_j(r1,r2);
        y2[1] = CL_mac_j(r1,r2);
        y3[1] = CL_mac_j(r3,r4);
        y4[1] = CL_msu_j(r3,r4);
    }
    {
        x0  = CL_shr(inp_data[10],SCALEFACTOR20);
        x1  = CL_shr(inp_data[6],SCALEFACTOR20);
        x2  = CL_shr(inp_data[2],SCALEFACTOR20);
        x3  = CL_shr(inp_data[18],SCALEFACTOR20);
        x4  = CL_shr(inp_data[14],SCALEFACTOR20);

        r4  = CL_sub(x1,x4);
        r2  = CL_sub(x2,x3);
        r1  = CL_add(x1,x4);
        r3  = CL_add(x2,x3);
        t   = CL_scale_t(CL_sub(r1,r3),C54);
        r1  = CL_add(r1,r3);
        y0[2] = CL_add(x0,r1);
        r1  = CL_add(y0[2],(CL_shl(CL_scale_t(r1,C55),1)));
        r3  = CL_sub(r1,t);
        r1  = CL_add(r1,t);
        t   = CL_scale_t((CL_add(r4,r2)),C51);
        r4  = CL_add(t,CL_shl(CL_scale_t(r4, C52),1));
        r2  = CL_add(t,CL_scale_t(r2,C53));


        y1[2] = CL_msu_j(r1,r2);
        y2[2] = CL_mac_j(r1,r2);
        y3[2] = CL_mac_j(r3,r4);
        y4[2] = CL_msu_j(r3,r4);
    }
    {
        x0  = CL_shr(inp_data[15],SCALEFACTOR20);
        x1  = CL_shr(inp_data[11],SCALEFACTOR20);
        x2  = CL_shr(inp_data[7],SCALEFACTOR20);
        x3  = CL_shr(inp_data[3],SCALEFACTOR20);
        x4  = CL_shr(inp_data[19],SCALEFACTOR20);

        r4  = CL_sub(x1,x4);
        r2  = CL_sub(x2,x3);
        r1  = CL_add(x1,x4);
        r3  = CL_add(x2,x3);
        t   = CL_scale_t(CL_sub(r1,r3),C54);
        r1  = CL_add(r1,r3);
        y0[3] = CL_add(x0,r1);
        r1  = CL_add(y0[3],(CL_shl(CL_scale_t(r1,C55),1)));
        r3  = CL_sub(r1,t);
        r1  = CL_add(r1,t);
        t   = CL_scale_t((CL_add(r4,r2)),C51);
        r4  = CL_add(t,CL_shl(CL_scale_t(r4, C52),1));
        r2  = CL_add(t,CL_scale_t(r2,C53));


        y1[3] = CL_msu_j(r1,r2);
        y2[3] = CL_mac_j(r1,r2);
        y3[3] = CL_mac_j(r3,r4);
        y4[3] = CL_msu_j(r3,r4);
    }

    {
        cmplx * ptr_y = y;
        {
            cmplx Cy0, Cy1, Cy2, Cy3;

            Cy0 = *ptr_y++;
            Cy1 = *ptr_y++;
            Cy2 = *ptr_y++;
            Cy3 = *ptr_y++;

            /*  Pre-additions */
            t0 = CL_add(Cy0,Cy2);
            t1 = CL_sub(Cy0,Cy2);
            t2 = CL_add(Cy1,Cy3);
            t3 = CL_sub(Cy1,Cy3);


            inp_data[0] = CL_add(t0,t2);
            inp_data[5] = CL_msu_j(t1,t3);
            inp_data[10] = CL_sub(t0,t2);
            inp_data[15] = CL_mac_j(t1,t3);
        }

        {
            cmplx Cy0, Cy1, Cy2, Cy3;

            Cy0 = *ptr_y++;
            Cy1 = *ptr_y++;
            Cy2 = *ptr_y++;
            Cy3 = *ptr_y++;

            /*  Pre-additions */
            t0 = CL_add(Cy0,Cy2);
            t1 = CL_sub(Cy0,Cy2);
            t2 = CL_add(Cy1,Cy3);
            t3 = CL_sub(Cy1,Cy3);


            inp_data[4] = CL_add(t0,t2);
            inp_data[9] = CL_msu_j(t1,t3);
            inp_data[14] = CL_sub(t0,t2);
            inp_data[19] = CL_mac_j(t1,t3);
        }

        {
            cmplx Cy0, Cy1, Cy2, Cy3;

            Cy0 = *ptr_y++;
            Cy1 = *ptr_y++;
            Cy2 = *ptr_y++;
            Cy3 = *ptr_y++;

            /*  Pre-additions */
            t0 = CL_add(Cy0,Cy2);
            t1 = CL_sub(Cy0,Cy2);
            t2 = CL_add(Cy1,Cy3);
            t3 = CL_sub(Cy1,Cy3);


            inp_data[8] = CL_add(t0,t2);
            inp_data[13] = CL_msu_j(t1,t3);
            inp_data[18] = CL_sub(t0,t2);
            inp_data[3] = CL_mac_j(t1,t3);
        }

        {
            cmplx Cy0, Cy1, Cy2, Cy3;

            Cy0 = *ptr_y++;
            Cy1 = *ptr_y++;
            Cy2 = *ptr_y++;
            Cy3 = *ptr_y++;

            /*  Pre-additions */
            t0 = CL_add(Cy0,Cy2);
            t1 = CL_sub(Cy0,Cy2);
            t2 = CL_add(Cy1,Cy3);
            t3 = CL_sub(Cy1,Cy3);

            inp_data[12] = CL_add(t0,t2);
            inp_data[17] = CL_msu_j(t1,t3);
            inp_data[2] = CL_sub(t0,t2);
            inp_data[7] = CL_mac_j(t1,t3);
        }

        {
            cmplx Cy0, Cy1, Cy2, Cy3;

            Cy0 = *ptr_y++;
            Cy1 = *ptr_y++;
            Cy2 = *ptr_y++;
            Cy3 = *ptr_y++;

            /*  Pre-additions */
            t0 = CL_add(Cy0,Cy2);
            t1 = CL_sub(Cy0,Cy2);
            t2 = CL_add(Cy1,Cy3);
            t3 = CL_sub(Cy1,Cy3);


            inp_data[16] = CL_add(t0,t2);
            inp_data[1] = CL_msu_j(t1,t3);
            inp_data[6] = CL_sub(t0,t2);
            inp_data[11] = CL_mac_j(t1,t3);
        }
    }
#if (WMOPS)
    multiCounter[currCounter].CL_move += 20;
#endif			
	
}


/**
 * \brief    Function performs a complex 30-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR30 bits.
 *
 *           WOPS with 32x16 bit multiplications:  828 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */

static void fft30_with_cmplx_data(cmplx * inp)
{
    cmplx *l  = &inp[0];
    cmplx *h  = &inp[15];

    cmplx z[30], y[15], x[15], rs1, rs2, rs3, rs4, t;

    /* 1. FFT15 stage */

    x[0]   = CL_shr(inp[0],SCALEFACTOR30_1);
    x[1]   = CL_shr(inp[18],SCALEFACTOR30_1);
    x[2]   = CL_shr(inp[6],SCALEFACTOR30_1);
    x[3]   = CL_shr(inp[24],SCALEFACTOR30_1);
    x[4]   = CL_shr(inp[12],SCALEFACTOR30_1);

    x[5]   = CL_shr(inp[20],SCALEFACTOR30_1);
    x[6]   = CL_shr(inp[8],SCALEFACTOR30_1);
    x[7]   = CL_shr(inp[26],SCALEFACTOR30_1);
    x[8]   = CL_shr(inp[14],SCALEFACTOR30_1);
    x[9]   = CL_shr(inp[2],SCALEFACTOR30_1);

    x[10]   = CL_shr(inp[10],SCALEFACTOR30_1);
    x[11]   = CL_shr(inp[28],SCALEFACTOR30_1);
    x[12]   = CL_shr(inp[16],SCALEFACTOR30_1);
    x[13]   = CL_shr(inp[4],SCALEFACTOR30_1);
    x[14]   = CL_shr(inp[22],SCALEFACTOR30_1);


    /* 1. FFT5 stage */
    rs1  = CL_add(x[1],x[4]);
    rs4  = CL_sub(x[1],x[4]);
    rs3  = CL_add(x[2],x[3]);
    rs2  = CL_sub(x[2],x[3]);
    t    = CL_scale_t(CL_sub(rs1,rs3),C54);
    rs1  = CL_add(rs1,rs3);
    y[0] = CL_add(x[0],rs1);
    rs1  = CL_add(y[0],(CL_shl(CL_scale_t(rs1,C55),1)));
    rs3  = CL_sub(rs1,t);
    rs1  = CL_add(rs1,t);
    t    = CL_scale_t(CL_add(rs4,rs2),C51);
    rs4  = CL_add(t,CL_shl(CL_scale_t(rs4, C52),1));
    rs2  = CL_add(t,CL_scale_t(rs2,C53));

    /* combination */
    y[1] = CL_msu_j(rs1,rs2);
    y[4] = CL_mac_j(rs1,rs2);
    y[2] = CL_mac_j(rs3,rs4);
    y[3] = CL_msu_j(rs3,rs4);


    /* 2. FFT5 stage */
    rs1  = CL_add(x[6],x[9]);
    rs4  = CL_sub(x[6],x[9]);
    rs3  = CL_add(x[7],x[8]);
    rs2  = CL_sub(x[7],x[8]);
    t    = CL_scale_t(CL_sub(rs1,rs3),C54);
    rs1  = CL_add(rs1,rs3);
    y[5] = CL_add(x[5],rs1);
    rs1  = CL_add(y[5],(CL_shl(CL_scale_t(rs1,C55),1)));
    rs3  = CL_sub(rs1,t);
    rs1  = CL_add(rs1,t);
    t    = CL_scale_t(CL_add(rs4,rs2),C51);
    rs4  = CL_add(t,CL_shl(CL_scale_t(rs4, C52),1));
    rs2  = CL_add(t,CL_scale_t(rs2,C53));

    /* combination */
    y[6] = CL_msu_j(rs1,rs2);
    y[9] = CL_mac_j(rs1,rs2);
    y[7] = CL_mac_j(rs3,rs4);
    y[8] = CL_msu_j(rs3,rs4);


    /* 3. FFT5 stage */
    rs1  = CL_add(x[11],x[14]);
    rs4  = CL_sub(x[11],x[14]);
    rs3  = CL_add(x[12],x[13]);
    rs2  = CL_sub(x[12],x[13]);
    t    = CL_scale_t(CL_sub(rs1,rs3),C54);
    rs1  = CL_add(rs1,rs3);
    y[10] = CL_add(x[10],rs1);
    rs1  = CL_add(y[10],(CL_shl(CL_scale_t(rs1,C55),1)));
    rs3  = CL_sub(rs1,t);
    rs1  = CL_add(rs1,t);
    t    = CL_scale_t(CL_add(rs4,rs2),C51);
    rs4  = CL_add(t,CL_shl(CL_scale_t(rs4, C52),1));
    rs2  = CL_add(t,CL_scale_t(rs2,C53));

    /* combination */
    y[11] = CL_msu_j(rs1,rs2);
    y[14] = CL_mac_j(rs1,rs2);
    y[12] = CL_mac_j(rs3,rs4);
    y[13] = CL_msu_j(rs3,rs4);
    /*for (i=10; i<15; i++)
    {
    printf("%d,\t %d,\t",y[i].re, y[i].im);
    }
    printf("\n\n");*/


    /* 1. FFT3 stage */
    /* real part */
    rs1  = CL_add(y[5],y[10]);
    rs2  = CL_scale_t(CL_sub(y[5],y[10]),C31);
    z[0] = CL_add(y[0],rs1);
    rs1   = CL_sub(y[0],CL_shr(rs1,1));

    z[10] = CL_mac_j(rs1,rs2);
    z[5]  = CL_msu_j(rs1,rs2);

    /* 2. FFT3 stage */
    rs1  = CL_add(y[6],y[11]);
    rs2  = CL_scale_t(CL_sub(y[6],y[11]),C31);
    z[6] = CL_add(y[1],rs1);
    rs1   = CL_sub(y[1],CL_shr(rs1,1));

    z[1] = CL_mac_j(rs1,rs2);
    z[11]  = CL_msu_j(rs1,rs2);


    /* 3. FFT3 stage */
    rs1  = CL_add(y[7],y[12]);
    rs2  = CL_scale_t(CL_sub(y[7],y[12]),C31);
    z[12] = CL_add(y[2],rs1);
    rs1   = CL_sub(y[2],CL_shr(rs1,1));

    z[7] = CL_mac_j(rs1,rs2);
    z[2]  = CL_msu_j(rs1,rs2);


    /* 4. FFT3 stage */
    rs1  = CL_add(y[8],y[13]);
    rs2  = CL_scale_t(CL_sub(y[8],y[13]),C31);
    z[3] = CL_add(y[3],rs1);
    rs1   = CL_sub(y[3],CL_shr(rs1,1));

    z[13] = CL_mac_j(rs1,rs2);
    z[8]  = CL_msu_j(rs1,rs2);


    /* 5. FFT3 stage */
    rs1  = CL_add(y[9],y[14]);
    rs2  = CL_scale_t(CL_sub(y[9],y[14]),C31);
    z[9] = CL_add(y[4],rs1);
    rs1   = CL_sub(y[4],CL_shr(rs1,1));

    z[4] = CL_mac_j(rs1,rs2);
    z[14]  = CL_msu_j(rs1,rs2);

    /*for (i=0; i<15; i++)
    printf("%d,\t %d,\t",z[i].re, z[i].im);
    printf("\n\n");*/


    /* 2. FFT15 stage */

    x[0]   = CL_shr(inp[15],SCALEFACTOR30_1);
    x[1]   = CL_shr(inp[3],SCALEFACTOR30_1);
    x[2]   = CL_shr(inp[21],SCALEFACTOR30_1);
    x[3]   = CL_shr(inp[9],SCALEFACTOR30_1);
    x[4]   = CL_shr(inp[27],SCALEFACTOR30_1);

    x[5]   = CL_shr(inp[5],SCALEFACTOR30_1);
    x[6]   = CL_shr(inp[23],SCALEFACTOR30_1);
    x[7]   = CL_shr(inp[11],SCALEFACTOR30_1);
    x[8]   = CL_shr(inp[29],SCALEFACTOR30_1);
    x[9]   = CL_shr(inp[17],SCALEFACTOR30_1);

    x[10]   = CL_shr(inp[25],SCALEFACTOR30_1);
    x[11]   = CL_shr(inp[13],SCALEFACTOR30_1);
    x[12]   = CL_shr(inp[1],SCALEFACTOR30_1);
    x[13]   = CL_shr(inp[19],SCALEFACTOR30_1);
    x[14]   = CL_shr(inp[7],SCALEFACTOR30_1);

    /* 1. FFT5 stage */
    rs1  = CL_add(x[1],x[4]);
    rs4  = CL_sub(x[1],x[4]);
    rs3  = CL_add(x[2],x[3]);
    rs2  = CL_sub(x[2],x[3]);
    t    = CL_scale_t(CL_sub(rs1,rs3),C54);
    rs1  = CL_add(rs1,rs3);
    y[0] = CL_add(x[0],rs1);
    rs1  = CL_add(y[0],(CL_shl(CL_scale_t(rs1,C55),1)));
    rs3  = CL_sub(rs1,t);
    rs1  = CL_add(rs1,t);
    t    = CL_scale_t(CL_add(rs4,rs2),C51);
    rs4  = CL_add(t,CL_shl(CL_scale_t(rs4, C52),1));
    rs2  = CL_add(t,CL_scale_t(rs2,C53));

    /* combination */
    y[1] = CL_msu_j(rs1,rs2);
    y[4] = CL_mac_j(rs1,rs2);
    y[2] = CL_mac_j(rs3,rs4);
    y[3] = CL_msu_j(rs3,rs4);


    /* 2. FFT5 stage */
    rs1  = CL_add(x[6],x[9]);
    rs4  = CL_sub(x[6],x[9]);
    rs3  = CL_add(x[7],x[8]);
    rs2  = CL_sub(x[7],x[8]);
    t    = CL_scale_t(CL_sub(rs1,rs3),C54);
    rs1  = CL_add(rs1,rs3);
    y[5] = CL_add(x[5],rs1);
    rs1  = CL_add(y[5],(CL_shl(CL_scale_t(rs1,C55),1)));
    rs3  = CL_sub(rs1,t);
    rs1  = CL_add(rs1,t);
    t    = CL_scale_t(CL_add(rs4,rs2),C51);
    rs4  = CL_add(t,CL_shl(CL_scale_t(rs4, C52),1));
    rs2  = CL_add(t,CL_scale_t(rs2,C53));

    /* combination */
    y[6] = CL_msu_j(rs1,rs2);
    y[9] = CL_mac_j(rs1,rs2);
    y[7] = CL_mac_j(rs3,rs4);
    y[8] = CL_msu_j(rs3,rs4);


    /* 3. FFT5 stage */
    rs1  = CL_add(x[11],x[14]);
    rs4  = CL_sub(x[11],x[14]);
    rs3  = CL_add(x[12],x[13]);
    rs2  = CL_sub(x[12],x[13]);
    t    = CL_scale_t(CL_sub(rs1,rs3),C54);
    rs1  = CL_add(rs1,rs3);
    y[10] = CL_add(x[10],rs1);
    rs1  = CL_add(y[10],(CL_shl(CL_scale_t(rs1,C55),1)));
    rs3  = CL_sub(rs1,t);
    rs1  = CL_add(rs1,t);
    t    = CL_scale_t(CL_add(rs4,rs2),C51);
    rs4  = CL_add(t,CL_shl(CL_scale_t(rs4, C52),1));
    rs2  = CL_add(t,CL_scale_t(rs2,C53));

    /* combination */
    y[11] = CL_msu_j(rs1,rs2);
    y[14] = CL_mac_j(rs1,rs2);
    y[12] = CL_mac_j(rs3,rs4);
    y[13] = CL_msu_j(rs3,rs4);
    /*for (i=10; i<15; i++)
    {
    printf("%d,\t %d,\t",y[i].re, y[i].im);
    }
    printf("\n\n");*/


    /* 1. FFT3 stage */
    /* real part */
    rs1  = CL_add(y[5],y[10]);
    rs2  = CL_scale_t(CL_sub(y[5],y[10]),C31);
    z[15] = CL_add(y[0],rs1);
    rs1   = CL_sub(y[0],CL_shr(rs1,1));

    z[25] = CL_mac_j(rs1,rs2);
    z[20]  = CL_msu_j(rs1,rs2);

    /* 2. FFT3 stage */
    rs1  = CL_add(y[6],y[11]);
    rs2  = CL_scale_t(CL_sub(y[6],y[11]),C31);
    z[21] = CL_add(y[1],rs1);
    rs1   = CL_sub(y[1],CL_shr(rs1,1));

    z[16] = CL_mac_j(rs1,rs2);
    z[26]  = CL_msu_j(rs1,rs2);


    /* 3. FFT3 stage */
    rs1  = CL_add(y[7],y[12]);
    rs2  = CL_scale_t(CL_sub(y[7],y[12]),C31);
    z[27] = CL_add(y[2],rs1);
    rs1   = CL_sub(y[2],CL_shr(rs1,1));

    z[22] = CL_mac_j(rs1,rs2);
    z[17]  = CL_msu_j(rs1,rs2);


    /* 4. FFT3 stage */
    rs1  = CL_add(y[8],y[13]);
    rs2  = CL_scale_t(CL_sub(y[8],y[13]),C31);
    z[18] = CL_add(y[3],rs1);
    rs1   = CL_sub(y[3],CL_shr(rs1,1));

    z[28] = CL_mac_j(rs1,rs2);
    z[23]  = CL_msu_j(rs1,rs2);


    /* 5. FFT3 stage */
    rs1  = CL_add(y[9],y[14]);
    rs2  = CL_scale_t(CL_sub(y[9],y[14]),C31);
    z[24] = CL_add(y[4],rs1);
    rs1   = CL_sub(y[4],CL_shr(rs1,1));

    z[19] = CL_mac_j(rs1,rs2);
    z[29]  = CL_msu_j(rs1,rs2);

    /*for (i=0; i<30; i++)
    printf("%d,\t %d,\t",z[i].re, z[i].im);
    printf("\n\n");*/


    /* 1. FFT2 stage */
    rs1 = CL_shr(z[0], SCALEFACTOR30_2);
    rs2 = CL_shr(z[15],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 2. FFT2 stage */
    rs1 = CL_shr(z[8], SCALEFACTOR30_2);
    rs2 = CL_shr(z[23],SCALEFACTOR30_2);
    *h = CL_add(rs1,rs2);
    *l = CL_sub(rs1,rs2);
    l+=1; h+=1;


    /* 3. FFT2 stage */
    rs1 = CL_shr(z[1], SCALEFACTOR30_2);
    rs2 = CL_shr(z[16],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;


    /* 4. FFT2 stage */
    rs1 = CL_shr(z[9], SCALEFACTOR30_2);
    rs2 = CL_shr(z[24],SCALEFACTOR30_2);
    *h = CL_add(rs1,rs2);
    *l = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 5. FFT2 stage */
    rs1 = CL_shr(z[2], SCALEFACTOR30_2);
    rs2 = CL_shr(z[17],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 6. FFT2 stage */
    rs1 = CL_shr(z[10], SCALEFACTOR30_2);
    rs2 = CL_shr(z[25],SCALEFACTOR30_2);
    *h = CL_add(rs1,rs2);
    *l = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 7. FFT2 stage */
    rs1 = CL_shr(z[3], SCALEFACTOR30_2);
    rs2 = CL_shr(z[18],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 8. FFT2 stage */
    rs1 = CL_shr(z[11], SCALEFACTOR30_2);
    rs2 = CL_shr(z[26],SCALEFACTOR30_2);
    *h = CL_add(rs1,rs2);
    *l = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 9. FFT2 stage */
    rs1 = CL_shr(z[4], SCALEFACTOR30_2);
    rs2 = CL_shr(z[19],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 10. FFT2 stage */
    rs1 = CL_shr(z[12], SCALEFACTOR30_2);
    rs2 = CL_shr(z[27],SCALEFACTOR30_2);
    *h = CL_add(rs1,rs2);
    *l = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 11. FFT2 stage */
    rs1 = CL_shr(z[5], SCALEFACTOR30_2);
    rs2 = CL_shr(z[20],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 12. FFT2 stage */
    rs1 = CL_shr(z[13], SCALEFACTOR30_2);
    rs2 = CL_shr(z[28],SCALEFACTOR30_2);
    *h = CL_add(rs1,rs2);
    *l = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 13. FFT2 stage */
    rs1 = CL_shr(z[6], SCALEFACTOR30_2);
    rs2 = CL_shr(z[21],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 14. FFT2 stage */
    rs1 = CL_shr(z[14], SCALEFACTOR30_2);
    rs2 = CL_shr(z[29],SCALEFACTOR30_2);
    *h = CL_add(rs1,rs2);
    *l = CL_sub(rs1,rs2);
    l+=1; h+=1;

    /* 15. FFT2 stage */
    rs1 = CL_shr(z[7], SCALEFACTOR30_2);
    rs2 = CL_shr(z[22],SCALEFACTOR30_2);
    *l = CL_add(rs1,rs2);
    *h = CL_sub(rs1,rs2);
    l+=1; h+=1;
	
#if (WMOPS)
    multiCounter[currCounter].CL_move += 30;
#endif			

}

/**
 * \brief    Function performs a complex 32-point FFT
 *           The FFT is performed inplace. The result of the FFT
 *           is scaled by SCALEFACTOR32 bits.
 *
 *           WOPS with 32x16 bit multiplications:  752 cycles
 *
 * \param    [i/o] re    real input / output
 * \param    [i/o] im    imag input / output
 * \param    [i  ] s     stride real and imag input / output
 *
 * \return   void
 */


static void fft32_with_cmplx_data(cmplx * inp)
{
    cmplx x[32], y[32], t[32], s[32], temp, temp1;
	const cmplx_s *pRotVector_32 = ( const cmplx_s *)RotVector_32;
	
    /* 1. FFT8 stage */

    x[0] = CL_shr(inp[0], SCALEFACTOR32_1);
    x[1] = CL_shr(inp[4], SCALEFACTOR32_1);
    x[2] = CL_shr(inp[8], SCALEFACTOR32_1);
    x[3] = CL_shr(inp[12], SCALEFACTOR32_1);
    x[4] = CL_shr(inp[16], SCALEFACTOR32_1);
    x[5] = CL_shr(inp[20], SCALEFACTOR32_1);
    x[6] = CL_shr(inp[24], SCALEFACTOR32_1);
    x[7] = CL_shr(inp[28], SCALEFACTOR32_1);


    t[0] = CL_add(x[0],x[4]);
    t[1] = CL_sub(x[0],x[4]);
    t[2] = CL_add(x[1],x[5]);
    t[3] = CL_sub(x[1],x[5]);
    t[4] = CL_add(x[2],x[6]);
    t[5] = CL_sub(x[2],x[6]);
    t[6] = CL_add(x[3],x[7]);
    t[7] = CL_sub(x[3],x[7]);

    /* Pre-additions and core multiplications */

    s[0] = CL_add(t[0], t[4]);
    s[2] = CL_sub(t[0], t[4]);
    s[4] = CL_mac_j(t[1], t[5]);
    s[5] = CL_msu_j(t[1], t[5]);
    s[1] = CL_add(t[2], t[6]);
    s[3] = CL_sub(t[2], t[6]);
    s[3] = CL_mul_j(s[3]);

    temp = CL_add(t[3], t[7]);
    temp1 = CL_sub(t[3], t[7]);
    s[6] = CL_scale_t(CL_msu_j(temp1, temp), C81);
    s[7] = CL_dscale_t(CL_swap_real_imag( CL_msu_j(temp, temp1)), C81, C82);


    y[0] = CL_add(s[0],s[1]);
    y[4] = CL_sub(s[0],s[1]);
    y[2] = CL_sub(s[2],s[3]);
    y[6] = CL_add(s[2],s[3]);
    y[3] = CL_add(s[4],s[7]);
    y[7] = CL_sub(s[4],s[7]);
    y[1] = CL_add(s[5],s[6]);
    y[5] = CL_sub(s[5],s[6]);

    /* 2. FFT8 stage */

    x[0] = CL_shr(inp[1], SCALEFACTOR32_1);
    x[1] = CL_shr(inp[5], SCALEFACTOR32_1);
    x[2] = CL_shr(inp[9], SCALEFACTOR32_1);
    x[3] = CL_shr(inp[13], SCALEFACTOR32_1);
    x[4] = CL_shr(inp[17], SCALEFACTOR32_1);
    x[5] = CL_shr(inp[21], SCALEFACTOR32_1);
    x[6] = CL_shr(inp[25], SCALEFACTOR32_1);
    x[7] = CL_shr(inp[29], SCALEFACTOR32_1);


    t[0] = CL_add(x[0],x[4]);
    t[1] = CL_sub(x[0],x[4]);
    t[2] = CL_add(x[1],x[5]);
    t[3] = CL_sub(x[1],x[5]);
    t[4] = CL_add(x[2],x[6]);
    t[5] = CL_sub(x[2],x[6]);
    t[6] = CL_add(x[3],x[7]);
    t[7] = CL_sub(x[3],x[7]);

    /* Pre-additions and core multiplications */

    s[0] = CL_add(t[0], t[4]);
    s[2] = CL_sub(t[0], t[4]);
    s[4] = CL_mac_j(t[1], t[5]);
    s[5] = CL_msu_j(t[1], t[5]);
    s[1] = CL_add(t[2], t[6]);
    s[3] = CL_sub(t[2], t[6]);
    s[3] = CL_mul_j(s[3]);

    temp = CL_add(t[3], t[7]);
    temp1 = CL_sub(t[3], t[7]);
    s[6] = CL_scale_t(CL_msu_j(temp1, temp), C81);
    s[7] = CL_dscale_t(CL_swap_real_imag( CL_msu_j(temp, temp1)), C81, C82);

    /* Post-additions */

    y[8] = CL_add(s[0],s[1]);
    y[12] = CL_sub(s[0],s[1]);
    y[10] = CL_sub(s[2],s[3]);
    y[14] = CL_add(s[2],s[3]);
    y[11] = CL_add(s[4],s[7]);
    y[15] = CL_sub(s[4],s[7]);
    y[9] = CL_add(s[5],s[6]);
    y[13] = CL_sub(s[5],s[6]);

    /* 3. FFT8 stage */

    x[0] = CL_shr(inp[2], SCALEFACTOR32_1);
    x[1] = CL_shr(inp[6], SCALEFACTOR32_1);
    x[2] = CL_shr(inp[10], SCALEFACTOR32_1);
    x[3] = CL_shr(inp[14], SCALEFACTOR32_1);
    x[4] = CL_shr(inp[18], SCALEFACTOR32_1);
    x[5] = CL_shr(inp[22], SCALEFACTOR32_1);
    x[6] = CL_shr(inp[26], SCALEFACTOR32_1);
    x[7] = CL_shr(inp[30], SCALEFACTOR32_1);


    t[0] = CL_add(x[0],x[4]);
    t[1] = CL_sub(x[0],x[4]);
    t[2] = CL_add(x[1],x[5]);
    t[3] = CL_sub(x[1],x[5]);
    t[4] = CL_add(x[2],x[6]);
    t[5] = CL_sub(x[2],x[6]);
    t[6] = CL_add(x[3],x[7]);
    t[7] = CL_sub(x[3],x[7]);

    /* Pre-additions and core multiplications */

    s[0] = CL_add(t[0], t[4]);
    s[2] = CL_sub(t[0], t[4]);
    s[4] = CL_mac_j(t[1], t[5]);
    s[5] = CL_msu_j(t[1], t[5]);
    s[1] = CL_add(t[2], t[6]);
    s[3] = CL_sub(t[2], t[6]);
    s[3] = CL_mul_j(s[3]);

    temp = CL_add(t[3], t[7]);
    temp1 = CL_sub(t[3], t[7]);
    s[6] = CL_scale_t(CL_msu_j(temp1, temp), C81);
    s[7] = CL_dscale_t(CL_swap_real_imag( CL_msu_j(temp, temp1)), C81, C82);

    /* Post-additions */

    y[16] = CL_add(s[0],s[1]);
    y[20] = CL_sub(s[0],s[1]);
    y[18] = CL_sub(s[2],s[3]);
    y[22] = CL_add(s[2],s[3]);
    y[19] = CL_add(s[4],s[7]);
    y[23] = CL_sub(s[4],s[7]);
    y[17] = CL_add(s[5],s[6]);
    y[21] = CL_sub(s[5],s[6]);

    /* 4. FFT8 stage */

    x[0] = CL_shr(inp[3], SCALEFACTOR32_1);
    x[1] = CL_shr(inp[7], SCALEFACTOR32_1);
    x[2] = CL_shr(inp[11], SCALEFACTOR32_1);
    x[3] = CL_shr(inp[15], SCALEFACTOR32_1);
    x[4] = CL_shr(inp[19], SCALEFACTOR32_1);
    x[5] = CL_shr(inp[23], SCALEFACTOR32_1);
    x[6] = CL_shr(inp[27], SCALEFACTOR32_1);
    x[7] = CL_shr(inp[31], SCALEFACTOR32_1);


    t[0] = CL_add(x[0],x[4]);
    t[1] = CL_sub(x[0],x[4]);
    t[2] = CL_add(x[1],x[5]);
    t[3] = CL_sub(x[1],x[5]);
    t[4] = CL_add(x[2],x[6]);
    t[5] = CL_sub(x[2],x[6]);
    t[6] = CL_add(x[3],x[7]);
    t[7] = CL_sub(x[3],x[7]);


    /* Pre-additions and core multiplications */

    s[0] = CL_add(t[0], t[4]);
    s[2] = CL_sub(t[0], t[4]);
    s[4] = CL_mac_j(t[1], t[5]);
    s[5] = CL_msu_j(t[1], t[5]);
    s[1] = CL_add(t[2], t[6]);
    s[3] = CL_sub(t[2], t[6]);
    s[3] = CL_mul_j(s[3]);

    temp = CL_add(t[3], t[7]);
    temp1 = CL_sub(t[3], t[7]);
    s[6] = CL_scale_t(CL_msu_j(temp1, temp), C81);
    s[7] = CL_dscale_t(CL_swap_real_imag( CL_msu_j(temp, temp1)), C81, C82);

    /* Post-additions */

    y[24] = CL_add(s[0],s[1]);
    y[28] = CL_sub(s[0],s[1]);
    y[26] = CL_sub(s[2],s[3]);
    y[30] = CL_add(s[2],s[3]);
    y[27] = CL_add(s[4],s[7]);
    y[31] = CL_sub(s[4],s[7]);
    y[25] = CL_add(s[5],s[6]);
    y[29] = CL_sub(s[5],s[6]);


    /* apply twiddle factors */
    y[0] = CL_shr(y[0],SCALEFACTOR32_2);
    y[1] = CL_shr(y[1],SCALEFACTOR32_2);
    y[2] = CL_shr(y[2],SCALEFACTOR32_2);
    y[3] = CL_shr(y[3],SCALEFACTOR32_2);
    y[4] = CL_shr(y[4],SCALEFACTOR32_2);
    y[5] = CL_shr(y[5],SCALEFACTOR32_2);
    y[6] = CL_shr(y[6],SCALEFACTOR32_2);
    y[7] = CL_shr(y[7],SCALEFACTOR32_2);
    y[8] = CL_shr(y[8],SCALEFACTOR32_2);
    y[16] = CL_shr(y[16],SCALEFACTOR32_2);
    y[24] = CL_shr(y[24],SCALEFACTOR32_2);
    y[20] = CL_shr(y[20],SCALEFACTOR32_2);


    y[9]  = CL_mult_32x16((CL_shr(y[9],1)), pRotVector_32[ 0 ]);
    y[10] = CL_mult_32x16((CL_shr(y[10],1)), pRotVector_32[ 1 ]);
    y[11] = CL_mult_32x16((CL_shr(y[11],1)), pRotVector_32[ 2 ]);
    y[12] = CL_mult_32x16((CL_shr(y[12],1)), pRotVector_32[ 3 ]);
    y[13] = CL_mult_32x16((CL_shr(y[13],1)), pRotVector_32[ 4 ]);
    y[14] = CL_mult_32x16((CL_shr(y[14],1)), pRotVector_32[ 5 ]);
    y[15] = CL_mult_32x16((CL_shr(y[15],1)), pRotVector_32[ 6 ]);
    y[17] = CL_mult_32x16((CL_shr(y[17],1)), pRotVector_32[ 7 ]);
    y[18] = CL_mult_32x16((CL_shr(y[18],1)), pRotVector_32[ 8 ]);
    y[19] = CL_mult_32x16((CL_shr(y[19],1)), pRotVector_32[ 9 ]);
    y[21] = CL_mult_32x16((CL_shr(y[21],1)), pRotVector_32[ 10 ]);
    y[22] = CL_mult_32x16((CL_shr(y[22],1)), pRotVector_32[ 11 ]);
    y[23] = CL_mult_32x16((CL_shr(y[23],1)), pRotVector_32[ 12 ]);
    y[25] = CL_mult_32x16((CL_shr(y[25],1)), pRotVector_32[ 13 ]);
    y[26] = CL_mult_32x16((CL_shr(y[26],1)), pRotVector_32[ 14 ]);
    y[27] = CL_mult_32x16((CL_shr(y[27],1)), pRotVector_32[ 15 ]);
    y[28] = CL_mult_32x16((CL_shr(y[28],1)), pRotVector_32[ 16 ]);
    y[29] = CL_mult_32x16((CL_shr(y[29],1)), pRotVector_32[ 17 ]);
    y[30] = CL_mult_32x16((CL_shr(y[30],1)), pRotVector_32[ 18 ]);
    y[31] = CL_mult_32x16((CL_shr(y[31],1)), pRotVector_32[ 19 ]);

    /* 1. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_add(y[0],y[16]);
    t[1] = CL_sub(y[0],y[16]);
    t[2] = CL_add(y[8],y[24]);
    t[3] = CL_mul_j(CL_sub(y[8],y[24]));

    /*  Post-additions */
    inp[0] = CL_add(t[0], t[2]);
    inp[8] = CL_sub(t[1], t[3]);
    inp[16] = CL_sub(t[0], t[2]);
    inp[24] = CL_add(t[1], t[3]);

    /* 2. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_add(y[1],y[17]);
    t[1] = CL_sub(y[1],y[17]);
    t[2] = CL_add(y[9],y[25]);
    t[3] = CL_mul_j(CL_sub(y[9],y[25]));

    /*  Post-additions */
    inp[1] = CL_add(t[0], t[2]);
    inp[9] = CL_sub(t[1], t[3]);
    inp[17] = CL_sub(t[0], t[2]);
    inp[25] = CL_add(t[1], t[3]);


    /* 3. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_add(y[2],y[18]);
    t[1] = CL_sub(y[2],y[18]);
    t[2] = CL_add(y[10],y[26]);
    t[3] = CL_mul_j(CL_sub(y[10],y[26]));

    /*  Post-additions */
    inp[2] = CL_add(t[0], t[2]);
    inp[10] = CL_sub(t[1], t[3]);
    inp[18] = CL_sub(t[0], t[2]);
    inp[26] = CL_add(t[1], t[3]);


    /* 4. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_add(y[3],y[19]);
    t[1] = CL_sub(y[3],y[19]);
    t[2] = CL_add(y[11],y[27]);
    t[3] = CL_mul_j(CL_sub(y[11],y[27]));


    /*  Post-additions */
    inp[3] = CL_add(t[0], t[2]);
    inp[11] = CL_sub(t[1], t[3]);
    inp[19] = CL_sub(t[0], t[2]);
    inp[27] = CL_add(t[1], t[3]);


    /* 5. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_msu_j(y[4],y[20]);
    t[1] = CL_mac_j(y[4],y[20]);
    t[2] = CL_add(y[12],y[28]);
    t[3] = CL_mul_j(CL_sub(y[12],y[28]));


    /*  Post-additions */
    inp[4] = CL_add(t[0], t[2]);
    inp[12] = CL_sub(t[1], t[3]);
    inp[20] = CL_sub(t[0], t[2]);
    inp[28] = CL_add(t[1], t[3]);


    /* 6. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_add(y[5],y[21]);
    t[1] = CL_sub(y[5],y[21]);
    t[2] = CL_add(y[13],y[29]);
    t[3] = CL_mul_j(CL_sub(y[13],y[29]));


    /*  Post-additions */
    inp[5] = CL_add(t[0], t[2]);
    inp[13] = CL_sub(t[1], t[3]);
    inp[21] = CL_sub(t[0], t[2]);
    inp[29] = CL_add(t[1], t[3]);


    /* 7. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_add(y[6],y[22]);
    t[1] = CL_sub(y[6],y[22]);
    t[2] = CL_add(y[14],y[30]);
    t[3] = CL_mul_j(CL_sub(y[14],y[30]));


    /*  Post-additions */
    inp[6] = CL_add(t[0], t[2]);
    inp[14] = CL_sub(t[1], t[3]);
    inp[22] = CL_sub(t[0], t[2]);
    inp[30] = CL_add(t[1], t[3]);


    /* 8. FFT4 stage */

    /*  Pre-additions */
    t[0] = CL_add(y[7],y[23]);
    t[1] = CL_sub(y[7],y[23]);
    t[2] = CL_add(y[15],y[31]);
    t[3] = CL_mul_j(CL_sub(y[15],y[31]));


    /*  Post-additions */
    inp[7] = CL_add(t[0], t[2]);
    inp[15] = CL_sub(t[1], t[3]);
    inp[23] = CL_sub(t[0], t[2]);
    inp[31] = CL_add(t[1], t[3]);

#if (WMOPS)
    multiCounter[currCounter].CL_move += 32;
#endif			


}


/**
 * \brief Combined FFT
 *
 * \param    [i/o] re     real part
 * \param    [i/o] im     imag part
 * \param    [i  ] W      rotation factor
 * \param    [i  ] len    length of fft
 * \param    [i  ] dim1   length of fft1
 * \param    [i  ] dim2   length of fft2
 * \param    [i  ] sx     stride real and imag part
 * \param    [i  ] sc     stride phase rotation coefficients
 * \param    [tmp] x      32-bit workbuffer of length=2*len
 * \param    [i  ] Woff   offset for addressing the rotation vector table
 *
 * \return void
 */

static void fftN2(
	cmplx *__restrict pComplexBuf,
	const Word16 *__restrict W,
	Word16 len,
	Word16 dim1,
	Word16 dim2,
	Word16 sc,
	Word32 *x,
	Word16 Woff
)
{
	Word16 i, j;
	cmplx *x_cmplx = (cmplx *)x;

	assert(len == (dim1*dim2));
	assert((dim1 == 3) || (dim1 == 5) || (dim1 == 8) || (dim1 == 10) || (dim1 == 15) || (dim1 == 16) || (dim1 == 20) || (dim1 == 30) || (dim1 == 32));
	assert((dim2 == 4) || (dim2 == 8) || (dim2 == 10) || (dim2 == 12) || (dim2 == 16) || (dim2 == 20));

	FOR(i = 0; i<dim2; i++)
	{
		FOR(j = 0; j<dim1; j++)
		{
			x_cmplx[i*dim1 + j] = pComplexBuf[i + j * dim2];
#if (WMOPS)
			multiCounter[currCounter].CL_move++;
#endif				
		}
	}

	SWITCH(dim1)
	{
    case   5:
		FOR(i = 0; i<dim2; i++)
		{
			fft5_with_cmplx_data(&x_cmplx[i*dim1]);
		}
		BREAK;
	case   8:
		FOR(i = 0; i<dim2; i++)
		{
			fft8_with_cmplx_data(&x_cmplx[i*dim1]);
		}
		BREAK;
	case  10:
		FOR(i = 0; i<dim2; i++)
		{
			fft10_with_cmplx_data(&x_cmplx[i*dim1]);
		}
		BREAK;

	case  15:
		FOR(i = 0; i<dim2; i++)
		{
			fft15_with_cmplx_data(&x_cmplx[i*dim1]);
		}
		BREAK;
	case  16:
		FOR(i = 0; i<dim2; i++)
		{
			fft16_with_cmplx_data(&x_cmplx[i*dim1], 1);
		}
		BREAK;
	case  20:
		FOR(i = 0; i<dim2; i++)
		{
			fft20_with_cmplx_data(&x_cmplx[i*dim1]);
		}
		BREAK;
	case  30:
		FOR(i = 0; i<dim2; i++)
		{
			fft30_with_cmplx_data(&x_cmplx[i*dim1]);
		}
		BREAK;
	case  32:
		FOR(i = 0; i<dim2; i++)
		{
			fft32_with_cmplx_data(&x_cmplx[i*dim1]);
		}
		BREAK;
	}

	SWITCH(dim2)
	{
	case   8:
	{
		cmplx y0, y1, y2, y3, y4, y5, y6, y7;
		cmplx t0, t1, t2, t3, t4, t5, t6, t7;
		cmplx s0, s1, s2, s3, s4, s5, s6, s7;

		i = 0;
		{
			y0 = CL_shr(x_cmplx[i + 0 * dim1], 1);
			y1 = CL_shr(x_cmplx[i + 1 * dim1], 1);
			y2 = CL_shr(x_cmplx[i + 2 * dim1], 1);
			y3 = CL_shr(x_cmplx[i + 3 * dim1], 1);
			y4 = CL_shr(x_cmplx[i + 4 * dim1], 1);
			y5 = CL_shr(x_cmplx[i + 5 * dim1], 1);
			y6 = CL_shr(x_cmplx[i + 6 * dim1], 1);
			y7 = CL_shr(x_cmplx[i + 7 * dim1], 1);

			t0 = CL_shr(CL_add(y0, y4), SCALEFACTORN2 - 1);
			t1 = CL_shr(CL_sub(y0, y4), SCALEFACTORN2 - 1);
			t2 = CL_shr(CL_add(y1, y5), SCALEFACTORN2 - 1);
			t3 = CL_sub(y1, y5);
			t4 = CL_shr(CL_add(y2, y6), SCALEFACTORN2 - 1);
			t5 = CL_shr(CL_sub(y2, y6), SCALEFACTORN2 - 1);
			t6 = CL_shr(CL_add(y3, y7), SCALEFACTORN2 - 1);
			t7 = CL_sub(y3, y7);


			s0 = CL_add(t0, t4);
			s2 = CL_sub(t0, t4);
			s4 = CL_mac_j(t1, t5);
			s5 = CL_msu_j(t1, t5);
			s1 = CL_add(t2, t6);
			s3 = CL_mul_j(CL_sub(t2, t6));
			t0 = CL_shr(CL_add(t3, t7), SCALEFACTORN2 - 1);
			t1 = CL_shr(CL_sub(t3, t7), SCALEFACTORN2 - 1);
			s6 = CL_scale_t(CL_msu_j(t1, t0), C81);
			s7 = CL_dscale_t(CL_swap_real_imag(CL_msu_j(t0, t1)), C81, C82);

			pComplexBuf[i + 0 * dim1] = CL_add(s0, s1);
			pComplexBuf[i + 1 * dim1] = CL_add(s5, s6);
			pComplexBuf[i + 2 * dim1] = CL_sub(s2, s3);
			pComplexBuf[i + 3 * dim1] = CL_add(s4, s7);
			pComplexBuf[i + 4 * dim1] = CL_sub(s0, s1);
			pComplexBuf[i + 5 * dim1] = CL_sub(s5, s6);
			pComplexBuf[i + 6 * dim1] = CL_add(s2, s3);
			pComplexBuf[i + 7 * dim1] = CL_sub(s4, s7);
		}


		FOR(i = 1; i<dim1; i++)
		{
			y0 = CL_shr(x_cmplx[i + 0 * dim1], 1);
			y1 = CL_shr(CL_mult_32x16(x_cmplx[i + 1 * dim1], *(const cmplx_s *)&W[sc*i + sc * 1 * dim1 - Woff]), 1);
			y2 = CL_shr(CL_mult_32x16(x_cmplx[i + 2 * dim1], *(const cmplx_s *)&W[sc*i + sc * 2 * dim1 - Woff]), 1);
			y3 = CL_shr(CL_mult_32x16(x_cmplx[i + 3 * dim1], *(const cmplx_s *)&W[sc*i + sc * 3 * dim1 - Woff]), 1);
			y4 = CL_shr(CL_mult_32x16(x_cmplx[i + 4 * dim1], *(const cmplx_s *)&W[sc*i + sc * 4 * dim1 - Woff]), 1);
			y5 = CL_shr(CL_mult_32x16(x_cmplx[i + 5 * dim1], *(const cmplx_s *)&W[sc*i + sc * 5 * dim1 - Woff]), 1);
			y6 = CL_shr(CL_mult_32x16(x_cmplx[i + 6 * dim1], *(const cmplx_s *)&W[sc*i + sc * 6 * dim1 - Woff]), 1);
			y7 = CL_shr(CL_mult_32x16(x_cmplx[i + 7 * dim1], *(const cmplx_s *)&W[sc*i + sc * 7 * dim1 - Woff]), 1);

			t0 = CL_shr(CL_add(y0, y4), SCALEFACTORN2 - 1);
			t1 = CL_shr(CL_sub(y0, y4), SCALEFACTORN2 - 1);
			t2 = CL_shr(CL_add(y1, y5), SCALEFACTORN2 - 1);
			t3 = CL_sub(y1, y5);
			t4 = CL_shr(CL_add(y2, y6), SCALEFACTORN2 - 1);
			t5 = CL_shr(CL_sub(y2, y6), SCALEFACTORN2 - 1);
			t6 = CL_shr(CL_add(y3, y7), SCALEFACTORN2 - 1);
			t7 = CL_sub(y3, y7);


			s0 = CL_add(t0, t4);
			s2 = CL_sub(t0, t4);
			s4 = CL_mac_j(t1, t5);
			s5 = CL_msu_j(t1, t5);
			s1 = CL_add(t2, t6);
			s3 = CL_mul_j(CL_sub(t2, t6));
			t0 = CL_shr(CL_add(t3, t7), SCALEFACTORN2 - 1);
			t1 = CL_shr(CL_sub(t3, t7), SCALEFACTORN2 - 1);
			s6 = CL_scale_t(CL_msu_j(t1, t0), C81);
			s7 = CL_dscale_t(CL_swap_real_imag(CL_msu_j(t0, t1)), C81, C82);

			pComplexBuf[i + 0 * dim1] = CL_add(s0, s1);
			pComplexBuf[i + 1 * dim1] = CL_add(s5, s6);
			pComplexBuf[i + 2 * dim1] = CL_sub(s2, s3);
			pComplexBuf[i + 3 * dim1] = CL_add(s4, s7);
			pComplexBuf[i + 4 * dim1] = CL_sub(s0, s1);
			pComplexBuf[i + 5 * dim1] = CL_sub(s5, s6);
			pComplexBuf[i + 6 * dim1] = CL_add(s2, s3);
			pComplexBuf[i + 7 * dim1] = CL_sub(s4, s7);

		}

		BREAK;
	}

	case  10:
	{
		cmplx  y[20];
		{
			FOR(j = 0; j<dim2; j++)
			{
				y[j] = CL_move(x_cmplx[j*dim1]);
			}
			fft10_with_cmplx_data(&y[0]);
			FOR(j = 0; j<dim2; j++)
			{
				pComplexBuf[j*dim1] = y[j];
			}
			FOR(i = 1; i<dim1; i++)
			{
				y[0] = CL_move(x_cmplx[i]);
				FOR(j = 1; j<dim2; j++)
				{
					y[j] = CL_mult_32x16(x_cmplx[i + j * dim1], *(const cmplx_s *)&W[sc*i + sc * j*dim1 - Woff]);
				}
				fft10_with_cmplx_data(&y[0]);
				FOR(j = 0; j<dim2; j++)
				{
					pComplexBuf[i + j * dim1] = y[j];
				}
			}

		}
		BREAK;
	}
	case  16:
	{
		cmplx y[20];

		FOR(j = 0; j<dim2; j++)
		{
			y[j] = CL_shr(x_cmplx[0 + j * dim1], SCALEFACTOR16);
		}
		fft16_with_cmplx_data(&y[0], 0);

		FOR(j = 0; j<dim2; j++)
		{
			pComplexBuf[j*dim1] = y[j];
		}
		FOR(i = 1; i<dim1; i++)
		{
			y[0] = CL_shr(x_cmplx[i + (0 + 0)*dim1], SCALEFACTOR16);
			y[1] = CL_shr(CL_mult_32x16(x_cmplx[i + dim1], *(const cmplx_s *)&W[len + sc * i + 0 * dim1 - Woff]), SCALEFACTOR16);

			FOR(j = 2; j<dim2; j = j + 2)
			{
				y[(j + 0)] = CL_shr(CL_mult_32x16(x_cmplx[i + (j + 0)*dim1], *(const cmplx_s *)&W[sc*i + j * dim1 - Woff]), SCALEFACTOR16);
				y[(j + 1)] = CL_shr(CL_mult_32x16(x_cmplx[i + (j + 1)*dim1], *(const cmplx_s *)&W[len + sc * i + j * dim1 - Woff]), SCALEFACTOR16);
			}
			fft16_with_cmplx_data(&y[0], 0);
			FOR(j = 0; j<dim2; j++)
			{
				pComplexBuf[i + j * dim1] = y[j];
			}
		}

	}
	BREAK;

	case  20:

		assert(dim1 == 20 || dim1 == 30); /* cplxMpy4_10_0 contains shift values hardcoded FOR 20x10 */
		IF(EQ_16(dim1, 20))
		{
			cmplx y[20];
			FOR(j = 0; j<dim2; j++)
			{
				y[j] = CL_move(x_cmplx[j*dim1]);
			}
			fft20_with_cmplx_data(&y[0]);
			FOR(j = 0; j<dim2; j++)
			{
				pComplexBuf[j*dim1] = y[j];
			}
			FOR(i = 1; i<dim1; i++)
			{
				y[0] = CL_move(x_cmplx[i]);
				y[1] = CL_mult_32x16(x_cmplx[i + dim1], *(const cmplx_s *)&W[len + sc * i + 0 * dim1 - Woff]);
				FOR(j = 2; j<dim2; j = j + 2)
				{

					y[j + 0] = CL_mult_32x16(x_cmplx[i + (j + 0)*dim1], *(const cmplx_s *)&W[sc*i + j * dim1 - Woff]);
					y[j + 1] = CL_mult_32x16(x_cmplx[i + (j + 1)*dim1], *(const cmplx_s *)&W[len + sc * i + j * dim1 - Woff]);
				}
				fft20_with_cmplx_data(&y[0]);
				FOR(j = 0; j<dim2; j++)
				{
					pComplexBuf[i + j * dim1] = y[j];
				}
			}

		}
		ELSE
		{
			cmplx y[20];
		FOR(j = 0; j<dim2; j++)
		{
			y[j] = CL_shl(x_cmplx[j*dim1],  (SCALEFACTOR30 - SCALEFACTOR20));
		}
		fft20_with_cmplx_data(&y[0]);
		FOR(j = 0; j<dim2; j++)
		{
			pComplexBuf[j*dim1] = y[j];
		}
		FOR(i = 1; i<dim1; i++)
		{
			y[0] = CL_shl(x_cmplx[i],  (SCALEFACTOR30 - SCALEFACTOR20));
			y[1] = CL_shl(CL_mult_32x16(x_cmplx[i + dim1], *(const cmplx_s *)&W[len + sc * i + 0 * dim1 - Woff]) ,  (SCALEFACTOR30 - SCALEFACTOR20));
			FOR(j = 2; j<dim2; j = j + 2)
			{

				y[j + 0] = CL_shl(CL_mult_32x16(x_cmplx[i + (j + 0)*dim1],  *(const cmplx_s *)&W[sc*i + j * dim1 - Woff]), (SCALEFACTOR30 - SCALEFACTOR20));
				y[j + 1] = CL_shl(CL_mult_32x16(x_cmplx[i + (j + 1)*dim1], *(const cmplx_s *)&W[len + sc * i + j * dim1 - Woff]),  (SCALEFACTOR30 - SCALEFACTOR20));
			}
			fft20_with_cmplx_data(&y[0]);
			FOR(j = 0; j<dim2; j++)
			{
				pComplexBuf[i + j * dim1] = y[j];
			}
		}

		}
		BREAK;
	}
#if (WMOPS)
	multiCounter[currCounter].CL_move += len;
#endif				

}


/**
 * \brief Complex valued FFT
 *
 * \param    [i/o] re          real part
 * \param    [i/o] im          imag part
 * \param    [i  ] sizeOfFft   length of fft
 * \param    [i  ] s           stride real and imag part
 * \param    [i  ] scale       scalefactor
 *
 * \return void
 */
void BASOP_cfft(cmplx *pComplexBuf, Word16 sizeOfFft, Word16 *scale, Word32 x[2*BASOP_CFFT_MAX_LENGTH])
{
	Word16 s;
    s = 0; 
    move16();	
    SWITCH(sizeOfFft)
    {
    case 5:
        fft5_with_cmplx_data(pComplexBuf);
        s = add(*scale,SCALEFACTOR5);
        BREAK;

    case 8:
        fft8_with_cmplx_data(pComplexBuf);
        s = add(*scale,SCALEFACTOR8);
        BREAK;

    case 10:
        fft10_with_cmplx_data(pComplexBuf);
        s = add(*scale,SCALEFACTOR10);
        BREAK;

    case 16:
        fft16_with_cmplx_data(pComplexBuf,1);
        s = add(*scale,SCALEFACTOR16);
        BREAK;

    case 20:
        fft20_with_cmplx_data(pComplexBuf);
        s = add(*scale,SCALEFACTOR20);
        BREAK;

    case 30:
        fft30_with_cmplx_data(pComplexBuf);
        s = add(*scale,SCALEFACTOR30);
        BREAK;

    case 32:
        fft32_with_cmplx_data(pComplexBuf);
        s = add(*scale,SCALEFACTOR32);
        BREAK;

    case 40:
        {
            fftN2(pComplexBuf,RotVector_320,40,5,8,8,x,40);
            s = add(*scale,SCALEFACTOR40);
            BREAK;
        }

    case 64:
        {
            fftN2(pComplexBuf,RotVector_256,64,8,8,8,x,64);
            s = add(*scale,SCALEFACTOR64);
            BREAK;
        }

    case 80:
        {
            fftN2(pComplexBuf,RotVector_320,80,10,8,4,x,40);
            s = add(*scale,SCALEFACTOR80);
            BREAK;
        }
    case 100:
        {
            fftN2(pComplexBuf,RotVector_400,100,10,10,4,x,40);
            s = add(*scale,SCALEFACTOR100);
            BREAK;
        }
    case 120:
        {
            fftN2(pComplexBuf,RotVector_480,120,15,8,4,x,60);
            s = add(*scale,SCALEFACTOR120);
            BREAK;
        }

    case 128:
        {
            fftN2(pComplexBuf,RotVector_256,128,16,8,4,x,64);
            s = add(*scale,SCALEFACTOR128);
            BREAK;
        }

    case 160:
        {
            fftN2(pComplexBuf,RotVector_320,160,20,8,2,x,40);
            s = add(*scale,SCALEFACTOR160);
            BREAK;
        }

    case 200:
        {
            fftN2(pComplexBuf,RotVector_400,200,20,10,2,x,40);
            s = add(*scale,SCALEFACTOR200);
            BREAK;
        }

    case 240:
        {
            fftN2(pComplexBuf,RotVector_480,240,30,8,2,x,60);
            s = add(*scale,SCALEFACTOR240);
            BREAK;
        }

    case 256:
        {
            fftN2(pComplexBuf,RotVector_256,256,32,8,2,x,64);
            s = add(*scale,SCALEFACTOR256);
            BREAK;
        }

    case 320:
        {
            fftN2(pComplexBuf,RotVector_320,320,20,16,2,x,40);
            s = add(*scale,SCALEFACTOR320);
            BREAK;
        }

    case 400:
        {
            fftN2(pComplexBuf,RotVector_400,400,20,20,2,x,40);
            s = add(*scale,SCALEFACTOR400);
            BREAK;
        }

    case 480:
        {
            fftN2(pComplexBuf,RotVector_480,480,30,16,2,x,60);
            s = add(*scale,SCALEFACTOR480);
            BREAK;
        }
    case 600:
        {
            fftN2(pComplexBuf,RotVector_600,600,30,20,2,x,60);
            s = add(*scale,SCALEFACTOR600);
            BREAK;
        }
    default:
        assert(0);
    }
    *scale = s;
    move16();
}



#define RFFT_TWIDDLE1(x, t1, t2, t3, t4, w1, w2, xb0, xb1, xt0, xt1) \
{ \
  xb0 = L_shr(x[2*i+0],2); \
  xb1 = L_shr(x[2*i+1],2); \
  xt0 = L_shr(x[sizeOfFft-2*i+0],2); \
  xt1 = L_shr(x[sizeOfFft-2*i+1],2); \
  t1 = L_sub(xb0,xt0); \
  t2 = L_add(xb1,xt1); \
  t3 = L_sub(Mpy_32_16_1(t1,w1),Mpy_32_16_1(t2,w2)); \
  t4 = L_add(Mpy_32_16_1(t1,w2),Mpy_32_16_1(t2,w1)); \
  t1 = L_add(xb0,xt0); \
  t2 = L_sub(xb1,xt1); \
}

#define RFFT_TWIDDLE2(x, t1, t2, t3, t4, w1, w2, xb0, xb1, xt0, xt1) \
{ \
  xb0 = L_shr(x[2*i+0],2); \
  xb1 = L_shr(x[2*i+1],2); \
  xt0 = L_shr(x[sizeOfFft-2*i+0],2); \
  xt1 = L_shr(x[sizeOfFft-2*i+1],2); \
  t1 = L_sub(xb0,xt0); \
  t2 = L_add(xb1,xt1); \
  t3 = L_add(Mpy_32_16_1(t1,w1),Mpy_32_16_1(t2,w2)); \
  t4 = L_sub(Mpy_32_16_1(t2,w1),Mpy_32_16_1(t1,w2)); \
  t1 = L_add(xb0,xt0); \
  t2 = L_sub(xb1,xt1); \
}

/**
 * \brief Real valued FFT
 *
 *        forward rFFT (isign == -1):
 *        The input vector contains sizeOfFft real valued time samples. The output vector contains sizeOfFft/2 complex valued
 *        spectral values. The spectral values resides interleaved in the output vector. x[1] contains re[sizeOfFft], because
 *        x[1] is zero by default. This allows use of sizeOfFft length buffer instead of sizeOfFft+1.
 *
 *        inverse rFFT (isign == +1):
 *        The input vector contains sizeOfFft complex valued spectral values. The output vector contains sizeOfFft real valued
 *        time samples. The spectral values resides interleaved in the input vector. x[1] contains re[sizeOfFft].
 *        (see also forward rFFT)
 *
 * \param    [i/o] x           real input / real and imag output interleaved
 * \param    [i  ] sizeOfFft   length of fft
 * \param    [i  ] scale       scalefactor
 * \param    [i  ] isign       forward (-1) / backward (+1)
 *
 * \return void
 */
void BASOP_rfft(Word32 *x, Word16 sizeOfFft, Word16 *scale, Word16 isign)
{
    Word16 i, s=0, sizeOfFft2, sizeOfFft4, sizeOfFft8, wstride;  /* clear s to calm down compiler */
    Word32 t1, t2, t3, t4, xb0, xb1, xt0, xt1;
    const PWord16 *w1;
    Word16 c1;
    Word16 c2;
    Word32 workBuffer[2*BASOP_CFFT_MAX_LENGTH];



    sizeOfFft2 = shr(sizeOfFft,1);
    sizeOfFft4 = shr(sizeOfFft,2);
    sizeOfFft8 = shr(sizeOfFft,3);

    BASOP_getTables(NULL, &w1, &wstride, sizeOfFft2);

    SWITCH (isign)
    {
    case -1:

        BASOP_cfft((cmplx *)x, sizeOfFft2, scale, workBuffer);

        xb0  = L_shr(x[0],1);
        xb1  = L_shr(x[1],1);
        x[0] = L_add(xb0,xb1);
        move32();
        x[1] = L_sub(xb0,xb1);
        move32();

        FOR (i=1; i<sizeOfFft8; i++)
        {
            RFFT_TWIDDLE1(x, t1, t2, t3, t4, w1[i*wstride].v.im, w1[i*wstride].v.re, xb0, xb1, xt0, xt1)
            x[2*i]             =  L_sub(t1,t3);
            move32();
            x[2*i+1]           =  L_sub(t2,t4);
            move32();
            x[sizeOfFft-2*i]   =  L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] =  L_negate(L_add(t2,t4));
            move32();
        }

        FOR (i=sizeOfFft8; i<sizeOfFft4; i++)
        {
            RFFT_TWIDDLE1(x, t1, t2, t3, t4, w1[(sizeOfFft4-i)*wstride].v.re, w1[(sizeOfFft4-i)*wstride].v.im, xb0, xb1, xt0, xt1)
            x[2*i]             =  L_sub(t1,t3);
            move32();
            x[2*i+1]           =  L_sub(t2,t4);
            move32();
            x[sizeOfFft-2*i]   =  L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] =  L_negate(L_add(t2,t4));
            move32();
        }

        x[sizeOfFft-2*i]   =  L_shr(x[2*i+0],1);
        move32();
        x[sizeOfFft-2*i+1] =  L_negate(L_shr(x[2*i+1],1));
        move32();

        *scale = add(*scale,1);
        move16();
        BREAK;

    case +1:

        xb0 = L_shr(x[0],2);
        xb1 = L_shr(x[1],2);
        x[0] = L_add(xb0,xb1);
        move32();
        x[1] = L_sub(xb1,xb0);
        move32();

        FOR (i=1; i<sizeOfFft8; i++)
        {
            RFFT_TWIDDLE2(x, t1, t2, t3, t4, w1[i*wstride].v.im, w1[i*wstride].v.re, xb0, xb1, xt0, xt1)

            x[2*i]             = L_sub(t1,t3);
            move32();
            x[2*i+1]           = L_sub(t4,t2);
            move32();
            x[sizeOfFft-2*i]   = L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] = L_add(t2,t4);
            move32();
        }

        FOR (i=sizeOfFft8; i<sizeOfFft4; i++)
        {
            RFFT_TWIDDLE2(x, t1, t2, t3, t4, w1[(sizeOfFft4-i)*wstride].v.re, w1[(sizeOfFft4-i)*wstride].v.im, xb0, xb1, xt0, xt1)

            x[2*i]             = L_sub(t1,t3);
            move32();
            x[2*i+1]           = L_sub(t4,t2);
            move32();
            x[sizeOfFft-2*i]   = L_add(t1,t3);
            move32();
            x[sizeOfFft-2*i+1] = L_add(t2,t4);
            move32();
        }

        x[sizeOfFft-2*i]     = L_shr(x[2*i+0],1);
        move32();
        x[sizeOfFft-2*i+1]   = L_shr(x[2*i+1],1);
        move32();

        BASOP_cfft((cmplx *)x, sizeOfFft2, scale, workBuffer);

        SWITCH (sizeOfFft)
        {
        case 40:
        case 80:
        case 320:
        case 640:
            c1 = FFTC(0x66666680);
            c2 = FFTC(0x99999980);
            FOR(i=0; i<sizeOfFft2; i++)
            {
                x[2*i  ] = Mpy_32_xx(x[2*i  ],c1);
                move32();
                x[2*i+1] = Mpy_32_xx(x[2*i+1],c2);
                move32();
            }
            BREAK;

        case 64:
        case 256:
        case 512:
            FOR(i=0; i<sizeOfFft2; i++)
            {
                x[2*i+1] = L_negate(x[2*i+1]);
                move32();
            }
            BREAK;

        default:
            assert(0);
        }

        SWITCH (sizeOfFft)
        {
        case 64:
            s = add(*scale,2-6);
            BREAK;

        case 512:
            s = add(*scale,2-9);
            BREAK;

        case 640:
            s = add(*scale,2-9);
            BREAK;

        default:
            assert(0);
        }
        *scale = s;
        move16();
        BREAK;
    }

}


