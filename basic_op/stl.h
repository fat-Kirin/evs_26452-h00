/*
  ===========================================================================
   File: STL.H                                           v.2.3 - 30.Nov.2009
  ===========================================================================

            ITU-T STL  BASIC OPERATORS

            MAIN      HEADER      FILE

   History:
   07 Nov 04   v2.0     Incorporation of new 32-bit / 40-bit / control
                        operators for the ITU-T Standard Tool Library as 
                        described in Geneva, 20-30 January 2004 WP 3/16 Q10/16
                        TD 11 document and subsequent discussions on the
                        wp3audio@yahoogroups.com email reflector.
   March 06   v2.1      Changed to improve portability.                        

  ============================================================================
*/


#ifndef _STL_H
#define _STL_H


#include <stdlib.h> /* for size_t */

#define ENH_U_32_BIT_OPERATOR
#define COMPLEX_OPERATOR
#define CONTROL_CODE_OPS
#define ENH_32_BIT_OPERATOR

#include "patch.h"
/* both ALLOW_40bits and ALLOW_ENH_UL32 shall be enabled for the EVS codec.  */
#define ALLOW_40bits         /* allow 32x16 and 32x32 multiplications */
#define ALLOW_ENH_UL32       /* allow enhanced  unsigned 32bit  operators */
#include "typedef.h"
#include "basop32.h"
#include "count.h"
#include "move.h"
#include "control.h"
#include "enh1632.h"
#include "oper_32b.h" 
#include "math_op.h" 
#include "log2.h" 

#include "enh40.h"


#ifdef ENH_64_BIT_OPERATOR
#include "enh64.h"
#endif

#ifdef ENH_32_BIT_OPERATOR
#include "enh32.h"
#endif

#ifdef COMPLEX_OPERATOR
#include "complex_basop.h"
#endif

#ifdef ENH_U_32_BIT_OPERATOR
#include "enhUL32.h"
#endif

#endif /* ifndef _STL_H */


/* end of file */
