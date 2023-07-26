/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include "options.h"        /* Compilation switches                   */
#include "stl.h"
#include "prot_fx.h"


/*--------------------------------------------------------------------------
*  get_delay_fx()
*
*  Function returns various types of delays in the codec in ms.
*--------------------------------------------------------------------------*/

Word32 get_delay_fx(            /* o  : delay value in ms                         */
    const Word16 what_delay,    /* i  : what delay? (ENC or DEC)                  */
    const Word32 io_fs          /* i  : input/output sampling frequency           */
)
{
    Word32 delay = 0;

    IF( EQ_16(what_delay,ENC))
    {
        delay = (DELAY_FIR_RESAMPL_NS + ACELP_LOOK_NS);
        move32();
    }
    ELSE
    {
        IF( EQ_32(io_fs,8000))
        {
            delay = DELAY_CLDFB_NS;
            move32();
        }
        ELSE
        {
            delay = DELAY_BWE_TOTAL_NS;
            move32();
        }
    }

    return delay;
}
