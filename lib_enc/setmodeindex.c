/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "options.h"
#include "stl.h"
#include "prot_fx.h"
#include "cnst_fx.h"
#include "rom_com_fx.h"
#include "rom_enc_fx.h"

/*---------------------------------------------------------------------------

   function name: SetModeIndex
   description:   function for configuring the codec between frames - currently bit rate and bandwidth only
                  must not be called while a frame is encoded - hence mutexes must be used in MT environments

   format:        BANDWIDTH*16 + BITRATE (mode index)

  ---------------------------------------------------------------------------*/

void SetModeIndex(
    Encoder_State_fx *st,
    Word32    total_brate,
    Word16    bwidth,
    const Word16 shift)
{

    /* Reconfigure the core coder */
    test();
    test();
    test();
    IF(
        (NE_32(st->last_total_brate_fx,total_brate) ) ||
        (NE_16(st->last_bwidth_fx,bwidth) )  ||
        (EQ_16(st->last_codec_mode,MODE1)  )
        || (NE_16(st->rf_mode_last,st->rf_mode)  )
    )
    {
        core_coder_mode_switch( st, st->bwidth_fx, total_brate, shift);
    }


    return;
}
