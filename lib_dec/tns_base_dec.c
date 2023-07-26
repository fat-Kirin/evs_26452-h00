/*====================================================================================
    EVS Codec 3GPP TS26.452 Nov 04, 2021. Version 16.4.0
  ====================================================================================*/


#include "stl.h"
#include "prot_fx.h"
#include "stl.h"
#include <memory.h>
#include <assert.h>
#include "rom_com_fx.h"

/** Linear prediction analysis/synthesis filter definition.
  * @param order filter order.
  * @param parCoeff filter (PARCOR) coefficients.
  * @param state state of the filter. Must be at least of 'order' size.
  * @param x the current input value.
  * @return the output of the filter.
  */
typedef Word32 (* TLinearPredictionFilter)(Word16 order, Word16 const parCoeff[], Word32 * state, Word32 x);

/********************************/
/*      Interface functions     */
/********************************/

Word16 ReadTnsData(STnsConfig const * pTnsConfig, Decoder_State_fx *st, Word16 * pnBits, Word16 * stream, Word16 * pnSize)
{
    Word16 start_bit_pos;


    move16();
    start_bit_pos = st->next_bit_pos_fx;

    IF ( GT_16(pTnsConfig->nMaxFilters, 1))
    {

        IF ( LT_16(pTnsConfig->iFilterBorders[0],512))
        {
            ReadFromBitstream(tnsEnabledSWBTCX10BitMap, 1, st, &stream, pnSize);
        }
        ELSE
        {
            ReadFromBitstream(tnsEnabledSWBTCX20BitMap, 1, st, &stream, pnSize);
        }
    }
    ELSE
    {
        ReadFromBitstream(tnsEnabledWBTCX20BitMap, 1, st, &stream, pnSize);
    }

    move16();
    *pnBits = sub(st->next_bit_pos_fx, start_bit_pos);


    return TNS_NO_ERROR;
}

Word16 DecodeTnsData(STnsConfig const * pTnsConfig, Word16 const * stream, Word16 * pnSize, STnsData * pTnsData)
{
    Word16 result;



    ResetTnsData(pTnsData);

    IF ( GT_16(pTnsConfig->nMaxFilters, 1))
    {

        IF ( LT_16(pTnsConfig->iFilterBorders[0],512))
        {
            SetParameters(tnsEnabledSWBTCX10BitMap, 1, pTnsData, &stream, pnSize);
        }
        ELSE
        {
            SetParameters(tnsEnabledSWBTCX20BitMap, 1, pTnsData, &stream, pnSize);
        }
    }
    ELSE
    {
        SetParameters(tnsEnabledWBTCX20BitMap, 1, pTnsData, &stream, pnSize);
    }

    move16();
    result = FALSE;
    if (pTnsData->nFilters > 0)
    {
        move16();
        result = TRUE;
    }


    return result;
}

