/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   STM32 Controller Driver                                             */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/* Include necessary system files.  */

#define UX_SOURCE_CODE
#define UX_HCD_STM32_SOURCE_CODE

#include "ux_api.h"
#include "ux_hcd_stm32.h"
#include "ux_host_stack.h"


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _ux_hcd_stm32_request_bulk_transfer                 PORTABLE C      */
/*                                                           6.0          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*     This function performs a bulk transfer request. A bulk transfer    */
/*     can be larger than the size of the stm32 buffer so it may be       */
/*     required to chain multiple tds to accommodate this transfer        */
/*     request. A bulk transfer is non blocking, so we return before the  */
/*     transfer request is completed.                                     */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    hcd_stm32                             Pointer to STM32 controller   */
/*    transfer_request                      Pointer to transfer request   */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    Completion Status                                                   */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _ux_hcd_stm32_regular_td_obtain     Obtain regular TD               */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    STM32 Controller Driver                                             */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/
UINT  _ux_hcd_stm32_request_bulk_transfer(UX_HCD_STM32 *hcd_stm32, UX_TRANSFER *transfer_request)
{

UX_ENDPOINT         *endpoint;
UX_HCD_STM32_ED     *ed;
UINT                direction;
UINT                length;


    /* Get the pointer to the Endpoint.  */
    endpoint =  (UX_ENDPOINT *) transfer_request -> ux_transfer_request_endpoint;

    /* Now get the physical ED attached to this endpoint.  */
    ed =  endpoint -> ux_endpoint_ed;

    /* Save the pending transfer in the ED.  */
    ed -> ux_stm32_ed_transfer_request = transfer_request;

    /* Direction, 0 : Output / 1 : Input */
    direction = (transfer_request -> ux_transfer_request_type & UX_REQUEST_DIRECTION) == UX_REQUEST_IN ? 1 : 0;

    /* If the direction is OUT, request size is larger than MPS, and DMA is not used, we need to set transfer length to MPS.  */
    if ((direction == 0) && (transfer_request -> ux_transfer_request_requested_length > endpoint -> ux_endpoint_descriptor.wMaxPacketSize)
#ifndef USB_DRD_FS
        && (hcd_stm32 -> hcd_handle -> Init.dma_enable == 0)
#endif /* USB_DRD_FS */
          )
    {

        /* Set transfer length to MPS.  */
        length = endpoint -> ux_endpoint_descriptor.wMaxPacketSize;
    }
    else
    {

        /* Keep the original transfer length.  */
        length = transfer_request -> ux_transfer_request_requested_length;
    }

    /* Save the transfer status in the ED.  */
    ed -> ux_stm32_ed_status = direction == 0 ? UX_HCD_STM32_ED_STATUS_BULK_OUT : UX_HCD_STM32_ED_STATUS_BULK_IN;

    /* Save the transfer length.  */
    transfer_request -> ux_transfer_request_packet_length = length;

    /* Submit the transfer request.  */
    HAL_HCD_HC_SubmitRequest(hcd_stm32 -> hcd_handle, ed -> ux_stm32_ed_channel,
                             direction,
                             EP_TYPE_BULK, USBH_PID_DATA,
                             transfer_request->ux_transfer_request_data_pointer,
                             length, 0);

    /* Return successful completion.  */
    return(UX_SUCCESS);
}

