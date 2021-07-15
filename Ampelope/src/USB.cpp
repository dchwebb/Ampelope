#include "USB.h"

bool USBDebug = true;


/*
 *
 * PMAAddress constants - not sure where they come from
HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x00 , PCD_SNG_BUF, 0x18);
HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x80 , PCD_SNG_BUF, 0x58);
HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x81 , PCD_SNG_BUF, 0xC0);
HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x01 , PCD_SNG_BUF, 0x110);
HAL_PCDEx_PMAConfig((PCD_HandleTypeDef*)pdev->pData , 0x82 , PCD_SNG_BUF, 0x100);
*/

__STATIC_INLINE uint16_t SWAPBYTE(uint8_t *addr)
{
  uint16_t _SwapVal, _Byte1, _Byte2;
  uint8_t *_pbuff = addr;

  _Byte1 = *(uint8_t *)_pbuff;
  _pbuff++;
  _Byte2 = *(uint8_t *)_pbuff;

  _SwapVal = (_Byte2 << 8) | _Byte1;

  return _SwapVal;
}

void USBHandler::USB_ReadPMA(uint16_t wPMABufAddr, uint16_t wNBytes)
{

	uint32_t n = (uint32_t)wNBytes >> 1;
	uint32_t i, temp;
	volatile uint16_t *pdwVal;
	uint8_t *pBuf = (uint8_t *)(&xfer_buff);

	pdwVal = (volatile uint16_t *)((uint32_t)(USB_PMAADDR) + ((uint32_t)wPMABufAddr));		// 0x40006018
	pdwVal = (volatile uint16_t *)(USB_PMAADDR + wPMABufAddr);		// 0x40006018

	for (i = n; i != 0; i--) {
		temp = *(volatile uint16_t *)pdwVal;
		pdwVal++;
		*pBuf = (uint8_t)((temp >> 0) & 0xFFU);
		pBuf++;
		*pBuf = (uint8_t)((temp >> 8) & 0xFFU);
		pBuf++;
	}

	if ((wNBytes % 2U) != 0U) {
		temp = *pdwVal;
		*pBuf = (uint8_t)((temp >> 0) & 0xFFU);
	}
}

/**
 * @brief Copy a buffer from user memory area to packet memory area (PMA)
 * @param   USBx USB peripheral instance register address.
 * @param   pbUsrBuf pointer to user memory area.
 * @param   wPMABufAddr address into PMA.
 * @param   wNBytes no. of bytes to be copied.
 * @retval None
 */
void USBHandler::USB_WritePMA(uint16_t wPMABufAddr, uint16_t wNBytes)
{
	uint32_t n = ((uint32_t)wNBytes + 1U) >> 1;
	uint32_t i, temp1, temp2;
	volatile uint16_t *pdwVal;
	uint8_t *pBuf = (uint8_t *)outBuff;

	pdwVal = (volatile uint16_t *)((uint32_t)USB + 0x400U + ((uint32_t)wPMABufAddr));

	for (i = n; i != 0; i--) {
		temp1 = *pBuf;
		pBuf++;
		temp2 = temp1 | ((uint16_t)((uint16_t) *pBuf << 8));
		*pdwVal = (uint16_t)temp2;
		pdwVal++;

		pBuf++;
	}
}

/*
void USBHandler::USBD_ParseSetupRequest()
{
	uint8_t *pbuff = (uint8_t *)(&xfer_buff);
	req.loadData(pbuff);

	req.mRequest = *(uint8_t *)(pbuff);

	pbuff++;
	req.Request = *(uint8_t *)(pbuff);

	pbuff++;
	req.Value = SWAPBYTE(pbuff);

	pbuff++;
	pbuff++;
	req.Index = SWAPBYTE(pbuff);

	pbuff++;
	pbuff++;
	req.Length = SWAPBYTE(pbuff);
}
*/

void USBHandler::USBD_LL_SetupStage()
{
//	USBD_ParseSetupRequest();
//	uint8_t *pbuff = (uint8_t *)(&xfer_buff);
	req.loadData(reinterpret_cast<uint8_t*>(&xfer_buff));		// Parse the setup request into the req object

#if (USB_DEBUG)
	usbDebug[usbDebugNo].Request = req;
#endif
	//pdev->ep0_state = USBD_EP0_SETUP;
	//pdev->ep0_data_len = req.Length;

	switch (req.mRequest & 0x1FU) {
	case USB_REQ_RECIPIENT_DEVICE:
		USBD_StdDevReq();
		break;

	case USB_REQ_RECIPIENT_INTERFACE:
		//USBD_StdItfReq(pdev, &pdev->request);
		break;

	case USB_REQ_RECIPIENT_ENDPOINT:
		//USBD_StdEPReq(pdev, &pdev->request);
		break;

	default:
		//USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
		break;
	}

}

// USB_EPStartXfer setup and starts a transfer over an EP
//void USB_EPStartXfer(USB_EPTypeDef *ep)
void USBHandler::USB_EPStartXfer(Direction direction, uint8_t endpoint, uint32_t xfer_len)
{
	uint32_t len;
	uint16_t pmabuffer;
	uint16_t wEPVal;

	/* IN endpoint */
	if (direction == Direction::in) {
		// Multi packet transfer
		if (xfer_len > ep_maxPacket) {
			len = ep_maxPacket;
		} else {
			len = xfer_len;
		}

		/* configure and validate Tx endpoint */

		//USB_WritePMA(ep->xfer_buff, 0x58, len);
		USB_WritePMA(0x58, len);
		//PCD_SET_EP_TX_CNT(USBx, ep->num, len);
		USB_PMA->COUNT0_TX = len;
		//PCD_SET_EP_TX_STATUS(USBx, ep->num, USB_EP_TX_VALID);
		USB->EP0R ^= USB_EP_TX_VALID;
	} else {		// OUT endpoint

		// Multi packet transfer
		if (xfer_len > ep_maxPacket) {
			len = ep_maxPacket;
			xfer_len -= len;
		} else {
			len = xfer_len;
			xfer_len = 0U;
		}
		// configure and validate Rx endpoint
		//PCD_SET_EP_RX_CNT(USBx, ep->num, len);
		//PCD_SET_EP_RX_STATUS(USBx, ep->num, USB_EP_RX_VALID);
	}
}

/*


	if (direction == Direction::in) {				// IN endpoint

		endpoint = endpoint & EP_ADDR_MASK;			// Strip out 0x80 if endpoint passed eg as 0x81

		USBx_INEP(endpoint)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
		USBx_INEP(endpoint)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);

		if (endpoint == 0 && xfer_len > ep_maxPacket) {				// If the transfer is larger than the maximum packet size send the maximum size and use the remaining flag to trigger a second send
			xfer_rem = xfer_len - ep_maxPacket;
			xfer_len = ep_maxPacket;
		}

		USBx_INEP(endpoint)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (((xfer_len + ep_maxPacket - 1) / ep_maxPacket) << 19));
		USBx_INEP(endpoint)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & xfer_len);

		USBx_INEP(endpoint)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);	// EP enable, IN data in FIFO

		// Enable the Tx FIFO Empty Interrupt for this EP
		if (xfer_len > 0) {
			USBx_DEVICE->DIEPEMPMSK |= 1UL << (endpoint & EP_ADDR_MASK);
		}
	} else { 		// OUT endpoint

		USBx_OUTEP(endpoint)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
		USBx_OUTEP(endpoint)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

		USBx_OUTEP(endpoint)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
		USBx_OUTEP(endpoint)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & xfer_len);

		USBx_OUTEP(endpoint)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);		// EP enable
	}

}
	*/

void USBHandler::USBInterruptHandler() {		// In Drivers\STM32F4xx_HAL_Driver\Src\stm32f4xx_hal_pcd.c

	uint16_t count, wIstr, wEPVal, TxByteNbre;
	int epnum, ep_intr, epint;

	// Handle spurious interrupt
	if ((USB->ISTR) == 0)
		return;

	USB->ISTR &= ~(USB_ISTR_SOF | USB_ISTR_ESOF);

/*
	///////////		10			RXFLVL: RxQLevel Interrupt:  Rx FIFO non-empty Indicates that there is at least one packet pending to be read from the Rx FIFO.
	if (USB_ReadInterrupts(USB_OTG_GINTSTS_RXFLVL))
	{
		USB->GINTMSK &= ~USB_OTG_GINTSTS_RXFLVL;

		uint32_t receiveStatus = USB->GRXSTSP;		// OTG status read and pop register: not shown in SFR, but read only (ie do not pop) register under OTG_FS_GLOBAL->FS_GRXSTSR_Device
		epnum = receiveStatus & USB_OTG_GRXSTSP_EPNUM;		// Get the endpoint number
		uint16_t packetSize = (receiveStatus & USB_OTG_GRXSTSP_BCNT) >> 4;

#if (USB_DEBUG)
		usbDebug[usbDebugNo].IntData = receiveStatus;
		usbDebug[usbDebugNo].endpoint = epnum;
		usbDebug[usbDebugNo].PacketSize = packetSize;
#endif

		if (((receiveStatus & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_DATA_UPDT && packetSize != 0) {		// 2 = OUT data packet received
			USB_ReadPacket(xfer_buff, packetSize);
		} else if (((receiveStatus & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_SETUP_UPDT) {				// 6 = SETUP data packet received
			USB_ReadPacket(xfer_buff, 8U);
		}
		if (packetSize != 0) {
			xfer_count = packetSize;
#if (USB_DEBUG)
			usbDebug[usbDebugNo].xferBuff0 = xfer_buff[0];
			usbDebug[usbDebugNo].xferBuff1 = xfer_buff[1];
#endif
		}
		USB->GINTMSK |= USB_OTG_GINTSTS_RXFLVL;
	}


	/////////// 	80000 		OEPINT OUT endpoint interrupt
	if (USB_ReadInterrupts(USB_OTG_GINTSTS_OEPINT)) {

		// Read the output endpoint interrupt register to ascertain which endpoint(s) fired an interrupt
		ep_intr = ((USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) & USB_OTG_DAINTMSK_OEPM_Msk) >> 16;

		// process each endpoint in turn incrementing the epnum and checking the interrupts (ep_intr) if that endpoint fired
		epnum = 0;
		while (ep_intr != 0) {
			if ((ep_intr & 1) != 0) {
				epint = USBx_OUTEP(epnum)->DOEPINT & USBx_DEVICE->DOEPMSK;

#if (USB_DEBUG)
				usbDebug[usbDebugNo].endpoint = epnum;
				usbDebug[usbDebugNo].IntData = epint;
#endif

				if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC) {		// 0x01 Transfer completed

					USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_XFRC;				// Clear interrupt

					if (epnum == 0) {

				        // In CDC mode after 0x21 0x20 packets (line coding commands)
						if (dev_state == USBD_STATE_CONFIGURED && CmdOpCode != 0) {
							if (CmdOpCode == 0x20) {			// SET_LINE_CODING - capture the data passed to return when queried with GET_LINE_CODING
								for (uint8_t i = 0; i < outBuffSize; ++i) {
									((uint8_t*)&USBD_CDC_LineCoding)[i] = ((uint8_t*)xfer_buff)[i];
								}
							}
							USB_EPStartXfer(Direction::in, 0, 0);
							CmdOpCode = 0;
						}

						ep0_state = USBD_EP0_IDLE;
						USBx_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
					} else {
						// Call appropriate data handler depending on endpoint of data
						USB_EPStartXfer(Direction::out, epnum, xfer_count);
						cdcDataHandler((uint8_t*)xfer_buff, xfer_count);
					}
				}

				if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP) {		// SETUP phase done: the application can decode the received SETUP data packet.
					// Parse Setup Request containing data in xfer_buff filled by RXFLVL interrupt
					req.loadData((uint8_t*)xfer_buff);
#if (USB_DEBUG)
					usbDebug[usbDebugNo].Request = req;
#endif

					ep0_state = USBD_EP0_SETUP;

					switch (req.mRequest & 0x1F) {		// originally USBD_LL_SetupStage
					case USB_REQ_RECIPIENT_DEVICE:
						//initially USB_REQ_GET_DESCRIPTOR >> USB_DESC_TYPE_DEVICE (bmrequest is 0x6)
						USBD_StdDevReq();
						break;

					case USB_REQ_RECIPIENT_INTERFACE:
						if ((req.mRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS) {		// 0xA1 & 0x60 == 0x20

							if (req.Length > 0) {
								if ((req.mRequest & USB_REQ_DIRECTION_MASK) != 0U) {		// Device to host [USBD_CtlSendData]
									// CDC request 0xA1, 0x21, 0x0, 0x0, 0x7		GetLineCoding 0xA1 0x21 0 Interface 7; Data: Line Coding Data Structure
									// 0xA1 [1|01|00001] Device to host | Class | Interface

									outBuffSize = req.Length;
									outBuff = (uint8_t*)&USBD_CDC_LineCoding;
									ep0_state = USBD_EP0_DATA_IN;

#if (USB_DEBUG)
									usbDebug[usbDebugNo].PacketSize = outBuffSize;
									usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)outBuff)[0];
									usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)outBuff)[1];
#endif

									USB_EPStartXfer(Direction::in, 0, req.Length);		// sends blank request back
								} else {
									//CDC request 0x21, 0x20, 0x0, 0x0, 0x7			// USBD_CtlPrepareRx
									// 0x21 [0|01|00001] Host to device | Class | Interface
									CmdOpCode = req.Request;
									USB_EPStartXfer(Direction::out, epnum, req.Length);
								}
							} else {
								// 0x21, 0x22, 0x0, 0x0, 0x0	SetControlLineState 0x21 | 0x22 | 2 | Interface | 0 | None
								// 0x21, 0x20, 0x0, 0x0, 0x0	SetLineCoding       0x21 | 0x20 | 0 | Interface | 0 | Line Coding Data Structure
								USB_EPStartXfer(Direction::in, 0, 0);
							}

						}
						break;

					case USB_REQ_RECIPIENT_ENDPOINT:
						break;

					default:
						//USBD_LL_StallEP(pdev, (req.mRequest & 0x80U));
						break;
					}

					USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_STUP;				// Clear interrupt
				}

				if ((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS) {	// OUT token received when endpoint disabled
					USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_OTEPDIS;			// Clear interrupt
				}
				if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) {	// Status Phase Received interrupt
					USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_OTEPSPR;			// Clear interrupt
				}
				if ((epint & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK) {			// 0x2000 OUT NAK interrupt
					USBx_OUTEP(epnum)->DOEPINT = USB_OTG_DOEPINT_NAK;				// Clear interrupt
				}
			}
			epnum++;
			ep_intr >>= 1U;
		}

	}

	///////////		40000 		IEPINT: IN endpoint interrupt
	if (USB_ReadInterrupts(USB_OTG_GINTSTS_IEPINT)) {

		// Read in the device interrupt bits [initially 1]
		ep_intr = (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) & USB_OTG_DAINTMSK_IEPM_Msk;

		// process each endpoint in turn incrementing the epnum and checking the interrupts (ep_intr) if that endpoint fired
		epnum = 0;
		while (ep_intr != 0) {
			if ((ep_intr & 1) != 0) {

				epint = USBx_INEP(epnum)->DIEPINT & (USBx_DEVICE->DIEPMSK | (((USBx_DEVICE->DIEPEMPMSK >> (epnum & EP_ADDR_MASK)) & 0x1U) << 7));

#if (USB_DEBUG)
				usbDebug[usbDebugNo].endpoint = epnum;
				usbDebug[usbDebugNo].IntData = epint;
#endif

				if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC) {			// 0x1 Transfer completed interrupt
					uint32_t fifoemptymsk = (0x1UL << (epnum & EP_ADDR_MASK));
					USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

					USBx_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_XFRC;

					if (epnum == 0) {
						if (ep0_state == USBD_EP0_DATA_IN && xfer_rem == 0) {
							ep0_state = USBD_EP0_STATUS_OUT;							// After completing transmission on EP0 send an out packet [HAL_PCD_EP_Receive]
							xfer_buff[0] = 0;
							USB_EPStartXfer(Direction::out, 0, ep_maxPacket);

						} else if (ep0_state == USBD_EP0_DATA_IN && xfer_rem > 0) {		// For EP0 long packets are sent separately rather than streamed out of the FIFO
							outBuffSize = xfer_rem;
							xfer_rem = 0;
#if (USB_DEBUG)
							usbDebug[usbDebugNo].PacketSize = outBuffSize;
							usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)outBuff)[0];
							usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)outBuff)[1];
#endif
							USB_EPStartXfer(Direction::in, epnum, outBuffSize);
						}
					} else {
						transmitting = false;
					}
				}

				if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE) {			// 0x80 Transmit FIFO empty
#if (USB_DEBUG)
					usbDebug[usbDebugNo].PacketSize = outBuffSize;
					usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)outBuff)[0];
					usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)outBuff)[1];
#endif
					if (epnum == 0) {
						if (outBuffSize > ep_maxPacket) {
							xfer_rem = outBuffSize - ep_maxPacket;
							outBuffSize = ep_maxPacket;
						}

						USB_WritePacket(outBuff, epnum, static_cast<uint16_t>(outBuffSize));

						outBuff += outBuffSize;		// Move pointer forwards
						uint32_t fifoemptymsk = (0x1UL << (epnum & EP_ADDR_MASK));
						USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
					} else {

						// For regular endpoints keep writing packets to the FIFO while space available [PCD_WriteEmptyTxFifo]
						uint16_t len = std::min(outBuffSize - outBuffCount, static_cast<uint32_t>(ep_maxPacket));
						uint16_t len32b = (len + 3) / 4;			// FIFO size is in 4 byte words

						// INEPTFSAV[15:0]: IN endpoint Tx FIFO space available: 0x0: Endpoint Tx FIFO is full; 0x1: 1 31-bit word available; 0xn: n words available
						while (((USBx_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= len32b) && (outBuffCount < outBuffSize) && (outBuffSize != 0)) {

							len = std::min(outBuffSize - outBuffCount, static_cast<uint32_t>(ep_maxPacket));
							len32b = (len + 3) / 4;
#if (USB_DEBUG)
							usbDebug[usbDebugNo].PacketSize = outBuffSize - outBuffCount;
							usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)outBuff)[0];
							usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)outBuff)[1];
#endif
							USB_WritePacket(outBuff, epnum, len);

							outBuff += len;
							outBuffCount += len;
						}

						if (outBuffSize <= outBuffCount) {
							uint32_t fifoemptymsk = (0x1UL << (epnum & EP_ADDR_MASK));
							USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
						}
					}
				}

				if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC) {					// Timeout condition
					USBx_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_TOC;
				}
				if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE) {			// IN token received when Tx FIFO is empty
					USBx_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_ITTXFE;
				}
				if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE) {			// IN endpoint NAK effective
					USBx_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_INEPNE;
				}
				if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD) {			// Endpoint disabled interrupt
					USBx_INEP(epnum)->DIEPINT = USB_OTG_DIEPINT_EPDISD;
				}

			}
			epnum++;
			ep_intr >>= 1U;
		}

	}
*/

	/////////// 	8000 		USB_ISTR_CTR: Correct Transfer
	if (USB_ReadInterrupts(USB_ISTR_CTR)) {
		uint8_t epindex;

		/* stay in loop while pending interrupts */
		while ((USB->ISTR & USB_ISTR_CTR) != 0)	{
			wIstr = USB->ISTR;
			epindex = wIstr & USB_ISTR_EP_ID;		// extract highest priority endpoint number

			if (epindex == 0U) {
				/* Decode and service control endpoint interrupt */

				if ((wIstr & USB_ISTR_DIR) == 0U) {		// Direction IN
					/* DIR = 0 implies that (EP_CTR_TX = 1) always */
					//PCD_CLEAR_TX_EP_CTR(hpcd->Instance, PCD_ENDP0);
					USB->EP0R &= ~USB_EP_CTR_TX;
					//ep = &hpcd->IN_ep[0];

					//xfer_count = PCD_EP_TX_CNT(0);
					xfer_count = USB_PMA->COUNT0_TX;
					//ep->xfer_buff += xfer_count;

					// TX COMPLETE
					//HAL_PCD_DataInStageCallback(hpcd, 0U);
/*
					if ((hpcd->USB_Address > 0U) && (ep->xfer_len == 0U)) {
						USB->DADDR = ((uint16_t)hpcd->USB_Address | USB_DADDR_EF);
						hpcd->USB_Address = 0U;
					}
					*/
				} else {			// Setup or OUT interrupt
					/* DIR = 1 & CTR_RX => SETUP or OUT int */
					/* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
					//ep = &hpcd->OUT_ep[0];
					//wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

					if ((USB->EP0R & USB_EP_SETUP) != 0) {
						xfer_count = USB_PMA->COUNT0_RX & USB_COUNT0_RX_COUNT0_RX_Msk;
						USB_ReadPMA(0x18, xfer_count);		// Read setup data into xfer_buff

						/* SETUP bit kept frozen while CTR_RX = 1 */
						//PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);
						USB->EP0R &= ~USB_EP_CTR_RX;
						USBD_LL_SetupStage();				// Parse setup packet into request, locate data (eg descriptor) and populate TX buffer

					} else if ((USB->EP0R & USB_EP_CTR_RX) != 0U) {
						//PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);
						USB->EP0R &= ~USB_EP_CTR_RX;

						// DW USB Sram buffers at 0x40006000 - 0x400063FF		See p2037 for offsets
						xfer_count = USB_PMA->COUNT0_RX;
						/* Get Control Data OUT Packet */
						//ep->xfer_count = PCD_EP_RX_CNT(hpcd->Instance, ep->num);

						if ((xfer_count != 0U) && (xfer_buff != 0U)) {
							//USB_ReadPMA(xfer_buff,	ep->pmaadress, xfer_count);

							//xfer_buff += xfer_count;

							/* Process Control Data OUT Packet */
							//HAL_PCD_DataOutStageCallback(hpcd, 0U);
						}
/*
						if ((PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0) & USB_EP_SETUP) == 0U) {
							PCD_SET_EP_RX_CNT(hpcd->Instance, PCD_ENDP0, ep->maxpacket);
							PCD_SET_EP_RX_STATUS(hpcd->Instance, PCD_ENDP0, USB_EP_RX_VALID);
						}
						*/
					}
				}
			} else {
				/* Decode and service non control endpoints interrupt */
			}

		}
		USB->ISTR &= ~USB_ISTR_CTR;
	}


	/////////// 	100 		USB_ISTR_ESOF: Expected Start of frame
	if (USB_ReadInterrupts(USB_ISTR_ESOF)) {
		USB->ISTR &= ~USB_ISTR_ESOF;
	}

	/////////// 	1000 		USB_ISTR_WKUP: Wake Up
	if (USB_ReadInterrupts(USB_ISTR_WKUP)) {
		USB->CNTR &= ~USB_CNTR_FSUSP;
		USB->CNTR &= ~USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_WKUP;
	}

	/////////// 	800 		SUSP: Suspend Interrupt
	if (USB_ReadInterrupts(USB_ISTR_SUSP)) {
		USB->CNTR |= USB_CNTR_FSUSP;
		USB->ISTR &= ~USB_ISTR_SUSP;
		USB->CNTR |= USB_CNTR_LPMODE;
	}


	/////////// 	400 		RESET: Reset Interrupt
	if (USB_ReadInterrupts(USB_ISTR_RESET))	{
		USB->ISTR &= ~USB_ISTR_RESET;

		USB->EP0R |= (USB_EP_CONTROL | USB_EP_CTR_RX | USB_EP_CTR_TX);
		// Open EP0 OUT
		USB->EP0R = ((USB->EP0R ^ USB_EP_RX_VALID) ^ USB_EP_TX_NAK);		// RX and TX status need to be set with XOR

		// Configure NAK status for the Endpoint
		//USB->EP0R = USB_EP_TX_NAK;

		// Enable endpoint and set address to 0
		USB->DADDR = USB_DADDR_EF;

		// Configure the PMA for EP 0
		USB_PMA->ADDR0_TX = 0x58;						// Offset of PMA used for EP0 TX
		//USB_PMA->COUNT0_TX = 0x1C;
		USB_PMA->ADDR0_RX = 0x18;						// Offset of PMA used for EP0 RX
		USB_PMA->COUNT0_RX = (1 << 15) | (1 << 10);		// configure block size = 1 (32 Bytes); number of blocks = 2 (64 bytes)
	}

/*
	/////////// 	2000		ENUMDNE: Enumeration done Interrupt
	if (USB_ReadInterrupts(USB_OTG_GINTSTS_ENUMDNE)) {
		// Set the Maximum packet size of the IN EP based on the enumeration speed
		USBx_INEP(0U)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
		USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;		//  Clear global IN NAK

		// Assuming Full Speed USB and clock > 32MHz Set USB Turnaround time
		USB->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT;
		USB->GUSBCFG |= (6 << 10);

		USB_ActivateEndpoint(0, Direction::out, Control);			// Open EP0 OUT
		USB_ActivateEndpoint(0, Direction::in, Control);			// Open EP0 IN

		ep0_state = USBD_EP0_IDLE;

		USB->ISTR &= USB_OTG_GINTSTS_ENUMDNE;
	}


	///////////		40000000	SRQINT: Connection event Interrupt
	if (USB_ReadInterrupts(USB_OTG_GINTSTS_SRQINT))	{
		//HAL_PCD_ConnectCallback(hpcd);		// this doesn't seem to do anything
		USB->ISTR &= USB_OTG_GINTSTS_SRQINT;
	}


	/////////// 	80000000	WKUINT: Resume Interrupt
	if (USB_ReadInterrupts(USB_OTG_GINTSTS_WKUINT)) {
		// Clear the Remote Wake-up Signaling
		USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
		USB->ISTR &= USB_OTG_GINTSTS_WKUINT;
	}


	/////////// OTGINT: Handle Disconnection event Interrupt
	if (USB_ReadInterrupts(USB_OTG_GINTSTS_OTGINT)) {
		uint32_t temp = USB->GOTGINT;

		if ((temp & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET)
		{
			//HAL_PCD_DisconnectCallback(hpcd);
			//pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
		}
		USB->GOTGINT |= temp;
	}
*/
}



void USBHandler::InitUSB()
{

	RCC->CRRCR |= RCC_CRRCR_HSI48ON;					// Enable Internal High Speed oscillator for USB
	while ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0);		// Wait till internal USB oscillator is ready
	//RCC->D2CCIP2R |= RCC_D2CCIP2R_USBSEL;				// Set the USB CLock MUX to RC48
	RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;				// USB2OTG (OTG_HS2) Peripheral Clocks Enable
	//PWR->CR3 |= PWR_CR3_USB33DEN;						// Enable VDD33USB supply level detector

	NVIC_SetPriority(USB_LP_IRQn, 3);
	NVIC_EnableIRQ(USB_LP_IRQn);

	/* Set winterruptmask variable */
	uint32_t winterruptmask = USB_CNTR_CTRM  | USB_CNTR_WKUPM |
			USB_CNTR_SUSPM | USB_CNTR_ERRM |
			USB_CNTR_RESETM | USB_CNTR_L1REQM;			// USB_CNTR_SOFM | USB_CNTR_ESOFM |

	/* Clear interrupt mask */
	USB->CNTR &= (uint16_t)(~winterruptmask);

	USB->CNTR = (uint16_t)USB_CNTR_FRES;				//  Force USB Reset
	USB->CNTR = 0U;
	USB->ISTR = 0U;										// Clear pending interrupts
	USB->BTABLE = 0;									// Set Buffer table Address BTABLE_ADDRESS

	/* Clear pending interrupts */
	USB->ISTR = 0U;
	USB->CNTR = (uint16_t)(winterruptmask);

	/* Enabling DP Pull-UP bit to Connect internal PU resistor on USB DP line */
	USB->BCDR |= (uint16_t)USB_BCDR_DPPU;

/*
	// USB GPIO Configuration: PA8: USB_SOF; PA9: USB_VBUS; PA10: USB_ID; PA11: USB_DM; PA12: USB_DP
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;				// GPIO port clock

	// PA8 (SOF), PA10 (ID), PA11 (DM), PA12 (DP) (NB PA9 - VBUS uses default values)
	GPIOA->MODER &= ~GPIO_MODER_MODE9;
	GPIOA->MODER &= ~GPIO_MODER_MODE11_0;
	GPIOA->MODER &= ~GPIO_MODER_MODE12_0;
	//GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12;		// 11: High speed FIXME - set to low in HAL
	GPIOA->AFR[1] |= (10 << GPIO_AFRH_AFSEL11_Pos) | (10 << GPIO_AFRH_AFSEL12_Pos);		// Alternate Function 10 is OTG_FS


	NVIC_SetPriority(OTG_FS_IRQn, 3);
	NVIC_EnableIRQ(OTG_FS_IRQn);

	USB->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;		// Disable the controller's Global Int in the AHB Config reg
	USB->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;		// Select FS Embedded PHY

	// Reset the core (needed after clock change) - NB removed delays
	while ((USB->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0);
	USB->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
	while ((USB->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

	USB->GCCFG |= USB_OTG_GCCFG_PWRDWN;			// Activate the transceiver in transmission/reception. When reset, the transceiver is kept in power-down. 0 = USB FS transceiver disabled; 1 = USB FS transceiver enabled
	USB->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;		// Force USB device mode
	//HAL_Delay(50U);

	// Clear all transmit FIFO address and data lengths - these will be set to correct values below for endpoints 0,1 and 2
	for (uint8_t i = 0U; i < 15U; i++) {
		USB->DIEPTXF[i] = 0U;
	}

	USB->GCCFG |= USB_OTG_GCCFG_VBDEN; 			// Enable HW VBUS sensing

	// FIXME - delay here???
	USBx_DEVICE->DCFG |= USB_OTG_DCFG_DSPD;				// 11: Full speed using internal FS PHY

	USB->GRSTCTL |= USB_OTG_GRSTCTL_TXFNUM_4;	// Select buffers to flush. 10000: Flush all the transmit FIFOs in device or host mode
	USB->GRSTCTL |= USB_OTG_GRSTCTL_TXFFLSH;		// Flush the TX buffers
	while ((USB->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	USB->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;		// Flush the RX buffers
	while ((USB->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

	USB->ISTR = 0xBFFFFFFFU;					// Clear pending interrupts (except SRQINT Session request/new session detected)

	// Enable interrupts
	USB->GINTMSK = 0U;							// Disable all interrupts
	USB->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_USBSUSPM |			// Receive FIFO non-empty mask; USB suspend
			USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |							// USB reset; Enumeration done
			USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_WUIM |	// IN endpoint; OUT endpoint; Resume/remote wakeup detected
			USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT;								// Session request/new session detected; OTG interrupt [both used for VBUS sensing]


	// NB - FIFO Sizes are in words NOT bytes. There is a total size of 320 (320x4 = 1280 bytes) available which is divided up thus:
	// FIFO		Start		Size
	// RX 		0			128
	// EP0 TX	128			64
	// EP1 TX	192			64
	// EP2 TX 	256			64

	USB->GRXFSIZ = 128;		 					// Rx FIFO depth

	// Endpoint 0 Transmit FIFO size/address (as in device mode - this is also used as the non-periodic transmit FIFO size in host mode)
	USB->DIEPTXF0_HNPTXFSIZ = (64 << USB_OTG_TX0FD_Pos) |		// IN Endpoint 0 Tx FIFO depth
			(128 << USB_OTG_TX0FSA_Pos);								// IN Endpoint 0 FIFO transmit RAM start address - this is offset from the RX FIFO (set above to 128)

	// Endpoint 1 FIFO size/address (address is offset from EP0 address+size above)
	USB->DIEPTXF[0] = (64 << USB_OTG_DIEPTXF_INEPTXFD_Pos) |		// IN endpoint 1 Tx FIFO depth
			(192 << USB_OTG_DIEPTXF_INEPTXSA_Pos);  					// IN endpoint 1 FIFO transmit RAM start address

	// Endpoint 2 FIFO size/address (address is offset from EP1 address+size above)
	USB->DIEPTXF[1] = (64 << USB_OTG_DIEPTXF_INEPTXFD_Pos) |		// IN endpoint 2 Tx FIFO depth
			(256 << USB_OTG_DIEPTXF_INEPTXSA_Pos);  					// IN endpoint 2 FIFO transmit RAM start address

    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;			// Activate USB
    USB->GAHBCFG |= USB_OTG_GAHBCFG_GINT;		// Activate USB Interrupts
*/
}


void USBHandler::USB_ActivateEndpoint(uint8_t endpoint, Direction direction, EndPointType eptype)
{
	endpoint = endpoint & 0xF;
/*
	if (direction == Direction::in) {
		USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & static_cast<uint32_t>(1UL << (endpoint & EP_ADDR_MASK));

		if ((USBx_INEP(endpoint)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U) {
			USBx_INEP(endpoint)->DIEPCTL |= (ep_maxPacket & USB_OTG_DIEPCTL_MPSIZ) |
					(static_cast<uint32_t>(eptype) << 18) | (endpoint << 22) |
					USB_OTG_DIEPCTL_USBAEP;
		}
	} else {
		USBx_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & (static_cast<uint32_t>(1UL << (endpoint & EP_ADDR_MASK)) << 16);

		if (((USBx_OUTEP(endpoint)->DOEPCTL) & USB_OTG_DOEPCTL_USBAEP) == 0U) {
			USBx_OUTEP(endpoint)->DOEPCTL |= (ep_maxPacket & USB_OTG_DOEPCTL_MPSIZ) |
					static_cast<uint32_t>(eptype << 18) |
					USB_OTG_DOEPCTL_USBAEP;
		}
	}
*/
}

// USB_ReadPacket : read a packet from the RX FIFO
void USBHandler::USB_ReadPacket(uint32_t* pDest, uint16_t len) {
	uint32_t count32b = (len + 3) / 4;
/*
	for (uint32_t i = 0; i < count32b; i++)	{
		*pDest = USBx_DFIFO(0);
		pDest++;
	}
	*/
}

void USBHandler::USB_WritePacket(const uint8_t* src, uint8_t endpoint, uint16_t len) {
	uint32_t* pSrc = (uint32_t*)src;
	uint32_t count32b = (static_cast<uint32_t>(len) + 3U) / 4U;

	for (uint32_t i = 0; i < count32b; i++) {
		//USBx_DFIFO(endpoint) = *pSrc;
		pSrc++;
	}
}

// Descriptors in usbd_desc.c
void USBHandler::USBD_GetDescriptor() {

	switch (req.Value >> 8)	{
	case USB_DESC_TYPE_DEVICE:
		outBuff = USBD_FS_DeviceDesc;
		outBuffSize = sizeof(USBD_FS_DeviceDesc);
		break;

	case USB_DESC_TYPE_CONFIGURATION:
		outBuff = USBD_CDC_CfgFSDesc;
		outBuffSize = sizeof(USBD_CDC_CfgFSDesc);
		break;

	case USB_DESC_TYPE_BOS:
		outBuff = USBD_FS_BOSDesc;
		outBuffSize = sizeof(USBD_FS_BOSDesc);
		break;

	case USB_DESC_TYPE_STRING:

		switch ((uint8_t)(req.Value)) {
		case USBD_IDX_LANGID_STR:			// 300
			outBuff = USBD_LangIDDesc;
			outBuffSize = sizeof(USBD_LangIDDesc);
			break;
		case USBD_IDX_MFC_STR:				// 301
			outBuffSize = USBD_GetString((uint8_t*)USBD_MANUFACTURER_STRING, USBD_StrDesc);
			outBuff = USBD_StrDesc;
			break;
		case USBD_IDX_PRODUCT_STR:			// 302
			outBuffSize = USBD_GetString((uint8_t*)USBD_PRODUCT_STRING, USBD_StrDesc);
			outBuff = USBD_StrDesc;
			break;
		case USBD_IDX_SERIAL_STR:			// 303
			{
				// STM32 unique device ID (96 bit number starting at UID_BASE)
				uint32_t deviceserial0 = *(uint32_t*) UID_BASE;
				uint32_t deviceserial1 = *(uint32_t*) UID_BASE + 4;
				uint32_t deviceserial2 = *(uint32_t*) UID_BASE + 8;
				deviceserial0 += deviceserial2;

				if (deviceserial0 != 0) {
					IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
					IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
				}
				outBuff = USBD_StringSerial;
				outBuffSize = sizeof(USBD_StringSerial);
			}
			break;
/*
		case USBD_IDX_MIDI_STR:				// 304
			outBuffSize = USBD_GetString((uint8_t*)USBD_MIDI_STRING, USBD_StrDesc);
			outBuff = USBD_StrDesc;
	      break;
*/
	    case USBD_IDX_CDC_STR:				// 304
			outBuffSize = USBD_GetString((uint8_t*)USBD_CDC_STRING, USBD_StrDesc);
			outBuff = USBD_StrDesc;
	      break;

		default:
			USBD_CtlError();
			return;
		}
		break;

		default:
			USBD_CtlError();
			return;
	}

	/*
	if (req.Length != 0U) {
		if (outBuffSize != 0U) {
			outBuffSize = std::min(outBuffSize, static_cast<uint32_t>(req.Length));
			USBD_CtlSendData(pdev, pbuf, outBuffSize);		// Ends up in USB_EPStartXfer
		} else {
			USBD_CtlError(pdev, req);
		}
	} else {
		USBD_CtlSendStatus(pdev);
	}
*/

	if ((outBuffSize != 0U) && (req.Length != 0U)) {

#if (USB_DEBUG)
		usbDebug[usbDebugNo].PacketSize = outBuffSize;
		usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)outBuff)[0];
		usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)outBuff)[1];
#endif

		ep0_state = USBD_EP0_DATA_IN;
		outBuffSize = std::min(outBuffSize, static_cast<uint32_t>(req.Length));
		USB_EPStartXfer(Direction::in, 0, outBuffSize);
	}

	if (req.Length == 0U) {
		USB_EPStartXfer(Direction::in, 0, 0);
	}
}

uint32_t USBHandler::USBD_GetString(const uint8_t* desc, uint8_t *unicode) {
	uint32_t idx = 2;

	if (desc != NULL) {
		while (*desc != '\0') {
			unicode[idx++] = *desc++;
			unicode[idx++] = 0;
		}
		unicode[0] = idx;
		unicode[1] = USB_DESC_TYPE_STRING;
	}
	return idx;
}

void USBHandler::IntToUnicode(uint32_t value, uint8_t* pbuf, uint8_t len) {

	for (uint8_t idx = 0; idx < len; idx++) {
		if ((value >> 28) < 0xA) {
			pbuf[2 * idx] = (value >> 28) + '0';
		} else {
			pbuf[2 * idx] = (value >> 28) + 'A' - 10;
		}

		value = value << 4;
		pbuf[2 * idx + 1] = 0;
	}
}

void USBHandler::USBD_StdDevReq()
{
	uint8_t dev_addr;
	switch (req.mRequest & USB_REQ_TYPE_MASK)
	{
	case USB_REQ_TYPE_CLASS:
	case USB_REQ_TYPE_VENDOR:
		// pdev->pClass->Setup(pdev, req);
		break;

	case USB_REQ_TYPE_STANDARD:

		switch (req.Request)
		{
		case USB_REQ_GET_DESCRIPTOR:
			USBD_GetDescriptor();
			break;

		case USB_REQ_SET_ADDRESS:
			dev_addr = static_cast<uint8_t>(req.Value) & 0x7FU;
			/*
			USB->DADDR &= ~(USB_OTG_DCFG_DAD);
			USBx_DEVICE->DCFG |= (static_cast<uint32_t>(dev_addr) << 4) & USB_OTG_DCFG_DAD;
			*/
			ep0_state = USBD_EP0_STATUS_IN;
			USB_EPStartXfer(Direction::in, 0, 0);
			dev_state = USBD_STATE_ADDRESSED;
			break;

		case USB_REQ_SET_CONFIGURATION:
			if (dev_state == USBD_STATE_ADDRESSED) {
				dev_state = USBD_STATE_CONFIGURED;

				USB_ActivateEndpoint(CDC_In,   Direction::in,  Bulk);			// Activate CDC in endpoint
				USB_ActivateEndpoint(CDC_Out,  Direction::out, Bulk);			// Activate CDC out endpoint
				USB_ActivateEndpoint(CDC_Cmd,  Direction::in,  Interrupt);		// Activate Command IN EP
				//USB_ActivateEndpoint(MIDI_In,  Direction::in,  Bulk);			// Activate MIDI in endpoint
				//USB_ActivateEndpoint(MIDI_Out, Direction::out, Bulk);			// Activate MIDI out endpoint

				//USB_EPStartXfer(Direction::out, req.Value, 0x40);		// FIXME maxpacket is 2 for EP 1: CUSTOM_HID_EPIN_SIZE, 0x40 = CDC_DATA_FS_OUT_PACKET_SIZE
				ep0_state = USBD_EP0_STATUS_IN;
				USB_EPStartXfer(Direction::in, 0, 0);
			}
			break;

		/*
		case USB_REQ_GET_CONFIGURATION:
			// USBD_GetConfig (pdev, req);
			break;

		case USB_REQ_GET_STATUS:
			//USBD_GetStatus (pdev, req);
			break;

		case USB_REQ_SET_FEATURE:
			//USBD_SetFeature (pdev, req);
			break;

		case USB_REQ_CLEAR_FEATURE:
			//USBD_ClrFeature (pdev, req);
			break;
*/

		default:
			USBD_CtlError();
			break;
		}
		break;

		default:
			USBD_CtlError();
			break;
	}

}

void USBHandler::USBD_CtlError() {
	/*
	USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
	*/
}


bool USBHandler::USB_ReadInterrupts(uint32_t interrupt) {

#if (USB_DEBUG)
	if ((USB->ISTR & interrupt) == interrupt && usbDebugEvent < USB_DEBUG_COUNT) {
		usbDebugNo = usbDebugEvent % USB_DEBUG_COUNT;
		usbDebug[usbDebugNo].eventNo = usbDebugEvent;
		usbDebug[usbDebugNo].Interrupt = USB->ISTR;
		usbDebugEvent++;
	}
#endif

	return (USB->ISTR & interrupt) == interrupt;
}

void USBHandler::SendData(const uint8_t* data, uint16_t len, uint8_t endpoint) {
	if (dev_state == USBD_STATE_CONFIGURED) {
		if (!transmitting) {
			transmitting = true;
			outBuff = (uint8_t*)data;
			outBuffSize = len;
			outBuffCount = 0;
			ep0_state = USBD_EP0_DATA_IN;
			USB_EPStartXfer(Direction::in, endpoint, len);
		}
	}
}
void USBHandler::SendString(const char* s) {
	uint16_t counter = 0;
	while (transmitting && counter < 10000) {
		++counter;
	}
	SendData((uint8_t*)s, strlen(s), CDC_In);
}
void USBHandler::SendString(std::string s) {
	SendString(s.c_str());
}


#if (USB_DEBUG)

//std::string IntToString(const int32_t& v) {
//	std::stringstream ss;
//	ss << v;
//	return ss.str();
//}
/*
std::string HexToString(const uint32_t& v, const bool& spaces) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << v;
	if (spaces) {
		//std::string s = ss.str();
		return ss.str().insert(2, " ").insert(5, " ").insert(8, " ");
	}
	return ss.str();
}

std::string HexByte(const uint16_t& v) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << v;
	return ss.str();
}

void USBHandler::OutputDebug() {
	USBDebug = false;

	SendString("Event,Interrupt,Desc,Int Data,Desc,Endpoint,mRequest,Request,Value,Index,Length,PacketSize,XferBuff0,XferBuff1,\n");
	uint16_t evNo = usbDebugEvent % USB_DEBUG_COUNT;
	std::string interrupt, subtype;
	for (int i = 0; i < USB_DEBUG_COUNT; ++i) {
		switch (usbDebug[evNo].Interrupt) {
		case USB_OTG_GINTSTS_RXFLVL:
			interrupt = "RXFLVL";

			switch ((usbDebug[evNo].IntData & USB_OTG_GRXSTSP_PKTSTS) >> 17) {
			case STS_DATA_UPDT:			// 2 = OUT data packet received
				subtype = "Out packet rec";
				break;
			case STS_XFER_COMP:			// 3 = Transfer completed
				subtype = "Transfer completed";
				break;
			case STS_SETUP_UPDT:		// 6 = SETUP data packet received
				subtype = "Setup packet rec";
				break;
			case STS_SETUP_COMP:		// 4 = SETUP comp
				subtype = "Setup comp";
				break;
			default:
				subtype = "";
			}

			break;
		case USB_OTG_GINTSTS_SRQINT:
			interrupt = "SRQINT";
			break;
		case USB_OTG_GINTSTS_USBSUSP:
			interrupt = "USBSUSP";
			break;
		case USB_OTG_GINTSTS_WKUINT:
			interrupt = "WKUINT";
			break;
		case USB_OTG_GINTSTS_USBRST:
			interrupt = "USBRST";
			break;
		case USB_OTG_GINTSTS_ENUMDNE:
			interrupt = "ENUMDNE";
			break;
		case USB_OTG_GINTSTS_OEPINT:
			interrupt = "OEPINT";

			switch (usbDebug[evNo].IntData) {
			case USB_OTG_DOEPINT_XFRC:
				subtype = "Transfer completed";
				break;
			case USB_OTG_DOEPINT_STUP:
				if (usbDebug[evNo].Request.Request == 6) {
					switch (usbDebug[evNo].Request.Value >> 8)	{
					case USB_DESC_TYPE_DEVICE:
						subtype = "Get Device Descriptor";
						break;
					case USB_DESC_TYPE_CONFIGURATION:
						subtype = "Get Configuration Descriptor";
						break;
					case USB_DESC_TYPE_BOS:
						subtype = "Get BOS Descriptor";
						break;

					case USB_DESC_TYPE_STRING:

						switch ((uint8_t)(req.Value)) {
						case USBD_IDX_LANGID_STR:			// 300
							subtype = "Get Lang Str Descriptor";
							break;
						case USBD_IDX_MFC_STR:				// 301
							subtype = "Get Manufacturor Str Descriptor";
							break;
						case USBD_IDX_PRODUCT_STR:			// 302
							subtype = "Get Product Str Descriptor";
							break;
						case USBD_IDX_SERIAL_STR:			// 303
							subtype = "Get Serial Str Descriptor";
							break;
					    case USBD_IDX_CDC_STR:				// 304
							subtype = "Get CDC Str Descriptor";
					      break;
						}

					default:
						subtype = "Get Descriptor";
					}
				} else if (usbDebug[evNo].Request.Request == 5) {
					subtype = "Set Address to " + std::to_string(usbDebug[evNo].Request.Value);
				} else if (usbDebug[evNo].Request.Request == 9) {
					subtype = "SET_CONFIGURATION";
				} else {
					subtype = "Setup phase done";
				}
				break;
			default:
				subtype = "";
			}
			break;
		case USB_OTG_GINTSTS_IEPINT:
			interrupt = "IEPINT";

				switch (usbDebug[evNo].IntData) {
				case USB_OTG_DIEPINT_XFRC:
					subtype = "Transfer completed";
					break;
				case USB_OTG_DIEPINT_TXFE:
					subtype = "Transmit FIFO empty";
					break;
				default:
					subtype = "";
				}

			break;
		default:
			interrupt = "";
		}

		if (usbDebug[evNo].Interrupt != 0) {
			SendString(std::to_string(usbDebug[evNo].eventNo) + ","
					+ HexToString(usbDebug[evNo].Interrupt, false) + "," + interrupt + ","
					+ HexToString(usbDebug[evNo].IntData, false) + "," + subtype + ","
					+ std::to_string(usbDebug[evNo].endpoint) + ","
					+ HexByte(usbDebug[evNo].Request.mRequest) + ", "
					+ HexByte(usbDebug[evNo].Request.Request) + ", "
					+ HexByte(usbDebug[evNo].Request.Value) + ", "
					+ HexByte(usbDebug[evNo].Request.Index) + ", "
					+ HexByte(usbDebug[evNo].Request.Length) + ", "
					+ HexByte(usbDebug[evNo].PacketSize) + ", "
					+ HexToString(usbDebug[evNo].xferBuff0, false) + ", "
					+ HexToString(usbDebug[evNo].xferBuff1, false) + "\n");
		}
		evNo = (evNo + 1) % USB_DEBUG_COUNT;
	}
}
*/

/* startup sequence:
0		40000000 SRQINT 	Session request/new session detected
1		800		USBSUSP 	USB suspend
2		80000000 WKUINT 	Resume/remote wakeup detected
3		1000	USBRST 		USB reset
4		2000	ENUMDNE 	Enumeration done
5		10 		RXFLVL
6  		10
7		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,100,0,40		Device descriptor USBD_FS_DeviceDesc
8		40000	IEPINT  	USB_OTG_DIEPINT_TXFE  Transmit FIFO empty
9		40000	IEPINT  	USB_OTG_DIEPINT_XFRC  Transfer completed
10 		10
11 		10
12		80000				USB_OTG_DOEPINT_XFRC
13		10					Address setup happens here
14 		10
15		80000	OEPINT		USB_OTG_DOEPINT_STUP req 0x0,5,31,0				Address setup - third param is address (0x31 in this case)
16		40000	IEPINT		USB_OTG_DIEPINT_XFRC
17		10					STS_SETUP_UPDT
18		10					STS_SETUP_COMP
19		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,100,12			Device descriptor again but with device address (rather than 0)
20		40000	IEPINT		USB_OTG_DIEPINT_TXFE
21		40000	IEPINT		USB_OTG_DIEPINT_XFRC
22		10					STS_DATA_UPDT
23		10					STS_SETUP_UPDT
24		80000	OEPINT 		USB_OTG_DOEPINT_XFRC
25		10					STS_SETUP_UPDT
26		10					STS_SETUP_COMP
27		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,200,0,FF		configuration descriptor usbd_audio_CfgDesc
28		40000	IEPINT		USB_OTG_DIEPINT_TXFE
29		40000	IEPINT		USB_OTG_DIEPINT_XFRC
30		40000	IEPINT		USB_OTG_DIEPINT_TXFE 							second part of configuration descriptor
31		40000	IEPINT		USB_OTG_DIEPINT_XFRC
32		10					STS_DATA_UPDT
33		10					STS_SETUP_UPDT
34		80000	OEPINT 		USB_OTG_DOEPINT_XFRC
35		10					STS_SETUP_UPDT
36		10					STS_SETUP_COMP
37		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,F00,0,FF		USBD_FS_BOSDesc
38		40000	IEPINT		USB_OTG_DIEPINT_TXFE
39		40000	IEPINT		USB_OTG_DIEPINT_XFRC
40		10					STS_DATA_UPDT
41		10					STS_SETUP_UPDT
42		80000	OEPINT 		USB_OTG_DOEPINT_XFRC
43		10					STS_SETUP_UPDT
44		10					STS_SETUP_COMP
45		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,303,409,FF		USBD_IDX_SERIAL_STR
46		40000	IEPINT		USB_OTG_DIEPINT_TXFE
47		40000	IEPINT		USB_OTG_DIEPINT_XFRC
48		10					STS_DATA_UPDT
49		10					STS_SETUP_UPDT
50		80000	OEPINT 		USB_OTG_DOEPINT_XFRC
51		10					STS_SETUP_UPDT
52		10					STS_SETUP_COMP
53		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,300,0,FF		USBD_IDX_LANGID_STR
54		40000	IEPINT		USB_OTG_DIEPINT_TXFE
55		40000	IEPINT		USB_OTG_DIEPINT_XFRC
56		10					STS_DATA_UPDT
57		10					STS_SETUP_UPDT
58		80000	OEPINT 		USB_OTG_DOEPINT_XFRC
59		10					STS_SETUP_UPDT
60		10					STS_SETUP_COMP
61		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,302,409,FF		USBD_IDX_PRODUCT_STR
62		40000	IEPINT		USB_OTG_DIEPINT_TXFE
63		40000	IEPINT		USB_OTG_DIEPINT_XFRC
64		10					STS_DATA_UPDT
65		10					STS_SETUP_UPDT
66		80000	OEPINT 		USB_OTG_DOEPINT_XFRC
67		10					STS_SETUP_UPDT
68		10					STS_SETUP_COMP
69		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,600,A			USB_DESC_TYPE_DEVICE_QUALIFIER > Stall
70		10					STS_DATA_UPDT
71		10					STS_SETUP_UPDT
72		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0,9,1,0,0 				Set configuration to 1
73		40000	IEPINT		USB_OTG_DIEPINT_XFRC
74		10					STS_DATA_UPDT
75		10					STS_SETUP_UPDT
76		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,302,409,4		USBD_IDX_PRODUCT_STR
77		40000	IEPINT		USB_OTG_DIEPINT_TXFE
78		40000	IEPINT		USB_OTG_DIEPINT_XFRC
79		10					STS_DATA_UPDT
80		10					STS_SETUP_UPDT
81		80000	OEPINT 		USB_OTG_DOEPINT_XFRC
82		10					STS_SETUP_UPDT reads
83		10					STS_SETUP_COMP
84		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,302,409,1C		USBD_IDX_PRODUCT_STR
92		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,302,409,1C		USBD_IDX_PRODUCT_STR
100		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,302,409,1C		USBD_IDX_PRODUCT_STR
108		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,600,A 			USB_DESC_TYPE_DEVICE_QUALIFIER > Stall
111		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,300,0,1FE		USBD_IDX_LANGID_STR
119		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,301,409,1FE		USBD_IDX_MFC_STR
127		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,302,409,1FE		USBD_IDX_PRODUCT_STR
135		80000	OEPINT 		USB_OTG_DOEPINT_STUP req 0x80,6,3EE,409,1FE		Custom user string?
*/
#endif


