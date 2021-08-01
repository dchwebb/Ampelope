#include "USB.h"

bool USBDebug = true;

void USBHandler::USB_ReadPMA(uint16_t wPMABufAddr, uint16_t wNBytes)
{
	uint32_t n = (uint32_t)wNBytes >> 1;
	uint32_t i, temp;
	volatile uint16_t *pdwVal;
	uint8_t *pBuf = (uint8_t *)(&xfer_buff);

	pdwVal = (volatile uint16_t *)(USB_PMAADDR + wPMABufAddr);		// 0x40006018

	for (i = n; i != 0; i--) {
		temp = *pdwVal;
		pdwVal++;
		*pBuf = (uint8_t)((temp >> 0) & 0xFFU);
		pBuf++;
		*pBuf = (uint8_t)((temp >> 8) & 0xFFU);
		pBuf++;
	}

	if ((wNBytes % 2U) != 0) {
		temp = *pdwVal;
		*pBuf = (uint8_t)((temp >> 0) & 0xFFU);
	}
}

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


void USBHandler::ProcessSetupPacket()
{
	req.loadData(reinterpret_cast<uint8_t*>(&xfer_buff));		// Parse the setup request into the req object

#if (USB_DEBUG)
	usbDebug[usbDebugNo].Request = req;
#endif
	// Previously USBD_StdDevReq
	if ((req.mRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE && (req.mRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD) {
		switch (req.Request) {
		case USB_REQ_GET_DESCRIPTOR:
			USBD_GetDescriptor();
			break;

		case USB_REQ_SET_ADDRESS:
			dev_address = static_cast<uint8_t>(req.Value) & 0x7FU;

			ep0_state = USBD_EP0_STATUS_IN;
			USB_EPStartXfer(Direction::in, 0, 0);
			dev_state = USBD_STATE_ADDRESSED;
			break;

		case USB_REQ_SET_CONFIGURATION:
			if (dev_state == USBD_STATE_ADDRESSED) {
				dev_state = USBD_STATE_CONFIGURED;

				USB_ActivateEndpoint(CDC_In,  Direction::in,  Bulk,      0xC0);			// Activate CDC in endpoint
				USB_ActivateEndpoint(CDC_Out, Direction::out, Bulk,      0x110);		// Activate CDC out endpoint
				USB_ActivateEndpoint(CDC_Cmd, Direction::in,  Interrupt, 0x100);		// Activate Command IN EP
				//USB_ActivateEndpoint(MIDI_In,  Direction::in,  Bulk);			// Activate MIDI in endpoint
				//USB_ActivateEndpoint(MIDI_Out, Direction::out, Bulk);			// Activate MIDI out endpoint

				ep0_state = USBD_EP0_STATUS_IN;
				USB_EPStartXfer(Direction::in, 0, 0);
			}
			break;

		default:
			USBD_CtlError();
			break;
		}

	// Previously USBD_StdItfReq
	} else if ((req.mRequest & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE && (req.mRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS) {
		if (req.Length != 0) {
			if ((req.mRequest & USB_REQ_DIRECTION_MASK) != 0)	{		// Device to host
				// CDC request 0xA1, 0x21, 0x0, 0x0, 0x7		GetLineCoding 0xA1 0x21 0 Interface 7; Data: Line Coding Data Structure
				// 0xA1 [1|01|00001] Device to host | Class | Interface
				outBuffSize = req.Length;
				outBuff = (uint8_t*)&USBD_CDC_LineCoding;
				ep0_state = USBD_EP0_DATA_IN;

				USB_EPStartXfer(Direction::in, 0, req.Length);
			} else {
				//CDC request 0x21, 0x20, 0x0, 0x0, 0x7
				// 0x21 [0|01|00001] Host to device | Class | Interface
				CmdOpCode = req.Request;
				USB_EPStartXfer(Direction::out, 0, req.Length);
			}
		} else {
			// 0x21, 0x22, 0x0, 0x0, 0x0	SetControlLineState 0x21 | 0x22 | 2 | Interface | 0 | None
			// 0x21, 0x20, 0x0, 0x0, 0x0	SetLineCoding       0x21 | 0x20 | 0 | Interface | 0 | Line Coding Data Structure
			USB_EPStartXfer(Direction::in, 0, 0);
		}
	} else {
		USBD_CtlError();
	}
}


// USB_EPStartXfer setup and starts a transfer over an EP
void USBHandler::USB_EPStartXfer(Direction direction, uint8_t endpoint, uint32_t xfer_len)
{
	uint32_t len;
	uint8_t ep_index = (endpoint & 0xF);

	if (direction == Direction::in) {		// IN endpoint
		// Multi packet transfer
		if (xfer_len > ep_maxPacket) {
			len = ep_maxPacket;
		} else {
			len = xfer_len;
		}

		//USB_WritePMA(0x58, len);
		USB_WritePMA(USB_PMA[ep_index].ADDR0_TX, len);
		USB_PMA[ep_index].COUNT0_TX = len;

		PCD_SET_EP_TX_STATUS(USB, ep_index, USB_EP_TX_VALID);
	} else {								// OUT endpoint
/*
		// Multi packet transfer
		if (xfer_len > ep_maxPacket) {
			len = ep_maxPacket;
			xfer_len -= len;
		} else {
			len = xfer_len;
			xfer_len = 0;
		}

		// There doesn't seem to be much point changing this as we can just leave it defaulted to the maximum endpoint size
		// configure and validate Rx endpoint
		//PCD_SET_EP_RX_CNT(USBx, ep->num, len);
		if (len == 0) {
//			USB_PMA->COUNT0_RX &= ~USB_COUNT0_RX_NUM_BLOCK;					// configure block size = 1 (32 Bytes); number of blocks = 2 (64 bytes)
		} else {
//			USB_PMA->COUNT0_RX |= (1 << USB_COUNT0_RX_NUM_BLOCK_Pos);		// configure block size = 1 (32 Bytes); number of blocks = 2 (64 bytes)
		}
*/
		PCD_SET_EP_RX_STATUS(USB, 0, USB_EP_RX_VALID);
	}
}


void USBHandler::USBInterruptHandler() {		// In Drivers\STM32F4xx_HAL_Driver\Src\stm32f4xx_hal_pcd.c

	uint16_t wIstr, wEPVal;
	int epint;

	// Handle spurious interrupt
	USB->ISTR &= ~(USB_ISTR_SOF | USB_ISTR_ESOF);
	if ((USB->ISTR) == 0) {
		return;
	}


	/////////// 	8000 		USB_ISTR_CTR: Correct Transfer
	while (USB_ReadInterrupts(USB_ISTR_CTR)) {				// Originally PCD_EP_ISR_Handler
		uint8_t epindex;		wIstr = USB->ISTR;
		epindex = wIstr & USB_ISTR_EP_ID;				// extract highest priority endpoint number

		if (usbDebugNo == 43) {
			int susp = 1;
		}

		if (epindex == 0) {
			if ((wIstr & USB_ISTR_DIR) == 0) {			// DIR = 0: Direction IN
				PCD_CLEAR_TX_EP_CTR(USB, 0);

				//PCD_SET_EP_TX_STATUS(USB, 0, USB_EP_TX_VALID);
				xfer_count = USB_PMA->COUNT0_TX & USB_COUNT0_TX_COUNT0_TX_Msk;
				//xfer_buff += xfer_count;
				outBuff += xfer_count;

				if (ep0_state == USBD_EP0_DATA_IN) {
					if (xfer_rem > ep_maxPacket) {
						xfer_rem -= ep_maxPacket;
						USB_EPStartXfer(Direction::in, 0, xfer_rem);
						USB_EPStartXfer(Direction::out, 0, 0);
					} else {
						// FIXME if (rem_length ==  maxpacket) etc - where non zero size packet and last packet is a multiple of max packet size
						PCD_SET_EP_TX_STATUS(USB, 0, USB_EP_TX_STALL);
						USB_EPStartXfer(Direction::out, 0, 0);
					}
				}

				if (dev_address > 0 && xfer_count == 0) {
					USB->DADDR = (dev_address | USB_DADDR_EF);
					dev_address = 0;
				}

			} else {									// DIR = 1: Setup or OUT interrupt

				if ((USB->EP0R & USB_EP_SETUP) != 0) {
					xfer_count = USB_PMA->COUNT0_RX & USB_COUNT0_RX_COUNT0_RX_Msk;
					USB_ReadPMA(0x18, xfer_count);		// Read setup data into xfer_buff

					PCD_CLEAR_RX_EP_CTR(USB, 0);		// clears 8000 interrupt

					ProcessSetupPacket();				// Parse setup packet into request, locate data (eg descriptor) and populate TX buffer

				} else if ((USB->EP0R & USB_EP_CTR_RX) != 0) {
					PCD_CLEAR_RX_EP_CTR(USB, 0);

					xfer_count = USB_PMA->COUNT0_RX & USB_COUNT0_RX_COUNT0_RX;
					// Get Control Data OUT Packet
					if ((xfer_count != 0) && (xfer_buff != 0)) {
						USB_ReadPMA(0x18, xfer_count);

						// In CDC mode after 0x21 0x20 packets (line coding commands)
						if (dev_state == USBD_STATE_CONFIGURED && CmdOpCode != 0) {
							if (CmdOpCode == 0x20) {			// SET_LINE_CODING - capture the data passed to return when queried with GET_LINE_CODING
								USBD_CDC_LineCoding = *(USBD_CDC_LineCodingTypeDef*)xfer_buff;
							}
							USB_EPStartXfer(Direction::in, 0, 0);
							CmdOpCode = 0;
						}

						//xfer_buff += xfer_count;

						/* Process Control Data OUT Packet */
						//HAL_PCD_DataOutStageCallback(hpcd, 0U);
					}

					if ((USB->EP0R & USB_EP_SETUP) == 0) {
						// FIXME: not using correct calculations for number of blocks
						//PCD_SET_EP_RX_CNT(hpcd->Instance, PCD_ENDP0, ep->maxpacket);

						//							USB_PMA->COUNT0_RX |= (1 << USB_COUNT0_RX_NUM_BLOCK_Pos);		// configure block size = 1 (32 Bytes); number of blocks = 2 (64 bytes)
						PCD_SET_EP_RX_STATUS(USB, 0, USB_EP_RX_VALID);
					}

				}
			}
		} else {

			if ((USB_EPR[epindex].EPR & USB_EP_CTR_RX) != 0) {

				PCD_CLEAR_RX_EP_CTR(USB, epindex);									// clear interrupt flag

				xfer_count = USB_PMA[epindex].COUNT0_RX & USB_COUNT0_RX_COUNT0_RX;

				if (xfer_count != 0) {
					USB_ReadPMA(USB_PMA[epindex].ADDR0_RX, xfer_count);
				}

				// multi-packet on the NON control OUT endpoint
				//xfer_count += count;
				//xfer_buff += count;

				uint8_t xfer_len = 0;		// FIXME not sure where this is set

				if ((xfer_len == 0) || (xfer_count < ep_maxPacket)) {							// RX COMPLETE
					cdcDataHandler((uint8_t*)xfer_buff, xfer_count);
					PCD_SET_EP_RX_STATUS(USB, epindex, USB_EP_RX_VALID);

				} else {
					//USB_EPStartXfer(hpcd->Instance, ep);
				}
			}

			if ((USB_EPR[epindex].EPR & USB_EP_CTR_TX) != 0) {
				//ep = &hpcd->IN_ep[epindex];
				transmitting = false;
				// clear int flag
				PCD_CLEAR_TX_EP_CTR(USB, epindex);

				uint16_t TxByteNbre = USB_PMA[epindex].COUNT0_TX & USB_COUNT0_TX_COUNT0_TX;

				transmitting = false;

				if (outBuffSize > TxByteNbre) {
					outBuffSize -= TxByteNbre;
					// Transfer is not yet Done
					outBuff += TxByteNbre;
					//ep->xfer_count += TxByteNbre;
					//USB_EPStartXfer(hpcd->Instance, ep);
					USB_EPStartXfer(Direction::in, epindex, outBuffSize);
				} else {
					outBuffSize = 0;
					// TX COMPLETE
					//HAL_PCD_DataInStageCallback(hpcd, ep->num);
				}

			}

		}
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

		USB_ActivateEndpoint(0, Direction::out, Control, 0x18);
		USB_ActivateEndpoint(0, Direction::in, Control, 0x58);

		USB->DADDR = USB_DADDR_EF;								// Enable endpoint and set address to 0
	}

	/////////// 	400 		RESET: Reset Interrupt
	if (USB_ReadInterrupts(USB_ISTR_ERR)) {
		USB->ISTR &= ~USB_ISTR_ERR;
	}
}



void USBHandler::InitUSB()
{

	RCC->CRRCR |= RCC_CRRCR_HSI48ON;					// Enable Internal High Speed oscillator for USB
	while ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0);		// Wait till internal USB oscillator is ready
	RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN;				// USB2OTG (OTG_HS2) Peripheral Clocks Enable

	NVIC_SetPriority(USB_LP_IRQn, 3);
	NVIC_EnableIRQ(USB_LP_IRQn);

	uint16_t winterruptmask = USB_CNTR_CTRM  | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_RESETM | USB_CNTR_L1REQM;

	USB->CNTR = USB_CNTR_FRES;							// Force USB Reset
	USB->BTABLE = 0;									// Set Buffer table Address BTABLE_ADDRESS
	USB->ISTR = 0;										// Clear pending interrupts
	USB->CNTR = winterruptmask;
	USB->BCDR |= (uint16_t)USB_BCDR_DPPU;				// Connect internal PU resistor on USB DP line
}


void USBHandler::USB_ActivateEndpoint(uint8_t endpoint, Direction direction, EndPointType eptype, uint16_t pmaAddress)
{
	endpoint = endpoint & 0xF;
	uint16_t ep_type;
	switch (eptype) {
		case Control:		ep_type = USB_EP_CONTROL;		break;
		case Isochronous:	ep_type = USB_EP_ISOCHRONOUS;	break;
		case Bulk:			ep_type = USB_EP_BULK;			break;
		case Interrupt:		ep_type = USB_EP_INTERRUPT;		break;
	}

	// Set the address (EA=endpoint) and type (EP_TYPE=ep_type)
	USB_EPR[endpoint].EPR = (USB_EPR[endpoint].EPR & USB_EP_T_MASK) | (endpoint | ep_type | USB_EP_CTR_RX | USB_EP_CTR_TX);

	if (direction == Direction::in) {
		USB_PMA[endpoint].ADDR0_TX = pmaAddress;						// Offset of PMA used for EP0 TX

		PCD_CLEAR_TX_DTOG(USB, endpoint);
		PCD_SET_EP_TX_STATUS(USB, endpoint, USB_EP_TX_NAK);
	} else {
		USB_PMA[endpoint].ADDR0_RX = pmaAddress;						// Offset of PMA used for EP0 RX
		USB_PMA[endpoint].COUNT0_RX = (1 << USB_COUNT0_RX_BLSIZE_Pos) | (1 << USB_COUNT0_RX_NUM_BLOCK_Pos);		// configure block size = 1 (32 Bytes); number of blocks = 2 (64 bytes)

		PCD_CLEAR_RX_DTOG(USB, 0);
		PCD_SET_EP_RX_STATUS(USB, endpoint, USB_EP_RX_VALID);
	}

}


// Descriptors in usbd_desc.c
void USBHandler::USBD_GetDescriptor()
{
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

	if ((outBuffSize != 0) && (req.Length != 0)) {

#if (USB_DEBUG)
		usbDebug[usbDebugNo].PacketSize = outBuffSize;
		usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)outBuff)[0];
		usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)outBuff)[1];
#endif

		ep0_state = USBD_EP0_DATA_IN;
		xfer_rem = outBuffSize;		// See USBD_CtlSendData
		outBuffSize = std::min(outBuffSize, static_cast<uint32_t>(req.Length));
		USB_EPStartXfer(Direction::in, 0, outBuffSize);
	}

	if (req.Length == 0) {
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



void USBHandler::USBD_CtlError() {
	PCD_SET_EP_TX_STATUS(USB, 0, USB_EP_TX_STALL);
	//PCD_SET_EP_RX_STATUS(USB, 0, USB_EP_TX_STALL);
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

void USBHandler::OutputDebug() {
	USBDebug = false;

	uartSendString("Event,Interrupt,Desc,Int Data,Desc,Endpoint,mRequest,Request,Value,Index,Length,PacketSize,XferBuff0,XferBuff1,\n");
	uint16_t evNo = usbDebugEvent % USB_DEBUG_COUNT;
	std::string interrupt, subtype;
	for (int i = 0; i < USB_DEBUG_COUNT; ++i) {
		if ((usbDebug[evNo].Interrupt & USB_ISTR_CTR) == USB_ISTR_CTR) {
			if ((usbDebug[evNo].Interrupt & USB_ISTR_DIR) == USB_ISTR_DIR) {
				interrupt = "CTR_OUT";
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

						switch ((uint8_t)(usbDebug[evNo].Request.Value & 0xFF)) {
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
						break;
					default:
						subtype = "Get Descriptor";
					}
				} else if (usbDebug[evNo].Request.Request == 5) {
					subtype = "Set Address to " + std::to_string(usbDebug[evNo].Request.Value);
				} else if (usbDebug[evNo].Request.Request == 9) {
					subtype = "SET_CONFIGURATION";
				} else if ((usbDebug[evNo].Request.mRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS) {
					switch (usbDebug[evNo].Request.Request) {
					case 0x20:
						subtype = "CDC: Set Line Coding";
						break;
					case 0x21:
						subtype = "CDC: Get Line Coding";
						break;
					case 0x22:
						subtype = "CDC: Set Control Line State";
						break;
					}
				} else {
					subtype = "";
				}
			} else {
				interrupt = "CTR_IN";
				subtype = "";
			}
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_SUSP) == USB_ISTR_SUSP) {
			interrupt = "SUSP";
		}

		if ((usbDebug[evNo].Interrupt & USB_ISTR_WKUP) == USB_ISTR_WKUP) {
			interrupt = "WKUP";
		}


		if ((usbDebug[evNo].Interrupt & USB_ISTR_RESET) == USB_ISTR_RESET) {
			interrupt = "RESET";
		}


		if (usbDebug[evNo].Interrupt != 0) {
			uartSendString(std::to_string(usbDebug[evNo].eventNo) + ","
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


