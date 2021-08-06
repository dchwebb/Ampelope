#include "USB.h"

bool USBDebug = true;		// Used if outputting debug over USB

inline void ClearRxInterrupt(uint8_t ep)
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_EPREG_MASK) & ~USB_EP_CTR_RX;
	USB_EPR[ep].EPR = wRegVal | USB_EP_CTR_TX;
}

inline void ClearTxInterrupt(uint8_t ep)
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_EPREG_MASK) & ~USB_EP_CTR_TX;
	USB_EPR[ep].EPR = wRegVal | USB_EP_CTR_RX;
}


inline void SetTxStatus(uint8_t ep, uint16_t status)		// Set endpoint transmit status - have to use XOR to toggle bits
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_EPTX_DTOGMASK) ^ status;
	USB_EPR[ep].EPR = wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX;
}


inline void SetRxStatus(uint8_t ep, uint16_t status)		// Set endpoint receive status - have to use XOR to toggle bits
{
	uint16_t wRegVal = (USB_EPR[ep].EPR & USB_EPRX_DTOGMASK) ^ status;
	USB_EPR[ep].EPR = wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX;
}


void USBHandler::ReadPMA(uint16_t pma, uint16_t bytes)
{
	volatile uint16_t* pmaBuff = reinterpret_cast<volatile uint16_t*>(USB_PMAADDR + pma);		// Eg 0x40006018

	for (int i = 0; i < (bytes + 1) / 2; i++) {
		reinterpret_cast<volatile uint16_t*>(rxBuff)[i] = *pmaBuff++;				// pma buffer can only be read in 16 bit words
	}

#if (USB_DEBUG)
	usbDebug[usbDebugNo].PacketSize = bytes;
	usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)rxBuff)[0];
	usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)rxBuff)[1];
#endif
}

void USBHandler::WritePMA(uint16_t wPMABufAddr, uint16_t wNBytes)
{
	uint32_t n = ((uint32_t)wNBytes + 1U) >> 1;
	uint32_t i, temp1, temp2;
	volatile uint16_t *pdwVal;
	uint8_t *pBuf = (uint8_t *)txBuff;

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
	req.loadData(rxBuff);		// Parse the setup request into the req object

#if (USB_DEBUG)
	usbDebug[usbDebugNo].Request = req;
#endif
	// Previously USBD_StdDevReq
	if ((req.mRequest & USB_REQ_RECIPIENT_MASK) == RequestRecipientDevice && (req.mRequest & USB_REQ_TYPE_MASK) == RequestTypeStandard) {
		switch (static_cast<Request>(req.Request)) {
		case Request::GetDescriptor:
			GetDescriptor();
			break;

		case Request::SetAddress:
			devAddress = static_cast<uint8_t>(req.Value) & 0x7F;			// Address address is set on the next interrupt - hold in temp storage

			EPStartXfer(Direction::in, 0, 0);
			dev_state = DeviceState::Addressed;
			break;

		case Request::SetConfiguration:
			if (dev_state == DeviceState::Addressed) {
				dev_state = DeviceState::Configured;

				ActivateEndpoint(CDC_In,  Direction::in,  Bulk,      0xC0);			// Activate CDC in endpoint
				ActivateEndpoint(CDC_Out, Direction::out, Bulk,      0x110);		// Activate CDC out endpoint
				ActivateEndpoint(CDC_Cmd, Direction::in,  Interrupt, 0x100);		// Activate Command IN EP
				//USB_ActivateEndpoint(MIDI_In,  Direction::in,  Bulk);			// Activate MIDI in endpoint
				//USB_ActivateEndpoint(MIDI_Out, Direction::out, Bulk);			// Activate MIDI out endpoint

				EPStartXfer(Direction::in, 0, 0);
			}
			break;

		default:
			SetTxStatus(0, USB_EP_TX_STALL);
			break;
		}

	// Previously USBD_StdItfReq
	} else if ((req.mRequest & USB_REQ_RECIPIENT_MASK) == RequestRecipientInterface && (req.mRequest & USB_REQ_TYPE_MASK) == RequestTypeClass) {
		if (req.Length != 0) {
			if ((req.mRequest & USB_REQ_DIRECTION_MASK) != 0)	{		// Device to host
				// CDC request 0xA1, 0x21, 0x0, 0x0, 0x7		GetLineCoding 0xA1 0x21 0 Interface 7; Data: Line Coding Data Structure
				// 0xA1 [1|01|00001] Device to host | Class | Interface
				txBuffSize = req.Length;
				txBuff = (uint8_t*)&USBD_CDC_LineCoding;

				EPStartXfer(Direction::in, 0, req.Length);
			} else {
				//CDC request 0x21, 0x20, 0x0, 0x0, 0x7			 0x21 = [0|01|00001] Host to device | Class | Interface
				cmdOpCode = req.Request;
				EPStartXfer(Direction::out, 0, req.Length);
			}
		} else {
			// 0x21, 0x22, 0x0, 0x0, 0x0	SetControlLineState 0x21 | 0x22 | 2 | Interface | 0 | None
			// 0x21, 0x20, 0x0, 0x0, 0x0	SetLineCoding       0x21 | 0x20 | 0 | Interface | 0 | Line Coding Data Structure
			EPStartXfer(Direction::in, 0, 0);
		}
	} else {
		SetTxStatus(0, USB_EP_TX_STALL);
	}
}


// EPStartXfer setup and starts a transfer over an EP
void USBHandler::EPStartXfer(Direction direction, uint8_t endpoint, uint32_t len)
{
	uint8_t epIndex = (endpoint & 0xF);

	if (direction == Direction::in) {						// IN endpoint
		if (len > maxPacket) {
			len = maxPacket;
		}

		WritePMA(USB_PMA[epIndex].ADDR_TX, len);
		USB_PMA[epIndex].COUNT_TX = len;

#if (USB_DEBUG)
				usbDebug[usbDebugNo].PacketSize = len;
				if (len > 0) {
					usbDebug[usbDebugNo].xferBuff0 = ((uint32_t*)txBuff)[0];
					usbDebug[usbDebugNo].xferBuff1 = ((uint32_t*)txBuff)[1];
				}
#endif

		SetTxStatus(epIndex, USB_EP_TX_VALID);
	} else {												// OUT endpoint
		SetRxStatus(0, USB_EP_RX_VALID);
	}
}


void USBHandler::USBInterruptHandler()						// Originally in Drivers\STM32F4xx_HAL_Driver\Src\stm32f4xx_hal_pcd.c
{
	// Handle spurious interrupt
	USB->ISTR &= ~(USB_ISTR_SOF | USB_ISTR_ESOF);
	if ((USB->ISTR) == 0) {
		return;
	}


	/////////// 	8000 		USB_ISTR_CTR: Correct Transfer
	while (ReadInterrupts(USB_ISTR_CTR)) {					// Originally PCD_EP_ISR_Handler
		uint8_t epIndex = USB->ISTR & USB_ISTR_EP_ID;		// Extract highest priority endpoint number

#if (USB_DEBUG)
		usbDebug[usbDebugNo].endpoint = epIndex;
#endif

		if (epIndex == 0) {
			if ((USB->ISTR & USB_ISTR_DIR) == 0) {			// DIR = 0: Direction IN
				ClearTxInterrupt(0);

				uint16_t txBytes = USB_PMA->COUNT_TX & USB_COUNT0_TX_COUNT0_TX_Msk;
				txBuff += txBytes;

				if (txRemaining > maxPacket) {
					txRemaining -= maxPacket;
					EPStartXfer(Direction::in, 0, txRemaining);
					EPStartXfer(Direction::out, 0, 0);
				} else {
					// FIXME if (rem_length ==  maxpacket) etc - where non zero size packet and last packet is a multiple of max packet size
					SetTxStatus(0, USB_EP_TX_STALL);
					EPStartXfer(Direction::out, 0, 0);
				}

				if (devAddress > 0 && txBytes == 0) {
					USB->DADDR = (devAddress | USB_DADDR_EF);
					devAddress = 0;
				}

			} else {										// DIR = 1: Setup or OUT interrupt

				if ((USB->EP0R & USB_EP_SETUP) != 0) {
					rxCount = USB_PMA->COUNT_RX & USB_COUNT0_RX_COUNT0_RX_Msk;
					ReadPMA(0x18, rxCount);					// Read setup data into rxBuff
					ClearRxInterrupt(0);					// clears 8000 interrupt
					ProcessSetupPacket();					// Parse setup packet into request, locate data (eg descriptor) and populate TX buffer

				} else {
					ClearRxInterrupt(0);
					rxCount = USB_PMA->COUNT_RX & USB_COUNT0_RX_COUNT0_RX;
					if (rxCount != 0) {
						ReadPMA(0x18, rxCount);

						// In CDC mode after 0x21 0x20 packets (line coding commands)
						if (dev_state == DeviceState::Configured && cmdOpCode != 0) {
							if (cmdOpCode == 0x20) {			// SET_LINE_CODING - capture the data passed to return when queried with GET_LINE_CODING
								USBD_CDC_LineCoding = *(reinterpret_cast<USBD_CDC_LineCodingTypeDef*>(rxBuff));
							}
							EPStartXfer(Direction::in, 0, 0);
							cmdOpCode = 0;
						}
					}

					SetRxStatus(0, USB_EP_RX_VALID);
				}
			}

		} else {
			// Non zero endpoint
			if ((USB_EPR[epIndex].EPR & USB_EP_CTR_RX) != 0) {

				ClearRxInterrupt(epIndex);
				rxCount = USB_PMA[epIndex].COUNT_RX & USB_COUNT0_RX_COUNT0_RX;
				if (rxCount != 0) {
					ReadPMA(USB_PMA[epIndex].ADDR_RX, rxCount);
				}
				SetRxStatus(epIndex, USB_EP_RX_VALID);

				cdcDataHandler(rxBuff, rxCount);
			}

			if ((USB_EPR[epIndex].EPR & USB_EP_CTR_TX) != 0) {
				transmitting = false;
				ClearTxInterrupt(epIndex);

				uint16_t txBytes = USB_PMA[epIndex].COUNT_TX & USB_COUNT0_TX_COUNT0_TX;
				if (txBuffSize >= txBytes) {					// Transmitting data larger than buffer size
					txBuffSize -= txBytes;
					txBuff += txBytes;
					EPStartXfer(Direction::in, epIndex, txBuffSize);
				}

			}

		}
	}


	/////////// 	1000 		USB_ISTR_WKUP: Wake Up
	if (ReadInterrupts(USB_ISTR_WKUP)) {
		USB->CNTR &= ~USB_CNTR_FSUSP;
		USB->CNTR &= ~USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_WKUP;
	}

	/////////// 	800 		SUSP: Suspend Interrupt
	if (ReadInterrupts(USB_ISTR_SUSP)) {
		USB->CNTR |= USB_CNTR_FSUSP;
		USB->ISTR &= ~USB_ISTR_SUSP;
		USB->CNTR |= USB_CNTR_LPMODE;
		dev_state = DeviceState::Suspended;
	}

	/////////// 	400 		RESET: Reset Interrupt
	if (ReadInterrupts(USB_ISTR_RESET))	{
		USB->ISTR &= ~USB_ISTR_RESET;

		ActivateEndpoint(0, Direction::out, Control, 0x18);
		ActivateEndpoint(0, Direction::in,  Control, 0x58);

		USB->DADDR = USB_DADDR_EF;						// Enable endpoint and set address to 0
	}

	/////////// 	100 		USB_ISTR_ESOF: Expected Start of frame
	if (ReadInterrupts(USB_ISTR_ESOF)) {
		USB->ISTR &= ~USB_ISTR_ESOF;
	}

	/////////// 	2000 		ERR: Error Interrupt
	if (ReadInterrupts(USB_ISTR_ERR)) {
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

	USB->CNTR = USB_CNTR_FRES;							// Force USB Reset
	USB->BTABLE = 0;									// Set Buffer table Address BTABLE_ADDRESS
	USB->ISTR = 0;										// Clear pending interrupts
	USB->CNTR = USB_CNTR_CTRM  | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_RESETM;
	USB->BCDR |= USB_BCDR_DPPU;							// Connect internal PU resistor on USB DP line
}


void USBHandler::ActivateEndpoint(uint8_t endpoint, Direction direction, EndPointType eptype, uint16_t pmaAddress)
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
		USB_PMA[endpoint].ADDR_TX = pmaAddress;						// Offset of PMA used for EP0 TX

		// Clear tx data toggle (data packets must alternate 1 and 0 in the data field)
		if ((USB_EPR[endpoint].EPR & USB_EP_DTOG_TX) != 0) {
			uint16_t wEPVal = USB_EPR[endpoint].EPR & USB_EPREG_MASK;
			USB_EPR[endpoint].EPR = wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_TX;
		}

		SetTxStatus(endpoint, USB_EP_TX_NAK);

	} else {
		USB_PMA[endpoint].ADDR_RX = pmaAddress;						// Offset of PMA used for EP0 RX
		USB_PMA[endpoint].COUNT_RX = (1 << USB_COUNT0_RX_BLSIZE_Pos) | (1 << USB_COUNT0_RX_NUM_BLOCK_Pos);		// configure block size = 1 (32 Bytes); number of blocks = 2 (64 bytes)

		// Clear rx data toggle
		if ((USB_EPR[endpoint].EPR & USB_EP_DTOG_RX) != 0) {
			uint16_t wEPVal = USB_EPR[endpoint].EPR & USB_EPREG_MASK;
			USB_EPR[endpoint].EPR = wEPVal | USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_RX;
		}

		SetRxStatus(endpoint, USB_EP_RX_VALID);
	}

}


void USBHandler::GetDescriptor()
{
	switch (static_cast<Descriptor>(req.Value >> 8))	{
	case DeviceDescriptor:
		txBuff = USBD_FS_DeviceDesc;
		txBuffSize = sizeof(USBD_FS_DeviceDesc);
		break;

	case ConfigurationDescriptor:
		txBuff = USBD_CDC_CfgFSDesc;
		txBuffSize = sizeof(USBD_CDC_CfgFSDesc);
		break;

	case BosDescriptor:
		txBuff = USBD_FS_BOSDesc;
		txBuffSize = sizeof(USBD_FS_BOSDesc);
		break;

	case StringDescriptor:

		switch ((uint8_t)(req.Value)) {
		case USBD_IDX_LANGID_STR:			// 300
			txBuff = USBD_LangIDDesc;
			txBuffSize = sizeof(USBD_LangIDDesc);
			break;
		case USBD_IDX_MFC_STR:				// 301
			txBuffSize = GetString((uint8_t*)USBD_MANUFACTURER_STRING, USBD_StrDesc);
			txBuff = USBD_StrDesc;
			break;
		case USBD_IDX_PRODUCT_STR:			// 302
			txBuffSize = GetString((uint8_t*)USBD_PRODUCT_STRING, USBD_StrDesc);
			txBuff = USBD_StrDesc;
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
				txBuff = USBD_StringSerial;
				txBuffSize = sizeof(USBD_StringSerial);
			}
			break;
/*
		case USBD_IDX_MIDI_STR:				// 304
			txBuffSize = USBD_GetString((uint8_t*)USBD_MIDI_STRING, USBD_StrDesc);
			txBuff = USBD_StrDesc;
	      break;
*/
	    case USBD_IDX_CDC_STR:				// 304
			txBuffSize = GetString((uint8_t*)USBD_CDC_STRING, USBD_StrDesc);
			txBuff = USBD_StrDesc;
	      break;

		default:
			SetTxStatus(0, USB_EP_TX_STALL);
			return;
		}
		break;

	default:
		SetTxStatus(0, USB_EP_TX_STALL);
		return;
	}

	if ((txBuffSize != 0) && (req.Length != 0)) {
		txRemaining = txBuffSize;
		txBuffSize = std::min(txBuffSize, static_cast<uint32_t>(req.Length));
		EPStartXfer(Direction::in, 0, txBuffSize);
	}

	if (req.Length == 0) {
		EPStartXfer(Direction::in, 0, 0);
	}
}


uint32_t USBHandler::GetString(const uint8_t* desc, uint8_t *unicode)
{
	uint32_t idx = 2;

	if (desc != nullptr) {
		while (*desc != '\0') {
			unicode[idx++] = *desc++;
			unicode[idx++] = 0;
		}
		unicode[0] = idx;
		unicode[1] = StringDescriptor;
	}
	return idx;
}


void USBHandler::IntToUnicode(uint32_t value, uint8_t* pbuf, uint8_t len)
{
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


bool USBHandler::ReadInterrupts(uint32_t interrupt)
{
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


void USBHandler::SendData(const uint8_t* data, uint16_t len, uint8_t endpoint)
{
	if (dev_state == DeviceState::Configured) {
		if (!transmitting) {
			transmitting = true;
			txBuff = (uint8_t*)data;
			txBuffSize = len;
			EPStartXfer(Direction::in, endpoint, len);
		}
	}
}


void USBHandler::SendString(const char* s)
{
	uint16_t counter = 0;
	while (transmitting && counter < 10000) {
		++counter;
	}
	SendData((uint8_t*)s, strlen(s), CDC_In);
}


void USBHandler::SendString(std::string s)
{
	SendString(s.c_str());
}


#if (USB_DEBUG)

void USBHandler::OutputDebug()
{
	USBDebug = false;

	uartSendString("Event,Interrupt,Name,Desc,Endpoint,mRequest,Request,Value,Index,Length,PacketSize,XferBuff,\n");
	uint16_t evNo = usbDebugEvent % USB_DEBUG_COUNT;
	std::string interrupt, subtype;

	for (int i = 0; i < USB_DEBUG_COUNT; ++i) {
		if ((usbDebug[evNo].Interrupt & USB_ISTR_CTR) == USB_ISTR_CTR) {
			if ((usbDebug[evNo].Interrupt & USB_ISTR_DIR) == USB_ISTR_DIR) {
				interrupt = "CTR_OUT";
				if (usbDebug[evNo].Request.Request == 6) {
					switch (static_cast<Descriptor>(usbDebug[evNo].Request.Value >> 8))	{
					case DeviceDescriptor:
						subtype = "Get Device Descriptor";
						break;
					case ConfigurationDescriptor:
						subtype = "Get Configuration Descriptor";
						break;
					case BosDescriptor:
						subtype = "Get BOS Descriptor";
						break;

					case StringDescriptor:

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
				} else if ((usbDebug[evNo].Request.mRequest & USB_REQ_TYPE_MASK) == RequestTypeClass) {
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
					+ HexToString(usbDebug[evNo].Interrupt, false) + ","
					+ interrupt + "," + subtype + ","
					+ std::to_string(usbDebug[evNo].endpoint) + ","
					+ HexByte(usbDebug[evNo].Request.mRequest) + ","
					+ HexByte(usbDebug[evNo].Request.Request) + ","
					+ HexByte(usbDebug[evNo].Request.Value) + ","
					+ HexByte(usbDebug[evNo].Request.Index) + ","
					+ HexByte(usbDebug[evNo].Request.Length) + ","
					+ HexByte(usbDebug[evNo].PacketSize) + ","
					+ HexToString(usbDebug[evNo].xferBuff0, true) + " "
					+ HexToString(usbDebug[evNo].xferBuff1, true) + "\n");
		}
		evNo = (evNo + 1) % USB_DEBUG_COUNT;
	}
}


/* startup sequence:
Event		Interrupt	Desc							mReq	Req	Value	Ind	Len	PacketSize	[XferBuff]
	400		RESET
1	400		RESET
2	800		SUSP
3	1000	WKUP
4	400		RESET
5	8010	CTR_OUT		Get Device Descriptor			80		6	100		0	40	12	[12 01 01 02 EF 02 01 40]
6	8000	CTR_IN
7	8010	CTR_OUT
8	8010	CTR_OUT		Set Address to 52				5		34	0		0
9	8000	CTR_IN
10	8010	CTR_OUT		Get Device Descriptor			80		6	100		0	12	12	[12 01 01 02 EF 02 01 40]
11	8000	CTR_IN
12	8010	CTR_OUT
13	8010	CTR_OUT		Get Configuration Descriptor	80		6	200		0	FF	4B	[09 02 4B 00 02 01 00 C0]
14	8000	CTR_IN
15	8000	CTR_IN
16	8010	CTR_OUT
17	8010	CTR_OUT		Get BOS Descriptor				80		6	F00		0	FF	C	[05 0F 0C 00 01 07 10 02]
18	8000	CTR_IN
19	8010	CTR_OUT
20	8010	CTR_OUT		Get Serial Str Descriptor		80		6	303		409	FF	1A	[1A 03 30 00 30 00 36 00]
21	8000	CTR_IN
22	8010	CTR_OUT
23	8010	CTR_OUT		Get Lang Str Descriptor			80		6	300		0	FF	4	[04 03 09 04]
24	8000	CTR_IN
25	8010	CTR_OUT
26	8010	CTR_OUT		Get Product Str Descriptor		80		6	302		409	FF	26	[26 03 4D 00 6F 00 75 00]
27	8000	CTR_IN
28	8010	CTR_OUT
29	8010	CTR_OUT		Get Descriptor					80		6	600		0	A
30	8010	CTR_OUT		SET_CONFIGURATION				9		1
31	8000	CTR_IN
32	8010	CTR_OUT		Get CDC Str Descriptor			80		6	304		409	4	2E	[2E 03 4D 00 6F 00 75 00]
33	8000	CTR_IN
34	8010	CTR_OUT
35	8010	CTR_OUT		Get CDC Str Descriptor			80		6	304		409	2E	2E	[2E 03 4D 00 6F 00 75 00]
36	8000	CTR_IN
37	8010	CTR_OUT
38	8010	CTR_OUT		Get Lang Str Descriptor			80		6	300		0	FF	4	[04 03 09 04 2E 03 4D 00]
39	8000	CTR_IN
40	8010	CTR_OUT
41	8010	CTR_OUT		Get Manufacturor Str Descriptor	80		6	301		409	FF	22	[22 03 4D 00 6F 00 75 00]
42	8000	CTR_IN
43	8010	CTR_OUT
44	8010	CTR_OUT		CDC: Get Line Coding			A1		21	0		0	7
45	8000	CTR_IN
46	8010	CTR_OUT
47	8010	CTR_OUT		Get Product Str Descriptor		80		6	302		409	FF	26	[26 03 4D 00 6F 00 75 00]
48	8000	CTR_IN
49	8010	CTR_OUT
50	8010	CTR_OUT		CDC: Set Control Line State		21		22
51	8000	CTR_IN
52	8010	CTR_OUT		CDC: Set Line Coding			21		20	0		0	7
53	8010	CTR_OUT
54	8000	CTR_IN
55	8010	CTR_OUT		CDC: Get Line Coding			A1		21	0		0	7
56	8000	CTR_IN
57	8010	CTR_OUT
58	8010	CTR_OUT		Get Descriptor					80		6	600		0	A
59	8010	CTR_OUT		Get Lang Str Descriptor			80		6	300		0	1FE	4	[04 03 09 04 26 03 4D 00]
60	8000	CTR_IN
61	8010	CTR_OUT
62	8010	CTR_OUT		Get Manufacturor Str Descriptor	80		6	301		409	1FE	22	[22 03 4D 00 6F 00 75 00]
63	8000	CTR_IN
64	8010	CTR_OUT
65	8010	CTR_OUT		Get Product Str Descriptor		80		6	302		409	1FE	26	[26 03 4D 00 6F 00 75 00]
66	8000	CTR_IN
67	8010	CTR_OUT
68	8010	CTR_OUT		Get CDC Str Descriptor			80		6	304		409	1FE	2E	[2E 03 4D 00 6F 00 75 00]
69	8000	CTR_IN
70	8010	CTR_OUT
71	8010	CTR_OUT										80		6	305		409	1FE
72	8010	CTR_OUT										80		6	3EE		409	1FE
*/
#endif


