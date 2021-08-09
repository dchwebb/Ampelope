#pragma once

#include "initialisation.h"
#include <functional>
#include <cstring>

// Enables capturing of debug data for output over STLink UART on dev boards
#define USB_DEBUG true
#if (USB_DEBUG)
#include "uartHandler.h"
extern bool USBDebug;
#define USB_DEBUG_COUNT 400
#endif


// Declare registers for PMA area
typedef struct {
  volatile uint16_t ADDR_TX;
  volatile uint16_t COUNT_TX;
  volatile uint16_t ADDR_RX;
  volatile uint16_t COUNT_RX;
} USB_PMA_TypeDef;

// Create struct for easy access to endpoint registers
typedef struct {
	volatile uint16_t EPR;
	volatile uint16_t reserved;
} USB_EPR_TypeDef;

#define  USB_PMA  ((USB_PMA_TypeDef*) USB_PMAADDR)
#define  USB_EPR  ((USB_EPR_TypeDef*)(&USB->EP0R))

#define USB_REQ_RECIPIENT_MASK			0x03
#define USB_REQ_DIRECTION_MASK			0x80
#define USB_REQ_TYPE_MASK				0x60
#define EP_ADDR_MASK					0x0F

#define USBD_VID						0x483		// Vendor ID - use STMicro
#define USBD_LANGID						1033		// Language - en-US
#define USBD_PID						22352		// Product ID

#define LOBYTE(x)  (static_cast<uint8_t>(x & 0x00FFU))
#define HIBYTE(x)  (static_cast<uint8_t>((x & 0xFF00U) >> 8))

class USBHandler {
public:
	void USBInterruptHandler();
	void InitUSB();
	void SendData(const uint8_t *data, uint16_t len, uint8_t endpoint);
	void SendString(const char* s);
	void SendString(std::string s);

	std::function<void(uint8_t*,uint32_t)> cdcDataHandler;			// Declare data handler to store incoming CDC data

	bool transmitting;

private:
	static constexpr const char* manufacturerString = "Mountjoy Modular";
	static constexpr const char* productString      = "Mountjoy Ampelope";
	static constexpr const char* cdcString          = "Mountjoy Ampelope CDC";

	enum EndPoint {CDC_In = 0x81, CDC_Out = 0x1, CDC_Cmd = 0x82};
	enum EndPointType {Control = 0, Isochronous = 1, Bulk = 2, Interrupt = 3};
	enum Descriptor {DeviceDescriptor = 0x1, ConfigurationDescriptor = 0x2, StringDescriptor = 0x3, InterfaceDescriptor = 0x4, EndpointDescriptor = 0x5, DeviceQualifierDescriptor = 0x6, IadDescriptor = 0xb, BosDescriptor = 0xF};
	enum RequestRecipient {RequestRecipientDevice = 0x0, RequestRecipientInterface = 0x1, RequestRecipientEndpoint = 0x2};
	enum RequestType {RequestTypeStandard = 0x0, RequestTypeClass = 0x20, RequestTypeVendor = 0x40};
	enum StrDescIndex {LangIDStrIndex = 0, MfcStrIndex = 1, ProductStrIndex = 2, SerialStrIndex = 3, CDCStrIndex = 4};
	enum class Request {GetStatus = 0x0, SetAddress = 0x5, GetDescriptor = 0x6, SetConfiguration = 0x9};
	enum class Direction {in, out};

	void ProcessSetupPacket();
	void ReadPMA(uint16_t wPMABufAddr, uint16_t wNBytes);
	void WritePMA(uint16_t wPMABufAddr, uint16_t wNBytes);
	void ActivateEndpoint(uint8_t endpoint, Direction direction, EndPointType eptype, uint16_t pmaAddress);
	void GetDescriptor();
	void EPStartXfer(Direction direction, uint8_t endpoint, uint32_t xfer_len);
	bool ReadInterrupts(uint32_t interrupt);
	void IntToUnicode(uint32_t value, uint8_t* pbuf, uint8_t len);
	uint32_t GetString(const char* desc);
	void StringToTxBuff(const char* desc);

	static const uint8_t maxPacket = 0x40;
	uint8_t rxBuff[maxPacket] __attribute__ ((aligned (4)));		// Receive data buffer - must be aligned to allow copying to other structures
	uint32_t rxCount;				// Amount of data to receive
	const uint8_t* txBuff;			// Pointer to transmit buffer (for transferring data to IN endpoint)
	uint32_t txBuffSize;			// Size of transmit buffer
	uint32_t txRemaining;			// If transfer is larger than maximum packet size store remaining byte count
	uint8_t cmdOpCode;				// stores class specific operation codes (eg CDC set line config)
	uint8_t devAddress = 0;			// Temporarily hold the device address as it cannot stored in the register until the 0 address response has been handled

	enum class DeviceState {Suspended, Addressed, Configured} devState;

	struct usbRequest {
		uint8_t bmRequest;
		uint8_t bRequest;
		uint16_t wValue;
		uint16_t wIndex;
		uint16_t wLength;

		void loadData(const uint8_t* data) {
			bmRequest = data[0];
			bRequest = data[1];
			wValue = static_cast<uint16_t>(data[2]) + (data[3] << 8);
			wIndex = static_cast<uint16_t>(data[4]) + (data[5] << 8);
			wLength = static_cast<uint16_t>(data[6]) + (data[7] << 8);
		}
	} req;

	struct USBD_CDC_LineCodingTypeDef {
		uint32_t bitrate;    					// Data terminal rate in bits per sec.
		uint8_t format;      					// Stop Bits: 0-1 Stop Bit; 1-1.5 Stop Bits; 2-2 Stop Bits
		uint8_t paritytype;  					// Parity: 0 = None; 1 = Odd; 2 = Even; 3 = Mark; 4 = Space; 6 bDataBits 1 Data bits
		uint8_t datatype;    					// Data bits (5, 6, 7,	8 or 16)
	} USBD_CDC_LineCoding;

	// USB standard device descriptor - in usbd_desc.c
	const uint8_t USBD_FS_DeviceDesc[0x12] = {
			0x12,								// bLength
			DeviceDescriptor,					// bDescriptorType
			0x01,								// bcdUSB  - 0x01 if LPM enabled
			0x02,
			0xEF,								// bDeviceClass: (Miscellaneous)
			0x02,								// bDeviceSubClass (Interface Association Descriptor- with below)
			0x01,								// bDeviceProtocol (Interface Association Descriptor)
			maxPacket,  						// bMaxPacketSize
			LOBYTE(USBD_VID),					// idVendor
			HIBYTE(USBD_VID),					// idVendor
			LOBYTE(USBD_PID),					// idProduct
			HIBYTE(USBD_PID),					// idProduct
			0x00,								// bcdDevice rel. 2.00
			0x02,
			MfcStrIndex,						// Index of manufacturer  string
			ProductStrIndex,					// Index of product string
			SerialStrIndex,						// Index of serial number string
			0x01								// bNumConfigurations
	};

	static const uint8_t cdcDescSize = 75;

	const uint8_t USBD_CDC_CfgFSDesc[cdcDescSize] = {
			// Configuration Descriptor
			0x09,								// bLength: Configuration Descriptor size
			ConfigurationDescriptor,			// bDescriptorType: Configuration
			LOBYTE(cdcDescSize),				// wTotalLength
			HIBYTE(cdcDescSize),
			0x02,								// bNumInterfaces: 2 interfaces
			0x01,								// bConfigurationValue: Configuration value
			0x00,								// iConfiguration: Index of string descriptor describing the configuration
			0xC0,								// bmAttributes: self powered
			0x32,								// MaxPower 0 mA

			//---------------------------------------------------------------------------
	        // IAD Descriptor - Interface association descriptor for CDC class
			0x08,								// bLength (8 bytes)
			IadDescriptor,						// bDescriptorType
			0x00,								// bFirstInterface
			0x02,								// bInterfaceCount
			0x02,								// bFunctionClass (Communications and CDC Control)
			0x02,								// bFunctionSubClass
			0x01,								// bFunctionProtocol
			CDCStrIndex,						// iFunction (String Descriptor 6)

			// Interface Descriptor
			0x09,								// bLength: Interface Descriptor size
			InterfaceDescriptor,				// bDescriptorType: Interface
			0x00,								// bInterfaceNumber: Number of Interface
			0x00,								// bAlternateSetting: Alternate setting
			0x01,								// bNumEndpoints: One endpoints used
			0x02,								// bInterfaceClass: Communication Interface Class
			0x02,								// bInterfaceSubClass: Abstract Control Model
			0x01,								// bInterfaceProtocol: Common AT commands
			CDCStrIndex,						// iInterface

			// Header Functional Descriptor
			0x05,								// bLength: Endpoint Descriptor size
			0x24,								// bDescriptorType: CS_INTERFACE
			0x00,								// bDescriptorSubtype: Header Func Desc
			0x10,								// bcdCDC: spec release number
			0x01,

			// Call Management Functional Descriptor
			0x05,								// bFunctionLength
			0x24,								// bDescriptorType: CS_INTERFACE
			0x01,								// bDescriptorSubtype: Call Management Func Desc
			0x00,								// bmCapabilities: D0+D1
			0x01,								// bDataInterface: 1

			// ACM Functional Descriptor
			0x04,								// bFunctionLength
			0x24,								// bDescriptorType: CS_INTERFACE
			0x02,								// bDescriptorSubtype: Abstract Control Management desc
			0x02,								// bmCapabilities

			// Union Functional Descriptor
			0x05,								// bFunctionLength
			0x24,								// bDescriptorType: CS_INTERFACE
			0x06,								// bDescriptorSubtype: Union func desc
			0x00,								// bMasterInterface: Communication class interface
			0x01,								// bSlaveInterface0: Data Class Interface

			// Endpoint 2 Descriptor
			0x07,								// bLength: Endpoint Descriptor size
			EndpointDescriptor,					// bDescriptorType: Endpoint
			CDC_Cmd,							// bEndpointAddress
			Interrupt,							// bmAttributes: Interrupt
			0x08,								// wMaxPacketSize
			0x00,
			0x10,								// bInterval

			//---------------------------------------------------------------------------

			// Data class interface descriptor
			0x09,								// bLength: Endpoint Descriptor size
			InterfaceDescriptor,				// bDescriptorType:
			0x01,								// bInterfaceNumber: Number of Interface
			0x00,								// bAlternateSetting: Alternate setting
			0x02,								// bNumEndpoints: Two endpoints used
			0x0A,								// bInterfaceClass: CDC
			0x00,								// bInterfaceSubClass:
			0x00,								// bInterfaceProtocol:
			0x00,								// iInterface:

			// Endpoint OUT Descriptor
			0x07,								// bLength: Endpoint Descriptor size
			EndpointDescriptor,					// bDescriptorType: Endpoint
			CDC_Out,							// bEndpointAddress
			Bulk,								// bmAttributes: Bulk
			LOBYTE(maxPacket),					// wMaxPacketSize:
			HIBYTE(maxPacket),
			0x00,								// bInterval: ignore for Bulk transfer

			// Endpoint IN Descriptor
			0x07,								// bLength: Endpoint Descriptor size
			EndpointDescriptor,					// bDescriptorType: Endpoint
			CDC_In,								// bEndpointAddress
			Bulk,								// bmAttributes: Bulk
			LOBYTE(maxPacket),					// wMaxPacketSize:
			HIBYTE(maxPacket),
			0x00								// bInterval: ignore for Bulk transfer
	};



	// Binary Object Store (BOS) Descriptor
	const uint8_t USBD_FS_BOSDesc[12] = {
			0x05,								// Length
			BosDescriptor,						// DescriptorType
			0x0C,								// TotalLength
			0x00, 0x01,							// NumDeviceCaps

			// USB 2.0 Extension Descriptor: device capability
			0x07,								// bLength
			0x10, 								// USB_DEVICE_CAPABITY_TYPE
			0x02,								// Attributes
			0x02, 0x00, 0x00, 0x00				// Link Power Management protocol is supported
	};


	uint8_t USBD_StringSerial[0x1A] = {
			0x1A,								// Length
			StringDescriptor, 					// DescriptorType
	};

	// USB lang indentifier descriptor
	const uint8_t USBD_LangIDDesc[4] = {
			0x04,
			StringDescriptor,
			LOBYTE(USBD_LANGID),
			HIBYTE(USBD_LANGID)
	};

	uint8_t unicodeString[128];

public:
#if (USB_DEBUG)
	uint16_t usbDebugNo = 0;
	uint16_t usbDebugEvent = 0;

	struct usbDebugItem {
		uint16_t eventNo;
		uint32_t Interrupt;
		usbRequest Request;
		uint8_t endpoint;
		uint16_t PacketSize;
		uint32_t xferBuff0;
		uint32_t xferBuff1;
	};
	usbDebugItem usbDebug[USB_DEBUG_COUNT];
	void OutputDebug();
#endif
};
