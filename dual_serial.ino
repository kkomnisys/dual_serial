

#include "dual_serial.h"

//Note: To use the secondary usb port, change USB type to USB + MIDI.
#if defined(CDC_STATUS_INTERFACE2) && defined(CDC_DATA_INTERFACE2)
	#include "usb_serial2.h"
#endif

#ifdef USBserial2_h_
	extern usb_serial_class2 SerialX;
#endif



void setup()
{
	
	Serial.begin(9600);
	Serial.flush();
	Serial.setTimeout(1000);
	
	#ifdef USBserial2_h_
		SerialX.begin(9600);
	#endif
	
}


void loop()
{

	#ifdef USBserial2_h_
		SerialX.println("Hello");
	#endif
	
	Serial.println("Hello");
	delay(1000);
}
