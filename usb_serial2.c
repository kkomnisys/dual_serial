/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "usb_dev.h"
#include "usb_serial2.h"
#include "core_pins.h" // for yield()
//#include "HardwareSerial.h"
#include <string.h> // for memcpy()

// defined by usb_dev.h -> usb_desc.h
#if defined(CDC_STATUS_INTERFACE2) && defined(CDC_DATA_INTERFACE2)
#if F_CPU >= 20000000

uint32_t usb_cdc_line_coding2[2];
volatile uint32_t usb_cdc_line_rtsdtr_millis2;
volatile uint8_t usb_cdc_line_rtsdtr2=0;
volatile uint8_t usb_cdc_transmit_flush_timer2=0;

static usb_packet_t *rx_packet2=NULL;
static usb_packet_t *tx_packet2=NULL;
static volatile uint8_t tx_noautoflush2=0;

#define TRANSMIT_FLUSH_TIMEOUT	5   /* in milliseconds */


// get the next character, or -1 if nothing received
int usb_serial_getchar2(void)
{
	unsigned int i;
	int c;

	if (!rx_packet2) {
		if (!usb_configuration) return -1;
		rx_packet2 = usb_rx(CDC_RX2_ENDPOINT);
		if (!rx_packet2) return -1;
	}
	i = rx_packet2->index;
	c = rx_packet2->buf[i++];
	if (i >= rx_packet2->len) {
		usb_free(rx_packet2);
		rx_packet2 = NULL;
	} else {
		rx_packet2->index = i;
	}
	return c;
}

// peek at the next character, or -1 if nothing received
int usb_serial_peekchar2(void)
{
	if (!rx_packet2) {
		if (!usb_configuration) return -1;
		rx_packet2 = usb_rx(CDC_RX2_ENDPOINT);
		if (!rx_packet2) return -1;
	}
	if (!rx_packet2) return -1;
	return rx_packet2->buf[rx_packet2->index];
}

// number of bytes available in the receive buffer
int usb_serial_available2(void)
{
	int count;
	count = usb_rx_byte_count(CDC_RX2_ENDPOINT);
	if (rx_packet2) count += rx_packet2->len - rx_packet2->index;
	return count;
}


// read a block of bytes to a buffer
int usb_serial_read2(void *buffer, uint32_t size)
{
	uint8_t *p = (uint8_t *)buffer;
	uint32_t qty, count=0;

	while (size) {
		if (!usb_configuration) break;
		if (!rx_packet2) {
			rx:
			rx_packet2 = usb_rx(CDC_RX2_ENDPOINT);
			if (!rx_packet2) break;
			if (rx_packet2->len == 0) {
				usb_free(rx_packet2);
				goto rx;
			}
		}
		qty = rx_packet2->len - rx_packet2->index;
		if (qty > size) qty = size;
		memcpy(p, rx_packet2->buf + rx_packet2->index, qty);
		p += qty;
		count += qty;
		size -= qty;
		rx_packet2->index += qty;
		if (rx_packet2->index >= rx_packet2->len) {
			usb_free(rx_packet2);
			rx_packet2 = NULL;
		}
	}
	return count;
}


// discard any buffered input
void usb_serial_flush_input2(void)
{
	usb_packet_t *rx;

	if (!usb_configuration) return;
	if (rx_packet2) {
		usb_free(rx_packet2);
		rx_packet2 = NULL;
	}
	while (1) {
		rx = usb_rx(CDC_RX2_ENDPOINT);
		if (!rx) break;
		usb_free(rx);
	}
}

// Maximum number of transmit packets to queue so we don't starve other endpoints for memory
#define TX_PACKET_LIMIT 8

// When the PC isn't listening, how long do we wait before discarding data?  If this is
// too short, we risk losing data during the stalls that are common with ordinary desktop
// software.  If it's too long, we stall the user's program when no software is running.
#define TX_TIMEOUT_MSEC 70

#if F_CPU == 240000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1600)
#elif F_CPU == 216000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1440)
#elif F_CPU == 192000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1280)
#elif F_CPU == 180000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1200)
#elif F_CPU == 168000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 1100)
#elif F_CPU == 144000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 932)
#elif F_CPU == 120000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 764)
#elif F_CPU == 96000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 596)
#elif F_CPU == 72000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 512)
#elif F_CPU == 48000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 428)
#elif F_CPU == 24000000
  #define TX_TIMEOUT (TX_TIMEOUT_MSEC * 262)
#endif

// When we've suffered the transmit timeout, don't wait again until the computer
// begins accepting data.  If no software is running to receive, we'll just discard
// data as rapidly as Serial.print() can generate it, until there's something to
// actually receive it.
static uint8_t transmit_previous_timeout=0;



// transmit a character.  0 returned on success, -1 on error
int usb_serial_putchar2(uint8_t c)
{
	return usb_serial_write2(&c, 1);
}


int usb_serial_write2(const void *buffer, uint32_t size)
{
	uint32_t ret = size;
	uint32_t len;
	uint32_t wait_count;
	const uint8_t *src = (const uint8_t *)buffer;
	uint8_t *dest;

	tx_noautoflush2 = 1;
	while (size > 0) {
		if (!tx_packet2) {
			wait_count = 0;
			while (1) {
				if (!usb_configuration) {
					tx_noautoflush2 = 0;
					return -1;
				}
				if (usb_tx_packet_count(CDC_TX2_ENDPOINT) < TX_PACKET_LIMIT) {
					tx_noautoflush2 = 1;
					tx_packet2 = usb_malloc();
					if (tx_packet2) break;
					tx_noautoflush2 = 0;
				}
				if (++wait_count > TX_TIMEOUT || transmit_previous_timeout) {
					transmit_previous_timeout = 1;
					return -1;
				}
				yield();
			}
		}
		transmit_previous_timeout = 0;
		len = CDC_TX2_SIZE - tx_packet2->index;
		if (len > size) len = size;
		dest = tx_packet2->buf + tx_packet2->index;
		tx_packet2->index += len;
		size -= len;
		while (len-- > 0) *dest++ = *src++;
		if (tx_packet2->index >= CDC_TX2_SIZE) {
			tx_packet2->len = CDC_TX2_SIZE;
			usb_tx(CDC_TX2_ENDPOINT, tx_packet2);
			tx_packet2 = NULL;
		}
		usb_cdc_transmit_flush_timer2 = TRANSMIT_FLUSH_TIMEOUT;
	}
	tx_noautoflush2 = 0;
	return ret;
}


int usb_serial_write_buffer_free2(void)
{
	uint32_t len;

	tx_noautoflush2 = 1;
	if (!tx_packet2) {
		if (!usb_configuration ||
		  usb_tx_packet_count(CDC_TX2_ENDPOINT) >= TX_PACKET_LIMIT ||
		  (tx_packet2 = usb_malloc()) == NULL) {
			tx_noautoflush2 = 0;
			return 0;
		}
	}
	len = CDC_TX2_SIZE - tx_packet2->index;
	// TODO: Perhaps we need "usb_cdc_transmit_flush_timer2 = TRANSMIT_FLUSH_TIMEOUT"
	// added here, so the SOF interrupt can't take away the available buffer
	// space we just promised the user could write without blocking?
	// But does this come with other performance downsides?  Could it lead to
	// buffer data never actually transmitting in some usage cases?  More
	// investigation is needed.
	// https://github.com/PaulStoffregen/cores/issues/10#issuecomment-61514955
	tx_noautoflush2 = 0;
	return len;
}


void usb_serial_flush_output2(void)
{
	if (!usb_configuration) return;
	tx_noautoflush2 = 1;
	if (tx_packet2) {
		usb_cdc_transmit_flush_timer2 = 0;
		tx_packet2->len = tx_packet2->index;
		usb_tx(CDC_TX2_ENDPOINT, tx_packet2);
		tx_packet2 = NULL;
	} else {
		usb_packet_t *tx = usb_malloc();
		if (tx) {
			usb_cdc_transmit_flush_timer2 = 0;
			usb_tx(CDC_TX2_ENDPOINT, tx);
		} else {
			usb_cdc_transmit_flush_timer2 = 1;
		}
	}
	tx_noautoflush2 = 0;
}


void usb_serial_flush_callback2(void)
{
	if (tx_noautoflush2) return;
	if (tx_packet2) {
		tx_packet2->len = tx_packet2->index;
		usb_tx(CDC_TX2_ENDPOINT, tx_packet2);
		tx_packet2 = NULL;
	} else {
		usb_packet_t *tx = usb_malloc();
		if (tx) {
			usb_tx(CDC_TX2_ENDPOINT, tx);
		} else {
			usb_cdc_transmit_flush_timer2 = 1;
		}
	}
}


#endif // F_CPU
#endif // CDC_STATUS_INTERFACE2 && CDC_DATA_INTERFACE2
