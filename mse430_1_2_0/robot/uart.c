/*
 * uart.c
 *
 *  Created on: Feb 27, 2016
 *      Author: Kristian Sims
 */

#include "uart.h"
#include <msp430.h>
#include "pins.h"

IObuffer _uart_rx_buf;
IObuffer _uart_tx_buf;
IObuffer* const uart_rx_buf = &_uart_rx_buf;
IObuffer* const uart_tx_buf = &_uart_tx_buf;
#define UART_BUF_SIZE  32
char uart_rx_chars[UART_BUF_SIZE];
char uart_tx_chars[UART_BUF_SIZE];

void uart_tx_callback() {
    IE2 |= UCA0TXIE;    // Enable TX interrupt (triggers immediately)
}

void uart_rx_callback() {
    // sys_event |= BIT(CMD_EVENT); // Signal command event
}

int uart_init() {

    // Create IObuffers (probably quite oversized)
    IObuffer_init(uart_rx_buf, (char*) &uart_rx_chars,
                  UART_BUF_SIZE, uart_rx_callback);
    uart_rx_buf->callback_once = 0;
    IObuffer_init(uart_tx_buf, (char*) &uart_tx_chars,
                  UART_BUF_SIZE, uart_tx_callback);
    uart_tx_buf->callback_once = 1;

    // Set up USCIA0
    UCA0CTL0 = 0x00;                // UART Mode, defaults

#ifdef UART_CRYSTAL                 // (Don't) use crystal for clock
    UCA0CTL1 |= UCSSEL0 | UCSWRST;  // ACLK, Reset
    UCA0BR0 = 3;
    UCA0MCTL = UCBRS_3;
#else                               // Set timing with DCO (less error)
    UCA0CTL1 |= UCSSEL1 | UCSWRST;  // SMCLK, Reset
    UCA0BR0 = 0x12;
    UCA0BR1 = 0x08;
    UCA0MCTL = UCBRS_6;
#endif

    P1SEL |= UART_RX | UART_TX;     // Pins in USCI mode
    P1SEL2 |= UART_RX | UART_TX;

    UCA0CTL1 &= ~UCSWRST;           // Start USCIA0
    IE2 |= UCA0RXIE;                // Enable RX interrupt
    return 0;
}


// Convenience functions

int uart_bytes_pending() {
    return uart_rx_buf->count;
}

void uart_read(char* buf, unsigned nbytes) {
    while (nbytes-- > 0)
        IOgetc(buf++, uart_rx_buf);
}

void uart_write(char* buf, unsigned nbytes) {
    IOnputs(buf, (int)nbytes, uart_tx_buf);
}


char uart_get_char() {
    char c;
    IOgetc(&c, uart_rx_buf);
    return c;
}

union intbuf {
    char bytes[2];
    signed int16;
    unsigned uint16;
};

int uart_get_int() {
    union intbuf buf;
    IOgetc(&buf.bytes[0], uart_rx_buf);
    IOgetc(&buf.bytes[1], uart_rx_buf);
    return buf.int16;
}

unsigned uart_get_uint() {
    union intbuf buf;
    IOgetc(&buf.bytes[0], uart_rx_buf);
    IOgetc(&buf.bytes[1], uart_rx_buf);
    return buf.uint16;
}

union longbuf {
    char bytes[4];
    signed long int32;
    unsigned long uint32;
};

long uart_get_long() {
    union longbuf buf;
    unsigned n;
    for (n=0; n<4; n++)
        IOgetc(&buf.bytes[n], uart_rx_buf);
    return buf.int32;
}

unsigned long uart_get_ulong() {
    union longbuf buf;
    unsigned n;
    for (n=0; n<4; n++)
        IOgetc(&buf.bytes[n], uart_rx_buf);
    return buf.uint32;
}

void uart_put_char(char c) {
    IOputc(c, uart_tx_buf);
}

void uart_put_int(int i) {
    union intbuf buf;
    buf.int16 = i;
    IOnputs(buf.bytes, 2, uart_tx_buf);
}

void uart_put_uint(unsigned u) {
    union intbuf buf;
    buf.uint16 = u;
    IOnputs(buf.bytes, 2, uart_tx_buf);
}

void uart_put_long(long x) {
    union longbuf buf;
    buf.int32 = x;
    IOnputs(buf.bytes, 4, uart_tx_buf);
}

// Interrupt handlers

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCI_TX_ISR() {
    if (IFG2 & UCA0TXIFG) {
        if (uart_tx_buf->count > 0) {           // If byte is available
            char c;
            IOgetc(&c, uart_tx_buf);
            UCA0TXBUF = (unsigned char) c;      // Load next byte
        } else {                                // Otherwise
            IE2 &= ~UCA0TXIE;                   // Shut off interrupt
        }
    }
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCI_RX_ISR() {
    if (IFG2 & UCA0RXIFG) {
        IOputc(UCA0RXBUF, uart_rx_buf);         // Copy into buffer
    }
    __bic_SR_register_on_exit(LPM0_bits);
}

