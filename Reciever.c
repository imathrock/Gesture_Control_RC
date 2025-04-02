#include <msp430.h>
#include "Includes/nrf_userconfig.h"
#include "Includes/msprf24.h"
#include <stdint.h>

volatile unsigned int user;
unsigned int TXByte;

volatile int16_t forward_input;
volatile int16_t turn_input;

void control(int16_t forward, int16_t turn);

void Debug_LED_Blinker(int blink_count);

void Backward(void){P6OUT = 0b00001010;}
void Forward(void){P6OUT = 0b00000101;}
void Left(void){P6OUT = 0b00001001;}
void Right(void){P6OUT = 0b00000110;}
void Stop(void){P6OUT = 0b00000000;}


void main(void){
    char addr[5] = "00005";
    char buf[32];

    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    P1DIR |= BIT0;
    P4DIR |= BIT7;
    P6DIR |= 0b00011111;
    /*   Initial values for nRF24L01+ library config variables   */
    rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
    rf_addr_width      = 5;
    rf_speed_power     = RF24_SPEED_1MBPS | RF24_POWER_0DBM;
    rf_channel         = 100;

    msprf24_init();
    msprf24_set_pipe_packetsize(0, 32);
    msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

    // Set our RX address
    //addr[0] = 0xDE; addr[1] = 0xAD; addr[2] = 0xBE; addr[3] = 0xEF; addr[4] = 0x00;
    w_rx_addr(0, addr);

    if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
            flush_rx();
        }
        msprf24_activate_rx();
        Debug_LED_Blinker(5);

    __bis_SR_register(GIE); // Enable global interrupts
    while(1){
        if (rf_irq & RF24_IRQ_FLAGGED) {
                    msprf24_get_irq_reason();
                }
                if (rf_irq & RF24_IRQ_RX) {
                    r_rx_payload(32, buf);
                    msprf24_irq_clear(RF24_IRQ_RX);
                    user = buf[0];
                    forward_input = (int16_t)(buf[0] << 8) | buf[1];
                    turn_input = (int16_t)(buf[2] << 8) | buf[3];
                    control(forward_input,turn_input);
                }
                    else {
                    user = 0xFF;
                }
                }
    }

void control(int16_t forward, int16_t turn){
    if(forward>0){Forward();}
    if(forward<0){Backward();}
    if(turn>0){Right();}
    if(turn<0){Left();}
}

void Debug_LED_Blinker(int blink_count) {
    P4OUT |= BIT7;
    P1OUT |= BIT0;
    while (blink_count--) {
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(200000);
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(200000);
    }
    P1OUT &= ~BIT0;
    P4OUT &= ~BIT7;
}
