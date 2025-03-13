#include <msp430.h> 

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    P6DIR &= ~BIT0; // Set P6.0 as input
    P6DIR |= BIT1;  // Set P6.1 as output
    P6DIR |= BIT2;  // Set P6.2 as output
    P4DIR |= BIT7;  // Set P4.7 as output
    P1DIR |= BIT0;  // Set P1.0 as output

    while(1) {
        if (P6IN & BIT0) {  // Check if P6.0 is high
            P4OUT |= BIT7;  // Set P4.7 high
            P1OUT &= ~BIT0; // Set P1.0 low
            P6OUT |= BIT1;  // Set P6.1 high
            P6OUT &= ~BIT2; // Set P6.2 low
        } else {
            P4OUT &= ~BIT7; // Set P4.7 low
            P1OUT |= BIT0;  // Set P1.0 high
            P6OUT &= ~BIT1; // Set P6.1 low
            P6OUT |= BIT2;  // Set P6.2 high
        }
    }

    return 0;
}
