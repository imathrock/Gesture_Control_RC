#include <msp430.h>
#include <stdint.h>

// I2C communication MSP430
#define SDA_PIN BIT1 // SDA (serial data) is P4.1
#define SCL_PIN BIT2 // SCL (serial clock) is P4.2


// MPU6050 Definitions
#define MPU_6050_ADDR 0x68
#define PWR_MGMT_1 0x6b
#define SMPLRT_DIV 0x19
#define CONFIG 0x1a
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1c
#define INT_ENABLE 0x38

#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40

#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42

#define GYRO_XOUT_H   0x43
#define GYRO_XOUT_L   0x44
#define GYRO_YOUT_H   0x45
#define GYRO_YOUT_L   0x46
#define GYRO_ZOUT_H   0x47
#define GYRO_ZOUT_L   0x48

void I2C_init(void);
uint8_t MPU6050_Check(void);
void MPU_init(void);
void MPU_CHECK_PASS(void);
void MPU_CHECK_FAIL(void);
uint8_t Read_byte(uint8_t register_address);
void Write_Byte (uint8_t RegAddress, uint8_t RegValue);
void Debug_LED_Blinker(int blink_count);
int16_t get_data(void);

void main(void){
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    P1DIR |= BIT0;
    P4DIR |= BIT7;
    P6DIR |= 0b00011111;
    P6OUT = 0b00010001;
    _delay_cycles(1000000);
    I2C_init();
    if(MPU6050_Check()){
        MPU_init();
        MPU_CHECK_PASS();
    }else{MPU_CHECK_FAIL();}
    while(1){
        if(get_data()>0){P6OUT = 0b00010000;}
        if(get_data()>10){P6OUT = 0b00000001;}
    }
}

int16_t get_data(void){
    uint8_t i1 = Read_byte(ACCEL_XOUT_H);
    uint8_t i2 = Read_byte(ACCEL_XOUT_L);
    int16_t accel_x = (int16_t)(i1 << 8) | i2;
    return accel_x;
}

void MPU_CHECK_PASS(void){
    int k = 25;
    P4OUT |= BIT7;
    P1OUT |= BIT0;
    while (k--) {
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(100000);
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(100000);
    }
    P1OUT &= ~BIT0;
    P4OUT &= ~BIT7;
}

void MPU_CHECK_FAIL(void){
    int k = 15;
    P4OUT |= BIT7;
    while (k--) {
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(250000);
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(250000);
    }
    P1OUT &= ~BIT0;
    P4OUT &= ~BIT7;
}

uint8_t Read_byte(uint8_t register_address) {
    // Wait for any previous stop condition to complete
    while (UCB1CTL1 & UCTXSTP);

    UCB1IFG = 0;                        // Clear interrupt flags

    // --- Write Phase: Send register address ---
    UCB1CTL1 |= UCTR;                   // Set I2C to transmit mode
    UCB1CTL1 |= UCTXSTT;                // Send start condition
    while (!(UCB1CTL1 & UCTXIFG));      // Wait for start condition to complete

    UCB1TXBUF = register_address;       // Load register address to TX buffer
    while (!(UCB1IFG & UCTXIFG)) {      // Wait for address to be sent
        if (UCB1IFG & UCNACKIFG) {      // Check for NACK (no acknowledgment)
            _delay_cycles(1000000);     // Delay before returning (optional for debugging)
            Debug_LED_Blinker(4);       // Indicate error with LED blink
            return 0;                   // Return 0 on NACK error
        }
    }

    // --- Read Phase: Receive data from register ---
    UCB1CTL1 &= ~UCTR;                  // Switch to receiver mode
    UCB1CTL1 |= UCTXSTT;                // Send repeated start condition
    while (UCB1CTL1 & UCTXSTT);         // Wait for repeated start to complete

    UCB1CTL1 |= UCTXSTP;                // Send stop condition

    // Wait for data to be received in RX buffer
    while (!(UCB1IFG & UCRXIFG));
    return UCB1RXBUF;                   // Return received data
}



void Write_Byte(uint8_t RegAddress, uint8_t RegValue) {
    // Wait for any previous stop condition to complete
    while (UCB1CTL1 & UCTXSTP);

    // --- Write Phase: Send register address ---
    UCB1CTL1 |= UCTR;                  // Set I2C to transmit mode
    UCB1CTL1 |= UCTXSTT;               // Send start condition
    while (!(UCB1IFG & UCTXIFG));      // Wait for start condition to complete

    UCB1TXBUF = RegAddress;            // Load register address to TX buffer
    while (!(UCB1IFG & UCTXIFG)) {     // Wait for address to be sent
        if (UCB1IFG & UCNACKIFG) {     // Check for NACK (no acknowledgment)
            return;                    // Exit if no acknowledgment received
        }
    }

    // --- Write Phase: Send data to register ---
    UCB1TXBUF = RegValue;              // Load data to be written
    while (!(UCB1IFG & UCTXIFG));      // Wait for data transmission to complete

    UCB1CTL1 |= UCTXSTP;               // Send stop condition
    while (UCB1CTL1 & UCTXSTP);        // Wait for stop condition to clear
    return;
}


uint8_t MPU6050_Check(void) {
    uint8_t check = Read_byte(0x75);     // Read WHO_AM_I register (0x75)
    if (check == 0x68)                   // Check if the returned value is 0x68
        return 1;                        // MPU6050 is connected
    else
        return 0;                        // Connection failed
}

void MPU_init(void){
    Write_Byte(PWR_MGMT_1, 0x00);        // Wake up MPU6050
    Write_Byte(SMPLRT_DIV, 0x07);        // Sample rate = 1 kHz / (1 + 7) = 125 Hz
    Write_Byte(CONFIG, 0x06);            // Set low pass filter to ~5 Hz
    Write_Byte(GYRO_CONFIG, 0x08);       // ±500 °/s (gyro sensitivity)
    Write_Byte(ACCEL_CONFIG, 0x01);      // ±2g (accel sensitivity)
    Write_Byte(INT_ENABLE, 0x01);        // Enable data ready interrupt
}

void I2C_init(void) {
    // Pin setup for UCB1
    P4SEL |= SDA_PIN | SCL_PIN; // Select I2C function for P4.1 and P4.2

    UCB1CTL1 |= UCSWRST; // Put eUSCI_B1 in reset state
    UCB1CTL0 = UCMST | UCMODE_3 | UCSYNC; // I2C master mode, synchronous
    UCB1CTL1 = UCSSEL_2 | UCSWRST; // Use SMCLK, keep reset state
    UCB1BR0 = 10;  // Baud rate (assuming SMCLK = 1MHz, 100kHz SCL)
    UCB1BR1 = 0;
    UCB1STAT = 0; // Clear all STAT bits
    UCB1IE = 0; // clear I2C interrupt enable registers
    UCB1IFG = 0; // Clear all interrupt flag bits (including I2C related)
    UCB1I2CSA = MPU_6050_ADDR;
    UCB1CTL1 &= ~UCSWRST; // Release reset to start operation
    return;
}

void Debug_LED_Blinker(int blink_count) {
    P4OUT |= BIT7;
    P1OUT |= BIT0;
    while (blink_count--) {
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(250000);
        P1OUT ^= BIT0; // Toggle P1.0
        P4OUT ^= BIT7; // Toggle P4.7
        __delay_cycles(250000);
    }
    P1OUT &= ~BIT0;
    P4OUT &= ~BIT7;
}
