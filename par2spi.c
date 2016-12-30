/*
 *  par2spi.c
 *
 * ATmega328p AVR used as parallel<-->serial SPI to interface
 * FlashLite v25 CPU board with ENC28J60 Ethernet, ST7735R LCD and/or SD Card (built into LCD)
 * the Flashlite-V25 is a single board computer based on the NEC V-25 Plus micro-controller.
 * the V-25 is a 16-bit, single-chip micro-controller that is software compatible with the Intel 8086/8088 family of microprocessors.
 * it comes with embedded DOS and utility software on the on-board flash disk.
 * this is a very simple implementation with no interrupt handling, per byte read and write operation only
 * AVR is used as a smart hand-shake capable shift register, with the interrupt handling done by the v25 CPU
 * v25 can use the RDY (AVR-PB6 to v25-INTP0) line as an interrupt input to drive bulk transfers etc.
 * 
 *     connectivity schema:
 * 
 *   AVR                                             FlashLite NEC V25
 * =========                                      ======================
 * 
 *                                             J5
 *                                         ------------------------------------------------
 *                                           Port 1: PMC1=0x00, PM1=0x00
 *                                           Port 2: PMC2=0x00, PM2=0x00 (out) / 0xff (in)
 * 
 *  PB6    <----------*-------------------< ^RD/WR P1.4
 *                    |
 *  PB7    >----------------*-------------> ^RDY   P1.1 (INTP0)
 *                    |     |
 *                +--------( )---+
 *                |  DIR   OE^   |
 *                |              |
 *                |   74LS245    |
 *                |              |
 *  PD0..7 <----->| [B]      [A] |<------->        Data in/out     P2.0 .. P2.7
 *                |              |
 *                |              |
 *                +--------------+
 * 
 *  PB0    <------------------------------< STB    P1.5
 * 
 *  PC0    <------------------------------< FSEL0  P1.6
 *  PC1    <------------------------------< FSEL1  P1.7
 *         
 *         
 *  name:  FSEL1       FSEL0      ETH-CS^    LCD-CS^     LCD-D/C^
 *  v25 :   P1.7        P1.6
 *  AVR :   PC1         PC0         PC5        PC4         PC3
 *      ----------   ---------  ----------  ----------  ----------
 *           0           0           0          1           1         - Ethernet select
 *           0           1           1          0           0         - LCD select command
 *           1           0           1          0           1         - LCD select data
 *           1           1           1          1           1         - nothing selected (maybe use for SD card CS?)
 * 
 *         write to device timing                  read from device timing
 *     ---------------------------------       ---------------------------------
 * 
 *     SSn^    `````\......../```````          SSn^    `````\......../```````
 *     
 *     RD/WR^  `````\......../```````          RD/WR^  ``````````````````````
 *     
 *     Data    -----<XXXXXXXX>-------          Data    --------<XXXXXXXX>----
 *     
 *     STB     ```````\...../````````          STB     `````\......./````````
 *     
 *     RDY^    `````````\..../```````          RDY^    `````````\..../```````
 * 
 */

#include    <stdint.h>

#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>

// IO ports B, C and D initialization
#define     PB_DDR_INIT     0xac    // port data direction (SS^ / PB2 pin defined as output even if not used! see sec 18.3.2 in spec sheet)
#define     PB_PUP_INIT     0x00    // port input pin pull-up
#define     PB_INIT         0x84    // port initial values

#define     PC_DDR_INIT     0x38    // port data direction
#define     PC_PUP_INIT     0x00    // port input pin pull-up
#define     PC_INIT         0x38    // port initial values

#define     PD_DDR_INIT     0x00    // port data direction
#define     PD_PUP_INIT     0x00    // port input pin pull-up
#define     PD_INIT         0x00    // port initial values

// SPI configuration
#define     SPI_CTRL        0x50    // SPI control register (per ST7735R and ENC28J60 data sheet CPOL and CPHA = 0, SPI mode o)
#define     PWR_REDUCION    0xeb    // turn off unused peripherals: I2C, timers, UASRT, ADC

// misc masks and definitions
#define     TOGG_RDY        0x80    // toggle PB7
#define     PAR_WRITE       0x40    // detect read or write from v25 on PB6
#define     V25_STROBE      0x01    // detect asserted strobe from v25 on PB0
#define     CS_MASK         0xc7    // mask to clear CS bits before setting
#define     TX_DONE         0x80    // SPIF flag test
#define     DUMMY_WR        0x00    // dummy data to write to slave when reading from it

/****************************************************************************
  special function prototypes
****************************************************************************/
// This function is called upon a HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/****************************************************************************
  Globals
****************************************************************************/
uint8_t     deviceCS[4] = {0x18, 0x20, 0x28, 0x38}; // easy selection input: PC0..PC1 to output: PC3..PC4..PC5

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  initialize IO interfaces
 *  timer and data rates calculated based on 4MHz internal clock
 *
 */
void ioinit(void)
{
    // reconfigure system clock scaler to 8MHz
    CLKPR = 0x80;   // change clock scaler (sec 8.12.2 p.37)
    CLKPR = 0x00;

    // power reduction setup
    PRR   = PWR_REDUCION;

    // initialize SPI interface for master mode
    SPCR  = SPI_CTRL;

    // initialize general IO PB, PC and PD pins for output
    // -
    DDRB  = PB_DDR_INIT;            // PB pin directions
    PORTB = PB_INIT | PB_PUP_INIT;  // initial value of pins and input with pull-up

    DDRC  = PC_DDR_INIT;            // PC pin directions
    PORTC = PC_INIT | PC_PUP_INIT;  // initial value of pins and input with pull-up

    DDRD   = PD_DDR_INIT;           // PD data direction
    PORTD  = PD_INIT | PD_PUP_INIT; // initial value of pins and input with pull-up
}

/* ----------------------------------------------------------------------------
 * reset()
 *
 *  Clear SREG_I on hardware reset.
 *  source: http://electronics.stackexchange.com/questions/117288/watchdog-timer-issue-avr-atmega324pa
 */
void reset(void)
{
    cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0; // clear reset flags
    wdt_disable();
}

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 * - initialize IO: Ports and SPI
 * - wait for CS selection lines from v25 to change, and enter a "session"
 * - while in a "session"
 *      0. setup CS (SS lines) for devices on PC3,4,5
 *      1. configure PORT-D direction per RD^/WR line (PB6)
 *      2. wait for strobe to be asserted by v25 on PB0
 *      3. if in read mode send a dummy byte
 *         wait to transmission to finish by checking SPIF flag in SPSR register
 *         read returned byte and send it to PORT-D for v25 to read
 *      4. if in write mode read data from PORT-D and send through SPI
 *         place in SPI Tx register and wait to transmission to finish
 *      5. assert RDY and wait for strobe to un-assert
 *      6. when strobe is un-asserted, un-assert RDY
 * - clear data in the MISO circular buffer at the end of a session
 * - reset CS (SS lines) to devices)
 * - go back to wait for a CS selection to change
 *
 */
int main(void)
{
    int     nIsRead = 0;
    uint8_t temp;

    // initialize peripherals
    ioinit();
    
    // disable all interrupts
    cli();

    while (1)
    {
        // wait for CS selection lines to change and indicate a device SS selection
        while ( (PINC & 0x03) < 0x03 )
        {
            // setup device CS (SS lines)
            temp = (PORTC & CS_MASK) | deviceCS[(PINC & 0x03)];
            PORTC = temp;

            // configure PORT-D direction
            if ( PINB & PAR_WRITE )
            {
                DDRD = 0x00;            // v25 writing, PORT-D is input
                nIsRead = 0;
            }
            else
            {
                DDRD = 0xff;            // v25 reading, PORT-D is output
                nIsRead = 1;
            }

            // wait for v25 to assert strobe *** potential lock up ***
            while ( PINB & V25_STROBE ) {};

            // handle v25 read or write
            if ( nIsRead )
            {
                SPDR = DUMMY_WR;        // send a dummy byte
                while ( ~(SPSR & TX_DONE) ) {}; // wait for transmission to complete
                PORTD = SPDR;           // transfer the data byte to v25
                PORTB ^= TOGG_RDY;      // assert RDY to v25
            }
            else
            {
                PORTB ^= TOGG_RDY;      // assert RDY to v25, also enabling tri-state buffer
                SPDR = PIND;            // read data from v25 and transmit to slave
                while ( ~(SPSR & TX_DONE) ) {}; // wait for transmission to complete
            }

            // wait for v25 to un-assert strobe before un-asserting RDY *** potential lock up ***
            while ( ~(PINB & V25_STROBE) ) {};
            PORTB ^= TOGG_RDY;          // *** to prevent race a condition, v25 should wait until RDY is un-asserted before continuing ***
        }

        // un-assert all CS lines
        PORTC |= ~CS_MASK;
    }

    return 0;
}
