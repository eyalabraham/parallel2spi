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
 * v25 uses a 82C55 PIO with PortA in Mode-2, with automatic hand shake signal generation and interrupt line to v25.
 *
 *     connectivity schema for v25 with 8255 and AVR:
 *
 *     v25 CPU bus                     8255                    AVR
 * ===================       ==========================    ===========
 *
 *                 +-------------------+   IO addresses
 *                 |                   |  --------------
 *                 |       8255*       |   PA   0xXXX0
 *                 |                   |   PB   0xXXX1
 * DMARQ0** <------| PC3               |   PC   0xXXX2
 * (P2.0)          |                   |   Ctrl 0xXXX3
 *                 |                   |
 * IORD^   ------->| RD^      PA0..PA7 |<------------>    PD0..PD7  Data
 *                 |               PC4 |<-------------    PB0       STB^ (AVR strobe data into 8255)
 * IOWR^   ------->| WR^           PC5 |------------->    PB1       IBF  (8255 latched data, not ready = '1', ready or data read by CPU = '0')
 *                 |               PC6 |<-------------    PB6       ACK^ (AVR open 3-state read and ack data was read after OBF^)
 * A0      ------->| A0            PC7 |------------->    PB7       OBF^ (indicate to AVR data is available for reading)
 * A1      ------->| A1                |
 * A2      ------->| CS^               |
 *                 |          PC0..PC2 |<------------>    n.c.
 * D0..D7  <------>| D0..D7   PB0..PB7 |<------------>    PB0..PB2  FSEL0..FSEL2
 *                 |                   |
 *                 +-------------------+
 *
 * *  8255 Port A in 'Mode 2'
 *
 *
 *  name:  FSEL2       FSEL1       FSEL0       ETH-CS^    LCD-CS^     LCD-D/C^     SD-CS^
 *  8255:   PB2         PB1         PB0
 *  AVR :   PC2         PC1         PC0         PC5         PC4         PC3        PB2/SS^
 *       ---------   ---------   ---------   ----------  ----------  ----------  ------------
 *           0           0           0           1           0           0            1        - LCD select command
 *           0           0           1           1           0           1            1        - LCD select data
 *           0           1           0           0           1           1            1        - Ethernet select - read
 *           0           1           1           0           1           1            1        - Ethernet select - write
 *           1           0           0           1           1           1            0        - SD Card select - read
 *           1           0           1           1           1           1            0        - SD Card select - write
 *           1           1           0           1           1           1            1        - Read AVR status
 *           1           1           1           1           1           1            1        - Write AVR command
 *
 */

#define     F_CPU           2000000UL

#include    <stdint.h>

#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>
#include    <avr/cpufunc.h>
#include    <util/delay.h>

// AVR IO ports B, C and D initialization

/* -----------------------------------------------------------------------
   Port B

  +-----+-----+-----+-----+-----+-----+-----+-----+
  | PB7 | PB6 | PB5 | PB4 | PB3 | PB2 | PB1 | PB0 |
  +-----+-----+-----+-----+-----+-----+-----+-----+
     |     |     |     |     |     |     |     |
     |     |     |     |     |     |     |     |
     |     |     |     |     |     |     |     +--- [o]  STB^ (AVR strobe data into 8255)
     |     |     |     |     |     |     +--------- [i]  IBF  (8255 latched data, not ready = '1', ready or data read by CPU = '0')
     |     |     |     |     |     +--------------- [o]  use SS^ for SD-Card select
     |     |     |     |     +--------------------- [o]  MOSI
     |     |     |     +--------------------------- [i]  MISO
     |     |     +--------------------------------- [o]  SCLK
     |     +--------------------------------------- [o]  ACK^ (AVR open 3-state read and ack data was read after OBF^)
     +--------------------------------------------- [i]  OBF^ (indicate to AVR data is available for reading)

----------------------------------------------------------------------- */
#define     PB_DDR_INIT     0x6d    // port data direction
#define     PB_PUP_INIT     0x00    // port input pin pull-up
#define     PB_INIT         0x45    // port initial values

/* -----------------------------------------------------------------------
   Port C

  +-----+-----+-----+-----+-----+-----+-----+-----+
  | xxx | PC6 | PC5 | PC4 | PC3 | PC2 | PC1 | PC0 |
  +-----+-----+-----+-----+-----+-----+-----+-----+
           |     |     |     |     |     |     |
           |     |     |     |     |     |     |
           |     |     |     |     |     |     +--- [i]  \
           |     |     |     |     |     +--------- [i]   | 3-bit device select
           |     |     |     |     +--------------- [i]  /
           |     |     |     +--------------------- [o]  LCD Data/Control^
           |     |     +--------------------------- [o]  LCD Select
           |     +--------------------------------- [o]  Ethernet Select
           +--------------------------------------- [i]  Reset

----------------------------------------------------------------------- */
#define     PC_DDR_INIT     0x38    // port data direction
#define     PC_PUP_INIT     0x00    // port input pin pull-up
#define     PC_INIT         0x38    // port initial values

/* -----------------------------------------------------------------------
   Port D

  +-----+-----+-----+-----+-----+-----+-----+-----+
  | PD7 | PC6 | PC5 | PC4 | PC3 | PC2 | PC1 | PC0 |
  +-----+-----+-----+-----+-----+-----+-----+-----+
     |     |     |     |     |     |     |     |
     |     |     |     |     |     |     |     |
     +-----+-----+-----+-----+-----+-----+-----+--- [i/o] data

----------------------------------------------------------------------- */
#define     PD_DDR_INIT     0x00    // port data direction
#define     PD_PUP_INIT     0x00    // port input pin pull-up
#define     PD_INIT         0x00    // port initial values

// SPI configuration
#define     SPI_CTRL        0x53    // SPI control register (per ST7735R and ENC28J60 data sheet CPOL and CPHA = 0, SPI mode o)
#define     SPI_STAT        0x01    // SPI2X flag at '1' -->  SCK=Fosc/8
#define     PWR_REDUCION    0xeb    // turn off unused peripherals: I2C, timers, UASRT, ADC
#define     DUMMY_BYTE      0xff

// misc masks and definitions
#define     AVR_SPI_SS      0x04    // for controlling PB2, AVR's SS^ line
#define     SS_MASK         0x07    // Sale Select mask for selections coming from 8255 PC0..PC2
#define     CS_MASK         0xc7    // mask to clear device select bit PC3/4/5
#define     TOGG_ACK        0x40    // toggle ACK to 8255 on PB6
#define     DET_OBF         0x80    // detect OBF from 8255 on PB7
#define     TOGG_STB        0x01    // toggle STB to 8255 on PB0
#define     DET_IBF         0x02    // detect IBF from 8255 on PB1
#define     TX_DONE         0x80    // SPIF flag test

/****************************************************************************
  type definitions
****************************************************************************/
typedef enum                                    // valid devices supported
{
    LCD_CMD,                                    // ST7735R LCD controller commands
    LCD_DATA,                                   // ST7735R LCD controller data
    ETHERNET_RD,                                // ENC28J60 Ethernet read
    ETHERNET_WR,                                // ENC28J60 Ethernet write
    SD_CARD_RD,                                 // SD card read
    SD_CARD_WR,                                 // SD card write
    KEEP_CS,                                    // leave last CS asserted
    NONE                                        // nothing selected
} spiDevice_t;

/****************************************************************************
  special function prototypes
****************************************************************************/
// This function is called upon a HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/****************************************************************************
  Globals
****************************************************************************/
uint8_t     deviceCS[8] = {0x20, 0x28, 0x18, 0x18, 0x38, 0x38, 0x38, 0x38}; // easy selection input: PC0..PC2 to output: PC3..PC5

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
    CLKPR = 0x82;   // change clock scaler (sec 8.12.2 p.37)
    CLKPR = 0x00;

    // power reduction setup
    //PRR   = PWR_REDUCION;

    // initialize SPI interface for master mode
    SPCR  = SPI_CTRL;
    SPSR  = SPI_STAT;

    // initialize general IO PB, PC and PD pins for output
    DDRB  = PB_DDR_INIT;            // PB pin directions
    PORTB = PB_INIT | PB_PUP_INIT;  // initial value of pins and input with pull-up

    DDRC  = PC_DDR_INIT;            // PC pin directions
    PORTC = PC_INIT | PC_PUP_INIT;  // initial value of pins and input with pull-up

    DDRD   = PD_DDR_INIT;           // PD data direction
    PORTD  = PD_INIT | PD_PUP_INIT; // initial value of pins and input with pull-up
}

/* ----------------------------------------------------------------------------
 * spiTxRx()
 *
 * send/receive byte on SPI
 *
 */
uint8_t spiTxRx(uint8_t byte)
{
    SPDR = byte;                                // send byte on SPI
    while ( !(SPSR & TX_DONE) ) {};             // and wait for transmission to complete
    return SPDR;                                // return received byte
}

/* ----------------------------------------------------------------------------
 * spi2par()
 *
 * read SPI device and transfer data to parallel interface
 *
 * 1. send dummy byte to clock data from salve
 * 2. change PORT-D to output, write byte to PORTD
 * 3. strobe STB^
 * 4. change PORT-D back to input
 * 5. wait for IBF == 'low' on PINB1
 *
 */
void spi2par(void)
{
    PORTD = spiTxRx(DUMMY_BYTE);                // read SPI into Port-D
                                                // this order of operations will ensure that 0's do not appear on Port-D pins when switching in to out direction
    DDRD = 0xff;                                // switch PORT-D to output
    PORTB &= ~TOGG_STB;                         // toggle STB^ to latch data into 8255, IBF is now 'hi'
    _delay_us(1);                               // stretch strobe pulse ~1uS
    PORTB |= TOGG_STB;
    DDRD = 0x00;                                // switch PORT-D back to input

    while (PINB & DET_IBF) {};                  // wait for IBF to be "low", indicating read by 8255
                                                // this will also sync with device select to do 1-byte read
}

/* ----------------------------------------------------------------------------
 * par2spi()
 *
 * write data from the parallel interface to SPI device
 *
 * 1. wait for OBF^ == 'low' on PINB7
 * 2. assert ACK^ = 'low', and read data from PIND
 * 3. change ACK^ = 'hi'
 * 4. send read data to SPI
 *
 */
void par2spi(void)
{
    uint8_t     temp;

    while (PINB & DET_OBF)                      // wait for OBF^ to be asserted ("low") by v25/8255
    {
        temp = (PINC & SS_MASK);
        if ( temp == NONE ||                    // because NEC v25 CPU is slower, AVR might wait here for OBF^ which will never assert
             temp == KEEP_CS )
            return;                             // the test of device selects here will facilitate aborting an SPI write cycle
    }

    PORTB &= ~TOGG_ACK;                         // assert ("low") Ack on PB6,
    _delay_us(1);                               // stretch the ACK^ pulse ~1uS
    temp = PIND;                                // to read byte from v25 on AVR PD
    PORTB |= TOGG_ACK;                          // un-assert ("hi") Ack on PB6

    spiTxRx(temp);
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
 * - enter endless loop
 *      1. assert CS selection lines according to v25 input on PINC0/1/2
 *      2. select read or write action
 *      3. for read
 *         wait for IBF == 'low' on PINB1
 *         send dummy byte to clock data from salve
 *         change PORT-D to output, write byte to PORTD
 *         strobe STB^
 *         change PORT-D back to input
 *      4. for write
 *         wait for OBF^ == 'low' on PINB6
 *         assert ACK^ = 'low', and read data from PIND
 *         change ACK^ = 'hi'
 *         send read data to SPI
 *
 */
int main(void)
{
    uint8_t temp;
    uint8_t devSel;

    ioinit();									// initialize peripherals

    cli();										// disable interrupts

    while (1)
    {
        // due to 8255 IO bit out settling time, recheck devSel
        devSel = PINC & SS_MASK;
        _delay_us(2);                           // wait ~2uS and read PINC again
        if (devSel != (PINC & SS_MASK) )        // does new read match first read?
            continue;                           // if not exit here

        // wait for a valid device selection
        // then setup device CS on P3/4/5 and SS^ (SS lines) according to
        // inputs lines PC0/1/2
        switch ( devSel )
        {
        case NONE:                              // no selection, un-assert all CS lines
            PORTC |= ~CS_MASK;                  // then fall through to continue waiting loop
            PORTB |= AVR_SPI_SS;
            /* no break */

        case KEEP_CS:                           // leave previous CS line asserted and keep waiting for device selection
            continue;
            break;

        case LCD_CMD:
        case LCD_DATA:
            PORTB |= AVR_SPI_SS;
            temp = ((PORTC & CS_MASK) | deviceCS[devSel]);
            PORTC = temp;
            break;

        case ETHERNET_RD:                        // handle device selects from PC3/4/5
        case ETHERNET_WR:
            PORTB |= AVR_SPI_SS;
            temp = ((PORTC & CS_MASK) | deviceCS[devSel]);
            PORTC = temp;
            break;

        case SD_CARD_RD:                        // special handling for SD card (SS^ line on PB2)
        case SD_CARD_WR:
            temp = ((PORTC & CS_MASK) | deviceCS[devSel]);
            PORTC = temp;
            PORTB &= ~AVR_SPI_SS;
            break;

        default:;                               // *** not a possible selection ***
        }

        // handle reads and writes separately
        // for reads send a dummy write and strobe the 8255 with data from slave
        // for writes, get data from 8255 and shift to slave
        switch ( devSel )
        {
        case LCD_CMD:
        case LCD_DATA:
        case ETHERNET_WR:
        case SD_CARD_WR:
            par2spi();                          // transfer byte from parallel port to SPI
            break;

        case ETHERNET_RD:
        case SD_CARD_RD:
            spi2par();                          // transfer data from SPI to parallel port
            break;

        default:;                               // *** should not reach here *** fall through for devSel .eq. NONE or KEEP_CS
        }
    }

    return 0;
}
