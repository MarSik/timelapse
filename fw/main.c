#include <stdint.h>
#include <msp430g2553.h>

// Port 1
#define LED BIT7

#define LCD_BACKLIGHT BIT6
#define LCD_POWER BIT5
#define LCD_RESET BIT1
#define LCD_CS BIT0
#define LCD_DC BIT3

// Port 2
#define TRIGGER BIT4
#define FOCUS BIT5

#define BUTTON1 BIT3
#define BUTTON2 BIT2
#define BUTTON3 BIT1
#define BUTTON4 BIT0

// LCD commands
// Extended mode commands
#define LCD_EXT_CONTRAST 0x80
#define LCD_EXT_TEMPCO 0x04
#define LCD_EXT_BIAS 0x10

// Mode setting
#define LCD_MODE 0x20
#define LCD_MODE_EXTENDED 0x01
#define LCD_MODE_NORMAL 0x00

#define LCD_MODE_POWERDOWN 0x04
#define LCD_MODE_ON 0x00

#define LCD_MODE_HORIZONTAL 0x00
#define LCD_MODE_VERTICAL 0x02

// Display mode
#define LCD_DISPLAY 0x08
#define LCD_DISPLAY_BLANK 0x00
#define LCD_DISPLAY_ALL 0x01
#define LCD_DISPLAY_NORMAL 0x04
#define LCD_DISPLAY_INVERSE 0x05

#define LCD_BACKLIGHT_TIMEOUT_S 2
#define LCD_POWER_TIMEOUT_S 4

void delay_ms(uint16_t ms);
void lcd_send_cmd(const uint8_t *cmd);
void lcd_send_data(uint8_t len, const uint8_t *data);

uint8_t trigger_period_deci_minutes = 10;
volatile uint8_t deci_minutes = 10; // 1 = 6s, 10 = 1 minute

volatile const uint8_t *spi_ptr = 0x0;
volatile uint8_t spi_count = 0;
volatile uint8_t spi_tx_count = 0;

const static uint8_t lcd_init_sequence[] = {
  LCD_MODE | LCD_MODE_ON | LCD_MODE_EXTENDED, // Extended commands
  LCD_EXT_CONTRAST | 0x40, // Vop - contrast
  LCD_EXT_TEMPCO | 0x00, // Temp. coefficients
  LCD_EXT_BIAS | 0x04, // Bias mode 3 (1:40; b100)
  LCD_MODE | LCD_MODE_NORMAL, // Basic commands

  LCD_DISPLAY | LCD_DISPLAY_NORMAL,
  0x00
};

const static uint8_t empty_8B[] = {
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00
};

void initialize_lcd(void) {
  // Turn on LCD
  P1OUT |= LCD_RESET;
  P1OUT &= ~LCD_POWER;
  P1OUT &= ~LCD_RESET;

  // Clear reset from LCD
  P1OUT |= LCD_RESET;

  lcd_send_cmd(lcd_init_sequence); // Initialize
}

void main(void)
{
  // Disable watchdog
  WDTCTL = WDTPW + WDTHOLD;

  // Set clock to 1Mhz calibrated
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;

  // Set SMCTL clock to 1/1 of DCO -> 1 MHz SPI clock
  BCSCTL2 &= ~(DIVS0 | DIVS1);

  // Disable LCD and enable status LED
  P1OUT = LCD_POWER | LCD_RESET | LCD_CS;
  P1DIR = BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;

  // Deactivate trigger and focus interface
  P2OUT = FOCUS | TRIGGER;
  P2DIR = BIT4 | BIT5;

  // Activate P2 interrupts, high to low transitions
  P2IE = BUTTON1 | BUTTON2 | BUTTON3 | BUTTON4;
  P2IES = BUTTON1 | BUTTON2 | BUTTON3 | BUTTON4;

  // Activate SPI mode
  UCA0CTL1 |= UCSWRST; // UCSI reset
  
  UCA0CTL0 = UCMST | UCMSB | UCCKPH | UCSYNC; // Master, SCK idle low, Slave samples during SCK L->H, MSB first, 8 bit 
  UCA0CTL1 |= UCSSEL0 | UCSSEL1; // SMCLK clock source, no prescaling
  UCA0BR0 = 0x01;
  UCA0BR1 = 0x00;

  P1SEL |= BIT2 | BIT4; // USCI mode for P1.2 and P1.4
  P1SEL2 |= BIT2 | BIT4;

  UCA0CTL1 &= ~UCSWRST; // UCSI reset clear
  IFG2 |= UCA0RXIFG;

  initialize_lcd();

  // Clear screen
  for (int i = 0; i < (84 * 48 / 64); i++) {
    lcd_send_data(8, empty_8B);
  }

  // Turn off backlight
  P1OUT &= ~LCD_BACKLIGHT;

  BCSCTL1 |= DIVA_3;				// ACLK/8
  BCSCTL3 = XCAP_3;				// 12.5pF cap- setting for 32768Hz crystal

  //
  // Slow timer
  //

  // Capture period
  TA0CCTL0 = CCIE;				// CCR0 interrupt enabled
  TA0CCR0 = 0;  				// Paused timer

  // Backlight timer
  TA0CCTL1 = CCIE;				// CCR1 interrupt enabled
  TA0CCR1 = LCD_BACKLIGHT_TIMEOUT_S * 512 - 1;			// Backlight timeout
  
  // LCD timer
  TA0CCTL2 = CCIE;				// CCR2 interrupt enabled
  TA0CCR2 = LCD_POWER_TIMEOUT_S * 512 - 1;			// LCD timeout

  TA0CTL = TASSEL_1 | ID_3 | MC_1;		// ACLK, /8 (512/s), upmode

  //
  // Faster timer
  //
  
  // End of picture time
  TA1CCTL0 = CCIE;				// CCR0 interrupt enabled
  TA1CCR0 = 0;					// 8192 -> 2 sec; but disabled by default

  // Led / Aux output time
  TA1CCTL1 = CCIE;				// CCR1 interrupt enabled
  TA1CCR1 = 511;				// LED / Aux delay - 0.125 sec
  
  // Trigger time
  TA1CCTL2 = CCIE;				// CCR2 interrupt enabled
  TA1CCR2 = 4095;				// Trigger delay - 1 sec

  TA1CTL = TASSEL_1 | ID_0 | MC_1;		// ACLK, /1 (4096/s), upmode

  // Wait for the crystal clock
  while(IFG1 & OFIFG)
  {
    IFG1 &= ~OFIFG;
    __delay_cycles(100000);
  }

  // Start timer
  TA0CCR0 = 6 * 512;  				// 0.1 min period (6 secs)

  // Setup complete, go to sleep
  P1OUT |= LED; // Disable led

  for(;;) {
    _BIS_SR(LPM3_bits | GIE); // Enter LPM3 w/ interrupts
  }
}

__attribute__((__interrupt__(PORT2_VECTOR)))
static void PORT2_ISR(void)
{
  // handle buttons - enable LCD backlight and set the TA0 CC1 to turn it off
  P1OUT |= LCD_BACKLIGHT;
  
  if (P1OUT & LCD_POWER) {
    // Reinitialize LCD
    initialize_lcd();
  }

  TA0CCR1 = TA0R + 512 * LCD_BACKLIGHT_TIMEOUT_S;
  if (TA0CCR1 > TA0CCR0) {
    TA0CCR1 -= TA0CCR0;
  }

  TA0CCR2 = TA0R + 512 * LCD_POWER_TIMEOUT_S;
  if (TA0CCR2 > TA0CCR0) {
    TA0CCR2 -= TA0CCR0;
  }

  TA0CCTL1 = CCIE;				// CCR1 interrupt enabled
  TA0CCTL2 = CCIE;				// CCR2 interrupt enabled

  // Clear the interrupt flags
  P2IFG = 0x0;
}

/*
 * Timer A0 - 512 steps / s
 * CC0 - photo capture period (activate FOCUS and Timer A1)
 * CC1 - lcd backlight timer (512 / 1s)
 */

// CC0
__attribute__((__interrupt__(TIMER0_A0_VECTOR)))
static void TIMER0_A0_ISR(void)
{
  P1OUT ^= LED;

  if (deci_minutes > 0) {
    deci_minutes--;
    return;
  }

  P2OUT &= ~FOCUS;

  TA1CCR0 = 8192; // Start the fast timer - 2 sec turn off time
  deci_minutes = trigger_period_deci_minutes; // Reset trigger timer
}

// CC1, CC2
__attribute__((__interrupt__(TIMER0_A1_VECTOR)))
static void TIMER0_A1_ISR(void)
{
  switch (TA0IV) {
    case 0x2:
      P1OUT &= ~LCD_BACKLIGHT;
      TA0CCTL1 &= ~CCIE;				// CCR1 interrupt disabled
      break;
    case 0x4:    
      // Turn off LCD
      P1OUT |= LCD_POWER;
      TA0CCTL2 &= ~CCIE;				// CCR2 interrupt disabled
      break;
  }
}

/*
 * Timer A1 - 4096 steps / s
 * CC1 - activate LED (posibbly used as external trigger)
 * CC2 - activate camera shutter
 * CC0 - deactivate all outputs
 */

// CC0
__attribute__((__interrupt__(TIMER1_A0_VECTOR)))
static void TIMER1_A0_ISR(void)
{
  P1OUT |= LED;  
  P2OUT |= FOCUS | TRIGGER;  
  TA1CCR0 = 0x0; // Stop fast timer
}

// CC1, CC2
__attribute__((__interrupt__(TIMER1_A1_VECTOR)))
static void TIMER1_A1_ISR(void)
{
  switch (TA1IV) {
    case 0x2:
      P1OUT &= ~LED;
      break;
    case 0x4:    
      P2OUT &= ~TRIGGER;
      break;
  }
}

__attribute__((__interrupt__(USCIAB0TX_VECTOR)))
static void SPI_TX_READY_ISR(void)
{
    if (spi_tx_count) {
      spi_tx_count--;
      UCA0TXBUF = *spi_ptr++;

    } else {
      IE2 &= ~UCA0TXIE;
    }
}

__attribute__((__interrupt__(USCIAB0RX_VECTOR)))
static void SPI_TX_DONE_ISR(void)
{
  spi_count--;
  IFG2 &= ~UCA0RXIFG;
}

void spi_wait(void) {
  // Wait for the current SPI job to be finished
  while (spi_count)
    ;
}

void spi_send(uint8_t size, const uint8_t *data) {
  IFG2 &= ~UCA0RXIFG;
  spi_ptr = data;
  spi_count = spi_tx_count = size;
  IE2 |= UCA0TXIE | UCA0RXIE;
  _BIS_SR(GIE); // Enable interrupts
}

void lcd_send_cmd(const uint8_t *cmd) {
  spi_wait();
  P1OUT &= ~(LCD_CS | LCD_DC);

  uint8_t len = 0;
  const uint8_t *ptr = cmd;
  while (*ptr++) {
    len++;
  }

  spi_send(len, cmd);
}

void lcd_send_data(uint8_t len, const uint8_t *cmd) {
  spi_wait();

  P1OUT &= ~LCD_CS;
  P1OUT |= LCD_DC;

  spi_send(len, cmd);
}

void delay_ms(uint16_t ms)
{
  volatile uint16_t ms_count = 0;
  volatile uint16_t timer = 0;
  while (ms_count < ms) {
    ms_count++;
    timer = 0;
    while (timer < 100) timer++;
  }
}
