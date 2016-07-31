#include <stdint.h>
#include <msp430g2553.h>

#include "font.h"
#include "banner.h"
#include "battery_low.h"
#include "battery_ok.h"

// Port 1
#define LED BIT7

#define LCD_BACKLIGHT BIT6
#define LCD_POWER BIT5
#define LCD_RESET BIT1
#define LCD_DC BIT3

#define EXT BIT0

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
void lcd_send_data(uint16_t len, const uint8_t *data, uint8_t invert);
void lcd_send_text_raw(const char *data, uint8_t mask);
void lcd_send_text(const char *data);
void lcd_send_text_inv(const char *data);
void backlight_lcd(void);

void spi_wait(void);

volatile const uint8_t *spi_ptr = 0x0;
volatile uint16_t spi_count = 0;
volatile uint16_t spi_tx_count = 0;
volatile uint8_t spi_tx_invert = 0x00;

volatile struct _lcd_status {
  int reinitialize:1;
  int redraw:1;
  int backlight:1;
  int ready:1;
  int poweringdown:1;
} lcd_status = {1, 0, 0, 0, 0};

const static uint8_t lcd_init_sequence[] = {
  LCD_MODE | LCD_MODE_ON | LCD_MODE_EXTENDED, // Extended commands
  LCD_EXT_CONTRAST | 0x40, // Vop - contrast
  LCD_EXT_TEMPCO | 0x00, // Temp. coefficients
  LCD_EXT_BIAS | 0x04, // Bias mode 3 (1:40; b100)
  LCD_MODE | LCD_MODE_NORMAL, // Basic commands

  LCD_DISPLAY | LCD_DISPLAY_NORMAL,
  0x00
};

const static uint8_t lcd_poweroff_sequence[] = {
  LCD_MODE | LCD_MODE_POWERDOWN,
  0x00
};

const static uint8_t lcd_first_line[] = {
  0x80, // Set X coordinates to 0
  0x40, // Set Y coordinates to 0
  0x00
};

const static uint8_t lcd_func_line[] = {
  0x80, // Set X coordinates to 0
  0x42, // Set Y coordinates to 2
  0x00
};

const static uint8_t lcd_delay_line[] = {
  0x80, // Set X coordinates to 0
  0x43, // Set Y coordinates to 3
  0x00
};

const static uint8_t lcd_last_line[] = {
  0x80, // Set X coordinates to 0
  0x45, // Set Y coordinates to 5
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
  lcd_status.reinitialize = 0;
  lcd_status.poweringdown = 0;
  lcd_status.redraw = 1;

  UCA0CTL1 |= UCSWRST; // UCSI reset
  UCA0CTL1 |= UCSSEL0 | UCSSEL1; // SMCLK clock source, no prescaling
  P1SEL |= BIT2 | BIT4; // USCI mode for P1.2 and P1.4
  P1SEL2 |= BIT2 | BIT4;
  UCA0CTL1 &= ~UCSWRST; // UCSI reset clear

  // Turn on LCD
  P1OUT |= LCD_RESET;
  P1OUT &= ~LCD_POWER;

  // Reset pulse
  P1OUT &= ~LCD_RESET;
  P1OUT |= LCD_RESET;

  lcd_send_cmd(lcd_init_sequence); // Initialize
  lcd_status.ready = 1;
}

void draw_digit(uint8_t digit, uint8_t inversion) {
  lcd_send_data(FONT_SIZE, FONT + FONT_SIZE * (digit + 0x30 - FONT_FIRST), inversion);
}

void draw_space(void) {
  lcd_send_data(FONT_SIZE + 1, empty_8B, 0x00);
}

void draw_number(uint8_t num, uint8_t inverted) {
  uint8_t ones = num % 10;
  num /= 10;
  uint8_t tens = num % 10;
  num /= 10;
  uint8_t hundreds = num;


  draw_digit(hundreds, inverted);
  draw_digit(tens, inverted);
  draw_digit(ones, inverted);
}

volatile struct _time_setup_cfg {
	unsigned int set:1;
	unsigned int armed:1; // Run or wait for external trigger
	unsigned int run:1; // Timers running!
	unsigned int selector:3;
	unsigned int prephase:1;
} time_cfg = {0, 0, 0, 2, 0};

volatile uint8_t trigger_count = 0;

#define TIMER 3
#define EXTON 5
#define COUNT 4
#define TRIGGER_DELAY 6
#define AUX_DELAY 7
#define MAX_SELECTOR AUX_DELAY

volatile uint8_t time_setup[] = {
  0x00, // hours
  0x00, // deka-minutes
  0x01, // minutes
  0x00, // deci-minutes

  0x00, // Number of pictures,
  0x00, // Ext trigger enable
  409 >> 2, // Trigger delay
  0x00  // Aux delay
};

volatile uint8_t time_trigger[] = {
  0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t time_setup_maximum[] = { 99, 5, 9, 9, 99, 1, 255, 255 };

volatile uint8_t battery_trigger = 0;
// every 5 minutes
#define BATTERY_TRIGGER_MAX (5 * 10)
volatile uint16_t battery = 0;

#define SYSTEM_PERIOD (512 * 6) // 6s
#define ACTIVITY_LED_TIMEOUT (512 / 16) // 0.0625s

#define CAPTURE_LENGTH 2048 // 0.5s

void poweroff_lcd(void) {
  lcd_status.ready = 0;
  lcd_status.reinitialize = 0;
  lcd_status.poweringdown = 0;
  lcd_send_cmd(lcd_poweroff_sequence);
  spi_wait();

  UCA0CTL1 = UCSWRST; // UCSI reset, no clock
  P1SEL &= ~(BIT2 | BIT4); // DIO mode for P1.2 and P1.4
  P1SEL2 &= ~(BIT2 | BIT4);

  // All wires to LCD should be on GND potential
  P1OUT &= ~(LCD_DC | LCD_RESET | BIT2 | BIT4);

  // Turn off power
  P1OUT |= LCD_POWER;
}

void draw_lcd(void) {
  lcd_status.redraw = 0;

  // Print setup line
  lcd_send_cmd(lcd_first_line);
  
  if (time_cfg.run) {
    lcd_send_text("T- ");
    
    draw_number(time_trigger[0], 0);
    lcd_send_text("h ");
    draw_digit(time_trigger[1], 0);
    draw_digit(time_trigger[2], 0);
    lcd_send_text(".");
    draw_digit(time_trigger[3], 0);
    lcd_send_text("m");

    lcd_send_cmd(lcd_func_line);
    lcd_send_text(" c:");
    
    if (time_trigger[COUNT] || time_setup[EXTON]) draw_number(time_trigger[COUNT], 0);
    else lcd_send_text("inf");

    lcd_send_text(" ext:");

    if (time_setup[EXTON]) lcd_send_text("ON ");
    else lcd_send_text("OFF");

    lcd_send_cmd(lcd_delay_line);
    lcd_send_text("tr:");
    draw_number(time_setup[TRIGGER_DELAY], 0);
    lcd_send_text(" aux:");
    draw_number(time_setup[AUX_DELAY], 0);
  }
  else {
    uint8_t inversion = time_cfg.set ? 0xff : ~0x7;
    if (time_cfg.armed) inversion = 0;

    lcd_send_text("   ");

    draw_number(time_setup[0], time_cfg.selector == 0 ? inversion : 0);
    lcd_send_text("h ");
    draw_digit(time_setup[1], time_cfg.selector == 1 ? inversion : 0);
    draw_digit(time_setup[2], time_cfg.selector == 2 ? inversion : 0);
    lcd_send_text(".");
    draw_digit(time_setup[3], time_cfg.selector == 3 ? inversion : 0);
    lcd_send_text("m");

    lcd_send_cmd(lcd_func_line);
    lcd_send_text(" c:");
    if (time_setup[COUNT]) draw_number(time_setup[COUNT], time_cfg.selector == COUNT ? inversion : 0);
    else lcd_send_text_raw("inf", time_cfg.selector == COUNT ? inversion : 0x00);

    lcd_send_text(" ext:");
    if (time_setup[EXTON]) lcd_send_text_raw("ON ", time_cfg.selector == EXTON ? inversion : 0x00);
    else lcd_send_text_raw("OFF", time_cfg.selector == EXTON ? inversion : 0x00);

    lcd_send_cmd(lcd_delay_line);
    lcd_send_text("tr:");
    draw_number(time_setup[TRIGGER_DELAY], time_cfg.selector == TRIGGER_DELAY ? inversion : 0);
    lcd_send_text(" aux:");
    draw_number(time_setup[AUX_DELAY], time_cfg.selector == AUX_DELAY ? inversion : 0);
  }

  // Print battery line
  lcd_send_cmd(lcd_last_line);
  if (battery == 0) {
    lcd_send_data(8, empty_8B, 0x00);
    lcd_send_data(BATTERY_OK_WIDTH - 8, empty_8B, 0x00);
  } else if (battery < 648) {
    lcd_send_data(BATTERY_LOW_WIDTH, battery_low_bits, 0x00);
  } else {
    lcd_send_data(BATTERY_OK_WIDTH, battery_ok_bits, 0x00);
  }

  lcd_send_text(" ");
  if (time_cfg.run) lcd_send_text("count");
  else if (time_cfg.armed) lcd_send_text_inv("armed");
  else if (time_cfg.set) lcd_send_text("edit ");
  else lcd_send_text("     ");
  lcd_send_data(2, empty_8B, 0x00);
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
  P1OUT = LCD_POWER | LCD_RESET;
  P1DIR = BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;

  // Deactivate trigger and focus interface
  P2OUT = FOCUS | TRIGGER;
  P2DIR = BIT4 | BIT5;

  // Activate P2 interrupts, high to low transitions
  P2IE = BUTTON1 | BUTTON2 | BUTTON3 | BUTTON4;
  P2IES = BUTTON1 | BUTTON2 | BUTTON3 | BUTTON4;

  // Activate P1 interrupt, high to low transition
  // P1IE = EXT; -- Activated only when ARMed with ext trigger enabled
  P1IES = EXT;

  // Configure pullups for ext trigger
  P1REN = EXT;

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

  // Clear screen with banner
  lcd_send_data(BANNER_WIDTH * BANNER_HEIGHT / 8, banner_bits, 0x00);

  // Turn off backlight
  P1OUT &= ~LCD_BACKLIGHT;

  BCSCTL1 |= DIVA_3;				// ACLK/8
  BCSCTL3 = XCAP_3;				// 12.5pF cap- setting for 32768Hz crystal

  //
  // Measure battery voltage
  // 2.5V internal reference
  // measure channel 1011 - (Vcc + GND) / 2
  // Set ADC clock to ACLK
  //
  // CONSEQ0 (single channel, single conversion)
  ADC10CTL0 = SREF0 | ADC10SHT1 | ADC10SHT0 | ADC10SR | REF2_5V | REFON | ADC10ON;
  ADC10CTL1 = INCH3 | INCH1 | INCH0 | ADC10DIV0 | ADC10SSEL0;  

  //
  // Slow timer
  //

  // Capture period
  TA0CCTL0 = CCIE;				// CCR0 interrupt enabled
  TA0CCR0 = 0;  				// Paused timer

  // LED timer
  TA0CCTL2 = CCIE;				// CCR2 interrupt enabled
  TA0CCR2 = ACTIVITY_LED_TIMEOUT - 1;           // Activity LED timeout
  
  // LCD timer
  TA0CCTL1 = CCIE;				// CCR1 interrupt enabled
  TA0CCR1 = LCD_BACKLIGHT_TIMEOUT_S * 512 - 1;	// LCD timeout

  TA0CTL = TASSEL_1 | ID_3 | MC_1;		// ACLK, /8 (512/s), upmode

  //
  // Faster timer
  //
  
  // End of picture time
  TA1CCTL0 = CCIE;				// CCR0 interrupt enabled
  TA1CCR0 = 0;					// 4096 = 1 sec; but disabled by default


  TA1CTL = TASSEL_1 | ID_0 | MC_1;		// ACLK, /1 (4096/s), upmode

  // Wait for the crystal clock
  while(IFG1 & OFIFG)
  {
    IFG1 &= ~OFIFG;
    __delay_cycles(100000);
  }

  // Setup complete, go to sleep
  TA0CCR0 = SYSTEM_PERIOD; // Start main timer
  P1OUT |= LED; // Disable led

  for(;;) {
   if (lcd_status.reinitialize) {
     // Reinitialize LCD
     initialize_lcd();
   }
   if (lcd_status.backlight) {
     backlight_lcd();
   }
   if (lcd_status.poweringdown) {
     poweroff_lcd();
   }
   if (lcd_status.redraw && lcd_status.ready) {
     draw_lcd();
   }
    _BIS_SR(LPM3_bits | GIE); // Enter LPM3 w/ interrupts
  }
}

void backlight_lcd(void) {
  P1OUT |= LCD_BACKLIGHT;
  lcd_status.backlight = 0;
  
  TA0CCR1 = TA0R + 512 * LCD_BACKLIGHT_TIMEOUT_S;
  if (TA0CCR1 > TA0CCR0) {
    TA0CCR1 -= TA0CCR0;
  }

  TA0CCTL1 = CCIE;				// CCR1 interrupt enabled
}

__attribute__((__interrupt__(PORT1_VECTOR)))
static void PORT1_ISR(void)
{
  // Clear the interrupt flags
  P1IFG = 0x0;

  // Do nothing when the timer is on or device is disarmed
  if (!time_cfg.armed || time_cfg.run) return;

  // Start system timer
  time_cfg.run = 1;

  // Setup time to capture 
  for (int8_t idx = COUNT; idx >= 0; idx--) {
    time_trigger[idx] = time_setup[idx];
  }

  if (!time_trigger[COUNT]) time_trigger[COUNT] = 1;

  // Reset system timer
  TA0R = 0;

  // Reset TA0CC interrupt flags
  while (TA0IV)
    ;

  // Redraw LCD if active
  lcd_status.redraw = 1;

  // Leave sleep to allow display refresh
  LPM3_EXIT; 
}

__attribute__((__interrupt__(PORT2_VECTOR)))
static void PORT2_ISR(void)
{
  // handle buttons - enable LCD backlight and set the TA0 CC1 to turn it off
  lcd_status.backlight = 1;

  if (!lcd_status.ready) {
    lcd_status.reinitialize = 1;
  
    // Clear the interrupt flags
    P2IFG = 0x0;
    LPM3_EXIT;
    return;
  }

  // Handle buttons
  if (P2IFG & BUTTON1) {
    time_cfg.armed ^= 0x1;
    time_cfg.run = time_cfg.armed && !time_setup[EXTON];
    lcd_status.redraw = 1;
    for (int8_t idx = COUNT; idx >= 0; idx--) {
      time_trigger[idx] = time_setup[idx];
    }

    uint16_t deci_minutes = time_setup[3] + \
				  time_setup[2] * 10 + \
				  time_setup[1] * 100 + \
				  time_setup[0] * 600;
    if (!deci_minutes) time_cfg.run = 0;
    time_cfg.set &= ~time_cfg.run;
 
    if (time_cfg.armed) {
      // Configure external trigger
      // Pull-up when EXT enabled, down when not
      if (time_setup[EXTON]) {
        P1IE = EXT;
	P1OUT |= EXT;
      } else {
	P1IE = 0;
	P1OUT &= ~EXT;
      }
      // Clear the interrupt flags
      P2IFG = 0x0;
      P1IFG = 0x0;
      // Reset system timer
      TA0R = 0;
      // Reset TA0CC interrupt flags
      while (TA0IV)
	;
    }
  }

  if (time_cfg.armed) {
    // Leave sleep to allow display refresh
    P2IFG = 0x0;
    LPM3_EXIT;
    return;
  }

  if (P2IFG & BUTTON3) {
    time_cfg.set ^= 0x01;
    lcd_status.redraw = 1;
  }

  if (P2IFG & BUTTON4) {
    if (time_cfg.set) {
      if (time_setup[time_cfg.selector] < time_setup_maximum[time_cfg.selector]) time_setup[time_cfg.selector]++;
    } else {
      if (time_cfg.selector > 0) time_cfg.selector--;
      else time_cfg.selector = MAX_SELECTOR;
    }
    lcd_status.redraw = 1;
  }

  if (P2IFG & BUTTON2) {
    if (time_cfg.set) {
      if (time_setup[time_cfg.selector] > 0) time_setup[time_cfg.selector]--;
    } else {
      if (time_cfg.selector < MAX_SELECTOR) time_cfg.selector++;
      else time_cfg.selector = 0;
    }
    lcd_status.redraw = 1;
  }

  // Clear the interrupt flags
  P2IFG = 0x0;
  LPM3_EXIT;
}

// Battery measurement
__attribute__((__interrupt__(ADC10_VECTOR)))
static void BATTERY_ADC_ISR(void)
{
  ADC10CTL0 &= ~(REFON | ADC10IE | ENC);
  lcd_status.redraw = 1;
  battery = ADC10MEM;
  LPM3_EXIT;
}

/*
 * Timer A0 - 512 steps / s
 * CC0 - photo capture period (activate FOCUS and Timer A1)
 * CC1 - activity led timeout
 * CC2 - lcd backlight timer (512 / 1s)
 */

// CC0
__attribute__((__interrupt__(TIMER0_A0_VECTOR)))
static void TIMER0_A0_ISR(void)
{
  if (battery_trigger) {
    battery_trigger--;
  } else {
    battery_trigger = BATTERY_TRIGGER_MAX;
    ADC10CTL0 |= REFON | REF2_5V | ADC10ON | ADC10IE;
    ADC10CTL0 |= ENC | ADC10SC;
  }

  if (!time_cfg.armed) {
    P1OUT |= LED;
    return;
  }

  // Flash activity LED
  P1OUT &= ~LED;

  if (!time_cfg.run) {
    return;
  }

  // Redraw LCD if active
  lcd_status.redraw = 1;

  // Decrement the lowest non-zero and all lower
  // are reset to defaults (1:00-- -> 0:59)
  for (int8_t idx = TIMER; idx >= 0; idx--) {
    if (time_trigger[idx]) {
      time_trigger[idx]--;
      break;
    } else {
      time_trigger[idx] = time_setup_maximum[idx];
    }
  }
      
  // Trigger when zero
  if (time_trigger[3] != 0
       || time_trigger[2] != 0
       || time_trigger[1] != 0
       || time_trigger[0] != 0) {
    LPM3_EXIT;
    return;
  }

  if (time_cfg.prephase) {
    time_cfg.prephase = 0;
  } else {
    P2OUT &= ~FOCUS;
    // Aux output time
    TA1CCTL1 = CCIE;				// CCR1 interrupt enabled
    TA1CCR1 = time_setup[AUX_DELAY] << 2; // Aux delay

    // Trigger time
    TA1CCTL2 = CCIE;				// CCR2 interrupt enabled
    TA1CCR2 = time_setup[TRIGGER_DELAY] << 2; // Trigger delay
    
    // Start the fast timer
    TA1CCR0 = CAPTURE_LENGTH; 
    
    // Count pictures
    if (time_setup[COUNT]) {
      time_trigger[COUNT]--;
      if (time_trigger[COUNT] == 0) {
        time_cfg.run = 0;
	time_cfg.armed = time_setup[EXTON];
      }
    } else {
      time_cfg.run = !time_setup[EXTON];	
    }
  }

  // Reset timer
  for (int8_t idx = TIMER; idx >= 0; idx--) {
    time_trigger[idx] = time_setup[idx];
  }
  LPM3_EXIT;
}

// CC1, CC2
__attribute__((__interrupt__(TIMER0_A1_VECTOR)))
static void TIMER0_A1_ISR(void)
{
  switch (TA0IV) {
    case 0x2:
      // Turn off backlight and then LCD power
      if (P1OUT & LCD_BACKLIGHT) {
        P1OUT &= ~LCD_BACKLIGHT;
        TA0CCR1 = TA0R + 512 * LCD_POWER_TIMEOUT_S;
        if (TA0CCR1 > TA0CCR0) {
          TA0CCR1 -= TA0CCR0;
        }
      } else {
        TA0CCTL1 &= ~CCIE;				// CCR1 interrupt disabled
        lcd_status.ready = 0;
	lcd_status.poweringdown = 1;
        LPM3_EXIT;
      }
      break;
    case 0x4:    
      // Turn off activity LED unless the capture is in progress
      if (P2OUT & FOCUS) P1OUT |= LED;
      break;
  }
}

/*
 * Timer A1 - 4096 steps / s
 * CC1 - activate AUX output (none present atm)
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

  // Pull-up when EXT enabled, down when not
  if (time_setup[EXTON]) {
    P1OUT |= EXT;
  } else {
    P1OUT &= ~EXT;
  }
}

// CC1, CC2
__attribute__((__interrupt__(TIMER1_A1_VECTOR)))
static void TIMER1_A1_ISR(void)
{
  switch (TA1IV) {
    case 0x2:
      P1OUT |= EXT; // Pull-up to notify AUX
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
      UCA0TXBUF = (*spi_ptr++) ^ spi_tx_invert;

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

void spi_send(uint16_t size, const uint8_t *data, uint8_t invert) {
  IFG2 &= ~UCA0RXIFG;
  spi_ptr = data;
  spi_count = spi_tx_count = size;
  spi_tx_invert = invert;
  IE2 |= UCA0TXIE | UCA0RXIE;
  _BIS_SR(GIE); // Enable interrupts
}

void lcd_send_cmd(const uint8_t *cmd) {
  spi_wait();
  P1OUT &= ~LCD_DC;

  uint8_t len = 0;
  const uint8_t *ptr = cmd;
  while (*ptr++) {
    len++;
  }

  spi_send(len, cmd, 0x00);
}

void lcd_send_data(uint16_t len, const uint8_t *cmd, uint8_t inversion) {
  spi_wait();

  P1OUT |= LCD_DC;

  spi_send(len, cmd, inversion);
}

void lcd_send_text_raw(const char *data, uint8_t inverse) {
  while (*data) {
    char chr = *data++;
    if (chr == ' ') draw_space();
    else {
      lcd_send_data(FONT_SIZE, FONT + FONT_SIZE * (chr - FONT_FIRST), inverse);
      lcd_send_data(1, empty_8B, inverse);
    }
  }
}

void lcd_send_text(const char *data) {
  lcd_send_text_raw(data, 0x00);
}

void lcd_send_text_inv(const char *data) {
  lcd_send_text_raw(data, 0xff);
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
