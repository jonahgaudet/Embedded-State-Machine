#include "msp.h"
#include <stdbool.h>

#define DELAY 20000
#define STATE_1 0
#define STATE_2 1
#define STATE_3 2
#define STATE_4 3
#define INPUT_0 '0'
#define INPUT_1 '1'
#define INPUT_2 '2'
#define STATE_VALUES "1234"

// MARK: Variable Definitions
static uint8_t currentState;

// MARK: Function prototypes
static void applyCurrentState(void);
static void resetBoard(void);
static void handleStateChange(bool);

// MARK: ISR Prototypes
void PORT1_IRQHandler(void);
void EUSCIA0_IRQHandler(void);

/**
 * Handles the changing of the state. increase_state is true if the
 * state is increasing, false otherwise
 * Transmits state
 */
void handleStateChange (bool increase_state)
{
	if (increase_state) {
		currentState = (currentState + 1) % 4;
	} else {
		currentState = (currentState - 1 < STATE_1) ? STATE_4 : currentState - 1;
	}
	applyCurrentState(); 
	EUSCI_A0->TXBUF = STATE_VALUES[currentState];
}

/**
 * Resets the board to the first state and applies it
 * Transmits current state
 */
void resetBoard (void) {
	currentState = STATE_1;
	applyCurrentState();
	EUSCI_A0->TXBUF = STATE_VALUES[currentState];
}

/**
 *  Ensure that both the SEL0 and SEL1 bits for a GPIO pin are cleared without
 *  the possibility of going through an intermediate state.
 *
 *  @param port Base address for the GPIO port that the pin is in.
 *  @param mask Bit mask for the GPIO pin.
 */
void ensure_func_gpio_odd (DIO_PORT_Odd_Interruptable_Type *port,
                                         uint8_t mask)
{
    if ((port->SEL0 & mask) && (port->SEL1 & mask)) {
        port->SELC |= mask;
    } else {
        port->SEL0 &= ~(mask);
        port->SEL1 &= ~(mask);
    }
}

/**
 *  Ensure that both the SEL0 and SEL1 bits for a GPIO pin are cleared without
 *  the possibility of going through an intermediate state.
 *
 *  @param port Base address for the GPIO port that the pin is in.
 *  @param mask Bit mask for the GPIO pin.
 */
void ensure_func_gpio_even (DIO_PORT_Even_Interruptable_Type *port,
                                          uint8_t mask)
{
    if ((port->SEL0 & mask) && (port->SEL1 & mask)) {
        port->SELC |= mask;
    } else {
        port->SEL0 &= ~(mask);
        port->SEL1 &= ~(mask);
    }
}

/**
 * Configure inputs and outputs so that P1.1, P1.4 are outputs and 
 * P1.0 and P2.0 are inputs. Sets resitors accordingly
 */
void configureInputsAndOutputs () 
{
	
	//Ensure GPIO
	ensure_func_gpio_odd(P1, (1<<1));
  ensure_func_gpio_odd(P1, (1<<4));
	P1DIR &=(uint8_t)(~((1<<4) | (1<<1)));	//P1.1, P1.4 as inputs
	P1REN |=(uint8_t)((1<<4) | (1<<1));			//Enable resistors
	P1OUT |=(uint8_t)((1<<4) | (1<<1));			//Set resistors pull-up for inputs
	P1->IE &= ~((1<<1) | (1<<4));						//Disable interrupts
	
	//Ensure GPIO
	ensure_func_gpio_odd(P1, (1<<0));
  ensure_func_gpio_even(P2, (1<<0));
	
	P1DIR |=(uint8_t)(1<<0);			//P1.0, P2.0 as outputs
	P2DIR |=(uint8_t)((1<<0));
	P1->DS &= ~(1<<0);						//High drive strength disabled
  P2->DS &= ~((1<<0));
	P1OUT &=~(uint8_t)(1<<0);			//Set default outputs to 0
	P2OUT &=~(uint8_t)((1<<0));	
	P1->IE &= ~((1<<0));					//Disable interrupts
  P2->IE &= ~((1<<0));
}

void UART0_init(void) 
{
	CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
  CS->CTL0 = 0;                           // Reset tuning parameters
  CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
  CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
          CS_CTL1_SELS_3 |                // SMCLK = DCO
          CS_CTL1_SELM_3;                 // MCLK = DCO
  CS->KEY = 0;                            // Lock CS module from unintended accesses

  // Configure UART pins
  P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

  // Configure UART
  EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
  EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
          EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
  // Baud Rate calculation
  // 12000000/(16*9600) = 78.125
  // Fractional portion = 0.125
  // User's Guide Table 21-4: UCBRSx = 0x10
  // UCBRFx = int ( (78.125-78)*16) = 2
  EUSCI_A0->BRW = 78;                     // 12000000/16/9600
  EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
          EUSCI_A_MCTLW_OS16;

  EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;	// Initialize eUSCI
  EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    	// Clear eUSCI RX interrupt flag
  EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        	// Enable USCI_A0 RX interrupt

  SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;  		// Enable sleep on exit from ISR

  __enable_irq();  													// Enable global interrupt

  NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);  // Enable eUSCIA0 interrupt in NVIC module
}

int main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // Stop watchdog timer
	
	currentState = STATE_1;
	configureInputsAndOutputs();
	UART0_init();
	
	//Configure button interrupts
	P1->IES |= (uint8_t)((1<<4) | (1<<1)); 					//Device interrupt configuration
	P1->IFG &= (uint8_t)(~((1<<4) | (1<<1))); 
	P1->IE |= (uint8_t)((1<<4) | (1<<1));
	
	NVIC_SetPriority(PORT1_IRQn, 2);								//NVIC configuration
	NVIC_ClearPendingIRQ(PORT1_IRQn);
	NVIC_EnableIRQ(PORT1_IRQn);
	
	__ASM("CPSIE I");																//Globally enable interrupts in CPU
  
	while(1)
  {		

  }
}

/**
 * Applies the current state using global variable currentState
 */
void applyCurrentState () 
{
	switch (currentState)
	{
		case STATE_1:								//Both LEDs off
      P1OUT &=~(uint8_t)(1<<0);
			P2OUT &=~(uint8_t)(1<<0);
      break;
    case STATE_2:								//P1.0 on, P2.0 off
      P1OUT |= (uint8_t)(1<<0);
			P2OUT &=~(uint8_t)(1<<0);
      break;
		case STATE_3:								//P1.0 off, P2.o on
      P1OUT &=~(uint8_t)(1<<0);
			P2OUT |= (uint8_t)(1<<0);
      break;
		case STATE_4:								//Both LEDs on
      P1OUT |= (uint8_t)(1<<0);
			P2OUT |= (uint8_t)(1<<0);
      break;
    default:										//Sets state to STATE_1, applies the state
      currentState = STATE_1;
			applyCurrentState();
			break;
	}
}

/* Interrupt Service Routines */
void PORT1_IRQHandler (void) 
{
	int i = DELAY;
	while (i > 0) {i--;}											// Debouncing
	
	if (P1->IN&BIT1 && P1->IN&BIT4) {					//Clears flags
		P1->IFG &= (uint8_t)(~((1<<4) | (1<<1)));
		return;
	}

	if ((P1IFG & (uint8_t)BIT1) != 0) { 			// P1.1, move forward
		P1->IFG &= (uint8_t)(~((1<<1)));
		handleStateChange(true);
	}
	else if ((P1IFG & (uint8_t)BIT4) != 0) {	//P1.4, move backwards
		P1->IFG &= (uint8_t)(~((1<<4)));
		handleStateChange(false);
	}
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{
	if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
  {
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));	// Check if the TX buffer is empty first
		
		if (EUSCI_A0->RXBUF == INPUT_0) {			// Resets if 0 is sent
			resetBoard();
		}
		
		//Modifies state IFF RXBUF is valid input
		if (EUSCI_A0->RXBUF == INPUT_1 || EUSCI_A0->RXBUF == INPUT_2) {
			handleStateChange(EUSCI_A0->RXBUF != INPUT_1);
		}
  }
}
