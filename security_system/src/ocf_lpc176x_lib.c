#include <lpc17xx.h>
#include "ocf_lpc176x_lib.h"
#include <stdio.h>

extern uint32_t SystemCoreClock;

//Retarget printf()
struct __FILE
{
  int dummyVar;
};

FILE __stdout;
FILE __stdin;

int fputc(int c, FILE * stream)
{
	U0Write(c);
	return c;
}

int fgetc(FILE * stream)
{
	char c = U0Read();
	U0Write(c);
	return c;
}

void initUART0(void)
{
	LPC_PINCON->PINSEL0 |= (1<<4) | (1<<6);
	LPC_UART0->LCR = 3 | DLAB_BIT;
	LPC_UART0->DLL = 12;
	LPC_UART0->DLM = 0;
	LPC_UART0->FCR |= Ux_FIFO_EN | Rx_FIFO_RST | Tx_FIFO_RST;
	LPC_UART0->FDR = (MULVAL<<4) | DIVADDVAL;
	LPC_UART0->LCR &= ~(DLAB_BIT);
}

void initTimer0(void)
{
	// UPROSZCZONA INICJALIZACJA Timer0
	// Włącz zasilanie Timer0
	LPC_SC->PCONP |= (1 << 1); // Bit 1 dla Timer0

	// Ustaw PCLK dla Timer0 na CCLK (najwyższa rozdzielczość)
	LPC_SC->PCLKSEL0 = (LPC_SC->PCLKSEL0 & ~(0x3 << 2)) | (0x01 << 2);

	LPC_TIM0->CTCR = 0x0; // Timer mode
	LPC_TIM0->PR = (SystemCoreClock / 1000000) - 1; // 1µs resolution
	LPC_TIM0->TCR = 0x02; // Reset Timer
}

void startTimer0(void)
{
	LPC_TIM0->TCR = 0x02; // Reset Timer
	LPC_TIM0->TCR = 0x01; // Enable timer
}

unsigned int stopTimer0(void)
{
	LPC_TIM0->TCR = 0x00; // Disable timer
	return LPC_TIM0->TC;
}

// NOWA IMPLEMENTACJA - bez Timer0
void delayUS(unsigned int microseconds)
{
	// Metoda 1: Proste opóźnienie CPU
	// Przy SystemCoreClock=100MHz, każda iteracja ~10ns
	// Dla 1µs potrzebujemy ~100 iteracji
	volatile unsigned int count = microseconds * (SystemCoreClock / 10000000);
	while(count--) {
		__NOP(); // No operation - 1 cykl CPU
	}
}

void delayMS(unsigned int milliseconds)
{
	// Dla dużych opóźnień użyj pętli z mniejszymi fragmentami
	for(unsigned int i = 0; i < milliseconds; i++) {
		delayUS(1000); // 1ms = 1000µs
	}
}

void U0Write(char cChar)
{
	 while ( !(LPC_UART0->LSR & THRE) );
	 if( cChar == '\n' )
	 {
			LPC_UART0->THR = CARRIAGE_RETURN;
			while( !(LPC_UART0->LSR & THRE ));
			LPC_UART0->THR = LINE_FEED;
	 }
	 else
	 {
			 LPC_UART0->THR = cChar;
	 }
}

char U0Read(void)
{
	 while( !(LPC_UART0->LSR & RDR ));
   return LPC_UART0->RBR;
}
