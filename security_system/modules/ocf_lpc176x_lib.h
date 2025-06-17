#ifndef OCF_LPC176X_LIB
#define OCF_LPC176X_LIB

#define RDR         (1<<0)
#define THRE        (1<<5)
#define MULVAL      15
#define DIVADDVAL   2
#define Ux_FIFO_EN  (1<<0)
#define Rx_FIFO_RST (1<<1)
#define Tx_FIFO_RST (1<<2)
#define DLAB_BIT    (1<<7)
#define LINE_FEED   0x0A
#define CARRIAGE_RETURN 0x0D

// POPRAWKA: Dynamiczne obliczanie PRESCALE na podstawie SystemCoreClock
extern uint32_t SystemCoreClock; // Zadeklarowane w system_LPC17xx.c

void initUART0(void);
void initTimer0(void);
void startTimer0(void);
unsigned int stopTimer0(void);
void delayUS(unsigned int microseconds);
void delayMS(unsigned int milliseconds);
void U0Write(char data);
char U0Read(void);

#endif
