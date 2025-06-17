/*************************
 *  1. oled
 *  2. spi i/f oled
 *  3. Czytnik RFID
 *  4. Czunijk ultradzwiekowy odlegosci
 *  5. Czujnik swiatła -> inversia ekranu
 *  6. Joystick
 *  7. timer
 *  8. pca9532 diody
 *  9. i2c i/f pca
 *  9.5. Glosniczek nutka 1/2
 *
 *  extra zewnetrzny sprzęt
 *************************/


#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_ssp.h"

#include "light.h"
#include "oled.h"
#include "temp.h"
#include "acc.h"
#include "pca9532.h"
#include "joystick.h"
#include "rc522.h"

#include "rgb.h"
#include <cr_section_macros.h>
#include <stdio.h>
                  

#define KONTAKTRON_PIN (10U)

// HC-SR04 pins
#define TRIG (1UL << 23U) // P0.23
#define ECHO (1UL << 24U) // P0.24 (z dzielnikiem napięcia!)

#define DARK_THRESHOLD (30U)   // 30 lux lub mniej = ciemno

static uint8_t isArmed = 0U;
static uint8_t isAuthorized = 0U;

// Autoryzowany UID - Twój dzyndzel
static const uint8_t authorizedUID[4] = {0xABU, 0x13U, 0xBAU, 0x0CU};

// Zmienne dla czujnika odległości
static uint8_t motionDetected = 0U;
static uint32_t motionCooldown = 0UL;

// **ZMIENNA - czy alarm został wyzwolony**
static uint8_t alarmTriggered = 0U;

// **ZMIENNE dla czujnika światła**
static uint8_t displayInverted = 0U;        // Aktualny stan inwersji
static uint32_t lightLevel = 0UL;            // Ostatni odczyt światła

#define NOTE_PIN_HIGH() GPIO_SetValue(0U, (1UL<<26U))
#define NOTE_PIN_LOW()  GPIO_ClearValue(0U, (1UL<<26U))

static volatile uint32_t tick_counter = 0UL;

static void GPIO_init_internal(void);
static void initUltrasonic_internal(void);
static float getUltrasonicDistance_internal(void);
static uint8_t checkMotionDetection_internal(void);
static void initAlarm_internal(void);
static void playNote_internal(uint32_t note, uint32_t durationMs);
static void runAlarm_internal(void);
static void init_ssp_internal(void);
static void init_i2c_internal(void);
static void moveBar_internal(uint16_t value);
static uint8_t checkRFID_internal(void);
static void writeCommand_internal(uint8_t data_byte);
static void oled_setInversion_internal(uint8_t inverted_state);
static void checkLightAndInversion_internal(void);
static void playConfirmationSound_internal(void);
static void playArmSound_internal(void);
static void playDisarmSound_internal(void);
static void handleJoystick_internal(uint8_t joyState);
static uint8_t is_closed_internal(void);
static void updateMainDisplay_internal(uint32_t authTime);


/*!
 *  @brief    Inicjalizuje pin dla kontaktronu jako wejście z podciąganiem.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Ustawia kierunek pinu GPIO i konfigurację podciągania.
 */


static void GPIO_init_internal(void)
{
    LPC_PINCON->PINSEL4 &= ~(3UL << 18U); 
    LPC_GPIO2->FIODIR &= ~(1UL << KONTAKTRON_PIN); 
    LPC_PINCON->PINMODE4 &= ~(3UL << 18U); 
                                         
}

/*!
 *  @brief    Inicjalizuje piny TRIG i ECHO czujnika ultradźwiękowego HC-SR04.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Zmienia konfigurację pinów P0.23 i P0.24.
 */

static void initUltrasonic_internal(void) {
    PINSEL_CFG_Type PinCfg;

    // Konfiguracja TRIG (P0.23) jako output
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PinCfg.Portnum = 0U;
    PinCfg.Pinnum = 23U;
    PINSEL_ConfigPin(&PinCfg);

    LPC_GPIO0->FIODIR |= TRIG;
    LPC_GPIO0->FIOCLR = TRIG; /* Initialize TRIG low */

    // Konfiguracja ECHO (P0.24) jako input
    PinCfg.Pinnum = 24U;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN; /* Use pull-down for ECHO */
    PINSEL_ConfigPin(&PinCfg);

    LPC_GPIO0->FIODIR &= ~(ECHO);
}


/*!
 *  @brief    Odczytuje odległość z czujnika ultradźwiękowego.
 *  @param    Brak
 *  @returns  Odległość w cm jako float.
 *            -1.0f – timeout oczekiwania na impuls,
 *            -2.0f – timeout trwania impulsu.
 *  @side effects:
 *            Generuje impuls TRIG i czeka na odpowiedź ECHO.
 */

static float getUltrasonicDistance_internal(void) {
    uint32_t timeout_val = 0UL;
    volatile uint32_t echo_pulse_width_counter = 0UL; 
    uint32_t i; 

    // Wyślij TRIG
    LPC_GPIO0->FIOSET = TRIG; /* TRIG wysoki */
    for(i = 0UL; i < 150UL; i++) { 
        __NOP();
    }
    LPC_GPIO0->FIOCLR = TRIG; /* TRIG niski */
    for(i = 0UL; i < 50UL; i++) {
        __NOP();
    }

    // Czekaj na ECHO start
    timeout_val = 0UL;
    while((LPC_GPIO0->FIOPIN & ECHO) == 0U) { 
        timeout_val++;
        if(timeout_val > 30000UL) {
            return -1.0f; // Timeout
        }
    }

    // Licz czas ECHO
    echo_pulse_width_counter = 0UL;
    while((LPC_GPIO0->FIOPIN & ECHO) != 0U) { 
        echo_pulse_width_counter++;
        if(echo_pulse_width_counter > 30000UL) { 
            return -2.0f; // Timeout or too long pulse
        }
    }

    // Konwersja na centymetry: Distance = (Pulse time * Speed of sound) / 2
    // Pulse time = echo_pulse_width_counter * (time per count)
    // Speed of sound = 34300 cm/s

    return (float)echo_pulse_width_counter / 100.0f;
}

/*!
 *  @brief    Sprawdza obecność obiektu w zasięgu czujnika ruchu.
 *  @param    Brak
 *  @returns  1 jeśli wykryto ruch, 0 w przeciwnym razie.
 *  @side effects:
 *            Może wypisać informacje przez printf.
 */


static uint8_t checkMotionDetection_internal(void) {
    float currentDistance = getUltrasonicDistance_internal();
    uint8_t detected_flag = 0U;

    // Jeśli błąd pomiaru - brak alarmu
    if (currentDistance < 0.0f) {
        detected_flag = 0U;
    }
    // **PROSTA LOGIKA: Coś w zasięgu <200cm = ALARM!**
    else if ((currentDistance >= 5.0f) && (currentDistance < 200.0f)) {
        // **Loguj tylko gdy NIE autoryzowany**
        if (isAuthorized == 0U) {
            (void)printf("OBIEKT WYKRYTY! Odleglosc: %.1f cm\n", currentDistance);
        }
        detected_flag = 1U; // ALARM!
    }
    else {
        detected_flag = 0U; // Poza zasięgiem lub za blisko - spokój
    }
    return detected_flag;
}

/*!
 *  @brief    Inicjalizuje piny do obsługi wzmacniacza audio LM4811 i głośnika.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Konfiguruje piny jako wyjścia i ustawia stany początkowe.
 */


static void initAlarm_internal(void) {
    // Inicjalizacja pinów LM4811 Audio Amplifier
    GPIO_SetDir(0U, (1UL<<27U), 1U);   // LM4811-clk (P0.27) - output
    GPIO_SetDir(0U, (1UL<<28U), 1U);   // LM4811-up/dn (P0.28) - output
    GPIO_SetDir(2U, (1UL<<13U), 1U);   // LM4811-shutdn (P2.13) - output
    GPIO_SetDir(0U, (1UL<<26U), 1U);   // Speaker output (P0.26) - output

    // Ustawienie pinów LM4811
    GPIO_ClearValue(0U, (1UL<<27U));  // LM4811-clk = LOW
    GPIO_ClearValue(0U, (1UL<<28U));  // LM4811-up/dn = LOW
    GPIO_ClearValue(2U, (1UL<<13U));  // LM4811-shutdn = LOW (WŁĄCZ wzmacniacz!)
}

/*!
 *  @brief    Generuje sygnał dźwiękowy o zadanym okresie i czasie trwania.
 *  @param    note  Okres sygnału (µs).
 *  @param    durationMs  Czas trwania (ms).
 *  @returns  Brak
 *  @side effects:
 *            Steruje pinem P0.26 generując sygnał dźwiękowy.
 */


static void playNote_internal(uint32_t note_period_us, uint32_t duration_ms) {
    uint32_t time_elapsed_us = 0UL;
    uint32_t duration_us = duration_ms * 1000UL;

    if (note_period_us > 0UL) {
        while (time_elapsed_us < duration_us) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note_period_us / 2UL);
            NOTE_PIN_LOW();
            Timer0_us_Wait(note_period_us / 2UL);
            time_elapsed_us += note_period_us;
        }
    } else {
        Timer0_Wait(duration_ms);
    }
}

/*!
 *  @brief    Odtwarza dźwięk alarmu, jeśli system jest uzbrojony i nieautoryzowany.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Wywołuje playNote_internal dwukrotnie z różnymi tonami.
 */


static void runAlarm_internal(void) {
    uint32_t highNote_period = 1136UL;
    uint32_t lowNote_period  = 3816UL; 

    // Alarm tylko gdy uzbrojony i nieupoważniony
    if ((isArmed != 0U) && (isAuthorized == 0U)) {
        playNote_internal(highNote_period, 200UL);
        playNote_internal(lowNote_period, 200UL);
    }
}

/*!
 *  @brief    Inicjalizuje interfejs SSP do komunikacji np. z OLED i RC522.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Ustawia piny MOSI, MISO, SCK i CS jako SSP lub GPIO.
 */


static void init_ssp_internal(void)
{
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    PinCfg.Funcnum = PINSEL_FUNC_2; 
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE; 
    PinCfg.Portnum = 0U;
    PinCfg.Pinnum = 7U;
    PINSEL_ConfigPin(&PinCfg);


    PinCfg.Pinnum = 8U;
    PINSEL_ConfigPin(&PinCfg);

    PinCfg.Pinnum = 9U;
    PINSEL_ConfigPin(&PinCfg);

    PinCfg.Funcnum = PINSEL_FUNC_0; 
    PinCfg.Portnum = 0U; 
    PinCfg.Pinnum = 6U;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0U, (1UL << 6U), 1U); 
    GPIO_SetValue(0U, (1UL << 6U));

    SSP_ConfigStructInit(&SSP_ConfigStruct);
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
    (void)SSP_Cmd(LPC_SSP1, ENABLE);
}

/*!
 *  @brief    Inicjalizuje interfejs I2C2 dla komunikacji np. z PCA9532.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Ustawia piny P0.10 i P0.11 jako linie SDA i SCL, uruchamia I2C.
 */


static void init_i2c_internal(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = PINSEL_FUNC_2; 
    PinCfg.Pinnum = 10U;
    PinCfg.Portnum = 0U;
    PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN; 
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE; 
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11U;
    PINSEL_ConfigPin(&PinCfg);

    I2C_Init(LPC_I2C2, 100000UL); // 100kHz
    I2C_Cmd(LPC_I2C2, ENABLE);
}

/*!
 *  @brief    Steruje paskiem diod LED przez PCA9532.
 *  @param    value  Maska bitowa określająca, które LEDy mają się świecić.
 *  @returns  Brak
 *  @side effects:
 *            Wywołuje pca9532_setLeds.
 */


//main.c
static void moveBar_internal(uint16_t value)
{
    pca9532_setLeds(value, 0xFFFFU);
}

/*!
 *  @brief    Sprawdza obecność i UID karty RFID.
 *  @param    Brak
 *  @returns  1 jeśli karta pasuje do autoryzowanej, 0 w przeciwnym razie.
 *  @side effects:
 *            Może wywołać rc522_request, rc522_anticoll, rc522_halt.
 */


static uint8_t checkRFID_internal(void) {
    uint8_t status;
    uint8_t str[MAX_LEN];
    uint8_t serNum[5]; 
    uint8_t uid_match = 0U;
    uint8_t i;

    status = rc522_request(PICC_REQIDL, str);
    if (status == 0) {
        status = rc522_anticoll(serNum);
        if (status == 0) {
            // Sprawdź czy UID pasuje do autoryzowanego (first 4 bytes)
            uid_match = 1U;
            for (i = 0U; i < 4U; i++) {
                if (serNum[i] != authorizedUID[i]) {
                    uid_match = 0U;
                    break;
                }
            }
            rc522_halt();
        }
    }
    return uid_match; // 1 = autoryzowana karta, 0 = nie lub błąd
}

/*!
 *  @brief    Wysyła bajt komendy do OLED przez SSP.
 *  @param    data_byte  Komenda do wysłania.
 *  @returns  Brak
 *  @side effects:
 *            Ustawia piny CS i D/C OLED, używa SSP do transmisji.
 */

static void writeCommand_internal(uint8_t data_byte)
{
    SSP_DATA_SETUP_Type xferConfig;

    GPIO_ClearValue(2, (1U<<7U)); 
                                 
                                 
                                

    GPIO_ClearValue(0, (1U<<6U)); 

    xferConfig.tx_data = &data_byte;
    xferConfig.rx_data = NULL;
    xferConfig.length  = 1U;

    (void)SSP_ReadWrite(LPC_SSP1, &xferConfig, SSP_TRANSFER_POLLING);

    GPIO_SetValue(0, (1U<<6U));   
}

/*!
 *  @brief    Ustawia tryb inwersji wyświetlacza OLED.
 *  @param    inverted_state  1 - inwersja, 0 - normalny tryb.
 *  @returns  Brak
 *  @side effects:
 *            Aktualizuje stan globalny i wysyła komendę do OLED.
 */
 
static void oled_setInversion_internal(uint8_t inverted_state) {
    if (inverted_state != 0U) {
        writeCommand_internal(0xA7U);  // Włącz inwersję
        displayInverted = 1U;
    } else {
        writeCommand_internal(0xA6U);  // Wyłącz inwersję
        displayInverted = 0U;
    }
}

/*!
 *  @brief    Odczytuje poziom światła i przełącza tryb OLED na nocny lub dzienny.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Może zmienić stan inwersji OLED, odczytuje light_read.
 */

static void checkLightAndInversion_internal(void) {
    uint8_t shouldBeInverted = 0U;
 
    lightLevel = light_read();

    if (lightLevel <= DARK_THRESHOLD) {
        shouldBeInverted = 1U;  // Tryb nocny
    } else {
        shouldBeInverted = 0U;  // Tryb dzienny
    }

    if (shouldBeInverted != displayInverted) {
        oled_setInversion_internal(shouldBeInverted);
    }
}

/*!
 *  @brief    Odtwarza sekwencję dźwięków potwierdzającą autoryzację.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Odtwarza 3 tony przez głośnik.
 */

static void playConfirmationSound_internal(void) {
    playNote_internal(1500UL, 100UL);
    Timer0_Wait(50UL);
    playNote_internal(1000UL, 100UL);
    Timer0_Wait(50UL);
    playNote_internal(750UL, 150UL);
}

/*!
 *  @brief    Odtwarza dźwięk uzbrojenia systemu.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Wydaje 2 dźwięki przez głośnik.
 */


static void playArmSound_internal(void) {
    playNote_internal(800UL, 200UL);
    Timer0_Wait(100UL);
    playNote_internal(1200UL, 200UL);
}

/*!
 *  @brief    Odtwarza dźwięk rozbrojenia systemu.
 *  @param    Brak
 *  @returns  Brak
 *  @side effects:
 *            Wydaje 2 dźwięki przez głośnik.
 */


static void playDisarmSound_internal(void) {
    playNote_internal(1200UL, 200UL);
    Timer0_Wait(100UL);
    playNote_internal(800UL, 200UL);
}

/*!
 *  @brief    Obsługuje stan joysticka i przełącza tryb uzbrojenia systemu.
 *  @param    joyState  Stan odczytany z joysticka.
 *  @returns  Brak
 *  @side effects:
 *            Zmienia stan systemu, aktualizuje ekran OLED, steruje dźwiękiem i LED.
 */


static void handleJoystick_internal(uint8_t joyState)
{
    // Check if joystick center is pressed
    if ((joyState & JOYSTICK_CENTER) != 0U) {
        if (isArmed != 0U) {
            // **WYŁĄCZ CAŁY ALARM - resetuj wszystkie flagi**
            isArmed = 0U;
            isAuthorized = 0U;
            motionDetected = 0U;
            motionCooldown = 0UL;
            alarmTriggered = 0U;
            playDisarmSound_internal();

            oled_clearScreen(OLED_COLOR_WHITE);
            oled_putString(1,1, (uint8_t*)"SYSTEM ALARMOWY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,9, (uint8_t*)"WYLACZONY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,18, (uint8_t*)"BEZPIECZNY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,27, (uint8_t*)"Czujnik: OFF", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

            //main.c
            moveBar_internal(0U);
            rgb_setLeds(RGB_GREEN);

        } else {
            // **WŁĄCZ ALARM - wyczyść flagę alarmu**
            isArmed = 1U;
            isAuthorized = 0U;
            motionDetected = 0U;
            motionCooldown = 0UL;
            alarmTriggered = 0U;
            playArmSound_internal();

            oled_clearScreen(OLED_COLOR_WHITE);
            oled_putString(1,1, (uint8_t*)"SYSTEM ALARMOWY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,9, (uint8_t*)"ZABEZPIECZONY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,18, (uint8_t*)"Zasieg: <200cm", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,27, (uint8_t*)"Gotowy!", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

            rgb_setLeds(RGB_GREEN);
        }
        Timer0_Wait(1500UL);
    }
}

/*!
 *  @brief    Sprawdza stan kontaktronu (czy drzwi są zamknięte).
 *  @param    Brak
 *  @returns  1 jeśli zamknięte, 0 jeśli otwarte.
 *  @side effects:
 *            Odczytuje stan pinu GPIO2.
 */


static uint8_t is_closed_internal(void)
{
   
    return ((LPC_GPIO2->FIOPIN & (1UL << KONTAKTRON_PIN)) == 0U) ? 1U : 0U;
}

/*!
 *  @brief    Aktualizuje ekran OLED na podstawie stanu systemu alarmowego.
 *  @param    authTime  Czas pozostały autoryzacji w tickach (100ms).
 *  @returns  Brak
 *  @side effects:
 *            Czyści i zapisuje dane na ekranie OLED.
 */


static void updateMainDisplay_internal(uint32_t authTime_ticks) {
    char statusLine[20];
    char lightInfo[20]; 
    int_fast8_t ret_sprintf; 

    oled_clearScreen(OLED_COLOR_WHITE);

    if (isArmed != 0U) {
        if (isAuthorized != 0U) {
            
            oled_putString(1,1, (uint8_t*)"SYSTEM ALARMOWY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            oled_putString(1,9, (uint8_t*)"AUTORYZOWANY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

            ret_sprintf = sprintf(statusLine, "Pozostalo: %lus", (authTime_ticks + 9UL) / 10UL);
            if (ret_sprintf > 0) {
                 oled_putString(1,18, (uint8_t*)statusLine, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            }

        } else {
            // **ZABEZPIECZONY lub ALARM**
            oled_putString(1,1, (uint8_t*)"SYSTEM ALARMOWY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

            if (alarmTriggered != 0U) {
                oled_putString(1,9, (uint8_t*)"ALARM!", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                oled_putString(1,18, (uint8_t*)"Naruszenie strefy", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                oled_putString(1,27, (uint8_t*)"Przyloz RFID!", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
            } else {
                oled_putString(1,9, (uint8_t*)"ZABEZPIECZONY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                oled_putString(1,18, (uint8_t*)"Zasieg: <200cm", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

                // Status drzwi i czujnika
                if (is_closed_internal() == 0U) { 
                    oled_putString(1,27, (uint8_t*)"Drzwi: OTWARTE!", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                } else if ((motionDetected != 0U) && (motionCooldown > 0UL)) {
                    oled_putString(1,27, (uint8_t*)"Obiekt wykryty!", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                } else {
                     oled_putString(1,27, (uint8_t*)"Gotowy!", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
                }
            }
        }
    } else {
        // **SYSTEM WYŁĄCZONY**
        oled_putString(1,1, (uint8_t*)"SYSTEM ALARMOWY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
        oled_putString(1,9, (uint8_t*)"WYLACZONY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
        oled_putString(1,18, (uint8_t*)"BEZPIECZNY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    }
    (void)lightInfo;
}

int main (void) {
    uint8_t joy_state = 0U;
    uint8_t lastRFIDState = 0U;
    uint32_t authorizationTimer_ticks = 0UL; 
    uint32_t motionCheckTimer_ticks = 0UL;   
    uint32_t displayUpdateTimer_ticks = 0UL; 
    uint32_t lightCheckTimer_ticks = 0UL;    

    GPIO_init_internal();
    init_i2c_internal();
    init_ssp_internal();
    oled_init(); 
    rgb_init();
    initAlarm_internal();
    initUltrasonic_internal();
    rc522_init();

    light_init();
    light_enable();

    // Ekran startowy
    oled_clearScreen(OLED_COLOR_WHITE);
    oled_putString(1,1, (uint8_t*)"SYSTEM ALARMOWY", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,9, (uint8_t*)"Z RFID + MOTION", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,18, (uint8_t*)"+ AUTO INWERSJA", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    oled_putString(1,27, (uint8_t*)"Gotowy!", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
    Timer0_Wait(2000UL);

    checkLightAndInversion_internal(); // Pierwsze sprawdzenie światła

    moveBar_internal(0U);
    rgb_setLeds(RGB_GREEN); 

    while(1) {
        joy_state = joystick_read();

        // Obsługa joysticka
        if (joy_state != 0U) {
            handleJoystick_internal(joy_state);
            displayUpdateTimer_ticks = 0UL; 
        }

        // **SPRAWDZENIE CZUJNIKA ŚWIATŁA - co 3 sekundy**
        if (lightCheckTimer_ticks >= 30UL) { // 30 * 100ms = 3 sekundy
            lightCheckTimer_ticks = 0UL;
            checkLightAndInversion_internal();
        } else {
            lightCheckTimer_ticks++;
        }

        if (isArmed != 0U) {
            uint8_t currentRFIDState = checkRFID_internal();

            if ((currentRFIDState != 0U) && (lastRFIDState == 0U)) { 
                isAuthorized = 1U;
                authorizationTimer_ticks = 100UL; // 10 sekund (100 * 100ms)
                motionDetected = 0U;
                motionCooldown = 0UL;
                alarmTriggered = 0U;

                playConfirmationSound_internal();
                (void)printf("RFID: Autoryzacja na 10 sekund - alarm wyciszony\n");
                displayUpdateTimer_ticks = 0UL; 
            }
            lastRFIDState = currentRFIDState;

            if ((isAuthorized != 0U) && (authorizationTimer_ticks > 0UL)) {
                authorizationTimer_ticks--;
                if (authorizationTimer_ticks == 0UL) {
                    isAuthorized = 0U;
                    (void)printf("Autoryzacja wygasla - powrot do monitorowania\n");
                    displayUpdateTimer_ticks = 0UL; //
                }
            }

            // **SPRAWDZANIE CZUJNIKA - co 200ms**
            if (motionCheckTimer_ticks >= 2UL) { // 2 * 100ms = 200ms
                motionCheckTimer_ticks = 0UL;

                if (checkMotionDetection_internal() != 0U) {
                    motionDetected = 1U;
                    motionCooldown = 30UL; // 3 sekundy cooldown (30 * 100ms)

                    if (isAuthorized == 0U) {
                        alarmTriggered = 1U;
                        (void)printf("ALARM: Obiekt w strefie ochronnej - ALARM WYZWOLONY!\n");
                    }
                    displayUpdateTimer_ticks = 0UL;
                }
            } else {
                motionCheckTimer_ticks++;
            }

            // **Sprawdź drzwi - ustaw flagę alarmu gdy otwarte i nie autoryzowany**
            if ((is_closed_internal() == 0U) && (isAuthorized == 0U)) { // 0U means open
                if (alarmTriggered == 0U) { // Trigger only once
                    alarmTriggered = 1U;
                    (void)printf("ALARM: Drzwi otwarte - ALARM WYZWOLONY!\n");
                    displayUpdateTimer_ticks = 0UL;
                }
            }

            if (motionCooldown > 0UL) {
                motionCooldown--;
                if (motionCooldown == 0UL) {
                    motionDetected = 0U;
                    if (isAuthorized == 0U) {
                        // (void)printf("Strefa czysta (ale alarm dalej aktywny jesli triggered)\n");
                    }
                    displayUpdateTimer_ticks = 0UL;
                }
            }

        } else { 
            isAuthorized = 0U;
            lastRFIDState = 0U;
            authorizationTimer_ticks = 0UL;
            motionDetected = 0U;
            motionCooldown = 0UL;
            motionCheckTimer_ticks = 0UL;
            alarmTriggered = 0U;
        }

        if (displayUpdateTimer_ticks >= 5UL) { // 5 * 100ms = 500ms
            updateMainDisplay_internal(authorizationTimer_ticks);
            displayUpdateTimer_ticks = 0UL;
        } else {
            displayUpdateTimer_ticks++;
        }

        // **MIGANIE i ALARM - na podstawie flagi alarmTriggered**
        if ((isArmed != 0U) && (isAuthorized == 0U) && (alarmTriggered != 0U)) {
            if (tick_counter == 0U) {
                moveBar_internal(0xFFU);
                tick_counter = 1U;
            } else {
                moveBar_internal(0xFF00U);
                tick_counter = 0U;
            }
            runAlarm_internal();
        } else {
            moveBar_internal(0U); 
            tick_counter = 0U;
        }

        Timer0_Wait(100UL); // Główna pętla co 100ms
    }
}
