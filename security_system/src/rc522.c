#include "rc522.h"
#include "lpc17xx_timer.h"

#define RC522_CS_LOW()   GPIO_ClearValue(RC522_CS_PORT, 1<<RC522_CS_PIN)
#define RC522_CS_HIGH()  GPIO_SetValue(RC522_CS_PORT, 1<<RC522_CS_PIN)
#define RC522_RST_LOW()  GPIO_ClearValue(RC522_RST_PORT, 1<<RC522_RST_PIN)
#define RC522_RST_HIGH() GPIO_SetValue(RC522_RST_PORT, 1<<RC522_RST_PIN)

void rc522_init(void) {
    // Konfiguracja pinów CS i RST
    GPIO_SetDir(RC522_CS_PORT, 1<<RC522_CS_PIN, 1);   // CS jako output
    GPIO_SetDir(RC522_RST_PORT, 1<<RC522_RST_PIN, 1); // RST jako output

    RC522_CS_HIGH();
    RC522_RST_HIGH();

    Timer0_Wait(50);
    rc522_reset();

    // Konfiguracja timera
    rc522_write(TModeReg, 0x8D);
    rc522_write(TPrescalerReg, 0x3E);
    rc522_write(TReloadRegL, 30);
    rc522_write(TReloadRegH, 0);

    rc522_write(TxAutoReg, 0x40);    // 100%ASK
    rc522_write(ModeReg, 0x3D);      // CRC initial value 0x6363

    rc522_antennaOn();
}

void rc522_write(uint8_t addr, uint8_t val) {
    SSP_DATA_SETUP_Type xferConfig;
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = (addr << 1) & 0x7E;  // Adres z bitem zapisu
    tx_buf[1] = val;

    RC522_CS_LOW();

    xferConfig.tx_data = tx_buf;
    xferConfig.rx_data = rx_buf;
    xferConfig.length = 2;

    SSP_ReadWrite(LPC_SSP1, &xferConfig, SSP_TRANSFER_POLLING);

    RC522_CS_HIGH();
}

uint8_t rc522_read(uint8_t addr) {
    SSP_DATA_SETUP_Type xferConfig;
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = ((addr << 1) & 0x7E) | 0x80;  // Adres z bitem odczytu
    tx_buf[1] = 0x00;

    RC522_CS_LOW();

    xferConfig.tx_data = tx_buf;
    xferConfig.rx_data = rx_buf;
    xferConfig.length = 2;

    SSP_ReadWrite(LPC_SSP1, &xferConfig, SSP_TRANSFER_POLLING);

    RC522_CS_HIGH();

    return rx_buf[1];
}

void rc522_setBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp;
    tmp = rc522_read(reg);
    rc522_write(reg, tmp | mask);
}

void rc522_clearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t tmp;
    tmp = rc522_read(reg);
    rc522_write(reg, tmp & (~mask));
}

void rc522_antennaOn(void) {
    uint8_t temp;
    temp = rc522_read(TxControlReg);
    if (!(temp & 0x03)) {
        rc522_setBitMask(TxControlReg, 0x03);
    }
}

void rc522_antennaOff(void) {
    rc522_clearBitMask(TxControlReg, 0x03);
}

void rc522_reset(void) {
    rc522_write(CommandReg, PCD_RESETPHASE);
}

uint8_t rc522_request(uint8_t reqMode, uint8_t *TagType) {
    uint8_t status;
    uint16_t backBits;

    rc522_write(BitFramingReg, 0x07);

    TagType[0] = reqMode;
    status = rc522_toCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != 0) || (backBits != 0x10)) {
        status = 1;
    }

    return status;
}

uint8_t rc522_toCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen) {
    uint8_t status = 1;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch (command) {
        case PCD_AUTHENT:
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        case PCD_TRANSCEIVE:
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        default:
            break;
    }

    rc522_write(ComIEnReg, irqEn | 0x80);
    rc522_clearBitMask(ComIrqReg, 0x80);
    rc522_setBitMask(FIFOLevelReg, 0x80);

    rc522_write(CommandReg, PCD_IDLE);

    // Writing data to the FIFO
    for (i = 0; i < sendLen; i++) {
        rc522_write(FIFODataReg, sendData[i]);
    }

    // Execute the command
    rc522_write(CommandReg, command);
    if (command == PCD_TRANSCEIVE) {
        rc522_setBitMask(BitFramingReg, 0x80);
    }

    // Waiting to receive data to complete
    i = 2000;
    do {
        n = rc522_read(ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    rc522_clearBitMask(BitFramingReg, 0x80);

    if (i != 0) {
        if (!(rc522_read(ErrorReg) & 0x1B)) {
            status = 0;
            if (n & irqEn & 0x01) {
                status = 1;
            }

            if (command == PCD_TRANSCEIVE) {
                n = rc522_read(FIFOLevelReg);
                lastBits = rc522_read(ControlReg) & 0x07;
                if (lastBits) {
                    *backLen = (n - 1) * 8 + lastBits;
                } else {
                    *backLen = n * 8;
                }

                if (n == 0) {
                    n = 1;
                }
                if (n > MAX_LEN) {
                    n = MAX_LEN;
                }

                // Reading the received data in FIFO
                for (i = 0; i < n; i++) {
                    backData[i] = rc522_read(FIFODataReg);
                }
            }
        } else {
            status = 1;
        }
    }

    return status;
}

uint8_t rc522_anticoll(uint8_t *serNum) {
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint16_t unLen;

    rc522_write(BitFramingReg, 0x00);

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = rc522_toCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == 0) {
        // Check card serial number
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i]) {
            status = 1;
        }
    }

    return status;
}

uint8_t rc522_selectTag(uint8_t *serNum) {
    uint8_t status;
    uint8_t size;
    uint16_t recvBits;
    uint8_t buffer[9];

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (uint8_t i = 0; i < 5; i++) {
        buffer[i + 2] = *(serNum + i);
    }
    rc522_calculateCRC(buffer, 7, &buffer[7]);
    status = rc522_toCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ((status == 0) && (recvBits == 0x18)) {
        size = buffer[0];
    } else {
        size = 0;
    }

    return size;
}

void rc522_calculateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData) {
    uint8_t i, n;

    rc522_clearBitMask(DivIrqReg, 0x04);
    rc522_setBitMask(FIFOLevelReg, 0x80);

    // Writing data to the FIFO
    for (i = 0; i < len; i++) {
        rc522_write(FIFODataReg, *(pIndata + i));
    }
    rc522_write(CommandReg, PCD_CALCCRC);

    // Wait for the CRC calculation to complete
    i = 0xFF;
    do {
        n = rc522_read(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));

    // Read the CRC calculation result
    pOutData[0] = rc522_read(CRCResultRegL);
    pOutData[1] = rc522_read(CRCResultRegM);
}

void rc522_halt(void) {
    uint8_t status;
    uint16_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    rc522_calculateCRC(buff, 2, &buff[2]);

    status = rc522_toCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}
