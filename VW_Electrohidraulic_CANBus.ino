/*
  Emisor CAN (STD ID) - Blue Pill STM32F103C8T6
  Envia repetidamente dos tramas:
    Trama 1 -> ID 0x320, DLC 8, Datos 01 00 00 00 00 00 00 00, cada 100 ms
    Trama 2 -> ID 0x280, DLC 8, Datos 00 00 A0 0F 00 00 00 00, cada 50 ms

  CAN remapeado a PB8 (RX) / PB9 (TX)  -> bxCAN interno del STM32,
  se requiere un transceiver externo (ej. TJA1050 / SN65HVD230),
  NO se usa modulo MCP2515 por SPI.

  Baudrate CAN configurado: 500 kbps (APB1 = 36 MHz)
  BTR = SJW=1tq, TS1=8tq, TS2=3tq, BRP=6  -> 12 tq -> 36MHz/(6*12)=500kbps

  Debug por Serial1 (PA9/PA10)
*/

// --- Trama 1 (mailbox 0) ---
#define TX1_STD_ID   0x320UL          // ID estandar (11 bit)
uint8_t tx1Data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t tx1Len = 8;
const uint32_t TX1_PERIOD_MS = 100;   // periodo de envio
uint32_t ultimoEnvio1 = 0;

// --- Trama 2 (mailbox 1) ---
#define TX2_STD_ID   0x280UL          // ID estandar (11 bit)
uint8_t tx2Data[8] = {0x00, 0x00, 0xA0, 0x0F, 0x00, 0x00, 0x00, 0x00};
const uint8_t tx2Len = 8;
const uint32_t TX2_PERIOD_MS = 50;    // periodo de envio
uint32_t ultimoEnvio2 = 0;

// ============================================================
// --- Inicializacion de pines CAN (remap a PB8/PB9) ---
// ============================================================
void CAN_GPIO_Init() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

  AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
  AFIO->MAPR |=  AFIO_MAPR_CAN_REMAP_REMAP2;   // CAN -> PB8(RX)/PB9(TX)

  // PB8 = RX -> entrada flotante
  GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
  GPIOB->CRH |=  GPIO_CRH_CNF8_0;

  // PB9 = TX -> salida AF push-pull, 50MHz
  GPIOB->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
  GPIOB->CRH |=  GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;
}

// ============================================================
// --- Inicializacion del periferico bxCAN ---
// ============================================================
bool CAN_Init() {
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

  CAN1->MCR |= CAN_MCR_INRQ;
  CAN1->MCR &= ~CAN_MCR_SLEEP;

  uint32_t t = 0;
  while (!(CAN1->MSR & CAN_MSR_INAK)) { if (++t > 1000000) return false; }

  // SJW=1tq, TS1=8tq, TS2=3tq, BRP=6 -> 500 kbps @ APB1=36MHz
  CAN1->BTR = (0UL << 24) | (2UL << 20) | (7UL << 16) | (5UL);

  CAN1->MCR &= ~CAN_MCR_INRQ;
  t = 0;
  while (CAN1->MSR & CAN_MSR_INAK) { if (++t > 1000000) return false; }

  return true;
}

// Filtro "pasa todo" (no es indispensable para transmitir, pero deja
// el periferico correctamente configurado si mas adelante quieres recibir)
void CAN_Filter_AcceptAll() {
  CAN1->FMR  |= CAN_FMR_FINIT;
  CAN1->FA1R &= ~(1UL << 0);
  CAN1->FS1R |=  (1UL << 0);
  CAN1->FM1R &= ~(1UL << 0);
  CAN1->sFilterRegister[0].FR1 = 0;
  CAN1->sFilterRegister[0].FR2 = 0;
  CAN1->FFA1R &= ~(1UL << 0);
  CAN1->FA1R  |=  (1UL << 0);
  CAN1->FMR   &= ~CAN_FMR_FINIT;
}

// ============================================================
// --- Transmision (ID estandar, 11 bit), mailbox seleccionable ---
// ============================================================
bool CAN_Transmit_Std(uint8_t mailbox, uint16_t stdId, uint8_t *data, uint8_t len) {
  uint32_t tmeMask;
  switch (mailbox) {
    case 0:  tmeMask = CAN_TSR_TME0; break;
    case 1:  tmeMask = CAN_TSR_TME1; break;
    default: tmeMask = CAN_TSR_TME2; break;
  }

  // Esperar a que el mailbox indicado este libre
  uint32_t t = 0;
  while (!(CAN1->TSR & tmeMask)) { if (++t > 1000000) return false; }

  CAN1->sTxMailBox[mailbox].TDTR = (CAN1->sTxMailBox[mailbox].TDTR & ~0x0FUL) | (len & 0x0F);

  CAN1->sTxMailBox[mailbox].TDLR = (uint32_t)data[0]        |
                                   ((uint32_t)data[1] << 8)  |
                                   ((uint32_t)data[2] << 16) |
                                   ((uint32_t)data[3] << 24);

  CAN1->sTxMailBox[mailbox].TDHR = (uint32_t)data[4]        |
                                   ((uint32_t)data[5] << 8)  |
                                   ((uint32_t)data[6] << 16) |
                                   ((uint32_t)data[7] << 24);

  // STID en bits [31:21], IDE=0 (standard), RTR=0 (data frame)
  CAN1->sTxMailBox[mailbox].TIR = ((uint32_t)stdId << 21) | CAN_TI0R_TXRQ;

  return true;
}

void CAN_Print_Frame(uint16_t stdId, uint8_t *data, uint8_t len) {
  Serial1.print("TX OK  ID=0x");
  Serial1.print(stdId, HEX);
  Serial1.print("  Data=");
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial1.print("0");
    Serial1.print(data[i], HEX);
    Serial1.print(" ");
  }
  Serial1.println();
}

// ============================================================
// --- setup / loop ---
// ============================================================
void setup() {
  Serial1.begin(115200);
  delay(100);

  CAN_GPIO_Init();
  CAN_Init();
  CAN_Filter_AcceptAll();

  Serial1.println("CAN TX listo (bxCAN interno, PB8/PB9, 500 kbps)");
}

void loop() {
  uint32_t ahora = millis();

  // Trama 1: ID 0x320 cada 100 ms (mailbox 0)
  if (ahora - ultimoEnvio1 >= TX1_PERIOD_MS) {
    ultimoEnvio1 = ahora;
    if (CAN_Transmit_Std(0, TX1_STD_ID, tx1Data, tx1Len)) {
      CAN_Print_Frame(TX1_STD_ID, tx1Data, tx1Len);
    } else {
      Serial1.println("TX1 FALLO (mailbox ocupado)");
    }
  }

  // Trama 2: ID 0x280 cada 50 ms (mailbox 1)
  if (ahora - ultimoEnvio2 >= TX2_PERIOD_MS) {
    ultimoEnvio2 = ahora;
    if (CAN_Transmit_Std(1, TX2_STD_ID, tx2Data, tx2Len)) {
      CAN_Print_Frame(TX2_STD_ID, tx2Data, tx2Len);
    } else {
      Serial1.println("TX2 FALLO (mailbox ocupado)");
    }
  }
}