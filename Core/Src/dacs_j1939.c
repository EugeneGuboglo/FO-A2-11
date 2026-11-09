#include "dacs_j1939.h"
#include <string.h>

static CAN_HandleTypeDef *s_hcan = NULL;

/* ---- Status fields (ACSS1) ---- */
static uint8_t s_fanSpeedRaw = 0xFF; /* NA */
static uint8_t s_airflowMode = DACS_MODE_NA;
static uint8_t s_recircState = DACS_ST_NA;
static uint8_t s_windshieldState = DACS_ST_NA;
static uint8_t s_purifState = DACS_ST_NA;
static uint8_t s_chestState = DACS_ST_NA;
static uint8_t s_legsState = DACS_ST_NA;

/* ---- Status fields (ACSS2) ---- */
static uint8_t s_interiorTempRaw = 0xFF;
static uint8_t s_exteriorTempRaw = 0xFF;

/* ---- Last request from CCU ---- */
static DACS_Request s_lastReq;

/* ---- TX timers ---- */
static uint32_t s_lastTx1 = 0;
static uint32_t s_lastTx2 = 0;

/* ============================================================ */
/*                       Low-level send                          */
/* ============================================================ */

static HAL_StatusTypeDef send_j1939(uint32_t extId, const uint8_t *data) {
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mailbox;

  if (s_hcan == NULL)
    return HAL_ERROR;
  if (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0U)
    return HAL_BUSY;

  tx.ExtId = extId;
  tx.IDE = CAN_ID_EXT;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;

  return HAL_CAN_AddTxMessage(s_hcan, &tx, (uint8_t *)data, &mailbox);
}

/* ============================================================ */
/*                    Filter configuration                       */
/* ============================================================ */

static HAL_StatusTypeDef configure_filters(void) {
  CAN_FilterTypeDef f = {0};
  uint32_t tmp;

  __HAL_RCC_CAN1_CLK_ENABLE();

  /* SlaveStartFilterBank = 14: банки 0..13 = CAN1, 14..27 = CAN2 */
  tmp = CAN1->FMR;
  tmp |= CAN_FMR_FINIT;
  tmp &= ~CAN_FMR_CAN2SB;
  tmp |= (14U << CAN_FMR_CAN2SB_Pos);
  CAN1->FMR = tmp;

  f.FilterBank = 14;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = 0x0000;
  f.FilterIdLow = 0x0000;
  f.FilterMaskIdHigh = 0x0000;
  f.FilterMaskIdLow = 0x0000;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;

  return HAL_CAN_ConfigFilter(s_hcan, &f);
}

/* ============================================================ */
/*                          Init / task                          */
/* ============================================================ */

HAL_StatusTypeDef DACS_Init(CAN_HandleTypeDef *hcan) {
  s_hcan = hcan;

  if (hcan->State != HAL_CAN_STATE_RESET) {
    (void)HAL_CAN_DeInit(hcan);
  }

  hcan->Instance = CAN2;
  hcan->Init.Prescaler = 6;
  hcan->Init.Mode = CAN_MODE_NORMAL;
  hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan->Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = ENABLE;
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = DISABLE;
  hcan->Init.ReceiveFifoLocked = DISABLE;
  hcan->Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(hcan) != HAL_OK)
    return HAL_ERROR;
  if (configure_filters() != HAL_OK)
    return HAL_ERROR;
  if (HAL_CAN_Start(hcan) != HAL_OK)
    return HAL_ERROR;
  HAL_CAN_ActivateNotification(hcan, CAN_IT_ERROR | CAN_IT_BUSOFF);
  memset(&s_lastReq, 0, sizeof(s_lastReq));

  s_fanSpeedRaw = 0xFF;
  s_airflowMode = DACS_MODE_NA;
  s_recircState = DACS_ST_NA;
  s_windshieldState = DACS_ST_NA;
  s_purifState = DACS_ST_NA;
  s_chestState = DACS_ST_NA;
  s_legsState = DACS_ST_NA;
  s_interiorTempRaw = 0xFF;
  s_exteriorTempRaw = 0xFF;

  s_lastTx1 = HAL_GetTick();
  s_lastTx2 = HAL_GetTick();
  return HAL_OK;
}

/* ---- TX ---- */

static void send_acss1(void) {
  uint8_t d[8];
  d[0] = s_fanSpeedRaw;
  d[1] = 0xFF;
  d[2] = 0xFF;
  d[3] = 0xF0U | (s_airflowMode & 0x0FU);
  d[4] = (s_recircState & 0x03U) | ((s_windshieldState & 0x03U) << 2) |
         ((s_purifState & 0x03U) << 4) | 0xC0U;
  d[5] = 0x0FU | ((s_chestState & 0x03U) << 4) | ((s_legsState & 0x03U) << 6);
  d[6] = 0xFF;
  d[7] = 0xFF;
  (void)send_j1939(DACS_ID_ACSS1, d);
}

static void send_acss2(void) {
  uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  d[0] = s_interiorTempRaw;
  d[1] = s_exteriorTempRaw;
  (void)send_j1939(DACS_ID_ACSS2, d);
}

/* ---- RX ---- */

static void parse_dacsc(const uint8_t *d) {
  s_lastReq.fanSpeedRaw = d[0];
  s_lastReq.tempRaw = (uint16_t)d[1] | ((uint16_t)d[2] << 8);
  s_lastReq.airflowMode = d[3] & 0x0FU;
  s_lastReq.recirc = d[4] & 0x03U;
  s_lastReq.windshield = (d[4] >> 2) & 0x03U;
  s_lastReq.purification = (d[4] >> 4) & 0x03U;
  s_lastReq.chest = (d[5] >> 4) & 0x03U;
  s_lastReq.legs = (d[5] >> 6) & 0x03U;
  s_lastReq.lastRxTick = HAL_GetTick();
  s_lastReq.received = 1;
}

/* НОВОЕ: Парсинг AMB (PGN 65269) для зеркалирования температур */
static void parse_amb(const uint8_t *d) {
  uint16_t intTempRaw = (uint16_t)d[1] | ((uint16_t)d[2] << 8);
  uint16_t extTempRaw = (uint16_t)d[3] | ((uint16_t)d[4] << 8);

  /* Конвертация: AMB (0.03125 C, -273) -> ACSS2 (1 C, -40)
     Формула: Raw_ACSS2 = (Raw_AMB / 32) - 233 */
  if (intTempRaw == 0xFFFF) {
    s_interiorTempRaw = 0xFF; // NA
  } else if (intTempRaw == 0xFFFE) {
    s_interiorTempRaw = 0xFE; // Error
  } else {
    int32_t t = (int32_t)(intTempRaw / 32) - 233;
    if (t < 0)
      t = 0;
    if (t > 250)
      t = 250;
    s_interiorTempRaw = (uint8_t)t;
  }

  if (extTempRaw == 0xFFFF) {
    s_exteriorTempRaw = 0xFF; // NA
  } else if (extTempRaw == 0xFFFE) {
    s_exteriorTempRaw = 0xFE; // Error
  } else {
    int32_t t = (int32_t)(extTempRaw / 32) - 233;
    if (t < 0)
      t = 0;
    if (t > 250)
      t = 250;
    s_exteriorTempRaw = (uint8_t)t;
  }
}

/* НОВОЕ: Обработка RQST (PGN 59904) */
static void parse_rqst(const uint8_t *d, uint8_t sa) {
  uint32_t reqPgn =
      (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16);

  if (reqPgn == PGN_SOFT) {
    // Ответ SOFT (PGN 65242)
    // Число: 2611050001 (YY=26, MM=11, DD=05, HW=00, SW=01)
    // В Hex: 0x9BA0B091. Порядок Intel (Little Endian): 91 B0 A0 9B
    uint8_t resp[8] = {0x01, 0x11, 0x76, 0xA1, 0x9B, 0xFF, 0xFF, 0xFF};
    uint32_t tx_id = (6UL << 26) | (0xFEDAUL << 8) | 0x60U;
    send_j1939(tx_id, resp);
  } else if (reqPgn == PGN_ERROR_STATUS) {
    // Ответ Error_Status (PGN 65534). Пока без ошибок.
    uint8_t resp[8] = {0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // В будущем можно добавить сюда логику установки битов ошибок
    // if (fan_is_fault) resp[0] |= 0x01;

    uint32_t tx_id = (6UL << 26) | (0xFFFEUL << 8) | 0x60U;
    send_j1939(tx_id, resp);
  }
}

static void poll_rx(void) {
  CAN_RxHeaderTypeDef rx;
  uint8_t d[8];

  while (HAL_CAN_GetRxFifoFillLevel(s_hcan, CAN_RX_FIFO0) > 0U) {
    if (HAL_CAN_GetRxMessage(s_hcan, CAN_RX_FIFO0, &rx, d) != HAL_OK)
      break;
    if (rx.IDE != CAN_ID_EXT)
      continue;

    uint32_t pf = (rx.ExtId >> 16) & 0xFF;
    uint8_t da = (rx.ExtId >> 8) & 0xFF;
    uint8_t sa = rx.ExtId & 0xFF;

    if (rx.ExtId == DACS_ID_DACSC) {
      parse_dacsc(d);
    } else if (rx.ExtId == DACS_ID_AMB) {
      parse_amb(d); /* НОВОЕ */
    } else if (pf == 0xEA && (da == 0x60 || da == 0xFF)) {
      // НОВОЕ: Пришел RQST (PGN 59904) конкретно нам (0x60) или всем (0xFF)
      parse_rqst(d, sa);
    }
  }
}

void DACS_Task(void) {
  uint32_t now;

  if (s_hcan == NULL)
    return;

  poll_rx();

  now = HAL_GetTick();
  if ((now - s_lastTx1) >= DACS_TX_PERIOD_MS) {
    s_lastTx1 = now;
    send_acss1();
  }
  if ((now - s_lastTx2) >= DACS_TX_PERIOD_MS) {
    s_lastTx2 = now;
    send_acss2();
  }
}

/* Setters / getters и DACS_MirrorRequestStatus остаются без изменений */
void DACS_SetFanSpeedPercent(uint8_t p) {
  if (p > 100)
    p = 100;
  s_fanSpeedRaw = (uint8_t)(((uint32_t)p * 5U) / 2U);
}
void DACS_SetFanSpeedRaw(uint8_t raw) {
  if (raw > 250)
    raw = 250;
  s_fanSpeedRaw = raw;
}
uint8_t DACS_GetFanSpeedRaw(void) { return s_fanSpeedRaw; }
void DACS_SetAirflowMode(uint8_t m) { s_airflowMode = m & 0x0FU; }
uint8_t DACS_GetAirflowMode(void) { return s_airflowMode; }
void DACS_SetRecirculation(uint8_t s) { s_recircState = s & 0x03U; }
uint8_t DACS_GetRecirculation(void) { return s_recircState; }
void DACS_SetWindshield(uint8_t s) { s_windshieldState = s & 0x03U; }
uint8_t DACS_GetWindshield(void) { return s_windshieldState; }
void DACS_SetPurification(uint8_t s) { s_purifState = s & 0x03U; }
uint8_t DACS_GetPurification(void) { return s_purifState; }
void DACS_SetChest(uint8_t s) { s_chestState = s & 0x03U; }
uint8_t DACS_GetChest(void) { return s_chestState; }
void DACS_SetLegs(uint8_t s) { s_legsState = s & 0x03U; }
uint8_t DACS_GetLegs(void) { return s_legsState; }

void DACS_SetInteriorTempC(int16_t t) {
  if (t < -40)
    t = -40;
  if (t > 210)
    t = 210;
  s_interiorTempRaw = (uint8_t)(t + 40);
}
void DACS_SetExteriorTempC(int16_t t) {
  if (t < -40)
    t = -40;
  if (t > 210)
    t = 210;
  s_exteriorTempRaw = (uint8_t)(t + 40);
}

const DACS_Request *DACS_GetLastRequest(void) { return &s_lastReq; }

void DACS_MirrorRequestStatus(void) {
  if (!s_lastReq.received)
    return;
  s_fanSpeedRaw = s_lastReq.fanSpeedRaw;
  s_airflowMode = s_lastReq.airflowMode & 0x0FU;
  s_recircState = s_lastReq.recirc;
  s_windshieldState = s_lastReq.windshield;
  s_purifState = s_lastReq.purification;
  s_chestState = s_lastReq.chest;
  s_legsState = s_lastReq.legs;
}