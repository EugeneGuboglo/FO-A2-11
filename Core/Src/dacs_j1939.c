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

/* ---- TX очередь ---- */
#define TXQ_SIZE 16
typedef struct {
  uint32_t extId;
  uint8_t data[8];
} TxSlot_t;
static TxSlot_t s_txq[TXQ_SIZE];
static volatile uint8_t s_txHead = 0, s_txTail = 0;
static volatile uint16_t s_txDropCount = 0;

static uint8_t txq_push(uint32_t id, const uint8_t *d) {
  uint8_t nxt = (s_txHead + 1U) % TXQ_SIZE;
  if (nxt == s_txTail) {
    s_txDropCount++;
    return 0;
  }
  s_txq[s_txHead].extId = id;
  memcpy(s_txq[s_txHead].data, d, 8);
  s_txHead = nxt;
  return 1;
}
static uint8_t txq_pop(TxSlot_t *out) {
  if (s_txTail == s_txHead)
    return 0;
  *out = s_txq[s_txTail];
  s_txTail = (s_txTail + 1U) % TXQ_SIZE;
  return 1;
}
static void txq_clear(void) { s_txHead = s_txTail = 0; }

/* ============================================================ */
/*                       Low-level send                          */
/* ============================================================ */

static HAL_StatusTypeDef send_j1939(uint32_t extId, const uint8_t *data) {
  if (s_hcan == NULL) return HAL_ERROR;
  return txq_push(extId, data) ? HAL_OK : HAL_BUSY;
}

/* Отправляем всё, что накопилось в очереди, пока есть свободные mailbox */
static void drain_tx_queue(void) {
  TxSlot_t msg;
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mailbox;

  while (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) > 0U && txq_pop(&msg)) {
    tx.ExtId = msg.extId;
    tx.IDE   = CAN_ID_EXT;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;
    if (HAL_CAN_AddTxMessage(s_hcan, &tx, msg.data, &mailbox) != HAL_OK) {
      break;   /* повторим на следующем цикле DACS_Task */
    }
  }
}

/* ============================================================ */
/*                    Filter configuration                       */
/* ============================================================ */

static HAL_StatusTypeDef config_filter_bank(uint32_t bank,
                                            uint32_t extId, uint32_t mask) {
  CAN_FilterTypeDef f = {0};
  f.FilterBank           = bank;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = (extId >> 13) & 0xFFFFU;
  f.FilterIdLow          = ((extId << 3) & 0xFFF8U) | CAN_ID_EXT;
  f.FilterMaskIdHigh     = (mask  >> 13) & 0xFFFFU;
  f.FilterMaskIdLow      = ((mask  << 3) & 0xFFF8U) | CAN_ID_EXT;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14;
  return HAL_CAN_ConfigFilter(s_hcan, &f);
}

static HAL_StatusTypeDef configure_filters(void) {
  uint32_t tmp;
  __HAL_RCC_CAN1_CLK_ENABLE();

  tmp  = CAN1->FMR;
  tmp |= CAN_FMR_FINIT;
  tmp &= ~CAN_FMR_CAN2SB;
  tmp |= (14U << CAN_FMR_CAN2SB_Pos);
  CAN1->FMR = tmp;

  if (config_filter_bank(14, DACS_ID_DACSC, 0x1FFFFFFFU) != HAL_OK) return HAL_ERROR;
  if (config_filter_bank(15, DACS_ID_AMB,   0x1FFFFFFFU) != HAL_OK) return HAL_ERROR;
  if (config_filter_bank(16, (0xEAUL << 16) | (0x60UL  << 8),
                             (0xFFUL  << 16) | (0xFFUL  << 8)) != HAL_OK) return HAL_ERROR;
  if (config_filter_bank(17, (0xEAUL << 16) | (0xFFUL  << 8),
                             (0xFFUL  << 16) | (0xFFUL  << 8)) != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

/* ============================================================ */
/*                          Init / task                          */
/* ============================================================ */

static HAL_StatusTypeDef can2_hardware_init(CAN_HandleTypeDef *hcan) {
  if (hcan->State != HAL_CAN_STATE_RESET) {
    (void)HAL_CAN_DeInit(hcan);   /* отцепляет пины через MspDeInit */
  }
  HAL_Delay(1);

  hcan->Instance                = CAN2;
  hcan->Init.Prescaler          = 6;
  hcan->Init.Mode               = CAN_MODE_NORMAL;
  hcan->Init.SyncJumpWidth      = CAN_SJW_1TQ;
  hcan->Init.TimeSeg1           = CAN_BS1_11TQ;   /* 500 kbps */
  hcan->Init.TimeSeg2           = CAN_BS2_2TQ;
  hcan->Init.TimeTriggeredMode  = DISABLE;
  hcan->Init.AutoBusOff         = DISABLE;        /* recovery — программное */
  hcan->Init.AutoWakeUp         = DISABLE;
  hcan->Init.AutoRetransmission = DISABLE;
  hcan->Init.ReceiveFifoLocked  = DISABLE;
  hcan->Init.TransmitFifoPriority = DISABLE;

  return HAL_CAN_Init(hcan);
}

HAL_StatusTypeDef DACS_Init(CAN_HandleTypeDef *hcan) {
  s_hcan = hcan;

  if (can2_hardware_init(hcan) != HAL_OK) return HAL_ERROR;
  if (configure_filters()       != HAL_OK) return HAL_ERROR;
  if (HAL_CAN_Start(hcan)       != HAL_OK) return HAL_ERROR;
  HAL_CAN_ActivateNotification(hcan, CAN_IT_ERROR | CAN_IT_BUSOFF);

  memset(&s_lastReq, 0, sizeof(s_lastReq));
  s_fanSpeedRaw = 0xFF;
  s_airflowMode = DACS_MODE_NA;
  s_recircState = s_windshieldState = s_purifState = DACS_ST_NA;
  s_chestState  = s_legsState      = DACS_ST_NA;
  s_interiorTempRaw = s_exteriorTempRaw = 0xFF;

  txq_clear();
  s_lastTx1 = s_lastTx2 = HAL_GetTick();
  return HAL_OK;
}

HAL_StatusTypeDef DACS_ReinitAfterBusOff(void) {
  if (s_hcan == NULL) return HAL_ERROR;
  txq_clear();
  if (can2_hardware_init(s_hcan) != HAL_OK) return HAL_ERROR;
  if (configure_filters()        != HAL_OK) return HAL_ERROR;
  if (HAL_CAN_Start(s_hcan)      != HAL_OK) return HAL_ERROR;
  HAL_CAN_ActivateNotification(s_hcan, CAN_IT_ERROR | CAN_IT_BUSOFF);
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

  if (__HAL_CAN_GET_FLAG(s_hcan, CAN_FLAG_FOV0)) {
    __HAL_CAN_CLEAR_FLAG(s_hcan, CAN_FLAG_FOV0);
  }

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
  if (s_hcan == NULL) return;

  poll_rx();
  drain_tx_queue();                       /* ← новая строка */

  uint32_t now = HAL_GetTick();
  if ((now - s_lastTx1) >= DACS_TX_PERIOD_MS) { s_lastTx1 = now; send_acss1(); }
  if ((now - s_lastTx2) >= DACS_TX_PERIOD_MS) { s_lastTx2 = now; send_acss2(); }
}

uint8_t DACS_GetErrorCounters(uint8_t *tec, uint8_t *rec) {
  if (s_hcan == NULL) return 0;
  uint32_t esr = s_hcan->Instance->ESR;
  *tec = (uint8_t)((esr >> 16) & 0xFFU);
  *rec = (uint8_t)((esr >> 24) & 0xFFU);
  return 1;
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