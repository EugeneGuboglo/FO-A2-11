#ifndef DACS_J1939_H
#define DACS_J1939_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* J1939 identifiers (SA = 0x60 for DACS) */
#define DACS_ID_DACSC   (0x18FF203AUL)  /* PGN 65312 - received from CCU */
#define DACS_ID_ACSS1   (0x18FF2660UL)  /* PGN 65318 - transmitted       */
#define DACS_ID_ACSS2   (0x18FF2760UL)  /* PGN 65319 - transmitted       */
#define DACS_ID_AMB     (0x18FEF53AUL)  /* PGN 65269 - received from CCU */

/* --- НОВЫЕ ID для диагностики --- */
#define DACS_ID_RQST     (0x18EA0000UL)  /* PGN 59904 - база для запросов */
#define DACS_ID_SOFT     (0x18FEDA00UL)  /* PGN 65242 - ответ на запрос ПО */
#define DACS_ID_ERROR    (0x18FFFE00UL)  /* PGN 65534 - ответ на запрос ошибок */

/* Номера PGN для сравнения */
#define PGN_SOFT          65242U
#define PGN_ERROR_STATUS  65534U

#define DACS_TX_PERIOD_MS   100U

/* Two-bit state values — одинаковые для req и status */
#define DACS_ST_OFF     0U   /* закрыто / не требуется / неактивно */
#define DACS_ST_ON      1U   /* открыто / активно                 */
#define DACS_ST_ERROR   2U   /* ошибка                            */
#define DACS_ST_NA      3U   /* не доступно                       */

/* Airflow modes */
#define DACS_MODE_OFF          0x0U
#define DACS_MODE_VENTILATION  0x1U
#define DACS_MODE_AC           0x2U
#define DACS_MODE_PTC          0x3U
#define DACS_MODE_HEATING      0x4U
#define DACS_MODE_AUTO         0x5U
#define DACS_MODE_PTC_HEATING  0x6U
#define DACS_MODE_ERROR        0xEU
#define DACS_MODE_NA           0xFU

/* Request from CCU (DACSC, PGN 65312) */
typedef struct {
    uint8_t  fanSpeedRaw;    /* 0.4 %/bit               */
    uint16_t tempRaw;        /* 0.03125 °C/bit, -273    */
    uint8_t  airflowMode;    /* 0..15                   */
    uint8_t  recirc;         /* 0..3                    */
    uint8_t  windshield;     /* 0..3                    */
    uint8_t  purification;   /* 0..3                    */
    uint8_t  chest;          /* 0..3                    */
    uint8_t  legs;           /* 0..3                    */
    uint32_t lastRxTick;
    uint8_t  received;
} DACS_Request;

/* ---- Init / task ---- */

HAL_StatusTypeDef DACS_Init(CAN_HandleTypeDef *hcan);
void              DACS_Task(void);

/* ---- Status setters (поля ACSS1, что уходит на пульт) ---- */
void    DACS_SetFanSpeedPercent(uint8_t percent);  /* 0..100 % */
void    DACS_SetFanSpeedRaw(uint8_t raw);          /* 0..250   */
void    DACS_SetAirflowMode(uint8_t mode);         /* DACS_MODE_xxx */
void    DACS_SetRecirculation(uint8_t state);      /* DACS_ST_xxx */
void    DACS_SetWindshield(uint8_t state);         /* DACS_ST_xxx */
void    DACS_SetPurification(uint8_t state);       /* DACS_ST_xxx */
void    DACS_SetChest(uint8_t state);              /* DACS_ST_xxx */
void    DACS_SetLegs(uint8_t state);               /* DACS_ST_xxx */

/* ---- Status getters (для проверки/отладки) ---- */
uint8_t DACS_GetFanSpeedRaw(void);
uint8_t DACS_GetAirflowMode(void);
uint8_t DACS_GetRecirculation(void);
uint8_t DACS_GetWindshield(void);
uint8_t DACS_GetPurification(void);
uint8_t DACS_GetChest(void);
uint8_t DACS_GetLegs(void);

/* ---- ACSS2: температуры ---- */
void DACS_SetInteriorTempC(int16_t tempC);   /* -40..210 °C */
void DACS_SetExteriorTempC(int16_t tempC);   /* -40..210 °C */

/* ---- Последний запрос от CCU ---- */
const DACS_Request *DACS_GetLastRequest(void);

/*
 * Удобный помощник: скопировать все поля из последнего DACSC
 * в статус ACSS1 (простое "эхо", как для теста).
 * Temperature НЕ копируется — разные шкалы у req (0.03125 °C) и
 * status ACSS2 (1 °C).
 */
void DACS_MirrorRequestStatus(void);
HAL_StatusTypeDef DACS_ReinitAfterBusOff(void);
uint8_t           DACS_GetErrorCounters(uint8_t *tec, uint8_t *rec);

#ifdef __cplusplus
}
#endif

#endif /* DACS_J1939_H */