#ifndef BUZZER_H
#define BUZZER_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_init.h"

/* 类型定义 ------------------------------------------------------------------*/
enum
{
    DO=262,
    RE=286,
    MI=311,
    FA=349,
    SO=392,
    LA=440,
    XI=494
};

/* 宏定义 --------------------------------------------------------------------*/
#define BEEP_PERIOD   2500
#define BEEP_ON_TIME  100
#define BEEP_OFF_TIME 100

#define BEEP_TUNE_VALUE 500
#define BEEP_CTRL_VALUE 150

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Buzzer_SetBeep(uint16_t tune, uint16_t value);
void BeepHandler(void);
void BeepTimesSet(uint8_t times);
void BeepTimeSet_ON_OFF(uint16_t on_time, uint16_t off_time);

#endif  // BUZZER_H

