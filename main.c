/*
 * TK-2026-PLC-002
 * 1 CMM compact bio-deodorizer integrated control logic
 *
 * Target logic source for LS ELECTRIC XEC-DN32H / XG5000 C style runtime.
 * The variable names intentionally avoid '.' characters.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
    bool previous;
} RTrig;

typedef struct
{
    bool in_previous;
    bool q;
    uint32_t elapsed_ms;
} Ton;

typedef struct
{
    int16_t iRaw_DiffPressure;
    int16_t iRaw_TDS;
    int16_t iRaw_Humidity_S1;
    int16_t iRaw_Humidity_S2;
    int16_t iRaw_SumpTemp;
    int16_t iRaw_AmbientTemp;
    int16_t iRaw_pH;

    bool xSumpLevel_Low;
    bool xSumpLevel_High;
    bool xManualMode_SW;
    bool xAlarmReset_PB;

    float rDesignArea_m2;
    float rDesignFlow_CMM;

    bool xRemoteMode_Cmd;
    bool xRemote_PumpStart;
    bool xRemote_DrainStart;
    bool xRemote_AlarmReset;
    bool xComm_Heartbeat;

    bool xBlowerRun;
    bool xPumpRun;
    bool xHeaterRun;
    bool xValveDrain;
    bool xValveMakeup;
    bool xValveBackwash;

    bool xAlarm_DP_High;
    bool xAlarm_DP_Backwash;
    bool xAlarm_SumpLow;
    bool xAlarm_HeaterFault;
    bool xAlarm_SensorFault;
    bool xAlarm_CommLost;
    bool xAlarm_pHBreakthrough;
    bool xAlarm_PumpOverrun;

    float rDiffPressure;
    float rTDS_Value;
    float rHumidity_S1;
    float rHumidity_S2;
    float rSumpTemp;
    float rAmbientTemp;
    float rEBCT_Check;

    float EBCT_TARGET_S;
    float LV_DESIGN_MS;
    float H_TOTAL_MEDIA_M;
    float DP_NORMAL_MAX;
    float DP_BACKWASH_TRIGGER;
    float DP_SHUTDOWN;
    float DP_RATE_ALARM_MMH;
    float TDS_DRAIN_START;
    float TDS_DRAIN_STOP;
    float HUMID_LOW_SETPOINT;
    float HUMID_HIGH_SETPOINT;
    float HEATER_ON_TEMP;
    float HEATER_OFF_TEMP;
    float HEATER_FAULT_TEMP;
    float AMBIENT_PREHEAT_TEMP;
    uint32_t PUMP_MIN_RUN_MS;
    uint32_t PUMP_MAX_RUN_MS;
    uint32_t DRAIN_SEQ_DELAY_MS;
    uint32_t COMM_WATCHDOG_MS;
    uint32_t HEATER_MIN_OFF_MS;

    bool xIsAutoMode;
    bool xRemoteModeActive;

    bool xRemotePumpPulse;
    bool xRemoteDrainPulse;
    bool xAlarmResetPulse;
    bool xCommHeartbeatPulse;
    bool xPumpMinDone;
    bool xPumpMaxDone;
    bool xDrainDelayDone;
    bool xCommWatchdogDone;
    bool xHeaterMinOffDone;

    RTrig trig_RemotePump;
    RTrig trig_RemoteDrain;
    RTrig trig_AlarmReset;
    RTrig trig_CommHeartbeat;

    Ton ton_PumpMinRun;
    Ton ton_PumpMaxRun;
    Ton ton_DrainSequence;
    Ton ton_CommWatchdog;
    Ton ton_HeaterMinOff;

    int16_t iDrainSeqState;

    bool xSensorFault_DP;
    bool xSensorFault_TDS;
    bool xSensorFault_Hum;
    bool xSensorFault_Temp;

    float rDP_Trend_Prev;
    float rDP_Trend_RateOfRise;
} OdorControl;

static bool RTrig_Update(RTrig *trig, bool clk)
{
    bool q = clk && !trig->previous;
    trig->previous = clk;
    return q;
}

static bool Ton_Update(Ton *timer, bool in, uint32_t pt_ms, uint32_t cycle_ms)
{
    if (!in)
    {
        timer->in_previous = false;
        timer->q = false;
        timer->elapsed_ms = 0U;
        return false;
    }

    if (!timer->in_previous)
    {
        timer->elapsed_ms = 0U;
    }
    else if (timer->elapsed_ms < pt_ms)
    {
        uint32_t next_elapsed = timer->elapsed_ms + cycle_ms;
        timer->elapsed_ms = (next_elapsed < timer->elapsed_ms) ? pt_ms : next_elapsed;
    }

    timer->in_previous = true;
    timer->q = timer->elapsed_ms >= pt_ms;
    return timer->q;
}

void OdorControl_Init(OdorControl *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->EBCT_TARGET_S = 15.0f;
    ctrl->LV_DESIGN_MS = 0.0556f;
    ctrl->H_TOTAL_MEDIA_M = 0.833f;
    ctrl->DP_NORMAL_MAX = 80.0f;
    ctrl->DP_BACKWASH_TRIGGER = 100.0f;
    ctrl->DP_SHUTDOWN = 150.0f;
    ctrl->DP_RATE_ALARM_MMH = 5.0f;
    ctrl->TDS_DRAIN_START = 4000.0f;
    ctrl->TDS_DRAIN_STOP = 1500.0f;
    ctrl->HUMID_LOW_SETPOINT = 70.0f;
    ctrl->HUMID_HIGH_SETPOINT = 80.0f;
    ctrl->HEATER_ON_TEMP = 5.0f;
    ctrl->HEATER_OFF_TEMP = 8.0f;
    ctrl->HEATER_FAULT_TEMP = 45.0f;
    ctrl->AMBIENT_PREHEAT_TEMP = 0.0f;
    ctrl->PUMP_MIN_RUN_MS = 30000U;
    ctrl->PUMP_MAX_RUN_MS = 600000U;
    ctrl->DRAIN_SEQ_DELAY_MS = 10000U;
    ctrl->COMM_WATCHDOG_MS = 5000U;
    ctrl->HEATER_MIN_OFF_MS = 60000U;
}

void OdorControl_Step(OdorControl *ctrl, uint32_t cycle_ms)
{
    ctrl->xRemotePumpPulse = RTrig_Update(&ctrl->trig_RemotePump, ctrl->xRemote_PumpStart);
    ctrl->xRemoteDrainPulse = RTrig_Update(&ctrl->trig_RemoteDrain, ctrl->xRemote_DrainStart);
    ctrl->xAlarmResetPulse = RTrig_Update(&ctrl->trig_AlarmReset,
                                          ctrl->xAlarmReset_PB || ctrl->xRemote_AlarmReset);
    ctrl->xCommHeartbeatPulse = RTrig_Update(&ctrl->trig_CommHeartbeat, ctrl->xComm_Heartbeat);

    ctrl->xIsAutoMode = ctrl->xManualMode_SW;

    ctrl->rDiffPressure = ((float)ctrl->iRaw_DiffPressure) * 300.0f / 27648.0f;
    ctrl->rTDS_Value = ((float)ctrl->iRaw_TDS) * 10000.0f / 27648.0f;
    ctrl->rHumidity_S1 = ((float)ctrl->iRaw_Humidity_S1) * 100.0f / 27648.0f;
    ctrl->rHumidity_S2 = ((float)ctrl->iRaw_Humidity_S2) * 100.0f / 27648.0f;
    ctrl->rSumpTemp = (((float)ctrl->iRaw_SumpTemp) * 100.0f / 27648.0f) - 20.0f;
    ctrl->rAmbientTemp = (((float)ctrl->iRaw_AmbientTemp) * 100.0f / 27648.0f) - 20.0f;

    ctrl->xSensorFault_DP = (ctrl->iRaw_DiffPressure <= 0) || (ctrl->rDiffPressure > 310.0f);
    ctrl->xSensorFault_TDS = (ctrl->iRaw_TDS <= 0) || (ctrl->rTDS_Value > 10200.0f);
    ctrl->xSensorFault_Hum = (ctrl->rHumidity_S1 < 0.0f) || (ctrl->rHumidity_S1 > 101.0f) ||
                             (ctrl->rHumidity_S2 < 0.0f) || (ctrl->rHumidity_S2 > 101.0f);
    ctrl->xSensorFault_Temp = (ctrl->rSumpTemp < -25.0f) || (ctrl->rSumpTemp > 90.0f);
    ctrl->xAlarm_SensorFault = ctrl->xSensorFault_DP || ctrl->xSensorFault_TDS ||
                               ctrl->xSensorFault_Hum || ctrl->xSensorFault_Temp;

    if ((ctrl->rDesignFlow_CMM > 0.0f) && (ctrl->rDesignArea_m2 > 0.0f))
    {
        ctrl->rEBCT_Check = ctrl->H_TOTAL_MEDIA_M /
                            ((ctrl->rDesignFlow_CMM / 60.0f) / ctrl->rDesignArea_m2);
    }
    else
    {
        ctrl->rEBCT_Check = 0.0f;
    }

    ctrl->rDP_Trend_RateOfRise = (ctrl->rDiffPressure - ctrl->rDP_Trend_Prev) * 3600.0f;
    ctrl->rDP_Trend_Prev = ctrl->rDiffPressure;
    ctrl->xAlarm_pHBreakthrough = (ctrl->rDP_Trend_RateOfRise > ctrl->DP_RATE_ALARM_MMH) &&
                                  !ctrl->xSensorFault_DP;

    ctrl->xAlarm_DP_Backwash = (ctrl->rDiffPressure >= ctrl->DP_BACKWASH_TRIGGER) &&
                               (ctrl->rDiffPressure < ctrl->DP_SHUTDOWN) &&
                               !ctrl->xSensorFault_DP;
    ctrl->xValveBackwash = ctrl->xAlarm_DP_Backwash && ctrl->xIsAutoMode;

    if (ctrl->xAlarmResetPulse)
    {
        ctrl->xAlarm_DP_High = false;
    }
    else if ((ctrl->rDiffPressure >= ctrl->DP_SHUTDOWN) && !ctrl->xSensorFault_DP)
    {
        ctrl->xAlarm_DP_High = true;
    }

    if (ctrl->xAlarm_DP_High || ctrl->xAlarm_SensorFault)
    {
        ctrl->xBlowerRun = false;
    }
    else if (ctrl->xIsAutoMode)
    {
        ctrl->xBlowerRun = true;
    }
    else
    {
        ctrl->xBlowerRun = false;
    }

    ctrl->xPumpMinDone = Ton_Update(&ctrl->ton_PumpMinRun, ctrl->xPumpRun,
                                    ctrl->PUMP_MIN_RUN_MS, cycle_ms);
    ctrl->xPumpMaxDone = Ton_Update(&ctrl->ton_PumpMaxRun, ctrl->xPumpRun,
                                    ctrl->PUMP_MAX_RUN_MS, cycle_ms);
    ctrl->xAlarm_PumpOverrun = ctrl->xPumpMaxDone;

    if (ctrl->xIsAutoMode && !ctrl->xAlarm_SensorFault)
    {
        if ((ctrl->rHumidity_S1 < ctrl->HUMID_LOW_SETPOINT) ||
            (ctrl->rHumidity_S2 < ctrl->HUMID_LOW_SETPOINT))
        {
            ctrl->xPumpRun = true;
        }
        else if ((ctrl->rHumidity_S1 >= ctrl->HUMID_HIGH_SETPOINT) &&
                 (ctrl->rHumidity_S2 >= ctrl->HUMID_HIGH_SETPOINT) &&
                 ctrl->xPumpMinDone)
        {
            ctrl->xPumpRun = false;
        }

        if (ctrl->xAlarm_PumpOverrun)
        {
            ctrl->xPumpRun = false;
        }
    }
    else
    {
        ctrl->xPumpRun = false;
    }

    ctrl->xHeaterMinOffDone = Ton_Update(&ctrl->ton_HeaterMinOff, !ctrl->xHeaterRun,
                                         ctrl->HEATER_MIN_OFF_MS, cycle_ms);
    ctrl->xAlarm_HeaterFault = (ctrl->rSumpTemp >= ctrl->HEATER_FAULT_TEMP) &&
                               !ctrl->xSensorFault_Temp;

    if (ctrl->xAlarm_HeaterFault || ctrl->xSensorFault_Temp)
    {
        ctrl->xHeaterRun = false;
    }
    else if ((ctrl->rSumpTemp <= ctrl->HEATER_ON_TEMP) ||
             (ctrl->rAmbientTemp <= ctrl->AMBIENT_PREHEAT_TEMP))
    {
        if (ctrl->xHeaterMinOffDone)
        {
            ctrl->xHeaterRun = true;
        }
    }
    else if (ctrl->rSumpTemp >= ctrl->HEATER_OFF_TEMP)
    {
        ctrl->xHeaterRun = false;
    }

    ctrl->xDrainDelayDone = Ton_Update(&ctrl->ton_DrainSequence,
                                       (ctrl->iDrainSeqState == 1) &&
                                           (ctrl->rTDS_Value < ctrl->TDS_DRAIN_STOP),
                                       ctrl->DRAIN_SEQ_DELAY_MS, cycle_ms);

    switch (ctrl->iDrainSeqState)
    {
    case 0:
        ctrl->xValveDrain = false;
        ctrl->xValveMakeup = false;
        if ((ctrl->rTDS_Value >= ctrl->TDS_DRAIN_START) &&
            !ctrl->xSensorFault_TDS &&
            ctrl->xIsAutoMode)
        {
            ctrl->iDrainSeqState = 1;
        }
        break;

    case 1:
        ctrl->xValveDrain = true;
        ctrl->xValveMakeup = false;
        if (ctrl->xDrainDelayDone)
        {
            ctrl->iDrainSeqState = 2;
        }
        if (ctrl->xSumpLevel_Low)
        {
            ctrl->iDrainSeqState = 0;
            ctrl->xAlarm_SumpLow = true;
        }
        break;

    case 2:
        ctrl->xValveDrain = false;
        ctrl->xValveMakeup = true;
        if (ctrl->xSumpLevel_High)
        {
            ctrl->iDrainSeqState = 3;
        }
        break;

    case 3:
        ctrl->xValveDrain = false;
        ctrl->xValveMakeup = false;
        ctrl->iDrainSeqState = 0;
        break;

    default:
        ctrl->iDrainSeqState = 0;
        break;
    }

    ctrl->xAlarm_SumpLow = ctrl->xSumpLevel_Low;

    ctrl->xCommWatchdogDone = Ton_Update(&ctrl->ton_CommWatchdog,
                                         !ctrl->xCommHeartbeatPulse,
                                         ctrl->COMM_WATCHDOG_MS, cycle_ms);
    if (ctrl->xCommWatchdogDone)
    {
        ctrl->xAlarm_CommLost = true;
        ctrl->xRemoteModeActive = false;
    }
    else
    {
        ctrl->xAlarm_CommLost = false;
        ctrl->xRemoteModeActive = ctrl->xRemoteMode_Cmd;
    }

    if (ctrl->xRemoteModeActive)
    {
        if (ctrl->xRemotePumpPulse)
        {
            ctrl->xPumpRun = true;
        }

        if (ctrl->xRemoteDrainPulse && (ctrl->iDrainSeqState == 0))
        {
            ctrl->iDrainSeqState = 1;
        }
    }
}

int main(void)
{
    OdorControl odor_ctrl;
    OdorControl *ctrl = &odor_ctrl;
    const uint32_t cycle_ms = 100U;

    OdorControl_Init(ctrl);

    ctrl->xManualMode_SW = true;
    ctrl->rDesignArea_m2 = 0.5f;
    ctrl->rDesignFlow_CMM = 1.0f;

    OdorControl_Step(ctrl, cycle_ms);

    return 0;
}
