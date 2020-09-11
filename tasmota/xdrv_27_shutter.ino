/*
  xdrv_27_Shutter[i].ino - Shutter/Blind support for Tasmota

  Copyright (C) 2020  Stefan Bode

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_SHUTTER
/*********************************************************************************************\
 * Shutter or Blind support using two consecutive relays
\*********************************************************************************************/

#define XDRV_27            27
#ifndef SHUTTER_STEPPER
  #define SHUTTER_STEPPER
#endif

#define D_SHUTTER "SHUTTER"

const uint16_t MOTOR_STOP_TIME = 500;   // in mS
const uint16_t RESOLUTION = 1000;       // incresed to 1000 in 8.5 to ramp servos
const uint8_t  STEPS_PER_SECOND = 20;   // FUNC_EVERY_50_MSECOND
const uint16_t PWM_MAX = 500;
const uint16_t PWM_MIN = 90;

uint8_t  CalibratePos[6] = {0,30,50,70,90,100};
uint16_t Messwerte[5] = {30,50,70,90,100};

int32_t  VelocityMax = 0;
int32_t  VelocityChangePerStepMax = 0;
int32_t  MinRuntimeMs = 0;
int32_t  CurrentStopWay = 0;
int32_t  NextPossibleStopPosition = 0;
int32_t  ToBeAcc = 0;


const uint8_t MAX_MODES = 7;
enum ShutterPositionMode {SHT_UNDEF, SHT_TIME, SHT_TIME_UP_DOWN, SHT_TIME_GARAGE, SHT_COUNTER, SHT_PWM_VALUE, SHT_PWM_TIME,}; // ??? not used anymore
enum ShutterSwitchMode {SHT_SWITCH, SHT_PULSE,}; // ??? not used anymore
enum ShutterButtonStates { SHT_NOT_PRESSED, SHT_PRESSED_MULTI, SHT_PRESSED_HOLD, SHT_PRESSED_IMMEDIATE, SHT_PRESSED_EXT_HOLD, SHT_PRESSED_MULTI_SIMULTANEOUS, SHT_PRESSED_HOLD_SIMULTANEOUS, SHT_PRESSED_EXT_HOLD_SIMULTANEOUS,};

const char kShutterCommands[] PROGMEM = D_PRFX_SHUTTER "|"
  D_CMND_SHUTTER_OPEN "|" D_CMND_SHUTTER_CLOSE "|" D_CMND_SHUTTER_TOGGLE "|" D_CMND_SHUTTER_TOGGLEDIR "|" D_CMND_SHUTTER_STOP "|" D_CMND_SHUTTER_POSITION "|"
  D_CMND_SHUTTER_OPENTIME "|" D_CMND_SHUTTER_CLOSETIME "|" D_CMND_SHUTTER_RELAY "|" D_CMND_SHUTTER_MODE "|"  D_CMND_SHUTTER_PWMRANGE "|"
  D_CMND_SHUTTER_SETHALFWAY "|" D_CMND_SHUTTER_SETCLOSE "|" D_CMND_SHUTTER_SETOPEN "|" D_CMND_SHUTTER_INVERT "|" D_CMND_SHUTTER_CLIBRATION "|"
  D_CMND_SHUTTER_MOTORDELAY "|" D_CMND_SHUTTER_FREQUENCY "|" D_CMND_SHUTTER_BUTTON "|" D_CMND_SHUTTER_LOCK "|" D_CMND_SHUTTER_ENABLEENDSTOPTIME "|" D_CMND_SHUTTER_INVERTWEBBUTTONS "|"
  D_CMND_SHUTTER_STOPOPEN "|" D_CMND_SHUTTER_STOPCLOSE "|" D_CMND_SHUTTER_STOPTOGGLE "|" D_CMND_SHUTTER_STOPTOGGLEDIR "|" D_CMND_SHUTTER_STOPPOSITION;

void (* const ShutterCommand[])(void) PROGMEM = {
  &CmndShutterOpen, &CmndShutterClose, &CmndShutterToggle, &CmndShutterToggleDir, &CmndShutterStop, &CmndShutterPosition,
  &CmndShutterOpenTime, &CmndShutterCloseTime, &CmndShutterRelay, &CmndShutterMode, &CmndShutterPwmRange,
  &CmndShutterSetHalfway, &CmndShutterSetClose, &CmndShutterSetOpen, &CmndShutterInvert, &CmndShutterCalibration , &CmndShutterMotorDelay,
  &CmndShutterFrequency, &CmndShutterButton, &CmndShutterLock, &CmndShutterEnableEndStopTime, &CmndShutterInvertWebButtons,
  &CmndShutterStopOpen, &CmndShutterStopClose, &CmndShutterStopToggle, &CmndShutterStopToggleDir, &CmndShutterStopPosition};

  const char JSON_SHUTTER_POS[] PROGMEM = "\"" D_PRFX_SHUTTER "%d\":{\"Position\":%d,\"Direction\":%d,\"Target\":%d}";
  const char JSON_SHUTTER_BUTTON[] PROGMEM = "\"" D_PRFX_SHUTTER "%d\":{\"Button%d\":%d}";

#include <Ticker.h>

Ticker TickerShutter;

struct SHUTTER {
  uint32_t Time;               // operating time of the shutter in 0.05sec
  int32_t  OpenMax;           // max value on maximum open calculated
  int32_t  TargetPosition;    // position to go to
  int32_t  StartPosition;     // position before a movement is started. init at start
  int32_t  RealPosition;      // value between 0 and Shutter[i].OpenMax
  uint16_t OpenTime;          // duration to open the Shutter[i]. 112 = 11.2sec
  uint16_t CloseTime;         // duration to close the Shutter[i]. 112 = 11.2sec
  uint16_t CloseVelocity;     // in relation to open velocity. higher value = faster
  int8_t   Direction;          // 1 == UP , 0 == stop; -1 == down
  int8_t   LastDirection;      // last direction (1 == UP , -1 == down)
  uint8_t  SwitchMode;        // how to switch relays: SHT_SWITCH, SHT_PULSE
  int16_t  MotorDelay;         // initial motorstarttime in 0.05sec. Also uses for ramp at steppers and servos
  int16_t  PwmVelocity;       // frequency of PWN for stepper motors or PWM duty cycle change for PWM servo
  uint16_t PwmValue;          // dutyload of PWM 0..1023 on ESP8266
  uint16_t CloseVelocityMax; // maximum of PWM change during closeing. Defines velocity on opening. Steppers and Servos only
  int32_t  Accelerator;        // speed of ramp-up, ramp down of shutters with velocity control. Steppers and Servos only
} Shutter[MAX_SHUTTERS];

struct SHUTTERGLOBAL {
  power_t  RelayShutterMask = 0;             // bit mask with 11 at the position of relays that belong to at least ONE shutter
  power_t  RelayOldMask = 0;                 // bitmatrix that contain the last known state of all relays. Required to detemine the manual changed relay.
  power_t  RelayCurrentMask = 0;             // bitmatrix that contain the current state of all relays
  uint8_t  PositionMode = 0;                // how to calculate actual position: SHT_TIME, SHT_COUNTER, SHT_PWM_VALUE, SHT_PWM_TIME
  uint8_t  SkipRelayChange;                // avoid overrun at endstops
  uint8_t  StartReported = 0;               // indicates of the shutter start was reported through MQTT JSON
  uint16_t OpenVelocityMax = 1000;         // maximum of PWM change during opening. Defines velocity on opening. Steppers and Servos only
} ShutterGlobal;

#define SHT_DIV_ROUND(__A, __B) (((__A) + (__B)/2) / (__B))

void ShutterLogPos(uint32_t i)
{
  char stemp2[10];
  dtostrfd((float)Shutter[i].Time / STEPS_PER_SECOND, 2, stemp2);
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Shutter %d Real %d, Start %d, Stop %d, Dir %d, Delay %d, Rtc %s [s], Freq %d, PWM %d"),
    i+1, Shutter[i].RealPosition, Shutter[i].StartPosition, Shutter[i].TargetPosition, Shutter[i].Direction, Shutter[i].MotorDelay, stemp2, Shutter[i].PwmVelocity, Shutter[i].PwmValue);
}

void ExecuteCommandPowerShutter(uint32_t device, uint32_t state, uint32_t source)
{
  // first implementation for virtual relays. Avoid switching relay numbers that do not exist.
  if (device <= devices_present) ExecuteCommandPower(device,state,source);
}

void ShutterUpdateVelocity(uint8_t i)
{
  // No Logging allowed. Part of RTC Timer
  // will be calles through RTC every 50ms.
  Shutter[i].PwmVelocity += Shutter[i].Accelerator;
  Shutter[i].PwmVelocity = tmax(0,tmin(Shutter[i].Direction==1 ? ShutterGlobal.OpenVelocityMax : Shutter[i].CloseVelocityMax,Shutter[i].PwmVelocity));
}

void ShutterRtc50mS(void)
{
  // No Logging allowed. RTC Timer
  for (uint8_t i = 0; i < shutters_present; i++) {
    if (Shutter[i].Direction) {
      // update position data before increasing counter
      Shutter[i].RealPosition =  ShutterCalculatePosition(i);
      Shutter[i].Time++;
      ShutterCalculateAccelerator(i);
      switch (ShutterGlobal.PositionMode) {
        case SHT_PWM_VALUE:
          ShutterUpdateVelocity(i);
          Shutter[i].RealPosition +=  Shutter[i].Direction > 0 ? Shutter[i].PwmVelocity : (Shutter[i].Direction < 0 ? -Shutter[i].PwmVelocity : 0);
          Shutter[i].PwmValue = SHT_DIV_ROUND((Settings.shutter_pwmrange[1][i]-Settings.shutter_pwmrange[0][i]) * Shutter[i].RealPosition , Shutter[i].OpenMax)+Settings.shutter_pwmrange[0][i];
          analogWrite(Pin(GPIO_PWM1, i), Shutter[i].PwmValue);
        break;

        case SHT_COUNTER:
          if (Shutter[i].Accelerator) {
            //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: accelerator i=%d ->  %d"),i, Shutter[i].Accelerator);
            ShutterUpdateVelocity(i);
            analogWriteFreq(Shutter[i].PwmVelocity);
            analogWrite(Pin(GPIO_PWM1, i), 50);
          }
        break;
      }
    } // if (Shutter[i].Direction)
  }
}

int32_t ShutterPercentToRealPosition(uint32_t percent, uint32_t index)
{
	if (Settings.shutter_set50percent[index] != 50) {
    return (percent <= 5) ? Settings.shuttercoeff[2][index] * percent : Settings.shuttercoeff[1][index] * percent + Settings.shuttercoeff[0][index];
	} else {
    uint32_t RealPos;
    // check against DIV 0
    for (uint32_t j = 0; j < 5; j++) {
      if (0 == Settings.shuttercoeff[j][index]) {
        AddLog_P2(LOG_LEVEL_ERROR, PSTR("SHT: RESET/INIT CALIBRATION MATRIX DIV 0"));
        for (uint32_t k = 0; k < 5; k++) {
          Settings.shuttercoeff[k][index] = SHT_DIV_ROUND(CalibratePos[k+1] * 1000, CalibratePos[5]);
        }
      }
    }
    for (uint32_t k = 0; k < 5; k++) {
      if ((percent * 10) >= Settings.shuttercoeff[k][index]) {
        RealPos = SHT_DIV_ROUND(Shutter[index].OpenMax * CalibratePos[k+1], 100);
        //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Realposition TEMP1: %d, %% %d, coeff %d"), RealPos, percent, Settings.shuttercoeff[i][index]);
      } else {
        if (0 == k) {
          RealPos =  SHT_DIV_ROUND(SHT_DIV_ROUND(percent * Shutter[index].OpenMax * CalibratePos[k+1], Settings.shuttercoeff[k][index]), 10);
        } else {
          //uint16_t addon = ( percent*10 - Settings.shuttercoeff[i-1][index] ) * Shutter_Open_Max[index] * (CalibratePos[i+1] - CalibratePos[i]) / (Settings.shuttercoeff[i][index] -Settings.shuttercoeff[i-1][index]) / 100;
          //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Realposition TEMP2: %d, %% %d, coeff %d"), addon, (CalibratePos[i+1] - CalibratePos[i]), (Settings.shuttercoeff[i][index] -Settings.shuttercoeff[i-1][index]));
          RealPos += SHT_DIV_ROUND(SHT_DIV_ROUND((percent*10 - Settings.shuttercoeff[k-1][index] ) * Shutter[index].OpenMax * (CalibratePos[k+1] - CalibratePos[k]), Settings.shuttercoeff[k][index] - Settings.shuttercoeff[k-1][index]), 100);
        }
        break;
      }
    }
		return RealPos;
	}
}

uint8_t ShutterRealToPercentPosition(int32_t RealPos, uint32_t index)
{
	if (Settings.shutter_set50percent[index] != 50) {
		return (Settings.shuttercoeff[2][index] * 5 > RealPos) ? SHT_DIV_ROUND(RealPos, Settings.shuttercoeff[2][index]) : SHT_DIV_ROUND(RealPos-Settings.shuttercoeff[0][index], Settings.shuttercoeff[1][index]);
	} else {
    uint16_t RealPercent;

    for (uint32_t j = 0; j < 5; j++) {
      if (RealPos >= Shutter[index].OpenMax * CalibratePos[j+1] / 100) {
        RealPercent = SHT_DIV_ROUND(Settings.shuttercoeff[j][index], 10);
        //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Realpercent TEMP1: %d, %% %d, coeff %d"), RealPercent, RealPos, Shutter_Open_Max[index] * CalibratePos[i+1] / 100);
      } else {
        if (0 == j) {
          RealPercent  = SHT_DIV_ROUND(SHT_DIV_ROUND((RealPos - SHT_DIV_ROUND(Shutter[index].OpenMax * CalibratePos[j], 100)) * 10 * Settings.shuttercoeff[j][index], CalibratePos[j+1]), Shutter[index].OpenMax);
        } else {
          //uint16_t addon = ( RealPos - (Shutter_Open_Max[index] * CalibratePos[i] / 100) ) * 10 * (Settings.shuttercoeff[i][index] - Settings.shuttercoeff[i-1][index]) / (CalibratePos[i+1] - CalibratePos[i])/ Shutter_Open_Max[index];
          //uint16_t addon = ( percent*10 - Settings.shuttercoeff[i-1][index] ) * Shutter_Open_Max[index] * (CalibratePos[i+1] - CalibratePos[i]) / (Settings.shuttercoeff[i][index] -Settings.shuttercoeff[i-1][index]) / 100;
          //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Realpercent TEMP2: %d, delta %d,  %% %d, coeff %d"), addon,( RealPos - (Shutter_Open_Max[index] * CalibratePos[i] / 100) ) , (CalibratePos[i+1] - CalibratePos[i])* Shutter_Open_Max[index]/100, (Settings.shuttercoeff[i][index] -Settings.shuttercoeff[i-1][index]));
          RealPercent += SHT_DIV_ROUND(SHT_DIV_ROUND((RealPos - SHT_DIV_ROUND(Shutter[index].OpenMax * CalibratePos[j], 100)) * 10 * (Settings.shuttercoeff[j][index] - Settings.shuttercoeff[j-1][index]), (CalibratePos[j+1] - CalibratePos[j])), Shutter[index].OpenMax) ;
        }
        break;
      }
    }
    return (int16_t)RealPercent < 0 ? 0 : RealPercent;
  }
}

void ShutterInit(void)
{
  shutters_present = 0;
  ShutterGlobal.RelayShutterMask = 0;
  //Initialize to get relay that changed
  ShutterGlobal.RelayOldMask = power;


  // if shutter 4 is unused
  if (Settings.shutter_startrelay[MAX_SHUTTERS -1] == 0) {
     ShutterGlobal.OpenVelocityMax = Settings.shuttercoeff[4][3] > 0 ? Settings.shuttercoeff[4][3] : ShutterGlobal.OpenVelocityMax;
  }
  for (uint32_t i = 0; i < MAX_SHUTTERS; i++) {
    // set startrelay to 1 on first init, but only to shutter 1. 90% usecase
    Settings.shutter_startrelay[i] = (Settings.shutter_startrelay[i] == 0 && i ==  0? 1 : Settings.shutter_startrelay[i]);
    if (Settings.shutter_startrelay[i] && (Settings.shutter_startrelay[i] < 9)) {
      shutters_present++;

      // Add the two relays to the mask to knaw they belong to shutters
      ShutterGlobal.RelayShutterMask |= 3 << (Settings.shutter_startrelay[i] -1)  ;

      // All shutters must have same mode. Switch OR Pulse. N
      switch (Settings.pulse_timer[i]) {
        case 0:
          Shutter[i].SwitchMode = SHT_SWITCH;
        break;
        default:
          Shutter[i].SwitchMode = SHT_PULSE;
        break;
      }

      if (Settings.shutter_mode == SHT_UNDEF) {
        bool RelayInInterlock = false;
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: mode undef.. calculate..."));

        for (uint32_t j = 0; j < MAX_INTERLOCKS * Settings.flag.interlock; j++) {  // CMND_INTERLOCK - Enable/disable interlock
          //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Interlock state i=%d %d, flag %d, , shuttermask %d, maskedIL %d"),i, Settings.interlock[i], Settings.flag.interlock,ShutterGlobal.RelayShutterMask, Settings.interlock[i]&ShutterGlobal.RelayShutterMask);
          if (Settings.interlock[j] && (Settings.interlock[j] & ShutterGlobal.RelayShutterMask)) {
            //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Relay in Interlock group"));
            RelayInInterlock = true;
          }
        }

        if (RelayInInterlock) {
          ShutterGlobal.PositionMode = SHT_TIME;
        } else {
          ShutterGlobal.PositionMode = SHT_TIME_UP_DOWN;
          if (PinUsed(GPIO_PWM1, i) && PinUsed(GPIO_CNTR1, i)) {
            ShutterGlobal.PositionMode = SHT_COUNTER;
          }
        }

      } else {
        ShutterGlobal.PositionMode = Settings.shutter_mode;
      }

      // main function for stepper and servos to control velocity and acceleration.
      TickerShutter.attach_ms(50, ShutterRtc50mS );

      // default the 50 percent should not have any impact without changing it. set to 60
      Settings.shutter_set50percent[i] = (Settings.shutter_set50percent[i] > 0) ? Settings.shutter_set50percent[i] : 50;

      // use 10 sec. as default to allow everybody to play without deep initialize
      Shutter[i].OpenTime = Settings.shutter_opentime[i] = (Settings.shutter_opentime[i] > 0) ? Settings.shutter_opentime[i] : 100;
      Shutter[i].CloseTime = Settings.shutter_closetime[i] = (Settings.shutter_closetime[i] > 0) ? Settings.shutter_closetime[i] : 100;

      // Update Calculation 20 because time interval is 0.05 sec ans time is in 0.1sec
      Shutter[i].OpenMax = STEPS_PER_SECOND * RESOLUTION * Shutter[i].OpenTime / 10;
      Shutter[i].CloseVelocity =  Shutter[i].OpenMax / Shutter[i].CloseTime / 2 ;

      // calculate a ramp slope at the first 5 percent to compensate that shutters move with down part later than the upper part
      if (Settings.shutter_set50percent[i] != 50) {
      	Settings.shuttercoeff[1][i] = Shutter[i].OpenMax * (100 - Settings.shutter_set50percent[i] ) / 5000;
      	Settings.shuttercoeff[0][i] = Shutter[i].OpenMax - (Settings.shuttercoeff[1][i] * 100);
      	Settings.shuttercoeff[2][i] = (Settings.shuttercoeff[0][i] + 5 * Settings.shuttercoeff[1][i]) / 5;
      }
      ShutterGlobal.RelayShutterMask |= 3 << (Settings.shutter_startrelay[i] -1);

      Shutter[i].RealPosition = ShutterPercentToRealPosition(Settings.shutter_position[i], i);

      Shutter[i].StartPosition = Shutter[i].TargetPosition = Shutter[i].RealPosition;
      Shutter[i].MotorDelay = Settings.shutter_motordelay[i];
      Shutter[i].LastDirection = (50 < Settings.shutter_position[i]) ? 1 : -1;

      switch (ShutterGlobal.PositionMode) {
        case SHT_PWM_VALUE:
          ShutterGlobal.OpenVelocityMax =  RESOLUTION;
          // Initiate pwm range with defaults if not already set.
          Settings.shutter_pwmrange[0][i] = Settings.shutter_pwmrange[0][i] > 0 ? Settings.shutter_pwmrange[0][i] : PWM_MIN;
          Settings.shutter_pwmrange[1][i] = Settings.shutter_pwmrange[1][i] > 0 ? Settings.shutter_pwmrange[1][i] : PWM_MAX;
        break;
      }
      Shutter[i].CloseVelocityMax = ShutterGlobal.OpenVelocityMax*Shutter[i].OpenTime / Shutter[i].CloseTime;

      //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Shutter %d Openvel %d, Closevel: %d"),i, ShutterGlobal.OpenVelocityMax, Shutter[i].CloseVelocityMax);
      AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT%d: Init. Pos: %d,inverted %d, locked %d, end stop time enabled %d, webButtons inverted %d"),
        i+1,  Shutter[i].RealPosition,
        (Settings.shutter_options[i]&1) ? 1 : 0, (Settings.shutter_options[i]&2) ? 1 : 0, (Settings.shutter_options[i]&4) ? 1 : 0, (Settings.shutter_options[i]&8) ? 1 : 0);

    } else {
      // terminate loop at first INVALID Shutter[i].
      break;
    }
    ShutterLimitRealAndTargetPositions(i);
    Settings.shutter_accuracy = 1;

  }
}

void ShutterReportPosition(bool Always, uint32_t Index)
{
  Response_P(PSTR("{"));
  rules_flag.shutter_moving = 0;
  uint32_t i = 0;
  uint32_t n = shutters_present;
  if( Index != MAX_SHUTTERS) {
    i = Index;
    n = Index+1;
  }
  for (i; i < n; i++) {
    //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Shutter %d: Real Pos: %d"), i+1,Shutter[i].RealPosition);
    uint32_t Position = ShutterRealToPercentPosition(Shutter[i].RealPosition, i);
    if (Shutter[i].Direction != 0) {
      rules_flag.shutter_moving = 1;
      ShutterLogPos(i);
    }
    if (i && Index == MAX_SHUTTERS) { ResponseAppend_P(PSTR(",")); }
    uint32_t Target = ShutterRealToPercentPosition(Shutter[i].TargetPosition, i);
    ResponseAppend_P(JSON_SHUTTER_POS, i+1, (Settings.shutter_options[i] & 1) ? 100 - Position : Position, Shutter[i].Direction,(Settings.shutter_options[i] & 1) ? 100 - Target : Target );
  }
  ResponseJsonEnd();
  if (Always || (rules_flag.shutter_moving)) {
    MqttPublishPrefixTopicRulesProcess_P(RESULT_OR_STAT, PSTR(D_PRFX_SHUTTER));  // RulesProcess() now re-entry protected
  }

  //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: rules_flag.shutter_moving: %d, moved %d"), rules_flag.shutter_moving, rules_flag.shutter_moved);

}

void ShutterLimitRealAndTargetPositions(uint32_t i) {
  if (Shutter[i].RealPosition<0) Shutter[i].RealPosition = 0;
  if (Shutter[i].RealPosition>Shutter[i].OpenMax) Shutter[i].RealPosition = Shutter[i].OpenMax;
  if (Shutter[i].TargetPosition<0) Shutter[i].TargetPosition = 0;
  if (Shutter[i].TargetPosition>Shutter[i].OpenMax) Shutter[i].TargetPosition = Shutter[i].OpenMax;
}

void ShutterCalculateAccelerator(uint8_t i)
{
  // No Logging allowed. Part of RTC Timer
  if (Shutter[i].Direction != 0) {
    switch (ShutterGlobal.PositionMode) {
      case SHT_COUNTER:
      case SHT_PWM_VALUE:
        // calculate max velocity allowed in this direction
        VelocityMax = Shutter[i].Direction == 1 ? ShutterGlobal.OpenVelocityMax : Shutter[i].CloseVelocityMax;
        // calculate max change of velocyty based on the defined motordelay in steps
        VelocityChangePerStepMax =  VelocityMax / (Shutter[i].MotorDelay>0 ? Shutter[i].MotorDelay : 1);
        // minimumtime required from current velocity to stop
        MinRuntimeMs = Shutter[i].PwmVelocity * 1000 / STEPS_PER_SECOND / VelocityChangePerStepMax;
        // decellartion way from current velocity
        CurrentStopWay = (MinRuntimeMs * (Shutter[i].PwmVelocity + VelocityChangePerStepMax)/100 - Shutter[i].PwmVelocity)*RESOLUTION/ShutterGlobal.OpenVelocityMax * Shutter[i].Direction ;
        NextPossibleStopPosition = Shutter[i].RealPosition + CurrentStopWay ;
        ToBeAcc = 0;
        // ensure that the accelerotor kicks in at least one step BEFORE it is to late and a hard stop required.
        if (Shutter[i].Accelerator < 0 || (NextPossibleStopPosition * Shutter[i].Direction) +RESOLUTION*Shutter[i].PwmVelocity/ShutterGlobal.OpenVelocityMax>= Shutter[i].TargetPosition * Shutter[i].Direction ) {
            // 10 times the deviation is the p-value of this simple p-regulator
            ToBeAcc = 100+(Shutter[i].Direction*(NextPossibleStopPosition-Shutter[i].TargetPosition)*velocity_max/Shutter[i].PwmVelocity*10/RESOLUTION);
            Shutter[i].Accelerator = - tmin(tmax( VelocityChangePerStepMax * ToBeAcc / 100  , (VelocityChangePerStepMax * 9 / 10)), (VelocityChangePerStepMax * 11 / 10));
        } else if (  Shutter[i].Accelerator > 0 && Shutter[i].PwmVelocity ==  VelocityMax) {
          Shutter[i].Accelerator = 0;
        }
      break;
    }
  }
}

void ShutterDecellerateForStop(uint8_t i)
{
  switch (ShutterGlobal.PositionMode) {
    case SHT_PWM_VALUE:
    case SHT_COUNTER:
      int16_t MissingSteps;
      Shutter[i].Accelerator = -(ShutterGlobal.OpenVelocityMax / (Shutter[i].MotorDelay>0 ? Shutter[i].MotorDelay : 1) *11/10);
      while (Shutter[i].PwmVelocity > -2*Shutter[i].Accelerator ) {
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: velocity: %ld, delta: %d"), Shutter[i].PwmVelocity, Shutter[i].Accelerator );
        //Shutter[i].PwmVelocity = tmax(Shutter[i].PwmVelocity-Shutter[i].Accelerator , 0);
        // Control will be done in RTC Ticker.
        delay(50);
      }
      if (ShutterGlobal.PositionMode == SHT_COUNTER){
        MissingSteps = ((Shutter[i].TargetPosition-Shutter[i].StartPosition)*Shutter[i].Direction*ShutterGlobal.OpenVelocityMax/RESOLUTION/STEPS_PER_SECOND) - RtcSettings.pulse_counter[i];
        //prepare for stop PWM
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Remain steps %d, counter %d, freq %d"), MissingSteps, RtcSettings.pulse_counter[i] ,Shutter[i].PwmVelocity);
        Shutter[i].Accelerator = 0;
        Shutter[i].PwmVelocity = Shutter[i].PwmVelocity > 250 ? 250 : Shutter[i].PwmVelocity;
        analogWriteFreq(Shutter[i].PwmVelocity);
        analogWrite(Pin(GPIO_PWM1, i), 50);
        Shutter[i].PwmVelocity = 0;
        analogWriteFreq(Shutter[i].PwmVelocity);
        while (RtcSettings.pulse_counter[i] < (uint32_t)(Shutter[i].TargetPosition-Shutter[i].StartPosition)*Shutter[i].Direction*ShutterGlobal.OpenVelocityMax/RESOLUTION/STEPS_PER_SECOND) {
          delay(1);
        }
        analogWrite(Pin(GPIO_PWM1, i), 0); // removed with 8.3 because of reset caused by watchog
        Shutter[i].RealPosition = ShutterCalculatePosition(i);
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Real %d, pulsecount %d, start %d"), Shutter[i].RealPosition,RtcSettings.pulse_counter[i], Shutter[i].StartPosition);

      }
      Shutter[i].Direction = 0;
      Shutter[i].PwmVelocity = 0;
    break;
  }
}

void ShutterPowerOff(uint8_t i) {
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Stop Shutter %d .."), i);
  ShutterDecellerateForStop(i);
  if (Shutter[i].Direction !=0) {
    Shutter[i].Direction = 0;
    delay(MOTOR_STOP_TIME);
  }
  switch (Shutter[i].SwitchMode) {
    case SHT_SWITCH:
      if ((1 << (Settings.shutter_startrelay[i]-1)) & power) {
        ExecuteCommandPowerShutter(Settings.shutter_startrelay[i], 0, SRC_SHUTTER);
      }
      if ((1 << (Settings.shutter_startrelay[i])) & power) {
        ExecuteCommandPowerShutter(Settings.shutter_startrelay[i]+1, 0, SRC_SHUTTER);
      }
    break;
    case SHT_PULSE:
      uint8_t CurRelay = Settings.shutter_startrelay[i] + (Shutter[i].Direction == 1 ? 0 : (uint8_t)(ShutterGlobal.PositionMode == SHT_TIME)) ;
      // we have a momentary switch here. Needs additional pulse on same relay after the end
      if ((SRC_PULSETIMER == last_source || SRC_SHUTTER == last_source || SRC_WEBGUI == last_source)) {
        ExecuteCommandPowerShutter(CurRelay, 1, SRC_SHUTTER);
        // switch off direction relay to make it power less
        if ((1 << (Settings.shutter_startrelay[i])) & power) {
          ExecuteCommandPowerShutter(Settings.shutter_startrelay[i]+1, 0, SRC_SHUTTER);
        }
      } else {
        last_source = SRC_SHUTTER;
      }
    break;
  }
  // Store current PWM value to ensure proper position after reboot.
  switch (ShutterGlobal.PositionMode) {
    case SHT_PWM_VALUE:
    char sCmnd[20];
    snprintf_P(sCmnd, sizeof(sCmnd), PSTR(D_CMND_PWM " %d" ),Shutter[i].PwmValue);
    ExecuteCommand(sCmnd, SRC_BUTTON);
    break;
  }
}

void ShutterUpdatePosition(void)
{

  char sCommand[CMDSZ];
  char sTopic[TOPSZ];

  for (uint32_t i = 0; i < shutters_present; i++) {
    if (Shutter[i].Direction != 0) {

      // Calculate position with counter. Much more accurate and no need for motordelay workaround
      //                        adding some steps to stop early
      //Shutter[i].RealPosition =  ShutterCalculatePosition(i);
      if (!ShutterGlobal.StartReported) {
        ShutterReportPosition(true, i);
        XdrvRulesProcess();
        ShutterGlobal.StartReported = 1;
      }
      //ShutterCalculateAccelerator(i);
      AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: time: %d, ToBeAcc %d, CurrentStopWay %d,vel_vur %d, vel_max %d, act_vel_change %d, MinRuntimeMs %d, act.pos %d, next_stop %d, target: %d,  VelocityChangePerStepMax %d"),Shutter[i].Time,ToBeAcc,CurrentStopWay,
                                    Shutter[i].PwmVelocity,velocity_max, Shutter[i].Accelerator,MinRuntimeMs,Shutter[i].RealPosition, NextPossibleStopPosition,Shutter[i].TargetPosition,VelocityChangePerStepMax);


      if ( Shutter[i].RealPosition * Shutter[i].Direction >= Shutter[i].TargetPosition * Shutter[i].Direction || Shutter[i].PwmVelocity < VelocityChangePerStepMax) {
        if (Shutter[i].Direction != 0) {
          Shutter[i].LastDirection = Shutter[i].Direction;
        }
        ShutterPowerOff(i);
        ShutterLimitRealAndTargetPositions(i);
        Settings.shutter_position[i] = ShutterRealToPercentPosition(Shutter[i].RealPosition, i);
        Shutter[i].StartPosition = Shutter[i].RealPosition;

        ShutterLogPos(i);

        // sending MQTT result to broker
        snprintf_P(sCommand, sizeof(sCommand),PSTR(D_SHUTTER "%d"), i+1);
        GetTopic_P(sTopic, STAT, mqtt_topic, sCommand);
        Response_P("%d", (Settings.shutter_options[i] & 1) ? 100 - Settings.shutter_position[i]: Settings.shutter_position[i]);
        MqttPublish(sTopic, Settings.flag.mqtt_power_retain);  // CMND_POWERRETAIN
        ShutterReportPosition(true, i);
        rules_flag.shutter_moved = 1;
        XdrvRulesProcess();
      }
    }
  }
}

bool ShutterState(uint32_t Device)
{
  Device--;
  Device &= 3;
  return (Settings.flag3.shutter_mode &&  // SetOption80 - Enable shutter support
          (ShutterGlobal.RelayShutterMask & (1 << (Settings.shutter_startrelay[Device]-1))) );
}

void ShutterStartInit(uint32_t i, int32_t Direction, int32_t TargetPos)
{
  //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: dir %d, delta1 %d, delta2 %d, grant %d"),Direction, (Shutter[i].OpenMax - Shutter[i].RealPosition) / Shutter[i].CloseVelocity, Shutter[i].RealPosition / Shutter[i].CloseVelocity, 2+Shutter[i].MotorDelay);
  if (  ( (1 == Direction) && ((Shutter[i].OpenMax - Shutter[i].RealPosition) / 100 <= 2) )
     || ( (-1 == Direction) && (Shutter[i].RealPosition / Shutter[i].CloseVelocity <= 2)) ) {
    ShutterGlobal.SkipRelayChange = 1;
  } else {
    Shutter[i].PwmVelocity = 0;
    switch (ShutterGlobal.PositionMode) {
#ifdef SHUTTER_STEPPER
      case SHT_COUNTER:
        analogWriteFreq(Shutter[i].PwmVelocity);
        analogWrite(Pin(GPIO_PWM1, i), 0);
        RtcSettings.pulse_counter[i] = 0;
      break;
#endif
    }
    Shutter[i].Accelerator = ShutterGlobal.OpenVelocityMax / (Shutter[i].MotorDelay>0 ? Shutter[i].MotorDelay : 1);
    Shutter[i].TargetPosition = TargetPos;
    Shutter[i].StartPosition = Shutter[i].RealPosition;
    Shutter[i].Time = 0;
    ShutterGlobal.SkipRelayChange = 0;
    Shutter[i].Direction = Direction;
    rules_flag.shutter_moving = 1;
    rules_flag.shutter_moved  = 0;
    ShutterGlobal.StartReported = 0;
    //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: real %d, start %d, counter %d,freq_max %d, dir %d, freq %d"),Shutter[i].RealPosition, Shutter[i].StartPosition ,RtcSettings.pulse_counter[i],ShutterGlobal.OpenVelocityMax , Shutter[i].Direction ,ShutterGlobal.OpenVelocityMax );
  }
  //AddLog_P2(LOG_LEVEL_DEBUG,  PSTR("SHT: Start shutter: %d from %d to %d in direction %d"), i, Shutter[i].StartPosition, Shutter[i].TargetPosition, Shutter[i].Direction);
}

int32_t ShutterCalculatePosition(uint32_t i)
{
    // No Logging allowed. Part of RTC Timer
  if (Shutter[i].Direction != 0) {
    switch (ShutterGlobal.PositionMode) {
      case SHT_COUNTER:
        return ((int32_t)RtcSettings.pulse_counter[i]*Shutter[i].Direction*STEPS_PER_SECOND*RESOLUTION / ShutterGlobal.OpenVelocityMax)+Shutter[i].StartPosition;
        break;
      case SHT_TIME:
      case SHT_TIME_UP_DOWN:
      case SHT_TIME_GARAGE:
        return Shutter[i].StartPosition + ( (Shutter[i].Time - Shutter[i].MotorDelay) * (Shutter[i].Direction > 0 ? RESOLUTION : -Shutter[i].CloseVelocity));
        break;
      case SHT_PWM_TIME:
        break;
      case SHT_PWM_VALUE:
        return Shutter[i].RealPosition;
      break;
      default:
        break;
      }
    } else {
      return Shutter[i].RealPosition;
    }
}

void ShutterRelayChanged(void)
{

  // ShutterGlobal.RelayCurrentMask = binary relay that was recently changed and cause an Action
  // powerstate_local = binary powermatrix and relays from shutter: 0..3
  // relays_changed = bool if one of the relays that belong to the shutter changed not by shutter or pulsetimer
  char sTemp1[10];

	for (uint32_t i = 0; i < shutters_present; i++) {
		power_t powerstate_local = (power >> (Settings.shutter_startrelay[i] -1)) & 3;
    // SRC_IGNORE added because INTERLOCK function bite causes this as last source for changing the relay.
		//uint8   manual_relays_changed = ((ShutterGlobal.RelayCurrentMask >> (Settings.shutter_startrelay[i] -1)) & 3) && SRC_IGNORE != last_source && SRC_SHUTTER != last_source && SRC_PULSETIMER != last_source ;
    uint8   manual_relays_changed = ((ShutterGlobal.RelayCurrentMask >> (Settings.shutter_startrelay[i] -1)) & 3) && SRC_SHUTTER != last_source && SRC_PULSETIMER != last_source ;
    AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Shutter %d: source: %s, powerstate_local %ld, ShutterGlobal.RelayCurrentMask %d, manual change %d"), i+1, GetTextIndexed(sTemp1, sizeof(sTemp1), last_source, kCommandSource), powerstate_local,ShutterGlobal.RelayCurrentMask,manual_relays_changed);
    if (manual_relays_changed) {
      //ShutterGlobal.SkipRelayChange = true;
      ShutterLimitRealAndTargetPositions(i);
      switch (Shutter[i].SwitchMode ) {
        case SHT_PULSE:
          if (Shutter[i].Direction != 0 && powerstate_local) {
            Shutter[i].TargetPosition = Shutter[i].RealPosition;
            powerstate_local = 0;
  					AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Shutter %d: Switch OFF motor. Target: %ld, source: %s, powerstate_local %ld, ShutterGlobal.RelayCurrentMask %d, manual change %d"), i+1, Shutter[i].TargetPosition, GetTextIndexed(stemp1, sizeof(stemp1), last_source, kCommandSource), powerstate_local,ShutterGlobal.RelayCurrentMask,manual_relays_changed);
          }
        break;
        default:
          last_source = SRC_SHUTTER; // avoid switch off in the next loop
          if (Shutter[i].Direction != 0 ) ShutterPowerOff(i);
      }
      switch (ShutterGlobal.PositionMode) {
        // enum ShutterPositionMode {SHT_TIME, SHT_TIME_UP_DOWN, SHT_TIME_GARAGE, SHT_COUNTER, SHT_PWM_VALUE, SHT_PWM_TIME,};
        case SHT_TIME_UP_DOWN:
        case SHT_COUNTER:
        case SHT_PWM_VALUE:
        case SHT_PWM_TIME:
          ShutterPowerOff(i);
          switch (powerstate_local) {
            case 1:
              ShutterStartInit(i, 1, Shutter[i].OpenMax);
              break;
            case 3:
              ShutterStartInit(i, -1, 0);
              break;
            default:
              //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Shutter %d: Switch OFF motor."),i);
              Shutter[i].TargetPosition = Shutter[i].RealPosition;
            }
        break;
        case SHT_TIME:
          switch (powerstate_local) {
            case 1:
              ShutterStartInit(i, 1, Shutter[i].OpenMax);
              break;
            case 2:
              ShutterStartInit(i, -1, 0);
              break;
              default:
                //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Shutter %d: Switch OFF motor."),i);
                Shutter[i].TargetPosition = Shutter[i].RealPosition;
          }
        break;
        case SHT_TIME_GARAGE:
         switch (powerstate_local) {
           case 1:
             ShutterStartInit(i, Shutter[i].LastDirection*-1 , Shutter[i].LastDirection == 1 ?  0 : Shutter[i].OpenMax);
             AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Shutter %d Garage. NewTarget %d"), i, Shutter[i].TargetPosition);
           break;
           default:
            Shutter[i].TargetPosition = Shutter[i].RealPosition;
         }


			  } // switch (ShutterGlobal.PositionMode)
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Shutter %d: Target: %ld, powerstatelocal %d"), i+1, Shutter[i].TargetPosition, powerstate_local);
		 } // if (manual_relays_changed)
  } // for (uint32_t i = 0; i < shutters_present; i++)
}

bool ShutterButtonIsSimultaneousHold(uint32_t ButtonIndex, uint32_t ShutterIndex) {
  // check for simultaneous shutter button hold
  uint32 min_shutterbutton_hold_timer = -1; // -1 == max(uint32)
  for (uint32_t i = 0; i < MAX_KEYS; i++) {
    if ((ButtonIndex != i) && (Settings.shutter_button[i] & (1<<31)) && ((Settings.shutter_button[i] & 0x03) == ShutterIndex) && (Button.hold_timer[i] < min_shutterbutton_hold_timer))
      min_shutterbutton_hold_timer = Button.hold_timer[i];
  }
  return ((-1 != min_shutterbutton_hold_timer) && (min_shutterbutton_hold_timer > (Button.hold_timer[ButtonIndex]>>1)));
}

void ShutterButtonHandler(void)
{
  uint8_t ButtonState = SHT_NOT_PRESSED;
  uint8_t Button = XdrvMailbox.payload;
  uint8_t PressIndex;
  uint32_t ButtonIndex = XdrvMailbox.index;
  uint8_t ShutterIndex = Settings.shutter_button[ButtonIndex] & 0x03;
  uint16_t LoopsPerSecond = 1000 / Settings.button_debounce;  // ButtonDebounce (50)

  if ((PRESSED == Button) && (NOT_PRESSED == Button.last_state[ButtonIndex])) {
    if (Settings.flag.button_single) {                   // SetOption13 (0) - Allow only single button press for immediate action
        ButtonState = SHT_PRESSED_MULTI;
        PressIndex = 1;
    } else {
      if ((Shutter[ShutterIndex].Direction) && (Button.press_counter[ButtonIndex]==0)) {
        ButtonState = SHT_PRESSED_IMMEDIATE;
        PressIndex = 1;
        Button.press_counter[ButtonIndex] = 99; // Remember to discard further action for press & hold within button timings
      } else {
        Button.press_counter[ButtonIndex] = (Button.window_timer[ButtonIndex]) ? Button.press_counter[ButtonIndex] +1 : 1;
        // Button.window_timer[ButtonIndex] = (Button.press_counter[ButtonIndex]==1) ? LoopsPerSecond / 2 : LoopsPerSecond;  // 0.5 second multi press window after 1st press, 1s afterwards
        Button.window_timer[ButtonIndex] = (LoopsPerSecond >> 2) * 3; // 0.75 second multi press window
      }
    }
    blinks = 201;
  }

  if (NOT_PRESSED == Button) {
    Button.hold_timer[ButtonIndex] = 0;
  } else {
    Button.hold_timer[ButtonIndex]++;
    if (!Settings.flag.button_single) {                   // SetOption13 (0) - Allow only single button press for immediate action
      if (Settings.param[P_HOLD_IGNORE] > 0) {         // SetOption40 (0) - Do not ignore button hold
        if (Button.hold_timer[ButtonIndex] > LoopsPerSecond * Settings.param[P_HOLD_IGNORE] / 10) {
          Button.hold_timer[ButtonIndex] = 0;         // Reset button hold counter to stay below hold trigger
          Button.press_counter[ButtonIndex] = 0;      // Discard button press to disable functionality
        }
      }
      if ((Button.press_counter[ButtonIndex]<99) && (Button.hold_timer[ButtonIndex] == LoopsPerSecond * Settings.param[P_HOLD_TIME] / 10)) {  // press still valid && SetOption32 (40) - Button hold
        // check for simultaneous shutter button hold
        if (ShutterButtonIsSimultaneousHold(ButtonIndex, ShutterIndex)) {
          // simultaneous shutter button hold detected
          for (uint32_t i = 0; i < MAX_KEYS; i++)
            if ((Settings.shutter_button[i] & (1<<31)) && ((Settings.shutter_button[i] & 0x03) == ShutterIndex))
              Button.press_counter[i] = 99; // Remember to discard further action for press & hold within button timings
          PressIndex = 0;
          ButtonState = SHT_PRESSED_HOLD_SIMULTANEOUS;
        }
        if (Button.press_counter[ButtonIndex]<99) {
          PressIndex = 0;
          ButtonState = SHT_PRESSED_HOLD;
        }
        Button.press_counter[ButtonIndex] = 0;
      }
      if ((Button.press_counter[ButtonIndex]==0) && (Button.hold_timer[ButtonIndex] == LoopsPerSecond * IMMINENT_RESET_FACTOR * Settings.param[P_HOLD_TIME] / 10)) {  // SetOption32 (40) - Button held for factor times longer
        PressIndex = -1;
        // check for simultaneous shutter button extend hold
        if (ShutterButtonIsSimultaneousHold(ButtonIndex, ShutterIndex)) {
          // simultaneous shutter button extend hold detected
          ButtonState = SHT_PRESSED_EXT_HOLD_SIMULTANEOUS;
        } else {
          ButtonState = SHT_PRESSED_EXT_HOLD;
        }
      }
    }
  }

  if (!Settings.flag.button_single) {                    // SetOption13 (0) - Allow multi-press
    if (Button.window_timer[ButtonIndex]) {
      Button.window_timer[ButtonIndex]--;
    } else {
      if (!restart_flag && !Button.hold_timer[ButtonIndex] && (Button.press_counter[ButtonIndex] > 0)) {
        if (Button.press_counter[ButtonIndex]<99) {
          // check for simultaneous shutter button press
          uint32 min_shutterbutton_press_counter = -1; // -1 == max(uint32)
          for (uint32_t i = 0; i < MAX_KEYS; i++) {
            AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Settings.shutter_button[i] %ld, ShutterIndex %d, Button.press_counter[i] %d, min_shutterbutton_press_counter %d, i %d"), Settings.shutter_button[i], ShutterIndex, Button.press_counter[i] , min_shutterbutton_press_counter, i);
            if ((ButtonIndex != i) && (Settings.shutter_button[i] & (1<<31)) && ((Settings.shutter_button[i] & 0x03) == ShutterIndex) && (i != ButtonIndex) && (Button.press_counter[i] < min_shutterbutton_press_counter)) {
              min_shutterbutton_press_counter = Button.press_counter[i];
              AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: min_shutterbutton_press_counter %d"), min_shutterbutton_press_counter);
            }
          }
          if (min_shutterbutton_press_counter == Button.press_counter[ButtonIndex]) {
            // simultaneous shutter button press detected
            AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: simultanous presss deteced"));
            PressIndex = Button.press_counter[ButtonIndex];
            for (uint32_t i = 0; i < MAX_KEYS; i++)
              if ((Settings.shutter_button[i] & (1<<31)) && ((Settings.shutter_button[i] & 0x03) != ShutterIndex))
                Button.press_counter[i] = 99; // Remember to discard further action for press & hold within button timings
            ButtonState = SHT_PRESSED_MULTI_SIMULTANEOUS;
          }
          if ((ButtonState != SHT_PRESSED_MULTI_SIMULTANEOUS) && (Button.press_counter[ButtonIndex]<99)) {
            // no simultaneous shutter button press >3 detected
            PressIndex = Button.press_counter[ButtonIndex];
            ButtonState = SHT_PRESSED_MULTI;
          }
        }
        Button.press_counter[ButtonIndex] = 0;
      }
    }
  }

  if (ButtonState != SHT_NOT_PRESSED) {
    if ((!Settings.flag.button_restrict) && (((PressIndex>=5) && (PressIndex<=7)) || (ButtonState == SHT_PRESSED_EXT_HOLD) || (ButtonState == SHT_PRESSED_EXT_HOLD_SIMULTANEOUS))){
      // check number of buttons for this shutter
      uint8_t ShutterIndexNumButtons = 0;  
      for (uint32_t i = 0; i < MAX_KEYS; i++) {
        if ((Settings.shutter_button[i] & (1<<31)) && ((Settings.shutter_button[i] & 0x03) == ShutterIndex)) {
          ShutterIndexNumButtons++;
        }
      }
      if ((ButtonState == SHT_PRESSED_MULTI_SIMULTANEOUS) || ((ShutterIndexNumButtons==1) && (ButtonState == SHT_PRESSED_MULTI))){
        // 5x..7x && no SetOption1 (0) checked above
        // simultaneous or stand alone button press 5x, 6x, 7x detected
        char sCmnd[20]; // ??? könnte vor das "if", damit die korrespondierende Zeile aus dem "else" gelöscht werden kann!
        snprintf_P(sCmnd, sizeof(sCmnd), PSTR(D_CMND_WIFICONFIG " 2"));
        ExecuteCommand(sCmnd, SRC_BUTTON);  // ??? Diese Zeile und die nachfolgende können nach dem "if" folgen, wenn man die korrespondierenden aus dem "else" löscht!
        return;
      } else if ((ButtonState == SHT_PRESSED_EXT_HOLD_SIMULTANEOUS) || ((ShutterIndexNumButtons==1) && (ButtonState == SHT_PRESSED_EXT_HOLD))){
        // no SetOption1 (0) checked above
        // simultaneous or stand alone button extended hold detected
        char sCmnd[20];
        snprintf_P(sCmnd, sizeof(sCmnd), PSTR(D_CMND_RESET " 1"));
        ExecuteCommand(sCmnd, SRC_BUTTON);
        return;
      }
    }
    if (ButtonState <= SHT_PRESSED_IMMEDIATE) {
      if (Settings.shutter_startrelay[ShutterIndex] && Settings.shutter_startrelay[ShutterIndex] <9) {
        uint8_t PosPressIndex = (ButtonState == SHT_PRESSED_HOLD) ? 3 : (PressIndex-1);
        if (PosPressIndex>3) PosPressIndex=3;
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: shutter %d, button %d = %d (single=1, double=2, tripple=3, hold=4)"), ShutterIndex+1, ButtonIndex+1, PosPressIndex+1);
        XdrvMailbox.index = ShutterIndex +1;
        last_source = SRC_BUTTON;
        XdrvMailbox.data_len = 0;
        char DataBuffer[1] = "";
        XdrvMailbox.data = DataBuffer;
        XdrvMailbox.command = NULL;
        if (ButtonState == SHT_PRESSED_IMMEDIATE) {
          XdrvMailbox.payload = XdrvMailbox.index;
          CmndShutterStop();
        } else {
          uint8_t Position = (Settings.shutter_button[ButtonIndex]>>(6*PosPressIndex + 2)) & 0x03f;
          if (Position) {
            if (Shutter[ShutterIndex].Direction) {
              XdrvMailbox.payload = XdrvMailbox.index;
              CmndShutterStop();
            } else {
              XdrvMailbox.payload = Position = (Position-1)<<1;
              //AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: shutter %d -> %d"), ShutterIndex+1, Position);
              if (102 == Position) {
                XdrvMailbox.payload = XdrvMailbox.index;
                CmndShutterToggle();
              } else {
                CmndShutterPosition();
              }
              if (Settings.shutter_button[ButtonIndex] & ((0x01<<26)<<PosPressIndex)) {
                // MQTT broadcast to grouptopic
                char sCommand[CMDSZ];
                char sTopic[TOPSZ];
                for (uint32_t i = 0; i < MAX_SHUTTERS; i++) {
                  if ((i==ShutterIndex) || (Settings.shutter_button[ButtonIndex] & (0x01<<30))) {
                    snprintf_P(sCommand, sizeof(sCommand),PSTR("ShutterPosition%d"), i+1);
                    GetGroupTopic_P(sTopic, sCommand, SET_MQTT_GRP_TOPIC);
                    Response_P("%d", Position);
                    MqttPublish(sTopic, false);
                  }
                } // for (uint32_t)
              } // if (Settings.shutter)
            } // ende else
          } // if (Position)
        } // end else
      } // if   if (Settings.shutter_startrelay[ShutterIndex]
    }
    Response_P(PSTR("{"));
    ResponseAppend_P(JSON_SHUTTER_BUTTON, ShutterIndex+1, (ButtonState <= SHT_PRESSED_EXT_HOLD) ? (ButtonIndex+1) : 0, PressIndex);
    ResponseJsonEnd();
    MqttPublishPrefixTopicRulesProcess_P(RESULT_OR_STAT, PSTR(D_PRFX_SHUTTER));
  }
}

void ShutterSetPosition(uint32_t device, uint32_t Position)
{
  char sValue[32];                   // Command and number parameter
  snprintf_P(sValue, sizeof(sValue), PSTR(D_PRFX_SHUTTER D_CMND_SHUTTER_POSITION "%d %d"), device, Position);
  ExecuteCommand(sValue, SRC_SHUTTER);
}

void ShutterToggle(bool Dir)
{
  AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Payload toggle: %d, i %d, Dir %d"), XdrvMailbox.payload, XdrvMailbox.index, Dir);
  if ((1 == XdrvMailbox.index) && (XdrvMailbox.payload != -99)) {
    XdrvMailbox.index = XdrvMailbox.payload;
  }
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    uint32_t Index = XdrvMailbox.index-1;
    if (Dir) {
      XdrvMailbox.payload = (Shutter[Index].LastDirection > 0) ? 0 : 100;
    }
    else {
      XdrvMailbox.payload = (50 < ShutterRealToPercentPosition(Shutter[Index].RealPosition, Index)) ? 0 : 100;
    }
    XdrvMailbox.data_len = 0;
    last_source = SRC_WEBGUI;
    CmndShutterPosition();
  }
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndShutterOpen(void)
{
  //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Payload open: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  if ((1 == XdrvMailbox.index) && (XdrvMailbox.payload != -99)) {
    XdrvMailbox.index = XdrvMailbox.payload;
  }
  XdrvMailbox.payload = 100;
  last_source = SRC_WEBGUI;
  CmndShutterPosition();
}

void CmndShutterStopOpen(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    uint32_t Index = XdrvMailbox.index-1;
    if (Shutter[Index].Direction) {
      CmndShutterStop();
    } else {
      CmndShutterOpen();
    }
  }
}

void CmndShutterClose(void)
{
  //AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Payload close: %d, i %d"), XdrvMailbox.payload, XdrvMailbox.index);
  if ((1 == XdrvMailbox.index) && (XdrvMailbox.payload != -99)) {
    XdrvMailbox.index = XdrvMailbox.payload;
  }
  XdrvMailbox.payload = 0;
  XdrvMailbox.data_len = 0;
  last_source = SRC_WEBGUI;
  CmndShutterPosition();
}

void CmndShutterStopClose(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    uint32_t Index = XdrvMailbox.index-1;
    if (Shutter[Index].Direction) {
      CmndShutterStop();
    } else {
      CmndShutterClose();
    }
  }
}

void CmndShutterToggle(void)
{
  ShutterToggle(false);
}

void CmndShutterToggleDir(void)
{
  ShutterToggle(true);
}

void CmndShutterStopToggle(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    uint32_t Index = XdrvMailbox.index-1;
    if (Shutter[Index].Direction) {
      CmndShutterStop();
    } else {
      CmndShutterToggle();
    }
  }
}

void CmndShutterStopToggleDir(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    uint32_t Index = XdrvMailbox.index-1;
    if (Shutter[Index].Direction) {
      CmndShutterStop();
    } else {
      CmndShutterToggleDir();
    }
  }
}

void CmndShutterStop(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (!(Settings.shutter_options[XdrvMailbox.index-1] & 2)) {
      if ((1 == XdrvMailbox.index) && (XdrvMailbox.payload != -99)) {
        XdrvMailbox.index = XdrvMailbox.payload;
      }
      uint32_t i = XdrvMailbox.index -1;
      if (Shutter[i].Direction != 0) {

        AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT: Stop moving %d: dir: %d"), XdrvMailbox.index, Shutter[i].Direction);
        // set stop position 10 steps ahead (0.5sec to allow normal stop)

        //ToDo: Replace with function
        int32_t TempRealPos = Shutter[i].StartPosition + ( (Shutter[i].Time+10) * (Shutter[i].Direction > 0 ? 100 : -Shutter[i].CloseVelocity));
        XdrvMailbox.payload = ShutterRealToPercentPosition(TempRealPos, i);
        //XdrvMailbox.payload = Settings.shuttercoeff[2][i] * 5 > TempRealPos ? TempRealPos / Settings.shuttercoeff[2][i] : (TempRealPos - Settings.shuttercoeff[0,i]) / Settings.shuttercoeff[1][i];
        last_source = SRC_WEBGUI;
        CmndShutterPosition();
      } else {
        if (XdrvMailbox.command)
          ResponseCmndDone();
      }
    } else {
      if (XdrvMailbox.command)
        ResponseCmndIdxChar("Locked");
    }
  }
}

void CmndShutterPosition(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (!(Settings.shutter_options[XdrvMailbox.index-1] & 2)) {
      uint32_t Index = XdrvMailbox.index-1;
      //limit the payload
      AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Pos. in: payload %s (%d), payload %d, idx %d, src %d"), XdrvMailbox.data , XdrvMailbox.data_len, XdrvMailbox.payload , XdrvMailbox.index, last_source );

      // value 0 with data_len > 0 can mean Open
      // special handling fo UP,DOWN,TOGGLE,STOP command comming with payload -99
      if ((XdrvMailbox.data_len > 1) && (XdrvMailbox.payload <= 0)) {
        //UpperCase(XdrvMailbox.data, XdrvMailbox.data);
        if (!strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_UP) || !strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_OPEN) || ((Shutter[Index].Direction==0) && !strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_STOPOPEN))) {
          CmndShutterOpen();
          return;
        }
        if (!strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_DOWN) || !strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_CLOSE) || ((Shutter[Index].Direction==0) && !strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_STOPCLOSE))) {
          CmndShutterClose();
          return;
        }
        if (!strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_TOGGLE)) {
          CmndShutterToggle();
          return;
        }
        if (!strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_TOGGLEDIR)) {
          CmndShutterToggleDir();
          return;
        }
        if (!strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_STOP) || ((Shutter[Index].Direction) && (!strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_STOPOPEN) || !strcasecmp(XdrvMailbox.data,D_CMND_SHUTTER_STOPCLOSE)))) {
          XdrvMailbox.payload = -99;
          CmndShutterStop();
          return;
        }
      }

      int8_t TargetPosPercent = (XdrvMailbox.payload < 0) ? (XdrvMailbox.payload == -99 ? ShutterRealToPercentPosition(Shutter[Index].RealPosition, Index) : 0) : ((XdrvMailbox.payload > 100) ? 100 : XdrvMailbox.payload);
      // webgui still send also on inverted shutter the native position.
      TargetPosPercent = ((Settings.shutter_options[Index] & 1) && (SRC_WEBGUI != last_source)) ? 100 - TargetPosPercent : TargetPosPercent;
      if (XdrvMailbox.payload != -99) {
        //TargetPosPercent = (Settings.shutter_options[Index] & 1) ? 100 - TargetPosPercent : TargetPosPercent;
        Shutter[Index].TargetPosition = ShutterPercentToRealPosition(TargetPosPercent, Index);
        //Shutter[i].Accelerator[Index] = ShutterGlobal.OpenVelocityMax / ((Shutter[i].MotorDelay[Index] > 0) ? Shutter[i].MotorDelay[Index] : 1);
        //Shutter[i].TargetPosition[Index] = XdrvMailbox.payload < 5 ?  Settings.shuttercoeff[2][Index] * XdrvMailbox.payload : Settings.shuttercoeff[1][Index] * XdrvMailbox.payload + Settings.shuttercoeff[0,Index];
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: lastsource %d:, real %d, target %d, payload %d"), last_source, Shutter[Index].RealPosition ,Shutter[Index].TargetPosition,TargetPosPercent);
      }
      if ( (TargetPosPercent >= 0) && (TargetPosPercent <= 100) && abs(Shutter[Index].TargetPosition - Shutter[Index].RealPosition ) / Shutter[Index].CloseVelocity > 2) {
        if (Settings.shutter_options[Index] & 4) {
          if (0   == TargetPosPercent) Shutter[Index].TargetPosition -= 1 * RESOLUTION * STEPS_PER_SECOND;
          if (100 == TargetPosPercent) Shutter[Index].TargetPosition += 1 * RESOLUTION * STEPS_PER_SECOND;
        }
        int8_t NewShutterDirection = Shutter[Index].RealPosition < Shutter[Index].TargetPosition ? 1 : -1;
        if (Shutter[Index].Direction == -NewShutterDirection) {
          ShutterPowerOff(Index);
        }
        if (Shutter[Index].Direction != NewShutterDirection) {
          ShutterStartInit(Index, NewShutterDirection, Shutter[Index].TargetPosition);
          switch (ShutterGlobal.PositionMode) {
            case SHT_COUNTER:
            case SHT_PWM_TIME:
            case SHT_PWM_VALUE:
            case SHT_TIME_UP_DOWN:
              if (!ShutterGlobal.SkipRelayChange) {
                // Code for shutters with circuit safe configuration, switch the direction Relay
                ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index] +1, NewShutterDirection == 1 ? 0 : 1, SRC_SHUTTER);
                // power on
                ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index], 1, SRC_SHUTTER);
              }
              if (ShutterGlobal.PositionMode != SHT_TIME_UP_DOWN) ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index]+2, 1, SRC_SHUTTER);
            break;
            case SHT_TIME:
              if (!ShutterGlobal.SkipRelayChange) {
                if ( (power >> (Settings.shutter_startrelay[Index] -1)) & 3 > 0 ) {
                  ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index] + (NewShutterDirection == 1 ? 1 : 0), Shutter[Index].SwitchMode == SHT_SWITCH ? 0 : 1, SRC_SHUTTER);
                }
                ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index] + (NewShutterDirection == 1 ? 0 : 1), 1, SRC_SHUTTER);
              }
            break;
            case SHT_TIME_GARAGE:
              if (!ShutterGlobal.SkipRelayChange) {
                if (NewShutterDirection == Shutter[Index].LastDirection) {
                  AddLog_P2(LOG_LEVEL_INFO, PSTR("SHT: Garage not move in this direction: %d"), Shutter[Index].SwitchMode == SHT_PULSE);
                  for (uint8_t k=0 ; k <= (uint8_t)(Shutter[Index].SwitchMode == SHT_PULSE) ; k++) {
                    ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index], 1, SRC_SHUTTER);
                    delay(500);
                    ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index], 0, SRC_SHUTTER);
                    delay(500);
                  }
                  // reset shutter time to avoid 2 seconds above count as runtime
                  Shutter[Index].Time = 0;
                } // if (NewShutterDirection == Shutter[i].LastDirection[Index])
                ExecuteCommandPowerShutter(Settings.shutter_startrelay[Index], 1, SRC_SHUTTER);
              } // if (!ShutterGlobal.SkipRelayChange)
            break;
          } // switch (ShutterGlobal.PositionMode)
          ShutterGlobal.RelayCurrentMask = 0;
        } // if (Shutter[i].Direction[Index] != NewShutterDirection)
      } else {
        TargetPosPercent = ShutterRealToPercentPosition(Shutter[Index].RealPosition, Index);
        ShutterReportPosition(true, Index);
      }
      XdrvMailbox.index = Index +1;  // Fix random index for ShutterClose
      if (XdrvMailbox.command)
        ResponseCmndIdxNumber((Settings.shutter_options[Index] & 1) ? 100 - TargetPosPercent : TargetPosPercent);
    } else {
      ShutterReportPosition(true, MAX_SHUTTERS);
      if (XdrvMailbox.command)
        ResponseCmndIdxChar("Locked");
    }
  }
}

void CmndShutterStopPosition(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    uint32_t Index = XdrvMailbox.index-1;
    if (Shutter[Index].Direction) {
      XdrvMailbox.payload = -99;
      CmndShutterStop();
    } else {
      CmndShutterPosition();
    }
  }
}
void CmndShutterOpenTime(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (XdrvMailbox.data_len > 0) {
      Settings.shutter_opentime[XdrvMailbox.index -1] = (uint16_t)(10 * CharToFloat(XdrvMailbox.data));
      ShutterInit();
    }
    char TimeChr[10];
    dtostrfd((float)(Settings.shutter_opentime[XdrvMailbox.index -1]) / 10, 1, TimeChr);
    ResponseCmndIdxChar(TimeChr);
  }
}

void CmndShutterCloseTime(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (XdrvMailbox.data_len > 0) {
      Settings.shutter_closetime[XdrvMailbox.index -1] = (uint16_t)(10 * CharToFloat(XdrvMailbox.data));
      ShutterInit();
    }
    char TimeChr[10];
    dtostrfd((float)(Settings.shutter_closetime[XdrvMailbox.index -1]) / 10, 1, TimeChr);
    ResponseCmndIdxChar(TimeChr);
  }
}

void CmndShutterMotorDelay(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (XdrvMailbox.data_len > 0) {
      Settings.shutter_motordelay[XdrvMailbox.index -1] = (uint16_t)(STEPS_PER_SECOND * CharToFloat(XdrvMailbox.data));
      ShutterInit();
    }
    char TimeChr[10];
    dtostrfd((float)(Settings.shutter_motordelay[XdrvMailbox.index -1]) / STEPS_PER_SECOND, 2, TimeChr);
    ResponseCmndIdxChar(TimeChr);
  }
}

void CmndShutterMode(void)
{
  if ((XdrvMailbox.payload >= 0) && (XdrvMailbox.payload <= MAX_MODES)) {
    ShutterGlobal.PositionMode =  XdrvMailbox.payload;
    Settings.shutter_mode =  XdrvMailbox.payload;
    ShutterInit();
  }
  ResponseCmndNumber(ShutterGlobal.PositionMode);
}

void CmndShutterRelay(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= MAX_SHUTTERS)) {
    if ((XdrvMailbox.payload >= 0) && (XdrvMailbox.payload <= 64)) {
      Settings.shutter_startrelay[XdrvMailbox.index -1] = XdrvMailbox.payload;
      if (XdrvMailbox.payload > 0) {
        ShutterGlobal.RelayShutterMask |= 3 << (XdrvMailbox.payload - 1);
      } else {
        ShutterGlobal.RelayShutterMask ^= 3 << (Settings.shutter_startrelay[XdrvMailbox.index -1] - 1);
      }
      Settings.shutter_startrelay[XdrvMailbox.index -1] = XdrvMailbox.payload;
      ShutterInit();
      // if payload is 0 to disable the relay there must be a reboot. Otherwhise does not work
    }
    ResponseCmndIdxNumber(Settings.shutter_startrelay[XdrvMailbox.index -1]);
  }
}

void CmndShutterButton(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= MAX_SHUTTERS)) {
    uint32_t Setting = 0;
    // (Setting>>31)&(0x01) : enabled
    // (Setting>>30)&(0x01) : mqtt broadcast to all index
    // (Setting>>29)&(0x01) : mqtt broadcast hold
    // (Setting>>28)&(0x01) : mqtt broadcast tripple press
    // (Setting>>27)&(0x01) : mqtt broadcast double press
    // (Setting>>26)&(0x01) : mqtt broadcast single press
    // (Setting>>20)&(0x3f) : shutter_position hold; 0 disabled, 1..101 == 0..100%, 102 == toggle
    // (Setting>>14)&(0x3f) : shutter_position tripple press 0 disabled, 1..101 == 0..100%, 102 == toggle
    // (Setting>> 8)&(0x3f) : shutter_position double press 0 disabled, 1..101 == 0..100%, 102 == toggle
    // (Setting>> 2)&(0x3f) : shutter_position single press 0 disabled, 1..101 == 0..100%, 102 == toggle
    // (Setting>> 0)&(0x03) : ShutterIndex
    if (XdrvMailbox.data_len > 0) {
        uint32_t i = 0;
        uint32_t ButtonIndex = 0;
        bool Done = false;
        bool IsShortCommand = false;
        char *pString;

        char DataCopy[strlen(XdrvMailbox.data) +1];
        strncpy(DataCopy, XdrvMailbox.data, sizeof(DataCopy));  // Duplicate data as strtok_r will modify it.
        // Loop through the data string, splitting on ' ' seperators.
        for (char *s = strtok_r(DataCopy, " ", &pString); s && i < (1+4+4+1); s = strtok_r(nullptr, " ", &pString), i++) {
          int Field;
          switch (s[0]) {
            case '-':
              Field = -1;
              break;
            case 't':
              Field = 102;
              break;
            default:
             Field = atoi(s);
             break;
          }
          switch (i) {
            case 0:
              if ((Field >= -1) && (Field<=4)) {
                ButtonIndex = (Field<=0)?(-1):Field;
                Done = (ButtonIndex==-1);
              } else
                Done = true;
            break;
            case 1:
              if (!strcmp_P(s, PSTR("up"))) {
                Setting |= (((100>>1)+1)<<2) | (((50>>1)+1)<<8) | (((75>>1)+1)<<14) | (((100>>1)+1)<<20);
                IsShortCommand = true;
                break;
              } else if (!strcmp_P(s, PSTR("down"))) {
                Setting |= (((0>>1)+1)<<2) | (((50>>1)+1)<<8) | (((25>>1)+1)<<14) | (((0>>1)+1)<<20);
                IsShortCommand = true;
                break;
              } else if (!strcmp_P(s, PSTR("updown"))) {
                Setting |= (((100>>1)+1)<<2) | (((0>>1)+1)<<8) | (((50>>1)+1)<<14);
                IsShortCommand = true;
                break;
              } else if (!strcmp_P(s, PSTR("toggle"))) {
                Setting |= (((102>>1)+1)<<2) | (((50>>1)+1)<<8);
                IsShortCommand = true;
                break;
              }
            case 2:
              if (IsShortCommand) {
                if ((Field==1) && (Setting & (0x3F<<(2+6*3))))
                  // if short command up or down (hold press position set) then also enable MQTT broadcast
                  Setting |= (0x3<<29);
                Done = true;
                break;
              }
            case 3:
            case 4:
              if ((Field >= -1) && (Field<=102))
                Setting |= (((Field>>1)+1)<<(i*6 + (2-6)));
            break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
              if (Field==1)
                Setting |= (1<<(i + (26-5)));
            break;
          }
          if (Done) break;
        }

        if (ButtonIndex) {
          if (ButtonIndex==-1) {
            // remove all buttons for this shutter
            for (uint32_t i=0 ; i < MAX_KEYS ; i++)
              if ((Settings.shutter_button[i]&0x3) == (XdrvMailbox.index-1))
                Settings.shutter_button[i] = 0;
          } else {
            if (Setting) {
              // anything was set
              Setting |= (1<<31);
              Setting |= (XdrvMailbox.index-1) & 0x3;
            }
            Settings.shutter_button[ButtonIndex-1] = Setting;
          }
        }
      }
      char SettingChr[30*MAX_KEYS] = "-", *setting_chr_ptr = SettingChr;
      for (uint32_t i=0 ; i < MAX_KEYS ; i++) {
        Setting = Settings.shutter_button[i];
        if ((Setting&(1<<31)) && ((Setting&0x3) == (XdrvMailbox.index-1))) {
          if (*setting_chr_ptr == 0)
            setting_chr_ptr += sprintf_P(setting_chr_ptr, PSTR("|"));
          setting_chr_ptr += snprintf_P(setting_chr_ptr, 2, PSTR("%d"), i+1);

          for (uint32_t j=0 ; j < 4 ; j++) {
            int8_t Position = (((Setting>> (2+6*j))&(0x3f))-1)<<1;
            if (0 <= Position)
              if (102 == Position) {
                setting_chr_ptr += sprintf_P(setting_chr_ptr, PSTR(" t"));
              } else {
                setting_chr_ptr += snprintf_P(setting_chr_ptr, 5, PSTR(" %d"), Position);
              }
            else
              setting_chr_ptr += sprintf_P(setting_chr_ptr, PSTR(" -"));
          }
          for (uint32_t j=0 ; j < 5 ; j++) {
            bool Mqtt = ((Setting>>(26+j))&(0x01)!=0);
            if (Mqtt)
              setting_chr_ptr += sprintf_P(setting_chr_ptr, PSTR(" 1"));
            else
              setting_chr_ptr += sprintf_P(setting_chr_ptr, PSTR(" -"));
          }
        }
      }
      ResponseCmndIdxChar(SettingChr);
   }
}

void CmndShutterSetHalfway(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if ((XdrvMailbox.payload >= 0) && (XdrvMailbox.payload <= 100)) {
      Settings.shutter_set50percent[XdrvMailbox.index -1] = (Settings.shutter_options[XdrvMailbox.index -1] & 1) ? 100 - XdrvMailbox.payload : XdrvMailbox.payload;
      ShutterInit();
    }
  ResponseCmndIdxNumber((Settings.shutter_options[XdrvMailbox.index -1] & 1) ? 100 - Settings.shutter_set50percent[XdrvMailbox.index -1] : Settings.shutter_set50percent[XdrvMailbox.index -1]);
  }
}

void CmndShutterFrequency(void)
{
  if ((XdrvMailbox.payload > 0) && (XdrvMailbox.payload <= 20000)) {
    ShutterGlobal.OpenVelocityMax =  XdrvMailbox.payload;
    if (shutters_present < 4) {
      Settings.shuttercoeff[4][3] = ShutterGlobal.OpenVelocityMax;
    }
    ShutterInit();
  }
  ResponseCmndNumber(ShutterGlobal.OpenVelocityMax);
}

void CmndShutterSetClose(void)
{
  // ??? die folgenden 6 Zeilen können in neue void "CmndShutterSetOpenClose(int32_t OpenMax, int_8 Position)", die in den nächsten 2 Functions aufgerufen wird
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    Shutter[XdrvMailbox.index -1].RealPosition = 0;
    ShutterStartInit(XdrvMailbox.index -1, 0, 0);
    Settings.shutter_position[XdrvMailbox.index -1] = 0;
    ResponseCmndIdxChar(D_CONFIGURATION_RESET);
  }
}

void CmndShutterSetOpen(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    Shutter[XdrvMailbox.index -1].RealPosition = Shutter[XdrvMailbox.index -1].OpenMax;
    ShutterStartInit(XdrvMailbox.index -1, 0, Shutter[XdrvMailbox.index -1].OpenMax);
    Settings.shutter_position[XdrvMailbox.index -1] = 100;
    ResponseCmndIdxChar(D_CONFIGURATION_RESET);
  }
}

void CmndShutterPwmRange(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (XdrvMailbox.data_len > 0) {
      uint8_t i = 0;
      char *pString;

      char DataCopy[strlen(XdrvMailbox.data) +1];
      strncpy(DataCopy, XdrvMailbox.data, sizeof(DataCopy));  // Duplicate data as strtok_r will modify it.
      // Loop through the data string, splitting on ' ' seperators.
      for (char *s = strtok_r(DataCopy, " ", &pString); s && i < 2; s = strtok_r(nullptr, " ", &pString), i++) {
        uint16_t Field = atoi(s);
        // The fields in a data string can only range from 1-30000.
        // and following value must be higher than previous one
        if ((Field <= 0) || (Field > 1023)) {
          break;
        }
        Settings.shutter_pwmrange[i][XdrvMailbox.index -1] = Field;
      }
      AddLog_P2(LOG_LEVEL_DEBUG, PSTR("SHT%d: Init1. pwmmin %d, pwmmax %d"), XdrvMailbox.index , Settings.shutter_pwmrange[0][XdrvMailbox.index -1], Settings.shutter_pwmrange[1][XdrvMailbox.index -1]);
      ShutterInit();
      ResponseCmndIdxChar(XdrvMailbox.data);
    } else {
      char SettingChr[30] = "0";
      snprintf_P(SettingChr, sizeof(SettingChr), PSTR("Shutter %d: min:%d max:%d"), XdrvMailbox.index, Settings.shutter_pwmrange[0][XdrvMailbox.index -1], Settings.shutter_pwmrange[1][XdrvMailbox.index -1]);
      ResponseCmndIdxChar(SettingChr);
    }
  }
}

void CmndShutterCalibration(void)
{
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (XdrvMailbox.data_len > 0) {
      uint8_t i = 0;
      char *pString;

      char DataCopy[strlen(XdrvMailbox.data) +1];
      strncpy(DataCopy, XdrvMailbox.data, sizeof(DataCopy));  // Duplicate data as strtok_r will modify it.
      // Loop through the data string, splitting on ' ' seperators.
      for (char *s = strtok_r(DataCopy, " ", &pString); s && i < 5; s = strtok_r(nullptr, " ", &pString), i++) {
        int Field = atoi(s);
        // The fields in a data string can only range from 1-30000.
        // and following value must be higher than previous one
        if ((Field <= 0) || (Field > 30000) || ( (i>0) && (Field <= Messwerte[i-1]) ) ) {
          break;
        }
        Messwerte[i] = Field;
      }
      for (i = 0; i < 5; i++) {
        Settings.shuttercoeff[i][XdrvMailbox.index -1] = SHT_DIV_ROUND((uint32_t)Messwerte[i] * 1000, Messwerte[4]);
        AddLog_P2(LOG_LEVEL_DEBUG, PSTR("Settings.shuttercoeff: %d, i: %d, value: %d, messwert %d"), i,XdrvMailbox.index -1,Settings.shuttercoeff[i][XdrvMailbox.index -1], Messwerte[i]);
      }
      ShutterInit();
      ResponseCmndIdxChar(XdrvMailbox.data);
    } else {
      char SettingChr[30] = "0";
      snprintf_P(SettingChr, sizeof(SettingChr), PSTR("%d %d %d %d %d"), Settings.shuttercoeff[0][XdrvMailbox.index -1], Settings.shuttercoeff[1][XdrvMailbox.index -1], Settings.shuttercoeff[2][XdrvMailbox.index -1], Settings.shuttercoeff[3][XdrvMailbox.index -1], Settings.shuttercoeff[4][XdrvMailbox.index -1]);
      ResponseCmndIdxChar(SettingChr);
    }
  }
}

void ShutterOptionsSetHelper(uint16_t Option){
  if ((XdrvMailbox.index > 0) && (XdrvMailbox.index <= shutters_present)) {
    if (XdrvMailbox.payload == 0) {
      Settings.shutter_options[XdrvMailbox.index -1] &= ~(Option);
    } else if (XdrvMailbox.payload == 1) {
      Settings.shutter_options[XdrvMailbox.index -1] |= (Option);
    }
    ResponseCmndIdxNumber((Settings.shutter_options[XdrvMailbox.index -1] & Option) ? 1 : 0);
  }
}

void CmndShutterInvert(void) {
  ShutterOptionsSetHelper(1);
}

void CmndShutterLock(void) {
  ShutterOptionsSetHelper(2);
}

void CmndShutterEnableEndStopTime(void) {
  ShutterOptionsSetHelper(4);
}

void CmndShutterInvertWebButtons(void) {
  ShutterOptionsSetHelper(8);
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv27(uint8_t function)
{
  bool result = false;

  if (Settings.flag3.shutter_mode) {  // SetOption80 - Enable shutter support
    switch (function) {
      case FUNC_PRE_INIT:
        ShutterInit();
        break;
      case FUNC_EVERY_50_MSECOND:
        ShutterUpdatePosition();
        break;
      case FUNC_EVERY_SECOND:
      //case FUNC_EVERY_250_MSECOND:
        ShutterReportPosition(false, MAX_SHUTTERS);
        break;

      case FUNC_COMMAND:
        result = DecodeCommand(kShutterCommands, ShutterCommand);
        break;
      case FUNC_JSON_APPEND:
        for (uint8_t i = 0; i < shutters_present; i++) {
          uint8_t Position = (Settings.shutter_options[i] & 1) ? 100 - Settings.shutter_position[i] : Settings.shutter_position[i];
          uint8_t Target   = (Settings.shutter_options[i] & 1) ? 100 - ShutterRealToPercentPosition(Shutter[i].TargetPosition, i) : ShutterRealToPercentPosition(Shutter[i].TargetPosition, i);

          ResponseAppend_P(",");
          ResponseAppend_P(JSON_SHUTTER_POS, i+1, Position, Shutter[i].Direction,Target);
#ifdef USE_DOMOTICZ
          if ((0 == tele_period) && (0 == i)) {
             DomoticzSensor(DZ_SHUTTER, Position);
          }
#endif  // USE_DOMOTICZ
        }
        break;
      case FUNC_SET_POWER:
        char sTemp[10];
        // extract the number of the relay that was switched and save for later in Update Position.
        ShutterGlobal.RelayCurrentMask = XdrvMailbox.index ^ ShutterGlobal.RelayOldMask;
        AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Switched relay: %d by %s"), ShutterGlobal.RelayCurrentMask,GetTextIndexed(sTemp, sizeof(sTemp), last_source, kCommandSource));
        ShutterRelayChanged();
        ShutterGlobal.RelayOldMask = XdrvMailbox.index;
      break;
      case FUNC_SET_DEVICE_POWER:
        if (ShutterGlobal.SkipRelayChange ) {
          uint8_t i;
          for (i = 0; i < devices_present; i++) {
            if (ShutterGlobal.RelayCurrentMask &1) {
              break;
            }
            ShutterGlobal.RelayCurrentMask >>= 1;
          }
          //AddLog_P2(LOG_LEVEL_ERROR, PSTR("SHT: skip relay change: %d"),i+1);
          result = true;
          ShutterGlobal.SkipRelayChange = 0;
          AddLog_P2(LOG_LEVEL_DEBUG_MORE, PSTR("SHT: Skipping switch off relay %d"),i);
          ExecuteCommandPowerShutter(i+1, 0, SRC_SHUTTER);
        }
      break;
      case FUNC_BUTTON_PRESSED:
        if (Settings.shutter_button[XdrvMailbox.index] & (1<<31)) {
          ShutterButtonHandler();
          result = true;
        }
      break;
    }
  }
  return result;
}

#endif //USE_SHUTTER
