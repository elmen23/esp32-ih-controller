/**
* ╔══════════════════════════════════════════════════════════╗
* ║        INDUCTION HEATER CONTROLLER  v3.0 PRO             ║
* ║   STM32F411CEU6 Black Pill | LCD 20x4 I2C               ║
* ║   Rotary Encoder | HW Timer TIM1 (CH1+CH1N+DeadTime)    ║
* ║   Auto Resonance Tracking | RMS Current | PID | Calib   ║
* ╠══════════════════════════════════════════════════════════╣
*  Pinout:
*    PWM CH1  → PA8  | PWM CH1N → PA9
*    Encoder  → PB3, PB4, PB5
*    Temp     → PA0  | Curr     → PA1
*    ZCD/Phase→ PB12 (Input from Comparator for Resonance)
*    Fan      → PB0  | HW Fault → PB1 (active LOW)
*    LCD      → PB7(SDA), PB6(SCL)
* ╚══════════════════════════════════════════════════════════╝
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <HardwareTimer.h>
#include <stdarg.h>
#include <math.h>

// ─── Pin Definitions ─────────────────────────────────────────────────────────
#define ENC_CLK     PB3
#define ENC_DT      PB4
#define ENC_SW      PB5
#define PWM_CH1     PA8
#define PWM_CH1N    PA9
#define TEMP_PIN    PA0
#define CURR_PIN    PA1
#define ZCD_PIN     PB12   // Zero-Crossing Detector for Resonance Tracking
#define FAN_PIN     PB0
#define FAULT_PIN   PB1

// ─── Limits & Constants ──────────────────────────────────────────────────────
#define FREQ_MIN         10000UL
#define FREQ_MAX        100000UL
#define FREQ_STEP         1000UL
#define DUTY_MIN              10
#define DUTY_MAX              95
#define TEMP_LIM_MIN          30
#define TEMP_LIM_MAX         120
#define CURR_LIM_MIN           5
#define CURR_LIM_MAX          50
#define DEAD_TIME_MIN          1   // µs
#define DEAD_TIME_MAX         10   // µs
#define SOFT_START_MIN         0
#define SOFT_START_MAX        30
#define FAN_COOLDOWN_MAX      60   // seconds
#define PID_KP_MIN  0.0f
#define PID_KP_MAX  5.0f
#define PID_KI_MIN  0.0f
#define PID_KI_MAX  2.0f
#define PID_KD_MIN  0.0f
#define PID_KD_MAX  5.0f

#define BTN_DEBOUNCE_MS       50
#define BTN_LONG_PRESS_MS   1500
#define LCD_REFRESH_MS       250
#define SENSOR_MS            100
#define SOFTSTART_TICK_MS    100
#define RMS_WIN_SIZE         16

// ─── EEPROM Layout (Expanded) ────────────────────────────────────────────────
#define ADDR_MAGIC             0
#define ADDR_FREQ              4
#define ADDR_DUTY              8
#define ADDR_TEMP_LIM         12
#define ADDR_CURR_LIM         16
#define ADDR_DEAD_TIME        20
#define ADDR_SOFT_START       24
#define ADDR_FAN_COOL         28
#define ADDR_PID_KP           32
#define ADDR_PID_KI           36
#define ADDR_PID_KD           40
#define ADDR_PID_TARGET       44
#define ADDR_CAL_TEMP_OFF     48
#define ADDR_CAL_TEMP_SC      52
#define ADDR_CAL_CURR_OFF     56
#define ADDR_CAL_CURR_SC      60
#define EEPROM_MAGIC_VAL    0xA5

// ─── State Machine ───────────────────────────────────────────────────────────
enum AppState : uint8_t {
    ST_MAIN = 0, ST_MENU, ST_EDIT, ST_SETTINGS, ST_SET_EDIT,
    ST_CONFIRM, ST_FAULT, ST_DIAG, ST_CALIB_TEMP, ST_CALIB_CURR
};

// ─── Data Structures ─────────────────────────────────────────────────────────
struct Settings {
    uint32_t freq; uint8_t duty; uint8_t tempLimit; uint8_t currLimit;
    uint8_t deadTime; uint8_t softStart; uint8_t fanCooldown;
    float pidKp, pidKi, pidKd; float pidTarget; bool pidEnable;
    bool autoTrackEnable;
};

struct Calibration { float tempOff, tempSc, currOff, currSc; };
struct Diagnostics {
    uint32_t lastFaultCode; uint32_t runTimeMin;    float maxTemp, maxCurr; char lastFaultMsg[21];
};

const Settings DEFAULT_CFG = {
    25000, 50, 70, 30, 2, 5, 15, 1.2f, 0.05f, 0.8f, 25.0f, false, false
};
const Calibration DEFAULT_CAL = { 0.0f, 1.0f, 0.0f, 1.0f };

// ─── Global Objects ──────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 20, 4);
HardwareTimer     pwmTimer(TIM1);

Settings  cfg = DEFAULT_CFG;
Calibration cal = DEFAULT_CAL;
Diagnostics diag = {0, 0, 0.0f, 0.0f, "None"};
AppState  appState   = ST_MAIN;
uint8_t   menuIdx    = 0;
uint8_t   settingIdx = 0;
bool      heaterOn   = false;
bool      faultActive = false;
char      confirmMsg[21] = "";
uint32_t  confirmTimer = 0;

// ─── ISR & Volatiles ─────────────────────────────────────────────────────────
volatile int8_t encDelta = 0;
volatile bool   faultRaw = false;
volatile uint32_t zcdLast = 0, zcdPeriod = 0;

// ─── Button State ────────────────────────────────────────────────────────────
bool     btnRaw=HIGH, btnStable=HIGH, longFired=false;
uint32_t btnChangeMs=0, btnPressMs=0;

// ─── Sensors & RMS ───────────────────────────────────────────────────────────
float temperature = 0.0f, current = 0.0f, currentRMS = 0.0f;
float currBuffer[RMS_WIN_SIZE] = {0}; uint8_t currIdx = 0;

// ─── Soft Start & PID ────────────────────────────────────────────────────────
uint8_t  runDuty = 0;
bool     softRunning = false;
uint32_t softTickMs = 0;
float    pidInt = 0.0f, pidPrevErr = 0.0f;
uint32_t fanStopTime = 0;
bool     fanCooldownActive = false;

// ─── Timers ──────────────────────────────────────────────────────────────────
uint32_t lcdTimer=0, sensorTimer=0;
bool     dirty = true;

const char* mainMenu[] = { "START/STOP", "FREQUENCY", "POWER", "SETTINGS", "DIAGNOSTICS" };
const char* settMenu[] = {  "TEMP LIMIT","CURR LIMIT","SOFT START","DEAD TIME","FAN COOL-DN",
  "PID KP","PID KI","PID KD","PID TARGET","PID ON/OFF","TRACKING ON/OFF",
  "CALIBRATE","SAVE EEPROM","FACTORY RST","< BACK"
};
#define MAIN_MENU_LEN  5
#define SETT_MENU_LEN  15

// ════════════════════════════════════════════════════════════════════════════
//  ISR HANDLERS
// ════════════════════════════════════════════════════════════════════════════
void onEncoderCLK() {
    bool clk = (bool)digitalRead(ENC_CLK);
    bool dt  = (bool)digitalRead(ENC_DT);
    if (clk != dt) encDelta++; else encDelta--;
}
void onHWFault() { faultRaw = true; }
void onZCD() {
    uint32_t now = micros();
    if (zcdLast > 0) zcdPeriod = now - zcdLast;
    zcdLast = now;
}

// ════════════════════════════════════════════════════════════════════════════
//  LCD HELPER
// ════════════════════════════════════════════════════════════════════════════
void lcdLine(uint8_t row, const char* fmt, ...) {
    char buf[22]; va_list args;
    va_start(args, fmt); vsnprintf(buf, sizeof(buf), fmt, args); va_end(args);
    size_t len = strlen(buf);
    while (len < 20) buf[len++] = ' ';
    buf[20] = '\0';
    lcd.setCursor(0, row); lcd.print(buf);
}

// ════════════════════════════════════════════════════════════════════════════
//  PWM & DEAD TIME (Direct Register for Precision)
// ════════════════════════════════════════════════════════════════════════════
void applyDeadTime() {
    // TIM1 runs at 100MHz on F411. 1 tick = 10ns.
    uint32_t ns = cfg.deadTime * 1000UL;
    uint8_t dtg;
    if (ns <= 2550) dtg = (ns / 10);
    else if (ns <= 5100) dtg = 0xC0 | ((ns - 2560) / 128);
    else dtg = 0xE0 | ((ns - 5120) / 512);
    if (dtg > 0xFF) dtg = 0xFF;
    TIM1->BDTR = (dtg) | TIM_BDTR_MOE | TIM_BDTR_OSSR | TIM_BDTR_OSSI;
}

void applyPWM() {
    if (!heaterOn || faultActive) {        pwmTimer.setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
        TIM1->CCER &= ~TIM_CCER_CC1NE; // Disable complementary
        return;
    }
    pinMode(PWM_CH1N, OUTPUT);
    TIM1->CCER |= TIM_CCER_CC1NE; // Enable complementary
    applyDeadTime();
    pwmTimer.setOverflow(cfg.freq, HERTZ_FORMAT);
    uint8_t d = softRunning ? runDuty : cfg.duty;
    pwmTimer.setCaptureCompare(1, d, PERCENT_COMPARE_FORMAT);
}

// ════════════════════════════════════════════════════════════════════════════
//  SENSORS, RMS, CALIBRATION
// ════════════════════════════════════════════════════════════════════════════
void readSensors() {
    float rawT = analogRead(TEMP_PIN) * 3.3f / 4095.0f;
    temperature = (rawT + cal.tempOff) * cal.tempSc;

    float rawI = analogRead(CURR_PIN) * 3.3f / 4095.0f;
    float instI = (rawI + cal.currOff) * cal.currSc;
    if (instI < 0) instI = 0;
    currBuffer[currIdx++] = instI;
    if (currIdx >= RMS_WIN_SIZE) currIdx = 0;

    float sumSq = 0;
    for(uint8_t i=0; i<RMS_WIN_SIZE; i++) sumSq += currBuffer[i] * currBuffer[i];
    currentRMS = sqrtf(sumSq / RMS_WIN_SIZE);
    current = currentRMS;

    if (temperature > diag.maxTemp) diag.maxTemp = temperature;
    if (current > diag.maxCurr) diag.maxCurr = current;
}

// ════════════════════════════════════════════════════════════════════════════
//  PID CONTROLLER
// ════════════════════════════════════════════════════════════════════════════
void runPID() {
    if (!cfg.pidEnable || !heaterOn) return;
    float err = cfg.pidTarget - current;
    pidInt += err;
    if (pidInt > 100) pidInt = 100; if (pidInt < -100) pidInt = -100;
    float diff = err - pidPrevErr;
    pidPrevErr = err;
    float out = cfg.pidKp * err + cfg.pidKi * pidInt + cfg.pidKd * diff;
    int newDuty = constrain(cfg.duty + (int)(out * 0.5f), DUTY_MIN, DUTY_MAX);
    cfg.duty = newDuty;
}

// ════════════════════════════════════════════════════════════════════════════
//  AUTO RESONANCE TRACKING
// ════════════════════════════════════════════════════════════════════════════
void autoTrackTick() {
    if (!cfg.autoTrackEnable || !heaterOn) return;
    noInterrupts();
    uint32_t p = zcdPeriod;
    zcdPeriod = 0;
    interrupts();
    if (p == 0) return;
    uint32_t resFreq = 1000000UL / p;
    if (resFreq < FREQ_MIN || resFreq > FREQ_MAX) {
        if (appState != ST_FAULT) { faultActive = true; diag.lastFaultCode = 4; }
        return;
    }
    int32_t diff = (int32_t)resFreq - cfg.freq;
    if (abs(diff) > 500) {
        cfg.freq += (diff > 0) ? (FREQ_STEP/2) : -(FREQ_STEP/2);
        cfg.freq = constrain(cfg.freq, FREQ_MIN, FREQ_MAX);
        applyPWM();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  HEATER START / STOP & FAN COOLDOWN
// ════════════════════════════════════════════════════════════════════════════
void startHeater() {
    heaterOn = true; faultActive = false;
    if (cfg.softStart > 0) {
        runDuty = DUTY_MIN; softRunning = true; softTickMs = millis();
    } else { runDuty = cfg.duty; softRunning = false; }
    digitalWrite(FAN_PIN, HIGH); fanCooldownActive = false;
}
void stopHeater() {
    heaterOn = false; softRunning = false; runDuty = 0;
    applyPWM();
    fanCooldownActive = true;
    fanStopTime = millis() + (cfg.fanCooldown * 1000UL);
}

// ════════════════════════════════════════════════════════════════════════════
//  PROTECTION
// ════════════════════════════════════════════════════════════════════════════
void checkProtection() {
    if (faultRaw) {
        faultActive = true;
        if (appState != ST_FAULT) diag.lastFaultCode = 3;
    } else if (!faultActive) {
        if (temperature >= cfg.tempLimit) { faultActive = true; if (appState != ST_FAULT) diag.lastFaultCode = 2; }
        else if (current >= cfg.currLimit) { faultActive = true; if (appState != ST_FAULT) diag.lastFaultCode = 1; }
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ENCODER & BUTTON HANDLERS
// ════════════════════════════════════════════════════════════════════════════
void handleEncoder() {
    noInterrupts(); int8_t d = encDelta; encDelta = 0; interrupts();
    if (d == 0) return;
    int dir = (d > 0) ? 1 : -1;
    switch (appState) {
        case ST_MAIN: appState = ST_MENU; menuIdx = 0; break;
        case ST_MENU: menuIdx = constrain(menuIdx + dir, 0, MAIN_MENU_LEN - 1); break;
        case ST_EDIT:
            if (menuIdx == 1) cfg.freq = constrain(cfg.freq + dir*FREQ_STEP, FREQ_MIN, FREQ_MAX);
            else if (menuIdx == 2) cfg.duty = constrain(cfg.duty + dir, DUTY_MIN, DUTY_MAX);
            applyPWM(); break;
        case ST_SETTINGS: settingIdx = constrain(settingIdx + dir, 0, SETT_MENU_LEN - 1); break;
        case ST_SET_EDIT:
            switch (settingIdx) {
                case 0: cfg.tempLimit = constrain(cfg.tempLimit+dir, TEMP_LIM_MIN, TEMP_LIM_MAX); break;
                case 1: cfg.currLimit = constrain(cfg.currLimit+dir, CURR_LIM_MIN, CURR_LIM_MAX); break;
                case 2: cfg.softStart = constrain(cfg.softStart+dir, SOFT_START_MIN, SOFT_START_MAX); break;
                case 3: cfg.deadTime = constrain(cfg.deadTime+dir, DEAD_TIME_MIN, DEAD_TIME_MAX); break;
                case 4: cfg.fanCooldown = constrain(cfg.fanCooldown+dir, 0, FAN_COOLDOWN_MAX); break;
                case 5: cfg.pidKp = constrain(cfg.pidKp + dir*0.1f, PID_KP_MIN, PID_KP_MAX); break;
                case 6: cfg.pidKi = constrain(cfg.pidKi + dir*0.01f, PID_KI_MIN, PID_KI_MAX); break;
                case 7: cfg.pidKd = constrain(cfg.pidKd + dir*0.1f, PID_KD_MIN, PID_KD_MAX); break;
                case 8: cfg.pidTarget = constrain(cfg.pidTarget + dir*0.5f, 0.0f, 100.0f); break;
            } break;
        case ST_CALIB_TEMP: cal.tempOff += dir * 0.05f; break;
        case ST_CALIB_CURR: cal.currOff += dir * 0.05f; break;
    }
    dirty = true;
}

void handleButton() {
    bool raw = digitalRead(ENC_SW);
    uint32_t now = millis();
    if (raw != btnRaw) { btnChangeMs = now; btnRaw = raw; }
    if ((now - btnChangeMs) < BTN_DEBOUNCE_MS) return;
    if (raw != btnStable) {
        btnStable = raw;
        if (btnStable == LOW) { btnPressMs = now; longFired = false; }
        else if (!longFired) { onShortPress(); dirty = true; }
    }
    if (btnStable == LOW && !longFired && (now - btnPressMs) >= BTN_LONG_PRESS_MS) {
        longFired = true; onLongPress(); dirty = true;    }
}

void onShortPress() {
    switch (appState) {
        case ST_MAIN: appState = ST_MENU; menuIdx = 0; break;
        case ST_MENU:
            if (menuIdx == 0) heaterOn ? stopHeater() : startHeater();
            else if (menuIdx == 3) { appState = ST_SETTINGS; settingIdx = 0; }
            else if (menuIdx == 4) appState = ST_DIAG;
            else appState = ST_EDIT;
            break;
        case ST_EDIT: appState = ST_MENU; break;
        case ST_SETTINGS:
            switch (settingIdx) {
                case 0: case 1: case 2: case 3: case 4:
                case 5: case 6: case 7: case 8: appState = ST_SET_EDIT; break;
                case 9: cfg.pidEnable = !cfg.pidEnable; break;
                case 10: cfg.autoTrackEnable = !cfg.autoTrackEnable; break;
                case 11: appState = ST_CALIB_CURR; break;
                case 12: saveSettings(); showConfirm("  SAVED TO EEPROM!  "); break;
                case 13: factoryReset(); showConfirm("  FACTORY  RESET!   "); break;
                case 14: appState = ST_MENU; break;
            } break;
        case ST_SET_EDIT: appState = ST_SETTINGS; break;
        case ST_FAULT: faultActive = false; appState = ST_MAIN; break;
        case ST_DIAG: appState = ST_SETTINGS; break;
        case ST_CALIB_TEMP: case ST_CALIB_CURR: saveCalibration(); appState = ST_SETTINGS; break;
        case ST_CONFIRM: appState = ST_SETTINGS; break;
    }
}
void onLongPress() { stopHeater(); faultActive = false; appState = ST_MAIN; }
void showConfirm(const char* msg) { strncpy(confirmMsg, msg, 20); confirmMsg[20]='\0'; confirmTimer=millis(); appState=ST_CONFIRM; }

// ════════════════════════════════════════════════════════════════════════════
//  EEPROM
// ════════════════════════════════════════════════════════════════════════════
void saveSettings() {
    EEPROM.put(ADDR_MAGIC, EEPROM_MAGIC_VAL);
    EEPROM.put(ADDR_FREQ, cfg.freq); EEPROM.put(ADDR_DUTY, cfg.duty);
    EEPROM.put(ADDR_TEMP_LIM, cfg.tempLimit); EEPROM.put(ADDR_CURR_LIM, cfg.currLimit);
    EEPROM.put(ADDR_DEAD_TIME, cfg.deadTime); EEPROM.put(ADDR_SOFT_START, cfg.softStart);
    EEPROM.put(ADDR_FAN_COOL, cfg.fanCooldown);
    EEPROM.put(ADDR_PID_KP, cfg.pidKp); EEPROM.put(ADDR_PID_KI, cfg.pidKi);
    EEPROM.put(ADDR_PID_KD, cfg.pidKd);     EEPROM.put(ADDR_PID_TARGET, cfg.pidTarget);
}
void saveCalibration() {
    EEPROM.put(ADDR_CAL_TEMP_OFF, cal.tempOff); EEPROM.put(ADDR_CAL_TEMP_SC, cal.tempSc);
    EEPROM.put(ADDR_CAL_CURR_OFF, cal.currOff);     EEPROM.put(ADDR_CAL_CURR_SC, cal.currSc);    showConfirm("  CAL SAVED!  ");
}
void loadAll() {
    uint8_t m; EEPROM.get(ADDR_MAGIC, m);
    if (m != EEPROM_MAGIC_VAL) { cfg = DEFAULT_CFG; cal = DEFAULT_CAL; saveSettings(); return; }
    EEPROM.get(ADDR_FREQ, cfg.freq); EEPROM.get(ADDR_DUTY, cfg.duty);
    EEPROM.get(ADDR_TEMP_LIM, cfg.tempLimit); EEPROM.get(ADDR_CURR_LIM, cfg.currLimit);
    EEPROM.get(ADDR_DEAD_TIME, cfg.deadTime); EEPROM.get(ADDR_SOFT_START, cfg.softStart);
    EEPROM.get(ADDR_FAN_COOL, cfg.fanCooldown);
    EEPROM.get(ADDR_PID_KP, cfg.pidKp); EEPROM.get(ADDR_PID_KI, cfg.pidKi);
    EEPROM.get(ADDR_PID_KD, cfg.pidKd); EEPROM.get(ADDR_PID_TARGET, cfg.pidTarget);
    EEPROM.get(ADDR_CAL_TEMP_OFF, cal.tempOff); EEPROM.get(ADDR_CAL_TEMP_SC, cal.tempSc);
    EEPROM.get(ADDR_CAL_CURR_OFF, cal.currOff); EEPROM.get(ADDR_CAL_CURR_SC, cal.currSc);
    cfg.freq = constrain(cfg.freq, FREQ_MIN, FREQ_MAX);
    cfg.duty = constrain(cfg.duty, DUTY_MIN, DUTY_MAX);
}
void factoryReset() { cfg = DEFAULT_CFG; cal = DEFAULT_CAL; saveSettings(); }

// ════════════════════════════════════════════════════════════════════════════
//  DISPLAY FUNCTIONS (FULLY IMPLEMENTED)
// ════════════════════════════════════════════════════════════════════════════
void updateDisplay() {
    switch (appState) {
        case ST_MAIN:
            lcdLine(0,"  INDUCTION HEATER  ");
            lcdLine(1,"F:%3lukHz  Duty:%2d%%", cfg.freq/1000, cfg.duty);
            lcdLine(2,"T:%3d%cC     I:%3dARMS",(int)(temperature+0.5f),(char)0xDF,(int)(current+0.5f));
            lcdLine(3, heaterOn ? (softRunning?"SOFT START... %2d%% ":"[RUN] PID:%s TRK:%s"):
                (fanCooldownActive?"COOL DOWN...  ":"[STOP] Btn=Menu "),
                softRunning?runDuty:0, cfg.pidEnable?"ON":"OFF", cfg.autoTrackEnable?"ON":"OFF"); break;
        case ST_MENU:
            lcdLine(0,"======MAIN MENU=====");
            for(int i=0; i<3; i++){
                int idx = constrain(menuIdx-1+i, 0, MAIN_MENU_LEN-1);
                if(idx < 0 || idx >= MAIN_MENU_LEN) continue;
                lcdLine(i+1, "%c%s", (idx==menuIdx)?'>':' ', mainMenu[idx]);
            } break;
        case ST_EDIT:
            lcdLine(0,"=====EDIT VALUE=====");
            lcdLine(1,"  %s", mainMenu[menuIdx]);
            if(menuIdx==1) lcdLine(2,"  Value: %lu kHz", cfg.freq/1000);
            else lcdLine(2,"  Value: %d %%", cfg.duty);
            lcdLine(3," <Rot>Adj  <Btn>OK  "); break;
        case ST_SETTINGS:
            lcdLine(0,"======SETTINGS======");
            for(int i=0; i<3; i++){
                int idx = constrain(settingIdx-1+i, 0, SETT_MENU_LEN-1);
                if(idx < 0 || idx >= SETT_MENU_LEN) continue;
                char c = (idx==settingIdx)?'>':' ';
                switch(idx){                    case 0: lcdLine(i+1, "%c%-11s%3d%cC", c, settMenu[0], cfg.tempLimit, (char)0xDF); break;
                    case 1: lcdLine(i+1, "%c%-11s%3dA",   c, settMenu[1], cfg.currLimit); break;
                    case 2: lcdLine(i+1, "%c%-11s%3ds",   c, settMenu[2], cfg.softStart); break;
                    case 3: lcdLine(i+1, "%c%-11s%3dus",  c, settMenu[3], cfg.deadTime);  break;
                    default:lcdLine(i+1, "%c%s", c, settMenu[idx]); break;
                }
            } break;
        case ST_SET_EDIT:
            lcdLine(0,"====EDIT SETTING====");
            lcdLine(1,"  %s", settMenu[settingIdx]);
            switch(settingIdx){
                case 0: lcdLine(2,"  Value: %d %cC",cfg.tempLimit,(char)0xDF); break;
                case 1: lcdLine(2,"  Value: %d A",cfg.currLimit); break;
                case 2: lcdLine(2,"  Value: %d s",cfg.softStart); break;
                case 3: lcdLine(2,"  Value: %d us",cfg.deadTime); break;
                case 4: lcdLine(2,"  Value: %d s",cfg.fanCooldown); break;
                case 5: lcdLine(2,"  Value: %.2f",cfg.pidKp); break;
                case 6: lcdLine(2,"  Value: %.2f",cfg.pidKi); break;
                case 7: lcdLine(2,"  Value: %.2f",cfg.pidKd); break;
                case 8: lcdLine(2,"  Target: %.1f A",cfg.pidTarget); break;
            }
            lcdLine(3," <Rot>Adj  <Btn>OK  "); break;
        case ST_CONFIRM:
            lcdLine(0,"                    "); lcdLine(1,"      ** OK! **     ");
            lcdLine(2, confirmMsg); lcdLine(3,"                    "); break;
        case ST_FAULT: {
            lcdLine(0,"!!!!!! FAULT !!!!!!!");
            const char* msgs[] = {"","OVER CURRENT   ","OVER TEMPERATURE ","HW DRIVER FAULT  ","RESONANCE LOST   "};
            lcdLine(1," %s", diag.lastFaultCode<5 ? msgs[diag.lastFaultCode] : "UNKNOWN        ");
            lcdLine(2,"T:%3d%cC    I:%3dA  ",(int)(temperature+0.5f),(char)0xDF,(int)(current+0.5f));
            lcdLine(3,"   Press to clear   "); break;
        }
        case ST_DIAG:
            lcdLine(0,"===DIAGNOSTICS====");
            lcdLine(1,"Last: %s", diag.lastFaultMsg);
            lcdLine(2,"Max T:%d%cC  Max I:%dA",(int)(diag.maxTemp+0.5f),(char)0xDF,(int)(diag.maxCurr+0.5f));
            lcdLine(3,"RunTime: %lu min", diag.runTimeMin); break;
        case ST_CALIB_TEMP: case ST_CALIB_CURR:
            lcdLine(0,"===CALIBRATION=====");
            lcdLine(1, appState==ST_CALIB_TEMP?"TEMP SENSOR CALIB":"CURR SENSOR CALIB");
            if(appState==ST_CALIB_TEMP) lcdLine(2,"Off:%.2f Sc:%.2f",cal.tempOff,cal.tempSc);
            else lcdLine(2,"Off:%.2f Sc:%.2f",cal.currOff,cal.currSc);
            lcdLine(3,"Set zero/load & OK"); break;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ════════════════════════════════════════════════════════════════════════════
void setup() {    Serial.begin(115200);
    pinMode(ENC_CLK, INPUT_PULLUP); pinMode(ENC_DT, INPUT_PULLUP); pinMode(ENC_SW, INPUT_PULLUP);
    pinMode(FAN_PIN, OUTPUT); digitalWrite(FAN_PIN, LOW);
    pinMode(FAULT_PIN, INPUT_PULLUP); pinMode(ZCD_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(ENC_CLK), onEncoderCLK, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FAULT_PIN), onHWFault, FALLING);
    attachInterrupt(digitalPinToInterrupt(ZCD_PIN), onZCD, RISING);
    Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin();
    lcd.init(); lcd.backlight();
    EEPROM.begin(); loadAll();
    pwmTimer.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PWM_CH1);
    pwmTimer.setOverflow(cfg.freq, HERTZ_FORMAT);
    pwmTimer.setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
    pwmTimer.resume();
    pinMode(PWM_CH1N, OUTPUT);
    lcdLine(0,"  INDUCTION HEATER  "); lcdLine(1,"   CONTROLLER v3.0  ");
    lcdLine(2,"  STM32F411 PRO    "); lcdLine(3,"   Initializing...  ");
    delay(1500); dirty = true;
}

void loop() {
    uint32_t now = millis();
    if (faultRaw || (heaterOn && (temperature>=cfg.tempLimit || current>=cfg.currLimit))) {
        faultActive = true; faultRaw = false;
    }
    if (faultActive && appState != ST_FAULT) {
        stopHeater(); applyPWM(); appState = ST_FAULT; dirty = true;
    }
    if (appState == ST_CONFIRM && (now - confirmTimer) >= 1500) { appState = ST_SETTINGS; dirty = true; }
    if (now - sensorTimer >= SENSOR_MS) {
        sensorTimer = now;
        if (appState != ST_FAULT) { readSensors(); checkProtection(); runPID(); autoTrackTick(); }
        if (heaterOn) { static uint32_t heaterStartMs = 0; if (heaterStartMs == 0) heaterStartMs = now; diag.runTimeMin = ((now - heaterStartMs) / 60000UL); }
    }
    if (heaterOn && softRunning && (now - softTickMs) >= SOFTSTART_TICK_MS) {
        softTickMs = now;
        if (runDuty < cfg.duty) runDuty++; else softRunning = false;
    }
    if (fanCooldownActive && now >= fanStopTime) {
        digitalWrite(FAN_PIN, LOW); fanCooldownActive = false; dirty = true;
    }
    if (encDelta != 0) handleEncoder();
    handleButton();
    applyPWM();
    if (dirty && (now - lcdTimer >= LCD_REFRESH_MS)) { lcdTimer = now; updateDisplay(); dirty = false; }
}