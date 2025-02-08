#ifndef _Limes_SW1_2H
#define _Limes_SW1_2H
// General macros
#define str(s)              #s
#define xstr(s)             str(s)

/***************************************************************************************************
* 用户配置                                                                               *
***************************************************************************************************/

//#define _DEVELOPMENT_                       /**< 允许打印诊断信息 */
//#define _BOOTSYS_                           /**< 强制引导到系统菜单进行测试 */

#define _LANG_EN_                           /**< 语言：LANG_EN/DE/FR/ES/IT*/
//#define _TESTING_                           /**< 允许忽略低电量警报 */
#define _SERIAL_BAUD_       115200          /**< 串行调试的通信速率 */

/***************************************************************************************************
* Pin and interrupt definitions                                                                    *
***************************************************************************************************/

#define ENC_INT             0               /**< CLK 输入的旋转中断 (Ph0) */
#define PIN_CLK             2               /**< 旋转编码器 CLK 输入 (Ph0) */
#define PIN_DT              8               /**< 旋转编码器 DT 输入 (Ph1) */
#define PIN_SW              6               /**< 旋转编码器按钮开关输入 */
#define PIN_FOOT_SWITCH     7               /**< 脚踏开关感应输入 */
#define PIN_AUTO_PULSE      3               /**< 自动脉冲感应输入 */
#define PIN_PULSE           5               /**< 焊接脉冲输出 */
#define PIN_LED             4               /**< 状态 LED 输出 */
#define PIN_BATT_V          A0              /**< 用于电池电压检测的 A/D 输入 */
#define PIN_BUZZ            A1              /**< 蜂鸣器输出 */
#define PIN_TEMP            A3              /**< 蜂鸣器输出 */

/***************************************************************************************************
* Macros                                                                                           *
***************************************************************************************************/
// Defaults for operational variables
#define DEF_AUTO_PLSDELAY   5               /**< 默认自动脉冲延迟时间 (ms*100) */ 
#define DEF_PULSE_TIME      5               /**< 默认脉冲时间 (ms) */ 
#define DEF_MAX_PULSE_TIME  50              /**< 默认最大脉冲时间 (ms) */ 
#define DEF_SPULSE_TIME     12              /**< 默认短脉冲时间（脉冲时间的 %）*/ 
#define DEF_NOM_BATT_V      58              /**< 默认标称电池电压（用于测试） */
#define DEF_MAX_BATT_V      58              /**< 默认最大电池电压 (V*10) */
#define DEF_PULSE_VOLTAGE   40              /**< 脉冲期间的默认电压（用于测试） (V*10) */
#define DEF_PULSE_AMPS      5000            /**< 脉冲期间的默认安培（用于测试）(A*10) */
#define DEF_BATTERY_OFFSET  0               /**< 默认电池校准偏移(V*10) */
#define DEF_BATT_ALARM      40              /**< 默认低电池电压 (V*10) */
#define DEF_HIGH_BATT_ALARM 60              /**< 默认高电池电压(V*10) */
#define DEF_HIGH_TEMP_ALARM 65              /**< 默认高温 */
#define DEF_AUTO_PULSE      true            /**< 默认自动脉冲启用 */
#define DEF_WELD_SOUND      true            /**< 默认焊接声音启用 */ 
#define DEF_OLED_INVERT     false           /**< 默认 OLED 方向 */ 

// Limits for operational variables
#define MIN_PULSE_TIME      1               /**< 最小焊接脉冲时间 */
#define MAX_PULSE_TIME      50              /**< 绝对最大焊接脉冲时间 */
#define MAX_APULSE_DELAY    50              /**< 最大自动脉冲延迟 */
#define MIN_APULSE_DELAY    5               /**< 最小自动脉冲延迟 */
#define MAX_SPULSE_TIME     50              /**< 最大短脉冲时间 */
#define MIN_SPULSE_TIME     0               /**< 最小短脉冲时间 */
#define MAX_BATT_ALARM      40              /**< 最大低电池报警电压 */
#define MIN_BATT_BALARM     40              /**< 最低低电池报警电压 */
#define MAX_BATT_V          59.5              /**< 绝对最大电池电压 */
#define MIN_BATT_V          40              /**< 绝对最低电池电压 */

// Timing macros
#define STANDBY_TIME_OUT    300000L         /**< 设备休眠超时 (ms) */ 
#define EEPROM_UPDATE_T     5000            /**< EEPROM 更新时间 (ms) */
#define WP_RETRIGGER_DELAY  500             /**< 焊接脉冲重触发延迟 (ms) */
#define FS_TRIGGER_DELAY    200             /**< 脚踏开关激活延迟 (ms) */
#define RS_DEBOUNCE         50  /*20*/      /**< 旋转编码器 & 开关防抖时间 (ms) */
#define BV_INTERVAL         2000            /**< 电池电压测量间隔 (ms) */
#define T_INTERVAL          1000            /**< 测温间隔 (ms) */
#define PV_DELAY            2000            /**< 在脉冲后显示脉冲数据之前，必须按住脚踏开关的时间 (ms) */

// Display screen layout
#define CHR_W               6               /**< 字符宽度 [size=1]（像素）*/   
#define CHR_H               8               /**< 字符高度 [size=1]（像素）*/   
#define LINE_H              (CHR_H+2)       /**< 线的高度 [size=1]（像素）*/   

// Macros to define logical states
#define DD_READ             true            /**< 数据传输方向 - 读取 */
#define DD_WRITE            false           /**< 数据传输方向 - 写入 */
#define P_ON                true            /**< ON 状态的通用宏 */
#define P_OFF               false           /**< OFF 状态的通用宏 */
#define B_DN                true            /**< DOWN 状态的通用宏 */
#define B_UP                false           /**< UP 状态的通用宏 */
#define PL_ACTIVE_H         false           /**< Active High 的引脚逻辑宏 */
#define PL_ACTIVE_L         true            /**< 有效低电平的引脚逻辑宏 */

// EEPROM macros
#define EEA_ID              0               /**< 唯一 ID 地址 */
#define EEA_PDATA           (EEA_ID+4)      /**< 程序数据的 Eeprom 地址 */
#define EE_UNIQUEID         0x18fae9c8      /**< 唯一 EePROM 验证 ID */
#define EE_FULL_RESET       true            /**< 重置参数以重置所有 Eeprom 参数 */

// 伪装成函数的宏 - 使代码更具可读性
/** 此宏读取编码器上按钮开关的状态。*/
#define btnState()          (!digitalRead(PIN_SW))

/** 此宏驱动焊接脉冲。*/
#define weldPulse(state)    digitalWrite(PIN_PULSE,state?HIGH:LOW)

/** Where has this macro gone?? It was in WString.h */
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))               

/***************************************************************************************************
*OLED 显示配置                                                                *
***************************************************************************************************/

#define SCREEN_WIDTH 128 // OLED 显示宽度，以像素为单位
#define SCREEN_HEIGHT 64 // OLED 显示屏高度，以像素为单位

#define OLED_RESET          4               /**< OLED mode */
#define OLED_INVERT         4               /**< OLED 定义的定向模式 - 检查 OLED 文档 */
#define SPLASHTIME          2500            /**< 启动屏幕时间（毫秒）*/


/***************************************************************************************************
* 结构、联合和枚举类型定义                                               *
***************************************************************************************************/

typedef  enum {                             /**< 变量格式的类型枚举 */
         VF_BATTALM,                        /**< 电池报警电压 */
         VF_TEMPALM,                        /**< 温度报警值 */
         VF_BATTV,                          /**< 电池电压 */
         VF_BATTA,                          /**< 电池放大器 */
         VF_TEMP,                           /**< 温度 */
         VF_WELDCNT,                        /**< 焊缝支数 */
         VF_PLSDLY,                         /**< 脉冲延迟 */
         VF_SHTPLS,                         /**< 短脉冲持续时间 */
         VF_DELAY                           /**< 延迟 */
} vf_Type;

typedef  struct   progData {                /**< 程序操作数据结构 */
         uint8_t  autoPulseDelay;           /**< 自动脉冲延迟（ms/100） */ 
         uint8_t  batteryAlarm;             /**< 低电池电压（A/D 计数）*/
         uint8_t  batteryhighAlarm;         /**< 电池电压高（A/D 计数） */
         uint8_t  TCelsius;                 /**< 摄氏温度 */
         uint8_t  maxTCelsius;              /**< 最高温度（摄氏）*/
         uint16_t weldCount;                /**< 焊接次数 */
         uint16_t pulseTime;                /**< 脉冲时间 (ms) */ 
         uint16_t maxPulseTime;             /**< 最大允许脉冲时间 (ms) */ 
         uint8_t  shortPulseTime;           /**< 短脉冲时间（脉冲时间的 %）*/ 
         int8_t   batteryOffset;            /**< 电池电压校准偏移（有符号）x10 */
         uint16_t  PulseBatteryVoltage;     /**< 脉冲期间的电池电压 x10 */
         uint16_t  PulseAmps;               /**< 脉冲中的 Esimated Amps x10 */  
         struct progFlags {                 /**< 程序逻辑标志 */
             unsigned en_autoPulse:  1;     /**< 自动脉冲使能 */
             unsigned en_Sound:  1;         /**< 焊接声音使能 */ 
             unsigned en_oledInvert: 1;     /**< OLED 方向 - 反转其他正常为真 */
             unsigned unused:        6;     /**< 未使用的程序标志 */ 
         } pFlags;
} progData;

/***************************************************************************************************
* 程序原型                                                                        *
***************************************************************************************************/

void     stateMachine();

void     resetEeprom(boolean = false);
void     loadEeprom();
void     updateEeprom();

void     checkForLowVoltageEvent();
void     checkForSleepEvent();
void     checkForBtnEvent();
void     checkTemp();
void     foot_switch_error();
void     FootSwitch_Alarm();
void     Boot_Sound();
void     LowBattery_Sound();
void     isr();
void     splash();
void     sendWeldPulse(uint8_t, uint16_t, uint16_t, boolean = PL_ACTIVE_H);  
void     message(const __FlashStringHelper*, const __FlashStringHelper*,
                 const __FlashStringHelper*, uint8_t = 0);
void     displayMenuType1(const __FlashStringHelper*, const __FlashStringHelper*,
                          const __FlashStringHelper*, const __FlashStringHelper*, 
                          uint8_t SelectedItem);
void     displayMenuType2(const __FlashStringHelper*, const char*, const __FlashStringHelper*);
void     displayMainScreen();
void     displayPulseData();
void     displayLowBattery();
void     displayHighBattery();
void     displayHighTemperature();
void     drawStatusLine();
void     setTextProp(uint8_t, uint8_t, uint8_t, uint16_t = WHITE, boolean = false);
char*    valStr(char*, uint16_t, vf_Type);

/***************************************************************************************************
* 语言字符串（非常简单的语言实现 - 英语是默认值）               *
***************************************************************************************************/

// 将 Language 子句中的语言字符串复制到您的语言特定子句中，然后更改
// 适合您语言的字符串。在此头文件的顶部定义您的语言。它是
// 保持字符串的正确格式很重要。每个字符串都有一个内联注释
// 定义它的格式。

// 评论图例：小字体字段宽度 21，大字体 10
//                 对齐左、中、右  justification        left, centre, right
//                 两端填充空格以填充字段

#ifdef _LANG_DE_
          
#elif defined _LANG_FR_

#elif defined _LANG_ES_

#elif defined _LANG_IT_

#else
static const char LS_APULSE[]     PROGMEM = "Pulse Set";              // 脉冲设置10 char, centre, padded
static const char LS_BATTALM1[]   PROGMEM = "Batt Alarm";             // 电压设置10 char, centre, padded
static const char LS_SHORTPLS1[]  PROGMEM = "Shrt Pulse";             // 预热脉冲10 char, centre, padded

static const char LS_MANAUTO[]    PROGMEM = "  Mode    ";             // 触发设置10 char, centre, padded
static const char LS_DELAY[]      PROGMEM = "  Delay   ";             // 脉冲延时10 char, centre, padded
static const char LS_WELDSOUND[]  PROGMEM = "  Sound   ";             // 声音开关10 char, centre, padded

static const char LS_STANDBY[]    PROGMEM = "  STANDBY ";             // 休眠中10 char, centre, padded
static const char LS_CLICKBTN[]   PROGMEM = " Please Click Button ";   // 点击按钮21 char, left
static const char LS_EXITSTBY[]   PROGMEM = "   to Exit Standby";     // 退出休眠21 char, left

static const char LS_BATTALM[]    PROGMEM = "Low Battery Alarm";      // 电量过低21 char, left
static const char LS_BATTERY[]    PROGMEM = "BATTERY";                // 电容10 char, left
static const char LS_LOWV[]       PROGMEM = "LOW VOLTS";              // 低压报警10 char, left
static const char LS_HIGHV[]      PROGMEM = "HIGH VOLTS";             // 高压报警10 char, left
static const char LS_PULSEV[]     PROGMEM = "VOLTS DURING PULSE";     // 脉冲电压21 char, left
static const char LS_PULSEA[]     PROGMEM = "EST. AMPS";              // 21 char, left

static const char LS_TEMPALM[]    PROGMEM = "Temperature Alarm";      // 温度报警器21 char, left
static const char LS_TEMP[]       PROGMEM = "TEMP";                   // 10 char, left
static const char LS_HIGHT[]      PROGMEM = "HIGH TEMP";              // 高温10 char, left
static const char LS_CEL[]        PROGMEM = "TEMP IN CELSIUS";        // 温度（摄氏度）21 char, left
static const char LS_COOL[]       PROGMEM = "PLEASE COOL DOWN";       // 请降温21 char, left

static const char LS_AUTOPLSON[]  PROGMEM = "Weld Pulse Activation";  // 焊接脉冲激活21 char, left
static const char LS_AUTO[]       PROGMEM = "Auto Pulse";             // 自动脉冲10 char, left
static const char LS_MANUAL[]     PROGMEM = "Manual";                 // 手动触发10 char, left
static const char LS_AUTO_BAR[]   PROGMEM = "Auto";                   // 自动触发10 char, left
static const char LS_MANUAL_BAR[] PROGMEM = "Manual";                 // 10 char, left

static const char LS_AUTOPLSDLY[] PROGMEM = "Auto Pulse Delay";       // 自动脉冲延迟21 char, left
static const char LS_SHORTPLS[]   PROGMEM = "Short Pulse Duration";   // 预热脉冲持续时间21 char, left
static const char LS_WPDRN[]      PROGMEM = "Limes_SW V1.6";    // 21 char, left

static const char LS_WELDSOUNDM[]  PROGMEM = "Weld Pulse Sound";      // 焊接脉冲声音 21 char, left
static const char LS_SOUNDON[]    PROGMEM = "ON";                     // 10 char, left
static const char LS_SOUNDOFF[]   PROGMEM = "OFF";                    // 10 char, lef

static const char LS_BATTMSG[]    PROGMEM = " Battery";               // 电容10 char, centre
static const char LS_MAXPMSG[]    PROGMEM = "   Duration Set";        // 持续时间设置21 char, centre

static const char LS_PCOF[]       PROGMEM = "% of Pulse Time";        // 脉冲时间21 char, left
static const char LS_SECONDS[]    PROGMEM = "Seconds";                // 21 char, left
static const char LS_VOLTAGE[]    PROGMEM = "Volts";                  // 21 char, left
static const char LS_MS[]         PROGMEM = "ms";                     // 2  char, left
static const char LS_VUNITS[]     PROGMEM = "V";                      // 1  char, left
static const char LS_AUNITS[]     PROGMEM = "A";                      // 1  char, left
static const char LS_WELDS[]      PROGMEM = "W";                      //次 1  char, left
static const char LS_TUNITS[]     PROGMEM = " C";                    // 2  char, left


static const char LS_REBOOTFR[]   PROGMEM = "   with Full Reset";     // 21 char, centre
static const char LS_REBOOTNR[]   PROGMEM = "   without Reset";       // 21 char, centre
static const char LS_REBOOTSR[]   PROGMEM = "   with Safe Reset";     // 21 char, centre
static const char LS_RECALMSG[]   PROGMEM = "   re-calibrated";       // 21 char, centre
static const char LS_WAITMSG[]    PROGMEM = " PLEASE REMOVE POWER";   // 21 char, centre

static const char LS_SYSMENU[]    PROGMEM = "System Menu";            // 21 char, left
static const char LS_SETTINGS[]   PROGMEM = " Settings ";             // 10 char, centre, padded
static const char LS_DISPLAY[]    PROGMEM = "  Display ";             // 10 char, centre, padded
static const char LS_BOOT[]       PROGMEM = "   Boot   ";             // 10 char, centre, padded

static const char LS_SETTMENU[]   PROGMEM = "System Settings";        // 21 char, left
static const char LS_MAXPULSE[]   PROGMEM = "Max Pulse ";             // 10 char, centre, padded

static const char LS_BOOTMENU[]   PROGMEM = "Reboot Spot Welder";     // 21 char, left
static const char LS_REBOOT[]     PROGMEM = "  Reboot  ";             // 10 char, centre, padded  
static const char LS_SAFERST[]    PROGMEM = "Safe Reset";             // 10 char, centre, padded
static const char LS_FULLRST[]    PROGMEM = "Full Reset";             // 10 char, centre, padded

static const char LS_INVERTMENU[] PROGMEM = "Screen Orientation";     // 21 char, left
static const char LS_SCRNORM[]    PROGMEM = "NORMAL";                 // 10 char, left
static const char LS_SCRINV[]     PROGMEM = "INVERTED";               // 10 char, left

static const char LS_MAXPLSMENU[] PROGMEM = "Set Max Weld Pulse";     // 21 char, left

static const char LS_BATCALMENU[] PROGMEM = "Calibrate Battery";      // 21 char, left
static const char LS_BATCALMSG[]  PROGMEM = "Set Measured Volts";     // 21 char, left
static const char LS_MSGHDR[]     PROGMEM = "@Limes3D";         //系统消息 21 char, left


static const unsigned char PROGMEM hans_mai[] = {
0x00, 0x00, 0x00, 0x80, 0x38, 0x70, 0x48, 0x00, 0x49, 0xE0, 0x48, 0x20, 0x78, 0x22, 0x4B, 0xF4, 
0x48, 0x78, 0x48, 0xA8, 0x78, 0xA8, 0x49, 0x24, 0x49, 0x22, 0x4A, 0x21, 0x48, 0x20, 0xB9, 0xE0}; /*"脉", 0*/

static const unsigned char PROGMEM hans_chong[] = {
0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x23, 0xF8, 0x24, 0x46, 0x24, 0x42, 0x04, 0x42, 0x04, 0x42, 
0x04, 0x42, 0x14, 0x42, 0x13, 0xFC, 0x20, 0x40, 0x20, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x40}; /*"冲", 1*/

static const unsigned char PROGMEM hans_she[] = {
0x00, 0x00, 0x00, 0x00, 0x20, 0xF8, 0x30, 0x88, 0x10, 0x88, 0x01, 0x08, 0x71, 0x0A, 0x10, 0x04, 
0x11, 0xFC, 0x10, 0x84, 0x10, 0x88, 0x10, 0x48, 0x10, 0x50, 0x14, 0x30, 0x18, 0xC8, 0x03, 0x06}; /*"设", 2*/

static const unsigned char PROGMEM hans_zhi[] = {
0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x44, 0x44, 0x44, 0x44, 0x3B, 0xB8, 0x7F, 0xFE, 0x01, 0x00, 
0x1F, 0xF8, 0x10, 0x08, 0x1F, 0xF8, 0x10, 0x08, 0x1F, 0xF8, 0x1F, 0xF8, 0x10, 0x08, 0xFF, 0xFF}; /*"置", 3*/
#endif            

#endif // _ARDUINO_SPOT_WELDER_V4_H

// EOF Arduino_Spot_Welder_V4.h
 
