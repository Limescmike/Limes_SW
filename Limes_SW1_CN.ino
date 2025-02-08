#define _Limes_SW1_CN
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "printf.h"
#include "Limes_SW1_CN.h"

// Machine states
enum states {
	ST_STANDBY,        /**< 机器状态：待机 */
	ST_MAIN_SCREEN,    /**< 机器状态：显示主屏幕 */
	ST_MAIN_SCREEN_CNT, /**< 机器状态：显示统计 */
	ST_MENU_SCREEN,    /**< 机器状态：显示菜单画面 */
	ST_SUB_MENU_1,     /**< 机器状态：显示子菜单 1 */
	ST_SUB_MENU_2,     /**< 机器状态：显示子菜单 2 */
	ST_BATTERY_LOW,    /**< 机器状态：电池电压低 */
  ST_BATTERY_HIGH,   /**< 机器状态：电池电压高 */
  ST_TEMP_HIGH,      /**< 机器状态：高温 */

	ST_SYSTEM_SCREEN,  /**< 机器状态：显示系统画面 */
	ST_SYSTEM_MENU,    /**< 机器状态：显示系统菜单 */
	ST_REBOOT_MENU,    /**< 机器状态：显示重启菜单 */
	ST_MAXWELD_SCREEN, /**< 机器状态：显示最大焊接设置屏幕 */
	ST_INVERT_SCREEN,  /**< 机器状态：显示反转设置画面 */
	ST_PULSE_VOLTAGE,  /**< 机器状态：显示测量脉冲电压画面 */
};

enum event {
	// Private machine events
	EV_NONE,           /**< Machine event: no pending event */
	EV_BTNDN,          /**< Machine event: button pressed */
	EV_BTNUP,          /**< Machine event: button released */
	EV_ENCUP,          /**< Machine event: encoder rotate right */
	EV_ENCDN,          /**< Machine event: encoder rotate left */

	// Public machine events
	EV_BOOTDN,         /**< Machine event: button pressed on boot */
	EV_STBY_TIMEOUT,   /**< Machine event: standby timer has timed out */
	EV_BATT_LV,        /**< Machine event: battery low voltage event */
  EV_BATT_HV,        /**< Machine event: battery high voltage event */
  EV_TEMP_HIGH,      /**< Machine event: maximum temperature reached */
	EV_EEUPD,          /**< Machine event: EEPROM needs updating */
	EV_PULSE_VOLTS,    /**< Machine event: foot switch hold during pulse */
};

/***************************************************************************************************
* 全局程序变量和对象                                                           *
***************************************************************************************************/

// Structures and objects
progData pData;                             /**< Program operating data */
Adafruit_SSD1306 display( 128, 64, &Wire, OLED_RESET, 800000L ); /**< OLED display object */

// Static variables
uint8_t mState         = ST_MAIN_SCREEN;    /**< 当前机器状态 */
uint8_t  TCelsius;                          /**< 系统温度（摄氏度） */ 
uint8_t batteryVoltage = DEF_NOM_BATT_V;    /**< 当前电池电压 x10 */
int8_t batt_gauge;                          /**< 要显示的电池电量表段 */
boolean sysMenu = false;             /**< 在系统菜单结构中 */

// 易失性变量 - 将由 ISR 更改
volatile unsigned long lastActiveTime;
volatile uint8_t mEvent;          /**< 当前待处理机器事件 */

 void reset_mcusr( void ) __attribute__( ( naked ) ) __attribute__( ( section( ".init3" ) ) );
void reset_mcusr( void ) {
	MCUSR = 0;
	wdt_disable();
}


/***************************************************************************************************
* 程序设置                                                                                   *
***************************************************************************************************/
/**
 *  \brief    程序设置。
 *  \remarks  仅在加电时执行一次。
 */
void setup() {
#if defined  _DEVELOPMENT_ || defined _BOOTSYS_
		Serial.begin( _SERIAL_BAUD_ );
#endif /* _DEVELOPMENT_ || _BOOTSYS_*/

	// Setup the pin states and directions.
	pinMode( PIN_LED,              OUTPUT );
	pinMode( PIN_PULSE,            OUTPUT );
  pinMode( PIN_BUZZ,             OUTPUT );
	pinMode( PIN_CLK,              INPUT );
	pinMode( PIN_DT,               INPUT );
	pinMode( PIN_SW,               INPUT_PULLUP );
	pinMode( PIN_FOOT_SWITCH,      INPUT_PULLUP );
	pinMode( PIN_AUTO_PULSE,       INPUT );
	digitalWrite( PIN_SW,          HIGH );
	digitalWrite( PIN_FOOT_SWITCH, HIGH );

	// 设置 OLED 显示屏并清除它。
	display.begin( SSD1306_SWITCHCAPVCC, 0x3C );
	display.clearDisplay(); // clear remnants when rebooting
	display.display();

// 中断用于感知编码器旋转。它也可以被轮询
// 没有损失响应能力。这实际上已经尝试过了，没有明显的性能
// 观察到退化。中断对于高速编码器很有用，例如
// 用于伺服系统。手动调整的编码器非常慢。
	attachInterrupt( ENC_INT, isr, FALLING );

	// 这是由待机定时器用来识别不活动的时期。
	lastActiveTime = millis();

// 在启动时测试按钮是否被按下。如果是这样，请确保进入系统
// 菜单通过发出启动按钮关闭事件。
#ifdef _BOOTSYS_
		mEvent = EV_BOOTDN;
#else
		mEvent = btnState() == B_DN ? EV_BOOTDN : EV_NONE;
#endif /* _BOOTSYS_ */

	loadEeprom();

	// 如果需要，反转显示并绘制启动屏幕。这只能在
  // 加载程序数据（包含屏幕反转开关）。
	display.setRotation( pData.pFlags.en_oledInvert ? OLED_INVERT : 0 );
	splash();

	if ( !digitalRead( PIN_FOOT_SWITCH ) ) {
		foot_switch_error();
    FootSwitch_Alarm();
	}

  batteryVoltage = DEF_NOM_BATT_V;
  Boot_Sound();

#ifdef _DEVELOPMENT_
		Serial.print( F( "Pulse Time       " ) ); Serial.println( pData.pulseTime );
		Serial.print( F( "Pulse Voltage    " ) ); Serial.println( pData.PulseBatteryVoltage );
		Serial.print( F( "Pulse Amps       " ) ); Serial.println( pData.PulseAmps );
		Serial.print( F( "Battery Alarm    " ) ); Serial.println( pData.batteryAlarm );
		Serial.print( F( "Weld Count       " ) ); Serial.println( pData.weldCount );
		Serial.print( F( "Auto Pulse Delay " ) ); Serial.println( pData.autoPulseDelay );
		Serial.print( F( "Max Pulse Time   " ) ); Serial.println( pData.maxPulseTime );
		Serial.print( F( "Short Pulse Time " ) ); Serial.println( pData.shortPulseTime );
		Serial.print( F( "Auto Pulse       " ) ); Serial.println( pData.pFlags.en_autoPulse ? "On" : "Off" );
    Serial.print( F( "Weld Sound       " ) ); Serial.println( pData.pFlags.en_Sound ? "On" : "Off" );
		Serial.print( F( "Display Invert   " ) ); Serial.println( pData.pFlags.en_oledInvert ? "Invert" : "Normal" );
#endif /* _DEVELOPMENT_ */
}

/***************************************************************************************************
* 主程序循环                                                                          *
***************************************************************************************************/
/**
 *  \brief    主程序循环。
 *  \remarks  管理状态机和电可擦只读存储器更新。
 */
void loop() {
	// 状态机在每个循环周期运行 - 尽可能快。
	stateMachine();

// 电可擦只读存储器用改变的变量进行更新，更新例程在内部确保
// 电可擦只读存储器每隔几秒才更新一次，以限制耗尽。
	updateEeprom();
}

/***************************************************************************************************
* 状态机                                                                                    *
***************************************************************************************************/
/**
 *  \brief  状态机的实现。
 */

// 整个状态机是经过大量修改和添加的遗留代码。它是
// 有点像 “狗的早餐”，真的需要完全重写。甚至我也有
// 困难遵循它和修改工作是一个惊喜 - JFF。

void stateMachine() {
	char str[5];
	static uint8_t selectedMenu       = 0;
	static uint8_t selectedMainMenu   = 0;
	static uint8_t selectedSubMenu    = 0;
	static boolean btn                = false;
	static uint8_t measuredVoltage;

	// 扫描事件 - 事件队列长度为 1。

	// 处理任何公共启动事件。这些事件是最高优先级的
// 并且必须在任何私人事件之前进行处理。
	if ( mEvent == EV_BOOTDN ) {
		mState = ST_SYSTEM_SCREEN;
		mEvent = EV_NONE;

	} else {
		// 搜索并处理任何私人事件。
		checkForBtnEvent();
		checkForSleepEvent();
		checkForLowVoltageEvent();
    checkTemp();

		switch ( mEvent ) {
		case EV_STBY_TIMEOUT: mState = ST_STANDBY;
			mEvent = EV_NONE;
			break;
		case EV_BATT_LV:

			if ( !sysMenu ) mState = ST_BATTERY_LOW;

			mEvent = EV_NONE;
			break;
    case EV_BATT_HV:

      if ( !sysMenu ) mState = ST_BATTERY_HIGH;

      mEvent = EV_NONE;
      break;
     case EV_TEMP_HIGH:

      if ( !sysMenu ) mState = ST_TEMP_HIGH;

      mEvent = EV_NONE;
      break;
		case EV_BTNUP:

			if ( mState != ST_REBOOT_MENU ) mEvent = EV_NONE;

			break;
		case EV_PULSE_VOLTS: mState = ST_PULSE_VOLTAGE;
			mEvent = EV_NONE;
			break;
		}
	}

	// 机器状态
	switch ( mState ) {

// 脉冲时按住脚踏开关按钮进入的脉冲电压状态，
// 它由按钮按下事件退出。
	case ST_PULSE_VOLTAGE:

		if ( mEvent == EV_BTNDN )
			mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;

		displayPulseData();
		mEvent = EV_NONE;
		break;

	// 接收到待机超时事件进入待机状态，
  // 它由按钮按下事件退出。
	case ST_STANDBY:

		if ( mEvent == EV_BTNDN ) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;

		message( FPSTR( LS_STANDBY ), FPSTR( LS_CLICKBTN ), FPSTR( LS_EXITSTBY ) );
		mEvent = EV_NONE;
		break;

// 低电池状态。通过接收低电量事件进入，
// 这个状态可以通过点击旋转编码器退出
	case ST_BATTERY_LOW:

		if ( mEvent == EV_BTNDN ) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;

		displayLowBattery();
    LowBattery_Sound();
		mEvent = EV_NONE;
		break;

// 过压电池状态。通过接收高电量事件进入，
// 除非重新启动，否则无法退出此状态。
  case ST_BATTERY_HIGH:

    if ( mEvent == EV_BTNDN ) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;

    displayHighBattery();
    mEvent = EV_NONE;
    break;


// 超温状态。通过接收高温事件进入，
// 这个状态可以通过点击旋转编码器退出
  case ST_TEMP_HIGH:

    if ( mEvent == EV_BTNDN ) mState = sysMenu ? ST_SYSTEM_SCREEN : ST_MAIN_SCREEN;

    displayHighTemperature();
    mEvent = EV_NONE;
    break;    

// 主屏幕状态显示主屏幕而不响应事件。这是
// 实际上是循环通过 this 和 the 的空闲循环的主位置
// 它查找事件的下一个状态。任何状态都可以返回到这个状态
// 启用空闲循环。
	case ST_MAIN_SCREEN:

		mState = ST_MAIN_SCREEN_CNT;
		sysMenu = false;
		selectedMenu = 0;
		displayMainScreen();
		break;

// 响应事件的主屏幕状态。此状态是
// 菜单结构并允许设置和显示操作变量。
// 如果需要，焊接脉冲也会从该状态发出。
	case ST_MAIN_SCREEN_CNT:

// 如果需要，激活焊接脉冲。
// 检查 AutoPulse 是否已启用和激活。
		if ( digitalRead( PIN_AUTO_PULSE ) && pData.pFlags.en_autoPulse ) {
			displayMainScreen( 1 );
      if (pData.pFlags.en_Sound) tone(PIN_BUZZ, 1000, 100); //Probe Sense Confirmation Sound
			sendWeldPulse( PIN_AUTO_PULSE, pData.autoPulseDelay * 100, WP_RETRIGGER_DELAY );
			displayMainScreen();
		}

// 否则检查脚踏开关是否被激活。
		else if ( !digitalRead( PIN_FOOT_SWITCH ) )
			sendWeldPulse( PIN_FOOT_SWITCH, FS_TRIGGER_DELAY, WP_RETRIGGER_DELAY, PL_ACTIVE_L );

		// 按钮按下事件将进入菜单结构。
		if ( mEvent == EV_BTNDN ) {
			mState = ST_MENU_SCREEN;
			selectedMenu = 0;
			mEvent = EV_NONE;
			displayMenuType1( NULL, FPSTR( LS_APULSE ), FPSTR( LS_BATTALM1 ), FPSTR( LS_SHORTPLS1 ), 0 );

		} else {

			// 旋转编码器事件将改变焊接脉冲时间的值。
      // 时间限制在 1ms 和存储的最大值之间。
			if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
				if ( mEvent == EV_ENCUP ) pData.pulseTime = pData.pulseTime < pData.maxPulseTime
			? pData.pulseTime + 1 : pData.maxPulseTime;
				else pData.pulseTime = pData.pulseTime > MIN_PULSE_TIME ?
					               pData.pulseTime - 1 : MIN_PULSE_TIME;

				pData.pulseTime = pData.pulseTime <MIN_PULSE_TIME ? MIN_PULSE_TIME :
				                                   pData.pulseTime> pData.maxPulseTime ? pData.maxPulseTime : pData.pulseTime;
				mEvent = EV_NONE;
			}

			displayMainScreen();
		}

		break;

// 菜单结构中的第一个菜单。这是从主屏幕访问的
// 通过按钮按下事件。
	case ST_MENU_SCREEN:

// 一个按钮按下事件将进入菜单结构的下一级。有
// 菜单中的三个项目来决定输入哪个新菜单。
		if ( mEvent == EV_BTNDN ) {
			mState = ST_SUB_MENU_1;
			selectedMainMenu = selectedMenu;

			// 进入子菜单选择更多选项。
			if ( selectedMainMenu == 0 )
				displayMenuType1( FPSTR( LS_APULSE ), FPSTR( LS_MANAUTO ),
				                  FPSTR( LS_DELAY ),  FPSTR( LS_WELDSOUND ), selectedMenu );

			// 允许设置电池低报警电压。
			else if ( selectedMainMenu == 1 ) displayMenuType2( FPSTR( LS_BATTALM ),
				                                            valStr( str, pData.batteryAlarm, VF_BATTALM ), FPSTR( LS_VOLTAGE ) );

			// 允许设置预热脉冲持续时间。
			else if ( selectedMainMenu == 2 ) displayMenuType2( FPSTR( LS_SHORTPLS ),
				                                            valStr( str, pData.shortPulseTime, VF_SHTPLS ), FPSTR( LS_PCOF ) );

			mEvent = EV_NONE;
			selectedMenu = 0;

			// 旋转编码器事件将允许滚动菜单选项。
		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			if ( mEvent == EV_ENCDN ) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
			else selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;

			mEvent = EV_NONE;
			displayMenuType1( NULL, FPSTR( LS_APULSE ), FPSTR( LS_BATTALM1 ),
			                  FPSTR( LS_SHORTPLS1 ), selectedMenu );
		}

		break;

// 菜单结构中的下一个菜单。这是从
// 按下按钮的第一个菜单。
	case ST_SUB_MENU_1:

		// 一个按钮按下事件将进入菜单结构的下一级。有
    // 菜单中的三个项目来决定输入哪个新菜单。
		if ( mEvent == EV_BTNDN ) {
			if ( selectedMainMenu == 0 ) {
				mState = ST_SUB_MENU_2;
				selectedSubMenu = selectedMenu;

				// 设置自动脉冲状态。
				if ( selectedSubMenu == 0 ) displayMenuType2( FPSTR( LS_AUTOPLSON ), NULL,
					                                      pData.pFlags.en_autoPulse ? FPSTR( LS_AUTO ) : FPSTR( LS_MANUAL ) );

				// 允许设置自动脉冲延迟时间。
				else if ( selectedSubMenu == 1 ) displayMenuType2( FPSTR( LS_AUTOPLSDLY ),
					                                           valStr( str, pData.autoPulseDelay, VF_DELAY ), FPSTR( LS_SECONDS ) );

				// 设置脉冲激活声音。
				else if ( selectedSubMenu == 2 ) displayMenuType2( FPSTR( LS_WELDSOUNDM ), NULL,
					                                                 pData.pFlags.en_autoPulse ? FPSTR( LS_SOUNDON ) : FPSTR( LS_SOUNDOFF ) );

			} else if ( selectedMainMenu == 1 ) mState = ST_MAIN_SCREEN;

			else if ( selectedMainMenu == 2 ) mState = ST_MAIN_SCREEN;
			else mState = ST_MAIN_SCREEN;

			mEvent = EV_NONE;
			selectedMenu = 0;

			// 菜单项类型已经从上一个菜单中选择。这里是
      // 变量根据所选类型进行更改。
		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			if ( selectedMainMenu == 0 ) {
				if ( mEvent == EV_ENCDN ) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
				else selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;

				displayMenuType1( FPSTR( LS_APULSE ), FPSTR( LS_MANAUTO ),
				                  FPSTR( LS_DELAY ),  FPSTR( LS_WELDSOUND ), selectedMenu );

				// 通过响应旋转编码器事件设置电池警报电压。
        // 电池报警电压不能设置高于当前电池电压
        // 并且在任何情况下都不高于允许的最大报警电压。
			} else if ( selectedMainMenu == 1 ) {
				if ( mEvent == EV_ENCDN ) pData.batteryAlarm = pData.batteryAlarm >
					                                       MIN_BATT_BALARM ? pData.batteryAlarm - 1 : MIN_BATT_BALARM;
				else pData.batteryAlarm = pData.batteryAlarm < min( batteryVoltage, MAX_BATT_ALARM ) ?
					                  pData.batteryAlarm + 1 : min( batteryVoltage, MAX_BATT_ALARM );

				displayMenuType2( FPSTR( LS_BATTALM ),
				                  valStr( str, pData.batteryAlarm, VF_BATTALM ), FPSTR( LS_VOLTAGE ) );

				// 通过响应旋转编码器事件设置预热脉冲持续时间。
			} else if ( selectedMainMenu == 2 ) {
				if ( mEvent == EV_ENCDN ) pData.shortPulseTime = pData.shortPulseTime >
					                                         MIN_SPULSE_TIME ?  pData.shortPulseTime - 1 : MIN_SPULSE_TIME;
				else pData.shortPulseTime = pData.shortPulseTime < MAX_SPULSE_TIME ?
					                    pData.shortPulseTime + 1 : MAX_SPULSE_TIME;

				displayMenuType2( FPSTR( LS_SHORTPLS ),
				                  valStr( str, pData.shortPulseTime, VF_SHTPLS ), FPSTR( LS_PCOF ) );
			}

			mEvent = EV_NONE;
		}

		break;

	case ST_SUB_MENU_2:

		if ( mEvent == EV_BTNDN ) {
			mState = ST_MAIN_SCREEN;
			mEvent = EV_NONE;
			selectedMenu = 0;

		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			if ( selectedSubMenu == 0 ) {
				pData.pFlags.en_autoPulse ^= 1;
				displayMenuType2( FPSTR( LS_AUTOPLSON ), NULL, pData.pFlags.en_autoPulse ?
				                  FPSTR( LS_AUTO ) : FPSTR( LS_MANUAL ) );
			} else if ( selectedSubMenu == 1 ) {
				if ( mEvent == EV_ENCDN ) pData.autoPulseDelay = pData.autoPulseDelay >
					                                         MIN_APULSE_DELAY ? pData.autoPulseDelay - 1 : MIN_APULSE_DELAY;
				else pData.autoPulseDelay = pData.autoPulseDelay < MAX_APULSE_DELAY ?
					                    pData.autoPulseDelay + 1 : MAX_APULSE_DELAY;

				displayMenuType2( FPSTR( LS_AUTOPLSDLY ),
				                  valStr( str, pData.autoPulseDelay, VF_DELAY ), FPSTR( LS_SECONDS ) );
			}
        else if ( selectedSubMenu == 2 ) {
				pData.pFlags.en_Sound ^= 1;
				displayMenuType2( FPSTR( LS_WELDSOUNDM ), NULL, pData.pFlags.en_Sound ?
				                  FPSTR( LS_SOUNDON ) : FPSTR( LS_SOUNDOFF ) );
      }                    

			mEvent = EV_NONE;
		}

		break;

// 系统屏幕状态显示系统屏幕而不响应事件。
// 这是另一个空闲循环的主状态，这个用于系统菜单。
	case ST_SYSTEM_SCREEN:

		mState = ST_SYSTEM_MENU;
		sysMenu = true;
		selectedMenu = 0;
		displayMenuType1( FPSTR( LS_SYSMENU ), FPSTR( LS_MAXPULSE ),
		                  FPSTR( LS_DISPLAY ), FPSTR( LS_BOOT ), 0 );
		break;

// 系统菜单结构中的第一个菜单。这是通过重新启动
// 焊机同时按住编码器按钮。
	case ST_SYSTEM_MENU:

		if ( mEvent == EV_BTNDN ) {
			mEvent = EV_NONE;

			if ( selectedMenu == 0 ) {
				displayMenuType2( FPSTR( LS_MAXPLSMENU ),
				                  valStr( str, pData.maxPulseTime, VF_PLSDLY ), FPSTR( LS_MS ) );
				mState = ST_MAXWELD_SCREEN;
				mEvent = EV_NONE;

			} else if ( selectedMenu == 1 ) {
				selectedSubMenu = 0;
				mState = ST_INVERT_SCREEN;
				displayMenuType2( FPSTR( LS_INVERTMENU ), NULL, pData.pFlags.en_oledInvert ?
				                  FPSTR( LS_SCRINV ) : FPSTR( LS_SCRNORM ) );

			} else if ( selectedMenu == 2 ) {
				btn = false;
				mState = ST_REBOOT_MENU;
				displayMenuType1( FPSTR( LS_BOOTMENU ), FPSTR( LS_REBOOT ),
				                  FPSTR( LS_SAFERST ),  FPSTR( LS_FULLRST ), selectedMenu = 0 );
			}

			selectedMenu = 0;

		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			if ( mEvent == EV_ENCDN ) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
			else selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;

			mEvent = EV_NONE;
			displayMenuType1( FPSTR( LS_SYSMENU ), FPSTR( LS_MAXPULSE ),
			                  FPSTR( LS_DISPLAY ), FPSTR( LS_BOOT ), selectedMenu );
		}

		break;


	case ST_REBOOT_MENU:

		if ( mEvent == EV_BTNDN ) btn = true;
		else if ( ( mEvent == EV_BTNUP ) && btn ) {
			if ( selectedMenu == 1 )
				resetEeprom( false );
			else if ( selectedMenu == 2 )
				resetEeprom( true );

			message( FPSTR( LS_REBOOT ), selectedMenu == 1 ? FPSTR( LS_REBOOTSR ) :
			         selectedMenu == 2 ? FPSTR( LS_REBOOTFR ) :
			         FPSTR( LS_REBOOTNR ), FPSTR( LS_WAITMSG ), 2 );


		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			if ( mEvent == EV_ENCDN ) selectedMenu = selectedMenu == 0 ? 2 : selectedMenu - 1;
			else selectedMenu = selectedMenu == 2 ? 0 : selectedMenu + 1;

			displayMenuType1( FPSTR( LS_BOOTMENU ), FPSTR( LS_REBOOT ),
			                  FPSTR( LS_SAFERST ),  FPSTR( LS_FULLRST ), selectedMenu );
		}

		mEvent = EV_NONE;
		break;

	case ST_MAXWELD_SCREEN:

		if ( mEvent == EV_BTNDN ) {
			message( FPSTR( LS_MAXPULSE ), FPSTR( LS_MAXPMSG ), FPSTR( LS_WAITMSG ), 2 );
			mState = ST_SYSTEM_SCREEN;
			selectedMenu = 0;

		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			if ( mEvent == EV_ENCDN ) pData.maxPulseTime = pData.maxPulseTime > MIN_PULSE_TIME ?
				                                       pData.maxPulseTime - 1 : MIN_PULSE_TIME;
			else pData.maxPulseTime = pData.maxPulseTime < MAX_PULSE_TIME ?
				                  pData.maxPulseTime + 1 : MAX_PULSE_TIME;

			pData.maxPulseTime = pData.maxPulseTime <MIN_PULSE_TIME ? MIN_PULSE_TIME :
			                                         pData.maxPulseTime> MAX_PULSE_TIME ? MAX_PULSE_TIME : pData.maxPulseTime;
			displayMenuType2( FPSTR( LS_MAXPLSMENU ),
			                  valStr( str, pData.maxPulseTime, VF_PLSDLY ), FPSTR( LS_MS ) );
		}

		mEvent = EV_NONE;
		break;


	case ST_INVERT_SCREEN:

		if ( mEvent == EV_BTNDN ) {
			mState = ST_SYSTEM_SCREEN;
			mEvent = EV_NONE;
			selectedMenu = 0;

		} else if ( mEvent == EV_ENCUP || mEvent == EV_ENCDN ) {
			pData.pFlags.en_oledInvert = !pData.pFlags.en_oledInvert;
			display.setRotation( pData.pFlags.en_oledInvert ? 2 : 0 );
			displayMenuType2( FPSTR( LS_INVERTMENU ), NULL, pData.pFlags.en_oledInvert ?
			                  FPSTR( LS_SCRINV ) : FPSTR( LS_SCRNORM ) );
		}

		mEvent = EV_NONE;
		break;

	default: break;
	}
}

/***************************************************************************************************
* 硬件管理例程                                                                    *
***************************************************************************************************/
struct progress {
	uint16_t time;
#define PGR_OFF   ( 0 << 0 )
#define PGR_ON    ( 1 << 0 )
#define PGR_INIT  ( 1 << 7 )
	uint8_t opt;

	uint8_t step;
	unsigned long millis;
};

/**
 *  \brief      绘制进度条
 *
 *  \param [in/out] o           进度控制结构
 *  \param [in] clear           清除进度指示器如果为真
 *
 *  \return 如果进度未完成，则为零，否则为零
 */
int drawProgress( struct progress *o, bool clear ) {
	const int steps = 25, height = 4, x = SSD1306_LCDWIDTH - steps - 1, y = 15;
	int res = 0;

	// Clear indicator?
	if ( clear != false ) {
		display.fillRect( x - 1, y - 1, steps + 2, height + 2, BLACK );
	} else
		// Does indicator initialized?
		if ( !( o->opt & PGR_INIT ) ) {
			display.drawRect( x - 1, y - 1, steps + 2, height + 2, WHITE );
			display.fillRect( x, y, steps, height,
			                  o->opt & PGR_ON ? BLACK : WHITE );
			display.display();
			o->step = 1;
			o->millis = millis();
			o->opt |= PGR_INIT;
		} else
			// Drawing progress if any
			if ( millis() >= ( o->millis + o->step * ( o->time / steps ) ) ) {
				display.fillRect( x, y, o->step, height,
				                  o->opt & PGR_ON ? WHITE : BLACK );
				display.display();
				o->step = (int)( millis() - o->millis ) / ( o->time / steps );
				res = o->step > steps;
			}

	return res;
}

/**
 *  \brief      产生焊接脉冲。
 *  \remarks    发送预热脉冲，延迟，发送主焊接脉冲，延迟，
 *              然后增加焊接计数。
 *  \param [in] uint8_t  sensePin         脉冲接合传感器引脚
 *  \param [in] uint16_t delayEngage      脉冲接合前延迟（触发延迟）
 *  \param [in] uint16_t delayRelease     脉冲脱离后的延迟（重新触发延迟）
 *  \param [in] boolean  senseActiveLevel 传感器引脚逻辑。默认 = PL_ACTIVE_H。
 */
void sendWeldPulse( uint8_t sensePin,
                    uint16_t delayEngage,
                    uint16_t delayRelease,
                    boolean senseActiveLevel ) {
	struct progress wait;
#ifdef _DEVELOPMENT_
		Serial.println( F( "Pulse Activated" ) );
#endif /* _DEVELOPMENT_ */
	bool activePinState = senseActiveLevel != PL_ACTIVE_H ? 0 : 1;

	// 计算短脉冲延迟并将其剪辑为大于或等于 1 ms。
	unsigned long shortPulseDelay = ( pData.pulseTime * pData.shortPulseTime ) / 100;

	if ( shortPulseDelay < 1 ) shortPulseDelay = 1;

	// 等待脉冲接合延迟时间。
	if ( sensePin == PIN_AUTO_PULSE ) {
		wait.opt = PGR_ON;
		wait.time = delayEngage;

		while ( !drawProgress( &wait, false ) ) {
			if ( digitalRead( PIN_AUTO_PULSE ) != activePinState ) {
				drawProgress( &wait, true );
				return;
			}
		}
	} else {
		delay( delayEngage );
	}

if (pData.pFlags.en_Sound) tone(PIN_BUZZ, 1500, 100); //焊接脉冲激活声音

	if ( pData.shortPulseTime > 0 && pData.pulseTime > 3 ) {
		// 发送预热脉冲。
		weldPulse( P_ON );
		delay( shortPulseDelay );
		weldPulse( P_OFF );

		// 等待脉冲间延迟时间。
		delay( shortPulseDelay );
	}

	// 发送主焊接脉冲。
	weldPulse( P_ON );
	delay( pData.pulseTime );
	uint16_t battery = analogRead( PIN_BATT_V );
	weldPulse( P_OFF );

	checkForPulseDataEvent( battery );

	unsigned long currentMillis = millis();

	// 等到脉冲接合传感器停用。
	do {
		weldPulse( P_OFF );
	} while ( digitalRead( sensePin ) == activePinState && ( millis() - currentMillis < PV_DELAY ) );

	delay( 10 );

	if ( millis() - currentMillis > PV_DELAY )
		mEvent = EV_PULSE_VOLTS;

	// 延迟焊接脉冲才能再次触发。
	wait.opt = PGR_OFF;
	wait.time = delayRelease;

	while ( !drawProgress( &wait, false ) )
		continue;

	// 增加焊接计数器和标志活动。
// if (pData.PulseBatteryVoltage > 0) pData.weldCount++;
    pData.weldCount++;
	  lastActiveTime = millis();

	if ( !digitalRead( PIN_FOOT_SWITCH ) )
		mEvent = EV_PULSE_VOLTS;
}
/**
 *  \brief    检查电池电压并响应结果。
 *  \remarks  从模拟引脚读取电池电压并将其传递给内部引脚
 *            由存储的校准偏移调整的表示。如果电压低于
 *            报警设置，然后发出低电压事件。
 */

void checkBatteryVoltageInstant() {
  
  batteryVoltage = (uint8_t)( analogRead( PIN_BATT_V ) * 5.0 * 4.03 * 10.0 / 1023.0 );
}

void checkForLowVoltageEvent() {

	static unsigned long lastBVTime = 0;

	// 不要经常读取电池电压。
	if ( millis() - lastBVTime > BV_INTERVAL ) {
		lastBVTime = millis();

		// 通过读取 a/d 转换器、缩放结果和
		// 添加存储的偏移电压。结果是电压 * 10
		batteryVoltage = (uint8_t)( analogRead( PIN_BATT_V ) * 5.0 * 4.03 * 10.0 / 1023.0);


#ifdef _TESTING_
			// 忽略低电量进行 UI 测试。这可以防止低压锁定
      // 在未连接 12V 电池的情况下使用 USB 电源运行时。
			batteryVoltage = DEF_NOM_BATT_V + pData.batteryOffset;
#endif /* _TESTING_ */

		// 如果电池电压低，则发出低电压事件。
		if ( pData.batteryAlarm > batteryVoltage ) mEvent = EV_BATT_LV;

      // 如果电池电压过高，则发出高压事件。
    if ( pData.batteryhighAlarm < batteryVoltage ) mEvent = EV_BATT_HV;

		// 计算要显示在状态行上的电池电量计段的数量。
		batt_gauge = (int8_t)( ( ( ( (int16_t)( batteryVoltage ) * 10 - (int16_t)( pData.batteryAlarm ) * 11 +
		                             DEF_MAX_BATT_V ) * 10 ) / ( DEF_MAX_BATT_V - (int16_t)( pData.batteryAlarm ) ) - 5 ) / 20 );
		batt_gauge = batt_gauge < 1 ? 0 : batt_gauge > 5 ? 5 : batt_gauge;
	}
}


/**
 *  \brief    检查设备温度并响应结果。
 *  \remarks  通过连接到模拟引脚的 NTC 电阻读取温度。温度以摄氏度存储
 */

void checkTemp ()
{
  
double TdCelsius = 0;                        
int bitwertNTC = 0;
double widerstandNTC =0;

  static unsigned long lastTTime = 0;

  // 不要经常读取电池电压。
  if ( millis() - lastTTime > T_INTERVAL ) {
    lastTTime = millis();

// 从模拟输入中读取模拟值并计算 NTC 的电阻
  bitwertNTC = analogRead(PIN_TEMP);       
  widerstandNTC = 10000*(((double)bitwertNTC/1024)/(1-((double)bitwertNTC/1024))); //10000 = nominal NTC resistor value , 

// 以摄氏度计算温度
  TdCelsius = (1/((1/298.15)+((double)1/3900)*log((double)widerstandNTC/10000)))-273.15;  //273.15 = 0° Celsius in kelvin , 298.15 = 273.15 + 25, where 25 is the nominal temperature of the NTC, 3900 = B-value of the NTC
  TCelsius = uint8_t(TdCelsius+0.5); //round towards closest full number and convert to integer   

//如果温度太高，则发出高温事件
    if ( TCelsius > DEF_HIGH_TEMP_ALARM ) mEvent = EV_TEMP_HIGH;   
  }
}


void checkForPulseDataEvent( uint16_t battery ) {
	pData.PulseBatteryVoltage = (uint16_t)( battery * 5.0 * 4.03 * 10.0 /
	                                        1023.0 + pData.batteryOffset );
                                         
// 如果电压脉冲高于 10V，这将使电压和安培读数为 0。100 等于 10.0V 测量电压，                                       
// if (pData.PulseBatteryVoltage > 100) pData.PulseBatteryVoltage = 0; 

	// 117 = 1/0.0085 --- where 0.0085 eqauls 8.5mOhm system resistance
	pData.PulseAmps = (uint16_t)( pData.PulseBatteryVoltage * 117 );
}

/**
 *  \brief    如果待机时间已过期而没有任何活动，则发出待机超时事件。
 *  \remarks  待机超时时间在文件头中以 ms 为单位定义。
 */
void checkForSleepEvent() {
	// 每次发生某些活动时都会更新上次活动时间。如果待机超时
  // 周期已过期，没有任何活动，然后发出超时事件。
	if ( lastActiveTime + STANDBY_TIME_OUT < millis() )
		if ( mState != ST_BATTERY_LOW ) mEvent = EV_STBY_TIMEOUT;
}
/**
 *  \brief    读取旋转编码器按钮开关转换事件。
 *  \remarks  根据是否已按下开关发出开关转换事件
*             防抖间隔是否已过期或开关是否已释放。
 */
void checkForBtnEvent() {
	static unsigned long lastBtnTime = 0;
	static boolean lastBtnState = B_UP;
	boolean thisBtnState;

	thisBtnState = btnState();

	// 忽略防抖期间发生的更改。
	if ( millis() - lastBtnTime > RS_DEBOUNCE ) {

		// 只对比顿状态的变化做出反应。
		if ( thisBtnState != lastBtnState ) {
			// 根据按钮的当前状态发出事件。
			mEvent = thisBtnState == B_UP ? EV_BTNUP : EV_BTNDN;
      
			lastActiveTime = lastBtnTime = millis();
			lastBtnState = thisBtnState;
		}
	}
}


/***************************************************************************************************
* 蜂鸣器声音例程                                                                      *
***************************************************************************************************/
/**
 *  \brief    Buzzer sound routines.
 *  \remarks  functions to create the different buzzer sounds
 */
 
void Boot_Sound() {
tone(PIN_BUZZ, 1000, 200);
delay(100); 
tone(PIN_BUZZ, 2000, 200);
delay(100); 
tone(PIN_BUZZ, 3000, 200);
delay(100); 
tone(PIN_BUZZ, 2000, 200);
}

void LowBattery_Sound() {
  unsigned long previousMillis = 0; 
  const long interval = 2000; 

  while ( btnState() != B_DN) {    
unsigned long currentMillis = millis(); 
  if (currentMillis - previousMillis >= interval) { 
     previousMillis = currentMillis;
tone(PIN_BUZZ, 1000, 200);
    }
  }
} 

void FootSwitch_Alarm() {
  while (1) {    
tone(PIN_BUZZ, 1000, 200);
delay(1000);
    }
  }

/***************************************************************************************************
* 中断服务程序                                                                      *
***************************************************************************************************/
/**
 *  \brief    中断服务程序。
 *  \remarks  响应来自旋转编码器的中断。管理和跟踪旋转编码器
 *            位置变化。根据是否发出编码器位置转换事件
 *            编码器顺时针（向上）或逆时针（向下）旋转。
 */
void isr() {
	static volatile unsigned long lastInterruptTime = 0;

	// Ignore changes that occur within the debounce period.
	if ( ( millis() - lastInterruptTime ) > RS_DEBOUNCE ) {

		// There are two pins to the encoder. One pin issues the interrupt. The other pin
		// therefore indicates the phase, which is effectively the direction of rotation.
		// Read the phase pin and issue an encoder direction event based on its state.
		mEvent = digitalRead( PIN_DT ) ? EV_ENCDN : EV_ENCUP;
		lastActiveTime = lastInterruptTime = millis();
	}
}

/***************************************************************************************************
* Menu management routines                                                                         *
***************************************************************************************************/
/**
 *  \brief      Displays a menu type 1 on the LCD.
 *  \remarks    This is a three line selection menu. If the title pointer is NULL then a standard
 *              status line is drawn otherwise the title is drawn on the screen.
 *  \param [in] const __FlashStringHelper *title       Menu title.
 *  \param [in] const __FlashStringHelper *line1       Line 1 text.
 *  \param [in] const __FlashStringHelper *line2       Line 2 text.
 *  \param [in] const __FlashStringHelper *line3       Line 3 text.
 *  \param [in] uint8_t                   SelectedItem Selected menu item.
 */
void displayMenuType1( const __FlashStringHelper *title,
                       const __FlashStringHelper *line1,
                       const __FlashStringHelper *line2,
                       const __FlashStringHelper *line3,
                       uint8_t SelectedItem ) {

	display.clearDisplay();

	if ( title == NULL )
		drawStatusLine();
	else {
		setTextProp( 1, 1, 1, WHITE );
		display.print( title );
		display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );
	}

	setTextProp( 2, 2, 16, WHITE, SelectedItem == 0 );
	display.print( line1 );
	setTextProp( 2, 2, 16 + 2 * CHR_H + 1, WHITE, SelectedItem == 1 );
	display.print( line2 );
	setTextProp( 2, 2, 16 + 4 * CHR_H + 2, WHITE, SelectedItem == 2 );
	display.print( line3 );
	display.drawRect( 0, SelectedItem == 0 ? 16 :
	                  SelectedItem == 1 ? 16 + 2 * CHR_H + 1 :
	                  16 + 4 * CHR_H + 2, 2, 2 * CHR_H, WHITE );
	display.display();
}
/**
 *  \brief      Displays a menu type 2 on the LCD.
 *  \remarks    This is a variable display and adjustment dialog. Normally the second parameter
 *              is a pointer to a string value in ram and the third parameter is a pointer to the
 *              units string in flash. If the second parameter is NULL then the third parameter
 *              is treated as the second parameter. This enables the display of two adjacent
 *              flash based strings.
 *  \param [in] const __FlashStringHelper *title  Menu title.
 *  \param [in] const char                *value  Item value.
 *  \param [in] const __FlashStringHelper *units  Item units.
 */
void displayMenuType2( const __FlashStringHelper *title,
                       const char *value,
                       const __FlashStringHelper *units ) {

	display.clearDisplay();

	setTextProp( 1, 1, 1, WHITE );
	display.print( title );
	display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );
	setTextProp( 2, 2, 16 + LINE_H );

	if ( value == NULL ) display.print( units );
	else {
		display.print( value );
		setTextProp( 1, 2, 16 + 3 * LINE_H );
		display.print( units );
	}

	display.display();
}

/***************************************************************************************************
* Display management routines                                                                      *
***************************************************************************************************/
/**
 *  \brief                      Sets the properties of a text string to be drawn to the display.
 *  \param [in] uint8_t  size   Text size.
 *  \param [in] uint8_t  xpos   Text x-coordinate in pixels from left.
 *  \param [in] uint8_t  ypos   Text x-coordinate in pixels from top.
 *  \param [in] uint16_t color  Text foreground  color (default WHITE).
 *  \param [in] boolean  invert True if colors are to be inverted (bg=color,fg=BLACK) default false.
 */
void setTextProp( uint8_t size, uint8_t xpos, uint8_t ypos, uint16_t color, boolean invert ) {

	display.setTextSize( size );
	display.setCursor( xpos, ypos );

	if ( invert )
		display.setTextColor( BLACK, color );
	else
		display.setTextColor( color );
}

/**
 *  \brief    Draws a status line to the LCD.
*  \remarks  状态线显示脉冲触发模式和执行的焊接总数。
* ...........在双色显示器上，状态线位于顶部彩色显示区域内。一个
* ...........在状态线下方绘制下划线。包含 5 个段的电池电量计是
* ...........绘制的每个段是从电池报警电压到
* ...........定义的电池最大电压。要绘制的段数在
* ...........检查电池电压的程序。如果电池低于 10%，则十字架是
* ...........在电池内部绘制。
 */
void drawStatusLine() {
	char str[8];

	setTextProp( 1, 1, 1 );
	display.print( pData.pFlags.en_autoPulse ? FPSTR( LS_AUTO_BAR ) : FPSTR( LS_MANUAL_BAR ) );
	setTextProp( 1, 8 * CHR_W, 1 );
	display.print( valStr( str, pData.weldCount, VF_WELDCNT ) ); //在状态栏中显示焊接计数
//	display.print( valStr( str, batteryVoltage, VF_BATTV ) ); //在状态栏中显示电压而不是焊接计数
	setTextProp( 1, 13 * CHR_W, 1 );
	display.print( FPSTR( LS_WELDS ) );  //在状态栏中显示焊接计数
//	display.print( FPSTR( LS_VUNITS ) ); //在状态栏中显示电压而不是焊接计数
	// 在状态行下划下划线。
	display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );
	// 绘制电池电量表骨架。这只是一个空矩形和另一个小矩形
	// 左侧的矩形代表电池端子。
	display.drawRect( SSD1306_LCDWIDTH - 33, 0, 33, CHR_H, WHITE );
	display.drawRect( SSD1306_LCDWIDTH - 35, ( CHR_H - 1 - 2 ) / 2, 2, 4, WHITE );

	// 绘制电池电量表段。每个段都是一个填充的矩形。
	if ( batt_gauge )
		for ( int8_t n = 0 ; n < batt_gauge ; n++ )
			display.fillRect( SSD1306_LCDWIDTH - 7 - 6 * n, 2, 5, 4, WHITE );

	else {
		// 如果电池电量低于 15%，则画一个十字以指示它是空的。
		display.drawLine( SSD1306_LCDWIDTH - 33,     0,   SSD1306_LCDWIDTH - 1, CHR_H - 1, WHITE );
		display.drawLine( SSD1306_LCDWIDTH - 33, CHR_H - 1, SSD1306_LCDWIDTH - 1,     0,   WHITE );
	}
}

/**
 *  \brief  将主屏幕绘制到 LCD。
 */
void displayMainScreen( bool signaled ) {
	char str[5];

	display.clearDisplay();
	drawStatusLine();

	// 写入焊接脉冲持续时间的当前值及其单位。
	setTextProp( 4, 2, 13 + CHR_H / 2 );
	display.print( valStr( str, pData.pulseTime, VF_PLSDLY ) );
	setTextProp( 2, 4 * CHR_W * 4 + CHR_W, 12 + CHR_H / 2 + 4 * CHR_H - 2 * CHR_H - 2, WHITE, signaled );
	display.print( FPSTR( LS_MS ) );
  // 下划线
	display.drawLine( 0, CHR_H + 42, SSD1306_LCDWIDTH - 1, CHR_H + 42, WHITE );
	// 在值下写一条消息。
	setTextProp( 1, ( SSD1306_LCDWIDTH - ( sizeof( LS_WPDRN ) - 1 ) * CHR_W ) / 2, 16 + CHR_H / 2 + 4 * CHR_H - 2 * CHR_H - 2 + 2 * LINE_H );
  display.print( FPSTR( LS_WPDRN ) );
	
	display.display();
}

void displayMainScreen() {
	displayMainScreen( false );
}

/**
 *  \brief  Display pulse voltage message and battery voltage measured during pulse to LCD.
 */

void displayPulseData() {
	char str[5];

	display.clearDisplay();
	drawStatusLine();

	setTextProp( 1, uint8_t( ( SSD1306_LCDWIDTH - ( sizeof( LS_PULSEV ) - 1 ) * CHR_W * 2 ) / 2 ), 16 );
	display.print( FPSTR( LS_PULSEV ) );
	setTextProp( 1, ( SSD1306_LCDWIDTH - 5 * CHR_W ) / 2, 16 + 2 * LINE_H );
	display.print( valStr( str, pData.PulseBatteryVoltage, VF_BATTV ) );
	display.print( FPSTR( LS_VUNITS ) );

	setTextProp( 1, 1, 16 + 4 * LINE_H );
	display.print( FPSTR( LS_PULSEA ) );
	setTextProp( 1, 57, 16 + 4 * LINE_H );
	display.print( valStr( str, pData.PulseAmps, VF_BATTA ) );
	display.print( FPSTR( LS_AUNITS ) );

	display.display();
}

/**
 *  \brief  Display LOW BATTERY message and BATTERY voltage to LCD.
 */

void displayLowBattery() {
	char str[5];

	display.clearDisplay();
	drawStatusLine();

	//setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_BATTERY ) - 1 ) * CHR_W * 2 ) / 2, 16 );
	//display.print( FPSTR( LS_BATTERY ) );
	setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_LOWV ) - 1 ) * CHR_W * 2 ) / 2, 16 + 2 * LINE_H );
	display.print( FPSTR( LS_LOWV ) );
	setTextProp( 1, ( SSD1306_LCDWIDTH - 1 * CHR_W ) / 2, 16 + 4 * LINE_H );
	display.print( valStr( str, batteryVoltage, VF_BATTV ) );
	display.print( FPSTR( LS_VUNITS ) );

	display.display();
}

void displayHighBattery() {
  char str[5];

  display.clearDisplay();
  drawStatusLine();

  //setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_BATTERY ) - 1 ) * CHR_W * 2 ) / 2, 16 );
  //display.print( FPSTR( LS_BATTERY ) );
  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_HIGHV ) - 1 ) * CHR_W * 2 ) / 2, 16 + 2 * LINE_H );
  display.print( FPSTR( LS_HIGHV ) );
  setTextProp( 1, ( SSD1306_LCDWIDTH - 1 * CHR_W ) / 2, 16 + 4 * LINE_H );
  display.print( valStr( str, batteryVoltage, VF_BATTV ) );
  display.print( FPSTR( LS_VUNITS ) );

  display.display();
}

void displayHighTemperature() {
  char str[5];

  display.clearDisplay();
  drawStatusLine();

  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_HIGHT ) - 1 ) * CHR_W * 2 ) / 2, 16 );
  display.print( valStr( str, TCelsius, VF_TEMP ) );
  display.print( FPSTR( LS_TUNITS ) );
  setTextProp( 2, ( SSD1306_LCDWIDTH - ( sizeof( LS_HIGHT ) - 1 ) * CHR_W * 2 ) / 2, 16 + 2 * LINE_H );
  display.print( FPSTR( LS_HIGHT ) );
  setTextProp( 1, ( SSD1306_LCDWIDTH - ( sizeof( LS_COOL )- 1 )* CHR_W ) / 2, 16 + 4 * LINE_H );
  display.print( FPSTR( LS_COOL ) );

  display.display();
}

/**
 *  \brief    Display a MESSAGE screen.
 *  \remarks  A message screen consists of a screen header with underline and 3 rows of text.
 *            The first row is double size font, other rows are normal size.
 *            The message screen is displayed for a specified number of seconds before
 *            returning to the caller. The default time is zero for immediate return.
 *  \param [in] const __FlashStringHelper* line1        Line 1 text.
 *  \param [in] const __FlashStringHelper* line1        Line 2 text.
 *  \param [in] const __FlashStringHelper* line3        Line 3 text.
 *  \param [in] uint8_t                    displayTime  Delay time (default = 0)
 */
void message( const __FlashStringHelper *line1,
              const __FlashStringHelper *line2,
              const __FlashStringHelper *line3,
              uint8_t displayTime ) {

	display.clearDisplay();

	setTextProp( 1, ( SSD1306_LCDWIDTH - ( sizeof( LS_MSGHDR ) - 1 ) * CHR_W ) / 2, 1 );
	display.print( FPSTR( LS_MSGHDR ) );
	display.drawLine( 0, CHR_H + 3, SSD1306_LCDWIDTH - 1, CHR_H + 3, WHITE );

	setTextProp( 2, 1, 16 );
	display.print( line1 );
	setTextProp( 1, 1, 16 + 2 * LINE_H );
	display.print( line2 );
	setTextProp( 1, 1, 16 + 3 * LINE_H );
	display.print( line3 );

	display.display();

	if ( displayTime ) delay( displayTime * 1000 );
}

/**
 *  \brief    Display the SPLASH screen.
 *  \remarks  This is shown once only at start-up and is for general information and advertising.
 */
void splash() {
	display.clearDisplay();
static const unsigned char PROGMEM logo_bmp[] =
{
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFC,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XF8,0X0F,0XE0,0X78,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0X0F,0X00,0X3F,0XFF,
0XFF,0XE0,0X03,0XE0,0X70,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0X1F,0X00,0X07,0XFF,
0XFF,0XC1,0XE1,0XE0,0X70,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0X1F,0X00,0X03,0XFF,
0XFF,0X8F,0XF8,0XE0,0X78,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0X1F,0X00,0X01,0XFF,
0XFF,0X1F,0X1C,0X60,0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0X3F,0X00,0X00,0XFF,
0XFF,0X1C,0X0E,0X60,0X70,0X7C,0X0C,0X1F,0XE0,0X3F,0XE0,0XC0,0X3F,0X00,0X00,0X7F,
0XFF,0X38,0X06,0X60,0X70,0X78,0X00,0X07,0XC0,0X1F,0XC0,0XF0,0X7F,0X03,0XE0,0X7F,
0XFE,0X30,0X06,0X60,0X70,0X70,0X00,0X07,0X80,0X0F,0X80,0XE0,0X7F,0X03,0XE0,0X7F,
0XFE,0X30,0XC6,0X60,0X70,0X70,0X00,0X07,0X00,0X07,0X80,0XE0,0X1F,0X03,0XF0,0X7F,
0XFE,0X31,0XC6,0X60,0X70,0X70,0X00,0X03,0X07,0X07,0X80,0XF8,0X1F,0X03,0XF0,0X7F,
0XFE,0X30,0X04,0X60,0X70,0X70,0X41,0X83,0X04,0X07,0X83,0XFC,0X0F,0X03,0XE0,0X7F,
0XFF,0X30,0X84,0XE0,0X70,0X70,0X41,0X83,0X00,0X07,0X03,0XFC,0X0F,0X03,0XE0,0X7F,
0XFF,0X10,0X80,0XE0,0X70,0X70,0X41,0X83,0X07,0XFF,0X03,0XFC,0X0F,0X03,0X80,0X7F,
0XFF,0X1D,0XC3,0XE0,0X10,0X70,0X41,0X83,0X07,0XFC,0X07,0XC0,0X0F,0X00,0X00,0XFF,
0XFF,0X8F,0XFF,0XF0,0X10,0X70,0X41,0X83,0X00,0X7C,0X07,0XC0,0X0F,0X00,0X00,0XFF,
0XFF,0XC1,0XF3,0XF0,0X10,0X70,0X41,0X83,0X80,0X3C,0X07,0XC0,0X1F,0X00,0X01,0XFF,
0XFF,0XE0,0X03,0XF8,0X10,0X70,0X41,0X83,0XC0,0X1C,0X0F,0XC0,0X3F,0X00,0X03,0XFF,
0XFF,0XF8,0X03,0XFC,0X10,0X70,0X41,0X83,0XE0,0X3C,0X3F,0XC0,0X7F,0X00,0X0F,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X10,0X08,0X00,0XC0,0X00,0X08,0X0C,0X04,0X10,0X00,0X00,0X12,0X01,0X80,0X00,
0X00,0X11,0X88,0X3F,0XFF,0X00,0X14,0X28,0X04,0X18,0X00,0X00,0X12,0X03,0XFE,0X00,
0X00,0X30,0XC8,0X06,0X18,0X03,0X34,0X28,0X0D,0XFF,0X81,0X1F,0XFF,0X86,0X04,0X00,
0X00,0X7C,0X68,0X06,0X18,0X02,0X65,0XEF,0X08,0X00,0X03,0X18,0X18,0X0C,0X0C,0X00,
0X00,0X30,0X08,0X06,0X18,0X06,0X4F,0XF9,0X18,0XFF,0X03,0X10,0X10,0X1F,0XFF,0X00,
0X00,0X11,0X08,0X3F,0XFF,0X84,0X10,0X19,0X18,0X00,0X02,0X1F,0XF9,0X04,0X21,0X00,
0X00,0X11,0X88,0X00,0X00,0X04,0X10,0X1A,0X38,0X00,0X06,0X10,0X0B,0X04,0X21,0X00,
0X00,0X3C,0XC8,0X00,0X00,0X0C,0X30,0X0A,0X28,0XFF,0X04,0X10,0X0A,0X07,0XFF,0X00,
0X00,0XF0,0X08,0X0F,0XFC,0X08,0X70,0X0E,0X08,0X00,0X04,0X13,0XCA,0X04,0X21,0X00,
0X00,0X10,0X0F,0X08,0X04,0X18,0X53,0XC6,0X08,0X00,0X0C,0X10,0X4E,0X04,0X21,0X00,
0X00,0X13,0XFC,0X08,0X04,0X18,0X12,0X44,0X08,0XFF,0X08,0X10,0X4C,0X07,0XFF,0X00,
0X00,0X10,0X08,0X08,0X04,0X10,0X12,0X66,0X08,0X81,0X18,0X12,0X4C,0X80,0X00,0X00,
0X00,0X10,0X08,0X08,0X04,0X30,0X16,0X6A,0X08,0X81,0X18,0X13,0XDC,0X80,0X00,0X00,
0X00,0X30,0X08,0X0F,0XFC,0X20,0X14,0X59,0X08,0XFF,0X10,0X30,0X37,0X9F,0XFF,0XC0,
0X00,0X70,0X08,0X08,0X04,0X00,0X14,0X11,0X08,0X81,0X00,0X20,0X23,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};
  display.drawBitmap(0,5,logo_bmp, 128, 55, 1);
  display.display();  // 要有這行才會把文字顯示出來
  delay(2000); // 停1秒

}

void foot_switch_error() {
	display.clearDisplay();
	display.display();

	setTextProp( 1, 1, 1 );
	display.print( "FOOT SWITCH ERROR!" ); // 21 chars per line
	setTextProp( 1, 1, 16 );
	display.print( F( "Please:" ) );
	setTextProp( 1, 1, 16 + LINE_H );
	display.print( F( "- turn off welder" ) );
	setTextProp( 1, 1, 16 + 2 * LINE_H );
	display.print( F( "- remove foot switch" ) );
	setTextProp( 1, 1, 16 + 3 * LINE_H );
	display.print( F( "- correct the wiring" ) );

	display.display();

	while ( !digitalRead( PIN_FOOT_SWITCH ) );
}

/***************************************************************************************************
* Utility Conversion Functions                                                                     *
***************************************************************************************************/
/**
 *  \brief                    Returns a character string representing a formatted numeric value.
 *  \param [in] char *str     Pointer to the string to receive the formatted result.
 *  \param [in] uint16_t val  The integer value to be formatted.
 *  \param [in] vf_Type fType The type of variable to be formatted.
 *  \return     char*         Pointer to the formatted string.
 */
char *valStr( char *str, uint16_t val, vf_Type fType ) {

	// We must resort to this awkward kludge to format the variables because variable width and
	// precision (* specifier) are not implemented in avr gcc - bugger!!!

	switch ( fType ) {
	case VF_BATTALM:
	case VF_BATTV:   sprintf_P( str, PSTR( "%2.1u.%01u" ), val / 10, val % 10 ); break;
	case VF_BATTA:   sprintf_P( str, PSTR( "%2.1u.%01u" ), val / 10, val % 10 ); break;
	case VF_WELDCNT: sprintf_P( str, PSTR( "%5.1u" ), val );                 break;
  case VF_TEMP:    sprintf_P( str, PSTR( "%5.1u" ), val );                 break;
	case VF_PLSDLY:  sprintf_P( str, PSTR( "%4.1u" ), val );                 break;
	case VF_SHTPLS:  sprintf_P( str, PSTR( "%3.1u" ), val );                 break;
	case VF_DELAY:   sprintf_P( str, PSTR( "%1.1u.%01u" ), val / 10, val % 10 ); break;
	}

	return str;
}

/***************************************************************************************************
* Utility EEPROM Functions                                                                         *
***************************************************************************************************/
/**
 *  \brief                    Reset the EEPROM and program data to factory default settings.
 *  \remarks                  EEPROM data is only written if the new data is different to the
 *                            existing data to limit EEPROM wearout.
 *  \param [in] boolean full  True to reset the weld count, battery offset, and screen inversion.
 */
void resetEeprom( boolean full ) {

  // Completely erase all data from the eeprom
for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }

	// Write the factory default data to the EEPROM. In the case of a full reset, the weld count
	// and screen orientation are zeroed, otherwise they are left unchanged.
	pData.autoPulseDelay       = DEF_AUTO_PLSDELAY;
	pData.PulseBatteryVoltage  = DEF_PULSE_VOLTAGE;
	pData.PulseAmps            = DEF_PULSE_AMPS;
	pData.batteryAlarm         = DEF_BATT_ALARM;
  pData.batteryhighAlarm     = DEF_HIGH_BATT_ALARM;
	pData.weldCount            = full == EE_FULL_RESET ? 0 : pData.weldCount;
	pData.pulseTime            = DEF_PULSE_TIME;
	pData.maxPulseTime         = DEF_MAX_PULSE_TIME;
	pData.shortPulseTime       = DEF_SPULSE_TIME;
	pData.pFlags.en_autoPulse  = DEF_AUTO_PULSE;
  pData.pFlags.en_Sound      = DEF_WELD_SOUND;
	pData.pFlags.en_oledInvert = full ? DEF_OLED_INVERT : pData.pFlags.en_oledInvert;

	// The put function does not write new data if the existing data is the same thereby
	// limiting eeprom wearout.
	EEPROM.put( EEA_PDATA, pData );

	// The unique id is a simple method to ensure that a valid data set exists in the eeprom
	// (there are much better methods but we don't have the code space to spare).
	EEPROM.put( EEA_ID, EE_UNIQUEID );

#if defined  _DEVELOPMENT_ || defined _BOOTSYS_

		if ( full ) Serial.print( F( "EEPROM Full Reset" ) );
		else Serial.println( F( "EEPROM Reset" ) );

#endif /* _DEVELOPMENT_ || _BOOTSYS_*/
}

void loadEeprom() {
	// Check the eeprom integrity by reading a magic number. If it is corrupt then the eeprom
	// is given a full factory reset, otherwise program data is loaded from the eeprom.
	uint32_t uniqueID;

	EEPROM.get( EEA_ID, uniqueID );

	if ( uniqueID != EE_UNIQUEID )
		resetEeprom( EE_FULL_RESET );
	else
		EEPROM.get( EEA_PDATA, pData );
}
/**
 *  \brief    Udates the EEPROM data with local program data structure.
 *  \remarks  EEPROM data is only written if the new data is different to the
 *            existing data to limit EEPROM wearout.
 */
void updateEeprom() {
	static unsigned long lastEEUpdatetime = 0;

	// Do not do this too often to prevent premature eeprom wearout.
	if ( millis() - lastEEUpdatetime > EEPROM_UPDATE_T ) {
		lastEEUpdatetime = millis();

		// Write the current program data to the eeprom.
		EEPROM.put( EEA_PDATA, pData );

#ifdef _DEVELOPMENT_
			Serial.println( F( "Updated EEPROM" ) );
#endif /* _DEVELOPMENT_ */
	}
}
