#include <Arduino.h>
#include "src/ESP32-BLE-Gamepad-0.7.3/BleGamepad.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "ws2812.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  #define BLE_MAX_TX_POWER 20 //ESP_PWR_LVL_P20, available only on c3/c6 and some others
#else
  #define BLE_MAX_TX_POWER 9 //ESP_PWR_LVL_P9
#endif

#define BATTERY_ADC_UNIT ADC_UNIT_1
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 //gpio 0 on esp32c3
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT

#define BATTERY_FULL_MV 4200
#define BATTERY_EMPTY_MV 3100
#define BATTERY_VERY_EMPTY_MV 2900

#define PIN_BUILTIN_LED 8

#define PIN_WS2812_LED_VCC 6
#define PIN_WS2812_LED_DATA 7

//buttons are active-low with pullups

//STAR is also used as BOOT button and PAIR button - this a dedicated bootstrap pin on esp32c3
//when MCU is powered up (not waken from sleep) with BOOT button pressed it'll go straight to the ROM bootloader
#define PIN_STAR_BOOT 9 

//MENU is also used as POWER button - this is one of RTC deepsleep wakeup-capable pins on esp32c3
#define PIN_MENU_POWER 5

//i have 2 guitars but the wiring for the buttons 
//other than STAR and MENU is random - 
//i havent marked the wires and just fixed this later by following defines
//so both controllers have the same layout seen by clone hero

#define CLONE_HERO_BLUE

#ifdef CLONE_HERO_BLUE
  #define PIN_STRUM_UP 21
  #define PIN_STRUM_DOWN 20
  #define PIN_FRET_1 3
  #define PIN_FRET_2 10
  #define PIN_FRET_3 2
  #define PIN_FRET_4 1
  #define PIN_FRET_5 4

  #define DEVICE_NAME "Clone Hero Blue"
#else  
  #define PIN_STRUM_UP 20
  #define PIN_STRUM_DOWN 21
  #define PIN_FRET_1 10
  #define PIN_FRET_2 4
  #define PIN_FRET_3 3
  #define PIN_FRET_4 1
  #define PIN_FRET_5 2

  #define DEVICE_NAME "Clone Hero Green"
#endif

BleGamepad bleGamepad(DEVICE_NAME, "thisiseth");
Ws2812 rgbLed(PIN_WS2812_LED_DATA);

adc_oneshot_unit_handle_t battery_adc_handle;
adc_cali_handle_t battery_adc_cali_handle;

enum class RgbLedState
{
  None,
  NotConnected,
  Connected,
  BatteryLow,
  BondsDeleted,
  //
};

RgbLedState ws2812_state = RgbLedState::None;

void goToSleep()
{
  if (bleGamepad.disconnectAll())
    delay(500);

  esp_deep_sleep_enable_gpio_wakeup(BIT(PIN_MENU_POWER), ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

void battery_adc_init()
{
  adc_oneshot_unit_init_cfg_t init_config = 
  {
    .unit_id = BATTERY_ADC_UNIT,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  
  adc_oneshot_new_unit(&init_config, &battery_adc_handle);

  adc_oneshot_chan_cfg_t channel_config = 
  {
    .atten = BATTERY_ADC_ATTEN,
    .bitwidth = BATTERY_ADC_BITWIDTH,
  };

  adc_oneshot_config_channel(battery_adc_handle, BATTERY_ADC_CHANNEL, &channel_config);

  //esp32-c3 calibration
  adc_cali_curve_fitting_config_t cali_config = 
  {
    .unit_id = BATTERY_ADC_UNIT,
    .chan = BATTERY_ADC_CHANNEL,
    .atten = BATTERY_ADC_ATTEN,
    .bitwidth = BATTERY_ADC_BITWIDTH,
  };

  adc_cali_create_scheme_curve_fitting(&cali_config, &battery_adc_cali_handle);
}

int get_battery_mv()
{
  int adc_raw, adc_mv;

  adc_oneshot_read(battery_adc_handle, BATTERY_ADC_CHANNEL, &adc_raw);
  adc_cali_raw_to_voltage(battery_adc_cali_handle, adc_raw, &adc_mv);

  return adc_mv * 3; //2m:1m resistor divider
}

uint8_t get_battery_percent()
{
  auto mv = get_battery_mv();

  if (mv > BATTERY_FULL_MV)
    return 100;
  if (mv < BATTERY_EMPTY_MV)
    return 0;

  return (mv - BATTERY_EMPTY_MV)/((BATTERY_FULL_MV - BATTERY_EMPTY_MV) / 100); //just linear aprox.
}

void set_rgb_led_state(RgbLedState state)
{
  if (ws2812_state == state)
    return;

  ws2812_state = state;

  switch (state)
  {
    case RgbLedState::None:
      rgbLed.SetColor(0, 0, 0);
      break;
    case RgbLedState::NotConnected:
      rgbLed.SetColor(0, 3, 0);
      break;
    case RgbLedState::Connected:
      rgbLed.SetColor(2, 3, 3);
      break;
    case RgbLedState::BatteryLow:
      rgbLed.SetColor(3, 0, 0);
      break;
    case RgbLedState::BondsDeleted:
      rgbLed.SetColor(3, 3, 0);
      break;
  }
}

void update_battery_status()
{
  auto battery_percent = get_battery_percent();

  if (battery_percent == 0)
  {
    set_rgb_led_state(RgbLedState::BatteryLow);
    delay(1000);
    goToSleep();
  }

  bleGamepad.setPowerStateAll(
    POWER_STATE_PRESENT,
    POWER_STATE_DISCHARGING,
    POWER_STATE_NOT_CHARGING,
    battery_percent > 10 ? POWER_STATE_GOOD : POWER_STATE_CRITICAL);

  bleGamepad.setBatteryLevel(battery_percent);
}

void setup() 
{
  battery_adc_init();

  if (get_battery_mv() < BATTERY_VERY_EMPTY_MV)
    goToSleep();
    
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_GPIO)
    goToSleep();

  pinMode(PIN_MENU_POWER, INPUT_PULLUP);

  delay(50); //'debounce'

  while (millis() < 2000) //button should be held for some time to register power-on
    if (digitalRead(PIN_MENU_POWER))
      goToSleep();

  pinMode(PIN_BUILTIN_LED, OUTPUT);
  digitalWrite(PIN_BUILTIN_LED, 0);

  pinMode(PIN_WS2812_LED_VCC, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)PIN_WS2812_LED_VCC, GPIO_DRIVE_CAP_3);

  digitalWrite(PIN_WS2812_LED_VCC, 1);
  set_rgb_led_state(RgbLedState::NotConnected);

  pinMode(PIN_STAR_BOOT, INPUT_PULLUP);
  pinMode(PIN_STRUM_UP, INPUT_PULLUP);
  pinMode(PIN_STRUM_DOWN, INPUT_PULLUP);
  pinMode(PIN_FRET_1, INPUT_PULLUP);
  pinMode(PIN_FRET_2, INPUT_PULLUP);
  pinMode(PIN_FRET_3, INPUT_PULLUP);
  pinMode(PIN_FRET_4, INPUT_PULLUP);
  pinMode(PIN_FRET_5, INPUT_PULLUP);
  
  auto config = BleGamepadConfiguration();
  config.setButtonCount(9);
  config.setWhichAxes(false, false, false, false, false, false, false, false);
  config.setHatSwitchCount(0);
  config.setTXPowerLevel(BLE_MAX_TX_POWER);

  update_battery_status();
  bleGamepad.begin(&config);
}

uint8_t buttonsDebounce[9] = {0};
int32_t pressedButtons = 0;
int32_t reportedButtons = 0;

int32_t powerPressedAt = 0;
int32_t pairPressedAt = 0;
int32_t lastActivityAt = 0;
int32_t batteryReadAt = 0;

void loop() 
{
  delay(1);

  if ((millis() - batteryReadAt) > 30*1000)
  {
    update_battery_status();
    batteryReadAt = millis();
  }

  for (auto i = 0; i < 9; ++i)
    buttonsDebounce[i] <<= 1;

  //pressed = 0v, released = +3.3v
  buttonsDebounce[0] += !digitalRead(PIN_STAR_BOOT);
  buttonsDebounce[1] += !digitalRead(PIN_MENU_POWER);
  buttonsDebounce[2] += !digitalRead(PIN_STRUM_UP);
  buttonsDebounce[3] += !digitalRead(PIN_STRUM_DOWN);
  buttonsDebounce[4] += !digitalRead(PIN_FRET_1);
  buttonsDebounce[5] += !digitalRead(PIN_FRET_2);
  buttonsDebounce[6] += !digitalRead(PIN_FRET_3);
  buttonsDebounce[7] += !digitalRead(PIN_FRET_4);
  buttonsDebounce[8] += !digitalRead(PIN_FRET_5);

  auto previousButtons = pressedButtons;

  for (auto i = 0; i < 9; ++i) 
    if (buttonsDebounce[i] == 0xFF)
      pressedButtons |= (1 << i);
    else if (buttonsDebounce[i] == 0)
      pressedButtons &= ~(1 << i);

  if (previousButtons != pressedButtons)
    lastActivityAt = millis();

  auto powerWasPressed = !!(previousButtons & 0x2);
  auto pairWasPressed = !!(previousButtons & 0x1);
  auto powerIsPressed = !!(pressedButtons & 0x2);
  auto pairIsPressed = !!(pressedButtons & 0x1);

  if (!powerWasPressed && powerIsPressed)
    powerPressedAt = millis();
  else if (powerWasPressed && powerIsPressed && (millis() - powerPressedAt) > 5000) //5 seconds to power-off
  {
    delay(100);
    goToSleep();
  }

  if (!pairWasPressed && pairIsPressed)
    pairPressedAt = millis();
  else if (pairWasPressed && pairIsPressed && (millis() - pairPressedAt) > 3000) //3 seconds to drop connections
  {
    set_rgb_led_state(RgbLedState::BondsDeleted);
    bleGamepad.disconnectAll();
    bleGamepad.deleteAllBonds();
    pairPressedAt = millis();
    return;
  }
  
  if (bleGamepad.isConnected()) 
  {
    set_rgb_led_state(RgbLedState::Connected);

    for (auto i = 0; i < 9; ++i)
    {
      auto mask = 1 << i;

      if ((reportedButtons & mask) && !(pressedButtons & mask))
        bleGamepad.release(BUTTON_1 + i);
      else if (!(reportedButtons & mask) && (pressedButtons & mask))
        bleGamepad.press(BUTTON_1 + i);
    }

    reportedButtons = pressedButtons;

    if ((millis() - lastActivityAt) > 20*60*1000) //20 mins if connected
      goToSleep();
  }
  else
  {
    set_rgb_led_state(RgbLedState::NotConnected);

    reportedButtons = 0;

    if ((millis() - lastActivityAt) > 10*60*1000) //10 mins if not connected
      goToSleep();
  }
}