#include <Arduino.h>
#include <BleGamepad.h>
#include "driver/rtc_io.h"
#include "esp_sleep.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BATTERY_ADC_UNIT ADC_UNIT_1
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 //gpio 0
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT

#define PIN_BUILTIN_LED 8

#define PIN_WS2812_LED_VCC 6
#define PIN_WS2812_LED_DATA 7

//buttons are active-low with pullups
#define PIN_STAR_BOOT 9 //is also used as BOOT button
#define PIN_MENU_POWER 5 //is also used as POWER button

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

adc_oneshot_unit_handle_t battery_adc_handle;
adc_cali_handle_t battery_adc_cali_handle;

void goToSleep()
{
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

  if (mv > 4200)
    return 100;
  if (mv < 3100)
    return 0;

  return (mv - 3100)/((4200 - 3100) / 100); //just linear aprox.
}

void update_battery_status()
{
  auto battery_percent = get_battery_percent();

  if (battery_percent == 0)
    goToSleep();

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
  auto battery_percent = get_battery_percent();

  if (battery_percent == 0)
    goToSleep();
    
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_GPIO)
    goToSleep();

  pinMode(PIN_MENU_POWER, INPUT_PULLUP);

  delay(50); //'debounce'

  while (millis() < 2000) //button should be held for some time to register power-on
    if (digitalRead(PIN_MENU_POWER))
      goToSleep();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 0);

  pinMode(PIN_STAR_BOOT, INPUT_PULLUP);
  pinMode(PIN_STRUM_UP, INPUT_PULLUP);
  pinMode(PIN_STRUM_DOWN, INPUT_PULLUP);
  pinMode(PIN_FRET_1, INPUT_PULLUP);
  pinMode(PIN_FRET_2, INPUT_PULLUP);
  pinMode(PIN_FRET_3, INPUT_PULLUP);
  pinMode(PIN_FRET_4, INPUT_PULLUP);
  pinMode(PIN_FRET_5, INPUT_PULLUP);
  
  auto config = new BleGamepadConfiguration();
  config->setButtonCount(9);
  config->setWhichAxes(false, false, false, false, false, false, false, false);
  config->setHatSwitchCount(0);

  update_battery_status();
  bleGamepad.begin(config);
}

uint8_t buttonsDebounce[9] = {0};
int32_t pressedButtons = 0;
int32_t reportedButtons = 0;

int32_t powerPressedAt = 0;
int32_t lastActivityAt = 0;
int32_t batteryReadAt = 0;
 
void loop() 
{
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

  auto powerWasPressed = !!(pressedButtons & 0x2);

  for (auto i = 0; i < 9; ++i) 
    if (buttonsDebounce[i] == 0xFF)
      pressedButtons |= (1 << i);
    else if (buttonsDebounce[i] == 0)
      pressedButtons &= ~(1 << i);

  auto powerIsPressed = !!(pressedButtons & 0x2);

  if (!powerWasPressed && powerIsPressed)
    powerPressedAt = millis();
  else if (powerWasPressed && powerIsPressed && (millis() - powerPressedAt) > 5000) //5 seconds to power-off
  {
    delay(100);
    goToSleep();
  }

  if (bleGamepad.isConnected()) 
  {
    for (auto i = 0; i < 9; ++i)
    {
      auto mask = 1 << i;

      if ((reportedButtons & mask) && !(pressedButtons & mask))
        bleGamepad.release(BUTTON_1 + i);
      else if (!(reportedButtons & mask) && (pressedButtons & mask))
        bleGamepad.press(BUTTON_1 + i);
    }

    if (pressedButtons != reportedButtons)
      lastActivityAt = millis();

    reportedButtons = pressedButtons;

    if ((millis() - lastActivityAt) > 20*60*1000) //20 mins if connected
      goToSleep();
  }
  else
  {
    reportedButtons = 0;

    if ((millis() - lastActivityAt) > 10*60*1000) //10 mins if not connected
      goToSleep();
  }

  delay(1);
}