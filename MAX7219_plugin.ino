/*******************************************************************

  MAX7219 plugin для YoRadio (ESP32-S3)
  ====================================
  Бегущая строка на матрицах 8x8 MAX7219 (FC-16).
  Работает параллельно с основным дисплеем.

  Функции:
    - Станция + трек (кириллица через fonts/5bite_rus.h)
    - Встроенная погода YoRadio (timekeeper.weatherBuf)
    - Время / дата
    - IP
    - Громкость при изменении
    - Номер станции при смене
    - “Умный пропуск” блоков (если погоды нет — пропускается без ожидания)
    - STOPPED режим: коротко “STOP”, затем цикл:
        Время (центр) -> Погода (если есть) -> IP (если WiFi) -> Дата

  Библиотеки (Arduino Library Manager):
    - MD_Parola by majicDesigns
    - MD_MAX72XX by majicDesigns

  Шрифт:
    - fonts/5bite_rus.h

*******************************************************************/


// =====================================================================
//                         Н А С Т Р О Й К И
// =====================================================================

// --- Тип матрицы / количество модулей ---
#define MX_HARDWARE_TYPE   MD_MAX72XX::FC16_HW   // FC-16 обычно так
#define MX_MAX_DEVICES     8                     // 4..16 (8 модулей = 64x8)

// --- Пины (Software SPI) ---
#define MX_CLK_PIN         41
#define MX_DATA_PIN        40
#define MX_CS_PIN          39

// --- Яркость матрицы ---
#define MX_INTENSITY       2                     // 0..15

// --- Скорость прокрутки ---
// меньше = быстрее. Рекомендуется 20..60
#define MX_SCROLL_SPEED    35
// пауза между повторами бегущей строки, мс (0..1500)
#define MX_SCROLL_PAUSE    0

// --- STOPPED (когда радио остановлено) ---
#define MX_STOPPED_ROTATE_INFO      true          // крутить инфо-цикл в STOPPED
#define MX_STOPPED_TIME_HOLD_MS     1500          // сколько держать время по центру
#define MX_SHOW_STOP_BANNER         true          // показать "STOP" при остановке
#define MX_STOP_BANNER_MS           1000          // длительность "STOP", мс

// --- Стек задачи (если нестабильно — увеличь до 1024*4) ---
#define MX_STACK_SIZE      (1024 * 3)

// --- Что показывать (true/false) ---
#define MX_SHOW_STATION               true   // "<Станция> Трек"
#define MX_SHOW_WEATHER               true   // встроенная погода YoRadio (timekeeper.weatherBuf)
#define MX_SHOW_WEATHER_TITLE         true   // перед погодой показать "ПОГОДА"
#define MX_SHOW_TIME                  true   // время
#define MX_SHOW_DATE                  true   // дата
#define MX_SHOW_IP                    true   // IP в цикле
#define MX_SHOW_IP_ON_CONNECT         true   // показать IP сразу после WiFi connect
#define MX_SHOW_CUSTOM                false  // пользовательская строка
#define MX_SHOW_VOL                   true   // VOL:xx при изменении громкости
#define MX_SHOW_NUM_IN_STATION        false  // в строке станции показывать [N]
#define MX_SHOW_STATION_NUM_CHANGE    true   // [N] при переключении станции

// --- Пользовательская строка (если MX_SHOW_CUSTOM = true) ---
#define MX_CUSTOM_MSG  "  Всем привет! Всех Люблю!  "

// --- Форматы времени/даты ---
#define MX_TIME_FMT    "%H:%M"        // если хочешь секунды: "%H:%M'%S"
#define MX_DATE_FMT    "%d.%m.%Y"


// =====================================================================
//  ПОДКЛЮЧЕНИЕ ЗАГОЛОВКОВ YORADIO
// =====================================================================
#include "src/core/options.h"
#include "src/core/config.h"
#include "src/core/player.h"
#include "src/core/network.h"
#include "src/core/timekeeper.h"
#include "src/core/display.h"

// =====================================================================
//  MAX7219 библиотеки
// =====================================================================
#include <WiFi.h>
#include <SPI.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

// =====================================================================
//  Шрифт кириллицы
// =====================================================================
#include "fonts/5bite_rus.h"


// =====================================================================
//  УДОБНЫЕ МАКРОСЫ (внутренние переменные YoRadio)
// =====================================================================
#define NUMNEXT    display.numOfNextStation
#define VOLUME     config.store.volume
#define WEATHER_S  config.store.showweather
#define NUMSTAT    config.lastStation()
#define NAMESTAT   config.station.name
#define TITLE      config.station.title
#define WEATHER    timekeeper.weatherBuf


// =====================================================================
//  ЭФФЕКТЫ ТЕКСТА
// =====================================================================
#define MX_SCROLL_EFFECT     PA_RIGHT,  MX_SCROLL_SPEED, MX_SCROLL_PAUSE, PA_SCROLL_LEFT, PA_SCROLL_LEFT
#define MX_CENTER_EFFECT     PA_CENTER, 0,               0,               PA_PRINT,       PA_NO_EFFECT
#define MX_CENTER_HOLD(ms)   PA_CENTER, 0,               (ms),            PA_PRINT,       PA_NO_EFFECT


// =====================================================================
//  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// =====================================================================
static MD_Parola Parola(MX_HARDWARE_TYPE, MX_DATA_PIN, MX_CLK_PIN, MX_CS_PIN, MX_MAX_DEVICES);

static TaskHandle_t MaxTask = nullptr;
static bool mx_started = false;

static uint8_t  mode_inf = 1;
static uint8_t  vol_old  = 0;
static uint16_t num_old  = 0;

static uint8_t  stopped_mode = 0;   // цикл в STOPPED режиме (0..3)
static char informer[300];

// “запрос показать STOP”
static volatile bool mx_stop_banner_req = false;
// пока millis() < mx_stop_banner_until — держим STOP и ничего не переключаем
static unsigned long mx_stop_banner_until = 0;


// =====================================================================
//  ПРОТОТИПЫ
// =====================================================================
void mx_init();
void createTask();
void loopMaxTask(void* pvParameters);

void max7219();
void info_play();
void PlayInfo();

void info_volume();
void info_station();
void info_num_station();
void info_updating();

void mx_show_ip(bool resetCycle, bool useCenter);
bool mx_schedule_play_block(uint8_t id);
bool mx_schedule_stopped_block(uint8_t id);

void mx_utf8fix(char* s);


// =====================================================================
//  UTF-8 -> однобайтная “кодировка” под fonts/5bite_rus.h
//  Важно для кириллицы + заменяет '*' на градусы.
// =====================================================================
void mx_utf8fix(char* s)
{
  uint8_t* src = (uint8_t*)s;
  uint8_t* dst = (uint8_t*)s;

  while (*src)
  {
    // '*' в строке погоды YoRadio — используем как градусы
    if (*src == '*') { *dst++ = 247; src++; continue; }   // 247 = Degree в шрифте

    // Ё / ё
    if (src[0] == 0xD0 && src[1] == 0x81) { *dst++ = 192; src += 2; continue; } // Ё
    if (src[0] == 0xD1 && src[1] == 0x91) { *dst++ = 193; src += 2; continue; } // ё

    // Кириллица UTF-8: D0 XX / D1 XX -> XX
    if ((src[0] == 0xD0 || src[0] == 0xD1) && src[1]) { *dst++ = src[1]; src += 2; continue; }

    // '°' UTF-8 (C2 B0)
    if (src[0] == 0xC2 && src[1] == 0xB0) { *dst++ = 247; src += 2; continue; }

    // NBSP (C2 A0)
    if (src[0] == 0xC2 && src[1] == 0xA0) { *dst++ = ' '; src += 2; continue; }

    // — / –  (E2 80 94/93)
    if (src[0] == 0xE2 && src[1] == 0x80 && (src[2] == 0x94 || src[2] == 0x93))
    { *dst++ = '-'; src += 3; continue; }

    // ASCII как есть
    *dst++ = *src++;
  }

  *dst = 0;
}


// =====================================================================
//  ИНИЦИАЛИЗАЦИЯ ПЛАГИНА
// =====================================================================
void mx_init()
{
  if (mx_started) return;
  mx_started = true;

  Parola.begin();
  Parola.setFont(_5bite_rus);
  Parola.setIntensity(MX_INTENSITY);
  Parola.displayClear();

  strcpy(informer, "yoRadio");
  Parola.displayText(informer, MX_CENTER_EFFECT);
  Parola.displayAnimate();

  vol_old  = VOLUME;
  num_old  = NUMSTAT;
  mode_inf = 1;
  stopped_mode = 0;

#if MX_SHOW_IP_ON_CONNECT
  mx_show_ip(true, false);
#endif

  createTask();
  Serial.println("[MAX7219] plugin init OK");
}


// =====================================================================
//  ЗАДАЧА FreeRTOS
// =====================================================================
void createTask()
{
  if (MaxTask != nullptr) return;

  int core = 0;
#if (portNUM_PROCESSORS > 1)
  core = (xPortGetCoreID() == 0) ? 1 : 0;
#endif

  xTaskCreatePinnedToCore(
    loopMaxTask,
    "MaxTask",
    MX_STACK_SIZE,
    NULL,
    4,
    &MaxTask,
    core
  );
}

void loopMaxTask(void* pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(3000));
  while (true)
  {
    max7219();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


// =====================================================================
//  ОСНОВНАЯ ЛОГИКА
// =====================================================================
void max7219()
{
  static int f_vol = 0;
  static int f_num = 0;

  // ждём синхронизации времени (NTP)
  if (network.timeinfo.tm_year < 100) return;

  // --- показать STOP баннер (по запросу из callback) ---
#if MX_SHOW_STOP_BANNER
  if (mx_stop_banner_req && player.status() == STOPPED)
  {
    mx_stop_banner_req = false;
    mx_stop_banner_until = millis() + MX_STOP_BANNER_MS;

    Parola.displayReset();
    Parola.displayText((char*)"STOP", MX_CENTER_EFFECT);
    Parola.displayAnimate(); // вывести сразу
  }

  // пока баннер активен — держим его и не переключаем режимы
  if (mx_stop_banner_until != 0)
  {
    if ((long)(millis() - mx_stop_banner_until) < 0)
    {
      Parola.displayAnimate();
      vTaskDelay(pdMS_TO_TICKS(10));
      return;
    }
    mx_stop_banner_until = 0;
  }
#endif

  // --- громкость ---
#if MX_SHOW_VOL
  if (VOLUME != vol_old)
  {
    vol_old = VOLUME;
    f_vol = 200;  // 200 * 10мс ≈ 2 сек
    info_volume();
  }
  if (f_vol > 0)
  {
    f_vol--;
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }
#endif

  // --- номер станции при переключении ---
#if MX_SHOW_STATION_NUM_CHANGE
  if (NUMSTAT != num_old)
  {
    num_old = NUMSTAT;
    f_num = 100;  // 100 * 10мс ≈ 1 сек
    info_num_station();
  }
  if (f_num > 0)
  {
    f_num--;
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }
#endif

  if (display.mode() == PLAYER)   info_play();
  if (display.mode() == NUMBERS)  info_station();
  if (display.mode() == UPDATING) info_updating();
}


// =====================================================================
//  IP вывод (можно вызывать как “блок” цикла или при подключении)
// =====================================================================
void mx_show_ip(bool resetCycle, bool useCenter)
{
  if (!MX_SHOW_IP) return;
  if (WiFi.status() != WL_CONNECTED) return;

  String ip = WiFi.localIP().toString();
  snprintf(informer, sizeof(informer), " IP:%s ", ip.c_str());
  mx_utf8fix(informer);

  Parola.displayReset();
  if (useCenter)
    Parola.displayText(informer, MX_CENTER_EFFECT);
  else
    Parola.displayText(informer, MX_SCROLL_EFFECT);
  Parola.displayAnimate();

  if (resetCycle) mode_inf = 1;
}


// =====================================================================
//  РЕЖИМ PLAYER
// =====================================================================
void info_play()
{
  if (player.status() == STOPPED)
  {
    if (!MX_STOPPED_ROTATE_INFO)
    {
      // просто время по центру
      if (Parola.displayAnimate())
      {
        strftime(informer, sizeof(informer), MX_TIME_FMT, &network.timeinfo);
        Parola.displayText(informer, MX_CENTER_EFFECT);
      }
      return;
    }

    // STOPPED: крутим цикл информации вместо “секунд”
    if (Parola.displayAnimate())
    {
      bool scheduled = false;
      for (uint8_t i = 0; i < 10 && !scheduled; i++)
      {
        scheduled = mx_schedule_stopped_block(stopped_mode);
        stopped_mode = (stopped_mode + 1) % 4; // 0..3
      }

      if (!scheduled)
      {
        strcpy(informer, "...");
        Parola.displayText(informer, MX_SCROLL_EFFECT);
      }
    }
    return;
  }

  if (player.status() == PLAYING)
  {
    if (Parola.displayAnimate())
      PlayInfo();
  }
}


// =====================================================================
//  STOPPED блоки (умный пропуск):
//   0: время (центр, удержание)
//   1: погода (если есть)
//   2: IP (если WiFi)
//   3: дата
// =====================================================================
bool mx_schedule_stopped_block(uint8_t id)
{
  switch (id)
  {
    case 0:
#if MX_SHOW_TIME
      strftime(informer, sizeof(informer), MX_TIME_FMT, &network.timeinfo);
      Parola.displayText(informer, MX_CENTER_HOLD(MX_STOPPED_TIME_HOLD_MS));
      return true;
#else
      return false;
#endif

    case 1:
#if MX_SHOW_WEATHER
      if (!WEATHER_S) return false;
      if (!WEATHER || WEATHER[0] == '\0') return false;
      strncpy(informer, WEATHER, sizeof(informer));
      informer[sizeof(informer) - 1] = 0;
      mx_utf8fix(informer);
      Parola.displayText(informer, MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif

    case 2:
#if MX_SHOW_IP
      if (WiFi.status() != WL_CONNECTED) return false;
      mx_show_ip(false, false);
      return true;
#else
      return false;
#endif

    case 3:
#if MX_SHOW_DATE
      strftime(informer, sizeof(informer), MX_DATE_FMT, &network.timeinfo);
      Parola.displayText(informer, MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif
  }

  return false;
}


// =====================================================================
//  PLAYING цикл (умный пропуск блоков)
// =====================================================================
void PlayInfo()
{
  bool scheduled = false;

  for (uint8_t attempt = 0; attempt < 12 && !scheduled; attempt++)
  {
    scheduled = mx_schedule_play_block(mode_inf);
    mode_inf++;
    if (mode_inf > 7) mode_inf = 1;   // цикл 1..7
  }

  if (!scheduled)
  {
    strcpy(informer, "...");
    Parola.displayText(informer, MX_SCROLL_EFFECT);
  }
}

bool mx_schedule_play_block(uint8_t id)
{
  switch (id)
  {
    // 1) Станция + трек (если трека нет — только станция)
    case 1:
    {
#if MX_SHOW_STATION
      if (TITLE && TITLE[0] != '\0')
      {
  #if MX_SHOW_NUM_IN_STATION
        snprintf(informer, sizeof(informer), "  [%d] <%s> %s ", NUMSTAT, NAMESTAT, TITLE);
  #else
        snprintf(informer, sizeof(informer), "  <%s> %s ", NAMESTAT, TITLE);
  #endif
      }
      else
      {
  #if MX_SHOW_NUM_IN_STATION
        snprintf(informer, sizeof(informer), "  [%d] <%s> ", NUMSTAT, NAMESTAT);
  #else
        snprintf(informer, sizeof(informer), "  <%s> ", NAMESTAT);
  #endif
      }
      mx_utf8fix(informer);
      Parola.displayText(informer, MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif
    }

    // 2) Заголовок "ПОГОДА" — только если погода реально есть
    case 2:
    {
#if MX_SHOW_WEATHER && MX_SHOW_WEATHER_TITLE
      if (!WEATHER_S) return false;
      if (!WEATHER || WEATHER[0] == '\0') return false;
      Parola.displayText((char*)"    ПОГОДА   ", MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif
    }

    // 3) Строка погоды
    case 3:
    {
#if MX_SHOW_WEATHER
      if (!WEATHER_S) return false;
      if (!WEATHER || WEATHER[0] == '\0') return false;
      strncpy(informer, WEATHER, sizeof(informer));
      informer[sizeof(informer) - 1] = 0;
      mx_utf8fix(informer);
      Parola.displayText(informer, MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif
    }

    // 4) Время
    case 4:
    {
#if MX_SHOW_TIME
      strftime(informer, sizeof(informer), " " MX_TIME_FMT " ", &network.timeinfo);
      Parola.displayText(informer, MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif
    }

    // 5) Дата
    case 5:
    {
#if MX_SHOW_DATE
      strftime(informer, sizeof(informer), " " MX_DATE_FMT " ", &network.timeinfo);
      Parola.displayText(informer, MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif
    }

    // 6) IP
    case 6:
    {
#if MX_SHOW_IP
      if (WiFi.status() != WL_CONNECTED) return false;
      mx_show_ip(false, false);
      return true;
#else
      return false;
#endif
    }

    // 7) Пользовательская строка
    case 7:
    {
#if MX_SHOW_CUSTOM
      strncpy(informer, MX_CUSTOM_MSG, sizeof(informer));
      informer[sizeof(informer) - 1] = 0;
      mx_utf8fix(informer);
      Parola.displayText(informer, MX_SCROLL_EFFECT);
      return true;
#else
      return false;
#endif
    }
  }

  return false;
}


// =====================================================================
//  КОРОТКИЕ ЭКРАНЫ: громкость, номера, updating
// =====================================================================
void info_volume()
{
  snprintf(informer, sizeof(informer), "VOL:%d", VOLUME);
  Parola.displayReset();
  Parola.displayText(informer, MX_CENTER_EFFECT);
  Parola.displayAnimate();
  mode_inf = 1;
}

void info_station()
{
  snprintf(informer, sizeof(informer), "= %d =", NUMNEXT);
  Parola.displayReset();
  Parola.displayText(informer, MX_CENTER_EFFECT);
  Parola.displayAnimate();
  mode_inf = 64;
}

void info_num_station()
{
  snprintf(informer, sizeof(informer), "[ %d ]", NUMSTAT);
  Parola.displayReset();
  Parola.displayText(informer, MX_CENTER_EFFECT);
  Parola.displayAnimate();
  mode_inf = 64;
}

void info_updating()
{
  Parola.displayReset();
  Parola.displayText((char*)"UPDATE...", MX_CENTER_EFFECT);
  Parola.displayAnimate();
  mode_inf = 1;
}


// =====================================================================
//  CALLBACKS YoRadio
// =====================================================================
void player_on_start_play()
{
  mode_inf = 1;
}

void player_on_stop_play()
{
  stopped_mode = 0;
  mode_inf = 1;

#if MX_SHOW_STOP_BANNER
  // Не трогаем Parola тут (чтобы не конфликтовать с задачей на другом ядре),
  // просто запрашиваем показ, а задача сама отрисует.
  mx_stop_banner_req = true;
#endif
}

// Запуск плагина после подключения к Wi-Fi
void network_on_connect()
{
  mx_init();
}
