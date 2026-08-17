/*
esp32 v3.1.0
MaTouch ESP32-S3 3.95 ST7796 + FT6236 + MaUWB AT UART

First positioning demo:
- Three fixed anchors (IDs 0, 1, 2)
- One displayed tag (TARGET_TAG_ID)
- MaUWB AT+RANGE reports are parsed and positioned on the local screen
*/

#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <Preferences.h>
#include "FT6236.h"
#include "MakerfabsLogo.h"

// ==================== Screen parameters ====================
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 480

#define GFX_BL    48
#define TFT_SCK   14
#define TFT_MOSI  13
#define TFT_DC    21
#define TFT_CS    15
#define TFT_RST   1

#define TOUCH_SDA 38
#define TOUCH_SCL 39

// ==================== MaUWB parameters ====================
#define UWB_INDEX 0
#define UWB_TAG_COUNT 64
#define UWB_ROLE 0  // 0: Tag, 1: Anchor

#define UWB_RX 16  // ESP32-S3 IO16 <- MaUWB UART1TX
#define UWB_TX 17  // ESP32-S3 IO17 -> MaUWB UART1RX

// Set this to the ID of the tag that should be shown in the first demo.
#define TARGET_TAG_ID 0

#define PRINT_UWB_LINES 1
#define MAX_RANGE_SLOTS 16

HardwareSerial UwbSerial(2);

// ==================== Positioning parameters ====================
struct Anchor {
  int id;
  float x;
  float y;
};

struct TagPosition {
  float x;
  float y;
  bool valid;
};

const int ANCHOR_COUNT = 3;
Anchor anchors[ANCHOR_COUNT] = {
  {0,   0.0f,   0.0f},
  {1, 200.0f,   0.0f},
  {2, 200.0f, 200.0f},
};

float mapMinX = 0.0f;
float mapMaxX = 200.0f;
float mapMinY = 0.0f;
float mapMaxY = 200.0f;
const float FILTER_ALPHA = 0.20f;
const float MOVE_DEADBAND_CM = 5.0f;
const float MAX_MEAN_RESIDUAL_CM = 50.0f;
const uint32_t POSITION_TIMEOUT_MS = 1500;
const uint32_t DISPLAY_INTERVAL_MS = 100;

// ==================== Screen objects ====================
Arduino_DataBus *bus = new Arduino_ESP32SPI(
  TFT_DC,
  TFT_CS,
  TFT_SCK,
  TFT_MOSI,
  GFX_NOT_DEFINED
);

Arduino_GFX *gfx = new Arduino_ST7796(
  bus,
  TFT_RST,
  0,
  false,
  SCREEN_WIDTH,
  SCREEN_HEIGHT
);

// LVGL owns all drawing after the startup logo.  A partial buffer keeps the
// RAM cost low while still allowing the ST7796 to render a polished UI.
static const uint16_t LVGL_DRAW_BUFFER_LINES = 40;
static lv_color_t lvglDrawBuffer[SCREEN_WIDTH * LVGL_DRAW_BUFFER_LINES];
static lv_disp_draw_buf_t lvglDisplayBuffer;
static lv_disp_drv_t lvglDisplayDriver;
static lv_indev_drv_t lvglTouchDriver;
static bool lvglReady = false;

// ==================== Runtime state ====================
char uwbLine[384];
size_t uwbLineLength = 0;

TagPosition tagPosition = {0.0f, 0.0f, false};
float lastRanges[ANCHOR_COUNT] = {0.0f, 0.0f, 0.0f};
bool rangeAvailable[ANCHOR_COUNT] = {false, false, false};
uint32_t lastPositionMs = 0;
uint32_t lastDisplayMs = 0;

enum PositionState {
  STATE_WAITING,
  STATE_VALID,
  STATE_RANGE_ERROR,
  STATE_GEOMETRY_ERROR,
  STATE_OUTSIDE_MAP,
};

PositionState positionState = STATE_WAITING;
Preferences preferences;
char anchorInputs[ANCHOR_COUNT * 2][12] = {""};
char anchorSetupStatus[48] = "Edit anchor coordinates";

// ==================== Display ====================
void showStartupLogo() {
  gfx->fillScreen(WHITE);
  gfx->draw16bitRGBBitmap((SCREEN_WIDTH - MAKERFABS_LOGO_WIDTH) / 2,
                           (SCREEN_HEIGHT - MAKERFABS_LOGO_HEIGHT) / 2,
                           (uint16_t *)makerfabsLogo, MAKERFABS_LOGO_WIDTH,
                           MAKERFABS_LOGO_HEIGHT);
}

const char *positionStateText() {
  if (positionState == STATE_VALID && millis() - lastPositionMs > POSITION_TIMEOUT_MS) {
    return "No UWB data";
  }

  switch (positionState) {
    case STATE_VALID: return "Position valid";
    case STATE_RANGE_ERROR: return "Range error";
    case STATE_GEOMETRY_ERROR: return "Geometry error";
    case STATE_OUTSIDE_MAP: return "Outside map";
    case STATE_WAITING: {
      static char waitingText[32];
      strcpy(waitingText, "Waiting for ");
      bool firstMissing = true;
      for (int i = 0; i < ANCHOR_COUNT; ++i) {
        if (!rangeAvailable[i]) {
          if (!firstMissing) {
            strcat(waitingText, "/");
          }
          char anchorText[4];
          snprintf(anchorText, sizeof(anchorText), "A%d", anchors[i].id);
          strcat(waitingText, anchorText);
          firstMissing = false;
        }
      }
      return firstMissing ? "Waiting for range" : waitingText;
    }
    default: return "Waiting for range";
  }
}

// ==================== Anchor setup screen ====================
void updateMapBounds() {
  float minimumX = anchors[0].x;
  float maximumX = anchors[0].x;
  float minimumY = anchors[0].y;
  float maximumY = anchors[0].y;

  for (int i = 1; i < ANCHOR_COUNT; ++i) {
    minimumX = fminf(minimumX, anchors[i].x);
    maximumX = fmaxf(maximumX, anchors[i].x);
    minimumY = fminf(minimumY, anchors[i].y);
    maximumY = fmaxf(maximumY, anchors[i].y);
  }

  float width = maximumX - minimumX;
  float height = maximumY - minimumY;
  if (width < 80.0f) {
    float center = (minimumX + maximumX) * 0.5f;
    minimumX = center - 40.0f;
    maximumX = center + 40.0f;
    width = 80.0f;
  }
  if (height < 80.0f) {
    float center = (minimumY + maximumY) * 0.5f;
    minimumY = center - 40.0f;
    maximumY = center + 40.0f;
    height = 80.0f;
  }

  float padding = fmaxf(20.0f, fmaxf(width, height) * 0.15f);
  mapMinX = minimumX - padding;
  mapMaxX = maximumX + padding;
  mapMinY = minimumY - padding;
  mapMaxY = maximumY + padding;
}

void syncAnchorInputs() {
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    snprintf(anchorInputs[i * 2], sizeof(anchorInputs[i * 2]), "%.0f", anchors[i].x);
    snprintf(anchorInputs[i * 2 + 1], sizeof(anchorInputs[i * 2 + 1]), "%.0f", anchors[i].y);
  }
}

bool applyAnchorInputs() {
  float values[ANCHOR_COUNT * 2];
  for (int i = 0; i < ANCHOR_COUNT * 2; ++i) {
    char *end = nullptr;
    values[i] = strtof(anchorInputs[i], &end);
    if (end == anchorInputs[i] || *end != '\0' || !isfinite(values[i]) || fabsf(values[i]) > 100000.0f) {
      int anchorIndex = i / 2;
      snprintf(anchorSetupStatus, sizeof(anchorSetupStatus), "A%d %c is invalid", anchors[anchorIndex].id,
               (i % 2 == 0) ? 'X' : 'Y');
      return false;
    }
  }

  float twiceArea = (values[2] - values[0]) * (values[5] - values[1])
                   - (values[4] - values[0]) * (values[3] - values[1]);
  if (fabsf(twiceArea) < 1.0f) {
    strcpy(anchorSetupStatus, "Anchors cannot be collinear");
    return false;
  }

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    anchors[i].x = values[i * 2];
    anchors[i].y = values[i * 2 + 1];
  }
  updateMapBounds();
  return true;
}

void saveAnchorConfiguration() {
  preferences.begin("uwbpos", false);
  preferences.putBool("valid", true);
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    char key[4];
    snprintf(key, sizeof(key), "a%dx", i);
    preferences.putFloat(key, anchors[i].x);
    snprintf(key, sizeof(key), "a%dy", i);
    preferences.putFloat(key, anchors[i].y);
  }
  preferences.end();
}

void loadAnchorConfiguration() {
  preferences.begin("uwbpos", true);
  bool valid = preferences.getBool("valid", false);
  if (valid) {
    for (int i = 0; i < ANCHOR_COUNT; ++i) {
      char key[4];
      snprintf(key, sizeof(key), "a%dx", i);
      anchors[i].x = preferences.getFloat(key, anchors[i].x);
      snprintf(key, sizeof(key), "a%dy", i);
      anchors[i].y = preferences.getFloat(key, anchors[i].y);
    }
    strcpy(anchorSetupStatus, "Saved coordinates loaded");
  }
  preferences.end();
  updateMapBounds();
  syncAnchorInputs();
}

// ==================== Positioning ====================
int findAnchorIndex(int id) {
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    if (anchors[i].id == id) {
      return i;
    }
  }
  return -1;
}

bool calculatePosition(const float ranges[ANCHOR_COUNT], float &x, float &y) {
  const Anchor &reference = anchors[0];
  const Anchor &anchor1 = anchors[1];
  const Anchor &anchor2 = anchors[2];

  float a11 = 2.0f * (anchor1.x - reference.x);
  float a12 = 2.0f * (anchor1.y - reference.y);
  float a21 = 2.0f * (anchor2.x - reference.x);
  float a22 = 2.0f * (anchor2.y - reference.y);

  float b1 = ranges[0] * ranges[0] - ranges[1] * ranges[1]
             + anchor1.x * anchor1.x - reference.x * reference.x
             + anchor1.y * anchor1.y - reference.y * reference.y;
  float b2 = ranges[0] * ranges[0] - ranges[2] * ranges[2]
             + anchor2.x * anchor2.x - reference.x * reference.x
             + anchor2.y * anchor2.y - reference.y * reference.y;

  float determinant = a11 * a22 - a12 * a21;
  if (fabsf(determinant) < 0.001f) {
    return false;
  }

  x = (b1 * a22 - a12 * b2) / determinant;
  y = (a11 * b2 - b1 * a21) / determinant;
  return isfinite(x) && isfinite(y);
}

bool positionHasAcceptableResidual(float x, float y, const float ranges[ANCHOR_COUNT]) {
  float totalResidual = 0.0f;
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    float dx = x - anchors[i].x;
    float dy = y - anchors[i].y;
    float calculatedRange = sqrtf(dx * dx + dy * dy);
    totalResidual += fabsf(calculatedRange - ranges[i]);
  }
  return totalResidual / ANCHOR_COUNT <= MAX_MEAN_RESIDUAL_CM;
}

void updateTagPosition(float rawX, float rawY) {
  if (!tagPosition.valid) {
    tagPosition.x = rawX;
    tagPosition.y = rawY;
    tagPosition.valid = true;
  } else {
    float filteredX = tagPosition.x * (1.0f - FILTER_ALPHA) + rawX * FILTER_ALPHA;
    float filteredY = tagPosition.y * (1.0f - FILTER_ALPHA) + rawY * FILTER_ALPHA;
    float dx = filteredX - tagPosition.x;
    float dy = filteredY - tagPosition.y;

    if (sqrtf(dx * dx + dy * dy) >= MOVE_DEADBAND_CM) {
      tagPosition.x = filteredX;
      tagPosition.y = filteredY;
    }
  }

  lastPositionMs = millis();
  positionState = STATE_VALID;
}

// ==================== MaUWB report parsing ====================
bool parseIntegerAfter(const char *text, const char *key, int &value) {
  const char *start = strstr(text, key);
  if (start == nullptr) {
    return false;
  }

  start += strlen(key);
  char *end = nullptr;
  long parsed = strtol(start, &end, 10);
  if (end == start) {
    return false;
  }

  value = (int)parsed;
  return true;
}

int parseNumberList(const char *text, int values[], int capacity) {
  int count = 0;
  const char *cursor = text;

  while (*cursor != '\0' && *cursor != ')') {
    while (*cursor == ' ' || *cursor == '\t') {
      ++cursor;
    }

    if (count >= capacity) {
      return -1;
    }

    char *end = nullptr;
    long parsed = strtol(cursor, &end, 10);
    if (end == cursor) {
      return -1;
    }
    values[count++] = (int)parsed;
    cursor = end;

    while (*cursor == ' ' || *cursor == '\t') {
      ++cursor;
    }
    if (*cursor == ',') {
      ++cursor;
    } else if (*cursor != ')') {
      return -1;
    }
  }

  return (*cursor == ')') ? count : -1;
}

void processRangeReport(const char *line) {
  int tagId = -1;
  if (!parseIntegerAfter(line, "tid:", tagId) || tagId != TARGET_TAG_ID) {
    return;
  }

  const char *rangeList = strstr(line, "range:(");
  const char *anchorList = strstr(line, "ancid:(");
  if (rangeList == nullptr || anchorList == nullptr) {
    positionState = STATE_RANGE_ERROR;
    return;
  }

  int rawRanges[MAX_RANGE_SLOTS];
  int rawAnchorIds[MAX_RANGE_SLOTS];
  int rangeCount = parseNumberList(rangeList + 7, rawRanges, MAX_RANGE_SLOTS);
  int anchorCount = parseNumberList(anchorList + 7, rawAnchorIds, MAX_RANGE_SLOTS);
  if (rangeCount <= 0 || rangeCount != anchorCount) {
    positionState = STATE_RANGE_ERROR;
    return;
  }

  float pairedRanges[ANCHOR_COUNT] = {0.0f, 0.0f, 0.0f};
  bool found[ANCHOR_COUNT] = {false, false, false};

  // Keep each ancid[i] paired with its original range[i].
  for (int i = 0; i < rangeCount; ++i) {
    int anchorIndex = findAnchorIndex(rawAnchorIds[i]);
    if (anchorIndex >= 0 && rawRanges[i] > 0) {
      pairedRanges[anchorIndex] = (float)rawRanges[i];
      found[anchorIndex] = true;
    }
  }

  // Update the distance display for every valid anchor in this report.
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    rangeAvailable[i] = found[i];
    if (found[i]) {
      lastRanges[i] = pairedRanges[i];
    }
  }

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    if (!found[i]) {
      positionState = STATE_WAITING;
      return;
    }
  }

  float rawX;
  float rawY;
  if (!calculatePosition(pairedRanges, rawX, rawY)) {
    positionState = STATE_GEOMETRY_ERROR;
    return;
  }

  if (!positionHasAcceptableResidual(rawX, rawY, pairedRanges)) {
    positionState = STATE_RANGE_ERROR;
    return;
  }

  if (rawX < mapMinX - 30.0f || rawX > mapMaxX + 30.0f ||
      rawY < mapMinY - 30.0f || rawY > mapMaxY + 30.0f) {
    positionState = STATE_OUTSIDE_MAP;
    return;
  }

  updateTagPosition(rawX, rawY);

  Serial.printf("T%d -> X:%.1f cm, Y:%.1f cm | A0:%.0f A1:%.0f A2:%.0f\n",
                tagId, rawX, rawY,
                pairedRanges[0], pairedRanges[1], pairedRanges[2]);
}

void processUwbLine(const char *line) {
  if (PRINT_UWB_LINES) {
    Serial.println(line);
  }

  if (strncmp(line, "AT+RANGE=", 9) == 0) {
    processRangeReport(line);
  }
}

// ==================== MaUWB setup and serial ====================
String uwbConfigCommand() {
  String command = "AT+SETCFG=";
  command += UWB_INDEX;
  command += ",";
  command += UWB_ROLE;
  command += ",1";  // 1: 6.8M, 0: 850K
  command += ",1";  // Range filter on
  return command;
}

String uwbCapacityCommand() {
  String command = "AT+SETCAP=";
  command += UWB_TAG_COUNT;
  command += ",10"; // 6.8M slot time: 10 ms
  command += ",1";  // Extended packet mode
  return command;
}

String sendUwbCommand(const String &command, uint32_t timeoutMs, bool debug) {
  String response = "";

  Serial.println(command);
  UwbSerial.println(command);

  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (UwbSerial.available()) {
      response += (char)UwbSerial.read();
    }
  }

  if (debug) {
    Serial.println(response);
  }

  return response;
}

void initUwb() {
  UwbSerial.begin(115200, SERIAL_8N1, UWB_RX, UWB_TX);
  delay(300);

  sendUwbCommand("AT?", 2000, true);
  sendUwbCommand("AT+RESTORE", 5000, true);
  sendUwbCommand(uwbConfigCommand(), 2000, true);
  sendUwbCommand(uwbCapacityCommand(), 2000, true);
  sendUwbCommand("AT+SETRPT=1", 2000, true);
  sendUwbCommand("AT+SAVE", 2000, true);
  sendUwbCommand("AT+RESTART", 2000, true);
}

void handleUwbSerial() {
  // Keep the USB serial bridge for manual MaUWB AT commands.
  while (Serial.available() > 0) {
    UwbSerial.write(Serial.read());
  }

  while (UwbSerial.available() > 0) {
    char c = (char)UwbSerial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (uwbLineLength > 0) {
        uwbLine[uwbLineLength] = '\0';
        processUwbLine(uwbLine);
        uwbLineLength = 0;
      }
    } else if (uwbLineLength < sizeof(uwbLine) - 1) {
      uwbLine[uwbLineLength++] = c;
    } else {
      // Drop an overlong line rather than corrupting the parser buffer.
      uwbLineLength = 0;
      positionState = STATE_RANGE_ERROR;
      Serial.println("UWB line too long; dropped");
    }
  }
}

// ==================== LVGL user interface ====================
const int LVGL_MAP_X = 14;
const int LVGL_MAP_Y = 98;
const int LVGL_MAP_W = 292;
const int LVGL_MAP_H = 232;
const int LVGL_MAP_MARGIN = 18;

const uint32_t UI_BG = 0x10191D;
const uint32_t UI_SURFACE = 0x1B292F;
const uint32_t UI_SURFACE_ALT = 0x243840;
const uint32_t UI_TEAL = 0x14A6A2;
const uint32_t UI_TEAL_DARK = 0x0B565B;
const uint32_t UI_TEXT = 0xF4FAFA;
const uint32_t UI_MUTED = 0x9AB1B6;
const uint32_t UI_GRID = 0x42646A;
const uint32_t UI_TAG = 0xFF6B5E;
const uint32_t UI_OK = 0x35C486;
const uint32_t UI_WARN = 0xF3B85B;
const uint32_t UI_ERROR = 0xEE625C;

lv_obj_t *positionScreen = nullptr;
lv_obj_t *setupScreen = nullptr;
lv_obj_t *lvglMap = nullptr;
lv_obj_t *coordinateLabel = nullptr;
lv_obj_t *statusLabel = nullptr;
lv_obj_t *rangeValueLabels[ANCHOR_COUNT] = {nullptr, nullptr, nullptr};
lv_obj_t *anchorDots[ANCHOR_COUNT] = {nullptr, nullptr, nullptr};
lv_obj_t *anchorTextLabels[ANCHOR_COUNT] = {nullptr, nullptr, nullptr};
lv_obj_t *rangeLines[ANCHOR_COUNT] = {nullptr, nullptr, nullptr};
lv_point_t rangeLinePoints[ANCHOR_COUNT][2];
lv_point_t mapGridVertical[3][2];
lv_point_t mapGridHorizontal[3][2];
lv_obj_t *tagDot = nullptr;
lv_obj_t *tagLabel = nullptr;
lv_obj_t *setupValueLabels[ANCHOR_COUNT * 2] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
lv_obj_t *setupStatusLabel = nullptr;
lv_obj_t *editOverlay = nullptr;
lv_obj_t *editInput = nullptr;
lv_obj_t *editHintLabel = nullptr;
int editingAnchorInput = -1;

lv_color_t uiColor(uint32_t color) {
  return lv_color_hex(color);
}

void styleSurface(lv_obj_t *object, uint32_t color, uint8_t radius) {
  lv_obj_set_style_bg_color(object, uiColor(color), 0);
  lv_obj_set_style_bg_opa(object, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(object, radius, 0);
  lv_obj_set_style_border_width(object, 0, 0);
  lv_obj_set_style_pad_all(object, 0, 0);
}

lv_obj_t *createLabel(lv_obj_t *parent, const char *text, uint32_t color) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, uiColor(color), 0);
  return label;
}

lv_obj_t *createButton(lv_obj_t *parent, int x, int y, int width, int height,
                       const char *text, uint32_t color, lv_event_cb_t callback,
                       void *userData = nullptr) {
  lv_obj_t *button = lv_btn_create(parent);
  lv_obj_set_pos(button, x, y);
  lv_obj_set_size(button, width, height);
  styleSurface(button, color, 7);
  lv_obj_set_style_bg_color(button, uiColor(UI_TEAL), LV_PART_MAIN | LV_STATE_PRESSED);
  lv_obj_set_style_text_color(button, uiColor(UI_TEXT), 0);
  lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, userData);
  if (text != nullptr && text[0] != '\0') {
    lv_obj_t *label = createLabel(button, text, UI_TEXT);
    lv_obj_center(label);
  }
  return button;
}

void lvglFlush(lv_disp_drv_t *display, const lv_area_t *area, lv_color_t *colors) {
  int width = area->x2 - area->x1 + 1;
  int height = area->y2 - area->y1 + 1;
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)colors, width, height);
  lv_disp_flush_ready(display);
}

void lvglTouchRead(lv_indev_drv_t *input, lv_indev_data_t *data) {
  (void)input;
  int position[2];
  if (ft6236_pos(position)) {
    data->point.x = constrain(position[0], 0, SCREEN_WIDTH - 1);
    data->point.y = constrain(position[1], 0, SCREEN_HEIGHT - 1);
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

float lvglMapScale() {
  float scaleX = (LVGL_MAP_W - 2 * LVGL_MAP_MARGIN) / (mapMaxX - mapMinX);
  float scaleY = (LVGL_MAP_H - 2 * LVGL_MAP_MARGIN) / (mapMaxY - mapMinY);
  return (scaleX < scaleY) ? scaleX : scaleY;
}

int worldToLvglMapX(float x) {
  float plotWidth = (mapMaxX - mapMinX) * lvglMapScale();
  int contentWidth = LVGL_MAP_W - 2 * LVGL_MAP_MARGIN;
  return LVGL_MAP_MARGIN + (contentWidth - (int)plotWidth) / 2
         + (int)lroundf((x - mapMinX) * lvglMapScale());
}

int worldToLvglMapY(float y) {
  float plotHeight = (mapMaxY - mapMinY) * lvglMapScale();
  int contentHeight = LVGL_MAP_H - 2 * LVGL_MAP_MARGIN;
  return LVGL_MAP_MARGIN + (contentHeight - (int)plotHeight) / 2
         + (int)lroundf((mapMaxY - y) * lvglMapScale());
}

void updateSetupValueLabels() {
  if (setupStatusLabel == nullptr) {
    return;
  }
  for (int i = 0; i < ANCHOR_COUNT * 2; ++i) {
    if (setupValueLabels[i] != nullptr) {
      lv_label_set_text(setupValueLabels[i], anchorInputs[i]);
    }
  }
  lv_label_set_text(setupStatusLabel, anchorSetupStatus);
}

void updateLvglMap() {
  if (lvglMap == nullptr) {
    return;
  }

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    int anchorX = worldToLvglMapX(anchors[i].x);
    int anchorY = worldToLvglMapY(anchors[i].y);
    lv_obj_set_pos(anchorDots[i], anchorX - 5, anchorY - 5);
    lv_obj_set_pos(anchorTextLabels[i], constrain(anchorX + 8, 2, LVGL_MAP_W - 26),
                   constrain(anchorY - 17, 2, LVGL_MAP_H - 18));
  }

  int tagX = 0;
  int tagY = 0;
  bool hasPosition = tagPosition.valid && positionState == STATE_VALID &&
                     millis() - lastPositionMs <= POSITION_TIMEOUT_MS;
  if (hasPosition) {
    tagX = constrain(worldToLvglMapX(tagPosition.x), LVGL_MAP_MARGIN, LVGL_MAP_W - LVGL_MAP_MARGIN);
    tagY = constrain(worldToLvglMapY(tagPosition.y), LVGL_MAP_MARGIN, LVGL_MAP_H - LVGL_MAP_MARGIN);
    lv_obj_clear_flag(tagDot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(tagLabel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(tagDot, tagX - 7, tagY - 7);
    lv_obj_set_pos(tagLabel, constrain(tagX + 10, 2, LVGL_MAP_W - 88),
                   constrain(tagY + 7, 2, LVGL_MAP_H - 18));
    lv_label_set_text_fmt(tagLabel, "T%d (%d, %d)", TARGET_TAG_ID,
                          (int)lroundf(tagPosition.x), (int)lroundf(tagPosition.y));
  } else {
    lv_obj_add_flag(tagDot, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(tagLabel, LV_OBJ_FLAG_HIDDEN);
  }

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    if (hasPosition && rangeAvailable[i]) {
      rangeLinePoints[i][0].x = worldToLvglMapX(anchors[i].x);
      rangeLinePoints[i][0].y = worldToLvglMapY(anchors[i].y);
      rangeLinePoints[i][1].x = tagX;
      rangeLinePoints[i][1].y = tagY;
      lv_line_set_points(rangeLines[i], rangeLinePoints[i], 2);
      lv_obj_clear_flag(rangeLines[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(rangeLines[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
}

void refreshLvglPositionPage() {
  if (!lvglReady || coordinateLabel == nullptr) {
    return;
  }

  bool valid = tagPosition.valid && positionState == STATE_VALID &&
               millis() - lastPositionMs <= POSITION_TIMEOUT_MS;
  if (valid) {
    lv_label_set_text_fmt(coordinateLabel, "X  %d cm     Y  %d cm",
                          (int)lroundf(tagPosition.x), (int)lroundf(tagPosition.y));
  } else {
    lv_label_set_text(coordinateLabel, "X  -- cm     Y  -- cm");
  }

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    if (rangeAvailable[i]) {
      lv_label_set_text_fmt(rangeValueLabels[i], "%d", (int)lroundf(lastRanges[i]));
    } else {
      lv_label_set_text(rangeValueLabels[i], "--");
    }
  }

  const char *state = positionStateText();
  lv_label_set_text(statusLabel, state);
  uint32_t stateColor = (positionState == STATE_VALID && valid) ? UI_OK
                      : (positionState == STATE_WAITING ? UI_WARN : UI_ERROR);
  lv_obj_set_style_text_color(statusLabel, uiColor(stateColor), 0);
  updateLvglMap();
}

void closeAnchorEditor() {
  if (editOverlay != nullptr) {
    lv_obj_del(editOverlay);
  }
  editOverlay = nullptr;
  editInput = nullptr;
  editHintLabel = nullptr;
  editingAnchorInput = -1;
}

void anchorEditorCancelEvent(lv_event_t *event) {
  (void)event;
  closeAnchorEditor();
}

void anchorEditorSaveEvent(lv_event_t *event) {
  (void)event;
  if (editingAnchorInput < 0 || editInput == nullptr) {
    return;
  }

  strncpy(anchorInputs[editingAnchorInput], lv_textarea_get_text(editInput),
          sizeof(anchorInputs[editingAnchorInput]) - 1);
  anchorInputs[editingAnchorInput][sizeof(anchorInputs[editingAnchorInput]) - 1] = '\0';
  if (!applyAnchorInputs()) {
    if (editHintLabel != nullptr) {
      lv_label_set_text(editHintLabel, anchorSetupStatus);
      lv_obj_set_style_text_color(editHintLabel, uiColor(UI_ERROR), 0);
    }
    return;
  }

  syncAnchorInputs();
  saveAnchorConfiguration();
  strcpy(anchorSetupStatus, "Coordinates saved");
  updateSetupValueLabels();
  updateLvglMap();
  closeAnchorEditor();
}

void coordinateKeypadEvent(lv_event_t *event) {
  const char *key = (const char *)lv_event_get_user_data(event);
  if (editInput == nullptr || key == nullptr) {
    return;
  }

  if (strcmp(key, "DEL") == 0) {
    lv_textarea_del_char(editInput);
  } else if (strcmp(key, "CLEAR") == 0) {
    lv_textarea_set_text(editInput, "");
  } else if (strcmp(key, "-") == 0) {
    const char *value = lv_textarea_get_text(editInput);
    if (value[0] == '\0') {
      lv_textarea_add_text(editInput, "-");
    }
  } else {
    lv_textarea_add_text(editInput, key);
  }
}

void showAnchorEditor(int inputIndex) {
  if (editOverlay != nullptr) {
    return;
  }
  editingAnchorInput = inputIndex;
  int anchorIndex = inputIndex / 2;

  editOverlay = lv_obj_create(lv_layer_top());
  lv_obj_set_size(editOverlay, SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_obj_set_pos(editOverlay, 0, 0);
  lv_obj_set_style_bg_color(editOverlay, uiColor(0x000000), 0);
  lv_obj_set_style_bg_opa(editOverlay, LV_OPA_60, 0);
  lv_obj_set_style_border_width(editOverlay, 0, 0);
  lv_obj_clear_flag(editOverlay, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *dialog = lv_obj_create(editOverlay);
  lv_obj_set_pos(dialog, 16, 28);
  lv_obj_set_size(dialog, 288, 424);
  styleSurface(dialog, UI_SURFACE, 10);
  lv_obj_set_style_pad_all(dialog, 12, 0);
  lv_obj_clear_flag(dialog, LV_OBJ_FLAG_SCROLLABLE);

  char title[32];
  snprintf(title, sizeof(title), "EDIT A%d %c", anchors[anchorIndex].id,
           (inputIndex % 2 == 0) ? 'X' : 'Y');
  lv_obj_t *titleLabel = createLabel(dialog, title, UI_TEXT);
  lv_obj_set_pos(titleLabel, 12, 12);

  editHintLabel = createLabel(dialog, "Coordinate in cm", UI_MUTED);
  lv_obj_set_pos(editHintLabel, 12, 35);

  editInput = lv_textarea_create(dialog);
  lv_obj_set_pos(editInput, 12, 62);
  lv_obj_set_size(editInput, 264, 44);
  lv_textarea_set_one_line(editInput, true);
  lv_textarea_set_text(editInput, anchorInputs[inputIndex]);
  lv_textarea_set_accepted_chars(editInput, "-0123456789");
  lv_textarea_set_max_length(editInput, sizeof(anchorInputs[inputIndex]) - 1);
  lv_obj_set_style_bg_color(editInput, uiColor(0xF2F7F7), 0);
  lv_obj_set_style_text_color(editInput, uiColor(UI_BG), 0);
  lv_obj_set_style_radius(editInput, 6, 0);

  static const char *numberKeys[4][3] = {
    {"1", "2", "3"},
    {"4", "5", "6"},
    {"7", "8", "9"},
    {"-", "0", "DEL"},
  };
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 3; ++column) {
      createButton(dialog, 12 + column * 91, 118 + row * 40, 82, 34,
                   numberKeys[row][column], UI_SURFACE_ALT,
                   coordinateKeypadEvent, (void *)numberKeys[row][column]);
    }
  }

  createButton(dialog, 12, 286, 124, 42, "CLEAR", UI_SURFACE_ALT,
               coordinateKeypadEvent, (void *)"CLEAR");
  createButton(dialog, 152, 286, 124, 42, "CANCEL", UI_SURFACE_ALT,
               anchorEditorCancelEvent);
  createButton(dialog, 12, 345, 264, 52, "SAVE COORDINATE", UI_TEAL_DARK,
               anchorEditorSaveEvent);
}

void anchorFieldEvent(lv_event_t *event) {
  int inputIndex = (int)(intptr_t)lv_event_get_user_data(event);
  showAnchorEditor(inputIndex);
}

void showPositionPageEvent(lv_event_t *event) {
  (void)event;
  if (!applyAnchorInputs()) {
    updateSetupValueLabels();
    return;
  }
  saveAnchorConfiguration();
  tagPosition.valid = false;
  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    rangeAvailable[i] = false;
  }
  positionState = STATE_WAITING;
  lv_scr_load_anim(positionScreen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 180, 0, false);
  refreshLvglPositionPage();
}

void showSetupPageEvent(lv_event_t *event) {
  (void)event;
  strcpy(anchorSetupStatus, "Tap a value to edit");
  updateSetupValueLabels();
  lv_scr_load_anim(setupScreen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 180, 0, false);
}

void createPositionScreen() {
  positionScreen = lv_obj_create(nullptr);
  styleSurface(positionScreen, UI_BG, 0);

  lv_obj_t *header = lv_obj_create(positionScreen);
  lv_obj_set_pos(header, 0, 0);
  lv_obj_set_size(header, SCREEN_WIDTH, 58);
  styleSurface(header, UI_TEAL_DARK, 0);
  lv_obj_t *title = createLabel(header, "UWB POSITION", UI_TEXT);
  lv_obj_set_pos(title, 14, 10);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
  lv_obj_t *live = createLabel(header, "LIVE", UI_OK);
  lv_obj_set_pos(live, 16, 37);
  createButton(header, 243, 9, 65, 36, "SETUP", UI_SURFACE_ALT, showSetupPageEvent);

  coordinateLabel = createLabel(positionScreen, "X  -- cm     Y  -- cm", UI_MUTED);
  lv_obj_set_pos(coordinateLabel, 15, 70);
  lv_obj_set_style_text_font(coordinateLabel, &lv_font_montserrat_18, 0);

  lvglMap = lv_obj_create(positionScreen);
  lv_obj_set_pos(lvglMap, LVGL_MAP_X, LVGL_MAP_Y);
  lv_obj_set_size(lvglMap, LVGL_MAP_W, LVGL_MAP_H);
  styleSurface(lvglMap, UI_SURFACE, 8);
  lv_obj_set_style_border_color(lvglMap, uiColor(UI_GRID), 0);
  lv_obj_set_style_border_width(lvglMap, 1, 0);

  for (int i = 1; i < 4; ++i) {
    mapGridVertical[i - 1][0] = {(lv_coord_t)(i * LVGL_MAP_W / 4), 0};
    mapGridVertical[i - 1][1] = {(lv_coord_t)(i * LVGL_MAP_W / 4), LVGL_MAP_H};
    lv_obj_t *line = lv_line_create(lvglMap);
    lv_line_set_points(line, mapGridVertical[i - 1], 2);
    lv_obj_set_style_line_color(line, uiColor(UI_GRID), 0);
    lv_obj_set_style_line_width(line, 1, 0);
    mapGridHorizontal[i - 1][0] = {0, (lv_coord_t)(i * LVGL_MAP_H / 4)};
    mapGridHorizontal[i - 1][1] = {LVGL_MAP_W, (lv_coord_t)(i * LVGL_MAP_H / 4)};
    line = lv_line_create(lvglMap);
    lv_line_set_points(line, mapGridHorizontal[i - 1], 2);
    lv_obj_set_style_line_color(line, uiColor(UI_GRID), 0);
    lv_obj_set_style_line_width(line, 1, 0);
  }

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    rangeLines[i] = lv_line_create(lvglMap);
    lv_line_set_points(rangeLines[i], rangeLinePoints[i], 2);
    lv_obj_set_style_line_color(rangeLines[i], uiColor(UI_TEAL), 0);
    lv_obj_set_style_line_opa(rangeLines[i], LV_OPA_60, 0);
    lv_obj_set_style_line_width(rangeLines[i], 1, 0);
    anchorDots[i] = lv_obj_create(lvglMap);
    lv_obj_set_size(anchorDots[i], 10, 10);
    styleSurface(anchorDots[i], UI_TEXT, 5);
    anchorTextLabels[i] = createLabel(lvglMap, "A", UI_TEXT);
  }
  lv_label_set_text(anchorTextLabels[0], "A0");
  lv_label_set_text(anchorTextLabels[1], "A1");
  lv_label_set_text(anchorTextLabels[2], "A2");

  tagDot = lv_obj_create(lvglMap);
  lv_obj_set_size(tagDot, 14, 14);
  styleSurface(tagDot, UI_TAG, 7);
  lv_obj_set_style_border_color(tagDot, uiColor(UI_TEXT), 0);
  lv_obj_set_style_border_width(tagDot, 2, 0);
  tagLabel = createLabel(lvglMap, "", UI_TAG);
  lv_obj_set_style_text_font(tagLabel, &lv_font_montserrat_12, 0);

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    int x = 14 + i * 99;
    lv_obj_t *card = lv_obj_create(positionScreen);
    lv_obj_set_pos(card, x, 346);
    lv_obj_set_size(card, 93, 60);
    styleSurface(card, UI_SURFACE_ALT, 7);
    char anchorName[8];
    snprintf(anchorName, sizeof(anchorName), "A%d RANGE", anchors[i].id);
    lv_obj_t *name = createLabel(card, anchorName, UI_MUTED);
    lv_obj_set_pos(name, 9, 7);
    rangeValueLabels[i] = createLabel(card, "--", UI_TEXT);
    lv_obj_set_pos(rangeValueLabels[i], 9, 26);
    lv_obj_set_style_text_font(rangeValueLabels[i], &lv_font_montserrat_20, 0);
    lv_obj_t *unit = createLabel(card, "cm", UI_MUTED);
    lv_obj_set_pos(unit, 58, 33);
  }

  lv_obj_t *statusPanel = lv_obj_create(positionScreen);
  lv_obj_set_pos(statusPanel, 14, 420);
  lv_obj_set_size(statusPanel, 292, 43);
  styleSurface(statusPanel, UI_SURFACE, 7);
  lv_obj_t *statusTitle = createLabel(statusPanel, "STATUS", UI_MUTED);
  lv_obj_set_pos(statusTitle, 10, 7);
  statusLabel = createLabel(statusPanel, "Waiting for range", UI_WARN);
  lv_obj_set_pos(statusLabel, 10, 23);
}

void createSetupScreen() {
  setupScreen = lv_obj_create(nullptr);
  styleSurface(setupScreen, UI_BG, 0);

  lv_obj_t *header = lv_obj_create(setupScreen);
  lv_obj_set_pos(header, 0, 0);
  lv_obj_set_size(header, SCREEN_WIDTH, 58);
  styleSurface(header, UI_TEAL_DARK, 0);
  lv_obj_t *title = createLabel(header, "ANCHOR SETUP", UI_TEXT);
  lv_obj_set_pos(title, 14, 10);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
  createButton(header, 254, 9, 54, 36, "MAP", UI_SURFACE_ALT, showPositionPageEvent);

  lv_obj_t *hint = createLabel(setupScreen, "Anchor coordinates in centimetres", UI_MUTED);
  lv_obj_set_pos(hint, 14, 72);

  for (int i = 0; i < ANCHOR_COUNT; ++i) {
    int rowY = 104 + i * 78;
    lv_obj_t *row = lv_obj_create(setupScreen);
    lv_obj_set_pos(row, 14, rowY);
    lv_obj_set_size(row, 292, 64);
    styleSurface(row, UI_SURFACE, 8);
    char anchorName[8];
    snprintf(anchorName, sizeof(anchorName), "A%d", anchors[i].id);
    lv_obj_t *name = createLabel(row, anchorName, UI_TEXT);
    lv_obj_set_pos(name, 11, 21);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_20, 0);

    for (int axis = 0; axis < 2; ++axis) {
      int inputIndex = i * 2 + axis;
      int x = axis == 0 ? 58 : 174;
      lv_obj_t *field = createButton(row, x, 10, 104, 44, "", UI_SURFACE_ALT,
                                     anchorFieldEvent, (void *)(intptr_t)inputIndex);
      lv_obj_t *axisLabel = createLabel(field, axis == 0 ? "X" : "Y", UI_MUTED);
      lv_obj_set_pos(axisLabel, 8, 14);
      setupValueLabels[inputIndex] = createLabel(field, anchorInputs[inputIndex], UI_TEXT);
      lv_obj_set_pos(setupValueLabels[inputIndex], 31, 13);
      lv_obj_set_style_text_font(setupValueLabels[inputIndex], &lv_font_montserrat_18, 0);
    }
  }

  setupStatusLabel = createLabel(setupScreen, "Tap a value to edit", UI_MUTED);
  lv_obj_set_pos(setupStatusLabel, 14, 352);
  lv_obj_set_width(setupStatusLabel, 292);
  lv_obj_set_style_text_align(setupStatusLabel, LV_TEXT_ALIGN_CENTER, 0);
  createButton(setupScreen, 14, 393, 292, 55, "START POSITIONING", UI_TEAL_DARK, showPositionPageEvent);
}

void initLvglUi() {
  lv_init();
  lv_disp_draw_buf_init(&lvglDisplayBuffer, lvglDrawBuffer, nullptr,
                        SCREEN_WIDTH * LVGL_DRAW_BUFFER_LINES);
  lv_disp_drv_init(&lvglDisplayDriver);
  lvglDisplayDriver.hor_res = SCREEN_WIDTH;
  lvglDisplayDriver.ver_res = SCREEN_HEIGHT;
  lvglDisplayDriver.draw_buf = &lvglDisplayBuffer;
  lvglDisplayDriver.flush_cb = lvglFlush;
  lv_disp_drv_register(&lvglDisplayDriver);

  lv_indev_drv_init(&lvglTouchDriver);
  lvglTouchDriver.type = LV_INDEV_TYPE_POINTER;
  lvglTouchDriver.read_cb = lvglTouchRead;
  lv_indev_drv_register(&lvglTouchDriver);

  createPositionScreen();
  createSetupScreen();
  lvglReady = true;
  lv_scr_load(setupScreen);
  updateSetupValueLabels();
  refreshLvglPositionPage();
}

// ==================== Initialization ====================
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-S3 ST7796 MaUWB Positioning Demo");

  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);

  gfx->begin();
  gfx->fillScreen(BLACK);

  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  showStartupLogo();
  loadAnchorConfiguration();
  initUwb();
  initLvglUi();
}

// ==================== Main loop ====================
void loop() {
  handleUwbSerial();

  // Touch is polled by LVGL through lvglTouchRead().
  lv_timer_handler();

  if (millis() - lastDisplayMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayMs = millis();
    refreshLvglPositionPage();
  }
}
