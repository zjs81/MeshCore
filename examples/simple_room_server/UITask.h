#pragma once

#include <helpers/ui/DisplayDriver.h>
#include <helpers/CommonCLI.h>

// Time sync status data structure
struct TimeSyncDisplayData {
  bool enabled;
  bool clock_is_set;
  int recent_samples;
  uint32_t current_time;
};

class UITask {
  DisplayDriver* _display;
  unsigned long _next_read, _next_refresh, _auto_off;
  int _prevBtnState;
  NodePrefs* _node_prefs;
  TimeSyncDisplayData _time_sync_data;
  char _version_info[32];

  void renderCurrScreen();
public:
  UITask(DisplayDriver& display) : _display(&display) { 
    _next_read = _next_refresh = 0; 
    _time_sync_data = {false, false, 0, 0};
  }
  void begin(NodePrefs* node_prefs, const char* build_date, const char* firmware_version);
  void updateTimeSyncData(const TimeSyncDisplayData& data);

  void loop();
};