#include <helpers/AdvertDataHelpers.h>

  uint8_t AdvertDataBuilder::encodeTo(uint8_t app_data[]) {
    app_data[0] = _type;
    int i = 1;
    if (!(_lat == 0 && _lon == 0)) {
      app_data[0] |= ADV_LATLON_MASK;
      memcpy(&app_data[i], &_lat, 4); i += 4;
      memcpy(&app_data[i], &_lon, 4); i += 4;
    }
    // TODO:  BATTERY encoding
    // TODO:  TEMPERATURE encoding
    if (_name && *_name != 0) { 
      app_data[0] |= ADV_NAME_MASK;
      const char* sp = _name;
      while (*sp && i < MAX_ADVERT_DATA_SIZE) {
        app_data[i++] = *sp++;
      }
    }
    return i;
  }

  AdvertDataParser::AdvertDataParser(const uint8_t app_data[], uint8_t app_data_len) {
    _name[0] = 0;
    _lat = _lon = 0;
    _flags = app_data[0];
    _valid = false;

    int i = 1;
    if (_flags & ADV_LATLON_MASK) {
      memcpy(&_lat, &app_data[i], 4); i += 4;
      memcpy(&_lon, &app_data[i], 4); i += 4;
    }
    if (_flags & ADV_BATTERY_MASK) {
      /* TODO: somewhere to store battery volts? */ i += 2;
    }
    if (_flags & ADV_TEMPERATURE_MASK) {
      /* TODO: somewhere to store temperature? */ i += 2;
    }

    if (app_data_len >= i) {
      int nlen = 0;
      if (_flags & ADV_NAME_MASK) {
        nlen = app_data_len - i;  // remainder of app_data
      }
      if (nlen > 0) {
        memcpy(_name, &app_data[i], nlen);
        _name[nlen] = 0;  // set null terminator
  MESH_DEBUG_PRINTLN("AdvertDataParser: _flags=%u, _name=%s", (uint32_t)_flags, _name);
      }
      _valid = true;
    }
  }
