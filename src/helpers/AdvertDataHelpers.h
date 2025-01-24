#pragma once

#include <stddef.h>
#include <stdint.h>

#define ADV_TYPE_NONE         0   // unknown
#define ADV_TYPE_CHAT         1
#define ADV_TYPE_REPEATER     2
#define ADV_TYPE_ROOM         3   // New kid on the block!
//FUTURE: 4..15

#define ADV_LATLON_MASK       0x10
#define ADV_BATTERY_MASK      0x20
#define ADV_TEMPERATURE_MASK  0x40
#define ADV_NAME_MASK         0x80

class AdvertDataBuilder {
  uint8_t _type;
  const char* _name;
  int32_t _lat, _lon;
public:
  AdvertDataBuilder(uint8_t adv_type) : _type(adv_type), _name(NULL), _lat(0), _lon(0) { }
  AdvertDataBuilder(uint8_t adv_type, const char* name) : _type(adv_type), _name(name), _lat(0), _lon(0)  { }
  AdvertDataBuilder(uint8_t adv_type, const char* name, double lat, double lon) : 
      _type(adv_type), _name(name), _lat(lat * 1E6), _lon(lon * 1E6)  { }

  /**
   * \brief  encode the given advertisement data.
   * \param app_data  dest array, must be MAX_ADVERT_DATA_SIZE
   * \returns  the encoded length in bytes
   */
  uint8_t encodeTo(uint8_t app_data[]) {
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
};

class AdvertDataParser {
  uint8_t _flags;
  bool _valid;
  char _name[MAX_ADVERT_DATA_SIZE];
  int32_t _lat, _lon;
public:
  AdvertDataParser(const uint8_t app_data[], uint8_t app_data_len) {
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
      }
      _valid = true;
    }
  }

  bool isValid() const { return _valid; }
  uint8_t getType() const { return _flags & 0x0F; }

  bool hasName() const { _name[0] != 0; }
  const char* getName() const { return _name; }

  bool hasLatLon() const { return !(_lat == 0 && _lon == 0); }
  int32_t getIntLat() const { return _lat; }
  int32_t getIntLon() const { return _lon; }
  double getLat() const { return ((double)_lat) / 1000000.0; }
  double getLon() const { return ((double)_lon) / 1000000.0; }
};
