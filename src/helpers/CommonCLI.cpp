#include <Arduino.h>
#include "CommonCLI.h"
#include "TxtDataHelpers.h"
#include <RTClib.h>
#define MIN_LOCAL_ADVERT_INTERVAL   60

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

enum CommandType {
  CMD_UNKNOWN = 0,
  // GET commands
  CMD_GET_AF,
  CMD_GET_INT_THRESH,
  CMD_GET_AGC_RESET_INTERVAL,
  CMD_GET_MULTI_ACKS,
  CMD_GET_ALLOW_READ_ONLY,
  CMD_GET_FLOOD_ADVERT_INTERVAL,
  CMD_GET_ADVERT_INTERVAL,
  CMD_GET_GUEST_PASSWORD,
  CMD_GET_PRV_KEY,
  CMD_GET_NAME,
  CMD_GET_REPEAT,
  CMD_GET_LAT,
  CMD_GET_LON,
  CMD_GET_RADIO,
  CMD_GET_RXDELAY,
  CMD_GET_TXDELAY,
  CMD_GET_FLOOD_MAX,
  CMD_GET_DIRECT_TXDELAY,
  CMD_GET_TX,
  CMD_GET_FREQ,
  CMD_GET_PUBLIC_KEY,
  CMD_GET_ROLE,
  // SET commands
  CMD_SET_AF,
  CMD_SET_INT_THRESH,
  CMD_SET_AGC_RESET_INTERVAL,
  CMD_SET_MULTI_ACKS,
  CMD_SET_ALLOW_READ_ONLY,
  CMD_SET_FLOOD_ADVERT_INTERVAL,
  CMD_SET_ADVERT_INTERVAL,
  CMD_SET_GUEST_PASSWORD,
  CMD_SET_PRV_KEY,
  CMD_SET_NAME,
  CMD_SET_REPEAT,
  CMD_SET_RADIO,
  CMD_SET_LAT,
  CMD_SET_LON,
  CMD_SET_RXDELAY,
  CMD_SET_TXDELAY,
  CMD_SET_FLOOD_MAX,
  CMD_SET_DIRECT_TXDELAY,
  CMD_SET_TX,
  CMD_SET_FREQ
};

struct ValidationResult {
  bool valid;
  const char* error_msg;
  
  ValidationResult(bool v = false, const char* msg = nullptr) : valid(v), error_msg(msg) {}
};

// Case insensitive hash function for command strings
static uint32_t hashCommandCI(const char* cmd, int max_len) {
  uint32_t hash = 0;
  for (int i = 0; i < max_len && cmd[i] != '\0' && cmd[i] != ' '; i++) {
    char c = cmd[i];
    if (c >= 'A' && c <= 'Z') {
      c = c + ('a' - 'A');
    }
    hash = hash * 31 + c;
  }
  return hash;
}

// Case insensitive memcmp
static int memcmp_ci(const char* s1, const char* s2, size_t n) {
  for (size_t i = 0; i < n; i++) {
    char c1 = s1[i];
    char c2 = s2[i];
    if (c1 >= 'A' && c1 <= 'Z') c1 = c1 + ('a' - 'A');
    if (c2 >= 'A' && c2 <= 'Z') c2 = c2 + ('a' - 'A');
    
    if (c1 != c2) return c1 - c2;
  }
  return 0;
}

static CommandType parseCommand(const char* command) {
  // Determine GET or SET 
  bool is_get = (memcmp_ci(command, "get ", 4) == 0);
  bool is_set = (memcmp_ci(command, "set ", 4) == 0);
  if (!is_get && !is_set) return CMD_UNKNOWN;

  const char* param = &command[4];

  // Lookup tables for GET and SET tokens
  struct CmdEntry { const char* name; uint8_t len; CommandType type; };

  static const CmdEntry get_cmds[] = {
    {"af", 2, CMD_GET_AF},
    {"int.thresh", 10, CMD_GET_INT_THRESH},
    {"agc.reset.interval", 18, CMD_GET_AGC_RESET_INTERVAL},
    {"multi.acks", 10, CMD_GET_MULTI_ACKS},
    {"allow.read.only", 15, CMD_GET_ALLOW_READ_ONLY},
    {"flood.advert.interval", 21, CMD_GET_FLOOD_ADVERT_INTERVAL},
    {"advert.interval", 15, CMD_GET_ADVERT_INTERVAL},
    {"guest.password", 14, CMD_GET_GUEST_PASSWORD},
    {"prv.key", 7, CMD_GET_PRV_KEY},
    {"name", 4, CMD_GET_NAME},
    {"repeat", 6, CMD_GET_REPEAT},
    {"lat", 3, CMD_GET_LAT},
    {"lon", 3, CMD_GET_LON},
    {"radio", 5, CMD_GET_RADIO},
    {"rxdelay", 7, CMD_GET_RXDELAY},
    {"txdelay", 7, CMD_GET_TXDELAY},
    {"flood.max", 9, CMD_GET_FLOOD_MAX},
    {"direct.txdelay", 14, CMD_GET_DIRECT_TXDELAY},
    {"tx", 2, CMD_GET_TX},
    {"freq", 4, CMD_GET_FREQ},
    {"public.key", 10, CMD_GET_PUBLIC_KEY},
    {"role", 4, CMD_GET_ROLE},
    {nullptr, 0, CMD_UNKNOWN}
  };

  static const CmdEntry set_cmds[] = {
    {"af ", 3, CMD_SET_AF},
    {"int.thresh ", 11, CMD_SET_INT_THRESH},
    {"agc.reset.interval ", 19, CMD_SET_AGC_RESET_INTERVAL},
    {"multi.acks ", 11, CMD_SET_MULTI_ACKS},
    {"allow.read.only ", 16, CMD_SET_ALLOW_READ_ONLY},
    {"flood.advert.interval ", 22, CMD_SET_FLOOD_ADVERT_INTERVAL},
    {"advert.interval ", 16, CMD_SET_ADVERT_INTERVAL},
    {"guest.password ", 15, CMD_SET_GUEST_PASSWORD},
    {"prv.key ", 8, CMD_SET_PRV_KEY},
    {"name ", 5, CMD_SET_NAME},
    {"repeat ", 7, CMD_SET_REPEAT},
    {"radio ", 6, CMD_SET_RADIO},
    {"lat ", 4, CMD_SET_LAT},
    {"lon ", 4, CMD_SET_LON},
    {"rxdelay ", 8, CMD_SET_RXDELAY},
    {"txdelay ", 8, CMD_SET_TXDELAY},
    {"flood.max ", 10, CMD_SET_FLOOD_MAX},
    {"direct.txdelay ", 15, CMD_SET_DIRECT_TXDELAY},
    {"tx ", 3, CMD_SET_TX},
    {"freq ", 5, CMD_SET_FREQ},
    {nullptr, 0, CMD_UNKNOWN}
  };

  if (is_get) {
    for (const CmdEntry* e = get_cmds; e->name; ++e) {
      if (memcmp_ci(param, e->name, e->len) == 0) {
        // Allow exact match or trailing space
        if (param[e->len] == 0 || param[e->len] == ' ') return e->type;
      }
    }
  } else { // is_set
    for (const CmdEntry* e = set_cmds; e->name; ++e) {
      if (memcmp_ci(param, e->name, e->len) == 0) return e->type;
    }
  }

  return CMD_UNKNOWN;
}

// Extract value from command string based on command type
static const char* extractValue(const char* command, CommandType cmd) {
  switch (cmd) {
    case CMD_SET_AF: return &command[7];           // "set af "
    case CMD_SET_INT_THRESH: return &command[15];  // "set int.thresh "
    case CMD_SET_AGC_RESET_INTERVAL: return &command[23]; // "set agc.reset.interval "
    case CMD_SET_MULTI_ACKS: return &command[15];  // "set multi.acks "
    case CMD_SET_ALLOW_READ_ONLY: return &command[20]; // "set allow.read.only "
    case CMD_SET_FLOOD_ADVERT_INTERVAL: return &command[26]; // "set flood.advert.interval "
    case CMD_SET_ADVERT_INTERVAL: return &command[20]; // "set advert.interval "
    case CMD_SET_GUEST_PASSWORD: return &command[19]; // "set guest.password "
    case CMD_SET_PRV_KEY: return &command[12];     // "set prv.key "
    case CMD_SET_NAME: return &command[9];         // "set name "
    case CMD_SET_REPEAT: return &command[11];      // "set repeat "
    case CMD_SET_RADIO: return &command[10];       // "set radio "
    case CMD_SET_LAT: return &command[8];          // "set lat "
    case CMD_SET_LON: return &command[8];          // "set lon "
    case CMD_SET_RXDELAY: return &command[12];     // "set rxdelay "
    case CMD_SET_TXDELAY: return &command[12];     // "set txdelay "
    case CMD_SET_FLOOD_MAX: return &command[14];   // "set flood.max "
    case CMD_SET_DIRECT_TXDELAY: return &command[19]; // "set direct.txdelay "
    case CMD_SET_TX: return &command[7];           // "set tx "
    case CMD_SET_FREQ: return &command[9];         // "set freq "
    default: return nullptr;
  }
}

// Validation function with case switch
static ValidationResult validateInput(CommandType cmd, const char* value) {
  switch (cmd) {
    case CMD_SET_AF:
    case CMD_SET_LAT:
    case CMD_SET_LON:
    case CMD_SET_FREQ: {
      float f = atof(value);
      if (f == 0.0f && value[0] != '0' && value[0] != '.' && value[0] != '-') {
        return ValidationResult(false, "Error, use: set PARAM NUMBER");
      }
      break;
    }
    
    case CMD_SET_RXDELAY:
    case CMD_SET_TXDELAY:
    case CMD_SET_DIRECT_TXDELAY: {
      float f = atof(value);
      if (f == 0.0f && value[0] != '0' && value[0] != '.') {
        return ValidationResult(false, "Error, use: set PARAM NUMBER");
      }
      if (f < 0) {
        return ValidationResult(false, "Error, cannot be negative");
      }
      break;
    }
    
    case CMD_SET_INT_THRESH:
    case CMD_SET_AGC_RESET_INTERVAL:
    case CMD_SET_MULTI_ACKS:
    case CMD_SET_TX: {
      int i = atoi(value);
      if (i == 0 && value[0] != '0') {
        return ValidationResult(false, "Error, use: set PARAM NUMBER");
      }
      break;
    }
    
    case CMD_SET_FLOOD_MAX: {
      int i = atoi(value);
      if (i == 0 && value[0] != '0') {
        return ValidationResult(false, "Error, use: set flood.max NUMBER");
      }
      if (i > 64) {
        return ValidationResult(false, "Error, max 64");
      }
      break;
    }
    
    case CMD_SET_FLOOD_ADVERT_INTERVAL: {
      int hours = _atoi(value);
      if ((hours > 0 && hours < 3) || (hours > 48)) {
        return ValidationResult(false, "Error: interval range is 3-48 hours");
      }
      break;
    }
    
    case CMD_SET_ADVERT_INTERVAL: {
      int mins = _atoi(value);
      if ((mins > 0 && mins < MIN_LOCAL_ADVERT_INTERVAL) || (mins > 240)) {
        static char err_msg[64];
        sprintf(err_msg, "Error: interval range is %d-240 minutes", MIN_LOCAL_ADVERT_INTERVAL);
        return ValidationResult(false, err_msg);
      }
      break;
    }
    
    default:
      // String parameters and other types don't need validation
      break;
  }
  
  return ValidationResult(true, nullptr); // Success
}

void CommonCLI::loadPrefs(FILESYSTEM* fs) {
  if (fs->exists("/com_prefs")) {
    loadPrefsInt(fs, "/com_prefs");   // new filename
  } else if (fs->exists("/node_prefs")) {
    loadPrefsInt(fs, "/node_prefs");
    savePrefs(fs);  // save to new filename
    fs->remove("/node_prefs");  // remove old
  }
}

void CommonCLI::loadPrefsInt(FILESYSTEM* fs, const char* filename) {
#if defined(RP2040_PLATFORM)
  File file = fs->open(filename, "r");
#else
  File file = fs->open(filename);
#endif
  if (file) {
    uint8_t pad[8];

    file.read((uint8_t *) &_prefs->airtime_factor, sizeof(_prefs->airtime_factor));  // 0
    file.read((uint8_t *) &_prefs->node_name, sizeof(_prefs->node_name));  // 4
    file.read(pad, 4);   // 36
    file.read((uint8_t *) &_prefs->node_lat, sizeof(_prefs->node_lat));  // 40
    file.read((uint8_t *) &_prefs->node_lon, sizeof(_prefs->node_lon));  // 48
    file.read((uint8_t *) &_prefs->password[0], sizeof(_prefs->password));  // 56
    file.read((uint8_t *) &_prefs->freq, sizeof(_prefs->freq));   // 72
    file.read((uint8_t *) &_prefs->tx_power_dbm, sizeof(_prefs->tx_power_dbm));  // 76
    file.read((uint8_t *) &_prefs->disable_fwd, sizeof(_prefs->disable_fwd));  // 77
    file.read((uint8_t *) &_prefs->advert_interval, sizeof(_prefs->advert_interval));  // 78
    file.read((uint8_t *) pad, 1);  // 79  was 'unused'
    file.read((uint8_t *) &_prefs->rx_delay_base, sizeof(_prefs->rx_delay_base));  // 80
    file.read((uint8_t *) &_prefs->tx_delay_factor, sizeof(_prefs->tx_delay_factor));  // 84
    file.read((uint8_t *) &_prefs->guest_password[0], sizeof(_prefs->guest_password));  // 88
    file.read((uint8_t *) &_prefs->direct_tx_delay_factor, sizeof(_prefs->direct_tx_delay_factor));  // 104
    file.read(pad, 4);   // 108
    file.read((uint8_t *) &_prefs->sf, sizeof(_prefs->sf));  // 112
    file.read((uint8_t *) &_prefs->cr, sizeof(_prefs->cr));  // 113
    file.read((uint8_t *) &_prefs->allow_read_only, sizeof(_prefs->allow_read_only));  // 114
    file.read((uint8_t *) &_prefs->multi_acks, sizeof(_prefs->multi_acks));  // 115
    file.read((uint8_t *) &_prefs->bw, sizeof(_prefs->bw));  // 116
    file.read((uint8_t *) &_prefs->agc_reset_interval, sizeof(_prefs->agc_reset_interval));  // 120
    file.read(pad, 3);   // 121
    file.read((uint8_t *) &_prefs->flood_max, sizeof(_prefs->flood_max));   // 124
    file.read((uint8_t *) &_prefs->flood_advert_interval, sizeof(_prefs->flood_advert_interval));  // 125
    file.read((uint8_t *) &_prefs->interference_threshold, sizeof(_prefs->interference_threshold));  // 126

    // sanitise bad pref values
    _prefs->rx_delay_base = constrain(_prefs->rx_delay_base, 0, 20.0f);
    _prefs->tx_delay_factor = constrain(_prefs->tx_delay_factor, 0, 2.0f);
    _prefs->direct_tx_delay_factor = constrain(_prefs->direct_tx_delay_factor, 0, 2.0f);
    _prefs->airtime_factor = constrain(_prefs->airtime_factor, 0, 9.0f);
    _prefs->freq = constrain(_prefs->freq, 400.0f, 2500.0f);
    _prefs->bw = constrain(_prefs->bw, 62.5f, 500.0f);
    _prefs->sf = constrain(_prefs->sf, 7, 12);
    _prefs->cr = constrain(_prefs->cr, 5, 8);
    _prefs->tx_power_dbm = constrain(_prefs->tx_power_dbm, 1, 30);
    _prefs->multi_acks = constrain(_prefs->multi_acks, 0, 1);

    file.close();
  }
}

void CommonCLI::savePrefs(FILESYSTEM* fs) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  fs->remove("/com_prefs");
  File file = fs->open("/com_prefs", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File file = fs->open("/com_prefs", "w");
#else
  File file = fs->open("/com_prefs", "w", true);
#endif
  if (file) {
    uint8_t pad[8];
    memset(pad, 0, sizeof(pad));

    file.write((uint8_t *) &_prefs->airtime_factor, sizeof(_prefs->airtime_factor));  // 0
    file.write((uint8_t *) &_prefs->node_name, sizeof(_prefs->node_name));  // 4
    file.write(pad, 4);   // 36
    file.write((uint8_t *) &_prefs->node_lat, sizeof(_prefs->node_lat));  // 40
    file.write((uint8_t *) &_prefs->node_lon, sizeof(_prefs->node_lon));  // 48
    file.write((uint8_t *) &_prefs->password[0], sizeof(_prefs->password));  // 56
    file.write((uint8_t *) &_prefs->freq, sizeof(_prefs->freq));   // 72
    file.write((uint8_t *) &_prefs->tx_power_dbm, sizeof(_prefs->tx_power_dbm));  // 76
    file.write((uint8_t *) &_prefs->disable_fwd, sizeof(_prefs->disable_fwd));  // 77
    file.write((uint8_t *) &_prefs->advert_interval, sizeof(_prefs->advert_interval));  // 78
    file.write((uint8_t *) pad, 1);  // 79  was 'unused'
    file.write((uint8_t *) &_prefs->rx_delay_base, sizeof(_prefs->rx_delay_base));  // 80
    file.write((uint8_t *) &_prefs->tx_delay_factor, sizeof(_prefs->tx_delay_factor));  // 84
    file.write((uint8_t *) &_prefs->guest_password[0], sizeof(_prefs->guest_password));  // 88
    file.write((uint8_t *) &_prefs->direct_tx_delay_factor, sizeof(_prefs->direct_tx_delay_factor));  // 104
    file.write(pad, 4);   // 108
    file.write((uint8_t *) &_prefs->sf, sizeof(_prefs->sf));  // 112
    file.write((uint8_t *) &_prefs->cr, sizeof(_prefs->cr));  // 113
    file.write((uint8_t *) &_prefs->allow_read_only, sizeof(_prefs->allow_read_only));  // 114
    file.write((uint8_t *) &_prefs->multi_acks, sizeof(_prefs->multi_acks));  // 115
    file.write((uint8_t *) &_prefs->bw, sizeof(_prefs->bw));  // 116
    file.write((uint8_t *) &_prefs->agc_reset_interval, sizeof(_prefs->agc_reset_interval));  // 120
    file.write(pad, 3);   // 121
    file.write((uint8_t *) &_prefs->flood_max, sizeof(_prefs->flood_max));   // 124
    file.write((uint8_t *) &_prefs->flood_advert_interval, sizeof(_prefs->flood_advert_interval));  // 125
    file.write((uint8_t *) &_prefs->interference_threshold, sizeof(_prefs->interference_threshold));  // 126

    file.close();
  }
}

#define MIN_LOCAL_ADVERT_INTERVAL   60

void CommonCLI::savePrefs() {
  if (_prefs->advert_interval * 2 < MIN_LOCAL_ADVERT_INTERVAL) {
    _prefs->advert_interval = 0;  // turn it off, now that device has been manually configured
  }
  _callbacks->savePrefs();
}

// Handle get/set commands with new switch-based system
bool CommonCLI::handleGetSetCommands(uint32_t sender_timestamp, const char* command, char* reply) {
  CommandType cmd = parseCommand(command);
  
  if (cmd == CMD_UNKNOWN) {
    return false; // Not a get/set command we recognize
  }
  
  switch (cmd) {
    // GET commands
    case CMD_GET_AF:
      sprintf(reply, "> %s", StrHelper::ftoa(_prefs->airtime_factor));
      break;
      
    case CMD_GET_INT_THRESH:
      sprintf(reply, "> %d", (uint32_t) _prefs->interference_threshold);
      break;
      
    case CMD_GET_AGC_RESET_INTERVAL:
      sprintf(reply, "> %d", ((uint32_t) _prefs->agc_reset_interval) * 4);
      break;
      
    case CMD_GET_MULTI_ACKS:
      sprintf(reply, "> %d", (uint32_t) _prefs->multi_acks);
      break;
      
    case CMD_GET_ALLOW_READ_ONLY:
      sprintf(reply, "> %s", _prefs->allow_read_only ? "on" : "off");
      break;
      
    case CMD_GET_FLOOD_ADVERT_INTERVAL:
      sprintf(reply, "> %d", ((uint32_t) _prefs->flood_advert_interval));
      break;
      
    case CMD_GET_ADVERT_INTERVAL:
      sprintf(reply, "> %d", ((uint32_t) _prefs->advert_interval) * 2);
      break;
      
    case CMD_GET_GUEST_PASSWORD:
      sprintf(reply, "> %s", _prefs->guest_password);
      break;
      
    case CMD_GET_PRV_KEY:
      if (sender_timestamp == 0) { // from serial command line only
        uint8_t prv_key[PRV_KEY_SIZE];
        int len = _callbacks->getSelfId().writeTo(prv_key, PRV_KEY_SIZE);
        strcpy(reply, "> ");
        mesh::Utils::toHex(&reply[2], prv_key, len);
      } else {
        return false; // Not allowed from remote
      }
      break;
      
    case CMD_GET_NAME:
      sprintf(reply, "> %s", _prefs->node_name);
      break;
      
    case CMD_GET_REPEAT:
      sprintf(reply, "> %s", _prefs->disable_fwd ? "off" : "on");
      break;
      
    case CMD_GET_LAT:
      sprintf(reply, "> %s", StrHelper::ftoa(_prefs->node_lat));
      break;
      
    case CMD_GET_LON:
      sprintf(reply, "> %s", StrHelper::ftoa(_prefs->node_lon));
      break;
      
    case CMD_GET_RADIO: {
      char freq[16], bw[16];
      strcpy(freq, StrHelper::ftoa(_prefs->freq));
      strcpy(bw, StrHelper::ftoa(_prefs->bw));
      sprintf(reply, "> %s,%s,%d,%d", freq, bw, (uint32_t)_prefs->sf, (uint32_t)_prefs->cr);
      break;
    }
    
    case CMD_GET_RXDELAY:
      sprintf(reply, "> %s", StrHelper::ftoa(_prefs->rx_delay_base));
      break;
      
    case CMD_GET_TXDELAY:
      sprintf(reply, "> %s", StrHelper::ftoa(_prefs->tx_delay_factor));
      break;
      
    case CMD_GET_FLOOD_MAX:
      sprintf(reply, "> %d", (uint32_t)_prefs->flood_max);
      break;
      
    case CMD_GET_DIRECT_TXDELAY:
      sprintf(reply, "> %s", StrHelper::ftoa(_prefs->direct_tx_delay_factor));
      break;
      
    case CMD_GET_TX:
      sprintf(reply, "> %d", (uint32_t) _prefs->tx_power_dbm);
      break;
      
    case CMD_GET_FREQ:
      sprintf(reply, "> %s", StrHelper::ftoa(_prefs->freq));
      break;
      
    case CMD_GET_PUBLIC_KEY:
      strcpy(reply, "> ");
      mesh::Utils::toHex(&reply[2], _callbacks->getSelfId().pub_key, PUB_KEY_SIZE);
      break;
      
    case CMD_GET_ROLE:
      sprintf(reply, "> %s", _callbacks->getRole());
      break;
      
    // SET commands with validation
    case CMD_SET_AF: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->airtime_factor = atof(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_INT_THRESH: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->interference_threshold = atoi(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_AGC_RESET_INTERVAL: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        int interval = atoi(value);
        _prefs->agc_reset_interval = interval / 4;
        this->savePrefs();
        sprintf(reply, "OK - interval rounded to %d", ((uint32_t) _prefs->agc_reset_interval) * 4);
      }
      break;
    }
    
    case CMD_SET_MULTI_ACKS: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->multi_acks = atoi(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_ALLOW_READ_ONLY: {
      const char* value = extractValue(command, cmd);
      _prefs->allow_read_only = (memcmp_ci(value, "on", 2) == 0);
      savePrefs();
      strcpy(reply, "OK");
      break;
    }
    
    case CMD_SET_FLOOD_ADVERT_INTERVAL: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        int hours = _atoi(value);
        _prefs->flood_advert_interval = (uint8_t)(hours);
        _callbacks->updateFloodAdvertTimer();
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_ADVERT_INTERVAL: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        int mins = _atoi(value);
        _prefs->advert_interval = (uint8_t)(mins / 2);
        _callbacks->updateAdvertTimer();
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_GUEST_PASSWORD: {
      const char* value = extractValue(command, cmd);
      StrHelper::strncpy(_prefs->guest_password, value, sizeof(_prefs->guest_password));
      savePrefs();
      strcpy(reply, "OK");
      break;
    }
    
    case CMD_SET_PRV_KEY: {
      if (sender_timestamp != 0) {
        return false; // Only from serial command line
      }
      const char* value = extractValue(command, cmd);
      uint8_t prv_key[PRV_KEY_SIZE];
      int hex_len = min((int)strlen(value), PRV_KEY_SIZE*2);
      int prv_key_len = hex_len / 2;
      bool success = mesh::Utils::fromHex(prv_key, PRV_KEY_SIZE, value);
      if (success) {
        mesh::LocalIdentity new_id;
        new_id.readFrom(prv_key, PRV_KEY_SIZE);
        _callbacks->saveIdentity(new_id);
        strcpy(reply, "OK - reboot to apply");
      } else {
        strcpy(reply, "Error, invalid key");
      }
      break;
    }
    
    case CMD_SET_NAME: {
      const char* value = extractValue(command, cmd);
      StrHelper::strncpy(_prefs->node_name, value, sizeof(_prefs->node_name));
      savePrefs();
      strcpy(reply, "OK");
      break;
    }
    
    case CMD_SET_REPEAT: {
      const char* value = extractValue(command, cmd);
      _prefs->disable_fwd = (memcmp_ci(value, "off", 3) == 0);
      savePrefs();
      strcpy(reply, "OK");
      break;
    }
    
    case CMD_SET_RADIO: {
      const char* value = extractValue(command, cmd);
      strcpy(this->tmp, value);
      const char *parts[4];
      int num = mesh::Utils::parseTextParts(this->tmp, parts, 4);
      float freq  = num > 0 ? atof(parts[0]) : 0.0f;
      float bw    = num > 1 ? atof(parts[1]) : 0.0f;
      uint8_t sf  = num > 2 ? atoi(parts[2]) : 0;
      uint8_t cr  = num > 3 ? atoi(parts[3]) : 0;
      if (freq >= 300.0f && freq <= 2500.0f && sf >= 7 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7.0f && bw <= 500.0f) {
        _prefs->sf = sf;
        _prefs->cr = cr;
        _prefs->freq = freq;
        _prefs->bw = bw;
        this->savePrefs();
        _callbacks->savePrefs();
        strcpy(reply, "OK - reboot to apply");
      } else {
        strcpy(reply, "Error, invalid radio params");
      }
      break;
    }
    
    case CMD_SET_LAT: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->node_lat = atof(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_LON: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->node_lon = atof(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_RXDELAY: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->rx_delay_base = atof(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_TXDELAY: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->tx_delay_factor = atof(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_FLOOD_MAX: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->flood_max = atoi(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_DIRECT_TXDELAY: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->direct_tx_delay_factor = atof(value);
        this->savePrefs();
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_TX: {
      const char* value = extractValue(command, cmd);
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->tx_power_dbm = atoi(value);
        this->savePrefs();
        _callbacks->setTxPower(_prefs->tx_power_dbm);
        strcpy(reply, "OK");
      }
      break;
    }
    
    case CMD_SET_FREQ: {
      const char* value = extractValue(command, cmd);
      if (sender_timestamp != 0) {
        return false; // Only from serial command line
      }
      ValidationResult result = validateInput(cmd, value);
      if (!result.valid) {
        strcpy(reply, result.error_msg);
      } else {
        _prefs->freq = atof(value);
        this->savePrefs();
        strcpy(reply, "OK - reboot to apply");
      }
      break;
    }
    
    default:
      sprintf(reply, "??: %s", command);
      break;
  }
  
  return true; // Command was handled
}

void CommonCLI::handleCommand(uint32_t sender_timestamp, const char* command, char* reply) {
    if (memcmp(command, "reboot", 6) == 0) {
      _board->reboot();  // doesn't return
    } else if (memcmp(command, "advert", 6) == 0) {
      _callbacks->sendSelfAdvertisement(1500);  // longer delay, give CLI response time to be sent first
      strcpy(reply, "OK - Advert sent");
    } else if (memcmp(command, "clock sync", 10) == 0) {
      uint32_t curr = getRTCClock()->getCurrentTime();
      if (sender_timestamp > curr) {
        getRTCClock()->setCurrentTime(sender_timestamp + 1);
        uint32_t now = getRTCClock()->getCurrentTime();
        DateTime dt = DateTime(now);
        sprintf(reply, "OK - clock set: %02d:%02d - %d/%d/%d UTC", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
      } else {
        strcpy(reply, "ERR: clock cannot go backwards");
      }
    } else if (memcmp(command, "start ota", 9) == 0) {
      if (!_board->startOTAUpdate(_prefs->node_name, reply)) {
        strcpy(reply, "Error");
      }
    } else if (memcmp(command, "clock", 5) == 0) {
      uint32_t now = getRTCClock()->getCurrentTime();
      DateTime dt = DateTime(now);
      sprintf(reply, "%02d:%02d - %d/%d/%d UTC", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
    } else if (memcmp(command, "time ", 5) == 0) {  // set time (to epoch seconds)
      uint32_t secs = _atoi(&command[5]);
      uint32_t curr = getRTCClock()->getCurrentTime();
      if (secs > curr) {
        getRTCClock()->setCurrentTime(secs);
        uint32_t now = getRTCClock()->getCurrentTime();
        DateTime dt = DateTime(now);
        sprintf(reply, "OK - clock set: %02d:%02d - %d/%d/%d UTC", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
      } else {
        strcpy(reply, "(ERR: clock cannot go backwards)");
      }
    } else if (memcmp(command, "neighbors", 9) == 0) {
      _callbacks->formatNeighborsReply(reply);
    } else if (memcmp(command, "neighbor.remove ", 16) == 0) {
      const char* hex = &command[16];
      uint8_t pubkey[PUB_KEY_SIZE];
      int hex_len = min((int)strlen(hex), PUB_KEY_SIZE*2);
      int pubkey_len = hex_len / 2;
      if (mesh::Utils::fromHex(pubkey, pubkey_len, hex)) {
        _callbacks->removeNeighbor(pubkey, pubkey_len);
        strcpy(reply, "OK");
      } else {
        strcpy(reply, "ERR: bad pubkey");
      }
    } else if (memcmp(command, "tempradio ", 10) == 0) {
      strcpy(tmp, &command[10]);
      const char *parts[5];
      int num = mesh::Utils::parseTextParts(tmp, parts, 5);
      float freq  = num > 0 ? atof(parts[0]) : 0.0f;
      float bw    = num > 1 ? atof(parts[1]) : 0.0f;
      uint8_t sf  = num > 2 ? atoi(parts[2]) : 0;
      uint8_t cr  = num > 3 ? atoi(parts[3]) : 0;
      int temp_timeout_mins  = num > 4 ? atoi(parts[4]) : 0;
      if (freq >= 300.0f && freq <= 2500.0f && sf >= 7 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7.0f && bw <= 500.0f && temp_timeout_mins > 0) {
        _callbacks->applyTempRadioParams(freq, bw, sf, cr, temp_timeout_mins);
        sprintf(reply, "OK - temp params for %d mins", temp_timeout_mins);
      } else {
        strcpy(reply, "Error, invalid params");
      }
    } else if (memcmp(command, "password ", 9) == 0) {
      // change admin password
      StrHelper::strncpy(_prefs->password, &command[9], sizeof(_prefs->password));
      savePrefs();
      sprintf(reply, "password now: %s", _prefs->password);   // echo back just to let admin know for sure!!
    } else if (memcmp(command, "clear stats", 11) == 0) {
      _callbacks->clearStats();
      strcpy(reply, "(OK - stats reset)");
    } else if (handleGetSetCommands(sender_timestamp, command, reply)) {
    } else if (sender_timestamp == 0 && strcmp(command, "erase") == 0) {
      bool s = _callbacks->formatFileSystem();
      sprintf(reply, "File system erase: %s", s ? "OK" : "Err");
    } else if (memcmp(command, "ver", 3) == 0) {
      sprintf(reply, "%s (Build: %s)", _callbacks->getFirmwareVer(), _callbacks->getBuildDate());
    } else if (memcmp(command, "log start", 9) == 0) {
      _callbacks->setLoggingOn(true);
      strcpy(reply, "   logging on");
    } else if (memcmp(command, "log stop", 8) == 0) {
      _callbacks->setLoggingOn(false);
      strcpy(reply, "   logging off");
    } else if (memcmp(command, "log erase", 9) == 0) {
      _callbacks->eraseLogFile();
      strcpy(reply, "   log erased");
    } else if (sender_timestamp == 0 && memcmp(command, "log", 3) == 0) {
      _callbacks->dumpLogFile();
      strcpy(reply, "   EOF");
    } else {
      strcpy(reply, "Unknown command");
    }
}
