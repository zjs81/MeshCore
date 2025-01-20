#include "IdentityStore.h"

bool IdentityStore::load(const char *name, mesh::LocalIdentity& id) {
  bool loaded = false;
  char filename[40];
  sprintf(filename, "%s/%s.id", _dir, name);
  if (_fs->exists(filename)) {
    File file = _fs->open(filename);
    if (file) {
      loaded = id.readFrom(file);
      file.close();
    }
  }
  return loaded;
}

bool IdentityStore::save(const char *name, const mesh::LocalIdentity& id) {
  char filename[40];
  sprintf(filename, "%s/%s.id", _dir, name);

#if defined(NRF52_PLATFORM)
  File file = _fs->open(filename, FILE_O_WRITE);
  if (file) { file.seek(0); file.truncate(); }
#else
  File file = _fs->open(filename, "w", true);
#endif
  if (file) {
    id.writeTo(file);
    file.close();
    return true;
  }
  return false;
}
