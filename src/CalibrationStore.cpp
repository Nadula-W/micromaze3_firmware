#include "CalibrationStore.h"
#include <stddef.h>

namespace MM3 {

bool CalibrationStore::load(CalibrationData &out) {
  Preferences prefs;
  if (!prefs.begin("mm3cal", true)) return false;
  size_t n = prefs.getBytesLength("data");
  // Legacy v3 records end before the appended front-alignment gains.
  if (n != sizeof(CalibrationData) && n != offsetof(CalibrationData, frontKp)) {
    prefs.end();
    return false;
  }
  CalibrationData tmp;
  size_t got = prefs.getBytes("data", &tmp, n);
  prefs.end();
  if (got != n || tmp.magic != CAL_MAGIC || tmp.version != CAL_VERSION) {
    return false;
  }
  out = tmp;
  return true;
}

bool CalibrationStore::save(const CalibrationData &data) {
  Preferences prefs;
  if (!prefs.begin("mm3cal", false)) return false;
  size_t n = prefs.putBytes("data", &data, sizeof(data));
  prefs.end();
  return n == sizeof(data);
}

void CalibrationStore::clear() {
  Preferences prefs;
  if (!prefs.begin("mm3cal", false)) return;
  prefs.clear();
  prefs.end();
}

} // namespace MM3
