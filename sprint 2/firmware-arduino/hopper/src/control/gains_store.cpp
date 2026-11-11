#include <Preferences.h>
#include "gains_store.h"
#include "controllers.h"

#define NAMESPACE "tg-gains"
#define KEY "gains"

static Preferences s_prefs;

void gains_store_load() {
  ctrl_gains_t g;
  s_prefs.begin(NAMESPACE, true);
  // getBytesLength is 0 when the key doesn't exist (and logs nothing in read-only mode)
  bool found = s_prefs.isKey(KEY) && s_prefs.getBytesLength(KEY) == sizeof(g) &&
               s_prefs.getBytes(KEY, &g, sizeof(g)) == sizeof(g);
  s_prefs.end();
  if (found) controller_set_gains(&g);  // refuses anything out of range: defaults stay
}

void gains_store_save() {
  s_prefs.begin(NAMESPACE, false);
  s_prefs.putBytes(KEY, controller_gains(), sizeof(ctrl_gains_t));
  s_prefs.end();
}

void gains_store_clear() {
  s_prefs.begin(NAMESPACE, false);
  s_prefs.remove(KEY);
  s_prefs.end();
}
