#pragma once
#include <stdbool.h>

#include "machine/machine.h"

/* Reads blob "cfg" from NVS namespace "machine". Any failure returns false and
 * writes machine_settings_defaults() to *out. */
bool settings_store_load(machine_settings_t *out);

/* Packs the settings and commits them to NVS. Returns false on any failure. */
bool settings_store_save(const machine_settings_t *settings);
