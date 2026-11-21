#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "machine/machine.h"

size_t settings_codec_size(void);
bool settings_codec_pack(const machine_settings_t *in, uint8_t *out, size_t out_len);
bool settings_codec_unpack(const uint8_t *in, size_t len, machine_settings_t *out);
