#include "settings/settings_codec.h"
#include "common/crc16.h"

#include <string.h>

#define SETTINGS_VERSION 1u
#define SETTINGS_BYTES 54u

size_t settings_codec_size(void) {
    return SETTINGS_BYTES;
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_fields(uint8_t *p, const machine_settings_t *in) {
    put_u32(p + 0, (uint32_t)in->target_g);
    put_u32(p + 4, (uint32_t)in->minimum_batch_g);
    put_u32(p + 8, (uint32_t)in->tolerance_g);
    put_u32(p + 12, in->stable_ms);
    put_u32(p + 16, (uint32_t)in->reset_threshold_g);
    put_u32(p + 20, in->grinder_startup_ms);
    put_u32(p + 24, in->shredder_startup_ms);
    put_u32(p + 28, in->door_open_ms);
    put_u32(p + 32, in->discharge_ms);
    put_u32(p + 36, in->door_close_ms);
    put_u32(p + 40, in->shredder_run_on_ms);
    put_u32(p + 44, in->grinder_run_on_ms);
}

static void read_fields(const uint8_t *p, machine_settings_t *out) {
    out->target_g = (int32_t)get_u32(p + 0);
    out->minimum_batch_g = (int32_t)get_u32(p + 4);
    out->tolerance_g = (int32_t)get_u32(p + 8);
    out->stable_ms = get_u32(p + 12);
    out->reset_threshold_g = (int32_t)get_u32(p + 16);
    out->grinder_startup_ms = get_u32(p + 20);
    out->shredder_startup_ms = get_u32(p + 24);
    out->door_open_ms = get_u32(p + 28);
    out->discharge_ms = get_u32(p + 32);
    out->door_close_ms = get_u32(p + 36);
    out->shredder_run_on_ms = get_u32(p + 40);
    out->grinder_run_on_ms = get_u32(p + 44);
}

bool settings_codec_pack(const machine_settings_t *in, uint8_t *out, size_t out_len) {
    if (in == NULL || out == NULL || out_len < SETTINGS_BYTES || !machine_settings_valid(in)) {
        return false;
    }
    memset(out, 0, SETTINGS_BYTES);
    put_u32(out, SETTINGS_VERSION);
    write_fields(out + 4, in);
    uint16_t crc = crc16_modbus(out, 52);
    out[52] = (uint8_t)crc;
    out[53] = (uint8_t)(crc >> 8);
    return true;
}

bool settings_codec_unpack(const uint8_t *in, size_t len, machine_settings_t *out) {
    if (in == NULL || out == NULL || len != SETTINGS_BYTES) {
        return false;
    }
    if (get_u32(in) != SETTINGS_VERSION) {
        return false;
    }
    uint16_t got = (uint16_t)(in[52] | (in[53] << 8));
    if (got != crc16_modbus(in, 52)) {
        return false;
    }
    machine_settings_t decoded;
    memset(&decoded, 0, sizeof decoded);
    read_fields(in + 4, &decoded);
    if (!machine_settings_valid(&decoded)) {
        return false;
    }
    *out = decoded;
    return true;
}
