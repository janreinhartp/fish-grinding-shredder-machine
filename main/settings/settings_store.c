#include "settings/settings_store.h"

#include <stdint.h>

#include "nvs.h"
#include "settings/settings_codec.h"

#define SETTINGS_NAMESPACE "machine"
#define SETTINGS_KEY "cfg"
#define SETTINGS_MAX_BYTES 64

bool settings_store_load(machine_settings_t *out) {
    if (out == NULL) {
        return false;
    }
    *out = machine_settings_defaults();

    nvs_handle_t handle;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    uint8_t blob[SETTINGS_MAX_BYTES];
    size_t len = sizeof(blob);
    esp_err_t err = nvs_get_blob(handle, SETTINGS_KEY, blob, &len);
    nvs_close(handle);
    if (err != ESP_OK) {
        return false;
    }

    machine_settings_t loaded;
    if (!settings_codec_unpack(blob, len, &loaded)) {
        return false;
    }
    *out = loaded;
    return true;
}

bool settings_store_save(const machine_settings_t *settings) {
    if (settings == NULL) {
        return false;
    }

    uint8_t blob[SETTINGS_MAX_BYTES];
    size_t len = settings_codec_size();
    if (len > sizeof(blob) || !settings_codec_pack(settings, blob, len)) {
        return false;
    }

    nvs_handle_t handle;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_set_blob(handle, SETTINGS_KEY, blob, len);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err == ESP_OK;
}
