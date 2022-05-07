#ifndef _DATA_MANAGER_H
#define _DATA_MANAGER_H

#include <esp_err.h>
#include "esp32/rom/ets_sys.h"

esp_err_t data_manager_init(void);
esp_err_t data_manager_deinit(void);

#endif // _DATA_MANAGER_H