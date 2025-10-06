#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

static const char *TAG = "robot_angel";

void app_main(void) {
    while (1) {
        ESP_LOGI(TAG, "Hello from Robot Angel!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
