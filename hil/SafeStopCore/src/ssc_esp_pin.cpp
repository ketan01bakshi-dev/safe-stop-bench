// ESP32 only (empty on the PC): keep the esp-mqtt task off core 1, where the 10 ms control loop runs.  (v2.34)
//
// Why: the Arduino core's esp-mqtt is built WITHOUT CONFIG_MQTT_TASK_CORE_SELECTION_ENABLED, so its task is created unpinned and may run on either core.
// Measured on board B (scripts/b_stall_watch.py, hil/firmware/SafetyNode/ssc_diag.h): when it runs on core 1 during a TLS handshake it starves the control
// loop for 130-190 ms, although both are at priority 1 and core 0 is idle. The task holds esp-mqtt's API lock for the whole connect; a short contention on any
// inner mutex with a higher-priority task (the Wi-Fi task, priority 23) raises it to that priority by priority inheritance, and FreeRTOS gives the priority back
// only when the task holds NO mutex, so it stays at 23 for the entire handshake, the loop is ready but never scheduled, and node.poll() then catches up with
// every missed 10 ms cycle at once. Lowering the priority does not help (the inherited one wins). Pinning does: on core 0 the boosted task can only compete with
// the Wi-Fi/TCP/IP tasks that are already there.
//
// How: esp-mqtt calls xTaskCreatePinnedToCore(..., tskNO_AFFINITY); this library's ldflags (library.properties) make the linker send every such call here
// (-Wl,--wrap=xTaskCreatePinnedToCore, the same mechanism the core itself uses for esp_panic_handler). Only the task named "mqtt_task" is changed.
#ifdef ARDUINO_ARCH_ESP32
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" BaseType_t __real_xTaskCreatePinnedToCore(TaskFunction_t, const char *const, const configSTACK_DEPTH_TYPE, void *const, UBaseType_t,
                                                     TaskHandle_t *const, const BaseType_t);

extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(TaskFunction_t fn, const char *const name, const configSTACK_DEPTH_TYPE stack, void *const arg,
                                                     UBaseType_t prio, TaskHandle_t *const handle, const BaseType_t core) {
  const bool mqtt = name && strcmp(name, "mqtt_task") == 0;
  return __real_xTaskCreatePinnedToCore(fn, name, stack, arg, prio, handle, mqtt ? 0 : core);
}
#endif
