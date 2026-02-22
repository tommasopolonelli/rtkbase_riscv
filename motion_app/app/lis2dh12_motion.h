#ifndef LIS2DH12_MOTION_H
#define LIS2DH12_MOTION_H

#define DEFAULT_EVENT_DURATION_MS 0
#define DEFAULT_EVENT_THRESHOLD_MG 250

void motion_detection_task(uint32_t event_duration_ms, uint32_t event_threshold_mg);

#endif /* LIS2DH12_MOTION_H */
