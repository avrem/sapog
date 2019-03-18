#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int motor_enc_init(void);
int motor_enc_count(void);
int motor_enc_step(void);
int motor_enc_offset_from_step(int step64);

extern void motor_enc_callback(void);

#ifdef __cplusplus
}
#endif
