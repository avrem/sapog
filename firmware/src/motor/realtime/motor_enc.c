#include "enc.h"
#include "internal.h"
#include <hal.h>
#include <stdio.h>
#include <unistd.h>
#include <zubax_chibios/config/config.h>

#define ENC_CPR 2048

CONFIG_PARAM_INT("mot_enc_offset",  0, 0, ENC_CPR-1)
CONFIG_PARAM_INT("mot_enc_reverse", 0, 0, 1)

static int _num_pole_pairs;
static int _steps_count;
static int _cnt_per_pole;

static int _motor_offset;
static bool _encoder_reverse; // not-reverse: encoder upcount in motor rotation direction

#define ENC_NUM_SAMPLES 32
volatile uint32_t _pos_abs_samples_left, _pos_abs_sum;

static int _index_offset;

static void init_timer(void)
{
	chSysDisable();

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB1RSTR |=  RCC_APB1RSTR_TIM3RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;

	nvicEnableVector(STM32_TIM3_NUMBER, MOTOR_IRQ_PRIORITY_MASK);

	chSysEnable();
}

#define TIM_CLOCK_HZ       3000000
#define ENC_PWM_PERIOD_MIN ((TIM_CLOCK_HZ / 1000000UL) * 1775UL)
#define ENC_PWM_PERIOD_MAX ((TIM_CLOCK_HZ / 1000000UL) * 1875UL)

static void pwm_callback(uint32_t rise_time, uint32_t fall_time)
{
	static uint32_t prev_rise;
	static bool first_period = true;

	uint32_t period = rise_time - prev_rise;
	prev_rise = rise_time;
	if (first_period) {
		first_period = false;
		return;
	}

	if (period < ENC_PWM_PERIOD_MIN || period > ENC_PWM_PERIOD_MAX)
		return;

	uint32_t pos = (fall_time - rise_time) * 4119 / period; // 4119 PWM clock periods per frame
	if (pos < 16 || pos > 4111)
		return;

	pos -= 16; // 12 PWM clocks init, 4 PWM clocks error detection
	pos = pos * ENC_CPR / 4096;

	if (_pos_abs_samples_left > 0) {
		_pos_abs_sum += pos;
		_pos_abs_samples_left--;
	}
}

CH_FAST_IRQ_HANDLER(STM32_TIM3_HANDLER)
{
	static uint32_t rise_time;

	uint32_t sr = TIM3->SR;
	sr &= TIM3->DIER & STM32_TIM_DIER_IRQ_MASK;
	TIM3->SR = ~sr;

	if (sr & STM32_TIM_SR_CC4IF) {
		if (TIM3->CCMR2 & TIM_CCMR2_CC4S_0)
			rise_time = TIM3->CCR4;
		else
			motor_enc_callback();
	}
	else if (sr & STM32_TIM_SR_CC3IF) {
		if (TIM3->CCMR2 & TIM_CCMR2_CC3S_1) // CH3 mapped to TI4
			pwm_callback(rise_time, TIM3->CCR3);
		else
			_index_offset = TIM3->CCR3;
	}
}

static void start_pwm_capture(void)
{
	TIM3->CR1 = 0;
	TIM3->CR2 = 0;

	TIM3->ARR = 0xFFFF;
	TIM3->PSC = 23; // 3 MHz

	TIM3->CCMR2 = TIM_CCMR2_CC3S_1 | TIM_CCMR2_CC4S_0; // input capture CH4 direct, CH3 indirect
	TIM3->CCER = TIM_CCER_CC3P | TIM_CCER_CC3E | TIM_CCER_CC4E; // enable capture, CH3 falling, CH4 rising

	TIM3->DIER = TIM_DIER_CC3IE | TIM_DIER_CC4IE;

	TIM3->EGR = TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_CEN;
}

static void stop_timer(void)
{
	// clear SR to drop all pending interrupts?..
	TIM3->CR1 = 0;
	TIM3->DIER = 0;
	TIM3->CCER = 0;
}

static void start_encoder(uint32_t pos)
{
	TIM3->ARR = ENC_CPR;
	TIM3->PSC = 0;

	TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0; // 1+2 channels input direct, no filter, no prescaler
	TIM3->CCMR2 = TIM_CCMR2_CC3S_0; // 3 input direct
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

	TIM3->DIER = TIM_DIER_CC3IE;

	TIM3->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // encoder mode 3

	TIM3->EGR = TIM_EGR_UG;
	TIM3->CNT = pos;
	TIM3->CR1 |= TIM_CR1_CEN;
}

int motor_enc_init(void)
{
	_num_pole_pairs = configGet("mot_num_poles") / 2;
	_steps_count = _num_pole_pairs * 6;
	_cnt_per_pole = ENC_CPR / _num_pole_pairs;

	_motor_offset = configGet("mot_enc_offset");
	_encoder_reverse = configGet("mot_enc_reverse");

	init_timer();

	_pos_abs_samples_left = ENC_NUM_SAMPLES;
	_pos_abs_sum = 0;

	start_pwm_capture();

	usleep(300 * 1000); // wait for samples
 
	stop_timer();

	if (_pos_abs_samples_left > 0)
		return -1;

	uint32_t pos = _pos_abs_sum / ENC_NUM_SAMPLES;
	printf("Encoder: samples %lu sum %lu pos %lu\n", _pos_abs_samples_left, _pos_abs_sum, pos);
	start_encoder(pos);

	return 0;
}

int motor_enc_count(void)
{
	int cnt = ((int)TIM3->CNT - _index_offset) & (ENC_CPR - 1);
	return _encoder_reverse ? 2048 - cnt : cnt;
}

static void prime_compare(int cnt)
{
	cnt = _encoder_reverse ? 2048 - cnt : cnt;
	TIM3->CCR4 = (cnt + _index_offset) & (ENC_CPR - 1);
	TIM3->CCER |= TIM_CCER_CC4E;
	TIM3->DIER |= TIM_DIER_CC4IE;
}

int motor_enc_step(void)
{
	int cnt = motor_enc_count();
	cnt = (cnt - _motor_offset) & (ENC_CPR - 1);

	int abs_step = cnt * _steps_count / ENC_CPR;
	int step = abs_step % 6;

	int next_step = abs_step + 1;
	if (next_step >= _steps_count)
		next_step = 0;

	int next_count = (next_step * ENC_CPR + _steps_count - 1) / _steps_count;
	next_count = (next_count + _motor_offset) & (ENC_CPR - 1);
	prime_compare(next_count);

	return step;
}

int motor_enc_offset_from_step(int step64)
{
	int pole_cnt64 = (motor_enc_count() * 64) % (_cnt_per_pole * 64);
	int expected64 = step64 * _cnt_per_pole / 6;

	int delta = (pole_cnt64 - expected64 + 32) / 64;
	if (delta < 0)
		delta += _cnt_per_pole;
	else if (delta >= _cnt_per_pole)
		delta -= _cnt_per_pole;
	return delta;
}
