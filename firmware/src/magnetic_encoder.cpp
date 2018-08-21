/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <magnetic_encoder.hpp>
#include <zubax_chibios/os.hpp>
#include <board/board.hpp>
#include <motor/motor.h>
#include <unistd.h>

CONFIG_PARAM_INT("mot_enc_offset",  0,  0, 2047)

namespace magnetic_encoder
{
namespace
{

uint8_t calc_pec(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t shift_reg = 0;
    bool do_invert;

    for (uint8_t i=0; i<len; i++) {
        shift_reg = buf[i];
        for (uint8_t j=0; j<8; j++) {
            do_invert = (crc ^ shift_reg) & 0x80;
            crc <<= 1;
            shift_reg <<= 1;
            if(do_invert)
                crc ^= 0x07;
        }
    }

    return crc;
}

static constexpr unsigned SENSOR_ADDRESS = 0x40;

#define REG_ZOFFSET 0x08
#define REG_POLES   0x09

void configure_encoder(unsigned angle_offset, unsigned num_poles)
{
	std::uint8_t tx[5];
	tx[0] = SENSOR_ADDRESS << 1;

	tx[1] = REG_ZOFFSET;
	*reinterpret_cast<std::uint16_t *>(tx + 2) = angle_offset;
	tx[4] = calc_pec(tx, 4);
	board::i2c_exchange(SENSOR_ADDRESS, tx + 1, 4, NULL, 0);

	tx[1] = REG_POLES;
	tx[2] = num_poles;
	tx[3] = calc_pec(tx, 3);
	board::i2c_exchange(SENSOR_ADDRESS, tx + 1, 3, NULL, 0);
}

class : public chibios_rt::BaseStaticThread<128>
{
	void main() override
	{
		const unsigned num_poles = configGet("mot_num_poles");
		const unsigned angle_offset = configGet("mot_enc_offset");

		while (!os::isRebootRequested()) {
			::usleep(1000 * 1000);

			if (!motor_is_idle()) {
				continue;
			}

			configure_encoder(angle_offset, num_poles);
		}
	}
} thread_;

}

int init()
{
	thread_.start(LOWPRIO);
	return 0;
}

}
