#include "base.h"
#include <math.h>
#include "sound.h"

Sound::Sound(int sample_rate, MODE mode, float ay_volume, float speaker_volume, float tape_volume) : sample_rate(sample_rate), mode(mode) {
	p_snd = new unsigned short[sample_rate * 2]();
    cpu_factor = sample_rate / (float)Z80_FREQ;
    ay_add = AY_RATE / (float)sample_rate;
    set_lpf(22050);
	set_volume(ay_volume, speaker_volume, tape_volume);
    // MONO
	mixer[MONO][LEFT][A] = 1.0;
	mixer[MONO][LEFT][B] = 1.0;
	mixer[MONO][LEFT][C] = 1.0;
	mixer[MONO][RIGHT][A] = 1.0;
	mixer[MONO][RIGHT][B] = 1.0;
	mixer[MONO][RIGHT][C] = 1.0;
    // ABC
	mixer[ACB][LEFT][A] = 1.0;
	mixer[ACB][LEFT][B] = 0.7;
	mixer[ACB][LEFT][C] = 0.5;
	mixer[ACB][RIGHT][A] = 0.5;
	mixer[ACB][RIGHT][B] = 0.7;
	mixer[ACB][RIGHT][C] = 1.0;
	// ACB
	mixer[ABC][LEFT][A] = 1.0;
	mixer[ABC][LEFT][B] = 0.5;
	mixer[ABC][LEFT][C] = 0.7;
	mixer[ABC][RIGHT][A] = 0.5;
	mixer[ABC][RIGHT][B] = 1.0;
	mixer[ABC][RIGHT][C] = 0.7;
	reset();
}

Sound::~Sound(){
	DELETE_ARRAY(p_snd);
}

void Sound::set_ay_volume(float volume){
	for (int i = 0; i < 0x10; i++)
		volume_table[i] = 0xFFFF * volume_level[i] * volume;
}

void Sound::set_speaker_volume(float volume){
	speaker_volume = 0xFFFF * volume;
}

void Sound::set_tape_volume(float volume){
	tape_volume = 0xFFFF * volume;
}

void Sound::set_volume(float ay_volume, float speaker_volume, float tape_volume){
	set_ay_volume(ay_volume);
	set_speaker_volume(speaker_volume);
	set_tape_volume(tape_volume);
}

void Sound::set_lpf(int cut_rate){
    float RC = 1.0 / (cut_rate * 2 * M_PI);
    float dt = 1.0 / sample_rate;
    float alpha = dt / (RC + dt);
    lpf = alpha * (1U << FRACT_BITS);
}

void Sound::update(int clk){
	unsigned int mix_l, mix_r, vol;
	for (int now = clk * cpu_factor; idx < now; idx++){
		tone_a_cnt += ay_add;
		if (tone_a_cnt >= tone_a_max){
			tone_a_cnt -= tone_a_max;
			tone_a ^= -1;
		}
		tone_b_cnt += ay_add;
		if (tone_b_cnt >= tone_b_max){
			tone_b_cnt -= tone_b_max;
			tone_b ^= -1;
		}
		tone_c_cnt += ay_add;
		if (tone_c_cnt >= tone_c_max){
			tone_c_cnt -= tone_c_max;
			tone_c ^= -1;
		}
		noise_cnt += ay_add;
		if (noise_cnt >= noise_max){
			noise_cnt -= noise_max;
			/* The input to the shift register is bit0 XOR bit3 (bit0 is the output).
 			This was verified on AY-3-8910 and YM2149 chips.*/
			/*
            noise_seed >>= 1;
			noise = noise_seed & 0x01 ? 0x00 : 0xFF;
            */
			/* The Random Number Generator of the 8910 is a 17-bit shift register.
			 Hacker KAY & Sergey Bulba*/
			/*
            noise_seed = ((noise_seed << 1) + 1) ^ (((noise_seed >> 16) ^ (noise_seed >> 13)) & 0x01);
			noise = (noise_seed >> 16) & 0x01 ? 0x00 : 0xFF;
            */
			if ((noise_seed & 0x01) ^ ((noise_seed & 0x02) >> 1))
				noise ^= -1;
			if (noise_seed & 0x01)
				noise_seed ^= 0x24000;
			noise_seed >>= 1;
		}
		envelope_cnt += ay_add;
		if (envelope_cnt >= envelope_max){
			envelope_cnt -= envelope_max;
			if (envelope_idx < 0x1F)
            	envelope_idx++;
            else{
				if (reg[ENV_SHAPE] < 4 || reg[ENV_SHAPE] & 0x01)
                	envelope_idx = 0x10;
            	else
                	envelope_idx = 0x00;
            }
			envelope = envelope_shape[reg[ENV_SHAPE] * 0x20 + envelope_idx];
		}

        mix_l = mix_r = 0;
		if ((tone_a | (reg[MIX] & 0x01)) && (noise | (reg[MIX] & 0x08))){
        	vol = reg[A_VOL] & 0x10 ? volume_table[envelope] : volume_table[reg[A_VOL] & 0x0F];
        	mix_l += vol * mixer[mode][LEFT][A];
        	mix_r += vol * mixer[mode][RIGHT][A];
		}
		if ((tone_b | (reg[MIX] & 0x02)) && (noise | (reg[MIX] & 0x10))){
        	vol = reg[B_VOL] & 0x10 ? volume_table[envelope] : volume_table[reg[B_VOL] & 0x0F];
        	mix_l += vol * mixer[mode][LEFT][B];
        	mix_r += vol * mixer[mode][RIGHT][B];
		}
		if ((tone_c | (reg[MIX] & 0x04)) && (noise | (reg[MIX] & 0x20))){
        	vol = reg[C_VOL] & 0x10 ? volume_table[envelope] : volume_table[reg[C_VOL] & 0x0F];
        	mix_l += vol * mixer[mode][LEFT][C];
        	mix_r += vol * mixer[mode][RIGHT][C];
		}
        mix_l = (mix_l + speaker + tape) >> 3;
        mix_r = (mix_r + speaker + tape) >> 3;
        left += lpf * (mix_l - (left >> FRACT_BITS));
        right += lpf * (mix_r - (right >> FRACT_BITS));
        p_snd[idx * 2] = left >> FRACT_BITS;
        p_snd[idx * 2 + 1] = right >> FRACT_BITS;
	}
}

bool Sound::io_wr(unsigned short port, unsigned char byte, int clk){
	if (!(port & 0x01)){
        unsigned char bits = pwFE ^ byte;
        if (bits & (SPEAKER | TAPE_OUT)){
            update(clk);
            if (bits & SPEAKER)
                speaker ^= speaker_volume;
            if (bits & TAPE_OUT)
           	    tape ^= tape_volume;
            pwFE = byte;
		}
    }else{
        if ((port & 0xC002) == 0xC000){
            pwFFFD = byte & 0x0F;
            return true;
        }else{
            if ((port & 0xC002) == 0x8000){
                update(clk);
                reg[pwFFFD] = byte;
				switch(pwFFFD){
					case A_PCH_H:
                        reg[A_PCH_H] &= 0x0F;
					case A_PCH_L:
						tone_a_max = reg[A_PCH_H] << 0x08 | reg[A_PCH_L];
						break;
					case B_PCH_H:
                        reg[B_PCH_H] &= 0x0F;
                    case B_PCH_L:
						tone_b_max = reg[B_PCH_H] << 0x08 | reg[B_PCH_L];
						break;
					case C_PCH_H:
                        reg[C_PCH_H] &= 0x0F;
					case C_PCH_L:
						tone_c_max = reg[C_PCH_H] << 0x08 | reg[C_PCH_L];
						break;
					case N_PCH:
						noise_max = reg[N_PCH] & 0x1F;// * 2;
						break;
                    case MIX: // bit (0 - 7/5?)
						break;
					case A_VOL:
						reg[A_VOL] &= 0x1F;
						break;
					case B_VOL:
						reg[B_VOL] &= 0x1F;
						break;
					case C_VOL:
						reg[C_VOL] &= 0x1F;
						break;
					case ENV_L:
					case ENV_H:
						envelope_max = reg[ENV_H] << 0x8 | reg[ENV_L];
						break;
					case ENV_SHAPE:
						reg[ENV_SHAPE] &= 0x0F;
						envelope = envelope_shape[reg[ENV_SHAPE] * 0x20];
						envelope_idx = 0x00;
						break;
					case PORT_A:
                        break;
					case PORT_B:
						break;
				}
                return true;
			}
		}
	}
    return false;
}

bool Sound::io_rd(unsigned short port, unsigned char *p_byte, int clk){
    if (!(port & 0x01)){
        if ((prFE ^ *p_byte) & TAPE_IN){
            update(clk);
            tape ^= tape_volume;
            prFE = *p_byte;
        }
    }
    if ((port & 0xC003) == 0xC001){
		*p_byte &= reg[pwFFFD];
        return true;
    }
    return false;
}

void Sound::reset(){
	tone_a = tone_b = tone_c = noise = envelope = 0;
    tone_a_cnt = tone_b_cnt = tone_c_cnt = noise_cnt = envelope_cnt = 0;
    tone_a_max = tone_b_max = tone_c_max = noise_max = envelope_max = 0xFFF;
	noise_seed = 12345;
    envelope_idx = 0;
    speaker = 0;
    tape = 0;
	left = 0;
    right = 0;
    for (pwFFFD = 0; pwFFFD < 0x10; pwFFFD++)
        reg[pwFFFD] = 0x0;
    pwFFFD = prFE = pwFE = 0;
    idx = 0;
}
void Sound::frame(int frame_clk){
	update(frame_clk);
	idx = 0;
}

