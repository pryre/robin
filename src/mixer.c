#include <stdint.h>

#include <breezystm32/breezystm32.h>

#include "mixer.h"
#include "param.h"
#include "rc.h"

int32_t _GPIO_outputs[8];
static int32_t prescaled_outputs[8];
int32_t _outputs[8];
command_t _command;
output_type_t _GPIO_output_type[8];

static mixer_t quadcopter_plus_mixing =
{
  {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

  { 1.0f,  1.0f,  1.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
  { 0.0f, -1.0f,  1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
  {-1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
  {-1.0f,  1.0f,  1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
};


static mixer_t quadcopter_x_mixing =
{
  {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

  { 1.0f, 1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
  {-1.0f,-1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
  {-1.0f, 1.0f,-1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
  { 1.0f,-1.0f,-1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
};

static mixer_t quadcopter_h_mixing =
{
  {M, M, M, M, NONE, NONE, NONE, NONE}, // output_type

  { 1.0f, 1.0f, 1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
  {-1057, -943, 1057,  943,  0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
  {-1005,  995,-1005,  995,  0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
  { 1.0f,-1.0f,-1.0f, 1.0f,  0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
};

static mixer_t tricopter_mixing =
{
  {M, M, M, S, NONE, NONE, NONE, NONE},

  { 1.0f,     1.0f,   1.0f,   0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // F Mix
  {-1.0f,     1.0f,   0.0f,   0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // X Mix
  {-0.667f,  -0.667f, 1.333f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // Y Mix
  { 0.0f,     0.0f,   0.0f,   1.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // Z Mix
};

static mixer_t Y6_mixing =
{
  {M, M, M, M, M, M, NONE, NONE},
  { 1.0f,   1.0f,    1.0f,    1.0f,    1.0f,    1.0f,   0.0f, 0.0f}, // F Mix
  { 0.0f,  -1.0f,    1.0f,    0.0f,   -1.0f,    1.0f,   0.0f, 0.0f}, // X Mix
  {-1.333f, 0.667f,  0.667f, -1.333f,  0.667f,  0.667f, 0.0f, 0.0f}, // Y Mix
  {-1.0f,   1.0f,    1.0f,    1.0f,   -1.0f,   -1.0f,   0.0f, 0.0f}  // Z Mix
};

static mixer_t mixer_to_use;

static mixer_t *array_of_mixers[NUM_MIXERS] =
{
  &quadcopter_plus_mixing,
  &quadcopter_x_mixing,
  &quadcopter_h_mixing,
  &tricopter_mixing,
  &Y6_mixing,
  &fixedwing_mixing
};



void init_mixing()
{
  // We need a better way to choosing the mixer
  mixer_to_use = *array_of_mixers[get_param_int(PARAM_MIXER)];

  for (int8_t i=0; i<8; i++)
  {
    _outputs[i] = 0;
    prescaled_outputs[i] = 0;
    _GPIO_outputs[i] = 0;
    _GPIO_output_type[i] = NONE;
  }
  _command.F = 0;
  _command.x = 0;
  _command.y = 0;
  _command.z = 0;
}

void init_PWM()
{
  bool useCPPM = false;
  if(get_param_int(PARAM_RC_TYPE) == 1)
  {
    useCPPM = true;
  }
  int16_t motor_refresh_rate = get_param_int(PARAM_MOTOR_PWM_SEND_RATE);
  int16_t idle_pwm = get_param_int(PARAM_MOTOR_IDLE_PWM);
  pwmInit(useCPPM, false, false, motor_refresh_rate, idle_pwm);
}


void write_motor(uint8_t index, int32_t value)
{
  value += 1000;
  if (_armed_state == ARMED)
  {
    if (value > 2000)
    {
      value = 2000;
    }
    else if (value < get_param_int(PARAM_MOTOR_IDLE_PWM) && get_param_int(PARAM_SPIN_MOTORS_WHEN_ARMED))
    {
      value = get_param_int(PARAM_MOTOR_IDLE_PWM);
    }
    else if (value < 1000)
    {
      value = 1000;
    }
  }
  else
  {
    value = 1000;
  }
  _outputs[index] = value;
  pwmWriteMotor(index, _outputs[index]);
}


void write_servo(uint8_t index, int32_t value)
{
  if (value > 500)
  {
    value = 500;
  }
  else if (value < -500)
  {
    value = -500;
  }
  _outputs[index] = value+1500;
  pwmWriteMotor(index, _outputs[index]);
}


void mix_output()
{
  int32_t max_output = 0;
  for (int8_t i=0; i<8; i++)
  {
    if (mixer_to_use.output_type[i] != NONE)
    {
      // Matrix multiply (in so many words) -- done in integer, hence the /1000 at the end
      prescaled_outputs[i] = (int32_t)((_command.F*mixer_to_use.F[i] + _command.x*mixer_to_use.x[i] +
                                        _command.y*mixer_to_use.y[i] + _command.z*mixer_to_use.z[i])*1000.0f);
      if (prescaled_outputs[i] > 1000 && prescaled_outputs[i] > max_output)
      {
        max_output = prescaled_outputs[i];
      }
      // negative motor outputs are set to zero when writing to the motor,
      // but they have to be allowed here because the same logic is used for
      // servo commands, which may be negative
    }
  }

  // saturate outputs to maintain controllability even during aggressive maneuvers
  if (max_output > 1000)
  {
    int32_t scale_factor = 1000*1000/max_output;
    for (int8_t i=0; i<8; i++)
    {
      if (mixer_to_use.output_type[i] == M)
      {
        prescaled_outputs[i] = (prescaled_outputs[i])*scale_factor/1000; // divide by scale factor
      }
    }
  }
}
