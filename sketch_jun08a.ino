#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>

PS2X ps2;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/* PS2 controller pins */
#define PS2_DAT       12
#define PS2_CMD       13
#define PS2_ATT       15
#define PS2_CLK       14

/* PWM channels for DC motors */
#define PWM_DC0A      8 // DC motor 0
#define PWM_DC0B      9
#define PWM_DC1A      10 // DC motor 1
#define PWM_DC1B      11
#define PWM_DC2A      12 // DC motor 2
#define PWM_DC2B      13
#define PWM_DC3A      14 // DC motor 3
#define PWM_DC3B      15

#define MOT_DC_LEFT   0
#define MOT_DC_RIGHT  3
#define MOT_DC_SHOOT  1
#define MOT_DC_INTAKE 2

#define SPD_SHOOT     4095
#define SPD_INTAKE    4095
#define SPD_DRIVE     3072
#define SPD_DEAD      80

#define S360_PW_MIN    400 
#define S360_PW_MID    1400 

/* control DC motor */
void dc_control(uint8_t motor, int16_t speed, bool brake = false) {
  if(speed > -SPD_DEAD && speed < SPD_DEAD) speed = 0;
  uint16_t speed_a = (brake) ? 4095 : ((speed > 0) ? speed : 0);
  uint16_t speed_b = (brake) ? 40
  +95 : ((speed < 0) ? (-speed) : 0);
  switch(motor) {
    case 0:
      pwm.setPWM(PWM_DC0A, 0, speed_a);
      pwm.setPWM(PWM_DC0B, 0, speed_b);
      break;
    case 1:
      pwm.setPWM(PWM_DC1A, 0, speed_a);
      pwm.setPWM(PWM_DC1B, 0, speed_b);
      break;
    case 2:
      pwm.setPWM(PWM_DC2A, 0, speed_a);
      pwm.setPWM(PWM_DC2B, 0, speed_b);
      break;
    case 3:
      pwm.setPWM(PWM_DC3A, 0, speed_a);
      pwm.setPWM(PWM_DC3B, 0, speed_b);
      break;
  }
}

void servo180_control(uint16_t ch, float angle) {
  uint16_t us = (uint16_t) (1000 * (1.0 + angle / 180.0));
  pwm.writeMicroseconds(ch,us);
}

void servo360_control(uint16_t ch, float speed) {
  pwm.writeMicroseconds(ch, S360_PW_MID + speed * (S360_PW_MID - S360_PW_MIN));
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // must be 115200 for ESP32 for debugging output

  Serial.print(F("Initializing PWM controller..."));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);
  Serial.println(F("done."));
  

  Serial.print(F("Initializing PS2 controller..."));
  while(ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT) != 0) {
    delay(500);
    Serial.println("PS2 Error");
  }
  Serial.println(F("done."));
}
int drive_speed = SPD_DRIVE;

bool speed_toggled = false;

void loop() {
  // put your main code here, to run repeatedly:
  ps2.read_gamepad(); // update from controller

   int16_t speed_l = map(ps2.Analog(PSS_LY), 0, 255, -SPD_DRIVE, SPD_DRIVE); dc_control(MOT_DC_LEFT, speed_l); Serial.print(speed_l, DEC); Serial.print(' ');
  int16_t speed_r = map(ps2.Analog(PSS_RY), 0, 255, SPD_DRIVE, -SPD_DRIVE); dc_control(MOT_DC_RIGHT, speed_r); Serial.println(speed_r, DEC);
   if (ps2.Button(PSB_RED)){
      dc_control(MOT_DC_SHOOT, SPD_SHOOT);
   } else dc_control(MOT_DC_SHOOT, 0);
   if (ps2.Button(PSB_BLUE)){
      servo360_control(6,100);
      servo360_control(7,100);
   }  else {
      servo360_control(6,0);
      servo360_control(7,0);
   }
}