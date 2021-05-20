/*
 * open_loop.ino
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 */

#define LEFT_MOTOR                  P1_5
#define RIGHT_MOTOR                 P1_4

#define RUN_TIME                    (15*1000)

unsigned long end_time = 0;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*---------------------------*/

float theta_left = 0.3052;
float theta_right = 0.3656;
float beta_left = -15.93;
float beta_right = -7.278;
float v_star = 57.2;

// PWM inputs to jolt the car straight
int left_jolt = 145;
int right_jolt = 165;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float driveStraight_left(float v_star) {
  return (v_star + beta_left) / theta_left;
}

float driveStraight_right(float v_star) {
  return (v_star + beta_right) / theta_right;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


void setup(void) {
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  write_pwm(0, 0); // Turn off motors
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker(); // Blink lights to indicate car is running
  write_pwm(left_jolt, right_jolt); // Jolt motors for 200ms
  delay(200);

  /*---------------------------*/
  /*      CODE BLOCK CON0      */
  /*---------------------------*/

  // Attempt to drive straight using open loop control
  // Compute the PWM input required for each wheel based on v_star
  int left_cur_pwm = 135;
  int right_cur_pwm = 137;
  write_pwm(left_cur_pwm, right_cur_pwm);

  /*---------------------------*/
  /*---------------------------*/
  /*---------------------------*/

  end_time = millis() + RUN_TIME;
}

void loop(void) {
  if (end_time <= millis()) {
    write_pwm(0, 0);
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}
