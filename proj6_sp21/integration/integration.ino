/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_4
#define LEFT_ENCODER                P6_4
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_RIGHT                 2
#define DRIVE_CLOSE                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.3053;
float theta_right = 0.3656;
float beta_left = -15.93;
float beta_right = -7.278;
float v_star = 57.2;

// PWM inputs to jolt the car straight
int left_jolt = 145;
int right_jolt = 165;

// Control gains
float k_left = 0.5;
float k_right = 0.5;

// Sampling periods
int T_c = 100;
int T_d = 500;
int F_c = 10;
int F_d = 2;
int m = 5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (((v_star) + beta_left)/theta_left) - ((k_left * delta)/theta_left);
}

float driveStraight_right(float delta) {
  return (((v_star) + beta_right)/theta_right) + ((k_right * delta)/theta_right);
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {5000, 5000, 5000, 2500}; // GO, TURN LEFT, TAKE A RIGHT, SLOW DOWN THERE BOY

float delta_reference(int n) {
  // YOUR CODE HERE
  if (drive_mode == DRIVE_RIGHT) {
    return (v_star * n * CAR_WIDTH) / TURN_RADIUS;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return -(v_star * n * CAR_WIDTH) / TURN_RADIUS;
  }
  else { // DRIVE_STRAIGHT
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int n) {
  // YOUR CODE HERE
  return (v_star * n * CAR_WIDTH) / STRAIGHT_RADIUS; // Replace this line
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P7_0

#define SIZE                        3200
#define SIZE_AFTER_FILTER           200
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  80
#define PRELENGTH                     5
#define THRESHOLD                     0.5

#define EUCLIDEAN_THRESHOLD         0.042
#define LOUDNESS_THRESHOLD          700

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {-0.01030302664092075, -0.00786003014601988, -0.0017040875230787678, -0.017869314934708724, -0.06134863994142305, -0.1539295864412114, -0.2689253168391352, -0.2264795316676792, -0.2731654833779553, -0.26044964774932206, -0.21928533441134163, -0.2435039629271885, -0.22442673420275966, -0.18685702304059895, -0.20926483500622742, -0.16872999333783692, -0.15213066744565673, -0.17900189958595128, -0.13392863186965126, -0.12509327196897813, -0.1017032725350408, -0.07466276444463302, -0.08147882199390677, -0.04975287985322955, -0.03984783939323008, -0.05461079817627827, -0.041648340275375006, -0.03626096566452624, -0.052571751918039866, -0.032746013884715354, -0.027233921302081025, -0.016068351700458297, -0.0032432977438650724, 0.009293280983040048, 0.03721934587904259, 0.04842941492979382, 0.06081631690731262, 0.07398748267783621, 0.07663650633112488, 0.08689996625858232, 0.07973244827470782, 0.08070465284315465, 0.08570745620478208, 0.07608076499285751, 0.08291790436842872, 0.07979890771373176, 0.08137239458044894, 0.08490353806227727, 0.07556319849632019, 0.07505176553441233, 0.06743518251244052, 0.06450016889915905, 0.07217940079711273, 0.08917894869258924, 0.07613975097596136, 0.09529551925835086, 0.07655583966144647, 0.08301905533258694, 0.08828187471466843, 0.07799783121792445, 0.08471768635752575, 0.07763894582916496, 0.07361464112207466, 0.06858779124970422, 0.07577310621660963, 0.07533276566058349, 0.06284856778478831, 0.07575144754803159, 0.0673239969401673, 0.07072590050211137, 0.07584059813657963, 0.06917011738075214, 0.08967376866062879, 0.09927049943890505, 0.11202741803234323, 0.11611341044363652, 0.11797504760623477, 0.1161114815417691, 0.10299234631930113, 0.11889758407201843};
float pca_vec2[SNIPPET_SIZE] = {-0.009042313988974315, -0.005425299006727224, 0.00995538022174347, 0.02434731350463415, 0.019806993704132775, -0.17975459512295247, -0.080213472927569, -0.08993594684254412, -0.05900605111801925, -0.09964308025919219, -0.14295001740280328, -0.09141410883292032, -0.10748262645497744, -0.07495173645561228, -0.047585356397613666, -0.03197893421624567, -0.016756849223326882, 0.00398698277390121, 0.036757298687286534, 0.10895530462893835, 0.14594019822948043, 0.17544595187560366, 0.1963810988289685, 0.17800112333614374, 0.18765464379281738, 0.17986983570947188, 0.16050027268778128, 0.1488987138873474, 0.1414706380566386, 0.118939515133809, 0.11831580463184721, 0.12888829167149113, 0.11113875624872732, 0.1068526201593169, 0.11054929137480812, 0.11293355858888988, 0.10788230935713318, 0.11330013044506297, 0.10817636082906369, 0.11166281720517457, 0.09306435954421341, 0.08820910185187354, 0.0874329997433242, 0.07190034310736246, 0.06308322391959523, 0.05607229822252845, 0.05196749126346518, 0.03446056169393042, -0.01173070507859834, -0.025496599875311048, -0.04693292651031032, -0.059600249115862904, -0.09392997940526458, -0.14500736743901343, -0.13386232102172294, -0.18358599055082905, -0.18170897387903626, -0.18524261752005577, -0.2190152552531477, -0.19183484149607624, -0.18193789765474133, -0.22819910524856704, -0.16378911075236385, -0.136357912029728, -0.16810045598309556, -0.1226292227577748, -0.09089192901776584, -0.07710995499589753, -0.024364934452596028, -0.026770170111830235, 0.00892519616733974, 0.016076296927088417, 0.020554443283305843, 0.031818415272755446, 0.02933925613527135, 0.03381937543594585, 0.03720043342460497, 0.016998397227231125, 0.0233639413015326, 0.0033415683074852626};
float projected_mean_vec[2] = {-0.05148992891034338, -0.014364305020967305};
float centroid1[2] = {0.05040243852605836, 0.01603272570617186};
float centroid2[2] = {-0.08081548806221897, 0.017170700354212077};
float centroid3[2] = {-0.00542987386314571, -0.05434836510142579};
float centroid4[2] = {0.0358429233993063, 0.021144939041041838};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' on the principal components
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += pca_vec1[i] * result[i];
          proj2 += pca_vec2[i] * result[i];
      }

      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];


      // Classification
      // Use the function l2_norm defined above
      // ith centroid: centroids[i]
      // YOUR CODE HERE
      float best_dist = 999999;
      int best_index = -1;
      for (int i = 0; i < 4; ++i) {
        float dist_i = l2_norm(proj1, proj2, centroids[i]);
        if (dist_i < best_dist) {
          best_dist = dist_i;
          best_index = i;
        }
      }


      // Check against EUCLIDEAN_THRESHOLD and execute identified command
      // YOUR CODE HERE
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        if (best_index == 0) {
          Serial.println("Classified as GO");
        } else if (best_index == 1) {
          Serial.println("Classified as TURN LEFT");
        } else if (best_index == 2) {
          Serial.println("Classified as TAKE A RIGHT");
        } else if (best_index == 3) {
          Serial.println("Classified as SLOW DOWN THERE BOY");
        }
        drive_mode = best_index; // from 0-3, inclusive
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
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

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TB0CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TB0CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TB0CCTL0 = CCIE; // enable interrupts for Timer B
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
