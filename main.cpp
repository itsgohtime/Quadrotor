#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>

// gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define USER_CTRL 0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1 0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2 0x6C

#define max_gyro_rate 300
#define max_roll_angle 45
#define max_pitch_angle 45
#define hb_timeout 0.25


#define PWM_MAX 1500
#define PWM_MIN 1000
#define frequency 25000000.0
#define LED0 0x6			
#define LED0_ON_L 0x6		
#define LED0_ON_H 0x7		
#define LED0_OFF_L 0x8	
#define LED0_OFF_H 0x9		
#define LED_MULTIPLYER 4
#define NEUTRAL_PWM 1250	

#define Pitch_P 0.0 //8
#define Pitch_D 0.0 //1.5
#define Pitch_I 0.0 //0.03
#define Roll_P 7 //10
#define Roll_D 1.5 //1
#define Roll_I 0.03

int kb_version = 0;
int THRUST = 1250;
float desired_pitch  = 0;
float desired_roll = 0;

enum Ascale
{
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale
{
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter(float A);
void safety_check();
void setup_keyboard();
void trap(int signal);
void pid_update();
void motors_off();
void init_pwm();
void init_motor(uint8_t channel);
void set_PWM( uint8_t channel, float time_on_us);
void keyboard_feedback();

// global variables
int imu;
float x_gyro_calibration = 0;
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0;
float pitch_calibration = 0;
float accel_z_calibration = 0;
float imu_data[6]; // gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;
float roll_angle_t1 = 0;
float pitch_angle_t1 = 0;
float gyro_roll = 0;
float gyro_pitch = 0;
float roll_accel = 0;
float pitch_accel = 0;

int last_heartbeat = 0;
long last_time = 0;

int pwm;
float pitch_error_sum;
float roll_error_sum;

char state = 'p';

// FILE *fptr = fopen("sample.txt", "w"); 

struct Keyboard
{
    char key_press;
    int heartbeat;
    int version;
};
Keyboard *shared_memory;
int run_program = 1;

int main(int argc, char *argv[])
{
    setup_imu();
    calibrate_imu();
    setup_keyboard();
    signal(SIGINT, &trap);

    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);
    while (run_program == 1)
    {
        keyboard_feedback();
        safety_check();

        Keyboard keyboard = *shared_memory;
        if (state == 'u') {
            read_imu();
            // printf("Gyro (xyz): %10.5f %10.5f %10.5f RP: %10.5f %10.5f \n", imu_data[0], imu_data[1], imu_data[2], roll_angle, pitch_angle);
            update_filter(0.2);
            pid_update();
        } else if (state == 'p') {
            motors_off();
        } else if (state == 'c' && kb_version != keyboard.version) {
            motors_off();
            calibrate_imu();
        }
    }
    // fclose(fptr);
    motors_off();
}

void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
  
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}

void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  {  
    time_on_us=1000;   
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}

void setup_keyboard()
{
    int segment_id;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey = 33222;
    /* Allocate a shared memory segment. */
    segment_id = shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
    /* Attach the shared memory segment. */
    shared_memory = (Keyboard *)shmat(segment_id, 0, 0);
    printf("shared memory attached at address %p\n", shared_memory);
    /* Determine the segment's size. */
    shmctl(segment_id, IPC_STAT, &shmbuffer);
    segment_size = shmbuffer.shm_segsz;
    printf("segment size: %d\n", segment_size);
    /* Write a string to the shared memory segment. */
    // sprintf (shared_memory, "test!!!!.");
}

void trap(int signal)
{
    printf("ending program\n\r");
    run_program = 0;
    motors_off();
    // fclose(fptr);

}

void calibrate_imu()
{
    // Sum each calibration variable over 1000 iterations
    float x_gyro_sum, y_gyro_sum, z_gyro_sum, roll_sum, pitch_sum, accel_z = 0;

    int c_iters = 1000;
    for (int i = 0; i < c_iters; i++)
    {
        read_imu();
        x_gyro_sum += imu_data[0];
        y_gyro_sum += imu_data[1];
        z_gyro_sum += imu_data[2];
        roll_sum += roll_angle;
        pitch_sum += pitch_angle;
    }

    // Updates calibration values with avg of each variable
    x_gyro_calibration = x_gyro_sum / c_iters;
    y_gyro_calibration = y_gyro_sum / c_iters;
    z_gyro_calibration = z_gyro_sum / c_iters;
    roll_calibration = roll_sum / c_iters;
    pitch_calibration = pitch_sum / c_iters;
    accel_z_calibration = 0;

    printf("calibration complete, %f %f %f %f %f %f\n\r", x_gyro_calibration, y_gyro_calibration, z_gyro_calibration, roll_calibration, pitch_calibration, accel_z_calibration);
}

void read_imu()
{
    // X acceleration
    int address = 59;
    float ax = 0;
    float az = 0;
    float ay = 0;
    int vh, vl;
    float max_accel = 2.0;
    float max_gyro = 500.0;
    // read in data
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    // convert 2 complement
    int vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[3] = vw * max_accel / 32768.0;

    // Y acceleration
    address = 61;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[4] = vw * max_accel / 32768.0;

    // Z acceleration
    address = 63;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[5] = (vw * max_accel / 32768.0) - accel_z_calibration;
    // printf("accel z: %f", imu_data[5]); // print statement to check z acceleration

    // X gyro
    address = 67;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[0] = -vw * max_gyro / 32768.0 - x_gyro_calibration;

    // Y gyro
    address = 69;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[1] = vw * max_gyro / 32768.0 - y_gyro_calibration;

    // Z gyro
    address = 71;
    vh = wiringPiI2CReadReg8(imu, address);
    vl = wiringPiI2CReadReg8(imu, address + 1);
    vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
    if (vw > 0x8000)
    {
        vw = vw ^ 0xffff;
        vw = -vw - 1;
    }
    imu_data[2] = vw * max_gyro / 32768.0 - z_gyro_calibration;
    ;

    // Updates roll and pitch angle
    roll_angle = atan2(imu_data[3], -imu_data[5]);
    roll_angle = (roll_angle * 180.0 / 3.14159) - roll_calibration;
    pitch_angle = atan2(imu_data[4], -imu_data[5]);
    pitch_angle = (pitch_angle * 180.0 / 3.14159) - pitch_calibration;
}

void update_filter(float A)
{
    // get current time in nanoseconds
    timespec_get(&te, TIME_UTC);
    time_curr = te.tv_nsec;
    // compute time since last execution
    float imu_diff = time_curr - time_prev;

    // check for rollover
    if (imu_diff <= 0)
    {
        imu_diff += 1000000000;
    }
    // convert to seconds
    imu_diff = imu_diff / 1000000000;
    time_prev = time_curr;

    // comp. filter for roll, pitch here:
    float roll_gyro = (imu_data[1] * imu_diff) + roll_angle_t1;
    gyro_roll += (imu_data[1] * imu_diff);
    roll_accel = roll_angle;
    roll_angle = roll_accel * A + (1 - A) * (roll_gyro);
    roll_angle_t1 = roll_angle;

    // printf("ROLL: Acceleration Angle: %f, Gyro Output: %f, Filtered Roll: %f\n", roll_accel, gyro_roll, roll_angle);

    float pitch_gyro = (imu_data[0] * imu_diff) + pitch_angle_t1;
    gyro_pitch += (imu_data[0] * imu_diff);
    pitch_accel = pitch_angle;
    pitch_angle = pitch_accel * A + (1 - A) * (pitch_gyro);
    pitch_angle_t1 = pitch_angle;

    // printf("PITCH: Acceleration Angle: %f, Gyro Output: %f, Filtered Roll: %f\n\n", pitch_accel, gyro_pitch, pitch_angle);
}

void pid_update()
{

    float pitch_error = pitch_angle - desired_pitch;
    pitch_error_sum += pitch_error * Pitch_I;

    float roll_error = roll_angle - desired_roll;
    roll_error_sum += roll_error * Roll_I;
    
    // float motor1 = THRUST + pitch_error * Pitch_P + imu_data[0] * Pitch_D + error_sum;
    // float motor2 = THRUST - pitch_error * Pitch_P - imu_data[0] * Pitch_D - error_sum;

    // fprintf(fptr, "Actual %f Desired %f\n", pitch_angle, desired_pitch);
    // printf("PitchValue %f Gyro %f Motor1 %f Motor2 %f\n", pitch_angle, imu_data[0], motor1, motor2);
    // printf("Pitch_Filter %f Pitch_Accel %f Pitch_Gyro %f Motor1 %f Motor2 %f\n", pitch_angle, pitch_accel, gyro_pitch, motor1, motor2);

    set_PWM(0,
    fmax(fminf(THRUST 
        + pitch_error * Pitch_P + imu_data[0] * Pitch_D + pitch_error_sum
        + roll_error * Roll_P + imu_data[1] * Roll_D + roll_error_sum,
    PWM_MAX), PWM_MIN));

    set_PWM(2,
    fmax(fminf(THRUST
        + pitch_error * Pitch_P + imu_data[0] * Pitch_D + pitch_error_sum
        - roll_error * Roll_P - imu_data[1] * Roll_D - roll_error_sum,
    PWM_MAX), PWM_MIN));
    
    set_PWM(1, 
    fmax(fminf(THRUST 
        - pitch_error * Pitch_P - imu_data[0] * Pitch_D - pitch_error_sum
        + roll_error * Roll_P + imu_data[1] * Roll_D + roll_error_sum,
    PWM_MAX), PWM_MIN));
    
    set_PWM(3, fmax(fminf(THRUST
        - pitch_error * Pitch_P - imu_data[0] * Pitch_D - pitch_error_sum
        - roll_error * Roll_P - imu_data[1] * Roll_D - roll_error_sum,
    PWM_MAX), PWM_MIN));

    // float low_motors = min(NEUTRAL_PWM + abs(roll_angle) * Pitch_P, PWM_MAX);
    // float high_motors = max(NEUTRAL_PWM - abs(roll_angle) * Pitch_P, PWM_MIN);
    // if (roll_angle > 0) {

    // } else {

    // }
}

void safety_check()
{
    if (abs(imu_data[0]) > max_gyro_rate)
    {
        run_program = 0;
        printf("X gyro rate is greater than 300");
    }
    else if (abs(imu_data[1]) > max_gyro_rate)
    {
        run_program = 0;
        printf("Y gyro rate is greater than 300");
    }
    else if (abs(imu_data[2]) > max_gyro_rate)
    {
        run_program = 0;
        printf("Z gyro rate is greater than 300");
    }

    if (roll_angle > max_roll_angle)
    {
        run_program = 0;
        printf("Roll angle is greater than max roll angle.");
    }
    else if (roll_angle < -max_roll_angle)
    {
        run_program = 0;
        printf("Roll angle is less than min roll angle.");
    }

    if (pitch_angle > max_pitch_angle)
    {
        run_program = 0;
        printf("Pitch angle is greater than max pitch angle.");
    }
    else if (pitch_angle < -max_pitch_angle)
    {
        run_program = 0;
        printf("Pitch angle is less than min pitch angle.");
    }

    Keyboard keyboard = *shared_memory;
    if (keyboard.key_press == ' ')
    {
        run_program = 0;
        printf("Space key was pressed.");
    }

    if (keyboard.heartbeat > last_heartbeat)
    {
        timespec_get(&te, TIME_UTC);
        time_curr = te.tv_nsec;
        last_time = time_curr;
        last_heartbeat = keyboard.heartbeat;
    }
    else if (keyboard.heartbeat == last_heartbeat)
    {
        timespec_get(&te, TIME_UTC);
        time_curr = te.tv_nsec;
        float time_diff = time_curr - last_time;
        if (time_diff <= 0)
        {
            time_diff += 1000000000;
        }
        time_diff = time_diff / 1000000000;
        if (time_diff > hb_timeout)
        {
            run_program = 0;
            printf("Keyboard Timeout.");
        }
    }
}

void keyboard_feedback()
{
    Keyboard keyboard = *shared_memory;

    // printf("Key Pressed is %d\n", keyboard.key_press);
    if (kb_version != keyboard.version){
        kb_version = keyboard.version;

        if (keyboard.key_press == 'u') {
            state = 'u';
    
        } else if (keyboard.key_press == 'p') {
            state = 'p';

        } else if (keyboard.key_press == 'c') {
            state = 'c';

        } else if (keyboard.key_press == '+'){
            THRUST += 50;
            // printf("Thrust is %d\n", THRUST);

        } else if (keyboard.key_press == '-'){
            THRUST -= 50;
            // printf("Thrust is %d\n", THRUST);

        } else if (keyboard.key_press == 3){
            // up arrow
            desired_pitch += 1;
            // printf("Pitch is %f\n", desired_pitch);

        } else if (keyboard.key_press == 2){
            // down arrow
            desired_pitch -= 1;
            // printf("Pitch is %f\n", desired_pitch);
            
        } else if (keyboard.key_press == 5){
            // right arrow
            desired_roll += 1;
            printf("Roll is %f\n", desired_roll);

        } else if (keyboard.key_press == 4){
            // down arrow
            desired_roll -= 1;
            printf("Roll is %f\n", desired_roll);
            
        }
    }
}

int setup_imu()
{
    wiringPiSetup();

    // setup imu on I2C
    imu = wiringPiI2CSetup(0x68); // accel/gyro address

    if (imu == -1)
    {
        printf("-----cant connect to I2C device %d --------\n", imu);
        return -1;
    }
    else
    {

        printf("connected to i2c device %d\n", imu);
        printf("imu who am i is %d \n", wiringPiI2CReadReg8(imu, 0x75));

        uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
        uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

        // init imu
        wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x00);
        printf("                    \n\r");
        wiringPiI2CWriteReg8(imu, PWR_MGMT_1, 0x01);
        wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
        wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); // 0x04
        int c = wiringPiI2CReadReg8(imu, GYRO_CONFIG);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0xE0);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
        wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
        c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c | Ascale << 3);
        // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c & ~0x0F); //
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c | 0x00);
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x06); // 5hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x04); // 20hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x03); // 41hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x02); // 92hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x00); // 460hz
        c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
        printf("C is %d\n", c);
    }
    return 0;
}

void motors_off(){
    for (int i = 0; i < 4; i++){
        set_PWM(i, 1000);
    }
}