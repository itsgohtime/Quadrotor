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
#include "vive.h"

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
#define hb_timeout 0.5

#define PWM_MAX 1900
#define PWM_MIN 1000
#define frequency 25000000.0
#define LED0 0x6
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4
#define NEUTRAL_PWM 1500

#define Pitch_P 4.0// 8
#define Pitch_I 0.03 // 0.03
#define Pitch_D 1.0 // 1.5

#define Roll_P 9.0 // 7
#define Roll_I 0.03 // 0.03
#define Roll_D 1.0  //1.4

#define Yaw_P 1.5
#define V_Yaw_P 200

#define V_Pitch_P 0.03
#define V_Pitch_D 0.02 // 0.2
#define V_Roll_P 0.03
#define V_Roll_D 0.02 // 0.2

#define MAX_THRUST 150

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
void calibrate_vive();
void read_imu();
void update_filter(float A);
void safety_check();
void setup_keyboard();
void trap(int signal);
void pid_update();
void motors_off();
void init_pwm();
void init_motor(uint8_t channel);
void set_PWM(uint8_t channel, float time_on_us);
void keyboard_feedback();

// global variables

// imu variables
int imu;
float imu_data[6]; // gyro xyz, accel xyz

// calibration variables
float x_gyro_calibration = 0;
float y_gyro_calibration = 0;
float z_gyro_calibration = 0;
float roll_calibration = 0;
float pitch_calibration = 0;
float accel_z_calibration = 0;

float vive_x_calib;
float vive_y_calib;

// time variables
long time_curr;
long time_prev;
struct timespec te;

// cf variables
float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;
float roll_angle_t1 = 0;
float pitch_angle_t1 = 0;
float gyro_roll = 0;
float gyro_pitch = 0;
float roll_accel = 0;
float pitch_accel = 0;

// keyboard variables
int last_heartbeat = 0;
long last_time = 0;
int joy_version = -1;

// vive variables
Position local_p;
Position prev_p;
Position estimated_p;
int vive_version = -1;
long vive_lt = 0;

// motor variables
int pwm;
int THRUST = 1250;

// controller variables
float desired_pitch = 0;
float desired_roll = 0;
float desired_pitch_js = 0;
float desired_roll_js = 0;
float desired_yaw = 0;
float pitch_error_sum;
float roll_error_sum;
float vive_d_y = 0.0;
float vive_d_x = 0.0;

// state variable
char state = 'p';

struct Data
{
    int key0;
    int key1;
    int key2;
    int key3;
    int pitch;
    int roll;
    int yaw;
    int thrust;
    int sequence_num;
};

Data *shared_memory;
int run_program = 1;

FILE *fptr = fopen("week9_data/tuning.txt", "w");

int main(int argc, char *argv[])
{
    init_shared_memory();
    setup_imu();
    calibrate_imu();
    calibrate_vive();
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

        local_p = *position;
        keyboard_feedback();
        safety_check();

        Data joy_data = *shared_memory;
        if (state == 'u')
        {
            read_imu();
            // printf("Gyro (xyz): %10.5f %10.5f %10.5f RP: %10.5f %10.5f \n", imu_data[0], imu_data[1], imu_data[2], roll_angle, pitch_angle);
            update_filter(0.005);
            pid_update();
        }
        else if (state == 'p')
        {
            motors_off();
        }
        else if (state == 'c')
        {
            printf("Calibrating\n");
            motors_off();
            calibrate_imu();
            calibrate_vive();
        }
    }
    fclose(fptr);
    motors_off();
}

void init_pwm()
{

    pwm = wiringPiI2CSetup(0x40);
    if (pwm == -1)
    {
        printf("-----cant connect to I2C device %d --------\n", pwm);
    }
    else
    {

        float freq = 400.0 * .95;
        float prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        uint8_t prescale = floor(prescaleval + 0.5);
        int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
        int sleep = settings | 0x10;
        int wake = settings & 0xef;
        int restart = wake | 0x80;
        wiringPiI2CWriteReg8(pwm, 0x00, sleep);
        wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
        wiringPiI2CWriteReg8(pwm, 0x00, wake);
        delay(10);
        wiringPiI2CWriteReg8(pwm, 0x00, restart | 0x20);
    }
}

void init_motor(uint8_t channel)
{
    int on_value = 0;

    int time_on_us = 900;
    uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);

    time_on_us = 1200;
    off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);

    time_on_us = 1000;
    off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);
}

void set_PWM(uint8_t channel, float time_on_us)
{
    if (run_program == 1)
    {
        if (time_on_us > PWM_MAX)
        {
            time_on_us = PWM_MAX;
        }
        else if (time_on_us < 1000)
        {
            time_on_us = 1000;
        }
        uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));
        wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value);
    }
    else
    {
        time_on_us = 1000;
        uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));
        wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value);
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
    shared_memory = (Data *)shmat(segment_id, 0, 0);
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
    fclose(fptr);
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

    // printf("calibration complete, %f %f %f %f %f %f\n\r", x_gyro_calibration, y_gyro_calibration, z_gyro_calibration, roll_calibration, pitch_calibration, accel_z_calibration);
}

void calibrate_vive()
{
    local_p = *position;
    vive_x_calib = local_p.x;
    vive_y_calib = local_p.y;
    printf("Vive calibration is {x: %0.2f, y: %0.2f}\n", vive_x_calib - vive_x_calib, vive_y_calib - vive_y_calib);
    prev_p.x = 0.0;
    prev_p.y = 0.0;
    estimated_p.x = 0.0;
    estimated_p.y = 0.0;
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
    -vive_y_calib;

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

void vive_control()
{
    estimated_p.x = estimated_p.x * 0.6 + (local_p.x - vive_x_calib) * 0.4;
    estimated_p.y = estimated_p.y * 0.6 + (local_p.y - vive_y_calib) * 0.4;
    // fprintf(fptr, "VIVEY %0.2f VIVEX %0.2f ESTIMATEY %0.2f ESTIMATEX %0.2f\n", local_p.y - vive_y_calib, local_p.x - vive_x_calib, estimated_p.y, estimated_p.x);
    
    timespec_get(&te, TIME_UTC);
    time_curr = te.tv_nsec;
    float time_diff = time_curr - vive_lt;
    if (time_diff <= 0)
    {
        time_diff += 1000000000;
    }
    time_diff = time_diff / 1000;

    vive_d_y = (estimated_p.y - prev_p.y) / time_diff;
    vive_d_x = (estimated_p.x - prev_p.x) / time_diff;
    vive_version = local_p.version;

    float desired_vive_pitch = V_Pitch_P * estimated_p.y - V_Pitch_D * vive_d_y;
    float desired_vive_roll = V_Roll_P * estimated_p.x - V_Roll_D * vive_d_x;

    // Automation
    desired_pitch = 0.5 * desired_pitch_js + 0.5 * desired_vive_pitch;
    desired_roll = 0.5 * desired_roll_js + 0.5 * desired_vive_roll;

    // printf("Vive dy %0.2f, Vive dx %0.2f\n", vive_d_y, vive_d_x);
    // printf("Error {%0.2f, %0.2f}, Desired_Pitch %0.2f, Desired Roll %0.2f\n\n", estimated_p.y, estimated_p.x, desired_pitch, desired_roll);

    // printf("Joystick pitch and roll {%0.2f, %0.2f}", desired_pitch_js, desired_roll_js);

    prev_p = estimated_p;
}

void pid_update()
{

    // printf("Actual pitch %0.2f Actual roll %0.2f\n", pitch_angle, roll_angle);
    // vive_control();

    fprintf(fptr, "Desired_Pitch %0.2f Actual_Pitch %0.2f EstimatedPY %0.2f VivprintfeDY %0.2f Desired_Roll %0.2f Actual_Roll %0.2f EstimatedPX %0.2f ViveDX %0.2f\n",
        desired_pitch, pitch_angle, estimated_p.y, vive_d_y, desired_roll, roll_angle, estimated_p.x, vive_d_x);
    // Pitch
    float pitch_error = pitch_angle - desired_pitch;
    pitch_error_sum += pitch_error * Pitch_I;
    if (pitch_error_sum > 200.0)
    {
        pitch_error_sum = 200.0;
    }
    else if (pitch_error_sum < -200.0)
    {
        pitch_error_sum = -200.0;
    }

    // Roll
    float roll_error = roll_angle - desired_roll;
    roll_error_sum += roll_error * Roll_I;
    if (roll_error_sum > 200.0)
    {
        roll_error_sum = 200.0;
    }
    else if (roll_error_sum < -200.0)
    {
        roll_error_sum = -200.0;
    }

    // Yaw
    float desired_yaw_rate = V_Yaw_P * local_p.yaw;
    float yaw_error = imu_data[2] - desired_yaw_rate;

    set_PWM(0,
            fmax(fminf(THRUST + pitch_error * Pitch_P + imu_data[0] * Pitch_D + pitch_error_sum + roll_error * Roll_P + imu_data[1] * Roll_D + roll_error_sum  + yaw_error * Yaw_P,
                       PWM_MAX),
                 PWM_MIN));

    set_PWM(2,
            fmax(fminf(THRUST + pitch_error * Pitch_P + imu_data[0] * Pitch_D + pitch_error_sum - roll_error * Roll_P - imu_data[1] * Roll_D - roll_error_sum - yaw_error * Yaw_P,
                       PWM_MAX),
                 PWM_MIN));

    set_PWM(1,
            fmax(fminf(THRUST - pitch_error * Pitch_P - imu_data[0] * Pitch_D - pitch_error_sum + roll_error * Roll_P + imu_data[1] * Roll_D + roll_error_sum - yaw_error * Yaw_P,
                       PWM_MAX),
                 PWM_MIN));

    set_PWM(3, fmax(fminf(THRUST - pitch_error * Pitch_P - imu_data[0] * Pitch_D - pitch_error_sum - roll_error * Roll_P - imu_data[1] * Roll_D - roll_error_sum + yaw_error * Yaw_P,
                          PWM_MAX),
                    PWM_MIN));  
}

void safety_check()
{
    if (abs(imu_data[0]) > max_gyro_rate)
    {
        run_program = 0;
        printf("X gyro rate is greater than 300\n");
    }

    if (abs(imu_data[1]) > max_gyro_rate)
    {
        run_program = 0;
        printf("Y gyro rate is greater than 300\n");
    }

    if (abs(imu_data[2]) > max_gyro_rate)
    {
        run_program = 0;
        printf("Z gyro rate is greater than 300\n");
    }

    if (roll_angle > max_roll_angle)
    {
        run_program = 0;
        printf("Roll angle is greater than max roll angle.\n");
    }
    else if (roll_angle < -max_roll_angle)
    {
        run_program = 0;
        printf("Roll angle is less than min roll angle.\n");
    }

    if (pitch_angle > max_pitch_angle)
    {
        run_program = 0;
        printf("Pitch angle is greater than max pitch angle.\n");
    }
    else if (pitch_angle < -max_pitch_angle)
    {
        run_program = 0;
        printf("Pitch angle is less than min pitch angle.\n");
    }

    Data joy_data = *shared_memory;

    if (joy_data.sequence_num != joy_version)
    {
        timespec_get(&te, TIME_UTC);
        time_curr = te.tv_nsec;
        last_time = time_curr;
        joy_version = joy_data.sequence_num;
    }
    else if (joy_data.sequence_num == joy_version)
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
            printf("Joystick Timeout.");
        }
    }

    if (joy_data.key1 == 1)
    {
        run_program = 0;
        printf("Kill All!\n");
    }

    // printf("Vive state  X: %0.2f, Y: %0.2f\n", local_p.x - vive_x_calib, local_p.y - vive_y_calib);
    // if (abs(local_p.x - vive_x_calib) > 1000)
    // {
    //     run_program = 0;
    //     printf("Vive x position outside allowable range\n");
    // }

    // if (abs(local_p.y - vive_y_calib) > 1000)
    // {
    //     run_program = 0;
    //     printf("Vive y position outside allowable range\n");
    // }

    if (local_p.version != vive_version)
    {
        timespec_get(&te, TIME_UTC);
        time_curr = te.tv_nsec;
        vive_lt = time_curr;
        vive_control();
    }
    else if (local_p.version == vive_version)
    {
        timespec_get(&te, TIME_UTC);
        time_curr = te.tv_nsec;
        float time_diff = time_curr - vive_lt;
        if (time_diff <= 0)
        {
            time_diff += 1000000000;
        }
        time_diff = time_diff / 1000000000;
        if (time_diff > hb_timeout)
        {
            run_program = 0;
            printf("Vive Timeout.\n");
        }
    }
}

void keyboard_feedback()
{
    Data joy_data = *shared_memory;

    // printf("Key Pressed is %d\n", keyboard.key_press);

    if (joy_data.key0 == 1)
    {
        state = 'u';
    }
    else if (joy_data.key2 == 1)
    {
        state = 'p';
    }
    else if (joy_data.key3 == 1)
    {
        state = 'c';
    }

    THRUST = (joy_data.thrust - 128) * -MAX_THRUST / 128 + NEUTRAL_PWM;
    // printf("Thrust is %d\n", THRUST);
    desired_pitch_js = (joy_data.pitch - 127.0) * -5.0 / 128.0;
    desired_roll_js = (joy_data.roll - 128.0) * -5.0 / 128.0;
    desired_yaw = (joy_data.yaw - 127.0) * 90.0 / 128.0;
    // yaw angle
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
        // c = wiringPiI2CReadReg8(imu, GYRO_CONFIG);
        // printf("gyro config %d\n", c);

        c = wiringPiI2CReadReg8(imu, CONFIG);
        // wiringPiI2CWriteReg8(imu, CONFIG, 0x06); // 5hz
        // wiringPiI2CWriteReg8(imu, CONFIG, 0x04); // 20hz
        wiringPiI2CWriteReg8(imu, CONFIG, 0x03); // 41hz
        // wiringPiI2CWriteReg8(imu, CONFIG, 0x02); // 92hz
        // wiringPiI2CWriteReg8(imu, CONFIG, 0x01); // 184hz
        // wiringPiI2CWriteReg8(imu, CONFIG, 0x00); // 250hz

        c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG, c | Ascale << 3);
        // c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c & ~0x0F); //
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, c | 0x00);
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x06); // 5hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x05); // 10hz
        wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x04); // 20hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x03); // 41hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x02); // 92hz
        // wiringPiI2CWriteReg8(imu, ACCEL_CONFIG2, 0x00); // 460hz
        c = wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
        printf("C is %d\n", c);
    }
    return 0;
}

void motors_off()
{
    for (int i = 0; i < 4; i++)
    {
        set_PWM(i, 1000);
    }
}