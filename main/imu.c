/*************************************************************************

IMU control code, for the LSM9DS1 9DoF MARG sensor (Magnetic, Angular Rate and Gravity)
3D accelerometer, 3D gyroscope, 3D magnetometer 
Controlled by I2C bus (at 400 KHz). It shows two addresses: 
the acelerometer/gyroscope and the magnetometer.

It must be placed in the robot car such that the dot on the chip is in the right bottom corner
when looking at the car from above and from the back to the front. 
The reference axis in this module are the following, looking from above the car:

                          X
                          ^
                          |
                          |
                          |
                          |
              Y<----------Z

              Z axis points above the car (right handed orientation)

The LSM9DS1 has a linear acceleration full scale of ±2g/±4g/±8g/±16g, 
a magnetic field full scale of ±4/±8/±12/±16 gauss and an angular rate full scale of ±245/±500/±2000 dps (or 40.8/83.3/333.3 RPM)

Magnetic field strength of Earth is about 0.5 gauss, 500 mGauss, 50 uTeslas or 50000 nTeslas

*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <stdatomic.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "esp32/rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"
//#include "ekf.h"
#include "oled96.h"



#define ERR(ret, format, arg...)                                       \
   {                                                                   \
         ESP_LOGE(TAG, "%s: " format, __func__ , ## arg);        \
         return ret;                                                   \
   }


#define READ_ATOMIC(var) atomic_load_explicit(&var, memory_order_acquire)
#define WRITE_ATOMIC(var,value) atomic_store_explicit(&var, value, memory_order_release)

#define I2C_BUS I2C_NUM_0   // i2c bus of IMU
#define I2C_MASTER_TIMEOUT_MS 100
#define FIFO_LINE_SIZE 12   // Size of FIFO lines (12 bytes each)

extern _Atomic bool collision;  // Car has crashed when moving forwards or backwards

// struct used to store sample values read when calibrating
typedef struct {
    int32_t *xvalues, *yvalues, *zvalues;
    int num_elems;
} SampleList_t;


static const char* TAG = __FILE__;
static uint8_t accelAddr, magAddr;  // I2C addresses

// Output variables of module
float roll, pitch, yaw, tilt;

/* 
Define ODR of accel/gyro and magnetometer. 
The accelerometer and gyroscope are both activated and use the same ODR.
Upsampling requires that ODR of accel/gyro is greater than ODR of magnetometer.
The IMU is read with the magnetometer ODR; the accelerometer and gyroscope data are stored
by the IMU in a FIFO (32 samples max), allowing a higher ODR in accel/gyro than in magnetometer.
In order for the algorithms to work, AG_ODR must be higher or equal than M_ODR. If it is much higher, the FIFO will overrun.
So eg AG_ODR_952 is only possible if ODR_M = M_ODR_80, otherwise the FIFO overruns.
In order for the upsampling of magnetometer data to work, ODR_AG must be an integer multiple of ODR_M (or very close).
*/
static const enum {AG_ODR_OFF,AG_ODR_14_9,AG_ODR_59_5,AG_ODR_119,AG_ODR_238,AG_ODR_476,AG_ODR_952} ODR_AG = AG_ODR_238;  
static const float odr_ag_modes[] = {0.0,14.9,59.5,119.0,238.0,476.0,952.0};  // Values in Hz

static const enum {M_ODR_0_625,M_ODR_1_25,M_ODR_2_5,M_ODR_5,M_ODR_10,M_ODR_20,M_ODR_40,M_ODR_80} ODR_M = M_ODR_40;   
static const float odr_m_modes[] = {0.625,1.25,2.5,5.0,10.0,20.0,40.0,80.0};  // Values in Hz

static const float delta_t = 1.0/odr_ag_modes[ODR_AG];  // Inverse of gyro/accel ODR
static int upsampling_factor;  // Ratio between both ODRs


// gRes, aRes, and mRes store the current resolution for each sensor. 
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15), as data resolution is 16 bits, signed.
static float gRes, aRes, mRes;

/* Store error offset (bias) for calibration of accel/gyro and magnetometer */
static int16_t err_AL[3];  // ex,ey,ez values (error or bias for each axis in accelerometer)
static int16_t err_GY[3];  // ex,ey,ez values (error or bias for each axis in gyroscope)
static int16_t err_MA[3];  // ex,ey,ez values (error or bias for each axis in magnetometer, hardiron effects)
static float scale_MA[3] = {1.0, 1.0, 1.0}; // ex,ey,ez values (scaling error for each axis in magnetometer, softiron effects)

static float stderror_AL[3];  // Measured standard error of mean value for bias in x, y and z of accelerometer
static float stderror_GY[3];  // Measured standard error of mean value for bias in x, y and z of gyroscope

static float deviation_AL[3];  // Measured standard deviation of x, y and z values of accelerometer
static float deviation_GY[3];  // Measured standard deviation of x, y and z values of gyroscope
static float deviation_MA[3];  // Measured standard deviation of x, y and z values of magnetometer

static const char *cal_file = "/spiffs/calibration.dat"; // File where calibration data is stored
static const char *dev_file = "/spiffs/deviation.dat";   // File where standard deviation data is stored
static const float declination = +1.962;    // Local magnetic declination as given by http://www.magnetic-declination.com/
static const float default_magnetic_field = 0.459;  // Magnitude of the local magnetic field in Gauss (does not need to be exact)

/* Madgwick filter variables */
static float q[4] = {1.0, 0.0, 0.0, 0.0};    // vector to hold quaternion

/// Quote from kriswinner regarding beta parameter:
/* 
There is a tradeoff in the beta parameter between accuracy and response speed.
In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
In any case, this is the free parameter in the Madgwick filtering and fusion scheme. 
*/

// gyroscope measurement error in rads/s (start at 40 deg/s)
#define GyroMeasError (M_PI * (40.0/180))
static const float beta = 1.73205/2 * GyroMeasError;   // compute beta, sqrt(3/4)*GyroMeasError


/* Prototypes */
extern void pito(uint32_t decimas, int modo);  // From main.c
static void IMU_read(void*);

static void calibrate_accel_gyro(void);
static void calibrate_magnetometer(void);
static esp_err_t calculate_std_dev_magnetometer(void);
static int read_magnetometer(int16_t *mx, int16_t *my, int16_t *mz);
static float ellipsoid_fit(const SampleList_t *const sample_list, float Bm);
static float quad_error_function(float Vx, float Vy, float Vz, float A, float B, float C, 
                                 float Bm, const SampleList_t *sample_list);
                                 
                
/** IMU functions **/
                                   
/*
Function to calculate heading, using magnetometer readings.
It only works if the sensor is lying flat (z-axis normal to Earth).
It is a non tilt-compensated compass.
*/
static void printHeading(float mx, float my)
{
float heading;
char str[OLED_MAX_LINE_SIZE];
  
   heading = -180/M_PI*atan2(my, mx);  // positive westwards
   //printf("Heading: %3.0f\n", heading);
   snprintf(str, sizeof(str), "Head:%-3.0f", heading);  
   oledWriteString(0, 5, str, false);  
}



/* 
This function calculates the LSM9DS1's orientation based on the
accelerometer and magnetometer data: its roll, pitch and yaw angles, in aerospace convention.
It represents a 3D tilt-compensated compass.
It also calculates the tilt: angle that the normal of the car forms with the vertical.
It produces correct results only if the car is stationary (or it does not accelerate).

Procedure according https://www.nxp.com/docs/en/application-note/AN4248.pdf, 
https://www.nxp.com/docs/en/application-note/AN4249.pdf and https://www.nxp.com/docs/en/application-note/AN3461.pdf 
Angles according extrinsic rotation sequence x-y-z (https://en.wikipedia.org/wiki/Euler_angles),
which is equivalent to the intrinsic rotation z-y'-x'' (so the angles are the same): yaw -> pitch -> roll.
See also https://en.wikipedia.org/wiki/Davenport_chained_rotations
*/
static void getOrientation(float ax, float ay, float az, float mx, float my, float mz)
{
float sinpitch, cospitch, sinroll, cosroll, rootayaz, rootaxayaz;
const float alpha = 0.05;
  
   rootayaz = sqrtf(ay*ay+az*az);
   rootaxayaz = sqrtf(ax*ax+ay*ay+az*az);
   
   /*********** Calculate roll and pitch *************/
   // Original roll equation: roll = atan2f(ay, az).
   // But this is unstable when ay and az tend to zero (pitch = 90 degrees).
   // To avoid this, add 5% of ax in denominator
   roll = atan2f(ay, az+alpha*ax);  // roll angle able to range between -180 and 180, positive clockwise
   pitch = atanf(-ax/rootayaz);     // pitch angle restricted between -90 and 90, positive downwards
   
   /*********** Calculate tilt-compensated heading (yaw angle) *************/
   // intermediate results
   sinroll = ay/rootayaz;
   cosroll = az/rootayaz;
   sinpitch = -ax/rootaxayaz;
   cospitch = rootayaz/rootaxayaz;
   
   /*********** Calculate yaw *************/
   // yaw angle able to range between -180 and 180, positive westwards
   yaw = atan2f(mz*sinroll-my*cosroll, mx*cospitch+my*sinpitch*sinroll+mz*sinpitch*cosroll);
   
   /*********** Calculate tilt angle from vertical: cos(tilt)=cos(roll)*cos(pitch) *************/ 
   tilt = acosf(cosroll*cospitch);  // tilt angle able to range between 0 and 180; tilt > 90: car is upside down
   
   /*** Translate angles in radians to degrees ***/
   tilt *= 180/M_PI; 
   yaw *= 180/M_PI; yaw -= declination; if (yaw < 0) yaw += 360;  // yaw (heading) must be positive
   pitch *= 180/M_PI; 
   roll *= 180/M_PI; 
}


/** I2C reading functions **/

// Send a single byte value to a register in the IMU
static esp_err_t i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val)
{
esp_err_t rc;
uint8_t buf[2];
    
   buf[0] = reg;
   buf[1] = val;
   rc = i2c_master_write_to_device(I2C_BUS, addr, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
   if (rc != ESP_OK) ERR(ESP_FAIL, "Error writing to IMU"); 
   return ESP_OK;
}


// Send a single byte value to a register in the IMU and read a number of bytes as response
static esp_err_t i2cWriteReadBytes(uint8_t addr, uint8_t reg, uint8_t *read_buf, size_t read_size)
{
esp_err_t rc;
          
   if (read_size == 0) return 0;
   if (read_buf == NULL) return -1;
   rc = i2c_master_write_read_device(I2C_BUS, addr, &reg, 1, read_buf, read_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
   if (rc != ESP_OK) ERR(ESP_FAIL, "Error reading from IMU"); 
   return ESP_OK;
}


// Read and discard several samples of accel/gyro data, useful when initializing
static int read_discard_fifo_samples(uint8_t samples)
{
uint8_t buf[FIFO_LINE_SIZE]; 
esp_err_t rc;
int delay;
  
   for (int i=0; i<samples; i++) {
      delay = lroundf(1000.0/odr_ag_modes[ODR_AG]);  // Time to wait for new data to arrive, in ms
      if (delay > portTICK_PERIOD_MS) vTaskDelay(pdMS_TO_TICKS(delay));
      else ets_delay_us(delay*1000); 
      
      rc = i2cWriteReadBytes(accelAddr, 0x18, buf, sizeof(buf));
      if (rc != ESP_OK) goto rw_error;   
   }
   return 0;

rw_error:
   ERR(-1, "Cannot read/write data from IMU");   
}



/*************** Calibration functions *************************/

/* Read the calibration data for accelerometer, gyroscope and magnetometer, if file exists 
   If file cannot be read, tell caller to calibrate IMU by returning -1 */
static int read_calibration_data(void)
{
FILE *fp;  
int rc;

   fp = fopen(cal_file, "r");
   if (!fp) goto nofile_error; //ERR(, "Cannot open calibration file %s: %s", cal_file, strerror(errno));
   rc = fscanf(fp, "AL: %hd, %hd, %hd\n", &err_AL[0], &err_AL[1], &err_AL[2]);
   if (rc==EOF || rc!=3) goto badfile_error;
   rc = fscanf(fp, "GY: %hd, %hd, %hd\n", &err_GY[0], &err_GY[1], &err_GY[2]);   
   if (rc==EOF || rc!=3) goto badfile_error;
   rc = fscanf(fp, "MGH: %hd, %hd, %hd\n", &err_MA[0], &err_MA[1], &err_MA[2]);
   if (rc==EOF || rc!=3) goto badfile_error;   
   rc = fscanf(fp, "MGS: %f, %f, %f\n", &scale_MA[0], &scale_MA[1], &scale_MA[2]);   
   if (rc==EOF || rc!=3) goto badfile_error;  
   fclose(fp);

   fp = fopen(dev_file, "r");
   if (!fp) goto nofile_error; //ERR(, "Cannot open deviation file %s: %s", dev_file, strerror(errno));
   
   rc = fscanf(fp, "AL: %f, %f, %f\n", &deviation_AL[0], &deviation_AL[1], &deviation_AL[2]);
   if (rc==EOF || rc!=3) goto badfile_error;
   rc = fscanf(fp, "GY: %f, %f, %f\n", &deviation_GY[0], &deviation_GY[1], &deviation_GY[2]);
   if (rc==EOF || rc!=3) goto badfile_error;  
   rc = fscanf(fp, "MG: %f, %f, %f\n", &deviation_MA[0], &deviation_MA[1], &deviation_MA[2]);
   if (rc==EOF || rc!=3) goto badfile_error;   
   fclose(fp);   
   
   return 0;

nofile_error:
   fclose(fp);
badfile_error:  
   err_AL[0] = err_AL[1] = err_AL[2] = 0;
   err_GY[0] = err_GY[1] = err_GY[2] = 0;
   err_MA[0] = err_MA[1] = err_MA[2] = 0; 
   scale_MA[0] = scale_MA[1] = scale_MA[2] = 1.0;
   deviation_AL[0] = deviation_AL[1] = deviation_AL[2] = 0.0;
   deviation_GY[0] = deviation_GY[1] = deviation_GY[2] = 0.0; 
   deviation_MA[0] = deviation_MA[1] = deviation_MA[2] = 0.0;      
   ERR(-1, "Calibration data not found. PLEASE CALIBRATE IMU!");
}
 

/* Write the calibration data for accelerometer, gyroscope and magnetometer into a file */ 
static void write_calibration_data(void)
{
FILE *fp;    

   fp = fopen(cal_file, "w");
   if (!fp) ERR(, "Cannot open calibration file %s: %s", cal_file, strerror(errno));

   fprintf(fp, "AL: %d, %d, %d\n", err_AL[0], err_AL[1], err_AL[2]);
   fprintf(fp, "GY: %d, %d, %d\n", err_GY[0], err_GY[1], err_GY[2]);   
   fprintf(fp, "MGH: %d, %d, %d\n", err_MA[0], err_MA[1], err_MA[2]);    
   fprintf(fp, "MGS: %.3f, %.3f, %.3f\n", scale_MA[0], scale_MA[1], scale_MA[2]);     
   fclose(fp);

   fp = fopen(dev_file, "w");
   if (!fp) ERR(, "Cannot open deviation file %s: %s", dev_file, strerror(errno));

   fprintf(fp, "AL: %.1f, %.1f, %.1f\n", deviation_AL[0], deviation_AL[1], deviation_AL[2]);
   fprintf(fp, "GY: %.1f, %.1f, %.1f\n", deviation_GY[0], deviation_GY[1], deviation_GY[2]);
   fprintf(fp, "MG: %.1f, %.1f, %.1f\n", deviation_MA[0], deviation_MA[1], deviation_MA[2]);  
   fclose(fp);    
}


/*
Calculate sample standard deviation of a group of N samples.
It uses the sum of the samples (s1), and the sum of their squares (s2)
*/
static inline float standard_deviation(float s1, float s2, int N)
{
   return sqrtf((N*s2-s1*s1)/(N*(N-1)));
}


/* 
 Calibrate accelerometer and gyroscope. The IMU should rest horizontal, so that
 measured accel values should be (0,0,1) and measured gyro values should be (0,0,0)
 It stores the measured bias in global variables err_AL and err_GY.
 It also calculates the standard deviation of the measured values.
 It stores the measured deviation in global variables deviation_AL and deviation_GY.
 */
static void calibrate_accel_gyro(void)
{
int32_t samples=0; 
int32_t s1_ax=0, s1_ay=0, s1_az=0;  // Sum of the accel samples
int64_t s2_ax=0, s2_ay=0, s2_az=0;  // Sum of the squares of the accel samples
int32_t s1_gx=0, s1_gy=0, s1_gz=0;  // Sum of the gyro samples
int64_t s2_gx=0, s2_gy=0, s2_gz=0;  // Sum of the squares of the gyro samples
int64_t start_tick, elapsed_useconds;
const int cal_seconds = 10; // Number of seconds to take samples
const float conf95_factor = 1.96; // 95% of the area under a normal curve lies within approximately 1.96 standard deviations of the mean
   
   ESP_LOGI(TAG, "Calibrating accelerometer and gyroscope");
   oledBigMessage(0, "HORIZ.");
   oledBigMessage(1, "WAIT...");
   vTaskDelay(pdMS_TO_TICKS(2000));   // 2 second delay, stabilize car
   
   start_tick = esp_timer_get_time();
   do {
      uint8_t byte;
      esp_err_t rc;
      int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
      int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer   
      uint8_t buf[FIFO_LINE_SIZE];
      
      // Wait for new data to arrive, acc. ODR selected (4.2 ms for 238 Hz)
      int32_t delay = lroundf(1000.0/odr_ag_modes[ODR_AG]);  // Time to wait for new data to arrive, in ms
      if (delay > portTICK_PERIOD_MS) vTaskDelay(pdMS_TO_TICKS(delay));
      else ets_delay_us(delay*1000); 
      
      rc = i2cWriteReadBytes(accelAddr, 0x27, &byte, 1);  // Read STATUS_REG register
      if (rc != ESP_OK) goto rw_error;
      if (byte&0x01) {  // New accelerometer data available
      
         // Burst read. Accelerometer and gyroscope sensors are activated at the same ODR
         // So read 12 bytes: 2x3x2
         rc = i2cWriteReadBytes(accelAddr, 0x18, buf, sizeof(buf));
         if (rc != ESP_OK) goto rw_error;   
 
         /* Read accel and gyro data. X and Y axis are exchanged, so that reference system is
            right handed, and filter algorithms work correctly */
         gy = buf[1]<<8 | buf[0]; gx = buf[3]<<8 | buf[2]; gz = buf[5]<<8 | buf[4];
         ay = buf[7]<<8 | buf[6]; ax = buf[9]<<8 | buf[8]; az = buf[11]<<8 | buf[10]; 
         //printf("%d;%d;%d\n", gx, gy, gz);
         az -= (int16_t)(1.0/aRes);  // Expected value for az is 1g, not 0g

         // Calculate the sum of the samples and of the squared samples
         // Use 64 bit arithmetic for the sum of the squares, as 32 bit might overflow
         s1_ax += ax; s2_ax += (int64_t)ax*(int64_t)ax; 
         s1_ay += ay; s2_ay += (int64_t)ay*(int64_t)ay; 
         s1_az += az; s2_az += (int64_t)az*(int64_t)az; 
         s1_gx += gx; s2_gx += (int64_t)gx*(int64_t)gx; 
         s1_gy += gy; s2_gy += (int64_t)gy*(int64_t)gy; 
         s1_gz += gz; s2_gz += (int64_t)gz*(int64_t)gz;              
         samples++;
      }
      elapsed_useconds = esp_timer_get_time() - start_tick;   
   } while (elapsed_useconds < cal_seconds*1E6);  // loop for given seconds  
   
   // Calculate the mean of accelerometer biases and store it in variable err_AL
   err_AL[0] = s1_ax/samples; err_AL[1] = s1_ay/samples; err_AL[2] = s1_az/samples; 
   // Calculate std deviation of samples and std error of bias
   deviation_AL[0] = standard_deviation(s1_ax, s2_ax, samples);
   deviation_AL[1] = standard_deviation(s1_ay, s2_ay, samples);
   deviation_AL[2] = standard_deviation(s1_az, s2_az, samples); 
   stderror_AL[0] = deviation_AL[0] / sqrtf(samples); stderror_AL[1] = deviation_AL[1] / sqrtf(samples); stderror_AL[2] = deviation_AL[2] / sqrtf(samples);
   printf("95%% confidence interval of accelerometer bias: x:%d+-%.1f, y:%d+-%.1f, z:%d+-%.1f\n", err_AL[0], conf95_factor*stderror_AL[0], 
                                                                                                  err_AL[1], conf95_factor*stderror_AL[1], 
                                                                                                  err_AL[2], conf95_factor*stderror_AL[2]);
   printf("Accelerometer standard deviation: sigma_x=%.1f, sigma_y=%.1f, sigma_z=%.1f\n", deviation_AL[0], deviation_AL[1], deviation_AL[2]);

   // Calculate the mean of gyroscope biases and store it in variable err_GY   
   err_GY[0] = s1_gx/samples; err_GY[1] = s1_gy/samples; err_GY[2] = s1_gz/samples; 
   // Calculate std deviation of samples and std error of bias
   deviation_GY[0] = standard_deviation(s1_gx, s2_gx, samples); 
   deviation_GY[1] = standard_deviation(s1_gy, s2_gy, samples); 
   deviation_GY[2] = standard_deviation(s1_gz, s2_gz, samples);
   stderror_GY[0] = deviation_GY[0] / sqrtf(samples); stderror_GY[1] = deviation_GY[1] / sqrtf(samples); stderror_GY[2] = deviation_GY[2] / sqrtf(samples);

   printf("95%% confidence interval of gyroscope bias: x:%d+-%.1f, y:%d+-%.1f, z:%d+-%.1f\n", err_GY[0], conf95_factor*stderror_GY[0], 
                                                                                              err_GY[1], conf95_factor*stderror_GY[1], 
                                                                                              err_GY[2], conf95_factor*stderror_GY[2]);
   printf("Gyroscope standard deviation: sigma_x=%.1f, sigma_y=%.1f, sigma_z=%.1f\n", deviation_GY[0], deviation_GY[1], deviation_GY[2]);   
                                                                                                   
   oledBigMessage(0, NULL);
   oledBigMessage(1, NULL);   
   return;
  
rw_error:
   err_AL[0] = err_AL[1] = err_AL[2] = 0;
   err_GY[0] = err_GY[1] = err_GY[2] = 0; 
   oledBigMessage(0, "ERROR!");
   oledBigMessage(1, NULL);  
   ERR(, "Cannot read/write data from IMU");
} 


/*
Step 1 of magnetometer calibration. 
While the car is stationary, it reads samples in order to estimate their standard deviation.
It stores the estimated values in global variable deviation_MA.
*/
static esp_err_t calculate_std_dev_magnetometer(void)
{
int64_t start_tick, elapsed_useconds;
int32_t s1_mx = 0, s1_my = 0, s1_mz = 0;
int64_t s2_mx = 0, s2_my = 0, s2_mz = 0;
int samples = 0;
const int error_meas_seconds = 8;  // Number of seconds for measurement
      
   oledBigMessage(0, "HORIZ.");
   oledBigMessage(1, "WAIT..."); 
   start_tick = esp_timer_get_time();
   
   do {
      int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
      
      // Wait for new data to arrive, acc. ODR selected (25 ms for 40 Hz)
      int32_t delay = lroundf(1000.0/odr_m_modes[ODR_M]);  // Time to wait for new data to arrive, in ms
      if (delay > portTICK_PERIOD_MS) vTaskDelay(pdMS_TO_TICKS(delay));
      else ets_delay_us(delay*1000); 

      // Read magnetometer data, if ready
      int rc = read_magnetometer(&mx, &my, &mz);
      if (rc < 0) return ESP_FAIL;
      if (rc == 1) {  // Data available
         // Calculate the sum of the samples
         s1_mx += mx; s1_my += my; s1_mz += mz;
         // Calculate the sum of the squared samples. Use 64 bit arithmetic, as 32 bit might overflow
         s2_mx += (int64_t)mx*(int64_t)mx; s2_my += (int64_t)my*(int64_t)my; s2_mz += (int64_t)mz*(int64_t)mz;
         samples++;
      }
      elapsed_useconds = esp_timer_get_time() - start_tick;
   } while (elapsed_useconds < error_meas_seconds*1E6);  // loop for given time

   oledBigMessage(0, NULL);
   oledBigMessage(1, NULL); 
   
   // Calculate std deviation of the samples
   deviation_MA[0] = standard_deviation(s1_mx, s2_mx, samples);
   deviation_MA[1] = standard_deviation(s1_my, s2_my, samples);
   deviation_MA[2] = standard_deviation(s1_mz, s2_mz, samples);
   return ESP_OK;
}




/* 
 Calibrate magnetometer.
 In step 1, the car must stay stationary. During this time, many magnetometer samples are read,
 in order to calculate the standard deviation of the samples, per axis.
 In step 2, the car must be rotated in all 3 axis and all directions, so that the magnetic field points
 in every possible direction.  
 The samples should be a sphere centered around the origin. However, due to 
 hardiron and softiron effects, the origin is not the center (hardiron) and the sphere is an ellipsoid, 
 because the 3 axis are stretched (softiron).
 The measured bias is estimated and stored in the global variables err_MA (hardiron) and scale_MA (softiron).
 These values are used later to correct the values read from the magnetometer.
 */
static void calibrate_magnetometer(void)
{
int64_t start_tick, elapsed_useconds;
SampleList_t sample_list = {.xvalues=NULL, .yvalues=NULL, .zvalues=NULL, .num_elems=0};
int16_t min_x=INT16_MAX, min_y=INT16_MAX, min_z=INT16_MAX;
int16_t max_x=INT16_MIN, max_y=INT16_MIN, max_z=INT16_MIN;
const int cal_seconds = 30; // Number of seconds to take samples when user rotates car
  
   ESP_LOGI(TAG, "Calibrating magnetometer");
   int max_num_samples = cal_seconds * odr_m_modes[ODR_M];  // Max number of samples to take, used to dimension memory needs
   if (!(sample_list.xvalues = calloc(max_num_samples, sizeof(int32_t))) || 
       !(sample_list.yvalues = calloc(max_num_samples, sizeof(int32_t))) || 
       !(sample_list.zvalues = calloc(max_num_samples, sizeof(int32_t)))) {
      ESP_LOGE(TAG, "Cannot allocate memory: %s", strerror(errno));
      goto cal_error; 
   }

   // Step 1: calculate standard deviation
   if (calculate_std_dev_magnetometer() != ESP_OK) goto cal_error;

   // Step 2: rotate car and take samples, to estimate hardiron and softiron errors
   printf("\nRotate robot car slowly in all directions for %d seconds...\n", cal_seconds);
   oledBigMessage(0, "ROTATE");
   oledBigMessage(1, "CAR...");
   pito(5, 1);    // Buzz for 5 tenths of a second, wait till done
   start_tick = esp_timer_get_time();
   
   do {
      uint8_t byte;
      esp_err_t rc;
      int ret;
      int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
 
      // Wait for new data to arrive, acc. ODR selected (25 ms for 40 Hz)
      int32_t delay = lroundf(1000.0/odr_m_modes[ODR_M]);  // Time to wait for new data to arrive, in ms
      if (delay > portTICK_PERIOD_MS) vTaskDelay(pdMS_TO_TICKS(delay));
      else ets_delay_us(delay*1000); 

      // Read magnetometer data, if ready
      ret = read_magnetometer(&mx, &my, &mz);
      if (ret < 0) goto cal_error;
      if (ret == 1) {  // Data available
         // Store samples in array sample_list
         if (sample_list.num_elems < max_num_samples) {
            sample_list.xvalues[sample_list.num_elems] = mx; 
            sample_list.yvalues[sample_list.num_elems] = my; 
            sample_list.zvalues[sample_list.num_elems] = mz;
            sample_list.num_elems++;
         }
         // Calculate limits of the ellipsoid: max and min values for all 3 axis
         if (mx>max_x) {max_x = mx;} if (my>max_y) {max_y = my;} if (mz>max_z) {max_z = mz;}
         if (mx<min_x) {min_x = mx;} if (my<min_y) {min_y = my;} if (mz<min_z) {min_z = mz;} 
      }
      elapsed_useconds = esp_timer_get_time() - start_tick;   
   } while (elapsed_useconds < cal_seconds*1E6);  // loop for given time
   
   pito(5, 1);    // Buzz for 5 tenths of a second, wait till done
   oledBigMessage(0, NULL);
   oledBigMessage(1, NULL);   
   
   /****** Now estimate hardiron and softiron effects ******/
   // Hardiron error, initial estimation: the center of the ellipsoid
   // This is estimated with the mean value of the extremes, for each axis
   err_MA[0] = (min_x + max_x)/2; err_MA[1] = (min_y + max_y)/2; err_MA[2] = (min_z + max_z)/2;
       
   // Softiron error, initial estimation: the stretching of the axis (compared to a sphere)
   // The axis are estimated based on the extremes, and the real sphere radius is estimated as their average
   int rad_x = (max_x - min_x)/2; int rad_y = (max_y - min_y)/2; int rad_z = (max_z - min_z)/2;     
   float rad_mean = (rad_x + rad_y + rad_z)/3.0;
   scale_MA[0] = rad_mean/rad_x; scale_MA[1] = rad_mean/rad_y; scale_MA[2] = rad_mean/rad_z; 
   
   // Estimate total strength of magnetic field in IMU units. If the ellipsoid is almost a sphere,
   // ie, its axis are very similar (5% error), then the softiron bias can be ignored and the field strength is the measured value
   // Otherwise, take value from the global variable default_magnetic_field
   float imu_magnetic_field = default_magnetic_field / mRes;   // Convert from Gauss to IMU units
   if ((fabsf(scale_MA[1]/scale_MA[0]-1.0) < 0.05) && (fabsf(scale_MA[2]/scale_MA[0]-1.0) < 0.05)) {
      imu_magnetic_field = rad_mean;  // If ellipsoid is almost a sphere, take this value as valid
      printf("Measured magnetic field: %.3f Gauss\n", imu_magnetic_field*mRes);
   }
   
   // Calculate a better approximation for err_MA and scale_MA, changing these global variables if succesful
   float mean_quad_error = ellipsoid_fit(&sample_list, imu_magnetic_field);  // Returns mean quadratic error value of new approximation
   if (mean_quad_error > 0.02) {  // Limit value found experimentally
      ESP_LOGW(TAG, "Mean quadratic error (%.3f) is too big, repeat calibration", mean_quad_error);
      goto cal_error;
   }
   
   printf("Magnetometer hardiron bias: %d %d %d\n", err_MA[0], err_MA[1], err_MA[2]);
   printf("Magnetometer softiron bias: %.3f %.3f %.3f\n", scale_MA[0], scale_MA[1], scale_MA[2]);
   printf("Magnetometer standard deviation: sigma_x=%.1f, sigma_y=%.1f, sigma_z=%.1f\n", deviation_MA[0], deviation_MA[1], deviation_MA[2]);
      
   free(sample_list.xvalues); free(sample_list.yvalues); free(sample_list.zvalues);
   return;
   
cal_error:
   if (sample_list.xvalues) free(sample_list.xvalues); 
   if (sample_list.yvalues) free(sample_list.yvalues); 
   if (sample_list.zvalues) free(sample_list.zvalues);
   err_MA[0] = err_MA[1] = err_MA[2] = 0; 
   scale_MA[0] = scale_MA[1] = scale_MA[2] = 1.0;
   oledBigMessage(0, "CAL");
   oledBigMessage(1, "ERROR");
   pito(10, 1);  // Buzz for 10 tenths of a second, wait till done
   oledBigMessage(0, NULL);
   oledBigMessage(1, NULL);   
   ERR(, "Cannot calibrate magnetometer");   
}


/* 
   Calculate a better approximation for err_MA and scale_MA when calibrating the magnetometer. 
   Do it by fitting the measured magnetic samples (obtained by slowly rotating the car) into an ellipsoid. 
   It returns the mean quadratic error of the fit.
   Non constant global variables accessed: err_MA, scale_MA, mRes. They must have correct values prior to call.
   err_MA and scale_MA must have plausible values when this function is called, as it will start searching from these values.
   The variables err_MA and scale_MA are modified to reflect the new fit.
   err_MA contains the coordinates of the center of the ellipsoid (Vx, Vy, Vz): hardiron bias
   scale_MA contains the scaling factors of the ellipsoid (A, B, C): softiron bias
   It is based on ideas from https://www.nxp.com/docs/en/application-note/AN4246.pdf
   
   This ellipsoid is assumed to have its axis parallel to X, Y and Z (ie, it is not rotated). This assumption is normally true.
   The assumption implies that the ellipsoid fit matrix A is diagonal, leaving only three unknowns, A^2, B^2 and C^2.
   The inverse softiron matrix W^(-1) is then also diagonal, with values A, B and C.
   The function calculates the ellipsoid that fits best into the given samples, 
   minimizing the quadratic error: sum of the square errors of all samples with respect to the ellipsoid surface of the fit.
   In order to do that, it travels its path, starting from the given initial values in err_MA and scale_MA, along
   the opposite gradient of the error function until a minimum is found.
   The correct magnitude of the magnetic field cannot be calculated from the samples, 
   as all 3 axis can be subject to softiron deformation. So the approximation uses the variable Bm, 
   which should contain the magnitude in IMU units of the magnetic field in the location.
   A correct value is not needed for orientation (ie, calculation of yaw, pitch and roll), so its accurate filling is optative.
*/
static float ellipsoid_fit(const SampleList_t *const sample_list, float Bm)
{
float Vx, Vy, Vz;
float dVx, dVy, dVz;
float A2, B2, C2;
float dA2, dB2, dC2; 
float err, init_err, min_err; 
float min_found[7];  // err, Vx, Vy, Vx, A^2, B^2, C^2 
   
   /* Set initial point from global variables */
   Vx = (float)err_MA[0]; Vy = (float)err_MA[1]; Vz = (float)err_MA[2];
   A2 = scale_MA[0]*scale_MA[0]; B2 = scale_MA[1]*scale_MA[1]; C2 = scale_MA[2]*scale_MA[2];  
   
   /* Iterate a maximum of 50 times to find a minimum in the error function */
   for (int iter=0; iter<50;iter++) {
      /* Compute error of current point */
      err = quad_error_function(Vx, Vy, Vz, A2, B2, C2, Bm, sample_list);
      if (iter==0) init_err = err;  // Store error of initial point
      /* Check if we found a minimum */
      if (iter==0 || err<min_err) {
         min_err = min_found[0] = err; 
         min_found[1] = Vx; min_found[2] = Vy; min_found[3] = Vz; 
         min_found[4] = A2; min_found[5] = B2; min_found[6] = C2; 
      }
      
      /* Calculate gradient of error function, using finite difference approximation */
      const float delta = 0.001;  // Small increment of variable, to compute gradient (partial derivatives)
      dA2 = (quad_error_function(Vx, Vy, Vz, A2+delta, B2, C2, Bm, sample_list) -
            quad_error_function(Vx, Vy, Vz, A2-delta, B2, C2, Bm, sample_list)) / (2*delta);
      dB2 = (quad_error_function(Vx, Vy, Vz, A2, B2+delta, C2, Bm, sample_list) -
            quad_error_function(Vx, Vy, Vz, A2, B2-delta, C2, Bm, sample_list)) / (2*delta);
      dC2 = (quad_error_function(Vx, Vy, Vz, A2, B2, C2+delta, Bm, sample_list) -
            quad_error_function(Vx, Vy, Vz, A2, B2, C2-delta, Bm, sample_list)) / (2*delta);
      dVx = (quad_error_function(Vx+delta, Vy, Vz, A2, B2, C2, Bm, sample_list) -
            quad_error_function(Vx-delta, Vy, Vz, A2, B2, C2, Bm, sample_list)) / (2*delta);
      dVy = (quad_error_function(Vx, Vy+delta, Vz, A2, B2, C2, Bm, sample_list) -
            quad_error_function(Vx, Vy-delta, Vz, A2, B2, C2, Bm, sample_list)) / (2*delta);
      dVz = (quad_error_function(Vx, Vy, Vz+delta, A2, B2, C2, Bm, sample_list) -
            quad_error_function(Vx, Vy, Vz-delta, A2, B2, C2, Bm, sample_list)) / (2*delta);
      
      /* Calculate new point in direction of negative gradient, separately for each 3 dimension space */
      float step = 0.5/(2+iter);  // we use 0.5 instead of 2.0 (as is usual eg in Frank–Wolfe algorithm) to get smaller steps
      float mod_gradient = sqrtf(dA2*dA2+dB2*dB2+dC2*dC2);
      A2 -= dA2*step/mod_gradient;
      B2 -= dB2*step/mod_gradient;  
      C2 -= dC2*step/mod_gradient;  
     
      step *= 100;  // The step must be higher for the center coordinates, otherwise they hardly move
      mod_gradient = sqrtf(dVx*dVx+dVy*dVy+dVz*dVz);
      Vx -= dVx*step/mod_gradient;
      Vy -= dVy*step/mod_gradient;
      Vz -= dVz*step/mod_gradient;
   }

   /* A minimum was possibly found */
   if (min_err < init_err) {  // if error of new point is less than initial error, eureka
      err_MA[0] = lroundf(min_found[1]); err_MA[1] = lroundf(min_found[2]); err_MA[2] = lroundf(min_found[3]);
      scale_MA[0] = sqrtf(min_found[4]); scale_MA[1] = sqrtf(min_found[5]); scale_MA[2] = sqrtf(min_found[6]); 
      printf("Minimum found, initial error=%.1f, final error=%.1f, mean quad error=%.3f\n", init_err, min_err, min_err/sample_list->num_elems);
   }
   return min_err/sample_list->num_elems;  // Mean quadratic error
}


/*
Compute the total quadratic error of the ellipsoid fit given by the parameters Vx, Vy, Vz (the displacement, or hardiron)
and A2, B2, C2 (the axis coefficients squared, or softiron), all in IMU units. It uses the given magnetometer samples in the sample_list.
Bm is the local magnetic field strength (in IMU scale, between -32768 and 32767).
The total error is the sum of the quadratic error of each sample for the given fit.
The ellipsoid equation is A^2*(Bx-Vx)^2+B^2*(By-Vy)^2+C^2*(Bz-Vz)^2 - Bm^2 = 0, so the 
error of each sample measures how far from zero is each sample in that equation.
*/
static float quad_error_function(float Vx, float Vy, float Vz, float A2, float B2, float C2, 
                            float Bm, const SampleList_t *const sample_list)
{
float quad_err_total = 0.0;
   
   for (int i=0; i < sample_list->num_elems; i++) {
      float ex = (sample_list->xvalues[i]-Vx)/Bm; 
      float ey = (sample_list->yvalues[i]-Vy)/Bm; 
      float ez = (sample_list->zvalues[i]-Vz)/Bm;
      float err = A2*ex*ex + B2*ey*ey + C2*ez*ez - 1;  // error of this sample in equation
      quad_err_total += err * err;  // add the squared errors
   }   
   return quad_err_total;
}



/************************************************************
Initialize IMU LSM9DS1
Input: I2C address of accel/gyro and of magnetometer, indication whether to perform calibration of IMU
Return value: 0 (OK), -1 (read/write error), -2 (must perform calibration)
************************************************************/
int setupLSM9DS1(uint8_t accel_addr, uint8_t mag_addr, bool do_calibrate)
{
esp_err_t rc;
uint8_t byte;

   /***** Initial checks *****/
   assert(odr_ag_modes[ODR_AG] > odr_m_modes[ODR_M]);
   upsampling_factor = lroundf(odr_ag_modes[ODR_AG] / odr_m_modes[ODR_M]);
   
   // Check acelerometer/gyroscope   
   rc = i2cWriteReadBytes(accel_addr, 0x0F, &byte, 1);  // Read WHO_AM_I register
   if (rc != ESP_OK) goto rw_error;
   if (byte != 0x68) ERR(-1, "Invalid accelerometer/gyroscope device");
   accelAddr = accel_addr;
   
   // Check magnetometer  
   rc = i2cWriteReadBytes(mag_addr, 0x0F, &byte, 1);  // Read WHO_AM_I register
   if (rc != ESP_OK) goto rw_error;
   if (byte != 0x3D) ERR(-1, "Invalid magnetometer device");  
   magAddr = mag_addr;
     
   /************************* Set Magnetometer ***********************/
   // Set magnetometer, CTRL_REG1_M
   // TEMP_COMP: Yes (b1), XY mode: ultra (b11), ODR: 40 Hz (b110), 0 (b0), ST: No (b0)
   byte = (0x01<<7) + (0x03<<5) + (ODR_M<<2) + 0x0; 
   rc = i2cWriteByte(magAddr, 0x20, byte);
   if (rc < 0) goto rw_error;  
   
   // Set magnetometer, CTRL_REG2_M
   // 0 (b0), FS: 4 Gauss (b00), REBOOT: 0 (b0), SOFT_RST: 0 (b0), 0 (b0), 0 (b0)
   // 4000 mGauss is enough for earth magnetic field + hardiron offset
   byte = 0x0; 
   rc = i2cWriteByte(magAddr, 0x21, byte);
   if (rc != ESP_OK) goto rw_error;   
   mRes = 4.0/32768;   // G/LSB
   
   // Activate magnetometer, CTRL_REG3_M
   // I2C: enabled (b0), 0 (b0), LP: No (b0), 0 (b0), 0 (b0), SPI: wo (b0), mode: Continuous (b00)
   byte = 0x0; 
   rc = i2cWriteByte(magAddr, 0x22, byte);
   if (rc != ESP_OK) goto rw_error; 
      
   // Set magnetometer, CTRL_REG4_M
   // OMZ: ultra (b11), BLE: data LSb at lower address (b0)
   byte = 0x03<<2; 
   rc = i2cWriteByte(magAddr, 0x23, byte);
   if (rc < 0) goto rw_error;
   
   // Set magnetometer, CTRL_REG5_M
   // BDU:  continuous update (b0)
   byte = 0x00; 
   rc = i2cWriteByte(magAddr, 0x24, byte);
   if (rc != ESP_OK) goto rw_error;
   
   /************************* Set Acelerometer / Gyroscope ***********************/   
   // Set IMU, CTRL_REG8
   // BOOT: normal mode (b0), BDU: continuous update (b0), H_LACTIVE: active high (b0),
   // PP_OD: 0 (b0), SIM: 0 (b0), IF_ADD_INC: enabled (b1), BLE: data LSB @ lower address (b0), SW_RESET: normal mode (b0)
   byte = 0x04; 
   rc = i2cWriteByte(accelAddr, 0x22, byte);
   if (rc != ESP_OK) goto rw_error; 
   
   // Set accelerometer, CTRL_REG5_XL
   // DEC: no decimation (b00), Zen_XL, Yen_XL, Xen_XL: enabled (b1)
   byte = (0x00<<6) + (0x07<<3); 
   rc = i2cWriteByte(accelAddr, 0x1F, byte);
   if (rc != ESP_OK) goto rw_error; 
 
   // Set accelerometer, CTRL_REG6_XL
   // ODR: Power down (b000), FS: 2g (b00), BW_SCAL: auto (b0), BW sel: 408 Hz (b00)
   byte = (0x00<<5) + (0x0<<3) + 0x0; 
   rc = i2cWriteByte(accelAddr, 0x20, byte);
   if (rc != ESP_OK) goto rw_error; 
   aRes = 2.0/32768;   // g/LSB

   // Set accelerometer, CTRL_REG7_XL
   // HR: enabled (b1), DCF: ODR/9 (b10), FDS: internal filter bypassed (b0), HPIS1: filter bypassed (b0)
   // both LPF enabled, HPF disabled
   byte = (0x01<<7) + (0x02<<5) + 0x0; 
   rc = i2cWriteByte(accelAddr, 0x21, byte);
   if (rc != ESP_OK) goto rw_error; 
   
   // Set gyro, CTRL_REG4
   // Zen_G, Yen_G, Xen_G: enabled (b1), LIR_XL1: 0 (b0), 4D_XL1: 0 (b0)
   byte = 0x07<<3; 
   rc = i2cWriteByte(accelAddr, 0x1E, byte);
   if (rc != ESP_OK) goto rw_error; 
   
   // Set gyro, ORIENT_CFG_G
   // SignX_G, SignX_G, SignX_G: positive sign (b0), Orient: 0 (b000) (Pitch:X, Roll:Y, Yaw:Z)
   byte = 0x00; 
   rc = i2cWriteByte(accelAddr, 0x13, byte);
   if (rc != ESP_OK) goto rw_error; 
   
   // Activate accelerometer and gyro, CTRL_REG1_G. Both use the same ODR
   // ODR: xxx Hz LPF1 31 Hz (b011), Gyro scale: 245 dps (b00), 0 (b0), Gyro BW LPF2: 31 Hz (b01)
   // The ODR is variable, acc. ODR_AG, LPF2 is set to be always around 30 Hz (b01) (for ODR of 119 Hz and above)
   byte = (ODR_AG<<5) + (0x0<<3) + 0x01; 
   rc = i2cWriteByte(accelAddr, 0x10, byte);
   if (rc != ESP_OK) goto rw_error; 
   gRes = 245.0/32768;   // dps/LSB
   
   // Set gyro, CTRL_REG2_G
   // INT_SEL: 0 (b00), OUT_SEL: 0 (b10) (output after LPF2)
   byte = 0x02; 
   rc = i2cWriteByte(accelAddr, 0x11, byte);
   if (rc != ESP_OK) goto rw_error; 
   
   // Set gyro, CTRL_REG3_G
   // LP_mode: disabled (b0), HP_EN: disabled (b0), HPCF_G: 0.5 Hz for 119 Hz ODR (b0100) 
   // both LPF enabled, HPF disabled
   byte = (0x0<<7) + (0x0<<6) + 0x04; 
   rc = i2cWriteByte(accelAddr, 0x12, byte);
   if (rc != ESP_OK) goto rw_error;  

   /* Empty and reset FIFO, in case it was active, so we start afresh */
   rc = i2cWriteByte(accelAddr, 0x2E, 0x00);  // Reset existing FIFO content by selecting FIFO Bypass mode 
   if (rc != ESP_OK) goto rw_error;  
 
   /* Initial samples after FIFO change and after power up (Table 12 of user manual) have to be discarded */
   rc = read_discard_fifo_samples(20);
   if (rc != ESP_OK) goto rw_error; 
   
   /************************** Handle calibration ********************/
   if (read_calibration_data()==-1 && do_calibrate==false) return -2;  // Must calibrate IMU, cannot run further
   if (do_calibrate) {
      calibrate_accel_gyro();
      calibrate_magnetometer();
      write_calibration_data();
      vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep 1 second, so the user can continue the start process
   }
   
   /************** Continue with accelerometer setting, activate FIFO ****************/
   // Set FIFO, FIFO_CTRL
   // FMODE: continuous mode (b110), threshold: 0 (b00000)
   byte = (0x06<<5) + 0x0; 
   rc = i2cWriteByte(accelAddr, 0x2E, byte);
   if (rc != ESP_OK) goto rw_error; 

   // Activate FIFO, CTRL_REG9
   // SLEEP_G: disabled (b0), FIFO_TEMP_EN: no (b0), 
   // DRDY_mask_bit: disabled (b0), I2C_DISABLE: both (b0), FIFO_EN: yes (b1), STOP_ON_FTH: no (b0)
   byte = 0x02; 
   rc = i2cWriteByte(accelAddr, 0x23, byte);
   if (rc != ESP_OK) goto rw_error;  
   
   rc = read_discard_fifo_samples(2);  // Discard samples after FIFO activation
   if (rc != ESP_OK) goto rw_error; 
   
   /************************* Final actions ***********************/
   
   // Start the IMU reading timer, calling IMU_read every delay_ms
   uint32_t delay_ms = lroundf(1000.0/odr_m_modes[ODR_M]*1.2);  // Read IMU with a period of magnetometer ODR, add margin
   esp_timer_handle_t timer;
   const esp_timer_create_args_t timer_args = {
            .dispatch_method = ESP_TIMER_TASK,
            .callback = &IMU_read,
            .arg = (void*)NULL,
            .name = "IMU_read"
            };

   ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
   ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000*delay_ms));
   ESP_LOGI(TAG, "IMU is read every %lu ms", delay_ms);
   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   ERR(-1, "Cannot read/write data from IMU");
}


/***
Read data from magnetometer. Return raw values for each axis in the 3 passed pointers.
Return value: -1 if error in reading the I2C bus, 0 if no data available yet, 1 if data available and successfully read
***/
static int read_magnetometer(int16_t *mx, int16_t *my, int16_t *mz)
{
uint8_t byte, buf[6];
esp_err_t rc;

   // Read status register in magnetometer, to check if new data is available
   rc = i2cWriteReadBytes(magAddr, 0x27, &byte, 1);  // Read magnetometer STATUS_REG register
   if (rc != ESP_OK) return -1;    // Read error, error message already sent
   if ((byte&0x08) == 0) return 0; // If no new data available, return 0

   /* New magnetometer data in XYZ is available
      Read magnetometer data. X and Y axis are exchanged, and then Y is negated, 
      so align the axis with the ones used in this module for car orientation */
         
   rc = i2cWriteReadBytes(magAddr, 0x28, buf, sizeof(buf));  // Read 6 bytes (X,Y,Z axis), starting in OUT_X_L_M register
   if (rc != ESP_OK) return -1;   // Read error, error message already sent
   
   *my = buf[1]<<8 | buf[0]; *mx = buf[3]<<8 | buf[2]; *mz = buf[5]<<8 | buf[4];  
   *my *= -1;
   
   return 1;  // Data available and successfully read, return 1
}



/***
This function is called periodically, with the rate of the magnetometer ODR.
It reads the IMU data and feeds the Magdwick fusion filter or the 3D tilt compensated compass algorithm. 
Both do the same and have the same results, but the fusion filter needs much longer to converge.

The magnetomer data is read. If data was available, the function goes on to read the FIFO
of the accelerometer/gyroscope, which has a much higher ODR (this is why the FIFO is used, 
to store data without having to poll so often).
For each value of the accel/gyro, the fusion filter or the 3D compensated compass are called, using
interpolated magnetometer data, so both sample rates are the same. 

CHECK It takes about 5 ms to complete (mostly between 4.5 and 5.5 ms) for ODR_M=40 Hz, ODR_AGG=238 Hz.
Runs on CPU0, in a very high priority task
***/
static void IMU_read(void* arg)
{
int rc;   
uint8_t samples, byte;
static uint8_t buf[FIFO_LINE_SIZE*32];  // static, so it does not grow the stack; 32 is FIFO size
char str[OLED_MAX_LINE_SIZE];
static unsigned int samples_count, count, collision_sample;
//static bool in_collision;

/* Real (scaled and compensated) readings of the sensors */
float axr, ayr, azr;
float gxr, gyr, gzr;
float mxr, myr, mzr;
static float o_mxr, o_myr, o_mzr; // Previous values
//float axrf, ayrf, azrf; // values after LPF
//float daxr;
   
   /** Read magnetometer data, if ready **/
   int16_t mx, my, mz; // x, y, and z axis raw readings of the magnetometer
   rc = read_magnetometer(&mx, &my, &mz);  // Takes ca. 0.5 ms
   if (rc < 0) goto rw_error;
   if (rc == 0) {
      ESP_LOGW(TAG, "No magnetometer data");
      return;  // No magnetometer data yet
   }
   /* Substract measured error values (hardiron effects) */
   mx -= err_MA[0]; my -= err_MA[1]; mz -= err_MA[2];      
   /* Compensate for softiron and scale the result */
   mxr = mx*scale_MA[0]*mRes; myr = my*scale_MA[1]*mRes; mzr = mz*scale_MA[2]*mRes;   

   /**
   Now, read accel/gyro data. Read data from FIFO, as ODR of accel/gyro is higher
   than that of magnetometer. The accel/gyro stores samples in the FIFO until it is read.
   **/

   rc = i2cWriteReadBytes(accelAddr, 0x2F, &byte, 1);  // Read FIFO_SRC register
   if (rc < 0) goto rw_error;
   if (byte & 0x40) ESP_LOGW(TAG, "%s: FIFO overrun!", __func__);  // Should not happen
   // samples in FIFO are between 3 and 4, average 3: AG ODR = 119 Hz, M ODR = 40 Hz, 119/40=3
   // if AG ODR = 238, samples is between 6 and 8
   samples = byte & 0x3F;   // samples in FIFO could be zero  
   //printf("Samples: %d\n", samples);
   if (samples < upsampling_factor) {  // Should not happen
      ESP_LOGW(TAG, "%s: Timing problem: number of samples of accelerometer (%d) is less than upsampling factor (%d)", 
            __func__, samples, upsampling_factor);    
   }    
   
   if (samples) {  // if FIFO has sth, read it
      /* Burst read. Accelerometer and gyroscope sensors are activated at the same ODR.
         So read all FIFO lines (12 bytes each) in a single I2C transaction */
      rc = i2cWriteReadBytes(accelAddr, 0x18, buf, FIFO_LINE_SIZE*samples);  // Takes ca. 2.2 ms for 7 samples
      if (rc < 0) goto rw_error;         
   }
   // Process samples
   for (int n=0; n<samples; n++) {
      /* These values are the raw signed 16-bit readings from the sensors */
      int16_t gx, gy, gz; // x, y, and z axis raw readings of the gyroscope
      int16_t ax, ay, az; // x, y, and z axis raw readings of the accelerometer      
      uint8_t *p = buf + FIFO_LINE_SIZE*n;  
      
      /* Store accel and gyro data. X and Y axis are exchanged, so that reference system is
      right handed, X axis points forwards, Y to the left, and filter algorithms work correctly */
      gy = p[1]<<8 | p[0]; gx = p[3]<<8 | p[2]; gz = p[5]<<8 | p[4];
      ay = p[7]<<8 | p[6]; ax = p[9]<<8 | p[8]; az = p[11]<<8 | p[10];        
   
      /* Substract the measured error values obtained during calibration */
      ax -= err_AL[0]; ay -= err_AL[1]; az -= err_AL[2]; 
      gx -= err_GY[0]; gy -= err_GY[1]; gz -= err_GY[2]; 

      /* Store real values in floating point variables, apply scaling */
      axr = ax*aRes; ayr = ay*aRes; azr = az*aRes;      
      gxr = gx*gRes; gyr = gy*gRes; gzr = gz*gRes;  

      /* Perform upsampling of magnetometer samples to the ODR of the accelerometer/gyro (ie, by a factor of N). 
      First, introduce N-1 0-valued samples to align both ODR. Then, filter with a low pass filter to
      eliminate the spectral replica of the original signal. This interpolates the values.
      The low pass filter is implemented as a single pole IIR filter: y(n) = a*x(n) + (1-a)*y(n-1), 0 < a < 1 */
      const float alpha = 0.02;  // Rapid decay response, with 16 dB (5 Hz), 22 dB (10 Hz), 28 dB (20 Hz), 34 dB (40 Hz), 39 dB (fsample/2, 120 Hz)
      //const float alpha = 0.01;  // Rapid decay response, with 22 dB (5 Hz), 28 dB (10 Hz), 34 dB (20 Hz), 40 dB (40 Hz), 46 dB (fsample/2, 120 Hz)
      // Calculate new mxr myr, mzr values by filtering the existing values
      if (samples_count++%upsampling_factor == 0) {  // Introduce measured value as input in filter every N (upsampling_factor) samples
         mxr = alpha*mxr + (1-alpha)*o_mxr; myr = alpha*myr + (1-alpha)*o_myr; mzr = alpha*mzr + (1-alpha)*o_mzr;
      }
      else { // Insert zeros as input in filter (mxr=0, etc) every N-1 samples
         mxr = (1-alpha)*o_mxr; myr = (1-alpha)*o_myr; mzr = (1-alpha)*o_mzr;  
      }
      o_mxr = mxr; o_myr = myr; o_mzr = mzr;  // Remember these output values for next sample
      
      mxr *= upsampling_factor; myr *= upsampling_factor; mzr *= upsampling_factor;  // Compensate for DC gain loss after interpolating filter
      //printf("axr=%.1f ayr=%.1f azr=%.1f\n", axr, ayr, azr);
      //printf("Magnetic field: %f\n", sqrtf(mxr*mxr+myr*myr+mzr*mzr));
      
      
      /***** IMU data is available. Now perform whatever operations are needed with the obtained values *****/
      
      getOrientation(axr, ayr, azr, mxr, myr, mzr);  // Sets global variables roll, pitch, yaw, tilt;
      //printf("Yaw %3.0f   Pitch %3.0f   Roll %3.0f   Tilt %3.0f\n", yaw, pitch, roll, tilt);  
      if (!(count++%10)) {
         snprintf(str, sizeof(str), "Yaw: %3.0f ", yaw);  
         oledWriteString(0, 6, str, false);    
      }
      
      
      
   
   
   }

   return;
   
/* error handling if read operation from I2C bus failed */
rw_error:
   //closeLSM9DS1();
   ERR(, "Cannot read/write data from IMU");
}



