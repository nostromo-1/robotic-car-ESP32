/*************************************************************************

IMU control code, for the LSM9DS1 9DoF MARG sensor (Magnetic, Angular Rate and Gravity)
3D accelerometer, 3D gyroscope, 3D magnetometer 
Controlled by I2C bus (should be 400 KHz). It shows two addresses: 
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
         ESP_LOGE(TAG, "%s: " format "\n" , __func__ , ## arg);        \
         return ret;                                                   \
   }

#define STD_DEV(s1, s2, N) sqrt(((s2) - ((s1)*(s1))/(float)(N))/((N)-1))

#define READ_ATOMIC(var) atomic_load_explicit(&var, memory_order_acquire)
#define WRITE_ATOMIC(var,value) atomic_store_explicit(&var, value, memory_order_release)


#define I2C_BUS I2C_NUM_0   // i2c bus of IMU
#define I2C_MASTER_TIMEOUT_MS   100


extern _Atomic bool collision;  // Car has crashed when moving forwards or backwards

typedef struct {
    int *xvalues, *yvalues, *zvalues;
    int num_elems;
} SampleList_t;


typedef struct {
  float *history, *taps;
  unsigned int last_index, taps_num;
} Filter_t;


static const char* TAG = __FILE__;
static uint8_t accelAddr, magAddr;  // I2C addresses
static FILE *accel_fp;


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
static const float odr_ag_modes[] = {0,14.9,59.5,119,238,476,952};  // Values in Hz

static const enum {M_ODR_0_625,M_ODR_1_25,M_ODR_2_5,M_ODR_5,M_ODR_10,M_ODR_20,M_ODR_40,M_ODR_80} ODR_M = M_ODR_40;   
static const float odr_m_modes[] = {0.625,1.25,2.5,5,10,20,40,80};  // Values in Hz

static const float deltat = 1.0/odr_ag_modes[ODR_AG];  // Inverse of gyro/accel ODR
static int upsampling_factor;  /* Ratio between both ODRs */



// gRes, aRes, and mRes store the current resolution for each sensor. 
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15), as data resolution is 16 bits, signed.
static float gRes, aRes, mRes;

/* Store error offset for calibration of accel/gyro and magnetometer */
static int16_t err_AL[3];  // ex,ey,ez values (error for each axis in accelerometer)
static int16_t err_GY[3];  // ex,ey,ez values (error for each axis in gyroscope)
static int16_t err_MA[3];  // ex,ey,ez values (error for each axis in magnetometer, hardiron effects)
static float scale_MA[3] = {1.0, 1.0, 1.0}; // ex,ey,ez values (error for each axis in magnetometer, softiron effects)

static float deviation_AL[3];  // Measured standard deviation of x, y and z values of accelerometer
static float deviation_GY[3];  // Measured standard deviation of x, y and z values of gyroscope
static float deviation_MA[3];  // Measured standard deviation of x, y and z values of magnetometer

static const char *cal_file = "/spiffs/calibration.dat"; // File where calibration data is stored
static const char *dev_file = "/spiffs/deviation.dat";   // File where standard deviation data is stored
static const float declination = +1.866;   // Local magnetic declination as given by http://www.magnetic-declination.com/
static const float magneticField = 0.458;  // Magnitude of the local magnetic field in Gauss (does not need to be exact)

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
#define GyroMeasError (M_PI * (40.0/180.0))
static const float beta = 1.73205/2 * GyroMeasError;   // compute beta, sqrt(3/4)*GyroMeasError



// Output variables of module
float roll, pitch, yaw, tilt;


/* Prototypes */

static void IMU_read(void*);
/*
static void calibrate_accel_gyro(void);
static void calibrate_magnetometer(void);
static float ellipsoid_fit(const SampleList_t *sample_list);
static float quad_error_function(float Vx, float Vy, float Vz, float A, float B, float C, 
                            float Bm, const SampleList_t *sample_list);
static void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, 
                                     float mx, float my, float mz);
*/                                     
                                     
                                     
                                     


/*
Function to calculate heading, using magnetometer readings.
It only works if the sensor is lying flat (z-axis normal to Earth).
It is a non tilt-compensated compass.
*/
void printHeading(float mx, float my)
{
float heading;
char str[17];
  
   heading = -180/M_PI*atan2(my, mx);  // positive westwards
   //printf("Heading: %3.0f\n", heading);
   snprintf(str, sizeof(str), "Head:%-3.0f", heading);  
   oledWriteString(0, 5, str, false);  
}


/*
This function prints the LSM9DS1's orientation based on the
accelerometer and magnetometer data: its roll, pitch and yaw angles, in aerospace convention.
It represents a 3D tilt-compensated compass.
It also calculates the tilt: angle that the normal of the car forms with the vertical.

Procedure according https://www.nxp.com/docs/en/application-note/AN4248.pdf, 
https://www.nxp.com/docs/en/application-note/AN4249.pdf and https://www.nxp.com/docs/en/application-note/AN3461.pdf 
Angles according extrinsic rotation sequence x-y-z (https://en.wikipedia.org/wiki/Euler_angles),
which is equivalent to the intrinsic rotation z-y'-x'' (so the angles are the same): yaw -> pitch -> roll.
See also https://en.wikipedia.org/wiki/Davenport_chained_rotations
*/
void updateOrientation(float ax, float ay, float az, float mx, float my, float mz)
{
float sinpitch, cospitch, sinroll, cosroll, rootayaz, rootaxayaz, alpha=0.05;
  
   rootayaz = sqrt(ay*ay+az*az);
   rootaxayaz = sqrt(ax*ax+ay*ay+az*az);
   
   /*********** Calculate roll and pitch *************/
   // Original roll equation: roll = atan2(ay, az).
   // But this is unstable when ay and az tend to zero (pitch=90 degrees).
   // To avoid this, add 5% of ax in denominator
   roll = atan2(ay, az+alpha*ax);  // roll angle able to range between -180 and 180, positive clockwise
   pitch = atan(-ax/rootayaz);     // pitch angle restricted between -90 and 90, positive downwards
   
   /*********** Calculate tilt-compensated heading (yaw angle) *************/
   // intermediate results
   sinroll = ay/rootayaz;
   cosroll = az/rootayaz;
   sinpitch = -ax/rootaxayaz;
   cospitch = rootayaz/rootaxayaz;
   
   // now, calculate yaw
   // yaw angle able to range between -180 and 180, positive westwards
   yaw = atan2(mz*sinroll-my*cosroll, mx*cospitch+my*sinpitch*sinroll+mz*sinpitch*cosroll);
   
   /*********** Calculate tilt angle from vertical: cos(tilt)=cos(roll)*cos(pitch) *************/ 
   tilt = acos(cosroll*cospitch);  // tilt angle able to range between 0 and 180
   
   /*** Translate angles in radians to degrees ***/
   tilt *= 180/M_PI; 
   yaw *= 180/M_PI; yaw -= declination; if (yaw<0) yaw += 360;  // yaw (heading) must be positive
   pitch *= 180/M_PI; 
   roll *= 180/M_PI; 
     
   //printf("M/A-based Yaw, Pitch, Roll: %3.0f %3.0f %3.0f   Tilt: %3.0f\n", yaw, pitch, roll, tilt);
}



/* Read the calibration data for accelerometer, gyroscope and magnetometer, if file exists */
static void read_calibration_data(void)
{
FILE *fp;  
int rc;
   
   fp = fopen(cal_file, "r");
   if (!fp) ERR(, "Cannot open calibration file %s: %s", cal_file, strerror(errno));
   rc = fscanf(fp, "AL: %hd, %hd, %hd\n", &err_AL[0], &err_AL[1], &err_AL[2]);
   if (rc==EOF || rc!=3) goto cal_error;
   rc = fscanf(fp, "GY: %hd, %hd, %hd\n", &err_GY[0], &err_GY[1], &err_GY[2]);   
   if (rc==EOF || rc!=3) goto cal_error;
   rc = fscanf(fp, "MGH: %hd, %hd, %hd\n", &err_MA[0], &err_MA[1], &err_MA[2]);
   if (rc==EOF || rc!=3) goto cal_error;   
   rc = fscanf(fp, "MGS: %f, %f, %f\n", &scale_MA[0], &scale_MA[1], &scale_MA[2]);   
   if (rc==EOF || rc!=3) goto cal_error;  
   fclose(fp);

   /*
   fp = fopen(dev_file, "r");
   if (!fp) ERR(, "Cannot open deviation file %s: %s", dev_file, strerror(errno));
   
   rc = fscanf(fp, "AL: %f, %f, %f\n", &deviation_AL[0], &deviation_AL[1], &deviation_AL[2]);
   if (rc==EOF || rc!=3) goto dev_error;
   rc = fscanf(fp, "GY: %f, %f, %f\n", &deviation_GY[0], &deviation_GY[1], &deviation_GY[2]);
   if (rc==EOF || rc!=3) goto dev_error;  
   rc = fscanf(fp, "MG: %f, %f, %f\n", &deviation_MA[0], &deviation_MA[1], &deviation_MA[2]);
   if (rc==EOF || rc!=3) goto dev_error;   
   fclose(fp);   
   */
   return;
   
cal_error:
   fclose(fp);
   err_AL[0] = err_AL[1] = err_AL[2] = 0;
   err_GY[0] = err_GY[1] = err_GY[2] = 0; 
   err_MA[0] = err_MA[1] = err_MA[2] = 0; 
   scale_MA[0] = scale_MA[1] = scale_MA[2] = 1.0;       
   ERR(, "Cannot read data from calibration file %s", cal_file);
/*   
dev_error:
   fclose(fp);
   deviation_AL[0] = deviation_AL[1] = deviation_AL[2] = 0.0;
   deviation_GY[0] = deviation_GY[1] = deviation_GY[2] = 0.0; 
   deviation_MA[0] = deviation_MA[1] = deviation_MA[2] = 0.0;      
   ERR(, "Cannot read data from deviation file %s", dev_file);  
   */
}
 



/* Read and discard several samples of accel/gyro data, useful when initializing */
static void read_discard_samples(int samples)
{
uint8_t buf[12]; 
esp_err_t rc;
int delay;
static const uint8_t reg = 0x18;
  
   for (int i=0; i<samples; i++) {
      delay = lroundf(1E3/odr_ag_modes[ODR_AG]);  // Time to wait for new data to arrive, in ms
      if (delay > portTICK_PERIOD_MS) vTaskDelay(pdMS_TO_TICKS(delay));
      else ets_delay_us(delay*1000); 
      
      rc = i2c_master_write_read_device(I2C_BUS, accelAddr, &reg, 1, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (rc < 0) goto rw_error;   
   }
   return;

rw_error:
   ERR(, "Cannot read/write data from IMU");   
}



// Send a single byte value to a register in the IMU
static int i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val)
{
esp_err_t rc;
uint8_t buf[2];
    
    buf[0] = reg;
    buf[1] = val;
    rc = i2c_master_write_to_device(I2C_BUS, addr, buf, sizeof(buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (rc != ESP_OK) ERR(-1, "Error writing to IMU"); 
    return 0;
}


// Send a single byte value to a register in the IMU and read a number of bytes as response
static int i2cWriteReadBytes(uint8_t addr, uint8_t reg, uint8_t *read_buf, size_t read_size)
{
esp_err_t rc;
          
    rc = i2c_master_write_read_device(I2C_BUS, addr, &reg, 1, read_buf, read_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (rc != ESP_OK) ERR(-1, "Error reading from IMU"); 
    return 0;
}



/************************************************************
Initialize IMU LSM9DS1
Input: I2C address of accel/gyro and of magnetometer
Output: Status (return value) and delay in ms to read the IMU (as parameter)
************************************************************/
int setupLSM9DS1(uint8_t accel_addr, uint8_t mag_addr)
{
int rc;
uint8_t byte;

   /***** Initial checks *****/
   assert(odr_ag_modes[ODR_AG] > odr_m_modes[ODR_M]);
   upsampling_factor = lroundf(odr_ag_modes[ODR_AG] / odr_m_modes[ODR_M]);
   
   // Check acelerometer/gyroscope   
   rc = i2cWriteReadBytes(accel_addr, 0x0F, &byte, 1);  // Read WHO_AM_I register
   if (rc != ESP_OK) goto rw_error;
   if (byte != 0x68) {
      //closeLSM9DS1();
      ERR(-1, "Invalid accelerometer/gyroscope device");
   }
   accelAddr = accel_addr;
   
   // Check magnetometer  
   rc = i2cWriteReadBytes(mag_addr, 0x0F, &byte, 1);  // Read WHO_AM_I register
   if (rc < 0) goto rw_error;
   if (byte != 0x3D) {
      //closeLSM9DS1();
      ERR(-1, "Invalid magnetometer device");  
   }
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
   if (rc < 0) goto rw_error;   
   mRes = 4.0/32768;   // G/LSB
   
   // Activate magnetometer, CTRL_REG3_M
   // I2C: enabled (b0), 0 (b0), LP: No (b0), 0 (b0), 0 (b0), SPI: wo (b0), mode: Continuous (b00)
   byte = 0x0; 
   rc = i2cWriteByte(magAddr, 0x22, byte);
   if (rc < 0) goto rw_error; 
      
   // Set magnetometer, CTRL_REG4_M
   // OMZ: ultra (b11), BLE: data LSb at lower address (b0)
   byte = 0x03<<2; 
   rc = i2cWriteByte(magAddr, 0x23, byte);
   if (rc < 0) goto rw_error;
   
   // Set magnetometer, CTRL_REG5_M
   // BDU:  continuous update (b0)
   byte = 0x00; 
   rc = i2cWriteByte(magAddr, 0x24, byte);
   if (rc < 0) goto rw_error;
   
   /************************* Set Acelerometer / Gyroscope ***********************/   
   // Set IMU, CTRL_REG8
   // BOOT: normal mode (b0), BDU: continuous update (b0), H_LACTIVE: active high (b0),
   // PP_OD: 0 (b0), SIM: 0 (b0), IF_ADD_INC: enabled (b1), BLE: data LSB @ lower address (b0), SW_RESET: normal mode (b0)
   byte = 0x04; 
   rc = i2cWriteByte(accelAddr, 0x22, byte);
   if (rc < 0) goto rw_error; 
   
   // Set accelerometer, CTRL_REG5_XL
   // DEC: no decimation (b00), Zen_XL, Yen_XL, Xen_XL: enabled (b1)
   byte = (0x00<<6) + (0x07<<3); 
   rc = i2cWriteByte(accelAddr, 0x1F, byte);
   if (rc < 0) goto rw_error; 
 
   // Set accelerometer, CTRL_REG6_XL
   // ODR: Power down (b000), FS: 2g (b00), BW_SCAL: auto (b0), BW sel: 408 Hz (b00)
   byte = (0x00<<5) + (0x0<<3) + 0x0; 
   rc = i2cWriteByte(accelAddr, 0x20, byte);
   if (rc < 0) goto rw_error; 
   aRes = 2.0/32768;   // g/LSB

   // Set accelerometer, CTRL_REG7_XL
   // HR: enabled (b1), DCF: ODR/9 (b10), FDS: internal filter bypassed (b0), HPIS1: filter bypassed (b0)
   // both LPF enabled, HPF disabled
   byte = (0x01<<7) + (0x02<<5) + 0x0; 
   rc = i2cWriteByte(accelAddr, 0x21, byte);
   if (rc < 0) goto rw_error; 
   
   // Set gyro, CTRL_REG4
   // Zen_G, Yen_G, Xen_G: enabled (b1), LIR_XL1: 0 (b0), 4D_XL1: 0 (b0)
   byte = 0x07<<3; 
   rc = i2cWriteByte(accelAddr, 0x1E, byte);
   if (rc < 0) goto rw_error; 
   
   // Set gyro, ORIENT_CFG_G
   // SignX_G, SignX_G, SignX_G: positive sign (b0), Orient: 0 (b000) (Pitch:X, Roll:Y, Yaw:Z)
   byte = 0x00; 
   rc = i2cWriteByte(accelAddr, 0x13, byte);
   if (rc < 0) goto rw_error; 
   
   // Activate accelerometer and gyro, CTRL_REG1_G. Both use the same ODR
   // ODR: xxx Hz LPF1 31 Hz (b011), Gyro scale: 245 dps (b00), 0 (b0), Gyro BW LPF2: 31 Hz (b01)
   // The ODR is variable, acc. ODR_AG, LPF2 is set to be always around 30 Hz (b01) (for ODR of 119 Hz and above)
   byte = (ODR_AG<<5) + (0x0<<3) + 0x01; 
   rc = i2cWriteByte(accelAddr, 0x10, byte);
   if (rc < 0) goto rw_error; 
   gRes = 245.0/32768;   // dps/LSB
   
   // Set gyro, CTRL_REG2_G
   // INT_SEL: 0 (b00), OUT_SEL: 0 (b10) (output after LPF2)
   byte = 0x02; 
   rc = i2cWriteByte(accelAddr, 0x11, byte);
   if (rc < 0) goto rw_error; 
   
   // Set gyro, CTRL_REG3_G
   // LP_mode: disabled (b0), HP_EN: disabled (b0), HPCF_G: 0.5 Hz for 119 Hz ODR (b0100) 
   // both LPF enabled, HPF disabled
   byte = (0x0<<7) + (0x0<<6) + 0x04; 
   rc = i2cWriteByte(accelAddr, 0x12, byte);
   if (rc < 0) goto rw_error;  

   /* Empty and reset FIFO, in case it was active, so we start afresh */
   rc = i2cWriteByte(accelAddr, 0x2E, 0x00);  // Reset existing FIFO content by selecting FIFO Bypass mode 
   if (rc < 0) goto rw_error;  
 
   /* Initial samples after FIFO change and after power up (Table 12 of user manual) have to be discarded */
   read_discard_samples(20);   
   
   /************************** Handle calibration ********************/
   /*
   if (calibrate) {
      calibrate_accel_gyro();
      calibrate_magnetometer();
      write_calibration_data();
      gpioSleep(PI_TIME_RELATIVE, 1, 0);  // Sleep 1 second, so the user can continue the start process
   }
   else read_calibration_data();   
   */
   read_calibration_data(); 
   
   /************** Continue with accelerometer setting, activate FIFO ****************/
   // Set FIFO, FIFO_CTRL
   // FMODE: continuous mode (b110), threshold: 0 (b00000)
   byte = (0x06<<5) + 0x0; 
   rc = i2cWriteByte(accelAddr, 0x2E, byte);
   if (rc < 0) goto rw_error; 

   // Activate FIFO, CTRL_REG9
   // SLEEP_G: disabled (b0), FIFO_TEMP_EN: no (b0), 
   // DRDY_mask_bit: disabled (b0), I2C_DISABLE: both (b0), FIFO_EN: yes (b1), STOP_ON_FTH: no (b0)
   byte = 0x02; 
   rc = i2cWriteByte(accelAddr, 0x23, byte);
   if (rc < 0) goto rw_error;  
   
   read_discard_samples(2);  // Discard samples after FIFO activation
   
   /************************* Final actions ***********************/   
   /*
   // Initialize low pass filter for interpolating magnetometer data
   rc = LPFilter_init(&filter_mx, LP_20_240_filter_taps, sizeof(LP_20_240_filter_taps)/sizeof(float));
   if (rc < 0) goto init_error;  
   LPFilter_init(&filter_my, LP_20_240_filter_taps, sizeof(LP_20_240_filter_taps)/sizeof(float));
   if (rc < 0) goto init_error;    
   LPFilter_init(&filter_mz, LP_20_240_filter_taps, sizeof(LP_20_240_filter_taps)/sizeof(float));
   if (rc < 0) goto init_error; 
   // Set gain to upsampling_factor, to compensate for DC gain reduction due to interpolation
   LPFilter_setDCgain(&filter_mx, upsampling_factor);  // No need for filter_my and filter_mz, as they share the LP_20_240_filter_taps
   
   // Initialize low pass filter for accelerometer data
   rc = LPFilter_init(&filter_ax, LP_10_240_filter_taps, sizeof(LP_10_240_filter_taps)/sizeof(float));
   if (rc < 0) goto init_error;  
   LPFilter_init(&filter_ay, LP_10_240_filter_taps, sizeof(LP_10_240_filter_taps)/sizeof(float));
   if (rc < 0) goto init_error;    
   LPFilter_init(&filter_az, LP_10_240_filter_taps, sizeof(LP_10_240_filter_taps)/sizeof(float));
   if (rc < 0) goto init_error; 
   LPFilter_setDCgain(&filter_ax, 1.0);  // No need for filter_ay and filter_az, as they share the LP_10_240_filter_taps  

   // Initialize high pass filter for 1st order derivation
   rc = LPFilter_init(&filter_d1_ax, HP_1st_deriv_filter_taps, sizeof(HP_1st_deriv_filter_taps)/sizeof(float));
   if (rc < 0) goto init_error;    
   */
   
   // Start the IMU reading timer
   uint32_t delay_ms = lroundf(1000.0/odr_m_modes[ODR_M]*1.2);  // Read IMU with a period of magnetometer ODR, add 20% margin
   esp_timer_handle_t timer;
   const esp_timer_create_args_t timer_args = {
            .dispatch_method = ESP_TIMER_TASK,
            .callback = &IMU_read,
            .arg = (void*)delay_ms,
            .name = "IMU_read"
            };

   ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
   ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000*delay_ms));
      
   return 0;
   
   /* error handling if read operation from I2C bus failed */
rw_error:
   //closeLSM9DS1();
   ERR(-1, "Cannot read/write data from IMU");

   /* error handling if initialization failed */   
//init_error:
   //closeLSM9DS1();
   //ERR(-1, "Error initializing the IMU");
}


/***
Read data from magnetometer. Return values for each axis (in Gauss) in the 3 passed pointers to float.
Return value: -1 if error in reading the I2C bus, 0 if no data available yet, 1 if data available and successfully read
***/
static int read_magnetometer(float *mxr, float *myr, float *mzr)
{
int16_t mx, my, mz; // x, y, and z axis raw readings of the magnetometer
uint8_t byte, buf[6];
int rc;
   
   // Read status register in magnetometer, to check if new data is available
   rc = i2cWriteReadBytes(magAddr, 0x27, &byte, 1);  // Read magnetometer STATUS_REG register
   if (rc < 0) return -1;   // Read error, error message already sent
   if ((byte&0x08) == 0) return 0; // If no new data available, return 0

   /* New magnetometer data in XYZ is available
      Read magnetometer data. X and Y axis are exchanged, and then Y is negated, 
      so align the axis with tha ones used in this module for car orientation */
         
   rc = i2cWriteReadBytes(magAddr, 0x28, buf, sizeof(buf));  // Read 6 bytes (X,Y,Z axis), starting in OUT_X_L_M register
   if (rc < 0) return -1;   // Read error, error message already sent
   my = buf[1]<<8 | buf[0]; mx = buf[3]<<8 | buf[2]; mz = buf[5]<<8 | buf[4];  
   my *= -1;     
   
   /* Substract measured error values (hardiron effects) */
   mx -= err_MA[0]; my -= err_MA[1]; mz -= err_MA[2];      
   /* Compensate for softiron and scale the result */
   *mxr = mx*scale_MA[0]*mRes; *myr = my*scale_MA[1]*mRes; *mzr = mz*scale_MA[2]*mRes;      
   
   return 1;  // Data available and successfully read, return 1
}




/* 
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
*/
static void IMU_read(void* arg)
{
static int64_t previous_tick;
int64_t start_tick;
uint32_t delay_should = (uint32_t)arg;

int n, rc;   
uint8_t samples, byte;
static uint8_t buf[12*32];  // static, so it does not grow the stack
char str[17];
static unsigned int samples_count, count, collision_sample;
//static bool in_collision;

/* These values are the raw signed 16-bit readings from the sensors */
int16_t gx, gy, gz; // x, y, and z axis raw readings of the gyroscope
int16_t ax, ay, az; // x, y, and z axis raw readings of the accelerometer

/* Real (scaled and compensated) readings of the sensors */
float axr, ayr, azr;
float gxr, gyr, gzr;
float mxr, myr, mzr; 
//float axrf, ayrf, azrf; // values after LPF
//float mxrf, myrf, mzrf; // values after LPF
//float daxr;

   start_tick = esp_timer_get_time();
   if (previous_tick == 0) previous_tick = start_tick;
   
   //printf("Delay since last IMU read: %llu ms\n", (start_tick-previous_tick)/1000);  
   
   /** Read magnetometer data, if ready **/
   rc = read_magnetometer(&mxr, &myr, &mzr);
   if (rc < 0) goto rw_error; 
   if (rc == 0) {
      printf("No magnetometer data\n");
      return;  // No magnetometer data yet
   }
   if (!(count++%10)) printHeading(mxr, myr);
   // snprintf(str, sizeof(str), "M:%-4.0f mG", 1000*sqrt(mxr*mxr+myr*myr+mzr*mzr));  
   // oledWriteString(0, 6, str, false);  
   
   /**
   Now, read accel/gyro data. Read data from FIFO, as ODR of accel/gyro is higher
   than that of magnetometer. The accel/gyro stores samples in the FIFO until it is read.
   **/

   rc = i2cWriteReadBytes(accelAddr, 0x2F, &byte, 1);  // Read FIFO_SRC register
   if (rc < 0) goto rw_error;
   if (byte & 0x40) ESP_LOGW(TAG, "%s in %s: FIFO overrun!", __func__, __FILE__);  // Should not happen
   // samples in FIFO are between 3 and 4, average 3: AG ODR = 119 Hz, M ODR = 40 Hz, 119/40=3
   // if AG ODR = 238, samples is between 6 and 8
   samples = byte & 0x3F;   // samples in FIFO could be zero  
   //printf("Samples: %d\n", samples);
   if (samples < upsampling_factor) {  // Should not happen. Emit a warning message.
      ESP_LOGW(TAG, "%s: Timing problem: number of samples of accelerometer (%d) is less than upsampling factor (%d)", 
            __func__, samples, upsampling_factor);    
   }    
   
   if (samples) {  // if FIFO has sth, read it
      /* Burst read. Accelerometer and gyroscope sensors are activated at the same ODR.
         So read all FIFO lines (12 bytes each) in a single I2C transaction */
      rc = i2cWriteReadBytes(accelAddr, 0x18, buf, 12*samples);    
      if (rc < 0) goto rw_error;         
   }


   
   previous_tick = start_tick;
   return;
   
/* error handling if read operation from I2C bus failed */
rw_error:
   //closeLSM9DS1();
   ERR(, "Cannot read/write data from IMU");
}



