
/**
 * MODIFIED BY CHARLES ZALOOM - 8/3/18
 * IMPLEMENTATION OF EMBEDDEDML IN LEARNING DEVICE ORIENTATION
 **/

/**
 ******************************************************************************
 * @file    DataLog/Src/main.c
 * @author  Central Labs
 * @version V1.1.1
 * @date    06-Dec-2016
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */
#include <stdarg.h> // for print
#include "embeddedML.h"
#include "main.h"

#include "datalog_application.h"
#include "usbd_cdc_interface.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Data acquisition period [ms] */
#define DATA_PERIOD_MS (10)

#define NUMBER_TEST_CYCLES 10
#define CLASSIFICATION_ACC_THRESHOLD 1
#define CLASSIFICATION_DISC_THRESHOLD 1.05
#define Z_ACCEL_THRESHOLD 300
#define START_POSITION_INTERVAL 3000
#define TRAINING_CYCLES 2000
#define LED_BLINK_INTERVAL 200

// Change for Angle Threshold
#define ANGLE_MAG_MAX_THRESHOLD 90
#define MAX_ROTATION_ACQUIRE_CYCLES 400

//#define NOT_DEBUGGING

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* SendOverUSB = 0  --> Save sensors data on SDCard (enable with double click) */
/* SendOverUSB = 1  --> Send sensors data via USB */
uint8_t SendOverUSB = 1;

USBD_HandleTypeDef USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
static volatile uint8_t no_H_HTS221 = 0;
static volatile uint8_t no_T_HTS221 = 0;
static volatile uint8_t no_GG = 0;

static RTC_HandleTypeDef RtcHandle;
static void *LSM6DSM_X_0_handle = NULL;
static void *LSM6DSM_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *LPS22HB_P_0_handle = NULL;
static void *LPS22HB_T_0_handle = NULL;
static void *HTS221_H_0_handle = NULL;
static void *HTS221_T_0_handle = NULL;
static void *GG_handle = NULL;

/* Private function prototypes -----------------------------------------------*/

static void Error_Handler(void);
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void initializeAllSensors(void);

/* Private functions ---------------------------------------------------------*/

static volatile uint8_t hasTrained = 0;
unsigned int training_cycles = TRAINING_CYCLES;

void ezprint(char *format[], ...) {
	char buffer[128];
	va_list args;
	va_start(args, format);

	vsprintf(buffer, format, args);
	va_end(args);

	CDC_Fill_Buffer((uint8_t *) buffer, strlen(buffer));
}

void stable_softmax(float *x, float *y) {
	int size = 3;
	float multiplier = 1.0;

	int i;

	//softmax implemented as square law algorithm to be accommodate numerical precision requirements

	y[0] = (x[0] * x[0] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));
	y[1] = (x[1] * x[1] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));
	y[2] = (x[2] * x[2] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));

	for (i = 0; i < size; i++) {
		if (x[i] < 0.0)
			y[i] = y[i] * -1.0;
	}
}

void motion_softmax(int size, float *x, float *y) {
	float norm1, norm2;

	norm1 = sqrt((x[0] * x[0]) + (x[1] * x[1]) + (x[2] * x[2]));
	y[0] = abs(x[0]) / norm1;
	y[1] = abs(x[1]) / norm1;
	y[2] = abs(x[2]) / norm1;

	norm2 = sqrt((x[3] * x[3]) + (x[4] * x[4]) + (x[5] * x[5]));
	y[3] = abs(x[3]) / norm2;
	y[4] = abs(x[4]) / norm2;
	y[5] = abs(x[5]) / norm2;

	int i;
	for (i = 0; i < size; i++) {
		if (x[i] < 0.0)
			y[i] = y[i] * -1.0;
	}
}

void LED_Code_Blink(int count) {

	int i;

	/*
	 * Alert signal of rapid blinks indicating code to be shown
	 */
	for (i = 0; i < 7; i++) {
		BSP_LED_On(LED1);
		HAL_Delay(20);
		BSP_LED_Off(LED1);
		HAL_Delay(50);
	}

	/*
	 * Code indicated by number of slow blinks
	 */

	if (count != 0) {
		HAL_Delay(1000);
		for (i = 0; i < count; i++) {
			BSP_LED_On(LED1);
			HAL_Delay(500);
			BSP_LED_Off(LED1);
			HAL_Delay(500);
		}
	}

	/*
	 * Alert signal of rapid blinks indicating end of code
	 */
	for (i = 0; i < 7; i++) {
		BSP_LED_On(LED1);
		HAL_Delay(20);
		BSP_LED_Off(LED1);
		HAL_Delay(30);
	}

}

void LED_Notification_Blink(int count) {

	int i;

	/*
	 * Rapid blink notification
	 */

	for (i = 0; i < count; i++) {
		BSP_LED_On(LED1);
		HAL_Delay(20);
		BSP_LED_Off(LED1);
		HAL_Delay(50);
	}
}

void getAccel(void *handle, int *xyz) {
	uint8_t status, id;
	SensorAxes_t acceleration;

	BSP_ACCELERO_Get_Instance(handle, &id);

	BSP_ACCELERO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR) {
			acceleration.AXIS_X = 0;
			acceleration.AXIS_Y = 0;
			acceleration.AXIS_Z = 0;
		}

		xyz[0] = (int) acceleration.AXIS_X;
		xyz[1] = (int) acceleration.AXIS_Y;
		xyz[2] = (int) acceleration.AXIS_Z;
	}
}

void getAngularVelocity(void *handle_g, int *xyz) {
	uint8_t id, status;

	SensorAxes_t angular_velocity;
	BSP_GYRO_Get_Instance(handle_g, &id);
	BSP_GYRO_IsInitialized(handle_g, &status);
	if (status == 1) {
	if (BSP_GYRO_Get_Axes(handle_g, &angular_velocity) == COMPONENT_ERROR) {
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}
		xyz[0] = (int) angular_velocity.AXIS_X;
		xyz[1] = (int) angular_velocity.AXIS_Y;
		xyz[2] = (int) angular_velocity.AXIS_Z;
	}
}

/*
 * Note : Feature_Extraction_State_0() sets Z-axis acceleration, ttt_3 = 0
 */

void Feature_Extraction_State_0(void *handle,
								int *features[]) {

	int ttt_initial[3]; // Base State Acceleration
	int ttt[3]; // State Acceleration
	int i; // Indexing Number
	int axis_index;
	float accel_mag;
	char msg[128];

	// Acquire acceleration values before motion
	getAccel(handle, ttt_initial);

	// Prompt First Motion
	sprintf(msg, "\r\nStart First Motion when LED On");
	CDC_Fill_Buffer((uint8_t *) msg, strlen(msg));
	BSP_LED_On(LED1);

	HAL_Delay(2000);

	sprintf(msg, "\r\nEnd Motion");
	CDC_Fill_Buffer((uint8_t *) msg, strlen(msg));
	HAL_Delay(1000);

	// Acquire acceleration values after motion
	getAccel(handle, ttt);

	// Compute Magnitude of Acceleration
	for (axis_index = 0; axis_index < 3; axis_index++) {
		accel_mag = pow((ttt[axis_index] - ttt_initial[axis_index]), 2);
	}

	accel_mag = sqrt(accel_mag);
	//*ttt_mag_scale = (int)(accel_mag);

	for (i = 0; i < 3; i++) {
		*(features + i) = ttt[i] - ttt_initial[i];
	}

	BSP_LED_Off(LED1);
	return;
}

/*
 * Feature_Extraction_State_1() determines a second orientation after
 * the action of Feature_Extraction_State_0().
 */
void Feature_Extraction_State_1(void *handle_g, int *features[]) {
	int ttt[3], ttt_initial[3], ttt_offset[3];
	char msg1[128];
	int axis_index, sample_index, i; // Indexing Numbers
	float rotate_angle[3];
	float angle_mag;
	float Tsample;
	/*
	* Compute sample period with scaling from milliseconds
	* to seconds
	*/
	Tsample = (float)(DATA_PERIOD_MS)/1000;
	/*
	* Initialize rotation angle values
	*/
	for (axis_index = 0; axis_index < 3; axis_index++) {
		ttt[axis_index] = 0;
		rotate_angle[axis_index] = 0;
	}
	/*
	* Rotation Rate Signal integration loop
	*
	* Note that loop cycle time is DATA_PERIOD_MS matching the SensorTile
	* sensor sampling period
	*
	* Permit integration loop to operate no longer than a maximum
	* number of cycles, MAX_ROTATION_ACQUIRE_CYCLES. Note: This sets
	* maximum acquisition time to be MAX_ROTATION_ACQUIRE_CYCLES*Tsample
	*
	* For 4 second delay, apply MAX_ROTATION_ACQUIRE_CYCLES 400
	*
	*/
	/*
	* Acquire Rotation Rate values prior to motion
	*
	* This includes the initial sensor offset value to be subtracted
	* from all subsequent samples
	*/
	getAngularVelocity(handle_g, ttt_offset);
	/*
	* Notify user to initiate motion
	*/
	sprintf(msg1, "\r\nStart Second Motion when LED On");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
	BSP_LED_On(LED1);
	for (sample_index = 0; sample_index < MAX_ROTATION_ACQUIRE_CYCLES; sample_index++) {
		/*
		* Acquire initial sample value of rotation rate and remove
		* offset value
		*/
		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt_initial[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}
		/*
		* Introduce integration time period delay
		*/
		HAL_Delay(DATA_PERIOD_MS);
		/*
		* Acquire current sample value of rotation rate and remove
		* offset value
		*/
		getAngularVelocity(handle_g, ttt);
		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}

		// Compute rotation angles by integration
		for (axis_index = 0; axis_index < 3; axis_index++) {
			rotate_angle[axis_index] = rotate_angle[axis_index] + (float)((ttt_initial[axis_index] + ttt[axis_index]) * Tsample / 2);
		}
		/*
		*
		* Compute magnitude of rotational angle summing over X and Y
		* axis Rotation Rates.
		*
		* Convert from milli-degrees to degrees (Note that Rotation
		* Rate is sampled in milli-degrees per second).
		*
		*/
		angle_mag = 0;
		for (axis_index = 0; axis_index < 3; axis_index++) {
			angle_mag = angle_mag + pow((rotate_angle[axis_index]), 2);
		}
		/*
		* Compute angle magnitude and convert from milli-degrees to
		* degrees
		*/
		angle_mag = sqrt(angle_mag)/1000;
		 /*
		* Compute rotation angle magnitude
		*/
		if (angle_mag >= ANGLE_MAG_MAX_THRESHOLD) {
			break;
		}

		for (i = 0; i < 3; i++) {
			*(features + i + 3) = (int) rotate_angle[i];
		}
	}

	sprintf(msg1, "\r\n\r\nMotion with Angle Mag of %i degrees complete.\nNow Return to Next Start Position, ",
			(int) angle_mag);
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

	BSP_LED_Off(LED1);
	HAL_Delay(3000);
	return;
}

void printOutput_ANN(ANN *net, int input_state, int * error) {

	char dataOut[256] = { };
	int i, loc, count;
	float point = 0.0;
	float rms_output, mean_output, mean_output_rem, next_max;
	float classification_metric;

	/*
	 * Initialize error state
	 */

	*error = 0;

	count = 0;
	mean_output = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		sprintf(dataOut, "Output %f", net->output[i]);
		CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
		mean_output = mean_output + (net->output[i]);
		if (net->output[i] > point && net->output[i] > 0.1) {
			point = net->output[i];
			loc = i;
		}
		count++;
	}

	next_max = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		if (i == loc) {
			continue;
		}
		if (net->output[i] > next_max && net->output[i] > 0.1) {
			next_max = net->output[i];
		}
	}

	mean_output = (mean_output) / (count);

	count = 0;
	mean_output_rem = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		mean_output_rem = mean_output_rem + (net->output[i]);
		if (i == loc) {
			continue;
		}
		count++;
	}

	mean_output_rem = (mean_output_rem) / (count);

	rms_output = 0;

	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		rms_output = rms_output + pow((net->output[i] - mean_output), 2);
	}

	rms_output = sqrt(rms_output / count);
	sprintf(dataOut, "RMS_OUTPUT: %d", rms_output);
	CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
	if (rms_output != 0) {
		classification_metric = (point - mean_output) / rms_output;
	} else {
		classification_metric = 0;
	}

	if (loc != input_state) {
		rms_output = 0;
		classification_metric = 0;
		point = 0;
		mean_output = 0;
		mean_output_rem = 0;
	}

	sprintf(dataOut, "\r\nState %i\tMax %i\tMean %i\t\tZ-score %i\tOutputs",
			loc, (int) (100 * point), (int) (100 * mean_output),
			(int) (100 * classification_metric));
	CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));

	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		sprintf(dataOut, "\t%i", (int) (100 * net->output[i]));
		CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
	}

	if (loc != input_state) {
		*error = 1;
		sprintf(dataOut, "\t Classification Error");
		CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
	}

	if ((loc == input_state)
			&& ((classification_metric < CLASSIFICATION_ACC_THRESHOLD)
					|| ((point / next_max) < CLASSIFICATION_DISC_THRESHOLD))) {
		*error = 1;
		sprintf(dataOut, "\t Classification Accuracy Limit");
		CDC_Fill_Buffer((uint8_t *) dataOut, strlen(dataOut));
	}

}

void TrainOrientation(void *handle, void *handle_g, ANN *net) {

	uint8_t id, id_g;
	SensorAxes_t acceleration, angular_velocity;
	uint8_t status, status_g;
	float training_dataset[6][8][6];
	float XYZ[6];
	float xyz[6];
	char msg1[256];
	int num_train_data_cycles;
	int i, j, k, m, n, r;
	int error, net_error;

	int features[6];

	int ttt_1, ttt_2, ttt_3, ttt_mag_scale; // First three features and their total magnitude
	int rrr_1, rrr_2, rrr_3, rrr_mag_scale; // Second three features and their total magnitude

	BSP_ACCELERO_Get_Instance(handle, &id);
	BSP_ACCELERO_IsInitialized(handle, &status);

	BSP_GYRO_Get_Instance(handle, &id_g);
	BSP_GYRO_IsInitialized(handle, &status_g);

	if (status == 1 && status_g == 1) {

		// Defaults for Component Errors
		if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR) {
			acceleration.AXIS_X = 0;
			acceleration.AXIS_Y = 0;
			acceleration.AXIS_Z = 0;
		}
		if (BSP_GYRO_Get_Axes(handle_g, &angular_velocity) == COMPONENT_ERROR) {
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}

		// Training Start

		sprintf(msg1, "\r\n\r\n\r\nTraining Start in 2 seconds ..");
		CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
		BSP_LED_Off(LED1);
		HAL_Delay(2000);

		/*
		 * Maximum of 8 cycles
		 */
		num_train_data_cycles = 1;

		for (k = 0; k < num_train_data_cycles; k++) {
			for (i = 0; i < 6; i++) {

				sprintf(msg1, "\r\nMove to Start Position - Wait for LED On");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				HAL_Delay(START_POSITION_INTERVAL);

				sprintf(msg1, "\r\nMove to Orientation %i on LED On", i+1);
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				Feature_Extraction_State_0(handle, &features);
				Feature_Extraction_State_1(handle_g, &features);

				sprintf(msg1, "\r\nAccel %i\t\%i\%i", features[0], features[1], features[2]);
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

				for (j = 0; j < 6; j++) {
					XYZ[j] = (float) features[j];
				}

				motion_softmax(net->topology[0], XYZ, xyz);

				for (j = 0; j < 6; j++) {
					training_dataset[i][k][j] = xyz[j];
				}

				int o;
				for (o = 0; o < 6; o++) {
					sprintf(msg1, "Dataset #%i, Val: %f", o, training_dataset[i][k][o]);
					CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				}

				sprintf(msg1, "\r\n Softmax Input \t");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				for (r = 0; r < 6; r++) {
					sprintf(msg1, "\t%i", (int) XYZ[r]);
					CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				}
				sprintf(msg1, "\r\n Softmax Output\t");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				for (r = 0; r < 6; r++) {
					sprintf(msg1, "\t%i", (int) (100 * xyz[r]));
					CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
				}
				sprintf(msg1, "\r\n\r\n");
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			}
		}

		/*
		 * Enter NN training
		 */

		float _Motion_1[6] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		float _Motion_2[6] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 };
		float _Motion_3[6] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
		float _Motion_4[6] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 };
		float _Motion_5[6] = { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
		float _Motion_6[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

		sprintf(msg1, "\r\n\r\nTraining Start\r\n");
		CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

		for (k = 0; k < num_train_data_cycles; k++) {

			i = 0;
			while (i < training_cycles) {
				for (j = 0; j < 6; j++) {


					if ((i % 20 == 0 && i < 100) || i % 100 == 0) {
						char print_train_time[128];
						sprintf(print_train_time,
								"\r\n\r\nTraining Epochs: %d\r\n", i);
						CDC_Fill_Buffer((uint8_t *) print_train_time,
								strlen(print_train_time));

						LED_Code_Blink(0);

						net_error = 0;
						for (m = 0; m < 6; m++) {
							run_ann(net, training_dataset[m][k]);
							printOutput_ANN(net, m, &error);
							if (error == 1) {
								net_error = 1;
							}
						}
						sprintf(msg1, "\r\nError State: %i\r\n",
								net_error);
						CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
						if (net_error == 0) {
							return;
						}

					}

					switch (j) {

					case 0:
						train_ann(net, training_dataset[j][k], _Motion_1);
						break;
					case 1:
						train_ann(net, training_dataset[j][k], _Motion_2);
						break;
					case 2:
						train_ann(net, training_dataset[j][k], _Motion_3);
						break;
					case 3:
						train_ann(net, training_dataset[j][k], _Motion_4);
						break;
					case 4:
						train_ann(net, training_dataset[j][k], _Motion_5);
						break;
					case 5:
						train_ann(net, training_dataset[j][k], _Motion_6);
						break;
					default:
						break;
					}
					i++;
					HAL_Delay(5);
				}

			}

		}
	}

	if (SendOverUSB) /* Write data on the USB */
	{
		//sprintf( dataOut, "\n\rAX: %d, AY: %d, AZ: %d", (int)acceleration.AXIS_X, (int)acceleration.AXIS_Y, (int)acceleration.AXIS_Z );
		//CDC_Fill_Buffer(( uint8_t * )dataOut, strlen( dataOut ));
	}

	if (net_error == 0){
		LED_Code_Blink(0);
		LED_Code_Blink(0);
	} else {
		LED_Code_Blink(1);
		LED_Code_Blink(1);
	}

	sprintf(msg1, "\r\n\r\nTraining Complete, Now Start Test Motions\r\n");
	CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
	return;
}

int Accel_Gyro_Sensor_Handler(void *handle, void *handle_g, ANN *net, int prev_loc) {
	uint8_t id, id_g;
	SensorAxes_t acceleration, angular_velocity;
	uint8_t status, status_g;
	float xyz[6];
	float XYZ[6];
	float point;
	int i, j, k, loc;
	int features[6];
	char msg1[128];

	BSP_ACCELERO_Get_Instance(handle, &id);
	BSP_ACCELERO_IsInitialized(handle, &status);

	BSP_GYRO_Get_Instance(handle_g, &id_g);
	BSP_GYRO_IsInitialized(handle_g, &status_g);

	if (status == 1) {
		if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR) {
			acceleration.AXIS_X = 0;
			acceleration.AXIS_Y = 0;
			acceleration.AXIS_Z = 0;
		}

		/*
		 * Perform limited number of NN execution and prediction cycles.
		 * Upon return, training will be repeased
		 */

		k = 0;

		while (k < NUMBER_TEST_CYCLES) {

			BSP_LED_Off(LED1);

			sprintf(msg1, "\n\rMove to Start Position - Wait for LED On");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			HAL_Delay(START_POSITION_INTERVAL);

			Feature_Extraction_State_0(handle, &features);

			Feature_Extraction_State_1(handle_g, &features);

			for (i = 0; i < 6; i++) {
				XYZ[i] = (float) features[i];
			}

			motion_softmax(net->topology[0], XYZ, xyz);

			sprintf(msg1, "\r\n Softmax Input: \t");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			for (j = 0; j < 6; j++) {
				sprintf(msg1, "%i\t", (int) XYZ[j]);
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			}
			sprintf(msg1, "\r\n Softmax Output: \t");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			for (j = 0; j < 6; j++) {
				sprintf(msg1, "%i\t", (int) (100 * xyz[j]));
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			}
			sprintf(msg1, "\r\n");

			run_ann(net, xyz);

			point = 0.0;
			loc = -1;

			for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
				if (net->output[i] > point && net->output[i] > 0.1) {
					point = net->output[i];
					loc = i;
				}
			}

			if (loc == -1) {
				LED_Code_Blink(0);
			} else {
				LED_Code_Blink(loc + 1);
			}

			sprintf(msg1, "\n\rNeural Network Classification - Motion %i", loc + 1);
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));

		k = k + 1;
		}
	}
	return prev_loc;
}


int main(void) {
	uint32_t msTick, msTickPrev = 0;
	uint8_t doubleTap = 0;
	char msg2[128];
	int i;

	/* STM32L4xx HAL library initialization:
	 - Configure the Flash prefetch, instruction and Data caches
	 - Configure the Systick to generate an interrupt each 1 msec
	 - Set NVIC Group Priority to 4
	 - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	if (SendOverUSB) {
		/* Initialize LED */
		BSP_LED_Init(LED1);
	}
#ifdef NOT_DEBUGGING
	else
	{
		/* Initialize LEDSWD: Cannot be used during debug because it overrides SWDCLK pin configuration */
		BSP_LED_Init(LEDSWD);
		BSP_LED_Off(LEDSWD);
	}
#endif

	/* Initialize RTC */
	RTC_Config();
	RTC_TimeStampConfig();

	/* enable USB power on Pwrctrl CR2 register */
	HAL_PWREx_EnableVddUSB();

	if (SendOverUSB) /* Configure the USB */
	{
		/*** USB CDC Configuration ***/
		/* Init Device Library */
		USBD_Init(&USBD_Device, &VCP_Desc, 0);
		/* Add Supported Class */
		USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
		/* Add Interface callbacks for AUDIO and CDC Class */
		USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
		/* Start Device Process */
		USBD_Start(&USBD_Device);
	} else /* Configure the SDCard */
	{
		DATALOG_SD_Init();
	}
	HAL_Delay(200);

	/* Configure and disable all the Chip Select pins */
	Sensor_IO_SPI_CS_Init_All();

	/* Initialize and Enable the available sensors */
	initializeAllSensors();
	enableAllSensors();

	/* Notify user */

	sprintf(msg2, "\n\rEmbeddedML Physical Therapy Two-Motion Exercise Classification\r\n");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));

	sprintf(msg2, "\n\rDOUBLE TAP to start recording motions");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));

	//---EMBEDDED ANN---
	float weights[108] = {0.982900, 0.478700, 0.926600, 0.947100, 0.939900,
	 0.126900, 0.812800, 0.532500, 0.415700, 0.694800,
	 0.785300, 0.685900, 0.763800, 0.324600, 0.117900,
	 0.978500, 0.437700, 0.179800, 0.182300, 0.266300,
	 0.742100, 0.736500, 0.533900, 0.173100, 0.726900,
	 0.560400, 0.657200, 0.712500, 0.662600, 0.847500,
	 0.226300, 0.316500, 0.910600, 0.783300, 0.857400,
	 0.808400, 0.176500, 0.967700, 0.246800, 0.598800,
	 0.655000, 0.569200, 0.319800, 0.526400, 0.805800,
	 0.815800, 0.149200, 0.295100, 0.321200, 0.461400,
	 0.464900, 0.707000, 0.633100, 0.137100, 0.462200,
	 0.673100, 0.773000, 0.646800, 0.849000, 0.358900,
	 0.229400, 0.284700, 0.778100, 0.950900, 0.527000,
	 0.533100, 0.006600, 0.354300, 0.983000, 0.125600,
	 0.140300, 0.385800, 0.604700, 0.123500, 0.300500,
	 0.918100, 0.721200, 0.198000, 0.804200, 0.306000,
	 0.393900, 0.372600, 0.512500, 0.546300, 0.188000,
	 0.662900, 0.938700, 0.609700, 0.459000, 0.078900,
	 0.857900, 0.020000, 0.605400, 0.784800, 0.740900,
	 0.397000, 0.428300, 0.975900, 0.127500, 0.397800,
	};
	float dedw[108];
	float bias[15];
	unsigned int network_topology[3] = { 6, 9, 6 };
	float output[6];

	ANN net;
	net.weights = weights;
	net.dedw = dedw;
	net.bias = bias;
	net.topology = network_topology;
	net.n_layers = 3;
	net.n_weights = 108;
	net.n_bias = 15;
	net.output = output;

	for (i = 0; i < 15; i++){
		bias[i] = 0.5;
	}
	for (i = 0; i < 6; i++){
		output[i] = 0.0;
	}
	for (i = 0; i < 108; i++){
		dedw[i] = 0.0;
	}

	//OPTIONS
	net.eta = 0.13;     //Learning Rate
	net.beta = 0.01;    //Bias Learning Rate
	net.alpha = 0.25;   //Momentum Coefficient
	net.output_activation_function = &relu2;
	net.hidden_activation_function = &relu2;

	init_ann(&net);
	//---------------------

	int loc = -1;
	while (1) {
		/* Get sysTick value and check if it's time to execute the task */
		msTick = HAL_GetTick();
		if (msTick % DATA_PERIOD_MS == 0 && msTickPrev != msTick) {
			msTickPrev = msTick;

			if (SendOverUSB) {
				BSP_LED_On(LED1);
			}

			//RTC_Handler( &RtcHandle );

			if (hasTrained){
				loc = Accel_Gyro_Sensor_Handler(LSM6DSM_X_0_handle, LSM6DSM_G_0_handle, &net, loc);
				/*
				 * Upon return from Accel_Gyro_Sensor_Handler, initiate retraining.
				 */
				hasTrained = 0;
				sprintf(msg2, "\n\r\n\DOUBLE TAP to start recording new Two-Motion Exercises");
				CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));
			}

			if (SendOverUSB) {
				BSP_LED_Off(LED1);
			}

		}

		/* Check LSM6DSM Double Tap Event  */
		if (!hasTrained) {
			BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(LSM6DSM_X_0_handle,
					&doubleTap);
			if (doubleTap) { /* Double Tap event */
				LED_Code_Blink(0);
				doubleTap = 0;
				TrainOrientation(LSM6DSM_X_0_handle, LSM6DSM_G_0_handle, &net);
				hasTrained = 1;
			}
		}

		/* Go to Sleep */
		__WFI();
	}
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void) {
	if (BSP_ACCELERO_Init(LSM6DSM_X_0, &LSM6DSM_X_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_GYRO_Init(LSM6DSM_G_0, &LSM6DSM_G_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_ACCELERO_Init(LSM303AGR_X_0, &LSM303AGR_X_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_MAGNETO_Init(LSM303AGR_M_0, &LSM303AGR_M_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_PRESSURE_Init(LPS22HB_P_0, &LPS22HB_P_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(LPS22HB_T_0, &LPS22HB_T_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(HTS221_T_0, &HTS221_T_0_handle)
			== COMPONENT_ERROR) {
		no_T_HTS221 = 1;
	}

	if (BSP_HUMIDITY_Init(HTS221_H_0, &HTS221_H_0_handle) == COMPONENT_ERROR) {
		no_H_HTS221 = 1;
	}

	/* Inialize the Gas Gauge if the battery is present */
	if (BSP_GG_Init(&GG_handle) == COMPONENT_ERROR) {
		no_GG = 1;
	}

	//if(!SendOverUSB)
	//{
	/* Enable HW Double Tap detection */
	BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSM_X_0_handle);
	BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSM_X_0_handle,
	LSM6DSM_TAP_THRESHOLD_MID);
	//}

}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableAllSensors(void) {
	BSP_ACCELERO_Sensor_Enable(LSM6DSM_X_0_handle);
	BSP_GYRO_Sensor_Enable(LSM6DSM_G_0_handle);
	BSP_ACCELERO_Sensor_Enable(LSM303AGR_X_0_handle);
	BSP_MAGNETO_Sensor_Enable(LSM303AGR_M_0_handle);
	BSP_PRESSURE_Sensor_Enable(LPS22HB_P_0_handle);
	BSP_TEMPERATURE_Sensor_Enable(LPS22HB_T_0_handle);
	if (!no_T_HTS221) {
		BSP_TEMPERATURE_Sensor_Enable(HTS221_T_0_handle);
		BSP_HUMIDITY_Sensor_Enable(HTS221_H_0_handle);
	}

}

/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
void disableAllSensors(void) {
	BSP_ACCELERO_Sensor_Disable(LSM6DSM_X_0_handle);
	BSP_ACCELERO_Sensor_Disable(LSM303AGR_X_0_handle);
	BSP_GYRO_Sensor_Disable(LSM6DSM_G_0_handle);
	BSP_MAGNETO_Sensor_Disable(LSM303AGR_M_0_handle);
	BSP_HUMIDITY_Sensor_Disable(HTS221_H_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(HTS221_T_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(LPS22HB_T_0_handle);
	BSP_PRESSURE_Sensor_Disable(LPS22HB_P_0_handle);
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void) {
	/*##-1- Configure the RTC peripheral #######################################*/
	RtcHandle.Instance = RTC;

	/* Configure RTC prescaler and RTC data registers */
	/* RTC configured as follow:
	 - Hour Format    = Format 12
	 - Asynch Prediv  = Value according to source clock
	 - Synch Prediv   = Value according to source clock
	 - OutPut         = Output Disable
	 - OutPutPolarity = High Polarity
	 - OutPutType     = Open Drain */
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
	RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
	RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK) {

		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void) {

	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;

	/*##-3- Configure the Date using BCD format ################################*/
	/* Set Date: Monday January 1st 2000 */
	sdatestructure.Year = 0x00;
	sdatestructure.Month = RTC_MONTH_JANUARY;
	sdatestructure.Date = 0x01;
	sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

	if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK) {

		/* Initialization Error */
		Error_Handler();
	}

	/*##-4- Configure the Time using BCD format#################################*/
	/* Set Time: 00:00:00 */
	stimestructure.Hours = 0x00;
	stimestructure.Minutes = 0x00;
	stimestructure.Seconds = 0x00;
	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss) {

	RTC_TimeTypeDef stimestructure;

	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.Hours = hh;
	stimestructure.Minutes = mm;
	stimestructure.Seconds = ss;
	stimestructure.SubSeconds = 0;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	MEMSInterrupt = 1;
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {

	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	while (1)
	{}
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


