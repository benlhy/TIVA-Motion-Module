/**
 * @file Control.h
 * @brief control header
 *
 * This file contains the pwm functions
 *
 * @author Benjamen Lim
 *
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define DECIMATION 10    // Decimate the data if necessary
#define REFERENCE_DATA 10000    // Reference data for trajectory tracking
#define BUFLEN 1024 // Actual data; sent to client using circular buffer

#include "motor.h"

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains;

// TODO in setup set these to zero
typedef struct {                          // Define data structure containing control data
    int Enew;
    int Eold;
    int Eint;
    int Edot;
    int desired;
    int actual;
    float u;
} control_error;

typedef enum {
    IDLE, /**< Sets motor effort to 0 */
    HOLD, /**< Sets motor effort to holding position */
    TRACK /**< Sets motor effort to tracking mode */
    } mode;    // define data structure containing modes


static volatile pid_gains gains1;
static volatile pid_gains gains2;

// Define position variables here.
int desiredPos1;
int desiredPos2;






/**
 * Data structure containing the control data
 */
typedef struct {
    int refPos[REFERENCE_DATA]; /**< The reference position of the motor */
    int actPos[BUFLEN];  /**< The actual position of the motor */
    float u[BUFLEN];  /**< The control effort of the motor */
} control_data_t;

/**
 * @brief Return the Mode of the robot
 *
 *
 *
 * @param mode operating mode of the robot
 * @return Void
 */
mode getMODE();                      // Return the current operating mode

/**
 * @brief Sets the Mode of the robot
 *
 * The avaliable modes are IDLE, HOLD, TRACK
 *
 * @param mode operating mode of the robot
 * @return Void
 */
void setMODE(mode newMODE);          // Set operating mode


/**
 * @brief Receive number N of samples to save into data buffer
 *
 *
 *
 * @param Void
 * @return Void
 */
void setN(void);

/**
 * @brief Returns the number of samples
 *
 *
 *
 * @param Void
 * @return int the number of samples
 */
int getN(void);


/**
 * @brief Writes the reference position
 *
 *
 *
 * @param int position
 * @param int index
 * @param int motor
 * @return Void
 */
void write_refPos(int position, int index, int motor);

/**
 * @brief Gets the reference position
 *
 *
 *
 * @param int index
 * @param int motor
 * @return int returns the reference position
 */
int get_refPos(int index, int motor);

/**
 * @brief Returns true if the buffer is empty
 *
 * READ == WRITE
 *
 * @param Void
 * @return int true if buffer is empty
 */
int buffer_empty(void);

/**
 * @brief Returns true if the buffer is full
 *
 * (WRITE + 1) % BUFLEN == READ
 *
 * @param Void
 * @return int true if buffer is full
 */
int buffer_full(void);

/**
 * @brief Reads position from the current buffer location
 *
 * This function assumes that buffer is not empty
 *
 * @param int motor the motor that is read from
 * @return int position of the current buffer location
 */
int buffer_read_position(int motor);

/**
 * @brief Reads current value from current buffer location
 *
 * This function assumes that buffer is not empty
 *
 * @param int motor motor that is read from
 * @return float current value in buffer
 */
float buffer_read_u(int motor);

/**
 * @brief Increments the buffer read index
 *
 *
 * @param Void
 * @return Void
 */
void buffer_read_increment(void);

/**
 * @brief Write data to buffer
 *
 *
 * @param int M1_actPos motor 1 position
 * @param int M2_actPos motor 2 position
 * @param float M1_u motor 1 effort
 * @param float M1_u motor 2 effort
 * @return Void
 */
void buffer_write(int M1_actPos, int M2_actPos, float M1_u, float M2_u);

/**
 * @brief Send data to client when it becomes available
 *
 *
 * @param Void
 * @return Void
 */
void send_data(void);


/**
* @brief Sets position gains on the Tiva microcontroller by reading from UART
*
* @param Void
* @return Void
*/
void set_position_gains(pid_gains gain);                      // Set position gains

/**
* @brief Gets position gains on the Tiva microcontroller by reading to UART
*
* @param Void
* @return Void
*/
void get_position_gains(pid_gains gain);                      // Get position gains


/**
* @brief Gets desired angle for a given motor
*
* @param motor reads the desired angle for the given motor
* @return Void
*/
int get_desired_angle(int motor);

/**
* @brief Sets desired angle for a given motor
*
* @param motor specifies the desired angle for the given motor
* @return Void
*/
void set_desired_angle(int angle, int motor);

/**
* @brief Resets desired position to origin
*
* @param Void
* @return Void
*/
void reset_pos(void);                               // Reset desired position to origin (0 um)

/**
* @brief Resets controller error on both controllers to be zero
*
* @param Void
* @return Void
*/
void reset_controller_error(void);                  // Reset the error on both controllers to be zero


/**
* @brief Loads position trajectory for a given motor over UART
*
* @param motor specifies the trajectory that is being assigned from UART
* @return Void
*/
void load_position_trajectory(int motor);                // Load desired position trajectory from client

/**
* @brief Returns the set pwm on a given motor
*
* @param motor the pwm on a given motor
* @return Void
*/
int get_motor_pwm(int motor);

/**
* @brief Calculates pwm to send to the motor drivr
*
* @param reference the reference angle
* @param actual the commanded angle
* @param motor the specified motor
* @return Void
*/

#endif /* CONTROL_H_ */
