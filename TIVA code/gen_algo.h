/*
 * gen_algo.h
 *
 * Stores general-purpose algorithms
 * 1. converts absolute readings to relative readings
 * 2. PID controllers
 *
 *  Created on: May 13, 2018
 *      Author: Ben
 */

#ifndef GEN_ALGO_H_
#define GEN_ALGO_H_

#define TEST_PERIOD 100000

typedef struct {
    int max_value;
    int cross_over_value;
    int last_value;
    int modifier;
} absolute_data;

typedef struct{
    float kp;
    float ki;
    float kd;
    int limit;
    int intlimit;
    int prev_value;
    int int_value;
} pid_values;

void absolute_to_relative_config(absolute_data* data,int max,int cross,int curr_value);

int absolute_to_relative(int curr_reading, absolute_data* data);

void pid_config(pid_values* pid,float kp, float ki, float kd, int limit, int intlimit);

int pid_controller(pid_values* pid,int desired, int current);

int pid_test(pid_values* pid,int desired_val,int(*curr_val_function)(void),void(*run_motor_function)(int speed,int motor),int motor);

void pid_auto_tune(pid_values* pid,int desired, int limit, int intlimit,int(*curr_val_function)(void),void(*run_motor_function)(int,int),void(*zero_motor_function)(void),int motor);

#endif /* GEN_ALGO_H_ */
