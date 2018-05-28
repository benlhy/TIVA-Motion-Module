#include "gen_algo.h"
#include <limits.h>
#include "system.h"

void absolute_to_relative_config(absolute_data *data,int max,int cross,int curr_value){
    data->max_value = max; // set max value
    data->cross_over_value = cross; // set the crossover value
    data->modifier = 0; // reset the modifier to zero
    data->last_value = curr_value; // set the last value to the one we are reading now
}

int absolute_to_relative(int absolute, absolute_data* data){ // computes and returns relative angles, keeps track of it over time.
    int diff_value = absolute - data->last_value; // take the difference in value

    if (diff_value > data->cross_over_value){ // if the difference is greater than the threshold value, remove from the modifier
        data->modifier = data->modifier - data->max_value;

    }
    else if (diff_value < -data->cross_over_value){ // if the difference is less than the threshold vaue, add to the modifer because we crossed over the value
        data->modifier = data->modifier + data->max_value;
    }
    data->last_value = absolute; // set our last value to the current value
    return absolute+data->modifier; // return current value + the modifier
}


// Set up pid values
void pid_config(pid_values *pid_val,float kp, float ki, float kd,int limit, int intlimit){
    pid_val->int_value=0;
    pid_val->prev_value=0;
    pid_val->limit = limit; // maximum pwm
    pid_val->intlimit = intlimit; // maximum integration value
    pid_val->kp=kp;
    pid_val->ki=ki;
    pid_val->kd=kd;
}

int pid_controller(pid_values* pid, int desired, int current){
    int e = desired - current; // get current error
    int eint = pid->int_value + e; // set the integral value
    int ediv = e - pid->prev_value; // no need to divide by time since you are multiplying by a constant->
    int controlsig = pid->kp*e+pid->kd*ediv+pid->ki*eint; // calculate the control signal


    // limit the control signal to the max value
    if(controlsig> pid->limit){
        controlsig = pid->limit;
    }
    else if (controlsig< -pid->limit){
        controlsig = -pid->limit;
    }

    // limit the integration value
    if (eint>pid->intlimit){
        pid->int_value = pid->intlimit;
    }
    else if(eint< -pid->intlimit){
        pid->int_value = -pid->intlimit;
    }
    else {
        pid->int_value = eint;
    }
    // set the previous error to the current error
    pid->prev_value = e;

    // output the control signal
    return controlsig;

}


// this tunes the PID controller to obtain the fastest rise time based on the Nelder Mead Method
void pid_auto_tune(pid_values* pid,int desired, int limit, int intlimit,int(*curr_val_function)(void),void(*run_motor_function)(int,int),void(*zero_motor_function)(void),int motor){

    // recommended constants

    float alpha = 1;
    float delta = 0.5;
    float gamma = 2;
    float rho = 0.5;

    // start with 4 test points because we are searching in R3 space
    char output[50];
    float testArray[4][3] = {
         {15.0,1.0,0.5},
         {5.5,0.2,0},
         {30.0,1.0,1.0},
         {5.0,1.0,0.2}
    };
    int resultArray[4];
    int i=0;
    // compute the results of each position
    int search = 0;
    while(search<20){
        for (i=0;i<4;i++){
            zero_motor_function(); // zero the motor
            pid_config(pid,testArray[i][0],testArray[i][1],testArray[i][2],limit,intlimit); // configure the pid object
            int result=pid_test(pid,desired,curr_val_function,run_motor_function,motor); // test the pid
            run_motor_function(0,1); // turn off motor
            resultArray[i] = result; // save the value
            //sprintf(output,"Result of %d is %d\r\n",i,result);
            //uartWrite(output);
            //delayMS(1000);
        }

        //order the results
        int tempresult;
        float tempkp;
        float tempkd;
        float tempki;
        int j;
        for (i=0;i<4;++i){
            for (j=i+1;j<4;++j){
                if(resultArray[i]>resultArray[j]){

                    tempresult = resultArray[i];
                    tempkp = testArray[i][0];
                    tempki = testArray[i][1];
                    tempkd = testArray[i][2];

                    resultArray[i]=resultArray[j];
                    testArray[i][0]=testArray[j][0];
                    testArray[i][1]=testArray[j][1];
                    testArray[i][2]=testArray[j][2];

                    resultArray[j]=tempresult;
                    testArray[j][0]=tempkp;
                    testArray[j][1]=tempki;
                    testArray[j][2]=tempkd;
                }
            }
        }
        sprintf(output,"Done Sorting!\r\n");
        uartWrite(output);
        delayMS(1000);

        for (i=0;i<4;i++){
            sprintf(output,"Sorted %d is result:%d kp:%f ki:%f kd:%f\r\n",i,resultArray[i],testArray[i][0],testArray[i][1],testArray[i][2]);
            uartWrite(output);
            delayMS(1000);
        }

        // calculate centroid

        float centroidkp;
        float centroidki;
        float centroidkd;
        centroidkp = (testArray[0][0]+testArray[1][0]+testArray[2][0])/3;
        centroidki = (testArray[0][1]+testArray[1][1]+testArray[2][1])/3;
        centroidkd = (testArray[0][2]+testArray[1][2]+testArray[2][2])/3;

        // calculate reflected point
        float reflectedkp = centroidkp+ alpha*(centroidkp-testArray[3][0]);
        float reflectedki = centroidki+ alpha*(centroidki-testArray[3][1]);
        float reflectedkd = centroidkd+ alpha*(centroidkd-testArray[3][2]);

        //sprintf(output,"Result of reflected points kp:%f ki:%f kd:%f\r\n", reflectedkp,reflectedki,reflectedkd);
        //uartWrite(output);
        //delayMS(1000);
        // compute reflected point // if this point is -ve then result is +ve infty

        int rresult;
        if (reflectedkp<0 ||reflectedki<0||reflectedkd<0){ // if any of the PID constants are negative
            //don't even calculate this result is horrible
            pid_config(pid,reflectedkp,reflectedki,reflectedkd,limit,intlimit);
            rresult = INT_MAX; // set the worst possible value to tell the algorithm to move away from this point
        }
        else {
            zero_motor_function(); // zero the motor
            pid_config(pid,reflectedkp,reflectedki,reflectedkd,limit,intlimit); // configure our PID struct
            rresult=pid_test(pid,desired,curr_val_function,run_motor_function,motor); // test our PID performance
            run_motor_function(0,1); // turn off motor
            //sprintf(output,"Result %d\r\n", rresult);
            //uartWrite(output);
            //delayMS(1000);
        }

        if ((resultArray[0]<=rresult) && (rresult<resultArray[1])){
            // if we are worse than the best result but better than the second best
            // replace last value with reflected point and reorder
            sprintf(output,"Case 1\r\n");
            uartWrite(output);
            delayMS(1000);




            testArray[3][0] = reflectedkp;
            testArray[3][1] = reflectedki;
            testArray[3][2] = reflectedkd;
            resultArray[3] = rresult;
            // go to.. top
        }


        else if (rresult<resultArray[0]) { // if we are better than the best result
            // we can expect to find interesting things along the reflected line.
            sprintf(output,"Case 2\r\n");
            uartWrite(output);
            delayMS(1000);


            // expand the point


            float expandedkp = centroidkp+ gamma*(reflectedkp-centroidkp);
            float expandedki = centroidki+ gamma*(reflectedki-centroidki);
            float expandedkd = centroidkd+ gamma*(reflectedkd-centroidkd);
            int eresult;


            if (expandedkp<0 ||expandedkp<0||expandedkp<0){ // negative PID constants
                //don't even calculate this result is horrible
                pid_config(pid,expandedkp,expandedkp,expandedkp,limit,intlimit);
                eresult = INT_MAX; // worst possible result
            }
            else{

                // compute expanded point
                zero_motor_function(); //zero motors
                pid_config(pid,expandedkp,expandedki,expandedkd,limit,intlimit); // configure with expanded point
                eresult=pid_test(pid,desired,curr_val_function,run_motor_function,motor); //test and obtain results
                run_motor_function(0,motor); // turn off motor
            }

            if (eresult<rresult){ // our expanded value works better
                // replace last value with expanded point and reorder
                testArray[3][0] = expandedkp;
                testArray[3][1] = expandedki;
                testArray[3][2] = expandedkd;
                resultArray[3] = eresult;
                // go to step 1
            }
            else { // expansion resulted in a worse result than our reflected value
                testArray[3][0] = reflectedkp;
                testArray[3][1] = reflectedki;
                testArray[3][2] = reflectedkd;
                resultArray[3] = rresult;

                // go to step 1
            }

        }
        else {
            sprintf(output,"Case 3\r\n");
                    uartWrite(output);
                    delayMS(1000);

            // all other test cases fail, our value is still worse than the second best, now contract this point
            float contractedkp = centroidkp+ rho*(testArray[3][0]-centroidkp);
            float contractedki = centroidki+ rho*(testArray[3][1]-centroidki);
            float contractedkd = centroidkd+ rho*(testArray[3][2]-centroidkd);

            // is there a negative test case here?





            zero_motor_function();
            pid_config(pid,contractedkp,contractedki,contractedkd,limit,intlimit);
            int cresult=pid_test(pid,desired,curr_val_function,run_motor_function,motor);
            run_motor_function(0,motor); // turn off motor
            if (cresult<resultArray[3]){
                testArray[3][0] = contractedkp;
                testArray[3][1] = contractedki;
                testArray[3][2] = contractedkd;
                resultArray[3] = cresult;

                // go to step 1
            }
            else {
                // shrink all values except for the best
                for (i=0;i<3;i++){
                    testArray[i][0]= testArray[0][0]+delta*(testArray[i][0]-testArray[0][0]);
                    testArray[i][1]= testArray[0][1]+delta*(testArray[i][1]-testArray[0][1]);
                    testArray[i][2]= testArray[0][2]+delta*(testArray[i][2]-testArray[0][2]);
                }
                // go to step 1
            }
        }


    search++; //increment the search counter
    }

    // reorder results
    int tempresult;
    float tempkp;
    float tempkd;
    float tempki;
    int j;
    for (i=0;i<4;++i){
        for (j=i+1;j<4;++j){
            if(resultArray[i]>resultArray[j]){

                tempresult = resultArray[i];
                tempkp = testArray[i][0];
                tempki = testArray[i][1];
                tempkd = testArray[i][2];

                resultArray[i]=resultArray[j];
                testArray[i][0]=testArray[j][0];
                testArray[i][1]=testArray[j][1];
                testArray[i][2]=testArray[j][2];

                resultArray[j]=tempresult;
                testArray[j][0]=tempkp;
                testArray[j][1]=tempki;
                testArray[j][2]=tempkd;
            }
        }
    }


    for (i=0;i<4;i++){
        sprintf(output,"Result of sorted %d is kp:%f kd:%f ki:%f\r\n",i,testArray[i][0],testArray[i][1],testArray[i][2]);
        uartWrite(output);
        delayMS(1000);
    }
    pid_config(pid,testArray[0][0],testArray[0][1],testArray[0][2],limit,intlimit);
}

int pid_test(pid_values* pid,int desired_val,int(*curr_val_function)(void),void(*run_motor_function)(int,int),int motor){
   int i = 0;
   int count = 0;
   while (i<TEST_PERIOD){
       count = count + abs(desired_val - curr_val_function());
       int output_val=pid_controller(pid,desired_val,curr_val_function());
       run_motor_function(output_val,motor);
       i++;
   }
   run_motor_function(0,motor);
   return count;
}
