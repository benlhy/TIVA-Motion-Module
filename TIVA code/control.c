#include "system.h"
#include "uart.h"
#include "control.h"
#include "motor.h"

extern int max_pwm;
/*
// PID gains
static volatile pid_gains gains1;
static volatile pid_gains gains2;
*/
// Control info


static volatile control_error E1;
static volatile control_error E2;

static volatile mode MODE;              // Operating mode

static volatile control_data_t M1;    // Struct containing data arrays
static volatile control_data_t M2;






void GAINSconfig(void){ // sets all gains to zero.
    gains1.kp = 0;
    gains1.ki = 0;
    gains1.kd = 0;
    gains2.kp = 0;
    gains2.ki = 0;
    gains2.kd = 0;
}




// Set new desired angle for motor
// takes angle in counts
void set_desired_angle(int angle, int motor)
{
    IntMasterDisable();
    if (motor == 1)
    {
        E1.desired = angle;
    }
    else if (motor == 2)
    {
        E2.desired = angle;
    }
    IntMasterEnable();
}

int get_desired_angle(int motor)
{
    int angle;
    if (motor == 1)
    {
        angle = E1.desired;
    }
    else if (motor == 2)
    {
        angle = E2.desired;
    }

    return angle;
}

// Reset error for all of the motors
void reset_controller_error(void)
{
    E1.Eold = 0;
    E1.Enew = 0;
    E1.Eint = 0;
    E1.Edot = 0;

    E2.Eold = 0;
    E2.Enew = 0;
    E2.Eint = 0;
    E2.Edot = 0;
}

int get_motor_pwm(int motor)
{
    int pwm;
    if (motor == 1)
    {
        pwm = E1.u;
    }
    else if (motor == 2)
    {
        pwm = E2.u;
    }

    return pwm;
}
/*
void
Timer1IntHandler(void)
{
    static int decctr = 0;  // counter for data decimation
    static int i = 0;   // trajectory index

    switch(getMODE())
    {
        case IDLE:
        {
            // do nothing
            break;
        }
        case HOLD:
        {
            E1.actual = getMotor1Counts(); // read positions
            E2.actual = getMotor2Counts();
            pid_controller(E1.desired, E1.actual, 1);    // motor1 control
            pid_controller(E2.desired, E2.actual, 2);    // motor2 control
            break;

        }
        case TRACK:
        {
            if (i == getN())    // Done tracking when index equals number of samples
            {
                i = 0; // reset index
                setMODE(HOLD);  // Hold final position, Could set PWM to zero instead
            }
            else
            {
                E1.actual = getMotor1Counts(); // actual read positions
                E2.actual = getMotor2Counts();
                E1.desired = get_refPos(i, 1);
                E2.desired = get_refPos(i, 2);
                pid_controller(E1.desired, E1.actual, 1);    // motor1 control
                pid_controller(E2.desired, E2.actual, 2);    // motor2 control

                i++;    // increment index

                // Handle data decimation
                decctr++;
                if (decctr == DECIMATION)
                {
                    buffer_write(E1.actual, E2.actual, E1.u, E2.u);
                    decctr = 0; // reset decimation counter
                }
            }
            break;
        }
    }

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // clear the interrupt flag
}

// Calculate control effort and set pwm value to control motor
void pid_controller(int reference, int actual, int motor)
{
    static float u;

    if (motor == 1)
    {
        E1.Enew = reference - actual;               // Calculate error
        E1.Eint = E1.Eint + E1.Enew;                // Calculate intergral error
        E1.Edot = E1.Enew - E1.Eold;                // Calculate derivative error
        E1.Eold = E1.Enew;                          // Update old error
        u = gains1.kp*E1.Enew + gains1.ki*E1.Eint + gains1.kd*E1.Edot;   // Calculate effort
    }
    else if (motor == 2)
    {
        E2.Enew = reference - actual;               // Calculate error
        E2.Eint = E2.Eint + E2.Enew;                // Calculate intergral error
        E2.Edot = E2.Enew - E2.Eold;                // Calculate derivative error
        E2.Eold = E2.Enew;                          // Update old error
        u = gains2.kp*E2.Enew + gains2.ki*E2.Eint + gains2.kd*E2.Edot;   // Calculate effort
    }

    // Max effort
    if (u > max_pwm)
    {
        u = max_pwm;
    }
    else if (u < -max_pwm)
    {
        u = -max_pwm;
    }

    // Set new control effort
    if (motor == 1)
    {
        setSpeed(motor,u);
        E1.u = u;
    }
    else if(motor == 2)
    {
        setSpeed(motor,u);
        E2.u = u;
    }

}

*/
void load_position_trajectory(int motor)      // Load trajectory for tracking
{
    int i, n, data;
    char buffer[10];
    setN();         // Receive number of samples from client
    n = getN();     // Determine number of samples

    for (i = 0; i < n; i++)
    {
        uartRead(buffer,10);           // Read reference position from client
        sscanf(buffer,"%d",&data);      // Store position in data
        write_refPos(data, i, motor);   // Write data to reference position array
    }
}

