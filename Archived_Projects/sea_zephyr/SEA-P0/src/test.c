#include "test.h"
#include <math.h>
#include <sys/util.h>
#include <timing/timing.h>
#include "stm32_std.h"

/* For the program:
    - Runs the motor for a number of seconds equal to testTime
    - Automatically chooses a print delay based on the inout frequency

*/

// Sets the value of pi
const double pi = 3.141592;
const double MAX_SPEED = 100;

const double MAX_MOTOR_SPEED = 175 * 6;

void runSpeedTest(double speed) {

    // setup time
    long long int dt;
    long long int time;
    long long int time_previous;

    double speed_actual;
    double xm;
    double xm_previous;

    dt = k_uptime_get(); // get initial time marker
    time = k_uptime_delta(&dt); // get current time in milliseconds

    // setup print variables and print delay
    double printDelay = 5000;
    long printCounter = 0;

    // signify start of new test
    printk("----------\n");
    k_msleep(5);
    reset_deg();
    // printk("%llu\n", time);

    while (time < 300 * 1000)
    {
        // Get the current time
        time += k_uptime_delta(&dt); // get current time in milliseconds

        double speed_frac = (speed / MAX_MOTOR_SPEED) * MAX_SPEED;
        setMotorSpeed(speed_frac);

        int deg3 = read_deg3();
        xm = (deg3 * 360.0 / 23040.0);
        int time_d = time - time_previous;

        if (printCounter >= printDelay)
        {
            speed_actual = (xm - xm_previous) / ((time - time_previous) / 1000.0);
            time_previous = time;
            xm_previous = xm;
            printk("%d, %d, %d\n", (int)time, (int)(speed / 6), deg3);
            printCounter = 0;
        }

        printCounter++;
    }

    // end off with stopping motor
    setMotorSpeed(0);
}

void runForceTest(double freq, int mag) {
    // setup constants
    const double fConst = pi * freq * 2;
    // const double mag = 90; // set constant value for initial testing

    // setup time
    long long int dt;
    long long int time;
    long long int previous_time;
    double load_val = 0;
    double x_load = 0;
    double xs = 0.0;
    double xm = 0.0;

    long long int test_time_ns = determineSimTime(freq);
    dt = k_uptime_get();        // get initial time marker
    time = k_uptime_delta(&dt); // get current time in milliseconds

    // setup print variables and print delay
    // long printDelay = mag > 20 ? (mag / 2) : 0;
    double printDelay = (1.0 / freq) * 10 * 10;
    long printCounter = 0;

    // create local variables
    double speed = 0.0;     // the speed of the motor
    double targetPos = 0; // desired location

    int Ts = 2000;
    double Mg = mag;

    // signify start of new test
    printk("----------\n");
    k_msleep(5);
    reset_deg();
    // printk("%llu\n", time);

    while (time < 3000 * 1000)
    {
        // Get the current time
        time += k_uptime_delta(&dt); // get current time in milliseconds

        // speed = fConst * mag * cos((double)(fConst * time / 1000)); // Speed in deg/second

        int deg5 = read_deg5();
        int deg3 = read_deg3();
        uint16_t adc_val = read_adc(10);

        xs = (deg5 * 360.0 / 40000.0) * (42.0 / 18.0);
        xm = (deg3 * 360.0 / 23040.0);

        // speed = 180.0;

        if (time % Ts <= Ts / 2)
        {
            // xd =   Mg * ((float)(time % Ts) / ((float)Ts / 2)) * (60.0 / 18.0);
            speed =   Mg / ((float)Ts / 2000.0);
        }

        if (time % Ts > Ts / 2 && time % Ts < Ts)
        {
            // xd =   Mg * (1.0 - ((float)(time % Ts) / ((float)Ts / 2) - 1.0)) * (60.0 / 18.0);
            speed = - Mg / ((float)Ts / 2000.0);
        }     

        double speed_frac = (speed / MAX_MOTOR_SPEED) * MAX_SPEED;
        setMotorSpeed(speed_frac);

        // x_load += (- 50.0 * 2.0 * 3.14 * x_load + (float)adc_val) * (time - previous_time) / 1000.0;
        // load_val = (50.0 * 2.0 * 3.14 * x_load - 1000.0) / (1190.0 - 1000.0);
        load_val = ((double)adc_val - 1000.0) / (1190.0 - 1000.0);

        // Low Pass Filer

        if (printCounter >= printDelay)
        {
            int converted_speed = ((speed > 0 ? speed : -speed) * 4096) / 100;
            int freq_conv = (freq * 100);

            // printk("%f, %f, %d\n", deg, 9.81 * load_val, (int)time);
            printk("%d, %f, %f\n", (int)time, xm, speed);
            printCounter = 0;
        }
        printCounter++;

        previous_time = time;
    }

    // end off with stopping motor
    setMotorSpeed(0);
}

void runImpedanceTest(double k, double a, double kp, double kd) {

    // setup time
    int force_control = 1;
    long long int dt;
    long long int time;
    long long int time_previous;
    double error_p = 0.0;
    double error_i = 0.0;

    double xs = 0.0;

    double xq_vs = 0.0;
    double xq_vs_dot = 0.0;
    double xq_vs_ddot = 0.0;
    double uq_vs = 0.0;
    double vs = 0.0;

    double xq_vd = 0.0;
    double xq_vd_dot = 0.0;
    double xq_vd_ddot = 0.0;
    double uq_vd = 0.0;
    double yq_vd = 0.0;
    double a_vd = 5.;

    double xq_speed = 0.0;
    double xq_speed_dot = 0.0;
    double xq_speed_ddot = 0.0;
    double uq_speed = 0.0;
    double yq_speed = 0.0;

    double ve = 0.0;
    double u0 = 0.0;
    double speed = 0.0;
    double xd = 0.0;
    double vd = 0.0;
    double xr = 0.0;
    double xm = 0.0;
    double xe = 0.0;
    double xs_error = 0.0;

    int Ts = 1500;
    int T3 = Ts / 3;
    int Te = Ts / 30;
    double Mg = 90.0;
    int deg5_offset = 0;

    dt = k_uptime_get();        // get initial time marker
    time = k_uptime_delta(&dt); // get current time in milliseconds

    // setup print variables and print delay
    // long printDelay = mag > 20 ? (mag / 2) : 0;
    double printDelay = 50;
    long printCounter = 0;

    // signify start of new test
    printk("----------\n");
    k_msleep(5);
    reset_deg();
    // printk("%llu\n", time);

    while (time < 30 * 1000)
    {
        // Get the current time
        time += k_uptime_delta(&dt); // get current time in milliseconds

        int deg5 = read_deg5();
        int deg5_index = read_qdec5_index();
        int deg3 = read_deg3();
        int deg3_index = read_qdec3_index();

        // Low Pass Filer

        if (printCounter >= printDelay)
        {
            // printk("%f, %f, %d\n", deg, 9.81 * load_val, (int)time);
            if (deg5_index == 1)
            {
            deg5_offset = 220 - deg5;
            printk("%d, %d, %d\n", (int)time, deg5, deg5_offset);
            }
            printCounter = 0;
        }
        printCounter++;

        time_previous = time;
    }

    while (time >= 30 * 1000 && time < 300 * 1000)
    {
        // Get the current time
        time += k_uptime_delta(&dt); // get current time in milliseconds

        int deg5 = read_deg5() + deg5_offset;
        int deg3 = read_deg3();
        int deg3_index = read_qdec3_index();
        int deg5_index = read_qdec5_index();

        // spring deflection
        xs = ((float)deg5 * 360.0 / 40000.0) * (42.0 / 18.0);
        xm = ((float)deg3 * 360.0 / 23040.0);
        xe = xm - xs;

        // spring deflection rate
        // vs = (xs - xs_previous) / ((time - time_previous) / 1000.0);

        // first order filer

        // uq_vs = xs;
        // xq_vs_dot = - a * xq_vs + uq_vs;
        // xq_vs += xq_vs_dot * ((time - time_previous) / 1000.0);
        // vs  = - a * a * xq_vs + a * uq_vs;

        // uq_vm = speed;
        // xq_vm_dot = - a * xq_vm + uq_vm;
        // xq_vm += xq_vm_dot * ((time - time_previous) / 1000.0);
        // yq_speed  =       a * xq_vm;

        // second order filer

        uq_vs = xs;
        xq_vs_ddot = - a * a * xq_vs - 2.0 * 0.707 * a * xq_vs_dot + uq_vs;
        xq_vs_dot += xq_vs_ddot * ((time - time_previous) / 1000.0);
        xq_vs     += xq_vs_dot  * ((time - time_previous) / 1000.0);
        vs  =   a * a * xq_vs_dot;

        uq_speed = speed;
        xq_speed_ddot = - a * a * xq_speed - 2.0 * 0.707 * a * xq_speed_dot + uq_speed;
        xq_speed_dot += xq_speed_ddot * ((time - time_previous) / 1000.0);
        xq_speed     += xq_speed_dot  * ((time - time_previous) / 1000.0);
        yq_speed  =   a * a * xq_speed;

        ve = yq_speed - vs;

        // ve = vm - vs;

        // time_frac = (float)(time % Ts) / ((float)Ts / 2);

        if (time % (2 * Ts) > 0 * T3 && time % (2 * Ts) <= 1 * T3)
        {
            xd = - Mg * ((float)(time % (2 * Ts)) / ((float)T3)) * (60.0 / 18.0);
            vd = - (Mg / ((float)T3 / 1000.0)) * (60.0 / 18.0);
            // xd =   (Mg / 2.0) * (60.0 / 18.0) * cos(((float)(time % (2 * Ts) - 0 * T3) / (float)(2 * T3)) * 2.0 * pi) - (Mg / 2.0) * (60.0 / 18.0);
            // vd = - (Mg / 2.0) * (60.0 / 18.0) * sin(((float)(time % (2 * Ts) - 0 * T3) / (float)(2 * T3)) * 2.0 * pi) * (2.0 * pi / (2.0 * (float)T3 / 1000.0));
        }

        if (time % (2 * Ts) > 1 * T3 && time % (2 * Ts) <= 2 * T3)
        {
            xd = - Mg * (60.0 / 18.0);
            vd = 0.0;
        }

        if (time % (2 * Ts) > 2 * T3 && time % (2 * Ts) <= 3 * T3)
        {
            xd = - Mg * (1.0 - ((float)(time % (2 * Ts)) / ((float)T3) - 2.0)) * (60.0 / 18.0);
            vd =   (Mg / ((float)T3 / 1000.0)) * (60.0 / 18.0);
            // xd = - (Mg / 2.0) * (60.0 / 18.0) * cos(((float)(time % (2 * Ts) - 2 * T3) / (float)(2 * T3)) * 2.0 * pi) - (Mg / 2.0) * (60.0 / 18.0);
            // vd =   (Mg / 2.0) * (60.0 / 18.0) * sin(((float)(time % (2 * Ts) - 2 * T3) / (float)(2 * T3)) * 2.0 * pi) * (2.0 * pi / (2.0 * (float)T3 / 1000.0));
        }

        if (time % (2 * Ts) > 3 * T3 && time % (2 * Ts) <= 4 * T3)
        {
            xd = 0.0;
            vd = 0.0;
        }

        if (time % (2 * Ts) > 4 * T3 && time % (2 * Ts) <  5 * T3)
        {
            xd = 30.0 * (60.0 / 18.0);
            vd = 0.0;
        }

        if (time % (2 * Ts) > 5 * T3 && time % (2 * Ts) <= 6 * T3)
        {
            xd = 0.0;
            vd = 0.0;
        }

        uq_vd = vd;
        xq_vd_ddot = - a_vd * a_vd * xq_vd - 2.0 * 0.707 * a_vd * xq_vd_dot + uq_vd;
        xq_vd_dot += xq_vd_ddot * ((time - time_previous) / 1000.0);
        xq_vd     += xq_vd_dot  * ((time - time_previous) / 1000.0);
        yq_vd  =   a_vd * a_vd * xq_vd;

        // xr = - kp * (xe - xd) - kd * (ve - yq_vd);

        xr = - kp * (xe - xd) - kd * (ve - 0.);

        if (xr >= 25.0)
        {
            xr = 25.0;
        }

        if (xr <= - 25.0)
        {
            xr = - 25.0;
        }

        // nominal speed command
        xs_error = xs - xr;

        u0 = - k * xs_error - 0.0 * vs;

        // u0 = - k * (xm - 10.0);

        // actual speed command
        if (deg3_index == 1)
        {      
            // if (xs_error >= - 0.1 && xs_error <= 0.1)
            // {
            //     force_control = 0;
            // }
            // else
            // {
            //     force_control = 1;
            // }

            if (force_control == 1)
            {               
                // speed = u0 + (yq_speed - vs);

                speed = u0;

                // if (xs_error >= - 0.1 && xs_error <= 0.1)
                // {
                //     force_control = 0;
                // }
                // else
                // {
                //     force_control = 1;
                // }
            }
            else
            {
                speed = 0.0;

                // if (xs_error >= - 0.1 && xs_error <= 0.1)
                // {
                //     force_control = 0;
                // }
                // else
                // {
                //     force_control = 1;
                // }
            }
        }
        else
        {
            speed = 0.0;
        }

        double speed_frac = (speed / MAX_MOTOR_SPEED) * MAX_SPEED;

        // if (speed_frac >= -5.0 && speed_frac <= 5.0)
        // {
        //     speed_frac = 0.0;
        // }

        setMotorSpeed(speed_frac);

        // Low Pass Filer

        if (printCounter >= printDelay)
        {
            int converted_speed = ((speed > 0 ? speed : -speed) * 4096) / 100;

            // printk("%f, %f, %d\n", deg, 9.81 * load_val, (int)time);
            // printk("%d, %d, %f\n", (int)time, deg3, deg3_index);
            printk("%d, %f, %f, %f, %f, %f, %f\n", (int)time, xd, xe, vd, ve, xr, xs);
            printCounter = 0;
        }
        printCounter++;

        time_previous = time;
    }

    while (time > 300 * 1000)
    {
        setMotorSpeed(0);
    }

    // end off with stopping motor
    setMotorSpeed(0);
}

uint64_t determineSimTime(double freq)
{
    if (freq < 0.02) {
        return 270;
    } else if (freq < 0.05) {
        return 150;
    } else if (freq < 0.1) {
        return 90;
    } else {
        return 60;
    }
}

void printVals(double degree, double target, uint64_t time, double freq, double mag, uint64_t maxtime, double speed)
{
    int converted_speed = ((speed > 0 ? speed : -speed) * 4096) / 100;
    int freq_conv = (freq * 100);

    // char buff[25];
    printk("%f, %f, %d, %d, %d\n", degree, target, (int)time, freq_conv, (int)mag);
    // printq_add(buff);
    // k_msleep(4);
}