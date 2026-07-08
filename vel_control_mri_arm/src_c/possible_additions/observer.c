/******************************************************************************
 *
 *  observer.c
 *
 *  Second-Order Fixed-Gain Luenberger Observer
 *
 *  Estimated states
 *
 *      Position
 *      Velocity
 *
 *  Inputs
 *
 *      Measured Position
 *      Measured Velocity
 *      Delayed Velocity Command
 *
 *  Notes:
        Not yet added to the STM32 code yet. I want to refactor this a bit and fine-tune it for our problem.
 ******************************************************************************/

#include <stdint.h>

typedef struct
{
    /*----------------------------------------------------------
        Estimated states
    ----------------------------------------------------------*/

    float position;
    float velocity;

    /*----------------------------------------------------------
        Prediction model
    ----------------------------------------------------------*/

    /*
        Approximate actuator time constant.

        Smaller = faster response.

        Units: seconds
    */

    float tau;

    /*----------------------------------------------------------
        Observer gains
    ----------------------------------------------------------*/

    /*
        Position correction.

        Dimensionless.
    */

    float Lp;

    /*
        Position -> velocity correction.

        Units: 1/s
    */

    float Lv;

    /*
        Direct velocity correction.

        Dimensionless.
    */

    float Lvv;

} Observer;


/******************************************************************************
    Initialize
******************************************************************************/

void Observer_Init(
    Observer *obs,
    float position,
    float velocity)
{
    obs->position = position;
    obs->velocity = velocity;

    /*
        These are conservative defaults.
    */

    obs->tau = 0.050f;      /* 50 ms */

    obs->Lp = 0.20f;

    obs->Lv = 12.0f;

    obs->Lvv = 0.15f;
}


/******************************************************************************
    Prediction
******************************************************************************/

static void Observer_Predict(
    Observer *obs,
    float dt,
    float delayed_command_velocity)
{
    /*
        First-order actuator model.

        dv/dt = (vcmd-v)/tau
    */

    float acceleration =
        (delayed_command_velocity -
         obs->velocity)
        / obs->tau;

    /*
        Integrate velocity.
    */

    obs->velocity +=
        acceleration * dt;

    /*
        Integrate position.

        Semi-implicit Euler.
    */

    obs->position +=
        obs->velocity * dt;
}


/******************************************************************************
    Correction
******************************************************************************/

static void Observer_Correct(
    Observer *obs,
    float dt,
    float measured_position,
    float measured_velocity)
{
    float ep =
        measured_position -
        obs->position;

    float ev =
        measured_velocity -
        obs->velocity;

    /*
        Correct position.
    */

    obs->position +=
        obs->Lp * ep;

    /*
        Position also corrects velocity.

        Scaling by dt makes tuning
        nearly sample-rate independent.
    */

    obs->velocity +=
        obs->Lv *
        ep *
        dt;

    /*
        Velocity sensor trims prediction.
    */

    obs->velocity +=
        obs->Lvv *
        ev;
}


/******************************************************************************
    Public update
******************************************************************************/

void Observer_Update(
    Observer *obs,
    float dt,
    float measured_position,
    float measured_velocity,
    float delayed_command_velocity)
{
    Observer_Predict(
        obs,
        dt,
        delayed_command_velocity);

    Observer_Correct(
        obs,
        dt,
        measured_position,
        measured_velocity);
}