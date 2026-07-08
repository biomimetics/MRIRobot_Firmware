/******************************************************************************
 *
 * adaptive_disturbance_observer.c
 *
 * Fixed-Gain Extended State Observer
 *
 * Features:
 *
 *   - Position + velocity estimation
 *   - Unknown disturbance estimation
 *   - Adaptive command delay estimation
 *   - Adaptive command confidence weighting
 *
 * Note:
 *  I don't really trust this code as-is and it needs some big refienments before trying to impliment this with our reactors.
 ******************************************************************************/

#include <stdint.h>
#include <math.h>


#define COMMAND_HISTORY_SIZE 128
#define DELAY_CORRELATION_WINDOW 32



typedef struct
{
    float command;
    float velocity;

} HistorySample;



typedef struct
{
    /*
        Estimated states
    */

    float position;
    float velocity;

    /*
        Estimated unknown acceleration.
    */

    float disturbance;



    /*
        Command confidence

        0.0 = ignore command
        1.0 = trust command completely
    */

    float command_confidence;



    /*
        Command history
    */

    HistorySample history[COMMAND_HISTORY_SIZE];

    uint16_t history_index;

    float sample_time;



    /*
        Delay estimator
    */

    float estimated_delay_samples;

    uint16_t max_delay_samples;

    float delay_filter_gain;


    float previous_command;

    float command_change_threshold;



    /*
        Command prediction model
    */

    float command_gain;



    /*
        Observer gains
    */

    float position_gain;

    float position_velocity_gain;

    float velocity_gain;



    /*
        Disturbance learning
    */

    float disturbance_gain;

    float disturbance_decay;



    /*
        Confidence adaptation
    */

    float confidence_gain;

    float minimum_command_confidence;

    float maximum_command_confidence;



    /*
        Innovation limits
    */

    float max_position_error;

    float max_velocity_error;


} Observer;



static float Clamp(
    float x,
    float min,
    float max)
{
    if (x < min)
        return min;

    if (x > max)
        return max;

    return x;
}



/******************************************************************************
 *
 * Initialize
 *
 ******************************************************************************/

void Observer_Init(
    Observer *obs,
    float initial_position,
    float initial_velocity,
    float sample_time)
{
    uint16_t i;


    obs->position = initial_position;

    obs->velocity = initial_velocity;

    obs->disturbance = 0.0f;


    obs->command_confidence = 0.5f;


    obs->sample_time = sample_time;


    for (i = 0;
         i < COMMAND_HISTORY_SIZE;
         i++)
    {
        obs->history[i].command = 0.0f;

        obs->history[i].velocity =
            initial_velocity;
    }


    obs->history_index = 0;


    obs->estimated_delay_samples = 0.0f;


    obs->max_delay_samples = 100;

    obs->delay_filter_gain = 0.05f;



    obs->previous_command = 0.0f;

    obs->command_change_threshold = 0.05f;



    /*
        Prediction

        Start conservative.
    */

    obs->command_gain = 8.0f;



    /*
        Correction
    */

    obs->position_gain = 0.25f;

    obs->position_velocity_gain = 12.0f;

    obs->velocity_gain = 0.2f;



    /*
        Disturbance
    */

    obs->disturbance_gain = 3.0f;

    obs->disturbance_decay = 0.2f;



    /*
        Confidence adaptation

        Small value = slow learning.
    */

    obs->confidence_gain = 0.5f;

    obs->minimum_command_confidence = 0.0f;

    obs->maximum_command_confidence = 1.0f;



    obs->max_position_error = 10.0f;

    obs->max_velocity_error = 50.0f;
}



/******************************************************************************
 *
 * Add history
 *
 ******************************************************************************/

static void AddHistory(
    Observer *obs,
    float command,
    float velocity)
{
    obs->history[obs->history_index].command =
        command;

    obs->history[obs->history_index].velocity =
        velocity;


    obs->history_index++;

    if (obs->history_index >= COMMAND_HISTORY_SIZE)
        obs->history_index = 0;
}



/******************************************************************************
 *
 * Delay estimation
 *
 ******************************************************************************/

static void UpdateDelayEstimate(
    Observer *obs)
{
    uint16_t delay;

    uint16_t best_delay = 0;

    float best_score = -1.0e30f;



    for (delay = 0;
         delay <= obs->max_delay_samples;
         delay++)
    {
        float score = 0.0f;


        uint16_t i;


        for (i = 1;
             i < DELAY_CORRELATION_WINDOW;
             i++)
        {
            int cmd_index =
                obs->history_index -
                i -
                delay;


            int vel_index =
                obs->history_index -
                i;


            while (cmd_index < 0)
                cmd_index += COMMAND_HISTORY_SIZE;


            while (vel_index < 0)
                vel_index += COMMAND_HISTORY_SIZE;



            cmd_index %= COMMAND_HISTORY_SIZE;

            vel_index %= COMMAND_HISTORY_SIZE;



            float cmd =
                obs->history[cmd_index].command;


            float vel =
                obs->history[vel_index].velocity;


            score +=
                cmd *
                vel;
        }


        if (score > best_score)
        {
            best_score = score;

            best_delay = delay;
        }
    }



    obs->estimated_delay_samples +=
        obs->delay_filter_gain *
        ((float)best_delay -
         obs->estimated_delay_samples);
}



/******************************************************************************
 *
 * Get delayed command
 *
 ******************************************************************************/

static float GetDelayedCommand(
    Observer *obs)
{
    int delay =
        (int)(obs->estimated_delay_samples + 0.5f);


    if (delay >= COMMAND_HISTORY_SIZE)
        delay = COMMAND_HISTORY_SIZE - 1;


    int index =
        obs->history_index -
        delay -
        1;


    while (index < 0)
        index += COMMAND_HISTORY_SIZE;


    return obs->history[index].command;
}



/******************************************************************************
 *
 * Update command confidence
 *
 ******************************************************************************/

static void UpdateCommandConfidence(
    Observer *obs,
    float velocity_error)
{
    /*
        Large velocity error means
        command prediction is unreliable.
    */

    float error_scale =
        fabsf(velocity_error) /
        obs->max_velocity_error;


    error_scale =
        Clamp(
            error_scale,
            0.0f,
            1.0f);



    float target_confidence =
        1.0f -
        error_scale;



    /*
        Slowly move confidence.
    */

    obs->command_confidence +=
        obs->confidence_gain *
        (target_confidence -
         obs->command_confidence) *
        obs->sample_time;



    obs->command_confidence =
        Clamp(
            obs->command_confidence,
            obs->minimum_command_confidence,
            obs->maximum_command_confidence);
}



/******************************************************************************
 *
 * Main update
 *
 ******************************************************************************/

void Observer_Update(
    Observer *obs,

    float command_velocity,

    float measured_position,
    uint8_t position_valid,

    float measured_velocity,
    uint8_t velocity_valid)
{

    AddHistory(
        obs,
        command_velocity,
        measured_velocity);



    /*
        Delay estimator excitation test.
    */

    float command_delta =
        fabsf(
            command_velocity -
            obs->previous_command);


    obs->previous_command =
        command_velocity;



    if (command_delta >
        obs->command_change_threshold)
    {
        UpdateDelayEstimate(obs);
    }



    float delayed_command =
        GetDelayedCommand(obs);



    /**********************************************************************
        Prediction
    **********************************************************************/

    float command_accel =
        obs->command_gain *
        (delayed_command -
         obs->velocity);



    /*
        Command influence is weighted.
    */

    float acceleration =
        obs->command_confidence *
        command_accel;



    /*
        Unknown dynamics.
    */

    acceleration +=
        obs->disturbance;



    obs->velocity +=
        acceleration *
        obs->sample_time;


    obs->position +=
        obs->velocity *
        obs->sample_time;



    /*
        Disturbance decay.
    */

    obs->disturbance *=
        1.0f -
        obs->disturbance_decay *
        obs->sample_time;



    /**********************************************************************
        Corrections
    **********************************************************************/

    float velocity_error = 0.0f;



    if (position_valid)
    {
        float error =
            measured_position -
            obs->position;


        error =
            Clamp(
                error,
                -obs->max_position_error,
                 obs->max_position_error);



        obs->position +=
            obs->position_gain *
            error;


        obs->velocity +=
            obs->position_velocity_gain *
            error *
            obs->sample_time;
    }



    if (velocity_valid)
    {
        velocity_error =
            measured_velocity -
            obs->velocity;


        velocity_error =
            Clamp(
                velocity_error,
                -obs->max_velocity_error,
                 obs->max_velocity_error);



        obs->velocity +=
            obs->velocity_gain *
            velocity_error;



        obs->disturbance +=
            obs->disturbance_gain *
            velocity_error *
            obs->sample_time;



        UpdateCommandConfidence(
            obs,
            velocity_error);
    }
}