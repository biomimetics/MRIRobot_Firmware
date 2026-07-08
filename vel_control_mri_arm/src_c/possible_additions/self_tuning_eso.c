/******************************************************************************
 *
 * self_tuning_eso.c
 *
 * Adaptive disturbance observer with:
 *
 *   - Position estimation
 *   - Velocity estimation
 *   - Disturbance estimation
 *   - Command delay estimation
 *   - Command confidence weighting
 *   - Position confidence weighting
 *   - Velocity confidence weighting
 *
 * Note:
 *  This file is way too complicated but might have some useful ideas for refactoring into something decent.
 ******************************************************************************/

#include <stdint.h>
#include <math.h>


#define COMMAND_HISTORY_SIZE 128


typedef struct
{
    float command;
    float velocity;

} HistorySample;



typedef struct
{
    /*
        Estimated state
    */

    float position;
    float velocity;


    /*
        Extended state:
        unknown acceleration disturbance
    */

    float disturbance;



    /*
        Confidence values

        Range:
            0 = ignore
            1 = trust fully
    */

    float command_confidence;
    float position_confidence;
    float velocity_confidence;



    /*
        Command history
    */

    HistorySample history[COMMAND_HISTORY_SIZE];

    uint16_t history_index;

    float sample_time;



    /*
        Delay estimation
    */

    float delay_samples;

    uint16_t max_delay_samples;

    float delay_learning_rate;

    float previous_command;

    float command_excitation_threshold;



    /*
        Prediction model
    */

    float command_gain;



    /*
        Correction gains
    */

    float position_gain;

    float position_to_velocity_gain;

    float velocity_gain;



    /*
        Disturbance learning
    */

    float disturbance_gain;

    float disturbance_decay;

    float max_disturbance;



    /*
        Confidence adaptation

    */

    float confidence_learning_rate;



    /*
        Minimum trust values

        Prevent complete sensor rejection.
    */

    float min_command_confidence;

    float min_position_confidence;

    float min_velocity_confidence;



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
    float position,
    float velocity,
    float dt)
{
    uint16_t i;


    obs->position = position;

    obs->velocity = velocity;

    obs->disturbance = 0.0f;


    obs->command_confidence = 0.5f;

    obs->position_confidence = 1.0f;

    obs->velocity_confidence = 1.0f;


    obs->sample_time = dt;



    for (i = 0;
         i < COMMAND_HISTORY_SIZE;
         i++)
    {
        obs->history[i].command = 0.0f;
        obs->history[i].velocity = velocity;
    }


    obs->history_index = 0;



    obs->delay_samples = 0.0f;

    obs->max_delay_samples = 100;

    obs->delay_learning_rate = 0.05f;


    obs->previous_command = 0.0f;

    obs->command_excitation_threshold = 0.05f;



    obs->command_gain = 8.0f;



    obs->position_gain = 0.25f;

    obs->position_to_velocity_gain = 12.0f;

    obs->velocity_gain = 0.2f;



    obs->disturbance_gain = 3.0f;

    obs->disturbance_decay = 0.2f;

    obs->max_disturbance = 100.0f;



    obs->confidence_learning_rate = 0.5f;



    obs->min_command_confidence = 0.05f;

    obs->min_position_confidence = 0.05f;

    obs->min_velocity_confidence = 0.05f;



    obs->max_position_error = 10.0f;

    obs->max_velocity_error = 50.0f;
}



/******************************************************************************
 *
 * Utility
 *
 ******************************************************************************/

static void PushHistory(
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
 * Delay estimator
 *
 ******************************************************************************/

static void UpdateDelay(
    Observer *obs)
{
    uint16_t delay;

    float best_score = -1e30f;

    uint16_t best_delay = 0;


    for (delay = 0;
         delay <= obs->max_delay_samples;
         delay++)
    {
        float score = 0.0f;


        for (uint16_t i = 1;
             i < 32;
             i++)
        {
            int c =
                obs->history_index -
                i -
                delay;


            int v =
                obs->history_index -
                i;


            while (c < 0)
                c += COMMAND_HISTORY_SIZE;


            while (v < 0)
                v += COMMAND_HISTORY_SIZE;


            score +=
                obs->history[c].command *
                obs->history[v].velocity;
        }


        if (score > best_score)
        {
            best_score = score;
            best_delay = delay;
        }
    }


    obs->delay_samples +=
        obs->delay_learning_rate *
        ((float)best_delay -
         obs->delay_samples);
}



static float GetDelayedCommand(
    Observer *obs)
{
    int delay =
        (int)(obs->delay_samples + 0.5f);


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
 * Confidence update
 *
 ******************************************************************************/

static void UpdateConfidence(
    float *confidence,
    float innovation,
    float scale,
    float rate,
    float minimum)
{
    float error =
        fabsf(innovation) / scale;


    error =
        Clamp(error, 0.0f, 1.0f);


    float target =
        1.0f - error;


    *confidence +=
        rate *
        (target - *confidence);



    *confidence =
        Clamp(
            *confidence,
            minimum,
            1.0f);
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

    PushHistory(
        obs,
        command_velocity,
        measured_velocity);



    /*
        Delay update only when command changes.
    */

    float command_change =
        fabsf(command_velocity -
              obs->previous_command);


    obs->previous_command =
        command_velocity;


    if (command_change >
        obs->command_excitation_threshold)
    {
        UpdateDelay(obs);
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



    float acceleration =
        obs->command_confidence *
        command_accel;



    acceleration +=
        obs->disturbance;



    obs->velocity +=
        acceleration *
        obs->sample_time;


    obs->position +=
        obs->velocity *
        obs->sample_time;



    obs->disturbance *=
        1.0f -
        obs->disturbance_decay *
        obs->sample_time;



    /**********************************************************************
        Measurement corrections
    **********************************************************************/

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


        UpdateConfidence(
            &obs->position_confidence,
            error,
            obs->max_position_error,
            obs->confidence_learning_rate *
            obs->sample_time,
            obs->min_position_confidence);



        obs->position +=
            obs->position_confidence *
            obs->position_gain *
            error;


        obs->velocity +=
            obs->position_confidence *
            obs->position_to_velocity_gain *
            error *
            obs->sample_time;
    }



    if (velocity_valid)
    {
        float error =
            measured_velocity -
            obs->velocity;


        error =
            Clamp(
                error,
                -obs->max_velocity_error,
                 obs->max_velocity_error);



        UpdateConfidence(
            &obs->velocity_confidence,
            error,
            obs->max_velocity_error,
            obs->confidence_learning_rate *
            obs->sample_time,
            obs->min_velocity_confidence);



        obs->velocity +=
            obs->velocity_confidence *
            obs->velocity_gain *
            error;



        obs->disturbance +=
            obs->disturbance_gain *
            error *
            obs->sample_time;



        obs->disturbance =
            Clamp(
                obs->disturbance,
                -obs->max_disturbance,
                 obs->max_disturbance);



        /*
            Command trust adapts from the
            same evidence.
        */

        UpdateConfidence(
            &obs->command_confidence,
            error,
            obs->max_velocity_error,
            obs->confidence_learning_rate *
            obs->sample_time,
            obs->min_command_confidence);
    }
}