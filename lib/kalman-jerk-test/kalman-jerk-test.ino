#include <Arduino.h>
#include "imxrt.h"
#include "TeensyTimerTool.h"
#include "./jerk/kalman_jerk.cpp"
using namespace TeensyTimerTool;

PeriodicTimer t1(GPT1);

double alpha = 0.000005;
double angular_resolution_error = 4.0;
double process_noise = 1.0;

kaepek::KalmanJerk1D filter = kaepek::KalmanJerk1D(alpha, angular_resolution_error, process_noise, true, 16384.0); // constructor with relative time and mod limit of 16384;
elapsedMicros micros_since_last_tick;
double pos = 0.0;
double iterations = 0.0;

void setup()
{
  micros_since_last_tick = 0;
  t1.begin(callback, 1'000'000);
}

void print_kalman(double *kalman_vec)
{
  Serial.print("Kalman vec: [");
  Serial.print(kalman_vec[0]);
  Serial.print(", ");
  Serial.print(kalman_vec[1]);
  Serial.print(", ");
  Serial.print(kalman_vec[2]);
  Serial.print(", ");
  Serial.print(kalman_vec[3]);
  Serial.print("]\n");
}

void print_kalman_flat(double *kalman_vec)
{
  Serial.print(kalman_vec[0]);
  Serial.print(",");
  Serial.print(kalman_vec[1]);
  Serial.print(",");
  Serial.print(kalman_vec[2]);
  Serial.print(",");
  Serial.print(kalman_vec[3]);
  Serial.print("\n");
}

void print_eular(double *eular_vec)
{
  Serial.print("Eular vec: [");
  Serial.print(eular_vec[0]);
  Serial.print(", ");
  Serial.print(eular_vec[1]);
  Serial.print(", ");
  Serial.print(eular_vec[2]);
  Serial.print(", ");
  Serial.print(eular_vec[3]);
  Serial.print(", ");
  Serial.print(eular_vec[4]);
  Serial.print("]\n");
}

void loop()
{
  double seconds_since_last = (double)micros_since_last_tick * (double) 1e-6;
  micros_since_last_tick = 0;
  filter.step(seconds_since_last, pos);
  double *kalman_vec = filter.get_kalman_vector();
  double *eular_vec = filter.get_eular_vector();
  //cli();
    //print_kalman_flat(kalman_vec);
    /*Serial.print("second_since_last: ");Serial.print(seconds_since_last); Serial.print("\n");
    Serial.print("micros_since_last_tick: ");Serial.print(micros_since_last_tick); Serial.print("\n");
    // micros_since_last_tick
    Serial.print("position: ");Serial.print(pos); Serial.print("\n");
    print_kalman(kalman_vec);
    print_eular(eular_vec);*/
  //sei();
  pos = pos + 1.0;
  iterations++;
}

void callback()
{
  Serial.print("Refresh rate: ");
  Serial.print(iterations);
  Serial.print("[hz]\n");
  iterations = 0.0;
}
