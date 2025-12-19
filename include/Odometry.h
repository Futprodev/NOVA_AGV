#pragma once
#include <Arduino.h>
#include <math.h>
#include "Config.h"

struct OdometryState {
  float x;
  float y;
  float theta; // rad heading
  float totalDistance; // m distance travelled
};

inline void initOdometry(OdometryState &odom) {
  odom.x = 0.0f;
  odom.y = 0.0f;
  odom.theta = 0.0f;
  odom.totalDistance = 0.0f;
}

inline void updateOdometry(OdometryState &odom, float dL, float dR, float baseWidth)
{
  float dCenter = 0.5f * (dL + dR);
  float dTheta  = (dR - dL) / baseWidth;

  float theta_mid = odom.theta + 0.5f * dTheta;

  odom.x += dCenter * cosf(theta_mid);
  odom.y += dCenter * sinf(theta_mid);

  odom.theta += dTheta;
  if (odom.theta > PI)       odom.theta -= 2.0f * PI;
  else if (odom.theta < -PI) odom.theta += 2.0f * PI;

  odom.totalDistance += fabsf(dCenter);
}