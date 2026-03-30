#include <algorithm>

namespace control
{

double pid_update(
  const double error,
  const double dt,
  const double kp,
  const double ki,
  const double kd,
  const double i_min,
  const double i_max,
  double & integral,
  double & prev_error)
{
  if (dt <= 1e-6) {
    return kp * error;
  }

  integral += error * dt;
  integral = std::clamp(integral, i_min, i_max);

  const double derivative = (error - prev_error) / dt;
  prev_error = error;

  return (kp * error) + (ki * integral) + (kd * derivative);
}

}  // namespace control
