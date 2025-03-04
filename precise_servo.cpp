#include <micro_rosso_2dof_arm.h>

// Similar to map but will have increased accuracy that provides a more
// symmetrical api (call it and use result to reverse will provide the original value)
double improved_map(double value, double minIn, double maxIn, double minOut, double  maxOut) {
    const double rangeIn = maxIn - minIn;
    const double rangeOut = maxOut - minOut;
    const double deltaIn = value - minIn;
    // fixed point math constants to improve accuracy of divide and rounding
    constexpr int fixedHalfDecimal = 1.f;
    constexpr int fixedDecimal = fixedHalfDecimal * 2.f;

    return ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + minOut;
}

int Precise_Servo::begin(pin_size_t pin, servo_type type){
  _type = type;
  _minUs = DEFAULT_MIN_PULSE_WIDTH;
  _maxUs = DEFAULT_MAX_PULSE_WIDTH;
  return begin(pin, _minUs, _maxUs, type);
}

int Precise_Servo::begin(pin_size_t pin, int min, int max, servo_type type){
  _type = type;
  _minUs = min;
  _maxUs = max;
  return attach(pin, min, max, DEFAULT_NEUTRAL_PULSE_WIDTH);
}

void Precise_Servo::write(float value) {
  this->writeMicroseconds(improved_map(value, 0, (_type == PRECISE_SERVO_180deg) ? 180 : 270, _minUs, _maxUs));
}

