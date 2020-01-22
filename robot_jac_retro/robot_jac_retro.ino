#include <DynamixelMotor.h>

#define BR 1000000
#define RAD2VALUE 195.37815f
#define VALUE2RAD 0.0051182f
#define MOTOR_1_ID 6
#define MOTOR_2_ID 4
#define OFFSET_M1 1.0472f
#define OFFSET_M2 2.7052f
#define INIT_SPEED 32
#define SPEED 32
#define TOLERANCE 0.001f
#define LR 0.001f

#define L1 9.3f
#define L2 8.2f

HardwareDynamixelInterface interface(Serial1);

DynamixelMotor motor1(interface, MOTOR_1_ID);
DynamixelMotor motor2(interface, MOTOR_2_ID);

void m1SetAngle(float angle);
void m2SetAngle(float angle);
float m1GetAngle();
float m2GetAngle();
void path(float t, float* px, float* py);
void deltaPath(float t, float* dpx, float* dpy);
void direct(float q1, float q2, float l1, float l2, float* px, float* py);
void deltaAngles(float q1, float q2, float l1, float l2, float dpx, float dpy, float* dq1, float* dq2);
void deltaErrors(float px, float py, float q1, float q2, float l1, float l2, float* deq1, float* deq2);
void decreaseError(float t, float l1, float l2, float* q1, float* q2, float* error);

float q1 = 1.0f;
float q2 = -1.5f;
float dpx, dpy, dq1, dq2;

void setup() {
  Serial.begin(115200);
  interface.begin(BR);
  
  // Motor 1  /////////////////
  uint8_t status = motor1.init();
  if(status != DYN_STATUS_OK)
  {
    Serial.println("Motor 1 error");
    while(1);
  }
  motor1.enableTorque(); 
  motor1.jointMode(200, 823);
  motor1.speed(INIT_SPEED);
  delay(1000);
  ///////////////////////////
  
  // Motor 2  /////////////////
  status = motor2.init();
  if(status != DYN_STATUS_OK)
  {
    Serial.println("Motor 2 error");
    while(1);
  }
  motor2.enableTorque(); 
  motor2.jointMode(100, 923);
  motor2.speed(INIT_SPEED);
  delay(1000);
  ///////////////////////////

  // Initial position correction
  float error;
  decreaseError(0, L1, L2, &q1, &q2, &error);
  m1SetAngle(q1);
  m2SetAngle(q2);
  
  float tolerance = 0.1f;
  while(
    abs(q1 - m1GetAngle()) > tolerance ||
    abs(q2 - m2GetAngle()) > tolerance) {
//      Serial.print("q1: ");
//      Serial.print(q1);
//      Serial.print("\t q2: ");
//      Serial.print(q2);
//      Serial.println();
  }
  motor1.speed(SPEED);
  motor2.speed(SPEED);
}


void loop() {
  static unsigned long total, past, uDelay = 100000;
  static float sDelay = uDelay * 0.000001f;
  static float goal = 2 * PI;
  static float dt = goal * sDelay / 15, t = 0;
  
  past = micros();

  m1SetAngle(q1);
  m2SetAngle(q2);
  float tol = 0.03f;
  while(abs(m1GetAngle() - q1) > tol || abs(m2GetAngle() - q2) > tol);
  
  deltaPath(t, &dpx, &dpy);
  deltaAngles(q1, q2, L1, L2, dpx, dpy, &dq1, &dq2);
  
  q1 += dq1 * dt;
  q2 += dq2 * dt;
  t += dt;

  float error;
  decreaseError(t, L1, L2, &q1, &q2, &error);
  while ( (total = micros() - past) < uDelay );
  
//  Serial.print(" Time: ");
//  Serial.print(total);
//  Serial.print(" error: ");
//  Serial.print(error, 4);
//  Serial.print(" q1: ");
  Serial.print(q1);
//  Serial.print(" q2: ");
  Serial.print(" ");
  Serial.print(q2);
  Serial.print(" ");
  Serial.print(m1GetAngle());
  Serial.print(" ");
  Serial.print(m2GetAngle());
  Serial.println();
  if (t >= goal) while(1);
}

void m1SetAngle(float angle) {
  motor1.goalPosition((angle + OFFSET_M1) * RAD2VALUE);
}

void m2SetAngle(float angle) {
  motor2.goalPosition((angle + OFFSET_M2) * RAD2VALUE);
}

float m1GetAngle() {
  return motor1.currentPosition() * VALUE2RAD - OFFSET_M1;
}

float m2GetAngle() {
  return motor2.currentPosition() * VALUE2RAD - OFFSET_M2;
}

void path(float t, float* px, float* py) {
  *px = L1 + 2 + 4 * cosf(PI + t);
  *py = L2 - 3 + 4 * sinf(PI + t);
}

void deltaPath(float t, float* dpx, float* dpy) {
  *dpx = -4 * sinf(PI + t);
  *dpy = 4 * cosf(PI + t);
}

void direct(float q1, float q2, float l1, float l2, float* px, float* py) {
  float sum = q1 + q2;
  *px = l1 * cosf(q1) + l2 * cosf(sum);
  *py = l1 * sinf(q1) + l2 * sinf(sum);
}

void deltaAngles(float q1, float q2, float l1, float l2, float dpx, float dpy, float* dq1, float* dq2) {
  float l1cq1 = l1 * cosf(q1);
  float l1sq1 = l1 * sinf(q1);
  float l1sq2 = l1 * sinf(q2);
  float l1l2sq2 = l2 * l1sq2;
  float cq1q2 = cosf(q1 + q2);
  float sq1q2 = sinf(q1 + q2);

  *dq1 = (cq1q2 * dpx + sq1q2 * dpy) / l1sq2;
  *dq2 = ((-l1cq1 - l2 * cq1q2) * dpx + (-l1sq1 - l2 * sq1q2) * dpy) / l1l2sq2;
}

void deltaErrors(float px, float py, float q1, float q2, float l1, float l2, float* deq1, float* deq2) {
  float l1cq1   = l1 * cosf(q1);
  float l2cq1q2 = l2 * cosf(q1 + q2);
  float l1sq1   = l1 * sinf(q1);
  float l2sq1q2 = l2 * sinf(q1 + q2);

  float t1 = (px - l1cq1 - l2cq1q2);
  float t2 = (py - l1sq1 - l2sq1q2);

  *deq1 = t1 * (l1sq1 + l2sq1q2) - t2 * (l1cq1 + l2cq1q2);
  *deq2 = t1 * l2sq1q2 - t2 * l2cq1q2;
}

void decreaseError(float t, float l1, float l2, float* q1, float* q2, float* error) {
  float px, py, rpx, rpy, deq1, deq2;
  path(t, &px, &py);
  while( 1 ) {
    direct(*q1, *q2, l1, l2, &rpx, &rpy);
    *error = pow(px - rpx, 2) + pow(py - rpy, 2);
    if (*error < TOLERANCE) {
      break;
    }
    deltaErrors(px, py, *q1, *q2, l1, l2, &deq1, &deq2);
    *q1 -= deq1 * LR;
    *q2 -= deq2 * LR;
  }
}
