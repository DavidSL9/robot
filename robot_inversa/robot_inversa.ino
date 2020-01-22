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

#define L1 9.3f
#define L2 8.2f

HardwareDynamixelInterface interface(Serial1);

DynamixelMotor motor1(interface, MOTOR_1_ID);
DynamixelMotor motor2(interface, MOTOR_2_ID);

void m1SetAngle(float angle);
void m2SetAngle(float angle);
float m1GetAngle();
float m2GetAngle();
void inverse(float px, float py, float l1, float l2, float* q1, float* q2);
void path(float t, float* px, float* py);

float px, py, q1, q2;

float* Q1;
float* Q2;
uint32_t steps;
unsigned long uDelay = 50000;

void setup() {
  Serial.begin(115200);
  interface.begin(BR);
  
  uint8_t status;
  
  // Motor 1  /////////////////
  status = motor1.init();
  if(status != DYN_STATUS_OK)
  {
    Serial.println("Motor 1 error");
    while(1);
  }
  motor1.jointMode(50, 973);
  motor1.speed(INIT_SPEED);
//  motor1.enableTorque(true);
  delay(1000);
  ///////////////////////////
  
  // Motor 2  /////////////////
  status = motor2.init();
  if(status != DYN_STATUS_OK)
  {
    Serial.println("Motor 2 error");
    while(1);
  }
  motor2.jointMode(50, 973);
  motor2.speed(INIT_SPEED);    
//  motor2.enableTorque(true);                                                                                                                                                           .enableTorque();
  delay(1000);
  ///////////////////////////

  // Set Initial position
  path(0, &px, &py);
  inverse(px, py, L1, L2, &q1, &q2);
  m1SetAngle(q1);
  m2SetAngle(q2);
  float tolerance = 0.05f;
  while(
    abs(q1 - m1GetAngle()) > tolerance ||
    abs(q2 - m2GetAngle()) > tolerance);
  motor1.speed(SPEED);
  motor2.speed(SPEED);

  float sDelay = uDelay * 0.000001f;
  float goal = 2 * PI;
  float dt = goal * sDelay / 15, t = 0, q1, q2;
  steps = goal / dt;
  Q1 = (float*)malloc(steps * sizeof(float));
  Q2 = (float*)malloc(steps * sizeof(float));
  for (int i = 0; i < steps; i++) {
    path(t, &px, &py);
    inverse(px, py, L1, L2, &q1, &q2);
    Q1[i] = q1;
    Q2[i] = q2;
    t += dt;
  }
}

void loop() {
  static unsigned long total, past;
  static uint32_t count = 0;

  past = micros();

  float q1, q2;
  q1 = Q1[count];
  q2 = Q2[count];
  m1SetAngle(q1);
  m2SetAngle(q2);
  
  float tol = 0.05f;
  while(abs(m1GetAngle() - q1) > tol || abs(m2GetAngle() - q2) > tol);

  while( (total = micros() - past) < uDelay );
  Serial.print(q1, 6);
  Serial.print(" ");
  Serial.print(q2, 6);
  Serial.print(" ");
  Serial.print(m1GetAngle(), 6);
  Serial.print(" ");
  Serial.print(m2GetAngle(), 6);
  Serial.println();
  
  if (count++ == steps - 1) while(1); // final position reached
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

void inverse(float px, float py, float l1, float l2, float* q1, float* q2) {
  *q2 = -(PI - acosf((l1*l1 + l2*l2 - px*px - py*py) / 2 / l1 / l2));
  *q1 = atan2f(py, px) - atan2f(l2 * sinf(*q2), l1 + l2 * cosf(*q2));
}

void path(float t, float* px, float* py) {
  *px = L1 + 2 + 4 * cosf(PI + t);
  *py = L2 - 3 + 4 * sinf(PI + t);
}
