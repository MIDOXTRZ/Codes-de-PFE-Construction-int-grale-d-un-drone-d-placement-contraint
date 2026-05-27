//Code du drone avec MPU6050 integration et PID//
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include <Adafruit_INA219.h>

// =====================================================
// capteur batterie ina219
// =====================================================
// ici on va utiliser le capteur pour lire la tension de la batterie
Adafruit_INA219 ina219;

// =====================================================
// moteurs du drone
// =====================================================
// on declare les 4 esc (moteurs brushless)
Servo M1, M2, M3, M4;

// broches des moteurs
#define M1_PIN 18
#define M2_PIN 19
#define M3_PIN 13
#define M4_PIN 12

// =====================================================
// capteur mpu6050 (gyro + accel)
// =====================================================
// adresse i2c du capteur
const int MPU = 0x68;

// variables brutes du capteur
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;

// angles du drone (roll pitch)
float Roll = 0;
float Pitch = 0;

// temps pour calcul dt (important pour pid)
float dt;
unsigned long lastTime;

// =====================================================
// pid controller
// =====================================================
// ici c'est les gains du pid
float Kp = 7.0;
float Ki = 0.01;
float Kd = 0.5;

// erreurs roll
float errRoll, prevErrRoll = 0, integralRoll = 0;

// erreurs pitch
float errPitch, prevErrPitch = 0, integralPitch = 0;

// limite pour eviter bug integrale
float iLimit = 150;

// =====================================================
// esp now (communication sans wifi classique)
// =====================================================
// adresse de la telecommande (broadcast ici)
uint8_t adresse_telecommande[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// structure commande recu
typedef struct {
  int gaz;
  bool arme;
} DonneesCommande;

DonneesCommande commandes;

// structure telemetrie envoyee
typedef struct {
  float batterie;
} DonneesDrone;

DonneesDrone telemetrie;

// etat drone
bool armed = false;
int throttle = 1000;

// =====================================================
// reception des commandes
// =====================================================
// cette fonction est appelee automatiquement quand on recoit data
void sur_reception(const esp_now_recv_info *info, const uint8_t *data, int len) {

  // on copie les donnees recues
  memcpy(&commandes, data, sizeof(commandes));

  // mise a jour gaz
  throttle = commandes.gaz;

  // armement drone
  armed = commandes.arme;
}

// =====================================================
// setup
// =====================================================
void setup() {

  // start i2c bus
  Wire.begin();

  // init capteur batterie
  ina219.begin();
  ina219.setCalibration_32V_2A(); // calibration simple pour li-po

  // reveil du mpu6050 (sinon il reste en sleep)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // config moteurs (50hz esc standard)
  M1.setPeriodHertz(50);
  M2.setPeriodHertz(50);
  M3.setPeriodHertz(50);
  M4.setPeriodHertz(50);

  // attache des moteurs
  M1.attach(M1_PIN, 1000, 2000);
  M2.attach(M2_PIN, 1000, 2000);
  M3.attach(M3_PIN, 1000, 2000);
  M4.attach(M4_PIN, 1000, 2000);

  // on met tout a 0 pour securite
  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);

  delay(4000); // petit temps pour armer esc

  // wifi en mode station (obligatoire esp now)
  WiFi.mode(WIFI_STA);

  // init esp now
  if (esp_now_init() != ESP_OK) {
    return; // si erreur on stop
  }

  // callback reception
  esp_now_register_recv_cb(sur_reception);

  // ajout telecommande
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, adresse_telecommande, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);

  // debut timer
  lastTime = millis();
}

// =====================================================
// loop principal
// =====================================================
void loop() {

  // calcul dt (temps entre 2 boucles)
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // lecture mpu6050
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  AccX = Wire.read() << 8 | Wire.read();
  AccY = Wire.read() << 8 | Wire.read();
  AccZ = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // skip temp

  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();

  // transformation axes (drone depend orientation)
  float Ax = AccZ;
  float Ay = AccY;
  float Az = -AccX;

  float Gx = GyroZ / 131.0;
  float Gy = GyroY / 131.0;

  // calcul angles acc
  float accRoll = atan(Ay / sqrt(Ax * Ax + Az * Az)) * 180 / PI;
  float accPitch = atan(-Ax / sqrt(Ay * Ay + Az * Az)) * 180 / PI;

  // filtre complementaire (simple mais efficace)
  Roll = 0.98 * (Roll + Gx * dt) + 0.02 * accRoll;
  Pitch = 0.98 * (Pitch + Gy * dt) + 0.02 * accPitch;

  // =====================================================
  // pid roll
  // =====================================================
  errRoll = -Roll;
  integralRoll += errRoll * dt;

  // anti windup sinon ca part en vrille
  integralRoll = constrain(integralRoll, -iLimit, iLimit);

  float dRoll = (errRoll - prevErrRoll) / dt;
  prevErrRoll = errRoll;

  float rollPID = Kp * errRoll + Ki * integralRoll + Kd * dRoll;

  // =====================================================
  // pid pitch
  // =====================================================
  errPitch = -Pitch;
  integralPitch += errPitch * dt;
  integralPitch = constrain(integralPitch, -iLimit, iLimit);

  float dPitch = (errPitch - prevErrPitch) / dt;
  prevErrPitch = errPitch;

  float pitchPID = Kp * errPitch + Ki * integralPitch + Kd * dPitch;

  // moteurs init
  int m1 = 1000, m2 = 1000, m3 = 1000, m4 = 1000;

  // mixage moteurs (classic quad x)
  if (throttle > 1050) {

    m1 = throttle - pitchPID + rollPID;
    m2 = throttle - pitchPID - rollPID;
    m3 = throttle + pitchPID - rollPID;
    m4 = throttle + pitchPID + rollPID;

    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);
  }

  // envoi moteurs si arme
  if (armed) {
    M1.writeMicroseconds(m1);
    M2.writeMicroseconds(m2);
    M3.writeMicroseconds(m3);
    M4.writeMicroseconds(m4);
  } else {
    // sinon arret total
    M1.writeMicroseconds(1000);
    M2.writeMicroseconds(1000);
    M3.writeMicroseconds(1000);
    M4.writeMicroseconds(1000);
  }

  // =====================================================
  // batterie via ina219
  // =====================================================
  float tension = ina219.getBusVoltage_V();

  float percent = map(tension * 100, 900, 1260, 0, 100);
  percent = constrain(percent, 0, 100);

  telemetrie.batterie = percent;

  // envoi telemetrie au controleur
  esp_now_send(adresse_telecommande, (uint8_t *)&telemetrie, sizeof(telemetrie));

  // petite pause (loop stable)
  delay(4);
}
