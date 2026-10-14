#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <WatchdogManager.h>
#include <iWatchdog.h>
// #include "algo.h"
#include "firmware/include/state_machine.h"
#include "firmware/include/fc_config.h"
#include "firmware/include/hal.h"
// 

// ---------------------------------------------------------------------------
// Hardware Pins & Serial Instances (STM32F411CE Black Pill Mappings)
// ---------------------------------------------------------------------------

float liveGpsLat = 0.0f; // Latitude (decimal degrees)
float liveGpsLon = 0.0f; // Longitude (decimal degrees)

const uint32_t PYRO1_PIN = PB_15;
const uint32_t PYRO2_PIN = PB_13;

const uint32_t DEPLOYED_PIN1 = PB_15;
const uint32_t DEPLOYED_PIN2 = PB_13;

const uint32_t MCU_PIN1= PB_15;
const uint32_t MCU_PIN2 = PB_13;

bool DROUGE_LAUNCHED = false;
bool PARACUTE_LAUNCHED = false;

unsigned long time_drouge ;
unsigned long time_paracute ;
// ----------------------------------------------------------------------------
float accel_navS[3] = {0.0f, 0.0f, 0.0f};
float accelMagnitude = 0.0f ;
float liveGpsAlt = 0.0f; // New variable for altitude (in meters)

// MPU-9250 (I2C1)
const uint8_t MPU_ADDR = 0x69; 
const uint32_t SDA_PIN  = PB7;    
const uint32_t SCL_PIN  = PB6;    

// BMP388 (Hardware SPI1 Mappings)
#define BMP_CS   PA4
#define BMP_SCK  PA5
#define BMP_MISO PA6  
#define BMP_MOSI PA7


Adafruit_BMP3XX bmp;

// XBee Serial Mapping (USART2) - Updated to PA3 (RX) and PA2 (TX)
HardwareSerial XBeeSerial(PA3, PA2); 


// //------------------------------------------------------------
// //---------configure gpio as usart {for backup fc}------------
// //------------------------------------------------------------

#define INTERRUPTD0 PB0
#define INTERRUPTD1 PB1

// void InterruptD1() {
//     if(digitalRead(MCU_PIN2) == LOW) && PARACUTE_LAUNCHED ){
//         digitalWrite(INTERRUPTD1, HIGH); 
//     }
// }

// void InterruptD0() {
//     if(digitalRead(MCU_PIN1) == LOW) && DROUGE_LAUNCHED){
//         digitalWrite(INTERRUPTD0, HIGH); 
//     }
// }

// ---------------------------------------------------------------------------
// EKF Dimensions & Tuning Parameters
// ---------------------------------------------------------------------------
#define Ns 6   
#define Nm 2   

#define Q_ATT_DIAG     0.0005f  
#define Q_VEL_DIAG     0.01f    
#define R_ACC_DIAG     0.05f    
#define R_GPS_DIAG     0.02f    

#define CAL_SAMPLES    500
#define GYRO_DEADBAND  0.005f
#define LOOP_MS        20        // 50Hz Core EKF execution loop
#define XBEE_TX_MS     100       // 10Hz Downsampled XBee Telemetry Frame
#define GIMBAL_GUARD_RAD  (88.0f * DEG_TO_RAD)
#define GRAVITY        9.81f

// ---------------------------------------------------------------------------
// Sensor & Navigation Variables
// ---------------------------------------------------------------------------
float alphaAlt = 0.2f;       
float maxJumpAlt = 1.5f;     
float altitudeFiltered = 0.0f;
float altitudeZeroOffset = 0.0f;
float launchPressure_hPa = 1013.25f; 

// Extracted GPS Live Velocity Storage
float liveGpsVel[3] = {0.0f, 0.0f, 0.0f};
bool newGpsDataAvailable = false;
String gpsBuffer = "";

// ---------------------------------------------------------------------------
// Global Filter State Arrays
// ---------------------------------------------------------------------------
float gyroBias[3] = {0, 0, 0};   
float accelBias[3] = {0, 0, 0};  

float x[6] = {0, 0, 0, 0, 0, 0}; // State Vector: [roll, pitch, yaw, vx, vy, vz]
float P[6][6];
float Q_mat[6][6];
float R_acc[2][2];
float R_gps[3][3];

unsigned long lastTime = 0;
unsigned long lastGpsTime = 0; 
unsigned long lastXBeeTxTime = 0; 

// ---------------------------------------------------------------------------
// Packed Telemetry Structural Payload (35 Bytes)
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
#pragma pack(push, 1)
struct TestPayload {
    uint8_t packetType = 0x01; // 1 byte
    uint16_t packetID;         // 2 bytes
    
    // Core EKF State Vector Fields (4 bytes per float)
    float roll;
    float pitch;
    float yaw;
    float vx;
    float vy;
    float vz;

    // Environmental Barometer Metrics (4 bytes per float)
    float pressure;
    float alt;

    // GPS Telemetry Data (4 bytes per float)
    float latitude;
    float longitude;
    float gpsAlt;
};
// #pragma pack(pop)
#pragma pack(pop)

uint16_t currentPacketID = 0;


// ---------------------------------------------------------------------------
// Math & Filter Utilities
// ---------------------------------------------------------------------------
float filterAltitude(float raw) {
    float diff = raw - altitudeFiltered;
    if (fabsf(diff) > maxJumpAlt) {
        raw = altitudeFiltered + (diff > 0 ? maxJumpAlt : -maxJumpAlt);
    }
    altitudeFiltered = alphaAlt * raw + (1.0f - alphaAlt) * altitudeFiltered;
    return altitudeFiltered;
}

void matMult(int rowsA, int colsA, int colsB, const float* A, const float* B, float* C) {
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            float sum = 0.0f;
            for (int k = 0; k < colsA; k++) {
                sum += A[i * colsA + k] * B[k * colsB + j];
            }
            C[i * colsB + j] = sum;
        }
    }
}

void matMultTransB(int rowsA, int colsA, int rowsB, const float* A, const float* B, float* C) {
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < rowsB; j++) {
            float sum = 0.0f;
            for (int k = 0; k < colsA; k++) {
                sum += A[i * colsA + k] * B[j * colsA + k];
            }
            C[i * rowsB + j] = sum;
        }
    }
}

static inline void mat2Inv(const float A[2][2], float inv[2][2]) {
    float det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    if (fabsf(det) < 1e-9f) det = 1e-9f;
    float id = 1.0f/det;
    inv[0][0]= A[1][1]*id; inv[0][1]=-A[0][1]*id;
    inv[1][0]=-A[1][0]*id; inv[1][1]= A[0][0]*id;
}

bool mat3Inv(const float A[3][3], float inv[3][3]) {

    // 1. Calculate the determinant using standard cofactor expansion along the first row

    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -

                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +

                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

               

    // Protect against non-invertible/singular matrices

    if (fabsf(det) < 1e-9f) return false;

   

    float id = 1.0f / det;

   

    // 2. Compute the adjugate matrix (transpose of the cofactor matrix) multiplied by 1/det

    inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * id;

    inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * id;

    inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * id;

   

    inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * id;

    inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * id;

    inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * id; // Corrected index combinations

   

    inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * id;

    inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * id;

    inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * id;

   

    return true;

}

static inline float wrapPi(float a) {
    while(a > PI) a -= 2*PI; while(a < -PI) a += 2*PI; return a;
}

static inline float db(float v, float t) {
    return fabsf(v) < t ? 0.0f : v;
}

// ---------------------------------------------------------------------------
// Hardware Initializations & Calibrations
// ---------------------------------------------------------------------------
bool readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    if(Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);
    if(Wire.available() < 14) return false;
    
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
    return true;
}

void autoCalibrateSensors() {
    Serial.println(F("--- STARTING SENSOR CALIBRATION (KEEP STILL) ---"));
    
    float sumPressure = 0;
    int validPressureSamples = 0;
    for (int i = 0; i < 100; i++) {
        if (bmp.performReading()) {
            sumPressure += bmp.pressure;
            validPressureSamples++;
        }
        delay(30);
    }
    if (validPressureSamples > 10) {
        launchPressure_hPa = (sumPressure / validPressureSamples) / 100.0f;
    }

    float altSum = 0;
    int validAltSamples = 0;
    for (int i = 0; i < 150; i++) {
        if (bmp.performReading()) {
            altSum += bmp.readAltitude(launchPressure_hPa);
            validAltSamples++;
        }
        delay(20);
    }
    altitudeZeroOffset = (validAltSamples > 0) ? (altSum / validAltSamples) : 0.0f;

    double sumAx=0, sumAy=0, sumAz=0;
    double sumGx=0, sumGy=0, sumGz=0;
    int count = 0;

    while(count < CAL_SAMPLES) {
        int16_t ax, ay, az, gx, gy, gz;
        if(!readRaw(ax, ay, az, gx, gy, gz)) { delay(5); continue; }
        sumAx += ax; sumAy += ay; sumAz += az;
        sumGx += gx; sumGy += gy; sumGz += gz;
        count++;
        delay(4);  
    }

    gyroBias[0] = (float)(sumGx / CAL_SAMPLES / 131.0) * DEG_TO_RAD;
    gyroBias[1] = (float)(sumGy / CAL_SAMPLES / 131.0) * DEG_TO_RAD;
    gyroBias[2] = (float)(sumGz / CAL_SAMPLES / 131.0) * DEG_TO_RAD;

    accelBias[0] = (float)(sumAx / CAL_SAMPLES / 16384.0);
    accelBias[1] = (float)(sumAy / CAL_SAMPLES / 16384.0);
    accelBias[2] = (float)(sumAz / CAL_SAMPLES / 16384.0) - 1.0f;

    Serial.println(F("--- CALIBRATION COMPLETE! SYSTEM READY ---"));
}

bool initSensorHardware() {
    bool mpuSuccess = true;
    bool bmpSuccess = true;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); 
    Wire.write(0x00); 
    if(Wire.endTransmission() != 0) {
        Serial.println("DIAGNOSTIC: [FAIL] MPU-9250 (I2C) is NOT responding!");
        mpuSuccess = false;
    } else {
        Serial.println("DIAGNOSTIC: [SUCCESS] MPU-9250 connected.");
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1B); Wire.write(0x00); 
        Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x1C); Wire.write(0x00); 
        Wire.endTransmission();
    }

    if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
        Serial.println("DIAGNOSTIC: [FAIL] BMP388 (SPI) is NOT responding!");
        bmpSuccess = false;
    } else {
        Serial.println("DIAGNOSTIC: [SUCCESS] BMP388 connected.");
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_31);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }

    return (mpuSuccess && bmpSuccess);
}
// ---------------------------------------------------------------------------
// Lightweight Inline NMEA GPS String Parser (With Altitude Support) {new}
// ---------------------------------------------------------------------------
// Helper function to convert NMEA (DDMM.MMMM) to standard Decimal Degrees (DD.DDDDDD)
float nmeaToDecimal(String rawVal, char dir) {
    if (rawVal.length() == 0) return 0.0f;
    float raw = rawVal.toFloat();
    int deg = (int)(raw / 100);
    float minutes = raw - (deg * 100);
    float decimal = deg + (minutes / 60.0f);
    if (dir == 'S' || dir == 'W') decimal = -decimal;
    return decimal;
}

void parseGpsSerial() {
    while (Serial1.available()) {
        char c = Serial1.read();
        
        if (c == '\n') {
            if (gpsBuffer.startsWith("$GPRMC") || gpsBuffer.startsWith("$GNRMC")) {
                int commaIdx[12];
                int idxCount = 0;
                for (int i = 0; i < gpsBuffer.length(); i++) {
                    if (gpsBuffer.charAt(i) == ',') {
                        commaIdx[idxCount++] = i;
                        if (idxCount >= 12) break;
                    }
                }
                
                if (idxCount >= 8 && gpsBuffer.charAt(commaIdx[1] + 1) == 'A') { 
                    // Extract Latitude & Longitude from RMC
                    String latStr = gpsBuffer.substring(commaIdx[2] + 1, commaIdx[3]);
                    char latDir   = gpsBuffer.charAt(commaIdx[3] + 1);
                    String lonStr = gpsBuffer.substring(commaIdx[4] + 1, commaIdx[5]);
                    char lonDir   = gpsBuffer.charAt(commaIdx[5] + 1);

                    liveGpsLat = nmeaToDecimal(latStr, latDir);
                    liveGpsLon = nmeaToDecimal(lonStr, lonDir);

                    // Extract Speed & Course
                    String speedStr  = gpsBuffer.substring(commaIdx[6] + 1, commaIdx[7]);
                    String courseStr = gpsBuffer.substring(commaIdx[7] + 1, commaIdx[8]);
                    
                    float speedKnot = speedStr.toFloat();
                    float courseDeg = courseStr.toFloat();
                    float speedMps  = speedKnot * 0.514444f; 
                    float courseRad = courseDeg * DEG_TO_RAD;

                    liveGpsVel[0] = speedMps * cosf(courseRad); // Vx
                    liveGpsVel[1] = speedMps * sinf(courseRad); // Vy
                    liveGpsVel[2] = 0.0f;                       // Vz 
                    newGpsDataAvailable = true;
                }
            }
            else if (gpsBuffer.startsWith("$GPGGA") || gpsBuffer.startsWith("$GNGGA")) {
                int commaIdx[15];
                int idxCount = 0;
                for (int i = 0; i < gpsBuffer.length(); i++) {
                    if (gpsBuffer.charAt(i) == ',') {
                        commaIdx[idxCount++] = i;
                        if (idxCount >= 15) break;
                    }
                }
                if (idxCount >= 10 && gpsBuffer.charAt(commaIdx[5] + 1) != '0') {
                    // Extract Latitude & Longitude from GGA if needed
                    String latStr = gpsBuffer.substring(commaIdx[1] + 1, commaIdx[2]);
                    char latDir   = gpsBuffer.charAt(commaIdx[2] + 1);
                    String lonStr = gpsBuffer.substring(commaIdx[3] + 1, commaIdx[4]);
                    char lonDir   = gpsBuffer.charAt(commaIdx[4] + 1);

                    liveGpsLat = nmeaToDecimal(latStr, latDir);
                    liveGpsLon = nmeaToDecimal(lonStr, lonDir);

                    String altStr = gpsBuffer.substring(commaIdx[8] + 1, commaIdx[9]);
                    liveGpsAlt = altStr.toFloat(); 
                    newGpsDataAvailable = true;
                }
            }
            
            gpsBuffer = ""; 
        } else if (c != '\r') {
            gpsBuffer += c;
        }
    }
}
// ---------------------------------------------------------------------------
// XBee API Frame Builder
// ---------------------------------------------------------------------------
void sendXBeeAPIPacket(uint8_t* payload, uint16_t payloadLen) {
    uint16_t length = 14 + payloadLen; 
    
    XBeeSerial.write(0x7E); // Start Byte
    XBeeSerial.write((length >> 8) & 0xFF); // Length MSB
    XBeeSerial.write(length & 0xFF);        // Length LSB
    
    uint8_t checksum = 0;
    auto writeByte = [&](uint8_t b) {
        XBeeSerial.write(b);
        checksum += b;
    };

    writeByte(0x10); // Transmit Request
    writeByte(0x00); // Frame ID (No status ACK response)
    
    // 64-bit Broadcast Mapping
    for(int i=0; i<6; i++) writeByte(0x00);
    writeByte(0xFF);
    writeByte(0xFF);
    
    // 16-bit Broadcast Mapping
    writeByte(0xFF);
    writeByte(0xFE);
    
    writeByte(0x00); // Radius
    writeByte(0x00); // Options

    // Write Payload Data
    for (uint16_t i = 0; i < payloadLen; i++) {
        writeByte(payload[i]);
    }
    
    // Checksum Append
    XBeeSerial.write(0xFF - checksum);
}

// ---------------------------------------------------------------------------
// EKF Processing Mathematical Core Matrix Functions
// ---------------------------------------------------------------------------
void ekfPredict(float p, float q, float r, float ax, float ay, float az, float dt) {
    float phi   = x[0];
    float theta = constrain(x[1], -GIMBAL_GUARD_RAD, GIMBAL_GUARD_RAD);
    float psi   = x[2];
   
    float sp = sinf(phi), cp = cosf(phi);
    float tt = tanf(theta), ct = cosf(theta);
    if(fabsf(ct) < 1e-4f) ct = (ct >= 0) ? 1e-4f : -1e-4f;

    float qsp_rcp = q*sp + r*cp;
    float qcp_rsp = q*cp - r*sp;

    x[0] = wrapPi(x[0] + (p + qsp_rcp*tt)   * dt);
    x[1] = constrain(wrapPi(x[1] + qcp_rsp * dt), -GIMBAL_GUARD_RAD, GIMBAL_GUARD_RAD);
    x[2] = wrapPi(x[2] + (qsp_rcp/ct)       * dt);

    float ct_ = cosf(theta);
    float spsi = sinf(psi), cpsi = cosf(psi);

    float R_b2n[3][3] = {
        {ct_*cpsi, sp*tanf(x[1])*ct_*cpsi - cp*spsi, cp*tanf(x[1])*ct_*cpsi + sp*spsi}, 
        {ct_*spsi, sp*tanf(x[1])*ct_*spsi + cp*cpsi, cp*tanf(x[1])*ct_*spsi - sp*cpsi},
        {-sinf(x[1]), sp*ct_, cp*ct_}
    };

    float accel_body_mps2[3] = { ax * GRAVITY, ay * GRAVITY, az * GRAVITY };
    float accel_nav[3] = {0.0f, 0.0f, 0.0f};
   
    for(int i=0; i<3; i++) {
        accel_nav[i] = R_b2n[i][0]*accel_body_mps2[0] +
                       R_b2n[i][1]*accel_body_mps2[1] +
                       R_b2n[i][2]*accel_body_mps2[2];
    }
    accel_navS[0] = accel_nav[0];
    accel_navS[1] = accel_nav[1];
    accel_nav[2] -= GRAVITY; 
    accel_navS[2] = accel_nav[2];
   

    x[3] += accel_nav[0] * dt;
    x[4] += accel_nav[1] * dt;
    x[5] += accel_nav[2] * dt;

    float F[6][6];
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) F[i][j] = (i == j) ? 1.0f : 0.0f;
    }
   
    float ct2 = ct*ct;
    F[0][0] = 1.0f + dt * qcp_rsp * tt;
    F[0][1] = dt * qsp_rcp / ct2;
    F[1][0] = -dt * qsp_rcp;
    F[1][1] = 1.0f;
    F[2][0] = dt * qcp_rsp / ct;
    F[2][1] = dt * qsp_rcp * tt / ct;

    float FP[6][6];
    float FPFt[6][6];
    matMult(6, 6, 6, (float*)F, (float*)P, (float*)FP);
    matMultTransB(6, 6, 6, (float*)FP, (float*)F, (float*)FPFt);
   
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            P[i][j] = FPFt[i][j] + Q_mat[i][j];
        }
    }
}

void ekfUpdateIMU(float phi_acc, float theta_acc) {
    float y[2] = { wrapPi(phi_acc - x[0]), wrapPi(theta_acc - x[1]) };
    float S[2][2] = {
        {P[0][0]+R_acc[0][0], P[0][1]+R_acc[0][1]},
        {P[1][0]+R_acc[1][0], P[1][1]+R_acc[1][1]}
    };
   
    float Si[2][2];
    mat2Inv(S, Si);
   
    float K[6][2];
    for(int i=0; i<6; i++) {
        for(int j=0; j<2; j++) {
            K[i][j] = P[i][0]*Si[0][j] + P[i][1]*Si[1][j];
        }
    }
   
    for(int i=0; i<6; i++) {
        x[i] += K[i][0]*y[0] + K[i][1]*y[1];
    }
   
    x[0] = wrapPi(x[0]);
    x[1] = constrain(wrapPi(x[1]), -GIMBAL_GUARD_RAD, GIMBAL_GUARD_RAD);
    x[2] = wrapPi(x[2]);

    float Pn[6][6];
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            Pn[i][j] = P[i][j] - (K[i][0]*P[0][j] + K[i][1]*P[1][j]);
        }
    }
   
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
        }
        if(P[i][i] < 1e-6f) P[i][i] = 1e-6f;
    }
}

void ekfUpdateGPS(float vel_gps[3]) {
    float y[3] = { vel_gps[0] - x[3], vel_gps[1] - x[4], vel_gps[2] - x[5] };
    float S[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) S[i][j] = P[3 + i][3 + j] + R_gps[i][j];
    }

    float Si[3][3];
    if(!mat3Inv(S, Si)) return; 

    float K[6][3];
    for(int i=0; i<6; i++) {
        for(int j=0; j<3; j++) {
            K[i][j] = P[i][3]*Si[0][j] + P[i][4]*Si[1][j] + P[i][5]*Si[2][j];
        }
    }

    for(int i=0; i<6; i++) {
        x[i] += K[i][0]*y[0] + K[i][1]*y[1] + K[i][2]*y[2];
    }

    x[0] = wrapPi(x[0]);
    x[1] = constrain(wrapPi(x[1]), -GIMBAL_GUARD_RAD, GIMBAL_GUARD_RAD);
    x[2] = wrapPi(x[2]);

    float Pn[6][6];
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            Pn[i][j] = P[i][j] - (K[i][0]*P[3][j] + K[i][1]*P[4][j] + K[i][2]*P[5][j]);
        }
    }

    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);
        }
        if(P[i][i] < 1e-6f) P[i][i] = 1e-6f;
    }
}
// ---------------------------------------------------------------------------
// HAL Interface Implementations
// ---------------------------------------------------------------------------
class Sensors : public fc::ISensors {
public:
    bool read(fc::SensorSample& s) override {
        s.t = millis() / 1000.0f;
        s.alt_agl = altitudeFiltered;
        s.az =accel_navS[2] ;
        s.accel_mag = accelMagnitude;
        s.gps_alt = liveGpsAlt;
        s.gps_vz = liveGpsVel[2];
        s.gps_valid =  (millis() - lastGpsTime) < 1000;;
        s.ekf_vz = x[5];
        s.ekf_valid = true;
        Serial.println("[FC] SensorSample updated.");
        return true;
      
    }
};

class Outputs : public fc::IOutputs {
public:
    void fire_eject_drogue() override {
        digitalWrite(PYRO1_PIN,HIGH);
        delay(100);
        digitalWrite(PYRO1_PIN,LOW);
          Serial.println("drouge fires ");
        time_drouge = millis();
        
    }
    void fire_main() override {
        digitalWrite(PYRO2_PIN,HIGH);
        delay(100);
        digitalWrite(PYRO2_PIN,LOW);
          Serial.println("firemain");
        time_paracute = millis();
    }
    void beacon(const fc::SensorSample& s) override {
        // send GPS coordinates
        Serial.println("beacon ");
    }
};
Sensors sensors;
Outputs outputs;
fc::FlightComputer flightComputer(fc::default_params());
// ---------------------------------------------------------------------------
// Setup Function
// ---------------------------------------------------------------------------
void setup() {
    //  uartBegin();

    // 1. Force LED configuration immediately
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH); // Turn off initially (Active LOW)
    
    // 2. Hardware Serial Initializations
    // Serial.begin(115200); 
    
    // // Give the native USB serial monitor up to 2 seconds to open on PC, then bypass
    // uint32_t serialStart = millis();
    // while (!Serial && (millis() - serialStart < 2000)) {
    //     delay(10); 
    // }

    // Serial.println("Black Pill Booting...");
    
    // Remap Pre-defined GPS Hardware Serial Pins (UART1)
    Serial1.setRx(PA9);
    Serial1.setTx(PA10);
    Serial1.begin(9600); 
    
    // Remap XBee Wireless Hardware Serial Pins (UART2)
    XBeeSerial.begin(115200);

    // 3. SAFE I2C Initialization Sequence
    pinMode(PB6, OUTPUT);
    pinMode(PB7, OUTPUT);
    digitalWrite(PB6, HIGH); // Force bus lines high to clear stuck states
    digitalWrite(PB7, HIGH);
    delay(50);

    Wire.setSDA(SDA_PIN); 
    delay(10);
    Wire.setSCL(SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    // 4. Non-blocking Sensor Hardware Init Check
   while(!initSensorHardware()) {
        Serial.println("ERR: Hardware Initialization Failed! Checking Pins...");
        delay(1000); 
   }
    // Initialize Identity Estimation Matrices
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            P[i][j] = (i == j) ? 0.1f : 0.0f;
            Q_mat[i][j] = (i == j) ? ((i < 3) ? Q_ATT_DIAG : Q_VEL_DIAG) : 0.0f;
        }
    }
    for(int i=0; i<2; i++) {
        for(int j=0; j<2; j++) R_acc[i][j] = (i == j) ? R_ACC_DIAG : 0.0f;
    }
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) R_gps[i][j] = (i == j) ? R_GPS_DIAG : 0.0f;
    }

    autoCalibrateSensors();
    
    pinMode(INTERRUPTD1, OUTPUT);
    // digitalWrite(TX_PIN, HIGH); 
    pinMode(PYRO1_PIN, OUTPUT);
    pinMode(PYRO2_PIN, OUTPUT);
    digitalWrite(PYRO1_PIN, LOW);
    digitalWrite(PYRO2_PIN, LOW);

    lastTime = millis();
    lastGpsTime = millis();
    lastXBeeTxTime = millis();
}
// ---------------------------------------------------------------------------
// Main Loop — Runs 50Hz Core Execution / 10Hz Downsampled Output
// ---------------------------------------------------------------------------
void loop() {
   unsigned long now = millis();
    float dt = (now - lastTime) * 1e-3f;
    
    // FIXED: Prevent initial loop crash due to 0ms delta time
    if(dt < 0.001f) { 
        delay(1); // Force a minimum 1ms step if running too fast
        return; 
    }
    if(dt > 0.5f) { 
        lastTime = now; 
        return; 
    }
    lastTime = now;

    
    // if((now - time_drouge )*1e-3f > 1.0f ){
    //      InterruptD0();
    // }
    // // time_paracute
    // if((now - time_paracute )*1e-3f > 1.0f ){
    //      InterruptD1();
    // }
    // Blink PC13 to prove the main loop code execution is actually alive
    
  
    // 1. Ingest asynchronous serial GPS packets
    parseGpsSerial();

    // 2. Transact IMU readings via I2C
    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    if(!readRaw(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz)) {
        delay(1); return;
    }

    float ax = rawAx/16384.0f - accelBias[0];
    float ay = rawAy/16384.0f - accelBias[1];
    float az = rawAz/16384.0f - accelBias[2];
    float p  = db((rawGx/131.0f)*DEG_TO_RAD - gyroBias[0], GYRO_DEADBAND);
    float q  = db((rawGy/131.0f)*DEG_TO_RAD - gyroBias[1], GYRO_DEADBAND);
    float r  = db((rawGz/131.0f)*DEG_TO_RAD - gyroBias[2], GYRO_DEADBAND);

    float phi_acc   = atan2f(ay, az);
    float theta_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

    ekfPredict(p, q, r, ax, ay, az, dt);
    ekfUpdateIMU(phi_acc, theta_acc);

    // 3. Perform BMP388 Barometer Reading via SPI
    float filteredAlt = 0.0f;
    float currentPressure_hPa = 0.0f;
    if (bmp.performReading()) {
        currentPressure_hPa = bmp.pressure / 100.0f;
        float rawAlt = bmp.readAltitude(launchPressure_hPa) - altitudeZeroOffset;
        filteredAlt = filterAltitude(rawAlt);
        altitudeFiltered = filteredAlt;
        float delta = fabsf(rawAlt - altitudeFiltered);
        alphaAlt = (delta > 0.5f) ? 0.3f : 0.2f;
    }
// 4. Update state vector if new live GPS parsing completed
    if (newGpsDataAvailable) {
        ekfUpdateGPS(liveGpsVel);
        newGpsDataAvailable = false;
        lastGpsTime = millis();
    }

    // 5. Asynchronous Non-blocking Wireless XBee Transmission (10Hz Trigger)
 // 5. Asynchronous Non-blocking Wireless XBee Transmission (10Hz Trigger)
    if (millis() - lastXBeeTxTime >= XBEE_TX_MS) {
        lastXBeeTxTime = millis();

        TestPayload payload;
        payload.packetID = currentPacketID++;
        
        // Export structural estimations directly from EKF array index targets
        payload.roll      = x[0] * RAD_TO_DEG;
        payload.pitch     = x[1] * RAD_TO_DEG;
        payload.yaw       = x[2] * RAD_TO_DEG;
        payload.vx        = x[3];
        payload.vy        = x[4];
        payload.vz        = x[5];
        payload.pressure  = currentPressure_hPa;
        payload.alt       = filteredAlt;

        // Assign GPS Position Data
        payload.latitude  = liveGpsLat;
        payload.longitude = liveGpsLon;
        payload.gpsAlt    = liveGpsAlt;

        // Broadcast frame through XBee API pipeline
        sendXBeeAPIPacket((uint8_t*)&payload, sizeof(payload));
        
        // Exact requested confirmation string
        Serial.println("data has been sent");
    }

    // 6. Output Local Debug CSV Data Sequence
    Serial.print(x[0]*RAD_TO_DEG, 2); Serial.print(","); 
    Serial.print(x[1]*RAD_TO_DEG, 2); Serial.print(","); 
    Serial.print(x[2]*RAD_TO_DEG, 2); Serial.print(","); 
    Serial.print(x[3], 2);             Serial.print(","); 
    Serial.print(x[4], 2);             Serial.print(","); 
    Serial.print(x[5], 2);             Serial.print(","); 
    Serial.print(currentPressure_hPa, 2); Serial.print(","); 
    Serial.println(filteredAlt, 2); 

    // 7. Send heartbeat to backup fc (\n)
    // uartPrintln("H");
    digitalWrite(INTERRUPTD0, HIGH); 
    // 8. UPDATE THE SENSOR DATA 


    accelMagnitude =
    sqrtf(accel_navS[0]*accel_navS[0] +
          accel_navS[1]*accel_navS[1] +
          accel_navS[2]*accel_navS[2]);

    fc::SensorSample sample;

    sensors.read(sample);

    fc::Event ev = flightComputer.update(sample);

    switch (ev)
    {
    case fc::Event::EJECT_DROGUE:
        outputs.fire_eject_drogue();
        break;

    case fc::Event::MAIN:
        outputs.fire_main();
        break;

    default:
        break;
    }
    Serial.print("State: ");
    Serial.print((int)flightComputer.state());

    Serial.print(" Event: ");
    Serial.println((int)ev);
    outputs.beacon(sample);
    digitalWrite(PC13 , LOW);

    // Ensure processing step runs strictly at 50Hz frequency
    long el = (long)(millis()-now);
    if(el < LOOP_MS) delay(LOOP_MS - el);


}