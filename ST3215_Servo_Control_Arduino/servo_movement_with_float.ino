#include <SCServo.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
// SSD1306: 0x3C
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels, 32 as default.
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === GLOBAL VARIABLES ===
#define MAX_ID 5
#define S_RXD 18
#define S_TXD 19
SMS_STS st;
#define ServoSpeed 250
#define ServoAcc 5

#define BaseMotorSpeed 500
#define BaseMotorAcc 10


// Feedback storage
s16 posRead[MAX_ID + 1];
s16 speedRead[MAX_ID + 1];
float MiddlePosition = 180; 
float FullBaseRotation = 1355.294;

// Constants for the movement
#define BASE_MOTOR 0
#define SHOULDER_MOTOR_1 1
#define SHOULDER_MOTOR_2 2
#define ELBOW_MOTOR 3
#define WRIST_MOTOR 4
#define GRIPPER_MOTOR 5
#define MAX_ID 5



// Define the receiver's MAC address
uint8_t receiverAdd[] = {0xE8, 0x6B, 0xEA, 0xF6, 0x73, 0xF8};
esp_now_peer_info_t peerInfo;

// Callback function for when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  displayMessage(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void InitServos(){
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
   // Configuring ESP NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    displayMessage("Error initializing ESP-NOW");
    return;
  }
  st.pSerial = &Serial1;
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, receiverAdd, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  while(!Serial1) {}
  for (byte id = 0; id <= MAX_ID; id++) {
//    if(id==0){
//      setMotorMode(id,3);
//      }else{
//        setMotorMode(id,0);
//       }
    st.EnableTorque(id, 1);
    delay(10);
  }};

void setAsMiddlePosition(byte InputID){
  st.CalibrationOfs(InputID);
  displayMessage("Middle position for all motors set!");
}

void setMiddleForAllMotors() {
  for (byte id = 0; id <= MAX_ID; id++) {
    setAsMiddlePosition(id);  // Set middle position for each motor
  }
  displayMessage("Middle position set!");
}



void setMotorMode(byte InputID, byte InputMode) {
  st.unLockEprom(InputID);
  if (InputMode == 0) {
    st.writeWord(InputID, 11, 4095); // Set maximum range for servo mode
    st.writeByte(InputID, SMS_STS_MODE, InputMode);
  } 
  else if (InputMode == 3) { // DC mode for continuous rotation
    st.writeByte(InputID, SMS_STS_MODE, InputMode);
    st.writeWord(InputID, 11, 0); // No position limit in DC mode
  }
  st.LockEprom(InputID);
}


void InitScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
}

void displayMessage(const char* message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); // Top-left corner
  display.println(message);
  display.display();
}

void servoStop(byte servoID){
  st.EnableTorque(servoID, 0);
  delay(10);
  st.EnableTorque(servoID, 1);
}

float calculateTargetPosition(int angle) {
    float pulses = (angle / 360.0) * 4095.0;
    return pulses;
}


void servoTorque(byte servoID, u8 enableCMD){
  st.EnableTorque(servoID, enableCMD);
}
void moveBase(float target) {
    static float previous_target = 90;
    float gear_ratio = 64.0f / 17.0f;    
    float target_position = 0.0f;
    char motorMessage[150];
    if (target < previous_target) {
        target_position = calculateTargetPosition(previous_target - target) * gear_ratio*-1; // Move in reverse
    } else {
        target_position = calculateTargetPosition(target - previous_target) * gear_ratio;  // Move forward
    }
    snprintf(motorMessage, sizeof(motorMessage), "Previous target: %.2f°, Current target: %.2f°", previous_target, target);
//    displayMessage(motorMessage);
    setMotorMode(BASE_MOTOR, 3);
    st.RegWritePosEx(BASE_MOTOR, target_position, BaseMotorSpeed, BaseMotorAcc);
    previous_target = target;
}



void moveShoulder(float target){
  float target_position = calculateTargetPosition(target);
  char motorMessage[100];
  snprintf(motorMessage, sizeof(motorMessage),"Moving shoulder motors to angle: %.2f°, position: %.2f",target, target_position);
  displayMessage(motorMessage);
  st.RegWritePosEx(SHOULDER_MOTOR_1, calculateTargetPosition(target), ServoSpeed, ServoAcc); // Move to target
  st.RegWritePosEx(SHOULDER_MOTOR_2, calculateTargetPosition(target), ServoSpeed, ServoAcc); // Move to target
  }


void moveJoint(int motorID,float target){
    float target_position = calculateTargetPosition(target);
    char motorMessage[100];
    snprintf(motorMessage, sizeof(motorMessage), "Joint %d Moving to target: %.2f", motorID, target_position);
    displayMessage(motorMessage);
    st.RegWritePosEx(motorID,target_position, ServoSpeed, ServoAcc); 
    st.RegWriteAction();
}

void moveArmToPosition(float basePosition,float shoulderPosition,float elbowPosition,float wristPosition,float gripperPosition,float gripperOpenClose){
  moveBase(basePosition);
  moveShoulder(shoulderPosition);
  moveJoint(3,elbowPosition);
  moveJoint(4,wristPosition);
  moveJoint(5,gripperPosition);
  moveGripper(gripperOpenClose);
  st.RegWriteAction();
  }

void moveGripper(float gripperOpenClose){
  esp_now_send(receiverAdd, (uint8_t *)&gripperOpenClose, sizeof(gripperOpenClose));
  }

void moveAllMotorsToMiddlePosition() {
  displayMessage("Moving motors to middle position");
  float target_position = calculateTargetPosition(MiddlePosition);
  for (byte id = 0; id <= MAX_ID; id++) {
    if(id==0){
//      moveBase(0);
    }else{
      st.RegWritePosEx(id, target_position, ServoSpeed, ServoAcc);
     }
  st.RegWriteAction();
  }
  
}


void setup() {
  InitScreen();
  InitServos();
  displayMessage("Pegasus Arm is ready!");
  moveAllMotorsToMiddlePosition();
//  setMiddleForAllMotors();
//  moveArmToPosition(-120,160,240,230,80,30);
}

void loop() {

}
