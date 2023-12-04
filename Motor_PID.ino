/////////// Libraries ////////////////////////
#include <LiquidCrystal.h>
#include <digitalWriteFast.h>

////////// Define the pins for encoder input as wells as variables////////////////

//////// Defined Pins and variables ////////
#define encoderA 2                      // Connect encoder A channel to digital pin 2 (yellow)
#define encoderB 3                      // Connect encoder B channel to digital pin 3 (blue)
#define MaxValue 4294967294             // Maximum encoder value before resetting ( Is the most 4,294,967,295)
LiquidCrystal lcd(12, 11, 4, 5, 8, 9);  //RS, E, D4, D5, D6, D7

/////////////  Long //////////////////
volatile unsigned long encoderPos = 0;  // Initialize encoder position           
unsigned long prevTime = 0;             // Initialize previous time
unsigned long lastEncoderPos = 0;
unsigned long deltaEncoder = 0;

////////////  Integer /////////////////
int targetRPM = 0; // Setpoint  
int count = 0;
int PWM = 0;
int PWM_LIVE = 0;

///////////// Constants  //////////////
const int pwmPin = 10;
const int btnIncrement = 7;             
const int btnDecrement = 6;  

////////////  Double   /////////////////
double Kp = 0.00;
double Ki = 0.00;
double Kd = 0.00;
double deltaTime = 0;
double e, lastError = 0.0;
double integral, derivative = 0.0;
double u = 0.0;
double rpm = 0.0;                       

///////////// Setting pins, ISR, LCD, and serial setup //////////////////////////
void setup() 
{
  // Set encoder pins as inputs as well as buttons
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(btnIncrement, INPUT_PULLUP);
  pinMode(btnDecrement, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT); 
  
  // Attach interrupt service routines to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderA), handleEncoder, HIGH);
  
  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);

  // Display Setpoint preset
  lcd.print("SetPoint:");
  lcd.setCursor(9, 0);
  
  // Display RPM Preset
  lcd.setCursor(0,1);
  lcd.print("RPM:");
  Serial.begin(9600); // Setup Serial Monitor
}


////////////////////// Main Loop /////////////////////////////////////
void loop() 
{                                                 // Start of loop
///////////////////////////// RPM and PID //////////////////////////////////////////////////////
  unsigned long currentTime = millis();                       // Current time in milliseconds
  if (currentTime - prevTime >= 200)                          // Update Rate of 0.2 seconds (200 milsec)
  {  
      deltaEncoder = encoderPos - lastEncoderPos;             // # of encoder counts the occur within the second
      deltaTime = (currentTime - prevTime) / 1000.0;          // Time duration in seconds
      if (deltaTime > 0 && count >= 10)                       // Ensures not a negative time
      {
        rpm = (deltaEncoder / 1000.0) / deltaTime * 60.0;     // Calculation of RPM (Revolutions Per Min)

        updateRPM();                                          // Updates the RPM Values to the LCD

        //PID To control the motor
        int PWM_LIVE = (PWM_LIVE + PID());
        PWM_LIVE = constrain(PWM_LIVE, 0, 255);
        analogWrite(pwmPin,PWM_LIVE);
      }
      else    
      {
        int PWM_LIVE = 0;
        PWM_LIVE = constrain(PWM_LIVE, 0, 255);
        analogWrite(pwmPin,PWM_LIVE);
      }
      lastEncoderPos = encoderPos;                            // Stores the encoder position for next iteration
      prevTime = currentTime;                                  // Stores the time for next iteration 
  }
    
  /////////////////////////// Cases ///////////////////////////////// 
  // (Any time count equals one of the cases, changes variables to what is under each case)
  switch (count) {
    case 10:      
      targetRPM = 1500;
      Kp = 0.11;
      Ki = 0.1;
      Kd = 0.003;
      break;
    case 20:
      targetRPM = 1650;
      Kp = 0.116;
      Ki = 0.14;
      Kd = 0.0;
      break;
    case 30:
      targetRPM = 1800;
      Kp = 0.116;
      Ki = 0.14;
      Kd = 0.002;
      break;
    case 40:
      targetRPM = 2100;
      Kp = 0.186;
      Ki = 0.125;
      Kd = 0.002;
      break;
    case 50:
      targetRPM = 2250;
      Kp = 0.186; 
      Ki = 0.125; 
      Kd = 0.00242; 
      break;
    case 60:
      targetRPM = 2400;
      break;
      Kp = 0.186; 
      Ki = 0.125; 
      Kd = 0.03; 
    case 70:
      targetRPM = 2550;
      break;
      Kp = 0.186; 
      Ki = 0.125; 
      Kd = 0.00242; 
    case 80:
      targetRPM = 2700;
      Kp = 0.186; 
      Ki = 0.125; 
      Kd = 0.00242;
      break;
    case 90:
      targetRPM = 2850;
      Kp = 0.03; 
      Ki = 0.15; 
      Kd = 0.003;
      break;
    case 100:    
      targetRPM = 3000;
      Kp = 0.17; 
      Ki = 0.25; 
      Kd = 0.00;
      break;
    default:
      targetRPM = 0;
      Kp = 0.00;
      Ki = 0.00;
      Kd = 0.00;
      lcd.setCursor(5, 1);
      lcd.print("        "); // Clear the previous RPM value
      break;

  }

  //////////// User Button Interface //////////////////////////
    // Check if the increment button is pressed
    if (digitalRead(btnIncrement) == LOW) 
    {
      delay(200);           // Debounce delay
      if (count < 100) 
      {
        count += 10;
        updateCount();      // Function to update the setpoint count on the LCD
      }
    }
  
    // Check if the decrement button is pressed
    if (digitalRead(btnDecrement) == LOW) 
    {
      delay(200);           // Debounce delay
      if (count > 0)
      {
        count -= 10;
        updateCount();      // Function to update the setpoint count on the LCD
      }
  }
}       // End of Loop




////////////////////// Functions ///////////////////////////////////


/////////// Handles the encoder position in a CW rotation (ISR) //////////
void handleEncoder() 
{
  // Read the state of both encoder pins
  int stateB = digitalReadFast(encoderB);

  // Determine the direction of rotation based on the state changes
  if (stateB == LOW) 
  {
    encoderPos++;
    if (encoderPos >= MaxValue)     // Ensures the encoderPos variable from overflow (unsigned long)
    {                               // MaxValue = 4,294,967,295 - 1 
      encoderPos = 0;
      lastEncoderPos = 0;
    }
  }     //End of if statement
}       // End of ISR




//////////// Updateds display on LCD for RPM///////////////
void updateRPM()
{
  lcd.setCursor(5, 1);
  lcd.print("       "); // Clear the previous RPM value
  lcd.setCursor(5, 1);
  lcd.print(rpm,2); // Display RPM with two decimal place
}


//////////// Updateds display on LCD for setpoint count///////////////
void updateCount()
{
  lcd.setCursor(9, 0);
  lcd.print("   "); // Clear previous count
  lcd.setCursor(9, 0);
  lcd.print(count);
}

/////////////////// PID ///////////////////////
int PID()
{
      //Calculate the error, derivative, integral
      e = targetRPM - rpm;                    // error
      integral = integral + (e * deltaTime);  // Integral of the error   (Area of error from setpoint to live value)
      derivative = (e - lastError)/deltaTime; // Derivative of the error (rate of change)
      lastError = e;                          //PID Error for next Itereation

      //PID Calculation                                    
      float u = Kp * e + Ki * integral + Kd * derivative;   // PID Def
      int PWMVAL = constrain(u, -255, 255);               //Constrains to PWMVAl to the range of -255 and 255
      
      // Function will output the integer associated to PWM Val
      return PWMVAL;  
}
