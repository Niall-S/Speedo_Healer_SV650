// This sketch takes a speed input in MPH from a GPS module over serial connection, and converts it to a pulses equivalent to the signal output from an SV650 speedometer drive at the same speed.

//Libraries
#include <Adafruit_DotStar.h> // Trinket M0 Specific - Only used to turn off onboard DotStar LED

//Predefined Pins
static const byte OutPin = 2; // Signal Output Pin
static const byte LEDPin = 13; // Led Pin to sebug if the signal is being generated
static const byte InPin = 1; // Pulse Input Pin

//Trinket M0 onboard DotStar Control
const byte DATAPIN = 7;
const byte CLOCKPIN = 8;
const byte NUMPIXELS = 1;
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

//User Dependent Variables //////////////////////

//Vehicle Specifications
const byte Rim_Diameter = 17; //Input wheel rim diameter in inches (not including tyre) 
const byte Tyre_Width = 120; // Input tyre width in mm
const byte Tyre_Profile = 70; // Input tyre profile in %
int Pulses_Out_Per_Revolution = 4; // Number of pulses out per revolution
int Pulses_Per_Revolution = 3; // Number of pulses in per revolution
const int Duty_Cycle = 50; // Input duty cycle of the pulse to determine pulse width

//Preferences
static const byte Min_Reading = 1; // Minimum speed read from GPS in MPH to be considered valid
static const byte Debug_Offset = 50; // Add this amount to GPS to allow stationary debugging. Set to 0 to disable
const unsigned long Zero_Timeout = 500000; // Lowering this value will increase the response time for a full stop scenario but will increase the lower limit of speed that can be measured.
const byte Num_Readings = 2; // Number of pulses averaged for readings. Increasing this means less fluctuation/more stable readings but will have slower response to changes.
unsigned int Amount_Of_Readings = 5; // Number of pulses before calculating rpm, low values give good responsiveness at low speed but greater fluctuation at high speed.

///////////////////////////////////////////////

//Initialise Constants
static const float Wheel_Diameter = ((Rim_Diameter * 25.4) + 2 * (Tyre_Width * Tyre_Profile / 100))/1000; // Calculates wheel diameter in metres
static const unsigned long Pulse_Conversion = (3.141592 * Wheel_Diameter * 3600000000) / (Pulses_Per_Revolution * 1609.34);
static const unsigned long Max_Period = 300000; //Pulse_Conversion / Min_Reading;

//Initialise Volatile Variables
unsigned long Last_Output_Pulse = 0; // Stores time of last output pulse
long New_Period = 0; // Stores new calculated output pulse period required
volatile unsigned long Last_Pulse_Measured = 0; // Stores time of last input pulse
volatile unsigned long Period_Between_Pulses = Zero_Timeout + 1000; // Stores measured period of pulses
volatile unsigned long Period_Average = Zero_Timeout + 1000; // Stores period once averaged
unsigned long Frequency_Raw = 0; // Frequency calculated from period (has lots of extra zeros)
unsigned long Frequency_Real = 0; // Frequency without decimals
unsigned long RPM = 0; // Raw RPM value with no averaging
unsigned int Pulse_Counter = 1; // Counts the number of pulses for averaging
unsigned long Period_Sum = 0; // Stores sum of all periods to be averaged
unsigned long Last_Time_Measured_In_Cycle = Last_Pulse_Measured; // Stores last measured pulse in cycle
unsigned long Current_Micros = micros(); // Stores the microseconds since startup at each cycle
unsigned int Zero_Debouncing = 0; // For debouncing 
unsigned long Readings[Num_Readings]; // Array for Input
unsigned long Read_Index = 0; // Index of current reading
unsigned long Total = 0; // Running Total
unsigned long RPM_Average = 0; // RPM Value after averaging

void setup() { // Runs once on start-up
  
  pinMode(InPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(InPin),Pulse_Event,FALLING); // Enables interruption when Input pin goes from low to high. This serves as the input signal.
  pinMode(OutPin, OUTPUT); // Set the Signal Out Pin as an output
  digitalWrite(OutPin, LOW); // Set the pin to Ground (LOW)
  digitalWrite(LEDPin, LOW); // Turns onboard LED off

  //Trinket M0 Specific Code
  strip.begin();
  strip.setBrightness(80);
  strip.show(); // Turns off onboard LED

  delay(1000); 
}

void loop() { // Runs in a constant Loop

  Last_Time_Measured_In_Cycle = Last_Pulse_Measured;
  Current_Micros = micros(); // Stores current time from startup in microseconds
  
  if (Current_Micros < Last_Time_Measured_In_Cycle) { // Takes into account the rare case where current micros is less than the last measured pulse in the cycle e.g. when micros() overflows
    Last_Time_Measured_In_Cycle = Current_Micros;
  }

  Frequency_Raw = 10000000000 / Period_Average; // Calculate frequency from average of periods 

  if (Period_Between_Pulses > Zero_Timeout - Zero_Debouncing || Current_Micros - Last_Time_Measured_In_Cycle > Zero_Timeout - Zero_Debouncing) { // Detect if vehicle is stopped or if no pulses are predicted
    Frequency_Raw = 0; // Set measured frequency to zero
    Zero_Debouncing = 2000; // add extra for debouncing
  }
  else {
    Zero_Debouncing = 0; // no need for debouncing
  }

  Frequency_Real = Frequency_Raw / 10000; // Remove decimals 
  RPM = Frequency_Raw / Pulses_Per_Revolution * 60; // Calculate RPM from Frequency and pulses for each revolution
  RPM = RPM / 10000; // Remove decimals
  Total = Total - Readings[Read_Index]; // Remove previous value from total
  Readings[Read_Index] = RPM; // Change reading to new RPM value
  Total = Total + Readings[Read_Index]; // Add new value to total
  Read_Index = Read_Index + 1; // Move to new Read Index

  if (Read_Index >= Num_Readings) { // Reset Read_Index if it exceeds set number of readings
    Read_Index = 0;
  }

  RPM_Average = Total / Num_Readings;

  New_Period = Period_Between_Pulses * (Pulses_Per_Revolution / Pulses_Out_Per_Revolution); //(60 * 1000000) / (Pulses_Out_Per_Revolution * RPM_Average); // Calculate the period of the pulses to output
  if (New_Period < Max_Period) { // if the speed value is above the validity threshold chosen 
    
    if ((micros() - Last_Output_Pulse) > (New_Period)) { // if the time since the last pulse exceeds the desired pulse period
      
      digitalWrite(OutPin, HIGH); // pull signal pin to equal input voltage to begin pulse
      digitalWrite(LEDPin, HIGH); // set debug status LED to match 
      Last_Output_Pulse = micros(); // Reset pulse timer
      
    }
    if ((micros() - Last_Output_Pulse) > (New_Period / 2)) { // Turn off pulse for requried duty cycle
      
      digitalWrite(OutPin, LOW); // ppull signal pin to ground to end pulse
      digitalWrite(LEDPin, LOW); // turn off debug LED to match 
      
    }
    
  }
  else {
    digitalWrite(OutPin,LOW);
    digitalWrite(LEDPin,LOW);
  }

}

void Pulse_Event() { // The function to run on interrupt from InPin

  Period_Between_Pulses = micros() - Last_Pulse_Measured; // Current time minus last time a pulse arrived to give the period

  Last_Pulse_Measured = micros(); // Reset last time a pulse was recieved

  if (Pulse_Counter >= Amount_Of_Readings) { // If the required amount of pulses to give a reading is reached

    Period_Average = Period_Sum / Amount_Of_Readings; // Output a reading

    Pulse_Counter = 1; // Reset the pulse counter
    Period_Sum = Period_Between_Pulses;

    int Remapped_Amount_Of_Readings = map(Period_Between_Pulses, 40000, 5000, 1, 10); // This section remaps the amount of readings required to give good accuracy at high speed by averaging over more readings and fewer readings at low speed to increase responsiveness
    Remapped_Amount_Of_Readings = constrain(Remapped_Amount_Of_Readings, 1, 10);
    Amount_Of_Readings = Remapped_Amount_Of_Readings;
  }
  else {
    
    Pulse_Counter++; // increment the pulse counter
    Period_Sum = Period_Sum + Period_Between_Pulses; // add the measured period to the total for averaging
    
  }
 
}
