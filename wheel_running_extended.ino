/*  
Ozburn Lab Wheel Running Data Logger
    Feb 2022 - Justin Anderson (andejust@ohsu.edu) and Kolter Grigsby (grigsbyk@ohsu.edu)
    Funded by NIH/VA grants ############
    
    Button logic inspired by Arudino Button library (madleech, 2016, 
    https://github.com/madleech/Button)

    A self contained data logger for recording mice wheel running. Tracks the 
    number of rotations of a mouse wheel by treating a reed switch as a button.

    Notes: 
      Pin 10 is used by the data shield to interact with the SD card
      Pin 13 is tied to an LED by default
      Pins 0,1 dont work while in Serial mode
      Pins 10,11,12,13 are needed to read/write to an SD file
      Analog pins can be used as digital (14,15,16,17,18,19)
      TEST_MODE doesn't work on pins 18 and 19, since they cant be OUTPUT pins
      Turning off SD/Serial allows us to reclaim pins 0,1 for input
      
   TO_DO:
      (1) Verify that not having it plugged into a serial doesn't make the program
          hang due to RAM issues.
      (2) Figure out copyright stuff
*/

#include <SdFat.h> //SD library
#include <RTClib.h> //Real time clock library

//Button Class
//inspired by Arudino Button library (madleech, 2016, https://github.com/madleech/Button)
//Duration and counts were internalized, queries added
class Button
{
  public:
    Button(uint8_t pin, uint16_t debounce_ms = 30);
    void begin();
    void soft_reset();
    void hard_reset();
    bool read();
    bool toggled();
    bool pressed();
    bool released();
    bool has_changed();
    uint32_t press_count();
    uint32_t press_duration();
    uint32_t release_count();
    uint32_t release_duration();
    
    const static bool PRESSED = LOW;
    const static bool RELEASED = HIGH;
  
  private:
    uint8_t  _pin;
    uint16_t _delay;
    bool     _state;
    uint32_t _ignore_until;
    bool     _has_changed;
    uint32_t _last_pressed;
    uint32_t _last_released;
    uint32_t _total_pressed_duration;
    uint32_t _total_released_duration;
    uint32_t _press_count;
    uint32_t _release_count;
};

//Button data queries
uint32_t Button::press_count()      {return _press_count;}
uint32_t Button::press_duration()   {return _total_pressed_duration;}
uint32_t Button::release_count()    {return _release_count;}
uint32_t Button::release_duration() {return _total_released_duration;} 

//Default values
Button::Button(uint8_t pin, uint16_t debounce_ms )
:  _pin(pin)
,  _delay(debounce_ms)
,  _state(HIGH)
,  _ignore_until(0)
,  _has_changed(false)
,  _press_count(0)
,  _release_count(0)
,  _total_pressed_duration(0)
,  _total_released_duration(0)
,  _last_pressed(0)
,  _last_released(0)
{
}

//Initialize the button
void Button::begin()
{
  pinMode(_pin, INPUT_PULLUP);
  hard_reset();
}

// Reset the variables while we are logging
void Button::soft_reset()
{ 
  _total_pressed_duration   = 0;
  _total_released_duration  = 0;
  _press_count              = 0;
  _release_count            = 0;
}

//Completely reset all variables
void Button::hard_reset()
{
  _ignore_until           = millis()+_delay;
  _last_pressed           =_ignore_until;
  _last_released          =_ignore_until;
  _total_pressed_duration = 0;
  _total_released_duration = 0;
  _press_count = 0;
  _release_count = 0;
}

//Logic for recording releases/presses
bool Button::read()
{
  // ignore pin changes until after this delay time to stop bouncing
  if (_ignore_until > millis())
  {
    return _state;
  }
  if (digitalRead(_pin) != _state)
  {
    _ignore_until = millis() + _delay;
    _state = !_state;
    _has_changed = true;

    //Button was just pressed
    if( _state == PRESSED )
    {
      _last_pressed = millis();
      _total_released_duration += (_last_pressed - _last_released);
      _press_count++; 
    }
    else //Button was just released
    {
      _last_released = millis();
      _total_pressed_duration += (_last_released - _last_pressed);
      _release_count++;
    }
  }
  return _state;
}

//See if the button has changed state since last queried
bool Button::has_changed()
{
  if (_has_changed)
  {
    _has_changed = false;
    return true;
  }
  return false;
}

// has the button gone from off -> on since last queried
bool Button::pressed()
{
  return (read() == PRESSED && has_changed());
}

// has the button gone from on -> off since last queried
bool Button::released()
{
  return (read() == RELEASED && has_changed());
}

//Defines
#define SD_CONFIG           SdSpiConfig(SS, DEDICATED_SPI, SD_SCK_MHZ(50)) //config for SDFat
#define UNIQUE_LOG_NAME     "runner_4.log"             // Unique log name for THIS arduino
#define LOG_PERIOD          60                        // Number of seconds between log events
#define ERROR_LED           2                         // Error LED pin
#define USE_SERIAL          0                         // Whether or not to use Serial output
#define TEST_PERIOD         300                       // Number of seconds to blink the error LED before recording data          

//Prototypes
void restart_io_pins();  //restart io pins (10,11,12,0,1) for read/write/serial
void disable_io_pins();  //disbale io pins (10,11,12,0,1) for data collection
int print_header();      //Print header to serial and log file
int log_data();          //Save data to serial and log.  

//Lightup the LED and stall to show a fatal error.
void fatal_error()
{
  #if USE_SERIAL
  if(Serial) Serial.println( F("FATAL ERROR") );
  #endif
  pinMode(ERROR_LED,OUTPUT);
  digitalWrite(ERROR_LED,HIGH);
  while(1);
}

//Globals
const uint8_t data_pins[] = {0,1,3,4,5,6,7,8,9,14,15,16,17};         // List of pins to track
Button        *wheel[sizeof(data_pins)];            // Button objects for each pin
RTC_PCF8523 rtc;                                    // Real time clock
uint8_t       time_mode = 0;                        // Whether to use RTC or millis for timing/naming
SdFat         sd;                                   // SD Card
File          file;                                 // SD File
uint32_t      datafile_time;                        // Time program started
char          datafile_name[25];                    // Data file name
uint32_t      log_time = 0;                         // When next to log
uint32_t      led_time[] = {0,0};                   // When to next {blink,turn off} the LED when working

//Arduino setup, called once on upload or reset
void setup()
{
  
  #if USE_SERIAL
  Serial.begin(9600);
  //Wait for serial for a few seconds, but we dont require it.  This allows resets in the lab
  while( !Serial && millis() < 2000 );
  if( !Serial ) Serial.end();
  else Serial.println( F("SETUP: Serial successfully initialized.") );
  #endif

  //Setup the Real Time Clock
  if( !rtc.begin() )
  {
    #if USE_SERIAL
    Serial.println( F( "SETUP: Could not initialize RTC, using millis() for timing and naming." ) );
    #endif
  }
  else
  {
      //This sets the datetime to the computer's on compile/upload
      //Only do it if we are connected via serial / to a computer
      if (Serial && (!rtc.initialized() || rtc.lostPower()) ) 
      {
        #if USE_SERIAL
        Serial.println( F( "RTC is NOT initialized, let's set the time!") );
        #endif
        rtc.adjust( DateTime( F(__DATE__) , F(__TIME__) ) );
      }
      time_mode = 1;
      #if USE_SERIAL
      Serial.println( F( "SETUP: RTC successfully initialized.") );
      #endif
  }

 
  //Setup the SD Card
  if (!sd.begin(SD_CONFIG)) 
  {
    #if USE_SERIAL
    Serial.println( F( "SETUP: Initializing SD card Failed. FATAL ERROR." ) );
    #endif
    fatal_error();
  }

  #if USE_SERIAL
  Serial.println( F( "SETUP: SD Card Successfully Initialized" ) );
  #endif
  
  if (!file.open(UNIQUE_LOG_NAME , FILE_WRITE))
  {
    #if USE_SERIAL
    Serial.print( F("SETUP: Could not open arduino log file <") );
    Serial.print( UNIQUE_LOG_NAME );
    Serial.println( F(">.") );
    #endif
    fatal_error();
  }
  else
  {
    
    if( time_mode )
    {
      DateTime now = rtc.now();

      #if USE_SERIAL
      Serial.print( F( "SETUP: Log file succesfully loaded in RTC mode, current time (UTC unix time) is " ) );
      Serial.print( now.unixtime() );
      Serial.println( F(".") );
      #endif
    
      file.print( F( "SETUP: Log file succesfully loaded in RTC mode, current time (UTC unix time) is " ) );
      file.print( now.unixtime() );
      file.println( F(".") );
            
      datafile_time = now.unixtime();
      sprintf( datafile_name , "%lu.csv" , datafile_time );
      while( sd.exists( datafile_name ) )
        sprintf( datafile_name , "%lu.csv" , ++datafile_time );
      
        
    }
    else
    {
      #if USE_SERIAL
      Serial.println( F( "SETUP: Log file successfully loaded in millis mode." ) );
      #endif
      file.println( F( "SETUP: Log file successfully loaded in millis mode." ) );

      datafile_time = 1;
      sprintf( datafile_name , "%lu.csv" , datafile_time );
      while( sd.exists( datafile_name ) )
        sprintf( datafile_name , "%lu.csv" , ++datafile_time );
      
    }
    
    #if USE_SERIAL  
    Serial.print( F("SETUP: Unique data file name is ") );
    Serial.println( datafile_name );
    #endif
    
    file.print( F("SETUP: Unique data file name is ") );
    file.print( datafile_name );
  }
    
  file.flush();
  #if USE_SERIAL
  Serial.flush();
  #endif
  //sd.ls(LS_DATE | LS_SIZE);

  #if USE_SERIAL
  Serial.println( F("Initializing wheels.") );
  #endif
  file.println( F("Initializing wheels.") );

  for(int i = 0 ; i < sizeof(data_pins) ; i ++ )
  {
    #if USE_SERIAL
    Serial.print( F("    Button ") );
    Serial.print( i );
    Serial.print( F(" on input ") );
    Serial.println( data_pins[i] );
    #endif
  
    file.print( F("    Button ") );
    file.print( i );
    file.print( F(" on input ") );
    file.println( data_pins[i] );
  }

  #if USE_SERIAL
  Serial.print( F("SETUP: Beginning loop.  Reminder that first ") );
  Serial.print( TEST_PERIOD );
  Serial.println( F(" seconds are a test period to verify wheels are operating." ) );
  #endif
  
  file.close();

  #if USE_SERIAL
  Serial.flush();
  Serial.end();
  disable_io_pins();
  #endif
  
  for(int i = 0 ; i < sizeof(data_pins) ; i ++ )
  {
    wheel[i] = new Button( data_pins[i] );
    wheel[i]->begin();
  }
  pinMode(ERROR_LED,OUTPUT);
  digitalWrite(ERROR_LED,LOW);
} //setup

//Disable Serial and SD pins so they can be used for data
void restart_io_pins()
{ 
  pinMode(0,INPUT);
  pinMode(1,OUTPUT);
} //restart_io_pins()

//Disable Serial and SD pins so they can be used for data
void disable_io_pins()
{
  pinMode(0,INPUT_PULLUP);
  pinMode(1,INPUT_PULLUP);
} //disable_io_pins();


//Prints the CSV Header row to datafile_name
int print_header()
{
  #if USE_SERIAL
  restart_io_pins();
  Serial.begin(9600);
  #endif
  
  if (!file.open( datafile_name , FILE_WRITE) ) return 0;
   
  for(int i = 0 ; i < sizeof(data_pins) ; i ++ )
  {
    file.print( data_pins[i] );
    file.print( F("_press_count,") );
    file.print( data_pins[i] );
    file.print( F("_total_pressed_duration,") );
    file.print( data_pins[i] );
    file.print( F("_release_count,") );
    file.print( data_pins[i] );
    file.print( F("_total_released_duration,") );

    #if USE_SERIAL
    Serial.print( data_pins[i] );
    Serial.print( F("_press_count,") );
    Serial.print( data_pins[i] );
    Serial.print( F("_total_pressed_duration,") );
    Serial.print( data_pins[i] );
    Serial.print( F("_release_count,") );
    Serial.print( data_pins[i] );
    Serial.print( F("_total_released_duration,") );
    #endif
  }
  
  if( time_mode )
  {
    file.println(F("millis,unixtime"));
    #if USE_SERIAL
    Serial.println(F("millis,unixtime"));
    #endif
  }
  else
  {
    file.println(F("millis"));
    #if USE_SERIAL
    Serial.println(F("millis"));
    #endif
  }
  file.flush();
  file.close();
  #if USE_SERIAL
  Serial.flush();
  Serial.end();
  disable_io_pins();
  #endif
  
  return 1; 
} //print_header

//Logs a row of data to datafile_name
int log_data()
{
  #if USE_SERIAL
  restart_io_pins();
  Serial.begin(9600);
  #endif
  
  if (!file.open( datafile_name , FILE_WRITE) ) return 0;

  #if USE_SERIAL
  Serial.println(F(""));
  Serial.print(F("Logging data at "));
  Serial.print(millis());
  Serial.println(F("."));
  #endif
  
  for(int i = 0 ; i < sizeof(data_pins) ; i ++ )
  {
    
    file.print( wheel[i]->press_count() );
    file.print( F(",") );
    file.print( wheel[i]->press_duration() );
    file.print( F(",") );
    file.print( wheel[i]->release_count() );
    file.print( F(",") );
    file.print( wheel[i]->release_duration() );
    file.print( F(",") );

    #if USE_SERIAL
    Serial.print( wheel[i]->press_count() );
    Serial.print( F(",") );
    Serial.print( wheel[i]->press_duration() );
    Serial.print( F(",") );
    Serial.print( wheel[i]->release_count() );
    Serial.print( F(",") );
    Serial.print( wheel[i]->release_duration() );
    Serial.print( F(",") );
    #endif
    
    wheel[i]->soft_reset();
  }
  if( time_mode )
  {
    DateTime now = rtc.now();
    file.print(millis());
    file.print( F(",") );
    file.println( now.unixtime() );
    
    #if USE_SERIAL
    Serial.print(millis());
    Serial.print( F(",") );
    Serial.println( now.unixtime() );
    #endif
  }
  else
  {
    file.println(millis());
    #if USE_SERIAL
    Serial.println(millis());
    #endif
  }

  file.flush();
  file.close();
  #if USE_SERIAL
  Serial.flush();
  Serial.end();
  disable_io_pins();
  #endif

  return 1; 
} //log_data


//Arduino loop, called continuously 
void loop()
{     
    //For the first TEST_PERIOD seconds we are in stall mode, blink LED if something is pressed
    if( millis()/1000 < TEST_PERIOD )
    {
      for(int i = 0 ; i < sizeof(data_pins) ; i ++ )
      {
        if( wheel[i]->pressed() )
        {
          digitalWrite( ERROR_LED , HIGH );
          delay( 250 );
          digitalWrite( ERROR_LED , LOW );
        } 
      }
      log_time = 0; //Sanity check to make sure log_time is zero on the first non-testing loop
      return; 
    }
    
    //First non-testing loop
    if( log_time == 0 ) 
    {
      log_time = millis()+(LOG_PERIOD*1000.0);
      led_time[0] = millis()+30*1000;     //LED on time
      led_time[1] =led_time[0]+500;        //LED off time
      if( !print_header() ) fatal_error();
      //Make sure all the buttons are reset before we start recording data
      for(int i = 0 ; i < sizeof(data_pins) ; i ++ ) wheel[i]->hard_reset();

    }

    //Logging data
    if( log_time < millis() )
    {
      log_time = millis()+(LOG_PERIOD*1000.0);

      if( !log_data() ) fatal_error(); 

    }
    //The working loop to count button presses/releases

    //LED Blinking
    if( led_time[0] < millis() )
    {
      digitalWrite(ERROR_LED,HIGH);
      led_time[0] = millis()+30*1000;     //LED on time
      led_time[1] = millis()+500; //LED off time
    }
    if( led_time[1] < millis() )
         digitalWrite(ERROR_LED,LOW);

    for(int i = 0 ; i < sizeof(data_pins) ; i ++ ) wheel[i]->read();


} //loop
