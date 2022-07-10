/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
 * 
 * Crank and Cam decoders
 * 
 * This file contains the various crank and cam wheel decoder functions.
 * Each decoder must have the following 4 functions (Where xxxx is the decoder name):
 * 
 * - **triggerSetup_xxxx** - Called once from within setup() and configures any required variables
 * - **triggerPri_xxxx** - Called each time the primary (No. 1) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **triggerSec_xxxx** - Called each time the secondary (No. 2) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **getRPM_xxxx** - Returns the current RPM, as calculated by the decoder
 * - **getCrankAngle_xxxx** - Returns the current crank angle, as calculated by the decoder
 * - **getCamAngle_xxxx** - Returns the current CAM angle, as calculated by the decoder
 *
 * Each decoder must utilise at least the following variables:
 * 
 * - toothLastToothTime - The time (In uS) that the last primary tooth was 'seen'
 */

/* Notes on Doxygen Groups/Modules documentation style:
 * - Installing doxygen (e.g. Ubuntu) via pkg mgr: sudo apt-get install doxygen graphviz
 * - @defgroup tag name/description becomes the short name on (Doxygen) "Modules" page
 * - Relying on JAVADOC_AUTOBRIEF (in Doxyfile, essentially automatic @brief), the first sentence (ending with period) becomes
 *   the longer description (second column following name) on (Doxygen) "Modules" page (old Desc: ... could be this sentence)
 * - All the content after first sentence (like old Note:...) is visible on the page linked from the name (1st col) on "Modules" page
 * - To group all decoders together add 1) @defgroup dec Decoders (on top) and 2) "@ingroup dec" to each decoder (under @defgroup)
 * - To compare Speeduino Doxyfile to default config, do: `doxygen -g Doxyfile.default ; diff Doxyfile.default Doxyfile`
 */

#include <limits.h>
#include "globals.h"
#include "decoders.h"
#include "scheduledIO.h"
#include "scheduler.h"
#include "crankMaths.h"
#include "timers.h"

void (*triggerHandler)(void); ///Pointer for the trigger function (Gets pointed to the relevant decoder)
void (*triggerSecondaryHandler)(void); ///Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
void (*triggerTertiaryHandler)(void); ///Pointer for the tertiary trigger function (Gets pointed to the relevant decoder)
uint16_t (*getRPM)(void); ///Pointer to the getRPM function (Gets pointed to the relevant decoder)
int (*getCrankAngle)(void); ///Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
void (*triggerSetEndTeeth)(void); ///Pointer to the triggerSetEndTeeth function of each decoder

volatile unsigned long curTime;
volatile unsigned long curGap;
volatile unsigned long curTime2;
volatile unsigned long curGap2;
volatile unsigned long lastGap;
volatile unsigned long targetGap;

unsigned long MAX_STALL_TIME = 500000UL; //The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder, depending on the number of teeth etc. 500000 (half a second) is used as the default value, most decoders will be much less.
volatile uint16_t toothCurrentCount = 0; //The current number of teeth (Once sync has been achieved, this can never actually be 0
volatile byte toothSystemCount = 0; //Used for decoders such as Audi 135 where not every tooth is used for calculating crank angle. This variable stores the actual number of teeth, not the number being used to calculate crank angle
volatile unsigned long toothSystemLastToothTime = 0; //As below, but used for decoders where not every tooth count is used for calculation
volatile unsigned long toothLastToothTime = 0; //The time (micros()) that the last tooth was registered
volatile unsigned long toothLastSecToothTime = 0; //The time (micros()) that the last tooth was registered on the secondary input
volatile unsigned long toothLastThirdToothTime = 0; //The time (micros()) that the last tooth was registered on the second cam input
volatile unsigned long toothLastMinusOneToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered
volatile unsigned long toothLastMinusOneSecToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered on secondary input
volatile unsigned long toothLastToothRisingTime = 0; //The time (micros()) that the last tooth rose (used by special decoders to determine missing teeth polarity)
volatile unsigned long toothLastSecToothRisingTime = 0; //The time (micros()) that the last tooth rose on the secondary input (used by special decoders to determine missing teeth polarity)
volatile unsigned long targetGap2;
volatile unsigned long toothOneTime = 0; //The time (micros()) that tooth 1 last triggered
volatile unsigned long toothOneMinusOneTime = 0; //The 2nd to last time (micros()) that tooth 1 last triggered
volatile bool revolutionOne = 0; // For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)

volatile unsigned int secondaryToothCount; //Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth
volatile unsigned long secondaryLastToothTime = 0; //The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long secondaryLastToothTime1 = 0; //The time (micros()) that the last tooth was registered (Cam input)

uint16_t triggerActualTeeth;
volatile unsigned long triggerFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
volatile unsigned long triggerSecFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the secondary input

volatile uint8_t decoderState = 0;

unsigned int triggerSecFilterTime_duration; // The shortest valid time (in uS) pulse DURATION
volatile uint16_t triggerToothAngle; //The number of crank degrees that elapse per tooth
byte checkSyncToothCount; //How many teeth must've been seen on this revolution before we try to confirm sync (Useful for missing tooth type decoders)
unsigned long elapsedTime;
unsigned long lastCrankAngleCalc;
int16_t lastToothCalcAdvance = 99; //Invalid value here forces calculation of this on first main loop
unsigned long lastVVTtime; //The time between the vvt reference pulse and the last crank pulse

uint16_t ignition1EndTooth = 0;
uint16_t ignition2EndTooth = 0;
uint16_t ignition3EndTooth = 0;
uint16_t ignition4EndTooth = 0;
uint16_t ignition5EndTooth = 0;
uint16_t ignition6EndTooth = 0;
uint16_t ignition7EndTooth = 0;
uint16_t ignition8EndTooth = 0;

int16_t toothAngles[24]; //An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder, but may grow later if there are other decoders that use this style


/** Universal (shared between decoders) decoder routines.
*
* @defgroup dec_uni Universal Decoder Routines
* 
* @{
*/
// whichTooth - 0 for Primary (Crank), 1 for Secondary (Cam)

/** Add tooth log entry to toothHistory (array).
 * Enabled by (either) currentStatus.toothLogEnabled and currentStatus.compositeLogEnabled.
 * @param toothTime - Tooth Time
 * @param whichTooth - 0 for Primary (Crank), 1 for Secondary (Cam)
 */
static inline void addToothLogEntry(unsigned long toothTime, bool whichTooth)
{
  if(BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) { return; }
  //High speed tooth logging history
  if( (currentStatus.toothLogEnabled == true) || (currentStatus.compositeLogEnabled == true) ) 
  {
    bool valueLogged = false;
    if(currentStatus.toothLogEnabled == true)
    {
      //Tooth log only works on the Crank tooth
      if(whichTooth == TOOTH_CRANK)
      { 
        toothHistory[toothHistoryIndex] = toothTime; //Set the value in the log. 
        valueLogged = true;
      } 
    }
    else if(currentStatus.compositeLogEnabled == true)
    {
      compositeLogHistory[toothHistoryIndex] = 0;
      if(READ_PRI_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI); }
      if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); }
      if(whichTooth == TOOTH_CAM) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG); }
      if(currentStatus.hasSync == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SYNC); }

      toothHistory[toothHistoryIndex] = micros();
      valueLogged = true;
    }

    //If there has been a value logged above, update the indexes
    if(valueLogged == true)
    {
     if(toothHistoryIndex < (TOOTH_LOG_SIZE-1)) { toothHistoryIndex++; BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }
     else { BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }
    }


  } //Tooth/Composite log enabled
}

/** Interrupt handler for primary trigger.
* This function is called on both the rising and falling edges of the primary trigger, when either the 
* composite or tooth loggers are turned on. 
*/
void loggerPrimaryISR(void)
{
  BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  bool validEdge = false; //This is set true below if the edge 
  /* 
  Need to still call the standard decoder trigger. 
  Two checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  If either of these are true, the primary decoder function is called
  */
  if( ( (primaryTriggerEdge == RISING) && (READ_PRI_TRIGGER() == HIGH) ) || ( (primaryTriggerEdge == FALLING) && (READ_PRI_TRIGGER() == LOW) ) || (primaryTriggerEdge == CHANGE) )
  {
    triggerHandler();
    validEdge = true;
  }
  if( (currentStatus.toothLogEnabled == true) && (BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Tooth logger only logs when the edge was correct
    if(validEdge == true) { addToothLogEntry(curGap, TOOTH_CRANK); }
  }
  else if( (currentStatus.compositeLogEnabled == true) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap, TOOTH_CRANK);
  }
}

/** Interrupt handler for secondary trigger.
* As loggerPrimaryISR, but for the secondary trigger.
*/
void loggerSecondaryISR(void)
{
  BIT_CLEAR(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder function is called
  */
  if( ( (secondaryTriggerEdge == RISING) && (READ_SEC_TRIGGER() == HIGH) ) || ( (secondaryTriggerEdge == FALLING) && (READ_SEC_TRIGGER() == LOW) ) || (secondaryTriggerEdge == CHANGE) )
  {
    triggerSecondaryHandler();
  }
  //No tooth logger for the secondary input
  if( (currentStatus.compositeLogEnabled == true) && (BIT_CHECK(decoderState, BIT_DECODER_VALID_TRIGGER)) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap2, TOOTH_CAM);
  }
}

/** Compute RPM.
* As nearly all the decoders use a common method of determining RPM (The time the last full revolution took) A common function is simpler.
* @param degreesOver - the number of crank degrees between tooth #1s. Some patterns have a tooth #1 every crank rev, others are every cam rev.
* @return RPM
*/
static inline uint16_t stdGetRPM(uint16_t degreesOver)
{
  uint16_t tempRPM = 0;

  if( currentStatus.hasSync || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) )
  {
    if( (currentStatus.RPM < currentStatus.crankRPM) && (currentStatus.startRevolutions == 0) ) { tempRPM = 0; } //Prevents crazy RPM spike when there has been less than 1 full revolution
    else if( (toothOneTime == 0) || (toothOneMinusOneTime == 0) ) { tempRPM = 0; }
    else
    {
      noInterrupts();
      revolutionTime = (toothOneTime - toothOneMinusOneTime); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
      interrupts();
      if(degreesOver == 720) { revolutionTime = revolutionTime / 2; }
      tempRPM = (US_IN_MINUTE / revolutionTime); //Calc RPM based on last full revolution time (Faster as /)
      if(tempRPM >= MAX_RPM) { tempRPM = currentStatus.RPM; } //Sanity check
    }
  }
  else { tempRPM = 0; }

  return tempRPM;
}

/**
 * Sets the new filter time based on the current settings.
 * This ONLY works for even spaced decoders.
 */
static inline void setFilter(unsigned long curGap)
{
  if(configPage4.triggerFilter == 0) { triggerFilterTime = 0; } //trigger filter is turned off.
  else if(configPage4.triggerFilter == 1) { triggerFilterTime = curGap >> 2; } //Lite filter level is 25% of previous gap
  else if(configPage4.triggerFilter == 2) { triggerFilterTime = curGap >> 1; } //Medium filter level is 50% of previous gap
  else if (configPage4.triggerFilter == 3) { triggerFilterTime = (curGap * 3) >> 2; } //Aggressive filter level is 75% of previous gap
  else { triggerFilterTime = 0; } //trigger filter is turned off.
}

/**
This is a special case of RPM measure that is based on the time between the last 2 teeth rather than the time of the last full revolution.
This gives much more volatile reading, but is quite useful during cranking, particularly on low resolution patterns.
It can only be used on patterns where the teeth are evenly spaced.
It takes an argument of the full (COMPLETE) number of teeth per revolution.
For a missing tooth wheel, this is the number if the tooth had NOT been missing (Eg 36-1 = 36)
*/
static inline int crankingGetRPM(byte totalTeeth, uint16_t degreesOver)
{
  uint16_t tempRPM = 0;
  if( (currentStatus.startRevolutions >= configPage4.StgCycles) && ((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) )
  {
    if( (toothLastToothTime > 0) && (toothLastMinusOneToothTime > 0) && (toothLastToothTime > toothLastMinusOneToothTime) )
    {
      noInterrupts();
      revolutionTime = (toothLastToothTime - toothLastMinusOneToothTime) * totalTeeth;
      interrupts();
      if(degreesOver == 720) { revolutionTime = revolutionTime / 2; }
      tempRPM = (US_IN_MINUTE / revolutionTime);
      if( tempRPM >= MAX_RPM ) { tempRPM = currentStatus.RPM; } //Sanity check. This can prevent spiking caused by noise on individual teeth. The new RPM should never be above 4x the cranking setting value (Remembering that this function is only called is the current RPM is less than the cranking setting)
    }
  }

  return tempRPM;
}

/**
On decoders that are enabled for per tooth based timing adjustments, this function performs the timer compare changes on the schedules themselves
For each ignition channel, a check is made whether we're at the relevant tooth and whether that ignition schedule is currently running
Only if both these conditions are met will the schedule be updated with the latest timing information.
If it's the correct tooth, but the schedule is not yet started, calculate and an end compare value (This situation occurs when both the start and end of the ignition pulse happen after the end tooth, but before the next tooth)
*/
#define MIN_CYCLES_FOR_ENDCOMPARE 6
static inline void checkPerToothTiming(int16_t crankAngle, uint16_t currentTooth)
{
  if ( (fixedCrankingOverride == 0) && (currentStatus.RPM > 0) )
  {
    if ( (currentTooth == ignition1EndTooth) )
    {
      if( (ignitionSchedule1.Status == RUNNING) ) { SET_COMPARE(IGN1_COMPARE, IGN1_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition1EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule1.endCompare = IGN1_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition1EndAngle - crankAngle) ) ) ); ignitionSchedule1.endScheduleSetByDecoder = true; }
    }
    else if ( (currentTooth == ignition2EndTooth) )
    {
      if( (ignitionSchedule2.Status == RUNNING) ) { SET_COMPARE(IGN2_COMPARE, IGN2_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition2EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule2.endCompare = IGN2_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition2EndAngle - crankAngle) ) ) ); ignitionSchedule2.endScheduleSetByDecoder = true; }
    }
    else if ( (currentTooth == ignition3EndTooth) )
    {
      if( (ignitionSchedule3.Status == RUNNING) ) { SET_COMPARE(IGN3_COMPARE, IGN3_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition3EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule3.endCompare = IGN3_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition3EndAngle - crankAngle) ) ) ); ignitionSchedule3.endScheduleSetByDecoder = true; }
    }
    else if ( (currentTooth == ignition4EndTooth) )
    {
      if( (ignitionSchedule4.Status == RUNNING) ) { SET_COMPARE(IGN4_COMPARE, IGN4_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition4EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule4.endCompare = IGN4_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition4EndAngle - crankAngle) ) ) ); ignitionSchedule4.endScheduleSetByDecoder = true; }
    }
#if IGN_CHANNELS >= 5
    else if ( (currentTooth == ignition5EndTooth) )
    {
      if( (ignitionSchedule5.Status == RUNNING) ) { SET_COMPARE(IGN5_COMPARE, IGN5_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition5EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule5.endCompare = IGN5_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition5EndAngle - crankAngle) ) ) ); ignitionSchedule5.endScheduleSetByDecoder = true; }
    }
#endif
#if IGN_CHANNELS >= 6
    else if ( (currentTooth == ignition6EndTooth) )
    {
      if( (ignitionSchedule6.Status == RUNNING) ) { SET_COMPARE(IGN6_COMPARE, IGN6_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition6EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule6.endCompare = IGN6_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition6EndAngle - crankAngle) ) ) ); ignitionSchedule6.endScheduleSetByDecoder = true; }
    }
#endif
#if IGN_CHANNELS >= 7
    else if ( (currentTooth == ignition7EndTooth) )
    {
      if( (ignitionSchedule7.Status == RUNNING) ) { SET_COMPARE(IGN7_COMPARE, IGN7_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition7EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule7.endCompare = IGN7_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition7EndAngle - crankAngle) ) ) ); ignitionSchedule7.endScheduleSetByDecoder = true; }
    }
#endif
#if IGN_CHANNELS >= 8
    else if ( (currentTooth == ignition8EndTooth) )
    {
      if( (ignitionSchedule8.Status == RUNNING) ) { SET_COMPARE(IGN8_COMPARE, IGN8_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition8EndAngle - crankAngle) ) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule8.endCompare = IGN8_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition8EndAngle - crankAngle) ) ) ); ignitionSchedule8.endScheduleSetByDecoder = true; }
    }
#endif
  }
}
/** @} */
  
/** A (single) multi-tooth wheel with one of more 'missing' teeth.
* The first tooth after the missing one is considered number 1 and is the basis for the trigger angle.
* Note: This decoder does not currently support dual wheel (ie missing tooth + single tooth on cam).
* @defgroup dec_miss Missing tooth wheel
* @{
*/
void triggerSetup_missingTooth(void)
{
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  if(configPage4.TrigSpeed == CAM_SPEED) 
  { 
    //Account for cam speed missing tooth
    triggerToothAngle = 720 / configPage4.triggerTeeth; 
    BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  } 
  triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  if (configPage4.trigPatternSec == SEC_TRIGGER_4_1)
  {
    triggerSecFilterTime = 1000000 * 60 / MAX_RPM / 4 / 2;
  }
  else 
  {
    triggerSecFilterTime = (1000000 / (MAX_RPM / 60));
  }
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  secondaryToothCount = 0; 
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  MAX_STALL_TIME = (3333UL * triggerToothAngle * (configPage4.triggerMissingTeeth + 1)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerPri_missingTooth(void)
{
   curTime = micros();
   curGap = curTime - toothLastToothTime;
   if ( curGap >= triggerFilterTime ) //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     toothCurrentCount++; //Increment the tooth counter
     BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

     //if(toothCurrentCount > checkSyncToothCount || currentStatus.hasSync == false)
      if( (toothLastToothTime > 0) && (toothLastMinusOneToothTime > 0) )
      {
        bool isMissingTooth = false;

        /*
        Performance Optimisation:
        Only need to try and detect the missing tooth if:
        1. WE don't have sync yet
        2. We have sync and are in the final 1/4 of the wheel (Missing tooth will/should never occur in the first 3/4)
        3. RPM is under 2000. This is to ensure that we don't interfere with strange timing when cranking or idling. Optimisation not really required at these speeds anyway
        */
        if( (currentStatus.hasSync == false) || (currentStatus.RPM < 2000) || (toothCurrentCount >= (3 * triggerActualTeeth >> 2)) )
        {
          //Begin the missing tooth detection
          //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
          if(configPage4.triggerMissingTeeth == 1) { targetGap = (3 * (toothLastToothTime - toothLastMinusOneToothTime)) >> 1; } //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to multiply by 3 then divide by 2. Much faster than multiplying by 1.5)
          else { targetGap = ((toothLastToothTime - toothLastMinusOneToothTime)) * configPage4.triggerMissingTeeth; } //Multiply by 2 (Checks for a gap 2x greater than the last one)

          if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { curGap = 0; }

          if ( (curGap > targetGap) || (toothCurrentCount > triggerActualTeeth) )
          {
            //Missing tooth detected
            isMissingTooth = true;
            if( (toothCurrentCount < triggerActualTeeth) && (currentStatus.hasSync == true) ) 
            { 
                //This occurs when we're at tooth #1, but haven't seen all the other teeth. This indicates a signal issue so we flag lost sync so this will attempt to resync on the next revolution.
                currentStatus.hasSync = false;
                BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //No sync at all, so also clear HalfSync bit.
                currentStatus.syncLossCounter++;
            }
            //This is to handle a special case on startup where sync can be obtained and the system immediately thinks the revs have jumped:
            //else if (currentStatus.hasSync == false && toothCurrentCount < checkSyncToothCount ) { triggerFilterTime = 0; }
            else
            {
                if((currentStatus.hasSync == true) || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC))
                {
                  currentStatus.startRevolutions++; //Counter
                  if ( configPage4.TrigSpeed == CAM_SPEED ) { currentStatus.startRevolutions++; } //Add an extra revolution count if we're running at cam speed
                }
                else { currentStatus.startRevolutions = 0; }
                
                toothCurrentCount = 1;
                if (configPage4.trigPatternSec == SEC_TRIGGER_POLL) // at tooth one check if the cam sensor is high or low in poll level mode
                {
                  if (configPage4.PollLevelPolarity == READ_SEC_TRIGGER()) { revolutionOne = 1; }
                  else { revolutionOne = 0; }
                }
                else {revolutionOne = !revolutionOne;} //Flip sequential revolution tracker if poll level is not used
                toothOneMinusOneTime = toothOneTime;
                toothOneTime = curTime;

                //if Sequential fuel or ignition is in use, further checks are needed before determining sync
                if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage2.injLayout == INJ_SEQUENTIAL) )
                {
                  //If either fuel or ignition is sequential, only declare sync if the cam tooth has been seen OR if the missing wheel is on the cam
                  if( (secondaryToothCount > 0) || (configPage4.TrigSpeed == CAM_SPEED) || (configPage4.trigPatternSec == SEC_TRIGGER_POLL) )
                  {
                    currentStatus.hasSync = true;
                    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
                    if(configPage4.trigPatternSec == SEC_TRIGGER_SINGLE) { secondaryToothCount = 0; } //Reset the secondary tooth counter to prevent it overflowing
                  }
                  else if(currentStatus.hasSync != true) { BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If there is primary trigger but no secondary we only have half sync.
                }
                else { currentStatus.hasSync = true;  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If nothing is using sequential, we have sync and also clear half sync bit

                triggerFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
                toothLastMinusOneToothTime = toothLastToothTime;
                toothLastToothTime = curTime;
                BIT_CLEAR(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //The tooth angle is double at this point
            }
          }
        }
        
        if(isMissingTooth == false)
        {
          //Regular (non-missing) tooth
          setFilter(curGap);
          toothLastMinusOneToothTime = toothLastToothTime;
          toothLastToothTime = curTime;
          BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
        }
      }
      else
      {
        //We fall here on initial startup when enough teeth have not yet been seen
        toothLastMinusOneToothTime = toothLastToothTime;
        toothLastToothTime = curTime;
      }
     

      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) ) 
      {
        int16_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + configPage4.triggerAngle;
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
        {
          crankAngle += 360;
          crankAngle = ignitionLimits(crankAngle);
          checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount)); 
        }
        else{ crankAngle = ignitionLimits(crankAngle); checkPerToothTiming(crankAngle, toothCurrentCount); }
      }
   }
}

void triggerSec_missingTooth(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if( (toothLastSecToothTime == 0) )
  { 
    curGap2 = 0; 
    toothLastSecToothTime = curTime2;
  }

  if ( curGap2 >= triggerSecFilterTime )
  {
    if ( configPage4.trigPatternSec == SEC_TRIGGER_4_1 )
    {
      targetGap2 = (3 * (toothLastSecToothTime - toothLastMinusOneSecToothTime)) >> 1; //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
      toothLastMinusOneSecToothTime = toothLastSecToothTime;
      if ( (curGap2 >= targetGap2) || (secondaryToothCount > 3) )
      {
        secondaryToothCount = 1;
        revolutionOne = 1; //Sequential revolution reset
        triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermittent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
      }
      else
      {
        triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed. Filter can only be recalculated for the regular teeth, not the missing one.
        secondaryToothCount++;
      }
    }
    else if ( configPage4.trigPatternSec == SEC_TRIGGER_SINGLE )
    {
      //Standard single tooth cam trigger
      revolutionOne = 1; //Sequential revolution reset
      triggerSecFilterTime = curGap2 >> 1; //Next secondary filter is half the current gap
      secondaryToothCount++;
    }
    toothLastSecToothTime = curTime2;
  } //Trigger filter

  //Record the VVT Angle
  if( (configPage6.vvtEnabled > 0) && (revolutionOne == 1) )
  {
    int16_t curAngle;
    curAngle = getCrankAngle();
    while(curAngle > 360) { curAngle -= 360; }
    curAngle -= configPage4.triggerAngle; //Value at TDC
    if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= configPage10.vvtCL0DutyAng; }

    currentStatus.vvt1Angle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);
  }
}

void triggerThird_missingTooth(void)
{
  //Record the VVT2 Angle (the only purpose of the third trigger)
  int16_t curAngle;
  curAngle = getCrankAngle();
  while(curAngle > 360) { curAngle -= 360; }
  curAngle -= configPage4.triggerAngle; //Value at TDC
  if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= configPage4.vvt2CL0DutyAng; }
  //currentStatus.vvt2Angle = int8_t (curAngle); //vvt1Angle is only int8, but +/-127 degrees is enough for VVT control
  currentStatus.vvt2Angle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt2Angle);
}

uint16_t getRPM_missingTooth(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM )
  {
    if(toothCurrentCount != 1)
    {
      if(configPage4.TrigSpeed == CAM_SPEED) { tempRPM = crankingGetRPM(configPage4.triggerTeeth, 720); } //Account for cam speed
      else { tempRPM = crankingGetRPM(configPage4.triggerTeeth, 360); }
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    if(configPage4.TrigSpeed == CAM_SPEED) { tempRPM = stdGetRPM(720); } //Account for cam speed
    else { tempRPM = stdGetRPM(360); }
  }
  return tempRPM;
}

int getCrankAngle_missingTooth(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempRevolutionOne = revolutionOne;
    tempToothLastToothTime = toothLastToothTime;
    interrupts();

    int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if ( (tempRevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += 360; }

    lastCrankAngleCalc = micros();
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= 720) { crankAngle -= 720; }
    else if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_missingTooth(void)
{
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  //Temp variables are used here to avoid potential issues if a trigger interrupt occurs part way through this function

  int16_t tempIgnition1EndTooth;
  tempIgnition1EndTooth = ( (ignition1EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition1EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition1EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition1EndTooth <= 0) { tempIgnition1EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition1EndTooth > triggerActualTeeth && tempIgnition1EndTooth <= configPage4.triggerTeeth) { tempIgnition1EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition1EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition1EndTooth = (triggerActualTeeth + toothAdder); }
  ignition1EndTooth = tempIgnition1EndTooth;

  int16_t tempIgnition2EndTooth;
  tempIgnition2EndTooth = ( (ignition2EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition2EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition2EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition2EndTooth <= 0) { tempIgnition2EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition2EndTooth > triggerActualTeeth && tempIgnition2EndTooth <= configPage4.triggerTeeth) { tempIgnition2EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition2EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition2EndTooth = (triggerActualTeeth + toothAdder); }
  ignition2EndTooth = tempIgnition2EndTooth;

  int16_t tempIgnition3EndTooth;
  tempIgnition3EndTooth = ( (ignition3EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition3EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition3EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition3EndTooth <= 0) { tempIgnition3EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition3EndTooth > triggerActualTeeth && tempIgnition3EndTooth <= configPage4.triggerTeeth) { tempIgnition3EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition3EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition3EndTooth = (triggerActualTeeth + toothAdder); }
  ignition3EndTooth = tempIgnition3EndTooth;

  int16_t tempIgnition4EndTooth;
  tempIgnition4EndTooth = ( (ignition4EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition4EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition4EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition4EndTooth <= 0) { tempIgnition4EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition4EndTooth > triggerActualTeeth && tempIgnition4EndTooth <= configPage4.triggerTeeth) { tempIgnition4EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition4EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition4EndTooth = (triggerActualTeeth + toothAdder); }
  ignition4EndTooth = tempIgnition4EndTooth;

#if IGN_CHANNELS >= 5
  int16_t tempIgnition5EndTooth;
  tempIgnition5EndTooth = ( (ignition5EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition5EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition5EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition5EndTooth <= 0) { tempIgnition5EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition5EndTooth > triggerActualTeeth && tempIgnition5EndTooth <= configPage4.triggerTeeth) { tempIgnition5EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition5EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition5EndTooth = (triggerActualTeeth + toothAdder); }
  ignition5EndTooth = tempIgnition5EndTooth;
#endif
#if IGN_CHANNELS >= 6
  int16_t tempIgnition6EndTooth;
  tempIgnition6EndTooth = ( (ignition6EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition6EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition6EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition6EndTooth <= 0) { tempIgnition6EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition6EndTooth > triggerActualTeeth && tempIgnition6EndTooth <= configPage4.triggerTeeth) { tempIgnition6EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition6EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition6EndTooth = (triggerActualTeeth + toothAdder); }
  ignition6EndTooth = tempIgnition6EndTooth;
#endif
#if IGN_CHANNELS >= 7
  int16_t tempIgnition7EndTooth;
  tempIgnition7EndTooth = ( (ignition7EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition7EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition7EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition7EndTooth <= 0) { tempIgnition7EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition7EndTooth > triggerActualTeeth && tempIgnition7EndTooth <= configPage4.triggerTeeth) { tempIgnition7EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition7EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition7EndTooth = (triggerActualTeeth + toothAdder); }
  ignition7EndTooth = tempIgnition7EndTooth;
#endif
#if IGN_CHANNELS >= 8
  int16_t tempIgnition8EndTooth;
  tempIgnition8EndTooth = ( (ignition8EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition8EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition8EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition8EndTooth <= 0) { tempIgnition8EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition8EndTooth > triggerActualTeeth && tempIgnition8EndTooth <= configPage4.triggerTeeth) { tempIgnition8EndTooth = triggerActualTeeth; }
  if((uint16_t)tempIgnition8EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition8EndTooth = (triggerActualTeeth + toothAdder); }
  ignition8EndTooth = tempIgnition8EndTooth;
#endif

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Dual wheels - 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
Note: There can be no missing teeth on the primary wheel.
* @defgroup dec_dual Dual wheels
* @{
*/
/** Dual Wheel Setup.
 * 
 * */
void triggerSetup_DualWheel(void)
{
  triggerToothAngle = 360 / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  if(configPage4.TrigSpeed == CAM_SPEED) { triggerToothAngle = 720 / configPage4.triggerTeeth; } //Account for cam speed
  toothCurrentCount = 255; //Default value
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerSecFilterTime = (1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT); //This is always true for this pattern
  MAX_STALL_TIME = (3333UL * triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

/** Dual Wheel Primary.
 * 
 * */
void triggerPri_DualWheel(void)
{
    curTime = micros();
    curGap = curTime - toothLastToothTime;
    if ( curGap >= triggerFilterTime )
    {
      toothCurrentCount++; //Increment the tooth counter
      BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;

      if ( currentStatus.hasSync == true )
      {
        if ( (toothCurrentCount == 1) || (toothCurrentCount > configPage4.triggerTeeth) )
        {
          toothCurrentCount = 1;
          revolutionOne = !revolutionOne; //Flip sequential revolution tracker
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;
          currentStatus.startRevolutions++; //Counter
          if ( configPage4.TrigSpeed == CAM_SPEED ) { currentStatus.startRevolutions++; } //Add an extra revolution count if we're running at cam speed
        }

        setFilter(curGap); //Recalc the new filter value
      }

      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) ) 
      {
        int16_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + configPage4.triggerAngle;
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
        {
          crankAngle += 360;
          checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount)); 
        }
        else{ checkPerToothTiming(crankAngle, toothCurrentCount); }
      }
   } //Trigger filter
}
/** Dual Wheel Secondary.
 * 
 * */
void triggerSec_DualWheel(void)
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  if ( curGap2 >= triggerSecFilterTime )
  {
    toothLastSecToothTime = curTime2;
    triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed

    if( (currentStatus.hasSync == false) || (currentStatus.startRevolutions <= configPage4.StgCycles) )
    {
      toothLastToothTime = micros();
      toothLastMinusOneToothTime = micros() - (6000000 / configPage4.triggerTeeth); //Fixes RPM at 10rpm until a full revolution has taken place
      toothCurrentCount = configPage4.triggerTeeth;
      triggerFilterTime = 0; //Need to turn the filter off here otherwise the first primary tooth after achieving sync is ignored

      currentStatus.hasSync = true;
    }
    else 
    {
      if ( (toothCurrentCount != configPage4.triggerTeeth) && (currentStatus.startRevolutions > 2)) { currentStatus.syncLossCounter++; } //Indicates likely sync loss.
      if (configPage4.useResync == 1) { toothCurrentCount = configPage4.triggerTeeth; }
    }

    revolutionOne = 1; //Sequential revolution reset
  }
  else 
  {
    triggerSecFilterTime = revolutionTime >> 1; //Set filter at 25% of the current cam speed. This needs to be performed here to prevent a situation where the RPM and triggerSecFilterTime get out of alignment and curGap2 never exceeds the filter value
  } //Trigger filter
}
/** Dual Wheel - Get RPM.
 * 
 * */
uint16_t getRPM_DualWheel(void)
{
  uint16_t tempRPM = 0;
  if( currentStatus.hasSync == true )
  {
    if( currentStatus.RPM < currentStatus.crankRPM )
    {
      if(configPage4.TrigSpeed == CAM_SPEED) { tempRPM = crankingGetRPM(configPage4.triggerTeeth, 720); } //Account for cam speed
      else { tempRPM = crankingGetRPM(configPage4.triggerTeeth, 360); }
    }
    else
    {
      if(configPage4.TrigSpeed == CAM_SPEED) { tempRPM = stdGetRPM(720); } //Account for cam speed
      else { tempRPM = stdGetRPM(360); }
    }
  }
  return tempRPM;
}
/** Dual Wheel - Get Crank angle.
 * 
 * */
int getCrankAngle_DualWheel(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    tempRevolutionOne = revolutionOne;
    lastCrankAngleCalc = micros();
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(tempToothCurrentCount == 0) { tempToothCurrentCount = configPage4.triggerTeeth; }

    int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.

    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    //Sequential check (simply sets whether we're on the first or 2nd revolution of the cycle)
    if ( (tempRevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += 360; }

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}
/** Dual Wheel - Set End Teeth.
 * 
 * */
void triggerSetEndTeeth_DualWheel(void)
{
  //The toothAdder variable is used for when a setup is running sequentially, but the primary wheel is running at crank speed. This way the count of teeth will go up to 2* the number of primary teeth to allow for a sequential count. 
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  int16_t tempIgnition1EndTooth;
  tempIgnition1EndTooth = ( (ignition1EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition1EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition1EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition1EndTooth <= 0) { tempIgnition1EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition1EndTooth = tempIgnition1EndTooth;

  int16_t tempIgnition2EndTooth;
  tempIgnition2EndTooth = ( (ignition2EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition2EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition2EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition2EndTooth <= 0) { tempIgnition2EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition2EndTooth = tempIgnition2EndTooth;

  int16_t tempIgnition3EndTooth;
  tempIgnition3EndTooth = ( (ignition3EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition3EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition3EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition3EndTooth <= 0) { tempIgnition3EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition3EndTooth = tempIgnition3EndTooth;

  int16_t tempIgnition4EndTooth;
  tempIgnition4EndTooth = ( (ignition4EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition4EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition4EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition4EndTooth <= 0) { tempIgnition4EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition4EndTooth = tempIgnition4EndTooth;

#if IGN_CHANNELS >= 5
  int16_t tempIgnition5EndTooth;
  tempIgnition5EndTooth = ( (ignition5EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition5EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition5EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition5EndTooth <= 0) { tempIgnition5EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition5EndTooth = tempIgnition5EndTooth;
#endif
#if IGN_CHANNELS >= 6
  int16_t tempIgnition6EndTooth;
  tempIgnition6EndTooth = ( (ignition6EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition6EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition6EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition6EndTooth <= 0) { tempIgnition6EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition6EndTooth = tempIgnition6EndTooth;
#endif
#if IGN_CHANNELS >= 7
  int16_t tempIgnition7EndTooth;
  tempIgnition7EndTooth = ( (ignition7EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition7EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition7EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition7EndTooth <= 0) { tempIgnition7EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition7EndTooth = tempIgnition7EndTooth;
#endif
#if IGN_CHANNELS >= 8
  int16_t tempIgnition8EndTooth;
  tempIgnition8EndTooth = ( (ignition8EndAngle - configPage4.triggerAngle) / (int16_t)(triggerToothAngle) );
  if(tempIgnition8EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition8EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition8EndTooth <= 0) { tempIgnition8EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition8EndTooth = tempIgnition8EndTooth;
#endif

  lastToothCalcAdvance = currentStatus.advance;

}
/** @} */

/** Basic Distributor where tooth count is equal to the number of cylinders and teeth are evenly spaced on the cam.
* No position sensing (Distributor is retained) so crank angle is
* a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_dist Basic Distributor
* @{
*/
void triggerSetup_BasicDistributor(void)
{
  triggerActualTeeth = configPage2.nCylinders;
  if(triggerActualTeeth == 0) { triggerActualTeeth = 1; }
  triggerToothAngle = 720 / triggerActualTeeth; //The number of degrees that passes from tooth to tooth
  triggerFilterTime = 60000000L / MAX_RPM / configPage2.nCylinders; // Minimum time required between teeth
  triggerFilterTime = triggerFilterTime / 2; //Safety margin
  triggerFilterTime = 0;
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_CLEAR(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  toothCurrentCount = 0; //Default value
  BIT_SET(decoderState, BIT_DECODER_HAS_FIXED_CRANKING);
  BIT_SET(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT);
  if(configPage2.nCylinders <= 4) { MAX_STALL_TIME = (1851UL * triggerToothAngle); }//Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm rather than 50rpm due to the potentially very high stall time on a 4 cylinder if we wait that long.
  else { MAX_STALL_TIME = (3200UL * triggerToothAngle); } //Minimum 50rpm. (3200uS is the time per degree at 50rpm).

}

void triggerPri_BasicDistributor(void)
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if ( (curGap >= triggerFilterTime) )
  {
    if(currentStatus.hasSync == true) { setFilter(curGap); } //Recalc the new filter value
    else { triggerFilterTime = 0; } //If we don't yet have sync, ensure that the filter won't prevent future valid pulses from being ignored. 
    
    if( (toothCurrentCount == triggerActualTeeth) || (currentStatus.hasSync == false) ) //Check if we're back to the beginning of a revolution
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.hasSync = true;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if( (toothCurrentCount < triggerActualTeeth) ) { toothCurrentCount++; } //Increment the tooth counter
      else
      {
        //This means toothCurrentCount is greater than triggerActualTeeth, which is bad.
        //If we have sync here then there's a problem. Throw a sync loss
        if( currentStatus.hasSync == true ) 
        { 
          currentStatus.syncLossCounter++;
          currentStatus.hasSync = false;
        }
      }
      
    }

    BIT_SET(decoderState, BIT_DECODER_VALID_TRIGGER); //Flag this pulse as being a valid trigger (ie that it passed filters)

    if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
    {
      endCoil1Charge();
      endCoil2Charge();
      endCoil3Charge();
      endCoil4Charge();
    }

    if(configPage2.perToothIgn == true)
    {
      int16_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + configPage4.triggerAngle;
      crankAngle = ignitionLimits((crankAngle));
      if(toothCurrentCount > (triggerActualTeeth/2) ) { checkPerToothTiming(crankAngle, (toothCurrentCount - (triggerActualTeeth/2))); }
      else { checkPerToothTiming(crankAngle, toothCurrentCount); }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  } //Trigger filter
}
void triggerSec_BasicDistributor(void) { return; } //Not required
uint16_t getRPM_BasicDistributor(void)
{
  uint16_t tempRPM;
  if( currentStatus.RPM < currentStatus.crankRPM)
  { 
    tempRPM = crankingGetRPM(triggerActualTeeth, 720);
  } 
  else { tempRPM = stdGetRPM(720); }

  MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
  if(triggerActualTeeth == 1) { MAX_STALL_TIME = revolutionTime << 1; } //Special case for 1 cylinder engines that only get 1 pulse every 720 degrees
  if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum

  return tempRPM;

}
int getCrankAngle_BasicDistributor(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    int crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle; //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);

    //crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_TOOTH);
    

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_BasicDistributor(void)
{

  int tempEndAngle = (ignition1EndAngle - configPage4.triggerAngle);
  tempEndAngle = ignitionLimits((tempEndAngle));

  
  if( (tempEndAngle > 180) || (tempEndAngle <= 0) )
  {
    ignition1EndTooth = 2;
    ignition2EndTooth = 1;
  }
  else
  {
    ignition1EndTooth = 1;
    ignition2EndTooth = 2;
  }


  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Non-360 Dual wheel with 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
There can be no missing teeth on the primary wheel.
* @defgroup dec_non360 Non-360 Dual wheel
* @{
*/
void triggerSetup_non360(void)
{
  triggerToothAngle = (360 * configPage4.TrigAngMul) / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth multiplied by the additional multiplier
  toothCurrentCount = 255; //Default value
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise
  triggerSecFilterTime = (1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  BIT_CLEAR(decoderState, BIT_DECODER_2ND_DERIV);
  BIT_SET(decoderState, BIT_DECODER_IS_SEQUENTIAL);
  MAX_STALL_TIME = (3333UL * triggerToothAngle); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}


void triggerPri_non360(void)
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

void triggerSec_non360(void)
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

uint16_t getRPM_non360(void)
{
  uint16_t tempRPM = 0;
  if( (currentStatus.hasSync == true) && (toothCurrentCount != 0) )
  {
    if(currentStatus.RPM < currentStatus.crankRPM) { tempRPM = crankingGetRPM(configPage4.triggerTeeth, 360); }
    else { tempRPM = stdGetRPM(360); }
  }
  return tempRPM;
}

int getCrankAngle_non360(void)
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(tempToothCurrentCount == 0) { tempToothCurrentCount = configPage4.triggerTeeth; }

    //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    int crankAngle = (tempToothCurrentCount - 1) * triggerToothAngle;
    crankAngle = (crankAngle / configPage4.TrigAngMul) + configPage4.triggerAngle; //Have to divide by the multiplier to get back to actual crank angle.

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= 720) { crankAngle -= 720; }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += 360; }

    return crankAngle;
}

void triggerSetEndTeeth_non360(void)
{
  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */
