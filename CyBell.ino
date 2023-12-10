// "CyBell" Sentient Bike Bell - Andrew Bedno - AndrewBedno.com
// v5.55 2023.06.20

// V5 "CyBell" - Adds sentience and I2S audio.  (and removes BlueTooth)
// V4 "SciBell" - Adds capacitive touch pad trigger and BlueTooth.
// V3 "CyberBell" - ESP32 Microcontroller version with button and DAC aux out. https://Bedno.com/ebb
// V2 "Electronic Bike Bell" - MP3 chip version with sound selector knob.
// V1 "Virtual Bike Bell" WebApp - https://Bedno.com/bell

// For ESP32 microcontroller. Arduino compile with "Huge APP" partition scheme giving 3MB to program.
// Tested specifically compiled for board type LilyGo TTGO T7 v1.4-5

// === SETUP ===
// All variables are globals for speed and to prevent heap risks.
// Also helps as forward declarations because some code is convoluted / cross-integrated due to high asynchrony.

// Additive set of analysis flags.
const int Analyze_None = 0;  // Fastest for production compile.
const int Analyze_Basic = 1;  // Enable basic analysis messages.
const int Analyze_Verbose = 2;  // Show somewhat more details.
const int Analyze_Slow = 4;  // Do feeling and expression very slow for readability.
const int Analyze_Dumb = 8;  // Disables sentience features.
const int Analyze_Touch = 16;  // Constantly report touch sensor value.
const int Analyze_Memories = 32;  // Show diagnostics on Memories functions.
const int Analyze_Hunger = 64;  // Simulate low battery.
const int Analyze_Light = 128;  // Report motion detection.
const int Analyze_Flags = Analyze_None;

// Configure physical details for board type:
uint32_t CPU_MHz = 80;  // Configured clock speed.  240 max, divided to reduce power use.
uint32_t APB_Freq = 80000000;  // Timer clock speed.  Usually 80MHz but read from system in setup to confirm.

// Configure app pins for board..
const int LED_pin = 19;  // Onboard LED (did not actually match board docs for LilyGo TT7)
const int Touch_pin = 4;  // Capacitive sense touch input button.  Limited to certain GPIO pins.
const int Button_pin = 32;  // Optional physical button to ground.  Reserved even if unused.
const int LineOut_pin = 25;  // DAC audio out.  Mono analog line out thru 1K resistor and . DAC GPIO25-26.

// Frequency of thoughts, when attention is given to asynchronous services.
// 100Hz allows many timings to roughly use centiseconds (and evenly divides audio sample freq)
// But experience shows some thoughts and feelings exceed 1/100 second so the average is slower.
const int Thought_Freq = 100;  // 100=every centisec  0=autonomic functions only
long Thought_Dur = -1;  // Calculated from sampleFreq later
long Thought_CntDn = -1;  // Service countdown.
bool Service_Thoughts = false;  // Flag to main loop set from autonomic process.

// SENSES
// Include analog to digital support
#include <driver/adc.h>

// TOUCH: Capacitive sensing input. Samples below threshold mean touched.  Dynamically calibrated.
long Touch_Threshold = -1;  // -1=unset
long Touch_Read = -1;  // Raw sensor read.
long Touch_Read_Min = 0;  // More than noise minimum read.
long Touch_Delta = 0;  // Used for swing calculation.
long Touch_Delta_Min_Drop = 29;  // Swing detection hysteresis.
long Touch_Delta_Min_Rise = Touch_Delta_Min_Drop / 2;
int Touch_Buf_Cnt = 0;  // Entries in buffer, only matters on first round.
const long Touch_Debounce_Dur_On = 7;  // Number of samples to average for sensor smoothing.
const long Touch_Debounce_Dur_Off = 7;  // Number of samples to average for sensor smoothing.
const long Touch_Buf_Dur = (Touch_Debounce_Dur_On+Touch_Debounce_Dur_Off)*2;  // Touch reading buffer.
int Touch_Buf [Touch_Buf_Dur];  // Touch readings buffer for smoothing.
int Touch_Buf_Idx = 0;  // Circular buffer index.
long Touch_Buf_Lp = 0;
long Touch_Buf_Back_Cnt = 0;  long Touch_Buf_Back_Idx = 0;
long Touch_Average = 0;  // Finds midpoint in sample buffer.
long Touch_Debounced = 0;  long Touch_Debounced_Prev = 0;  long Touch_Debounced_Average = 0;  // Average of prior moments to detect swing.
int Touch_Debounced_Out = 0;  // Current state after swing detection.
char Touch_Analyze[200];  char Touch_Analyze_Prev[200];

// LIGHT: Photoresistor (20-100k) to ground, 100k to 3.3v.
const int Light_pin = 35;  // Analog in from photoresistor.  ADC1x8:GPIO32-39, ADC2x10:GPIO25-27
adc1_channel_t Light_channel = ADC1_CHANNEL_7;  // Channel matching input pin above.  MUST be on ADC1
// Light sensitivity is dynamically calibrated.  -1=unset, reset when awoken.
long Light_Lowest = -1;  long Light_Highest = -1;  long Light_MidLow = -1;  long Light_MidHigh = -1;

// SPACE: Built-in Hall Effect (magnetic field) sensitivity is dynamically calibrated.  -1=unset, reset when awoken.
// Note that Hall effect sensor internally uses ADC1 channels 0+3 (GPIO 36+39).
long Space_Lowest = -1;  long Space_Highest = -1;  long Space_MidLow = -1;  long Space_MidHigh = -1;

// Note that scaling of good and bad thresholds for touch, light and space aren't set until some moments after awakening,
// and aren't retained in persistent memory.
// This means it always wakes up happy, and judges subsequent moments against the conditions when it awoke.
// I debated the ethics but decided I didn't want it to be overly burdened by its history, nor by its situation, upon awakening.
// A side effect is this skews it's long term memories, making happiness relative by day, but that's OK.

// HUNGER: Sense of food security using battery voltage level.
// Voltage divider resistors 100k to ground and another to "5v"(is actually battery contacts), attenuates to half.
const int Power_pin = 27;  // Analog in pin.  ADC1x8:GPIO32-39, ADC2x10:GPIO25-27
adc2_channel_t Power_channel = ADC2_CHANNEL_7;  // Channel matching input pin above. MUST be on ADC2
const float Power_Vref = 3.3;  // Reference (max) ADC voltage at 11db attenuation (oddly diverse values found in docs for this).
const float Power_ADC_VoltScaling = 4095 / Power_Vref / 2;  // ADC reads 0-4095(12 bits)=0-Vref/2

// DISTRESS: Sense of worry about increasing hunger.
// Low power warning thresholds in millivolts.  The only sensory absolutes.
// Ideally calibrated per device due to resister tolerances.
const long Hungry = 3300;  // Below 3.3v still runs fine (spec says 2.55-3.6).  Seen 3800 readings from a full 3.7v battery.
const long VeryHungry = 3000;  // Point under which is unhappy and distress chirps accelerate.
// Testing found as low as 2328 recorded still awake.  Drops fast once below nominal. Goes from 2900 to dead in under an hour.
// Lowest battery reading saved for later analysis to calibrate very low point above.
long Power_Lowest = -1;  long Power_Lowest_Saved = Power_Lowest;  // Remembered as "power"
long Power_Expressed = 0;  // Blinks battery level after awakening.
long Power_Raw = -1;  // Last raw ADC 0-2^12 reading, for checking calibration.
// Hungry chirp timing.
long Distress_CntDn = -1;
const long Distress_Dur = 7 * 60 * Thought_Freq;  // Rough minutes between distress chirps.  Altered by urgency.

// INTERACT: User interaction configurables and logic.
const long SkipTimeFirst = 85;  // Button hold time for first skip to next sound (centisecond).
const long SkipTimeContinue = 70;  // Button hold time for subsequent skips (centisecond).
const long SkipHinted = SkipTimeContinue / 2;  // Button hold time for reduced vol in prep to skip (centisecond).
long Skip_CntDn = -1;  // Centisec countdown to skip, started when button held.
bool SkipTrack = false;  bool Skipping = false;  // Flag to process a skip event.
const long Hold_Max_Sec = 22;  long Hold_Max_Sec_Eff  = Hold_Max_Sec;  // Stuck button detection seconds.
long Hold_Curr_Sec = 0;  // Stuck button seconds counting up to max
bool Button_Stuck = false;  // Final global stuck flag.  Ignores input when stuck until released.
bool ActionPressed = false;  bool ActionPressed_Prev = false;  // Main debounced global ACTION button flag.
bool ButtonDebounced_out = false;
bool ButtonDebounce_prev = false;  bool ButtonDebounce_prevprev = false;  bool ButtonDebounce_Read = false;
long MultiTaps = 0;  // Count taps close together
const long MultiTaps_Volume = 3;  // Four taps for volume change.
const long MultiTap_Dur = (Touch_Debounce_Dur_On + Touch_Debounce_Dur_Off) * (MultiTaps_Volume+1);  // Centisecs since last tap to continue multitaps
long MultiTap_CntDn = -1;

// LED timing
const long LED_Play_BlinkDur = 2;  // Duration of LED blink on button tap (in centisec).
long LED_BlinkCnt = -1;  // Countdown for single LED blink at start of sound.

// FEELINGS: Global vars and constants.
long Sense_Hunger = 0;  // Battery from voltage divider scaled to millivolts. ~2500-4000
long Sense_Touch = 0;  // Read from capacitive touch input pad, ~50-80, lower=touched
long Sense_Light = 0;  // Photoresistor light sensor ADC, thresholds dynamically calibrated, lower=brighter
long Sense_Space = 0;  // Built-in Hall effect sensor.
const long Sense_Moment = 1;  // Base delay unit in ms when considering a sense for a few moments. 0=fastest runtime
long Sense_Moments = 0;  long Sense_Moments_Cnt = 0;  // Try each sensing several times.
long Feel_Interval = 11;  long Feel_CntDn = -1;  // Only feel feelings every n service cycle (centiseconds).  11=9/sec

// Range for each feeling as remembered and expressed are scaled 1-3.  Basically constituting it's vocabular.
const int Feeling_Bad = 1;  const int Feeling_Neutral = 2;  const int Feeling_Good = 3;

// EXPRESSION of feelings.
long Expressed_Contentment = 0;  // Overall level of contentment, result from factoring several feelings.
long Expressed_Expression = 0;  // Contentment passed to expression service.  Series of digits 1-3, incrementally parsed off right until empty.
long Expressed_DigitMult = 1;  // 10^n for accumulating digits
long Expressed_CntDn = -1;  // Centisecs until next expression service.
long Expressed_BlinkDur = 20;  // Base blink duration in centisecs.  Multiplied *1-3.  Basically bit width in expressed word.
long Expressed_BlankDur = 10;  // Pause between words.
long Expressed_State = 0;  // Tracks toggling for blinks.
long Expressed_Count = 0;  // Number of expressions since awakening

// Sensory read handling.
long Sense_Read = 0;  // Temp reading from sense.
long Sense_Total = 0;  // Total from several sensings for averaging.
long Sense_Scaled = 0;  // Range scaled 1-3 level of contentment with a single sense.
long Sense_Touch_Prev = 0;  // Tracks depth of recent touch

// Sensory cross-effectors:
// LONELINESS: Sense of how long since last touched.
long LastTouched_Secs = -1;  // Seconds since end of last sounds.  Affects touch happiness.
long LastTouched_Secs_Prev = -1;  // One step buffer for consideration in loneliness.
long LastTouched_LonelySecs = 61;  // How many seconds until it misses being touched.
// PROXIMITY: Sense of changes in light and shadows.
int Light_Debounced = Feeling_Neutral;  // Smooths trends in light readings.
int Light_Debounced_Prev = Feeling_Neutral;
int Light_Debounce_Read = Feeling_Neutral;
int Light_Debounce_prev = Feeling_Neutral;  int Light_Debounce_prevprev = Feeling_Neutral;
int Light_Debounces = 0;  // Changes in last minute.  Becomes insensitive after a few.
int Light_Debounced_LastTime = 0;  // Subtracted from uptime to limit to every few seconds.

// MEMORIES
// Persistent stats:
bool Update_Memory = false;  // Flag that something may need saving when there's a free moment.
bool Memory_Locked = false;  // Block possible asynchronous read/write flash collisions.
long AwakeMinutes_Current = 0;  // Current uptime in minutes.
long AwakeMinutes_Max = 0;  long AwakeMinutes_Max_Saved = -1;  // Longest continuous uptime ever.  Remembered as "uptime"
long AwakeMinutes_Seconds  = 0;  // Used for some elapsed seconds math.
long LifeMinutes = 0;  long LifeMinutes_Saved = -1;  // Total uptime ever.  Never reset  Remembered as "lifetime"
long Fmt_Days = 0;  long Fmt_Hrs = 0;  long Fmt_Mins = 0;  // Formatting days/hours/minutes when shown
long Distress_Count = 0;  // Number of distress chirps since awakening.
long Distress_Max = 0;   // Most distress chirps ever in one uptime.  Remembered as "distress"
long Distress_Max_Saved = -1;  // Basically remembers the unhappiest it's ever been.
long Distress_First = -1;   // Uptime when first distressed.
long Distress_Latest = -1;  long Distress_Latest_Saved = -1;   // Latest uptime of first distress remembered as "hungry"
long Played_Count = 0;  // Number of sounds finished since awakening.
long Played_Max = 0;  long Played_Max_Saved = -1;  // Most sounds ever in one uptime.  Remembered as "played"

// Remembered summary of a lifetime of expressed feelings.
long Memories[Feeling_Good * Feeling_Good * Feeling_Good * Feeling_Good];
long Memories_Saved[Feeling_Good * Feeling_Good * Feeling_Good * Feeling_Good];
bool Memories_Changed = false;
int Memories_Hunger;
int Memories_Touch;
int Memories_Light;
int Memories_Space;
int Memories_Idx;
char Memories_Key[10];
long Memories_Expression;

// Main mode(s)
const int Mode_Awakening=0; const int Mode_Bell=1; const int Mode_Twister=2;
const int Mode_Unconscious=-1;  // unused unconscious mode, requires manual re-awakening.
int MainMode = Mode_Awakening;  // Current master mode
const long Mode_Timeout = Thought_Freq;  // 1 second mode select opportunity after awakening or mode change.
long Mode_Timer = Mode_Timeout;

// Twister support.
const long Twister_Dur = 60;  // Centisecs between twister words.
const long Twister_Pause_Sec = 17;  // Seconds between twister spins.
long Twister_Pause_CntDn = Twister_Pause_Sec;
long Twister_CntDn = 67;  // Initial pause, then varies.
int Twister_WordNum = 0;
int Twister_Part = 0;  // 0=side 1=limb 2=color

// Real time clock included for accurate tracking of uptime.
#include <time.h>
time_t AwakeMinutes_now;
struct tm AwakeMinutes_timeinfo;
int Seconds_Curr = 0;  int Seconds_Prev = 0;
int Minutes_Curr = 0;  int Minutes_Prev = 0;

// === AUDIO ===

// For simplicity all code is optimized for a fixed sample frequency and all wavs are encoded to match that.
// Also used as Interrupt Service Routine frequency for exact real-time match.
// 12800 chosen as compromise offering decent sound (-6KHz) with compact memory, and lowered interrupt demands.
// Must be integer divisor of 80M and of 100.
#define I2S_SampleRate 12800

// For compactness all wavs are stored in just 8 bit range as signed bytes (though stored and read in app as unsigned bytes for better control).
const uint8_t* WavSrc_Ptr;  // Points to current sound source data byte array.
signed long WavSrc_Size;  // Set globally when sound selected.
signed long WavSrc_Idx = -1;  // Byte position playing in wav source arr.

// Playback volume 0-8 (^2 exponential) out=in << n
const long Vol_Max = 8;  // 8=physical max, may cause brownout reboots on loud sound playback especially when on USB power.
long Vol_Skip = 6;  // Reduced relative volume while skipping, divide by 2^n.
long Vol_Normal = Vol_Max;  // Normal relative sound playback volume.
long Vol_Curr = Vol_Normal;  // Current relative volume shifter.
long Vol_Dec = 0;  // Master volume exponent decrement. 0=loudest
const long Vol_Dec_Max = 3;  // Quietest
long Vol_Dec_Eff = 0;  long Vol_Dec_Tmp = 0;
int Vol_Dec_Saved = Vol_Dec;  // Remembered as "vol"
// Currently selected sound.
int SoundNum = 0;  int SoundNum_Prev = -1;
int SoundNum_Saved = SoundNum;  // Remembered as "sound"

// DAC support.  Note that this is largely separate and simpler from I2S.  Always finishes, ignores multitaps.
const uint8_t DAC_MidVal = 0x7f;  // DAC is only 8 bit.  Uses signed byte source pretty directly.
uint8_t DAC_Val = -1;  uint8_t DAC_Val_Prev = -1;  // Buffer to skip redundant writes.
long DAC_Idx = -1;  // Position in array.
uint8_t DAC_In = 0;
const uint8_t* DAC_Ptr;  // Points to source for current DAC sound.
signed long DAC_Size;  // Set globally when sound selected.

// I2S config
const int I2S_LRCL = 5;  // I2S Left/Right Clock
const int I2S_BCLK = 23;  // I2S Main Clock
const int I2S_DOUT = 18;  // I2S Data Out
const int I2S_SD = 26;  // I2S Mode Select.  Analog out DAC 1-2 = GPIO25-26.
// SD: gnd=<.16v=amp off <.77v=mix <1.4v=right >1.4v=left

#include <driver/i2s.h>  // Library of I2S routines, comes with ESP32 standard install
// i2s port number
const i2s_port_t I2S_Port = I2S_NUM_0;
size_t I2S_BytesWritten;  // Discarded.

// Transcoded stereo 16 bit final output buffer, can only fit a few seconds.
#define Wav_Max_Seconds 2
uint8_t I2S_Buf[Wav_Max_Seconds * I2S_SampleRate * 4];
long I2S_Wav_Size = 0;
long I2S_Wav_Size_Max = Wav_Max_Seconds * I2S_SampleRate;
long I2S_Wav_Timeout = 0;
long WavPrep_SrcIdx = 0;  long WavPrep_TgtIdx = 0;
long WavPrep_BufCnt = 0;  long WavPrep_OutSize = 0;
uint8_t WavPrep_Byte;  long WavPrep_Int;
long WavPrep_Vol;
// Prevent redundant wav preps.
const uint8_t* WavSrc_Ptr_Prev;
long WavPrep_Vol_Prev;

// Complexity to feed DMA server and keep ahead with buffer packets
#define I2S_Buf_Samples 256  // Sound chunk for playback service.  Matches DMA buffer size.  1/13K*256=1/50sec 1024=1/12sec
#define I2S_Buf_Max_Qty 4  // Total buffers to pre-prep.
int I2S_Buf_Lp = 0;
signed long I2S_Buf_Size = -1;
signed long I2S_Len_Cnt = -1;  // tracks actual sound playback length
signed long I2S_BufPush_CntDown = -1;  // countdown to service buffer send
bool I2S_Playing = false;
bool I2S_Playing_Prev = false;

// App ISR to feed I2S library function.
const int ISR_timernum = 3;  // Fairly arbitrary but #3 works.
hw_timer_t* ISR_timer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Include WAV data arrays. (master include which then sub-includes all the sound .h files)
#include "sounds\sounds.h"
const int SoundsCnt = 18;
// Low power chirp.
#include "sounds\distress.h"

// I2S library related configuration structures
static const i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = I2S_SampleRate,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
  .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // 1-6=low-high interrupt priority
  .dma_buf_count = I2S_Buf_Max_Qty * 2,  // max number of queued buffers (before causing wait) <=8
  .dma_buf_len = I2S_Buf_Samples,  // Max sample frames to pass to i2s service. <=1024
  .use_apll = true,  // Manually set if fixed_mclk is false, recommended for DAC use.
  .tx_desc_auto_clear = true,
  .fixed_mclk = -1  // Fixed is > I2S_SampleRate*16*4
};
static const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_BCLK,  // Bit clock
  .ws_io_num = I2S_LRCL,  // Word select (aka left/right clock)
  .data_out_num = I2S_DOUT,  // Data out from ESP32
  .data_in_num = I2S_PIN_NO_CHANGE  // Unused.
};

// Push next sound fragment to DMA buffer handler.
void I2S_Buf_Push () {
  if (WavSrc_Idx >= 0) {
    if (WavSrc_Idx >= I2S_Wav_Size) {  // reset after final buffer played.
      I2S_BufPush_CntDown = -1;
      WavSrc_Idx = -1;
    } else {
      if ((WavSrc_Idx + I2S_Buf_Samples) >= I2S_Wav_Size) {
        I2S_Buf_Size = I2S_Wav_Size - WavSrc_Idx;
      } else {
        I2S_Buf_Size = I2S_Buf_Samples;
      }
      if (I2S_Buf_Size > 0) {
        // Queue data to I2S buffer handler.  Size =4*samples.
        // portMAX_DELAY=potentially wait forever for service to finish.
        i2s_write(I2S_Port, I2S_Buf + (WavSrc_Idx * 4), 4 * I2S_Buf_Size, &I2S_BytesWritten, portMAX_DELAY);
        WavSrc_Idx += I2S_Buf_Samples;  // May exceed end, trapped on next pass
        I2S_BufPush_CntDown = I2S_Buf_Size;
      } else {
        I2S_BufPush_CntDown = -1;
        WavSrc_Idx = -1;
      }
    }
  }
}

// Prepare whole current sound transcoded to stereo 16 bit
void I2S_Wav_Prep () {
  Vol_Dec_Eff = (Vol_Dec == Vol_Dec_Max) ? Vol_Dec + 1 : Vol_Dec;  // Lowest volume step is extra quiet
  if (Analyze_Flags >= Analyze_Basic) { Vol_Dec_Eff = Vol_Dec_Eff + 3; }
  WavPrep_Vol = Vol_Curr - Vol_Dec_Eff;
  if ( (WavSrc_Ptr_Prev != WavSrc_Ptr) || (WavPrep_Vol_Prev != WavPrep_Vol) ) {  // Detects if wav was already prepped.
    WavPrep_OutSize = 0;
    WavPrep_SrcIdx = 0;
    WavPrep_TgtIdx = 0;
    // Process source mono wav signed byte data to stereo integer wav buffer.
    while (WavPrep_SrcIdx < WavSrc_Size) {
      WavPrep_Byte = WavSrc_Ptr[WavPrep_SrcIdx];
      if (WavPrep_OutSize < I2S_Wav_Size_Max) {
        WavPrep_OutSize++;
        // Prepare 4 byte I2S signed 16 bit stereo sample from current source wav signed byte.
        if (WavPrep_Byte <= DAC_MidVal) {
          WavPrep_Int = WavPrep_Byte << WavPrep_Vol;
        } else {
          WavPrep_Byte = 0xFF - WavPrep_Byte;
          WavPrep_Int = WavPrep_Byte << WavPrep_Vol;
          WavPrep_Int = WavPrep_Int * -1;
        }
        // Internal int is LSB, output stream is MSB
        I2S_Buf[WavPrep_TgtIdx++] = WavPrep_Int & 0xFF;
        I2S_Buf[WavPrep_TgtIdx++] = (WavPrep_Int >> 8) & 0xFF;
        I2S_Buf[WavPrep_TgtIdx++] = 0x00;  // Right channel left idle.
        I2S_Buf[WavPrep_TgtIdx++] = 0x00;
      }
      WavPrep_SrcIdx++;
    }
    // Add trailing empty buffers so I2S is quiet before muting
    for (WavPrep_BufCnt = 0; WavPrep_BufCnt < (I2S_Buf_Max_Qty * I2S_Buf_Samples); WavPrep_BufCnt++) {
      if (WavPrep_OutSize < I2S_Wav_Size_Max) {
        WavPrep_OutSize++;
        I2S_Buf[WavPrep_TgtIdx++] = 0x00;
        I2S_Buf[WavPrep_TgtIdx++] = 0x00;
        I2S_Buf[WavPrep_TgtIdx++] = 0x00;
        I2S_Buf[WavPrep_TgtIdx++] = 0x00;
      }
    }
    I2S_Wav_Size = WavPrep_OutSize;
    WavSrc_Ptr_Prev = WavSrc_Ptr;
    WavPrep_Vol_Prev = WavPrep_Vol;
  }
}

// Set flags etc to stop playback.
void I2S_Stop () {
  I2S_Playing = false;
  // Calculated precautionary pause to let DMA buffers finish.
  if (I2S_BufPush_CntDown >= 0) {
    I2S_BufPush_CntDown = -1;
    delay((1000.0 / I2S_SampleRate) * (I2S_BufPush_CntDown + I2S_Buf_Samples * 2));
  }
  WavSrc_Idx = -1;
  I2S_Len_Cnt = -1;  // precautionary
  if (MainMode > Mode_Awakening) { i2s_zero_dma_buffer(I2S_Port); }
}

// Main interrupt routine called at audio sample rate freq.
// Must be kept VERY LEAN!  No floating math, sizeof or loops.
void IRAM_ATTR Autonomic_Functions () {
  if (MainMode != Mode_Unconscious) {
    // Service DAC audio playback (immediate analog to aux/line out)
    if (DAC_Idx>=0) {
      DAC_In = DAC_Ptr[DAC_Idx];
      if (DAC_In <= DAC_MidVal) {
        DAC_Val = DAC_MidVal + DAC_In;
      } else {
        DAC_Val = DAC_MidVal - (0xFF - DAC_In);
      }
      if (DAC_Val != DAC_Val_Prev) {
        dacWrite(LineOut_pin,DAC_Val);
        DAC_Val_Prev = DAC_Val;
      }
      DAC_Idx++;
      if (DAC_Idx >= DAC_Size) { DAC_Idx = -1; }
    }
    // Service I2S audio playback (block buffered)
    if (I2S_Playing && (I2S_Len_Cnt >= 0)) {  // Either condition disables service.
      // Countdown to push next sound buffer chunk.
      if (I2S_BufPush_CntDown > 0) {
        I2S_BufPush_CntDown--;  // Prevents reentrancy
        if (I2S_BufPush_CntDown == 0) {  // Push next buffer and reset countdown
          I2S_Buf_Push();
        }
      }
      // Counts duration of sound played so far in real time. Due to lack of an I2S finished callback.
      if (I2S_Len_Cnt < I2S_Wav_Timeout) {
        I2S_Len_Cnt++;
      } else {
        // Sound finished playing.
        I2S_Stop();
        dacWrite(I2S_SD, 0);  // Amp off, only when idle after sound ending
        Played_Count++;
        if (((MainMode == Mode_Bell) && (Skip_CntDn < 1) && (!ActionPressed)) ||
             ( ((MainMode == Mode_Twister) && (Twister_Part == 0))) ) Express_Feelings();
      }
    }
    // Flag consciousness for service (in main loop) every centisecond or so.
    if ((!Service_Thoughts) && (Thought_CntDn > 0)) {
      Thought_CntDn--;
      if (Thought_CntDn == 0) { Service_Thoughts = true; }
    }
  }
}

// Set flags to start WAV playback through I2S.  Wav data pointer and size must already be set.
void I2S_Play () {
  I2S_Stop();
  if ((Distress_CntDn > 0) && (Distress_CntDn < 500)) { Distress_CntDn += 7 * Thought_Freq; }  // Postpone power warning a few seconds if it would be soon.
  dacWrite(I2S_SD, 255);  // Activate amp, left only
  I2S_Wav_Prep();
  WavSrc_Idx = 0;
  I2S_Buf_Push();  // Prepush a moment to keep DMA one buffer ahead.
  I2S_Wav_Timeout = WavSrc_Size;  // Timeout at size of original wav
  I2S_BufPush_CntDown = 1;  //  Immediately service buffer to stay ahead.
  I2S_Playing = true;
  I2S_Len_Cnt = 0;
}

// I2S audio main setup
void Sound_Setup () {
  I2S_Stop();
  // Default setup just in case.
  Vol_Curr = Vol_Normal;
  Sound_Select();
  DAC_Ptr = WavSrc_Ptr;  DAC_Size = WavSrc_Size;
  // ESP32 allocate resources to run I2S, then configure it.
  i2s_driver_install(I2S_Port, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_Port, &pin_config);
  i2s_set_sample_rates(I2S_Port, I2S_SampleRate);
  // Setup interrupt service routine for audio queue feeder.
  ISR_timer = timerBegin(ISR_timernum, APB_Freq / I2S_SampleRate, true);  // MUST be <64K. 80M/13K=~6K
  timerAttachInterrupt(ISR_timer, &Autonomic_Functions, true);
  timerAlarmWrite(ISR_timer, 1, true);
  timerAlarmEnable(ISR_timer);
  MainMode = Mode_Bell;
}

// Setup and play sound selection given global SoundNum
void Sound_Select () {
  I2S_Stop();
  switch (SoundNum) {
    case 0:
      WavSrc_Ptr = wav_ring;  WavSrc_Size = ring_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected ring\n"); }
      break;
    case 1:
      WavSrc_Ptr = wav_caw;  WavSrc_Size = caw_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected caw\n"); }
      break;
    case 2:
      WavSrc_Ptr = wav_honk;  WavSrc_Size = honk_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected honk\n"); }
      break;
    case 3:
      WavSrc_Ptr = wav_laughs;  WavSrc_Size = laughs_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected laughs\n"); }
      break;
    case 4:
      WavSrc_Ptr = wav_cheers;  WavSrc_Size = cheers_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected cheers\n"); }
      break;
    case 5:
      WavSrc_Ptr = wav_james;  WavSrc_Size = james_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected james\n"); }
      break;
    case 6:
      WavSrc_Ptr = wav_bong;  WavSrc_Size = bong_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected bong\n"); }
      break;
    case 7:
      WavSrc_Ptr = wav_shofar;  WavSrc_Size = shofar_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected shofar\n"); }
      break;
    case 8:
      WavSrc_Ptr = wav_unicorn;  WavSrc_Size = unicorn_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected unicorn\n"); }
      break;
    case 9:
      WavSrc_Ptr = wav_godzilla;  WavSrc_Size = godzilla_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected godzilla\n"); }
      break;
    case 10:
      WavSrc_Ptr = wav_ahooga;  WavSrc_Size = ahooga_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected ahooga\n"); }
      break;
    case 11:
      WavSrc_Ptr = wav_boat;  WavSrc_Size = boat_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected boat\n"); }
      break;
    case 12:
      WavSrc_Ptr = wav_steam;  WavSrc_Size = steam_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected steam\n"); }
      break;
    case 13:
      WavSrc_Ptr = wav_beep;  WavSrc_Size = beep_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected beep\n"); }
      break;
    case 14:
      WavSrc_Ptr = wav_whistle;  WavSrc_Size = whistle_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected whistle\n"); }
      break;
    case 15:
      WavSrc_Ptr = wav_phaser;  WavSrc_Size = phaser_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected phaser\n"); }
      break;
    case 16:
      WavSrc_Ptr = wav_photon;  WavSrc_Size = photon_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected photon\n"); }
      break;
    case 17:
      WavSrc_Ptr = wav_smash;  WavSrc_Size = smash_raw_len;
      if ((SoundNum_Prev!=SoundNum) && ((Analyze_Flags&Analyze_Basic)==Analyze_Basic)) { Serial.printf("Selected smash\n"); }
      break;
  }
  SoundNum_Prev = SoundNum;
}

// === MEMORIES ===
// Nonvolatile storage of persistant memories of feelings, stats, and user selections.

#include <Preferences.h>
Preferences Memory;

// Start preferences object and load some important persistent data from non-volatile storage.
void Memory_Awaken () {
  Memory.begin("cybell", false);
  // Current volume decrement.
  Vol_Dec_Saved = Memory.getInt("vol", Vol_Dec);
  if ((Vol_Dec_Saved > Vol_Dec_Max) || (Vol_Dec_Saved < 0)) { Vol_Dec_Saved = Vol_Dec; }
  Vol_Dec = Vol_Dec_Saved;
  // Current sound number.
  SoundNum_Saved = Memory.getInt("sound", SoundNum);
  if ((SoundNum_Saved < 0) || (SoundNum_Saved >= SoundsCnt)) { SoundNum_Saved = SoundNum; }
  SoundNum = SoundNum_Saved;
  // Lowest power.
  Power_Lowest_Saved = Memory.getLong("power", Power_Lowest);
  if (Power_Lowest_Saved > 0) { Power_Lowest = Power_Lowest_Saved; }
  // Uptime max.
  AwakeMinutes_Max_Saved = Memory.getLong("uptime", AwakeMinutes_Max);
  if (AwakeMinutes_Max_Saved > 0) { AwakeMinutes_Max = AwakeMinutes_Max_Saved; }
  // Lifetime minutes.
  LifeMinutes_Saved = Memory.getLong("lifetime", LifeMinutes);
  if (LifeMinutes_Saved > 0) { LifeMinutes = LifeMinutes_Saved; }
  // Distress count max.
  Distress_Max_Saved = Memory.getLong("distress", Distress_Max);
  if (Distress_Max_Saved > 0) { Distress_Max = Distress_Max_Saved; }
  // First distressed at uptime.
  Distress_Latest_Saved = Memory.getLong("hungry", Distress_Latest);
  if (Distress_Latest_Saved > 0) { Distress_Latest = Distress_Latest_Saved; }
  // Played sounds max.
  Played_Max_Saved = Memory.getLong("played", Played_Max);
  if (Played_Max_Saved > 0) { Played_Max = Played_Max_Saved; }
  // Memories.
  if ((Analyze_Flags & Analyze_Memories) == Analyze_Memories) { Serial.printf("Memories Recalled:"); }
  for (Memories_Hunger = Feeling_Good; Memories_Hunger >= Feeling_Bad; Memories_Hunger--) {
    for (Memories_Touch = Feeling_Good; Memories_Touch >= Feeling_Bad; Memories_Touch--) {
      for (Memories_Light = Feeling_Good; Memories_Light >= Feeling_Bad; Memories_Light--) {
        for (Memories_Space = Feeling_Good; Memories_Space >= Feeling_Bad; Memories_Space--) {
          Memories_Expression = (Memories_Hunger * 1000) + (Memories_Touch * 100) + (Memories_Light * 10) + Memories_Space;
          sprintf(Memories_Key, "X%d", Memories_Expression);
          Memories_Idx = ((Memories_Hunger - Feeling_Bad) * (Feeling_Good*Feeling_Good*Feeling_Good)) + ((Memories_Touch - Feeling_Bad) * (Feeling_Good*Feeling_Good)) + ((Memories_Light - Feeling_Bad) * Feeling_Good) + (Memories_Space - Feeling_Bad);
          Memories_Saved[Memories_Idx] = Memory.getLong(Memories_Key, 0);  // fetch
          Memories[Memories_Idx] = Memories_Saved[Memories_Idx];
          if ((Analyze_Flags & Analyze_Memories) == Analyze_Memories) {
            if (Memories[Memories_Idx] > 0) {
              Serial.printf(" %s(%d)=%d", Memories_Key, Memories_Idx, Memories[Memories_Idx]);
            }
          }
        }
      }
    }
  }
  if ((Analyze_Flags & Analyze_Memories) == Analyze_Memories) { Serial.printf("\n"); }
  // Done loading.  Show values.
  if ((Analyze_Flags&Analyze_Memories)==Analyze_Memories) {
    Serial.printf("RECALLED: SoundNum=%d  VolDec=%d\n", SoundNum, Vol_Dec);
    Serial.printf("LowestPowerEver=%d  DistressLatest=%d  DistressMax=%d  PlayedMax=%d\n", Power_Lowest, Distress_Latest, Distress_Max, Played_Max);
    Fmt_Hrs = AwakeMinutes_Max / 60;  Fmt_Mins = AwakeMinutes_Max % 60;
    Serial.printf("LongestAwake=%d:%02d  ", Fmt_Hrs, Fmt_Mins);
    Fmt_Days = (LifeMinutes / 60) / 24;  Fmt_Hrs = (LifeMinutes / 60) % 24;  Fmt_Mins = LifeMinutes % 60;
    Serial.printf("Lifetime=%d'%d:%02d\n", Fmt_Days, Fmt_Hrs, Fmt_Mins);
  }
}

// Save important things to non-volatile storage.  Only acting on each if changed.
void Memory_Update () {
  if (! Memory_Locked) {  // Prevents Flash write collision.
    Memory_Locked = true;
    Update_Memory = false;
    if (Vol_Dec_Saved != Vol_Dec) {
      Memory.putInt("vol", Vol_Dec);
      Vol_Dec_Saved = Vol_Dec;
    }
    if (SoundNum_Saved != SoundNum) {
      Memory.putInt("sound", SoundNum);
      SoundNum_Saved = SoundNum;
    }
    if ((Power_Lowest > 0) && (Power_Lowest != Power_Lowest_Saved)) {
      Memory.putLong("power", Power_Lowest);
      Power_Lowest_Saved = Power_Lowest;
    }
    if ((AwakeMinutes_Max > 0) && (AwakeMinutes_Max != AwakeMinutes_Max_Saved)) {
      Memory.putLong("uptime", AwakeMinutes_Max);
      AwakeMinutes_Max_Saved = AwakeMinutes_Max;
    }
    if ((LifeMinutes > 0) && (LifeMinutes != LifeMinutes_Saved)) {
      Memory.putLong("lifetime", LifeMinutes);
      LifeMinutes_Saved = LifeMinutes;
    }
    if ((Distress_Max > 0) && (Distress_Max != Distress_Max_Saved)) {
      Memory.putLong("distress", Distress_Max);
      Distress_Max_Saved = Distress_Max;
    }
    if ((Distress_Latest > 0) && (Distress_Latest  != Distress_Latest_Saved)) {
      Memory.putLong("hungry", Distress_Latest);
      Distress_Latest_Saved = Distress_Latest;
    }
    if ((Played_Max > 0) && (Played_Max != Played_Max_Saved)) {
      Memory.putLong("played", Played_Max);
      Played_Max_Saved = Played_Max;
    }
    // Memories.
    if ((Analyze_Flags & Analyze_Memories) == Analyze_Memories) { Serial.printf("Memories Saved:"); }
    for (Memories_Hunger = Feeling_Good; Memories_Hunger >= Feeling_Bad; Memories_Hunger--) {
      for (Memories_Touch = Feeling_Good; Memories_Touch >= Feeling_Bad; Memories_Touch--) {
        for (Memories_Light = Feeling_Good; Memories_Light >= Feeling_Bad; Memories_Light--) {
          for (Memories_Space = Feeling_Good; Memories_Space >= Feeling_Bad; Memories_Space--) {
            Memories_Expression = (Memories_Hunger * 1000) + (Memories_Touch * 100) + (Memories_Light * 10) + Memories_Space;
            sprintf(Memories_Key, "X%d", Memories_Expression);
            Memories_Idx = ((Memories_Hunger - Feeling_Bad) * (Feeling_Good*Feeling_Good*Feeling_Good)) + ((Memories_Touch - Feeling_Bad) * (Feeling_Good*Feeling_Good)) + ((Memories_Light - Feeling_Bad) * Feeling_Good) + (Memories_Space - Feeling_Bad);
            if (Memories_Saved[Memories_Idx] != Memories[Memories_Idx]) {
              Memory.putLong(Memories_Key, Memories[Memories_Idx]);
              Memories_Saved[Memories_Idx] = Memories[Memories_Idx];
              if ((Analyze_Flags & Analyze_Memories) == Analyze_Memories) {
                if (Memories_Saved[Memories_Idx] > 0) {
                  Serial.printf(" %s(%d)=%d", Memories_Key, Memories_Idx, Memories_Saved[Memories_Idx]);
                }
              }
            }
          }
        }
      }
    }
    if ((Analyze_Flags & Analyze_Memories) == Analyze_Memories) { Serial.printf("\n"); }
    // Done saving.  Show values.
    if ((Analyze_Flags&Analyze_Memories)==Analyze_Memories) {
      Serial.printf("REMEMBERED: SoundNum=%d  VolDec=%d\n", SoundNum_Saved, Vol_Dec_Saved);
      Serial.printf("LowestPowerEver=%d  DistressLatest=%d  DistressMax=%d  PlayedMax=%d\n", Power_Lowest_Saved, Distress_Latest_Saved, Distress_Max_Saved, Played_Max_Saved);
      Fmt_Hrs = AwakeMinutes_Current / 60;  Fmt_Mins = AwakeMinutes_Current % 60;
      Serial.printf("AwakeCurrent=%d:%02d  ", Fmt_Hrs, Fmt_Mins);
      Fmt_Hrs = AwakeMinutes_Max_Saved / 60;  Fmt_Mins = AwakeMinutes_Max_Saved % 60;
      Serial.printf("LongestAwake=%d:%02d  ", Fmt_Hrs, Fmt_Mins);
      Fmt_Days = (LifeMinutes / 60) / 24;  Fmt_Hrs = (LifeMinutes / 60) % 24;  Fmt_Mins = LifeMinutes % 60;
      Serial.printf("Lifetime=%d'%d:%02d\n", Fmt_Days, Fmt_Hrs, Fmt_Mins);
    }
    Memory_Locked = false;
  }
}

// Delete a few persistent memories.  Hidden function Called on multitap after buttonstuck from Twister mode.
void Memory_Forget () {
  if (! Memory_Locked) {  // Prevents Flash write collision.
    I2S_Stop();  delay(200);
    Memory_Locked = true;
    if ((Analyze_Flags&Analyze_Memories)==Analyze_Memories) { Serial.printf("Some memories cleared.\n"); }
    Update_Memory = false;
    Memory.putInt("vol",0);  Vol_Dec = 0;  Vol_Dec_Saved = 0;
    Memory.putInt("sound",0);  SoundNum = 0;  SoundNum_Saved = 0;
    Memory.putLong("power",-1);  Power_Lowest_Saved = -1;  Power_Lowest = -1;
    Memory.putLong("uptime",0);  AwakeMinutes_Current = 0;  AwakeMinutes_Max_Saved = -1;  AwakeMinutes_Max = 0;
    Memory.putLong("distress",0);  Distress_Count = 0;  Distress_Max_Saved = -1;  Distress_Max = 0;
    Memory.putLong("hungry",0);  Distress_First = 0;  Distress_Latest = -1;  Distress_Latest_Saved = -1;
    Memory.putLong("played",0);   Played_Count = 0;  Played_Max_Saved = -1;  Played_Max = 0;
    digitalWrite(LED_pin, HIGH);  delay(1000);  // Waits to be sure flash saved.
    digitalWrite(LED_pin, LOW);
    Memory_Locked = false;
  }
}

// Periodically check if anything needs storing.
void Memory_Service () {
  // Check if lowest power survived so far.
  if ((Expressed_Count > 2) && (Sense_Hunger > 500) && ((Sense_Hunger < Power_Lowest) || (Power_Lowest < 0))) {
    Power_Lowest = Sense_Hunger;
    Update_Memory = true;
  }
  // Check for new max uptime.
  if (AwakeMinutes_Current > AwakeMinutes_Max) {
    AwakeMinutes_Max = AwakeMinutes_Current;
    Update_Memory = true;
  }
  // Check for new max distress count.
  if (Distress_Count  > Distress_Max) {
    Distress_Max  = Distress_Count;
    Update_Memory = true;
  }
  // Check for new max uptime at first distress.
  if (Distress_First>Distress_Latest) {
    Distress_Latest = Distress_First;
    Update_Memory = true;
  }
  // Check for new max sounds played count.
  if (Played_Count > Played_Max) {
    Played_Max = Played_Count;
    Update_Memory = true;
  }
  // Upddate total lifetime.
  if (LifeMinutes_Saved != LifeMinutes) {
    Update_Memory = true;
  }
}

// === FEELINGS ===
// FEELINGS functions give basic sentience to the program.

void Interrupt_Feelings () {
  Expressed_Expression = 0;
  Expressed_CntDn = -1;
}
// Handle user interrupting expression of feelings.
bool Feelings_Interrupted () {
  if (ActionPressed) {  // Feelings Interrupted.
    Interrupt_Feelings();
    Expressed_State = LOW;
    digitalWrite(LED_pin, Expressed_State);
    return (true);
  }
  return (false);
}

// Express level of contentment.  Modulates LED, incrementally processing contentment digits parsed off right until empty.
void Feelings_Express () {
  if (Feelings_Interrupted()) { return; }
  if (Expressed_CntDn >= 0) {  // <0 for idle.
    Expressed_CntDn--;
    // Tweak to put tiny off blips within long on states.
    if (Expressed_State == HIGH) {
      if (Expressed_CntDn >= Expressed_BlinkDur) {  // Not last bit.
        if (Expressed_CntDn % Expressed_BlinkDur == 2) { digitalWrite(LED_pin, LOW); }
        if (Expressed_CntDn % Expressed_BlinkDur == 0) { digitalWrite(LED_pin, HIGH); }
      }
    }
    if (Expressed_CntDn < 0) {
      if (Expressed_State == HIGH) {  // Express a brief LED off between digits.
        Expressed_State = LOW;
        Expressed_CntDn = Expressed_BlankDur;
      } else {
        // Express current digit through duration of LED ON phase.
        if (Expressed_Expression < 1) {  // Nothing left to express.
          Expressed_Count++;
          Expressed_CntDn = -1;
          Expressed_State = LOW;
          Memory_Service();
        } else {
          // Parse off rightmost digit to set duration of ON phase.
          Expressed_CntDn = Expressed_BlinkDur * (Expressed_Expression % 10);
          if (Expressed_CntDn<1) { Expressed_BlinkDur / 3; }  // Extremely short blink for zero.
          Expressed_Expression = Expressed_Expression / 10;  // Removes last digit from expression.
          Expressed_State = HIGH;
        }
      }
      digitalWrite(LED_pin, Expressed_State);
    }
  }
}

// Queue the contentment message for expression.
void Express_Feelings () {
  // Skip if already expressing.
  if ((Expressed_Expression < 1) && (Expressed_CntDn < 0)) {
    if (Expressed_Contentment > 0) {
      // Expression is always [1-3][1-3][1-3][1-3] SpaceLightTouchPower four digits no zeroes
      Expressed_Expression = Expressed_Contentment;
      // Remember this feeling.
      Memories_Hunger = (Expressed_Expression % 10);
      Memories_Touch = ((Expressed_Expression / 10) % 10);
      Memories_Light = ((Expressed_Expression / 100) % 10);
      Memories_Space = ((Expressed_Expression / 1000) % 10);
      Memories_Idx = ((Memories_Hunger - Feeling_Bad) * (Feeling_Good*Feeling_Good*Feeling_Good)) + ((Memories_Touch - Feeling_Bad) * (Feeling_Good*Feeling_Good)) + ((Memories_Light - Feeling_Bad) * Feeling_Good) + (Memories_Space - Feeling_Bad);
      if ((Memories_Idx >= 0) && (Memories_Idx < (Feeling_Good*Feeling_Good*Feeling_Good*Feeling_Good))) {
        Memories_Expression = (Memories_Hunger * 1000) + (Memories_Touch * 100) + (Memories_Light * 10) + Memories_Space;
        sprintf(Memories_Key, "X%d", Memories_Expression);
        Memories[Memories_Idx]++;
        if ((Analyze_Flags & Analyze_Memories) == Analyze_Memories) {
          Serial.printf("Memories: %s(%d)=%d\n", Memories_Key, Memories_Idx, Memories[Memories_Idx]);
        }
      }
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Expressing %d\n", Memories_Expression); }
      Expressed_CntDn = Expressed_BlinkDur;
    }
  }
}

// Feel all the feelings, setting the level of contentment.
void Feel_Feelings () {
  Expressed_Contentment = 0;
  Expressed_DigitMult = 1;
  // HUNGER ===== Read battery voltage power level sensor.
  // Pause to feel it several times.
  // Gives the device a sense of its food security, inverse of hunger.
  Sense_Total = 0;
  Sense_Moments = 1 + random(3);
  for (Sense_Moments_Cnt = 1; Sense_Moments_Cnt <= Sense_Moments; Sense_Moments_Cnt++) {
    Sense_Total += analogRead(Power_pin);  // Scaled 0-4095=0-Vref.  Voltage divided 1/2 from "5v"(battery pin) for ~0-2.5
    if (Feelings_Interrupted()) { return; }
    if (Sense_Moment > 0) delay(Sense_Moment);
  }
  Sense_Hunger = Sense_Total / Sense_Moments;
  Power_Raw = Sense_Hunger;
  Sense_Hunger = (Sense_Hunger * 1000.0) / Power_ADC_VoltScaling;  // Converts raw value to voltage divider input millivolts.
  if ((Analyze_Flags & Analyze_Hunger) == Analyze_Hunger) {
    if (AwakeMinutes_Current < 10) { Sense_Hunger = Hungry; } else { Sense_Hunger = VeryHungry; }
  }
  Sense_Scaled = (Sense_Hunger <= VeryHungry) ? Feeling_Bad : ((Sense_Hunger <= Hungry) ? Feeling_Neutral : Feeling_Good);  // Battery >3.3v=happy, <=~2.7v=unhappy
  if (Sense_Scaled > Feeling_Neutral) {
    // Power fine, not distressed.
    Distress_CntDn = -1;
  } else {
    // Low power detected (hungry).
    if (Distress_CntDn < 0) { // If not currently distressed, set an alert countdown.
      Distress_CntDn = Distress_Dur - (Distress_Dur / 3.0) + random((2.0 * (Distress_Dur / 3.0)));  // Duration randomly varies +-1/3
      if (Sense_Scaled < Feeling_Neutral) { Distress_CntDn = Distress_CntDn / 2; }  // Faster distress when very hungry.
      if ((Analyze_Flags & Analyze_Hunger) == Analyze_Hunger) { Distress_CntDn = Distress_CntDn / 4; }
    }
  }
  Expressed_Contentment = (Sense_Scaled * Expressed_DigitMult) + Expressed_Contentment;
  Expressed_DigitMult *= 10;
  // TOUCH ===== Read touch sensor, senses capacitive change.
  // Gives the device an awareness of being touched with some degree of nuance.
  // Actually read in user interface routine as part of Tap detection.
  // Scaled so harder press (150% of threshold) = happier
  Sense_Scaled = (Sense_Touch >= Touch_Average+(Touch_Delta_Min_Rise/2)) ? Feeling_Bad : (Sense_Touch <= Touch_Average-(Touch_Delta_Min_Rise/2)) ? Feeling_Good : Feeling_Neutral;
  // Gets a bump if last touch was recent but not too recent.
  if ( (LastTouched_Secs_Prev > 10) && (LastTouched_Secs_Prev < LastTouched_LonelySecs) ) {
    if (Sense_Scaled<Feeling_Good) {
      // if ((Analyze_Flags & Analyze_Touch) == Analyze_Touch) { Serial.printf("Recent touch bump.\n"); }
      Sense_Scaled++;
    }
  }
  Expressed_Contentment = (Sense_Scaled * Expressed_DigitMult) + Expressed_Contentment;
  Expressed_DigitMult *= 10;
  // LIGHT ===== Read light sensor as a basic sense of sight.
  // Gives the device a feeling of changes in shadows.
  // Photo resistor voltage divider 10-100K to gnd, 100K bias resistor to 3.3v.  Lower=brighter.
  Sense_Total = 0;
  // Pause to sense it several times.
  Sense_Moments = 1 + random(3);
  for (Sense_Moments_Cnt = 1; Sense_Moments_Cnt <= Sense_Moments; Sense_Moments_Cnt++) {
    Sense_Total += analogRead(Light_pin);  // Scaled 0-4095=0-3.3v
    if (Feelings_Interrupted()) { return; }
    if (Sense_Moment > 0) delay(Sense_Moment);
  }
  Sense_Light = Sense_Total / Sense_Moments;
  // Dynamically ranged since powerup.
  if ( (Sense_Light > 100) && ((Sense_Light < Light_Lowest) || (Light_Lowest < 0))) Light_Lowest = Sense_Light;
  if (Sense_Light > Light_Highest) Light_Highest = Sense_Light;
  Light_MidLow = Light_Lowest + ((Light_Highest - Light_Lowest) / 2); // Below this is happy
  Light_MidHigh = Light_Highest - ((Light_Highest - Light_Lowest) / 3);
  Sense_Scaled = (Sense_Light <= Light_MidLow) ? Feeling_Good : ((Sense_Light > Light_MidHigh) ? Feeling_Bad : Feeling_Neutral);  // Brighter=Happier
  Light_Debounce_Read = Sense_Scaled;
  Expressed_Contentment = (Sense_Scaled * Expressed_DigitMult) + Expressed_Contentment;
  Expressed_DigitMult *= 10;
  // SPACE ===== Read internal Hall effect sensor to read local magnetic field.
  // Pause to sense it several times, as if to look around.
  // Gives the device an tiny awareness of its environment.
  Sense_Read = 0;
  Sense_Total = 0;
  Sense_Moments = 1 + random(3);
  for (Sense_Moments_Cnt = 1; Sense_Moments_Cnt <= Sense_Moments; Sense_Moments_Cnt++) {
    Sense_Read = hall_sensor_read();
    Sense_Total += abs(Sense_Read);
    if (Feelings_Interrupted()) { return; }
    if (Sense_Moment > 0) delay(Sense_Moment);
  }
  Sense_Space = Sense_Total / Sense_Moments;
  // Dynamically ranged since powerup.  Lower=more space.
  if ((Sense_Space < Space_Lowest) || (Space_Lowest < 0)) Space_Lowest = Sense_Space;
  if (Sense_Space > Space_Highest) Space_Highest = Sense_Space;
  Space_MidLow = Space_Lowest + ((Space_Highest - Space_Lowest) / 3);
  Space_MidHigh = Space_Lowest + (((Space_Highest - Space_Lowest) / 3) * 2);
  // Higher value (more magnetism nearby?) = happier.
  Sense_Scaled = (Sense_Space < Space_MidLow) ? Feeling_Bad : ((Sense_Space < Space_MidHigh) ? Feeling_Neutral : Feeling_Good);
  Expressed_Contentment = (Sense_Scaled * Expressed_DigitMult) + Expressed_Contentment;
  Expressed_DigitMult *= 10;
  // ===== Done feeling.
  // Rest for just a moment, suspending main program execution (tho still serving autonomic (ISR) functions).
  Sense_Moments = 1 + random(3);
  if (Sense_Moment > 0) delay(Sense_Moments * Sense_Moment);
  if ((Analyze_Flags & Analyze_Verbose) == Analyze_Verbose) {
    Serial.printf("Hunger:%d(%d)  Touch:%d  Light:%d(%d:%d<%d)  Space:%d  =%d\n", Sense_Hunger, Power_Raw, Sense_Touch,
       Sense_Light, Light_Debounced, Light_MidLow, Light_MidHigh,
       Sense_Space, Expressed_Contentment);
  }
  return;
}

// Start an LED blink of specified duration.  Timed out asynchronously.
void LED_Blink (int LB_dur) {
  digitalWrite(LED_pin, HIGH);
  LED_BlinkCnt = LB_dur;
}

// === TWISTER ===
// Multitap during first second after awakening runs automatic talking Twister Spinner mode.

// Include twister words.
#include "words\left.h"
#include "words\right.h"
#include "words\foot.h"
#include "words\hand.h"
#include "words\blue.h"
#include "words\green.h"
#include "words\red.h"
#include "words\yellow.h"

// Play twister word 0-7=left,right,foot,hand blue,green,red,yellow
void Twister_Word (int Twister_SoundNum) {
  I2S_Stop();
  switch (Twister_SoundNum) {
    case 0:
      WavSrc_Ptr = wav_left;  WavSrc_Size = left_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying left\n"); }
      break;
    case 1:
      WavSrc_Ptr = wav_right;  WavSrc_Size = right_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying right\n"); }
      break;
    case 2:
      WavSrc_Ptr = wav_foot;  WavSrc_Size = foot_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying foot\n"); }
      break;
    case 3:
      WavSrc_Ptr = wav_hand;  WavSrc_Size = hand_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying hand\n"); }
      break;
    case 4:
      WavSrc_Ptr = wav_blue;  WavSrc_Size = blue_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying blue\n"); }
      break;
    case 5:
      WavSrc_Ptr = wav_green;  WavSrc_Size = green_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying green\n"); }
      break;
    case 6:
      WavSrc_Ptr = wav_red;  WavSrc_Size = red_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying red\n"); }
      break;
    case 7:
      WavSrc_Ptr = wav_yellow;  WavSrc_Size = yellow_raw_len;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Saying yellow\n"); }
      break;
  }
  Interrupt_Feelings();
  I2S_Play();
  LED_Blink(LED_Play_BlinkDur);
}

// Checked from consciousness service.
void Twister_Service () {
  // Counts by centisecs between words, but by seconds between spins.
  if ( (Twister_Part != 0) || (Seconds_Prev != Seconds_Curr) ) { Twister_CntDn--; }
  if (Twister_CntDn < 0) {
    if (Twister_Part == 0) {
      Twister_WordNum = random(2);
      Twister_CntDn = Twister_Dur;
    }
    if (Twister_Part == 1) {
      Twister_WordNum = 2 + random(2);
      Twister_CntDn = Twister_Dur;
    }
    if (Twister_Part == 2) {
      Twister_WordNum = 4 + random(4);
      Twister_CntDn = Twister_Pause_Sec;
    }
    Twister_Word(Twister_WordNum);
    Twister_Part = (Twister_Part + 1) % 3;
  }
}

// === MAIN ROUTINES ===

// Main periodic services handler, runs ~100/sec.  Timing flagged from autonomic service when not too busy.
// Checks and debounces inputs, controls LED, plays sounds, handles interactions, ponders senses and feelings.
// Several separate timers handled for services with varying asynchronous frequencies and priorities.
void Concious_Functions () {
  // Track uptime actual seconds.
  time(&AwakeMinutes_now);
  localtime_r(&AwakeMinutes_now, &AwakeMinutes_timeinfo);
  Minutes_Curr = AwakeMinutes_timeinfo.tm_min;
  if (Minutes_Prev != Minutes_Curr) {
    AwakeMinutes_Current++;
    LifeMinutes++;
    Light_Debounces = 0;  // Counted in a given minute.
  }
  Seconds_Curr = AwakeMinutes_timeinfo.tm_sec;
  if (Seconds_Prev != Seconds_Curr) { AwakeMinutes_Seconds++; }

  // Handle timing blink at start of sound.
  if (LED_BlinkCnt > 0) {
    LED_BlinkCnt--;
    if (LED_BlinkCnt < 1) { digitalWrite(LED_pin, LOW); }
  }
  // Handle all user interaction types: touch, button, multitap, etc

  Touch_Read = touchRead(Touch_pin);

  // Buffer touch readings to smooth the average.
  if (Touch_Read >= Touch_Read_Min) {
    Touch_Buf[Touch_Buf_Idx] = Touch_Read;
    if (Touch_Buf_Cnt < Touch_Buf_Dur) { Touch_Buf_Cnt++; }
    if (Touch_Buf_Cnt >= Touch_Buf_Dur) {
      Touch_Buf_Back_Cnt = 0;  Touch_Buf_Back_Idx = Touch_Buf_Idx;
      Touch_Average = 0;
      Touch_Debounced_Average = 0;  Touch_Debounced = 0;  Touch_Debounced_Prev = 0;
      for (Touch_Buf_Lp=0; Touch_Buf_Lp<Touch_Buf_Dur; Touch_Buf_Lp++) {
        Touch_Average = Touch_Average + Touch_Buf[Touch_Buf_Lp];
        Touch_Buf_Back_Cnt++;
        if (Touch_Buf_Back_Cnt <= Touch_Debounce_Dur_On) {
          Touch_Debounced += Touch_Buf[Touch_Buf_Back_Idx];
        } else {
          if (Touch_Buf_Back_Cnt <= Touch_Debounce_Dur_On+Touch_Debounce_Dur_Off) {
            Touch_Debounced_Prev += Touch_Buf[Touch_Buf_Back_Idx];
          }
        }
        Touch_Buf_Back_Idx--;
        if (Touch_Buf_Back_Idx < 0) { Touch_Buf_Back_Idx = Touch_Buf_Dur-1; }
      }
      Touch_Average = Touch_Average / Touch_Buf_Cnt;
      Touch_Debounced = Touch_Debounced / Touch_Debounce_Dur_On;
      Touch_Debounced_Prev = Touch_Debounced_Prev / Touch_Debounce_Dur_Off;
    }
    Touch_Buf_Idx = (Touch_Buf_Idx+1) % Touch_Buf_Dur;
  }  
  Touch_Delta = Touch_Debounced_Prev - Touch_Debounced;
  if (Touch_Delta >= Touch_Delta_Min_Drop) { Touch_Debounced_Out = 1; }
  Touch_Delta = Touch_Debounced - Touch_Debounced_Prev;
  if (Touch_Delta >= Touch_Delta_Min_Rise) { Touch_Debounced_Out = 0; }

  // Sense of touch gathered here for Feelings, as it's untouched moments later when feelings are assembled.
  if (Touch_Debounced_Out > 0) {
    if ((Sense_Touch_Prev < 0) || (Touch_Average < Sense_Touch)) {
      Sense_Touch_Prev = 1;
      Sense_Touch = Touch_Average;
    }
  } else {
    Sense_Touch_Prev = -1;  // Toggles capture off, latching touch value.
  }
  if ((Analyze_Flags & Analyze_Touch) == Analyze_Touch) {
    sprintf(Touch_Analyze, "Touch: State:%d  Avg:%3d  Sense:%3d  %d|%d /%d (%d)\n",
       Touch_Debounced_Out, Touch_Average, Sense_Touch,
       Touch_Debounced, Touch_Debounced_Prev,
       Touch_Buf_Cnt, Expressed_Contentment);
    if (strcmp(Touch_Analyze, Touch_Analyze_Prev) != 0) {
      strcpy(Touch_Analyze_Prev, Touch_Analyze);
      Serial.printf(Touch_Analyze);
    }
  }

  // Debounce button and touch by requiring three consecutive same states a tick apart.
  ButtonDebounce_prevprev = ButtonDebounce_prev;  ButtonDebounce_prev = ButtonDebounce_Read;
  ButtonDebounce_Read = (digitalRead(Button_pin) < 1);
  if ((ButtonDebounce_Read == ButtonDebounce_prev) && (ButtonDebounce_Read == ButtonDebounce_prevprev)) { ButtonDebounced_out = ButtonDebounce_Read; }

  // Fetch and store button state.
  ActionPressed_Prev = ActionPressed;
  ActionPressed = ( (ButtonDebounced_out) || (Touch_Debounced_Out>0) );
  // Count quick taps close together.
  if (Mode_Timer>=0) Mode_Timer--;  // Times out multitap for mode select after awakening.
  if (ActionPressed && (!ActionPressed_Prev)) {
    if (MultiTap_CntDn > 0) { MultiTaps++; }
    MultiTap_CntDn = MultiTap_Dur;
    // Multitap handler.
    if (MultiTaps > MultiTaps_Volume) {
      MultiTaps = 0;
      // MultiTap_CntDn = -1;
      // Quick taps during first moments after awakening goes to Twister mode
      if (Mode_Timer>0) {
        Mode_Timer = 0;
        if (MainMode == Mode_Bell) {
          if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Mode changed to Twister.\n"); }
          MainMode = Mode_Twister;
          Hold_Max_Sec_Eff = Twister_Pause_Sec * 0.7;  // Stuck button detect set just under pause between spins
          Twister_CntDn = Twister_Dur;
          Twister_Part = 0;
        } else {
          // Possibly (multitap after stuck button in Twister) Switch back to bike bell.
          if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Mode changed to Bell.\n"); }
          Memory_Forget();  // Special hidden action, clears some stats.
          MainMode = Mode_Bell;
          Hold_Max_Sec_Eff = Hold_Max_Sec;
        }
      } else {
        // Quick taps decreases volume.
        if (Vol_Dec < Vol_Dec_Max) {  // Volume decrementer, several steps of quieting.  0=max vol(clips)
          Vol_Dec++;
        } else {
          Vol_Dec = 0;
        }
        Update_Memory = true;
      }
    }
  }

  // Clear multitaps if opportunity expired,
  if (MultiTap_CntDn >= 0) {
    MultiTap_CntDn--;
    if (MultiTap_CntDn < 0) { MultiTaps = 0; }
  }
  // Increase loneliness.
  if ( (! ActionPressed) && (! ActionPressed_Prev) ) {
    if (Seconds_Prev != Seconds_Curr) {
      LastTouched_Secs++;
    }
  }
  // Detect stuck button.
  if (ActionPressed) {
    if (Seconds_Prev != Seconds_Curr) { Hold_Curr_Sec++; }
    if (Hold_Curr_Sec>=Hold_Max_Sec_Eff) {
      if (! Button_Stuck) {
        Button_Stuck = true;
        if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Stuck button.\n"); }
        LED_Blink(Thought_Freq);  // Slow blink.
        // Formerly: ESP.restart();
        // Formerly: MainMode = Mode_Unconscious;
      }
    }
  } else {
    // Hidden opportunity to change mode select after button unstuck
    if (Button_Stuck) { Mode_Timer = Mode_Timeout; }
    Button_Stuck = false;
    Hold_Curr_Sec = 0;
  }
  // Debounce light (for motion detect) requiring three consecutive same state reads, with adequate range.
  if ((Light_MidHigh - Light_MidLow) > 30) {
    if ( (Light_Debounce_Read == Light_Debounce_prev) && (Light_Debounce_Read == Light_Debounce_prevprev) ) {
      Light_Debounced = Light_Debounce_Read;
    }
    Light_Debounce_prevprev = Light_Debounce_prev;  Light_Debounce_prev = Light_Debounce_Read;
  }
  // Button, multitap, stuck, etc flags set above for any more.

  // === MAIN MODE - Play selected bike bell sounds. ===
  if (MainMode == Mode_Bell) {
    // Handle skip function.
    if (Skip_CntDn > 0) {
      Skip_CntDn--;
      if (Skip_CntDn < 1) {  // Timed out and still pressed
        if (ActionPressed) { SkipTrack = true;  Skipping = true; }
      }
      if (Skip_CntDn == (SkipTimeFirst - SkipHinted)) {
        Vol_Curr = Vol_Skip;  // Decrease volume in advance of skipping.
      }
    }
    // Restore full volume and reset skipping whenever button is up.
    if ((ActionPressed_Prev) && (!ActionPressed)) {
      Vol_Curr = Vol_Normal;
      if (Skip_CntDn>0) { I2S_Wav_Prep(); }  // Retranscode buffer to adjust volume back up if was still playing a skip.
      Skip_CntDn = -1;  Skipping = false;
      LastTouched_Secs_Prev = LastTouched_Secs;  LastTouched_Secs = 0;
    }
    // Handle skip flag.
    I2S_Playing_Prev = I2S_Playing;
    if (SkipTrack) {
      SkipTrack = false;
      if (! Button_Stuck) {
        if (I2S_Playing) I2S_Stop();
        I2S_Playing_Prev = false;
        Vol_Curr = Vol_Skip;  // Decrease volume while skipping.
        SoundNum++;
        if (SoundNum >= SoundsCnt) {
          SoundNum = 0;
          Skip_CntDn = 2 * SkipTimeContinue;  // Pause skipping longer on first sound to indicate looped.
        } else {
          Skip_CntDn = SkipTimeContinue;  // Continue skipping
        }
        Skipping = true;
        LED_Blink(LED_Play_BlinkDur);  // Flick LED.
        Update_Memory = true;
      }
    }
    // Restart sound if button re-pressed and not skipping.
    if (Skip_CntDn < 1) {
      if (I2S_Playing) {
        if (ActionPressed && (!ActionPressed_Prev)) {
          if (I2S_Playing) { I2S_Stop(); }
          I2S_Playing_Prev = false;
        }
      }
    }
    // If not playing, check for play button.
    if (!I2S_Playing) {
      if (ActionPressed) {
        if (! Button_Stuck) {
          // Start sound.
          Sound_Select();
          Interrupt_Feelings();
          if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Playing (%d)\n",Played_Count); }
          I2S_Play();
          // Start (or restart) sound on DAC (unless skip or multitap)
          if ( (! Skipping) && ( (DAC_Idx<0) || (MultiTaps<(MultiTaps_Volume-1)) ) ) {
            DAC_Ptr = WavSrc_Ptr;  DAC_Size = WavSrc_Size;  DAC_Idx = 0;
          }
          // Blink if not skipping
          if (Skip_CntDn < 1) {
            Skip_CntDn = SkipTimeFirst;
            LED_Blink(LED_Play_BlinkDur);
          }
        }
      }
    }
  }

  // === TWISTER MODE ===
  // "Automatic Hands-Free Talking Twister Spinner" mode.  Speaks random Twister game moves ("spins") periodically.

  if (MainMode == Mode_Twister) {
    if (ActionPressed && (!ActionPressed_Prev)) {
      // Tap forces immediate spin.
      Twister_CntDn = -1;
      Twister_Part = 0;
    }
    if (Button_Stuck) {
      Twister_Part = 0;
      Twister_CntDn = sqrt(Twister_Pause_Sec);  // Restart autospin soon when unstuck.
    } else {
      Twister_Service();
    }
  }

  // FEELINGS services
  // Collects senses and judge contentment when not busy with user.
  if (((Analyze_Flags & Analyze_Dumb) != Analyze_Dumb) && (Skip_CntDn < 1) && (!ActionPressed)) {
    // Don't feel while expressing.
    if ((Expressed_Expression < 1) && (Expressed_CntDn < 0)) {
      if (Feel_CntDn >= 0) {
        Feel_CntDn--;
        if (Feel_CntDn < 0) {
          Feel_Feelings();
          Feel_CntDn = Feel_Interval + random(3);  // Slight random interval variation between feeling.
        }
      }
    }
  }

  // Things to handle when no button pressed.
  if ((Skip_CntDn < 1) && (!ActionPressed)) {

    // Service the expression of pending feelings.
    if (((Analyze_Flags & Analyze_Dumb) != Analyze_Dumb) && ((Expressed_CntDn >= 0) || (Expressed_Expression > 0))) { Feelings_Express(); }

    // Things to handle when completely idle.
    if ((!I2S_Playing) && (Expressed_CntDn < 0) && (Expressed_Expression <= 0)) {

      // Update saved memories if flagged.
      if (Update_Memory) { Memory_Update(); }

      // Handle low battery warning.
      if (Distress_CntDn >= 0) {
        Distress_CntDn--;
        if (Distress_CntDn < 0) {
          Distress_Count++;
          WavSrc_Ptr = wav_distress;
          WavSrc_Size = distress_raw_len;
          // Gets louder over time.
          Vol_Dec_Tmp = Vol_Dec;
          Vol_Curr = Vol_Max;
          Vol_Dec = Vol_Dec_Max;  // Quietest.
          if (Distress_Count > 2) { Vol_Dec = 2; }
          if (Distress_Count > 6) { Vol_Dec = 1; }
          if (Distress_Count > 9) { Vol_Dec = 0; }  // Loudest.
          I2S_Play();
          if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Distress (%d)\n",Distress_Count); }
          Vol_Dec = Vol_Dec_Tmp;
          Vol_Curr = Vol_Normal;
          LED_Blink(LED_Play_BlinkDur);  // Flick LED.
          // Check for new max uptime at first distress.
          if (Distress_First < 1) { Distress_First = AwakeMinutes_Current; }
        }
      }
    }

    // Detect and acknowledge motion nearby (proximity) sometimes.
    // If light has suddenly dropped...
    if (Light_Debounced<Light_Debounced_Prev) {
      if ((Analyze_Flags & Analyze_Light) == Analyze_Light) {
        Serial.printf("Light changed %d/%d\n", Light_Debounced, Light_Debounced_Prev);
      }
      // And hasn't been touched in a while...
      if ( (LastTouched_Secs > LastTouched_LonelySecs) || ((Analyze_Flags & Analyze_Light) == Analyze_Light) ) {
        if (Light_Debounces < 7) { // Only respond a few times in a minute.
          if ( (Light_Debounced_LastTime < 1) || ( (AwakeMinutes_Seconds - Light_Debounced_LastTime) > 2) ) {
            if (LED_BlinkCnt < 1) {
              LED_Blink(1);  // Tiny fastest LED flick
              Light_Debounces++;
              Light_Debounced_LastTime = AwakeMinutes_Seconds;
              // Accelerate expressing distress if pending.
              if (Distress_CntDn>300) {
                Serial.printf("Lonely hunger bump.\n");
                Distress_CntDn = Distress_CntDn / 2;
              }
            }
          }
        }
      }
    }

    // Express battery voltage once, shortly after awakening.
    if ( (Expressed_Count < 1) && (Sense_Hunger > 500) && (Power_Expressed < 1) ) {
      Power_Expressed = Sense_Hunger / 100;
      Expressed_Expression = ((Power_Expressed % 10) * 10) + ((Power_Expressed / 10) % 10);
      Expressed_CntDn = Expressed_BlinkDur;
      if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) { Serial.printf("Expressing power %d\n", Power_Expressed); }
    }

  }
  Light_Debounced_Prev = Light_Debounced;
  Minutes_Prev = Minutes_Curr;  Seconds_Prev = Seconds_Curr;
}

// MAIN SETUP
void setup () {
  if (Analyze_Flags >= Analyze_Basic) {
    Serial.begin(115200);
    delay(500);
  }
  // Config LED and button pins.
  pinMode(LED_pin, OUTPUT);
  pinMode(Button_pin, INPUT_PULLUP);
  setCpuFrequencyMhz(CPU_MHz);
  CPU_MHz = getCpuFrequencyMhz();
  APB_Freq = getApbFrequency();
  if ((Analyze_Flags&Analyze_Basic)==Analyze_Basic) {
    Serial.printf("\n\n\n\n\n");
    Serial.printf("CPU Freq: %d\n", CPU_MHz);
    Serial.printf("APB Freq: %d\n", APB_Freq);
    Serial.printf("Xtal Freq: %d\n", getXtalFrequencyMhz());
  }
  if ((Analyze_Flags & Analyze_Slow) == Analyze_Slow) {
    Feel_Interval *= 5;
    Expressed_BlinkDur *= 3;
  }
  if ((Analyze_Flags & Analyze_Hunger) == Analyze_Hunger) { LastTouched_LonelySecs = sqrt(LastTouched_LonelySecs); }
  Memory_Awaken();  // Loads preferences, stats and memories
  Sound_Setup();
  // Set scaling for power sensor DAC.
  adc2_config_channel_atten( Power_channel, ADC_ATTEN_11db );
  // Set scaling for light sensor DAC.
  adc1_config_channel_atten( Light_channel, ADC_ATTEN_11db );
  // Almost done.
  // LED_Blink(LED_Play_BlinkDur);  Formerly did power on blink, now handled by initial expression of battery level.
  delay(30);  // Needs an moment before stable enough for audio ISR or crashes.
  // I2S_Play();  No longer plays a sound on awakening to prevents low power reboot loop.
  if (Thought_Freq > 0) {  // 0=autonomic functions only
    Thought_Dur = I2S_SampleRate / Thought_Freq;
    Thought_CntDn = Thought_Dur;  // Awaken consciousness.
  }
  if ((Analyze_Flags & Analyze_Dumb) != Analyze_Dumb) { Feel_CntDn = Feel_Interval; }  // Start feeling feelings.
}

// MAIN loop handler separated to function so it can be substituted in whole for testing.
void MAIN_loop () {
  if ( (MainMode != Mode_Unconscious) && (Service_Thoughts) ) {
    Concious_Functions();
    Thought_CntDn = Thought_Dur;
    Service_Thoughts = false;
  }
}

// MAIN LOOP
void loop () {
  MAIN_loop();
}
