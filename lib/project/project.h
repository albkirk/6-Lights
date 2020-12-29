// **** Project code definition here ...
#include <ambient.h>
#include <Arduino.h>
#include <Wire.h>
#include "arduinoFFT.h" // Standard Arduino FFT library
// https://github.com/kosme/arduinoFFT, in IDE, Sketch, Include Library, Manage Library, then search for FFT



/////////////////////////////////////////////////////////////////////////
// Comment out the display your nNOT using e.g. if you have a 1.3" display comment out the SSD1306 library and object
//#include "SH1106.h"     // https://github.com/squix78/esp8266-oled-ssd1306
//SH1106 display(0x3c, D3,D4); // 1.3" OLED display object definition (address, SDA, SCL)

#include "SSD1306.h"  // https://github.com/squix78/esp8266-oled-ssd1306
SSD1306 display(0x3c, 5,4);  // 0.96" OLED display object definition (address, SDA, SCL) 
/////////////////////////////////////////////////////////////////////////
//arduinoFFT FFT = arduinoFFT();
/////////////////////////////////////////////////////////////////////////
#define SAMPLES 128                         //Must be a power of 2
#define SAMPLING_FREQUENCY 6000             //Hz, must be 8000 or less due to ADC conversion time (~120ms).
                                            // Half of this value is the maximum frequency that can be analysed by the FFT.
#define Data_Size 64                        // number of samples on each data buffer
#define buf_max 3 // SAMPLES / Data_Size + 1   // number of buffers
#define BAR_Max 50                          // Display Bar Graph Max height value
#define sampling_period_ticks 80000000 / SAMPLING_FREQUENCY  - 160      // 80MHz ~1 sec and 160 is ~2us (average processing delay)

double Sensitivity = 2.5;                   // Audio sensitivity (whippers or LOUD!)
double Gain_Factor = 10;                    // Gain applied after FTT Magnitude's Logaritm of 2 calculation

static int peak[] = {0,0,0,0,0,0,0};
static int level[] = {0,0,0,0,0,0,0};
int LightPIN[] = {0,12,13,14,15,16, -1};    //   Light PWM PIN number. -1 Means no PIN.
static int pwmvalue = 0;

struct __attribute__((__packed__)) buffer_struct {
    int Data[Data_Size];
    //unsigned long smpltime[Data_Size];
    bool Buf_read_done;
    bool dummy1 = true;     // packed struct size must be multpliple of 4 bytes
    bool dummy2 = true;     // packed struct size must be multpliple of 4 bytes
    bool dummy3 = true;     // packed struct size must be multpliple of 4 bytes
};

static int Data_index = 0;
static uint8_t buf_slot = 0;
//static unsigned long start = 0;
static buffer_struct Buffer[buf_max];

double vReal[SAMPLES];
double vImag[SAMPLES];
volatile double Peak_Freq;
volatile double Peak_Magnitude;
static uint8_t blk_idx = buf_max - 2;            // Buffer Block index
static uint8_t pblk_idx = 0;                     // Previous Buffer Block index
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
/////////////////////////////////////////////////////////////////////////

// **** Project code functions here ...
void project_hw() {
 // Output GPIOs

 // Input GPIOs

}

void ICACHE_RAM_ATTR sampling_sound() {
    timer0_write( ESP.getCycleCount() + sampling_period_ticks);
    //start = micros();
    if (Buffer[buf_slot].Buf_read_done) {
        Buffer[buf_slot].Data[Data_index] = analogRead(A0);
        //Buffer[buf_slot].smpltime[Data_index] = micros();
        if (Data_index < Data_Size) {
            Data_index ++;
        }
        else {
            Data_index = 0;
            Buffer[buf_slot].Buf_read_done = false;
            buf_slot=(buf_slot+1)%buf_max;
        }
    }
}

void sensitivity_calib() {
    int Peak_avg = 0;
    for (size_t band = 0; band <= 6; band++) Peak_avg += peak[band];
    Peak_avg = Peak_avg / 6;
    if (Peak_avg < 35) Sensitivity = Sensitivity + 0.1;
    if (Peak_avg > 40) Sensitivity = Sensitivity - 0.1;
    if (Sensitivity > 2.5) Sensitivity = 2.5;                 // for lin use 10
    if (Sensitivity < 0.0 ) Sensitivity = 0.0;
    //Serial.println(); Serial.print("Sensitivity= "); Serial.println(Sensitivity);

}

void set_level_peak(int band, int dsize){
    if (dsize > BAR_Max) dsize = BAR_Max;
    if (dsize > level[band]) {level[band] = dsize;}
    if (dsize > peak[band]) {peak[band] = dsize;}
}

void band_linear_level_peak_calculator() {
    for (size_t band = 0; band <= 6; band++) level[band] = 0;    // Reset Levels back to value 0
    /*
    //for (size_t i = 0; i < 4          ; i++ ) {Serial.print((int)(vReal[i]*Sensitivity*Gain_Factor)); Serial.print(", ");}
    //for (size_t i = 4; i < (SAMPLES/2); i+=4) {Serial.print((int)(vReal[i]*Sensitivity*Gain_Factor)); Serial.print(", ");}
    for (size_t i = 0; i < (SAMPLES/2); i++ ) {Serial.print((int)(vReal[i]*Sensitivity*Gain_Factor)); Serial.print(", ");}
    Serial.println();
    */

    for (int i = 1; i < (SAMPLES/2); i++){ // Don't use sample 0 (DC signal) and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the Gain_Factor.
        if ((vReal[i]*Sensitivity*Gain_Factor) > 5) { // Add a crude noise filter, VReal should Peak values of ~100
            if (i<=1 )            set_level_peak(0,(int)(vReal[i]*Sensitivity*Gain_Factor)); // 47Hz     63Hz    1   2
            if (i >1  && i<=  3 ) set_level_peak(1,(int)(vReal[i]*Sensitivity*Gain_Factor)); // 94Hz     125Hz   3   5
            if (i >3  && i<=  5 ) set_level_peak(2,(int)(vReal[i]*Sensitivity*Gain_Factor)); // 188Hz    250Hz   5   9
            if (i >5  && i<=  9 ) set_level_peak(3,(int)(vReal[i]*Sensitivity*Gain_Factor)); // 375Hz    500Hz   9   17
            if (i >9  && i<= 17 ) set_level_peak(4,(int)(vReal[i]*Sensitivity*Gain_Factor)); // 750Hz    1000Hz  17  33
            if (i >17 && i<= 33 ) set_level_peak(5,(int)(vReal[i]*Sensitivity*Gain_Factor)); // 1500Hz   2000Hz  33  65
            if (i >33 && i<= 65 ) set_level_peak(6,(int)(vReal[i]*Sensitivity*Gain_Factor)); // 3000Hz   4000Hz  65  129 257
        }
    }
}

void band_log_level_peak_calculator() {
    for (size_t band = 0; band <= 6; band++) level[band] = 0;    // Reset Levels back to value 0
    /*
    for (size_t i = 0; i < (SAMPLES/2); i++ ) {Serial.print((int)(vReal[i]*Sensitivity*Gain_Factor)); Serial.print(", ");}
    Serial.println();
    */

    for (int i = 1; i < (SAMPLES/2); i++){ // Don't use sample 0 (DC signal) and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the Gain_Factor.
        if ((log(vReal[i])+Sensitivity-1) > 0) { // Add a crude noise filter, VReal should Peak values of ~100
            if (i<=1 )            set_level_peak(0,(int)((log(vReal[i])+Sensitivity)*Gain_Factor)); // 47Hz     63Hz    1   2
            if (i >1  && i<=  3 ) set_level_peak(1,(int)((log(vReal[i])+Sensitivity)*Gain_Factor)); // 94Hz     125Hz   3   5
            if (i >3  && i<=  5 ) set_level_peak(2,(int)((log(vReal[i])+Sensitivity)*Gain_Factor)); // 188Hz    250Hz   5   9
            if (i >5  && i<=  9 ) set_level_peak(3,(int)((log(vReal[i])+Sensitivity)*Gain_Factor)); // 375Hz    500Hz   9   17
            if (i >9  && i<= 17 ) set_level_peak(4,(int)((log(vReal[i])+Sensitivity)*Gain_Factor)); // 750Hz    1000Hz  17  33
            if (i >17 && i<= 33 ) set_level_peak(5,(int)((log(vReal[i])+Sensitivity)*Gain_Factor)); // 1500Hz   2000Hz  33  65
            if (i >33 && i<= 65 ) set_level_peak(6,(int)((log(vReal[i])+Sensitivity)*Gain_Factor)); // 3000Hz   4000Hz  65  129 257
        }
    }
}

void displayBand(){
    display.clear();
    display.drawString(0,0,"  47  .1  .2 .35  .7 1K5  3K");
    //display.drawString(0,0,"  63 .12 .25  .5  1K  2K  4K");

    for (size_t band = 0; band <= 6; band++) {
        for (int s = 0; s <= level[band]; s=s+2){display.drawHorizontalLine(18*band,64-s, 14);} // Draw Level bar
        display.drawHorizontalLine(18*band,64-peak[band],14);                                   // Draw Peak Line
    }
    display.display();
}

void Lightshow(){
    for (size_t band = 0; band <= 6; band++) {
        pwmvalue = level[band]*5.2;               // Display Mas is 50 so, it should multiply by 5.2 (50*5.1=~255). Adding something more... 
        if (pwmvalue > 255) pwmvalue = 255;
        if (pwmvalue < 0) pwmvalue = 0;
        if (LightPIN[band] >=0) analogWrite((uint8_t)LightPIN[band], pwmvalue);
    }
}


void project_setup() {
    // Start Ambient devices
    ambient_setup();
      //TIMER = 15;                                       // TIMER value (Recommended 15 minutes) to get Ambient data.

    Wire.begin(5,4); // SDA, SCL
    display.init();
    display.setFont(ArialMT_Plain_10);
    display.flipScreenVertically(); // Adjust to suit or remove

    //analogWriteFreq(255);
    //analogWriteRange(255);

    //Initialize audio Buffers with zeros
    for (size_t n = 0; n < buf_max; n++) {
        Buffer[n].Buf_read_done = true;
        for (int i = 0; i < Data_Size; i++) {
            Buffer[n].Data[i] = 512;
        }
    }
    
    //Initialize Sampling timer every sampling_period_us
    timer0_isr_init();
    timer0_attachInterrupt(sampling_sound);
    timer0_write(ESP.getCycleCount() + sampling_period_ticks);
    interrupts();

}

void project_loop() {
    // Ambient handing
    //if (TIMER >0) if ((millis() - 3500) % (TIMER * 60000) < 5) ambient_data();      // (TIMER+1) bigger than zero or dog bites!!
    //if (millis()%10 < 1) { Serial.print(blk_idx); Serial.print("-");Serial.print(buf_slot); Serial.print("->"); }
    if (!Buffer[pblk_idx].Buf_read_done && !Buffer[blk_idx].Buf_read_done) {
        //Serial.print(blk_idx); Serial.print("-");Serial.print(buf_slot); Serial.print("->");
        for (int i = 0; i < Data_Size; i++) {
            //Serial.print(Buffer[blk_idx].smpltime[i+1]-Buffer[blk_idx].smpltime[i]); Serial.print(", "); 
            for (size_t n = 0; n < (buf_max - 1); n++) {
                vReal[Data_Size * (buf_max - 2 - n) + i] = (double)((Buffer[(blk_idx + buf_max - n)%buf_max].Data[i]-512)/51.2);      // Remove off-set and normalize to [-10..10]
                //vReal[Data_Size * (buf_max - 2 - n) + i] = (double)((Buffer[(blk_idx + buf_max - n)%buf_max].Data[i])/102.3);           // Normalize to [0..10]
                vImag[Data_Size * (buf_max - 2 - n) + i] = 0;                                                 // Start IMAG array with Zeros
            }
        }
        //Serial.println("");
        FFT.DCRemoval();
        //for (size_t n = 0; n < 20; n++) {Serial.print(vReal[n]); Serial.print(", ");}Serial.println();    // Time domain values
        FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        //
        FFT.Compute(FFT_FORWARD);
        FFT.ComplexToMagnitude();
        //for (size_t n = 0; n < 20; n++) {Serial.print(vReal[n]); Serial.print(", ");}Serial.println();    // Freq domain values

        // Parametric EQ ---
        for (int i = 1; i <= 1; i++) vReal[i] = vReal[i]*.75;
        for (int i = 2; i <= 4; i++) vReal[i] = vReal[i]*.85;
        //for (int i = 5; i <= 14; i++) vReal[i] = vReal[i]*.90;
        for (int i = (SAMPLES/2)-24; i <= (SAMPLES/2); i++) vReal[i] = vReal[i]*1.20;
        // --- Parametric EQ
        /*
        Peak_Freq = 0;
        Peak_Magnitude = 0;
        for (int i = 2; i < (SAMPLES/2); i++) {
            if (vReal[i] > Peak_Magnitude ) {
                    Peak_Magnitude = vReal[i];
                    Peak_Freq = i;
            }
        }
        Serial.print((int)Peak_Freq); Serial.print("i - "); Serial.print(Peak_Magnitude); Serial.print("\t");
        Serial.println();
        */

        //band_linear_level_peak_calculator();
        band_log_level_peak_calculator();
        displayBand();
        Lightshow();
        pblk_idx = (blk_idx+2)%buf_max;
        Buffer[pblk_idx].Buf_read_done = true;
        blk_idx=(blk_idx+1)%buf_max;
    }
    if ((millis()-23)%1000 < 1) sensitivity_calib();    // Calibrate sensitivity on every second
    if (millis()%100 == 0) {for (byte band = 0; band <= 6; band++) {if (peak[band] > 0) peak[band] -= 1;}} // Decay the peak
    yield();
}

