/*
 * TEF tuner v2
 * Credits RDS routine µTube (https://www.youtube.com/user/KCS1000)
 * 
 *
 * Changelog:
 * v1.00:
 * - Initial version
 * 
 * v2.01:
 * - Small fix in tuner initialisation
 * - After entering frequency manually when in advanced RDS view,
 *   the tuner didn't return to the advanced RDS view.
 * - When powering the tuner with squelch enabled, a short burst of
 *   audio was audible.
 * - If no RDS is available, RDS quality should always be 0%
 * - AUX was not shown after boot and if active
 * - Possibility to swap coaxswitch behaviour on mainboard.
 *   To change press mode button during boot.
 * - Sometimes tuned was incorrect displayed when stereo squelch was used.
 * - UHF module initialisation optimised
 * 
 * v2.02:
 * - Added Longwave, Mediumwave and HF AM reception.
 * - Added standby function, hold BAND button > 1 seconds.
 * - Possible to reverse rotary encoder, press BW button during boot.
  * - Fixed some cosmetic bugs
 * 
 * v2.03: (Nextion update v1.01 needed!)
 * - Radiotext routine is now compatible with old RDS encoders
 * - Fixed AM frequency offset
 * - Fixed a problem which can cause the display to crash
 * - Added options to enable/disable bands
 * - Change in display, top scope shows multipath level, bottom scope shows noise level
 * - Fixed some layout issues on the display
 * - When NOT using the UHF frontend, you can choose to use the 2nd antenna input for AM
*
* v2.04:
* - Fixed a bug which could cause to activate standby when using UHF module
* - Small cosmetic update in Nextion library, new version 1.02
*
* v2.05:
* - Fix AM bandwidth setting on F8605 chipset 
* - Added microcode for whole TEF668x family (Lithio)
* - Added C/N readout
* - Added RT A/B indicator
* - FM signallevel calibrated
* 
* v2.06:
* - Fix wrong C/N meter display when nothing to measure
* - Fixed a problem which causes to write AUTO BW logo all the time and slow things up
* - Fixed a minor problem which could cause WiFi apps not to work
* - Fixed a bug which causes the microcode to not upload correctly
* - Function added to choose between regular and optical rotary encoder (hold rotary button during boot)
* 
* v2.07: (Nextion Display update version 1.04 needed!)
* - RDSSpy: After changing frequency the RDS data will be resetted
* - Added stepsize 10MHz (FM), 1MHz (AM)
* - Clean up RadioText A/B indicator
* - Bugfix: Sometimes after first installation the rotary encoder mode was not working correctly
* - Splash screen shows if bluetooth module is detected
* - You can now toggle between scopes:
*   Option 1: Top scope: FM multipath
*             Low scope: Ultrasonic FM noise
*   Option 2: Top scope: Signal level
*             Low scope: Modulation waveform
*   You can toggle between modes by tapping the scopes. In AM mode option 2 is always enabled.
* - TEF6687/TEF6689: Added FMSI options to DSP menu
* - Added possibility to store up to 30 memorychannels:
*   - You can now switch between MAN/AUTO/MEMORY with the mode button
*   - If you want to store a frequency:
*     1. Tune to the wanted frequency.
*     2. Switch to MEMORY mode.
*     3. Press the rotary encoder.
*     4. Select the memory channel you want to store the frequency to.
*     5. Press the rotary encoder again.
* - Bugfix: Radiotext refresh improved.
* - Added live meters in DSP menu for stereo seperation threshold, high-cut threshold and FMSI bands 1-4.
* - Bugfix: Rotary direction and rotary mode was not working with bandscanner.
* - Hold BW and Rotary encoder button together at startup to recover default settings. (all your settings will be lost!)
* - Reduced data flow to display.
*/

#include <SoftwareSerial.h>
#include "TEF6686.h"
#include "ADF4351.h"
#include <EEPROM.h>
#include <RotaryEncoder.h>          // https://github.com/enjoyneering/RotaryEncoder
#include <EasyNextionLibrary.h>     // https://github.com/Seithan/EasyNextionLibrary
#include <trigger.h>
#include <Hash.h>                   // https://github.com/bbx10/Hash_tng
#include <WiFiClient.h>             //
#include "WiFiConnect.h"            // https://github.com/mrfaptastic/WiFiConnectLite
#include <TimeLib.h>                // https://github.com/PaulStoffregen/Time

WiFiConnect wc;
WiFiServer Server(7373);
WiFiUDP Udp;
WiFiClient RemoteClient;
EasyNex Display(Serial2);
TEF6686 radio;
ADF4351 Frontend;
SoftwareSerial swSer(13,14);

#define ROTARY_PIN_A          36
#define ROTARY_PIN_B          34
#define ROTARY_BUTTON         39
#define PIN_POT               35
#define BWBUTTON              25
#define MODEBUTTON            26
#define RFA                   32
#define RFB                   33
#define RFC                   4
#define BAND                  27
#define VERSION               "v2.07"
#define DISPLAYVER            104

RotaryEncoder encoder(ROTARY_PIN_A, ROTARY_PIN_B, ROTARY_BUTTON);
int16_t position = 0;

void ICACHE_RAM_ATTR encoderISR() {
  encoder.readAB(); 
}
void ICACHE_RAM_ATTR encoderButtonISR() {
  encoder.readPushButton();
}

bool mlogo_on;
bool mlogo_off;
bool slogo_on;
bool slogo_off;
bool mutelogo_on;
bool mutelogo_off;
bool usblogo_on;
bool usblogo_off;
bool mutestatus = true;
bool memorystore;
byte SNRold;
byte SNR;
byte btselect;
byte TEF;
byte scopeview;
bool af2show;
bool btconnect;
bool btsetup;
bool power;
String MAC[10];
String NAME[10];
bool BWreset = true;
bool ctshow;
bool direction;
bool displayreset = false;
bool donesearch = false;
bool EONold;
bool manfreq = false;
bool menu;
bool millisoff = false;
bool MS;
bool RDSSpyReset;
bool RDSSpy;
bool RDSstatus;
bool RDSstatusold;
bool RTPlus;
bool scanfound = false;
bool seek;
bool showrdsinfo;
bool spec = false;
bool SQ;
bool stationlist;
bool Stereostatus;
bool Stereostatusold;
bool StereoToggle = true;
bool store;
bool TA;
bool TAold;
bool TP;
bool TPold;
byte tunemode ;
bool UHF;
bool USBstatus = false;
bool wificonnect;
bool XDRGTK;
bool XDRGTKTCP;
bool XDRMute;
byte af_counterold;
byte af_scan;
byte band;
byte CoaxSwitch;
byte BWset;
byte BWsetAM = 2;
byte ContrastSet;
byte demp;
byte displaysize;
byte ECCold;
byte EQset;
byte change;
byte IF;
byte iMSEQ;
byte iMSset;
byte ip1;
byte ip2;
byte ip3;
byte ip4;
byte lf;
byte optrot;
byte rotarymode;
byte scanner_band;
byte scanner_speed;
byte scanner_th;
byte scanner_thenable;
byte scanstatus;
byte stepsize;
byte usbmode;
byte wifienable;
byte wifienableold;
byte am;
byte fm;
byte fmsi;
byte uhf1;
byte uhf2;
byte uhf3;
byte uhf4;
byte coaxmode;
char buff[16];
char musicArtistPrevious[48];
char musicTitlePrevious[48];
char programServicePrevious[9];
char programTypePrevious[17];
char radioIdPrevious[5];
char radioTextPrevious[65];
char stationEventPrevious[48];
char stationHostPrevious[48];
unsigned int BWOld;
byte BWsetOld = 254;
int HighCutLevel;
int HighCutOffset;
int BlendLevel;
int BlendOffset;
int NBLevel;
int AM_Cochannel;
int AM_NBLevel;
int AM_att;
int HighEdgeSet0;
int HighEdgeSet1;
int HighEdgeSet2;
int HighEdgeSet3;
int HighEdgeSet4;
int LevelOffset0;
int LevelOffset1;
int LevelOffset2;
int LevelOffset3;
int LevelOffset4;
int LowEdgeSet0;
int LowEdgeSet1;
int LowEdgeSet2;
int LowEdgeSet3;
int LowEdgeSet4;
int scanner_filter;
int Squelch;
int Squelchold;
int SStatusold;
byte OStatusold;
int StereoLevel;
int timeoffset;
int VolSet;
int XDRBWset;
int XDRBWsetold;
int displayversion;
int16_t OStatus;
int16_t SStatus;
int16_t SAvg;
IPAddress remoteip;
long rssi;
long rssiold = 1;
String af;
String af2;
String clockPrevious;
String cryptedpassword;
String datePrevious;
String packet;
String password = "12345";
String PIcode;
String rds_clock = "--:--";
String rds_date = "----------";
String salt;
String saltkey = "                ";
uint16_t BW;
uint16_t errorsold;
uint16_t minutesold;
uint16_t MStatus;
uint16_t USN;
uint16_t WAM;
uint16_t highcut;
uint16_t stereo;
uint16_t sthiblend;
uint8_t stband_1;
uint8_t stband_2;
uint8_t stband_3;
uint8_t stband_4;
uint8_t af_counter;
uint8_t buff_pos = 0;
unsigned int fmsi_attack;
unsigned int fmsi_release;
unsigned int fmsi_11;
unsigned int fmsi_12;
unsigned int fmsi_21;
unsigned int fmsi_22;
unsigned int fmsi_31;
unsigned int fmsi_32;
unsigned int fmsi_41;
unsigned int fmsi_42;
unsigned int freq;
unsigned int freq_scan;
unsigned int freqtemp;
unsigned int frequency0;
unsigned int frequency0old;
unsigned int frequency1;
unsigned int frequency1old;
unsigned int frequency2;
unsigned int frequency2old;
unsigned int frequency3;
unsigned int frequency3old;
unsigned int frequency4;
unsigned int frequency4old;
unsigned int frequency5;
unsigned int frequency5old;
unsigned int scanner_end;
unsigned int scanner_found;
unsigned int scanner_start;
unsigned int scanner_vbw;
unsigned int XDRfreq_scan;
unsigned int XDRscanner_end;
unsigned int XDRscanner_filter;
unsigned int XDRscanner_old;
unsigned int XDRscanner_start;
unsigned int XDRscanner_step;
unsigned long showmillis;
unsigned long stlmillis;
unsigned long time_now;
unsigned long XDRshowmillis;
unsigned int memory[30];
byte memory_pos;

void setup() {
  swSer.begin(9600);

  encoder.begin();
  btStop();
  Display.begin(38400);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MODEBUTTON, INPUT);
  pinMode(BAND, INPUT_PULLUP);
  pinMode(BWBUTTON, INPUT);
  pinMode(RFA, OUTPUT);
  pinMode(RFB, OUTPUT);
  pinMode(RFC, OUTPUT);

  Display.writeNum("sleep", 0);
  delay(500);
  EEPROM.begin(351);
  if (EEPROM.readByte(41) != 207) {
    FactoryDefaults();
  }

  if (digitalRead(ROTARY_BUTTON) == LOW && digitalRead(MODEBUTTON) == HIGH && digitalRead(BWBUTTON) == HIGH) {
    optrot = EEPROM.readByte(184);
    EEPROM.commit();
    if (optrot == 1) {
      optrot = 0;
      Display.writeStr("version2.txt", "standard encoder");
    } else {
      optrot = 1;
      Display.writeStr("version2.txt", "optical encoder");
    }
    EEPROM.writeByte(184, optrot);
    EEPROM.commit();
    Display.writeStr("version.txt", "Changed to: ");
    delay(2000);
    Display.writeStr("version.txt", "Please release");
    Display.writeStr("version2.txt", "the button!");
    while (digitalRead(ROTARY_BUTTON) == LOW) {
      delay(50);
    }
  }


  if (digitalRead(MODEBUTTON) == LOW && digitalRead(ROTARY_BUTTON) == HIGH && digitalRead(BWBUTTON) == HIGH) {
    CoaxSwitch = EEPROM.readByte(157);
    EEPROM.commit();
    if (CoaxSwitch == 1) {
      CoaxSwitch = 0;
    } else {
      CoaxSwitch = 1;
    }
    EEPROM.writeByte(157, CoaxSwitch);
    EEPROM.commit();
    Display.writeStr("version.txt", "Coax switch");
    Display.writeStr("version2.txt", "changed!");
    delay(2000);
    Display.writeStr("version.txt", "Please release");
    Display.writeStr("version2.txt", "the button!");
    while (digitalRead(MODEBUTTON) == LOW) {
      delay(50);
    }
  }

  if (digitalRead(BWBUTTON) == LOW && digitalRead(ROTARY_BUTTON) == HIGH && digitalRead(MODEBUTTON) == HIGH) {
    rotarymode = EEPROM.readByte(163);
    EEPROM.commit();
    if (rotarymode == 0) {
      rotarymode = 1;
    } else {
      rotarymode = 0;
    }
    EEPROM.writeByte(163, rotarymode);
    EEPROM.commit();
    Display.writeStr("version.txt", "Rotary direc-");
    Display.writeStr("version2.txt", "tion changed");
    delay(2000);
    Display.writeStr("version.txt", "Please release");
    Display.writeStr("version2.txt", "the button!");
    while (digitalRead(BWBUTTON) == LOW) {
      delay(50);
    }
  }

  if (digitalRead(BWBUTTON) == LOW && digitalRead(ROTARY_BUTTON) == LOW && digitalRead(MODEBUTTON) == HIGH) {
    FactoryDefaults();
    Display.writeStr("version.txt", "Tuner back to");
    Display.writeStr("version2.txt", "default values");
    delay(2000);
    Display.writeStr("version.txt", "Please release");
    Display.writeStr("version2.txt", "the buttons!");
    while (digitalRead(BWBUTTON) == LOW && digitalRead(ROTARY_BUTTON) == LOW) {
      delay(50);
    }
  }
  

  String displayversionext;
  displayversion = Display.readNumber("versie");
  if (displayversion >= 7777) {
    displayversion = Display.readNumber("versie");
  }
  if (displayversion % 10 < 10 && displayversion % 10 > 0) {
    displayversionext = ".0" + String (displayversion % 10);
  } else if (displayversion % 10 == 0) {
    displayversionext = ".00";
  } else {
    displayversionext = "." + String (displayversion % 10);
  }
  String showdisplayversion = String ("Display v" + String (displayversion / 100) + String(displayversionext));
  Display.writeStr("page 0");
  delay(100);
  Display.writeStr("version.txt", "Version " + String(VERSION));
  Display.writeStr("version2.txt", showdisplayversion);

  if (displayversion != DISPLAYVER) {
    Display.writeStr("page 0");
    delay(100);
    Display.writeStr("version.txt", "Incompatible display version");
    Display.writeStr("version2.txt", "Update display to v" + String (DISPLAYVER / 100) + ".0" + String (DISPLAYVER % 10) + "!");
    for(;;);
  }

  TEF = EEPROM.readByte(183);
  
  if (TEF != 101 && TEF != 102 && TEF != 205) {
    SetTunerPatch();
  }

  Frontend.Init(5, 25000000);
  radio.init(TEF);
  uint16_t device;
  uint16_t hw;
  uint16_t sw;
  radio.getIdentification(device, hw, sw);
  if (TEF != (highByte(hw) * 100 + highByte(sw))) {
    SetTunerPatch();  
  }
  
  radio.power(0);
  radio.setMute();
  radio.getStatus(SStatus, USN, WAM, OStatus, BW, MStatus);

  if (lowByte(device) != 0) {
    Display.writeNum("TEFicon.pic", 4);
    Display.writeStr("vis TEFicon,1");
    Display.writeNum("TEF.pco", 34800);
    if (lowByte(device) == 14) {
      Display.writeStr("TEF.txt", "TEF6686 detected!");
    } else if (lowByte(device) == 1) {
      Display.writeStr("TEF.txt", "TEF6687 detected!");
      Display.writeNum("fmsienable", 1);
    } else if (lowByte(device) == 9) {
      Display.writeStr("TEF.txt", "TEF6688 detected!");
    } else if (lowByte(device) == 3) {
      Display.writeStr("TEF.txt", "TEF6689 detected!");
      Display.writeNum("fmsienable", 1);
    }
  } else {
    Display.writeNum("TEFicon.pic", 5);
    Display.writeStr("vis TEFicon,1");
    Display.writeNum("TEF.pco", 63488);
    Display.writeStr("TEF.txt", "Tuner NOT detected!");
  }
  if (digitalRead(BAND) == LOW) {
    Display.writeNum("UHFicon.pic", 4);
    Display.writeStr("vis UHFicon,1");
    Display.writeNum("UHF.pco", 34800);
    Display.writeStr("UHF.txt", "UHF Board detected!");
    Display.writeNum("uhf", 1);
    UHF = true;
  } else {
    Display.writeNum("UHFicon.pic", 5);
    Display.writeStr("vis UHFicon,1");
    Display.writeNum("UHF.pco", 63488);
    Display.writeStr("UHF.txt", "UHF Board NOT detected!");
    UHF = false;
  }
  swSer.print("AT+STATUS");
  delay(100);
  if (swSer.available()) {
    Display.writeNum("BTicon.pic", 4);
    Display.writeStr("vis BTicon,1");
    Display.writeNum("BT.pco", 34800);
    Display.writeStr("BT.txt", "Bluetooth detected!");
    Display.writeNum("btenable", 1);
  } else {
    Display.writeNum("BTicon.pic", 5);
    Display.writeStr("vis BTicon,1");
    Display.writeNum("BT.pco", 63488);
    Display.writeStr("BT.txt", "Bluetooth NOT detected!");    
  }
  Frontend.SetFreq(100000);
  delay(400);
  EEpromReadData();

  if (usbmode == 1) {
    Serial.begin(19200);
  } else {
    Serial.begin(115200);
  }

  Display.writeNum("smeterold", 0);

  if (iMSset == 1 && EQset == 1) {
    iMSEQ = 2;
  }
  if (iMSset == 0 && EQset == 1) {
    iMSEQ = 3;
  }
  if (iMSset == 1 && EQset == 0) {
    iMSEQ = 4;
  }
  if (iMSset == 0 && EQset == 0) {
    iMSEQ = 1;
  }

  if (wifienable == 2) {
    if (wc.autoConnect()) {
      Server.begin();
      if (stationlist == 1) {
        Udp.begin(9031);
      }
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      WiFi.mode(WIFI_OFF);
      wifienable = 1;
      EEPROM.writeByte(49, 1);
      EEPROM.commit();
    }
  } else {
    WiFi.mode(WIFI_OFF);
  }

  radio.setVolume(VolSet);
  radio.setMute();
  radio.setStereoLevel(StereoLevel);
  radio.setHighCutLevel(HighCutLevel);
  radio.setHighCutOffset(HighCutOffset);
  radio.setStHiBlendLevel(BlendLevel);
  radio.setStHiBlendOffset(BlendOffset);
  radio.setNoiseBlanker(NBLevel);
  radio.setDeemphasis(demp);
  radio.setAudio(lf);
  radio.setNoiseBlanker(NBLevel);
  radio.setAMCoChannel(AM_Cochannel);
  radio.setAMNoiseBlanker(AM_NBLevel);
  radio.setAMAttenuation(AM_att);
  radio.setFMSI(fmsi);
  radio.setFMSI_Time(fmsi_attack, fmsi_release);
  radio.setFMSI_Gain(fmsi_11, fmsi_21, fmsi_31, fmsi_41);
  radio.setFMSI_Bias(fmsi_12, fmsi_22, fmsi_32, fmsi_42);

  Display.writeStr("page 1");
  
  if (band == 0) {
    RF(0);
  }
  if (band == 1) {
    RF(1);
  }
  if (band == 2) {
    RF(2);
  }
  if (band == 3) {
    RF(3);
  }
  if (band == 4) {
    RF(4);
  }
  if (band == 5) {
    RF(5);
  }
  ShowFreq();
  ShowStepSize();
  ShowiMS();
  ShowEQ();
  ShowTuneMode();
  
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON), encoderButtonISR, FALLING);

}

void loop() {
  Display.NextionListen();
  Communication();
  
  if (seek == true) {
    Seek(direction);
  }

  if (spec == true) {
    scan_loop();
  }

  if (menu == true) {
    radio.getProcessing(highcut, stereo, sthiblend, stband_1, stband_2, stband_3, stband_4);
    Display.writeNum("highcut", highcut);
    delay(10);
    Display.writeNum("stereol", stereo);
    delay(10);
    Display.writeNum("sthiblend", sthiblend);
    delay(10);
    Display.writeNum("stband_1", stband_1);
    delay(10);
    Display.writeNum("stband_2", stband_2);
    delay(10);
    Display.writeNum("stband_3", stband_3);
    delay(10);
    Display.writeNum("stband_4", stband_4);
  }
  
  if (menu == false && manfreq == false && spec == false) {
    if ((SStatus < 100) || (OStatus < -200 || OStatus > 200) || (USN > 200) && (WAM > 230)) {
      if (millis() >= showmillis + 250) {
        if (band == 5) {
          radio.getStatusAM(SStatus, USN, WAM, OStatus, BW, MStatus);
        } else {
          radio.getStatus(SStatus, USN, WAM, OStatus, BW, MStatus);
        }
        readRds();
        ShowStereoStatus();
        showmillis += 250;
      }
    } else {
        if (band == 5) {
          radio.getStatusAM(SStatus, USN, WAM, OStatus, BW, MStatus);
        } else {
          radio.getStatus(SStatus, USN, WAM, OStatus, BW, MStatus);
        }
        readRds();
        ShowStereoStatus();
    }

    if (XDRGTKTCP == true || XDRGTK == true) {
      if (millis() >= XDRshowmillis + 200) {
        ShowBW();
        ShowSignalLevel();
        ShowOffset();
        ShowUSBstatus();
        ShowRSSI();
        doSquelch();
        XDRshowmillis += 200;
      }
    } else {
      ShowBW();
      ShowSignalLevel();
      ShowOffset();
      ShowUSBstatus();
      ShowRSSI();
      doSquelch();
    }

    doEEpromWrite();
      displayreset = false;

    if (position < encoder.getPosition()) {
      if (rotarymode == 0) {
        KeyUp();
      } else {
        KeyDown();
      }
    }

    if (position > encoder.getPosition()) {
      if (rotarymode == 0) {
        KeyDown();
      } else {
        KeyUp();
      }
    }

    if (encoder.getPushButton() == true) {
      ButtonPress();
    }

    if (digitalRead(MODEBUTTON) == LOW) {
      ModeButtonPress();
    }

    if (digitalRead(BWBUTTON) == LOW) {
      BWButtonPress();
    }

    if (digitalRead(BAND) == LOW) {
      BandSet();
    }
  }
}

void BandSet() {
  if (XDRGTK == false && XDRGTKTCP == false) {
    unsigned long counterold = millis();
    unsigned long counter = millis();
    while (digitalRead(BAND) == LOW && counter - counterold <= 1000) {
      counter = millis();
    }
    if (counter - counterold < 1000 || power == true) {
      if (power == true) {
        Display.writeNum("sleep", 0);
        Display.writeStr("page 0");
        ESP.restart();
      }
      seek = false;
      tunemode = 0;
      ShowTuneMode();
      if (UHF == true) {
        band ++;
        if (band > 5) {
          band = 0;
        }
        if (band == 0) {
          RF(0);
        }
        if (band == 1) {
          RF(1);
        }
        if (band == 2) {
          RF(2);
        }
        if (band == 3) {
          RF(3);
        }
        if (band == 4) {
          RF(4);
        }
        if (band == 5) {
          RF(5);
        }
      } else {
        if (band == 0)
        {
          band = 5;
          RF(5);
        } else {
          band = 0;
          RF(0);
        }
      }
      EEPROM.writeByte(46, band);
      EEPROM.commit();
      ShowFreq();
      radio.clearRDS();
    } else {
      power = true;
      Display.writeNum("sleep", 1);
      Frontend.Power(0);
      radio.power(1);
      if (wifienable == 1) {
        digitalWrite(LED_BUILTIN, LOW);
        Server.end();
        Udp.stop();
        WiFi.mode(WIFI_OFF);
      }
    }
    while (digitalRead(BAND) == LOW) {
      delay(50);
    }
  }
}

void RF(byte RFset) {
  Display.writeNum("outlimit.en",0);
  Display.writeStr("vis outoflimit,0");
  if (displayversion > 100) {
    if (RFset == 0 && fm == 0) {
        band ++;
        RFset ++;
      }
      if (RFset == 1 && uhf1 == 0) {
        band ++;
        RFset ++;
      }
      if (RFset == 2 && uhf2 == 0) {
        band ++;
        RFset ++;
      }
      if (RFset == 3 && uhf3 == 0) {
        band ++;
        RFset ++;
      }
      if (RFset == 4 && uhf4 == 0) {
        band ++;
        RFset ++;
      }
      if (RFset == 5 && am == 0) {
        band = 0;
        RFset = 0;
        RF(0);
      }
    }
  if (RFset == 5) {
  if (showrdsinfo == true) {
      Display.writeStr("page 1");
      showrdsinfo = false;
    }
    Display.writeStr("vis m0,0");
    if (scopeview == 0) {
      Display.writeStr("vis U_disp,0");
      Display.writeStr("vis W_disp,0");
      Display.writeNum("tm4.en", 0);
      Display.writeStr("vis m1,0");
      Display.writeStr("vis signalform,1");
      Display.writeStr("vis modpos,1");
      Display.writeNum("scopeview", 1);
    }
    Display.writeNum("BW.pco", 65504);
    Display.writeNum("autobwlogo.pic", 22);
    Display.writeNum("imslogo.pic", 28);
    Display.writeNum("eqlogo.pic", 26);
    Display.writeNum("stereo.pic", 10);
    Display.writeStr("BER.txt", "");
    Display.writeNum("t4.pco", 33840);
    Display.writeNum("t5.pco", 33840);
    Display.writeNum("t6.pco", 33840);
    Display.writeNum("t7.pco", 33840);
    Display.writeNum("t10.pco", 33840);
    String count = String(frequency5, DEC);
    if (count.length() > 3) {
      Display.writeNum("freq.vvs1", 3);
      Display.writeStr("t2.txt", "MHz");
    } else {
      Display.writeNum("freq.vvs1", 0);
      Display.writeStr("t2.txt", "kHz");
    }
  } else {
    ShowiMS();
    ShowEQ();
    Display.writeStr("vis m0,1");
    if (scopeview == 0) {
      Display.writeStr("vis U_disp,1");
      Display.writeStr("vis W_disp,1");
      Display.writeNum("tm4.en", 1);
      Display.writeStr("vis m1,1");
      Display.writeStr("vis signalform,0");
      Display.writeStr("vis modpos,0");
      Display.writeNum("scopeview", 0);
    }
    Display.writeNum("freq.vvs1", 2);
    Display.writeStr("t2.txt", "MHz");
    Display.writeNum("t4.pco", 65535);
    Display.writeNum("t5.pco", 65535);
    Display.writeNum("t6.pco", 65535);
    Display.writeNum("t7.pco", 65535);
    Display.writeNum("t10.pco", 65535);
    ShowBW();
  }
  if (RFset == 0) {
  if (CoaxSwitch == 1) {
      digitalWrite(RFC, LOW);
    } else {
      digitalWrite(RFC, HIGH);
    }
    digitalWrite(RFA, HIGH);
    digitalWrite(RFB, HIGH);
    radio.power(0);
    delay(50);
    radio.SetFreq(frequency0);
    radio.setOffset(LevelOffset0);
    Frontend.Power(0);
  } else if (RFset == 1) {
  if (CoaxSwitch == 1) {
      digitalWrite(RFC, HIGH);
    } else {
      digitalWrite(RFC, LOW);
    }
    digitalWrite(RFA, LOW);
    digitalWrite(RFB, LOW);
    radio.power(0);
    delay(50);
    radio.SetFreq(IF * 100);
    radio.clearRDS();
    radio.setOffset(LevelOffset1 - 7);
    Frontend.Power(1);
    Frontend.SetFreq(frequency1 - (IF * 100));
  } else if (RFset == 2) {
  if (CoaxSwitch == 1) {
      digitalWrite(RFC, HIGH);
    } else {
      digitalWrite(RFC, LOW);
    }
    digitalWrite(RFA, HIGH);
    digitalWrite(RFB, LOW);
    radio.power(0);
    delay(50);
    radio.SetFreq(IF * 100);
    radio.clearRDS();
    radio.setOffset(LevelOffset2 - 4);
    Frontend.SetFreq(frequency2 - (IF * 100));
  } else if (RFset == 3) {
  if (CoaxSwitch == 1) {
      digitalWrite(RFC, HIGH);
    } else {
      digitalWrite(RFC, LOW);
    }
    digitalWrite(RFA, LOW);
    digitalWrite(RFB, HIGH);
    radio.power(0);
    delay(50);
    radio.SetFreq(IF * 100);
    radio.clearRDS();
    radio.setOffset(LevelOffset3 - 0);
    Frontend.SetFreq(frequency3 - (IF * 100));
  } else if (RFset == 4) {
  if (CoaxSwitch == 1) {
      digitalWrite(RFC, HIGH);
    } else {
      digitalWrite(RFC, LOW);
    }
    digitalWrite(RFA, HIGH);
    digitalWrite(RFB, HIGH);
    radio.power(0);
    delay(50);
    radio.SetFreq(IF * 100);
    radio.clearRDS();
    radio.setOffset(LevelOffset4 + 12);
    Frontend.SetFreq(frequency4 - (IF * 100));
  } else if (RFset == 5) {
  if (CoaxSwitch == 1) {
      if (coaxmode == 0) {
        digitalWrite(RFC, LOW);
      } else {
        digitalWrite(RFC, HIGH);
      }
    } else {
      if (coaxmode == 0) {
        digitalWrite(RFC, HIGH);
      } else {
        digitalWrite(RFC, LOW);
      }
    }
    digitalWrite(RFA, HIGH);
    digitalWrite(RFB, HIGH);
    radio.clearRDS();
    radio.SetFreqAM(frequency5);
    Frontend.Power(0);
  }
  OStatusold = 0;
  BWreset = true;
}

void readRds() {
  if (band == 5) {
    RDSstatus = 0;
  } else {
    RDSstatus = radio.readRDS();
  }
  ShowRDS(RDSstatus);

  if (RDSstatus == true && wificonnect == true && RDSSpy == true && band != 5) {
    if (RDSSpyReset == true) {
      RemoteClient.print("G:\r\n");
      RemoteClient.print("RESET-------");
      RemoteClient.print("\r\n\r\n");
      RDSSpyReset = false;
    }
    RemoteClient.print("G:\r\n");
    tcp_hex(radio.rds.rdsA >> 8);
    tcp_hex(radio.rds.rdsA);
    tcp_hex(radio.rds.rdsB >> 8);
    tcp_hex(radio.rds.rdsB);
    tcp_hex(radio.rds.rdsC >> 8);
    tcp_hex(radio.rds.rdsC);
    tcp_hex(radio.rds.rdsD >> 8);
    tcp_hex(radio.rds.rdsD);
    RemoteClient.print("\r\n\r\n");
  }

  if (RDSstatus == true && USBstatus == true && RDSSpy == true && band != 5) {
    if (RDSSpyReset == true) {
      Serial.print("G:\r\n");
      Serial.print("RESET-------");
      Serial.print("\r\n\r\n");
      RDSSpyReset = false;
    }
    Serial.print("G:\r\n");
    serial_hex(radio.rds.rdsA >> 8);
    serial_hex(radio.rds.rdsA);
    serial_hex(radio.rds.rdsB >> 8);
    serial_hex(radio.rds.rdsB);
    serial_hex(radio.rds.rdsC >> 8);
    serial_hex(radio.rds.rdsC);
    serial_hex(radio.rds.rdsD >> 8);
    serial_hex(radio.rds.rdsD);
    Serial.print("\r\n\r\n");
  }

  if (XDRGTK == true && RDSstatus == true && band != 5) {
    Serial.print("P");
    serial_hex(radio.rds.rdsA >> 8);
    serial_hex(radio.rds.rdsA);
    Serial.print("\nR");
    serial_hex(radio.rds.rdsB >> 8);
    serial_hex(radio.rds.rdsB);
    serial_hex(radio.rds.rdsC >> 8);
    serial_hex(radio.rds.rdsC);
    serial_hex(radio.rds.rdsD >> 8);
    serial_hex(radio.rds.rdsD);
    serial_hex(radio.rds.errors >> 8);
    Serial.print("\n");
  }

  if (XDRGTKTCP == true && RDSstatus == true && band != 5) {
    RemoteClient.print("P");
    tcp_hex(radio.rds.rdsA >> 8);
    tcp_hex(radio.rds.rdsA);
    RemoteClient.print("\nR");
    tcp_hex(radio.rds.rdsB >> 8);
    tcp_hex(radio.rds.rdsB);
    tcp_hex(radio.rds.rdsC >> 8);
    tcp_hex(radio.rds.rdsC);
    tcp_hex(radio.rds.rdsD >> 8);
    tcp_hex(radio.rds.rdsD);
    tcp_hex(radio.rds.errors >> 8);
    RemoteClient.print("\n");
  }
}

void ShowRDS(bool RDSstatus) {

  if (menu == false && spec == false && manfreq == false)
  {
    if (RDSstatus == true) {
      if (radio.rds.MS == false) {
        if (slogo_on == false) {
          Display.writeNum("s.pic", 41);
          slogo_on = true;
          mlogo_on = false;
          delay(10);
          Display.writeNum("m.pic", 38);
        }
      } else {
        if (mlogo_on == false) {
          Display.writeNum("s.pic", 40);
          delay(10);
          Display.writeNum("m.pic", 39);
          mlogo_on = true;
          slogo_on = false;
        }
      }
    }
    
    if (RDSstatusold != RDSstatus || displayreset == true)
    {
      if (band == 5) {
        Display.writeNum("rdslogo.pic", 19);
        delay(10);
        Display.writeNum("m.pic", 38);
        delay(10);
        Display.writeNum("s.pic", 40);
        delay(10);
        Display.writeNum("PTY.pco", 65535);
        delay(10);
        Display.writeNum("RT.pco", 65535);
        delay(10);
        Display.writeNum("PS.pco", 65535);
        delay(10);
        Display.writeNum("PI.pco", 65535);
        delay(10);
        Display.writeNum("rtshow", 0);
        delay(10);
        Display.writeStr("vis BER,0");
        delay(10);
        Display.writeNum("a.pic", 118);
        delay(10);
        Display.writeNum("b.pic", 120);
        RDSstatusold = 0;
      } else {
        if (RDSstatus == true && USN < 250 && WAM < 250 && SQ == false && OStatus > -250 && OStatus < 250) {
          Display.writeNum("rdslogo.pic", 18);
          delay(10);
          Display.writeNum("PTY.pco", 65504);
          delay(10);
          Display.writeNum("RT.pco", 65504);
          delay(10);
          Display.writeNum("PS.pco", 65504);
          delay(10);
          Display.writeNum("PI.pco", 65504);
          delay(10);
          Display.writeNum("rtshow", 0);
          Display.writeStr("vis BER,1");
          RDSstatusold = 1;
        } else {
          Display.writeNum("rdslogo.pic", 19);
          delay(10);
          Display.writeNum("m.pic", 38);
          delay(10);
          Display.writeNum("s.pic", 40);
          delay(10);
          Display.writeNum("PTY.pco", 65535);
          delay(10);
          Display.writeNum("RT.pco", 65535);
          delay(10);
          Display.writeNum("PS.pco", 65535);
          delay(10);
          Display.writeNum("PI.pco", 65535);
          delay(10);
          Display.writeNum("rtshow", 0);
          delay(10);
          Display.writeStr("vis BER,0");
          delay(10);
          Display.writeNum("a.pic", 118);
          delay(10);
          Display.writeNum("b.pic", 120);
          RDSstatusold = 0;
        }
      }
    }

    if (radio.rds.afclear == false) {
      af2show = false;
      af = "";
      af2 = "";
      radio.rds.afclear = true;
    }
    if (radio.af_counter != af_counterold && radio.rds.hasAF == true) {
      if (stationlist == 1 && wifienable == 2) {
        Udp.beginPacket(remoteip, 9030);
        Udp.print("AF=");
      }
      for (af_scan = 0; af_scan < radio.af_counter; af_scan++)
      {
        if (stationlist == 1 && wifienable == 2) {
          if ((radio.af[af_scan].frequency - 8750) / 10 < 0x10) {
            Udp.print("0");
          }
          Udp.print((radio.af[af_scan].frequency - 8750) / 10, HEX);
        }
        String extrazero;
        if (radio.af[af_scan].frequency % 100 < 10) {
          extrazero = "0";
        }
        if (radio.af[af_scan].frequency != frequency0) {
          if (displaysize == 35) {
            if (af_scan < 13) {
              af += String(radio.af[af_scan].frequency / 100) + "." + extrazero + String(radio.af[af_scan].frequency % 100) + "\\r";
              af2show = false;
            } else {
              af2 += String(radio.af[af_scan].frequency / 100) + "." + extrazero + String(radio.af[af_scan].frequency % 100) + "\\r";
              af2show = true;
            }
          } else {
            if (af_scan < 10) {
              af += String(radio.af[af_scan].frequency / 100) + "." + extrazero + String(radio.af[af_scan].frequency % 100) + "\\r";
              af2show = false;
            } else {
              af2 += String(radio.af[af_scan].frequency / 100) + "." + extrazero + String(radio.af[af_scan].frequency % 100) + "\\r";
              af2show = true;
            }
          }
        }
      }
      af_counterold = radio.af_counter;
      if (showrdsinfo == true) {
        Display.writeStr("AFLIST.txt", af);
        if (af2show == true) {
          Display.writeNum("tm0.en", 1);
          Display.writeStr("AFLIST2.txt", af2);
        } else {
          Display.writeNum("tm0.en", 0);
          Display.writeStr("vis AFLIST2,0");
        }
      }
      if (stationlist == 1 && wifienable == 2) {
        for (int i = 0; i < 25 - radio.af_counter; i++) {
          Udp.print("00");
        }
        Udp.endPacket();
      }
    } else {
      af = "";
    }

    if (TAold != radio.rds.hasTA || displayreset == true) {
      if (RDSstatus == true && radio.rds.hasTA == true) {
        Display.writeNum("ta.pic", 43);
      } else {
        Display.writeNum("ta.pic", 42);
      }
      TAold = radio.rds.hasTA;
    }

    if (TPold != radio.rds.hasTP || displayreset == true) {
      if (RDSstatus == true && radio.rds.hasTP == true) {
        Display.writeNum("tp.pic", 45);
      } else {
        Display.writeNum("tp.pic", 44);
      }
      TPold = radio.rds.hasTP;
    }

    if (EONold != radio.rds.hasEON || displayreset == true) {
      if (RDSstatus == true && radio.rds.hasEON == true) {
        Display.writeNum("eon.pic", 53);
      } else {
        Display.writeNum("eon.pic", 52);
      }
      EONold = radio.rds.hasEON;
    }

    if (RTPlus != radio.rds.hasRDSplus && showrdsinfo == true) {
      if (RDSstatus == true && radio.rds.hasRDSplus == true) {
        Display.writeNum("rtplus.pic", 4);
      } else {
        Display.writeNum("rtplus.pic", 5);
      }
      RTPlus = radio.rds.hasRDSplus;
    }

    if (ECCold != radio.rds.ECC || displayreset == true) {
      if (stationlist == 1 && wifienable == 2) {
        Udp.beginPacket(remoteip, 9030);
        Udp.print("ECC=");
        if (radio.rds.ECC < 0x10) {
          Udp.print("0");
        }
        Udp.print(radio.rds.ECC, HEX);
        Udp.endPacket();
      }
      if (radio.rds.picode[0] == '1') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 55);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 56);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 57);
        }
        if (radio.rds.ECC == 228) {
          Display.writeNum("ECC.pic", 58);
        }
      } else if (radio.rds.picode[0] == '2') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 59);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 60);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 61);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 62);
        }
        if (radio.rds.ECC == 228) {
          Display.writeNum("ECC.pic", 63);
        }
      } else if (radio.rds.picode[0] == '3') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 64);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 65);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 66);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 67);
        }
      } else if (radio.rds.picode[0] == '4') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 68);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 69);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 70);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 71);
        }
      } else if (radio.rds.picode[0] == '5') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 72);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 73);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 74);
        }
      } else if (radio.rds.picode[0] == '6') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 75);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 76);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 77);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 78);
        }
        if (radio.rds.ECC == 228) {
          Display.writeNum("ECC.pic", 79);
        }
      } else if (radio.rds.picode[0] == '7') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 80);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 81);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 82);
        }
      } else if (radio.rds.picode[0] == '8') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 83);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 84);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 85);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 86);
        }
        if (radio.rds.ECC == 228) {
          Display.writeNum("ECC.pic", 87);
        }
      } else if (radio.rds.picode[0] == '9') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 88);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 89);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 90);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 91);
        }
        if (radio.rds.ECC == 228) {
          Display.writeNum("ECC.pic", 92);
        }
      } else if (radio.rds.picode[0] == 'A') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 93);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 94);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 95);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 96);
        }
      } else if (radio.rds.picode[0] == 'B') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 97);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 98);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 99);
        }
      } else if (radio.rds.picode[0] == 'C') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 100);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 101);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 102);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 103);
        }
      } else if (radio.rds.picode[0] == 'D') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 104);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 105);
        }
      } else if (radio.rds.picode[0] == 'E') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 106);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 107);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 108);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 109);
        }
      } else if (radio.rds.picode[0] == 'F') {
        if (radio.rds.ECC == 224) {
          Display.writeNum("ECC.pic", 110);
        }
        if (radio.rds.ECC == 225) {
          Display.writeNum("ECC.pic", 111);
        }
        if (radio.rds.ECC == 226) {
          Display.writeNum("ECC.pic", 112);
        }
        if (radio.rds.ECC == 227) {
          Display.writeNum("ECC.pic", 113);
        }
        if (radio.rds.ECC == 228) {
          Display.writeNum("ECC.pic", 114);
        }
      } else {
        Display.writeNum("ECC.pic", 115);
      }
      ECCold = radio.rds.ECC;
    }

    if ((strcmp(radio.rds.stationEvent, stationEventPrevious) || strcmp(radio.rds.stationHost, stationHostPrevious)) && showrdsinfo == true) {
      String rtplusinfo = String(radio.rds.stationEvent) + "   " + String(radio.rds.stationHost) + "   ";
      Display.writeStr("rtplusinfo.txt", rtplusinfo);
      strcpy(stationEventPrevious, radio.rds.stationEvent);
      strcpy(stationHostPrevious, radio.rds.stationHost);
    }

    if ((strcmp(radio.rds.musicTitle, musicTitlePrevious)) && showrdsinfo == true) {
      Display.writeStr("title.txt", radio.rds.musicTitle);
      strcpy(musicTitlePrevious, radio.rds.musicTitle);
    }

    if ((strcmp(radio.rds.musicArtist, musicArtistPrevious)) && showrdsinfo == true) {
      Display.writeStr("artist.txt", radio.rds.musicArtist);
      strcpy(musicArtistPrevious, radio.rds.musicArtist);
    }

    if (strcmp(radio.rds.stationName, programServicePrevious) || displayreset == true) {
      Display.writeStr("PS.txt", radio.rds.stationName);
      strcpy(programServicePrevious, radio.rds.stationName);

      if (stationlist == 1 && wifienable == 2) {
        Udp.beginPacket(remoteip, 9030);
        Udp.print("from=TEF tuner " + String(VERSION) + ";PS=");
        char PShex[9];
        strcpy (PShex, radio.rds.stationName);
        for (int i = 0; i < 8; i++)
        {
          if (PShex[i] < 0x10) {
            Udp.print("0");
          }
          if (PShex[i] == 0x20) {
            PShex[i] =  '_';
          }
          Udp.print(PShex[i], HEX);
        }
        Udp.endPacket();
      }
    }

    if (strcmp(radio.rds.stationText, radioTextPrevious) || displayreset == true) {
      Display.writeStr("RT.txt", radio.rds.stationText);
      strcpy(radioTextPrevious, radio.rds.stationText);
      if (radio.rds.hasRT == true) {
        if (radio.rds.rtAB == 0) {
          Display.writeNum("a.pic", 119);
          delay(10);
          Display.writeNum("b.pic", 120);
        } else {
          Display.writeNum("a.pic", 118);
          delay(10);
          Display.writeNum("b.pic", 121);          
        }
      } else {
        Display.writeNum("a.pic", 118);
        delay(10);
        Display.writeNum("b.pic", 120);
      }
      if (stationlist == 1 && wifienable == 2) {
        Udp.beginPacket(remoteip, 9030);
        Udp.print("from=TEF tuner " + String(VERSION) + ";RT1=");
        char RThex[65];
        strcpy (RThex, radio.rds.stationText);
        for (int i = 0; i < 64; i++)
        {
          if (RThex[i] < 0x10) {
            Udp.print("0");
          }
          if (RThex[i] == ' ') {
            RThex[i] =  '_';
          }
          Udp.print(RThex[i], HEX);
        }
        Udp.endPacket();
      }
    }

    if (strcmp(radio.rds.picode, radioIdPrevious) || displayreset == true) {
      Display.writeStr("PI.txt", radio.rds.picode);
      if (stationlist == 1 && wifienable == 2) {
        Udp.beginPacket(remoteip, 9030);
        Udp.print("from=TEF tuner " + String(VERSION) + ";PI=");
        Udp.print(radio.rds.picode);
        Udp.endPacket();
      }
      strcpy(radioIdPrevious, radio.rds.picode);
    }

    if (strcmp(radio.rds.stationType, programTypePrevious) || displayreset == true) {
      Display.writeStr("PTY.txt", radio.rds.stationType);
      strcpy(programTypePrevious, radio.rds.stationType);
      if (stationlist == 1 && wifienable == 2) {
        Udp.beginPacket(remoteip, 9030);
        Udp.print("from=TEF tuner " + String(VERSION) + ";PTY=");
        Udp.print(radio.rds.stationTypeCode, HEX);
        Udp.endPacket();
      }
    }

    if ((radio.rds.errors != errorsold) || displayreset == true) {
      if (RDSstatus == true) {
        Display.writeStr("BER.txt", String(100 - radio.rds.errors) + "%");
      } else {
        Display.writeStr("BER.txt", "0%");
      }
      errorsold = radio.rds.errors;
    }

    if (radio.rds.hasCT == true && minutesold != radio.rds.minutes) {
      if (radio.rds.offsetplusmin == true) {
        timeoffset = (-1 * radio.rds.offset) / 2;
      } else {
        timeoffset = radio.rds.offset / 2;
      }
      setTime((radio.rds.hours + timeoffset), radio.rds.minutes, 0, radio.rds.days, radio.rds.months, radio.rds.years);

      String day0;
      String month0;
      String hour0;
      String minute0;
      if (hour() < 10) {
        hour0 = "0";
      }
      if (minute() < 10) {
        minute0 = "0";
      }
      if (day() < 10) {
        day0 = "0";
      }
      if (month() < 10) {
        month0 = "0";
      }
      rds_clock = (hour0 + String(hour()) + ":" + minute0 + String(minute())) + "  " + (day0 + String(day()) + "-" + month0 + String(month()) + "-" + String(year()));

      Display.writeNum("tm3.en", 1);
      Display.writeStr("CT.txt", rds_clock);
      ctshow = true;
      minutesold = radio.rds.minutes;
    }

    if (ctshow == true && radio.rds.hasCT == false) {
      Display.writeNum("tm3.en", 0);
      Display.writeStr("vis 52,0");
      Display.writeStr("t7.txt", "PTY:");
      ctshow = false;
    }
  }
}

void ShowRSSI()
{
  if (wifienable == 2) {
    rssi = WiFi.RSSI();
  } else {
    rssi = 0;
  }

  if (rssiold != rssi) {
    rssiold = rssi;
    if (rssi == 0) {
      Display.writeNum("wifilogo.pic", 46);
    } else if (rssi > -50 && rssi < 0) {
      Display.writeNum("wifilogo.pic", 50);
    } else if (rssi > -60) {
      Display.writeNum("wifilogo.pic", 49);
    } else if (rssi > -70) {
      Display.writeNum("wifilogo.pic", 48);
    } else if (rssi < -70) {
      Display.writeNum("wifilogo.pic", 47);
    }
  }
}

void ShowFreq()
{
  detachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A));
  detachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B));
  if (band == 0) {
    freq = frequency0;
  }
  if (band == 1) {
    freq = frequency1;
  }
  if (band == 2) {
    freq = frequency2;
  }
  if (band == 3) {
    freq = frequency3;
  }
  if (band == 4) {
    freq = frequency4;
  }
  if (band == 5) {
    freq = frequency5;
  }

  if(stationlist == 1 && wifienable == 2) {
    Udp.beginPacket(remoteip, 9030);
    Udp.print("from=TEF tuner " + String(VERSION) + ";freq=");
    if (band == 5) {
      Udp.print(frequency5);
      Udp.print("000;ClearRDS=1");
    } else {
      Udp.print(frequency0);
      Udp.print("0000;ClearRDS=1");
    }
    Udp.endPacket();
    stlmillis += 500;
  }
  if (band == 5) {
    String count = String(frequency5, DEC);
    if (count.length() > 3) {
      Display.writeNum("freq.vvs1", 3);
      Display.writeStr("t2.txt", "MHz");
    } else {
      Display.writeNum("freq.vvs1", 0);
      Display.writeStr("t2.txt", "kHz");
    }
  }
  Display.writeNum("freq.val", freq);
  RDSSpyReset = true;

  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A),  encoderISR,       CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B),  encoderISR,       CHANGE);
}

void ShowModLevel()
{
  if (showrdsinfo==false) {

    if (seek == false) {
      if (SQ == false) {
        Display.writeNum("vu.val", MStatus / 2);
      } else {
        Display.writeNum("vu.val", 0);
      }
    } else {
      Display.writeNum("vu.val", 0);
    }
  }
}

void ShowBW()
{
  if ((BW != BWOld || BWreset == true) || displayreset == true)
  {
    if (BWreset == true) {
    if (band != 5) {
      if (BWsetOld != BWset) {
        if (BWset == 0) {
          Display.writeNum("BW.pco", 2047);
          if (showrdsinfo == false) {
            delay(10);
            Display.writeNum("autobwlogo.pic", 23);
          }
          radio.setFMABandw();
        } else {
          Display.writeNum("BW.pco", 65504);
          if (showrdsinfo == false) {
            delay(10);
            Display.writeNum("autobwlogo.pic", 22);
          }
          BWsetOld = BWset;
        }
      }
    }
    }
    delay(10);
    Display.writeNum("BW.val", BW);
    BWOld = BW;
    BWreset = false;
    if (stationlist == 1 && wifienable == 2) {
      Udp.beginPacket(remoteip, 9030);
      Udp.print("from=TEF tuner " + String(VERSION) + ";Bandwidth=");
      Udp.print(BW * 1000);
      Udp.endPacket();
    }
  }
}

void ShowSignalLevel()
{
  if (band != 5) {
    SNR = int(0.46222375 * (float)(SStatus / 10) - 0.082495118 * (float)(USN)) + 10;
  } else {
    SNR = -((int8_t)(USN));
  }
  
  SAvg = (((SAvg * 9) + 5) / 10) + SStatus;
  SStatus = SAvg / 10;

  if (showrdsinfo == false) {
    if (USN < 250 && WAM < 250 && OStatus > -250 && OStatus < 250) {
      if (SNR > (SNRold + 1) || SNR < (SNRold - 1)) {
        Display.writeNum("SNR.val", SNR);
        SNRold = SNR;
      }
    } else {
      Display.writeNum("SNR.val", 0);
      SNRold = 0;
    }
    
    if (displayversion > 100) {
      Display.writeNum("WAM.val", WAM / 5);
      Display.writeNum("USN.val", USN / 5);
    }
    if ((SStatus > (SStatusold + 3) || SStatus < (SStatusold - 3)) || displayreset == true)
    {
      Display.writeNum("signal.val", SStatus / 10);
      if (SStatus < 0) {
        String negative = String (SStatus % 10, DEC);
        Display.writeNum("signaldec.val", negative[1]);
      } else {
        Display.writeNum("signaldec.val", SStatus % 10);
      }
      SStatusold = SStatus;
      if (stationlist == 1 && wifienable == 2) {
        Udp.beginPacket(remoteip, 9030);
        Udp.print("from=TEF tuner " + String(VERSION) + ";RcvLevel=");
        Udp.print(SStatus / 10);
        Udp.endPacket();
      }
    }
  }
}

void ShowStereoStatus()
{
  if (StereoToggle == true && band != 5)
  {
    Stereostatus = radio.getStereoStatus();
    if ((Stereostatus != Stereostatusold) || displayreset == true)
    {
      if (Stereostatus == true)
      {
        Display.writeNum("stereo.pic", 9);
      } else {
        Display.writeNum("stereo.pic", 10);
      }
      Stereostatusold = Stereostatus;
    }
  }
}

void ShowOffset()
{

  if (band == 5) {
    if (OStatus < -2)
    {
      if (OStatusold != 1)
      {
        Display.writeNum("tune_neg.pic", 14);
        delay(10);
        Display.writeNum("tuned.pic", 12);
        delay(10);
        Display.writeNum("tune_pos.pic", 6);
        delay(10);
        MStatus = 0;
        OStatusold = 1;
      }
    } else if (SQ == false && OStatus > -2 && OStatus < 2)
    {
      if (OStatusold != 2)
      {
        Display.writeNum("tune_neg.pic", 14);
        delay(10);
        Display.writeNum("tuned.pic", 7);
        delay(10);
        Display.writeNum("tune_pos.pic", 13);
        delay(10);
        OStatusold = 2;
      }
    } else if (OStatus > 2)
    {
      if (OStatusold != 3)
      {
        Display.writeNum("tune_neg.pic", 8);
        delay(10);
        Display.writeNum("tuned.pic", 12);
        delay(10);
        Display.writeNum("tune_pos.pic", 13);
        delay(10);
        MStatus = 0;
        OStatusold = 3;
      }
    } else {
      if (OStatusold != 4)
      {
        Display.writeNum("tune_neg.pic", 14);
        delay(10);
        Display.writeNum("tuned.pic", 12);
        delay(10);
        Display.writeNum("tune_pos.pic", 13);
        delay(10);
        MStatus = 0;
        OStatusold = 4;
      }
    }
  } else {
    if (OStatus < -250)
    {
      if (OStatusold != 1)
      {
        Display.writeNum("tune_neg.pic", 14);
        delay(10);
        Display.writeNum("tuned.pic", 12);
        delay(10);
        Display.writeNum("tune_pos.pic", 6);
        delay(10);
        MStatus = 0;
        OStatusold = 1;
      }
    } else if (USN < 250 && WAM < 250 && SQ == false && OStatus > -250 && OStatus < 250)
    {
      if (OStatusold != 2)
      {
        Display.writeNum("tune_neg.pic", 14);
        delay(10);
        Display.writeNum("tuned.pic", 7);
        delay(10);
        Display.writeNum("tune_pos.pic", 13);
        delay(10);
        OStatusold = 2;
      }
    } else if (OStatus > 250)
    {
      if (OStatusold != 3)
      {
        Display.writeNum("tune_neg.pic", 8);
        delay(10);
        Display.writeNum("tuned.pic", 12);
        delay(10);
        Display.writeNum("tune_pos.pic", 13);
        MStatus = 0;
        OStatusold = 3;
      }
    } else {
      if (OStatusold != 4)
      {
        Display.writeNum("tune_neg.pic", 14);
        delay(10);
        Display.writeNum("tuned.pic", 12);
        delay(10);
        Display.writeNum("tune_pos.pic", 13);
        delay(10);
        MStatus = 0;
        OStatusold = 4;
      }
    }
  }
  ShowModLevel();
}

void serial_hex(uint8_t val) {
  Serial.print((val >> 4) & 0xF, HEX);
  Serial.print(val & 0xF, HEX);
}

void tcp_hex(uint8_t val) {
  RemoteClient.print((val >> 4) & 0xF, HEX);
  RemoteClient.print(val & 0xF, HEX);
}

bool strcmp(char* str1, char* str2, int length) {
  for (int i = 0; i < length; i++) {
    if (str1[i] != str2[i]) {
      return false;
    }
  }
  return true;
}

bool pascmp(String str1, String str2, int length) {
  for (int i = 0; i < length; i++) {
    if (str1[i] != str2[i]) {
      return false;
    }
  }
  return true;
}

void ShowStepSize() {
  Display.writeNum("step001.pic", 37);
  Display.writeNum("step010.pic", 37);
  Display.writeNum("step100.pic", 37);
  Display.writeNum("step1000.pic", 37);

  if (stepsize == 1) {
    Display.writeNum("step001.pic", 36);
  }

  if (stepsize == 2) {
    Display.writeNum("step010.pic", 36);
  }

  if (stepsize == 3) {
    Display.writeNum("step100.pic", 36);
  }

  if (stepsize == 4) {
    Display.writeNum("step1000.pic", 36);
  }
}

void doStepSize() {
  stepsize++;
  if (stepsize > 4) {
    stepsize = 0;
  }
  ShowStepSize();
  EEPROM.writeByte(42, stepsize);
  EEPROM.commit();
  if (stepsize == 0) {
    RoundStep();
    ShowFreq();
  }
}

void doTuneMode() {
  tunemode++;
  if (tunemode > 2) {
    tunemode = 0;
  }
  ShowTuneMode();
  if (stepsize != 0) {
    stepsize = 0;
    RoundStep();
    ShowStepSize();
    ShowFreq();
  }
}

void ShowTuneMode() {
   ShowStepSize();
   if (tunemode == 0) {
      Display.writeNum("autologo.pic", 20);
      Display.writeNum("manlogo.pic", 31);
      Display.writeNum("memlogo.pic", 24);
   }

   if (tunemode == 1) {
      Display.writeNum("autologo.pic", 21);
      Display.writeNum("manlogo.pic", 30);
      Display.writeNum("memlogo.pic", 24);
   }

   if (tunemode == 2) {
      Display.writeNum("autologo.pic", 20);
      Display.writeNum("manlogo.pic", 30);
      Display.writeNum("memlogo.pic", 25);
   }
}

void ShowBTstatus() {
  if (btconnect == true)
  {
    Display.writeNum("BT.pic", 117);
  } else {
    Display.writeNum("BT.pic", 116);
  }
}

void ShowUSBstatus() {
  if (USBstatus == true)
  {
    if (usblogo_on == false) {
      Display.writeNum("usblogo.pic", 34);
      usblogo_on = true;
      usblogo_off = false;
    }
  } else {
    if (usblogo_off == false) {
      Display.writeNum("usblogo.pic", 35);
      usblogo_on = false;
      usblogo_off = true;
    }
  }
}

void doFilter() {
  if (iMSEQ == 0) {
    iMSEQ = 1;
  }
  if (iMSEQ == 4) {
    iMSset = 0;
    EQset = 0;
    ShowiMS();
    ShowEQ();
    iMSEQ = 0;
  }
  if (iMSEQ == 3) {
    iMSset = 1;
    EQset = 0;
    ShowiMS();
    ShowEQ();
    iMSEQ = 4;
  }
  if (iMSEQ == 2) {
    iMSset = 0;
    EQset = 1;
    ShowiMS();
    ShowEQ();
    iMSEQ = 3;
  }
  if (iMSEQ == 1) {
    iMSset = 1;
    EQset = 1;
    ShowiMS();
    ShowEQ();
    iMSEQ = 2;
  }
  EEPROM.writeByte(44, iMSset);
  EEPROM.writeByte(45, EQset);
  EEPROM.commit();
}


void ButtonPress() {
  if (power == false) {
    if (seek == true) {
      seek = false;
    }
    if (menu == false) {
      unsigned long counterold = millis();
      unsigned long counter = millis();
      while (digitalRead(ROTARY_BUTTON) == LOW && counter - counterold <= 1000) {
        counter = millis();
      }
      if (counter - counterold < 1000) {
        if (tunemode == 0) {
          doStepSize();
        }
        if (tunemode == 2) {
          if (memorystore == false) {
            memorystore = true;
            Display.writeNum("store.en",1);
          } else {
            Display.writeNum("store.en",0);
            Display.writeNum("memlogo.pic",25);
            memorystore = false;
            EEPROM.writeUInt((memory_pos*4) + 230, freq);
            EEPROM.commit();
            memory[memory_pos] = freq;
          }
        }
      } else {
        doFilter();
      }
    }
    while (digitalRead(ROTARY_BUTTON) == LOW) {
      delay(50);
    }
  }
}

void ShowiMS() {
  if (showrdsinfo==false) {
   if (iMSset == 0) {
      Display.writeNum("imslogo.pic", 29);
      radio.setiMS(1);
    } else {
      Display.writeNum("imslogo.pic", 28);
      radio.setiMS(0);
    }
  }
}

void ShowEQ() {
  if (showrdsinfo==false) {
    if (EQset == 0) {
      Display.writeNum("eqlogo.pic", 27);
      radio.setEQ(1);
    } else {
      Display.writeNum("eqlogo.pic", 26);
      radio.setEQ(0);
    }
  }
}

void RoundStep() {
  if (band == 0) {
    unsigned int freq = frequency0;
    if (freq % 10 < 3) {
      frequency0 = (freq - freq % 10);
    }
    else if (freq % 10 > 2 && freq % 10 < 8) {
      frequency0 = (freq - (freq % 10 - 5));
    }
    else if (freq % 10 > 7) {
      frequency0 = (freq - (freq % 10) + 10);
    }
    radio.SetFreq(frequency0);
  }
  if (band == 1) {
    unsigned int freq = frequency1;
    if (freq % 10 < 3) {
      frequency1 = (freq - freq % 10);
    }
    else if (freq % 10 > 2 && freq % 10 < 8) {
      frequency1 = (freq - (freq % 10 - 5));
    }
    else if (freq % 10 > 7) {
      frequency1 = (freq - (freq % 10) + 10);
    }
    Frontend.SetFreq(frequency1 - IF * 100);
  }
  if (band == 2) {
    unsigned int freq = frequency2;
    if (freq % 10 < 3) {
      frequency2 = (freq - freq % 10);
    }
    else if (freq % 10 > 2 && freq % 10 < 8) {
      frequency2 = (freq - (freq % 10 - 5));
    }
    else if (freq % 10 > 7) {
      frequency2 = (freq - (freq % 10) + 10);
    }
    Frontend.SetFreq(frequency2 - IF * 100);
  }
  if (band == 3) {
    unsigned int freq = frequency3;
    if (freq % 10 < 3) {
      frequency3 = (freq - freq % 10);
    }
    else if (freq % 10 > 2 && freq % 10 < 8) {
      frequency3 = (freq - (freq % 10 - 5));
    }
    else if (freq % 10 > 7) {
      frequency3 = (freq - (freq % 10) + 10);
    }
    Frontend.SetFreq(frequency3 - IF * 100);
  }
  if (band == 4) {
    unsigned int freq = frequency4;
    if (freq % 10 < 3) {
      frequency4 = (freq - freq % 10);
    }
    else if (freq % 10 > 2 && freq % 10 < 8) {
      frequency4 = (freq - (freq % 10 - 5));
    }
    else if (freq % 10 > 7) {
      frequency4 = (freq - (freq % 10) + 10);
    }
    Frontend.SetFreq(frequency4 - IF * 100);
  }
  if (band == 5) {
    unsigned int freq = frequency5 / 9;
    frequency5 = freq * 9;
    radio.SetFreqAM(frequency5);
  }
  while (digitalRead(ROTARY_BUTTON) == LOW) {
    delay(50);
  }
  EEPROM.writeUInt(0, frequency0);
  EEPROM.writeUInt(83, frequency1);
  EEPROM.writeUInt(87, frequency2);
  EEPROM.writeUInt(91, frequency3);
  EEPROM.writeUInt(95, frequency4);
  EEPROM.commit();
}

void ModeButtonPress() {
  if (power == false) {
    if (menu == false) {
      unsigned long counterold = millis();
      unsigned long counter = millis();
      while (digitalRead(MODEBUTTON) == LOW && counter - counterold <= 1000) {
        counter = millis();
      }
      if (counter - counterold <= 1000) {
        if (seek == true) {
          seek = false;
        }
        if (band == 5 && tunemode == 0) {
          tunemode = 1;
        }
        doTuneMode();
      } else {
        if (XDRGTKTCP == false && XDRGTK == false && wificonnect == false && manfreq == false && spec == false) {
          if (RDSSpy == true)
          {
            RDSSpy = false;
            USBstatus = false;
          }
          menu = true;
          wifienableold = wifienable;
          Display.writeStr("page 5");
          delay(50);
          Display.writeNum("fmsi", fmsi);
        }
      }
    }
    while (digitalRead(MODEBUTTON) == LOW) {
      delay(50);
    }
  }
}

void BWButtonPress() {
  if (power == false) {
    if (seek == true) {
      seek = false;
    }
    if (menu == false) {
      unsigned long counterold = millis();
      unsigned long counter = millis();
      while (digitalRead(BWBUTTON) == LOW && counter - counterold <= 1000) {
        counter = millis();
      }
      if (counter - counterold < 1000) {
        if (band == 5) {
          BWsetAM++;
        } else {
          BWset++;
        }
        doBW();
      } else {
        if (band != 5) {
          doStereoToggle();
        }
      }
    }
    while (digitalRead(BWBUTTON) == LOW) {
      delay(50);
    }
  }
}

void KeyUp() {
  if (power == false) {
    if (menu == false) {
      if (tunemode == 0) {
        TuneUp();
      }
      if (tunemode == 1) {
        freqtemp = freq;
        direction = true;
        seek = true;
        Seek(direction);
      }
      if (tunemode == 2) {
        memory_pos++;
        if (memory_pos > 29) {
          memory_pos = 0;
        }
        MemoryTune();
      }
      if (XDRGTK == true) {
        if (band == 5) {
          Serial.print("T" + String(frequency5) + "\n");
        } else {
          Serial.print("T" + String(frequency0 * 10) + "\n");
        }
      }

      if (XDRGTKTCP == true) {
        if (band == 5) {
          RemoteClient.print("T" + String(frequency5) + "\n");
        } else {
          RemoteClient.print("T" + String(frequency0 * 10) + "\n");
        }
      }
      change = 0;
      ShowFreq();
      store = true;
    }
    if (optrot != 1) {
      while (digitalRead(ROTARY_PIN_A) == LOW || digitalRead(ROTARY_PIN_B) == LOW) {
        delay(5);
      }
    }
    position = encoder.getPosition();
  }
}

void KeyDown() {
  if (power == false) {
    if (menu == false) {
      if (tunemode == 0) {
        TuneDown();
      }
      if (tunemode == 1) {
        freqtemp = freq;
        direction = false;
        seek = true;
        Seek(direction);
      }
      if (tunemode == 2) {
        memory_pos--;
        if (memory_pos > 29) {
          memory_pos = 29;
        }
        MemoryTune();
      }
      if (XDRGTK == true) {
        if (band == 5) {
          Serial.print("T" + String(frequency5) + "\n");
        } else {
          Serial.print("T" + String(frequency0 * 10) + "\n");
        }
      }

      if (XDRGTKTCP == true) {
        if (band == 5) {
          RemoteClient.print("T" + String(frequency5) + "\n");
        } else {
          RemoteClient.print("T" + String(frequency0 * 10) + "\n");
        }
      }
      change = 0;
      ShowFreq();
      store = true;
    }
    if (optrot != 1) {
      while (digitalRead(ROTARY_PIN_A) == LOW || digitalRead(ROTARY_PIN_B) == LOW) {
        delay(5);
      }
    }

    position = encoder.getPosition();
  }
}

void MemoryTune() {
  Display.writeStr("vis t14,1");
  Display.writeStr("vis channel,1");
  Display.writeNum("memvis",0);
  Display.writeStr("vis freq,0");
  Display.writeNum("showmem.en", 1);
  Display.writeNum("channel.val", memory_pos + 1);
  if (memorystore == false) {
    if (memory[memory_pos] >= LowEdgeSet0 * 100 && memory[memory_pos] <= HighEdgeSet0 * 100) {
      frequency0 = memory[memory_pos];
      RF(0);
      band = 0;
    } else if (UHF == true && memory[memory_pos] >= LowEdgeSet1 * 100 && memory[memory_pos] <= HighEdgeSet1 * 100) {
      frequency1 = memory[memory_pos];
      RF(1);
      band = 1;
    } else if (UHF == true && memory[memory_pos] >= LowEdgeSet2 * 100 && memory[memory_pos] <= HighEdgeSet2 * 100) {
      frequency2 = memory[memory_pos];
      RF(2);
      band = 2;
    } else if (UHF == true && memory[memory_pos] >= LowEdgeSet3 * 100 && memory[memory_pos] <= HighEdgeSet3 * 100) {
      frequency3 = memory[memory_pos];
      RF(3);
      band = 3;
    } else if (UHF == true && memory[memory_pos] >= LowEdgeSet4 * 100 && memory[memory_pos] <= HighEdgeSet4 * 100) {
      frequency4 = memory[memory_pos];
      RF(4);
      band = 4;
    } else if (memory[memory_pos] >= 14 && memory[memory_pos] <= 2700) {
      frequency5 = memory[memory_pos];
      if (frequency5 < 144) {
        frequency5 = 144;
      }
      RF(5);
      band = 5;
    } else {
      Display.writeNum("outlimit.en", 1);
    }
  }
  store = true;
  radio.clearRDS();
}

void trigger1() {
  doStereoToggle();
}

void trigger2() {
  BWset++;
  doBW();
  ShowBW();
}

void trigger3() {
  doFilter();
}

void trigger4() {
  BandSet();
}

void trigger5() {
  Display.writeStr("page 2");
  manfreq = true;
}

void trigger6() {
  doTuneMode();
}

void trigger7() {
  spec = true;
  Display.writeStr("page 3");
  BuildScanScreen();
}

void trigger8() {
  bool failed = false;
  freqtemp = Display.readNumber("freqresult.val");
  if (freqtemp >= 7777) {
    freqtemp = Display.readNumber("freqresult.val");
  }
  if (freqtemp >= LowEdgeSet0 * 100 && freqtemp <= HighEdgeSet0 * 100) {
    frequency0 = freqtemp;
    RF(0);
    band = 0;
  } else if (UHF == true && freqtemp >= LowEdgeSet1 * 100 && freqtemp <= HighEdgeSet1 * 100) {
    frequency1 = freqtemp;
    RF(1);
    band = 1;
  } else if (UHF == true && freqtemp >= LowEdgeSet2 * 100 && freqtemp <= HighEdgeSet2 * 100) {
    frequency2 = freqtemp;
    RF(2);
    band = 2;
  } else if (UHF == true && freqtemp >= LowEdgeSet3 * 100 && freqtemp <= HighEdgeSet3 * 100) {
    frequency3 = freqtemp;
    RF(3);
    band = 3;
  } else if (UHF == true && freqtemp >= LowEdgeSet4 * 100 && freqtemp <= HighEdgeSet4 * 100) {
    frequency4 = freqtemp;
    RF(4);
    band = 4;
  } else if (freqtemp >= 14 && freqtemp <= 2700) {
    frequency5 = freqtemp * 10;
    if (frequency5 < 144) {
      frequency5 = 144;
    }
    RF(5);
    band = 5;
  } else {
    Display.writeStr("vis outofrange,1");
    delay(1000);
    Display.writeStr("page 2");
    failed = true;
  }
  if (failed == false) {
    EEPROM.writeByte(46, band);
    EEPROM.commit();
    doExit();
  }
}

void trigger9() {
  Display.writeNum("freqresult.val", 0);
  Display.writeNum("fcount", 0);
  doExit();
}

void trigger10() { // Menu afsluiten met opslaan
  Display.writeNum("b0.pic", 123);
  Display.writeNum("b1.pic", 127);
  Display.writeNum("b2.pic", 129);
  Display.writeNum("b3.pic", 125);
  Display.writeNum("b4.pic", 131);
  Display.writeNum("b5.pic", 133);
  Display.writeStr("tsw b0, 0");
  Display.writeStr("tsw b1, 0");
  Display.writeStr("tsw b2, 0");
  Display.writeStr("tsw b3, 0");
  Display.writeStr("tsw b4, 0");
  Display.writeStr("tsw b5, 0");
  delay(100);

  LowEdgeSet0 = Display.readNumber("l0");
  if (LowEdgeSet0 >= 7777) {
    LowEdgeSet0 = Display.readNumber("l0");
  }
  delay(100);
  LowEdgeSet1 = Display.readNumber("l1");
  if (LowEdgeSet1 >= 7777) {
    LowEdgeSet1 = Display.readNumber("l1");
  }
  delay(100);
  LowEdgeSet2 = Display.readNumber("l2");
  if (LowEdgeSet2 >= 7777) {
    LowEdgeSet2 = Display.readNumber("l2");
  }
  delay(100);
  LowEdgeSet3 = Display.readNumber("l3");
  if (LowEdgeSet3 >= 7777) {
    LowEdgeSet3 = Display.readNumber("l3");
  }
  delay(100);
  LowEdgeSet4 = Display.readNumber("l4");
  if (LowEdgeSet4 >= 7777) {
    LowEdgeSet4 = Display.readNumber("l4");
  }
  delay(100);
  HighEdgeSet0 = Display.readNumber("h0");
  if (HighEdgeSet0 >= 7777) {
    HighEdgeSet0 = Display.readNumber("h0");
  }
  delay(100);
  HighEdgeSet1 = Display.readNumber("h1");
  if (HighEdgeSet1 >= 7777) {
    HighEdgeSet1 = Display.readNumber("h1");
  }
  delay(100);
  HighEdgeSet2 = Display.readNumber("h2");
  if (HighEdgeSet2 >= 7777) {
    HighEdgeSet2 = Display.readNumber("h2");
  }
  delay(100);
  HighEdgeSet3 = Display.readNumber("h3");
  if (HighEdgeSet3 >= 7777) {
    HighEdgeSet3 = Display.readNumber("h3");
  }
  delay(100);
  HighEdgeSet4 = Display.readNumber("h4");
  if (HighEdgeSet4 >= 7777) {
    HighEdgeSet4 = Display.readNumber("h4");
  }
  delay(100);
  LevelOffset0 = Display.readNumber("o0") - 25;
  if (LevelOffset0 >= 7777) {
    LevelOffset0 = Display.readNumber("o0") - 25;
  }
  delay(100);
  LevelOffset1 = Display.readNumber("o1") - 25;
  if (LevelOffset1 >= 7777) {
    LevelOffset1 = Display.readNumber("o1") - 25;
  }
  delay(100);
  LevelOffset2 = Display.readNumber("o2") - 25;
  if (LevelOffset2 >= 7777) {
    LevelOffset2 = Display.readNumber("o2") - 25;
  }
  delay(100);
  LevelOffset3 = Display.readNumber("o3") - 25;
  if (LevelOffset3 >= 7777) {
    LevelOffset3 = Display.readNumber("o3") - 25;
  }
  delay(100);
  LevelOffset4 = Display.readNumber("o4") - 25;
  if (LevelOffset4 >= 7777) {
    LevelOffset4 = Display.readNumber("o4") - 25;
  }
  delay(100);
  IF = Display.readNumber("iffreq");
  if (IF >= 7777) {
    IF = Display.readNumber("iffreq");
  }
  delay(100);
  lf = Display.readNumber("lf");
  if (lf >= 7777) {
    lf = Display.readNumber("lf");
  }
  delay(100);
  demp = Display.readNumber("demp");
  if (demp >= 7777) {
    demp = Display.readNumber("demp");
  }
  delay(100);
  usbmode = Display.readNumber("usbmode");
  if (usbmode >= 7777) {
    usbmode = Display.readNumber("usbmode");
  }
  delay(100);
  wifienable = Display.readNumber("wifienable");
  if (wifienable >= 7777) {
    wifienable = Display.readNumber("wifienable");
  }
  delay(100);
  ip1 = Display.readNumber("pcip1");
  if (ip1 >= 7777) {
    ip1 = Display.readNumber("pcip1");
  }
  delay(100);
  ip2 = Display.readNumber("pcip2");
  if (ip2 >= 7777) {
    ip2 = Display.readNumber("pcip2");
  }
  delay(100);
  ip3 = Display.readNumber("pcip3");
  if (ip3 >= 7777) {
    ip3 = Display.readNumber("pcip3");
  }
  delay(100);
  ip4 = Display.readNumber("pcip4");
  if (ip4 >= 7777) {
    ip4 = Display.readNumber("pcip4");
  }
  delay(100);
  stationlist = Display.readNumber("stationlog");
  if (stationlist >= 7777) {
    stationlist = Display.readNumber("stationlog");
  }
  delay(100);
  VolSet = Display.readNumber("Volset") - 15;
  if (VolSet >= 7777) {
    VolSet = Display.readNumber("Volset") - 15;
  }
  delay(100);
  ContrastSet = Display.readNumber("dim");
  if (ContrastSet >= 7777) {
    ContrastSet = Display.readNumber("dim");
  }
  delay(100);
  StereoLevel = Display.readNumber("StereoLevel");
  if (StereoLevel >= 7777) {
    StereoLevel = Display.readNumber("StereoLevel");
  }
  delay(100);
  HighCutLevel = Display.readNumber("HighCutLevel") / 100;
  if (HighCutLevel >= 77) {
    HighCutLevel = Display.readNumber("HighCutLevel") / 100;
  }
  delay(100);
  HighCutOffset = Display.readNumber("HighCutOffset");
  if (HighCutOffset >= 7777) {
    HighCutOffset = Display.readNumber("HighCutOffset");
  }
  delay(100);
  BlendLevel = Display.readNumber("BlendLevel") /100;
  if (BlendLevel >= 77) {
    BlendLevel = Display.readNumber("BlendLevel") /100;
  }
  delay(100);
  BlendOffset = Display.readNumber("BlendOffset");
  if (BlendOffset >= 7777) {
    BlendOffset = Display.readNumber("BlendOffset");
  }
  delay(100);
  NBLevel = Display.readNumber("NBLevel");
  if (NBLevel >= 7777) {
    NBLevel = Display.readNumber("NBLevel");
  }
  delay(100);
  AM_Cochannel = Display.readNumber("AMcochannel");
  if (AM_Cochannel >= 7777) {
    AM_Cochannel = Display.readNumber("AMcochannel");
  }
  delay(100);
  AM_NBLevel = Display.readNumber("AMnb");
  if (AM_NBLevel >= 7777) {
    AM_NBLevel = Display.readNumber("AMnb");
  }
  delay(100);
  AM_att = Display.readNumber("AMatt");
  if (AM_att >= 7777) {
    AM_att = Display.readNumber("AMatt");
  }
  delay(100);
  fm = Display.readNumber("fm");
  if (fm >= 2) {
    fm = Display.readNumber("fm");
  }
  delay(100);
  am = Display.readNumber("am");
  if (am >= 2) {
    am = Display.readNumber("am");
  }
  delay(100);
  uhf1 = Display.readNumber("uhf1");
  if (uhf1 >= 2) {
    uhf1 = Display.readNumber("uhf1");
  }
  delay(100);
  uhf2 = Display.readNumber("uhf2");
  if (uhf2 >= 2) {
    uhf2 = Display.readNumber("uhf2");
  }
  delay(100);
  uhf3 = Display.readNumber("uhf3");
  if (uhf3 >= 2) {
    uhf3 = Display.readNumber("uhf3");
  }
  delay(100);
  uhf4 = Display.readNumber("uhf4");
  if (uhf4 >= 2) {
    uhf4 = Display.readNumber("uhf4");
  }
  delay(100);
  coaxmode = Display.readNumber("coaxmode");
  if (coaxmode >= 2) {
    coaxmode = Display.readNumber("coaxmode");
  }
  delay(100);
  fmsi = Display.readNumber("fmsi");
  if (fmsi > 2) {
    fmsi = Display.readNumber("fmsi");
  }
  delay(100);
  fmsi_attack = Display.readNumber("fmsiattack");
  if (fmsi_attack > 1000) {
    fmsi_attack = Display.readNumber("fmsiattack");
  }
  delay(100);
  fmsi_release = Display.readNumber("fmsirelease");
  if (fmsi_release > 1000) {
    fmsi_release = Display.readNumber("fmsirelease");
  }
  delay(100);
  fmsi_11 = Display.readNumber("b1sens");
  if (fmsi_11 > 150) {
    fmsi_11 = Display.readNumber("b1sens");
  }
  delay(100);
  fmsi_12 = Display.readNumber("b1bias");
  if (fmsi_12 > 50) {
    fmsi_12 = Display.readNumber("b1bias");
  }
  delay(100);
  fmsi_21 = Display.readNumber("b2sens");
  if (fmsi_21 > 150) {
    fmsi_21 = Display.readNumber("b2sens");
  }
  delay(100);
  fmsi_22 = Display.readNumber("b2bias");
  if (fmsi_22 > 50) {
    fmsi_22 = Display.readNumber("b2bias");
  }
  delay(100);
  fmsi_31 = Display.readNumber("b3sens");
  if (fmsi_31 > 150) {
    fmsi_31 = Display.readNumber("b3sens");
  }
  delay(100);
  fmsi_32 = Display.readNumber("b3bias");
  if (fmsi_32 > 50) {
    fmsi_32 = Display.readNumber("b3bias");
  }
  delay(100);
  fmsi_41 = Display.readNumber("b4sens");
  if (fmsi_41 > 150) {
    fmsi_41 = Display.readNumber("b4sens");
  }
  delay(100);
  fmsi_42 = Display.readNumber("b4bias");
  if (fmsi_42 > 50) {
    fmsi_42 = Display.readNumber("b4bias");
  }

  EEPROM.writeInt(20, ContrastSet);
  EEPROM.writeInt(4, VolSet);
  EEPROM.writeInt(28, StereoLevel);
  EEPROM.writeInt(32, HighCutLevel);
  EEPROM.writeInt(36, HighCutOffset);
  EEPROM.writeInt(12, LowEdgeSet0);
  EEPROM.writeInt(51, LowEdgeSet1);
  EEPROM.writeInt(59, LowEdgeSet2);
  EEPROM.writeInt(67, LowEdgeSet3);
  EEPROM.writeInt(75, LowEdgeSet4);
  EEPROM.writeInt(16, HighEdgeSet0);
  EEPROM.writeInt(55, HighEdgeSet1);
  EEPROM.writeInt(63, HighEdgeSet2);
  EEPROM.writeInt(71, HighEdgeSet3);
  EEPROM.writeInt(79, HighEdgeSet4);
  EEPROM.writeInt(24, LevelOffset0);
  EEPROM.writeInt(117, LevelOffset1);
  EEPROM.writeInt(121, LevelOffset2);
  EEPROM.writeInt(125, LevelOffset3);
  EEPROM.writeInt(129, LevelOffset4);
  EEPROM.writeByte(47, IF);
  EEPROM.writeByte(8, lf);
  EEPROM.writeByte(9, demp);
  EEPROM.writeByte(48, usbmode);
  EEPROM.writeByte(49, wifienable);
  EEPROM.writeString(134, password);
  EEPROM.writeByte(140, ip1);
  EEPROM.writeByte(141, ip2);
  EEPROM.writeByte(142, ip3);
  EEPROM.writeByte(143, ip4);
  EEPROM.writeByte(43, stationlist);
  EEPROM.writeInt(144, BlendLevel);
  EEPROM.writeInt(148, BlendOffset);
  EEPROM.writeInt(152, NBLevel);
  EEPROM.writeInt(164, AM_Cochannel);
  EEPROM.writeInt(168, AM_NBLevel);
  EEPROM.writeInt(172, AM_att);
  EEPROM.writeByte(176, am);
  EEPROM.writeByte(177, fm);
  EEPROM.writeByte(178, uhf1);
  EEPROM.writeByte(179, uhf2);
  EEPROM.writeByte(180, uhf3);
  EEPROM.writeByte(181, uhf4);
  EEPROM.writeByte(182, coaxmode);
  EEPROM.writeByte(185, fmsi);
  EEPROM.writeUInt(187, fmsi_attack);
  EEPROM.writeUInt(191, fmsi_release);
  EEPROM.writeUInt(195, fmsi_11);  
  EEPROM.writeUInt(199, fmsi_12);
  EEPROM.writeUInt(203, fmsi_21);
  EEPROM.writeUInt(207, fmsi_22);
  EEPROM.writeUInt(211, fmsi_31);
  EEPROM.writeUInt(215, fmsi_32);
  EEPROM.writeUInt(219, fmsi_41);
  EEPROM.writeUInt(223, fmsi_42);
  EEPROM.commit();
  
  remoteip = IPAddress (ip1,ip2,ip3,ip4);

  if (wifienable == 2) {
    Udp.stop();
    if (stationlist == 1) {
      Udp.begin(9031);
    }
  }

  Serial.end();
  if (usbmode == 1) {
    Serial.begin(19200);
  } else {
    Serial.begin(115200);
  }

  if (band == 0) {
    radio.setOffset(LevelOffset0);
  } else if (band == 1) {
    radio.setOffset(LevelOffset1);
    radio.SetFreq(IF * 100);
  } else if (band == 2) {
    radio.setOffset(LevelOffset2);
    radio.SetFreq(IF * 100);
  } else if (band == 3) {
    radio.setOffset(LevelOffset3);
    radio.SetFreq(IF * 100);
  } else if (band == 4) {
    radio.setOffset(LevelOffset4);
    radio.SetFreq(IF * 100);
  }

  doExit();
}

void trigger11() { // Menu afsluiten zonder opslaan
  Display.writeNum("b0.pic", 123);
  Display.writeNum("b1.pic", 127);
  Display.writeNum("b2.pic", 129);
  Display.writeNum("b3.pic", 125);
  Display.writeNum("b4.pic", 131);
  Display.writeNum("b5.pic", 133);
  Display.writeStr("tsw b0, 0");
  Display.writeStr("tsw b1, 0");
  Display.writeStr("tsw b2, 0");
  Display.writeStr("tsw b3, 0");
  Display.writeStr("tsw b4, 0");
  Display.writeStr("tsw b5, 0");

  EEpromReadData();
  radio.setVolume(VolSet);
  radio.setDeemphasis(demp);
  radio.setAudio(lf);
  trigger17();
  doExit();
}

void trigger12() { // Menu oude volume terugzetten
  radio.setVolume(VolSet);
  radio.setDeemphasis(demp);
  radio.setAudio(lf);
  Display.writeNum("demp", demp);
  Display.writeNum("lf", lf);
  Display.writeNum("Volset", VolSet + 15);
}

void trigger13() { // Nieuwe volume waarde toepassen
  radio.setVolume(Display.readNumber("slider.val") - 15);

}

void trigger14() { // Nieuwe waarde high cut offset waarde (Hz) toepassen
  radio.setHighCutLevel(Display.readNumber("slider2.val") / 100);
}

void trigger15() { // Nieuwe waarde high cut threshold toepassen
  radio.setHighCutOffset(Display.readNumber("slider3.val"));
}

void trigger16() { // Nieuwe waarde stereo sep. threshold toepassen
  radio.setStereoLevel(Display.readNumber("slider.val"));
}

void trigger17() { // oude waarde DSP terugzetten
  radio.setHighCutLevel(HighCutLevel);
  radio.setHighCutOffset(HighCutOffset);
  radio.setStereoLevel(StereoLevel);
  radio.setStHiBlendLevel(BlendLevel);
  radio.setStHiBlendOffset(BlendOffset);
  radio.setNoiseBlanker(NBLevel);
  radio.setAMNoiseBlanker(AM_NBLevel);
  radio.setAMAttenuation(AM_att);
  radio.setAMCoChannel(AM_Cochannel);
  radio.setFMSI(fmsi);
  radio.setFMSI_Time(fmsi_attack, fmsi_release);
  radio.setFMSI_Gain(fmsi_11, fmsi_21, fmsi_31, fmsi_41);
  radio.setFMSI_Bias(fmsi_12, fmsi_22, fmsi_32, fmsi_42);
}

void trigger18() { // de-emphasis instellen
  radio.setDeemphasis(Display.readNumber("demp"));
}

void trigger19() { // Audio mode instellen
  radio.setAudio(Display.readNumber("lf"));
}

void trigger20() { // Open Wifi config pagina geopend, vul wifi status in...
  wifienableold = wifienable;
  if (wifienable == 2) {
    String ssid = String(WiFi.SSID());
    Display.writeStr("IP.txt", WiFi.localIP().toString());
    Display.writeStr("SSID.txt", ssid);
    Display.writeNum("p1.pic", 183);
    Display.writeStr("password.txt", password);
  } else {
    Display.writeStr("SSID.txt", "DISCONNECTED");
    Display.writeNum("p1.pic", 175);
  }
}

void trigger21() { // Wifi connect pagina geopend, vul SSID AP in....
  String ssid = "ESP_" + String(ESP_getChipId());
  String WifiAP = String ("WIFI:T:nopass;S:" + ssid + ";P:;H:;");
  Display.writeStr("qr.txt", WifiAP);
  String SSID_AP = String ("SSID: " + ssid);
  Display.writeStr("SSID_AP.txt", SSID_AP);
  wc.startConfigurationPortal(AP_WAIT);
  ESP.restart();
}

void trigger22() { // Enable/disable WiFi
  wifienable++;
  if (wifienable == 3) {
    wifienable = 1;
  }
  if (wifienable == 1) {
    Display.writeNum("p1.pic", 175);
    digitalWrite(LED_BUILTIN, LOW);
    Server.end();
    Udp.stop();
    WiFi.mode(WIFI_OFF);
    Display.writeNum("wifienable", wifienable);
    Display.writeStr("IP.txt", "");
    Display.writeStr("SSID.txt", "DISCONNECTED");
    Display.writeStr("password.txt", "");
  } else if (wifienable == 2) {
    Display.writeNum("p1.pic", 184);
    Display.writeNum("wifienable", wifienable);
    Display.writeStr("SSID.txt", "Connecting.....");
    WiFi.begin();
    if (wc.autoConnect()) {
      Server.begin();
      Udp.begin(9031);
      digitalWrite(LED_BUILTIN, HIGH);
      String ssid = String(WiFi.SSID());
      Display.writeStr("IP.txt", WiFi.localIP().toString());
      Display.writeStr("SSID.txt", ssid);
      Display.writeStr("password.txt", password);
      Display.writeNum("p1.pic", 183);
    } else {
      Display.writeStr("page 10");
      trigger21();
    }
  }
}

void trigger23() {
  passwordgenerator();
  Display.writeStr("password.txt", password);
}

void trigger24() {
  // Open RDS info
  displayreset = true;
  showrdsinfo = true;
  OStatusold = 0;
  delay(200);
  ShowFreq();
  delay(300);
  ShowBW();
  ShowStereoStatus();
  ShowOffset();
  ShowStepSize();
  ShowTuneMode();
  ShowUSBstatus();
  ShowBTstatus();
  ctshow = false;
  radio.clearRDS();
  if (wificonnect == true) {
    Display.writeNum("tm2.en", 1);
  }
}

void trigger25() {
  // Sluit RDS info
  showrdsinfo = false;
  doExit();
  if (wificonnect == true) {
    Display.writeNum("tm2.en", 1);
  }
}

void trigger26() {
  btsetup = true;
  btconnect = false;
  swSer.print("AT+DISCON");
  delay(250);
  swSer.print("AT+REST");
}

void trigger27() {
  btsetup=false;
}

void trigger28() {
  radio.setStHiBlendLevel(Display.readNumber("slider2.val") / 100);
}

void trigger29() {
  radio.setStHiBlendOffset(Display.readNumber("slider3.val"));
}

void trigger30() { 
  radio.setNoiseBlanker(Display.readNumber("slider.val"));
}

void trigger31() { // Set AM noise blanker
  radio.setAMNoiseBlanker(Display.readNumber("slider2.val"));
}

void trigger32() { // AM att
  radio.setAMAttenuation(Display.readNumber("slider3.val"));
}

void trigger33() { // Set cochannel
  radio.setAMCoChannel(Display.readNumber("slider.val"));
}

void trigger34() { // Set fmsi
  radio.setFMSI(Display.readNumber("fmsi"));
}

void trigger35() { // Set FMSI attack/release
  radio.setFMSI_Time(Display.readNumber("slider.val"), Display.readNumber("slider2.val"));
}

void trigger36() { // Set FMSI bands
  radio.setFMSI_Gain(Display.readNumber("b1sens"), Display.readNumber("b2sens"), Display.readNumber("b3sens"), Display.readNumber("b4sens"));
  radio.setFMSI_Bias(Display.readNumber("b1bias"), Display.readNumber("b2bias"), Display.readNumber("b3bias"), Display.readNumber("b4bias"));
}

void trigger37() { 
  scopeview = Display.readNumber("scopeview");
  EEPROM.writeByte(186, scopeview);
  EEPROM.commit();
}

void doExit() {
  if (band == 5) {
    showrdsinfo = false;
  }
  BWsetOld = 254;
  displayreset = true;
  manfreq = false;
  menu = false;
  spec = false;
  BWreset = true;
  Display.writeNum("smeterold", 0);  
  radio.clearRDS();
  if (showrdsinfo == false) {
    Display.writeStr("page 1");
  } else {
    Display.writeStr("page 12");
  }
  if (band == 5) {
    Display.writeStr("vis m0,0");
    Display.writeNum("freq.vvs1", 3);
    Display.writeStr("BER.txt", " ");
  } else {
    Display.writeStr("vis m0,1");
    Display.writeNum("freq.vvs1", 2);
  }
  delay(200);
  ShowFreq();
  delay(200);
  RF(band);
  ShowiMS();
  ShowEQ();
  Squelchold = 200;
  SNRold = 0;
  doSquelch();
  ShowBW();
  ShowSignalLevel();
  ShowStereoStatus();
  ShowOffset();
  ShowStepSize();
  ShowTuneMode();
  ShowUSBstatus();
  ShowBTstatus();
  Display.writeNum("tm3.en", 0);
  Display.writeStr("vis 52,0");
  Display.writeStr("t7.txt", "PTY:");
  ctshow = false;
  store = true;
}

void doBW() {
  if (band == 5) {
    if (BWsetAM > 4) {
      BWsetAM = 1;
    }
    ShowBW();
    BWreset = true;
    if (BWsetAM == 1) {
      radio.setAMBandw(3);
    }
    if (BWsetAM == 2) {
      radio.setAMBandw(4);
    }
    if (BWsetAM == 3) {
      radio.setAMBandw(6);
    }
    if (BWsetAM == 4) {
      radio.setAMBandw(8);
    }
  } else {
    if (BWset > 16) {
      BWset = 0;
    }
    ShowBW();
    BWreset = true;

    if (BWset == 1) {
      radio.setFMBandw(56);
    }
    if (BWset == 2) {
      radio.setFMBandw(64);
    }
    if (BWset == 3) {
      radio.setFMBandw(72);
    }
    if (BWset == 4) {
      radio.setFMBandw(84);
    }
    if (BWset == 5) {
      radio.setFMBandw(97);
    }
    if (BWset == 6) {
      radio.setFMBandw(114);
    }
    if (BWset == 7) {
      radio.setFMBandw(133);
    }
    if (BWset == 8) {
      radio.setFMBandw(151);
    }
    if (BWset == 9) {
      radio.setFMBandw(168);
    }
    if (BWset == 10) {
      radio.setFMBandw(184);
    }
    if (BWset == 11) {
      radio.setFMBandw(200);
    }
    if (BWset == 12) {
      radio.setFMBandw(217);
    }
    if (BWset == 13) {
      radio.setFMBandw(236);
    }
    if (BWset == 14) {
      radio.setFMBandw(254);
    }
    if (BWset == 15) {
      radio.setFMBandw(287);
    }
    if (BWset == 16) {
      radio.setFMBandw(311);
    }
  }
}

void doSquelch() {
  if (power == false) {
    if (XDRGTK == false && XDRGTKTCP == false) {
      Squelch = analogRead(PIN_POT) / 4 - 100;
      if (Squelch > 920) {
        Squelch = 920;
      }
      if (showrdsinfo == false && menu == false && Squelch != Squelchold) {
        if (Squelch == -100) {
          Display.writeStr("SQ.txt", "OFF");
        } else if (Squelch == 920) {
          Display.writeStr("SQ.txt", "ST");
        } else {
          String SQVAL = String(Squelch / 10);
          Display.writeStr("SQ.txt", SQVAL);
        }
        Squelchold = Squelch;
      }

    }
    if (XDRGTK == true || XDRGTKTCP == true) {
      if (XDRMute == false) {
        if (Squelch != -1) {
          if (seek == false && (Squelch == 0)) {
          if (mutestatus == true) {
            radio.setUnMute();
            mutestatus = false;
          }
            if (showrdsinfo == false) {
              Display.writeStr("SQ.txt", "OFF");
            }
            SQ = false;
          } else if (seek == false && (Squelch < SStatus || Squelch == -100)) {
          if (mutestatus == true) {
            radio.setUnMute();
          mutestatus = false;
          }
            SQ = false;
          } else {
            if (mutestatus == false) {
              radio.setMute();
              mutestatus = true;
            }
            SQ = true;
          }
        } else {
          if (seek == false && Stereostatus == true) {
          if (mutestatus == true) {
            radio.setUnMute();
          mutestatus = false;
          }
            SQ = false;
          } else {
            if (mutestatus == false) {
              radio.setMute();
              mutestatus = true;
            }
            SQ = true;
          }
        }
        if (showrdsinfo == false) {
          if (Squelch != Squelchold) {
            if (Squelch == -1) {
              Display.writeStr("SQ.txt", "ST");
            } else {
              String SQVAL = String(Squelch / 10);
              Display.writeStr("SQ.txt", SQVAL);
            }
            Squelchold = Squelch;
          }
        }
      }
    } else {
      if (Squelch != 920) {
        if (seek == false && (Squelch < SStatus || Squelch == -100)) {
          if (mutestatus == true) {
            radio.setUnMute();
            mutestatus = false;
          }
          SQ = false;
        } else {
            if (mutestatus == false) {
              radio.setMute();
              mutestatus = true;
            }
          SQ = true;
        }
      } else {
        if (Stereostatus == true) {
          if (mutestatus == true) {
            radio.setUnMute();
            mutestatus = false;
          }
          SQ = false;
        } else {
            if (mutestatus == false) {
              radio.setMute();
              mutestatus = true;
            }
          SQ = true;
        }
      }
    }
    ShowSquelch();
  }
}

void ShowSquelch() {
  if (menu == false) {
    if (SQ == false) {
      if (mutelogo_off == false) {
        Display.writeNum("mutelogo.pic", 32);
        mutelogo_off = true;
        mutelogo_on = false;
      }
    } else {
      if (mutelogo_on == false) {
        Display.writeNum("mutelogo.pic", 33);
        mutelogo_on = true;
        mutelogo_off = false;
      }
    }
  }
}

void doStereoToggle() {
  if (StereoToggle == true) {
    Display.writeNum("stereo.pic", 11);
    radio.setMono(2);
    StereoToggle = false;
  } else {
    Display.writeNum("stereo.pic", 10);
    radio.setMono(0);
    Stereostatusold = false;
    StereoToggle = true;
  }
}

void SpectrumAnalyser() {
  radio.setFMBandw(scanner_vbw);
  radio.setMute();
  RF(scanner_band);
  int scancounter = 20;
  if (displaysize == 35) {
    scancounter = 35;
    Display.writeStr("fill 35,286,222,3,BLACK");
  } else {
    Display.writeStr("fill 20,222,222,3,BLACK");
  }

  for (freq_scan = scanner_start; freq_scan <= scanner_end; freq_scan += (((scanner_end - scanner_start) / 100) / 2) * scanner_speed)
  {
    if (scanner_band == 0) {
      radio.SetFreq(freq_scan);
    } else {
      Frontend.SetFreq(freq_scan - (IF * 100));
    }
    delay(20);

    radio.getStatus(SStatus, USN, WAM, OStatus, BW, MStatus);

    if (displaysize == 35) {
      for (int i = 1; i <= scanner_speed && scancounter < 236; i++) {
        scancounter++;
        if (SStatus < 0) {
          SStatus = 5;
        }
        String LevelString = "line " + String(scancounter, DEC) + ",284," + String(scancounter, DEC) + "," + String(284 - SStatus / 5, DEC) + ",65535";
        String LevelBlankString = "line " + String(scancounter, DEC) + ",84," + String(scancounter, DEC) + "," + String(284 - SStatus / 5, DEC) + ",BLACK";
        String LevelDot = "line " + String(scancounter, DEC) + ",286," + String(scancounter, DEC) + ",288,65504";
        Display.writeStr(LevelString);
        Display.writeStr(LevelBlankString);
        Display.writeStr(LevelDot);

        for (int dot = 284; dot >= 84; dot -= 20) {
          String DotString = "fill " + String(scancounter, DEC) + "," + String(dot, DEC) + ",2,2,33840";
          Display.writeStr(DotString);
        }

        for (int dot = 235; dot >= 20; dot -= 20) {
          String DotString = "fill " + String(dot, DEC) + ",84,2,201,33840";
          Display.writeStr(DotString);
        }
      }
    } else {
      for (int i = 1; i <= scanner_speed && scancounter < 221; i++) {
        scancounter++;
        if (SStatus < 0) {
          SStatus = 5;
        }
        String LevelString = "line " + String(scancounter, DEC) + ",220," + String(scancounter, DEC) + "," + String(220 - SStatus / 5, DEC) + ",65535";
        String LevelBlankString = "line " + String(scancounter, DEC) + ",1," + String(scancounter, DEC) + "," + String(219 - SStatus / 5, DEC) + ",BLACK";
        String LevelDot = "line " + String(scancounter, DEC) + "," + String(222, DEC) + "," + String(scancounter, DEC) + "," + String(224, DEC) + ",65504";
        Display.writeStr(LevelString);
        Display.writeStr(LevelBlankString);
        Display.writeStr(LevelDot);

        for (int dot = 220; dot >= 20; dot -= 20) {
          String DotString = "fill " + String(scancounter, DEC) + "," + String(dot, DEC) + ",2,2,33840";
          Display.writeStr(DotString);
        }

        for (int dot = 220; dot >= 20; dot -= 20) {
          String DotString = "fill " + String(dot, DEC) + ",20,2,201,33840";
          Display.writeStr(DotString);
        }
      }
    }
    if (scanner_thenable == 2 && scanfound == false && SStatus / 10 > scanner_th) {
      scanner_found = freq_scan - ((scanner_end - scanner_start) / 100) / 2;
      scanfound = true;
      Display.writeNum("scanstatus", 5);
    } else {
      donesearch = true;
    }
  }
}

void BuildScanScreen() {
  millisoff = true;
  delay(200);
  Display.writeStr("tsw bstart,1");
  Display.writeStr("tsw bconfig,1");
  Display.writeStr("tsw bexit,1");
  Display.writeStr("tsw bstop,0");
  Display.writeNum("bstart.pic", 147);
  Display.writeNum("bconfig.pic", 151);
  Display.writeNum("bexit.pic", 153);
  Display.writeNum("bstop.pic", 150);
  String rbw = String(((((scanner_end - scanner_start) / 100) / 2) * 10) * scanner_speed, DEC) + " kHz";
  String vbw = String(scanner_vbw, DEC) + " kHz";
  String span = String((scanner_end - scanner_start) / 100, DEC) + " MHz";
  Display.writeStr("RBW.txt", rbw);
  Display.writeStr("VBW.txt", vbw);
  Display.writeStr("SPAN.txt", span);

  if (displaysize == 35) {
    for (int dot = 284; dot >= 84; dot -= 20)
    {
      String DotString = "fill 35," + String(dot, DEC) + ",201,2,33840";
      Display.writeStr(DotString);
    }
    for (int dot = 235; dot >= 20; dot -= 20)
    {
      String DotString = "fill " + String(dot, DEC) + ",84,2,201,33840";  
      Display.writeStr(DotString);
    }
  } else {
      for (int dot = 220; dot >= 20; dot -= 20)
      {
        String DotString = "fill 20," + String(dot, DEC) + ",201,2,33840";
        Display.writeStr(DotString);
      }
      for (int dot = 220; dot >= 20; dot -= 20)
      {
        String DotString = "fill " + String(dot, DEC) + ",20,2,201,33840";  
        Display.writeStr(DotString);
      }
  }
}

void scan_loop() {
  // Loop voor Spectrum Analyser
  if (millisoff == true) {
    scanstatus = Display.readNumber("scanstatus");
  }

  if (millis() >= time_now + 2000) {
    time_now += 2000;
    millisoff = true;
  }

  // // Afsluiten van configuratie, knoppen grijs, gegevens wel opgeslagen
  if (scanstatus == 2) {
    Display.writeStr("tsw b0,0");
    Display.writeStr("tsw b1,0");
    Display.writeStr("tsw b2,0");
    Display.writeStr("tsw b3,0");
    Display.writeNum("b0.bco", 33840);
    Display.writeNum("b1.bco", 33840);
    Display.writeNum("b2.bco", 33840);
    Display.writeNum("b3.bco", 33840);
    delay(100);
    scanner_start = Display.readNumber("scanstart.val") * 100;
    delay (100);
    scanner_end = Display.readNumber("scanstop.val") * 100;
    delay (100);
    scanner_vbw = Display.readNumber("scanbw.val");
    delay (100);
    scanner_th = Display.readNumber("scanthreshold.val");
    delay (100);
    scanner_band = Display.readNumber("sband");
    delay (100);
    scanner_speed = Display.readNumber("speed");
    delay (100);
    scanner_thenable = Display.readNumber("then.val");
    EEPROM.writeUInt(99, scanner_start);
    EEPROM.writeUInt(103, scanner_end);
    EEPROM.writeUInt(107, scanner_vbw);
    EEPROM.writeByte(10, scanner_th);
    EEPROM.writeByte(11, scanner_band);
    EEPROM.writeByte(40, scanner_speed);
    EEPROM.writeByte(113, scanner_thenable);
    EEPROM.commit();
    Display.writeStr("page 3");
    BuildScanScreen();

    // Afsluiten van configuratie, knoppen grijs, gegevens niet opgeslagen
  } else if (scanstatus == 3 ) {
    Display.writeStr("tsw b0,0");
    Display.writeStr("tsw b1,0");
    Display.writeStr("tsw b2,0");
    Display.writeStr("tsw b3,0");
    Display.writeNum("b0.bco", 33840);
    Display.writeNum("b1.bco", 33840);
    Display.writeNum("b2.bco", 33840);
    Display.writeNum("b3.bco", 33840);
    Display.writeStr("page 3");
    BuildScanScreen();

    // Indien op start wordt gedrukt de analyser starten totdat er op stop wordt gedrukt.
  } else if (scanstatus == 4) {
    Display.writeStr("vis scanner_found,0");
    if (displaysize == 35) {
      for (int dot = 284; dot >= 80; dot -= 20)
      {
        String DotString = "fill 35," + String(dot, DEC) + ",201,2,33840";
        Display.writeStr(DotString);
      }

      for (int dot = 235; dot >= 20; dot -= 20)
      {
        String DotString = "fill " + String(dot, DEC) + ",84,2,201,33840";
        Display.writeStr(DotString);
      }
    } else {
      for (int dot = 220; dot >= 20; dot -= 20)
      {
        String DotString = "fill 20," + String(dot, DEC) + ",201,2,33840";
        Display.writeStr(DotString);
      }

      for (int dot = 220; dot >= 20; dot -= 20)
      {
        String DotString = "fill " + String(dot, DEC) + ",20,2,201,33840";
        Display.writeStr(DotString);
      }
    }

    Display.writeStr("tsw bstop,1");
    Display.writeStr("tsw bconfig,0");
    Display.writeStr("tsw bexit,0");
    Display.writeStr("tsw bstart,0");
    Display.writeNum("bstop.pic", 149);
    Display.writeNum("bconfig.pic", 152);
    Display.writeNum("bexit.pic", 154);
    Display.writeNum("bstart.pic", 148);
    donesearch = false;
    SpectrumAnalyser();

  } else if (scanstatus == 5) {
    Display.writeStr("tsw bstart,1");
    Display.writeStr("tsw bconfig,1");
    Display.writeStr("tsw bexit,1");
    Display.writeStr("tsw bstop,0");
    Display.writeNum("bstart.pic", 147);
    Display.writeNum("bconfig.pic", 151);
    Display.writeNum("bexit.pic", 153);
    Display.writeNum("bstop.pic", 150);
    Display.writeStr("vis scanner_found,1");

    if (scanfound == true) {
      if (scanner_band == 0)
      {
        radio.SetFreq(scanner_found);
      } else {
        radio.SetFreq(IF * 100);
        Frontend.SetFreq(scanner_found - (IF * 100));
      }
      radio.setUnMute();
      radio.setFMABandw();
      donesearch = false;
      scanfound = false;
      Display.writeNum("scanner_found.val", scanner_found);
      String DotString;
      if (displaysize == 35) {
        Display.writeStr("fill 35,286,237,3,BLACK");
        DotString = "fill 35,286," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
      } else {
        Display.writeStr("fill 20,222,222,3,BLACK");
        DotString = "fill 20,222," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
      }
      Display.writeStr(DotString);
      position = encoder.getPosition();
    }

    if (donesearch == true) {
      scanner_found = ((scanner_end - scanner_start) / 2) + scanner_start;
      if (scanner_band == 0)
      {
        radio.SetFreq(scanner_found);
      } else {
        radio.SetFreq(IF * 100);
        Frontend.SetFreq(scanner_found - (IF * 100));
      }

      radio.setUnMute();
      radio.setFMABandw();
      donesearch = false;
      Display.writeNum("scanner_found.val", scanner_found);
      String DotString;
      if (displaysize == 35) {
        Display.writeStr("fill 35,286,222,3,BLACK");
        DotString = "fill 35,286," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
      } else {
        Display.writeStr("fill 20,222,222,3,BLACK");
        DotString = "fill 20,222," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
      }
      Display.writeStr(DotString);
      position = encoder.getPosition();
    } else {
      if (position < encoder.getPosition()) {
        if (rotarymode == 0) {
          scanup();
        } else {
          scandown();
        }
      }

      if (position > encoder.getPosition()) {
        if (rotarymode == 0) {
          scandown();
        } else {
          scanup();
        }
      }

      if (encoder.getPushButton() == true) {
        freq = scanner_found;
        band = scanner_band;
        if (band == 0) {
          frequency0 = freq;
        }
        if (band == 1) {
          frequency1 = freq;
        }
        if (band == 2) {
          frequency2 = freq;
        }
        if (band == 3) {
          frequency3 = freq;
        }
        if (band == 4) {
          frequency4 = freq;
        }
        doExit();
        RoundStep();
      }
    }

    // Indien op exit wordt gedrukt de analyser afsluiten zonder af te stemmen
  } else if (scanstatus == 6) {
    spec = false;
    doExit();
  }
}

void scanup() {
  scanner_found += (((scanner_end - scanner_start) / 100) / 2);
  if (scanner_found > scanner_end) {
    scanner_found = scanner_start;
  }

  if (scanner_band == 0)
  {
    radio.SetFreq(scanner_found);
  } else {
    Frontend.SetFreq(scanner_found - (IF * 100));
  }
  String DotString;
  if (displaysize == 35) {
    Display.writeStr("fill 35,286,237,3,BLACK");
    DotString = "fill 35,286," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
  } else {
    Display.writeStr("fill 20,222,222,3,BLACK");
    DotString = "fill 20,222," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
  }
  Display.writeNum("scanner_found.val", scanner_found);
  Display.writeStr(DotString);
  while (digitalRead(ROTARY_PIN_A) == LOW || digitalRead(ROTARY_PIN_B) == LOW) {
    delay(5);
  }
  if (optrot != 1) {
    while (digitalRead(ROTARY_PIN_A) == LOW || digitalRead(ROTARY_PIN_B) == LOW) {
      delay(5);
    }
  }
  position = encoder.getPosition();
  millisoff = false;
  time_now = millis();
}

void scandown() {
  scanner_found -= (((scanner_end - scanner_start) / 100) / 2);
  if (scanner_found < scanner_start) {
    scanner_found = scanner_end;
  }

  if (scanner_band == 0)
  {
    radio.SetFreq(scanner_found);
  } else {
    Frontend.SetFreq(scanner_found - (IF * 100));
  }
  String DotString;
  if (displaysize == 35) {
    Display.writeStr("fill 35,286,237,3,BLACK");
    DotString = "fill 35,286," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
  } else {
    Display.writeStr("fill 20,222,222,3,BLACK");
    DotString = "fill 20,222," + String((scanner_found - scanner_start) / (((scanner_end - scanner_start) / 100) / 2)) + ",2,65504";
  }
  Display.writeNum("scanner_found.val", scanner_found);
  Display.writeStr(DotString);
  while (digitalRead(ROTARY_PIN_A) == LOW || digitalRead(ROTARY_PIN_B) == LOW) {
    delay(5);
  }
  if (optrot != 1) {
    while (digitalRead(ROTARY_PIN_A) == LOW || digitalRead(ROTARY_PIN_B) == LOW) {
      delay(5);
    }
  }
  position = encoder.getPosition();
  millisoff = false;
  time_now = millis();
    // Einde Spectrum analyser loop
}

void EEpromReadData() {
  frequency0 = EEPROM.readUInt(0);
  VolSet = EEPROM.readInt(4);
  ContrastSet = EEPROM.readInt(20);
  StereoLevel = EEPROM.readInt(28);
  HighCutLevel = EEPROM.readInt(32);
  HighCutOffset = EEPROM.readInt(36);
  scanner_speed = EEPROM.readByte(40);
  stepsize = EEPROM.readByte(42);
  stationlist = EEPROM.readByte(43);
  iMSset = EEPROM.readByte(44);
  EQset = EEPROM.readByte(45);
  band = EEPROM.readByte(46);
  IF = EEPROM.readByte(47);
  LowEdgeSet0 = EEPROM.readInt(12);
  HighEdgeSet0 = EEPROM.readInt(16);
  LowEdgeSet1 = EEPROM.readInt(51);
  HighEdgeSet1 = EEPROM.readInt(55);
  LowEdgeSet2 = EEPROM.readInt(59);
  HighEdgeSet2 = EEPROM.readInt(63);
  LowEdgeSet3 = EEPROM.readInt(67);
  HighEdgeSet3 = EEPROM.readInt(71);
  LowEdgeSet4 = EEPROM.readInt(75);
  HighEdgeSet4 = EEPROM.readInt(79);
  frequency1 = EEPROM.readUInt(83);
  frequency2 = EEPROM.readUInt(87);
  frequency3 = EEPROM.readUInt(91);
  frequency4 = EEPROM.readUInt(95);
  scanner_start = EEPROM.readUInt(99);
  scanner_end = EEPROM.readUInt(103);
  scanner_vbw = EEPROM.readUInt(107);
  scanner_th = EEPROM.readByte(10);
  scanner_band = EEPROM.readByte(11);
  scanner_thenable = EEPROM.readByte(113);
  LevelOffset0 = EEPROM.readInt(24);
  LevelOffset1 = EEPROM.readInt(117);
  LevelOffset2 = EEPROM.readInt(121);
  LevelOffset3 = EEPROM.readInt(125);
  LevelOffset4 = EEPROM.readInt(129);
  lf = EEPROM.readByte(8);
  demp = EEPROM.readByte(9);
  usbmode = EEPROM.readByte(48);
  wifienable = EEPROM.readByte(49);
  password = EEPROM.readString(134);
  ip1 = EEPROM.readByte(140);
  ip2 = EEPROM.readByte(141);
  ip3 = EEPROM.readByte(142);
  ip4 = EEPROM.readByte(143);
  BlendLevel = EEPROM.readInt(144);
  BlendOffset = EEPROM.readInt(148);
  NBLevel = EEPROM.readInt(152);
  CoaxSwitch = EEPROM.readByte(157);
  frequency5 = EEPROM.readUInt(158);
  rotarymode = EEPROM.readByte(163);
  AM_Cochannel = EEPROM.readInt(164);
  AM_NBLevel = EEPROM.readInt(168);
  AM_att = EEPROM.readInt(172);
  am = EEPROM.readByte(176);
  fm = EEPROM.readByte(177);
  uhf1 = EEPROM.readByte(178);
  uhf2 = EEPROM.readByte(179);
  uhf3 = EEPROM.readByte(180);
  uhf4 = EEPROM.readByte(181);
  coaxmode = EEPROM.readByte(182);
  optrot = EEPROM.readByte(184);
  fmsi = EEPROM.readByte(185);
  scopeview = EEPROM.readByte(186);
  fmsi_attack = EEPROM.readUInt(187);
  fmsi_release = EEPROM.readUInt(191);
  fmsi_11 = EEPROM.readUInt(195);
  fmsi_12 = EEPROM.readUInt(199);
  fmsi_21 = EEPROM.readUInt(203);
  fmsi_22 = EEPROM.readUInt(207);
  fmsi_31 = EEPROM.readUInt(211);
  fmsi_32 = EEPROM.readUInt(215);
  fmsi_41 = EEPROM.readUInt(219);
  fmsi_42 = EEPROM.readUInt(223);
  memory_pos = EEPROM.readByte(227);
  tunemode = EEPROM.readByte(228);

  uint8_t i;
  for (i = 0; i < 30; i++) {
    memory[i] = EEPROM.readUInt((i*4) + 230);
  }
  EEPROM.commit();

  if (UHF == false && band !=0 && band !=5) {
    band = 0;
    if (scanner_start > 10800) {
      scanner_band = 0;
      scanner_start = LowEdgeSet0 * 100;
      scanner_end = HighEdgeSet0 * 100;
    }
  }
  
  remoteip = IPAddress (ip1,ip2,ip3,ip4);
  delay(100);
  displaysize = Display.readNumber("displaysize");
  if (displaysize >= 7777) {
    delay(100);
    displaysize = Display.readNumber("displaysize");
  }
  delay(20);
  Display.writeNum("sstart", scanner_start /100);
  delay(20);
  Display.writeNum("sstop", scanner_end /100);
  delay(20);
  Display.writeNum("sbw", scanner_vbw);
  delay(20);
  Display.writeNum("sth", scanner_th);
  delay(20);
  Display.writeNum("sband", scanner_band);
  delay(20);
  Display.writeNum("sspeed", scanner_speed);
  delay(20);
  Display.writeNum("thenable", scanner_thenable);
  delay(20);
  Display.writeNum("l0", LowEdgeSet0 / 2 * 2);
  delay(20);
  Display.writeNum("l1", LowEdgeSet1 / 2 * 2);
  delay(20);
  Display.writeNum("l2", LowEdgeSet2 / 2 * 2);
  delay(20);
  Display.writeNum("l3", LowEdgeSet3 / 2 * 2);
  delay(20);
  Display.writeNum("l4", LowEdgeSet4 / 2 * 2);
  delay(20);
  Display.writeNum("h0", HighEdgeSet0 / 2 * 2);
  delay(20);
  Display.writeNum("h1", HighEdgeSet1 / 2 * 2);
  delay(20);
  Display.writeNum("h2", HighEdgeSet2 / 2 * 2);
  delay(20);
  Display.writeNum("h3", HighEdgeSet3 / 2 * 2);
  delay(20);
  Display.writeNum("h4", HighEdgeSet4 / 2 * 2);
  delay(20);
  Display.writeNum("Volset", VolSet + 15);
  delay(20);
  Display.writeNum("dim", ContrastSet);
  delay(20);
  Display.writeNum("HighCutOffset", HighCutOffset);
  delay(20);
  Display.writeNum("HighCutLevel", HighCutLevel * 100);
  delay(20);
  Display.writeNum("StereoLevel", StereoLevel);
  delay(20);
  Display.writeNum("o0", LevelOffset0 + 25);
  delay(20);
  Display.writeNum("o1", LevelOffset1 + 25);
  delay(20);
  Display.writeNum("o2", LevelOffset2 + 25);
  delay(20);
  Display.writeNum("o3", LevelOffset3 + 25);
  delay(20);
  Display.writeNum("o4", LevelOffset4 + 25);
  delay(20);
  Display.writeNum("iffreq", IF);
  delay(20);
  Display.writeNum("demp", demp);
  delay(20);
  Display.writeNum("lf", lf);
  delay(20);
  Display.writeNum("usbmode", usbmode);
  delay(20);
  Display.writeNum("wifienable", wifienable);
  delay(20);
  Display.writeNum("pcip1", ip1);
  delay(20);
  Display.writeNum("pcip2", ip2);
  delay(20);
  Display.writeNum("pcip3", ip3);
  delay(20);
  Display.writeNum("pcip4", ip4);
  delay(20);
  Display.writeNum("stationlog", stationlist);
  delay(20);
  Display.writeNum("BlendLevel", BlendLevel * 100);
  delay(20);
  Display.writeNum("BlendOffset", BlendOffset);
  delay(20);
  Display.writeNum("NBLevel", NBLevel);
  delay(20);
  Display.writeNum("AMcochannel", AM_Cochannel);
  delay(20);
  Display.writeNum("AMnb", AM_NBLevel);
  delay(20);
  Display.writeNum("AMatt", AM_att);
  delay(20);
  Display.writeNum("am", am);
  delay(20);
  Display.writeNum("fm", fm);
  delay(20);
  Display.writeNum("uhf1", uhf1);
  delay(20);
  Display.writeNum("uhf2", uhf2);
  delay(20);
  Display.writeNum("uhf3", uhf3);
  delay(20);
  Display.writeNum("uhf4", uhf4);
  delay(20);
  Display.writeNum("coaxmode", coaxmode);  
  delay(20);
  Display.writeNum("fmsi", fmsi);  
  delay(20);
  Display.writeNum("scopeview", scopeview);
  delay(20);
  Display.writeNum("fmsiattack", fmsi_attack);  
  delay(20);
  Display.writeNum("fmsirelease", fmsi_release);
  delay(20);
  Display.writeNum("b1sens", fmsi_11);
  delay(20);
  Display.writeNum("b1bias", fmsi_12);  
  delay(20);
  Display.writeNum("b2sens", fmsi_21);
  delay(20);
  Display.writeNum("b2bias", fmsi_22);
  delay(20);
  Display.writeNum("b3sens", fmsi_31);
  delay(20);
  Display.writeNum("b3bias", fmsi_32);  
  delay(20);
  Display.writeNum("b4sens", fmsi_41);
  delay(20);
  Display.writeNum("b4bias", fmsi_42);
  delay(20);  
}

void doEEpromWrite() {
  if (store == true) {
    change++;
  }

  if (change > 200 && store == true) {
    detachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A));
    detachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B));
    detachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON));
    EEPROM.writeUInt(0, frequency0);
    EEPROM.writeUInt(83, frequency1);
    EEPROM.writeUInt(87, frequency2);
    EEPROM.writeUInt(91, frequency3);
    EEPROM.writeUInt(95, frequency4);
    EEPROM.writeUInt(158, frequency5);
    EEPROM.writeByte(227, memory_pos);
    EEPROM.writeByte(228, tunemode);
    EEPROM.commit();
    store = false;
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A),  encoderISR,       CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B),  encoderISR,       CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON), encoderButtonISR, FALLING);
  }
}

void Seek(bool mode) {
  radio.setMute();
  if (mode == false) {
    TuneDown();
  } else {
    TuneUp();
  }
  delay(50);
  ShowFreq();
  if (XDRGTK == true) {
    if (band == 0) {
      Serial.print("T" + String(frequency0 * 10) + "\n");
    } else {
      Serial.print("T" + String(frequency5) + "\n");
    }
  }

  if (XDRGTKTCP == true) {
    if (band == 0) {
      RemoteClient.print("T" + String(frequency0 * 10) + "\n");
    } else {
      RemoteClient.print("T" + String(frequency5) + "\n");
    }
  }

  radio.getStatus(SStatus, USN, WAM, OStatus, BW, MStatus);

  if ((USN < 200) && (WAM < 230) && (OStatus < 80 && OStatus > -80) && (Squelch < SStatus || Squelch == 920)) {
    seek = false;
    radio.setUnMute();
    store = true;
  } else {
    seek = true;
  }
}

void TuneUp() {
  unsigned int temp;
  if (stepsize == 0) {
    if (band == 5) {
      if (frequency5 < 1998) {
        temp = 9;
        frequency5 = (frequency5 /9) * 9;  
      } else {
        temp = 5;
        frequency5 = (frequency5 /5) * 5;
      }
    } else {
      temp = 5;
    }
  }
  if (stepsize == 1) {
    temp = 1;
  }
  if (stepsize == 2) {
    temp = 10;
  }
  if (stepsize == 3) {
    temp = 100;
  }
  if (stepsize == 4) {
    temp = 1000;
  }
  
  if (band == 0) {
    frequency0 += temp;
    if (frequency0 >= (HighEdgeSet0 * 100) + 1) {
      frequency0 = LowEdgeSet0 * 100;
    }
    radio.SetFreq(frequency0);
  }
  if (band == 1) {
    frequency1 += temp;
    if (frequency1 >= (HighEdgeSet1 * 100) + 1) {
      frequency1 = LowEdgeSet1 * 100;
    }
    Frontend.SetFreq(frequency1 - (IF * 100));
  }
  if (band == 2) {
    frequency2 += temp;
    if (frequency2 >= (HighEdgeSet2 * 100) + 1) {
      frequency2 = LowEdgeSet2 * 100;
    }
    Frontend.SetFreq(frequency2 - (IF * 100));
  }
  if (band == 3) {
    frequency3 += temp;
    if (frequency3 >= (HighEdgeSet3 * 100) + 1) {
      frequency3 = LowEdgeSet3 * 100;
    }
    Frontend.SetFreq(frequency3 - (IF * 100));
  }
  if (band == 4) {
    frequency4 += temp;
    if (frequency4 >= (HighEdgeSet4 * 100) + 1) {
      frequency4 = LowEdgeSet4 * 100;
    }
    Frontend.SetFreq(frequency4 - (IF * 100));
  }
  if (band == 5) {
    frequency5 += temp;
    if (frequency5 > 27000) {
      frequency5 = 144;
    }
    radio.SetFreqAM(frequency5);
  }
  radio.clearRDS();
}

void TuneDown() {
  unsigned int temp;
  if (stepsize == 0) {
    if (band == 5) {
      if (frequency5 <= 2000) {
        if (frequency5 == 2000) {
          frequency5 = 1998;
          temp = 0;
        } else {
          temp = 9;
          frequency5 = (frequency5 /9) * 9;
        }
      } else {
        temp = 5;
        frequency5 = (frequency5 /5) * 5;
      }
    } else {
      temp = 5;
    }
  }
  if (stepsize == 1) {
    temp = 1;
  }
  if (stepsize == 2) {
    temp = 10;
  }
  if (stepsize == 3) {
    temp = 100;
  }
  if (stepsize == 4) {
    temp = 1000;
  }
  
  if (band == 0) {
    frequency0 -= temp;
    if (frequency0 < LowEdgeSet0 * 100) {
      frequency0 = HighEdgeSet0 * 100;
    }
    radio.SetFreq(frequency0);
  }
  if (band == 1) {
    frequency1 -= temp;
    if (frequency1 < LowEdgeSet1 * 100) {
      frequency1 = HighEdgeSet1 * 100;
    }
    Frontend.SetFreq(frequency1 - (IF * 100));
  }
  if (band == 2) {
    frequency2 -= temp;
    if (frequency2 < LowEdgeSet2 * 100) {
      frequency2 = HighEdgeSet2 * 100;
    }
    Frontend.SetFreq(frequency2 - (IF * 100));
  }
  if (band == 3) {
    frequency3 -= temp;
    if (frequency3 < LowEdgeSet3 * 100) {
      frequency3 = HighEdgeSet3 * 100;
    }
    Frontend.SetFreq(frequency3 - (IF * 100));
  }
  if (band == 4) {
    frequency4 -= temp;
    if (frequency4 < LowEdgeSet4 * 100) {
      frequency4 = HighEdgeSet4 * 100;
    }
    Frontend.SetFreq(frequency4 - (IF * 100));
  }
  if (band == 5) {
    if (temp == 1000 && frequency5 <= 1440)
    {
      frequency5 = 27000;
    } else {
      frequency5 -= temp;
      if (frequency5 < 144) {
        frequency5 = 27000;
      }
    }
    radio.SetFreqAM(frequency5);
  }
  radio.clearRDS();
}

void Communication() {

  if (swSer.available()) {
    String data_str = swSer.readStringUntil('\n');
    if (data_str.substring(0, 3) == "CON") {
      btconnect = true;
      if (menu == false) {
        ShowBTstatus();
      }
    } else if (data_str.substring(0, 3) == "DIS") {
      btconnect = false;
      if (menu == false) {
        ShowBTstatus();
      }
    }

    if (data_str.charAt(11) == ':') {
      int number = data_str.substring(12, 13).toInt();
      MAC[number] = data_str.substring(data_str.indexOf(",MacAdd:0x") + 10, data_str.indexOf(",MacAdd:0x") + 22);
      NAME[number] = data_str.substring(data_str.indexOf(",Name:") + 6);
    }

    if (btsetup == true) {
      Display.writeStr("bt1.txt", NAME[1]);
      Display.writeStr("bt2.txt", NAME[2]);
      Display.writeStr("bt3.txt", NAME[3]);
      Display.writeStr("bt4.txt", NAME[4]);
      Display.writeStr("bt5.txt", NAME[5]);
      Display.writeStr("bt6.txt", NAME[6]);
      int number = Display.readNumber("bt");
      if (number > 0) {
        if (number == 7) {
          swSer.print("AT+DELVMLINK");
          delay(250);
          swSer.print("AT+ADDLINKADD=0xffffffffffff");
          delay(250);
          swSer.print("AT+REST");
          Display.writeStr("page 15");
          delay(100);
        } else if (number < 7) {
          swSer.print("AT+ADDLINKADD=0x" + MAC[number]);
          delay(100);
          Display.writeNum("bt", 0);
          delay(100);
          Display.writeStr("page 14");
          delay(100);
        }
      }
    }
  }

  if (menu == false) {
    if (manfreq == false && spec == false) {
      if (stationlist == 1 && wifienable == 2) {
        int packetSize = Udp.parsePacket();
        if (packetSize) {
          char packetBuffer[packetSize];
          Udp.read(packetBuffer, packetSize);
          Udp.endPacket();
          packet = String(packetBuffer);
          if (packetBuffer == "from=StationList;freq=?;bandwidth=?") {
            ShowFreq();
            Udp.beginPacket(remoteip, 9030);
            Udp.print("from=TEF tuner " + String(VERSION) + ";Bandwidth=");
            Udp.print(BW * 1000);
            Udp.endPacket();
          } else {
            int symPos = packet.indexOf("freq=");
            String stlfreq = packet.substring(symPos + 5, packetSize);
            if ((stlfreq.toInt()) / 10000 > 6500 && (stlfreq.toInt()) / 10000 < 10800) {
              if (band != 0) {
                band = 0;
                RF(band);
              }
              frequency0 = (stlfreq.toInt()) / 10000;
              radio.SetFreq(frequency0);
            }
            if ((stlfreq.toInt()) / 1000 > 144 && (stlfreq.toInt()) / 1000 < 27000) {
              if (band != 5) {
                band = 5;
                RF(band);
              }
              frequency5 = (stlfreq.toInt()) / 1000;
              radio.SetFreqAM(frequency5);
            }
            radio.clearRDS();
            ShowFreq();
            store = true;
          }
        }
      }

      if (Server.hasClient())
      {
        if (RemoteClient.connected())
        {
          Server.available().stop();
        } else {
          wificonnect = true;
          RemoteClient = Server.available();
          passwordcrypt();
          RemoteClient.print(saltkey + "\n");
          Display.writeNum("tm2.en", 1);
        }
      } else {
        if (Server.hasClient())
        {
          Server.available().stop();
        }
      }

      if (wificonnect == true && !RemoteClient.connected()) {
        wificonnect = false;
        RDSSpy = false;
        XDRGTKTCP = false;
        Display.writeNum("tm2.en", 0);
        Display.writeStr("vis 47,1");
      }


      if (XDRGTKTCP == false && wificonnect == true && RemoteClient.available()) {
        String data_str = RemoteClient.readStringUntil('\n');
        int data = data_str.toInt();
        Serial.println(data_str);
        if (data_str.length() > 30 && data_str.equals(cryptedpassword))
        {
          radio.setFMABandw();
          if (band != 0) {
            band = 0;
            RF(band);
          }
          XDRGTKTCP = true;
          RemoteClient.print("o1,0\n");
          store = true;
        } else if (RDSSpy == false && XDRGTKTCP == false && data_str.length() < 5 && data_str == ("*R?F"))
        {
          RDSSpy = true;
          RDSSpyReset = false;
        } else if (RDSSpy == true) {
          int symPos = data_str.indexOf("*F");
          if (symPos >= 5) {
            String freq = data_str.substring(0, symPos);
            freq = freq.substring(0, freq.length() - 1);
            frequency0 = freq.toInt();
            radio.SetFreq(frequency0);
            radio.clearRDS();
            if (band != 0) {
              band = 0;
              RF(0);
            }
            ShowFreq();
            store = true;
          }
        } else {
          RemoteClient.print("a0\n");
        }
      }

      if (XDRGTK == false && Serial.available())
      {
        String data_str = Serial.readStringUntil('\n');
        int data = data_str.toInt();
        if (data_str.length() > 1 && data_str == ("*D*R?F"))
        {
          USBstatus = true;
          RDSSpy = true;
          RDSSpyReset = false;
        }
        int symPos = data_str.indexOf("*F");
        if (symPos >= 5) {
          String freq = data_str.substring(0, symPos);
          freq = freq.substring(0, freq.length() - 1);
          frequency0 = freq.toInt();
          radio.SetFreq(frequency0);
          if (band != 0) {
            band = 0;
            RF(0);
          }
          ShowFreq();
          store = true;
        }
        if (data_str.charAt(0) == 'x') {
          radio.setFMABandw();
          if (band != 0) {
            band = 0;
            RF(band);
          }
          Serial.print("OK\nT" + String(frequency0 * 10) + "\n");
          store = true;
          USBstatus = true;
          XDRGTK = true;
        }
      }

      if (XDRGTK == true) {
        XDRGTKRoutine();
      }
      if (XDRGTKTCP == true) {
        XDRGTKRoutineTCP();
      }
    }
  }
}
  
void XDRGTKRoutine() {
  if (Serial.available() > 0)
  {
    buff[buff_pos] = Serial.read();
    if (buff[buff_pos] != '\n' && buff_pos != 16 - 1)
    {
      buff_pos++;
    } else {
      buff[buff_pos] = 0;
      buff_pos = 0;
      switch (buff[0])
      {
        case 'A':
          int AGC;
          AGC = atol(buff + 1);
          Serial.print("A" + String(AGC) + "\n");
          radio.setAGC(AGC);
          break;

        case 'C':
          byte scanmethod;
          scanmethod = atol(buff + 1);
          if (scanmethod == 1) {
            Serial.print("C1\n");
            direction = false;
            Seek(direction);
            ShowFreq();
          }
          if (scanmethod == 2) {
            Serial.print("C2\n");
            direction = true;
            Seek(direction);
            ShowFreq();
          }
          Serial.print("C0\n");
          break;

        case 'N':
          doStereoToggle();
          break;

        case 'D':
          byte XDRdemp;
          XDRdemp = atol(buff + 1);
          Serial.print("D" + String(XDRdemp) + "\n");
          if (XDRdemp == 0) {
            demp = 1;
          } else if (XDRdemp == 1) {
            demp = 2;
          } else if (XDRdemp == 2) {
            demp = 3;
          }
          radio.setDeemphasis(demp);
          break;

        case 'F':
          XDRBWset = atol(buff + 1);
          Serial.print("F" + String(XDRBWset) + "\n");
          if (XDRBWset < 0) {
            XDRBWsetold = XDRBWset;
            BWset = 0;
          } else if (XDRBWset == 0) {
            BWset = 1;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 1) {
            BWset = 2;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 2) {
            BWset = 3;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 3) {
            BWset = 4;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 4) {
            BWset = 5;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 5) {
            BWset = 6;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 6) {
            BWset = 7;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 7) {
            BWset = 8;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 8) {
            BWset = 9;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 9) {
            BWset = 10;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 10) {
            BWset = 11;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 11) {
            BWset = 12;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 12) {
            BWset = 13;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 13) {
            BWset = 14;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 14) {
            BWset = 15;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 15) {
            BWset = 16;
            XDRBWsetold = XDRBWset;
          } else {
            XDRBWset = XDRBWsetold;
          }
          doBW();
          break;

        case 'G':
          // Twee vinkjes
          break;

        case 'M':
          byte XDRband;
          XDRband = atol(buff + 1);
          if (XDRband == 0) {
            band = 0;
            RF(band);
            Serial.print("M0\n");
            Serial.print("T" + String(frequency0 * 10) + "\n");
          } else {
            band = 5;
            RF(band);
            Serial.print("M1\n");
            Serial.print("T" + String(frequency5) + "\n");
          }
          ShowFreq();
          break;

        case 'T':
          unsigned int XDRfreq;
          XDRfreq = atoi(buff + 1);
          if (XDRfreq > 143 && XDRfreq < 27001) {
            frequency5 = XDRfreq;
            if (band != 5) {
              band = 5;
              RF(band);
              Serial.print("M1\n");
            } else {
              radio.SetFreqAM(frequency5);
            }
          } else if (XDRfreq > 64999 && XDRfreq < 108001){
            frequency0 = XDRfreq / 10;
            if (band != 0) {
              band = 0;
              RF(band);
              Serial.print("M0\n");
            } else {
              radio.SetFreq(frequency0);
            }
          }
          if (band == 0) {
            Serial.print("T" + String(frequency0*10) + "\n");
          } else {
            Serial.print("T" + String(frequency5) + "\n");
          }
            radio.clearRDS();
            ShowFreq();
          break;

        case 'Q':
          Squelch = atoi(buff + 1);
          if (Squelch == -1) {
            Serial.print("Q-1\n");
          } else {
            Squelch *= 10;
            Serial.print("Q\n");
            Serial.print(Squelch / 10);
          }
          break;

        case 'S':
          radio.setMute();
          if (buff[1] == 'a')
          {
            XDRscanner_start = (atol(buff + 2) + 5) / 10;
          } else if (buff[1] == 'b')
          {
            XDRscanner_end = (atol(buff + 2) + 5) / 10;

          } else if (buff[1] == 'c')
          {
            XDRscanner_step = (atol(buff + 2) + 5) / 10;
          } else if (buff[1] == 'f')
          {
            XDRscanner_filter = atol(buff + 2);
          } else if (XDRscanner_start > 0 && XDRscanner_end > 0 && XDRscanner_step > 0 && XDRscanner_filter >= 0)
          {
            XDRscanner_old = frequency0;
            if (XDRscanner_filter < 0) {
              BWset = 1;
            } else if (XDRscanner_filter == 1) {
              BWset = 2;
            } else if (XDRscanner_filter == 2) {
              BWset = 3;
            } else if (XDRscanner_filter == 3) {
              BWset = 4;
            } else if (XDRscanner_filter == 4) {
              BWset = 5;
            } else if (XDRscanner_filter == 5) {
              BWset = 6;
            } else if (XDRscanner_filter == 6) {
              BWset = 7;
            } else if (XDRscanner_filter == 7) {
              BWset = 8;
            } else if (XDRscanner_filter == 8) {
              BWset = 9;
            } else if (XDRscanner_filter == 9) {
              BWset = 10;
            } else if (XDRscanner_filter == 10) {
              BWset = 11;
            } else if (XDRscanner_filter == 11) {
              BWset = 12;
            } else if (XDRscanner_filter == 12) {
              BWset = 13;
            } else if (XDRscanner_filter == 13) {
              BWset = 14;
            } else if (XDRscanner_filter == 14) {
              BWset = 15;
            } else if (XDRscanner_filter == 15) {
              BWset = 16;
            }
            doBW();
            Serial.print('U');
            for (XDRfreq_scan = XDRscanner_start; XDRfreq_scan <= XDRscanner_end; XDRfreq_scan += XDRscanner_step)
            {
              radio.SetFreq(XDRfreq_scan);
              Display.writeNum("freq.val", XDRfreq_scan);
              Serial.print(XDRfreq_scan * 10, DEC);
              Serial.print('=');
              delay(10);
              radio.getStatus(SStatus, USN, WAM, OStatus, BW, MStatus);
              Serial.print((SStatus / 10) + 10, DEC);
              Serial.print(',');
            }
            Serial.print('\n');
          }
          radio.SetFreq(XDRscanner_old);
          ShowFreq();
          BWset = XDRBWsetold;
          doBW();
          radio.setUnMute();
          break;

        case 'Y':
          VolSet = atoi(buff + 1);
          if (VolSet == 0) {
            radio.setMute();
            XDRMute = true;
            SQ = true;
          } else {
            radio.setVolume((VolSet - 70) / 10);
            XDRMute = false;
          }
          Serial.print("Y" + String(VolSet) + "\n");
          break;

        case 'X':
          USBstatus = false;
          if (XDRGTK == true) {
            XDRGTK = false;
          }
          if (XDRGTKTCP == true) {
            XDRGTKTCP = false;
          }
          store = true;
          XDRMute = false;
          break;

        case 'Z':
          byte iMSEQX;
          iMSEQX = atol(buff + 1);
          Serial.print("Z" + String(iMSEQX) + "\n");
          if (iMSEQX == 0) {
            iMSset = 0;
            EQset = 0;
            iMSEQ = 1;
          }
          if (iMSEQX == 1) {
            iMSset = 0;
            EQset = 1;
            iMSEQ = 3;
          }
          if (iMSEQX == 2) {
            iMSset = 1;
            EQset = 0;
            iMSEQ = 4;
          }
          if (iMSEQX == 3) {
            iMSset = 1;
            EQset = 1;
            iMSEQ = 2;
          }
          doFilter();
          break;
      }
    }
  }
  if (XDRGTK == true) {
    if (band == 5) {
      Serial.print("Sm");
    } else {
      if (StereoToggle == false) {
        Serial.print("SS");
      } else if (Stereostatus == true) {
        Serial.print("Ss");
      } else {
        Serial.print("Sm");
      }
    }
    Serial.print(String(((SStatus * 100) + 10875) / 1000) + "," + String(WAM / 10) + "," + String(SNR) + "\n");
  }
}

void XDRGTKRoutineTCP() {
  if (RemoteClient.available() > 0)
  {
    buff[buff_pos] = RemoteClient.read();
    if (buff[buff_pos] != '\n' && buff_pos != 16 - 1)
    {
      buff_pos++;
    } else {
      buff[buff_pos] = 0;
      buff_pos = 0;

      switch (buff[0])
      {
        case 'A':
          int AGC;
          AGC = atol(buff + 1);
          RemoteClient.print("A" + String(AGC) + "\n");
          radio.setAGC(AGC);
          break;

        case 'C':
          byte scanmethod;
          scanmethod = atol(buff + 1);
          if (scanmethod == 1) {
            RemoteClient.print("C1\n");
            direction = false;
            Seek(direction);
            ShowFreq();
          }
          if (scanmethod == 2) {
            RemoteClient.print("C2\n");
            direction = true;
            Seek(direction);
            ShowFreq();
          }
          RemoteClient.print("C0\n");
          break;

        case 'N':
          doStereoToggle();
          break;

        case 'D':
          byte XDRdemp;
          XDRdemp = atol(buff + 1);
          RemoteClient.print("D" + String(XDRdemp) + "\n");
          if (XDRdemp == 0) {
            demp = 1;
          } else if (XDRdemp == 1) {
            demp = 2;
          } else if (XDRdemp == 2) {
            demp = 3;
          }
          radio.setDeemphasis(demp);
          break;

        case 'F':
          XDRBWset = atol(buff + 1);
          RemoteClient.print("F" + String(XDRBWset) + "\n");
          if (XDRBWset < 0) {
            XDRBWsetold = XDRBWset;
            BWset = 0;
          } else if (XDRBWset == 0) {
            BWset = 1;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 1) {
            BWset = 2;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 2) {
            BWset = 3;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 3) {
            BWset = 4;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 4) {
            BWset = 5;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 5) {
            BWset = 6;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 6) {
            BWset = 7;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 7) {
            BWset = 8;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 8) {
            BWset = 9;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 9) {
            BWset = 10;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 10) {
            BWset = 11;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 11) {
            BWset = 12;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 12) {
            BWset = 13;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 13) {
            BWset = 14;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 14) {
            BWset = 15;
            XDRBWsetold = XDRBWset;
          } else if (XDRBWset == 15) {
            BWset = 16;
            XDRBWsetold = XDRBWset;
          } else {
            XDRBWset = XDRBWsetold;
          }
          doBW();
          break;

        case 'G':
          // Twee vinkjes
          break;

        case 'M':
          byte XDRband;
          XDRband = atol(buff + 1);
          if (XDRband == 0) {
            band = 0;
            RF(band);
            RemoteClient.print("M0\n");
            RemoteClient.print("T" + String(frequency0 * 10) + "\n");
          } else {
            band = 5;
            RF(band);
            RemoteClient.print("M1\n");
            RemoteClient.print("T" + String(frequency5) + "\n");
          }
          ShowFreq();
          break;

        case 'T':
          unsigned int XDRfreq;
          XDRfreq = atoi(buff + 1);
          if (XDRfreq > 143 && XDRfreq < 27001) {
            frequency5 = XDRfreq;
            if (band != 5) {
              band = 5;
              RF(band);
              RemoteClient.print("M1\n");
            } else {
              radio.SetFreqAM(frequency5);
            }
          } else if (XDRfreq > 64999 && XDRfreq < 108001){
            frequency0 = XDRfreq / 10;
            if (band != 0) {
              band = 0;
              RF(band);
              RemoteClient.print("M0\n");
            } else {
              radio.SetFreq(frequency0);
            }
          }
          if (band == 0) {
            RemoteClient.print("T" + String(frequency0*10) + "\n");
          } else {
            RemoteClient.print("T" + String(frequency5) + "\n");
          }
            radio.clearRDS();
            ShowFreq();
          break;
          
        case 'Q':
          Squelch = atoi(buff + 1);
          if (Squelch == -1) {
            RemoteClient.print("Q-1\n");
          } else {
            Squelch *= 10;
            RemoteClient.print("Q\n");
            RemoteClient.print(Squelch / 10);
          }
          break;

        case 'S':
          radio.setMute();
          if (buff[1] == 'a')
          {
            XDRscanner_start = (atol(buff + 2) + 5) / 10;
          } else if (buff[1] == 'b')
          {
            XDRscanner_end = (atol(buff + 2) + 5) / 10;

          } else if (buff[1] == 'c')
          {
            XDRscanner_step = (atol(buff + 2) + 5) / 10;
          } else if (buff[1] == 'f')
          {
            XDRscanner_filter = atol(buff + 2);
          } else if (XDRscanner_start > 0 && XDRscanner_end > 0 && XDRscanner_step > 0 && XDRscanner_filter >= 0)
          {
            XDRscanner_old = frequency0;
            if (XDRscanner_filter < 0) {
              BWset = 1;
            } else if (XDRscanner_filter == 1) {
              BWset = 2;
            } else if (XDRscanner_filter == 2) {
              BWset = 3;
            } else if (XDRscanner_filter == 3) {
              BWset = 4;
            } else if (XDRscanner_filter == 4) {
              BWset = 5;
            } else if (XDRscanner_filter == 5) {
              BWset = 6;
            } else if (XDRscanner_filter == 6) {
              BWset = 7;
            } else if (XDRscanner_filter == 7) {
              BWset = 8;
            } else if (XDRscanner_filter == 8) {
              BWset = 9;
            } else if (XDRscanner_filter == 9) {
              BWset = 10;
            } else if (XDRscanner_filter == 10) {
              BWset = 11;
            } else if (XDRscanner_filter == 11) {
              BWset = 12;
            } else if (XDRscanner_filter == 12) {
              BWset = 13;
            } else if (XDRscanner_filter == 13) {
              BWset = 14;
            } else if (XDRscanner_filter == 14) {
              BWset = 15;
            } else if (XDRscanner_filter == 15) {
              BWset = 16;
            }
            doBW();
            RemoteClient.print('U');
            for (XDRfreq_scan = XDRscanner_start; XDRfreq_scan <= XDRscanner_end; XDRfreq_scan += XDRscanner_step)
            {
              radio.SetFreq(XDRfreq_scan);
              Display.writeNum("freq.val", XDRfreq_scan);
              RemoteClient.print(XDRfreq_scan * 10, DEC);
              RemoteClient.print('=');
              delay(10);
              radio.getStatus(SStatus, USN, WAM, OStatus, BW, MStatus);
              RemoteClient.print((SStatus / 10) + 10, DEC);
              RemoteClient.print(',');
            }
            RemoteClient.print('\n');
          }
          radio.SetFreq(XDRscanner_old);
          ShowFreq();
          BWset = XDRBWsetold;
          doBW();
          radio.setUnMute();
          break;

        case 'Y':
          VolSet = atoi(buff + 1);
          if (VolSet == 0) {
            radio.setMute();
            XDRMute = true;
            SQ = true;
          } else {
            radio.setVolume((VolSet - 70) / 10);
            XDRMute = false;
          }
          RemoteClient.print("Y" + String(VolSet) + "\n");
          break;

        case 'x':
          RemoteClient.print("OK\nT" + String(frequency0 * 10) + "\n");
          store = true;
          break;

        case 'X':
          XDRGTKTCP = false;
          store = true;
          XDRMute = false;
          break;

        case 'Z':
          byte iMSEQX;
          iMSEQX = atol(buff + 1);
          RemoteClient.print("Z" + String(iMSEQX) + "\n");
          if (iMSEQX == 0) {
            iMSset = 0;
            EQset = 0;
            iMSEQ = 1;
          }
          if (iMSEQX == 1) {
            iMSset = 0;
            EQset = 1;
            iMSEQ = 3;
          }
          if (iMSEQX == 2) {
            iMSset = 1;
            EQset = 0;
            iMSEQ = 4;
          }
          if (iMSEQX == 3) {
            iMSset = 1;
            EQset = 1;
            iMSEQ = 2;
          }
          doFilter();
          break;
      }
    }
  }
  if (XDRGTKTCP == true) {
    if (band == 5) {
      RemoteClient.print("Sm");
    } else {
      if (StereoToggle == false) {
        RemoteClient.print("SS");
      } else if (Stereostatus == true) {
        RemoteClient.print("Ss");
      } else {
        RemoteClient.print("Sm");
      }
    }
    RemoteClient.print(String(((SStatus * 100) + 10875) / 1000) + "," + String(WAM / 10) + "," + String(SNR) + "\n");
  }
}

void passwordcrypt() {
  int generated = 0;
  while (generated < 16)
  {
    byte randomValue = random(0, 26);
    char letter = randomValue + 'a';
    if (randomValue > 26)
      letter = (randomValue - 26) ;
    saltkey.setCharAt(generated, letter);
    generated ++;
  }
  salt = saltkey + password;
  cryptedpassword = String(sha1(salt));
}

void passwordgenerator() {
  int generated = 0;
  while (generated < 5)
  {
    byte randomValue = random(0, 26);
    char letter = randomValue + 'A';
    if (randomValue > 26)
      letter = (randomValue - 26) ;
    password.setCharAt(generated, letter);
    generated ++;
  }
}

void SetTunerPatch() {
    radio.init(102);
    uint16_t device;
    uint16_t hw;
    uint16_t sw;
    radio.getIdentification(device, hw, sw);
    TEF = highByte(hw) * 100 + highByte(sw);
    if (TEF == 0) {
      Display.writeStr("version.txt", "Tuner not detected");
    } else {
      Display.writeStr("version.txt", String("Tuner version set: v") + String(TEF));
    }
    Display.writeStr("version2.txt", "Please restart tuner");
    EEPROM.writeByte(183, TEF);
    EEPROM.commit();
    while (true);
    for (;;);
}

void FactoryDefaults() {
    swSer.print("AT+DELVMLINK");
    delay(250);
    swSer.print("AT+ADDLINKADD=0xffffffffffff");
    passwordgenerator();
    EEPROM.writeUInt(0, 9630);
    EEPROM.writeInt(4, 0);
    EEPROM.writeByte(8, 1);
    EEPROM.writeByte(9, 1);
    EEPROM.writeByte(10, 50);
    EEPROM.writeByte(11, 0);
    EEPROM.writeInt(12, 84);
    EEPROM.writeInt(16, 108);
    EEPROM.writeInt(20, 50);
    EEPROM.writeInt(24, 0);
    EEPROM.writeInt(28, 0);
    EEPROM.writeInt(32, 70);
    EEPROM.writeInt(36, 0);
    EEPROM.writeByte(40, 1);
    EEPROM.writeByte(41, 207);
    EEPROM.writeByte(42, 0);
    EEPROM.writeByte(43, 1);
    EEPROM.writeByte(44, 1);
    EEPROM.writeByte(45, 1);
    EEPROM.writeByte(46, 0);
    EEPROM.writeByte(47, 80);
    EEPROM.writeByte(48, 1);
    EEPROM.writeByte(49, 1);
    EEPROM.writeInt(51, 430);
    EEPROM.writeInt(55, 440);
    EEPROM.writeInt(59, 870);
    EEPROM.writeInt(63, 880);
    EEPROM.writeInt(67, 1290);
    EEPROM.writeInt(71, 1310);
    EEPROM.writeInt(75, 2320);
    EEPROM.writeInt(79, 2400);
    EEPROM.writeUInt(83, 43500);
    EEPROM.writeUInt(87, 87400);
    EEPROM.writeUInt(91, 129900);
    EEPROM.writeUInt(95, 232500);
    EEPROM.writeUInt(99, 8800);
    EEPROM.writeUInt(103, 10800);
    EEPROM.writeUInt(107, 56);
    EEPROM.writeByte(113, 0);
    EEPROM.writeInt(117, 0);
    EEPROM.writeInt(121, 0);
    EEPROM.writeInt(125, 0);
    EEPROM.writeInt(129, 0);
    EEPROM.writeString(134, password);
    EEPROM.writeByte(140, 192);
    EEPROM.writeByte(141, 168);
    EEPROM.writeByte(142, 178);
    EEPROM.writeByte(143, 150);
    EEPROM.writeInt(144, 70);
    EEPROM.writeInt(148, 0);
    EEPROM.writeInt(152, 0);
    EEPROM.writeByte(157, 1);
    EEPROM.writeUInt(158, 828);
    EEPROM.writeByte(163, 0);
    EEPROM.writeInt(164, 100);
    EEPROM.writeInt(168, 0);
    EEPROM.writeInt(172, 0);
    EEPROM.writeByte(176, 1);
    EEPROM.writeByte(177, 1);
    EEPROM.writeByte(178, 1);
    EEPROM.writeByte(179, 1);
    EEPROM.writeByte(180, 1);
    EEPROM.writeByte(181, 1);
    EEPROM.writeByte(182, 1);
    EEPROM.writeByte(183, 0);
    EEPROM.writeByte(184, 0);
    EEPROM.writeByte(185, 2);
    EEPROM.writeByte(186, 0);
    EEPROM.writeUInt(187, 50);
    EEPROM.writeUInt(191, 50);
    EEPROM.writeUInt(195, 100);
    EEPROM.writeUInt(199, 175);
    EEPROM.writeUInt(203, 100);
    EEPROM.writeUInt(207, 215);
    EEPROM.writeUInt(211, 100);
    EEPROM.writeUInt(215, 225);
    EEPROM.writeUInt(219, 100);
    EEPROM.writeUInt(223, 225);
    EEPROM.writeByte(227, 0);
    EEPROM.writeByte(228, 0);
    uint8_t i;
    for (i = 0; i < 30; i++) {
      EEPROM.writeUInt((i*4) + 230, 8750);
    }
    EEPROM.commit();
}
