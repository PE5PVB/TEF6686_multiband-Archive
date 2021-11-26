#define TEF6686_h

#include "Arduino.h"
#include "Tuner_Drv_Lithio.h"
#include "Tuner_Interface.h"

enum RDS_GROUPS
{
  RDS_GROUP_0A,  RDS_GROUP_0B,  RDS_GROUP_1A,  RDS_GROUP_1B,  RDS_GROUP_2A,  RDS_GROUP_2B,  RDS_GROUP_3A,  RDS_GROUP_3B,
  RDS_GROUP_4A,  RDS_GROUP_4B,  RDS_GROUP_5A,  RDS_GROUP_5B,  RDS_GROUP_6A,  RDS_GROUP_6B,  RDS_GROUP_7A,  RDS_GROUP_7B,
  RDS_GROUP_8A,  RDS_GROUP_8B,  RDS_GROUP_9A,  RDS_GROUP_9B,  RDS_GROUP_10A, RDS_GROUP_10B, RDS_GROUP_11A, RDS_GROUP_11B,
  RDS_GROUP_12A, RDS_GROUP_12B, RDS_GROUP_13A, RDS_GROUP_13B, RDS_GROUP_14A, RDS_GROUP_14B, RDS_GROUP_15A, RDS_GROUP_15B
};

typedef struct _rds_
{
  char     stationName[9];
  char     stationText[65];
  byte     stationTypeCode;
  char     stationType[17];
  char     musicTitle[48];
  char     musicArtist[48];
  char     stationHost[48];
  char     stationEvent[48];

  uint16_t hours, minutes, days, months, years, offsetplusmin, stationID = 0, rdsA, rdsB, rdsC, rdsD, rdsError, errors = 0;
  int8_t offset;
  uint8_t stationTextOffset = 0;
  char picode[5];
  int ECC;
  bool hasMusicTitle = false;
  bool hasMusicArtist = false;
  bool hasStationHost = false;
  bool hasStationEvent = false;
  bool hasRDSplus = false;
  bool hasRDS = false;
  bool hasPS = false;
  bool hasRT = false;
  bool hasTP = false;
  bool hasTA = false;
  bool hasEON = false;
  bool hasAF = false;
  bool MS = false;
  bool hasPTY = false;
  bool hasCT = false;
  bool readyRT = false;
  bool afclear = false;
} rds_;

typedef struct _af_
{
  uint16_t  frequency;
} af_;


class TEF6686 {
  public:
    af_  af[50];
    rds_ rds;
    bool readRDS(void);
    void SetFreq(uint16_t frequency);
    void SetFreqAM(uint16_t frequency);
    bool getStatus(int16_t &level, uint16_t &USN, uint16_t &WAM, int16_t &offset, uint16_t &bandwidth, uint16_t &modulation);
    bool getStatusAM(int16_t &level, uint16_t &noise, uint16_t &cochannel, int16_t &offset, uint16_t &bandwidth, uint16_t &modulation);
    void setMono(bool mono);
    bool getStereoStatus();
    void init();
    void clearRDS();
    void power(bool mode);
    void setAGC(uint8_t agc);
    void setiMS(bool mph);
    void setEQ(bool eq);
    void setDeemphasis(uint8_t timeconstant);
    void setAudio(uint8_t audio);
    void setFMABandw();
    void setFMBandw(uint16_t bandwidth);
    void setAMBandw(uint16_t bandwidth);
    void setHighCutLevel(uint16_t limit);
    void setHighCutOffset(uint8_t start);
    void setStHiBlendLevel(uint16_t limit);
    void setStHiBlendOffset(uint8_t start);
    void setNoiseBlanker(uint16_t start);
    void setMute();
    void setOffset(int8_t offset);
    void setStereoLevel(uint8_t start);
    void setUnMute();
    void setVolume(int8_t volume);
    uint8_t af_counter;

  private:
    uint8_t ascii_converter   (uint8_t src);
    uint16_t rdsTimeOut  = 32768;
    uint8_t ps_process;
    uint8_t rt_process;
    char ps_buffer[9];
    char ps_buffer2[9];
    char rt_buffer[65];
    char rt_buffer2[65];
    bool useRTPlus = true;
    bool checkDouble (uint16_t value);
};
