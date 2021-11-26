#include<SPI.h>

class ADF4351
{
  public:
    void Power(bool OnOff);
    void Init(uint8_t LE, uint32_t Ref);
    void SetFreq(uint32_t freq_Hz);

  private:
    struct {
      uint16_t Integer;
      uint16_t Frac;
    } R0;
    struct {
      uint16_t Mod;
    } R1;
    struct {
      uint8_t	Div;
    } R4;
    uint32_t RefIn;
    uint8_t pin_LE;
    void WriteRegister(uint32_t data);
};

void ADF4351::Power(bool OnOff)
{
  if (OnOff == 0) {
    WriteRegister(0xD80005);
    WriteRegister(0x19C1C);
    WriteRegister(0x8020E42);
  } else {
    WriteRegister(0xD80005);
    WriteRegister(0x19C3C);
    WriteRegister(0x18020E42);
  }
}

void ADF4351::Init(uint8_t LE, uint32_t Ref)
{
  pin_LE = LE;
  RefIn = Ref;
  pinMode(pin_LE, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);

  WriteRegister(0x980005);
  WriteRegister(0x1943C);
  WriteRegister(0x84B3);
  WriteRegister(0x18021E42);//WriteRegister(0x18020E42);
  WriteRegister(0x800E1A9);
  WriteRegister(0x1664E18);
}

void ADF4351::SetFreq(uint32_t freq_Hz)
{
  freq_Hz = freq_Hz * 10000;
  double fPFD = RefIn / 8;

  if (freq_Hz >= 2200000000) R4.Div = 0;
  if (freq_Hz < 2200000000) R4.Div = 1;
  if (freq_Hz < 1100000000)	R4.Div = 2;
  if (freq_Hz < 550000000) R4.Div = 3;
  if (freq_Hz < 275000000) R4.Div = 4;
  if (freq_Hz < 137500000) R4.Div = 5;
  if (freq_Hz < 68750000)	R4.Div = 6;

  double VCO_Freq = freq_Hz * pow(2, R4.Div);
  R0.Integer = VCO_Freq / fPFD;
  R1.Mod = fPFD / 1000;
  R0.Frac = R1.Mod * (((float)VCO_Freq / fPFD) - (float)R0.Integer);

  WriteRegister((uint32_t)(R4.Div) << 20 | 0x81943C);
  WriteRegister((uint32_t)(R1.Mod) << 3 | 0x8008001);
  WriteRegister((uint32_t)(R0.Integer) << 15 | (uint32_t)(R0.Frac) << 3);
  WriteRegister(0xD80005);
}

void ADF4351::WriteRegister(uint32_t data)
{
  digitalWrite(pin_LE, LOW);
  for (int i = 0; i < 4 ; i++)
  {
    uint8_t dataByte = data >> (8 * (3 - i));
    SPI.transfer(dataByte);
  }
  digitalWrite(pin_LE, HIGH);
}
