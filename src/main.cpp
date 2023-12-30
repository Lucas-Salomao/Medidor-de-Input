/*
Copyright (c) Lucas Salomao <lucastadeusalomao@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

PINAGEM CONECTOR SENSOR M12
Marrom 2 SINAL 1 MILISEGUNDOS
Amarelo 1 GND 1
Vermelho 4 SINAL 2 SEGUNDOS
Laranja 3 GND 2

*/

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <ItemInput.h>
#include <ItemSubMenu.h>
#include <ItemCommand.h>
#include <ItemToggle.h>
#include <LcdMenu.h>
#include "Button2.h"
#include "pwm16.h"
#include <Adafruit_MCP4725.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <EEPROM.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>

#define PIN_IN1 A2
#define PIN_IN2 A3
#define LCD_ROWS 2
#define LCD_COLS 16
#define LCD_ADDR 0x27
#define CHARSET_SIZE 11
#define CONFIG_ADDR 0x00
#define BUTTON_PIN A1
#define EEPROM_SIZE 1024
#define PWMSEC 9
#define PWMMILI 10

RotaryEncoder *encoder = nullptr;
LcdMenu menu(LCD_ROWS, LCD_COLS); // Construct the LcdMenu
Button2 button;
Adafruit_MCP4725 DACSec;
Adafruit_MCP4725 DACMili;
Adafruit_ADS1X15 ADS1X15;
// Adafruit_ADS1115 ADS1X15;

extern MenuItem *settingsMenu[];
extern MenuItem *monitorMenu[];

int valor = 0;
int newPos = 0;
int pos = 0;
struct Config
{
  char voltas[3];
  char tempoMax[4];
  char pwm[3];
  char delay[6];
};
Config config;
const char charset[] PROGMEM = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.'};
uint8_t charsetPosition;
volatile unsigned long int current_time = 0;
volatile unsigned long int last_time = 0;
volatile unsigned long int elapsed_time = 0;
volatile unsigned long int tempo_total = 0;
volatile uint8_t volta_atual = 0;
volatile uint8_t volta_configurada = 1;
volatile uint16_t tempo_maximo = 600;
volatile uint16_t tempo_segundos = 0;
volatile uint16_t tempo_milisegundos = 0;
volatile float voltageSec = 0.0;
volatile float voltageMili = 0.0;
volatile uint16_t pwmBits = 12;
char str_segundos[16];
char str_milisegundos[16];
char str_tensao_segundos[16];
char str_tensao_milisegundos[16];
volatile uint16_t teste = 0;
volatile unsigned long tempo_atual_dac;
volatile unsigned long tempo_anterior_dac;
volatile unsigned long tempo_atraso_teste = 1000;
volatile uint16_t dac_voltage = 0;
volatile unsigned long tempo_atual_pwm;
volatile unsigned long tempo_anterior_pwm;
volatile uint16_t pwm_bits = 0;
volatile uint8_t atualiza_tensao = 0;
volatile uint16_t adc_sec, adc_mili = 0;
volatile float tensao_ads_sec, tensao_ads_mili = 0.0;
volatile int16_t corretor = 0;
volatile uint16_t precisao_bits_dac = 1;
volatile uint16_t erroDAC=30;

void (*funcReset)() = 0;
void menu_back(void);
void read_encoder(void);
void handler(Button2 &btn);
void time_to_pwm();
void time_to_voltage();
void inputCallbackVoltas(char *value);
void inputCallbackTempoMax(char *value);
void inputCallbackPWM(char *value);
void inputCallbackDelay(char *value);
void save_configuration(void);
void load_configuration(void);
void EEPROM_Clear(void);
void update_display(void);
void test_output(void);
void rotina_teste(uint16_t isOn);
long mapeamento(long x, long in_min, long in_max, long out_min, long out_max);
void read_ADS(void);
void corrige_DAC(void);

MAIN_MENU(
    ITEM_SUBMENU("Monitorar", monitorMenu),
    ITEM_SUBMENU("Configuracao", settingsMenu));

SUB_MENU(settingsMenu, mainMenu,
         ITEM_INPUT("Voltas", config.voltas, inputCallbackVoltas),
         ITEM_INPUT("Tempo Max", config.tempoMax, inputCallbackTempoMax),
         ITEM_INPUT("PWM-Resol", config.pwm, inputCallbackPWM),
         ITEM_COMMAND("Apagar Memo", EEPROM_Clear),
         ITEM_TOGGLE("Teste", "ON", "OFF", rotina_teste),
         ITEM_INPUT("Delay", config.delay, inputCallbackDelay),
         ITEM_COMMAND("Voltar", menu_back));
SUB_MENU(monitorMenu, mainMenu,
         ITEM_BASIC(str_segundos),
         ITEM_BASIC(str_milisegundos),
         ITEM_BASIC(str_tensao_segundos),
         ITEM_BASIC(str_tensao_milisegundos),
         ITEM_COMMAND("Voltar", menu_back));

void read_encoder(void)
{
  int newPos = encoder->getPosition();
  // if (pos != newPos)
  // {
  //   Serial.print("pos:");
  //   Serial.print(newPos);
  //   Serial.print(" dir:");
  //   Serial.println((int)(encoder->getDirection()));
  // }
  if (newPos > pos)
  {
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition = (charsetPosition + 1) % CHARSET_SIZE;
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.down();
    pos = newPos;
  }
  if (newPos < pos)
  {
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition = constrain(charsetPosition - 1, 0, CHARSET_SIZE);
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.up();
    pos = newPos;
  }
}

void handler(Button2 &btn)
{
  switch (btn.getType())
  {
  case single_click:
    menu.type(charset[charsetPosition]); // Works only in edit mode
    menu.enter();
    break;
  case double_click:
    menu.back();
    menu.setCursorPosition(0);
    break;
  case triple_click:
    funcReset();
    break;
  case long_click:
    menu.backspace();
    break;
  case empty:
    break;
  }
}

void time_to_pwm()
{
  uint16_t pwmSec, pwmMili = 0;

  uint16_t resolution = (uint16_t)pow(2, pwmBits);
  uint16_t fundo_escala = resolution - 1;

  pwmSec = (uint16_t)mapeamento(tempo_segundos, 0, tempo_maximo, 0, fundo_escala);
  pwmMili = (uint16_t)mapeamento(tempo_milisegundos, 0, 999, 0, fundo_escala);

  // setupPWM16(pwm_resolution);
  analogWrite16(PWMSEC, pwmSec);
  analogWrite16(PWMMILI, pwmMili);

  voltageSec = (10.0 / resolution) * pwmSec;
  voltageMili = (10.0 / resolution) * pwmMili;
}

void time_to_voltage()
{
  char str[50];
  uint16_t t = mapeamento(tempo_segundos, 0, tempo_maximo, 50, 4050);
  DACSec.setVoltage(t + erroDAC+ corretor, false);
  voltageSec = t * (5.0 / 4096);
  t = mapeamento(tempo_milisegundos, 0, 999, 50, 4050);
  DACMili.setVoltage(t + erroDAC+ corretor, false);
  voltageMili = t * (5.0 / 4096);

  //sprintf(str, "DAC-Tensao segundos:%2.4f V", voltageSec);
  Serial.print("DAC-Tensao segundos:");
  Serial.println(voltageSec,6);
  //sprintf(str, "DAC-Tensao segundos:%2.4f V", voltageMili);
  Serial.print("DAC-Tensao milisegundos:");
  Serial.println(voltageMili,6);
}

void test_output(void)
{
  uint16_t resolution = (uint16_t)pow(2, pwmBits);
  tempo_atraso_teste = 1000;

  analogWrite16(PWMSEC, (resolution / 8) * 0);
  analogWrite16(PWMMILI, (resolution / 8) * 0);
  DACMili.setVoltage(0, false);
  DACSec.setVoltage(0, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 1) - 1);
  analogWrite16(PWMMILI, ((resolution / 8) * 1) - 1);
  DACMili.setVoltage(511, false);
  DACSec.setVoltage(511, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 2) - 1);
  analogWrite16(PWMMILI, ((resolution / 8) * 2) - 1);
  DACMili.setVoltage(1023, false);
  DACSec.setVoltage(1023, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 3) - 1);
  analogWrite16(PWMMILI, ((resolution / 8) * 3) - 1);
  DACMili.setVoltage(1535, false);
  DACSec.setVoltage(1535, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 4) - 1);
  analogWrite16(PWMMILI, ((resolution / 8) * 4) - 1);
  DACMili.setVoltage(2047, false);
  DACSec.setVoltage(2047, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 5) - 1);
  analogWrite16(PWMMILI, ((resolution / 8) * 5) - 1);
  DACMili.setVoltage(2559, false);
  DACSec.setVoltage(2559, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 6) - 1);
  analogWrite16(PWMMILI, ((resolution / 8) * 6) - 1);
  DACMili.setVoltage(3071, false);
  DACSec.setVoltage(3071, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 7) - 1);
  analogWrite16(PWMMILI, ((resolution / 8) * 7) - 1);
  DACMili.setVoltage(3583, false);
  DACSec.setVoltage(3583, false);
  read_ADS();
  delay(tempo_atraso_teste);

  analogWrite16(PWMSEC, ((resolution / 8) * 8));
  analogWrite16(PWMMILI, ((resolution / 8) * 8));
  DACMili.setVoltage(4095, false);
  DACSec.setVoltage(4095, false);
  read_ADS();
  delay(tempo_atraso_teste);
}

void inputCallbackVoltas(char *value)
{
  strcpy(config.voltas, value);
  save_configuration();
  volta_configurada = atoi(value);
}

void inputCallbackTempoMax(char *value)
{
  strcpy(config.tempoMax, value);
  save_configuration();
  tempo_maximo = atoi(value);
}

void inputCallbackPWM(char *value)
{
  strcpy(config.pwm, value);
  save_configuration();
  pwmBits = atoi(value);
  setupPWM16(pwmBits);
  pwm_bits = 0;
}

void inputCallbackDelay(char *value)
{
  strcpy(config.delay, value);
  save_configuration();
  tempo_atraso_teste = atoi(value);
}

void menu_back(void)
{
  menu.back();
}

void save_configuration(void)
{
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<128> doc;

  // Set the values in the document
  doc["voltas"] = config.voltas;
  doc["tempoMax"] = config.tempoMax;
  doc["pwm"] = config.pwm;
  doc["delay"] = config.delay;

  EepromStream eepromStream(CONFIG_ADDR, sizeof(doc));
  serializeJson(doc, eepromStream);
  Serial.println(F("Successful to save configuration!"));
}

void load_configuration(void)
{
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<128> doc;

  EepromStream eepromStream(CONFIG_ADDR, sizeof(doc));
  DeserializationError error = deserializeJson(doc, eepromStream);
  if (error)
  {
    Serial.print(F("Failed to read file, using default configuration!\n\rError:"));
    Serial.println(error.f_str());
    strlcpy(config.voltas, "01", sizeof("01"));
    strlcpy(config.tempoMax, "600", sizeof("600"));
    strlcpy(config.pwm, "11", sizeof("11"));
    strlcpy(config.delay, "5000", sizeof("5000"));
    save_configuration();
    load_configuration();
  }
  else
  {
    Serial.println(F("Successful to read configuration!"));
    strlcpy(config.voltas, doc["voltas"], sizeof(config.voltas));
    strlcpy(config.tempoMax, doc["tempoMax"], sizeof(config.tempoMax));
    strlcpy(config.pwm, doc["pwm"], sizeof(config.pwm));
    strlcpy(config.delay, doc["delay"], sizeof(config.delay));
    volta_configurada = int(doc["voltas"]);
    pwmBits = float(doc["pwm"]);
    tempo_maximo = int(doc["tempoMax"]);
    tempo_atraso_teste = (unsigned long)(doc["delay"]);
    Serial.print("Voltas configurada: ");
    Serial.println(volta_configurada);
    Serial.print("Resolucao PWM: ");
    Serial.println(pwmBits);
    Serial.print("Tempo maximo configurado: ");
    Serial.println(tempo_maximo);
    Serial.print("Tempo atraso teste: ");
    Serial.println(tempo_atraso_teste);
  }
}

void EEPROM_Clear(void)
{
  Serial.println("Apagando memoria EEPROM");
  for (unsigned int i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.write(i, 0);
  }
  Serial.println("Memoria EEPROM apagada com sucesso!");
  delay(100);
  funcReset();
}

long mapeamento(long x, long in_min, long in_max, long out_min, long out_max)
{
  float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return round(result); // arredonda para o número inteiro mais próximo
}

void update_display(void)
{
  int num1 = tempo_segundos;
  char charBuf1[16];
  itoa(num1, charBuf1, 10);
  // Serial.println(charBuf1);

  int num2 = tempo_milisegundos;
  char charBuf2[16];
  itoa(num2, charBuf2, 10);
  // Serial.println(charBuf2);

  float num3 = voltageSec;
  char charBuf3[16];
  dtostrf(num3, 4, 2, charBuf3);
  // Serial.println(charBuf3);

  float num4 = voltageMili;
  char charBuf4[16];
  dtostrf(num4, 4, 2, charBuf4);
  // Serial.println(charBuf4);

  strcpy(str_segundos, charBuf1);
  strcpy(str_milisegundos, charBuf2);
  strcpy(str_tensao_segundos, charBuf3);
  strcpy(str_tensao_milisegundos, charBuf4);
}

void rotina_teste(uint16_t isOn)
{
  teste = isOn;
  if (isOn == 0)
  {
    pwm_bits = 0;
    dac_voltage = 0;
  }
}

void read_ADS()
{
  char str[50];
  //adc_sec = ADS1X15.readADC_SingleEnded(0);
  //adc_mili = ADS1X15.readADC_SingleEnded(1);
  tensao_ads_sec = ADS1X15.computeVolts(ADS1X15.readADC_SingleEnded(0));
  tensao_ads_mili = ADS1X15.computeVolts(ADS1X15.readADC_SingleEnded(1));

  //sprintf(str, "ADC-Tensao segundos:%2.4f V", tensao_ads_sec);
  Serial.print("ADC-Tensao segundos:");
  Serial.println(tensao_ads_sec,6);
  //sprintf(str, "ADC-Tensao milisegundos:%2.4f V", tensao_ads_mili);
  Serial.print("ADC-Tensao milisegundos:");
  Serial.println(tensao_ads_mili,6);
}

void corrige_DAC()
{
  if (((voltageSec - tensao_ads_sec) > 0) & ((voltageSec - tensao_ads_sec) > (precisao_bits_dac * (6.144 / 4096))))
  {
    corretor++;
    atualiza_tensao = 1;
    return;
  }
  if (((voltageSec - tensao_ads_sec) > 0) & ((voltageSec - tensao_ads_sec) <= (precisao_bits_dac * (6.144 / 4096))))
  {
    atualiza_tensao = 0;
    Serial.println("Leitua final apos correcao");
    time_to_voltage();
    read_ADS();
    Serial.print("Corretor:");
    Serial.println(corretor);
    Serial.println();
    return;
  }
  if (((voltageSec - tensao_ads_sec) < 0) & ((voltageSec - tensao_ads_sec) < (-1 * (precisao_bits_dac * (6.144 / 4096)))))
  {
    corretor--;
    atualiza_tensao = 1;
    return;
  }
  if (((voltageSec - tensao_ads_sec) < 0) & ((voltageSec - tensao_ads_sec) > (-1 * (precisao_bits_dac * (6.144 / 4096)))))
  {
    atualiza_tensao = 0;
    Serial.println("Leitura final apos correcao");
    time_to_voltage();
    read_ADS();
    Serial.print("Corretor:");
    Serial.println(corretor);
    return;
  }
}

void checkPosition(void)
{
  encoder->tick(); // just call tick() to check the state.
}

void count_time(void)
{
  if (current_time != last_time)
  {
    char msg_time[60];
    if (last_time == 0)
    {
      last_time = current_time;
      volta_atual=0;
    }
    // Calcula o tempo decorrido desde a última interrupção
    elapsed_time += current_time - last_time;
    // Atualiza o tempo da interrupção anterior
    last_time = current_time;
    volta_atual++;
    // Se a quantidade de interrupções for atingida, imprime o tempo acumulado na serial
    if (volta_atual > volta_configurada)
    {
      tempo_segundos = elapsed_time / 1000;
      tempo_milisegundos = elapsed_time % 1000;
      sprintf(msg_time, "Tempo total decorrido: %lu ms", elapsed_time);
      Serial.println(msg_time);
      if (teste == 0)
      {
        atualiza_tensao = 1;
      }
      elapsed_time = 0;
      volta_atual = 1;
    }
  }
}

void atualiza_tempo(void)
{
  current_time = millis();
}

ISR(PCINT1_vect)
{
  checkPosition();
}

void setup()
{
  Serial.begin(1000000);
  Serial.println(F("*********************************************************"));
  Serial.println(F("Iniciando Medidor de Input Rinnai V1.0"));
  Serial.println(F("Desenvolvido por DK Solutions"));
  Serial.println(F("Engenheiro Responsavel:Lucas Salomao"));
  Serial.println(F("Contato:lucastadeusalomao@gmail.com"));
  Serial.println(F("*********************************************************"));
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // setup the rotary encoder functionality

  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  Serial.println(F("Configurando encoder rotativo"));
  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);

  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  // attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  // pinMode(PIN_IN1,INPUT_PULLUP);
  // pinMode(PIN_IN2,INPUT_PULLUP);
  PCICR |= (1 << PCIE1);                     // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11); // This enables the interrupt for pin 2 and 3 of Port C.

  Serial.println(F("Configurando interrupcao do sensor"));
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), atualiza_tempo, FALLING);

  // lcd->setCursor(5,0);
  // lcd->print("Rinnai");
  // lcd->setCursor(0,1);
  // lcd->print("Medidor de Input");

  Serial.println(F("Configurando LCD"));
  menu.setupLcdWithMenu(0x27, mainMenu);
  charsetPosition = 0;

  Serial.println(F("Configurando botao"));
  button.begin(BUTTON_PIN);
  button.setClickHandler(handler);
  // button.setLongClickHandler(handler);       // this will only be called upon release
  // button.setLongClickDetectedHandler(handler); // this will only be called upon detection
  button.setDoubleClickHandler(handler);
  button.setTripleClickHandler(handler);

  Serial.println(F("Lendo configuracoes na EEPROM"));
  load_configuration();

  // Serial.println(F("Configurando saida PWM"));
  // setupPWM16(pwmBits);
  // analogWrite16(PWMSEC, 0);
  // analogWrite16(PWMMILI, 0);

  Serial.println(F("Configurando DAC"));
  DACSec.begin(0x60);
  DACMili.begin(0x61);
  DACSec.setVoltage(0, false);
  DACMili.setVoltage(0, false);

  if (!ADS1X15.begin())
  {
    ADS1X15.setGain(GAIN_TWOTHIRDS);
    ADS1X15.setDataRate(RATE_ADS1015_1600SPS);
    Serial.println(F("Falha ao iniciar ADC"));
    funcReset();
  }
  else
    Serial.println(F("ADC inicializado!"));
  corretor = 0;

  current_time = 0;
  last_time = 0;
  elapsed_time = 0;
  volta_atual = 0;
}

void loop()
{
  count_time();
  //read_encoder();
  // button.loop();
  // checkPosition();
  if (teste == 1)
  {
    test_output();
  }
  if (atualiza_tensao == 1)
  {
    time_to_voltage();
    read_ADS();
    corrige_DAC();
  }
}