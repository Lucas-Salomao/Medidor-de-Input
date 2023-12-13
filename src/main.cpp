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
};
Config config;
char charset[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.'};
uint8_t charsetPosition;

unsigned long int current_time = 0;
unsigned long int last_time = 0;
unsigned long int elapsed_time = 0;
unsigned long int tempo_total = 0;
int volta_atual = 0;
int volta_configurada = 1;
int tempo_maximo = 600;
int tempo_segundos = 0;
int tempo_milisegundos = 0;
float voltageSec = 0.0;
float voltageMili = 0.0;
int pwm_resolution = 16;

char str_segundos[16];
char str_milisegundos[16];
char str_tensao_segundos[16];
char str_tensao_milisegundos[16];

uint16_t teste = 0;

void menu_back(void);
void read_encoder(void);

void handler(Button2 &btn);

void (*funcReset)() = 0;

void time_to_pwm();
void time_to_voltage();

void inputCallbackVoltas(char *value);
void inputCallbackTempoMax(char *value);
void inputCallbackPWM(char *value);

void save_configuration(void);
void load_configuration(void);

void EEPROM_Clear(void);

void update_display(void);

void test_pwm(void);
void test_pwm2(void);
void test_dac(void);

void rotina_teste(uint16_t isOn);

double mapeamento(double x, double in_min, double in_max, double out_min, double out_max);

MAIN_MENU(
    ITEM_SUBMENU("Monitorar", monitorMenu),
    ITEM_SUBMENU("Configuracao", settingsMenu));

SUB_MENU(settingsMenu, mainMenu,
         ITEM_INPUT("Voltas", config.voltas, inputCallbackVoltas),
         ITEM_INPUT("Tempo Max", config.tempoMax, inputCallbackTempoMax),
         ITEM_INPUT("PWM-Resol", config.pwm, inputCallbackPWM),
         ITEM_COMMAND("Apagar Memo", EEPROM_Clear),
         ITEM_TOGGLE("Teste", "ON", "OFF", rotina_teste),
         ITEM_COMMAND("Voltar", menu_back));
SUB_MENU(monitorMenu, mainMenu,
         ITEM_BASIC(str_segundos),
         ITEM_BASIC(str_milisegundos),
         ITEM_BASIC(str_tensao_segundos),
         ITEM_BASIC(str_tensao_milisegundos),
         ITEM_COMMAND("Voltar", menu_back));

void menu_back(void)
{
  menu.back();
}

void checkPosition(void)
{
  encoder->tick(); // just call tick() to check the state.
}

void count_time(void)
{
  Serial.println("Interrupcao Externa");

  Serial.print("Volta inicio da interrupcao:");
  Serial.println(volta_atual, DEC);
  char msg_time[60];
  // Atualiza o tempo atual
  current_time = millis();
  sprintf(msg_time, "Tempo desde a inicializacao: %lu ms", current_time);
  Serial.println(msg_time);

  if (last_time == 0)
  {
    last_time = current_time;
  }
  // Calcula o tempo decorrido desde a última interrupção
  elapsed_time += current_time - last_time;
  sprintf(msg_time, "Tempo entre pulsos: %lu ms", elapsed_time);
  Serial.println(msg_time);

  // Atualiza o tempo da interrupção anterior
  last_time = current_time;

  volta_atual++;
  // Se a quantidade de interrupções for atingida, imprime o tempo acumulado na serial
  if (volta_atual > volta_configurada)
  {
    tempo_segundos = elapsed_time / 1000;
    tempo_milisegundos = elapsed_time % 1000;
    sprintf(msg_time, "Tempo total: %lu ms", elapsed_time);
    Serial.println(msg_time);
    time_to_pwm();
    time_to_voltage();
    elapsed_time = 0;
    volta_atual = 1;
    update_display();
  }

  Serial.print("Volta fim da interrupcao:");
  Serial.println(volta_atual, DEC);
  Serial.println("");
  sei();
}
void setup()
{
  Serial.begin(115200);
  while (!Serial);
  //Serial.println("Iniciando Medidor de Input");

  // setup the rotary encoder functionality

  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);

  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  // attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
  //pinMode(PIN_IN1,INPUT_PULLUP);
  //pinMode(PIN_IN2,INPUT_PULLUP);
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), count_time, FALLING);

  menu.setupLcdWithMenu(0x27, mainMenu);
  charsetPosition = 0;

  current_time = millis();
  last_time = 0;
  elapsed_time = 0;

  button.begin(BUTTON_PIN);
  button.setClickHandler(handler);
  // button.setLongClickHandler(handler);       // this will only be called upon release
  //button.setLongClickDetectedHandler(handler); // this will only be called upon detection
  button.setDoubleClickHandler(handler);
  button.setTripleClickHandler(handler);

  load_configuration();
  // setupPWM16(pwm_resolution);
  // DACSec.begin(0x61);
  // DACMili.begin(0x60);
}

void loop()
{
  //encoder->tick(); // just call tick() to check the state.
  read_encoder();
  button.loop();
  //test_dac();
  //test_pwm();
}

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
    // Serial.print("double ");
    break;
  case triple_click:
    funcReset();
    // Serial.print("triple ");
    break;
  case long_click:
    menu.backspace();
    // Serial.print("long");
    break;
  }
  // Serial.print("click");
  // Serial.print(" (");
  // Serial.print(btn.getNumberOfClicks());
  // Serial.println(")");
}

void time_to_pwm()
{
  uint16_t pwmSec, pwmMili = 0;

  uint16_t bits = (uint16_t)pow(2, pwm_resolution);
  uint16_t fundo_escala = bits - 1;
  //Serial.println(tempo_maximo);
  //Serial.println(fundo_escala);

  pwmSec = (uint16_t)map(tempo_segundos, 0, tempo_maximo, 0, fundo_escala);
  pwmMili = (uint16_t)map(tempo_milisegundos, 0, 999, 0, fundo_escala);

  // setupPWM16(pwm_resolution);
  analogWrite16(PWMSEC, pwmSec);
  analogWrite16(PWMMILI, pwmMili);

  voltageSec = (10.0 / bits) * (pwmSec * 1.0);
  voltageMili = (10.0 / bits) * (pwmMili * 1.0);

  // Serial.println(pwmSec);
  // Serial.println(pwmMili);
  // Serial.println(voltageSec);
  // Serial.println(voltageMili);
}

void time_to_voltage()
{
  // voltageSec = mapeamento(tempo_segundos, 0, tempo_maximo, 0, 5);
  // voltageMili = mapeamento(tempo_milisegundos, 0, 999, 0, 5);

  uint16_t t = map(tempo_segundos, 0, tempo_maximo, 0, 4095);
  DACSec.setVoltage(t, false);
  t = map(tempo_milisegundos, 0, 999, 0, 4095);
  DACMili.setVoltage(t, false);
}

void test_pwm(void)
{
  uint16_t bits = (uint16_t)pow(2, pwm_resolution);
  setupPWM16(pwm_resolution);
  for (uint16_t pwm = 0; pwm <= bits; pwm = pwm + bits / 8)
  {
    if (pwm > 0)
      pwm--;
    analogWrite16(PWMSEC, pwm);
    analogWrite16(PWMMILI, pwm);
    delay(5000);
  }
}
void test_pwm2(void)
{
  setupPWM16(12);
  analogWrite16(PWMSEC, 2047);
  analogWrite16(PWMMILI, 2047);

  analogWrite16(PWMSEC, 4095);
  analogWrite16(PWMMILI, 4095);
  
}

void test_dac(void)
{
  for (uint16_t t = 0; t <= 4096; t = t + (4096 / 8))
  {
    if (t > 0)
      t--;
    DACMili.setVoltage(t, false);
    DACSec.setVoltage(t, false);
    delay(5000);
  }
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
  pwm_resolution = atoi(value);
  setupPWM16(pwm_resolution);
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
    strlcpy(config.pwm, "16", sizeof("16"));
    save_configuration();
    load_configuration();
  }
  else
  {
    Serial.println(F("Successful to read configuration!"));
    strlcpy(config.voltas, doc["voltas"], sizeof(config.voltas));
    strlcpy(config.tempoMax, doc["tempoMax"], sizeof(config.tempoMax));
    strlcpy(config.pwm, doc["pwm"], sizeof(config.pwm));
    volta_configurada = int(doc["voltas"]);
    pwm_resolution = float(doc["pwm"]);
    tempo_maximo = int(doc["tempoMax"]);
    Serial.print("Voltas configurada: ");
    Serial.println(volta_configurada);
    Serial.print("Resolucao PWM: ");
    Serial.println(pwm_resolution);
    Serial.print("Tempo maximo configurado: ");
    Serial.println(tempo_maximo);
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
  funcReset();
}

void update1(void)
{
  String tempstr;
  tempstr.reserve(20);
  int uint_tensao = 0;
  int prt_inteira = 0;
  int prt_fracionaria = 0;

  uint_tensao = (uint16_t)(voltageMili * 100);
  // Serial.println(uint_tensao);
  prt_inteira = uint_tensao / 100;
  prt_fracionaria = uint_tensao % 100;
  tempstr = String("V-MILI:") + String(prt_inteira) + String(",") + String(prt_fracionaria) + String("V");
  // Serial.println(tempstr);
  strcpy(str_tensao_milisegundos, tempstr.c_str());
}

void update2(void)
{
  String tempstr;
  tempstr.reserve(20);
  int uint_tensao = 0;
  int prt_inteira = 0;
  int prt_fracionaria = 0;

  uint_tensao = (uint16_t)(voltageSec * 100);
  // Serial.println(uint_tensao);
  prt_inteira = uint_tensao / 100;
  prt_fracionaria = uint_tensao % 100;
  tempstr = String("V-SEG:") + String(prt_inteira) + String(",") + String(prt_fracionaria) + String("V");
  // Serial.println(tempstr);
  strcpy(str_tensao_segundos, tempstr.c_str());
}

void update3(void)
{
  String tempstr;
  tempstr.reserve(20);
  tempstr = String("T-SEG:") + String(tempo_segundos) + String("s");
  // Serial.println(tempstr);
  strcpy(str_segundos, tempstr.c_str());
}

void update4(void)
{
  String tempstr;
  tempstr.reserve(20);
  tempstr = String("T-MILI:") + String(tempo_milisegundos) + String("ms");
  // Serial.println(tempstr);
  strcpy(str_milisegundos, tempstr.c_str());
}

double mapeamento(double x, double in_min, double in_max, double out_min, double out_max)
{
  double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (int)(result + 0.5); // arredonda para o número inteiro mais próximo
}

void update_display(void)
{
  int num1 = tempo_segundos;
  char charBuf1[16];
  itoa(num1, charBuf1, 10);
  //Serial.println(charBuf1);

  int num2 = tempo_milisegundos;
  char charBuf2[16];
  itoa(num2, charBuf2, 10);
  //Serial.println(charBuf2);

  float num3 = voltageSec;
  char charBuf3[16];
  dtostrf(num3, 4, 2, charBuf3);
  //Serial.println(charBuf3);

  float num4 = voltageMili;
  char charBuf4[16];
  dtostrf(num4, 4, 2, charBuf4);
  //Serial.println(charBuf4);

  strcpy(str_segundos, charBuf1);
  strcpy(str_milisegundos, charBuf2);
  strcpy(str_tensao_segundos, charBuf3);
  strcpy(str_tensao_milisegundos, charBuf4);
}

void rotina_teste(uint16_t isOn)
{
  teste = isOn;
}

ISR(PCINT1_vect) {
  encoder->tick(); // just call tick() to check the state.
  //Serial.println(encoder->getPosition());
}