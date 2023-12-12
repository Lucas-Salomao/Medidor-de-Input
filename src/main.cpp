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
#include <LcdMenu.h>

#include "Button2.h"

#include "pwm16.h"

#include <MCP4725.h>
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

#define PWM_RESOLUTION 10
#define PWMSEC 9
#define PWMMILI 10

RotaryEncoder *encoder = nullptr;
LcdMenu menu(LCD_ROWS, LCD_COLS); // Construct the LcdMenu

Button2 button;

MCP4725 MCPSec(0x61);  // conversor digital-analógico MCP4725
MCP4725 MCPMili(0x60); // conversor digital-analógico MCP4725

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

char str_segundos[20];
char str_milisegundos[20];
char str_tensao_segundos[20];
char str_tensao_milisegundos[20];

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

void update1(void);
void update2(void);
void update3(void);
void update4(void);

double mapeamento(double x, double in_min, double in_max, double out_min, double out_max);

MAIN_MENU(
    ITEM_SUBMENU("Monitorar", monitorMenu),
    ITEM_SUBMENU("Configuracao", settingsMenu));

SUB_MENU(settingsMenu, mainMenu,
         ITEM_INPUT("Voltas", config.voltas, inputCallbackVoltas),
         ITEM_INPUT("Tempo Max", config.tempoMax, inputCallbackTempoMax),
         ITEM_INPUT("PWM-Resol", config.pwm, inputCallbackPWM),
         ITEM_COMMAND("Apagar Memo", EEPROM_Clear),
         ITEM_COMMAND("Voltar", menu_back));
SUB_MENU(monitorMenu, mainMenu,
         //ITEM_BASIC(str_segundos),
         //ITEM_BASIC(str_milisegundos),
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
  cli();
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
    // time_to_voltage();
    elapsed_time = 0;
    volta_atual = 1;
    update1();
    update2();
    //update3();
    //update4();
  }

  Serial.print("Volta fim da interrupcao:");
  Serial.println(volta_atual, DEC);
  Serial.println("");
  sei();
}
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Iniciando Medidor de Input");

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
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), count_time, FALLING);

  menu.setupLcdWithMenu(0x27, mainMenu);

  current_time = millis();
  last_time = 0;
  elapsed_time = 0;

  button.begin(BUTTON_PIN);
  button.setClickHandler(handler);
  // button.setLongClickHandler(handler);       // this will only be called upon release
  button.setLongClickDetectedHandler(handler); // this will only be called upon detection
  button.setDoubleClickHandler(handler);
  button.setTripleClickHandler(handler);

  setupPWM16(PWM_RESOLUTION);
  MCPSec.begin();
  MCPMili.begin();

  load_configuration();
} // setup()

// Read the current position of the encoder and print out when changed.
void loop()
{
  encoder->tick(); // just call tick() to check the state.

  read_encoder();

  button.loop();

} // loop ()

void read_encoder(void)
{
  int newPos = encoder->getPosition();
  if (pos != newPos)
  {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
  }
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
    Serial.print("double ");
    break;
  case triple_click:
    funcReset();
    Serial.print("triple ");
    break;
  case long_click:
    menu.backspace();
    Serial.print("long");
    break;
  }
  Serial.print("click");
  Serial.print(" (");
  Serial.print(btn.getNumberOfClicks());
  Serial.println(")");
}

void time_to_pwm()
{
  cli();
  uint16_t pwmSec, pwmMili = 0;

  pwmSec = mapeamento(tempo_segundos, 0, tempo_maximo, 0, pow(2, pwm_resolution));
  pwmMili = mapeamento(tempo_milisegundos, 0, 999, 0, pow(2, pwm_resolution));

  analogWrite16(PWMSEC, pwmSec);
  analogWrite16(PWMMILI, pwmMili);

  voltageSec = (5.0 / pow(2, pwm_resolution)) * (pwmSec * 1.0);
  voltageMili = (5.0 / pow(2, pwm_resolution)) * (pwmMili * 1.0);

  // Serial.println(pwmSec);
  // Serial.println(pwmMili);
  // Serial.println(voltageSec);
  // Serial.println(voltageMili);
}

void time_to_voltage()
{
  cli();

  voltageSec = mapeamento(tempo_segundos, 0, tempo_maximo, 0, 5);
  voltageMili = mapeamento(tempo_milisegundos, 0, 999, 0, 5);

  MCPSec.setVoltage(voltageSec);
  MCPMili.setVoltage(voltageMili);
}

void test_pwm(void)
{
  for (int pwm = 0; pwm <= pow(2, pwm_resolution); pwm = pwm + (pow(2, pwm_resolution) / 8))
  {
    analogWrite16(PWMSEC, pwm);
    analogWrite16(PWMMILI, pwm);
  }
}

void test_dac(void)
{
  for (float t = 0.0; t <= 5.0; t = t + 0.5)
  {
    MCPMili.setVoltage(t);
    MCPSec.setVoltage(t);
  }
}

void inputCallbackVoltas(char *value)
{
  strcpy(config.voltas, value);
  save_configuration();
}

void inputCallbackTempoMax(char *value)
{
  strcpy(config.tempoMax, value);
  save_configuration();
}

void inputCallbackPWM(char *value)
{
  strcpy(config.pwm, value);
  save_configuration();
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
  cli();
  String tempstr;
  tempstr.reserve(20);
  int uint_tensao=0;
  int prt_inteira=0;
  int prt_fracionaria=0;

  uint_tensao=(uint16_t)(voltageMili*100);
  //Serial.println(uint_tensao);
  prt_inteira=uint_tensao/100;
  prt_fracionaria=uint_tensao%100;
  tempstr=String("V-MILI:")+String(prt_inteira)+String(",")+String(prt_fracionaria)+String("V");
  Serial.println(tempstr);
  strcpy(str_tensao_milisegundos,tempstr.c_str());
}

void update2(void)
{
  cli();
  String tempstr;
  tempstr.reserve(20);
  int uint_tensao=0;
  int prt_inteira=0;
  int prt_fracionaria=0;

  uint_tensao=(uint16_t)(voltageSec*100);
  //Serial.println(uint_tensao);
  prt_inteira=uint_tensao/100;
  prt_fracionaria=uint_tensao%100;
  tempstr=String("V-SEG:")+String(prt_inteira)+String(",")+String(prt_fracionaria)+String("V");
  Serial.println(tempstr);
  strcpy(str_tensao_segundos,tempstr.c_str());
}

void update3(void)
{
  cli();
  String tempstr;
  tempstr.reserve(20);
  tempstr=String("T-SEG:")+String(tempo_segundos)+String("s");
  Serial.println(tempstr);
  strcpy(str_segundos,tempstr.c_str());
}

void update4(void)
{
  cli();
  String tempstr;
  tempstr.reserve(20);
  tempstr=String("T-MILI:")+String(tempo_milisegundos)+String("ms");
  Serial.println(tempstr);
  strcpy(str_milisegundos,tempstr.c_str());
}

double mapeamento(double x, double in_min, double in_max, double out_min, double out_max)
{
  double result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (int)(result + 0.5); // arredonda para o número inteiro mais próximo
}