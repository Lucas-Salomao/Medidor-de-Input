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

*/

#include <ItemInput.h>
#include <ItemSubMenu.h>
#include <ItemCommand.h>
#include <LcdMenu.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <EEPROM.h>
#include <RotaryEncoder.h>
#include <OneButton.h>
#include "pwm16.h"
#include <MCP4725.h>

#define LCD_ROWS 2
#define LCD_COLS 16
#define LCD_ADDR 0x27
#define CHARSET_SIZE 11
#define CONFIG_ADDR 0x00
#define swEncoder 7
#define ckEncoder A2
#define dtEncoder A3
#define sensor 2
#define PWM_RESOLUTION 10
#define PWMSEC 9
#define PWMMILI 10

// Configure keyboard keys (ASCII)
#define UP 56       // NUMPAD 8
#define DOWN 50     // NUMPAD 2
#define LEFT 52     // NUMPAD 4
#define RIGHT 54    // NUMPAD 6
#define ENTER 53    // NUMPAD 5
#define BACK 55     // NUMPAD 7
#define BACKSPACE 8 // BACKSPACE
#define CLEAR 46    // NUMPAD .

// Pinos de ligacao do encoder
RotaryEncoder encoder(ckEncoder, dtEncoder);
// Variavel para o botao do encoder
int valor = 0;
int newPos = 0;
int pos = 0;
// Setup a new OneButton on pin A1 for SW encoder.
OneButton button1(A1, true);

struct Config
{
  char voltas[10];
  char calibracao[10];
};
Config config;
StringStream stream;

extern MenuItem *settingsMenu[];
// Declare the call back function
void inputCallbackVoltas(char *value);
void inputCallbackTensao(char *value);
void menu_back(void);
// Create your charset
char charset[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.'};
// Active index of the charset
uint8_t charsetPosition;
// Initialize the main menu items
MAIN_MENU(
    ITEM_BASIC("Medidor Input"),
    ITEM_SUBMENU("Configuracao", settingsMenu));

SUB_MENU(settingsMenu, mainMenu,
         ITEM_INPUT("Num Voltas", config.voltas, inputCallbackVoltas),
         ITEM_INPUT("Calib.", config.calibracao, inputCallbackTensao),
         ITEM_BASIC("Media de tempo"),
         ITEM_COMMAND("Voltar", menu_back));

// Construct the LcdMenu
LcdMenu menu(LCD_ROWS, LCD_COLS);

char Voltas[10] = "1";
char Tensao[10] = "5.00";

unsigned long int current_time = 0;
unsigned long int last_time = 0;
unsigned long int elapsed_time = 0;
int volta_atual = 0;
int volta_configurada = 1;
void count_time(void);
void time_to_pwm(unsigned long int time);
void time_to_voltage(unsigned long int time);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

void save_configuration(void);
void load_configuration(void);

/**
 * @brief Declaração dos objetos do conversor digital-analógico MCP4725
 *
 */
MCP4725 MCPSec(0x63);
MCP4725 MCPMili(0x64);

/**
 * @brief Função de inicialização do display LCD e do menu de opções
 *
 */
void init_LCD(void)
{
  // Initialize LcdMenu with the menu items
  menu.setupLcdWithMenu(LCD_ADDR, mainMenu);
  // Initialize active index to zero
  charsetPosition = 0;
}

/**
 * @brief Função para recuperar as configurações na memória EEPROM interna, através de um arquivo JSON
 *
 */
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
    Serial.println(F("Failed to read file, using default configuration"));
    Serial.println(error.f_str());
    strlcpy(config.voltas, Voltas, sizeof(config.voltas));
    strlcpy(config.calibracao, Tensao, sizeof(config.calibracao));
    save_configuration();
    load_configuration();
  }
  else
  {
    for (JsonPair pair : doc.as<JsonObject>())
    {
      Serial.print(pair.key().c_str());
      Serial.print(" = ");
      Serial.println(pair.value().as<String>().c_str());
    }
    Serial.println(F("Successful to read configuration"));
    strlcpy(config.voltas, doc["voltas"], sizeof(config.voltas));
    strlcpy(config.calibracao, doc["calibracao"], sizeof(config.calibracao));
  }

  String strVoltas = config.voltas;
  volta_configurada = strVoltas.toInt();
  Serial.print("Numero de voltas: ");
  Serial.println(volta_configurada);
}

/**
 * @brief Função para salvar as configurações na memória EEPROM interna, através de um arquivo JSON
 *
 */
void save_configuration()
{
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<128> doc;

  // Set the values in the document
  doc["voltas"] = config.voltas;
  doc["calibracao"] = config.calibracao;

  EepromStream eepromStream(CONFIG_ADDR, sizeof(doc));
  serializeJson(doc, eepromStream);
}

/**
 * @brief Função de leitura da porta serial e manipulação do menu no LCD
 *
 */
void monitora_teclado()
{
  if (!Serial.available())
    return;
  char command = Serial.read();

  switch (command)
  {
  case UP:
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition = (charsetPosition + 1) % CHARSET_SIZE;
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.up();
    break;
  case DOWN:
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition =
          constrain(charsetPosition - 1, 0, CHARSET_SIZE);
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.down();
    break;
  case LEFT:
    menu.left();
    break;
  case RIGHT:
    menu.right();
    break;
  case ENTER:                            // Press enter to go to edit mode : for ItemInput
    menu.type(charset[charsetPosition]); // Works only in edit mode
    menu.enter();
    break;
  case BACK:
    menu.back();
    break;
  case CLEAR:
    menu.clear();
    break;
  case BACKSPACE: // Remove one character from tail
    menu.backspace();
    break;
  default:
    break;
  }
}

/**
 * @brief Função para leitura do encoder rotativo KY-040 e manipulação do menu no LCD
 *
 */
void read_encoder()
{
  encoder.tick();
  int newPos = encoder.getPosition();
  if (newPos > pos)
  {
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition =
          constrain(charsetPosition - 1, 0, CHARSET_SIZE);
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.down();
    pos = newPos;
  }
  if (newPos < pos)
  {
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition = (charsetPosition + 1) % CHARSET_SIZE;
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.up();
    pos = newPos;
  }
}

// ----- button 1 callback functions

// This function will be called when the button1 was pressed 1 time (and no 2. button press followed).
void click1()
{
  menu.type(charset[charsetPosition]); // Works only in edit mode
  menu.enter();
  Serial.println("Button 1 click...");
} // click1

// This function will be called when the button1 was pressed 2 times in a short timeframe.
void doubleclick1()
{
  menu.back();
  Serial.println("Button 1 doubleclick...");
} // doubleclick1

// This function will be called once, when the button1 is pressed for a long time.
void longPressStart1()
{
  menu.backspace();
  Serial.println("Button 1 longPressStart1...");
} // longPressStart1

// This function will be called often, while the button1 is pressed for a long time.
void longPress1()
{
  Serial.println("Button 1 longPress...");
} // longPress1

// This function will be called once, when the button1 is released after beeing pressed for a long time.
void longPressStop1()
{
  Serial.println("Button 1 longPress stop");
} // longPressStop1

void multiclick()
{
  Serial.println("Button 1 multiclick");
}

void menu_back()
{
  menu.back();
}

/**
 * @brief Função que retorna o valor configurado de voltas no menu do LCD
 *
 * @param value string que representa o numero de voltas que o dispositivo deve contar
 */
void inputCallbackVoltas(char *value)
{
  strcpy(config.voltas, value);
  Serial.print(F("# "));
  Serial.println(value);
  Serial.println(config.voltas);
  Serial.println(config.calibracao);
  save_configuration();
}

/**
 * @brief Função que retorna o valor configurado de calibração no menu do LCD
 *
 * @param value string que representa o valor de calibração
 */
void inputCallbackTensao(char *value)
{
  strcpy(config.calibracao, value);
  Serial.print(F("# "));
  Serial.println(value);
  Serial.println(config.voltas);
  Serial.println(config.calibracao);
  save_configuration();
}

/**
 * @brief Função para tratar a interrupção ocasionada pelo pulso do sensor óptico. Essa função determina o tempo entre o primeiro pulso e o ultimo pulso configurado, que representa a última volta do ponteiro.
 *
 */
void count_time(void)
{
  // interrupt_time = millis();
  // char msg_time_elapsed[100];
  // sprintf(msg_time_elapsed, "Tempo decorrido desde a inicializacao: %lu ms", interrupt_time);
  // Serial.println(msg_time_elapsed);
  // Serial.print("Volta atual:");
  // Serial.println(volta_atual,DEC);

  // if (volta_atual == 1)
  // {
  //   start_time = interrupt_time;
  //   char msg_time_elapsed[100];
  //   sprintf(msg_time_elapsed, "Tempo inicial: %lu ms", start_time);
  //   Serial.println(msg_time_elapsed);
  // }

  // if (volta_atual == 2)
  // {
  //   Serial.print("Volta atual:");
  //   Serial.println(volta_atual,DEC);
  //   elapsed_time = interrupt_time - start_time;
  //   volta_atual = 1;
  //   char msg_time_elapsed[50];
  //   sprintf(msg_time_elapsed, "Tempo decorrido entre pulsos configurados: %lu ms", elapsed_time);
  //   Serial.println(msg_time_elapsed);
  // }
  // volta_atual++;

  Serial.print("Volta inicio da interrupcao:");
  Serial.println(volta_atual, DEC);
  char msg_time[60];
  // Atualiza o tempo atual
  current_time = millis();
  sprintf(msg_time, "Tempo decorrido desde a inicializacao: %lu ms", current_time);
  Serial.println(msg_time);

  if (last_time == 0)
  {
    last_time = current_time;
  }
  // Calcula o tempo decorrido desde a última interrupção
  elapsed_time += current_time - last_time;
  sprintf(msg_time, "Tempo decorrido entre pulsos: %lu ms", elapsed_time);
  Serial.println(msg_time);

  // Atualiza o tempo da interrupção anterior
  last_time = current_time;

  volta_atual++;
  // Se a quantidade de interrupções for atingida, imprime o tempo acumulado na serial
  if (volta_atual > volta_configurada)
  {
    sprintf(msg_time, "Tempo decorrido total: %lu ms", elapsed_time);
    Serial.println(msg_time);
    elapsed_time = 0;
    volta_atual = 1;
  }

  Serial.print("Volta fim da interrupcao:");
  Serial.println(volta_atual, DEC);
  Serial.println("");
}

/**
 * @brief Função para converter um tempo em milisegundos para 2 faixas de saída PWM, sendo uma saída para segundos e outra para milisegundos
 *
 * @param time tempo em milisegundos a ser convertido em 2 canais PWM
 */
void time_to_pwm(unsigned long int time)
{
  unsigned int seconds;
  unsigned int remaining_milliseconds;
  int pwmSec, pwmMili = 0;

  // Calcule os segundos
  seconds = elapsed_time / 1000;

  // Calcule os milissegundos restantes
  remaining_milliseconds = elapsed_time % 1000;

  pwmSec = seconds;
  pwmMili = remaining_milliseconds;
  map(pwmSec, 0, 9, 0, pow(2, PWM_RESOLUTION));
  map(pwmMili, 0, 999, 0, pow(2, PWM_RESOLUTION));

  analogWrite16(PWMSEC, pwmSec);
  analogWrite16(PWMMILI, pwmMili);
}

/**
 * @brief Função para converter um tempo em milisegundos para 2 faixas de tensão, sendo uma saída para segundos e outra para milisegundos
 *
 * @param time tempo em milisegundos a ser convertido em tensão
 */
void time_to_voltage(unsigned long int time)
{
  unsigned int seconds;
  unsigned int remaining_milliseconds;

  // Calcule os segundos
  seconds = elapsed_time / 1000;

  // Calcule os milissegundos restantes
  remaining_milliseconds = elapsed_time % 1000;

  float voltageSec = mapfloat(seconds, 0.0, 9.0, 0.0, 5.0);
  float voltageMili = mapfloat(remaining_milliseconds, 0.0, 999.0, 0.0, 5.0);

  MCPSec.setVoltage(voltageSec);
  MCPMili.setVoltage(voltageMili);
}

/**
 * @brief
 *
 * @param x Valor que necessita de adequação de range
 * @param in_min menor valor de entrada
 * @param in_max maior valor de entrada
 * @param out_min menor valor de saída
 * @param out_max maior valor de saída
 * @return float retorno do valor mapeado
 */
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Função de inicialização e configuração inicial do dispositivo
 *
 */
void setup()
{
  Serial.begin(115200);
  init_LCD();
  load_configuration();
  pinMode(swEncoder, INPUT);

  // link the button 1 functions.
  button1.attachClick(click1);
  button1.attachDoubleClick(doubleclick1);
  button1.attachLongPressStart(longPressStart1);
  button1.attachLongPressStop(longPressStop1);
  button1.attachDuringLongPress(longPress1);
  button1.attachMultiClick(multiclick);

  attachInterrupt(digitalPinToInterrupt(sensor), count_time, FALLING);
  current_time = millis();
  last_time = 0;
  elapsed_time = 0;
  // setupPWM16(PWM_RESOLUTION);
  // MCPSec.begin();
  // MCPMili.begin();
}

/**
 * @brief Loop principal do programa
 *
 */
void loop()
{
  // monitora_teclado();
  // read_encoder();
  // button1.tick();
  // time_to_pwm(elapsed_time);
  // time_to_voltage(elapsed_time);
}