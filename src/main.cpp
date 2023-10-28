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

#define LCD_ROWS 2
#define LCD_COLS 16
#define LCD_ADDR 0x27
#define CHARSET_SIZE 11
#define CONFIG_ADDR 0x00
#define swEncoder 7
#define ckEncoder A2
#define dtEncoder A3

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
// Setup a new OneButton on pin A1.
OneButton button1(A1, true);

extern MenuItem *settingsMenu[];

// Create your charset
char charset[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.'};
// Active index of the charset
uint8_t charsetPosition;
char Voltas[10] = "1";
char Tensao[10] = "5.00";

struct Config
{
  char voltas[10];
  char calibracao[10];
};
Config config;

// Declare the call back function
void inputCallbackVoltas(char *value);
void inputCallbackTensao(char *value);
void menu_back(void);

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

StringStream stream;

void init_LCD(void)
{
  // Initialize LcdMenu with the menu items
  menu.setupLcdWithMenu(LCD_ADDR, mainMenu);
  // Initialize active index to zero
  charsetPosition = 0;
}

void load_configuration(void)
{
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use https://arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<128> doc;

  EepromStream eepromStream(CONFIG_ADDR, sizeof(doc));
  DeserializationError error = deserializeJson(doc, eepromStream);
  for (JsonPair pair : doc.as<JsonObject>())
  {
    Serial.print(pair.key().c_str());
    Serial.print(" = ");
    Serial.println(pair.value().as<String>().c_str());
  }
  if (error)
  {
    Serial.println(F("Failed to read file, using default configuration"));
    Serial.println(error.f_str());
    strlcpy(config.voltas, Voltas, sizeof(config.voltas));
    strlcpy(config.calibracao, Tensao, sizeof(config.calibracao));
  }
  else
  {
    Serial.println(F("Successful to read configuration"));
    strlcpy(config.voltas, doc["voltas"], sizeof(config.voltas));
    strlcpy(config.calibracao, doc["calibracao"], sizeof(config.calibracao));
  }
}

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

void read_encoder()
{
  // //Verifica se o botao do encoder foi pressionado
  // valor = digitalRead(swEncoder);
  // if (valor != 1)
  // {
  //   Serial.println("Botao pressionado");
  //   while (digitalRead(swEncoder) == 0)
  //     delay(10);
  // }
  // //Le as informacoes do encoder
  // static int pos = 0;
  encoder.tick();
  // int newPos = encoder.getPosition();
  // //Se a posicao foi alterada, mostra o valor
  // //no Serial Monitor
  // if (pos != newPos) {
  //   Serial.print(newPos);
  //   Serial.println();
  //   pos = newPos;
  // }

  if (encoder.getDirection() == RotaryEncoder::Direction::CLOCKWISE)
  {
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition =
          constrain(charsetPosition - 1, 0, CHARSET_SIZE);
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.down();
  }
  if (encoder.getDirection() == RotaryEncoder::Direction::COUNTERCLOCKWISE)
  {
    if (menu.isInEditMode()) // Update the position only in edit mode
      charsetPosition = (charsetPosition + 1) % CHARSET_SIZE;
    menu.drawChar(charset[charsetPosition]); // Works only in edit mode
    menu.up();
  }
  // if (digitalRead(swEncoder) == LOW)
  // {
  //   while (digitalRead(swEncoder) == LOW)
  //     delay(10);
  //   menu.type(charset[charsetPosition]); // Works only in edit mode
  //   menu.enter();
  // }
}

// ----- button 1 callback functions

// This function will be called when the button1 was pressed 1 time (and no 2. button press followed).
void click1()
{
  menu.type(charset[charsetPosition]); // Works only in edit mode
  menu.enter();
} // click1

// This function will be called when the button1 was pressed 2 times in a short timeframe.
void doubleclick1()
{
  menu.back();
} // doubleclick1

// This function will be called once, when the button1 is pressed for a long time.
void longPressStart1()
{
  menu.backspace();
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
}

void loop()
{
  monitora_teclado();
  read_encoder();
}

void menu_back(void)
{
  menu.back();
}

void inputCallbackVoltas(char *value)
{
  strcpy(config.voltas, value);
  Serial.print(F("# "));
  Serial.println(value);
  Serial.println(config.voltas);
  Serial.println(config.calibracao);
  save_configuration();
}

void inputCallbackTensao(char *value)
{
  strcpy(config.calibracao, value);
  Serial.print(F("# "));
  Serial.println(value);
  Serial.println(config.voltas);
  Serial.println(config.calibracao);
  save_configuration();
}