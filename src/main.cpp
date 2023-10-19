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

#define LCD_ROWS 2
#define LCD_COLS 16
#define LCD_ADDR 0x27
#define CHARSET_SIZE 10

// Configure keyboard keys (ASCII)
#define UP 56        // NUMPAD 8
#define DOWN 50      // NUMPAD 2
#define LEFT 52      // NUMPAD 4
#define RIGHT 54     // NUMPAD 6
#define ENTER 53     // NUMPAD 5
#define BACK 55      // NUMPAD 7
#define BACKSPACE 8  // BACKSPACE
#define CLEAR 46     // NUMPAD .

extern MenuItem* settingsMenu[];

// Create your charset
char charset[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
// Active index of the charset
uint8_t charsetPosition;

// Declare the call back function
void inputCallback(char* value);
void menu_back(void);

// Initialize the main menu items
MAIN_MENU(
    ITEM_BASIC("Medidor Input"),
    ITEM_SUBMENU("Configuracao", settingsMenu)
);

SUB_MENU(settingsMenu, mainMenu,
    ITEM_INPUT("Num Voltas", inputCallback),
    ITEM_BASIC("Calib. tensao"),
    ITEM_BASIC("Media de tempo"),
    ITEM_COMMAND("Voltar", menu_back)
);

// Construct the LcdMenu
LcdMenu menu(LCD_ROWS, LCD_COLS);

void menu_back(void){
  menu.back();
}

void setup() {
    Serial.begin(115200);
    // Initialize LcdMenu with the menu items
    menu.setupLcdWithMenu(LCD_ADDR, mainMenu);
    // Initialize active index to zero
    charsetPosition = 0;
}

void loop() {
    if (!Serial.available()) return;
    char command = Serial.read();

     switch (command) {
        case UP:
            if (menu.isInEditMode())  // Update the position only in edit mode
                charsetPosition = (charsetPosition + 1) % CHARSET_SIZE;
            menu.drawChar(charset[charsetPosition]);  // Works only in edit mode
            menu.up();
            break;
        case DOWN:
            if (menu.isInEditMode())  // Update the position only in edit mode
                charsetPosition =
                    constrain(charsetPosition - 1, 0, CHARSET_SIZE);
            menu.drawChar(charset[charsetPosition]);  // Works only in edit mode
            menu.down();
            break;
        case LEFT:
            menu.left();
            break;
        case RIGHT:
            menu.right();
            break;
        case ENTER:  // Press enter to go to edit mode : for ItemInput
            menu.type(charset[charsetPosition]);  // Works only in edit mode
            menu.enter();
            break;
        case BACK:
            menu.back();
            break;
        case CLEAR:
            menu.clear();
            break;
        case BACKSPACE:  // Remove one character from tail
            menu.backspace();
            break;
        default:
            break;
    }
}

void inputCallback(char* value) {
    Serial.print(F("# "));
    Serial.println(value);
}