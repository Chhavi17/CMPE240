#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <TFT_22_ILI9225.h>
#include "tasks.hpp"
#include "utilities.h"
#include "io.hpp"

#define RST (uint8_t)29
#define RS (uint8_t)26
#define CS (uint8_t)31
#define BL (uint8_t)28

class LCD: public scheduler_task
{
private:
        TFT_22_ILI9225 lcd;
        bool flag = true;
        int x,y,x1,y1,x2,y2,x3,y3;
        int side;
public:
    LCD(uint8_t priority) : scheduler_task("LCD_interface", 1024*20, priority){    }

    bool init(){
        lcd.setValues(RST,RS,CS,BL);
        return true;
    }

    bool run(void *p){
        if (flag)
        {
            lcd.begin();
            flag = false;
        }

        //-----natural scene of beach-----//
        /*lcd.drawLine(88,25,92,27,COLOR_BLACK);
        lcd.drawLine(92,27,90,29,COLOR_BLACK);
        lcd.drawLine(89,25,93,27,COLOR_BLACK);
        lcd.drawLine(93,27,91,29,COLOR_BLACK);

        //lcd.drawTriangle(0,0,50,100,150,100,COLOR_WHITE);
        lcd.fillCircle(25,25,15,COLOR_GOLD);
        lcd.fillRectangle(140,219,175,0,COLOR_SKYBLUE);*/

        //----- ScreenSaver-1 -----//
        //lcd.screenSS(0);


        //trees
        //x = LCD_WIDTH/2;
        x = rand()%LCD_WIDTH;
        y = 0;
        uint32_t color = rand()%0xFFFFFFu;

            lcd.draw_tree(x,y,color);

//lcd.drawLine(LCD_WIDTH/2,0,LCD_WIDTH+10,LCD_HEIGHT-15,color);





//        //working code for squares
//        x = rand()% LCD_WIDTH;
//        y = rand()% LCD_HEIGHT;
//        side = rand()% LCD_WIDTH;
//        uint32_t color = rand()%0xFFFFFFu;
//        // x = 15;
//        // y = 15;
//        // side = LCD_WIDTH-50;
//        x1 = x+side;
//        y1 = y;
//        x2 = x1;
//        y2 = y+side;
//        x3 = x;
//        y3 = y2;
//
//        //lcd.drawRectangle(x,y,x1,y1,(rand() % 16777215));
//        if (x < LCD_WIDTH && x1 < LCD_WIDTH && x2 < LCD_WIDTH && x3 < LCD_WIDTH &&
//            y < LCD_HEIGHT && y1 < LCD_HEIGHT && y2 < LCD_HEIGHT && y3 < LCD_HEIGHT)
//        {
//            lcd.startSS1(x,y,x1,y1,x2,y2,x3,y3,color);
//        }



        if (SW.getSwitch(1))
        {
            lcd.clear();
        }

        return true;
    }
};

int main(void)
{
    //scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    scheduler_add_task(new LCD(2));

    // Alias to vScheduleRSTart();
    scheduler_start();
    return -1;
}
