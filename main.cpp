#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <TFT_22_ILI9225.h>
#include "tasks.hpp"
#include "ssp0.h"
#include "printf_lib.h"
#include "utilities.h"
#include "io.hpp"

#define RST (uint8_t)29
#define RS (uint8_t)26
#define CS (uint8_t)31
#define BL (uint8_t)28

#define OFFSETX 176/2                  // offsets
#define OFFSETY 176/2
#define OFFSETZ 30
#define f 176/2

class LCD: public scheduler_task {
private:

    TFT_22_ILI9225 lcd;
    bool flag = true;
    int x, y, x1, y1, x2, y2, x3, y3;
    int side;

public:
    LCD(uint8_t priority) :
            scheduler_task("LCD_interface", 1024 * 20, priority)
    {
    }

    bool init()
    {
        lcd.setValues(RST, RS, CS, BL);
        return true;
    }

    bool run(void *p)
    {
        if (flag) {
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

        //-------working code for trees--------//
        x = rand() % LCD_WIDTH;
        y = 0;
        uint32_t color = rand() % 0xFFFFFFu;

        lcd.draw_tree(x, y, color);
//lcd.drawLine(LCD_WIDTH/2,0,LCD_WIDTH+10,LCD_HEIGHT-15,color);

        //-------------working code for squares-------------//
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

        if (SW.getSwitch(4))
        {
            lcd.clear();
        }

        return true;
    }
};

class Transformation: public scheduler_task {
private:

    TFT_22_ILI9225 lcd;
    bool flag;

    //Lookup table for sin and cosine values
    float sin[18] = { 0, 0.34, 0.64, 0.87, 0.98, 0.98, 0.87, 0.64, 0.34, 0, -0.34, -0.64, -0.87, -0.98, -0.98, -0.87, -0.64, -0.34 };
    float cos[18] = { 1, 0.94, 0.77, 0.5, 0.17, -0.17, -0.5, -0.77, -0.94, -1, -0.94, -0.77, -0.5, -0.17, 0.17, 0.5, 0.77, 0.94 };

    float a_px, a_py, a_pz;                     //after rotation temp values
    int rotx = 0, roty = 0, rotz = 0;           //rotation angle iterators for x, y, z-axis
    int newx[10] = { 0 }, newy[10] = { 0 };     //2D co-ordinates
    bool change;

    //color for axes
    uint32_t color_[3] = { COLOR_RED, COLOR_BLUE, COLOR_GREEN };

    //Cube
    int sv[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4 };          // start vertex for lines
    int ev[12] = { 2, 3, 4, 1, 6, 7, 8, 5, 6, 7, 8, 5 };          // end vertex for lines

public:
    Transformation(uint8_t priority) :
            scheduler_task("Rotating 3D cube and displaying in 2D plane", 1024 * 20, priority)
    {
    }

    bool init()
    {
        lcd.setValues(RST, RS, CS, BL);
        return true;
    }

    bool run(void *p)
    {
        if (flag) {
            lcd.begin();
            flag = false;
        }


        //Original Points for axis
        //        int b_px[10] = { 0, 20, 0, 0 };
        //        int b_py[10] = { 0, 0, -20, 0 };
        //        int b_pz[10] = { 0, 0, 0, 20 };

        //Original Points for cube
        int b_px[10] = { 0, 10, 10, 0, 0, 0, 10, 10 };
        int b_py[10] = { 0, 0, 0, 0, 10, 10, 10, 10 };
        int b_pz[10] = { 0, 0, 10, 10, 10, 0, 0, 10 };

        if (SW.getSwitch(1)) //rotation wrt x-axis
        {
            if(rotx == 17) rotx = 0;
            else
            rotx += 1;
            change = true;
        }
        if (SW.getSwitch(2)) //rotation wrt y-axis
        {

            if(roty == 17) roty = 0;
            else
            roty += 1;
            change = true;
        }
        if (SW.getSwitch(3)) //rotation wrt z-axis
        {
            if(rotz == 17) rotz = 0;
            else
            rotz += 1;
            change = true;
        }

        //Rotation matrix
        for (int i = 0; i < 8; i++) {
            a_py = b_py[i] * cos[rotx] - b_pz[i] * sin[rotx];
            a_pz = b_py[i] * sin[rotx] + b_pz[i] * cos[rotx];
            b_py[i] = a_py;
            b_pz[i] = a_pz;

            a_px = b_px[i] * cos[roty] + b_pz[i] * sin[roty];
            a_pz = -(b_px[i] * sin[roty]) + b_pz[i] * cos[roty];
            b_px[i] = a_px;
            b_pz[i] = a_pz;

            a_px = b_px[i] * cos[rotz] - b_py[i] * sin[rotz];
            a_py = b_px[i] * sin[rotz] + b_py[i] * cos[rotz];
            b_py[i] = a_py;
            b_px[i] = a_px;

            if (change == true) {
                //for axis
                //                      for (int j = 1; j < 4; j++) {
                //                          lcd.drawLine(newx[0], newy[0], newx[j], newy[j], COLOR_BLACK);
                //                      }
                //for cube
                for (int j = 0; j < 12; j++) {
                    lcd.drawLine(newx[sv[j] - 1], newy[sv[j] - 1], newx[ev[j] - 1], newy[ev[j] - 1], COLOR_BLACK);
                }
                change = false;
            }

            //scaling and adding offset for 2D co-ordinates
            b_pz[i] += OFFSETZ;
            newx[i] = (b_px[i] * f / b_pz[i]) + OFFSETX;
            newy[i] = (b_py[i] * f / b_pz[i]) + OFFSETY;
        }

        ////for axis
        //            for (int j = 1; j < 4; j++) {
        //                lcd.drawLine(newx[0], newy[0], newx[j], newy[j], color_[j - 1]);
        //
        //            }

        //for cube
        for (int j = 0; j < 12; j++) {
            lcd.drawLine(newx[sv[j] - 1], newy[sv[j] - 1], newx[ev[j] - 1], newy[ev[j] - 1], COLOR_GOLD);
        }

        if (SW.getSwitch(4))
        {
            lcd.clear();
        }

        return true;

    }
};


class master: public scheduler_task {
private:
    char dat;
public:
    master(uint8_t priority) :
            scheduler_task("SPI_interface", 1024, priority)
    {
    }

    bool init()
    {
        ssp0_init(1);
        LPC_PINCON->PINSEL3 |= (3 << 8) | (3 << 14) | (3 << 16);

        LPC_PINCON->PINSEL3 &= ~(3 << 12);
        LPC_GPIO1->FIODIR |= (1 << 22);
        LE.on(1);

        ssp0_set_max_clock(4);
        return true;
    }

    bool run(void *p)
    {

        LPC_GPIO1->FIOCLR = (1 << 22);

        dat = ssp0_exchange_byte('C');
        u0_dbg_printf("received: %c\n", dat);
        // dat = ssp0_exchange_byte(dat);
        LE.off(1);
        LPC_GPIO1->FIOSET = (1 << 22);
        vTaskDelay(100);
        return true;
    }
};

int main(void)
{
    //scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    //----LAB1----
    //scheduler_add_task(new LCD(2));

    //----LAB2----
    scheduler_add_task(new Transformation(2));
    // scheduler_add_task(new master(2));

    // Alias to vScheduleRSTart();
    scheduler_start();
    return -1;
}
