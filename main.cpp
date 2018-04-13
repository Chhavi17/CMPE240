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

#define OFFSETX 176/2                  // offsets
#define OFFSETY 176/2
#define OFFSETZ 30
#define magic 176/2
const signed int a[4] = { 0, 20, 0, 0 };
const signed int b[4] = { 0, 0, -20, 0 };
const signed int c[4] = { 0, 0, 0, 20 };
const int f[3] = { 1, 1, 1 };          // start vertex for lines
const int g[3] = { 2, 3, 4 };          // end vertex for lines

int sx[5], sy[5], ex[5], ey[5];                    // define global vars for calling graphics subroutines
uint32_t color_[3] = { COLOR_RED, COLOR_BLUE, COLOR_GREEN };


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


      int newx[4]; //projected 2d X
        int newy[4]; //projected 2d Y
        int i, loop;
        float xt, yt, zt, x, y, z, sinax, cosax, sinay, cosay, sinaz, cosaz, vertex;
        float xpos = 0; // position for object in 3d space, in x
        float ypos = 0; // y
        float zpos = 0; // and z values
        float rotx = 0; // starting amount of x rotation
        float roty = 0; // starting amount of y rotation
        float rotz = 0; // starting amount of z rotation
        bool change = false;

     //   for (loop = 0; loop <= 100; loop++) // rotate the axis 100 times
        while(1)

                {

            //translation
            xpos = xpos + 0.0;            // move the object
            ypos = ypos + 0.0;
            zpos = zpos + 0.0;

            //rotations by 10 degrees
            if (SW.getSwitch(1))
            {
                rotx = rotx + (10 * (3.14 / 180));
                change = true;

            }
            if (SW.getSwitch(2))
            {
                roty = roty + (10*(3.14/180));
                change = true;
                // rotate on Y axis
            }
            if (SW.getSwitch(3))
            {
                rotz = rotz + (10*(3.14/180));                // rotate on Z axis
                change = true;
            }

            //can be made to lookup table
            sinax = sin(rotx);
            cosax = cos(rotx);

            sinay = sin(roty);
            cosay = cos(roty);

            sinaz = sin(rotz);
            cosaz = cos(rotz);

            for (i = 0; i < 4; i++)            // translate 3d vertex position to 2d screen position
                    {
                x = a[i];                    // get x for vertex i
                y = b[i];                    // get y for vertex i
                z = c[i];                    // get z for vertex i

                yt = y * cosax - z * sinax;                    // rotate around the x axis
                zt = y * sinax + z * cosax;                    // using the Y and Z for the rotation
                y = yt;
                z = zt;

                xt = x * cosay - z * sinay;                    // rotate around the Y axis
                zt = x * sinay + z * cosay;                    // using X and Z
                x = xt;
                z = zt;

                xt = x * cosaz - y * sinaz;                    // finaly rotate around the Z axis
                yt = x * sinaz + y * cosaz;                    // using X and Y
                x = xt;
                y = yt;

                x = x + xpos;                    // add the object position offset
                y = y + ypos;                    // for both x and y
                z = z + OFFSETZ - zpos;         // as well as Z

                newx[i] = (x * magic / z) + OFFSETX;                    // translate 3d to 2d coordinates for screen
                newy[i] = (y * magic / z) + OFFSETY;                    // drawing
            }

          //  lcd.clear();                    // clear the screen to remove old axis
        if(change == true)
        {
            lcd.drawLine(sx[0], sy[0], ex[0], ey[0], COLOR_BLACK);
           lcd.drawLine(sx[1], sy[1], ex[1], ey[1], COLOR_BLACK);
           lcd.drawLine(sx[2], sy[2], ex[2], ey[2], COLOR_BLACK);
            change = false;
        }


            for (i = 0; i < 3; i++)                    // draw the lines that make up the object
{
                char c;
                vertex = f[i] - 1;         // temp = start vertex for this line
                sx[i] = newx[(int) vertex];         // set line start x to vertex[i] x position
                sy[i] = newy[(int) vertex];         // set line start y to vertex[i] y position
                vertex = g[i] - 1;         // temp = end vertex for this line
                ex[i] = newx[(int) vertex];         // set line end x to vertex[i+1] x position
                ey[i] = newy[(int) vertex];         // set line end y to vertex[i+1] y position
                lcd.drawLine(sx[i], sy[i], ex[i], ey[i], color_[i]);


//                  if (i == 0) lcd.drawChar(10, 10, 'X', color_[i]);
//                else if (i == 1) lcd.drawChar(10, 20, 'Y', color_[i]);
//                else
//                    lcd.drawChar(10, 30, 'Z', color_[i]);

            }
        }


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
