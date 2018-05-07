#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "TFT_22_ILI9225.h"
#include "tasks.hpp"
#include "ssp0.h"
#include "printf_lib.h"
#include "utilities.h"
#include "io.hpp"

//LCD and LPC1758 pins
#define RST (uint8_t)29
#define RS (uint8_t)26
#define CS (uint8_t)31
#define BL (uint8_t)28

//Offsets
#define OFFSETX 176/3
#define OFFSETY 176/2
#define OFFSETZ 30
#define f 100

struct coordinate_pnt {
    int x;
    int y;
};

TFT_22_ILI9225 _lcd;

int height = LCD_HEIGHT;
int width = LCD_WIDTH;
int cam_x = 100, cam_y = 100, cam_z = 100;
bool semaphore_from_master = false;

struct coordinate_pnt transformation(int x_w, int y_w, int z_w)
{
    int scrn_x, scrn_y, x_diff = 176 / 1.7, y_diff = 176 / 2;
    double x_p, y_p, z_p, theta, phi, rho;
    struct coordinate_pnt screen;

    theta = 0.785549163;
    phi = 0.955295882;
    rho = sqrt((pow(cam_x, 2)) + (pow(cam_y, 2)) + (pow(cam_z, 2)));
    x_p = (y_w * cos(theta)) - (x_w * sin(theta));
    y_p = (z_w * sin(phi)) - (x_w * cos(theta) * cos(phi)) - (y_w * cos(phi) * sin(theta));
    z_p = rho - (y_w * sin(phi) * cos(theta)) - (x_w * sin(phi) * cos(theta)) - (z_w * cos(phi));
    scrn_x = x_p * f / z_p;
    scrn_y = y_p * f / z_p;
    scrn_x = x_diff + scrn_x;
    scrn_y = y_diff - scrn_y;
    screen.x = scrn_x;
    screen.y = scrn_y;
    return screen;
}

void drawAxis()
{
    struct coordinate_pnt lcd;
    float a_px[4], a_py[4], a_pz[4];
    uint32_t color[3] = { COLOR_RED, COLOR_GREEN, COLOR_BLUE };

    //co-ordinates for axis
    int b_px[10] = { 0, 100, 0, 0 };
    int b_py[10] = { 0, 0, 100, 0 };
    int b_pz[10] = { 0, 0, 0, 100 };

    for (int i = 0; i < 4; i++) {
        lcd = transformation(b_px[i], b_py[i], b_pz[i]);
        a_px[i] = lcd.x;
        a_py[i] = lcd.y;
    }

    for (int i = 0; i < 2; i++) {
        _lcd.drawLine(a_px[0], a_py[0], a_px[i + 1], a_py[i + 1], color[i]);          //x(red) and y(green) axis
        _lcd.drawLine(a_px[0], a_py[0] + 1, a_px[i + 1], a_py[i + 1] + 1, color[i]);
        _lcd.drawLine(a_px[0], a_py[0] - 1, a_px[i + 1], a_py[i + 1] - 1, color[i]);
    }
    _lcd.drawLine(a_px[0], a_py[0], a_px[2 + 1], a_py[2 + 1], color[2]);          //z(blue) axis
    _lcd.drawLine(a_px[0] + 1, a_py[0], a_px[2 + 1] + 1, a_py[2 + 1], color[2]);
    _lcd.drawLine(a_px[0] - 1, a_py[0], a_px[2 + 1] - 1, a_py[2 + 1], color[2]);

    for (int i = 0; i < 3; i++) {
        _lcd.fillCircle(a_px[i + 1], a_py[i + 1], 3, color[i]);
    }
}

void draw_cube(int start_pnt, int size)
{
    struct coordinate_pnt lcd;
    int a_px[10], a_py[10], a_pz[10];
    cam_x = 120, cam_y = 120, cam_z = 120;

    //Co-ordinates for cube
    int b_px[10] = { start_pnt, start_pnt + size, start_pnt + size, start_pnt, start_pnt + size, start_pnt + size, start_pnt };
    int b_py[10] = { start_pnt, start_pnt, start_pnt + size, start_pnt + size, start_pnt, start_pnt + size, start_pnt + size };
    int b_pz[10] = { start_pnt + size, start_pnt + size, start_pnt + size, start_pnt + size, start_pnt, start_pnt, start_pnt };

    //2D transformation of each point
    for (int i = 0; i < 7; i++) {
        lcd = transformation(b_px[i], b_py[i], b_pz[i]);
        a_px[i] = lcd.x;
        a_py[i] = lcd.y;
    }

    _lcd.drawLine(a_px[0], a_py[0], a_px[1], a_py[1], 0x00FF0000);
    _lcd.drawLine(a_px[1], a_py[1], a_px[2], a_py[2], 0x00FF0000);
    _lcd.drawLine(a_px[2], a_py[2], a_px[3], a_py[3], 0x00FF0000);
    _lcd.drawLine(a_px[3], a_py[3], a_px[0], a_py[0], 0x00FF0000);
    _lcd.drawLine(a_px[1], a_py[1], a_px[4], a_py[4], 0x00FF0000);
    _lcd.drawLine(a_px[4], a_py[4], a_px[5], a_py[5], 0x00FF0000);
    _lcd.drawLine(a_px[5], a_py[5], a_px[2], a_py[2], 0x00FF0000);
    _lcd.drawLine(a_px[5], a_py[5], a_px[6], a_py[6], 0x00FF0000);
    _lcd.drawLine(a_px[6], a_py[6], a_px[3], a_py[3], 0x00FF0000);

}

void draw_square(int angle)
{
    struct coordinate_pnt lcd;
    uint32_t color[4] = { COLOR_CYAN, COLOR_BROWN, COLOR_YELLOW, COLOR_WHITE };
    int x0, y0, y1, x1, x2, y2, x3, y3, size, i = 0;

    while (i < 4)
    {
        i++;
        x0 = 5 + rand() % (angle - 10);
        y0 = 5 + rand() % (angle - 10);
        size = 30 + rand() % (angle / 4);

        x1 = size + x0;
        if (x1 > angle) {
            x1 = angle - 5;
        }
        x2 = x1;
        x3 = x0;
        y1 = y0;
        y2 = size + y1;
        if (y2 > angle) {
            y2 = angle - 1;
        }
        y3 = y2;

        lcd = transformation(angle, x0, y0);
        x0 = lcd.x;
        y0 = lcd.y;
        lcd = transformation(angle, x1, y1);
        x1 = lcd.x;
        y1 = lcd.y;
        lcd = transformation(angle, x2, y2);
        x2 = lcd.x;
        y2 = lcd.y;
        lcd = transformation(angle, x3, y3);
        x3 = lcd.x;
        y3 = lcd.y;

        _lcd.drawLine(x0, y0, x1, y1, color[i]);
        _lcd.drawLine(x1, y1, x2, y2, color[i]);
        _lcd.drawLine(x2, y2, x3, y3, color[i]);
        _lcd.drawLine(x3, y3, x0, y0, color[i]);

        //for rotation
        int it;
        for (it = 0; it < 6; it++) {

            x0 = (x0 + (0.2 * (x1 - x0)));
            y0 = (y0 + (0.2 * (y1 - y0)));
            x1 = (x1 + (0.2 * (x2 - x1)));
            y1 = (y1 + (0.2 * (y2 - y1)));
            x2 = (x2 + (0.2 * (x3 - x2)));
            y2 = (y2 + (0.2 * (y3 - y2)));
            x3 = (x3 + (0.2 * (x0 - x3)));
            y3 = (y3 + (0.2 * (y0 - y3)));

            _lcd.drawLine(x0, y0, x1, y1, color[i]);
            _lcd.drawLine(x1, y1, x2, y2, color[i]);
            _lcd.drawLine(x2, y2, x3, y3, color[i]);
            _lcd.drawLine(x3, y3, x0, y0, color[i]);

            vTaskDelay(50);
        }
    }
}
void draw_tree(uint32_t color, int start_pnt, int size)
{
    int i = 0, angle;
    struct coordinate_pnt lcd;
    int tree_branch[3][3] = { { start_pnt + 10, start_pnt + 30, 0.5 * size }, { start_pnt + 20, start_pnt + 30, 0.3 * size }, { start_pnt + 15, start_pnt + 37,
            0.8 * size } };
    while (i < 3) {
        color = rand() % 0xFFFFFFu;
        int x0, y0, y1, x1, xp0, xp1, yp0, yp1;
        angle = start_pnt + size;
        x0 = tree_branch[i][0];
        x1 = tree_branch[i][1];
        y0 = tree_branch[i][2];
        y1 = y0;
        i++;
        lcd = transformation(y0, angle, x0);
        xp0 = lcd.x;
        yp0 = lcd.y;
        lcd = transformation(y1, angle, x1);
        xp1 = lcd.x;
        yp1 = lcd.y;
        _lcd.drawLine(xp0, yp0, xp1, yp1, color);       //level 0 straight line
        _lcd.drawLine((xp0 + 1), (yp0 + 1), (xp1 + 1), (yp1 + 1), color);       //level 0 straight line
        _lcd.drawLine((xp0 - 1), (yp0 - 1), (xp1 - 1), (yp1 - 1), color);       //level 0 straight line

        int it = 0;
        for (it = 0; it < 5; it++) {
            int16_t x2 = (0.6 * (x1 - x0)) + x1;                // length of level 1 = 0.8 of previous level
            int16_t y2 = y1;
            lcd = transformation(y2, angle, x2);
            int xp2 = lcd.x;
            int yp2 = lcd.y;
            _lcd.drawLine(xp1, yp1, xp2, yp2, color);        //level 1 straight line
            // _lcd.draw_rect(0, 0, 127, 159, 0x00000000);
            vTaskDelay(50);

            //for right rotated angle 30 degree
            int16_t xr = ((0.134 * x1) + (0.866 * x2) - (0.5 * y2) + (0.5 * y1));
            int16_t yr = ((0.5 * x2) - (0.5 * x1) + (0.866 * y2) - (0.866 * y1) + y1);
            lcd = transformation(yr, angle, xr);
            int xpr = lcd.x;
            int ypr = lcd.y;

            //for left rotated angle 30 degree
            int16_t xl = ((0.134 * x1) + (0.866 * x2) + (0.5 * y2) - (0.5 * y1));
            int16_t yl = ((0.5 * x1) - (0.5 * x2) + (0.134 * y2) + (0.866 * y1));
            lcd = transformation(yl, angle, xl);
            int xpl = lcd.x;
            int ypl = lcd.y;

            _lcd.drawLine(xp1, yp1, xpr, ypr, color);
            _lcd.drawLine(xp1, yp1, xpl, ypl, color);
            //_lcd.draw_rect(0, 0, 127, 159, 0x00000000);
            vTaskDelay(50);
            //for branches on right rotated branch angle 30 degree
            int16_t xrLen = sqrt(pow((xr - x1), 2) + pow((yr - y1), 2));   //length of right branch
            int16_t xrImag = (0.8 * xrLen) + xr;                 //imaginary vertical line x coordinate, y= yr
            int16_t xr1 = ((0.134 * xr) + (0.866 * xrImag) - (0.5 * yr) + (0.5 * yr));
            int16_t yr1 = ((0.5 * xrImag) - (0.5 * xr) + (0.866 * yr) - (0.866 * yr) + yr);
            lcd = transformation(yr1, angle, xr1);
            int xpr1 = lcd.x;
            int ypr1 = lcd.y;
            //for right branch
            int16_t xrr, xrl, yrr, yrl;
            xrr = ((0.134 * xr) + (0.866 * xr1) - (0.5 * yr1) + (0.5 * yr));
            yrr = ((0.5 * xr1) - (0.5 * xr) + (0.866 * yr1) - (0.866 * yr) + yr);
            lcd = transformation(yrr, angle, xrr);
            int xprr = lcd.x;
            int yprr = lcd.y;
            //for left branch
            xrl = ((0.134 * xr) + (0.866 * xr1) + (0.5 * yr1) - (0.5 * yr));
            yrl = ((0.5 * xr) - (0.5 * xr1) + (0.134 * yr) + (0.866 * yr1));
            lcd = transformation(yrl, angle, xrl);
            int xprl = lcd.x;
            int yprl = lcd.y;
            //for branches on left rotated branch angle 30 degree
            int16_t xlImag = (0.8 * xrLen) + xl;                 //imaginary vertical line x coordinate, y= yr
            int16_t xl1 = ((0.134 * xl) + (0.866 * xlImag) + (0.5 * yl) - (0.5 * yl));
            int16_t yl1 = ((0.5 * xl) - (0.5 * xlImag) + (0.134 * yl) + (0.866 * yl));
            lcd = transformation(yl1, angle, xl1);
            int xpl1 = lcd.x;
            int ypl1 = lcd.y;
            //for right branch
            int16_t xlr, xll, ylr, yll;
            xlr = ((0.134 * xl) + (0.866 * xl1) - (0.5 * yl1) + (0.5 * yl));
            ylr = ((0.5 * xl1) - (0.5 * xl) + (0.866 * yl1) - (0.866 * yl) + yl);
            lcd = transformation(ylr, angle, xlr);
            int xplr = lcd.x;
            int yplr = lcd.y;
            //for left branch
            xll = ((0.134 * xl) + (0.866 * xl1) + (0.5 * yl1) - (0.5 * yl));
            yll = ((0.5 * xl) - (0.5 * xl1) + (0.134 * yl) + (0.866 * yl1));
            lcd = transformation(yll, angle, xll);
            int xpll = lcd.x;
            int ypll = lcd.y;
            _lcd.drawLine(xpr, ypr, xpr1, ypr1, color);
            _lcd.drawLine(xpr, ypr, xprr, yprr, color);
            _lcd.drawLine(xpr, ypr, xprl, yprl, color);
            _lcd.drawLine(xpl, ypl, xpl1, ypl1, color);
            _lcd.drawLine(xpl, ypl, xplr, yplr, color);
            _lcd.drawLine(xpl, ypl, xpll, ypll, color);

            x0 = x1;
            x1 = x2;
        }
        vTaskDelay(50);
    }
}


void fill_cube(int start_pnt, int size)
{
    struct coordinate_pnt s1;
    uint32_t top_color = COLOR_BLUE;
    int i, j;

    size = size + start_pnt;

    for (i = 0; i < size; i++) {
        if (i % 10 == 0) top_color -= 4352;
        for (j = 0; j < size; j++) {

            // fill right side of cube green
            s1 = transformation(i, size, j);
            _lcd.drawPixel(s1.x, s1.y, COLOR_GREEN);

            // fill left side of cube red
            s1 = transformation(size, j, i);
            _lcd.drawPixel(s1.x, s1.y, COLOR_RED);

            //fill top side of cube green
            s1 = transformation(j, i, size);
            _lcd.drawPixel(s1.x, s1.y, top_color);
        }
    }
}

void fill_shade(int start_pnt, int size)
{
    struct coordinate_pnt s1;
    int i, j;

    size = size + start_pnt;

    for (i = 0; i < size; i++) {
        for (j = size; j < 2 * size; j++) {
            s1 = transformation(j, i, 0);
            _lcd.drawPixel(s1.x, s1.y, COLOR_GRAY);
        }
    }
}

void draw_shadow(double start_pnt, double size, double xShad, double yShad, double zShad)
{
    int i, j, k;
    int xs[8] = { 0 }, ys[8] = { 0 }, zs[8] = { 0 };
    struct coordinate_pnt s5, s6, s7, s8;
    uint32_t color = 0x00000000;
    double x[8] = { start_pnt, (start_pnt + size), (start_pnt + size), start_pnt, start_pnt, (start_pnt + size), (start_pnt + size), start_pnt };
    double y[8] = { start_pnt, start_pnt, start_pnt + size, start_pnt + size, start_pnt, start_pnt, (start_pnt + size), (start_pnt + size) };
    double z[8] = { start_pnt, start_pnt, start_pnt, start_pnt, (start_pnt + size), (start_pnt + size), (start_pnt + size), (start_pnt + size) };

    for (i = 0; i < 8; i++) {
        xs[i] = x[i] - ((z[i] / (zShad - z[i])) * (xShad - x[i]));
        ys[i] = y[i] - ((z[i] / (zShad - z[i])) * (yShad - y[i]));
        zs[i] = z[i] - ((z[i] / (zShad - z[i])) * (zShad - z[i]));
    }
    s5 = transformation(xs[4], ys[4], zs[4]);
    s6 = transformation(xs[5], ys[5], zs[5]);
    s7 = transformation(xs[6], ys[6], zs[6]);
    s8 = transformation(xs[7], ys[7], zs[7]);

    for (k = 0; k < size; k = k + 2) {
        for (j = 0; j < size; j = j + 2) {
            _lcd.drawLine((s5.x) - j, s5.y, (s8.x) - j, s8.y, color);
            _lcd.drawLine((s5.x) - (j + 1), s5.y, (s8.x) - (j + 1), s8.y, color);
        }
    }
}

void draw_A(int start_pnt, int size)
{
    struct coordinate_pnt p1;
    int i, j;
    size = size + start_pnt;
    int map[size][size];

    for (i = 0; i < size; i++) {
        for (j = 0; j < size; j++) {
            if (i >= start_pnt + 5 && j >= start_pnt + 10 && j <= size - 10 && i <= start_pnt + 15) map[i][j] = 1;
            else if (i >= start_pnt + 15 && j >= start_pnt + 10 && j <= start_pnt + 20 && i <= size - 5) map[i][j] = 1;
            else if (i >= start_pnt + 30 && j >= start_pnt + 10 && j <= size - 10 && i <= start_pnt + 40) map[i][j] = 1;
            else if (i >= start_pnt + 15 && j >= start_pnt + 50 && j <= start_pnt + 60 && i <= size - 5) map[i][j] = 1;
            else
                map[i][j] = 0;
        }
    }
    for (i = 0; i < size; i++) {
        for (j = 0; j < size; j++) {
            if (map[i][j] == 1) {
                p1 = transformation(j, i, size);
                _lcd.drawPixel(p1.x, p1.y, COLOR_BROWN);
            }
            else if (map[i][j] == 0) {
                p1 = transformation(j, i, size);
            }
        }
    }
}

class LCD: public scheduler_task {
private:

    bool flag = true;
    int x, y, x1, y1, x2, y2, x3, y3;
    int side;

    uint32_t i;
    int size = 70, start_pnt = 0;

public:
    LCD(uint8_t priority) :
            scheduler_task("LCD_interface", 1024 * 20, priority)
    {
    }

    bool init()
    {
        //Set LCD pins
        _lcd.setValues(RST, RS, CS, BL);
        return true;
    }

    bool run(void *p)
    {
        if (flag && semaphore_from_master) {
            _lcd.begin();
            _lcd.clear();
            drawAxis();
            draw_cube(start_pnt, size);
            fill_cube(start_pnt, size);
            draw_square(size + start_pnt);
            draw_tree(0x0066CC00, start_pnt, size);
            draw_A(start_pnt, size);
            fill_shade(start_pnt, size);
            flag = false;
        }
        return true;
    }
};


class master: public scheduler_task
{
private:
    char data = '5';
public:
    master(uint8_t priority) : scheduler_task("SPI_interface", 1024, priority){    }

    bool init()
    {
        ssp0_init(8);
        LPC_PINCON->PINSEL3 &= ~(3<<12);
        LPC_GPIO1->FIODIR |= (1<<22);
        return true;
    }

    bool run(void *p)
    {
        while(data != '*')
        {
            LPC_GPIO1->FIOCLR = (1<<22);
            data =  ssp0_exchange_byte('$');
            vTaskDelay(100);
            LPC_GPIO1->FIOSET = (1<<22);
        }
        semaphore_from_master = true;
        vTaskDelay(5000);
        return true;
    }
};

int main(void)
{


    //----LAB3----
    //SPI Communication
    scheduler_add_task(new master(PRIORITY_HIGH));
    //Computing Graphics
    scheduler_add_task(new LCD(PRIORITY_MEDIUM));

    // Alias to vScheduleRSTart();
    scheduler_start();
    return -1;
}
