//======================================================================================================================
// Copyright(c) 2018 Microchip Technology Inc. and its subsidiaries.
// Subject to your compliance with these terms, you may use Microchip software and any derivatives exclusively with
// Microchip products. It is your responsibility to comply with third party license terms applicable to your use of
// third-party software (including open source software) that may accompany Microchip software.
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO
// THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
// PURPOSE.
// IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE,
// COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED
// OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY
// ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE
// PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//======================================================================================================================

//======================================================================================================================
// @file dev_lcd.c
//
// @brief driver for the lcd screen
//
//======================================================================================================================

#include <stdint.h>
#include "stdbool.h"
#include <xc.h>
#include "misc/delay.h"
#include "driver/drv_lcd_interface.h"

//======================================================================================================================
//  defines commands for the Newhaven NHD-C0216CZ-FSW-FBW lcd controller
//======================================================================================================================

//there is a difference in real display size and display memory
#define LCD_ADDRESS_LINE_1 0x00
#define LCD_ADDRESS_LINE_2 0x40

#define LCD_DISPLAYSIZE_X   16
#define LCD_DISPLAYSIZE_Y   2

#define LCD_CLEAR       0x01
#define LCD_HOME        0x02

#define CURSOR_nSHIFT   0x00
#define CURSOR_SHIFT    0x01
#define DATA_DECREMENT  0x00
#define DATA_INCREMENT  0x02
#define LCD_ENTRY_MODE  0x04

#define CURSOR_OFF      0x00
#define CURSOR_ON       0x02
#define BLINK_OFF       0x00
#define BLINK_ON        0x01
#define LCD_DISPLAY_OFF 0x08
#define LCD_DISPLAY_ON  0x0C

#define FUNCTION_nIS    0x00
#define FUNCTION_IS     0x01
#define FUNCTION_1_HIGH 0x00
#define FUNCTION_2_HIGH 0x04
#define FUNCTION_1_LINE 0x00
#define FUNCTION_2_LINE 0x08
#define FUNCTION_4BITS  0x00
#define FUNCTION_8BITS  0x10

#define LCD_FUNCTION    0x20

#define LCD_CGRAM_ADDRESS(adr) (0x40 | (adr & 0x3F))
#define LCD_DDRAM_ADDRESS(adr) (0x80 | (adr & 0x7F))

// Second Instruction Page (IS)
#define BIAS_1_5      0x00
#define BIAS_1_4      0x08
#define FREQ_CNTRL(f) (f&0x07)
#define LCD_OSC_FREQ  0x10

#define LCD_ICON_ADDRESS(adr) (0x40 | (adr & 0x0F))

#define nICON           0x00
#define ICON            0x08
#define nBOOST          0x00
#define BOOSTLCD        0x04
#define CONTRAST(c)    (c&0x03)
#define LCD_PWR_CONTROL 0x50

#define FOLLOWER_GAIN(g) (g&0x07)
#define LCD_FOLLOWER_OFF   0x60
#define LCD_FOLLOWER_ON    0x68

#define LCD_CONTRAST(c) (0x70 | (c & 0x0F))

#define LCD_BUSY_FLAG_MASK 0x80
#define LCD_ADDRESS_MASK   0x7F

//======================================================================================================================
//	variables
//======================================================================================================================
const uint8_t line_address[] = {LCD_ADDRESS_LINE_1,LCD_ADDRESS_LINE_2};
uint8_t pos_x = 0;
uint8_t pos_y = 0;
uint8_t change_position = false;

//======================================================================================================================
// @brief initializes the LCD Device, needs to be called once at bootup before that device can be used
//======================================================================================================================
void Dev_Lcd_Init(void)
{
    Lcd_Interface_Init();
    Lcd_Interface_Reset();
    
    Lcd_Interface_SendCmd(LCD_FUNCTION | FUNCTION_8BITS | FUNCTION_1_HIGH | FUNCTION_1_LINE | FUNCTION_nIS);   //function set
 
    __delay_ms(25);

    Lcd_Interface_SendCmd(LCD_FUNCTION | FUNCTION_8BITS | FUNCTION_1_HIGH | FUNCTION_1_LINE | FUNCTION_nIS);   //function set
    Lcd_Interface_SendCmd(LCD_FUNCTION | FUNCTION_8BITS | FUNCTION_1_HIGH | FUNCTION_1_LINE | FUNCTION_nIS);   //function set

    // Enter the second page of instructions
    Lcd_Interface_SendCmd(LCD_FUNCTION | FUNCTION_8BITS | FUNCTION_1_HIGH | FUNCTION_2_LINE | FUNCTION_IS);   //function set
    Lcd_Interface_SendCmd(LCD_OSC_FREQ | BIAS_1_5 | FREQ_CNTRL(4));   //internal osc frequency
    Lcd_Interface_SendCmd(LCD_PWR_CONTROL | nICON | BOOSTLCD | CONTRAST(2));   //power control
    Lcd_Interface_SendCmd(LCD_FOLLOWER_ON | FOLLOWER_GAIN(5));   //follower control
    Lcd_Interface_SendCmd(LCD_CONTRAST(0));   //contrast
    // leave second instruction page

    //Dev_Lcd_WriteCommand(LCD_FUNCTION | FUNCTION_8BITS | FUNCTION_1_HIGH | FUNCTION_2_LINE | FUNCTION_nIS);   //function set
    Lcd_Interface_SendCmd(LCD_DISPLAY_ON | CURSOR_OFF | BLINK_OFF);           //display on
    Lcd_Interface_SendCmd(LCD_ENTRY_MODE | CURSOR_nSHIFT | DATA_INCREMENT);   //entry mode
    Lcd_Interface_SendCmd(LCD_CLEAR);   //clear

    __delay_ms(150);
}


//======================================================================================================================
// @brief clears the lcd screen and sets the cursor position at 0,0 (left upper corner))
// @note this can also be achieved by sending the character '\f' within a string
//======================================================================================================================
void Dev_Lcd_Clear(void)
{
    Lcd_Interface_SendCmd(LCD_CLEAR);
    __delay_ms(1);
}


//======================================================================================================================
// @brief   sets the cursor position to the given x- and y-coordinates starting with zero
// @param   x x-coordinates for the new cursor position starting with zero
// @param   y y-coordinates for the new cursor position starting with zero
//======================================================================================================================
void Dev_Lcd_GotoXY(uint8_t x,uint8_t y)
{
    Lcd_Interface_SendCmd(LCD_DDRAM_ADDRESS((line_address[y] + x)));
    pos_x = x;
    pos_y = y;
    change_position = false;
}

//======================================================================================================================
// @brief   writes the given string on the lcd screen
// @param   str is the string to be written on the lcd screen
// @note    '\f' clears the screen and positions the cursor on the upper left corner,
// @note    '\r' sets the x position of the cursor to 0
// @note    '\n' poairiona the cursor on the next line (without changing the x-position)
//======================================================================================================================
void Dev_Lcd_WriteString(const char *str)
{
    while(*str)
    {
        if (change_position)
        {
            Dev_Lcd_GotoXY(pos_x, pos_y);
            change_position = false;
        }
                
        switch (*str)
        {
            case '\f':          //sets position to 0,0 after clearing the screen. this is slow (1ms)!
                Dev_Lcd_Clear();
                pos_x = 0;
                pos_y = 0;
                change_position = false;
                break;
            case '\v':          //sets position to 0,0 without clearing the screen
                pos_x = 0;
                pos_y = 0;
                change_position = true;
                break;
            case '\r':  //carriage return ==> x=0;
                pos_x = 0;
                change_position = true;
                break;
            case '\n':  //new line return ==> y++;
                pos_y++;
                change_position = true;
                break;
            default:
               	Lcd_Interface_SendChar(*str);
                if (++pos_x >= LCD_DISPLAYSIZE_X)
                {
                    pos_x = 0;
                    pos_y++;
                    change_position = true;
                }
                break;
        }
        //if (pos_y >= LCD_DISPLAYSIZE_Y)
        //{
        //    pos_y = 0;
        //    change_position = true;
        //}
        str++;
    }
}

//======================================================================================================================
// @brief   sets the cursor position to the given x- and y-coordinates and writes the given string on the lcd screen
// @param   x x-coordinates for the new cursor position starting with zero
// @param   y y-coordinates for the new cursor position starting with zero
// @param   str is the string to be written on the lcd screen
// @note    '\f' clears the screen and positions the cursor on the upper left corner,
// @note    '\r' sets the x position of the cursor to 0
// @note    '\n' poairiona the cursor on the next line (without changing the x-position)
//======================================================================================================================
void Dev_Lcd_WriteStringXY(uint8_t column_index, uint8_t line_index, const char *str)
{
    Dev_Lcd_GotoXY(column_index, line_index);
    Dev_Lcd_WriteString(str);    
}
