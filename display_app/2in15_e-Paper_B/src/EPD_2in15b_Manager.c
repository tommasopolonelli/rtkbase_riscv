#include "EPD_2in15b_Manager.h"
#include "EPD_2in15b.h"
#include <time.h> 

int EPD_2in15b_Manager(const char *file_r, const char *file_b)
{

    if(DEV_Module_Init()!=0){
        return -1;
    }

    Debug("e-Paper Init and Clear...\r\n");
    EPD_2IN15B_Init();
    //EPD_2IN15B_Clear();
    //DEV_Delay_ms(500);

    //Create a new image cache
    UBYTE *BlackImage, *RedImage;
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    UWORD Imagesize = ((EPD_2IN15B_WIDTH % 8 == 0)? (EPD_2IN15B_WIDTH / 8 ): (EPD_2IN15B_WIDTH / 8 + 1)) * EPD_2IN15B_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        Debug("Failed to apply for black memory...\r\n");
        return -1;
    }
    if((RedImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        Debug("Failed to apply for red memory...\r\n");
        return -1;
    }

    Debug("show bmp------------------------\r\n");
    Paint_NewImage(BlackImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 0, WHITE);
    Paint_SelectImage(BlackImage);
    GUI_ReadBmp(file_b, 0, 0);
    Paint_NewImage(RedImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 0, WHITE);
    Paint_SelectImage(RedImage);
    GUI_ReadBmp(file_r, 0, 0);
    EPD_2IN15B_Display(BlackImage, RedImage);
    DEV_Delay_ms(100);

#if 0   // show image for array    
    Debug("show image for array\r\n");
    EPD_2IN15B_Display(gImage_2in15b_B, gImage_2in15b_R);
    DEV_Delay_ms(2000);
#endif

#if 0   // Drawing on the image
    Debug("Drawing\r\n");

    //1.Draw black image
    Paint_NewImage(BlackImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 270, WHITE);
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);
    Paint_DrawPoint(10, 80, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 90, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 100, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 110, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    Paint_DrawLine(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(70, 70, 20, 120, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawRectangle(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(80, 70, 130, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawString_EN(10, 0, "waveshare", &Font16, BLACK, WHITE);
    Paint_DrawString_CN(130, 20, "΢ѩ����", &Font24CN, WHITE, BLACK);
    Paint_DrawNum(10, 50, 987654321, &Font16, WHITE, BLACK);

    //2.Draw red image
    Paint_NewImage(RedImage, EPD_2IN15B_WIDTH, EPD_2IN15B_HEIGHT, 270, WHITE);
    Paint_SelectImage(RedImage);
    Paint_Clear(WHITE);
    Paint_DrawCircle(160, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(210, 95, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawLine(85, 95, 125, 95, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine(105, 75, 105, 115, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawString_CN(130, 0,"���abc", &Font12CN, BLACK, WHITE);
    Paint_DrawString_EN(10, 20, "hello world", &Font12, WHITE, BLACK);
    Paint_DrawNum(10, 33, 123456789, &Font12, BLACK, WHITE);

    EPD_2IN15B_Display(BlackImage, RedImage);
    DEV_Delay_ms(2000);

#endif

    Debug("Goto Sleep...\r\n");
    EPD_2IN15B_Sleep();
    free(BlackImage);
    BlackImage = NULL;
    DEV_Delay_ms(2000);//important, at least 2s
    // close 5V
    Debug("Disable VDD, Module enters 0 power consumption ...\r\n");
    DEV_Module_Exit();

    return 0;
}

