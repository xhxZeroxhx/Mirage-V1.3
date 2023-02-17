/* Includes ------------------------------------------------------------------*/
#include "TLC5947.h"
#include "main.h"
/* Private user code ---------------------------------------------------------*/

uint8_t g_LedsMatrix[1][TOTAL_CHANNELS]={}; // todo en 0, le pongo valores con FillArray, g_LedsMatrix indicates the led value per degree
uint8_t g_spi_send[SPI_BYTE_AMOUNT]={},g_command =0;//g_command is used to indicate what to display with all the leds
uint8_t letterO[TOTAL_LEDS][6]=
{
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,255,255,0,0,
		0,255,255,255,255,0,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		255,0,0,0,0,255,
		0,255,255,255,255,0,
		0,0,255,255,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0

};

void TLC_Update(void)
{
	uint8_t tlcDevice =1,matrixIndex = TLC5947_CHANNELS*tlcDevice-1,sendByteIndex = 0;//Lo uso para el vector a enviar via SPI



    HAL_GPIO_WritePin(TLC5947_BLANK1_GPIO_Port, TLC5947_BLANK1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC5947_BLANK2_GPIO_Port, TLC5947_BLANK2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC5947_BLANK3_GPIO_Port, TLC5947_BLANK3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC5947_BLANK4_GPIO_Port, TLC5947_BLANK4_Pin, GPIO_PIN_SET);



    for (int8_t i = TOTAL_CHANNELS-1; i >= 0 ; i -= 2) // It to start at the first channel of each TLC5947, starts at U4
    {

        uint8_t send1 = 0;
        uint8_t send = g_LedsMatrix[0][matrixIndex] >> 4; // Sending MSB, for initial test we'll only use 0° since it's not spinning


        g_spi_send[sendByteIndex]=send;
        sendByteIndex++;

        send = (g_LedsMatrix[0][matrixIndex] & 0x000F);
        send <<= 4;

        send1 = (g_LedsMatrix[0][matrixIndex-1]) >> 8;
        send |= send1; //me quedo con 4 bits menos significativos del canal i y 4 bits más significativos del canal i-1

        g_spi_send[sendByteIndex]=send;//
        sendByteIndex++;

        send = g_LedsMatrix[0][matrixIndex-1];//Erasing 4 most significant bits from chanel i-1, and sending LSB from chanel i-1

        g_spi_send[sendByteIndex]=send;//
        sendByteIndex++;
        matrixIndex-=2;
        if(sendByteIndex%36==0){
        	/*
        	 * I realized that each tlc5947 is isolated in the sense that u either start at the start or
        	 * at the end of each and there is no logic checking this.
        	 * With the code i had, the first value of g_LedsMatrix was being assigned
        	 * to the first led of u4 so i shifted my starting point in g_LedsMatrix
        	 * so as to align with each tlc5947 ux
        	 */
        	tlcDevice++;
        	matrixIndex=TLC5947_CHANNELS*tlcDevice-1;
        }


    }



    TLC_Write(g_spi_send);

    HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TLC5947_BLANK1_GPIO_Port, TLC5947_BLANK1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TLC5947_BLANK2_GPIO_Port, TLC5947_BLANK2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TLC5947_BLANK3_GPIO_Port, TLC5947_BLANK3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TLC5947_BLANK4_GPIO_Port, TLC5947_BLANK4_Pin, GPIO_PIN_RESET);

}
//void TLC_Write(uint8_t data[])
////void TLC_Write(uint8_t *data)
//{
//	HAL_SPI_Transmit(&hspi1,data, SPI_BYTE_AMOUNT,1000); // envio via el sp1 de 1 todos los bytes que tenga que mandar
////    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY); // espero a que termine la transferen
//}
/*
 * Uso esta funcion para llenar los vectores de prueba
 * por algún motivo al poner valores en dudo aparecian valores espúreos
 * en las últimas posiciones de 65535
 *
 */
void FillArray(uint8_t colorIntensity,uint8_t ledControl)
{
	static uint8_t increment = 3,array_index = 0,letCol = 0;
	uint8_t letRow = 0;

	switch(ledControl){

	case 0:
		//FABITEST
		for (array_index = 0; array_index<TOTAL_CHANNELS;array_index++)
			g_LedsMatrix[0][array_index]=0;

		for (array_index=colorIntensity; array_index<TOTAL_CHANNELS;array_index+=increment)
			g_LedsMatrix[0][array_index]=255;
	break;

	case 1:
		//LINE
		for (array_index = colorIntensity; array_index <TOTAL_CHANNELS;array_index+=increment){
//			if(g_degreeCount == 200)//when the motor is plugged change this to 179
			if(array_index == 0||array_index == 3||array_index == 90)
				g_LedsMatrix[0][array_index]=255;
			else
				 g_LedsMatrix[0][array_index]=0;
		}

	break;
	case 2:
		//LETTER O

		for (array_index = colorIntensity; array_index <TOTAL_CHANNELS;array_index+=increment){
			if(g_degreeCount >= 179 && g_degreeCount<= 184)
			{//when the motor is plugged change this to 179
				g_LedsMatrix[0][array_index]=letterO[letRow][letCol];
				letRow++;
				if(letRow==31)
				{
					/*
					 * This cycle allows to go through the letter vector as TIM4 updates g_degreeCount
					 * the colum needs to change everytime the function is accessed, but the row must always start at 0
					 */
					letCol++;
					if(letCol>=6)
						letCol =0;
				}
			}
			else
				 g_LedsMatrix[0][array_index]=0;
		}


//		for (array_index = colorIntensity; array_index <TOTAL_CHANNELS;array_index+=increment){
//			if(g_degreeCount >= 30 && g_degreeCount<= 80)
//			{
//				if(letterO[letRow][letCol]!=-1)
//					g_LedsMatrix[0][letterO[letRow][letCol]]=255;
//				letRow++;
//				if(letRow==31)
//				{
//					/*
//					 * This cycle allows to go through the letter vector as TIM4 updates g_degreeCount
//					 * the colum needs to change everytime the function is accessed, but the row must always start at 0
//					 */
//					letCol++;
//					if(letCol>=6)
//						letCol =0;
//				}
//			}
//			else
//				 g_LedsMatrix[0][array_index]=0;
//		}

	break;

	}

}
