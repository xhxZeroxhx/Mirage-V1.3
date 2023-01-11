/* Includes ------------------------------------------------------------------*/
#include "TLC5947.h"
/* Private user code ---------------------------------------------------------*/

uint16_t leds[24] = {}; // todo en 0, le pongo valores con FillArray
uint8_t spi_send[SPI_BYTE_AMOUNT]={};

void TLC_Update(void)
{
	uint8_t si = 0;//Lo uso para el vector a enviar via SPI

    HAL_GPIO_WritePin(TLC5947_BLANK_GPIO_Port, TLC5947_BLANK_Pin, GPIO_PIN_SET);

    for (int8_t i = 0; i < TOTAL_CHANNELS; i += 2) // lleno
    {
        uint8_t send1 = 0;
        uint8_t send = leds[i] >> 4; // mando MSB

        spi_send[si]=send;//
        si++;

        send = (leds[i] & 0x000F);
        send <<= 4;
        send1 = (leds[i+1]) >> 8;
        send |= send1; //me quedo con 4 bits menos significativos del canal i y 4 bits más significativos del canal i-1

        spi_send[si]=send;//
        si++;


        send = leds[i+1];//borro 4 bits más significativos del canal i-1 y mando LSB del canal i-1

        spi_send[si]=send;//
        si++;

    }

    TLC_Write(spi_send);

    HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TLC5947_XLAT_GPIO_Port, TLC5947_XLAT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TLC5947_BLANK_GPIO_Port, TLC5947_BLANK_Pin, GPIO_PIN_RESET);

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
void FillArray(uint8_t colorIntensity)
{
	static uint8_t position = 0, increment = 3,array_index = 0;
	static uint16_t intensity = 4095;

	  if(colorIntensity == 48)
		  //0
		  position = BLUE;//RED

	  if(colorIntensity == 49)
		  //1
		  position = GREEN;//GREEN

	  if(colorIntensity == 50)
		  //2
		  position = RED;//BLUE

	  if(colorIntensity == 52)
		  //HIGH - 4
		  intensity = 4095;

	  if(colorIntensity == 53)
		  //MID - 5
		  intensity = 1024;

	  if(colorIntensity == 54)
		  //LOW - 6
		  intensity = 32;

if(position >-1 && position<3)
{

	for (array_index= 0; array_index<TOTAL_CHANNELS;array_index++)
		leds[array_index] = 0;//all previous values are erased
}




	for (array_index=position; array_index<TOTAL_CHANNELS;array_index+=increment)
		leds[array_index] = intensity;

}
