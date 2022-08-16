#include<stdint.h>
/*ad7606驱动需要的一些参数和函数声明-add by lichen 20220720*/

void AD7606_IOset();
void AD7606_SetInputRange(int range);
void AD7606_OSset();
void AD7606_Reset();
void AD7606_Init();
void AD7606_StartConv();
bool AD7606_StartADC();

#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_RST  33
#define PIN_NUM_CVAB 2
#define PIN_NUM_BUSY 16 
#define PIN_NUM_RAGE 32
#define PIN_NUM_OS0  25
#define PIN_NUM_OS1  26
#define PIN_NUM_OS2  27
#define CH_NUM       8u  //u表示无符号整形
#define DMA_CHAN     2
#define AD_HOST    SPI3_HOST
#define SPP_DATA_LEN 16   

uint8_t ADC_val[CH_NUM * 2];

static uint8_t spp_data[SPP_DATA_LEN]; 


// uint8_t *spp_2data; 

