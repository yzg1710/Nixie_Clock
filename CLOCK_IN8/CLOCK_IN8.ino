#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <Ethernet.h>

#include <SPI.h>

#include <Arduino.h>
#include "RTClib.h"

  #define VSPI_MISO   MISO
  #define VSPI_MOSI   MOSI
  #define VSPI_SCLK   SCK
  #define VSPI_SS     SS

//  #define DARK  0x303802
  #define DARK  0x151020
  
//  #define UP  {4,0x151020}, \
//              {4,0x152040}, \
//              {4,0x253060}, \
//              {4,0x2540A0}, \
//              {4,0x3550F0},
//              
//  #define DOWN  {4,0x2540A0}, \
//                {4,0x253060}, \
//                {4,0x152040}, \
//                {4,0x151030}, \
//                {4,0x051020}, 

  #define UP  {4,0x1010ff}, \
              {4,0x1010ff}, \
              {4,0x1010ff}, \
              {4,0x1010ff}, \
              {4,0x1010ff},
              
  #define DOWN  {4,0x1010ff}, \
                {4,0x1010ff}, \
                {4,0x1010ff}, \
                {4,0x1010ff}, \
                {4,0x1010ff}, 
                

  #define UP_DOWN   UP \
                    DOWN        
                
//定义2个定时器句柄
//esp_timer_handle_t LED_Data_High_Time_handle = 0;
//esp_timer_handle_t LED_Data_Low_Time_handle = 0;

const uint16_t Data2SbitMap[]=
{
0x0200,     //  0
0x0001,     //  1
0x0002,     //  2
0x0004,     //  3
0x0008,     //  4   
0x0010,     //  5
0x0020,     //  6
0x0040,     //  7
0x0080,     //  8
0x0100,     //  9
};
uint8_t hour;
uint8_t minute;
uint8_t second;
DateTime now;

uint8_t TimeData[2];//units,tens，
uint8_t SpiDisData[8];

static const int spiClk = 10000000; // 10 MHz
//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;

const int Led_Data_Out_Pin=25;    //
const int HV_Ctl_Pin=26;          //
const int Seconds_Int_Pin=33;     //
const int Test_Pin=4;     //

const uint32_t Led_amount = 39;   //背光led数量
uint32_t stepCnt[Led_amount];
uint32_t startTick[Led_amount];
uint32_t Led_activeTable_Size;

typedef struct 
{
  uint32_t holdTick;
  uint32_t rgb;
}LED_DATA_TYPE;

typedef struct 
{
  LED_DATA_TYPE *pLedData;
  uint32_t itemSize;
}LED_ACTIVE_TYPE;

LED_DATA_TYPE Led_0_dataDisplay[]=
{
  {10,0xD0D00A},       //
  {20,0xD0D00A},      //
};

LED_DATA_TYPE Led_1_dataDisplay[]=
{
  {10,0x10D0FA},       //
  {30,0x10D0FA},      //
};

LED_DATA_TYPE Led_2_dataDisplay[]=
{
  {10,0xF010FA},       //
  {10,0xF010FA},      //
};

LED_DATA_TYPE Led_3_dataDisplay[]=
{
  {10,0x20F000},       //
  {10,0x20F000},      //
};

LED_DATA_TYPE Led_4_dataDisplay[]=
{
  {10,0xF04000},       //
  {10,0xF04000},      //
};

LED_DATA_TYPE Led_5_dataDisplay[]=
{
  {10,0x0040F0},       //
  {10,0x0040F0},      //
};

LED_DATA_TYPE Led_6_dataDisplay[]=
{
  {10,0x00F020},       //
  {10,0x00F020},      //
};

LED_DATA_TYPE Led_7_dataDisplay[]=
{
  {10,0xF00050},       //
  {10,0xF00050},      //
};

LED_DATA_TYPE Led_8_dataDisplay[]=
{
  {10,0x00FF00},       //
  {10,0x000000},      //
};

LED_DATA_TYPE Led_9_dataDisplay[]=
{
  {10,0xF002FA},       //
  {10,0xF002FA},      //
};

LED_DATA_TYPE Led_10_dataDisplay[]=
{
  {10,0xF002FA},       //
  {10,0xF002FA},      //
};

LED_DATA_TYPE Led_11_dataDisplay[]=
{
  {10,0xF002FA},       //
  {10,0xF002FA},      //
};

LED_DATA_TYPE Led_12_dataDisplay[]=
{
  {10,0xF002FA},       //
  {10,0xF002FA},      //
};

LED_DATA_TYPE Led_13_dataDisplay[]=
{
  {10,0xF002FA},       //
  {10,0xF002FA},      //
};

LED_DATA_TYPE Led_14_dataDisplay[]=
{
  {10,0xF002FA},       //
  {10,0xF002FA},      //
};

LED_DATA_TYPE Led_15_dataDisplay[]=
{
  {10,0x00ff00},       //
  {10,0x00ff00},      //
};

LED_DATA_TYPE Led_16_dataDisplay[]=
{
  {160,DARK},       //
  
  UP_DOWN

  {100,DARK},
};

LED_DATA_TYPE Led_17_dataDisplay[]=
{
  {140,DARK},       //
  
  UP_DOWN

  {120,DARK},
};

LED_DATA_TYPE Led_18_dataDisplay[]=
{
  {120,DARK},       //
  
  UP_DOWN

  {140,DARK}, 
};

LED_DATA_TYPE Led_19_dataDisplay[]=
{
  {100,DARK},       //
  
  UP_DOWN

  {160,DARK}, 
};

LED_DATA_TYPE Led_20_dataDisplay[]=
{
  {80,DARK},       //
  
  UP_DOWN

  {180,DARK}, 
};

LED_DATA_TYPE Led_21_dataDisplay[]=
{
  {60,DARK},       //
  
  UP_DOWN

  {200,DARK}, 
};

LED_DATA_TYPE Led_22_dataDisplay[]=    //U23
{
  {40,DARK},       //
  
  UP_DOWN

  {220,DARK}, 
};

LED_DATA_TYPE Led_23_dataDisplay[]=     //U24
{
  {20,DARK},       //
  
  UP_DOWN

  {240,DARK}, 
};

////格式 G_R_B  
LED_DATA_TYPE Led_24_dataDisplay[]=
{
//  {2,0x4f4f4f},       //
  {2,0x485848},       //
  {2,0x3f5f3f},       //
  {2,0x386838},       //
  {2,0x2f6f2f},       //
  
  {2,0x287828},       //
  {2,0x1f7f1f},       //
  {2,0x188818},       //
  {2,0x0f8f0f},       //
  {2,0x089808},       //10

  {2,0x019f01},       //
  {2,0x01a801},       //
  {2,0x01af01},       //
  {2,0x01b801},       //
  {2,0x01bf01},       //
  
  {2,0x01c801},       //
  {2,0x01cf01},       //
  {2,0x01d801},       //
  {2,0x01df01},       //
  {2,0x01e801},       //20

  {2,0x01ef01},       //
  {2,0x01f801},       //
  {2,0x01ff01},       //
  {2,0x01ff01},       //
  {2,0x01ff01},       //

  {2,0x01f801},       //
  {2,0x01ef01},       //
  {2,0x01e801},       //
  {2,0x01df01},       //
  {2,0x01d801},       //

  {2,0x01cf01},       //
  {2,0x01c801},       //
  {2,0x01bf01},       //
  {2,0x01b801},       //
  {2,0x01af01},       //

  {2,0x089f08},       //
  {2,0x0f8f0f},       //
  {2,0x1f7f1f},       //
  {2,0x2f6f2f},       //
  {2,0x3f5f3f},       //
  
  {2,0x4f4f4f},       //
  {2,0x4f4f5f},       //
  {2,0x4f4f5f},       //
  {2,0x4f4f5f},       //
//  {2,0x4f4f4f},       //30
//  
//  {2,0x4f4f4f},      //
//  {2,0x4f4f4f}, 
//  {2,0x4f4f4f}, 
//  {2,0x4f4f4f},
//  {2,0x4f4f4f},
};

LED_DATA_TYPE Led_25_dataDisplay[]=
{
  {180,DARK},       //
  
  UP_DOWN

  {80,DARK},
};

LED_DATA_TYPE Led_26_dataDisplay[]=
{  
  UP_DOWN

  {260,DARK}, 
};

LED_DATA_TYPE Led_27_dataDisplay[]=
{
  {200,DARK},       //
  
  UP_DOWN

  {60,DARK},
};

LED_DATA_TYPE Led_28_dataDisplay[]=
{
  {10,0xF002FA},       //
  {10,0xF002FA},      //
};

LED_DATA_TYPE Led_29_dataDisplay[]=
{
  {220,DARK},       //
  
  UP_DOWN

  {40,DARK},
};

LED_DATA_TYPE Led_30_dataDisplay[]=
{
  {240,DARK},       //
  
  UP_DOWN

  {20,DARK},
};

LED_DATA_TYPE Led_31_dataDisplay[]=
{
  {10,0x0010ff},       //
  {10,0x0010ff},      //
};

LED_DATA_TYPE Led_32_dataDisplay[]=
{
  {260,DARK},       //
  
  UP_DOWN
};

LED_DATA_TYPE Led_33_dataDisplay[]=
{
  {280,DARK},       //
  
  UP

};

LED_DATA_TYPE Led_34_dataDisplay[]=
{
  {10,0x10D0FA},       //
  {10,0x10D0FA},      //
};

LED_DATA_TYPE Led_35_dataDisplay[]=
{
  {20,0xF02000},       //
  {20,0x000000},      //
};

LED_DATA_TYPE Led_36_dataDisplay[]=
{
  {10,0x10D0FA},       //
  {10,0x10D0FA},      //
};

LED_DATA_TYPE Led_37_dataDisplay[]=
{
  {10,0x20F000},       //
  {10,0x20F000},      //
};

LED_DATA_TYPE Led_38_dataDisplay[]=
{
  {10,0x10D0FA},       //
  {10,0x10D0FA},      //
};


LED_ACTIVE_TYPE Led_activeTable[]=
{
  {Led_0_dataDisplay,  sizeof(Led_0_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_1_dataDisplay,  sizeof(Led_1_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_2_dataDisplay,  sizeof(Led_2_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_3_dataDisplay,  sizeof(Led_3_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_4_dataDisplay,  sizeof(Led_4_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_5_dataDisplay,  sizeof(Led_5_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_6_dataDisplay,  sizeof(Led_6_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_7_dataDisplay,  sizeof(Led_7_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_8_dataDisplay,  sizeof(Led_8_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_9_dataDisplay,  sizeof(Led_9_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_10_dataDisplay,  sizeof(Led_10_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_11_dataDisplay,  sizeof(Led_11_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_12_dataDisplay,  sizeof(Led_12_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_13_dataDisplay,  sizeof(Led_13_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_14_dataDisplay,  sizeof(Led_14_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_15_dataDisplay,  sizeof(Led_15_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_16_dataDisplay,  sizeof(Led_16_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_17_dataDisplay,  sizeof(Led_17_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_18_dataDisplay,  sizeof(Led_18_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_19_dataDisplay,  sizeof(Led_19_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_20_dataDisplay,  sizeof(Led_20_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_21_dataDisplay,  sizeof(Led_21_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_22_dataDisplay,  sizeof(Led_22_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_23_dataDisplay,  sizeof(Led_23_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_24_dataDisplay,  sizeof(Led_24_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_25_dataDisplay,  sizeof(Led_25_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_26_dataDisplay,  sizeof(Led_26_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_27_dataDisplay,  sizeof(Led_27_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_28_dataDisplay,  sizeof(Led_28_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_29_dataDisplay,  sizeof(Led_29_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_30_dataDisplay,  sizeof(Led_30_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_31_dataDisplay,  sizeof(Led_31_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_32_dataDisplay,  sizeof(Led_32_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_33_dataDisplay,  sizeof(Led_33_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_34_dataDisplay,  sizeof(Led_34_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_35_dataDisplay,  sizeof(Led_35_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_36_dataDisplay,  sizeof(Led_36_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_37_dataDisplay,  sizeof(Led_37_dataDisplay)/sizeof(LED_DATA_TYPE)},
  {Led_38_dataDisplay,  sizeof(Led_38_dataDisplay)/sizeof(LED_DATA_TYPE)},
};

uint8_t Clock_Display_Data_buf[]={1,2,3,4,5};

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//hw_timer_t *timer = NULL;
//
//void Data_High_Time_cb(void *arg);
//void Data_Low_Time_cb(void *arg);

//定义一个单次运行的定时器结构体
//esp_timer_create_args_t Data_High_Time_once_arg = 
//{ .callback = &Data_High_Time_cb, //设置回调函数
//    .arg = NULL, //不携带参数
//    .name = "DataHighTimer" //定时器名字
//};
//
//esp_timer_create_args_t Data_Low_Time_once_arg = 
//{ .callback = &Data_Low_Time_cb, //设置回调函数
//    .arg = NULL, //不携带参数
//    .name = "DataLowTimer" //定时器名字
//};

//esp32 上有 __rdtsc()， 不用计时器就可用很精确的延时, 特别是关掉中断的时候超级准, 像这样 跑 240M 的时候 delay_clock(240) 就是一微秒, 我经常用这个来延时 0.1 微秒用 。。
//static 
__inline void delay_clock(uint32_t ts)
{
  uint32_t start, curr;

  __asm__ __volatile__("rsr %0, ccount" : "=r"(start));
  do
  __asm__ __volatile__("rsr %0, ccount" : "=r"(curr));
  while (curr - start <= ts);
}

void SmartConfig()
{
  WiFi.mode(WIFI_STA);
  Serial.println("\r\nWait for Smartconfig...");
  WiFi.beginSmartConfig();
  while (1)
  {
    Serial.print(".");
    delay(500);                   // wait for a second
    if (WiFi.smartConfigDone())
    {
      Serial.println("SmartConfig Success");
      Serial.printf("SSID:%s\r\n", WiFi.SSID().c_str());
      Serial.printf("PSW:%s\r\n", WiFi.psk().c_str());
      break;
    }
  }
}

bool AutoConfig()
{
    WiFi.begin();
    //如果觉得时间太长可改
    for (int i = 0; i < 20; i++)
    {
        int wstatus = WiFi.status();
        if (wstatus == WL_CONNECTED)
        {
            Serial.println("WIFI SmartConfig Success");
            Serial.printf("SSID:%s", WiFi.SSID().c_str());
            Serial.printf(", PSW:%s\r\n", WiFi.psk().c_str());
            Serial.print("LocalIP:");
            Serial.print(WiFi.localIP());
            Serial.print(" ,GateIP:");
            Serial.println(WiFi.gatewayIP());
            return true;
        }
        else
        {
            Serial.print("WIFI AutoConfig Waiting......");
            Serial.println(wstatus);
            delay(1000);
        }
    }
    Serial.println("WIFI AutoConfig Faild!" );
    return false;
}

void Led_bitData_1_out()
{
  //高0.6us
  digitalWrite(Led_Data_Out_Pin, HIGH);
  //240=1us
  delay_clock(120);
  //低0.6us
  digitalWrite(Led_Data_Out_Pin, LOW);  
  delay_clock(120);
}

void Led_bitData_0_out()
{
  //高0.3us
  digitalWrite(Led_Data_Out_Pin, HIGH);
  delay_clock(48);
  //低0.9us
  digitalWrite(Led_Data_Out_Pin, LOW);  
  delay_clock(190);
}

void Led_bitReset_out()
{
  //低>200us
  digitalWrite(Led_Data_Out_Pin, LOW);  
  delay_clock(100000);
}

void Led_RGB_Data_out(uint32_t grb)
{
  uint32_t tmp;
  uint8_t i;
  
  tmp = grb;
  for(i=0;i<24;i++)
  {
    tmp=tmp<<1;
    if (0x01000000 & tmp)
    {
      Led_bitData_1_out();
    }
    else
    {
      Led_bitData_0_out();
    }
  }
}

uint16_t Data2Sbit(uint8_t data)
{
    uint16_t ret;

    if (data > 9)
    {
        ret = 0;
    }
    else
    {
        ret = Data2SbitMap[data];
    }

    return ret;
}

void Byte2TimeData(uint8_t data, uint8_t *timedata)
{
    if (data < 100)
    {
        timedata[0] = data % 10;    //units
        timedata[1] = data / 10;    //tens
    }
    else
    {
        timedata[0] = 0;
        timedata[1] = 0;            
    }
}

void Time2SpiData(void)
{
    uint16_t tmp16;
    // uint8_t *pt8;

    Byte2TimeData(second, TimeData);
    tmp16 = Data2Sbit(TimeData[0]) << 2;        //second_units
    //pt8 = (uint8_t *)&tmp16;
    SpiDisData[0] &= (~0x0f);
    SpiDisData[1] &= (~0xfc);   
    SpiDisData[0] |= (tmp16 & 0x0f00) >> 8;
    SpiDisData[1] |= (tmp16 & 0xfc);
    tmp16 = Data2Sbit(TimeData[1]);             //second_tens
    SpiDisData[1] &= (~0x03);
    SpiDisData[2] &= (~0xff);
    SpiDisData[1] |= (tmp16 & 0x0300) >> 8;
    SpiDisData[2] |= (tmp16 & 0xff);

    Byte2TimeData(minute, TimeData);
    tmp16 = Data2Sbit(TimeData[0]) << 6;        //minute_units
    SpiDisData[3] &= (~0xff);
    SpiDisData[4] &= (~0xc0);
    SpiDisData[3] |= (tmp16 & 0xff00) >> 8;
    SpiDisData[4] |= (tmp16 & 0xc0);
    tmp16 = Data2Sbit(TimeData[1]) << 4;        //minute_tens    
    SpiDisData[4] &= (~0x3f);
    SpiDisData[5] &= (~0xf0);
    SpiDisData[4] |= (tmp16 & 0x3f00) >> 8;
    SpiDisData[5] |= (tmp16 & 0xf0);

    Byte2TimeData(hour, TimeData);
    tmp16 = Data2Sbit(TimeData[0]) << 2;        //hour_units
    SpiDisData[5] &= (~0x0f);
    SpiDisData[6] &= (~0xfc);
    SpiDisData[5] |= (tmp16 & 0x0f00) >> 8;
    SpiDisData[6] |= (tmp16 & 0xfc);
    tmp16 = Data2Sbit(TimeData[1]);             //hour_tens  
    SpiDisData[6] &= (~0x03);
    SpiDisData[7] &= (~0xff);
    SpiDisData[6] |= (tmp16 & 0x0300) >> 8;
    SpiDisData[7] |= (tmp16 & 0xff);
}

void vspi_Display(uint8_t *databuf,uint8_t len) 
{
  uint8_t i;
  uint8_t *pdata;
  
//  byte data = 0b01010101; // junk data to illustrate usage

  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(VSPI_SS, LOW); //pull SS slow to prep other end for transfer
  if (len > 0)
  {
    pdata = databuf;
    for (i=0; i<len; i++)
    {
      vspi->transfer(*pdata); 
      pdata++;
    }
  } 
  digitalWrite(VSPI_SS, HIGH); //pull ss high to signify end of data transfer
  vspi->endTransaction();
}

void seconds_tick()
{
//  Serial.println("1HZ");
  digitalWrite(Test_Pin, 1^digitalRead(Test_Pin));
  
  second = now.second();
//  Serial.println(now);
//  Serial.println("current time: " + rtc.now().timestamp());

  Time2SpiData();
  vspi_Display(SpiDisData,sizeof(SpiDisData));

//  if (second < 59)
//  {
//    second++;
//  }
//  else
//  {
//    second = 0;
//  }
}

void setup() {
  // put your setup code here, to run once:
  uint8_t i;

  for(i=0;i<Led_amount;i++)
  {
    stepCnt[i]=0;
    startTick[i]=0;
  }

    hour = 0;
    minute = 0;
    second = 0;
    
  Led_activeTable_Size = sizeof(Led_activeTable)/sizeof(Led_activeTable[0]);
  
  Serial.begin(115200);
  
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  vspi = new SPIClass(VSPI);
  vspi->begin();
  
  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(VSPI_SS, OUTPUT); //VSPI SS
 
  pinMode(Led_Data_Out_Pin, OUTPUT);
  pinMode(HV_Ctl_Pin, OUTPUT);
  pinMode(Seconds_Int_Pin, INPUT);
  pinMode(Test_Pin, OUTPUT);
  
  Led_bitReset_out();

//  delay(100);
//  if (!AutoConfig())
//  {
//      SmartConfig();
//  }
//  
  if (!rtc.begin()) 
  {
    Serial.println("Couldn't find RTC");
    while (1) delay(50);
  }

  if (rtc.lostPower()) 
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);

  attachInterrupt(digitalPinToInterrupt(Seconds_Int_Pin), seconds_tick, FALLING);
  
  digitalWrite(HV_Ctl_Pin, HIGH);
  
  //开始创建一个单次周期的定时器并且执行
  //esp_err_t err = esp_timer_create(&Data_High_Time_once_arg, &LED_Data_High_Time_handle);
  //err = esp_timer_start_once(LED_Data_High_Time_handle, 10 * 1000 * 1000);

  //timer = timerBegin(0, 80, true);                  // 选择timer0，分频系数为80，向上计数.( tick = 1us)

  //timerAttachInterrupt(timer, &Led_DataTime_Interrupt, true);  // 绑定中断函数
  //timerAlarmWrite(timer, 1000 * 1000, false); // 设置报警保护函数
  //timerAlarmEnable(timer);                          // 使能报警器
  
}

void loop() {
  // put your main code here, to run repeatedly:
uint8_t i;
static uint32_t ledtick=0;
//static uint32_t distick=0;

  now = rtc.now();
  for (i=0;i<Led_activeTable_Size;i++)
  {
    if (Led_activeTable[i].pLedData[stepCnt[i]].holdTick <= (ledtick - startTick[i]))
    {
      stepCnt[i]++;
      startTick[i]=ledtick;
      
      if(stepCnt[i] >= Led_activeTable[i].itemSize)
      {
        stepCnt[i]=0;
      }
    }
    Led_RGB_Data_out(Led_activeTable[i].pLedData[stepCnt[i]].rgb);
  }
  ledtick++;

//  //格式 G_R_B    
//  Led_RGB_Data_out(0x002200);   //u1
  
  Led_bitReset_out();
  
  delay(10);//延迟10ms
}


//void Data_High_Time_cb(void *arg) {
//  digitalWrite(Led_Data_Out_Pin, HIGH); 
//}
//
//void Data_Low_Time_cb(void *arg) {
//  digitalWrite(Led_Data_Out_Pin, LOW); 
//}
//
//void IRAM_ATTR Led_DataTime_Interrupt() { // 中断函数
//
//}

//hw_timer_t * timerBegin(uint8_t num, uint16_t divider, bool countUp){
//    if(num > 3){
//        return NULL;
//    }
//    * timer = &hw_timer[num];
//    if(timer->group) {
//        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_TIMERGROUP1_CLK_EN);
//        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_TIMERGROUP1_RST);
//        TIMERG1.int_ena.val &= ~BIT(timer->timer);
//    } else {
//        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_TIMERGROUP_CLK_EN);
//        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_TIMERGROUP_RST);
//        TIMERG0.int_ena.val &= ~BIT(timer->timer);
//    }
//    timer->dev->config.enable = 0;
//    timerSetDivider(timer, divider);
//    timerSetCountUp(timer, countUp);
//    timerSetAutoReload(timer, false);
//    timerAttachInterrupt(timer, NULL, false);
//    timerWrite(timer, 0);
//    timer->dev->config.enable = 1;
//    addApbChangeCallback(timer, _on_apb_change);
//    return timer;
//}
