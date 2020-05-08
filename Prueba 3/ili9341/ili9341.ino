//*********************************************
/* Librería para el uso de la pantalla ILI9341 en modo 8 bits
 * IE3027: Electrónica Digital 2 - 2019
   Basado en el código de martinayotte - https://www.stm32duino.com/viewtopic.php?t=637
   Adaptación, migración y creación de nuevas funciones: Pablo Mazariegos y José Morales
   Con ayuda de: Pablo Mazariegos
  
*/


//Version de Juan Gerardo y Jeffrey 
//------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "bitmaps.h"
#include "font.h"
#include "lcd_registers.h"

#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};
//------------------------------------------------------------------
// Functions Prototypes
//------------------------------------------------------------------
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(String text, int x, int y, int fontSize, int color, int background);

void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset);
void make_floor();
//------------------------------------------------------------------
// Inicialización
//------------------------------------------------------------------
//                              variables 1 jugador
//-------------------------------------------------------------------
uint8_t tupuk = 1;
uint8_t bottomok = 1;
uint8_t shut=60;
uint8_t cubod = 150;
uint8_t cubod2;
uint8_t cubod3;
uint8_t cubod4;
uint8_t cubod5;
uint8_t cubod6;
uint8_t cubdo7;

uint8_t spikescroll = -3;
uint8_t spikescroll2 = +2;
uint8_t animc = 0;
uint8_t animsp = 0;
uint8_t animcstate = 0;
uint8_t animspstate = 0;
uint8_t spikeon = 0;
uint8_t Start = false;
uint8_t Points = 0;
uint8_t grounded = true;
uint8_t caidaradioint = 0;
float   caidaradio = 0;
uint8_t botomestadopre = 0;
uint8_t botomestadopre2= 0;
const int buttonPin = PUSH1;
const int buttonPin2 = PUSH2;
uint8_t bottomestado2 = 0;
uint8_t bottomestado = 0;
uint8_t xP = 319;
uint8_t yP = 100;
uint8_t yB = 176;
static uint8_t x = 1;
static uint8_t x2 = 1;
static uint8_t x3 = 1;
static uint8_t x4 = 1;
static uint8_t xy = 0;
static uint8_t x5 = 1;
static uint8_t x6 = 1;
static uint8_t x7 = 1;
static uint8_t x8 = 1;

static uint8_t xx = 0;
static uint8_t y=0;
static uint8_t sa=0;
static uint8_t n1=0;
static uint8_t n2=0;
static uint8_t n3=0;


static uint8_t xb ;
static uint8_t xa ;
static uint8_t xp ;
static uint8_t xm ;
//----------------------------------------------------------------------------------
//                            Variables multijugador
//---------------------------------------------------------------------------------
uint8_t tupukm = 1;
uint8_t bottomokm = 1;
uint8_t shutm=60;
uint8_t cubodm = 150;
uint8_t cubod2m;
uint8_t cubod3m;
uint8_t spikescrollm = -3;
uint8_t spikescroll2m = +2;
uint8_t animcm = 0;
uint8_t animspm = 0;
uint8_t animcstatem = 0;
uint8_t animspstatem = 0;
uint8_t spikeonm = 0;

uint8_t Pointsm = 0;
uint8_t groundedm = true;
uint8_t caidaradiointm = 0;
float   caidaradiom = 0;
uint8_t botomestadoprem = 0;
uint8_t botomestadopre2m= 0;
const int buttonPinm = PUSH1;
const int buttonPin2m = PUSH2;
uint8_t bottomestado2m = 0;
uint8_t bottomestadom = 0;
uint8_t xPm = 319;
uint8_t yPm = 5;
uint8_t yBm = 81;
static uint8_t xM = 1;
static uint8_t x2m = 1;
static uint8_t x3m = 1;
static uint8_t x4m = 1;
static uint8_t xym = 0;
static uint8_t xxm = 0;
static uint8_t ym=0;
static uint8_t sam=0;
static uint8_t n1m=0;

static uint8_t xbm ;
static uint8_t xam ;
static uint8_t xpm ;
static uint8_t xmm ;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  Serial.begin(9600);
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  Serial.println("Inicio");
  LCD_Init();
  LCD_Clear(0x00);
  empezarjuego();
  while (!Start) {
    pinMode(buttonPin2, INPUT_PULLUP); 
    bottomestado2 = digitalRead(buttonPin2);
    bottomestado = digitalRead(buttonPin);
    if (bottomestado == LOW) {
      Start = true;
      xp=1;
      xm=0;
    }
    if (bottomestado2 == LOW) {
      Start = true;
      xm=1;
      xp=0;
  hacerpiso();
    }
  }
  FillRect(0, 0, 319, 219, 0x421b);
  String soloplayer = "Level 1";
  LCD_Print(soloplayer, 30, 20, 2, 0xffff, 0xD090);
 
  hacerpiso();

  //---------------------------------------------------------------
  

}
//------------------------------------------------------------------
// Loop Infinito
//------------------------------------------------------------------

void loop() {
  if(xm==1){
    goto multi;
  }
  if(xp==1){
   FillRect(125, 146, 20, 20, 0x421b);
    FillRect(130, 146, 20, 20, 0x421b);
    FillRect(140, 146, 20, 20, 0x421b);
    FillRect(140, 187, 20, 20, 0x421b);
    FillRect(140, 190, 20, 20, 0x421b);
  cubod2 = cubod + 100;
  xb = cubod2 + 200;
  cubod4 = cubod + 150;
  xa+=-4;
  if (yB >= 176) {
    grounded = true;
  }
  else if (yB < 176) {
    grounded = false;
  }
  Points ++;
  Serial.println(Points);
main_game:
 pinMode(buttonPin2, INPUT_PULLUP); 
  bottomestado = digitalRead(buttonPin);

 bottomestado2 = digitalRead(buttonPin2);
     

  
  if (yB < 176) {
    bottomok = 1;
  }
  else if (yB > 176) {
    bottomok = 0;
  }
  if (yB > 100) {
    tupuk = 1;
  }
  else if (yB < 100) {
    tupuk = 0;
  }


  ///Jump

  jump();
  disparo();
 botomestadopre = bottomestado;
 botomestadopre2= bottomestado2;
  //spikes
  cubod += spikescroll;
  if(n1>=1){
    
    x=0;
    x2=0;
    x3=0;
    x4=0;
    goto nivel2;
    }
  if (cubod < 320) {
    if( shut>=cubod && y==1){
       FillRect(cubod, 190, 39, 23, 0x421b);
       x=0;
  
    } 
    if (x==1){
    LCD_Sprite(cubod, 190, 34, 23, enemigo1, 3, animsp, 0, 0);
    FillRect(cubod + 40, 190, 16, 24, 0x421b);
    }
  }
 if (cubod <= 5) {
    
   x=1;
  }
  
  if( shut>=cubod2 && y==1){
    FillRect(cubod2, 190, 39, 23, 0x421b);
    x2=0;
  }
  if (x2==1){
  if (cubod2 < 319) {
    LCD_Sprite(cubod2, 190, 34, 23, enemigo1, 3, animsp, 0, 0);
    FillRect(cubod2 + 40, 190, 16, 24, 0x421b);
  }
  }
  else if (cubod2 <= 5) {
   x2=1; 
  }
  if( shut>=xb && y==1 && yB<=160 ){
   LCD_Bitmap(xb, 160, 18, 20, tile);
   FillRect(xb + 16, 160, 16, 24, 0x421b);
   FillRect(xb , 146, 19, 14, 0x421b);
    x3=0;
  }
  if(x3==1){
  if (xb < 319) {
      
      LCD_Bitmap(xb, 160, 18, 20, tile);
      FillRect(xb + 16, 160, 18, 20, 0x421b);
      LCD_Sprite(xb, 146, 19, 14, enemigo2, 2, animsp, 0, 0);
      FillRect(xb + 19, 146, 19, 14, 0x421b);
      FillRect(0, 160, 16, 24, 0x421b);
      FillRect(0, 146, 19, 14, 0x421b);
      x4=0;
      if (xa< xb){
      LCD_Bitmap(xa, 146, 8, 8, shut2);
      FillRect(xa+8, 146, 8, 8, 0x421b);
      x4=1;
      }  
      }
  }

  else if (xb <= 5) {
    x3=1;
    n1+=1;
    
  }


  //animaciones
 
 nivel2:
 if(n1>=1){
  cubod4 = cubod + 150;
  cubod5 = cubod4 +100;
  if (cubod4 < 320){
    LCD_Sprite(cubod4, 177, 16, 17, flame, 1, 0, 0, 0);
    FillRect(cubod4 + 15, 177, 16, 17, 0x421b);
    LCD_Sprite(cubod4, 194, 16, 17, flame, 1, 0, 0, 0);
    FillRect(cubod4 + 15, 194, 16, 17, 0x421b);
    }

   if( shut>=cubod5 && y==1 && yB<=160 ){
   LCD_Bitmap(cubod5, 160, 18, 20, tile);
   FillRect(cubod5 + 16, 160, 16, 24, 0x421b);
   FillRect(cubod5 , 137, 34, 23, 0x421b);
   FillRect(cubod5+100, 190, 34, 24, 0x421b);
    x5=0;
  }
  if(x5==1){
  if (cubod5 < 319) {
      
      LCD_Bitmap(cubod5, 160, 18, 20, tile);
      LCD_Bitmap(cubod5+18, 160, 18, 20, tile);
      FillRect(cubod5 + 36, 160, 18, 20, 0x421b);
      LCD_Sprite(cubod5, 137, 34, 23, enemigo1, 3, animsp, 0, 0);
      FillRect(cubod5 + 34, 137, 34, 23, 0x421b);
      FillRect(0, 160, 16, 24, 0x421b);
      FillRect(0, 137, 34, 23, 0x421b);
      LCD_Sprite(cubod5+100, 190, 34, 23, enemigo1, 3, animsp, 0, 0);
      FillRect(cubod5+134, 190, 34, 24, 0x421b);
      
      x6=0;
      if (xa< cubod5){
      LCD_Bitmap(xa, 137, 14, 14, shut3);
      FillRect(xa+14, 137, 14, 14, 0x421b);
      x6=1;
      }  
      }
  }
  else if (cubod5 <= 5) {
    x5=1;
    n2+=1;
  }
 
  }
  if (n2>=1){
    x5=0;
    x6=0;
    x4=0;
    
    goto nivel3;
  }
   nivel3:{
    if(n2>=1){
    if(x7==1){
 
      LCD_Sprite(200, 146, 57, 32, boss, 1, 0, 1, 0);
      
      FillRect(0, 160, 32, 57, 0x421b);
      FillRect(0, 146, 32, 57, 0x421b);
      x8=0;
      if (xa< 200){
      LCD_Bitmap(xa, 146, 8, 8, shut2);
      FillRect(xa+8, 146, 8, 8, 0x421b);
      x8=1;
      }  
      }
  }

  else if (xb <= 5) {
    x7=1; 
  }
  
    }
   

   
  

 animate();

  collision();
 disparo();

 }


//-----------------------------------------------------------------------------------
//multijugador
//-----------------------------------------------------------------------------------
multi:
if (xm==1){
    hacerpiso2();
    
    FillRect(125, 146, 20, 20, 0x421b);
    FillRect(130, 146, 20, 20, 0x421b);
    FillRect(140, 146, 20, 20, 0x421b);
    FillRect(140, 187, 20, 20, 0x421b);
    FillRect(140, 190, 20, 20, 0x421b);
  cubod2 = cubod + 100;
  xb = cubod2 + 200;

  xa+=-6;
//-----------------------------------------------------------------------------------
//                        un jugador
//-----------------------------------------------------------------------------------
  if (yB >= 176) {
    grounded = true;
  }
  else if (yB < 176) {
    grounded = false;
  }
  Points ++;
  Serial.println(Points);
//-----------------------------------------------------------------------------------
//                        multijugador
//-----------------------------------------------------------------------------------
 if (yBm >= 81) {
    groundedm = true;
  }
  else if (yBm < 81) {
    groundedm = false;
  }
  Pointsm ++;
  Serial.println(Pointsm);
//-----------------------------------------------------------------------------------
//                        un jugador
//-----------------------------------------------------------------------------------
 pinMode(buttonPin2, INPUT_PULLUP); 
  bottomestado = digitalRead(buttonPin);

 bottomestado2 = digitalRead(buttonPin2);

  if (yB < 176) {
    bottomok = 1;
  }
  else if (yB > 176) {
    bottomok = 0;
  }
  if (yB > 100) {
    tupuk = 1;
  }
  else if (yB < 100) {
    tupuk = 0;
  }
//-----------------------------------------------------------------------------------
//                        multijugador
//-----------------------------------------------------------------------------------
pinMode(buttonPin2m, INPUT_PULLUP); 
  bottomestadom = digitalRead(buttonPinm);

 bottomestado2m = digitalRead(buttonPin2m);

  if (yBm < 81) {
    bottomokm = 1;
  }
  else if (yBm > 81) {
    bottomokm = 0;
  }
  if (yBm > 5) {
    tupukm = 1;
  }
  else if (yBm < 5) {
    tupukm = 0;
  }
  //--------------------------------------------------------------------------------
  jump();
  //------------------------------------------------------------------------------
  jump2();
  //----------------------------------------------------------------------------
  disparo();
  //----------------------------------------------------------------------------
 botomestadopre = bottomestado;
 botomestadopre2= bottomestado2;
 //---------------------------------------------------------------------------------
 botomestadoprem = bottomestadom;
 botomestadopre2m= bottomestado2m;
 //-------------------------------------------------------------------------------
  //spikes
  cubod += -5;
  
  if (cubod < 320) {
    if( shut>=cubod && y==1){
       FillRect(cubod, 190, 39, 23, 0x421b);
       x=0;
  
    } 
    if (x==1){
    LCD_Sprite(cubod, 190, 34, 23, enemigo1, 3, animsp, 0, 0);
    FillRect(cubod + 40, 190, 16, 24, 0x421b);
    }
  }
 if (cubod <= 5) {
    
   x=1;
  }
  
  if( shut>=cubod2 && y==1){
    FillRect(cubod2, 190, 39, 23, 0x421b);
    x2=0;
  }
  if (x2==1){
  if (cubod2 < 319) {
    LCD_Sprite(cubod2, 190, 34, 23, enemigo1, 3, animsp, 0, 0);
    FillRect(cubod2 + 40, 190, 16, 24, 0x421b);
  }
  }
  else if (cubod2 <= 5) {
   x2=1; 
  }
  if( shut>=xb && y==1 && yB<=160 ){
   LCD_Bitmap(xb, 160, 18, 20, tile);
   FillRect(xb + 16, 160, 16, 24, 0x421b);
   FillRect(xb , 146, 19, 14, 0x421b);
    x3=0;
  }
  if(x3==1){
  if (xb < 319) {
      
      LCD_Bitmap(xb, 160, 18, 20, tile);
      FillRect(xb + 16, 160, 18, 20, 0x421b);
      LCD_Sprite(xb, 146, 19, 14, enemigo2, 2, animsp, 0, 0);
      FillRect(xb + 19, 146, 19, 14, 0x421b);
      FillRect(0, 160, 16, 24, 0x421b);
      FillRect(0, 146, 19, 14, 0x421b);
      x4=0;
      if (xa< xb){
      LCD_Bitmap(xa, 146, 8, 8, shut2);
      FillRect(xa+8, 146, 8, 8, 0x421b);
      x4=1;
      }  
      }
  }

  else if (xb <= 5) {
    x3=1;
    n1+=1;   
  }
  animate();

  collision();
 
 disparo();
 if(n1==5){
    gameover();
    n1=0;
 }

//-------------------------------------------------------------------------------
//                           multijugador
//----------------------------------------------------------------------------
cubodm += -5;
cubod2m = cubodm + 100;
  xbm = cubod2m + 200;
   xam+=-6;
  if (cubodm < 320) {
    if( shutm>=cubodm && ym==1){
       FillRect(cubodm, 95, 39, 23, 0x421b);
       xM=0;
  
    } 
    if (xM==1){
    LCD_Sprite(cubodm, 95, 34, 23, enemigo1, 3, animsp, 0, 0);
    FillRect(cubodm + 40, 95, 16, 24, 0x421b);
    }
  }
 if (cubodm <= 5) {
    
   xM=1;
  }
  
  if( shutm>=cubod2m && ym==1){
    FillRect(cubod2m, 95, 39, 23, 0x421b);
    x2m=0;
  }
  if (x2m==1){
  if (cubod2m < 319) {
    LCD_Sprite(cubod2m, 95, 34, 23, enemigo1, 3, animsp, 0, 0);
    FillRect(cubod2m + 40, 95, 16, 24, 0x421b);
  }
  }
  else if (cubod2m <= 5) {
   x2m=1; 
  }
  
  if( shutm>=xbm && ym==1 && yBm<=65 ){
   LCD_Bitmap(xbm, 65, 18, 20, tile);
   FillRect(xbm + 16, 65, 16, 24, 0x421b);
   FillRect(xbm , 51, 19, 14, 0x421b);
    x3m=0;
  }
  if(x3m==1){
  if (xbm< 319) {
      
      LCD_Bitmap(xbm, 65, 18, 20, tile);
      FillRect(xbm + 16, 65, 18, 20, 0x421b);
      LCD_Sprite(xbm, 51, 19, 14, enemigo2, 2, animsp, 0, 0);
      FillRect(xbm + 19, 51, 19, 14, 0x421b);
      FillRect(0, 65, 16, 24, 0x421b);
      FillRect(0, 51, 19, 14, 0x421b);
      x4m=0;
      if (xam< xbm){
      LCD_Bitmap(xam, 51, 8, 8, shut2);
      FillRect(xam+8, 51, 8, 8, 0x421b);
      x4m=1;
      }  
      }
  }

  else if (xbm <= 5) {
    x3m=1;
    n1m+=1;   
  }
  
  animate();

  collision();
 disparo2();
 disparo();
 if(n1m==5){
    gameover();
    n1m=0;
 }
}
}
//--------------------------------------------------------
//                       fin de programacon
//------------------------------------------------------------------
// accion para inicializar LCD
//------------------------------------------------------------------
void LCD_Init(void) {
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(DPINS[i], OUTPUT);
  }
  //**************
  // Secuencia de Inicialización
  //**************
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(150);
  digitalWrite(LCD_CS, LOW);
  //**************
  LCD_CMD(0xE9);  // SETPANELRELATED
  LCD_DATA(0x20);
  //**************
  LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
  delay(100);
  //**************
  LCD_CMD(0xD1);    // (SETVCOM)
  LCD_DATA(0x00);
  LCD_DATA(0x71);
  LCD_DATA(0x19);
  //**************
  LCD_CMD(0xD0);   // (SETPOWER)
  LCD_DATA(0x07);
  LCD_DATA(0x01);
  LCD_DATA(0x08);
  //**************
  LCD_CMD(0x36);  // (MEMORYACCESS)
  LCD_DATA(0x40 | 0x80 | 0x20 | 0x08); // LCD_DATA(0x19);
  //**************
  LCD_CMD(0x3A); // Set_pixel_format (PIXELFORMAT)
  LCD_DATA(0x05); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
  //**************
  LCD_CMD(0xC1);    // (POWERCONTROL2)
  LCD_DATA(0x10);
  LCD_DATA(0x10);
  LCD_DATA(0x02);
  LCD_DATA(0x02);
  //**************
  LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
  LCD_DATA(0x00);
  LCD_DATA(0x35);
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x02);
  //**************
  LCD_CMD(0xC5); // Set Frame Rate (VCOMCONTROL1)
  LCD_DATA(0x04); // 72Hz
  //**************
  LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
  LCD_DATA(0x01);
  LCD_DATA(0x44);
  //**************
  LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
  LCD_DATA(0x04);
  LCD_DATA(0x67);
  LCD_DATA(0x35);
  LCD_DATA(0x04);
  LCD_DATA(0x08);
  LCD_DATA(0x06);
  LCD_DATA(0x24);
  LCD_DATA(0x01);
  LCD_DATA(0x37);
  LCD_DATA(0x40);
  LCD_DATA(0x03);
  LCD_DATA(0x10);
  LCD_DATA(0x08);
  LCD_DATA(0x80);
  LCD_DATA(0x00);
  //**************
  LCD_CMD(0x2A); // Set_column_address 320px (CASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x3F);
  //**************
  LCD_CMD(0x2B); // Set_page_address 480px (PASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0xE0);
  //  LCD_DATA(0x8F);
  LCD_CMD(0x29); //display on
  LCD_CMD(0x2C); //display on

  LCD_CMD(ILI9341_INVOFF); //Invert Off
  delay(120);
  LCD_CMD(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  LCD_CMD(ILI9341_DISPON);    //Display on
  digitalWrite(LCD_CS, HIGH);
}
//------------------------------------------------------------------
// accion para enviar comandos a la LCD
//------------------------------------------------------------------
void LCD_CMD(uint8_t cmd) {
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = cmd;
  digitalWrite(LCD_WR, HIGH);
}
//------------------------------------------------------------------
// accion  para enviar datos a la LCD - parámetro (dato)
//------------------------------------------------------------------
void LCD_DATA(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = data;
  digitalWrite(LCD_WR, HIGH);
}
//------------------------------------------------------------------
// accion definir rango de direcciones de memoria
//------------------------------------------------------------------
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  LCD_CMD(0x2a); // Set_column_address 4 parameters
  LCD_DATA(x1 >> 8);
  LCD_DATA(x1);
  LCD_DATA(x2 >> 8);
  LCD_DATA(x2);
  LCD_CMD(0x2b); // Set_page_address 4 parameters
  LCD_DATA(y1 >> 8);
  LCD_DATA(y1);
  LCD_DATA(y2 >> 8);
  LCD_DATA(y2);
  LCD_CMD(0x2c); // Write_memory_start
}
//------------------------------------------------------------------
// Función para borrar la pantalla - parámetros (color)
//------------------------------------------------------------------
void LCD_Clear(unsigned int c) {
  unsigned int x, y;
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  SetWindows(0, 0, 319, 239); // 479, 319);
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);
    }
  digitalWrite(LCD_CS, HIGH);
}
//------------------------------------------------------------------
// Función para dibujar una línea horizontal - parámetros ( coordenada x, cordenada y, longitud, color)
//------------------------------------------------------------------
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + x;
  SetWindows(x, y, l, y);
  j = l;// * 2;
  for (i = 0; i < l; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//------------------------------------------------------------------
// dibujar una línea vertical  
//------------------------------------------------------------------
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + y;
  SetWindows(x, y, x, l);
  j = l; //* 2;
  for (i = 1; i <= j; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//------------------------------------------------------------------
// Función para dibujar un rectángulo - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//*------------------------------------------------------------------
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  H_line(x  , y  , w, c);
  H_line(x  , y + h, w, c);
  V_line(x  , y  , h, c);
  V_line(x + w, y  , h, c);
}
//------------------------------------------------------------------
// relleno de rectangulos
//------------------------------------------------------------------
//void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
//  unsigned int i;
//  for (i = 0; i < h; i++) {
//    H_line(x  , y  , w, c);
//    H_line(x  , y+i, w, c);
//  }
//}

void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + w;
  y2 = y + h;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = w * h * 2 - 1;
  unsigned int i, j;
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);

      //LCD_DATA(bitmap[k]);
      k = k - 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//------------------------------------------------------------------
// Función para dibujar texto 
//------------------------------------------------------------------
void LCD_Print(String text, int x, int y, int fontSize, int color, int background) {
  int fontXSize ;
  int fontYSize ;

  if (fontSize == 1) {
    fontXSize = fontXSizeSmal ;
    fontYSize = fontYSizeSmal ;
  }
  if (fontSize == 2) {
    fontXSize = fontXSizeBig ;
    fontYSize = fontYSizeBig ;
  }

  char charInput ;
  int cLength = text.length();
  Serial.println(cLength, DEC);
  int charDec ;
  int c ;
  int charHex ;
  char char_array[cLength + 1];
  text.toCharArray(char_array, cLength + 1) ;
  for (int i = 0; i < cLength ; i++) {
    charInput = char_array[i];
    Serial.println(char_array[i]);
    charDec = int(charInput);
    digitalWrite(LCD_CS, LOW);
    SetWindows(x + (i * fontXSize), y, x + (i * fontXSize) + fontXSize - 1, y + fontYSize );
    long charHex1 ;
    for ( int n = 0 ; n < fontYSize ; n++ ) {
      if (fontSize == 1) {
        charHex1 = pgm_read_word_near(smallFont + ((charDec - 32) * fontYSize) + n);
      }
      if (fontSize == 2) {
        charHex1 = pgm_read_word_near(bigFont + ((charDec - 32) * fontYSize) + n);
      }
      for (int t = 1; t < fontXSize + 1 ; t++) {
        if (( charHex1 & (1 << (fontXSize - t))) > 0 ) {
          c = color ;
        } else {
          c = background ;
        }
        LCD_DATA(c >> 8);
        LCD_DATA(c);
      }
    }
    digitalWrite(LCD_CS, HIGH);
  }
}
//------------------------------------------------------------------
// dibujar imagenes colores (Bitmap) Formato (Color 16bit R 5bits G 6bits B 5bits)
//------------------------------------------------------------------
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + width;
  y2 = y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = 0;
  unsigned int i, j;

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k + 1]);
      //LCD_DATA(bitmap[k]);
      k = k + 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//------------------------------------------------------------------
//acciones sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta
//------------------------------------------------------------------
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 =   x + width;
  y2 =    y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  int k = 0;
  int ancho = ((width * columns));
  if (flip) {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width - 1 - offset) * 2;
      k = k + width * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k - 2;
      }
    }
  } else {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width + 1 + offset) * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k + 2;
      }
    }


  }
  digitalWrite(LCD_CS, HIGH);
}
//------------------------------------------------------------------
// acciones del juego
//------------------------------------------------------------------
void jump() { 
  caidaradio = 0 ;
  caidaradioint = int(caidaradio);
  yB += caidaradioint;
  if (grounded == true) {
    
         
    LCD_Sprite(30, yB+12, 34, 28, megacor1, 3, animc, 1, 1);
  
    FillRect(0, 187, 32, 36, 0x421b);
    FillRect(0, 146, 19, 14, 0x421b);
   
     xy=0;
    xx=1;
    
    if (bottomestado == LOW && tupuk == 1 && botomestadopre == HIGH) {
      caidaradio = -50;
      
    }
    caidaradioint = int(caidaradio);
    yB += caidaradioint;
  }
  if (grounded == false) {
    
          
    if ( bottomestado == HIGH && bottomok == 1 ) {

      caidaradio = 6;
    }
    caidaradioint = int(caidaradio);
    yB += caidaradioint;
    LCD_Sprite(30, yB, 32, 36, megasalto1, 4, animc, 1, 1);
    FillRect(30, 180, 34, 28, 0x421b);
    FillRect(30, yB - 32, 32, 36, 0x421b);
    
    xy=1;
    xx=0;
    
 
  }
}
//-----------------------------------------------------------------------------------
//                                     jump 2ndo jugador
//-----------------------------------------------------------------------------------
void jump2(){
   
  caidaradiom = 0 ;
  caidaradiointm = int(caidaradiom);
  yBm += caidaradiointm;
  if (groundedm == true) {
    
         
    LCD_Sprite(30, yBm+12, 34, 28, megacor1, 3, animc, 1, 1);
  
    FillRect(0, 92, 32, 36, 0x421b);
    FillRect(0, 51, 19, 14, 0x421b);
   
     xym=0;
    xxm=1;
    
    if (bottomestadom == LOW && tupukm == 1 && botomestadoprem == HIGH) {
      caidaradiom = -50;
      
    }
    caidaradiointm = int(caidaradiom);
    yBm += caidaradiointm;
  }
  if (groundedm == false) {
    
          
    if ( bottomestadom == HIGH && bottomokm == 1 ) {

      caidaradiom = 6;
    }
    caidaradiointm = int(caidaradiom);
    yBm += caidaradiointm;
    LCD_Sprite(30, yBm, 32, 36, megasalto1, 4, animc, 1, 1);
    FillRect(30, 85, 34, 28, 0x421b);
    FillRect(30, yBm - 32, 32, 36, 0x421b);
    
    xym=1;
    xxm=0;
    
 
  }
}

//---------------------------------------------------------------------------------------
void hacerpiso() {

  for (int x = 0; x < 319; x++) {
    LCD_Bitmap(x, 219, 18, 20, tile);
    x += 16;
  }
}
void hacerpiso2() {

  for (int x = 0; x < 319; x++) {
    LCD_Bitmap(x, 119, 18, 20, tile);
    x += 16;
  }
}
//--------------------------------------------------------------------------------------
//                                       un jugador
//-------------------------------------------------------------------------------------
  void disparo(){
    if(xx==1 && xy==0){
    if(bottomestado2==LOW){
       y=1;}
     if(y==1){
       
     if (shut <140) {
    shut  += 3;
    LCD_Sprite(shut, 190, 16, 13, shoot, 1, 0, 0, 0);
    FillRect(shut -16, 190, 16, 13, 0x421b);
    
  }
    if (shut >= 139) {
    shut = 60;
    y=0;}
  }
    }
   //------------------------------------------------------------
   if(xx==0 && xy==1 && yB<=130){
    if(bottomestado2==LOW){
       y=1;}
     if(y==1){
       
     if (shut <140) {
    shut  += 3;
    LCD_Sprite(shut, yB+15, 16, 13, shoot, 1, 0, 0, 0);
    FillRect(shut -16, yB+15, 16, 13, 0x421b);
    FillRect(shut -16, yB+15, 16, 13, 0x421b);
    
  }
    if (shut >= 139) {
    shut = 60;
    y=0;}
  }
    }
   
  }
//-------------------------------------------------------------------------------------
//                               multijugador
//------------------------------------------------------------------------------------
 void disparo2(){
    if(xxm==1 && xym==0){
    if(bottomestado2m==LOW){
       ym=1;}
     if(ym==1){
       
     if (shutm <140) {
    shutm  += 3;
    LCD_Sprite(shutm, 95, 16, 13, shoot, 1, 0, 0, 0);
    FillRect(shutm -16, 95, 16, 13, 0x421b);
    
  }
    if (shutm >= 139) {
    shutm = 60;
    ym=0;}
  }
    }
   //------------------------------------------------------------
   if(xxm==0 && xym==1 && yBm<=35){
    if(bottomestado2m==LOW){
       ym=1;}
     if(ym==1){
       
     if (shutm <140) {
    shutm  += 3;
    LCD_Sprite(shutm, yB+15, 16, 13, shoot, 1, 0, 0, 0);
    FillRect(shutm -16, yB+15, 16, 13, 0x421b);
    FillRect(shutm -16, yB+15, 16, 13, 0x421b);
    
  }
    if (shutm >= 139) {
    shutm = 60;
    ym=0;}
  }
    }
   
  }

//---------------------------------------------------------------------------------------
void empezarjuego() {
  FillRect(0, 0, 319, 219, 0x00ff);
  String Start1 = "Welcome to:";
  String Start2 = "MEGA-MAN";
  String Player1 = "Single Player";
  String Player2 = "Multiplayer";
  LCD_Print(Start1, 50, 50, 1, 0xFF00, 0x0000);
  LCD_Print(Start2, 10, 70, 2, 0xffff, 0xD082);
  LCD_Print(Player1, 100, 130, 1, 0xffff, 0xD082);
  LCD_Print(Player2, 100, 150, 1, 0xffff, 0xD082);
  cubod = 319;
}

 



void collision() {
  uint8_t cuboy1 = yB;
  uint8_t cuboy2 = yB + 23;
  uint8_t cubox1 = 40;
  uint8_t cubox2 = 40;
  uint8_t cubox3 = 40;
  
  uint8_t cuboenex1 = cubod;
  uint8_t cuboenex2 = cubod + 32;
  uint8_t cuboenex3 = cubod +100;
  uint8_t cuboenex4 = cubod + 132;
  uint8_t cuboenex5 = xb;
  uint8_t cuboenex6 = xb+19;
  uint8_t cuboenex7 = xa;
  uint8_t cuboenex8 = xa+8;
//nivel 2 
  uint8_t cuboenex9=cubod +150;
  uint8_t cuboenex10=cubod +163;
  uint8_t cuboenex11=cuboenex10 +100;
  uint8_t cuboenex12=cuboenex10 +123;

  uint8_t cuboeney1 = 187;
  uint8_t cuboeney2 = 187 + 26;
  uint8_t cuboeney3 = 155;
 //------------------------------------------------------------------
// fin del juego
//------------------------------------------------------------------

 if ((cubox2 >= cuboenex1) && (cubox1 <= cuboenex2) && (cuboy2 >= cuboeney1)&&(x==1)) {
    gameover();
 }
 if ((cubox2 >= cuboenex3) && (cubox1 <= cuboenex4) && (cuboy2 >= cuboeney1)&&(x2==1)) {
    gameover();
    }
 if ((cubox2 >= cuboenex5) && (cubox1 <= cuboenex6) && (yB <= cuboeney3)&&(x4==1)) {
    gameover();
    }
 if ((cubox2 >= cuboenex7) && (cubox1 <= cuboenex8) && (yB <= cuboeney3)&&(x3==1)) {
    gameover();
    }
  if ((cubox2 >= cuboenex9) && (cubox1 <= cuboenex10) && (cuboy2 >= cuboeney1)&&(n1>=1)) {
    gameover();
    }
   if ((cubox2 >= cuboenex11) && (cubox1 <= cuboenex12) && (yB <= cuboeney3)&&(x5==1)&&(n1>=1)) {
    gameover();
    }
     if ((cubox2 >= cuboenex7) && (cubox1 <= cuboenex8) && (yB <= cuboeney3)&&(x6==1)&&(n1>=1)) {
    gameover();
     }
     if ((cubox2 >= cuboenex11+100) && (cubox1 <= cuboenex12+100) && (cuboy2 >= cuboeney1)&&(x5==1)&&(n1>=1)) {
    gameover();
    }
    if ((cubox2 >= cuboenex7) && (cubox1 <= cuboenex8) && (yB <= cuboeney3)&&(x8==1)&&(n2>=1)) {
    gameover();
     }
  //  if ((cuboenex3 <= 30) && (cuboeney3<= 160)&&(cuboenex3<=60)) {
  // yB=160;
//}
}

   
void animate() {
  animcstate += 3;
  animspstate += 3;
  animc = (animcstate / 18) % 16;
  animsp = (animspstate / 15) % 20;
}
void gameover() {
  Start = false;
  FillRect(0, 0, 319, 219, 0x421b);
  while (!Start) {
    String over = "GAME OVER!!";
    LCD_Print(over, 100, 100, 2, 0xffff, 0xD090);
    String again = "Press jump to go back";
    LCD_Print(again, 100, 150, 1, 0xf420, 0x0000);
    bottomestado = digitalRead(buttonPin);
    if (bottomestado== LOW) {
      Start = true;
    }
  }
  empezarjuego();
  Start = false;
  while (!Start) {
    bottomestado = digitalRead(buttonPin);
    if (bottomestado == LOW) {
      Start = true;
    }
  }
  FillRect(0, 0, 319, 219, 0x421b);
  String text1 = "Level 1";
  LCD_Print(text1, 30, 20, 2, 0xffff, 0xD082);
  hacerpiso();
}
