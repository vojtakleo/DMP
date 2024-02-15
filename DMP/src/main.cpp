#include <SPI.h>
#include "FS.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <math.h>
#include <iostream>
#include <ArduinoEigenDense.h>
#include <cmath>

using Eigen::MatrixXd;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

//---------------------------------------------------------------------------funkce-------------------------------------------------------------------------
void uvod();
void osovy_rezim();
void rezim_efektoru();
void credits();
void osa1();
void osa2();
void osa3();
void osa4();
void osa5();
void osa6();
void enkoder();
void osovyrezim();
void ovl_osy1();
void ovl_osy2();
void ovl_osy3();
void ovl_osy4();
void ovl_osy5();
void ovl_osy6();
void zpet1();
void rezimefektoru();
void karx();
void kary();
void karz();
void osaX();
void osaY();
void osaZ();
void potvrdBod();
void bod();
void efektor1();
void zpet2();
void autor1();
void kontrola();
void aktpoz();
void inv_kinematika();
void aktualni_pozice();
//---------------------------------------------------------------------------inicializace serv---------------------------------------------------------------------------
Servo zakladna;
Servo zapesti;
Servo loket;
Servo rameno;
Servo predlokti;
Servo ruka;
Servo efektor;
//---------------------------------------------------------------------------globalni promene----------------------------------------------------------------------------
int CLK = 4;
int DT = 16;
int SW = 17;
int miror = 180;
int maximum = 181;
int minimum = -1;
int poz = 1;
int nav = 0;
int stavPred;
int stavCLK;
int stavSW;
float parek = 0;
int poziceEnkod = 0;
int tlacitko =0;
int poz_serva1;
float pol;
int poz_serva2;
int poz_serva3;
int poz_serva4;
int poz_serva5;
int poz_serva6;
float polohaX,polohaY,polohaZ;
float pozosx,pozosy,pozosz;

//---------------------------------------------------------------------------souradnice----------------------------------------------------------------------------
float const alfa = 90/180;
float const alfa2 = -90/180;
float const alfa3 = 90/180;
int const L1 = 242;     //zem   -> osa2
int const L2 = 0;       //osa2  -> osa3 v x
int const L3 = 209;     //osa2  -> osa3 v y
int const L4 = 100;     //osa3  -> osa4
int const L5 = 112;     //osa4  -> osa5
int const L6 = 142;     //osa5  -> konec efektoru
float x, y, z;
int s,r;
float rad1, rad2, rad3, rad4, rad5;
//-------------------------------------------------------------------------setup-----------------------------------------------------------------------------
void setup() {
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  stavPred = digitalRead(CLK);
  pinMode(SW, INPUT_PULLUP);
  display.begin(SSD1306_SWITCHCAPVCC,  0x3C);
  display.clearDisplay();
  display.display();
  uvod();
  zakladna.attach(32);
  rameno.attach(33);
  ruka.attach(25);
  loket.attach(26);
  predlokti.attach(27);
  zapesti.attach(14);
  efektor.attach(12);
}
//-------------------------------------------------------------------------loop-----------------------------------------------------------------------------
// z loopu jsou pomoci logiky funkcí if a switch volány funkce níže v kodu
void loop() {
enkoder();
kontrola();

if (tlacitko == 0){
  uvod();
}
if (tlacitko == 1){

if (poz > 3 || poz < 1)
  {
  poz = 1;
  }
  
    switch (poz)
    {
    case 1:
      osovy_rezim();
      break;
    case 2:
      rezim_efektoru();
      break;
    case 3:
      credits();
      break;  
    }
}

if (tlacitko == 2 && poz == 1)
{
  
  if (nav > 7 || nav < 0)
  {
  nav = 0;
  }
  switch (nav)
    {
    case 1:
      osa1();
      break;
    case 2:
      osa2();
      break;
    case 3:
      osa3();
      break;
    case 4:
      osa4();
      break;
    case 5:
      osa5();
      break;
    case 6:
      osa6();
      break;
    case 7:
      zpet1();
        stavSW = digitalRead(SW);
        if (stavSW == 0) {
        tlacitko=1;
        delay(250);
        }
      break;  
      default:
      osovyrezim();
      break;
    }
}
if (tlacitko == 2 && poz == 2)
{
  if (nav > 6 || nav < 0)
  {
  nav = 0;
  }
  switch (nav)
    {
    case 1:
      karx();
      break;
    case 2:
      kary();
      break;
    case 3:
      karz();
      break;
    case 4:
      potvrdBod();
      break;
    case 5:
      aktualni_pozice();
      aktpoz();
      break; 
      case 6:
        zpet2();
        break; 
      default:
      rezimefektoru();
      break;
    }
}

if(tlacitko == 3 && poz == 3 )
{
autor1();
  stavSW = digitalRead(SW);
        if (stavSW == 0) {
        tlacitko=1;
        delay(150);
        }
}



if (tlacitko == 3 && poz == 1)
{
  if (nav > 6 || nav < 1)
  {
  nav = 1;
  }
  switch (nav)
    {
    case 1:
      ovl_osy1();
      break;
    case 2:
      ovl_osy2();
      break;
    case 3:
      ovl_osy3();
      break;
    case 4:
      ovl_osy4();
      break;
    case 5:
      ovl_osy5();
      break;
      case 6:
      ovl_osy6();
      break; 
    }
}  
if (tlacitko == 3 && poz == 2)
{
  if (nav > 6 || nav < 1)
  {
  nav = 1;
  }
 switch (nav)
    {
    case 1:
      osaX();
      break;
    case 2:
      osaY();
      break;
    case 3:
      osaZ();
      break;
    case 4:
      inv_kinematika();
      bod();
      break;
    case 5:
      efektor1();
    break;
    case 6:
    if (stavSW == 0) {
        tlacitko=1;
        delay(250);
        }
     break;
    } 
}
if (tlacitko == 4 )
  {
  tlacitko = 2;
  miror = 180 - poz_serva2;
  parek = ((pol+135)/270)*180;
  poz_serva1 = parek;
  zakladna.write(poz_serva1);
  rameno.write(poz_serva2);
  ruka.write(miror);
  loket.write(poz_serva3);
  predlokti.write(poz_serva4);
  zapesti.write(poz_serva5);
  efektor.write(poz_serva6);
  } 


}

//------------------------------------------------------------enkoder--------------------------------------------------------------------------------
// zdroj kodu na čtení enkodéru: https://navody.dratek.cz/navody-k-produktum/rotacni-enkoder-ky-040.html
void enkoder()
{

  stavCLK = digitalRead(CLK);
  if (stavCLK != stavPred) {
    if (digitalRead(DT) != stavCLK) {
      if (tlacitko == 1)
      {poz++;}
      if (tlacitko == 2)
      {nav++;}
      if (tlacitko == 3 && poz == 1 && nav == 1)
      {pol++;}
      if (tlacitko == 3 && poz == 1 && nav == 2)
      {poz_serva2++;}
      if (tlacitko == 3 && poz == 1 && nav == 3)
      {poz_serva3++;}
      if (tlacitko == 3 && poz == 1 && nav == 4)
      {poz_serva4++;}
      if (tlacitko == 3 && poz == 1 && nav == 5)
      {poz_serva5++;}
      if (tlacitko == 3 && poz == 1 && nav == 6)
      {poz_serva6++;}
      if (tlacitko == 3 && poz == 2 && nav == 1)
      {polohaX++;}
      if (tlacitko == 3 && poz == 2 && nav == 2)
      {polohaY++;}
      if (tlacitko == 3 && poz == 2 && nav == 3)
      {polohaZ++;}
      
    }
   else {
      if (tlacitko == 1)
      {poz--;}
      if (tlacitko == 2)
      {nav--;}
      if (tlacitko == 3 && poz == 1 && nav == 1)
      {pol--;}
      if (tlacitko == 3 && poz == 1 && nav == 2)
      {poz_serva2--;}
      if (tlacitko == 3 && poz == 1 && nav == 3)
      {poz_serva3--;}
      if (tlacitko == 3 && poz == 1 && nav == 4)
      {poz_serva4--;}
      if (tlacitko == 3 && poz == 1 && nav == 5)
      {poz_serva5--;}
      if (tlacitko == 3 && poz == 1 && nav == 6)
      {poz_serva6--;}
      if (tlacitko == 3 && poz == 2 && nav == 1)
      {polohaX--;}
      if (tlacitko == 3 && poz == 2 && nav == 2)
      {polohaY--;}
      if (tlacitko == 3 && poz == 2 && nav == 3)
      {polohaZ--;}
    }
  }
stavPred = stavCLK;
stavSW = digitalRead(SW);
if (stavSW == 0) {
    tlacitko++;
    delay(250);
  }

  
}
//--------------------------------------------------------------------serva------------------------------------------------------------------------------
void kontrola()
{

  if (pol >= 135)
  { pol = 135; }
  if (poz_serva2 >= maximum)
  { poz_serva2 = 180; }
  if (poz_serva3 >= maximum)
  { poz_serva3 = 180; }
  if (poz_serva4 >= maximum)
  { poz_serva4 = 180; }
  if (poz_serva5 >= maximum)
  { poz_serva5 = 180; }
  if (poz_serva6 >= 70)
  { poz_serva6 = 70; }

  if (pol <= -135)
  { pol = -135; }
  if (poz_serva2 <= minimum)
  { poz_serva2 = 0; }
  if (poz_serva3 <= minimum)
  { poz_serva3 = 0; }
  if (poz_serva4 <= minimum)
  { poz_serva4 = 0; }
  if (poz_serva5 <= minimum)
  { poz_serva5 = 0; }
  if (poz_serva6 <= minimum)
  { poz_serva6 = 0; }

}

// zde jsou všechny funkce na vypsání textu na oled displej
//-----------------------------------------------------------------prvni menu---------------------------------------------------------------------------

void uvod()
{
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,5);
  display.println("vitejte pokud si");
  display.setCursor(0,13);
  display.println("prejete pokracovat");
  display.setCursor(0,21);
  display.println("stisknite tlacitko ");
  display.setCursor(0,29);
  display.println("enkoderu. menu je ");
  display.setCursor(0,37);
  display.println("ovladano otacenim ");
  display.setCursor(0,45);
  display.println("enkoderu.");
  display.display();
 }

void osovy_rezim(){
 
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(5, 8);
  display.println("1.osovy rezim");
  display.setCursor(0, 0);
  display.println("_____________________");
  display.setCursor(0, 8);
  display.println("|");
  display.setCursor(0, 12);
  display.println("_____________________");
  display.setCursor(120, 8);
  display.println("|");
  display.setCursor(5, 25);
  display.println("2.rezim efektoru");
  display.setCursor(5, 42);
  display.println("3. credits");
  display.display();

  }

  void rezim_efektoru()
  {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);  
  display.setCursor(0, 17);
  display.println("_____________________");
  display.setCursor(0, 25);
  display.println("|");
  display.setCursor(0, 29);
  display.println("_____________________");
  display.setCursor(120, 25);
  display.println("|");
  display.setCursor(5, 8);
  display.println("1.osovy rezim");
  display.setCursor(5, 25);
  display.println("2.rezim efektoru"); 
  display.setCursor(5, 42);
  display.println("3. credits");
  display.display();
  }
  void credits()
  {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("1.osovy rezim"); 
  display.setCursor(5, 25);
  display.println("2.rezim efektoru");
  display.setCursor(5, 42);
  display.println("3. credits");
  display.display();
  }
  //-----------------------------------------------------------------------------osovy rezim menu------------------------------------------------------------------------
  void osovyrezim(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 5);
  display.println("vyteje v osovem ");
  display.setCursor(0, 13);
  display.println("rezimu otocenim si  ");
  display.setCursor(0, 21);
  display.println("muzete v menu vybrat  ");
  display.setCursor(0, 29);
  display.println("kterou z os chcete");
  display.setCursor(0, 37);
  display.println("ovladat.");
  display.display();
}
  
void osa1(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("_____________________");
  display.setCursor(0, 8);
  display.println("|");
  display.setCursor(0, 12);
  display.println("_____________________");
  display.setCursor(120, 8);
  display.println("|");
  display.setCursor(5, 8);
  display.println("1.prvni osa");
  display.setCursor(5, 25);
  display.println("2.druha osa");
  display.setCursor(5, 42);
  display.println("3.treti osa");
  display.setCursor(5, 59);
  display.println("4.ctvrta osa");
  display.display();

}
void osa2(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 17);
  display.println("_____________________");
  display.setCursor(0, 25);
  display.println("|");
  display.setCursor(0, 29);
  display.println("_____________________");
  display.setCursor(120, 25);
  display.println("|");
  display.setCursor(5, 8);
  display.println("1.prvni osa");
  display.setCursor(5, 25);
  display.println("2.druha osa");
  display.setCursor(5, 42);
  display.println("3.treti osa");
  display.setCursor(5, 59);
  display.println("4.ctvrta osa");
  display.display();
}

void osa3(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("1.prvni osa");
  display.setCursor(5, 25);
  display.println("2.druha osa");
  display.setCursor(5, 42);
  display.println("3.treti osa");
  display.setCursor(5, 59);
  display.println("4.ctvrta osa");
  display.display();
}

void osa4(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("2.druha osa");
  display.setCursor(5, 25);
  display.println("3.treti osa");
  display.setCursor(5, 42);
  display.println("4.ctvrta osa");
  display.setCursor(5, 59);
  display.println("5.pata osa");
  display.display();
}
void osa5(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("3.druha osa");
  display.setCursor(5, 25);
  display.println("4.ctvrta osa");
  display.setCursor(5, 42);
  display.println("5.pata osa");
  display.setCursor(5, 59);
  display.println("6.efektor");
  display.display();
}
void osa6(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("4.ctvrta osa");
  display.setCursor(5, 25);
  display.println("5.pata osa");
  display.setCursor(5, 42);
  display.println("6.efektor");
  display.setCursor(5, 59);
  display.println("zpet");
  display.display();

}
void zpet1()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("5.pata osa");
  display.setCursor(5, 25);
  display.println("6.efektor");
  display.setCursor(5, 42);
  display.println("zpet");
  display.display();
}
//------------------------------------------------------------------------------rezim efektoru menu---------------------------------------------------------------------
void rezimefektoru()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 5);
  display.println("vyteje v rezimu ");
  display.setCursor(0, 13);
  display.println("efektoru otocenim si  ");
  display.setCursor(0, 21);
  display.println("muzete v menu vybrat  ");
  display.setCursor(0, 29);
  display.println("kterou z os ");
  display.setCursor(0, 37);
  display.println("kartezskeho systemu ");
  display.setCursor(0, 45);
  display.println("chcete ovladat.");
  display.display(); 
}

void karx()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("_____________________");
  display.setCursor(0, 8);
  display.println("|");
  display.setCursor(0, 12);
  display.println("_____________________");
  display.setCursor(120, 8);
  display.println("|");
  display.setCursor(5, 8);
  display.println("osa X");
  display.setCursor(5, 25);
  display.println("osa Y");
  display.setCursor(5, 42);
  display.println("osa Z");
  display.setCursor(5, 59);
  display.println("potvrdit bod");
  display.display();
}
void kary()
{
 display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 17);
  display.println("_____________________");
  display.setCursor(0, 25);
  display.println("|");
  display.setCursor(0, 29);
  display.println("_____________________");
  display.setCursor(120, 25);
  display.println("|");
  display.setCursor(5, 8);
  display.println("osa X");
  display.setCursor(5, 25);
  display.println("osa Y");
  display.setCursor(5, 42);
  display.println("osa Z");
  display.setCursor(5, 59);
  display.println("potvrdit bod");
  display.display(); 
}
void karz()
{
 display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("osa X");
  display.setCursor(5, 25);
  display.println("osa Y");
  display.setCursor(5, 42);
  display.println("osa Z");
  display.setCursor(5, 59);
  display.println("potvrdit bod");
  display.display(); 
}
void potvrdBod()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("osa Y");
  display.setCursor(5, 25);
  display.println("osa Z");
  display.setCursor(5, 42);
  display.println("potvrdit bod");
  display.setCursor(5, 59);
  display.println("aktualni pozice");
  display.display();
}
void aktpoz()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("osa Z");
  display.setCursor(5, 25);
  display.println("potvrdit bod");
  display.setCursor(5, 42);
  display.println("aktualni pozice");
  display.setCursor(5, 59);
  display.println("zpet");
  display.display();
}
void zpet2()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 34);
  display.println("_____________________");
  display.setCursor(0, 42);
  display.println("|");
  display.setCursor(0, 46);
  display.println("_____________________");
  display.setCursor(120, 42);
  display.println("|");
  display.setCursor(5, 8);
  display.println("potvrdit bod");
  display.setCursor(5, 25);
  display.println("aktualni pozice");
  display.setCursor(5, 42);
  display.println("zpet");
  display.display();
}

//------------------------------------------------------------------------------efektor osy-----------------------------------------------------------------------------------
void osaX()
{
  pozosx = x + polohaX;
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa X ");
  display.setCursor(0, 8);
  display.println("pozice efektoru:");
  display.setCursor(20, 20);
  display.println(pozosx);
  display.setCursor(0, 28);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 36);
  display.println("se vratite zpet");
  display.display();
}
void osaY()
{
  pozosy = y + polohaY;
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa Y ");
  display.setCursor(0, 8);
  display.println("pozice efektoru:");
  display.setCursor(20, 20);
  display.println(pozosy);
  display.setCursor(0, 28);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 36);
  display.println("se vratite zpet");
  display.display();
}
void osaZ()
{
  pozosz = z + polohaZ;
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa Z ");
  display.setCursor(0, 8);
  display.println("pozice efektoru:");
  display.setCursor(20, 20);
  display.println(pozosz);
  display.setCursor(0, 28);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 36);
  display.println("se vratite zpet");
  display.display();
  }
void bod(){
  inv_kinematika(); 
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(5, 8);
  display.println(" probyha vypocet a  presun");
  display.display(); 
  }
 

void efektor1()
{
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,5);
  display.println("aktualni souradnice");
  display.setCursor(0,13);
  display.println("jsou :");
  display.setCursor(0,21);
  display.println("X =");
  display.setCursor(20,21);
  display.println(x);
  display.setCursor(0,29);
  display.println("Y = ");
  display.setCursor(20,29);
  display.println(y);
  display.setCursor(0,37);
  display.println("Z = ");
  display.setCursor(20,37);
  display.println(z);
  display.display();
 }
//--------------------------------------------------------------------efektor----------------------------------------------------------------------------

//------------------------------------------------------------------------------autor-----------------------------------------------------------------------------------
void autor1()
{
 display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.println("autor: Vojtech");
  display.setCursor(0, 16);
  display.println("Kleofas Prihoda");
  display.setCursor(0, 24);
  display.println("vedouci prace:");
  display.setCursor(0, 32);
  display.println("Tomas Hajducik");
  display.setCursor(0, 40);
  display.println("mikrokontroler: ESP32");
  display.setCursor(0, 48);
  display.println("programovano v:");
  display.setCursor(0, 56);
  display.println("platformIO");
  display.display(); 
}
//------------------------------------------------------------------------------ovladani os-----------------------------------------------------------------------------
void ovl_osy1()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa 1 zakladna");
  display.setCursor(0, 8);
  display.println("uhel otoceni:");
  display.setCursor(100, 8);
  display.println(pol);
  display.setCursor(0, 20);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 28);
  display.println("se vratite zpet");
  display.display();
}
void ovl_osy2()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa 2 zakladna");
  display.setCursor(0, 8);
  display.println("uhel otoceni:");
  display.setCursor(100, 8);
  display.println(poz_serva2);
  display.setCursor(0, 20);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 28);
  display.println("se vratite zpet");
  display.display();
}
void ovl_osy3()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa 3 zakladna");
  display.setCursor(0, 8);
  display.println("uhel otoceni:");
  display.setCursor(100, 8);
  display.println(poz_serva3);
  display.setCursor(0, 20);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 28);
  display.println("se vratite zpet");
  display.display();
}
void ovl_osy4()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa 4 zakladna");
  display.setCursor(0, 8);
  display.println("uhel otoceni:");
  display.setCursor(100, 8);
  display.println(poz_serva4);
  display.setCursor(0, 20);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 28);
  display.println("se vratite zpet");
  display.display();
}
void ovl_osy5()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa 5 zakladna");
  display.setCursor(0, 8);
  display.println("uhel otoceni:");
  display.setCursor(100, 8);
  display.println(poz_serva5);
  display.setCursor(0, 20);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 28);
  display.println("se vratite zpet");
  display.display();
}
void ovl_osy6()
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("osa 6 zakladna");
  display.setCursor(0, 8);
  display.println("uhel otoceni:");
  display.setCursor(100, 8);
  display.println(poz_serva6);
  display.setCursor(0, 20);
  display.println("stisknutim tlacitka");
  display.setCursor(0, 28);
  display.println("se vratite zpet");
  display.display();
}
void inv_kinematika(){
    s = pozosz - 242;
    // 242 vzdálenost od zeme k druhe ose
    r = sqrt(pow(pozosx, 2) + pow(pozosy, 2));
    rad1 = atan2(pozosy,pozosx);

    rad2 = asin(((alfa2 + alfa3 * cos(poz_serva3)) * s - alfa3 * sin(poz_serva3) * r) / (pow(r, 2) * pow(s, 2)));

    rad3 = acos((pow(r, 2) + pow(s, 2) - pow(alfa2, 2) - pow(alfa3, 2)) / (2 * alfa2 * alfa3));

    rad4 = atan2(-cos(poz_serva1) * sin(poz_serva2*poz_serva3) * pozosy - sin(poz_serva1) * sin(poz_serva2*poz_serva3) * pozosz + cos(poz_serva2*poz_serva3) * pozosz
    + cos(poz_serva2*poz_serva3) * pozosz , cos(poz_serva1) * cos(poz_serva2*poz_serva3) * pozosy + sin(poz_serva1) * cos(poz_serva2*poz_serva3) * pozosz + sin(poz_serva2*poz_serva3) * 0);
    
    rad5 = atan2(+-sqrt(1-pow(sin(poz_serva1)*pozosy-cos(poz_serva1) *pozosz,2)),sin(poz_serva1)*pozosy-cos(poz_serva1)*sin(poz_serva2*poz_serva3));

    poz_serva1 = rad1*180;
    poz_serva2 = rad2;
    poz_serva3 = rad3;
    poz_serva4 = rad4*180;
    poz_serva5 = rad5*180;

    miror = 180 - poz_serva2;
    zakladna.write(poz_serva1);
    rameno.write(poz_serva2);
    ruka.write(miror);
    loket.write(poz_serva3);
    predlokti.write(poz_serva4);
    zapesti.write(poz_serva5);
}
void aktualni_pozice(){
MatrixXd A(4, 4);
A << 
    cos(poz_serva1), -sin(poz_serva1), 0, 0,
    sin(poz_serva1), cos(poz_serva1),0,0,
    0, 0,1,L1,
    0, 0, 0, 1;

MatrixXd B(4, 4);
B << 
    cos(poz_serva2), -sin(poz_serva2), 0, 0,
    sin(poz_serva2)*cos(alfa),cos(poz_serva2)*cos(alfa), -sin(alfa), 0,
    sin(poz_serva2)*sin(alfa), cos(poz_serva2)*sin(alfa), cos(alfa), 0,
    0, 0, 0, 1;

MatrixXd C(4, 4);
C << 
    cos(poz_serva3 + 90), -sin(poz_serva3 + 90), 0, -L2,
    sin(poz_serva3 + 90) , cos(poz_serva3 + 90) , 0, 0,
    0, 0, 1, L3,
    0, 0, 0, 1;

MatrixXd D(4, 4);
D << 
    cos(poz_serva4), -sin(poz_serva4), 0, 0,
    sin(poz_serva4) * cos(alfa2), cos(poz_serva4) * cos(alfa2), -sin(alfa2),0,
    sin(poz_serva4) * sin(alfa2), cos(poz_serva4) * sin(alfa2), cos(alfa2),0,
    0, 0, 0, 1;

MatrixXd E(4, 4);
E << 
    cos(poz_serva5), -sin(poz_serva5), 0, L4 + L5,
    sin(poz_serva5) * cos(alfa3), cos(poz_serva5) * cos(alfa3), -sin(alfa3), 0,
    sin(poz_serva5) * sin(alfa3), cos(poz_serva5) * sin(alfa3), cos(alfa3),0,
    0, 0, 0, 1;

MatrixXd G = A * B * C * D * E;
x = G(0, 3);
y = G(1, 3);
z = G(2, 3);
polohaX = 0;
polohaY = 0;
polohaZ = 0;
}