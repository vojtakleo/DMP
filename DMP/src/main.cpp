#include <SPI.h>
#include "FS.h"
#include <Wire.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <math.h>
#include <iostream>
#include <ArduinoEigenDense.h>
#include <cmath>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
int lcdColumns = 20;
int lcdRows = 4;
using Eigen::MatrixXd;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  
#define EEPROM_SIZE 220
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
float polohaX=0,polohaY=0,polohaZ=0;
float pozosx=0,pozosy=0,pozosz=0;

//---------------------------------------------------------------------------souradnice----------------------------------------------------------------------------
float const alfa = 0;
float const alfa2 = -90/180;
float const alfa3 = 0;
float const alfa4 = -90/180;
float const alfa5 = 90/180;
float const alfa6 = 0;
int const D1 = 249;    
int const D4 = 237;       
int const D6 = 150;     
int const A2 = 209;     
float x, y, z;
int s,r;
float rad1, rad2, rad3, rad4, rad5;
//-------------------------------------------------------------------------setup-----------------------------------------------------------------------------
void setup() {
  EEPROM.begin(EEPROM_SIZE);
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  stavPred = digitalRead(CLK);
  pinMode(SW, INPUT_PULLUP);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  uvod();
  aktualni_pozice();
  zakladna.attach(32);
  rameno.attach(33);
  ruka.attach(25);
  loket.attach(26);
  predlokti.attach(27);
  zapesti.attach(14);
  efektor.attach(12);
  //-------------eeprom----------
  pol = EEPROM.read(1);
  poz_serva2 =EEPROM.read(2);
  miror = EEPROM.read(3);
  poz_serva3 = EEPROM.read(4);
  poz_serva4 = EEPROM.read(5);
  poz_serva5  =EEPROM.read(6);
  poz_serva6 =EEPROM.read(7);
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
//-------------------------------------------------------------------------loop-----------------------------------------------------------------------------
// z loopu jsou pomoci logiky funkcí if a switch volány funkce níže v kodu
void loop() {
  enkoder();
  kontrola();

  if (tlacitko == 0)
  {
    uvod();
}
if (tlacitko == 1){

  if (poz > 3)
  {
  poz = 3;
  }
  if (poz < 1)
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
  

  if (nav > 7)
  {
  nav = 7;
  }
  if (nav < 0)
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
  if (nav > 6)
  {
  nav = 6;
  }
  if (nav < 0)
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
  if (nav > 6)
  {
  nav = 6;
  }
  if (nav < 0)
  {
  nav = 0;
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
  if (nav > 6)
  {
  nav = 6;
  }
  if (nav < 0)
  {
  nav = 0;
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
      //inv_kinematika();
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
  EEPROM.write(1, pol);
  EEPROM.write(2, poz_serva2);
  EEPROM.write(3, miror);
  EEPROM.write(4, poz_serva3);
  EEPROM.write(5, poz_serva4);
  EEPROM.write(6, poz_serva5);
  EEPROM.write(7, poz_serva6);
  EEPROM.commit();
  }

}

//------------------------------------------------------------enkoder--------------------------------------------------------------------------------
// zdroj kodu na čtení enkodéru: https://navody.dratek.cz/navody-k-produktum/rotacni-enkoder-ky-040.html
void enkoder()
{

  stavCLK = digitalRead(CLK);
  if (stavCLK != stavPred) {
    lcd.clear();
    if (digitalRead(DT) != stavCLK) {
      if (tlacitko == 1)
      {poz++;}
      if (tlacitko == 2)
      {nav++;}
      if (tlacitko == 3 && poz == 1 && nav == 1)
      {pol=pol+5;}
      if (tlacitko == 3 && poz == 1 && nav == 2)
      {poz_serva2=poz_serva2+5;}
      if (tlacitko == 3 && poz == 1 && nav == 3)
      {poz_serva3=poz_serva3+5;}
      if (tlacitko == 3 && poz == 1 && nav == 4)
      {poz_serva4=poz_serva4+5;}
      if (tlacitko == 3 && poz == 1 && nav == 5)
      {poz_serva5=poz_serva5+5;}
      if (tlacitko == 3 && poz == 1 && nav == 6)
      {poz_serva6=poz_serva6+5;}
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
      {pol=pol-5;}
      if (tlacitko == 3 && poz == 1 && nav == 2)
      {poz_serva2=poz_serva2-5;}
      if (tlacitko == 3 && poz == 1 && nav == 3)
      {poz_serva3=poz_serva3-5;}
      if (tlacitko == 3 && poz == 1 && nav == 4)
      {poz_serva4=poz_serva4-5;}
      if (tlacitko == 3 && poz == 1 && nav == 5)
      {poz_serva5=poz_serva5-5;}
      if (tlacitko == 3 && poz == 1 && nav == 6)
      {poz_serva6=poz_serva6-5;}
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
    lcd.clear();
    delay(250);
  }

  
}
//--------------------------------------------------------------------serva------------------------------------------------------------------------------
void kontrola()
{

  if (pol >= 122)
  { pol = 122; }
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

  if (pol <= -122)
  { pol = -122; }
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

  lcd.setCursor(0,0);
  lcd.print("pokud si prejete");
  lcd.setCursor(0,1);
  lcd.print("pokracovat stisknite");
  lcd.setCursor(0,2);
  lcd.print("tlacitko enkoderu.");
  lcd.setCursor(0,3);
  lcd.print("ovladejte otacenim");
  lcd.setCursor(0,4);


 }

void osovy_rezim(){
 

  lcd.setCursor(0,0);
  lcd.print("> 1.osovy rezim");
  lcd.setCursor(0,1);
  lcd.print("  2.rezim efektoru");
  lcd.setCursor(0,2);
  lcd.print("  3.credits");
 
  }

  void rezim_efektoru()
  {

  lcd.setCursor(0,0);
  lcd.print("  1.osovy rezim");
  lcd.setCursor(0,1);
  lcd.print("> 2.rezim efektoru");
  lcd.setCursor(0,2);
  lcd.print("  3.credits");

  }
  void credits()
  {

  lcd.setCursor(0,0);
  lcd.print("  1.osovy rezim");
  lcd.setCursor(0,1);
  lcd.print("  2.rezim efektoru");
  lcd.setCursor(0,2);
  lcd.print("> 3.credits");

  }
  //-----------------------------------------------------------------------------osovy rezim menu------------------------------------------------------------------------
  void osovyrezim(){

  lcd.setCursor(0,0);
  lcd.print("vyteje v osovem");
  lcd.setCursor(0,1);
  lcd.print("rezimu oladejte ");
  lcd.setCursor(0,2);
  lcd.print(" otocenim");

}
  
void osa1(){

  lcd.setCursor(0,0);
  lcd.print("> 1.prvni osa");
  lcd.setCursor(0,1);
  lcd.print("  2.druha osa");
  lcd.setCursor(0,2);
  lcd.print("  3.treti osa");
  lcd.setCursor(0,3);
  lcd.print("  4.ctvrta osa");

}
void osa2(){
  lcd.setCursor(0,0);
  lcd.print("  1.prvni osa");
  lcd.setCursor(0,1);
  lcd.print("> 2.druha osa");
  lcd.setCursor(0,2);
  lcd.print("  3.treti osa");
  lcd.setCursor(0,3);
  lcd.print("  4.ctvrta osa");

}

void osa3(){
  lcd.setCursor(0,0);
  lcd.print("  1.prvni osa");
  lcd.setCursor(0,1);
  lcd.print("  2.druha osa");
  lcd.setCursor(0,2);
  lcd.print("> 3.treti osa");
  lcd.setCursor(0,3);
  lcd.print("  4.ctvrta osa");
  lcd.display();
}

void osa4(){
  lcd.setCursor(0,0);
  lcd.print("  2.druha osa");
  lcd.setCursor(0,1);
  lcd.print("  3.treti osa");
  lcd.setCursor(0,2);
  lcd.print("> 4.ctvrta osa");
  lcd.setCursor(0,3);
  lcd.print("  5.pata osa");

}
void osa5(){
  lcd.setCursor(0,0);
  lcd.print("  3.treti osa");
  lcd.setCursor(0,1);
  lcd.print("  4.ctvrta osa");
  lcd.setCursor(0,2);
  lcd.print("> 5.pata osa");
  lcd.setCursor(0,3);
  lcd.print("  6.efektor");

}
void osa6(){
  lcd.setCursor(0,0);
  lcd.print("  4.ctvrta osa");
  lcd.setCursor(0,1);
  lcd.print("  5.pata osa");
  lcd.setCursor(0,2);
  lcd.print("> 6.efektor");
  lcd.setCursor(0,3);
  lcd.print("  zpet");

}
void zpet1()
{
  lcd.setCursor(0,0);
  lcd.print("  5.pata osa");
  lcd.setCursor(0,1);
  lcd.print("  6.efektor");
  lcd.setCursor(0,2);
  lcd.print("> zpet");

}
//------------------------------------------------------------------------------rezim efektoru menu---------------------------------------------------------------------
void rezimefektoru()
{
  lcd.setCursor(0,0);
  lcd.print("rezim efektoru");
  lcd.setCursor(0,1);
  lcd.print("otocenim si vybrer");
  lcd.setCursor(0,2);
  lcd.print("osu kterou chces");
  lcd.setCursor(0,3);
  lcd.print("ovladat");

}

void karx()
{
  lcd.setCursor(0,0);
  lcd.print("> osa X");
  lcd.setCursor(0,1);
  lcd.print("  osa Y");
  lcd.setCursor(0,2);
  lcd.print("  osa Z");
  lcd.setCursor(0,3);
  lcd.print("  potvrdit bod");

}
void kary()
{
  lcd.setCursor(0,0);
  lcd.print("  osa X");
  lcd.setCursor(0,1);
  lcd.print("> osa Y");
  lcd.setCursor(0,2);
  lcd.print("  osa Z");
  lcd.setCursor(0,3);
  lcd.print("  potvrdit bod");

}
void karz()
{
  lcd.setCursor(0,0);
  lcd.print("  osa X");
  lcd.setCursor(0,1);
  lcd.print("  osa Y");
  lcd.setCursor(0,2);
  lcd.print("> osa Z");
  lcd.setCursor(0,3);
  lcd.print("  potvrdit bod");

}
void potvrdBod()
{
  lcd.setCursor(0,0);
  lcd.print("  osa Y");
  lcd.setCursor(0,1);
  lcd.print("  osa Z");
  lcd.setCursor(0,2);
  lcd.print("> potvrdit bod");
  lcd.setCursor(0,3);
  lcd.print("  aktualni pozice");


}
void aktpoz()
{
  lcd.setCursor(0,0);
  lcd.print("  osa Z");
  lcd.setCursor(0,1);
  lcd.print("  potvrdit bod");
  lcd.setCursor(0,2);
  lcd.print("> aktualni pozice");
  lcd.setCursor(0,3);
  lcd.print("  zpet");
 
}
void zpet2()
{
  lcd.setCursor(0,0);
  lcd.print("  potvrdit bod");
  lcd.setCursor(0,1);
  lcd.print("  aktualni pozice");
  lcd.setCursor(0,2);
  lcd.print("> zpet");

}

//------------------------------------------------------------------------------efektor osy-----------------------------------------------------------------------------------
void osaX()
{
  pozosx = x + polohaX;
  lcd.setCursor(0,0);
  lcd.print("osa X ");
  lcd.setCursor(0,1);
  lcd.print("pozice:");
  lcd.setCursor(10,1);
  lcd.print(pozosx);
  lcd.setCursor(0,2);
  lcd.print("stisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}
void osaY()
{
  pozosy = y + polohaY;
  lcd.setCursor(0,0);
  lcd.print("osa Y ");
  lcd.setCursor(0,1);
  lcd.print("pozice:");
  lcd.setCursor(10,1);
  lcd.print(pozosy);
  lcd.setCursor(0,2);
  lcd.print("stisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}
void osaZ()
{
  pozosz = z + polohaZ;
  lcd.setCursor(0,0);
  lcd.print("osa Z");
  lcd.setCursor(0,1);
  lcd.print("pozice:");
  lcd.setCursor(10,1);
  lcd.print(pozosz);
  lcd.setCursor(0,2);
  lcd.print("stisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

  }
void bod(){
  //inv_kinematika(); 
  lcd.setCursor(0,0);
  lcd.print("tento rezim ");
  lcd.setCursor(0,1);
  lcd.print("nefungoval jak bylo");
  lcd.setCursor(0,2);
  lcd.print("planovano a musel");
  lcd.setCursor(0,3);
  lcd.print("byt odstranen");

  }
 

void efektor1()
{
  lcd.setCursor(0,0);
  lcd.print("aktualni souradnice");
  lcd.setCursor(0,1);
  lcd.print("jsou :X =");
  lcd.setCursor(10,1);
  lcd.print(x);
  lcd.setCursor(0,2);
  lcd.print("Y = ");
  lcd.setCursor(10,2);
  lcd.print(y);
  lcd.setCursor(0,3);
  lcd.print("Z = ");
  lcd.setCursor(10,3);
  lcd.print(z);


 }
//--------------------------------------------------------------------efektor----------------------------------------------------------------------------

//------------------------------------------------------------------------------autor-----------------------------------------------------------------------------------
void autor1()
{
  lcd.setCursor(0,0);
  lcd.print("autor: Vojtech");
  lcd.setCursor(0,1);
  lcd.print("Kleofas Prihoda");
  lcd.setCursor(0,2);
  lcd.print("vedouci prace:");
  lcd.setCursor(0,3);
  lcd.print("Tomas Hajducik");

}
//------------------------------------------------------------------------------ovladani os-----------------------------------------------------------------------------
void ovl_osy1()
{
  lcd.setCursor(0,0);
  lcd.print("osa 1 zakladna");
  lcd.setCursor(0,1);
  lcd.print("uhel otoceni:");
  lcd.setCursor(13,1);
  lcd.print(pol);
  lcd.setCursor(0,2);
  lcd.print("tisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}
void ovl_osy2()
{
  lcd.setCursor(0,0);
  lcd.print("osa 2 rameno 1");
  lcd.setCursor(0,1);
  lcd.print("uhel otoceni:");
  lcd.setCursor(13,1);
  lcd.print(poz_serva2);
  lcd.setCursor(0,2);
  lcd.print("tisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}
void ovl_osy3()
{
  lcd.setCursor(0,0);
  lcd.print("osa 3 rameno 2");
  lcd.setCursor(0,1);
  lcd.print("uhel otoceni:");
  lcd.setCursor(13,1);
  lcd.print(poz_serva3);
  lcd.setCursor(0,2);
  lcd.print("tisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}
void ovl_osy4()
{
  lcd.setCursor(0,0);
  lcd.print("osa 4 natoceni ");
  lcd.setCursor(0,1);
  lcd.print("uhel otoceni:");
  lcd.setCursor(13,1);
  lcd.print(poz_serva4);
  lcd.setCursor(0,2);
  lcd.print("tisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}

void ovl_osy5()
{
  lcd.setCursor(0,0);
  lcd.print("osa 5 rameno 3");
  lcd.setCursor(0,1);
  lcd.print("uhel otoceni:");
  lcd.setCursor(13,1);
  lcd.print(poz_serva5);
  lcd.setCursor(0,2);
  lcd.print("tisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}

void ovl_osy6()
{
  lcd.setCursor(0,0);
  lcd.print("osa 6 efektor");
  lcd.setCursor(0,1);
  lcd.print("uhel otoceni:");
  lcd.setCursor(13,1);
  lcd.print(poz_serva6);
  lcd.setCursor(0,2);
  lcd.print("tisknutim tlacitka");
  lcd.setCursor(0,3);
  lcd.print("se vratite zpet");

}


void inv_kinematika(){
  pozosx = x + polohaX;
  pozosy = y + polohaY;
  pozosz = z + polohaZ;

    s = pozosz - D1;
    // L1 vzdálenost od zeme k druhe ose
    r = sqrt(pow(pozosx, 2) + pow(pozosy, 2));
    rad1 = atan2(pozosy,pozosx);

    rad2 = asin(((alfa2 + alfa3 * cos(poz_serva3)) * s - alfa3 * sin(poz_serva3) * r) / (pow(r, 2) * pow(s, 2)));

    rad3 = acos((pow(r, 2) + pow(s, 2) - pow(alfa2, 2) - pow(alfa3, 2)) / (2 * alfa2 * alfa3));

    rad4 = atan2(-cos(poz_serva1) * sin(poz_serva2*poz_serva3) * pozosy - sin(poz_serva1) * sin(poz_serva2*poz_serva3) * pozosz + cos(poz_serva2*poz_serva3) * pozosz
    + cos(poz_serva2*poz_serva3) * pozosz , cos(poz_serva1) * cos(poz_serva2*poz_serva3) * pozosy + sin(poz_serva1) * cos(poz_serva2*poz_serva3) * pozosz + sin(poz_serva2*poz_serva3) * 0);
    
    rad5 = atan2(+-sqrt(1-pow(sin(poz_serva1)*pozosy-cos(poz_serva1) *pozosz,2)),sin(poz_serva1)*pozosy-cos(poz_serva1)*sin(poz_serva2*poz_serva3));

    pol = rad1*180;
    poz_serva2 = rad2;
    poz_serva3 = rad3;
    poz_serva4 = rad4*180;
    poz_serva5 = rad5*180;
    parek = ((pol+135)/270)*180;
    poz_serva1 = parek;
    zakladna.write(poz_serva1);
    rameno.write(poz_serva2);
    miror = 180 - poz_serva2;
    ruka.write(miror);
    loket.write(poz_serva3);
    predlokti.write(poz_serva4);
    zapesti.write(poz_serva5);

    EEPROM.write(1, pol);
    EEPROM.write(2, poz_serva2);
    EEPROM.write(3, miror);
    EEPROM.write(4, poz_serva3);
    EEPROM.write(5, poz_serva4);
    EEPROM.write(6, poz_serva5);
    EEPROM.commit();
    //---------vynulování zadávaných změn--------------------

}
void aktualni_pozice(){
MatrixXd A(4, 4);
A << 
    cos(poz_serva1), -sin(poz_serva1), 0, 0,
    sin(poz_serva1)*cos(alfa), cos(poz_serva1)*cos(alfa),-sin(alfa),-sin(alfa)*D1,
    sin(poz_serva1)*sin(alfa), cos(poz_serva1)*sin(alfa),cos(alfa),cos(alfa)*D1,
    0, 0, 0, 1;

MatrixXd B(4, 4);
B << 
    cos(poz_serva2-90), -sin(poz_serva2-90), 0, A2,
    sin(poz_serva2-90)*cos(alfa2),cos(poz_serva2-90)*cos(alfa2), -sin(alfa2), 0,
    sin(poz_serva2-90)*sin(alfa2), cos(poz_serva2-90)*sin(alfa2), cos(alfa2), 0,
    0, 0, 0, 1;

MatrixXd C(4, 4);
C << 
    cos(poz_serva3), -sin(poz_serva3), 0, 0,
    sin(poz_serva3-90)*cos(alfa3) ,cos(poz_serva3-90)*cos(alfa3) , -sin(alfa3), -sin(alfa3)*0,
    sin(poz_serva3-90)*sin(alfa3), cos(poz_serva3-90)*sin(alfa3), cos(alfa3), cos(alfa3)* 0,
    0, 0, 0, 1;

MatrixXd D(4, 4);
D << 
    cos(poz_serva4), -sin(poz_serva4), 0, 0,
    sin(poz_serva4) * cos(alfa4), cos(poz_serva4) * cos(alfa4), -sin(alfa4),-sin(alfa4)*D4,
    sin(poz_serva4) * sin(alfa4), cos(poz_serva4) * sin(alfa4), cos(alfa4),cos(alfa4)*D4,
    0, 0, 0, 1;

MatrixXd E(4, 4);
E << 
    cos(poz_serva5), -sin(poz_serva5), 0, 0,
    sin(poz_serva5) * cos(alfa5), cos(poz_serva5) * cos(alfa5), -sin(alfa5),-sin(alfa5)*0,
    sin(poz_serva5) * sin(alfa5), cos(poz_serva5) * sin(alfa5), cos(alfa5),cos(alfa5)*0,
    0, 0, 0, 1;
MatrixXd H(4, 4);
H << 
    cos(0), -sin(0), 0, 0,
    sin(0) * cos(alfa6), cos(0) * cos(alfa6), -sin(alfa6),-sin(alfa6)*D6,
    sin(0) * sin(alfa6), cos(0) * sin(alfa6), cos(alfa6),cos(alfa6)*D6,
    0, 0, 0, 1;

MatrixXd G = A * B * C * D * E * H;

x = G(0, 3);
y = G(1, 3);
z = G(2, 3);

}