#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <math.h>
#include <iostream>
#include <WROVER_KIT_LCD.h>

#define OLED_ADDR   0x3C
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
void efektor1();
void zpet2();
void autor1();
void kontrola();
void aktpoz();
void souradniceA();
void souradniceB();
void souradniceefeektor();
//---------------------------------------------------------------------------inicializace serv---------------------------------------------------------------------------
Servo zakladna;
Servo zapesti;
Servo loket;
Servo rameno;
Servo predlokti;
Servo dlan;
//---------------------------------------------------------------------------globalni promene----------------------------------------------------------------------------
int CLK = 0;
int DT = 2;
int SW = 15;
int maximum = 181;
int minimum = -1;
int poz = 1;
int nav = 0;
int stavPred;
int stavCLK;
int stavSW;
int parek = 0;
int poziceEnkod = 0;
int tlacitko;
int poz_serva1;
int poz_serva2;
int poz_serva3;
int poz_serva4;
int poz_serva5;
int poz_serva6;
float polohaX;
float pozosx;

//---------------------------------------------------------------------------souradnice----------------------------------------------------------------------------
double rada,radb;
double radg,radd;
double alfa ,beta;
double gama, delta;
double lenght1 = 210;
double lenght2 = 205;
double X ,X1 ,X2;
double Y ,Y1 ,Y2;
double Z ,Z1 ,Z2;
float delka0A,delkaAB;
double uhel,pomocuhel;
double delka;
//-------------------------------------------------------------------------setup-----------------------------------------------------------------------------
void setup() {
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  stavPred = digitalRead(CLK);
  pinMode(SW, INPUT_PULLUP);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  uvod();
  zakladna.attach(32);
  rameno.attach(33);
  loket.attach(25);
  predlokti.attach(26);
  zapesti.attach(27);
  dlan.attach(14);
}
//-------------------------------------------------------------------------loop-----------------------------------------------------------------------------
// z loopu jsou pomoci logiky funkcí if a switch volány funkce níže v kodu
void loop() {
enkoder();
kontrola();
souradniceA();
souradniceB();
souradniceefeektor();

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
  if (nav > 5 || nav < 0)
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
      aktpoz();
      break;
    case 5:
      zpet2();
        stavSW = digitalRead(SW);
        if (stavSW == 0) {
        tlacitko=1;
        delay(250);
        }
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
        delay(250);
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
      zakladna.write(poz_serva1);
      break;
    case 2:
      ovl_osy2();
      rameno.write(poz_serva2);
      break;
    case 3:
      ovl_osy3();
      loket.write(poz_serva3);
      break;
    case 4:
      ovl_osy4();
      predlokti.write(poz_serva4);
      break;
    case 5:
      ovl_osy5();
      zapesti.write(poz_serva5);
      break;
      case 6:
      ovl_osy6();
      dlan.write(poz_serva6);
      break; 
    }
}  
if (tlacitko == 3 && poz == 2)
{
  if (nav > 4 || nav < 1)
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
      efektor1();
      break;
    } 
}
  
}

//------------------------------------------------------------ss--------------------------------------------------------------------------------
// výpočet polohy efektoru pomocí převodu sferických souřadnic na kartezské a pote z délky vektorů a známeho úhlu(cos. věta ) dopočítáme třetí stranu trojúhelníku
// díky které jsme poté schopni určit polohu efektoru
void souradniceA()
{
  alfa = poz_serva1;
  beta = poz_serva2;
  rada = PI / 180 * alfa;
  radb = PI / 180 * beta;
  X1 = sin(radb) * cos(rada) * lenght1;
  Y1 = sin(radb) * sin(rada) * lenght1;
  Z1 = cos(radb) * lenght1;
  delka0A = sqrt(pow(X1-0 ,2)+pow(Y1-0 ,2)+pow(Z1-0 ,2));
}

void souradniceB()
{
  gama = poz_serva3;
  radg = PI / 180 * gama;
  X2 = sin(radg) * cos(rada) * lenght2 ;
  Y2 = sin(radg) * sin(rada) * lenght2 ;
  Z2 = cos(radg) * lenght2 + Z1;
  delkaAB = sqrt(pow(X2-X1 ,2)+pow(Y2-Y1 ,2)+pow(Z2-Z1 ,2));
  if (poz_serva3 > 40 )
  {
    uhel = 40;
  }
  else
  {
    uhel = poz_serva3;
  }
  delka = sqrt(pow(delka0A,2)+pow(delkaAB,2)-2*delkaAB*delka0A*cos(uhel))   ;
}
void souradniceefeektor()
{
  pomocuhel = delka/delkaAB*sin(uhel);
  delta = poz_serva2 - pomocuhel;
  radd = PI / 180 * delta;
  X = sin(radd) * cos(rada) * delka;
  Y = sin(radd) * sin(rada) * delka;
  Z = cos(radd) * delka;
  pozosx = X + polohaX;
  
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
      {poz_serva1++;}
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
      
    }
    else {
      if (tlacitko == 1)
      {poz--;}
      if (tlacitko == 2)
      {nav--;}
      if (tlacitko == 3 && poz == 1 && nav == 1)
      {poz_serva1--;}
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
    }
  }
stavPred = stavCLK;
stavSW = digitalRead(SW);
if (stavSW == 0) {
    tlacitko++;
    delay(250);
  }

if (tlacitko > 3)
  {
  tlacitko = 2;
  } 
  
}
//--------------------------------------------------------------------serva------------------------------------------------------------------------------
void kontrola()
{
  if (poz_serva1 >= maximum)
  { poz_serva1 = 180; }
  if (poz_serva2 >= maximum)
  { poz_serva2 = 180; }
  if (poz_serva3 >= maximum)
  { poz_serva3 = 180; }
  if (poz_serva4 >= maximum)
  { poz_serva4 = 180; }
  if (poz_serva5 >= maximum)
  { poz_serva5 = 180; }
  if (poz_serva6 >= maximum)
  { poz_serva6 = 180; }

  if (poz_serva1 <= minimum)
  { poz_serva1 = 0; }
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
  display.println("6.sesta osa");
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
  display.println("6.sesta osa");
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
  display.println("6.sesta osa");
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
  display.println("aktualni pozice");
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
  display.println("aktualni pozice");
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
  display.println("osa Y");
  display.setCursor(5, 25);
  display.println("osa Z");
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
  display.println("osa Z");
  display.setCursor(5, 25);
  display.println("aktualni pozice");
  display.setCursor(5, 42);
  display.println("zpet");
  display.display();
}
//------------------------------------------------------------------------------efektor osy-----------------------------------------------------------------------------------
void osaX()
{
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
void osaY(){}
void osaZ(){}

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
  display.println(pozosx);
  display.setCursor(0,29);
  display.println("Y = ");
  display.setCursor(20,29);
  display.println(Y);
  display.setCursor(0,37);
  display.println("Z = ");
  display.setCursor(20,37);
  display.println(Z);
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
  display.println(poz_serva1);
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
