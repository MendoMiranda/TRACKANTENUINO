

/*Programa de CT4BB que usa as livrarias open source do ARDUINO para leitura de cartões SD, Relógios RTC, I2C, SPI, etc.
O sistema destina-se a efectuar o tracking de sondas espaciais e objectos celestes usando a Ephemeris da NASA em https://ssd.jpl.nasa.gov/horizons.cgi#top
Imprimem-se os dados em txt em ficheiros com o nome dos objectos.
O sistema lê o ficheiro verifica se a data está correcta ( se não estiver fica tudo nulo).
Caso esteja correcta, começa o loop de informação da hora real UTC e do Azimute/Elevação em que está a antena.
A cada 5 minutos compara a hora real com as do ficheiro se for igual, lê o azimute e a elevação respectiva.
Depois, compara estes valores com o Azimute e a Elevação anteriores para decidir o movimento CCW ou CW dos respectivos motores.
***************************************************************************************************************************
A pinagem utilizada no Arduino é a seguinte:
Pino 11 digital OUTPUT: comando CCW do motor da Elevação
Pino 10 digital OUTPUT: comando CW do motor da Elevação
Pino 8 digital OUTPUT: comando CW do motor do Azimute
Pino 9 digital OUTPUT: comando CCW do motor do Azimute
Pino 7 digital INPUT_PULLUP: sensor de fim de curso da Elevação
Pino 6 digital INPUT_PULLUP: sensor de início de curso da Elevação
Pino 5 digital INPUT_PULLUP: sensor de fim de curso do Azimute
Pino 3 digital INPUT_PULLUP: sensor de início de curso do Azimute
Pino 4 digital INPUT_PULLUP: Clicks do Botão Menu (3 opções: click simples, duplo click ou pressão longa 3 segundos)
Pino SPI-1 (pintinha) digital MISO de comunicação com o SD CARD. SPI-2 +Vcc. Esta ficha está no meio do ARDUINO DUE
Pino SPI-4 digital MOSI de comunicação SPI com o SD CARD. SOI-5 RESET e SPI-6 GND
Pino SPI-3 digital SLC de comunicação SPI com o SD CARD
Pino 2 digital CS (Chip select) de comunicação SPI com o SD CARD
Pino A4 analógico SDA para comunicações I2C Relogio RTC DS1307 e do LCD (O LCD usa um interface I2C para usar aoenas 2 pinos. Caso contrário seriam necessários 7 pinos e inviabilizava o projecto
Pino A5 analógico SCL para comunicações I2C Relogio RTC DS1307 e do LCD
Pino A0 analógico INPUT para análise da tensão do potenciómetro dos azimutes
Pino A1 analógico INPUT para análise da tensão do potenciómetro das elevações
********************************************************************************
*Estrutura e funcionamento
*Ao ligar correm as Definições e o setup(),
*O software no setup(), vai ler a Data a Hora e o dia na função DataHoraRTC() seguidamente ainda no setup(), vai comparar as datas Ano,Mês,dia do cartão com a real do RTC. 
*usando como arranque a leitura da data do ficheiro SOL.txt definido no dentro do setup()como inicio A função que faz esta operação é a lerSOL_Data().
*Nesta função, no fim de lidas e registadas regressa ao setup() que faz executar o comparaDatas() Comparando as datas e, se forem iduais, vai ler as Directorias dos ficheiros todos do SDCard 
*com através da função * lerDIRECTORIA_SD (File dir, int numTabs)  Depois de lidas as directoria e armazenadas em Strings A,B,C,D,E,F,G,H fica preparado o menu para a escolhe do ficheiro a abrir. 
*Seguidament faz-se : SONDAS com sondastr = "Click para abrir" que aparecerá no LCD. Ao clicar para abrir com a função sondas() escolhe-se o ficheiro do objecto que se pretende seguir 
* ao definir essa escolha, carregam-se no buffer Data(u) os dados todos do objecto em causa numa matriz que irá ser acessada criteriosamente.
* Inicia-se então o Loop com o objecto seleccionado onde a cada 5 minutos o TRACKANTENUINO verifica se a  Hora e os Minutos reais se encontram no ficheiro respectivo. 
* Se existir a condição HORAS=HORASSD e MINUTOS=MINUTOSSD vão ser lidos o AzNovo e a ELNovo respectivos através da função lerHM_AZ_EL (). 
* Estes dados ficam disponíveis para processamento do movimento da antena. Durante os 5 minutos de intervalo de leitura de dados, o LCD vai informando a Data/Hora UTC o azimute e a elevação para onde 
* está orientada a antena ou se estiver em movimento,para onde se está a deslocar a antena.
*
*
*/

#include <math.h>
#include <Wire.h>
#define myWire TwoWire
#define I2C Wire
#include <RTClib.h>
RTC_DS1307    rtc;
#include <LiquidCrystal_I2C.h> // LCD que só utiliza 2 pinos do ARDUINO.
#include <OneButton.h> // Protocolo OneButton
#include <time.h>
#include <SD.h>
#include <SPI.h>
#define en 2
#define rw 1
#define rs 0
#define d4 4
#define d5 5 
#define d6 6
#define d7 7
#define bl 3
// Os LCD 16x2 que funcionam com I2C têm esta configuração de pinos
#define i2c_addr 0x27  // Endereço do LCD
LiquidCrystal_I2C lcd(i2c_addr,en,rw,rs,d4,d5,d6,d7, bl,POSITIVE);
#define CS 4  // Pino de selecção do SD Card
#define Bmenu 12 // pino do botão Menu
#define PotAzVal A0             // Entrada analogica da tensão proveniente do potenciómetro dos azimutes
#define PotElVal A1            // Entrada analogica da tensão proveniente do potenciómetro das elevações
#define CWAz 8          // pino 8 OUT para comando do motor dos azimutes em CW
#define CCWAz 9        // pino 9 OUT para comando do motor dos azimutes em CCW
#define CWEl 10       // pino 10 OUT para comando do motor das Elevações em CW
#define CCWEl 11     // pino 11 OUT para comando do motor das Elevações em  CCW
#define INIAZ 3
#define FIMAZ 5
#define INIEL 6
#define FIMEL 7
String DATA,DATASTR,DATASD,DATASDSTR,HORAMIN,HORAMINSTR,HORAMINSD,HORAMINSDSTR,MESSDSTR,MESSDSTRint,KBYINIAZstr,KBYFIMAZstr,KBYINIELstr,KBYFIMELstr;
char* SONDAS[8];
String A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P;// 
String sondastr,SONDATEMP,SubstringAz,SubstringEl ;
uint8_t h,s,l,g,t;
unsigned int KBYAz,KBYEl,KBYINIAZ,KBYFIMAZ,KBYINIEL,KBYFIMEL,ANORTC,MESRTC,DIARTC,HORARTC,MINUTOSRTC;
unsigned short n,k,flag;
bool b;
uint16_t u;
char Dados[10119];
char KBY[50];
String Dadosstr, KBYstr;
float azim =180;//Buffer onde é armazenado o azimute em que se encontra o sistema retirado ds posicionador (Potenciómetros ou Encoders rotativos "Rotary Encoders"
float elev=45;// Buffer onde é armazenado o azimute em que se encontra o sistema retirado ds posicionador (Potenciómetros ou Encoders rotativos "Rotary Encoders"
float elevnovo,elevdif,azimnovo,f; //f será para fazer f=f+0,5 nos Offsets da antena.
float offset=0;  // OffSet da antena em graus para entrar no menu de duplo click . Por defeito está em 22,40 se não se entrou um outro novo OffSet.
OneButton button(Bmenu, true);
File myFile ;
File RAIZSD;
  
///////////////////     FUNÇÕES //////////////////////////////////////////////

//******************************************************************************* DELAY  4 SEGUNDOS

void delay4S()
{
  int l=0;
  while(l<=40){delay(100); button.tick();l++;}
}

//************************************************************************** DELAY 2 SEGUNDOS

void delay2S()
{
  int l=0;
  while(l<=20){delay(100);button.tick(); l++;}
  }

// ******************************************************************************  Escreve em KBY.TXT os dados do RESET/SETUP
void EscreverKBY()
{

SD.remove("ZKBY.TXT");// Remove o ficheiro anterior com dados de SETUP velhos
delay(100);
  myFile = SD.open("ZKBY.TXT",FILE_WRITE);// Cria um novo ficheiro ZKBY.TXT
  myFile.close();
/**
  u=0;
for(u=0;u<=55;u++)
    {
      (KBY[u])=0;// Apaga o buffer
    }
    
KBYstr.remove(0);//Limpeza da String Dadosstr S

**/
  
  myFile = SD.open("ZKBY.TXT",FILE_WRITE); // Abrir o ficheiro para gravar os novos dados após o RESET/SETUP
  
 delay(100);
 
  if (myFile) 
  {
    
     Serial.println("A abrir o ficheiro  ");
     Serial.println("ZKBY.TXT");
     Serial.println("A escrever dados KBY...");
    myFile.print("KBYINIAZ=");
    myFile.println(KBYINIAZ);
    myFile.print("KBYFIMAZ=");
    myFile.println(KBYFIMAZ);
    myFile.print("KBYINIEL=");
    myFile.println(KBYINIEL);
    myFile.print("KBYFIMEL=");
    myFile.println(KBYFIMEL);
  
 // close the file:
    myFile.close();
    
  } 
  else
  {
   
    Serial.println("Erro ao abrir ZKBY.TXT");
  } 
Serial.println("Gravados os KBYs...");
}


//  ****************************************** Ler no cartão SD e no ficheiro KBY.TXT, os dados dos KBYINIAZ, KBYFIMAZ, KBYINIEL, KBYFIMEL guardados aquando do RESET/SETUP

void LerKBY()
{


 // Abrir ficheiro para ler
  myFile = SD.open("ZKBY.TXT", FILE_READ);
 delay(100);
 
  if (myFile) 
  {
   Serial.print("A abrir o ficheiro  ");
   Serial.println("ZKBY.TXT");
u=0;
            
    for(u=0;u<=59;u++)
    {
  KBY[u]=(myFile.read());
  KBYstr.concat(KBY[u]);
  Serial.print(KBY[u]);

    }
      // close the file:
    myFile.close();
    
    Serial.println("");
    Serial.println("Dados lidos");
    Serial.println("Estes a seguir são os dados tirados da string KBYstr");
    Serial.print(KBYstr);
    Serial.println("");
    }
   else
  {
    // if the file didn't open, print an error:
    lcd.print("ERRO ao abrir...");
    lcd.println("KBY.TXT");
     myFile.close();
  }

KBYINIAZstr=KBYstr.substring(9,13);
KBYINIAZstr.trim();
Serial.println("Identificação de que o caractere nº 23 é sinal de igual e avança uma casa");
Serial.println(KBYstr.substring(23,24));
delay(100);
if((KBYstr.substring(23,24)).compareTo("=") == 0){KBYFIMAZstr=KBYstr.substring(24,28);}
else {KBYFIMAZstr=KBYstr.substring(23,28);}
KBYFIMAZstr.trim();
Serial.println("Identificação de que o caractere nº 38 é sinal de igual e avança uma casa");
Serial.println(KBYstr.substring(38,39));
delay(100);
if((KBYstr.substring(38,39)).compareTo("=") == 0){KBYINIELstr=KBYstr.substring(39,43);}
else {KBYINIELstr=KBYstr.substring(38,43);}
KBYINIELstr.trim();
Serial.println("Identificação de que o caractere nº 53 é sinal de igual e avança uma casa");
Serial.println(KBYstr.substring(53,54));
delay(100);
if((KBYstr.substring(53,54)).compareTo("=") == 0){KBYFIMELstr=KBYstr.substring(54,58); }
else {KBYFIMELstr=KBYstr.substring(53,58);}
KBYFIMELstr.trim();
KBYINIAZ=KBYINIAZstr.toInt();
KBYFIMAZ=KBYFIMAZstr.toInt();
KBYINIEL=KBYINIELstr.toInt();
KBYFIMEL=KBYFIMELstr.toInt();


}






// **********************************************************   LER OS FICHEIROS DO CARTÃO SD  
void lerDIRECTORIA_SD (File dir, int numTabs)
{

n=0;
while (true) 
{
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
 
SONDAS[n]={entry.name()};
delay(50);
if(n==1){A= SONDAS[n];}
delay(50);
if(n==2){B= SONDAS[n];}
delay(50);
if(n==3){C= SONDAS[n];}
delay(50);
if(n==4){D= SONDAS[n];}
delay(50);
if(n==5){E= SONDAS[n];}
delay(50);
if(n==6){F= SONDAS[n];}
delay(50);
if(n==7){G= SONDAS[n];}
delay(50);
if(n==8){H= SONDAS[n];}
delay(50);
if(n==9){I= SONDAS[n];}
delay(50);
if(n==10){J= SONDAS[n];}
delay(50);
if(n==11){K= SONDAS[n];}
delay(50);
if(n==12){L= SONDAS[n];}
delay(50);
if(n==13){M= SONDAS[n];}
delay(50);
if(n==14){N= SONDAS[n];}
delay(50);
if(n==15){O= SONDAS[n];}
delay(50);
if(n==16){P= SONDAS[n];}
delay(50);

entry.close();
delay(10);
n++;

}

 Serial.println(A);
 delay(200);
 Serial.println(B);
 delay(200);
 Serial.println(C);
 delay(200);
 Serial.println(D);
 delay(200);
 Serial.println(E);
 delay(200);
 Serial.println(F);
 delay(200);
 Serial.println(G);
 delay(200);
 Serial.println(H);
 delay(200);
 Serial.println(I);
 delay(200);
 Serial.println(J);
 delay(200);
 Serial.println(K);
 delay(200);
 Serial.println(L);
 delay(200);
 Serial.println(M);
 delay(200);
 Serial.println(N);
 delay(200);
 Serial.println(O);
 delay(200);
 Serial.println(P);
 delay(200);
 
loop();
}

//******************************  LER a DATA DO FICHEIRO SOL QUE ESTÁ NO SDCARD, PARA COMPARAR  COM A DATA RTC  E VALIDAR O CARTÃO  

void lerDataSD()
{
   sondastr="SOL.TXT";  
 //Abrir ficheiro para ler
   myFile = SD.open(sondastr, FILE_READ);
delay(100);
 
  if (myFile) 
  {
    Serial.println("A abrir o ficheiro  ");
    Serial.println(sondastr);
u=0;
    // read from the file until there's nothing else in it:
    if (myFile.available()) {
    for(u=0;u<=50;u++)
    {
  Dados[u]=(myFile.read());
  Dadosstr.concat(Dados[u]);
  DATASDSTR.concat(Dados[u]);

       }
       delay(100);
      // close the file:
    myFile.close();

  
    }
    
 DATASDSTR.remove(12);
 DATASDSTR.trim();
 Serial.print("String DATASDSTR =");
 Serial.println(DATASDSTR);
 MESSDSTR=DATASDSTR.substring(5,8);
 Serial.print("SubString MESSDSTR =");
 Serial.println(MESSDSTR);
if(MESSDSTR.compareTo("Jan")==0){MESSDSTRint="01";}
if(MESSDSTR.compareTo("Feb")==0){MESSDSTRint="02";}
if(MESSDSTR.compareTo("Mar")==0){MESSDSTRint="03";}
if(MESSDSTR.compareTo("Apr")==0){MESSDSTRint="04";}
if(MESSDSTR.compareTo("May")==0){MESSDSTRint="05";}
if(MESSDSTR.compareTo("Jun")==0){MESSDSTRint="06";}
if(MESSDSTR.compareTo("Jul")==0){MESSDSTRint="07";}
if(MESSDSTR.compareTo("Aug")==0){MESSDSTRint="08";}
if(MESSDSTR.compareTo("Sep")==0){MESSDSTRint="09";}
if(MESSDSTR.compareTo("Oct")==0){MESSDSTRint="10";}
if(MESSDSTR.compareTo("Nov")==0){MESSDSTRint="11";}
if(MESSDSTR.compareTo("Dec")==0){MESSDSTRint="12";}
DATASDSTR.replace(MESSDSTR,MESSDSTRint);

Serial.print("Nova DATA mudada para numérico =");
Serial.println(DATASDSTR);


//Tirar aqui a data do SD
 //Serial.println("A camparar datas");
 Serial.print("Data RTC: ");
 Serial.println(DATA); 
 Serial.print("Data SD: ");
 Serial.println(DATASDSTR);
 sondastr="Sem objecto";
 
    }
   else
  {
    // if the file didn't open, print an error:
    Serial.print("ERRO ao abrir...");
   Serial.println(sondastr);
     myFile.close();
  }
 
  if(DATA.compareTo(DATASDSTR)==0)
  {
Serial.println("Datas compatíveis.Lê os ficheiros");   
delay(100); 
lcd.clear();
lcd.print("A ler ficheiros");
lcd.setCursor(0,1);
lcd.print("do SDCARD.");
delay(2000);
RAIZSD = SD.open("/");
lerDIRECTORIA_SD(RAIZSD, 0);
delay(2000);
//sondastr="Sem objecto";//Retirado porque obrigava a repôr 2 vezes o Sol no arranque
  }
  else
  {
Serial.println("Datas incompatíveis. Não leu os ficheiros do SDCARD"); 
delay(100); 
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Datas diferentes");
lcd.setCursor(0,1);
lcd.print("Sem Directorias.");
delay(4000);
sondastr= "Sem objecto";
  }

  
}//Fim do lerDataSD()



//*****************************************  LER TODOS OS DADOS DO OBJECTO SELECCIONADO 
void lerSD()
{
   for(u=0;u<=10119;u++)//estavam 4900 //Limpeza do Buffer
    {
      (Dados[u])=0;
    }
Dadosstr.remove(0);//Limpeza da String Dadosstr 

 // Abrir ficheiro para ler
  myFile = SD.open(sondastr);
 delay(100);
 
  if (myFile) 
  {
    Serial.print("A abrir o ficheiro  ");
   Serial.println(sondastr);
u=0;
    // read from the file until there's nothing else in it:
    if (myFile.available()) {
    for(u=0;u<=10119;u++)//estavam 4900
    {
    Dados[u]=(myFile.read());
  Dadosstr.concat(Dados[u]);
 // Serial.print(Dados[u]);

       }
      // close the file:
    myFile.close();
    }
    Serial.println("");
     Serial.println("Estes são os dados tirados da string Dadosstr");
    Serial.print(Dadosstr);

    }
   else
  {
    // if the file didn't open, print an error:
    lcd.print("ERRO ao abrir...");
    lcd.println(sondastr);
     myFile.close();
  }

}

//**** PARA ACERTAR A DATA HORA DEVE-SE CORRER O PROGRAMA DS1307 
//**** Exemplos->Grove RTC DS1037-> SetTimeAndDisplay  

//*************************************************************     DATA HORA RTC   

void DataHoraRTC()
{
   DateTime time = rtc.now();    
   lcd.clear();
   delay(10);
     lcd.setCursor(0,0); 
      delay(10);
      lcd.println("Data Hora UTC      ");
         delay(10);
         //Serial.println(time.timestamp(DateTime::TIMESTAMP_DATE));
         //Serial.println(time.timestamp(DateTime::TIMESTAMP_TIME));
     DATA=(time.timestamp(DateTime::TIMESTAMP_DATE));
     HORAMIN=(time.timestamp(DateTime::TIMESTAMP_TIME));
      lcd.setCursor(0,1); 
      delay(10);
    lcd.print(DATA);
    lcd.print(" ");
    lcd.print(HORAMIN);
    delay2S();
     }



//*************************************  LER  HORAS:MINUTOS E COMPARAR PARA DEPOIS TIRAR AZ E EL  

void lerHM_AZ_EL ()
{
  lcd.clear();
//Tirar HORAMIN do Time Stamp e Trim
HORAMIN.remove(5);
HORAMIN.trim();
//Serial.print("String da hora e minuto do RTC=");
//Serial.println(HORAMIN);

k=13;// Analisar a razão porque era 22 quando devia ser 13 !!!!!! antes de se fazer a limpeza Dadosstr.remove(0)
while(k<10119)//
{

k=k+42;
HORAMINSD=Dadosstr.substring(k,k+5);
HORAMINSD.trim();
//Serial.print("String da hora e minuto do SD=");
//Serial.println(HORAMINSD);


  if(HORAMIN.compareTo(HORAMINSD)== 0)
  {
lcd.clear();
Serial.println("Encontrada hora do SD igual à do RTC... Buscar Az e El");
lcd.print("Hora igual !!");
lcd.setCursor(0,1);
lcd.print("Busca de Az e El");
delay(2000);
SubstringAz= Dadosstr.substring(k+10,k+15);
SubstringAz.trim();
Serial.print("SubstringAz=");
Serial.println(SubstringAz);
SubstringEl= Dadosstr.substring(k+19,k+24);
SubstringEl.trim();
Serial.print("SubstringEl=");
Serial.println(SubstringEl);
azimnovo=SubstringAz.toFloat();
Serial.print("azimnovo=");
Serial.println(azimnovo);
elevnovo=SubstringEl.toFloat();
Serial.print("elevnovo=");
Serial.println(elevnovo);


//*******************************************************    COMANDOS DOS MOTORES *************************    COMANDOS DOS MOTORES
//ELEVAÇÃO ////////////////////////////////////////  ELEVAÇÃO



 elevdif = elevnovo-offset;

if(elevdif < elev)
{

 digitalWrite(CCWEl,HIGH);
 delay(200);
lcd.clear();
lcd.print("Elev. em CCWEl");

while(2)
{
KBYEl=analogRead(PotElVal);// 11,377 Bytes por grau
elev =float(map(KBYEl,KBYINIEL,KBYFIMEL,920,0))/10;// mapeado até 90 graus, apesar da antena com cerca de 20graus  de Offset atinge-se a elevação de 90 graus aos 70m físicos.
elevnovo=SubstringEl.toFloat();
Serial.print("Elevação em curso em CCWEl= ");
Serial.println(elev,6);
Serial.println(elevdif,6);

elev= (int(elev*10)+5)/10;  // Arredondamento com aproximação a uma casa decimal.
Serial.println(elev,10);
elevdif=(int(elevdif*10)+5)/10;
Serial.println(elevdif,10);
if(digitalRead(FIMEL)==HIGH){digitalWrite(CCWEl,LOW);delay(200);break;}  //Para parar em caso de não haver leitura e o movimento prosseguir até ao fim
if(elev == elevdif)
{
  digitalWrite(CCWEl,LOW);
  delay(200);
  digitalWrite(CWEl,LOW);
  delay(200);
  break;
}
}
}
elevdif = elevnovo-offset;
if (elevdif > elev)
{

digitalWrite(CWEl,HIGH);
delay(200);
lcd.clear();
lcd.print("Elev. em CWEl");

while(2)
{
KBYEl=analogRead(PotElVal);// 11,377 Bytes por grau
elev =float(map(KBYEl,KBYINIEL,KBYFIMEL,920,0))/10; // mapeado entre 0 graus e 90 graus 
elevnovo=SubstringEl.toFloat();
Serial.print("Elevação em curso em CWEl= ");
Serial.println(elev,6);
Serial.println(elevdif,6);

elev= (int(elev*10)+5)/10; // Arredondamento com aproximação a uma casa decimal.
Serial.println(elev,10);
elevdif=(int(elevdif*10)+5)/10;
Serial.println(elevdif,10);
if(digitalRead(INIEL)==HIGH){digitalWrite(CWEl,LOW);delay(200);break;}     // Para parar em caso de não haver leitura e o movimento prosseguir até ao fim
if(elev == elevdif)
{
  digitalWrite(CWEl,LOW);
  delay(200);
  digitalWrite(CCWEl,LOW);
  delay(200);
  break;
}
}

}




//AZIMUTE  /////////////////////////////////   AZIMUTE ///////////////////////////////////////////////////////////////////

if(azim > azimnovo){
  digitalWrite(CCWAz,HIGH);
  delay(200);
lcd.clear();
lcd.print("Azimute em CCWAz");
while(1)
{
KBYAz= analogRead(PotAzVal);
azim=float(map(KBYAz,KBYINIAZ,KBYFIMAZ,0,3300))/10;
azimnovo=SubstringAz.toFloat();
Serial.print("Azimute em curso em CCWAz= ");
Serial.println(azim);
Serial.println(azimnovo);

azim= (int(azim*10)+5)/10;// Arredondamento com aproximação a uma casa decimal.
Serial.println(azim,10);
azimnovo=(int(azimnovo*10)+5)/10;
Serial.println(azimnovo,10);
 if(digitalRead(INIAZ)== HIGH){digitalWrite(CCWAz,LOW);delay(200);break;}//Para parar em caso de não haver leitura e o movimento prosseguir até ao fim
if(azim == azimnovo)
{
  digitalWrite(CCWAz,LOW);
  delay(200);
   break;
}

}
}

if(azim < azimnovo)
{
  digitalWrite(CWAz,HIGH);
  delay(200);
lcd.clear();
lcd.print("Azimute em CWAz");
while(1)
{
KBYAz= analogRead(PotAzVal);
azim=float(map(KBYAz,KBYINIAZ,KBYFIMAZ,0,3300))/10;
azimnovo=SubstringAz.toFloat();
Serial.print("Azimute em curso em CWAz= ");
Serial.println(azim);
Serial.println(azimnovo);


azim= (int(azim*10)+5)/10;  // Arredondamento com aproximação a uma casa decimal.
Serial.println(azim,10);
azimnovo=(int(azimnovo*10)+5)/10;
Serial.println(azimnovo,10);
if(digitalRead(FIMAZ)==HIGH){digitalWrite(CWAz,LOW);delay(200);break;}//Para parar em caso de não haver leitura e o movimento prosseguir até ao fim
if(azim == azimnovo)
{
  digitalWrite(CWAz,LOW);
  delay(200);
  break;
    }
}
}

lcd.setCursor(0,1);
lcd.print("AZ=");
lcd.print(azim,1);//com uma casa decimal
lcd.print("  EL=");
lcd.print(elev,1);// com uma casa decimal 

break;
}
  else
  {
delay(20);
lcd.setCursor(0,0);
lcd.print("Lendo horas SD..");
lcd.setCursor(0,1);
lcd.print("Sem hora igual.");
delay(20);
//Serial.println("Esta hora do SD não é igual à do RTC");
delay(20);
 }


 
 }
 
}
//************** OFFSET DA ANTENA  VEM DO DUPLO CLICK DO BOTÃO MENU********** OFFSET **************  OFFSET

void OFFSETantena()
{
 
  flag=0;
  g=0;
 lcd.clear();
  lcd.print("OffSet Prima 3s"); 
  delay(2000);
 offset=0;
  if(digitalRead(Bmenu) != HIGH)
{
   delay(1000);
 for (g=0; g<=21; g++)
  {
   if(g==0){offset =0;}
     delay(50);
    if(g==1){offset =20;}
     delay(50);
    if(g==2){offset =20.5;}
    delay(50);
    if(g==3){offset =21;}
    delay(50);
    if(g==4){offset =21.5;}
    delay(50);
    if(g==5){offset =22;}
    delay(50);
    if(g==6){offset =22.5;}
    delay(50);
    if(g==7){offset =23;}
    delay(50);
    if(g==8){offset =23.5;}
    delay(50);
    if(g==9){offset =24;}
     delay(50);
    if(g==10){offset =24.5;}
    delay(50);
    if(g==11){offset =25;}
    delay(50);
    if(g==12){offset =25.5;}
    delay(50);
    if(g==13){offset =26;}
    delay(50);
    if(g==14){offset =26.5;}
    delay(50);
    if(g==15){offset =27;}
    delay(50);
    if(g==16){offset =27.5;}
    delay(50);
    if(g==17){offset =28;}
     delay(50);
    if(g==18){offset =28.5;}
    delay(50);
    if(g==19){offset =29;}
    delay(50);
    if(g==20){offset =29.5;}
    delay(50);
    if(g==21){offset =30;}
    else
  { 
    offset=0;
  }
  delay(50);
    lcd.clear();
   delay(200);
  lcd.print(offset);
  delay(10);
 // Serial.println(offset);
    delay(1000);
    if(digitalRead(Bmenu)==LOW)
  {
 lcd.clear();
  delay(50);
 lcd.setCursor(0,0);
  delay(50);
  lcd.print("OK ! OFFSet");
  delay(50);
  lcd.setCursor(0,1);
   delay(50);
  lcd.print(offset);
  lcd.print("  Graus.");
  delay(50);
  delay(500);
  

break;

  
  
  }
 
  }//Fim do for
  
  }// Fim do digital BMenu HIGH

}// Fim da função OFFSet()
  



//********************************************************************* LER SONDAS 
void sondas()
{
 
 

  
 g=0;
  lcd.clear();
  lcd.print("Sondas Prima 3s");
  delay(2000);
   SONDATEMP=sondastr; //Guarda a sonda prévia.
  if(digitalRead(Bmenu) != HIGH)
{
   
 for(g=1;g<=16;g++)
  {
    delay(50);
    if(g==1){sondastr=A ;}
     delay(150);
    if(g==2){sondastr=B ;}
    delay(150);
    if(g==3){sondastr=C ;}
    delay(150);
    if(g==4){sondastr=D;}
    delay(150);
    if(g==5){sondastr=E ;}
    delay(150);
    if(g==6){sondastr=F ;}
    delay(150);
    if(g==7){sondastr=G;}
    delay(150);
    if(g==8){sondastr= H;}
    delay(150);
    if(g==9){sondastr=I ;}
     delay(150);
    if(g==10){sondastr=J ;}
    delay(150);
    if(g==11){sondastr=K ;}
    delay(150);
    if(g==12){sondastr=L;}
    delay(150);
    if(g==13){sondastr=M ;}
    delay(150);
    if(g==14){sondastr=N ;}
    delay(150);
    if(g==15){sondastr=O;}
    delay(150);
    if(g==16){sondastr= P;}
    delay(150);
     
     if(sondastr==NULL)
  {
     lcd.clear();
     lcd.print("Sem objecto...");
     sondastr="Sem objecto";
     delay(2000);
     break;
  }

     
  lcd.clear();
   delay(200);
  lcd.print(sondastr);
  delay(10);
 // Serial.println(sondastr);
    delay(2000);
  if(digitalRead(Bmenu)==LOW)
  {
 lcd.clear();
  delay(50);
 lcd.setCursor(0,0);
  delay(50);
  lcd.print("OK !! a ler...");
  delay(50);
  lcd.setCursor(0,1);
   delay(50);
  lcd.print(sondastr);
  delay(50);
 
  
 lerSD();// Depois de selecionado o objecto vão-se ler os dados do ficheiro respectivo
  delay(3000);

break;
  }

 else
  {
    //Caso não se seleccione nada, volta à sonda que estava
    //anteriormente seleccionada
    sondastr=SONDATEMP;
  }
  
  }//Fim do for
 
}//Fim do if Menu High
 
  
}// Fim das sondas


//***********************************************************************************  RESET   

void reset()
{


 lcd.clear();
  lcd.print("RESET");
  delay(2000);

//ELEVAÇÃO

digitalWrite(CWEl,HIGH);//1  passar a CWEl
lcd.clear();
lcd.print("Em curso CWELEV");//2 IDEM
while(1)
{
KBYINIEL=analogRead(PotElVal);// 
Serial.println(KBYINIEL);// Byte de inicio das Elevações
if(digitalRead(INIEL)==HIGH){digitalWrite(CWEl,LOW);break;}//PINO 6 IDEM A CWEl

}
elev=92; // Importante definir para qua o sistema saiba onde se encontra o elevação após a calibração

  delay(1000);
digitalWrite(CCWEl,HIGH);//3 passar a CCWEl
lcd.clear();
lcd.print("Em curso CCWELEV");//4 IDEM CCWELEV
while(1)
{
KBYFIMEL=analogRead(PotElVal);// 11,377 Bytes por grau
Serial.println(KBYFIMEL);//Byte do fim das elevações
if(digitalRead(FIMEL)==HIGH){digitalWrite(CCWEl,LOW);break;}//PINO 7 IDEM A CCWEl
}
elev=0;// Importante definir para que o sistema saiba onde se encontra a elevação após a calibração
lcd.clear();
delay(50);
lcd.setCursor(0,0);
delay(50);
lcd.print("Vai para AZIM.");
delay(50);
lcd.setCursor(0,1);
delay(50);
lcd.print("Aguarde...");
delay(2000);
 // A calibração electrica por leitura do  PotAzVal vai determinar o KBYINIAZ e o KBYFIMAZ que irão mapear os angulos 


//AZIMUTES

digitalWrite(CCWAz,HIGH);
 lcd.clear();
 lcd.print("Em curso CCWAZIM");
while(1)
{
KBYINIAZ= analogRead(PotAzVal);
Serial.println(KBYINIAZ);// Detecta o Byte do azimute 0 graus
delay(20);
if(digitalRead(INIAZ)== HIGH){digitalWrite(CCWAz,LOW);break;}// PINO 3
}
azim=0; // Importante definir para que o sistema saiba onde se encontra o azimute após a calibração
delay(2000);


digitalWrite(CWAz,HIGH);
lcd.clear();
lcd.print("Em curso CWAZIM");
while(1)
{
KBYFIMAZ= analogRead(PotAzVal);
Serial.println(KBYFIMAZ);// Byte do Fim dos Azimutes
delay(20);
if(digitalRead(FIMAZ)==HIGH){digitalWrite(CWAz,LOW);break;}//PINO 5
}

azim=330;// Importante definir para que o sistema saiba onde se encontra o azimute após a calibração
//******************************************

EscreverKBY();  // Guardar no SD CARD os dados KBY


delay(1000);
lcd.clear();
lcd.print("Testar com o SOL");
delay(1000);

  

  
}

 
//****************************************************************************CLICK COM UM BOTÃO  
void click(){  sondas();}
void doubleclick(){OFFSETantena();}
void press(){reset();}

//************************************************************************************  SETUP 
 void setup() 
{
Wire.begin();
SPI.begin(); 
delay(20);
SPI.setClockDivider (21);
delay(200);
digitalWrite(CS,LOW);//Selecciona o leitor SD
delay(200);
pinMode(CWAz,OUTPUT);//8
pinMode(CCWAz,OUTPUT);//9
pinMode(CWEl,OUTPUT);//10
pinMode(CCWEl,OUTPUT);//11
pinMode(INIAZ,INPUT);// As pull up dos sensores fazem a estabilidade destas entradas
pinMode(FIMAZ,INPUT);
pinMode(INIEL,INPUT);
pinMode(FIMEL,INPUT);
analogReadResolution(12);
pinMode(Bmenu,INPUT);//12 Os input digitais são colocados em PULLUP com resistencias de 10K
//porque as internas do ARDUINO são altas (50K) e provocam instabilidade.

Serial.begin(9600);
delay(200);
lcd.begin(16,2);
lcd.clear();
//lcd.backlight();
//Serial.println("LCD OK !!");
lcd.print("LCD OK !!");
delay(2000);
if (!SD.begin(CS)) 
{
    lcd.clear();
    lcd.print("Falhou SD CARD");
    delay(5000);
    while (1);
  }
  delay(100);
   lcd.clear();
   lcd.setCursor(0,0);
  lcd.print("OK! SD CARD");
 // Serial.println("OK o cartão SD está operacional");
   delay(2000);
rtc.begin();
lcd.clear();
lcd.setCursor(0,0);
delay(100);
lcd.print("RTC OK !!");
delay(2000);

// ***********************************************   ACERTO DA DATA E HORA ***************************  ACERTO DA DATA E HORA *******************
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Acerta Relogio?");
lcd.setCursor(0,1);
lcd.print("Um click...");
t=0;

while(t<3)
{
  if(digitalRead(Bmenu)==LOW)
  {

/*    
 if (! rtc.begin()) {
   lcd.println("SEM RTC !!");
    while (1);
  }


  if (! rtc.isrunning()) {
   Serial.println("Acertar a Data e a Hora");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    //rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  */
// ***************************************************   ANO
g=0;
flag=0;
 lcd.clear();
  lcd.print("ANO Prima 3s"); 
  delay(3000);
 
  if(digitalRead(Bmenu) == LOW)
{
   delay(1000);
 for (g=0; g<10; g++)
  {
    if(g==0){ANORTC =2020;}
    delay(20);
    if(g==1){ANORTC =2021;}
    delay(20);
    if(g==2){ANORTC =2022;}
    delay(20);
    if(g==3){ANORTC =2023;}
   delay(20);
    if(g==4){ANORTC =2024;}
   delay(20);
    if(g==5){ANORTC =2025;}
   delay(20);
    if(g==6){ANORTC =2026;}
    delay(20);
    if(g==7){ANORTC =2027;}
   delay(20);
    if(g==8){ANORTC =2028;}
    delay(20);
    if(g==9){ANORTC =2029;}
   delay(20);
    if(g==10){ANORTC =2030;}
    delay(20);
     if(g==11){ANORTC =2031;}
    delay(20);
     
   lcd.clear();
   delay(200);
  lcd.print(ANORTC);
  delay(10);
 // Serial.println(ANORTC);
    delay(1000);
  
  if(digitalRead(Bmenu)==LOW)
  {
 lcd.clear();
  delay(50);
 lcd.setCursor(0,0);
  delay(50);
  lcd.print("OK ! ANO");
  delay(50);
  lcd.setCursor(0,1);
   delay(50);
  lcd.print(ANORTC);
    delay(2000);
    break;

  }
 
  }//Fim do for
  }// Fim do digital BMenu HIGH

  //************************************************  MES
  g=0;
 lcd.clear();
  lcd.print("MES Prima 3s"); 
  delay(3000);
 
  if(digitalRead(Bmenu) != HIGH)
{
   delay(1000);
 for (g=0; g<=11; g++)
  {
    if(g==0){MESRTC =1;}
    delay(50);
    if(g==1){MESRTC =2;}
     delay(50);
    if(g==2){MESRTC =3;}
    delay(50);
    if(g==3){MESRTC =4;}
    delay(50);
    if(g==4){MESRTC =5;}
    delay(50);
    if(g==5){MESRTC =6;}
    delay(50);
    if(g==6){MESRTC =7;}
    delay(50);
    if(g==7){MESRTC =8;}
    delay(50);
    if(g==8){MESRTC =9;}
    delay(50);
    if(g==9){MESRTC =10;}
     delay(50);
    if(g==10){MESRTC =11;}
    delay(50);
    if(g==11){MESRTC =12;}
    delay(50);
    lcd.clear();
   delay(200);
  lcd.print(MESRTC);
  delay(10);
 // Serial.println(MESRTC);
    delay(1000);
  
  if(digitalRead(Bmenu)==LOW)
  {
 lcd.clear();
  delay(50);
 lcd.setCursor(0,0);
  delay(50);
  lcd.print("OK ! MES.");
  delay(50);
  lcd.setCursor(0,1);
   delay(50);
  lcd.print(MESRTC);
  delay(2000);
  break;

  }
 
  }//Fim do for
  }// Fim do digital BMenu HIGH

  //****************************************************   DIA
  g=0;
 lcd.clear();
  lcd.print("Dia, Prima 3s"); 
  delay(3000);
 
  if(digitalRead(Bmenu) != HIGH)
{
   delay(1000);
 for (g=0; g<=30; g++)
  {
    if(g==0){DIARTC=1;}
   delay(20);
    if(g==1){DIARTC=2;}
   delay(20);
    if(g==2){DIARTC=3;}
   delay(20);
    if(g==3){DIARTC=4;}
    delay(20);
    if(g==4){DIARTC=5;}
    delay(20);
    if(g==5){DIARTC=6;}
   delay(20);
    if(g==6){DIARTC=7;}
   delay(20);
    if(g==7){DIARTC=8;}
   delay(20);
    if(g==8){DIARTC=9;}
   delay(20);
    if(g==9){DIARTC=10;}
   delay(20);
    if(g==10){DIARTC=11;}
   delay(20);
    if(g==11){DIARTC=12;}
    delay(20);
    if(g==12){DIARTC=13;}
    delay(20);
    if(g==13){DIARTC=14;}
    delay(20);
    if(g==14){DIARTC=15;}
   delay(20);
    if(g==15){DIARTC=16;}
    delay(20);
    if(g==16){DIARTC=17;}
   delay(20);
    if(g==17){DIARTC=18;}
    delay(20);
    if(g==18){DIARTC=19;}
    delay(20);
    if(g==19){DIARTC=20;}
    delay(20);
    if(g==20){DIARTC=21;}
    delay(20);
    if(g==21){DIARTC=22;}
    delay(20);
    if(g==22){DIARTC=23;}
   delay(20);
    if(g==23){DIARTC=24;}
    delay(20);
    if(g==24){DIARTC=25;}
    delay(20);
    if(g==25){DIARTC=26;}
     delay(20);
    if(g==26){DIARTC=27;}
    delay(20);
    if(g==27){DIARTC=28;}
    delay(20);
    if(g==28){DIARTC=29;}
    delay(20);
    if(g==29){DIARTC=30;}
    delay(20);
    if(g==30){DIARTC=31;}
    delay(20);
    
    lcd.clear();
   delay(200);
  lcd.print(DIARTC);
  delay(10);
 // Serial.println(DIARTC);
    delay(1000);
  
  if(digitalRead(Bmenu)==LOW)
  {
 lcd.clear();
  delay(50);
 lcd.setCursor(0,0);
  delay(50);
  lcd.print("OK ! DIA.");
  delay(50);
  lcd.setCursor(0,1);
   delay(50);
  lcd.print(DIARTC);
  delay(2000);
  break;
  }
 
  }//Fim do for
  }// Fim do digital BMenu HIGH

 // ******************************************************************  HORA
 g=0;
 lcd.clear();
  lcd.print("HORA, Prima 3s"); 
  delay(3000);
 
  if(digitalRead(Bmenu) != HIGH)
{
   delay(1000);
 for (g=0; g<=22; g++)
  {
    if(g==0){HORARTC =1;}
    delay(20);
    if(g==1){HORARTC =2;}
    delay(20);
    if(g==2){HORARTC =3;}
    delay(20);
    if(g==3){HORARTC =4;}
    delay(20);
    if(g==4){HORARTC =5;}
    delay(20);
    if(g==5){HORARTC =6;}
    delay(20);
    if(g==6){HORARTC =7;}
    delay(20);
    if(g==7){HORARTC =8;}
    delay(20);
    if(g==8){HORARTC =9;}
    delay(20);
    if(g==9){HORARTC =10;}
    delay(20);
    if(g==10){HORARTC =11;}
   delay(20);
    if(g==11){HORARTC =12;}
    delay(20);
    if(g==12){HORARTC =13;}
    delay(20);
    if(g==13){HORARTC =14;}
    delay(20);
    if(g==14){HORARTC =15;}
    delay(20);
    if(g==15){HORARTC =16;}
    delay(20);
    if(g==16){HORARTC =17;}
    delay(20);
    if(g==17){HORARTC =18;}
   delay(20);
    if(g==18){HORARTC =19;}
    delay(20);
    if(g==19){HORARTC =20;}
    delay(20);
    if(g==20){HORARTC =21;}
    delay(20);
    if(g==21){HORARTC =22;}
    delay(20);
    if(g==22){HORARTC =23;}
    delay(20);
    
    lcd.clear();
   delay(200);
  lcd.print(HORARTC);
  delay(10);
 // Serial.println(HORARTC);
    delay(1000);
  
  if(digitalRead(Bmenu)==LOW)
  {
 lcd.clear();
  delay(50);
 lcd.setCursor(0,0);
  delay(50);
  lcd.print("OK ! HORA.");
  delay(50);
  lcd.setCursor(0,1);
   delay(50);
  lcd.print(HORARTC);
  delay(2000);
  break;

  }
 
  }//Fim do for
  }// Fim do digital BMenu HIGH

 // ***********************************************************   MINUTOS
 g=0;
 lcd.clear();
  lcd.print("MINUTOS,Prima 3s"); 
  delay(3000);
 
  if(digitalRead(Bmenu) != HIGH)
{
     delay(1000);
 for (g=0; g<=59; g++)
  {
    if(g==0){MINUTOSRTC =1;}
    delay(10);
    if(g==1){MINUTOSRTC =2;}
    delay(10);
    if(g==2){MINUTOSRTC =3;}
    delay(10);
    if(g==3){MINUTOSRTC =4;}
    delay(10);
    if(g==4){MINUTOSRTC =5;}
  delay(10);
    if(g==5){MINUTOSRTC =6;}
   delay(10);
    if(g==6){MINUTOSRTC =7;}
   delay(10);
    if(g==7){MINUTOSRTC =8;}
   delay(10);
    if(g==8){MINUTOSRTC =9;}
    delay(10);
    if(g==9){MINUTOSRTC =10;}
    delay(10);
    if(g==10){MINUTOSRTC =11;}
   delay(10);
    if(g==11){MINUTOSRTC =12;}
   delay(10);
    if(g==12){MINUTOSRTC =13;}
   delay(10);
    if(g==13){MINUTOSRTC =14;}
    delay(10);
    if(g==14){MINUTOSRTC =15;}
  delay(10);
    if(g==15){MINUTOSRTC =16;}
   delay(10);
    if(g==16){MINUTOSRTC =17;}
    delay(10);
    if(g==17){MINUTOSRTC =18;}
    delay(10);
    if(g==18){MINUTOSRTC =19;}
    delay(10);
    if(g==19){MINUTOSRTC =20;}
    delay(10);
    if(g==20){MINUTOSRTC =21;}
    delay(10);
    if(g==21){MINUTOSRTC =22;}
    delay(10);
     if(g==22){MINUTOSRTC =23;}
    delay(10);
    if(g==23){MINUTOSRTC =24;}
     delay(10);
    if(g==24){MINUTOSRTC =25;}
    delay(10);
    if(g==25){MINUTOSRTC =26;}
    delay(10);
    if(g==26){MINUTOSRTC =27;}
    delay(10);
    if(g==27){MINUTOSRTC =28;}
    delay(10);
    if(g==28){MINUTOSRTC =29;}
    delay(10);
    if(g==29){MINUTOSRTC =30;}
    delay(10);
    if(g==30){MINUTOSRTC =31;}
    delay(10);
    if(g==31){MINUTOSRTC =32;}
     delay(50);
    if(g==32){MINUTOSRTC =33;}
    delay(50);
    if(g==33){MINUTOSRTC =34;}
    delay(10);
    if(g==34){MINUTOSRTC =35;}
   delay(10);
    if(g==35){MINUTOSRTC =36;}
    delay(10);
    if(g==36){MINUTOSRTC =37;}
    delay(10);
    if(g==37){MINUTOSRTC =38;}
    delay(10);
    if(g==38){MINUTOSRTC =39;}
    delay(10);
    if(g==39){MINUTOSRTC =40;}
    delay(10);
    if(g==40){MINUTOSRTC =41;}
   delay(10);
    if(g==41){MINUTOSRTC =42;}
    delay(10);
    if(g==42){MINUTOSRTC =43;}
    delay(10);
    if(g==43){MINUTOSRTC =44;}
   delay(10);
     if(g==44){MINUTOSRTC =45;}
   delay(10);
    if(g==45){MINUTOSRTC =46;}
    delay(10);
    if(g==46){MINUTOSRTC =47;}
    delay(10);
    if(g==47){MINUTOSRTC =48;}
    delay(10);
    if(g==48){MINUTOSRTC =49;}
   delay(10);
    if(g==49){MINUTOSRTC =50;}
   delay(10);
    if(g==50){MINUTOSRTC =51;}
   delay(10);
    if(g==51){MINUTOSRTC =52;}
   delay(10);
    if(g==52){MINUTOSRTC =53;}
   delay(10);
    if(g==53){MINUTOSRTC =54;}
    delay(10);
    if(g==54){MINUTOSRTC =55;}
   delay(10);
    if(g==55){MINUTOSRTC =56;}
   delay(10);
    if(g==56){MINUTOSRTC =57;}
   delay(10);
    if(g==57){MINUTOSRTC =58;}
   delay(10);
    if(g==58){MINUTOSRTC =59;}
  delay(10);
    
        
    lcd.clear();
   delay(200);
  lcd.print(MINUTOSRTC);
  delay(10);
 // Serial.println(MINUTOSRTC);
    delay(1000);
  
  if(digitalRead(Bmenu)==LOW)
  {
 lcd.clear();
  delay(50);
 lcd.setCursor(0,0);
  delay(50);
  lcd.print("OK ! MINUTOS");
  delay(50);
  lcd.setCursor(0,1);
   delay(50);
  lcd.print(MINUTOSRTC);
   rtc.adjust(DateTime(ANORTC, MESRTC, DIARTC, HORARTC, MINUTOSRTC, 0));
   flag=1;// Depois de acertados os minutos prepara-se este flag para se sair do "While"
   break;// Sai dio "for" para o "while"
  
  }
 
  }//Fim do for
  
  }// Fim do digital BMenu HIGH

  }// Fim do if Bmenu == LOW
  
if (flag==1){break;}// Se o Flag está em 1 sai o break faz sair novamente para o menu...
 
 delay(1000);
  t=t+1;
  }



// ***************************************************** FIM DO ACERTO DO  RTC ******************************************************************************

button.attachClick(click);
button.attachDoubleClick(doubleclick);
button.attachLongPressStart(press);// Com a actualização do onebutton.h esta declaração teve de ser alterada em 2021/03/27
digitalWrite (CWAz,LOW);
digitalWrite (CCWAz,LOW);
digitalWrite (CWEl,LOW);
digitalWrite (CCWEl,LOW);

//Valores de calibração "RESET" do Rotor
LerKBY();

azim=0;
elev=0;
delay(10);
KBYAz= analogRead(PotAzVal);
azim = float(map(KBYAz,KBYINIAZ,KBYFIMAZ,0,3300))/10;
KBYEl=analogRead(PotElVal);
elev =float(map(KBYEl,KBYINIEL,KBYFIMEL,900,0))/10;


lcd.clear();
lcd.setCursor(0,0);
lcd.print("TRACKUINO ct4bb");
delay(4000);
DataHoraRTC();
delay(2000);
lcd.print("Verif.data do SD");
lerDataSD();// Lê só a data ANO MÊS DIA do ficheiro SOL.txt do SD e compara com a data actual do relógio RTC. Validando ou não o cartão SD como controlo.
//sondastr="Sem objecto";// Retirado ???? Restabelecia sempre duas vezes o objecto no arranque.
/*
 //Para verificar o espaço da SRAM
  char *heapend=sbrk(0);
  register char * stack_ptr asm ("sp");
  struct mallinfo mi=mallinfo();
  printf("\nDynamic ram used: %d\n",mi.uordblks);
  printf("Program static ram used %d\n",&_end - ramstart);
  printf("Stack ram used %d\n\n",ramend - stack_ptr);
  printf("My guess at free mem: %d\n",stack_ptr - heapend + mi.fordblks);
*/
  

}
//Fim do SetUp

//**************************************************************************************  LOOP   
void loop()
{
  
Serial.print("KBYINIAZ=" );
Serial.println(KBYINIAZ);
Serial.print("KBYFIMAZ=");
Serial.println(KBYFIMAZ); 
Serial.print("KBYINIEL=");
Serial.println(KBYINIEL);
Serial.print("KBYFIMEL=");
Serial.println(KBYFIMEL);


  s=0;
while(s<=3)
{
DataHoraRTC();
delay2S();
lcd.clear();
delay(10);
lcd.setCursor(0,0);
delay(20);
lcd.print(sondastr);
Serial.println(sondastr);
lcd.setCursor(12,0);
lcd.print(offset);
 delay(10);
lcd.setCursor(0,1);
delay(10);
lcd.print("Az=");
 delay(10);
lcd.print(azim,0);//com uma casa decimal
 delay(10);
lcd.print(" El=");
 delay(10);
lcd.print(elev,0);// com uma casa decimal
 delay(10);
delay4S();
delay4S();

s++;
}
//************** I if a seguir, faz com que não havendo objectos seleccionados, as datas dos ficheiros não necessitam de ser lidas. 
//Ou seja, quando o sondastr for Nulo, salta sem ler a DataHora do SDCARD
if(sondastr.compareTo("Sem objecto")!=0)
{
lerHM_AZ_EL();
}
}

 
