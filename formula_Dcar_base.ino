/* 
 *  Ejemplo de codigo Formula Dcar
 *  Seguimiento de degradado con 4 sensores de línea
 *  Sensores frontal y laterales 
 *  Para teensy 3.2
*/
#include <TimerOne.h> // per la interrupcio de temps
#include <EEPROM.h> // Per desar les dades de calibracio

#define SPFront 21
#define SPDret 9
#define SPEsq 7

#define SF_E1 A3
#define SF_E0 A2
#define SF_D0 A1
#define SF_D1 A0


#define led_orange 13

int DIRM1=12;
int PWM1=5;
int DIRM2=4;
int PWM2=6;

bool ledState=LOW;
int led = 13;
int polsador=2;
int incomingByte;
bool estat=0;


int lectura[4]={0,0,0,0};
int maxs[4]={0,0,0,0};
int mins[4]={1000,1000,1000,1000};

volatile int posicio=0;
volatile int posicio_ant=0;
volatile int temps=1;
unsigned char Kp=120; // Com mes petita, mes corregeix
unsigned char Kd=15; // Com mes gran, mes corregeix
volatile int velocitat=100;
volatile int velocitatc=180;
volatile int centre=-150;
int dins=-250;
int fora=250;

volatile char comptador_robot_davant=0;
volatile char comptador_robot_dreta=0;
volatile char comptador_robot_esquerra=0;
volatile char robot_davant=0;
volatile char robot_dreta=0;
volatile char robot_esquerra=0;
volatile bool moviment_motors=1;
volatile int K=0;

volatile bool disponible_adelanta=0;
volatile bool canvi_carril=0;
volatile int temps_fora=0;


void setup() {
  Serial.begin(115200);
  pinMode(DIRM1, OUTPUT);
  pinMode(DIRM2, OUTPUT);
  analogWriteFrequency(PWM1, 23438); // Quan cremaven 46875); // 
  analogWriteResolution(10);  // analogWrite value 0 to 1023

  pinMode(led_orange, OUTPUT); 
         
  pinMode(polsador,INPUT_PULLUP);
    
  pinMode(SPFront,INPUT);
  pinMode(SPDret,INPUT);
  pinMode(SPEsq,INPUT);  
}

void loop() {
  sequencia_inici();
  if (!digitalRead(polsador))
      calibra_maxmin();
  else
      llegir_maxmins_eeprom();

  delay(1000);
  seleccio_velocitat();
  while(digitalRead(polsador))
  {
    calcula_posicio();
    if (posicio>0)
      {
        if (posicio>(fora+5))
            {
              orange_off();
            }
         else 
         {
         if (posicio<(fora-5))
            {
              orange_off();
            }
          else
            {
              orange_on();
            }
         }    
      }
      else
            {
        if (posicio>(dins+5))
            {
              orange_off();
            }
         else 
         {
         if (posicio<(dins-5))
            {
              orange_off();
            }
            else
            {
              orange_on();
            }
         }    
      }
    
  }
  if (posicio>0)
    centre=fora;
  else
    centre=dins;
  delay(1000);

  Timer1.initialize(1000); //pd cada 1 ms.
  Timer1.attachInterrupt(pd); // 
  Timer1.start();
  velocitatc=velocitat=100+estrategia*20;

   while (1)
  {

    if ((centre==dins)&&(robot_davant==1)&&(robot_dreta==0))
    {
      canvi_fora();
      delay(2000);
      temps_fora=0;
      robot_davant=0;
      robot_esquerra=0;
      robot_dreta=0;
    }
    
    if ((centre==fora)&&(robot_davant==1)&&(robot_esquerra==0))
    {
      canvi_dins();
      delay(2000);
      robot_davant=0;
      robot_esquerra=0;
      robot_dreta=0;
    }
    
    if ((centre==fora)&&(temps_fora>10000)&&(robot_esquerra==0))
    {
      canvi_dins();
      delay(2000);
      robot_davant=0;
      robot_esquerra=0;
      robot_dreta=0;
    }
  // si robot davant i estas dins --> canvia a fora i t'esperes 2s
  // si robot davnat i estas fora --> canvia a dins i t'esperes 2s
  }
  atura();
  delay(2000);
}


void seleccio_velocitat(void)
{
  estrategia=0;
  while (digitalRead(polsador)) // mentre no confirmem amb el boto 
  {
    if (!digitalRead(SPDret)) // si tapem sensor lateral esquerra
    {
      estrategia++;
      orange_on();
      while (!digitalRead(SPDret));
      delay(200);
      orange_off();
    }
  }
  for (char i=0;i<estrategia;i++)
  {
    orange_on();
    delay(200);
    orange_off();
    delay(300);
  }
}
  

void atura(void)
{
  Timer1.stop();
  delay(10);
  mou(0,0);
}

void pd(void)
{
 
 if (digitalRead(SPFront))
 {
    comptador_robot_davant=0;
     if (velocitat < velocitatc)
         velocitat++;
 }
 else
 {
    comptador_robot_davant++;
    if (comptador_robot_davant>20)
         robot_davant=1;
     if (velocitat > 0)
         velocitat--;
     if (velocitat<50)
        turbina.write(0);
 }
 if (digitalRead(SPDret))
 {
    if (comptador_robot_dreta>0)
    {
        comptador_robot_dreta--;
    }
    else
      robot_dreta=0;
 }
else
    {
      comptador_robot_dreta+=5;
      if (comptador_robot_dreta>50)
          robot_dreta=1;
    }
 if (digitalRead(SPEsq))
 {
    if (comptador_robot_esquerra>0)
    {
        comptador_robot_esquerra--;
    }
    else
      robot_esquerra=0;
 }
else
    {
      comptador_robot_esquerra+=5;
      if (comptador_robot_esquerra>50)
          robot_esquerra=1;
    }
      
  // calculem Ep
  if (centre==fora)
      temps_fora++;
  calcula_posicio();
  int errortf;
  int errordf=0;
  int errorpf=0;
  if (abs(posicio-centre)>100) // si estic molt deplacat, corregeixo a ma
      {
        if (posicio>centre)
            errorpf=velocitat;
        else
            errorpf=-velocitat;
      }
   else
   {
      errorpf=velocitat*(posicio-centre);
      errorpf=errorpf/Kp;
   }

  //calculem Ed
  if (posicio==posicio_ant)
  {
      temps++;
      if (temps>500)
          temps=500;
      errordf=errordf*(temps-1)/temps;
  }
  else
  {
      errordf=Kd*velocitat*(posicio-posicio_ant)/100;
      temps=1;
  }
  //Serial.println(posicio,DEC);
  //Serial.println(errorpf,DEC);
  // Et
  errortf= errorpf + errordf;
 
  // Anem a moure motors
  if (moviment_motors){
    if (K!=0)
    {
       if (errortf > velocitat*(100-K)/100)
           errortf=velocitat*(100-K)/100;
       if (errortf< -velocitat*(100-K)/100)
           errortf = -velocitat*(100-K)/100; 
    } 
       if (errortf>= 0){
            mou(velocitat,velocitat-errortf);
            }
       else
      mou(velocitat+errortf,velocitat);
  }

}

void calcula_posicio(){
 //llegim
      posicio_ant=posicio;
      lectura[0]=analogRead(SF_E1);
      lectura[1]=analogRead(SF_E0);
      lectura[2]=analogRead(SF_D0);
      lectura[3]=analogRead(SF_D1);
 //transformem 
      for (int j=0;j<4;j++){
      lectura[j]=map(lectura[j],mins[j],maxs[j],0,400);
      }
  // mitjanes posicio
                if (abs(lectura[0]-lectura[3])>150){
                    if (lectura[0]<lectura[3])
                        posicio=-400;//piano esquerra
                    else
                        posicio=400;//piano dret
                }
                else
                {
                  // zones laterals
                  posicio=(lectura[1]+lectura[2])/2;
                  if (lectura[0]>lectura[3])
                      posicio=-posicio;
                  // zona central
                  if (abs(posicio)<50){
                    posicio=(lectura[1]+lectura[2])/2;
                    if (lectura[1]>lectura[2]){
                        posicio=-posicio;
                    }
                    }
                }
}

void calibra_maxmin(){
//  red_on();
  allibera_polsador();
  for (int j=0;j<4;j++){
    maxs[j]=0;
    mins[j]=1000;
  }
  delay(1000);
  orange_on();
    while (digitalRead(polsador)){
      lectura[0]=analogRead(SF_E1);
      lectura[1]=analogRead(SF_E0);
      lectura[2]=analogRead(SF_D0);
      lectura[3]=analogRead(SF_D1);
          
      for (int j=0;j<4;j++){
        if (lectura[j]>maxs[j])
            maxs[j]=lectura[j];
        if (lectura[j]<mins[j])
            mins[j]=lectura[j];   
      } 
    }
  orange_off();
 //Escrivim dades a l'eeprom
        for (int j=0;j<4;j++){
          EEPROMWriteInt(2*j+100, maxs[j]);
          delay(5);
          EEPROMWriteInt(2*j+108, mins[j]);
          delay(5);
      }
     // red_off();
}

void orange_on(void)
{
  digitalWrite(led_orange,HIGH);
}

void orange_off(void)
{
  digitalWrite(led_orange,LOW);
}

void mou(int md, int me)
{
  if (md>0){
  digitalWrite(DIRM1,LOW);
  analogWrite(PWM1,md);
}
else
{
  digitalWrite(DIRM1,HIGH);
  analogWrite(PWM1,-md);
}


if (me>0)
{
  digitalWrite(DIRM2,HIGH);
  analogWrite(PWM2,me);
}
else
{
  digitalWrite(DIRM2,LOW);
  analogWrite(PWM2,-me);
}
}


void allibera_polsador(){
  while (digitalRead(polsador));
  delay(200);
  while (!digitalRead(polsador));
}


//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
     {
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);

     EEPROM.write(p_address, lowByte);
     EEPROM.write(p_address + 1, highByte);
     }

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
     {
     byte lowByte = EEPROM.read(p_address);
     byte highByte = EEPROM.read(p_address + 1);

     return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
     }

void llegir_maxmins_eeprom(void) {
                  for (int j=0;j<4;j++){
                    maxs[j]=EEPROMReadInt(2*j+100);
                    mins[j]=EEPROMReadInt(2*j+108);                          
                      
                Serial.print(maxs[j]);
                delay(10);
                Serial.print(" ");
                delay(10);
                Serial.println(mins[j]); 
                delay(10);              
                }
}

void sequencia_inici(void){
  for (int i=0;i<3;i++){
  orange_on();
  delay(100);
  orange_off();
  delay(100);
  }
  orange_on();
  delay(200);
  orange_off();
}



void canvi_fora(void)
{
  turbina.write(70);
  moviment_motors=0;
  delay(5);
  mou(max(velocitat*7/10,70),max(velocitat,100));
  calcula_posicio();
  while (abs(posicio-fora)>50);
  moviment_motors=1;
  centre=fora;
  robot_davant=0;
  turbina.write(60);
}

void canvi_dins(void)
{
  turbina.write(70);
  moviment_motors=0;
  delay(5);
  mou(max(velocitat,100),max(velocitat*5/10,70));
  calcula_posicio();
  while (abs(posicio-dins)>50);
  moviment_motors=1;
  centre=dins;
  robot_davant=0;
  turbina.write(60);
}
