#include <SPI.h>
int CS=8;
char POWER_CTL=0x2D;
char DATA_FORMAT=0x31;
char DATAX0=0x32;
char DATAX1=0x33;
char DATAY0=0x34;
char DATAY1=0x35;
char DATAZ0=0x36;
char DATAZ1=0x37;
unsigned char values[10];
int x,y,z;
void setup(){
SPI.begin();
SPI.setDataMode(SPI_MODE3);
Serial.begin(9600);
pinMode(CS,OUTPUT);
digitalWrite(CS,HIGH);
writeRegister(DATA_FORMAT, 0x01);
writeRegister(POWER_CTL,0x08);  
}
void loop()
{
readRegister(DATAX0,6,values);
x=((int)values[1]<<8)|(int)values[0];
y=((int)values[3]<<8)|(int)values[2];
z=((int)values[5]<<8)|(int)values[4];
Serial.print(x,DEC);
Serial.print(',');  
Serial.print(y,DEC);
Serial.print(',');  
Serial.print(z,DEC);
delay(10);
}
void writeRegister(char registerAddress,char value){
digitalWrite(CS,LOW);
SPI.transfer(registerAddress);
SPI.transfer(value);
digitalWrite(CS,HIGH);
}
void readRegister(char registerAddress,int numBytes,unsigned char * values)
{
  char address=0x80 | registerAddress;
  if(numBytes>1)address=address|0x40;
  digitalWrite(CS,LOW);
  SPI.transfer(address);
  for(int i=0;i<numBytes;i++){
    values[i]=SPI.transfer(0x00);
  }
  digitalWrite(CS,HIGH);
}
