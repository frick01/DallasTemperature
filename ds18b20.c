
#include"ds18b20.h"
#include"floatstring.h"


/*=======================================================================================================

                                        BIBLIOTECA PARA SENSOR DE TEMPERATURA 
                                                       DS18B20 
                                          PROTOCOLO DE COMUNICAÇÃO ONE-WIRE
                                     PLATAFORMA DE DESENVOLVIMENTO: TM4C123GH6PM
                                           DESENVOLVEDOR : GUILHEREME FRICK
                                                        2018
=========================================================================================================
PINAGEM : |1- GND|
          |2-  DQ|-> Pino de dados entrada\saída para operação one wire.
          |3- VCC|

Necessita-se colocar um resistor de pull-up de 4.7k entre o pino de dados e o vcc
========================================================================================================*/



#define PD1 (*((volatile long*)0x40007008)) // Pino digital utilizado para comunicação one-wire
                           
  
/*======================================================================================================
                                          DELAYS 
========================================================================================================*/

void delayMs(uint32_t ui32Ms) {

   // 1 clock cycle = 1 / SysCtlClockGet() second
   // 1 SysCtlDelay = 3 clock cycle = 3 / SysCtlClockGet() second
   // 1 second = SysCtlClockGet() / 3
   // 0.001 second = 1 ms = SysCtlClockGet() / 3 / 1000
   
   SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3 / 1000));
}

void delayUs(uint32_t ui32Us) {
   SysCtlDelay(ui32Us * (SysCtlClockGet() / 3 / 1000000));
}

/*=======================================================================================================
INICIALIZAÇAO : INITIALIZATION
Todas as transações no barramento 1-Wire começam com uma seqüência de inicialização. A sequência de 
inicialização consiste de um impulso de reposição transmitida pelo mestre de barramento seguido por 
pulso (s) presença transmitida pelo escravo (s).
=========================================================================================================
FUNÇAO DE RESET: 
*/

short DS1820_reset()
{
   short pres_pulse_state = 0;

  // disable_interrupts(GLOBAL);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
   GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_1); 
   GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,0);                 //output low
   delayUs(reset_pulse);                                       //delay de 480us 
   GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1);        //output high                                 
   GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_1);           //pino configurado como entrada
   delayUs(presence_delay);                                    //delay de 40 us.
   pres_pulse_state = GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1); //lê o pulso de presença.
   delayUs(424);                   
   
   //enable_interrupts(GLOBAL);

   return pres_pulse_state;                                    //retorna o bit do pulso de detecção
}  
/*=======================================================================================================
 Passos para o mestre emitir um pedido de leitura para o dispositivo escravo no dispositivo de escravo 
de leite bus aka
* puxar o barramento para baixo
* segure por 2us
* liberar o barramento
* esperar por 45us por recuperação (46us + 5us = 61us)
========================================================================================================*/

short DS1820_read_bit()                                          
{              
   short bit_value = 0;

   //disable_interrupts(GLOBAL);     
   GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_1);
   GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,0);               //output low
   delayUs(bit_start_pulse);                                 //2us
   GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_1);         //modo entrada
   GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1);                  //lê a entrada
   delayUs(bit_read_delay);                                  //5us de delay para a leitura
   bit_value = GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1);
   //enable_interrupts(GLOBAL);
   return bit_value;
}   


/*=======================================================================================================


*/
void DS1820_write_bit(short bit_value)   
{
   //disable_interrupts(GLOBAL); 
   GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_1);
   GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,0);//output low
   delayUs(bit_start_pulse);                           
   if (bit_value != 0)        
   {
      GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1);//output high 
   }

   delayUs(bit_write_delay);
  GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1);//output high 
   
   //enable_interrupts(GLOBAL);
}   

unsigned char DS1820_read_byte()
{                         
   unsigned char s = 0; 
   unsigned char value = 0;
                                                      
   for(s = 0; s < 8; s++)
   {
      if (DS1820_read_bit())
      {
         value |= (1 << s);
      }
      delayUs(120); 
   }
   return value;
} 

void DS1820_write_byte(unsigned char value)
{
   unsigned char s = 0;
   unsigned char tmp = 0;

   for(s = 0; s < 8; s++)      
   {                                    
      tmp = (value >> s);    
      tmp &= 0x01;          
      DS1820_write_bit(tmp); 
   }
                        
   delayUs(105); 
}   

void DS1820_device_addr(unsigned char addr_method)
{
   unsigned char s = 0;
   
   if(addr_method == match_rom_cmd)
   {
      DS1820_write_byte(match_rom_cmd);    
      for (s = 0; s < addr_len; s++)
      {
         DS1820_write_byte(RomAddr[s]);
       //  UARTprintf("%x", RomAddr[s]);// solicitaçao do endereço
      }
   }
   else
   {
      DS1820_write_byte(skip_rom_cmd);     
   }
}      


short DS1820_find_next_device()
{                         
    unsigned char state = 0;
    unsigned char byte_index = 0;
    unsigned char mask = 1;
    unsigned char bit_pos = 1;
    unsigned char discrepancy_marker = 0;
    short bit_b = FALSE;                   
    short b_Status = FALSE;   
    short next_b = FALSE;

    for(byte_index = 0; byte_index < 8; byte_index++)
    {
        RomAddr[byte_index] = 0x00;
    }         
                                                             
    b_Status = DS1820_reset(); 

    if(b_Status || done)    
    {
        last_discrepancy = 0;  
        return FALSE;
    }

    DS1820_write_byte(search_rom_cmd);
    
    byte_index = 0;
    do
    {
        state = 0;

        if(DS1820_read_bit() != 0)
        {
            state = 2;
          
        }
        delayUs(120); 

        if ( DS1820_read_bit() != 0)
        {
            state |= 1;
        }
        delayUs(120); 
        
        if (state == 3)
      
        {
            break;
        }
        else
        {
            if (state > 0)
            {
                bit_b = (short)(state >> 1);
            }
            else
            {
                if (bit_pos < last_discrepancy)
                {
                    bit_b = ((RomAddr[byte_index] & mask) > 0 );
                }
                else
                {
                    bit_b = (bit_pos == last_discrepancy);
                }

                if (bit_b == 0)
                {
                    discrepancy_marker = bit_pos;
                }
            }

           if (bit_b != 0)
           {
               RomAddr[byte_index] |= mask;
           }
           else
           {
               RomAddr[byte_index] &= ~mask;
           }

           DS1820_write_bit(bit_b);
           
           bit_pos++;                    
           mask <<= 1;

           if(mask == 0)
           {
               byte_index++;  
               mask = 1; 
           }
        }
    }while (byte_index < addr_len);

    if(bit_pos < 65)
    {
        last_discrepancy = 0;
    }
    else
    {
        last_discrepancy = discrepancy_marker;
        done = (last_discrepancy == 0);
        next_b = TRUE;
    }

    return next_b;
}


short DS1820_find_first_device()
{               
    last_discrepancy = 0;
    done = FALSE;

    return (DS1820_find_next_device());
}

                                          
void DS1820_write_EEPROM(unsigned char THigh, unsigned char TLow)
{           
    DS1820_reset();
    DS1820_device_addr(match_rom_cmd);
    DS1820_write_byte(write_mem_cmd); 
    DS1820_write_byte(THigh);
    DS1820_write_byte(TLow);
    delayUs(10);
    DS1820_reset();
    DS1820_device_addr(match_rom_cmd);
    DS1820_write_byte(copy_mem_cmd); 
    delayMs(10);
}

signed long DS1820_get_raw_temp()
{
    unsigned char s = 0;
    unsigned long tmp = 0;
    unsigned long highres;
    unsigned char scrpad[scrpd_len];
    
    DS1820_reset();
    DS1820_device_addr(match_rom_cmd);   
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1);//output high
    DS1820_write_byte(convert_temp_cmd); 
    delayMs(750);
    DS1820_reset();
    DS1820_device_addr(match_rom_cmd);
    DS1820_write_byte(read_mem_cmd); 

    for(s = 0; s < scrpd_len; s++)
    {
        scrpad[s] = DS1820_read_byte();
    }
    
    tmp = 0;                              
    tmp = (unsigned long)((unsigned long)scrpad[temp_MSB] << 8);
    tmp |= (unsigned long)(scrpad[temp_LSB]);

    if(RomAddr[0] == DS18S20_family_code)
    {                                       \
        tmp >>= 1;             
        tmp = ((unsigned long)tmp << 8);
                                           
        tmp -= ((unsigned long)temp_res >> 2);
                                                       
        highres = scrpad[cnt_per_s] - scrpad[cnt_remain];
        highres = ((unsigned long)highres << 8);
        if(scrpad[cnt_per_s])
        {                             
            highres = highres / ((unsigned long)scrpad[cnt_per_s]);
        }
                   
        highres = highres + tmp;
    }
    else
    {
        highres = tmp;
        highres <<= 4;
    }                      
                     
    return highres;                                  
}

float DS1820_get_temp()
{
    return ((float)DS1820_get_raw_temp() / (float)temp_res);
}                                               

/********************************************************************************************************/
 float t = 0.0;  
float t_ante=0.0;
int mediu_temp=0;
unsigned char sensor_count = 0;
void setup_sonda_temperatura(void)
{

 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
 GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
}
char strT1[20], strT2[20], strT3[20], strT4[20], strT5[20];
uint32_t T1cent, T2cent, T3cent, T4cent, T5cent;

void get_temperature_multiple_sensors(void)
 {
     sensor_count = 0;
      

    
      if(DS1820_find_first_device())
      { 
         do
        {      
           mediu_temp=0;
           UARTprintf("");
           if (sensor_count == 0){
           t = DS1820_get_temp(); 
           T1cent=t*1000;
           Float2Str(strT1,t,2,2 );
          
           }
          if (sensor_count == 1){
           t = DS1820_get_temp();
           T2cent=t*1000;
           Float2Str(strT2,t,2,2 );}
          if (sensor_count == 2){
           t = DS1820_get_temp();
           T3cent=t*1000;
           Float2Str(strT3,t,2,2 );}
           if (sensor_count == 3){
           t = DS1820_get_temp(); 
           T4cent=t*1000;
           Float2Str(strT4,t,2,2 );}
          if (sensor_count == 4){
           t = DS1820_get_temp(); 
           T5cent=t*1000;
           Float2Str(strT5,t,2,2 );}
          
          //UARTprintf("Sensor%d\n ", sensor_count );  
      
           sensor_count++;
            
            delayMs(2000);
         }
        
          
         while(DS1820_find_next_device());
     
        UARTprintf("\x1B[2J");     
        UARTprintf("\x1B[0;0H");
        UARTprintf("\nTmp %d/ C: %s\n ", sensor_count=0,strT1 );  
        UARTprintf("Tmp %d/ C: %s\n ", sensor_count=1,strT2 );  
        UARTprintf("Tmp %d/ C: %s\n ", sensor_count=2,strT3 );
        UARTprintf("Tmp %d/ C: %s\n ", sensor_count=3,strT4 );
        UARTprintf("Tmp %d/ C: %s\n ", sensor_count=4,strT5 );
       
        sensor_count = 0;    
           
             
             
      }    
     
     }

