
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "math.h"
#include "stdlib.h"
#include "math.h"
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "bibliotecafrick/floatstring.h"
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


#define reset_pulse             480         //Delay utilizado no reset de 480us
#define bit_start_pulse           2       //  
#define presence_delay           40        // Deley que é utilizado para esperar e escravo ennviar
                                           // um pulso de presença para o mestre.
#define post_presence_delay     480        // Delay após o mestre detectar o pulso de presença.
#define bit_read_delay            5                        
#define bit_write_delay         100                        

#define temp_LSB               0
#define temp_MSB                1
#define cnt_remain              6
#define cnt_per_s              7
#define scrpd_len               9     

#define addr_len                8

#define FALSE 0
#define TRUE  1
/*
========================================================================================================
COMANDOS ROM
Depois que o mestre do barramento detectou um pulso de presença, pode emitir um comando ROM. Esses 
comandos operam nos códigos exclusivos de ROM de 64 bits de cada dispositivo escravo e permitir que o 
mestre selecione um dispositivo específico se muitos estão presentes no barramento de 1 fio. Esses
comandos também permitir que o mestre determine quantos e quais tipos de dispositivos estão presentes
no barramento ou em qualquer dispositivo experimentou uma condição de alarme. Existem cinco ROM comandos,
e cada comando tem 8 bits de comprimento. O mestre dispositivo deve emitir um comando ROM apropriado 
antes emitindo um comando de função DS18B20. 
========================================================================================================
COMANDO SEARCH_ROM_CMD [0XF0]: Quando um sistema é inicialmente ligado, o mestre deveidentificar os 
códigos ROM de todos os dispositivos escravos no barramento, que permite ao mestre determinar o número
de escravos e seus tipos de dispositivos. O mestre aprende o Códigos ROM através de um processo de 
eliminação que requer o mestre para executar um ciclo de ROM de pesquisa (por exemplo, Comando ROM 
seguido de troca de dados) quantos vezes necessário para identificar todos os dispositivos escravos.
Se houver apenas um escravo no barramento, o mais simples O comando ROM [33h] pode ser usado no lugar 
da search rom . 
========================================================================================================
COMANDO READ_ROM_CMD [0X33]: Este comando só pode ser usado quando há um escravo no ônibus. Ele permite 
que o mestre de ônibus leia os Código ROM de 64 bits sem usar o procedimento de ROM de pesquisa.
Se este comando é usado quando há mais de um escravo presente no ônibus, uma colisão de dados ocorrerá
quando todos os escravos tentam responder ao mesmo tempo.
========================================================================================================
COMANDO MACH_ROM_CMD [0X55]: 
O comando match ROM seguido por uma ROM de 64 bits seqüência de código permite que o mestre do barramento
dispositivo escravo específico em um barramento multiponto ou single-drop. Apenas o escravo que 
corresponde exatamente ao código ROM de 64 bits sequência responderá ao comando de função emitido pelo
mestre; todos os outros escravos no ônibus vão esperar por um redefinir pulso.
========================================================================================================
COMANDO SKIP_ROM_CMD [0XCC]: O mestre pode usar este comando para endereçar todos os dispositivos no 
bus simultaneamente sem enviar nenhuma ROM informações de código. Por exemplo, o mestre pode fazer tudo
DS18B20s no barramento executam temperatura simultânea conversões emitindo um comando Skip ROM seguido
por um comando Convert T [44h].
========================================================================================================
COMANDO ALRM_SEARCH_ROM [0XEC]: A operação deste comando é idêntica à operação do comando Search ROM,
exceto que apenas os escravos com um sinalizador de alarme definido responderá.
========================================================================================================

*/
#define search_rom_cmd         0xF0
#define read_rom_cmd           0x33
#define match_rom_cmd          0x55
#define skip_rom_cmd           0xCC
#define alarm_search_cmd       0xEC

/*========================================================================================================
COMANDOS DE FUNÇÃO: Esses comandos permitem que o mestre grave e leia a partir da memória de rascunho 
do DS18B20, inicie conversões de temperatura e  determine o modo de fornecimento de energia. 
É importante observar que omestre pode emitir um dos comandos de função do DS18B20 A seguir, lista de 
comandos de função relevantes para o DS18B20. 
=========================================================================================================
Função CONVERTE T [0x44]:  Usado pelo Mestre para instruir o Escravo a iniciar a 
conversão de temperatura. Se o DS18B20 for alimentado por uma fonte externa, 
o mestre pode emitir intervalos de tempo de leitura após o comando Convert T 
e o DS18B20 responderá transmitindo um 0 enquanto a conversão de temperatura 
estiver em andamento e 1 quando a conversão estiver concluída. A conversão de 
temperatura leva um mínimo de 750 ms. Então, depois de emitir o comando, o mestre tem que esperar pelo 
mínimo de 750 ms antes que o bus busque a resposta do escravo.
=========================================================================================================
Função WRITE_MEM_CMD [0X4E]: Este comando permite ao mestre escrever 3 bytes de dados para o scratchpad 
do DS18B20. O primeiro byte de dados é escrito no registrador TH (byte 2 do scratchpad), o segundo
byte é escrito no registrador TL (byte 3), e o terceiro byte é escrito no registrador de configuração 
(byte 4). Dados deve ser transmitido o bit menos significativo primeiro. Todos três bytes devem ser 
escritos antes que o mestre emita um reset, ou os dados podem estar corrompidos.
=========================================================================================================
Função READ_MEM_CMD [0XBE]: Este comando permite ao mestre ler o conteúdo do screatchpad do DS18b20. 
A transferência de dados começa com o menos significativo bit de byte 0 e continua pelo scratchpad
até que o 9º byte (byte 8 - CRC) seja lido. O mestre pode emitir um reset para terminar a leitura a 
qualquer momento se apenas parte dos dados do bloco de anotações é necessário.
=========================================================================================================
Função COPY_MEM_CMD [0X48]: Este comando copia o conteúdo do scratchpad TH, TL e registros de 
configuração (bytes 2, 3 e 4) para EEPROM. Este comando copia o conteúdo do scratchpad no  dispositivo 
Slave para EEPROM também no dispositivo Slave. DS18B20 saída será 0 no ônibus enquanto ele está ocupado
copiar o rascunho de E 2; ele irá retornar a 1 quando o processo de cópia for concluída.
=========================================================================================================
Função RECALL_EE_CMD [0xB8]: Esse comando recupera os valores do 
acionador de  alarme e os dados de configuração da EEPROM e coloca os dados nos 
bytes 2, 3 e 4, respectivamente, na memória do rascunho.
=========================================================================================================
*/

#define PD1 (*((volatile long*)0x40007008)) // Pino digital utilizado para comunicação one-wire

#define convert_temp_cmd       0x44                           
#define write_mem_cmd          0x4E
#define read_mem_cmd           0xBE
#define copy_mem_cmd           0x48
#define recall_ee_cmd          0xB8

#define temp_res               256

#define DS18B20_family_code    0x28     //código exclusivo da familia 1-wire
#define DS18S20_family_code    0x10   

void delayMs(uint32_t ui32Ms)
void delayUs(uint32_t ui32Us) 

short DS1820_reset();
short DS1820_read_bit();
void DS1820_write_bit(short bit_value);
unsigned char DS1820_read_byte();
void DS1820_write_byte(unsigned char value);    
void DS1820_device_addr(unsigned char addr_method);   
short DS1820_find_next_device();
short DS1820_find_first_device();
void DS1820_write_EEPROM(unsigned char THigh, unsigned char TLow);
signed long DS1820_get_raw_temp();
float DS1820_get_temp();

static short done;
static unsigned char last_discrepancy;
static unsigned char RomAddr[addr_len];  