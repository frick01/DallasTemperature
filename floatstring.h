/*=======================================================================================================

                                        BIBLIOTECA PARA CONVERSAO DE FLOAT PARA STRING
                                     PLATAFORMA DE DESENVOLVIMENTO: TM4C123GH6PM
                                           DESENVOLVEDOR : GUILHEREME FRICK
                                                        2018
=========================================================================================================*/

static char table[]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};//aqui está a tabela com correspondência entre números de caracteres //com números digitais
void Float2Str(char *str, double number, uint8_t int_bit, uint8_t decimal_bit)
{

uint8_t i;

uint32_t temp = (uint8_t)number/1;

double t2 = 0.0;

for (i = 1; i<=int_bit; i++)     //aqui lidar com a parte int
{
if(temp==0)
str[int_bit-i] = table [0];
else
str[int_bit-i] = table [temp%10];

temp = temp/10;
}

*(str+int_bit) = '.';  //aqui lidar com a saída "."
temp = 0;
t2 = number;


for(i=1; i<=decimal_bit; i++)    //aqui lidar com a parte decima
{

temp = t2*10;
str[int_bit+i] = table[temp%10];
t2 = t2*10;
}

*(str+int_bit+decimal_bit+1) = '\0';
}


 