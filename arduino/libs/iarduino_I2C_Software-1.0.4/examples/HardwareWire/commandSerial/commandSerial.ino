// КОМАНДЫ ИЗ МОНИТОРА УПРАВЛЯЮТ ЧТЕНИЕМ/ЗАПИСЬЮ ДАННЫХ I2C:        // * Строки со звёздочкой являются необязательными.
// При старте скетча, в мониторе появится список всех команд.       //
// Данный список можно можно повторно запросить командой '?'.       //
                                                                    //
/*   ПРИМЕР ОТПРАВКИ КОМАНД:                                        //
 *   Выполним следующие действия:                                   //
 *   Читаем 3 регистра, модуля с адресом 0x9,  начнём с регистра 4. //
 *   Запишем значения 1,2,3 в тот же модуль,   начнём с регистра 6F.//
 *   Читаем 2 регистра, модуля с адресом 0x10, начнём с регистра 6. //
 *   Читаем 2 регистра, модуля с адресом 0x10, начнём с регистра 4. //
 *                                                                  //
 *   Введите следующие строки:                                      //
 *   address 9                                                      //   В мониторе появится надпись: УСТАНОВКА АДРЕСА МОДУЛЯ...                           Установлен адрес модуля 0x09
 *   get 4 3                                                        //   В мониторе появится надпись: УСТАНОВЛЕН РЕЖИМ ЧТЕНИЯ 0x03 РЕГИСТРОВ               Чтение регистров:[модуль 0x09, регистры 0x04-0x06] = 0x09 0x04 0x12
 *   set 6F 1 2 3                                                   //   В мониторе появится надпись: УСТАНОВЛЕН РЕЖИМ ЗАПИСИ В РЕГИСТРЫ, НАЧИНАЯ С 0x6F   Запись значений: [модуль 0x09, регистры 0x6F-0x71] = 0x01 0x02 0x03 - успешно
 *   address 10                                                     //   В мониторе появится надпись: УСТАНОВКА АДРЕСА МОДУЛЯ...                           Установлен адрес модуля 0x10
 *   get 6 2                                                        //   В мониторе появится надпись: УСТАНОВЛЕН РЕЖИМ ЧТЕНИЯ 0x02 РЕГИСТРОВ               Чтение регистров:[модуль 0x10, регистры 0x06-0x07] = 0x12 0x3C
 *   4                                                              //   Последняя команда сохраняется. Значит мы можем указать только номер первого из двух читаемых регистров, без команды.
 *                                                                  //
 *   ВАЖНО: Все числа вводятся и выводятся в 16-ти ричном формате.  //
 */                                                                 //
                                                                    //
#include <Wire.h>                                                   //   Подключаем библиотеку для работы с аппаратной шиной I2C.
                                                                    //   Далее в примере используется объект Wire для работы с основной аппаратной шиной I2C, но можно использовать объекты Wire1, Wire2 ... если они поддерживаются платой.
uint8_t  fncFind     (int*);                                        //   Объявляем  функцию поиска устройств на шине I2C в указанный массив.  Возвращает количество найденных адресов.
uint8_t  fncRead     (uint8_t,uint8_t,int*,uint8_t);                //   Объявляем  функцию чтения параметры: адрес_модуля, регистр, массив, количество. Возвращает код ошибки.
uint8_t  fncRead     (uint8_t,        int*,uint8_t);                //   Объявляем  функцию чтения параметры: адрес_модуля,          массив, количество. Возвращает код ошибки.
uint8_t  fncWrite    (uint8_t,uint8_t,int*,uint8_t);                //   Объявляем  функцию записи параметры: адрес_модуля, регистр, массив, количество. Возвращает код ошибки.
uint8_t  fncWrite    (uint8_t,        int*,uint8_t);                //   Объявляем  функцию записи параметры: адрес_модуля,          массив, количество. Возвращает код ошибки.
void     fncShiftArr (void);                                        //   Объявляем  функцию сдвига данных в массиве чисел valArr.
void     fncPrintERR (uint8_t);                                     //   Объявляем  функцию вывода ошибки чтения/записи по указанному коду.
void     fncPrintHEX (uint8_t);                                     //   Объявляем  функцию вывода указанного числа в формате '0x00'.
void     fncPrintInfo(void);                                        //   Объявляем  функцию вывода подсказки по вводу команд.
int      valArr[255];                                               //   Объявляем  массив для хранения чисел.
uint8_t  lenArr;                                                    //   Объявляем  переменную для хранения количества чисел в массиве valArr.
char     strCom[11];                                                //   Объявляем  строку для хранения принятой команды.
uint8_t  lenCom;                                                    //   Объявляем  переменную для хранения количества символов в строке команды strCom.
bool     flgCom;                                                    //   Объявляем  флаг указывающий на поступление команды.
char     valMode=' ';                                               //   Определяем текущий режим работы.
uint8_t  valAddr=0;                                                 //   Определяем переменную для хранения адреса модуля с которым требуется работать.
uint8_t  valReg;                                                    //   Объявляем  переменную для хранения регистра модуля с которым требуется работать.
uint8_t  valSum;                                                    //   Объявляем  переменную для хранения количества регистров модуля с которыми требуется работать.
                                                                    //
void setup(){                                                       //
     delay(500);                                                    // * Ждём завершение переходных процессов связанных с подачей питания.
     Serial.begin(9600);                                            //   Инициируем передачу данных в монитор последовательного порта на скорости 9600 бит/сек.
     while(!Serial){;}                                              // * Ждём завершения инициализации шины UART.
     fncPrintInfo();                                                //   Печатаем подсказку по вводу команд с монитора.
     Wire.begin();                                                  //   Инициируем работу с аппаратной шиной I2C.
//   Wire.setClock(400000);                                         // * Изменить скорость передачи данных на указанную. По умолчанию скорость 100'000 бит/сек.
//   Wire.setPins(5,6);                                             // * Поддерживается не всеми платами. Изменить назначенные ранее выводы шины I2C на указанные: SDA, SCL.
//   Wire.setWireTimeout(25000);                                    // * Поддерживается не всеми платами. Задать таймаут ожидания ведомого в мкс (0 = таймаут отключён). Если вторым параметром указать true, то по достижении таймаута шина будет отключена, как end().
//   Wire.end();                                                    // * Отключение шины. Для включения нужно вновь инициировать шину begin().
}                                                                   //
                                                                    //
void loop(){                                                        //
//   Если введены данные в монитор:                                 //
     if( Serial.available()>0 ){                                    //
     //  Готовимся к приёму команды из монитора:                    //
         lenArr=0; lenCom=0; flgCom=0;                              //   Сбрасываем размер массива чисел lenArr, размер строки команды lenCom и флаг поступления команды flgCom
         memset(valArr,0,255);                                      //   Обнуляем массив чисел valArr.
         strCom[lenCom]=0;                                          //   Чистим строку команды strCom.
     //  Посимвольно получаем данные в valArr и strCom:             //
         while( Serial.available() ){                               //
             char i=Serial.read();                                  //   Читаем очередной символ в переменную i.
             int &j=valArr[lenArr];                                 //   Создаём ссылку на элемент массива чисел valArr[lenArr].
             if( i==' '||i=='.'||i==',' ){ lenArr++;     }else      //   Если принят пробел или точка или запятая, то увеличиваем индекс lenArr массива valArr (переходим к получению следующего числа).
             if( i>='0' && i<='9' )      { j=j*16+i-'0'; }else      //   Если принята цифра, то умножаем ранее принятое число на 16 и добавляем принятую цифру.
             if( i=='A' || i=='a' )      { j=j*16+10;    }else      //   Если принята цифра, то умножаем ранее принятое число на 16 и добавляем принятую цифру.
             if( i=='B' || i=='b' )      { j=j*16+11;    }else      //   Если принята цифра, то умножаем ранее принятое число на 16 и добавляем принятую цифру.
             if( i=='C' || i=='c' )      { j=j*16+12;    }else      //   Если принята цифра, то умножаем ранее принятое число на 16 и добавляем принятую цифру.
             if( i=='D' || i=='d' )      { j=j*16+13;    }else      //   Если принята цифра, то умножаем ранее принятое число на 16 и добавляем принятую цифру.
             if( i=='E' || i=='e' )      { j=j*16+14;    }else      //   Если принята цифра, то умножаем ранее принятое число на 16 и добавляем принятую цифру.
             if( i=='F' || i=='f' )      { j=j*16+15;    }else      //   Если принята цифра, то умножаем ранее принятое число на 16 и добавляем принятую цифру.
             if( i!='\r'&& i!='\n')      { valMode='r';  }          //   Если приняты символы не относящиеся к цифрам, то временно устанавливаем режим 'r' - error.
             if( i=='\r'|| i=='\n')      { ;             }else      //   Не добавляем символы переноса каретки в строку команды strCom.
             if( lenCom<10 && lenArr==0 ){ strCom[lenCom++]=i; }    //   Добавляем принятый символ в строку команды strCom и символ конца строки (для сравнения с реальными командами).
             strCom[lenCom]=0;                                      //   Последний символ команды strCom = символ конца строки 0.
             delay(5);                                              //   Устанавливаем задержку для получения всех символов строки (т.к. 9600 это медленная скорость).
         }   lenArr++;                                              //   Увеличиваем количество элементов массиве чисел valArr, т.к. если не вводились символы ' ', '.', ',', то индекс lenArr==0.
     //  Преобразуем прописные символы команды в строчные:          //
         for(uint8_t i=0; i<lenCom; i++){                           //   Проходим по всем символам строки команды strCom.
             if(strCom[i]>='A' && strCom[i]<='Z'){strCom[i]+=' ';}  //   Если очередной символ входит в диапазон A-Z, то меняем его на соответствующий a-z.
         }                                                          //
     //  Сверяем данные в строке strCom с имеющимися командами:     //
              if( !strcmp(strCom,     "find"   )                    ){   flgCom=true;  valMode='f'; Serial.println( F("======================\r\nПОИСК АДРЕСОВ НА ШИНЕ I2C:"                   ) ); }
         else if( !strcmp(strCom, "a" "find"   )                    ){   flgCom=true;  valMode='F'; Serial.println( F("======================\r\nПОСТОЯННЫЙ ПОИСК АДРЕСОВ НА ШИНЕ I2C:"        ) ); }
         else if( !strcmp(strCom,     "stop"   )                    ){   flgCom=true;  valMode='S'; Serial.println( F("======================\r\nВЫВОД ОСТАНОВЛЕН!"                            ) ); }
         else if( !strcmp(strCom,     "address")                    ){   flgCom=true;  valMode='a'; Serial.println( F("======================\r\nУСТАНОВКА АДРЕСА МОДУЛЯ..."                   ) ); }
         else if( !strcmp(strCom,     "get"    )                    ){   flgCom=true;  valMode='g'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ЧТЕНИЯ "                     ) ); }
         else if( !strcmp(strCom, "a" "get"    )                    ){   flgCom=true;  valMode='G'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ПОСТОЯННОГО ЧТЕНИЯ "         ) ); }
         else if( !strcmp(strCom,     "getnr"  )                    ){   flgCom=true;  valMode='x'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ЧТЕНИЯ "                     ) ); }
         else if( !strcmp(strCom, "a" "getnr"  )                    ){   flgCom=true;  valMode='X'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ПОСТОЯННОГО ЧТЕНИЯ "         ) ); }
         else if( !strcmp(strCom,     "set"    )                    ){   flgCom=true;  valMode='w'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ЗАПИСИ В РЕГИСТР"            ) ); }
         else if( !strcmp(strCom, "a" "set"    )                    ){   flgCom=true;  valMode='W'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ПОСТОЯННОЙ ЗАПИСИ В РЕГИСТР" ) ); }
         else if( !strcmp(strCom,     "setnr"  )                    ){   flgCom=true;  valMode='y'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ЗАПИСИ"                      ) ); }
         else if( !strcmp(strCom, "a" "setnr"  )                    ){   flgCom=true;  valMode='Y'; Serial.print  ( F("======================\r\nУСТАНОВЛЕН РЕЖИМ ПОСТОЯННОЙ ЗАПИСИ"           ) ); }
         else if( !strcmp(strCom,     "?"      )                    ){   flgCom=false; valMode=' '; fncPrintInfo(); return; }
         else                                                        {   flgCom=false; if(valMode=='r'){ valMode=' '; Serial.print( F("ОШИБКА - НЕКОРРЕКТНАЯ КОМАНДА \"") ); Serial.print(strCom); Serial.println('"'); return; } }
     //  Если поступила команда, проверяем и сохраняем параметры:   //
         if( flgCom ){                                              //
             fncShiftArr();                                         //   Сдвигаем все элементы массива чисел valArr, т.к. при поступлении команды его первый элемент valArr[0] содержит хлам из символов команды.
             /* ->  address A */   if( valMode=='a'                 ){   if( lenArr!=1            ){ flgCom=false; }else{ valAddr=valArr[0];} }
             /* ->  get R [N] */   if( valMode=='g' || valMode=='G' ){   if( lenArr<1 || lenArr>2 ){ flgCom=false; }else{ valReg=valArr[0];                valSum=valArr[1]; if(valSum==0){valSum=1;} if(valSum==1){Serial.print("ОДНОГО");}else{fncPrintHEX(valSum);} Serial.print(F(" РЕГИСТР")); Serial.print(valSum==1?"А (для чтения другого":"ОВ (для смены первого"); Serial.println(F(" регистра можно вести только его адрес, без команды get)")); }}
             /* ->  getnr [N] */   if( valMode=='x' || valMode=='X' ){                                                                                     valSum=valArr[0]; if(valSum==0){valSum=1;} Serial.print(valSum==1?"БАЙТА":"БАЙТОВ"); Serial.println(F(" БЕЗ УКАЗАНИЯ НОМЕРА ПЕРВОГО РЕГИСТРА (для чтения дргого количества байт, введите их количество, без команды getnr)")); }
             /* ->  set R D...*/   if( valMode=='w' || valMode=='W' ){   if( lenArr<2             ){ flgCom=false; }else{ valReg=valArr[0]; fncShiftArr(); valSum=lenArr; Serial.print(valSum==1?" ":"Ы, НАЧИНАЯ С "); fncPrintHEX(valReg); Serial.println(F(" (для записи другого или других значений, введите их без команды set и без номера регистра)")); }}
             /* ->  setnr D...*/   if( valMode=='y' || valMode=='Y' ){   if( lenArr<1             ){ flgCom=false; }else{                                  valSum=lenArr; Serial.print(valSum==1?" БАЙТА":"БАЙТОВ");  Serial.println(F(" БЕЗ УКАЗАНИЯ НОМЕРА ПЕРВОГО РЕГИСТРА (для записи другого или других значений, введите их без команды setnr)")); }}
                                   if(!flgCom       ){ valMode=' ';      Serial.println( F(" ОШИБКА - НЕКОРРЕКТНЫЙ ПАРАМЕТР!") ); return; }
     //  Если поступили параметры без команды, тоже их сохраняем:   //
         }else{                                                     //
             /* ->  address A */   if( valMode=='a'                 ){   if( lenArr==1 ){ valAddr = valArr[0]; flgCom=true; } }else // После команда address можно неоднократно вводить новый адрес без команды.
             /* ->  get R [N] */   if( valMode=='g' || valMode=='G' ){   if( lenArr==1 ){ valReg  = valArr[0]; flgCom=true; } }else // После команды get     можно неоднократно вводить новый номер регистра для его(их) чтения без команды.
             /* ->  getnr [N] */   if( valMode=='x' || valMode=='X' ){   if( lenArr==1 ){ valSum  = valArr[0]; flgCom=true; } }else // После команды getnr   можно неоднократно вводить новое количество вновь читаемых байт без команды.
             /* ->  set R D...*/   if( valMode=='w' || valMode=='W' ){   if( lenArr>0  ){ valSum  = lenArr;    flgCom=true; } }else // После команды set     можно неоднократно вводить новые значения(е) для записи в те же (тот же) регистр без команды.
             /* ->  setnr D...*/   if( valMode=='y' || valMode=='Y' ){   if( lenArr>0  ){ valSum  = lenArr;    flgCom=true; } }else // После команды setnr   можно неоднократно вводить новые значения(е) для записи без команды.
             /* ->  введён параметр для неизвестной команды */       {   Serial.println( F("ОШИБКА - ПРЕДЫДУЩАЯ КОМАНДА НЕПОДДЕРЖИВАЕТ ПАРАМЕТРЫ!") ); return; }
         }                                                          //
     }                                                              //
//   Выполнение действий и вывод результата:                        //
     if( millis()%1000<10 || flgCom ){ delay(10);                   //   Если начинается очередная секунда с момента старта скетча, или поступила команда.
         uint8_t tmp;                                               //
         switch(valMode){                                           //
             case 'f': if( !flgCom   ){ break; }                    //   Команда поиска устройств на шине: find.
             case 'F': tmp = fncFind(valArr);                       //   Команда поиска устройств на шине: afind.
                       if(tmp){                                          Serial.print(F("Найдено ")); fncPrintHEX(tmp); Serial.print(F(" устройств")); Serial.print(((tmp%10==1)&&(tmp%100!=11))?"o:":(((tmp%10>1)&&(tmp%10<5)&&(tmp%100!=12)&&(tmp%100!=13)&&(tmp%100!=14))?"a:":":")); for(uint8_t i=0; i<tmp; i++){ Serial.print(' '); fncPrintHEX(valArr[i]);} Serial.println(); }
                       else   {                                          Serial.print(F("Устройства не найдены\r\n")); }
             break;                                                 //
             case 'a': if( !flgCom   ){ break; }                    //   Команда указания адреса: 'a' - address.
                                                                         Serial.print(F("Установлен адрес модуля ")); fncPrintHEX(valAddr); Serial.println();
             break;                                                 //
             case 'g': if( !flgCom   ){ break; }                    //   Команда чтения из регистров: get.
             case 'G': tmp = fncRead(valAddr,valReg,valArr,valSum); //   Команда чтения из регистров: aget.
                       if( valSum==1 ){                                  Serial.print(F("Чтение регистра: [модуль ")); fncPrintHEX(valAddr); Serial.print(F(", регистр " )); fncPrintHEX(valReg);                                                                                                                                     Serial.print(F("] =")); if( tmp==0 ){                                  Serial.print(' '); fncPrintHEX(valArr[0]);  Serial.println(); }else{ Serial.print(" отказано в доступе "); fncPrintERR(tmp); } }
                       else           {                                  Serial.print(F("Чтение регистров:[модуль ")); fncPrintHEX(valAddr); Serial.print(F(", регистры ")); fncPrintHEX(valReg); Serial.print('-'); fncPrintHEX(valReg+valSum-1);                                                                                    Serial.print(F("] =")); if( tmp==0 ){ for(uint8_t i=0; i<valSum; i++){ Serial.print(' '); fncPrintHEX(valArr[i]);} Serial.println(); }else{ Serial.print(" отказано в доступе "); fncPrintERR(tmp); } }
             break;                                                 //
             case 'x': if( !flgCom   ){ break; }                    //   Команда чтения без указания регистра: getnr.
             case 'X': tmp = fncRead(valAddr, valArr, valSum);      //   Команда чтения без указания регистра: agetnr.
                       if( valSum==1 ){                                  Serial.print(F("Чтение байта:    [модуль ")); fncPrintHEX(valAddr); Serial.print(F(", запрошен один байт] = "));                                                                                                                                                                     if( tmp==0 ){                                                     fncPrintHEX(valArr[0]);  Serial.println(); }else{ Serial.print(" отказано в доступе "); fncPrintERR(tmp); } }
                       else           {                                  Serial.print(F("Чтение байтов:   [модуль ")); fncPrintHEX(valAddr); Serial.print(F(", запрошено "            )); fncPrintHEX(valSum);     Serial.print(F(" байт")); if((valSum%10>1)&&(valSum%10<5)&&((valSum%100<12)||(valSum%100>14))){Serial.print('a');} Serial.print(F("] =")); if( tmp==0 ){ for(uint8_t i=0; i<valSum; i++){ Serial.print(' '); fncPrintHEX(valArr[i]);} Serial.println(); }else{ Serial.print(" отказано в доступе "); fncPrintERR(tmp); } }
             break;                                                 //
             case 'w': if( !flgCom   ){ break; }                    //   Команда записи в регистры: set.
             case 'W': tmp = fncWrite(valAddr,valReg,valArr,valSum);//   Команда записи в регистры: aset.
                       if( valSum==1 ){                                  Serial.print(F("Запись значения: [модуль ")); fncPrintHEX(valAddr); Serial.print(F(", регистр " )); fncPrintHEX(valReg);                                                                                                                                     Serial.print(F("] = "));                                                                  fncPrintHEX(valArr[0]);  if( tmp==0 ){ Serial.println(" - успешно"); }else{ Serial.print(" - неудачно "); fncPrintERR(tmp); } }
                       else           {                                  Serial.print(F("Запись значений: [модуль ")); fncPrintHEX(valAddr); Serial.print(F(", регистры ")); fncPrintHEX(valReg); Serial.print('-'); fncPrintHEX(valReg+valSum-1);                                                                                    Serial.print(F("] =" ));              for(uint8_t i=0; i<valSum; i++){ Serial.print(' '); fncPrintHEX(valArr[i]);} if( tmp==0 ){ Serial.println(" - успешно"); }else{ Serial.print(" - неудачно "); fncPrintERR(tmp); } }
             break;                                                 //
             case 'y': if( !flgCom   ){ break; }                    //   Команда записи без указания регистра: setnr.
             case 'Y': tmp = fncWrite(valAddr, valArr, valSum);     //   Команда записи без указания регистра: asetnr.
                       if( valSum==1 ){                                  Serial.print(F("Запись значения: [модуль ")); fncPrintHEX(valAddr); Serial.print(F(", записывается один байт] = "));                                                                                                                                                                                                                                   fncPrintHEX(valArr[0]);  if( tmp==0 ){ Serial.println(" - успешно"); }else{ Serial.print(" - неудачно "); fncPrintERR(tmp); } }
                       else           {                                  Serial.print(F("Запись значений: [модуль ")); fncPrintHEX(valAddr); Serial.print(F(", записываются "             )); fncPrintHEX(valSum); Serial.print(F(" байт")); if((valSum%10>1)&&(valSum%10<5)&&((valSum%100<12)||(valSum%100>14))){Serial.print('a');} Serial.print(F("] ="));               for(uint8_t i=0; i<valSum; i++){ Serial.print(' '); fncPrintHEX(valArr[i]);} if( tmp==0 ){ Serial.println(" - успешно"); }else{ Serial.print(" - неудачно "); fncPrintERR(tmp); } }
             break;                                                 //
         }                                                          //
         flgCom=false;                                              //   Сбрасываем флаг поступления команды
     }                                                              //
}                                                                   //
                                                                    //
//      Функции чтения записи и вывода данных:                      //
uint8_t fncFind    (                       int* m                   ){ uint8_t j=0; for(uint8_t i=1; i<128; i++){ Wire.beginTransmission(i); if( Wire.endTransmission()==0 ){ m[j++]=i; } } return j; }
uint8_t fncRead    ( uint8_t a, uint8_t r, int* m, uint8_t n        ){ Wire.beginTransmission(a); Wire.write(r); uint8_t i = Wire.endTransmission(false); if(i){ return i; } i = Wire.requestFrom(a, n); if(i==0){ for(i=0; i<n; i++){m[i]=0;} return 6; } if(i!=n){ for(i=0; i<n; i++){m[i]=0;} return 7; } i=0; while( Wire.available() ){ m[i++]=Wire.read(); } return 0; }
uint8_t fncRead    ( uint8_t a,            int* m, uint8_t n        ){                                           uint8_t i =                                                     Wire.requestFrom(a, n); if(i==0){ for(i=0; i<n; i++){m[i]=0;} return 6; } if(i!=n){ for(i=0; i<n; i++){m[i]=0;} return 7; } i=0; while( Wire.available() ){ m[i++]=Wire.read(); } return 0; }
uint8_t fncWrite   ( uint8_t a, uint8_t r, int* m, uint8_t n        ){ Wire.beginTransmission(a); Wire.write(r); for(uint8_t i=0; i<n; i++){ Wire.write(m[i]); } return Wire.endTransmission(); }
uint8_t fncWrite   ( uint8_t a,            int* m, uint8_t n        ){ Wire.beginTransmission(a);                for(uint8_t i=0; i<n; i++){ Wire.write(m[i]); } return Wire.endTransmission(); }
void    fncShiftArr( void                                           ){ for(uint8_t i=1; i<lenArr; i++){ valArr[i-1]=valArr[i]; } lenArr--; valArr[lenArr]=0; }
void    fncPrintHEX( uint8_t i                                      ){ Serial.print( i<16? "0x0":"0x" ); Serial.print(i,HEX); }
void    fncPrintERR( uint8_t i                                      ){
        switch(i){
            case 1: Serial.println(F("переполнен буфер"                         )); break;
            case 2: Serial.println(F("получен NACK при отправке адреса модуля"  )); break;
            case 3: Serial.println(F("получен NACK при отправке адреса регистра")); break;
            case 4: Serial.println(F("неизвестная ошибка"                       )); break;
            case 5: Serial.println(F("timeout ожидания"                         )); break;
            case 6: Serial.println(F("модуль не вернул ни одного байта данных"  )); break;
            case 7: Serial.println(F("модуль вернул иное количество байт данных")); break;
        }
}
void fncPrintInfo(void){
     Serial.println(F("КОМАНДЫ:"));
     Serial.println(F("?  - - - - - - - - вывести данную подсказку"));
     Serial.println(F("find - - - - - - - выполнить поиск всех доступных адресов на шине I2C"));
     Serial.println(F("afind  - - - - - - то же самое, но выполнять каждую секунду"));
     Serial.println(F("stop - - - - - - - остановить ежесекундный вывод данных"));
     Serial.println(F("                   (описанные выше команды вводятся без параметров)"));
     Serial.println(F("address A  - - - - работать с модулем у которого установлен адрес A на шине I2С"));
     Serial.println(F("                   (по умолчанию A=0 (адрес общего вызова)"));
     Serial.println(F(" get R [N] - - - - начиная с регистра R прочитать N регистров из модуля с адресом A "));
     Serial.println(F("aget R [N] - - - - то же самое, но выполнять каждую секунду"));
     Serial.println(F("                   (по умолчанию N=1 (получить один байт из указанного регистра)"));
     Serial.println(F(" getnr [N] - - - - прочитать N регистров из модуля с адресом A без указания регистра"));
     Serial.println(F("agetnr [N] - - - - то же самое, но выполнять каждую секунду"));
     Serial.println(F("                   (по умолчанию N=1 (получить один байт без указания регистра)"));
     Serial.println(F(" set R D [... Dn]  записать начиная с регистра R указанное количество байт (D...Dn) в модуль с адресом A"));
     Serial.println(F("aset R D [... Dn]  то же самое, но выполнять каждую секунду"));
     Serial.println(F("                   (длина вводимой строки не более 255 символов)"));
     Serial.println(F(" setnr D [... Dn]  записать один [или несколько] байт (D...Dn) в модуль с адресом A, без указания регистра"));
     Serial.println(F("asetnr D [... Dn]  то же самое, но выполнять каждую секунду"));
     Serial.println(F("                   (длина вводимой строки не более 255 символов)"));
     Serial.println(F("ПАРАМЕТРЫ:"));
     Serial.println(F("                   параметры разделяются пробелом, запятой или точкой"));
     Serial.println(F("                   все параметры указываются в 16-ричном формате, без префикса 0x"));
     Serial.println(F("[N] [... Dn]       квадратные скобки в этой подсказке означают, что параметр не обязательный"));
     Serial.println(F("A R N D [... Dn]   эти параметры можно указывать без команды, если они участвовали в предыдущей команде"));
}
