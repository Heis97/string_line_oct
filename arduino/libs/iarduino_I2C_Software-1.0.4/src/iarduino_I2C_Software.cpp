#include "iarduino_I2C_Software.h"																				//
																												//
//		КОНСТРУКТОР КЛАССА:																						//
		SoftTwoWire::SoftTwoWire(void){																			//	Параметры:				отсутствуют.
			flgPinsEnabled			= false;																	//	Сбрасываем флаг подключения выводов SDA и SCL.
			flgBeginEnabled			= false;																	//	Сбрасываем флаг инициализации работы с программной шиной I2C.
			flgTimeoutEnabled		= false;																	//	Сбрасываем флаг инициализации таймаута ожидания ведомого.
			flgTimeoutTriggered		= false;																	//	Сбрасываем флаг сработавшего  таймаута ожидания ведомого.
			flgTimeoutReset			= false;																	//	Сбрасываем флаг указывающий отключить выводы end(), если сработал таймаут.
			flgMaster				= false;																	//	Сбрасываем флаг работы в качестве мастера.
			flgSlave				= false;																	//	Сбрасываем флаг работы в качестве ведомого.
			flgBeginTransmission	= false;																	//	Сбрасываем флаг инициализации передачи данных   ведомому.
}																												//
																												//
//		КОНСТРУКТОР КЛАССА:																						//
		SoftTwoWire::SoftTwoWire(int sda, int scl){																//	Параметры:				«sda» - номер вывода SDA, «scl» - номер вывода SCL.
			flgPinsEnabled			= false;																	//	Сбрасываем флаг подключения выводов SDA и SCL.
			flgBeginEnabled			= false;																	//	Сбрасываем флаг инициализации работы с программной шиной I2C.
			flgTimeoutEnabled		= false;																	//	Сбрасываем флаг инициализации таймаута ожидания ведомого.
			flgTimeoutTriggered		= false;																	//	Сбрасываем флаг сработавшего  таймаута ожидания ведомого.
			flgTimeoutReset			= false;																	//	Сбрасываем флаг указывающий отключить выводы end(), если сработал таймаут.
			flgMaster				= false;																	//	Сбрасываем флаг работы в качестве мастера.
			flgSlave				= false;																	//	Сбрасываем флаг работы в качестве ведомого.
			flgBeginTransmission	= false;																	//	Сбрасываем флаг инициализации передачи данных   ведомому.
			setPins(sda, scl);																					//	Назначаем выводы программной шины I2C.
}																												//
																												//
//		НАЗНАЧЕНИЕ ВЫВОДОВ:																						//
bool	SoftTwoWire::setPins(int sda, int scl){																	//	Параметры:				«sda» - номер вывода SDA, «scl» - номер вывода SCL.
			pinSDA         = sda;																				//	Сохраняем номер вывода «SDA».
			pinSCL         = scl;																				//	Сохраняем номер вывода «SCL».
		//	Сохраняем адреса портов и маски выводов:															//
			 pinMaskSDA    = digitalPinToBitMask(sda);															//	Сохраняем маску вывода «SDA» в регистрах выходных и выходных значений.
			 pinMaskSCL    = digitalPinToBitMask(scl);															//	Сохраняем маску вывода «SCL» в регистрах выходных и выходных значений.
			_pinMaskSDA    = ~pinMaskSDA;																		//	Сохраняем маску вывода «SDA» в регистрах выходных и выходных значений под инверсией.
			_pinMaskSCL    = ~pinMaskSCL;																		//	Сохраняем маску вывода «SCL» в регистрах выходных и выходных значений под инверсией.
			int portSDA    = digitalPinToPort(sda);																//	Получаем  номер порта на котором находится вывод  «SDA».
			int portSCL    = digitalPinToPort(scl);																//	Получаем  номер порта на котором находится вывод  «SCL».
		#if defined(ESP32)																						//
			portModeSDA    = (uint32_t*)portModeRegister   (portSDA);											//	Сохраняем адрес регистра конфигурирования  вывода «SDA».
			portModeSCL    = (uint32_t*)portModeRegister   (portSCL);											//	Сохраняем адрес регистра конфигурирования  вывода «SCL».
			portInputSDA   = (uint32_t*)portInputRegister  (portSDA);											//	Сохраняем адрес регистра входных  значений вывода «SDA».
			portInputSCL   = (uint32_t*)portInputRegister  (portSCL);											//	Сохраняем адрес регистра входных  значений вывода «SCL».
			portOutputSDA  = (uint32_t*)portOutputRegister (portSDA);											//	Сохраняем адрес регистра выходных значений вывода «SDA».
			portOutputSCL  = (uint32_t*)portOutputRegister (portSCL);											//	Сохраняем адрес регистра выходных значений вывода «SCL».
		#else																									//
			portModeSDA    = (uint16_t*)portModeRegister   (portSDA);											//	Сохраняем адрес регистра конфигурирования  вывода «SDA».
			portModeSCL    = (uint16_t*)portModeRegister   (portSCL);											//	Сохраняем адрес регистра конфигурирования  вывода «SCL».
			portInputSDA   = (uint16_t*)portInputRegister  (portSDA);											//	Сохраняем адрес регистра входных  значений вывода «SDA».
			portInputSCL   = (uint16_t*)portInputRegister  (portSCL);											//	Сохраняем адрес регистра входных  значений вывода «SCL».
			portOutputSDA  = (uint16_t*)portOutputRegister (portSDA);											//	Сохраняем адрес регистра выходных значений вывода «SDA».
			portOutputSCL  = (uint16_t*)portOutputRegister (portSCL);											//	Сохраняем адрес регистра выходных значений вывода «SCL».
		#endif																									//
		//	Конфигурируем выводы:																				//
			*portModeSDA   &= _pinMaskSDA;																		//	Конфигурируем вывод «SDA» как вход, который подтянется внешним резистором до Vcc.
			*portModeSCL   &= _pinMaskSCL;																		//	Конфигурируем вывод «SCL» как вход, который подтянется внешним резистором до Vcc.
		//	Устанавливаем на выводах уровень логического 0:														//
			*portOutputSDA &= _pinMaskSDA;																		//	Устанавливаем SDA=0. Уровень установится при конфигурировании вывода на выход.
			*portOutputSCL &= _pinMaskSCL;																		//	Устанавливаем SCL=0. Уровень установится при конфигурировании вывода на выход.
		//	Возвращаем флаг успешного подключения выводов SDA и SCL:											//
			flgPinsEnabled = true;																				//	Устанавливаем флаг подключения выводов SDA и SCL.
			return true;																						//
}																												//
																												//
//		ЧТЕНИЕ НОМЕРОВ НАЗНАЧЕННЫХ ВЫВОДОВ:																		//
bool	SoftTwoWire::getPins(int* sda, int* scl){																//
			if( flgPinsEnabled ){ *sda=pinSDA; *scl=pinSCL; return true; }else{ return false; }					//	Возвращаем true если выводы были назначены.
}																												//
																												//
//		ИНИЦИАЛИЗАЦИЯ РАБОТЫ ПО ШИНЕ I2C В КАЧЕСТВЕ МАСТЕРА:													//
bool	SoftTwoWire::begin(void){																				//	Параметры:				отсутствуют.
			if( flgPinsEnabled ){																				//	Если установлен флаг подключения выводов SDA и SCL.
				flgBeginEnabled		= true;																		//	Устанавливаем флаг инициализации работы с программной шиной I2C.
				flgMaster			= true;																		//	Устанавливаем флаг работы в качестве мастера.
				flgSlave			= false;																	//	Сбрасываем флаг работы в качестве ведомого.
				setClock(valBaudrate);																			//	Устанавливаем cкорость передачи данных в Гц.
				setWireTimeout();																				//	Устанавливаем время таймаута по умолчанию.
			}																									//
			return flgBeginEnabled;																				//	Возвращаем флаг инициализации работы с программной шиной I2C.
}																												//
																												//
//		ИНИЦИАЛИЗАЦИЯ РАБОТЫ ПО ШИНЕ I2C В КАЧЕСТВЕ ВЕДОМОГО:													//
bool	SoftTwoWire::begin(uint8_t addr){																		//	Параметр:				«addr» - адрес ведомого.
			if( flgPinsEnabled ){																				//	Если установлен флаг подключения выводов SDA и SCL.
				flgBeginEnabled		= false; // true - если в будущем добавить поддержку ведомого				//	Устанавливаем флаг инициализации работы с программной шиной I2C.
				flgMaster			= false;																	//	Сбрасываем флаг работы в качестве мастера.
				flgSlave			= true;																		//	Устанавливаем флаг работы в качестве ведомого.
				valAddress			= addr;																		//	Сохраняем адрес приёма/передачи данных.
				setClock(valBaudrate);																			//	Устанавливаем cкорость передачи данных в Гц.
				setWireTimeout();																				//	Устанавливаем время таймаута по умолчанию.
			}																									//
			return flgBeginEnabled;																				//	Возвращаем флаг инициализации работы с программной шиной I2C.
}																												//
																												//
//		ДЕИНИЦИАЛИЗАЦИЯ РАБОТЫ ПО ШИНЕ I2C:																		//
void	SoftTwoWire::end(void){																					//	Параметры:				отсутствуют.
		//	Сбрасываем флаги:																					//
			flgPinsEnabled			= false;																	//	Сбрасываем флаг подключения выводов SDA и SCL.
			flgBeginEnabled			= false;																	//	Сбрасываем флаг инициализации работы с программной шиной I2C.
			flgMaster				= false;																	//	Сбрасываем флаг работы в качестве мастера.
			flgSlave				= false;																	//	Сбрасываем флаг работы в качестве ведомого.
			flgBeginTransmission	= false;																	//	Сбрасываем флаг инициализации передачи данных ведомому.
		//	Конфигурируем выводы:																				//
			*portModeSCL &= _pinMaskSCL;																		//	Конфигурируем вывод «SCL» как вход, который подтянется внешним резистором до Vcc.
			*portModeSDA &= _pinMaskSDA;																		//	Конфигурируем вывод «SDA» как вход, который подтянется внешним резистором до Vcc.
}																												//
																												//
//		УСТАНОВКА СКОРОСТИ ПЕРЕДАЧИ ДАННЫХ:																		//
void	SoftTwoWire::setClock(uint32_t baudrate	){																//	Параметр:				«baudrate» - скорость передачи данных в Гц.
			valBaudrate				= baudrate;																	//	Скорость передачи данных в Гц.
		//	Определяем количество тактов микроконстроллера при выполнении функций управления выводами:			//
			float tactPerHalfPeriod	= F_CPU / valBaudrate / 2.0f;												//	Количество тактов МК на пол периода тактирования шины I2C.
			float tactPerCallSet	= 17.0f*getTactPegCalc(16) + 13.0f*getTactPegCalc(8);						//	Количество тактов МК на вызов функций установки логического уровня setXXX_X_HalfPeriod() без цикла ожидания полупериода.
			float tactPerCallGet	= 17;																		//	Количество тактов МК на вызов функций чтения    логического уровня getXXX без цикла ожидания освобождения линии ведомым.
			float tactPerWaitSet	= 5.0f*getTactPegCalc(16) + 4.0f*getTactPegCalc(8);							//	Количество тактов МК на один проход цикла ожидания полупериода в функции установки логического уровня setXXX_X_HalfPeriod().
			float tactPerWaitTimeOut= 14.0f*getTactPegCalc(16) + 16.0f*getTactPegCalc(8);						//	Количество тактов МК на один проход цикла проверки порта входных данных, до истечения таймаута.
		//	Определяем количество проходов циклов ожидания:														//
			float i = (tactPerHalfPeriod-tactPerCallSet-tactPerCallGet) / tactPerWaitSet; if(i<0){i=0.0f;}		//
			float j = F_CPU / 1000000L * valTimeout / tactPerWaitTimeOut;										//
			sumHalfPeriod			= (uint16_t)i;																//	Количество проходов цикла while с декрементом счётчика в функции setXXX_X_HalfPeriod() до истечения половины периода valBaudrate.
			sumTimeout				= (uint32_t)j;																//	Количество проходов цикла while с проверкой порта входных данных, до истечения таймаута.
}																												//
																												//
//		УСТАНОВКА ВРЕМЕНИ ОЖИДАНИЯ ВЕДОМОГО:																	//
void	SoftTwoWire::setWireTimeout(uint32_t timeout, bool reset){												//	Параметры:				«timeout» - время в мкс (если 0 значит таймаут отключён), «reset» - блокировать шину при превышении таймаута.
			if( flgBeginEnabled ){																				//	Если установлен флаг инициализации работы с программной шиной I2C.
				valTimeout			= timeout;																	//	Время ожидания (таймаут) в мкс.
				flgTimeoutReset		= reset;																	//	Устанавливаем флаг указывающий отключить выводы end(), если сработал таймаут.
				flgTimeoutEnabled   = timeout? true:false;														//	Устанавливаем флаг инициализации таймаута ожидания ведомого.
				flgTimeoutTriggered = false;																	//	Сбрасываем    флаг сработавшего  таймаута ожидания ведомого.
				setClock(valBaudrate);																			//	Обновляем интервалы таймаута.
			}																									//
}																												//
																												//
//		ИНИЦИАЦИЯ ПЕРЕДАЧИ ДАННЫХ К ВЕДОМОМУ:																	//
void	SoftTwoWire::beginTransmission(uint8_t addr){															//	Параметр:				«addr» - адрес ведомого.
			if( flgBeginEnabled ){																				//	Если установлен флаг инициализации работы с программной шиной I2C.
				flgBeginTransmission	= true;																	//	Устанавливаем флаг инициализации передачи данных ведомому.
				flgBufferFull			= false;																//	Сбрасываем флаг переполнения буфера приёма/передачи данных.
				bufIndex				= 0;																	//	Сбрасываем индекс буфера приёма/передачи данных
				bufLen					= 0;																	//	Сбрасываем количество байт в буфере приёма/передачи данных.
				valAddress				= addr;																	//	Сохраняем адрес приёма/передачи данных.
			}																									//
}																												//
																												//
//		ПЕРЕДАЧА ДАННЫХ ИЗ БУФЕРА ВЕДОМОМУ:																		//
uint8_t	SoftTwoWire::endTransmission(uint8_t stop){																//	Параметр:				«stop» - после передачи установить STOP. Если флаг сброшен, то будет установлен RESTART и можно продолжить общение с ведомым.
			uint8_t i=0;																						//	0 - Передача успешна.
			if( !flgBeginEnabled							){ return 4; }										//	4 - Другая ошибка. Не выполнена инициализация работы с программной шиной I2C.
			if( !flgMaster									){ return 4; }										//	4 - Другая ошибка. Объект работает не в роли мастера.
			if( !flgBeginTransmission						){ return 4; }										//	4 - Другая ошибка. Не выполнена инициализация передачи данных ведомому.
			if(  flgBufferFull								){ return 1; }										//	1 - Переполнен буфер для приёма/передачи.
		//	Устанавливаем состояние START:																		//
			if( !setStart()									){ i=flgTimeoutTriggered? 5:4; }					//	5 - timeout, 4 - Другая ошибка. Прижат SDA или удерживается SCL.
		//	Передаём байт адреса с битом RW=0 (передача):														//
			if( i==0 ){ if( !setByte( valAddress<<1 )		){ i=flgTimeoutTriggered? 5:2; }}					//	5 - timeout, 2 - Получен NACK при передаче адреса.
		//	Передаём байты из буфера приёма/передачи данных:													//
			if( i==0 ){																							//
				for(uint8_t j=0; j<bufLen; j++){																//
					if( i==0 ){ if( !setByte(arrBuffer[j])	){ i=flgTimeoutTriggered? 5:3; }}					//	5 - timeout, 3 - Получен NACK при передаче данных.
				}																								//
			}																									//
		//	Устанавливаем состояние STOP или готовимся к RESTART:												//
			if( !stop && !i ){ if( !setReStart()			){ i=flgTimeoutTriggered? 5:4; }}					//	5 - timeout, 4 - Другая ошибка. Если флаг stop сброшен и нет ошибок, то готовимся к RESTART.
			if(  stop ||  i ){ if( !setStop() && i==0		){ i=flgTimeoutTriggered? 5:4; }}					//	5 - timeout, 4 - Другая ошибка. Если флаг stop установлен или есть ошибки то устанавливаем состояние STOP.
		//	Отключаем выводы при наличии таймаута, если это разрешает flgTimeoutReset:							//
			if( flgTimeoutEnabled && flgTimeoutTriggered && flgTimeoutReset ){ end(); }							//
		//	Сбрасываем флаги:																					//
			flgBeginTransmission=false;																			//	Сбрасываем флаг инициализации передачи данных ведомому.
			clearWireTimeoutFlag();																				//	Сбрасываем флаг сработавшего таймаута.
			return i;																							//	Возвращаем результат передачи данных.
}																												//
																												//
//		ПОМЕЩЕНИЕ БАЙТА В БУФЕР ДЛЯ ПЕРЕДАЧИ ВЕДОМОМУ:															//
size_t	SoftTwoWire::write(uint8_t data){																		//	Параметр:				«data» - байт данных для передачи.
			size_t i=0;																							//	Количество записанных байт при текущем обращении к функции.
			if( flgBeginEnabled ){																				//	Если установлен флаг инициализации работы с программной шиной I2C.
				if( flgMaster ){																				//	Если установлен флаг работы в качестве мастера.
					if( flgBeginTransmission ){																	//	Если установлен флаг инициализации передачи данных ведомому.
						if( bufLen<SOFT_I2C_BUFFER_LENGTH ){													//	Если в буфере приёма/передачи данных есть место.
							arrBuffer[bufLen++] = data;															//	Добавляем очередной байт в буфер для приёма/передачи и увеличиваем его индекс.
							i++;																				//	Увеличиваем количество записанных байт при текущем обращении к функции.
						}else{ flgBufferFull = true; }															//	Устанавливаем флаг переполнения буфера приёма/передачи данных.
			}	}	}																							//
			return i;																							//	Возвращает количество записанных байт при текущем обращении к функции.
}																												//
																												//
//		ПОМЕЩЕНИЕ МАССИВА В БУФЕР ДЛЯ ПЕРЕДАЧИ ВЕДОМОМУ:														//
size_t	SoftTwoWire::write(const uint8_t* arr, size_t sum){														//	Параметры:				«arr» - массив данных для передачи, «sum» - количество элементов массива.
			size_t i=0;																							//	Количество записанных байт при текущем обращении к функции.
			if( flgBeginEnabled ){																				//	Если установлен флаг инициализации работы с программной шиной I2C.
				if( flgMaster ){																				//	Если установлен флаг работы в качестве мастера.
					if( flgBeginTransmission ){																	//	Если установлен флаг инициализации передачи данных ведомому.
						for(uint8_t j=0; j<sum; j++){															//	Если есть данные для записи в бефер.
							if( bufLen<SOFT_I2C_BUFFER_LENGTH ){												//	Если в буфере приёма/передачи данных есть место.
								arrBuffer[bufLen++] = arr[j];													//	Добавляем очередной байт в буфер для приёма/передачи и увеличиваем его индекс.
								i++;																			//	Увеличиваем количество записанных байт при текущем обращении к функции.
							}else{ flgBufferFull = true; j=sum; }												//	Устанавливаем флаг переполнения буфера приёма/передачи данных.
			}	}	}	}																						//
			return i;																							//	Возвращает количество записанных байт.
}																												//
																												//
//		ЧТЕНИЕ ДАННЫХ ОТ ВЕДОМОГО:																				//
uint8_t	SoftTwoWire::requestFrom(uint8_t addr, uint8_t sum, bool stop){											//	Параметры:				«addr» - адрес ведомого, «sum» - количество читаемых байт, «stop» - после передачи установить STOP. Если флаг сброшен, то будет установлен RESTART и можно продолжить общение с ведомым.
			flgBufferFull = false;																				//	Сбрасываем флаг переполнения буфера приёма/передачи данных.
			bufIndex = 0;																						//	Сбрасываем индекс буфера приёма/передачи данных
			bufLen = 0;																							//	Сбрасываем количество байт в буфере приёма/передачи данных.
			if( flgBeginEnabled ){																				//	Если установлен флаг инициализации работы с программной шиной I2C.
				if( flgMaster ){																				//	Если установлен флаг работы в качестве мастера.
				//	Устанавливаем состояние START:																//
					if( setStart() ){																			//	Возвращает false если прижат SDA или удерживается SCL.
					//	Передаём байт адреса с битом RW=1 (получение):											//
						if( setByte( (valAddress<<1)|1 ) ){														//	Возвращает false если получен NACK.
						//	Получаем байты в буфер приёма/передачи данных:										//
							while( sum-- ){																		//	Выполняем цикл sum раз.
								if( bufLen>=SOFT_I2C_BUFFER_LENGTH ){ flgBufferFull=true; sum=0; }else			//	Выходим из цикла если буфер переполнен.
								if( !getByte( arrBuffer[bufLen++], sum ) ){ sum=0; }							//	Выходим из цикла если байт не получен.
							}																					//
						}																						//
					}																							//
				//	Устанавливаем состояние STOP или готовимся к RESTART:										//
					if( !stop ){ if( !setReStart() ){ setStop(); }}												//	Если флаг stop сброшен, то готовимся к RESTART. Если RESTART не установлен, то ставим STOP.
					if(  stop ){                      setStop();  }												//	Если флаг stop установлен то устанавливаем состояние STOP.
				//	Отключаем выводы при наличии таймаута, если это разрешает flgTimeoutReset:					//
					if( flgTimeoutEnabled && flgTimeoutTriggered && flgTimeoutReset ){ end(); }					//
				//	Сбрасываем флаги:																			//
					flgBeginTransmission=false;																	//	Сбрасываем флаг инициализации передачи данных ведомому.
					clearWireTimeoutFlag();																		//	Сбрасываем флаг сработавшего таймаута.
				}																								//
			}																									//
			return bufLen;																						//	Возвращаем количество прочитанных байт.
}																												//
																												//
//		ПОЛУЧЕНИЕ КОЛИЧЕСТВА БАЙТ, ДОСТУПНЫХ ДЛЯ ЧТЕНИЯ ИЗ БУФЕРА ПРИНЯТЫХ ДАННЫХ:								//
int		SoftTwoWire::available(void){																			//	Параметры:				отсутствуют.
			if( !flgBeginEnabled		){ return 0; }															//	Если сброшен флаг инициализации работы с программной шиной I2C.
			if( !flgMaster				){ return 0; }															//	Если сброшен флаг работы в качестве мастера.
			if( flgBeginTransmission	){ return 0; }															//	Если установлен флаг инициализации передачи данных ведомому.
			if( flgBufferFull			){ return 0; }															//	Если установлен флаг переполнения буфера приёма/передачи данных.
			if( bufIndex>=bufLen		){ return 0; }															//	Если в буфере приёма/передачи данных есть байты для чтения функцией read() или peek().
			return bufLen-bufIndex;																				//	Возвращаем количество байт буфера доступных для получения функцией read().
}																												//
																												//
//		ПОЛУЧИТЬ ОЧЕРЕДНОЙ БАЙТ ИЗ БУФЕРА ПРИНЯТЫХ ДАННЫХ:														//
int		SoftTwoWire::read(void){																				//	Параметры:				отсутствуют.
			if( !flgBeginEnabled		){ return 0; }															//	Если сброшен флаг инициализации работы с программной шиной I2C.
			if( !flgMaster				){ return 0; }															//	Если сброшен флаг работы в качестве мастера.
			if( flgBeginTransmission	){ return 0; }															//	Если установлен флаг инициализации передачи данных ведомому.
			if( flgBufferFull			){ return 0; }															//	Если установлен флаг переполнения буфера приёма/передачи данных.
			if( bufIndex>=bufLen		){ return 0; }															//	Если в буфере приёма/передачи данных есть байты для чтения функцией read().
			return arrBuffer[bufIndex++];																		//	Возвращаем очередной байт из буфера.
}																												//
																												//
//		ПОЛУЧИТЬ ОЧЕРЕДНОЙ БАЙТ ИЗ БУФЕРА ПРИНЯТЫХ ДАННЫХ (без удаления этого байта из буфера):					//
int		SoftTwoWire::peek(void){																				//	Параметры:				отсутствуют.
			if( !flgBeginEnabled		){ return 0; }															//	Если сброшен флаг инициализации работы с программной шиной I2C.
			if( !flgMaster				){ return 0; }															//	Если сброшен флаг работы в качестве мастера.
			if( flgBeginTransmission	){ return 0; }															//	Если установлен флаг инициализации передачи данных ведомому.
			if( flgBufferFull			){ return 0; }															//	Если установлен флаг переполнения буфера приёма/передачи данных.
			if( bufIndex>=bufLen		){ return 0; }															//	Если в буфере приёма/передачи данных есть байты для чтения функцией read() или peek().
			return arrBuffer[bufIndex];																			//	Возвращаем очередной байт из буфера.
}																												//
																												//
//		УСТАНОВКА СОСТОЯНИЯ START:																				//
bool	SoftTwoWire::setStart(void){																			//	Параметры:				отсутствуют.
		//	     _ _ _ _____:___	           _ _ _	Состояние START это:									//
		//	SCL:            :   \___ _ _ _ _ _/     	Спад логического уровня на линии SDA с «1» в «0»,		//
		//	     _ _ _ _____:            _ _ _ _ _ _	при наличии уровня логической «1» на линии SCL.			//	 
		//	SDA:            :\______ _ _/_ _ _ _ _ _															//
		//																										//
		//	Отключаем прерывания:																				//
			noInterrupts();																						//
		//	Устанавливаем счётчик ожидания таймаута:															//
			cntTimeout=sumTimeout;																				//	Устанавливаем значение счётчика оставшихся циклов while до наступления таймаута.
		//	Если линия SDA прижата, значит мы потеряли арбитраж:												//
			if( getSDA()==0 ){ interrupts(); return false; }													//	Если SDA==0, то разрешаем прерывания и возвращаем false.
		//	Поднимаем линию SCL на пол периода и проверяем не удерживает ли ведомый линию SCL:					//
			setSCL_1_HalfPeriod(); while( getSCL()==0 && cntTimeout ){ cntTimeout--; }							//
		//	Устанавливаем флаг сработавшего таймаута:															//
			if(cntTimeout==0){ if(flgTimeoutEnabled){ flgTimeoutTriggered=true; interrupts(); return false; }}	//	Если счётчик cntTimeout обнулён, значит вышло время ожидания, устанавливаем флаг сработавшего таймаута flgTimeoutTriggered, разрешаем прерывания и возвращаем false.
		//	Прижимаем линию SDA на пол периода:																	//
			setSDA_0_HalfPeriod();																				//
		//	Прижимаем линию SCL на пол периода:																	//
			setSCL_0_HalfPeriod();																				//
		//	Разрешаем прерывания:																				//
			interrupts();																						//
		//	Возвращаем флаг успешной установки состояния START:													//
			return true;																						//
}																												//
																												//
//		УСТАНОВКА СОСТОЯНИЯ STOP:																				//
bool	SoftTwoWire::setStop(void){																				//	Параметры:				отсутствуют.
		//	     _ _ _         ___:____ _ _ _			Состояние STOP это:										//
		//	SCL:      \_ _ _ _/   :        				Подъём логического уровня на линии SDA с «0» в «1»,		//
		//	     _ _ _ _ _        :____ _ _ _			при наличии уровня логической «1» на линии SCL.			//
		//	SDA: _ _ _ _ _\______/:      																		//	
		//																										//
		//	Отключаем прерывания:																				//
			noInterrupts();																						//
		//	Устанавливаем счётчик ожидания таймаута:															//
			cntTimeout=sumTimeout;																				//	Устанавливаем значение счётчика оставшихся циклов while до наступления таймаута.
		//	Прижимаем линию SDA на пол периода:																	//
			setSDA_0_HalfPeriod();																				//
		//	Поднимаем линию SCL на пол периода и проверяем не удерживает ли ведомый линию SCL:					//
			setSCL_1_HalfPeriod(); while( getSCL()==0 && cntTimeout ){ cntTimeout--; }							//
		//	Поднимаем линию SDA на пол периода и проверяем не удерживает ли ведомый линию SDA:					//
			setSDA_1_HalfPeriod(); if( getSDA()==0 ){ interrupts(); return false; }								//	Если SDA==0, то разрешаем прерывания и возвращаем false.
		//	Устанавливаем флаг сработавшего таймаута:															//
			if(cntTimeout==0){ if(flgTimeoutEnabled){ flgTimeoutTriggered=true; interrupts(); return false; }}	//	Если счётчик cntTimeout обнулён, значит вышло время ожидания, устанавливаем флаг сработавшего таймаута flgTimeoutTriggered, разрешаем прерывания и возвращаем false.
		//	Разрешаем прерывания:																				//
			interrupts();																						//
		//	Возвращаем флаг успешной установки состояния STOP:													//
			return true;																						//
}																												//
																												//
//		ПЕРЕДАЧА ОДНОГО БАЙТА ДАННЫХ:																			//
bool	SoftTwoWire::setByte(uint8_t data){																		//	Параметр:				«data» - байт данных для передачи.
		//	                 1  2  3  4  5  6  7  8   9 			Передача 1 байта:							//
		//	SCL: _ _ _ _____/\_/\_/\_/\_/\_/\_/\_/\__/\_____ _ _ _	Каждый бит SDA передаётся по фронту SCL.	//
		//	               ________________________					На девятый такт SCL принимается бит ACK.	//
		//	SDA: _ _ _ ___/__________DATA__________>----____ _ _ _	ACK (0 - принято) / NACK (1 - провал)		//
		//										    вход														//
		//	Отключаем прерывания:																				//
			noInterrupts();																						//
		//	Устанавливаем счётчик ожидания таймаута:															//
			cntTimeout  = sumTimeout;																			//	Устанавливаем значение счётчика оставшихся циклов while до наступления таймаута.
		//	Определяем локальные переменные:																	//
			bool    ack = false;																				//	флаг наличия подтверждения приёма байта.
			uint8_t pos = 8;																					//	Позиция передаваемого бита в байте.
		//	Передаём 8 бит байта:																				//
			while(pos--){																						//
			//	Устанавливаем требуемый уровень на линии SDA:													//
				if( data & bit(pos) ){																			//	
				//	Поднимаем линию SDA на пол периода и проверяем не удерживает ли ведомый линию SDA:			//
					setSDA_1_HalfPeriod(); if( getSDA()==0 ){ interrupts(); return false; }						//
				}else{																							//
				//	Прижимаем линию SDA на пол периода:															//
					setSDA_0_HalfPeriod();																		//	В функция вызывается когда линии SDA и SCL прижаты.
				}																								//        _____
			//	Поднимаем линию SCL на пол периода и проверяем не удерживает ли ведомый линию SCL:				//	SDA^ |_____		уровень на линии меняется в начале каждого прохода цикла.
				setSCL_1_HalfPeriod(); while( getSCL()==0 && cntTimeout ){ cntTimeout--; }						//	         __
			//	Прижимаем линию SCL:																			//	SCL:  __|  |	линия остаётся прижатой после каждого прохода цикла.
				setSCL_0();																						//	
			}																									//
		//	Поднимаем линию SDA на пол периода. НО НЕ ЖДЁМ её поднятия, её может удерживать ведомый для ACK:	//
			setSDA_1_HalfPeriod();																				//
		//	Поднимаем линию SCL на пол периода и проверяем не удерживает ли ведомый линию SCL:					//
			setSCL_1_HalfPeriod(); while( getSCL()==0 && cntTimeout ){ cntTimeout--; }							//
		//	Читаем бит подтверждения с линии SDA:																//
			ack = getSDA()? false:true;																			//
		//	Прижимаем линию SCL и SDA:																			//	Функция завершается прижатием линий SDA и SCL.
			setSCL_0();																							//
			setSDA_0();																							//
		//	Устанавливаем флаг сработавшего таймаута:															//
			if(cntTimeout==0){ if(flgTimeoutEnabled){ flgTimeoutTriggered=true; interrupts(); return false; }}	//	Если счётчик cntTimeout обнулён, значит вышло время ожидания, устанавливаем флаг сработавшего таймаута flgTimeoutTriggered, разрешаем прерывания и возвращаем false.
		//	Разрешаем прерывания:																				//
			interrupts();																						//
		//	Возвращаем флаг наличия подтверждения приёма байта:													//
			return ack;																							//
}																												//
																												//
//		ЧТЕНИЕ ОДНОГО БАЙТА ДАННЫХ:																				//
bool	SoftTwoWire::getByte(uint8_t& data, bool ack){															//	Параметры:				«data» - ссылка на получения принятого байта, «ack» - флаг отправки бита подтверждения приёма.
		//	                 1  2  3  4  5  6  7  8   9 			Чтение 1 байта:								//
		//	SCL: _ _ _ _____/\_/\_/\_/\_/\_/\_/\_/\__/\_____ _ _ _	Каждый бит SDA читается по фронту SCL.		//
		//	                                         __				На девятый такт SCL передаётся бит ACK.		//
		//	SDA: _ _ _ ___/----------DATA-----------<__\____ _ _ _	ACK (0 - давай ещё) / NACK (1 - хватит)		//
		//	                         вход           выход														//
		//	Отключаем прерывания:																				//
			noInterrupts();																						//
		//	Устанавливаем счётчик ожидания таймаута:															//
			cntTimeout=sumTimeout;																				//	Устанавливаем значение счётчика оставшихся циклов while до наступления таймаута.
		//	Определяем переменные для чтения:																	//
			data = 0;																							//	Чистим место для получения байта.
			uint8_t pos = 8;																					//	Позиция читаемого бита в байте.
		//	Поднимаем линию SDA и оставляем в этом положении. Освобождаем линию SDA для чтения:					//
			setSDA_1();																							//
		//	Читаем 8 бит байта:																					//
			while(pos--){																						//
			//	Поднимаем линию SCL на пол периода и проверяем не удерживает ли ведомый линию SCL:				//	В функция вызывается когда линия SCL прижата.
				setSCL_1_HalfPeriod(); while( getSCL()==0 && cntTimeout ){ cntTimeout--; }						//
			//	Читаем состояние линии SDA:																		//         _____
				if( getSDA() ){ data |= (1<<pos); }																//	SDA^  |_____	уровень на линии меняется по фронту линии SCL.
			//	Прижимаем линию SCL на пол периода:																//	       __
				setSCL_0_HalfPeriod();																			//	SCL:  |  |__	линия остаётся прижатой после каждого прохода цикла.
			}																									//
		//	Отправляем бит подтверждения приёма ACK:															//
			if( ack ){ setSDA_0(); }																			//
		//	Поднимаем линию SCL на пол периода и проверяем не удерживает ли ведомый линию SCL:					//
			setSCL_1_HalfPeriod(); while( getSCL()==0 && cntTimeout ){ cntTimeout--; }							//
		//	Прижимаем линию SCL на пол периода:																	//
			setSCL_0_HalfPeriod();																				//
		//	Устанавливаем флаг сработавшего таймаута:															//
			if(cntTimeout==0){ if(flgTimeoutEnabled){ flgTimeoutTriggered=true; interrupts(); return false; }}	//	Если счётчик cntTimeout обнулён, значит вышло время ожидания, устанавливаем флаг сработавшего таймаута flgTimeoutTriggered, разрешаем прерывания и возвращаем false.
		//	Разрешаем прерывания:																				//
			interrupts();																						//
		//	Возвращаем флаг чтения байта:																		//
			return (bool)cntTimeout;																			//
}																												//
																												//
//		КОЛИЧЕСТВО ТАКТОВ CPU ДЛЯ ПРОСТЫХ ВЫЧИСЛЕНИЙ:															//
uint8_t	SoftTwoWire::getTactPegCalc(uint8_t bits){																//	Параметр:				«bits» - разрядность числа с которым производится вычисление: 8 / 16 / 32.
		//	Отключаем прерывания:																				//
			noInterrupts();																						//
		//	Создаём две переменные в стеке:																		//
			volatile int32_t i=0;																				//	Создаём 32 битную переменную с любым числом.
			volatile int8_t  j=0;																				//	Создаём  8 битную переменную с любым числом.
		//	Разрешаем прерывания:																				//
			interrupts();																						//
		//	Определяем разрядность микроконтроллера:															//
			i = (int32_t)&i - (int32_t)&j; if( i<0 ){ i*=-1; } i = 32/i;										//	Получаем разницу между адресами. Избавляемся от знака. Берём обратное число умноженное на 8.
		//	Определяем количество затрачиваемых тактов на вычисления:											//
			i = bits/i; if( i<=0 ){ i=1; } return (uint8_t)i;													//	Возвращаем количество тактов: 1 / 2 / 4.
}																												//
																												//
//		ФУНКЦИИ УПРАВЛЕНИЯ ВЫВОДАМИ:																			//
bool	SoftTwoWire::getSDA				(void){ return (bool)(*portInputSDA & pinMaskSDA); }					//
void	SoftTwoWire::setSDA_0           (void){ *portModeSDA |= pinMaskSDA; }									//
void	SoftTwoWire::setSDA_0_HalfPeriod(void){ *portModeSDA |= pinMaskSDA; cntHalfPeriod=sumHalfPeriod; while(cntHalfPeriod--); }
void	SoftTwoWire::setSDA_1           (void){ *portModeSDA &=_pinMaskSDA; }									//
void	SoftTwoWire::setSDA_1_HalfPeriod(void){ *portModeSDA &=_pinMaskSDA; cntHalfPeriod=sumHalfPeriod; while(cntHalfPeriod--); }
																												//
bool	SoftTwoWire::getSCL				(void){ return (bool)(*portInputSCL & pinMaskSCL); }					//
void	SoftTwoWire::setSCL_0           (void){ *portModeSCL |= pinMaskSCL; }									//
void	SoftTwoWire::setSCL_0_HalfPeriod(void){ *portModeSCL |= pinMaskSCL; cntHalfPeriod=sumHalfPeriod; while(cntHalfPeriod--); }
void	SoftTwoWire::setSCL_1_HalfPeriod(void){ *portModeSCL &=_pinMaskSCL; cntHalfPeriod=sumHalfPeriod; while(cntHalfPeriod--); }
