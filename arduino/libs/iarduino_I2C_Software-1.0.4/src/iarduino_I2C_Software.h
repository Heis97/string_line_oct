//	Библиотека для работы с программной шиной I2C в качестве мастера.
//  Версия: 1.0.4
//  Последнюю версию библиотеки Вы можете скачать по ссылке: https://iarduino.ru/file/627.html
//  Подробное описание функции бибилиотеки доступно по ссылке: https://wiki.iarduino.ru/
//  Библиотека является собственностью интернет магазина iarduino.ru и может свободно использоваться и распространяться!
//  При публикации устройств или скетчей с использованием данной библиотеки, как целиком, так и её частей,
//  в том числе и в некоммерческих целях, просим Вас опубликовать ссылку: http://iarduino.ru
//  Автор библиотеки: Панькин Павел.
//  Если у Вас возникли технические вопросы, напишите нам: shop@iarduino.ru
#pragma once
#ifndef iarduino_I2C_Software_h																							//
#define iarduino_I2C_Software_h																							//
																														//
#if defined(ARDUINO) && (ARDUINO >= 100)																				//
#include		<Arduino.h>																								//
#else																													//
#include		<WProgram.h>																							//
#endif																													//
																														//
#define SOFT_I2C_BUFFER_LENGTH	32																						//	Размер буфера программной шины I2C.
#define SOFT_I2C_BAUDRATE		100000																					//	Скорость передачи данных по умолчанию (Гц).
#define SOFT_I2C_TIMEOUT		25000																					//	Максимальное время ожидания мастером ведомого (мкс) по умолчанию.
																														//
class SoftTwoWire{																										//	Для class SoftTwoWire : public TwoWire{ Требуется заранее знать класс TwoWire, если <Wire.h> не поддерживается можно создать класс TwoWire здесь, но если <Wire.h> поддерживается, то Arduino IDE (<1.8.16) подключит его после, даже если в скетче он подключён до данной библиотеки.
																														//
	public:						SoftTwoWire				(void												);			//	Конструктор класса.
								SoftTwoWire				(int  sda, int  scl									);			//	Конструктор класса.
					bool		setPins					(int  sda, int  scl									);			//	Назначение выводов. Возвращает результат назначения выводов.
					bool		getPins					(int* sda, int* scl									);			//	Получение номеров назначенных выводов. Возвращает результат назначения выводов.
					bool		begin					(void												);			//	Инициализация работы на программной шине I2C в качестве мастера.  Возвращает результат инициализации.
					bool		begin					(uint8_t addr										);			//	Инициализация работы на программной шине I2C в качестве ведомого. Не поддерживается.
					void		end						(void												);			//	Отключает выводы sda и scl. Для включения нужно вновь указать выводы setPins(), а потом begin().
					void		setClock				(uint32_t baudrate									);			//	Скорость передачи данных по шине I2C.
					void		setWireTimeout			(uint32_t timeout=SOFT_I2C_TIMEOUT, bool reset=false);			//	Максимальное время ожидания мастером ведомого в мкс. Если 0 мкс, то таймаут отключён. Если установлен флаг reset, то будет выполнен end().
					bool		getWireTimeoutFlag		(void				){ return flgTimeoutTriggered;	}			//	Возвращает true если сработал таймаут. Этот флаг нужно сбросить функцией clearWireTimeoutFlag() или setWireTimeout().
					void		clearWireTimeoutFlag	(void				){ flgTimeoutTriggered=false;	}			//	Сброс флага сработавшего таймаута.
					void		beginTransmission		(uint8_t addr										);			//	Инициации передачи данных ведомому с адресом addr.
					uint8_t		endTransmission			(uint8_t stop=true									);			//	Выполнение инициированной ранее передачи данных из буфера ведомому. Если флаг stop==false, то вместо STOP на шине будет RESTART и можно продолжить общение с ведомым. Возвращает: 0-передача успешна / 1 - переполнен буфер для передачи / 2 - получен NACK при передаче адреса / 3 - получен NACK при передаче данных / 4 - другая ошибка / 5 - timeout.
					size_t		write					(uint8_t data										);			//	Помещение байта   в буфер для передачи ведомому. Возвращает количество записанных байт.
					size_t		write					(const uint8_t* arr, size_t sum						);			//	Помещение массива в буфер для передачи ведомому. Возвращает количество записанных байт.
					uint8_t		requestFrom				(uint8_t addr, uint8_t sum, bool stop=true			);			//	Чтение от ведомого sum байт. Если флаг stop==false, то вместо STOP на шине будет RESTART и можно продолжить общение с ведомым. Возвращает реальное количество прочитанных байт.
					int			available				(void												);			//	Возвращает количество байт, доступных для чтения из буфера принятых данных.
					int			read					(void												);			//	Возвращает очередной байт из буфера принятых данных.
					int			peek					(void												);			//	Возвращает очередной байт из буфера принятых данных, но не удаляет его из буфера.
					void		flush					(void				){ return;						}			//	Ожидает завершения передачи исходящих последовательных данных. Не поддерживается.
					void		onReceive				(void (*func)(int )	){ user_onReceive = func;		}			//	Добавляет пользовательскую функцию обработчик получения данных  от мастера. Не поддерживается.
					void		onRequest				(void (*func)(void)	){ user_onRequest = func;		}			//	Добавляет пользовательскую функцию обработчик получения запроса от мастера. Не поддерживается.
																														//
		inline		size_t		write					(unsigned long n	){ return write((uint8_t)n);	}			//
		inline		size_t		write					(long n				){ return write((uint8_t)n);	}			//
		inline		size_t		write					(unsigned int n		){ return write((uint8_t)n);	}			//
		inline		size_t		write					(int n				){ return write((uint8_t)n);	}			//
																														//
	private:																											//
					bool		setStart				(void												);			//	Установка  состояния Start.   Возвращает результат: true/false,                  при таймауте устанавливает flgTimeoutTriggered если это разрешает flgTimeoutEnabled.
					bool		setStop					(void												);			//	Установка  состояния Stop.    Возвращает результат: true/false,                  при таймауте устанавливает flgTimeoutTriggered если это разрешает flgTimeoutEnabled.
					bool		setReStart				(void){ setSDA_1_HalfPeriod(); return getSDA()==1;	}			//	Подготовка состояния ReStart. Возвращает результат: true/false, далее должен следовать Start.
					bool		setByte					(uint8_t data										);			//	Передача одного байта данных. Возвращает наличие бита подтверждения: true/false, при таймауте устанавливает flgTimeoutTriggered если это разрешает flgTimeoutEnabled.
					bool		getByte					(uint8_t& data, bool ack=true						);			//	Чтение байта данных в data.   Возвращает результат: true/false,                  при таймауте устанавливает flgTimeoutTriggered если это разрешает flgTimeoutEnabled.
					uint8_t		getTactPegCalc			(uint8_t bits										);			//	Получение количества тактов.  Возвращает количество тактов CPU для простых вычислений с целыми числами указанной разрядности.
																														//
					int			pinSDA;																					//
					bool		getSDA					(void												);			//	Чтение логического уровня с линии SDA.
					void		setSDA_0				(void												);			//	Установка логического  0 на линии SDA.
					void		setSDA_0_HalfPeriod		(void												);			//	Установка логического  0 на линии SDA с ожиданием в пол периода.
					void		setSDA_1				(void												);			//	Установка логической   1 на линии SDA.
					void		setSDA_1_HalfPeriod		(void												);			//	Установка логической   1 на линии SDA с ожиданием в пол периода.
																														//
					int			pinSCL;																					//
					bool		getSCL					(void												);			//	Чтение логического уровня с линии SCL.
					void		setSCL_0				(void												);			//	Установка логического  0 на линии SCL.
					void		setSCL_0_HalfPeriod		(void												);			//	Установка логического  0 на линии SCL с ожиданием в пол периода.
					void		setSCL_1_HalfPeriod		(void												);			//	Установка логической   1 на линии SCL с ожиданием в пол периода.
																														//
					bool		flgPinsEnabled;																			//	Флаг подключения выводов SDA и SCL.
					bool		flgBeginEnabled;																		//	Флаг инициализации работы с программной шиной I2C.
					bool		flgTimeoutEnabled;																		//	Флаг инициализации таймаута ожидания ведомого.
					bool		flgTimeoutTriggered;																	//	Флаг сработавшего  таймаута ожидания ведомого.
					bool		flgTimeoutReset;																		//	Флаг указывает отключить выводы end(), если сработал таймаут.
					bool		flgMaster;																				//	Флаг работы в качестве мастера.
					bool		flgSlave;																				//	Флаг работы в качестве ведомого.
					bool		flgBeginTransmission;																	//	Флаг инициализации передачи данных   ведомому.
					bool		flgBufferFull;																			//	Флаг переполнения буфера передачи данных.
																														//
					uint32_t	valTimeout;																				//	Время ожидания (таймаут) в мкс.
					uint32_t	valBaudrate = SOFT_I2C_BAUDRATE;														//	Скорость передачи данных в Гц.
					uint8_t		valAddress;																				//	Адрес приёма/передачи данных.
					uint8_t		arrBuffer[SOFT_I2C_BUFFER_LENGTH];														//	Буфер приёма/передачи данных.
					uint8_t		bufIndex;																				//	Индекс буфера приёма/передачи данных.
					uint8_t		bufLen;																					//	Количество байт в буфере приёма/передачи данных.
					uint16_t	sumHalfPeriod;																			//	Количество циклов while с декрементом счётчика в функции setXXX_X_HalfPeriod() до истечения половины периода valBaudrate.
		volatile	uint16_t	cntHalfPeriod;																			//	Счётчик оставшихся циклов while в функции setXXX_X_HalfPeriod() до истечения половины периода valBaudrate.
					uint32_t	sumTimeout;																				//	Количество циклов while с проверкой порта входных данных и декрементом счётчика cntTimeout, до истечения таймаута.
		volatile	uint32_t	cntTimeout;																				//	Счётчик оставшихся циклов while в функциях waitSDA() и waitSCL() до наступления таймаута.
																														//
	#if defined(ESP32)																									//
		volatile	uint32_t*	portModeSDA;																			//	Указатель на адрес регистра конфигурирования направления вывода  pinSDA.
		volatile	uint32_t*	portModeSCL;																			//	Указатель на адрес регистра конфигурирования направления вывода  pinSCL.
		volatile	uint32_t*	portInputSDA;																			//	Указатель на адрес регистра чтения входных       данных  вывода  pinSDA.
		volatile	uint32_t*	portInputSCL;																			//	Указатель на адрес регистра чтения входных       данных  вывода  pinSCL.
		volatile	uint32_t*	portOutputSDA;																			//	Указатель на адрес регистра управления выходными данными вывода  pinSDA.
		volatile	uint32_t*	portOutputSCL;																			//	Указатель на адрес регистра управления выходными данными вывода  pinSCL.
					uint32_t	 pinMaskSDA;																			//	Маска регистров входных и выходных данных для управления выводом pinSDA.
					uint32_t	 pinMaskSCL;																			//	Маска регистров входных и выходных данных для управления выводом pinSCL.
					uint32_t	_pinMaskSDA;																			// ~Маска регистров входных и выходных данных для управления выводом pinSDA.
					uint32_t	_pinMaskSCL;																			// ~Маска регистров входных и выходных данных для управления выводом pinSCL.
	#else																												//
		volatile	uint16_t*	portModeSDA;																			//	Указатель на адрес регистра конфигурирования направления вывода  pinSDA.
		volatile	uint16_t*	portModeSCL;																			//	Указатель на адрес регистра конфигурирования направления вывода  pinSCL.
		volatile	uint16_t*	portInputSDA;																			//	Указатель на адрес регистра чтения входных       данных  вывода  pinSDA.
		volatile	uint16_t*	portInputSCL;																			//	Указатель на адрес регистра чтения входных       данных  вывода  pinSCL.
		volatile	uint16_t*	portOutputSDA;																			//	Указатель на адрес регистра управления выходными данными вывода  pinSDA.
		volatile	uint16_t*	portOutputSCL;																			//	Указатель на адрес регистра управления выходными данными вывода  pinSCL.
					uint16_t	 pinMaskSDA;																			//	Маска регистров входных и выходных данных для управления выводом pinSDA.
					uint16_t	 pinMaskSCL;																			//	Маска регистров входных и выходных данных для управления выводом pinSCL.
					uint16_t	_pinMaskSDA;																			// ~Маска регистров входных и выходных данных для управления выводом pinSDA.
					uint16_t	_pinMaskSCL;																			// ~Маска регистров входных и выходных данных для управления выводом pinSCL.
	#endif																												//
					void		(*user_onReceive)(int );																//	Указатель на пользовательскую функцию обработчик получения данных  от мастера. Не поддерживается.
					void		(*user_onRequest)(void);																//	Указатель на пользовательскую функцию обработчик получения запроса от мастера. Не поддерживается.
};

#endif // iarduino_I2C_Software_h
