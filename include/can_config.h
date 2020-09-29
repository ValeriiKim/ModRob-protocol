#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

#include "stm32f1xx.h"
#include <string.h>

namespace can_bus
{
	
#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1

uint8_t can_rx_ready = 0;

enum class CanTestMode: uint8_t
{
	SilentMode = 0,
	LoopBackMode = 1,
	LoopBackSilentMode = 2
};

struct CAN_Message
{
	unsigned int id;	   // 29 bit identifier
	unsigned char data[8]; // data field
	unsigned char len;	   // length of data field
	unsigned char format;  // format: standard or extended
	unsigned char type;	   // type: data or remote frame

/* Конструктор для создания пустого CAN сообщения */
	CAN_Message()
	{
		unsigned int id   = 0U;
		memset(data, 0, 8);
		unsigned char len = 8U;
		unsigned char format = STANDARD_FORMAT;
		unsigned char type = DATA_FRAME;
	}

/** Конструктор для создания CAN сообщения с определённым 
 * содержанием по умолчанию используем стандартный формат кадра
 * 
 *  @param _id      Message ID
 *  @param _data    Message Data
 *  @param _len     Message Data length
 *  @param _type    Type of Data: EXTENDED_FORMAT or STANDARD_FORMAT
 *  @param _format  Data Format: DATA_FRAME or REMOTE_FRAME
 */
    CAN_Message(unsigned int _id, const unsigned char *_data, unsigned char _len = 8,
	            unsigned char _format = STANDARD_FORMAT, unsigned char _type = DATA_FRAME)
	{
		id = _id;
		memcpy(data, _data, _len);
		len = _len & 0xF;
		format = _format;
		type = _type;
	}
};

/** Настраиваем bxCAN: пины, скорость 1 Мбит/с, прерывания, а также фильтры.
 * После того как эта функция выполнится, bxCAN всё ещё будет находиться в режиме ИНИЦИАЛИЗАЦИИ 
 * @param rx_interrupt флаг включащий прерывания по приёму сообщений
 */
void can_setup(bool rx_interrupt)
{
/* 	(1) Включаем тактирование CAN1 
	(2) Сбрасываем CAN remap, чтобы направить RX и TX на другие порты
	(3) Режим CAN remap 10: CANRx на PB8, CANTx на PB9
	(4) Настраиваем PB8 на вход с pull-up (?)
	(5) Настраиваем PB9 на выход в режиме alternate output push pull 
*/
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;                // (1)
	AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;                // (2) 
	AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;          // (3)
	GPIOB->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);   // (4)
	GPIOB->CRH |= GPIO_CRH_CNF8_1;
	GPIOB->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);   // (5)
	GPIOB->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;
	GPIOB->CRH |= GPIO_CRH_CNF9_1;
	
	// включение прерываний для CAN - RX в зависимости от флага
	if (rx_interrupt)
	{
		NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
	    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	}
	// NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
	// NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);

/* 	(1) Переход в режим инициализации
	(2) Ожидание установки режима инициализации
	(3) На всякий случай сбрасываем бит SLEEP чтобы выйти из режима сна
	(4) Выключаем автоматическую переотправку сообщений
	(5) Выключение режима Time Triggered Communication Mode
	(6) Выключение режима Automatic bus-off management
	(7) Выключение режима Automatic wakeup mode
	(8) FIFO не блокируется, если оно заполнено: новое сообщение будет перезаписывать предыдущее
	(9) Приоритет передачи сообщений определяется идентификаторами
*/
	CAN1->MCR |= CAN_MCR_INRQ;   // (1)
	while ((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) {}  // (2)
	CAN1->MCR &= ~CAN_MCR_SLEEP; // (3)
	CAN1->MCR |= CAN_MCR_NART;   // (4)
	CAN1->MCR &= ~CAN_MCR_TTCM;  // (5)
	CAN1->MCR &= ~CAN_MCR_ABOM;  // (6)
	CAN1->MCR &= ~CAN_MCR_AWUM;  // (7)
	CAN1->MCR &= ~CAN_MCR_RFLM;  // (8)
	CAN1->MCR &= ~CAN_MCR_TXFP;  // (9)

/* Настройка скорости CAN-шины на 1 Мбит/с 
    (1) На всякий случай очищаем биты прескейлера
    (2) Устанавливаем прескейлер для скорости 1 Мбит/с
    (3) Обнуляем биты Time Segment 1 и Time Segment 2
    (4) Устанавливаемы биты Time Segment 1 так, чтобы получилось число 110 = 6
    (5) Устанавливаем биты SJW в 0 (чтобы потом получить 1)
*/
	uint8_t prescaler = 0x03;
	CAN1->BTR &= ~CAN_BTR_BRP;                    // (1)
	CAN1->BTR |= prescaler;                       // (2)
	CAN1->BTR &= ~(CAN_BTR_TS1 | CAN_BTR_TS2);    // (3) 
	CAN1->BTR |= (CAN_BTR_TS1_1 | CAN_BTR_TS1_2); // (4)
	CAN1->BTR &= ~CAN_BTR_SJW;                    // (5)   

/* Настройка прерываний: включаем прерывание по получению сообщения в FIFO0,
пока непонятно, нужно ли использовать другой буфер (FIFO1).
*/
    if (rx_interrupt) {CAN1->IER |= CAN_IER_FMPIE0;}

/* Настройка фильтра - пока что фильтр настроен следующим образом - 
узел просто принимает все сообщения, которые есть на шине. На самом деле он должен
игнорировать свои сообщения, однако пока непонятно, как настроить фильтр
    (1) Входим в режим инициализации фильтра
    (2) Деактивируем фильтр под номером 0
	(3) Установка конфигурации - Single 32-bit scale configuration
	(4) Два 32 битных регистра фильтра в режиме маски
	(5) Устанавливаем регистр идентификатора в 0
	(6) Устанавливаем регистр маски в 0
	(7) Сообщения с фильтра 0 будут попадать в FIFO0
	(8) Активируем нулевой фильтр
	(9) Выходим из режима инициализации и активируем банки фильтров
 */ 
    static uint32_t CAN_filter_ID0 = 0x00;
    CAN1->FMR |= CAN_FMR_FINIT;                     // (1) 
	CAN1->FA1R &= ~(1 << CAN_filter_ID0);           // (2)
	CAN1->FS1R |= (1 << CAN_filter_ID0);            // (3)
	CAN1->FM1R &= ~(1 << CAN_filter_ID0);           // (4)
	CAN1->sFilterRegister[CAN_filter_ID0].FR1 = 0;  // (5)
	CAN1->sFilterRegister[CAN_filter_ID0].FR2 = 0;  // (6)
	CAN1->FFA1R &= ~(1 << CAN_filter_ID0);          // (7)
	CAN1->FA1R |= (1 << CAN_filter_ID0);            // (8)
	CAN1->FMR &= ~CAN_FMR_FINIT;                    // (9)
}

/* Выходим из режима инициализации bxCAN и переходим в нормальный режим */
void can_start()
{
	CAN1->MCR &= ~CAN_MCR_INRQ;
	while ((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {}
}

/** Устанавливаем режим Test mode: silent mode, loop back mode или loop back 
 * silent mode. Функция должна быть вызвана, когда bxCAN находится в режиме 
 * инициализации
*@param mode режим, который нужно установить из num class CanTestMode 
*/
void can_set_test_mode(CanTestMode mode)
{
	CAN1->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);
	switch (mode)
	{
	case CanTestMode::SilentMode:
	{
		CAN1->BTR |= CAN_BTR_SILM;
		break;
	}
	case CanTestMode::LoopBackMode:
	{
		CAN1->BTR |= CAN_BTR_LBKM;
		break;
	}
	case CanTestMode::LoopBackSilentMode:
	{
		CAN1->BTR |= CAN_BTR_SILM | CAN_BTR_LBKM;
		break;
	}
	default:
		break;
	}
}


uint8_t can_write(CAN_Message *msg)
{
/** Используем нулевой mailbox
 *  (1) Сброс регистра TIR  
 *  (2) Установка идентификатора в регистр TIR в зависимости от типа фрейма (стандартный или расширенный)
 *  (3) Будем отправлять только data фреймы, поэтому устанавливаем соответствующий 
 *  бит в регистре TIR
 *  (4) Записываем байты данных в регистры TDLR
 *  (5) Записываем байты данных в регистры TDHR
 *  (6) Сбрасываем биты DLC определяющие число байт данных
 *  (7) Устанавливаем количество отправляемых байт в DLC - 8
 *  (8) Запрос на отправку сообщения TXRQ = 1
*/
	CAN1->sTxMailBox[0].TIR = 0;                             // (1)
	if (msg->format == STANDARD_FORMAT)                      // (2)
	{
		CAN1->sTxMailBox[0].TIR |= static_cast<uint32_t>(msg->id << 21 | CAN_ID_STD);
	}
	else
	{
		CAN1->sTxMailBox[0].TIR |= static_cast<uint32_t>(msg->id << 3 | CAN_ID_EXT);
	}
	CAN1->sTxMailBox[0].TIR |= CAN_RTR_DATA;                 // (3)
	CAN1->sTxMailBox[0].TDLR = ((static_cast<uint32_t>(msg->data[3]) << 24U) |
	                            (static_cast<uint32_t>(msg->data[2]) << 16U) |
								(static_cast<uint32_t>(msg->data[1]) << 8U)  |
								 static_cast<uint32_t>(msg->data[0])); // (4)

	CAN1->sTxMailBox[0].TDHR = ((static_cast<uint32_t>(msg->data[7]) << 24U) |
	                            (static_cast<uint32_t>(msg->data[6]) << 16U) |
								(static_cast<uint32_t>(msg->data[5]) << 8U)  |
								 static_cast<uint32_t>(msg->data[4])); // (5)

	CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;              // (6)
	CAN1->sTxMailBox[0].TDTR |= (msg->len & CAN_TDT0R_DLC);  // (7)
	CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;                // (8)
	return 1;
}

uint8_t can_read(CAN_Message *msg)
{
	/** Используем нулевой mailbox
 *  (1) Проверяем, есть ли хотя бы одно сообщение в FIFO0
 *  (2) Получаем информацию об идентификаторе входящего сообщения 
 *  (3) Поскольку мы отправляем только data фреймы, то присваиваем типу 
 *      входящего сообщения DATA_FRAME
 *  (4) Читаем длину полезной нагрузки - число полученных байт (обычно 8)
 *  (5) Извлекаем из регистра RDLR младшие байты данных 
 *  (6) Извлекаем из регистра RDHR старшие байты данных 
 *  (7) Устанавливаем бит RF0M0 чтобы освободить FIFO0 (уменьшаем число FMP0)
*/
    if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0)                             // (1)
	{
		return 0; // сообщений нет
	}
    if ((CAN1->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0)                 // (2)
	{
		msg->format = STANDARD_FORMAT;
		msg->id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x000007FFU; // 7FF = 11 битовых единиц
	}
	else
	{
		msg->format = EXTENDED_FORMAT;
		msg->id = (CAN1->sFIFOMailBox[0].RIR >> 3) & 0x1FFFFFFFU; // 1FFFFFFF = 29 битовых единиц
	}
	msg->type = DATA_FRAME;                                           // (3)  
	msg->len = CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC;            // (4)
	msg->data[0] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDLR);        // (5)
	msg->data[1] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDLR >> 8);
	msg->data[2] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDLR >> 16);
	msg->data[3] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDLR >> 24);

	msg->data[4] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDHR);        // (6)
	msg->data[5] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDHR >> 8);
	msg->data[6] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDHR >> 16);
	msg->data[7] = 0x000000FFU & (CAN1->sFIFOMailBox[0].RDHR >> 24);
	
	CAN1->RF0R |= CAN_RF0R_RFOM0;                                     // (7)
	return 1;
}


/**
 * Обработчик прерывания по приёму сообщений, нужен если включены прерывания. Эта функция 
 * должна быть размещена внутри функции void USB_LP_CAN1_RX0_IRQHandler(void) 
*/
inline void can_rx0_irq_handler(CAN_Message *msg)
{
	can_read(msg);
	can_rx_ready = 1;
}

} // namespace CAN1

#endif