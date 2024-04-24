#include <stdlib.h>
#include <serial2USBprotocol.h>
#include "crc.h"
#include <FreeRTOS.h>
#include "usb_dev_serial.h"

//Funcion que realiza el stuffing en una trama. Esta funcion no es publica.
static int32_t frame_stuffing(uint8_t *origen, uint8_t *destino,int32_t longitud, int32_t longitud_maxima)
{
	int32_t i,j;
	uint8_t tmp;
	
	for (i=0,j=0;((i<longitud)&&(j<longitud_maxima));i++,j++)
	{
		tmp=*origen;
		origen++;
		if ((START_FRAME_CHAR==tmp)||(STOP_FRAME_CHAR==tmp)||(ESCAPE_CHAR==tmp))
		{
			*destino=ESCAPE_CHAR;
			destino++;
			j++;
			*destino=tmp^STUFFING_MASK;
			destino++;
		}
		else
		{
			*destino=tmp;
			destino++;
		}
	}
		
	
	if (j>longitud_maxima) 
		return PROT_ERROR_STUFFED_FRAME_TOO_LONG;
	else
		return j;
}

//Funcion para hacer el "Desestuffamiento" de una trama recibida, antes de pasar a procesarla.
// Esta funcion no es publica, es interna de la biblioteca.
static int32_t frame_destuffing(uint8_t *frame,int32_t longitud)
{
	int32_t i,numStuffedBytes;
	uint8_t *auxptr;
	uint8_t escape_seq_detected=0;
	
	//No hace falta duplicar, se trabaja sobre el propio array, ya que se van eliminando posiciones.
	auxptr=frame;

	for (i=0,numStuffedBytes=0;i<longitud;i++)
	{
		if (*auxptr==ESCAPE_CHAR)
		{
			auxptr++;
			i++;	//Se incremente i
			numStuffedBytes++;
			if ((*auxptr)!=ESCAPE_CHAR)
			{
				*frame=(*auxptr)^STUFFING_MASK;	//Hace el XoR si el siguiente no es tambien 0xFE
				frame++;
				auxptr++;
			}
			else{
				auxptr++;
				escape_seq_detected++;
				//Aqui iria el codigo para tratar una secuencia de escape en caso de querer hacerlo
			}
		}
		else
		{
			*frame=(*auxptr);
			frame++;
			auxptr++;
		}
	}
	
	longitud-=numStuffedBytes;

	return longitud;
}

//Funcion para calcular el checksum y "estuffar" un paquete para su envio
static int add_checksum_and_stuff(uint8_t *frame,int32_t size, int32_t max_size)
{
	uint16_t checksum;
	uint8_t *ptrtochar;
	
	checksum=create_checksum(frame,size);	//Calcula el checksum del paquete antes del Stuffing
	frame[size]=(uint8_t)(checksum&0x0FF);
	frame[size+1]=(uint8_t)(checksum>>8);	//A�ade el checksum
	size+=CHECKSUM_SIZE;

	//Hace una copia temporal del paquete
	//Se hace siempre del mismo tamanyo para que no se fragmente el Heap
	ptrtochar=(uint8_t *)pvPortMalloc(max_size);
	if (!ptrtochar) 
	{
		return PROT_ERROR_NOMEM;
	}
	memcpy(ptrtochar,(void *)frame,size);
	
	size=frame_stuffing(ptrtochar, frame,size, max_size);	//Hace el stuffing	
	vPortFree(ptrtochar);
	
	return (size);
}

//Desestufa y chequea el checksum de un paquete recibido
int32_t destuff_and_check_checksum(uint8_t *frame, int32_t max_size)
{
	uint16_t checksum,checksum_calc;
	int32_t size;

	
	
	size=frame_destuffing(frame,max_size);
	
	checksum_calc=create_checksum(frame,size-CHECKSUM_SIZE);	//Calcula el checksum del paquete despues del deStuffing
	checksum=(((uint16_t)frame[size-(CHECKSUM_SIZE-1)])<<8)|((uint16_t)frame[size-CHECKSUM_SIZE]);
	
	if (checksum!=checksum_calc) 
		return PROT_ERROR_BAD_CHECKSUM;
	else
		return (size);
}


//Funcion que crea un mensaje y lo introduce en una trama
int32_t create_frame(uint8_t  *frame, uint8_t  message_type, void * param, int32_t param_size, int32_t max_size)
{
	int32_t min_size;
	uint8_t *auxptr;
	int32_t size;
	
	auxptr=frame;
	
	*auxptr=START_FRAME_CHAR;
	auxptr++;
	
	min_size=(MINIMUM_FRAME_SIZE+param_size);	//1 START +1 MENSAJE + 2 CHECKSUM +1 STOP
		
	if (min_size>=max_size) 
	{
		return PROT_ERROR_MESSAGE_TOO_LONG;
	}
		
	*auxptr=message_type;
	auxptr++;
	if (param_size>0)
	{
		memcpy(auxptr,param,param_size);	//Copia los argumentos
		auxptr+=param_size;
	}
	
	size=add_checksum_and_stuff((frame+START_SIZE),(min_size-(START_SIZE+END_SIZE+CHECKSUM_SIZE)),max_size-(START_SIZE+END_SIZE));	//No se aplica el chesksum y el stuff a los caracteres de inicio y fin
	auxptr=(frame+size+START_SIZE);
	*auxptr=STOP_FRAME_CHAR;
		
	return (size+START_SIZE+END_SIZE);
}

//Esta funcion obtiene el campo "tipo mensaje" de la trama
uint8_t decode_message_type(uint8_t * buffer)
{
	return buffer[0];
}

//Esta funcion extrae el parametro de la trama y comprueba que el tamanyo sea correcto
//ptrtoparam es el puntero a la zona de memoria donde esta el parametro
//param_size es su tamanyo
//param es una estructura por REFERENCIA
//payload es el tamanyo de la estructura que se pasa por REFERENCIA
//Devuelve: Tamanyo del parametro recibido o un valor negativo si hay error
int32_t check_and_extract_message_param(void *ptrtoparam, int32_t param_size, uint32_t payload,void *param)
{
	if (payload==param_size)
	{
		memcpy(param,ptrtoparam,payload);
		return payload;
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE;
	}
}

//Esta funcion obtiene un puntero a la zona de memoria donde esta el parametro dentro de la trama
//buffer es la zona de memoria donde esta almacenada la trama "Desestufada"
//frame_size es su tamanyo
//campo es un puntero a void que se pasa por REFERENCIA. Quedaria apuntando a la zona de memoria donde estz el parametro
//Devuelve: Tamanyo del parametro recibido o un valor negativo si hay error
int32_t get_message_param_pointer(uint8_t * buffer, int32_t frame_size, void **campo)
{
	int32_t param_size=frame_size-MESSAGE_SIZE-CHECKSUM_SIZE;

	*campo=buffer+MESSAGE_SIZE;
	if (param_size<0)
		return PROT_ERROR_BAD_SIZE; //Devuelve un codigo de error
	else
		return param_size;
}

/* Recibe una trama (sin los caracteres de inicio y fin */
/* Utiliza la funcion bloqueante SerialGetChar ---> Esta funcion es bloqueante y no se puede llamar desde una ISR !!!*/
int receive_frame(uint8_t *frame, int32_t maxFrameSize)
{
    int i;
    uint8_t rxByte;
    uint8_t *initFrame=frame;
	
    do
    {
        //Elimino bytes de la cola de recepcion hasta llegar a comienzo de nueva trama
        USBSerialGetChar( ( char *)&rxByte, portMAX_DELAY);
    } while (rxByte!=START_FRAME_CHAR);

    i=0;
    do
    {
        USBSerialGetChar( ( char *)&rxByte, portMAX_DELAY);
        if(rxByte!=START_FRAME_CHAR){
            i++;
            *(frame)=rxByte;
            frame++;
        }else{
            i=0;
            frame=initFrame;
        }
    } while ((rxByte!=STOP_FRAME_CHAR)&&(i<maxFrameSize));

    if (i == maxFrameSize)
    {
        return PROT_ERROR_RX_FRAME_TOO_LONG;    //Numero Negativo indicando error
    }
    else
    {

        if (i<(MINIMUM_FRAME_SIZE-START_SIZE))
        {
            return PROT_ERROR_BAD_SIZE; //La trama no tiene el tamanio minimo
        }
        else
        {
            return (i-END_SIZE); //Tamanio de la trama sin los bytes de inicio y fin
        }
    }
}

/* Envia la trama usando la funcion bloqueante SerialPutChar ---> Esta funcion es bloqueante  y no se puede llamar desde una ISR */
int32_t send_frame(uint8_t *frame, int32_t FrameSize)
{
    int32_t i;

    for(i=FrameSize;i>0;i--)
    {
        USBSerialPutChar(  *frame++, portMAX_DELAY);
    }

    return FrameSize;
}





