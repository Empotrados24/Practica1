//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT (Febrero2024)
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de mensajes con un interfaz
//  gráfico (GUI) Qt.
//  La aplicacion se basa en un intercambio de mensajes con ordenes e informacion, a traves  de la
//  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementara la recepcion de un mensaje
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  mensaje enviado desde la placa al GUI, para mostrar el estado de los botones.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (ROM_)
#include "driverlib/rom_map.h"   // TIVA: Para usar la opción MAP en las funciones API (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "utils/uartstdioMod.h"  // TIVA: Funciones API UARTSTDIO (printf)
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADC
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
#include "utils/cpu_usage.h"
#include "commands.h"
#include <serial2USBprotocol.h>
#include <usb_dev_serial.h>
#include "usb_messages_table.h"

// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
SemaphoreHandle_t mutexUSB;  // Para proteccion del canal USB, ya que ahora lo usan varias tareas distintas
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ulLine)
{
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

void vApplicationTickHook( void )
{
    static uint8_t ui8Count = 0;

    if (++ui8Count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        ui8Count = 0;
    }
    //return;
}

void vApplicationIdleHook (void)
{
    SysCtlSleep();
}

void vApplicationMallocFailedHook (void)
{
    while(1);
}

//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

//*****************************************************************************
//
// Codigo de tarea que procesa los mensajes recibidos a traves del canal USB
//
//*****************************************************************************
static portTASK_FUNCTION( USBMessageProcessingTask, pvParameters ){

    uint8_t pui8Frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
    int32_t i32Numdatos;
    uint8_t ui8Message;
    void *ptrtoreceivedparam;
    uint32_t ui32Errors=0;

    /* The parameters are not used. */
    ( void ) pvParameters;

    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\n\nBienvenido a la aplicacion Fabrica de Refrescos (curso 2023/24)!\n");
    UARTprintf("\nAutores: XXXXXX y XXXXX ");

    for(;;)
    {
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
        i32Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE); //Esta funcion es bloqueante
        if (i32Numdatos>0)
        {	//Si no hay error, proceso la trama que ha llegado.
            i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobación checksum
            if (i32Numdatos<0)
            {
                //Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
                ui32Errors++;
                // Procesamiento del error 
            }
            else
            {
                //El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo mensaje
                ui8Message=decode_message_type(pui8Frame);
                //Obtiene un puntero al campo de parametros y su tamanio.
                i32Numdatos=get_message_param_pointer(pui8Frame,i32Numdatos,&ptrtoreceivedparam);
                switch(ui8Message)
                {
                case MENSAJE_PING :
                    //A un mensaje de ping se responde con el propio mensaje
                    i32Numdatos=create_frame(pui8Frame,ui8Message,0,0,MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        xSemaphoreTake(mutexUSB,portMAX_DELAY);
                        send_frame(pui8Frame,i32Numdatos);
                        xSemaphoreGive(mutexUSB);
                    }else{
                        //Error de creacion de trama: determinar el error y abortar operacion
                        ui32Errors++;
                        // Procesamiento del error
                        // Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
                        // tener que copiar y pegar todo en cada operacion de creacion de paquete
                        switch(i32Numdatos){
                        case PROT_ERROR_NOMEM:
                            // Procesamiento del error NO MEMORY
                            break;
                        case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
                            // Procesamiento del error STUFFED_FRAME_TOO_LONG
                            break;
                        case PROT_ERROR_MESSAGE_TOO_LONG:
                            // Procesamiento del error MESSAGE TOO LONG
                            break;
                        }
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                        {
                            // Procesamiento del error INCORRECT PARAM SIZE 
                        }
                        break;
                    }
                    break;
                default:
                {
                    PARAM_MENSAJE_NO_IMPLEMENTADO parametro;
                    parametro.message=ui8Message;
                    //El mensaje esta bien pero no esta implementado
                    i32Numdatos=create_frame(pui8Frame,MENSAJE_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
                    if (i32Numdatos>=0)
                    {
                        xSemaphoreTake(mutexUSB,portMAX_DELAY);
                        send_frame(pui8Frame,i32Numdatos);
                        xSemaphoreGive(mutexUSB);
                    }
                    break;
                }
                }// switch
            }
        }else{ // if (ui32Numdatos >0)
            //Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
            ui32Errors++;
            // Procesamiento del error
        }
    }
}

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

    //
    // Reloj del sistema definido a 40MHz
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Obtiene el reloj del sistema
    g_ui32SystemClock = SysCtlClockGet();

    //Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
    //                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
    MAP_SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);


    /**                                              Creacion de tareas 									**/
    // Inicializa el sistema de depuración e interprete de comandos por terminal UART
    if (initCommandLine(256,tskIDLE_PRIORITY + 1) != pdPASS)
    {
        while(1);
    }

    USBSerialInit(32,32);	//Inicializo el  sistema USB
    //
    // Crea la tarea que gestiona los mensajes USB (definidos en USBMessageProcessingTask)
    //
    if(xTaskCreate(USBMessageProcessingTask,"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS)
    {
        while(1);
    }

    //
    // A partir de aqui se crean las tareas de usuario, y los recursos IPC que se vayan a necesitar
    //

	mutexUSB=xSemaphoreCreateMutex();
    if(NULL==mutexUSB)
    {
        while(1);
    }

    //
    // Pone en marcha el planificador. La llamada NO tiene retorno
    //
    vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

    //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
    while(1)
    {
        //Si llego aqui es que algo raro ha pasado
    }
}

