#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include <math.h>

#include "Fusion.h"
#define SAMPLE_PERIOD (0.1f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

QueueHandle_t xQueuePos;

typedef struct btn {
    int axis;
    int val;
} btn;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // Configuração do barramento I2C para comunicação com o sensor MPU6050
    i2c_init(i2c_default, 400 * 1000);             // Inicializa o I2C a 400 kHz (modo rápido)
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C); // Define o pino SDA (dados) como função I2C
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C); // Define o pino SCL (clock) como função I2C
    gpio_pull_up(I2C_SDA_GPIO);                     // Ativa resistor de pull-up (necessário no barramento I2C)
    gpio_pull_up(I2C_SCL_GPIO);                     // Idem para o pino SCL

    mpu6050_reset(); // Reinicia o sensor e o tira do modo “sleep”

    // Declara arrays para armazenar leituras do sensor
    int16_t acceleration[3], gyro[3], temp;

    // Estrutura da biblioteca Fusion para cálculo de orientação (AHRS)
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs); // Inicializa o algoritmo de fusão de sensores

    // Estrutura para envio de dados pela fila do FreeRTOS
    btn newbtn;

    // Loop infinito da task (executa continuamente enquanto o RTOS estiver rodando)
    while(1) {
        // Lê valores brutos do sensor: aceleração, giroscópio e temperatura
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Converte as leituras do giroscópio de unidades brutas para graus por segundo
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        // Converte as leituras do acelerômetro de unidades brutas para “g” (gravidade)
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };      

        // Atualiza o filtro AHRS (usa apenas acelerômetro + giroscópio, sem magnetômetro)
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Converte o quaternion calculado em ângulos de Euler (pitch, roll e yaw)
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        //------------------------------------------------------------
        // ENVIO DOS DADOS DE ORIENTAÇÃO PELA FILA
        //------------------------------------------------------------

        // Se o ângulo de pitch estiver fora do intervalo -20° a +20°, envia pela fila
        if (euler.angle.pitch < 20.0 && euler.angle.pitch > -20.0){
            ; // não faz nada (dentro da zona morta)
        }
        else {
            newbtn.axis = 1;                        // 1 representa o eixo pitch
            newbtn.val = (int) euler.angle.pitch;   // valor convertido para inteiro
            xQueueSend(xQueuePos, &newbtn, 0);      // envia estrutura 'btn' para a fila
        }

        // Se o ângulo de roll estiver fora do intervalo -20° a +20°, envia pela fila
        if (euler.angle.roll < 20.0 && euler.angle.roll > -20.0){
            ; // idem
        }
        else {
            newbtn.axis = 0;                        // 0 representa o eixo roll
            newbtn.val = (int) euler.angle.roll;
            xQueueSend(xQueuePos, &newbtn, 0);
        }

        //------------------------------------------------------------
        // DETECÇÃO DE MOVIMENTO BRUSCO (BATIDA / AGITAÇÃO)
        //------------------------------------------------------------

        int acc;

        // Calcula o módulo do vetor de aceleração e subtrai o valor médio (~1g ≈ 16384)
        acc = sqrt((acceleration[0]*acceleration[0]) +
                   (acceleration[1]*acceleration[1]) +
                   (acceleration[2]*acceleration[2])) - 15000;

        // Se o módulo exceder um limiar (±15000), considera um movimento brusco
        if (acc < 15000 && acc > -15000){
            ; // nada detectado
        } 
        else {
            newbtn.axis = 2;     // 2 representa “evento especial” (movimento brusco)
            newbtn.val = 0;      // não há valor associado
            xQueueSend(xQueuePos, &newbtn, 0);
        }

        // Aguarda 10 ms antes da próxima leitura (define a taxa de amostragem)
        vTaskDelay(pdMS_TO_TICKS(70));
    }
}

void uart_task(void *p){
    btn xya;

    while (1){
        if (xQueueReceive(xQueuePos, &xya, pdMS_TO_TICKS(200)) == pdTRUE){
        uart_putc_raw(uart0, xya.axis);
        uart_putc_raw(uart0, xya.val);
        uart_putc_raw(uart0, xya.val >> 8);
        uart_putc_raw(uart0, -1);
        // printf("Axis: %d | Val: %d\n", xya.axis, xya.val);
        }
    }
}

int main() {
    stdio_init_all();

    // Inicializa UART0 para transmissão de bytes brutos
    // Ajuste baudrate/pinos conforme seu hardware se necessário
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART); // TX
    gpio_set_function(1, GPIO_FUNC_UART); // RX

    xQueuePos = xQueueCreate(3, sizeof(btn));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 18192, NULL, 1, NULL);
    xTaskCreate(uart_task, "mpu4323446050_Task 1", 10192, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true)
        ;
}