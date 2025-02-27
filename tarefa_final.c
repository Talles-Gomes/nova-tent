#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>      
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "hardware/timer.h"
#include "display_oled.h"

#define LED_G 11    // representa o relé
#define LED_R 13    // representa o alerta de estoque vazio
#define botA 5      // representa a ativação manual pelo push-button A
#define botB 6      // botão implementado para testes da simulação
#define Buzzer 10   // representa o circuito que reproduzirá a voz

#define I2C_PORT i2c1   // Declaração das portas do Oled que representam a câmera 
#define I2C_SDA 14 
#define I2C_SCL 15

bool hab = true; // variável de controle do temporizador
int carga = 5; // quantidade de porções de ração inicial
static volatile uint32_t last_time = 0;         //variável de tempo para debouncing
ssd1306_t oled;// configuração para o Oled
bool cor = true;// configuração para o Oled

static void gpio_irq_handler(uint gpio,uint32_t events);    // declaração da ativação manual

void chamado() // função que reproduz a simulação do áudio
{
    int time = 500; // tempo de acionamento do buzzer
    // executa enquanto o contador não zera
    while (time > 0)
    {
        // ativa o buzzer
        gpio_put(Buzzer, 1);
        // fica ativo por 1 ms
        sleep_ms(1);
        // decrementa contador
        time -= 1;
        // desativa o buzzer
        gpio_put(Buzzer, 0);
        // fica desativado por 3 ms
        sleep_ms(3);
        // decrementa contador
        time -= 3;
    }
}
void gpio_irq_handler(uint gpio,uint32_t events) // a função a ser chamada pela ativação manual
        {
            uint32_t current_time = to_us_since_boot(get_absolute_time());
            if(current_time - last_time > 200000) // 200 ms de debouncing
            {
                last_time = current_time;
                if(gpio == 5){
                    hab = !hab;// desliga o temporizador
                    cachorro();
                    chamado();

                }if (gpio == 6){ // função auxiliar para renovar o estoque
                    gpio_put(LED_R, 0); //desliga o alerta de estoque vazio
                    carga = 5; // renova para o valor máximo definido
                    }
                }
            }

int64_t turn_off_callback(alarm_id_t id, void *user_data) {// função chamada pelo temporizador
    if(carga>0){
carga --; // reduz o contador de porções
chamado(); //inicia o comando da voz
gpio_put(LED_G, 1);// LED acende simbolizando a abertura pra alimentação
busy_wait_ms(2000);// espera ativa para manter o LED aceso
gpio_put(LED_G, 0);// o relé fecha, completando a oferta da ração
                
    }return 0;
}
int main (){

    stdio_init_all();

    // declaração dos botões
    gpio_init(botA);
    gpio_set_dir(botA,GPIO_IN);
    gpio_pull_up(botA);

    gpio_init(botB);
    gpio_set_dir(botB,GPIO_IN);
    gpio_pull_up(botB);
    // declaração dos leds
    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);

    gpio_init(LED_R);
    gpio_set_dir(LED_R, GPIO_OUT);
    // declaração do buzzer
    gpio_init(Buzzer);
    gpio_set_dir(Buzzer, GPIO_OUT);
    //inicialização do Oled
    i2c_init(I2C_PORT, 400 * 1000);  // Inicializa I2C a 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&oled, 128, 64, false, 0x3C, I2C_PORT);  // Inicializa display
    ssd1306_config(&oled); // Configura o display
    //Habilita as interrupções manuais
    gpio_set_irq_enabled_with_callback(botA,GPIO_IRQ_EDGE_FALL,true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(botB,GPIO_IRQ_EDGE_FALL,true, &gpio_irq_handler);

    gpio_put(LED_R, 1);// envia alerta de que o estoque acabou

while(true)
   {
        if(carga>0){
            gpio_put(LED_R, 0);// desliga o alerta
            if(hab){
            add_alarm_in_ms(5000, turn_off_callback, NULL, false); //inicia o temporizador
            }

        }else{
            gpio_put(LED_R, 1);// envia alerta de que o estoque acabou
        }
   }
   return 0;
}