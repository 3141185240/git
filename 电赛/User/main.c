#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ========== 引脚 ==========
#define La_1 GPIO_Pin_14

#define M1_EN    GPIO_Pin_0
#define M1_STEP  GPIO_Pin_1
#define M1_DIR   GPIO_Pin_12
#define M1_MS2   GPIO_Pin_4
#define M1_MS1   GPIO_Pin_5

#define M2_EN    GPIO_Pin_7
#define M2_STEP  GPIO_Pin_6   
#define M2_DIR   GPIO_Pin_5   
#define M2_MS2   GPIO_Pin_6
#define M2_MS1   GPIO_Pin_7

#define MOTOR_GPIO_PORT GPIOB
#define MOTOR_GPIO_RCC  RCC_APB2Periph_GPIOB

// ========== 固定速度 + 防抖死区 ==========
#define STEP_DELAY   700
#define DEAD_ZONE    4


// ========== 串口缓冲区 ==========
#define RX_BUF_SIZE 64
uint8_t uart_rx_buf[RX_BUF_SIZE];
uint8_t rx_idx = 0;
uint8_t data_ready = 0;

int16_t err_x = 0;
int16_t err_y = 0;
uint8_t target_ok = 0;
uint8_t target_no = 0;

uint8_t i = 0;
uint8_t l = 0;
uint8_t n = 0;

// ========== 延时 ==========
void delay_us(uint32_t us)
{
    us *= 9;
    while(us--) __NOP();
}

void delay_ms(uint32_t ms)
{
    SysTick->LOAD = 72000 * ms;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x05;
    while(!(SysTick->CTRL & (1<<16)));
    SysTick->CTRL = 0x04;
}

//电机细分初始化
void xifen_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_xifen;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_xifen.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_xifen.GPIO_Pin = M1_MS1 | M1_MS2 | M2_MS1 | M2_MS2;
	GPIO_xifen.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_xifen);
	
	GPIO_ResetBits(GPIOA,M1_MS1);
	GPIO_SetBits(GPIOA,M1_MS2);
	GPIO_ResetBits(GPIOA,M2_MS1);
	GPIO_SetBits(GPIOA,M2_MS2);
}

// ========== 电机初始化 ==========
void motor_gpio_init(void)
{
    GPIO_InitTypeDef gpio_conf;
    RCC_APB2PeriphClockCmd(MOTOR_GPIO_RCC, ENABLE);

    gpio_conf.GPIO_Pin = M1_EN|M1_STEP|M1_DIR|M2_EN|M2_STEP|M2_DIR|La_1;
    gpio_conf.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_conf.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_GPIO_PORT, &gpio_conf);

    // ---- 上电直接释放电机 ----
    GPIO_ResetBits(MOTOR_GPIO_PORT, M1_EN | M2_EN);
}

// ========== 单步运行 ==========
void motor_run(uint16_t STEP, uint16_t DIR, int16_t err)
{
    // 误差为0  不动
    if(err == 0) return;

    // 设置方向
    if(err > 0){
        GPIO_SetBits(MOTOR_GPIO_PORT, DIR);
    }else{
        GPIO_ResetBits(MOTOR_GPIO_PORT, DIR);
    }

    // 误差绝对值 > 死区 → 才动
    if(abs(err) > DEAD_ZONE)
    {
        GPIO_SetBits(MOTOR_GPIO_PORT, STEP);
        delay_us(STEP_DELAY);
        GPIO_ResetBits(MOTOR_GPIO_PORT, STEP);
        delay_us(STEP_DELAY);
    }
}

// 功能：画心电图
void draw_ecg_wave(void)
{
    uint16_t j;
			for(j = 0; j < 173; j++)
			{
				// X走一步
				GPIO_SetBits(MOTOR_GPIO_PORT,M1_DIR);
				GPIO_SetBits(MOTOR_GPIO_PORT, M1_STEP);
				delay_ms(5);
				GPIO_ResetBits(MOTOR_GPIO_PORT, M1_STEP);
				delay_ms(5);

				// Y同步画波形
				if(j >= 10 && j < 20)
				{
					GPIO_SetBits(GPIOA,M2_MS2);
					GPIO_SetBits(GPIOA,M2_MS1);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_DIR);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
				}
				else if(j >= 20 && j < 30)
				{
					GPIO_SetBits(GPIOA,M2_MS2);
					GPIO_SetBits(GPIOA,M2_MS1);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_DIR);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
				}
				else if(j >= 30 && j < 45)
				{
					GPIO_ResetBits(GPIOA,M2_MS2);
					GPIO_ResetBits(GPIOA,M2_MS1);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_DIR);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
				}
				else if(j >= 45 && j < 60)
				{
					GPIO_ResetBits(GPIOA,M2_MS2);
					GPIO_ResetBits(GPIOA,M2_MS1);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_DIR);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(25);
				}
				else if(j >= 60 && j < 75)
				{
					GPIO_SetBits(GPIOA,M2_MS2);
					GPIO_ResetBits(GPIOA,M2_MS1);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_DIR);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(40);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(40);
				}
				else if(j >= 75 && j < 90)
				{
					GPIO_SetBits(GPIOA,M2_MS2);
					GPIO_ResetBits(GPIOA,M2_MS1);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_DIR);
					GPIO_SetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(40);
					GPIO_ResetBits(MOTOR_GPIO_PORT, M2_STEP);
					delay_us(40);
				}
		
}
}
void motor_run_1(void)
{
    if(i == 1)
    {
        delay_ms(4000);       
        GPIO_SetBits(MOTOR_GPIO_PORT, La_1);  // 激光亮
		for(n=0;n<3;n++)
		{
			draw_ecg_wave();
		}
        GPIO_ResetBits(MOTOR_GPIO_PORT, La_1); // 激光关
		GPIO_SetBits(MOTOR_GPIO_PORT, M1_EN | M2_EN);
		l = 1;
    }
    else
    {
        if(target_no)
        {
            GPIO_SetBits(MOTOR_GPIO_PORT, La_1);

            // X轴
            if(abs(err_x) > DEAD_ZONE)
            {
                if(err_x > 0) GPIO_SetBits(MOTOR_GPIO_PORT, M1_DIR);
                else GPIO_ResetBits(MOTOR_GPIO_PORT, M1_DIR);

                GPIO_SetBits(MOTOR_GPIO_PORT, M1_STEP);
                delay_us(STEP_DELAY);
                GPIO_ResetBits(MOTOR_GPIO_PORT, M1_STEP);
                delay_us(STEP_DELAY);
            }

            // Y轴
            if(abs(err_y) > DEAD_ZONE)
            {
                if(err_y > 0) GPIO_SetBits(MOTOR_GPIO_PORT, M2_DIR);
                else GPIO_ResetBits(MOTOR_GPIO_PORT, M2_DIR);

                GPIO_SetBits(MOTOR_GPIO_PORT, M2_STEP);
                delay_us(STEP_DELAY);
                GPIO_ResetBits(MOTOR_GPIO_PORT, M2_STEP);
                delay_us(STEP_DELAY);
            }

            // ======================
            // 关键：追踪到位 → 自动触发画图
            // ======================
            if(abs(err_x) <= DEAD_ZONE && abs(err_y) <= DEAD_ZONE)
            {
                i = 1;
            }
        }
        else
        {
            GPIO_ResetBits(MOTOR_GPIO_PORT, La_1);
        }
    }
}

// ========== 串口2初始化 ==========
void uart2_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio_conf;
    USART_InitTypeDef uart_conf;
    NVIC_InitTypeDef nvic_conf;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // TX PA2
    gpio_conf.GPIO_Pin = GPIO_Pin_2;
    gpio_conf.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio_conf);

    // RX PA3
    gpio_conf.GPIO_Pin = GPIO_Pin_3;
    gpio_conf.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio_conf);

    uart_conf.USART_BaudRate = baud;
    uart_conf.USART_WordLength = USART_WordLength_8b;
    uart_conf.USART_StopBits = USART_StopBits_1;
    uart_conf.USART_Parity = USART_Parity_No;
    uart_conf.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_conf.USART_Mode = USART_Mode_Rx;
    USART_Init(USART2, &uart_conf);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic_conf.NVIC_IRQChannel = USART2_IRQn;
    nvic_conf.NVIC_IRQChannelPreemptionPriority = 2;
    nvic_conf.NVIC_IRQChannelSubPriority = 2;
    nvic_conf.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_conf);

    USART_Cmd(USART2, ENABLE);
}

// ========== 串口接收 ==========
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        uint8_t c = USART_ReceiveData(USART2);

        // ========================
        // 新协议结束符 D
        // ========================
        if(c == 'D')
        {
            uart_rx_buf[rx_idx] = 0;
            rx_idx = 0;
            data_ready = 1;
        }
        // ========================
        // 旧协议结束符 E
        // ========================
        else if(c == 'E')
        {
            uart_rx_buf[rx_idx] = 0;
            rx_idx = 0;
            data_ready = 1;
        }
        // ========================
        // 正常存数据
        // ========================
        else if(rx_idx < RX_BUF_SIZE-1)
        {
            uart_rx_buf[rx_idx++] = c;
        }
        else
        {
            rx_idx = 0;
        }
    }
}

// ========== 解析数据 ==========
void parse_data(void)
{
    if(!data_ready) return;
    data_ready = 0;

    // ----------------------------
    // 先判断是哪种协议
    // ----------------------------
    if(strstr((char*)uart_rx_buf, "A") != NULL)
    {
        // ========================
        // 新协议：A[x]B[y]C[状态]D
        // ========================
        char *pA = strstr((char*)uart_rx_buf, "A");
        char *pB = strstr((char*)uart_rx_buf, "B");
        char *pC = strstr((char*)uart_rx_buf, "C");

        if(pA && pB && pC)
        {
            err_x = atoi(pA + 1);   // A 后面是 X 误差
            err_y = atoi(pB + 1);   // B 后面是 Y 误差
            target_no = atoi(pC + 1); // C 后面是状态
        }
    }
    else
    {
        // ========================
        // 原来的协议：X[x]Y[y]Z[状态]E
        // ========================
        char *pX = strstr((char*)uart_rx_buf, "X");
        char *pY = strstr((char*)uart_rx_buf, "Y");
        char *pZ = strstr((char*)uart_rx_buf, "Z");

        if(pX && pY && pZ)
        {
            err_x = atoi(pX + 1);
            err_y = atoi(pY + 1);
            target_ok = atoi(pZ + 1);
        }
    }
}

// ========== 主函数 ==========
int main(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    SysTick->CTRL |= (1<<2);

    motor_gpio_init();
    uart2_init(115200);
	xifen_gpio_init();

    while(1)
    {
        parse_data();       

        if(target_ok)
        {
			GPIO_ResetBits(GPIOA,M1_MS1);
			GPIO_SetBits(GPIOA,M1_MS2);
			GPIO_ResetBits(GPIOA,M2_MS1);
			GPIO_SetBits(GPIOA,M2_MS2);
			
			GPIO_ResetBits(MOTOR_GPIO_PORT, M1_EN | M2_EN);
            GPIO_SetBits(MOTOR_GPIO_PORT,La_1);
			motor_run(M1_STEP, M1_DIR, err_x);
            motor_run(M2_STEP, M2_DIR, err_y);
        }
        else if(target_no)
		{
			GPIO_ResetBits(MOTOR_GPIO_PORT, M1_EN | M2_EN);
			if(l == 0)
			{
				motor_run_1();
			}
			
		}else
        {
            GPIO_ResetBits(MOTOR_GPIO_PORT,La_1);
        }
    }
}