#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "inc/ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <arm_math.h>

// OLED configurations
const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

// Microphone configurations
#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)
#define FFT_SIZE 256
#define HOP_SIZE 128
#define NUM_WINDOWS 20
#define SAMPLES (FFT_SIZE + (NUM_WINDOWS-1)*HOP_SIZE) // 256 + 19*128 = 2688

// ADC Clock Divider para 8 kHz
#define ADC_CLOCK_DIV 62.5f  // 48e6 / (8000 * 96)
#define SAMPLE_RATE 8000.0f

// FFT configurations
arm_rfft_fast_instance_f32 fft_instance;
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];

// DMA channel and configurations
uint dma_channel;
dma_channel_config dma_cfg;

// ADC sample buffer
uint16_t adc_buffer[SAMPLES];


#define ARM_MATH_CM0PLUS  // Para RP2040 (Cortex-M0+)

void sample_mic();
void calculate_frequency_contour_fft(float *samples, int num_samples, float *contour, int contour_length);
void normalize_contour(float *contour, int contour_length);
const char* identify_tone(float *contour, int len);
void display_tone(uint8_t *ssd, struct render_area *area, const char *tone);

int main() {
    stdio_init_all();
    
    // Initialize FFT
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);

    // I2C initialization for OLED
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // OLED initialization
    ssd1306_init();

    struct render_area frame_area = {
        start_column : 0,
        end_column   : ssd1306_width - 1,
        start_page   : 0,
        end_page     : ssd1306_n_pages - 1
    };

    calculate_render_area_buffer_length(&frame_area);

    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    // ADC Setup
    adc_gpio_init(MIC_PIN);
    adc_init();
    adc_select_input(MIC_CHANNEL);

    adc_fifo_setup(true, true, 1, false, false);
    adc_set_clkdiv(ADC_CLOCK_DIV);

    // DMA Configuration
    dma_channel = dma_claim_unused_channel(true);
    dma_cfg = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_dreq(&dma_cfg, DREQ_ADC);

    float samples[SAMPLES];
    float contour[NUM_WINDOWS];

    while (true) {
        sample_mic();

        // Normalização com ganho ajustado
        float gain_factor = 2.5f;
        for (int i = 0; i < SAMPLES; i++) {
            samples[i] = ((adc_buffer[i] / 4095.0f) * gain_factor * 2.0f - 1.0f);
        }

        calculate_frequency_contour_fft(samples, SAMPLES, contour, NUM_WINDOWS);
        normalize_contour(contour, NUM_WINDOWS);

        const char *tone = identify_tone(contour, NUM_WINDOWS);
        display_tone(ssd, &frame_area, tone);
        
        sleep_ms(200);
    }

    return 0;
}

void sample_mic() {
    adc_fifo_drain();
    adc_run(false);
    dma_channel_configure(dma_channel, &dma_cfg,
                          adc_buffer,
                          &(adc_hw->fifo),
                          SAMPLES,
                          true);
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);
    adc_run(false);
}

void calculate_frequency_contour_fft(float *samples, int num_samples, float *contour, int contour_length) {
    const float freq_res = SAMPLE_RATE / FFT_SIZE;
    const int start_bin = (int)(65.0f / freq_res);
    const int end_bin = (int)(500.0f / freq_res);

    for (int win = 0; win < contour_length; win++) {
        // Aplica janela de Hamming com sobreposição
        for (int i = 0; i < FFT_SIZE; i++) {
            int idx = win * HOP_SIZE + i;
            float window = 0.54f - 0.46f * cosf(2 * M_PI * i / (FFT_SIZE - 1));
            fft_input[i] = (idx < num_samples) ? samples[idx] * window : 0.0f;
        }

        // Executa FFT
        arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

        // Encontra magnitude máxima na banda alvo
        float max_mag = 0.0f;
        for (int bin = start_bin; bin <= end_bin; bin++) {
            float real = fft_output[2*bin];
            float imag = fft_output[2*bin+1];
            float mag = sqrtf(real*real + imag*imag);
            mag *= 1.5f; // Aplicar um ganho adicional
            if(mag > max_mag) max_mag = mag;
        }
        
        contour[win] = max_mag;
    }
}

void normalize_contour(float *contour, int contour_length) {
    float max_val = 0.01f;
    for (int i = 0; i < contour_length; i++) {
        if(contour[i] > max_val) max_val = contour[i];
    }
    
    for (int i = 0; i < contour_length; i++) {
        contour[i] = (contour[i] / max_val) * 100.0f;
    }
}

const char* identify_tone(float *contour, int len) {
    int seg = len / 3;
    float inicio = 0.0f, meio = 0.0f, fim = 0.0f;
    float max_valor = 0.0f;

    if(seg > 0) {
        for(int i = 0; i < seg; i++) {
            inicio += contour[i];
            max_valor = fmaxf(max_valor, contour[i]);
        }
        inicio /= seg;
        
        for(int i = seg; i < 2*seg; i++) {
            meio += contour[i];
            max_valor = fmaxf(max_valor, contour[i]);
        }
        meio /= seg;
        
        for(int i = 2*seg; i < len; i++) {
            fim += contour[i];
            max_valor = fmaxf(max_valor, contour[i]);
        }
        fim /= (len - 2*seg);
    }

    float tol_base = fmaxf(3.2f, 14.0f - (max_valor * 0.05f));
    float tol_DA = tol_base * 0.6f;

    static uint8_t confirm_count = 0;
    static const char *last_tone = "Silencio";
    
    static float hist_delta[2] = {0};
    float delta1 = (meio - inicio) * 0.85f + hist_delta[0] * 0.15f;
    float delta2 = (fim - meio) * 0.85f + hist_delta[1] * 0.15f;
    hist_delta[0] = delta1;
    hist_delta[1] = delta2;

    float min_energy = fmaxf(7.5f, max_valor * 0.18f);
    if(max_valor < min_energy) {
        confirm_count = 0;
        last_tone = "Silencio";
    }
    else {
        float rel_tol = tol_base * (0.85f - (max_valor/150.0f));
        
        // Definir uma tolerância muito pequena para variações
        float small_variation = 0.02f * max_valor; // 3% do valor máximo, por exemplo
        float small_variton_for_first_tone = 0.4f * max_valor;
        if (fabsf(delta1) < small_variation && fabsf(delta2) < small_variton_for_first_tone) {
            if (++confirm_count > 0) last_tone = "Primeiro Tom ";
        }

        else if(delta1 > (tol_base * 0.5f) && delta2 > (-tol_base * 0.05f)) {
            if(++confirm_count > 0) last_tone = "Segundo Tom";
        }
        else if(delta1 < (-tol_base * 0.5f) && delta2 < (tol_base * 0.05f)) {
            if(++confirm_count > 0) last_tone = "Quarto Tom";
        }
        else if((delta1 < -tol_base*0.7f) && 
            (delta2 > tol_DA*0.8f) && 
            ((fim - inicio) > -tol_base*0.85f)) {
            if(++confirm_count > 1) last_tone = "Terceiro Tom";
        }
    }

    return (confirm_count > 1) ? last_tone : "Aguardando confirmacao";
}

void display_tone(uint8_t *ssd, struct render_area *area, const char *tone) {
    memset(ssd, 0, ssd1306_buffer_length);
    int x_pos = (ssd1306_width - ((int)strlen(tone) * 6)) / 2;
    if (x_pos < 0) x_pos = 0;
    ssd1306_draw_string(ssd, x_pos, 28, tone);
    render_on_display(ssd, area);
}
