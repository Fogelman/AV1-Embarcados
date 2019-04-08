/*
 * main.c
 *
 * Created: 05/03/2019 18:00:58
 *  Author: eduardo
 */ 

#include <asf.h>
#include "tfont.h"
#include "sourcecodepro_28.h"
#include "calibri_36.h"
#include "arial_72.h"

#define PI 3.14
#define RAD 0.650
#define DT 4
struct ili9488_opt_t g_ili9488_display_opt;


// RTT 
volatile Bool f_rtt_alarme = false;
volatile Bool tc_alarm = false;
volatile unsigned int totalPulses = 0;
volatile unsigned int previousPulses = 0;
volatile float totalDist = 0;


#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)


#define BUT_PIO_ID			  ID_PIOA
#define BUT_PIO				  PIOA
#define BUT_PIN				  11
#define BUT_PIN_MASK		  (1 << BUT_PIN)

//RTC

#define YEAR        2019
#define MOUNTH      4
#define DAY         7
#define WEEK        1
#define HOUR        0
#define MINUTE      0
#define SECOND      0

//TC

#define TC_CH TC0
#define TC_ID ID_TC1

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			
			rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
			rtc_set_time(RTC, HOUR, MINUTE, SECOND);
			
			
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);

}

static void Button0_Handler(uint32_t id, uint32_t mask)
{
	totalPulses = totalPulses + 1;
}



void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		f_rtt_alarme = true;                  // flag RTT alarme
	}
}



void BUT_init(void){
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button0_Handler);
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	
}


static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}


void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	tc_alarm = true;
}


void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	
}


void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}	
}


float atualizaDist(int pulses){
	//totalDist = totalDist + (2*PI*pulses)/dt;
	totalDist = totalDist +( 2*PI*RAD*pulses);
	return totalDist;
}

float calculaVel(int pulses){
	//totalDist = totalDist + (2*PI*pulses)/dt;
	float vel = 2*PI*pulses/DT;
	return vel;
}

int main(void) {
	board_init();
	sysclk_init();	
	configure_lcd();
	TC_init(TC_CH, TC_ID, 1, 8);
	BUT_init();
	RTC_init();
	
	sysclk_init();
	
	char buf[20];
	f_rtt_alarme = true;
	uint16_t pllPreScale = (int) (((float) 32768) / 2.0);
	uint32_t irqRTTvalue  = 8;
	 
	//font_draw_text(&sourcecodepro_28, "OIMUNDO", 50, 50, 1);
	//font_draw_text(&calibri_36, "Oi Mundo! #$!@", 50, 100, 1);
	font_draw_text(&calibri_36, "Speed", 20, 10, 1);
	font_draw_text(&calibri_36, "Total Distance", 20, 80, 1);
	font_draw_text(&calibri_36, "Total Time", 20, 160, 1);
	//font_draw_text(&arial_72, "102456", 50, 200, 2);
	tc_alarm = false;
	totalDist = 0;
	totalPulses = 0;
	while(1) {
		
	if (f_rtt_alarme){
		
	  sprintf(buf, "%.3f m/s",calculaVel(totalPulses));
	 /* ili9488_draw_filled_rectangle(0, 50, ILI9488_LCD_WIDTH-1, 10);*/
      font_draw_text(&calibri_36, buf, 50, 50, 1);
	  
	  sprintf(buf, "%.3f m",atualizaDist(totalPulses));
	  /*ili9488_draw_filled_rectangle(0, 120, ILI9488_LCD_WIDTH-1, 0);*/
	  font_draw_text(&calibri_36, buf, 50, 120, 1);
 
      RTT_init(pllPreScale, irqRTTvalue);         
	  
      f_rtt_alarme = false;
	  
totalPulses = 0;
	  
    }
	if(tc_alarm){
		uint32_t hour;
		uint32_t minute;
		uint32_t second;
		rtc_get_time(RTC,&hour,&minute,&second );
		 sprintf(buf, "%d:%d:%d",hour,minute,second);
		font_draw_text(&calibri_36, buf, 20, 180, 1);
		tc_alarm = false;
	}
	
	
		
	}
}