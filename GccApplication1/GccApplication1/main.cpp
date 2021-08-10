
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


unsigned int PLR_TIME_1,PLR_TIME_2;
unsigned char memPLR1, memPLR2;
signed int L1,L2,L3,L4,R1,R2,R3,R4;
unsigned char C_L1=' ',C_L2=' ',C_L3=' ',C_L4=' ',C_R1=' ',C_R2=' ',C_R3=' ',C_R4=' ';
bool flag_plr1 = false,flag_plr2 = false, set_time = true;

void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
	;
	/* Set up address and data registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMWE */
	EECR |= (1<<EEMWE);
	/* Start eeprom write by setting EEWE */
	EECR |= (1<<EEWE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE))
	;
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}


void segchar (unsigned int seg)
{
	switch(seg)
	{
		case 1: PORTA = 0b11111001;break;
		case 2: PORTA = 0b10100100;break;
		case 3: PORTA = 0b10110000;break;
		case 4: PORTA = 0b10011001;break;
		case 5: PORTA = 0b10010010;break;
		case 6: PORTA = 0b10000010;break;
		case 7: PORTA = 0b11111000;break;
		case 8: PORTA = 0b10000000;break;
		case 9: PORTA = 0b10010000;break;
		case 0: PORTA = 0b11000000;break;
		case '-': PORTA = 0b10111111;break;
	}
}

void uartINIT ()
{
	UBRRH = (unsigned char) (12>>8);
	UBRRL = (unsigned char) 12;
	UCSRB = (1<<TXEN);
	UCSRB|= (1<<RXCIE);
	UCSRA|= (1<<U2X);
	UCSRC = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);
}

void uartSEND (unsigned char data)
{
	while(!(UCSRA&(1<<UDRE)));
	UDR = data;
	
		

	
}
void timerINIT (void)
{
	TCCR1B |=(1<<WGM12);
	TIMSK |= (1<<OCIE1A);
	OCR1AH = 0b00001111; 
	OCR1AL = 0b01000010;
	TCCR1B |= (1<<CS12);
}
void showNumber(unsigned int num1,unsigned int num2)
{
	
	
	
	
	L4 = num1%10;
	num1/=10;
	L3 = num1%10;
	num1/=10;
	L2 = num1%10;
	num1/=10;
	L1 = num1%10;	
	
	R4 = num2%10;
	num2/=10;
	R3 = num2%10;
	num2/=10;
	R2 = num2%10;
	num2/=10;
	R1 = num2%10;
	
}
ISR (TIMER1_COMPA_vect)
{
	
	if (flag_plr1 == true) R4--;
	if (flag_plr2 == true) L4--;
	
	if (R4 < 0) {R3--; R4 = 9; }
	if (R3 < 0) {R2--; R3 = 5;}	
	if (R2 < 0) {R1--; R2 = 9;}	
		
	if (L4 < 0) {L3--; L4 = 9;}
	if (L3 < 0) {L2--; L3 = 5;}
	if (L2 < 0) {L1--; L2 = 9;}		
	PLR_TIME_1 = (L1*1000+L2*100+L3*10+L4);
	PLR_TIME_2 = (R1*1000+R2*100+R3*10+R4);
	
	
	EEPROM_write(1,(L4*10+L3));
	EEPROM_write(2,(L2*10+L1));
	
	EEPROM_write(3,(R4*10+R3));
	EEPROM_write(4,(R2*10+R1));
	
}


int main(void)
{
	uartINIT();
	timerINIT();
	flag_plr1 = false;
	flag_plr2 = false;
	
    DDRC = 0xFF;
	DDRA = 0xFF;
    DDRB = 0b01110000;
	PORTB =0b00001111;
	DDRD |= (1<<PD1);
	
	
	//PLR_TIME_1=EEPROM_read(1)*100+EEPROM_read(4);
	//PLR_TIME_2=EEPROM_read(3)*100+EEPROM_read(4);
	//showNumber(PLR_TIME_1,PLR_TIME_2);
	if ((EEPROM_read(1)==0) & (EEPROM_read(2)==0)&(EEPROM_read(3)==0)&(EEPROM_read(4)==0))
		{
			PLR_TIME_1=1500;
			PLR_TIME_2=1500;
			showNumber(PLR_TIME_1,PLR_TIME_2);
		} else{
			PLR_TIME_1=((EEPROM_read(1))*100+EEPROM_read(2));
			PLR_TIME_2=((EEPROM_read(3))*100+EEPROM_read(4));
			
			unsigned int kostil_L4 = PLR_TIME_1%10;
			PLR_TIME_1/=10;
			unsigned int kostil_L3 = PLR_TIME_1%10;
			PLR_TIME_1/=10;
			unsigned int kostil_L2 = PLR_TIME_1%10;
			PLR_TIME_1/=10;
			unsigned int kostil_L1 = PLR_TIME_1%10;
			PLR_TIME_1 = kostil_L4*1000+kostil_L3*100+kostil_L2*10+kostil_L1;
			
			unsigned int kostil_R4 = PLR_TIME_2%10;
			PLR_TIME_2/=10;
			unsigned int kostil_R3 = PLR_TIME_2%10;
			PLR_TIME_2/=10;
			unsigned int kostil_R2 = PLR_TIME_2%10;
			PLR_TIME_2/=10;
			unsigned int kostil_R1 = PLR_TIME_2%10;
			PLR_TIME_2 = kostil_R4*1000+kostil_R3*100+kostil_R2*10+kostil_R1;		
			showNumber(PLR_TIME_1,PLR_TIME_2);
		}
		
	
	
	
	/*
	if ((EEPROM_read(1)*100+EEPROM_read(2))!=1500)
	{
		showNumber(EEPROM_read(1),EEPROM_read(2));
	}else{
			PLR_TIME_1=1500;
			PLR_TIME_2=1500;
			showNumber(PLR_TIME_1,PLR_TIME_2);
		 }
			
    */
	
		
	
    sei();
	
    while (1) 
    {
		if (!(PINB&0b00000001)) {flag_plr1 = true; flag_plr2 = false;}
			
		if (!(PINB&0b00000010)) {flag_plr1 = false; flag_plr2 = true;}	
			
			
		
		if((set_time == false) & (!(PINB&0b00000100)))
		{
			PLR_TIME_1=1500;
			PLR_TIME_2=1500;
			set_time = true;
			flag_plr1 = false; flag_plr2 = false;
			showNumber(PLR_TIME_1,PLR_TIME_2);
			_delay_ms(1000);
		}
		
		
		if((set_time == true) & (!(PINB&0b00000100)))
		{
			PLR_TIME_1=500;
			PLR_TIME_2=500;
			set_time = false;
			flag_plr1 = false; flag_plr2 = false;
			showNumber(PLR_TIME_1,PLR_TIME_2);
			_delay_ms(1000);
		}
		
		
		if (!(PINB&0b00001000)) 
			{
				//C_R4 = L1 + '0';
				uartSEND(L1+'0');
				uartSEND(L2+'0');
				uartSEND(L3+'0');
				uartSEND(L4+'0');
				uartSEND(':');
				uartSEND(R1+'0');
				uartSEND(R2+'0');
				uartSEND(R3+'0');
				uartSEND(R4+'0');
				uartSEND(' ');
				PLR_TIME_1=((EEPROM_read(1))*100+EEPROM_read(2));
				PLR_TIME_2=((EEPROM_read(3))*100+EEPROM_read(4));
				
				unsigned int kostil_L4 = PLR_TIME_1%10;
				PLR_TIME_1/=10;
				unsigned int kostil_L3 = PLR_TIME_1%10;
				PLR_TIME_1/=10;
				unsigned int kostil_L2 = PLR_TIME_1%10;
				PLR_TIME_1/=10;
				unsigned int kostil_L1 = PLR_TIME_1%10;
				PLR_TIME_1 = kostil_L4*1000+kostil_L3*100+kostil_L2*10+kostil_L1;
				
				unsigned int kostil_R4 = PLR_TIME_2%10;
				PLR_TIME_2/=10;
				unsigned int kostil_R3 = PLR_TIME_2%10;
				PLR_TIME_2/=10;
				unsigned int kostil_R2 = PLR_TIME_2%10;
				PLR_TIME_2/=10;
				unsigned int kostil_R1 = PLR_TIME_2%10;
				PLR_TIME_2 = kostil_R4*1000+kostil_R3*100+kostil_R2*10+kostil_R1;
				showNumber(PLR_TIME_1,PLR_TIME_2);
				 _delay_ms(300);
			}
			
			
			if ((PLR_TIME_1<16)) {PORTB |=(1<<PB4);} else {PORTB &= ~(1<<PB4);	}
			if ((PLR_TIME_2<16)) {PORTB |=(1<<PB5);} else {PORTB &= ~(1<<PB5);	}
			if ((PLR_TIME_1 <=4) | (PLR_TIME_2<=4)) {PORTB |=(1<<PB6);} else {PORTB &= ~(1<<PB6);}
				
			if (PLR_TIME_1 == 0) 
			{
				L1='-';
				L2='-';
				L3='-';
				L4='-';
			}
			
			if (PLR_TIME_2 == 0)
			{
				R1='-';
				R2='-';
				R3='-';
				R4='-';
			}
			
		PORTC = 0b00000001; segchar(L1);
		_delay_ms(1);
		
		PORTC = 0b00000010; segchar(L2); PORTA+=0b10000000;
		_delay_ms(1);
		
		PORTC = 0b00000100; segchar(L3);
		_delay_ms(1);
		
		PORTC = 0b00001000; segchar(L4);
		_delay_ms(1);
		
		PORTC = 0b00010000; segchar(R1);
		_delay_ms(1);
		
		PORTC = 0b00100000; segchar(R2); PORTA+=0b10000000;
		_delay_ms(1);
		
		PORTC = 0b01000000; segchar(R3);
		_delay_ms(1);
		
		PORTC = 0b10000000; segchar(R4);
		_delay_ms(1);
		
    }
}

