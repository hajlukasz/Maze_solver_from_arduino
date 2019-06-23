/*
 * Maze_solver_from_arduino.c
 *
 * Created: 22.06.2019 18:09:45
 * Author : Łukasz
 */ 

#define F_CPU 16000000UL				//potrzebne narazie głównie do transmisji
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define EnA PORTB1  //prawy
#define EnB PORTB2  // lewy
#define In1 PORTD6  // prawy ?tyl
#define In2 PORTD7   // prawy przod
#define In3 PORTD4 //lewy tył
#define In4 PORTD5  //lewy przod

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

void USART_Init( unsigned int ubrr);
void USART_Transmit( unsigned char data );


void setupADC();  // rejestry do pomiaru analogowego
void setupPWM();
void pomiar();	// przekształcam pomiar analogowy do 0 i 1(jesi wykryje czarny), wedlug wczesniejszej kalibracji
void kalibracja(); // robie 10 pomiarów licze z nich srednią i wiem jaki odczyt daje podloze
void manewr_lewo();
void manewr_prawo();
void manewr_zawroc();
void prosto(int czas);
void cofaj(int czas);
void w_prawo(int czas);
void w_lewo(int czas);
void delay_ms(uint16_t count);


int pomiary [] = {0,0,0,0,0}; // indeks 0 to maks lewy, indeks 4 to maks prawy |  podłaczenie PC4, PC3, PC5, PC1, PC0
int pomiary_[] = {0,0,0,0,0}; // pomiary przeksztalcone do 0 1
int zkalibrowane [] = {1023,1023,1023,1023,1023};

double dutyCycleA = 110;		//inicjalizuje w tym miejscu zeby miec pewnosc ze przerwania beda dzialac
double dutyCycleB = 110;		//zmienna double bo z intem nie dziala


int main(void)
{
	DDRD |= 0b11110000;
	DDRB |= 0b00000011;
	//PORTB |= (1 << EnA) | (1 << EnB);
    USART_Init(MYUBRR);
	kalibracja();
	setupPWM();
	
    while (1) 
    {
		prosto(200);
		cofaj(200);
		w_prawo(200);
		w_lewo(200);
		
	
		/*
		pomiar();
		USART_Transmit(pomiary_[0]);
		USART_Transmit(pomiary_[1]);
		USART_Transmit(pomiary_[2]);
		USART_Transmit(pomiary_[3]);
		USART_Transmit(pomiary_[4]);
	
		USART_Transmit(99);
		_delay_ms(1500);
		*/
    }
}










// ponizsza funkcja dokonuje pomiaru i uzywa przetwornika na 5 kanalach
ISR(ADC_vect)
{
	uint8_t theLOW = ADCL;
	uint16_t theTenBitResult = ADCH << 8 | theLOW ;   //shifting because we need space for first 8 bits from ADCL
	//theTenBitResult = theTenBitResult /10 ;
	
	
	//we need to determinate nest channel to make conversion
	switch (ADMUX)
	{
		case 0x40:
			pomiary[4] = theTenBitResult;
			ADMUX = 0x41;
			break;
		case 0x41:
			pomiary[3] = theTenBitResult;
			ADMUX = 0x43;
			break;
		case 0x43:
			pomiary[1] = theTenBitResult;
			ADMUX = 0x44;
			break;
		case 0x44:
			pomiary[0] = theTenBitResult;
			ADMUX = 0x45;
			break;
		case 0x45:
			pomiary[2] = theTenBitResult;
			ADMUX = 0x40;
			break;
		default:
			break;
												
	}
	
	ADCSRA |= (1 << ADSC);
	
	if (ADMUX == 45)
	{
		ADCSRA |= (0 << ADSC);
	}
}
void setupADC()
{
	ADCSRA |= (1 << ADPS2);
	ADMUX |= (1 << REFS0); 
	ADCSRA |= (1 << ADIE);
	ADCSRA |= (1 << ADEN);
	sei();
	ADCSRA |= (1 << ADSC);
	
}

// potrzebene do PWM, troche prob i bledow troche filmiki troche datasheet
void setupPWM()
{
	DDRB |= (1 << DDB1)|(1 << DDB2);	// PB1 and PB2 is now an output
	ICR1 = 0xFFFF;	// set TOP to 16bit
	OCR1A = (dutyCycleA/100)*65535; // set PWM for duty cycle @ 16bit
	OCR1B = (dutyCycleB/100)*65535;
	sei();
	TCCR1A |= (1 << COM1A1)|(1 << COM1B1);	// set none-inverting mode
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12)|(1 << WGM13);	// set Fast PWM mode using ICR1 as TOP
	TIMSK1 |= (1 << TOIE1);					// its for interrupt
	TCCR1B |= (1 << CS10);	// START the timer with no prescaler
}
ISR(TIMER1_OVF_vect)
{
	OCR1A = (dutyCycleA/255)*65535;
	OCR1B = (dutyCycleB/255)*65535;
}

void pomiar()
{
	setupADC();
	for (int i = 0; i < 5; ++i)
	{
		if (pomiary[i] > zkalibrowane[i])
		{
			pomiary_[i] = 1;
		}
		else
		{
			pomiary_[i] = 0;
		}
	}
}




void kalibracja()
 {

	int roznica = 135;                //zmien jesli chcesz zeby łapał jedynke przy wiekszej zmianie
	int pomiary_kalibracja[10][5];   //tablica na potrzebne pomiary


	//petla z takimi warunnkami zeby mi nie wyszla głupota [case np jak stoi jednym czujnikiem przypadkiem na lini zamiast na podłozu]
	while(zkalibrowane[0] > 600 || zkalibrowane[1] > 600 || zkalibrowane[2] > 600 || zkalibrowane[3] > 600 || zkalibrowane[4] > 600)
	{             
		
		for (int i = 0 ; i<10 ; i++)
		{
			setupADC();
			pomiary_kalibracja[i][0] = pomiary[0];
			pomiary_kalibracja[i][1] = pomiary[1];
			pomiary_kalibracja[i][2] = pomiary[2];
			pomiary_kalibracja[i][3] = pomiary[3];
			pomiary_kalibracja[i][4] = pomiary[4];
			_delay_ms(50);
		};

		int suma[5] = {0,0,0,0,0};
		
		for (int i = 0 ; i<10 ; i++)
		{
			suma[0] = pomiary_kalibracja[i][0] + suma[0];
			suma[1] = pomiary_kalibracja[i][1] + suma[1];
			suma[2] = pomiary_kalibracja[i][2] + suma[2];
			suma[3] = pomiary_kalibracja[i][3] + suma[3];
			suma[4] = pomiary_kalibracja[i][4] + suma[4];
		}

		// nie jest w petli zeby miec dostep do poszczegolnych i je sobie modyfikowac w miare potrzeby
		zkalibrowane[0] = suma[0] / 10 + roznica;
		zkalibrowane[1] = suma[1] /10 + roznica;
		zkalibrowane[2] = suma[2] /10 + roznica - 35; //odjalem bo srodek jest kluczowy w slepych uliczkach
		zkalibrowane[3] = suma[3] /10 + roznica;
		zkalibrowane[4] = suma[4] /10 + roznica;
		
	}

	DDRB |= (1 << DDB4);
	PORTB |= (1 << PORTB4); // zapala diode sygnalizujac ze kalibracja sie udala i ruszy za 2 sek
	_delay_ms(2000);  
}

//-------------------------------------------------------------------

void prosto(int czas)
{
	
	PORTD |= (1 << In1);
	PORTD |= (1 << In3);
	
	delay_ms(czas);
	
	PORTD &= ~(1 << In1);
	PORTD &= ~(1 << In3);
}



//-------------------------------------------------------------------

void cofaj(int czas)
{

	PORTD |= (1 << In2);
	PORTD |= (1 << In4);

	delay_ms(czas);

	PORTD &= ~(1 << In2);
	PORTD &= ~(1 << In4);
}



//-------------------------------------------------------------------

void w_prawo (int czas)
{

	PORTD |= (1 << In3);
	PORTD |= (1 << In2);

	delay_ms(czas);
	
	PORTD &= ~(1 << In3);
	PORTD &= ~(1 << In2);

}


//-------------------------------------------------------------------

void w_lewo (int czas)
{
	
	PORTD |= (1 << In1);
	PORTD |= (1 << In4);

	delay_ms(czas);
	
	PORTD &= ~(1 << In1);
	PORTD &= ~(1 << In4);
}


//ze wzgledu na to ze funkcja _delay_ms nie pozwala na uzywanie zmiennej 
void delay_ms(uint16_t count) 
{
	while(count--)
	 {
		_delay_ms(1);
	}
}


void USART_Init( unsigned int ubrr)
{
	/*Set baud rate */
	/* UBRR0H contains the 4 most significant bits of the
	baud rate. UBRR0L contains the 8 least significant
	bits.*/  
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	

	/*Enable transmitter */
	UCSR0B = (1<<TXEN0);
	
	/* Set frame format: 8data */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

