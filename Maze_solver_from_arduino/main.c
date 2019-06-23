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
void manewr_zawroc();
void manewr_lewo();
void manewr_prawo();
void decyduj();
int logika();
void delay_ms(uint16_t count);



int pomiary [] = {0,0,0,0,0}; // indeks 0 to maks lewy, indeks 4 to maks prawy |  podłaczenie PC4, PC3, PC5, PC1, PC0
int pomiary_[] = {0,0,0,0,0}; // pomiary przeksztalcone do 0 1
int zkalibrowane [] = {1023,1023,1023,1023,1023};
uint8_t mode = 0;

// regulacja predkosci silnikow, moge ja zmieniac w trakcie dzialania programu
double dutyCycleA = 150;		//inicjalizuje w tym miejscu zeby miec pewnosc ze przerwania beda dzialac
double dutyCycleB = 150;		//zmienna double bo z intem nie dziala






int main(void)
{
	DDRD |= 0b11110000;
	DDRB |= 0b00000011;
	//PORTB |= (1 << EnA) | (1 << EnB);
    USART_Init(MYUBRR);
	kalibracja();
	setupPWM();
	int i_ = 0;

    while (1)
    {
	    switch(mode){
		    case 0:

		    pomiar();

		    if ((pomiary_[1] == 0) && (pomiary_[0] == 0) && (pomiary_[2] == 1) && (pomiary_[3] == 0) && ( pomiary_[4] == 0 ))
		    {
			    prosto(30);
			    i_ =0;
		    }

		    else if ((pomiary_[1] == 0) && (pomiary_[0] == 0) && (pomiary_[2] == 0) && (pomiary_[3] == 0) && ( pomiary_[4] == 0 ))
		    {
			    i_ += 1;
			    if (i_ > 20)			//czasem randomowo srodek nie pokazuje dlatego zeby poradzic sobie z problem rozpoznawania slepej uliczki
			    {
				    cofaj(480);
				    mode = 7;       //wymyslilem cos takiego co stwierdza ze jestes w slepiej uliczce jak to sie stanie duzo razy pod rzad
				    i_ = 0;
			    }
			    prosto(30);
		    }
		    
		    else if ((pomiary_[1] == 0) && (pomiary_[0] == 0) && (pomiary_[2] == 1) && (pomiary_[3] == 1) && ( pomiary_[4] == 0 ))
		    {
			    w_prawo(30);
			    i_ =0;
		    }

		    else if ((pomiary_[1] == 0) && (pomiary_[0] == 0) && (pomiary_[2] == 0) && (pomiary_[3] == 1) && ( pomiary_[4] == 0 ))
		    {
			    w_prawo(30);
			    i_ =0;
		    }

		    else if ((pomiary_[1] == 1) && (pomiary_[0] == 0) && (pomiary_[2] == 1) && (pomiary_[3] == 0) && ( pomiary_[4] == 0 ))
		    {
			    w_lewo(30);
			    i_ =0;
		    }
		    
		    else if ((pomiary_[1] == 1) && (pomiary_[0] == 0) && (pomiary_[2] == 0) && (pomiary_[3] == 0) && ( pomiary_[4] == 0 ))
		    {
			    w_lewo(30);
			    i_ =0;
		    }

		    else
		    {
			    _delay_ms(300);
			    decyduj();
			    _delay_ms(300);
			    i_ =0;
		    }

		    break;
		    
		    case 1:
		    manewr_lewo();
		    mode = 0;
		    _delay_ms(300);
		    break;

		    case 2:
		    manewr_lewo();
		    mode = 0;
		    _delay_ms(300);
		    break;

		    case 3:
		    manewr_prawo();
		    mode = 0;
		    _delay_ms(300);
		    break;

		    case 4:
		    manewr_lewo();
		    mode = 0;
		    _delay_ms(300);
		    break;

		    case 5:
		    //jedz dalej
		    mode = 0;
		    break;

		    case 6:
		    manewr_lewo();
		    mode = 0;
		    _delay_ms(300);
		    break;

		    case 7:
		    manewr_zawroc();
		    mode = 0;
		    _delay_ms(300);
		    break;

		    case 8:
		    mode = 0;
		    while(1)
		    {
			    PORTB |= (1 << PORTB4);
			    _delay_ms(200);
			    PORTB &= ~(1 << PORTB4);
			    _delay_ms(200);
		    }
		    break;
		    
		    default:
		    break;
		    
	    }
		
/*
		pomiar();
		USART_Transmit(pomiary_[0]);
		USART_Transmit(pomiary_[1]);
		USART_Transmit(pomiary_[2]);
		USART_Transmit(pomiary_[3]);
		USART_Transmit(pomiary_[4]);

		USART_Transmit(222);
		
		USART_Transmit(pomiary[0]/10);
		USART_Transmit(pomiary[1]/10);
		USART_Transmit(pomiary[2]/10);
		USART_Transmit(pomiary[3]/10);
		USART_Transmit(pomiary[4]/10);

		_delay_ms(3333);
*/
    }
}







// ponizsze funkcje dokonuja pomiaru i uzywa przetwornika na 5 kanalach
void setupADC()
{
	ADCSRA |= (1 << ADPS2);
	ADMUX |= (1 << REFS0); 
	ADCSRA |= (1 << ADIE);
	ADCSRA |= (1 << ADEN);
	sei();
	ADCSRA |= (1 << ADSC);
	
}
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
	_delay_ms(5000);  
}


void prosto(int czas)
{
	
	PORTD |= (1 << In1);
	PORTD |= (1 << In3);
	
	delay_ms(czas);
	
	PORTD &= ~(1 << In1);
	PORTD &= ~(1 << In3);
}


void cofaj(int czas)
{

	PORTD |= (1 << In2);
	PORTD |= (1 << In4);

	delay_ms(czas);

	PORTD &= ~(1 << In2);
	PORTD &= ~(1 << In4);
}


void w_prawo (int czas)
{

	PORTD |= (1 << In3);
	PORTD |= (1 << In2);

	delay_ms(czas);
	
	PORTD &= ~(1 << In3);
	PORTD &= ~(1 << In2);

}


void w_lewo (int czas)
{
	
	PORTD |= (1 << In1);
	PORTD |= (1 << In4);

	delay_ms(czas);
	
	PORTD &= ~(1 << In1);
	PORTD &= ~(1 << In4);
}

//90 stopni lewo
void manewr_lewo(){
	
	w_lewo(300);
	delay_ms(100);
	prosto(150);
	w_lewo(200);
		
	do
	{
		pomiar();
		w_lewo(40);
	}while(logika());	// kreci sie dopoki nie usyska tego co chcemy
}

// 90 stopni w prawo
void manewr_prawo(){
	
	w_prawo(300);
	_delay_ms(100);
	prosto(150);
	w_prawo(200);
	do
	{
		pomiar();
		w_prawo(40);
	}while(logika());      // kreci sie dopoki nie usyska tego co chcemy
}

// wyrzuca 0 jesli znajdzie linie do sledzenia podczas manewru, wyrzuca 1 jesli nie widzi
int logika()
{
	if((pomiary_[3] == 0) && (pomiary_[1] == 1) && (pomiary_[0] == 0) && (pomiary_[4] == 0) && (pomiary_[2] == 0 || pomiary_[2] == 1))
	{
		return 0;
	}

	else if ((pomiary_[3] == 1) && (pomiary_[1] == 0) && (pomiary_[0] == 0) && (pomiary_[4] == 0) && (pomiary_[2] == 0 || pomiary_[2] == 1))
	{
		return 0;
	}

	else if ((pomiary_[3] == 0) && (pomiary_[1] == 0) && (pomiary_[0] == 0) && (pomiary_[4] == 0) && (pomiary_[2] == 1))
	{
		return 0;
	}

	else
	{
		return 1;
	}
}   

//zawracanie
void manewr_zawroc(){
	
	w_prawo(400);
	int j = 0;
	do
	{
		j +=1;
		if (j>60)
		{
			cofaj(250);
			j =0;
		}
		pomiar();
		w_prawo(40);
	}while(logika());

}



/*-------------------------------------decyduj-------------------------------------------
 * po zauważeniu przez robota zdarzerzenia innego niz linia, przejezdza odpowiednia droge 
 * i do zmiennej mode wpisuje w zaleznosci od napotkaniej przeszkody
 * [ 0 = zwykla linia ]
 * 1 = skrzyzowanie (linie prawo lewo i posrodku
 * 2 = T (linie prawo lewo)
 * 3 = tylko prawo
 * 4 = tylko lewo
 * 5 = prosto/prawo
 * 6 = prosto lewo
 * 7 = slepa uliczka
 * 8 = koniec trasy
 * 
 */
void decyduj()
{
	int pomiary_decyduj[11][5];
    //pomiar na ktorym sie zatrzymal - przed cofnieciem
    pomiar();
    pomiary_decyduj[10][0] = pomiary_[0];
    pomiary_decyduj[10][1] = pomiary_[1];
    pomiary_decyduj[10][2] = pomiary_[2];
    pomiary_decyduj[10][3] = pomiary_[3];
    pomiary_decyduj[10][4] = pomiary_[4];

	cofaj(190);

	for (int i =0; i<10;i++)
	{
		pomiar();
		pomiary_decyduj[i][0] = pomiary_[0];
		pomiary_decyduj[i][1] = pomiary_[1];
		pomiary_decyduj[i][2] = pomiary_[2];
		pomiary_decyduj[i][3] = pomiary_[3];
		pomiary_decyduj[i][4] = pomiary_[4];
		prosto(40);   
	}
     
	int suma[5];
		suma[0] = 0;
		suma[2] = 0;
		suma[4] = 0;

	for (int i = 0; i<11; i++)
	{
		suma[0] = suma[0] + pomiary_decyduj[i][0];
		suma[4] = suma[4] + pomiary_decyduj[i][4];
	}

	for (int i = 5; i<10; i++)
	{
		suma[2] = suma[2] + pomiary_decyduj[i][2];
	}


	if ((suma[0] >0 ) && (suma[4] >0 ) &&(suma[2] >= 2) )
	{
		if((suma[0] >= 6) &&(suma[4] >= 6) && (suma[2] >= 3) )
		{
			mode = 8; //koniec trasy 
		}
		else
		{
			mode =1; //skrzyzowanie
		}
	}

	else if((suma[0] > 0) &&(suma[4] > 0) && (suma[2] <= 1) )
	{
		mode = 2;  // T
	}

	else if((suma[0] == 0) &&(suma[4] > 0) && (suma[2] <= 1) )
	{
		mode = 3;   // tylko prawo
	}

	else if((suma[0] > 0) &&(suma[4] == 0) && (suma[2] <= 1) )
	{
		mode = 4;   //tylko lewo
	}

	else if((suma[0] == 0) &&(suma[4] > 0) && (suma[2] >= 2) )
	{
		mode = 5;     // prosto - prawo
	}

	else if((suma[0] > 0) &&(suma[4] == 0) && (suma[2] >= 2) )
	{
		mode = 6;     // prosto - lewo
	}

// z tym narazie jest taki problem ze wszystkie zera narazie sa normalna sytuacja w ktorej autko jedzie do przodu bo za czesto w srodku wywala :/
//else if((suma[0] == 0) &&(suma[4] == 0) && (suma[2] == 0) ){
//  mode = 7;     // slepa uliczka
//}

}


//ze wzgledu na to ze funkcja _delay_ms nie pozwala na uzywanie zmiennej 
void delay_ms(uint16_t count) 
{
	while(count--)
	 {
		_delay_ms(1);
	}
}

//wziete z internetu funkcje pozwalajace na przesylanie 8bitowych charow przez seriala
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

