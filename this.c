/* 
   ZADATAK: Preko serijske veze mikrokontroler od racunara prima poruku u formatu Sx<CR><LF>. Pri
   tome x predstavlja broj u opsegu 1-4, koji odgovara analognim kanalima koji su povezani na
   potenciometre P1-P4. Nakon pristizanja poruke startuje se AD konverzija na zadatom kanalu.
   Nakon završene konverzije šalje se povratna poruka na racunar u formatu Cxxxx<CR><LF>, pri
   cemu je xxxx cetvorocifreni broj u opsegu 0-4095 i predstavlja digitalizovanu vrednost na
   selektovanom analognom kanalu. Istovremeno se digitalizovana vrednost ispisuje na
   cetvorocifreni multipleksirani LED displej. Pored toga, digitalizovana vrednost se skalira na
   opseg 0-100% i koristi se za definisanje faktora ispune PWM signala koji se šalje na LED diodu.
   Zadatom kanalu vezanom za potenciometar P1 odgovara dioda LD1, potenciometru P2 dioda
   LD2 itd.. Obezbediti da u jednom trenutku svetli samo ona dioda kojoj odgovara potenciometar
   cija je akvizicija zadata.
*/

#include <msp430.h>

/* 
   N = 1 048 576 / 2400 = 436.9
   round(N) = 436 = 0x01B4 
*/
#define BR2400_H (0x01) // Visi bajt broja N.
#define BR2400_L (0xB4) // Nizi bajt broja N.  

#define DIGIT2ASCII(x) (x + 48)  // Broj -> ASCII.
#define ASCII2DIGIT(x) (x - 48)  // ASCII -> Broj.

//const unsigned char tabelaSegmenata[] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B};

/*
void ispisiNaLED(const unsigned char index) {
    P6OUT = tabelaSegmenata[index];
}
*/

unsigned int  intenzitet     = 0;
unsigned char ledIndeks      = 0;
unsigned char podaci;
unsigned int  brojac         = 0;
unsigned int  brojKanala;
unsigned char krajKonverzije = 0;
unsigned char nizPodataka[]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char podaciIndeks;
unsigned int  ADCRezultat;
unsigned int  rezultati[4];
unsigned int  prviProlaz     = 1;

/* 
   Glavna funkcija programa. 
*/
int main() {

    WDTCTL = WDTPW | WDTHOLD;
	
//------------------------- Podesavanje dioda i LED ekrana -------------------------
    P6DIR  |= ~BIT7;            // Aktiviranje svih dioda.
    P6OUT  |= ~BIT0;            // Ispisivanje nule.
    P11DIR |= BIT1 | BIT0;      // Aktiviranje segmenata SEG1 i SEG2.
    P10DIR |= BIT7 | BIT6;      // Aktiviranje segmenata SEG3 i SEG4.
    P11OUT  = BIT1 | BIT0;      // SEG1 i SEG2 se trenutno ne koriste.
    P10OUT  = BIT7;             // SEG3 se trenutno ne koristi. SEG4 se koristi, ispisuje nulu.
//----------------------------------------------------------------------------------

//-------------------------    Inicijalizacija tajmera A   -------------------------
    TA0CCR0  = 200;             // Podesavanje tajmera na 100Hz.
    TA0CTL   = TASSEL_1 + MC_1; // Postavljanje ACLK i up mode rada.
    TA0CCTL0 = CCIE;            // Omogucavanje prekida od tajmera A.
//----------------------------------------------------------------------------------

//-------------------------    Inicijalizacija tajmera B   -------------------------
    P4DIR    = 0x78;            // Aktiviranje odgovarajucih pinova porta 4.
    P4SEL    = 0x78;            // Selektovanje alternativnih opcija pinova porta 4.
    TB0CCR0  = 100;             // PWM period na tajmeru B.
    TB0CCTL3 = OUTMOD_7;        // Set/reset nacina rada CCR3 tajmera B.
    TB0CCR3  = 10;              // Pocetna vrednost za duty cycle CCR3 PWM-a.
    TB0CCTL4 = OUTMOD_7;        // Set/reset nacina rada CCR4 tajmera B.
    TB0CCR4  = 10;              // Pocetna vrednost za duty cycle CCR4 PWM-a.
    TB0CCTL5 = OUTMOD_7;        // Set/reset nacina rada CCR5 tajmera B.
    TB0CCR5  = 10;              // Pocetna vrednost za duty cycle CCR5 PWM-a.
    TB0CCTL6 = OUTMOD_7;        // Set/reset nacina rada CCR6 tajmera B.
    TB0CCR6  = 80;              // Pocetna vrednost za duty cycle CCR6 PWM-a.
    TB0CTL   = TBSSEL_1 + MC_1; // Postavljanje ACLK i up mode rada.
//----------------------------------------------------------------------------------

//------------------------ Inicijalizacija veze sa PC-ijem   -----------------------
    P3SEL    |= BIT5 | BIT4;    // Alternativna selekcija na pinovima 5 i 6 porta 3.
    UCA0CTL1 |= UCSWRST;        // USCI se drzi pod resetom, dok se konfigurise.
    UCA0CTL1 |= UCSSEL_2;       // Postavljanje BRCLK = SMCLK.
    UCA0BR0   = BR2400_L;       // Postavljanje brzine komunikacije na 2400b/s. (1. deo)
    UCA0BR1   = BR2400_H;       // Postavljanje brzine komunikacije na 2400b/s. (2. deo)
    UCA0MCTL |= UCBRS_7;        // Postavljanje brzine komunikacije na 2400b/s. (3. deo)
    UCA0CTL1 &= ~UCSWRST;       // USCI vise nije pod resetom, jer je konfigurisan.
    UCA0IE   |= UCRXIE + UCTXIE;// Omogucavanje prekida prijemnika i predajnika.
//----------------------------------------------------------------------------------

//------------------------ Inicijalizacija ADC konverzije   ------------------------
    P7SEL |= BIT7 + BIT6;       // Alternativna selekcija na pinovima 7 i 8 porta 7.
    P5SEL |= BIT1 + BIT0;       // Alternativna selekcija na pinovima 1 i 2 porta 5.
    ADC12CTL0  = ADC12ON + ADC12MSC + ADC12SHT0_15;  // Ukljucuje se AD konvertor. 
	                                           // Omogucavanje visestruke konverzije i postavljanje sampling time-a.
    ADC12CTL1  = ADC12SHP + ADC12CONSEQ_1;     // Koriscenje sampling tajmera i postavljanje conseq nacina rada.
    ADC12MCTL0 = ADC12INCH_14;                 // Aktivacija kanala 14.
    ADC12MCTL1 = ADC12INCH_15;                 // Aktivacija kanala 15.
    ADC12MCTL2 = ADC12INCH_8;                  // Aktivacija kanala 8.
    ADC12MCTL3 = ADC12INCH_9 + ADC12EOS;       // Aktivacija kanala 9. 
	                                           // Ujedno i poslednji kanal u sekvenci.
    ADC12IE   |= ADC12IE3;                     // Omogucavanje prekida pri zavrsetku konverzije na kanalu 9.
    ADC12CTL0 |= ADC12ENC;                     // Omogucavanje prekida od AD konvertor.
//----------------------------------------------------------------------------------

    __bis_SR_register(GIE);    // Omogucavanje prekida na globalnom nivou.

    while(1) {                 // Glavna petlja programa.
        if(krajKonverzije) {
            nizPodataka[0] = 0x00;
            dohvRezultat(brojKanala);
            nizPodataka[1] = 'C';
            nizPodataka[2] = ADCRezultat/1000;
            nizPodataka[3] = (ADCRezultat%1000)/100;
            nizPodataka[4] = (ADCRezultat%100)/10;
            nizPodataka[5] = ADCRezultat%10;
            nizPodataka[6] = 0x0A;
            nizPodataka[7] = 0x0D;
            podaciIndeks = 1;
            prviProlaz = 0;
            UCA0TXBUF = nizPodataka[0];
            ukljuciOdgovarajucuDiodu(brojKanala);
            krajKonverzije = 0;
        }
    }
}

/* 
   Telo prekidne rutine tajmera A. 
   Multipleksiranje ispisa na LED ekranima SEG1-4, u zavisnosti od vrednosti ledIndeks promenjive.
*/
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A(void) {
    ledIndeks = (ledIndeks + 1) % 4;
	P11OUT = 0x03;
    P10OUT = 0xC0;
    switch(ledIndeks) {
        case 0:
            ispisiNaLED(nizPodataka[2]);
            P11OUT = BIT0;
            break;
        case 1:
            ispisiNaLED(nizPodataka[3]);
            P11OUT = BIT1;
            break;
        case 2:
            ispisiNaLED(nizPodataka[4]);
            P10OUT = BIT6;
            break;
        case 3:
            ispisiNaLED(nizPodataka[5]);
            P10OUT = BIT7;
            break;
    }
}

/* 
   Telo prekidne rutine od serijske veze sa PC-ijem. 
   Prihvatanje poruke unete od strane korisnika u formatu Sx<CR><LN>. 
   Slanje poruke na terminal PC-ija u formatu Sxxxx<CR><LN>.
   NAPOMENA: Prilikom prihvatanje poruke unete od strane korisnika u formatu Sx<CR><LN>. Nece biti prihvacen <LN> deo poruke.
*/
#pragma vector = USCI_A0_VECTOR
__interrupt void usciA0handler(void){
    switch(UCA0IV) {
        case 0:                                            // Vector 0: No interrupt.
		break;
        case 2:                                            // Vector 2: RXIFG.
            podaci = UCA0RXBUF;
            switch(brojac) {
                case 0:
                    if(podaci == 'S')                      // Citanje prvog dela poruke, 'S'.
                        ++brojac;
                    break;
                case 1:
                    brojKanala = ASCII2DIGIT(podaci);      // Citanje drugog dela poruke, x.
                    if(brojKanala >= 1 && brojKanala <= 4)
                        ++brojac;
                    else
                        brojac = 0;
                    break;
                case 2:
                    if(podaci == 0x0D)                     // Citanje treceg dela poruke <CR>.
                        ADC12CTL0 |= ADC12SC;              // Zapocinjanje konverzije.
                    brojac = 0;
                    break;
            }
        break;
        case 4:                                            // Vector 4: TXIFG.
            if(!prviProlaz) {
                if(podaciIndeks < 8) {
                    if(podaciIndeks >= 2 && podaciIndeks <=5)
                        UCA0TXBUF = DIGIT2ASCII(nizPodataka[podaciIndeks]); // Slanje podataka na terminal PC-ija. Prve 4 cifre poruke.
                    else
                        UCA0TXBUF = nizPodataka[podaciIndeks];              // Slanje podataka na terminal PC-ija. Ostatak poruke <CR> i <LN>.
                    ++podaciIndeks;
                }
                else {
                    // Zavrseno slanje podataka.
                }
            }
	    break;
        default:                                              // **** GRESKA: Doslo je do problema! ****
		break;
    }
}

/* 
   Telo prekidne rutine od AD konvertora. 
   Multipleksiranje ispisa na LED ekranima SEG1-4, u zavisnosti od vrednosti ledIndeks promenjive.
*/
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
    switch (ADC12IV) {
          case  0: break;                           // Vector 0:  No interrupt.
          case  2: break;                           // Vector 2:  ADC overflow.
          case  4: break;                           // Vector 4:  ADC timing overflow.
          case  6: break;                           // Vector 6:  ADC12IFG0.
          case  8: break;                           // Vector 8:  ADC12IFG1.
          case 10: break;                           // Vector 10: ADC12IFG2.
          case 12:                                  // Vector 12: ADC12IFG3.
            rezultati[0] = ADC12MEM0;                 // Dohvati rezultat. IFG je obrisan.
            rezultati[1] = ADC12MEM1;                 // Dohvati rezultat. IFG je obrisan.
            rezultati[2] = ADC12MEM2;                 // Dohvati rezultat. IFG je obrisan.
            rezultati[3] = ADC12MEM3;                 // Dohvati rezultat. IFG je obrisan.
          case 14: break;                           // Vector 14: ADC12IFG4.
          case 16: break;                           // Vector 16: ADC12IFG5.
          case 18: break;                           // Vector 18: ADC12IFG6.
          case 20: break;                           // Vector 20: ADC12IFG7.
          case 22: break;                           // Vector 22: ADC12IFG8.
          case 24: break;                           // Vector 24: ADC12IFG9.
          case 26: break;                           // Vector 26: ADC12IFG10.
          case 28: break;                           // Vector 28: ADC12IFG11.
          case 30: break;                           // Vector 30: ADC12IFG12.
          case 32: break;                           // Vector 32: ADC12IFG13.
          case 34: break;                           // Vector 34: ADC12IFG14.
          default: break;
    }
    ADC12IFG = 0x0000; //ima 16 bita
    krajKonverzije = 1;
}

/*
   Dohvatanje rezultata konverzije sa kanala odredjenog unosom korisnika.
   Odredjivanje intenziteta LED diode na osnovu rezultata konverzije.
*/
void dohvRezultat(int brojKanala) {
    ADCRezultat = rezultati[brojKanala - 1];
    intenzitet = (int)((double)(ADCRezultat)/4095.0*100.0);
}

/*
   Ukljucivanje odgovarajuce diode na osnovu unosa korisnika.
*/
void ukljuciOdgovarajucuDiodu(unsigned int brojDiode) {
    TB0CCR3 = 0;
    TB0CCR4 = 0;
    TB0CCR5 = 0;
    TB0CCR6 = 0;
    switch(brojDiode) {
        case 1:
            TB0CCR3 = intenzitet;
            break;
        case 2:
            TB0CCR4 = intenzitet;
            break;
        case 3:
            TB0CCR5 = intenzitet;
            break;
        case 4:
            TB0CCR6 = intenzitet;
            break;
        
    }
}
