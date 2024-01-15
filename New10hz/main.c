/*
 * New10hz.c
 *
 * Created: 2024-01-06 15:57:03
 * Author : bermerlin
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uart.h"

// SPI protocol address bits 
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT			(1<<6)

// LSM6DSO accelerometer
#define CTRL1_XL		0x10
#define CTRL2_G			0x11
#define CTRL3_C			0x12
#define OUTX_L_A		0x28
#define STATUS_REG_A    0x1E

// LIS3MDL magnetometer
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define CTRL_REG5		0x24
#define STATUS_REG		0x27
#define OUT_X_L			0x28
#define ZYXDA              3

#define M_PI_F 3.14159265
#define RAD_TO_DEG ((double) (180.0/M_PI_F))

uint8_t i, reg;
uint8_t mode;  //  1:accelerometer calibration  2:magnetometer calibration  4:nmea0183 
uint8_t ncalib;
double ax, ay, az;
double mx, my, mz;

volatile uint8_t iflag;

// Buffer for accelerometer and magnetometer readings.
int16_t reads[3];
int16_t readsm[3];

int32_t axreads;
int32_t ayreads;
int32_t azreads;

double xc, yc, y_ax_ay;

// Magnetometer calibration data
double m_xBias, m_sens_x[3];
double m_yBias, m_sens_y[3];
double m_zBias, m_sens_z[3];

// Accelerometer calibration data
double a_xBias, sens_x[3];
double a_yBias, sens_y[3];
double a_zBias, sens_z[3];

// Heel and pitch angles
double rho, phi;

static char buf[500];
uint8_t	click;

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

unsigned char kbhit(void)
{
	// return nonzero if char waiting
	unsigned char b;
	b = 0;
	if(UCSR0A & (1<<RXC0))
		b = 1;
	return b;
}

void USART_Flush( void )
{
	unsigned char dummy;
	while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;
}

static int uart_putchar0(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}

/*
 * This interrupt routine is called when the DRDY line
 * of the LSM303D goes high
 */
ISR(INT0_vect)
{
   iflag = 1;
}

void SPI_MasterInit(void)
{
	// SS(PB2), MOSI(PB3), MISO(PB4), SCK(PB5)

	// Enable pull-up on PB2(SS)
	PORTB = _BV(PB2);
	// Set PB2(SS) as output high, PB3(MOSI) and PB5(SCK) as output low
	DDRB = _BV(DDB2) | _BV(DDB3) |_BV(DDB5);
	
	// Enable pull-up on PD2 (INT0) ans PD4 (SS_mag)
	PORTD = _BV(PD2) | _BV(PD4);
	
	// Set PD4(SS_mag) as output high
	DDRD = _BV(DDD4);
	
	// Enable SPI, Master, set clock rate fck/16
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); // 460.8 kHz

	uint8_t i;

	for(i = 1; i < 100; i++)  // 1 s delay
		_delay_loop_2(20000);
}

uint8_t read_reg(uint8_t Address)
{
	uint8_t result = 0;
	PORTB &= ~(_BV(PB2)); // select LSM6DSO
	SPDR = DIR_READ | Address;
	while(!(SPSR & _BV(SPIF)));
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	result = SPDR;
	PORTB |= _BV(PB2); // deselect LSM6DSO
	return(result);
}

void write_reg(uint8_t Address, uint8_t Value)
{
	PORTB &= ~(_BV(PB2)); // select LSM6DSO
	SPDR = DIR_WRITE | Address;
	while(!(SPSR & _BV(SPIF)));
	SPDR = Value;
	while(!(SPSR & _BV(SPIF)));
	PORTB |= _BV(PB2); // deselect LSM6DSO
}

uint8_t mag_read_reg(uint8_t Address)
{
	uint8_t result = 0;
	PORTD &= ~(_BV(PD4)); // select LIS3MDL
	SPDR = DIR_READ | Address;
	while(!(SPSR & _BV(SPIF)));
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	result = SPDR;
	PORTD |= _BV(PD4); // deselect LIS3MDL
	return(result);
}

void mag_write_reg(uint8_t Address, uint8_t Value)
{
	PORTD &= ~(_BV(PD4)); // select LIS3MDL
	SPDR = DIR_WRITE | Address;
	while(!(SPSR & _BV(SPIF)));
	SPDR = Value;
	while(!(SPSR & _BV(SPIF)));
	PORTD |= _BV(PD4); // deselect LIS3MDL
}

void acc_enableDefault(void)
{
	// Accelerometer

	// 0x52 = 0b01010010
	// ODR = 0101 (208 Hz (high performance)); FS_XL = 00 (+/-2 g full scale); LPF2_XL_EN = 1 (output from LPF2 second filtering stage selected)
	write_reg(CTRL1_XL, 0x52);

	// Gyro

	// 0x00 = 0b000000000
	// ODR = 0000 (Power-Down)
	write_reg(CTRL2_G, 0x00);

	// Common

	// 0x04 = 0b00000100
	// IF_INC = 1 (automatically increment register address)
	write_reg(CTRL3_C, 0x04);
}

/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void mag_enableDefault(void)
{
	// 0x70 = 0b01110000
	// OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
	mag_write_reg(CTRL_REG1, 0x70);

	// 0x00 = 0b00000000
	// FS = 00 (+/- 4 gauss full scale)
	mag_write_reg(CTRL_REG2, 0x00);

	// 0x00 = 0b00000000
	// MD = 00 (continuous-conversion mode)
	mag_write_reg(CTRL_REG3, 0x00);

	// 0x0C = 0b00001100
	// OMZ = 11 (ultra-high-performance mode for Z)
	mag_write_reg(CTRL_REG4, 0x0C);

	// 0x40 = 0b01000000
	// BDU = 1 (block data update)
	mag_write_reg(CTRL_REG5, 0x40);
}

void ReadAccelerometer(void)
{
	uint8_t axl, axh, ayl, ayh, azl, azh;
	
	PORTB &= ~(_BV(PB2)); // select LSM6DSO

	SPDR = DIR_READ | OUTX_L_A;   // ADDR_INCREMENT already set in acc_enableDefault function
	while(!(SPSR & _BV(SPIF)));
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	axl = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	axh = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	ayl = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	ayh = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	azl = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	azh = SPDR;
	
	PORTB |= _BV(PB2); // deselect LSM6DSO
	
	reads[0] = (int16_t)(axh << 8 | axl);
	reads[1] = (int16_t)(ayh << 8 | ayl);
	reads[2] = (int16_t)(azh << 8 | azl);
}

void ReadMagnetometer(void)
{
	uint8_t mxl, mxh, myl, myh, mzl, mzh;
	
	PORTD &= ~(_BV(PD4)); // select LIS3MDL
	SPDR = DIR_READ | ADDR_INCREMENT | OUT_X_L;
		
	while(!(SPSR & _BV(SPIF)));
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	mxl = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	mxh = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	myl = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	myh = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	mzl = SPDR;
	SPDR = 0x00;
	while(!(SPSR & _BV(SPIF)));
	mzh = SPDR;
	
	PORTD |= _BV(PD4); // deselect LIS3MDL

	readsm[0] = (int16_t)(mxh << 8 | mxl);
	readsm[1] = (int16_t)(myh << 8 | myl);
	readsm[2] = (int16_t)(mzh << 8 | mzl);
}


int main(void)
{
    char buffer[64];
	mode = 4;
	ncalib = 0;
	click = 0;
	
	stdout = stdin = &uart_str;
	stderr = &uart_str;
	
	/* Set baud rate : 57600 bps @ 8.0 MHz */   // Note : results to 115200 (to be verified)
	UBRR0L = (unsigned char)(8);
	/* Enable transmitter */
	UCSR0B = _BV(TXEN0) | _BV(RXEN0);
	
	SPI_MasterInit();
	
	ncalib = 0;
	
	// enable external interrupt on DRDY line  (rising edge)
	EICRA = _BV(ISC00) | _BV(ISC01);
	EIMSK |= _BV(INT0);  // enable interrupt on DRDY line

	sei();
	
	a_xBias = 157.409051;
	a_yBias = -193.594777;
	a_zBias = -114.775853;

	sens_x[0] = 1.021264;
	sens_x[1] = -0.009813;
	sens_x[2] = -0.012086;
	
	sens_y[0] = -0.009813;
	sens_y[1] = 0.970292;
	sens_y[2] = -0.008260;
	
	sens_z[0] = -0.012086;
	sens_z[1] = -0.008260;
	sens_z[2] = 0.978060;

	m_xBias = -3410.055352;
	m_yBias = -1639.537992;
	m_zBias = -1077.670039;

	m_sens_x[0] = 1.097440;
	m_sens_x[1] = 0.004222;
	m_sens_x[2] = -0.011218;
	
	m_sens_y[0] = 0.004222;
	m_sens_y[1] = 1.129547;
	m_sens_y[2] = -0.010282;
	
	m_sens_z[0] = -0.011218;
	m_sens_z[1] = -0.010282;
	m_sens_z[2] = 1.097048;
	
	acc_enableDefault();
	mag_enableDefault();
	
	for(i = 1; i < 100; i++)  // 1 s delay
		_delay_loop_2(20000);
		
	 while(!(mag_read_reg(STATUS_REG) & _BV(ZYXDA)));
	 ReadMagnetometer();
	
	uint16_t n_acc = 0;
	axreads = 0;
	ayreads = 0;
	azreads = 0;
	
    while (1) 
    {
		if(kbhit())     // data are available from USB-to-serial
		{
			/* List of valid commands received
			
			M1<ENTER> : put in accelerometer calibration mode
			M2<ENTER> : put in magnetometer calibration mode
			M4<ENTER> : put in NMEA mode (default)
				
			x<ENTER> : print set of raw values when in mode 1 or 2
			*/
		
			if(fgets(buf, sizeof buf - 1, stdin) == NULL)
			{
				USART_Flush();
				continue;
			}
			else
			{
				USART_Flush();
				printf("%s\r\n", buf);
			}

			if(strstr(buf, "x") != NULL)
			{
				click = 1;
			}
			else if(strstr(buf, "M1") != NULL)
			{
				mode = 1;
			}
			else if(strstr(buf, "M2") != NULL)
			{
				mode = 2;
			}
			else if(strstr(buf, "M4") != NULL)
			{
				mode = 4;
			}
		}
		
		if(iflag  > 0)
		{
			ReadMagnetometer();
		 
			if(mode == 2)
			{
				if(click == 1)
				{
					if(ncalib < 16)
					{
						sprintf(buffer, "%i %i %i", readsm[0], readsm[1], readsm[2]);
						for(i = 0; i < strlen(buffer); i++)
							uart_putchar0((unsigned char)(buffer[i]));
						uart_putchar0('\r');
						uart_putchar0('\n');
						ncalib++;
					}
					else    // ncalib = 16
					{
						ncalib = 0;
						click = 0;
					}
				}
			}
		 
			double mrx = readsm[0] - m_xBias;
			double mry = readsm[1] - m_yBias;
			double mrz = readsm[2] - m_zBias;
			
			mx = m_sens_x[0] * mrx + m_sens_x[1] * mry + m_sens_x[2] * mrz;
			my = m_sens_y[0] * mrx + m_sens_y[1] * mry + m_sens_y[2] * mrz;
			mz = m_sens_z[0] * mrx + m_sens_z[1] * mry + m_sens_z[2] * mrz;
					
			// calculate accelerometer average
			reads[0] = axreads / n_acc;
			reads[1] = ayreads / n_acc;
			reads[2] = azreads / n_acc;
		 		 
			double rx = reads[0] - a_xBias;
			double ry = reads[1] - a_yBias;
			double rz = reads[2] - a_zBias;
	 
		 			
			ax = sens_x[0] * rx + sens_x[1] * ry + sens_x[2] * rz;
			ay = sens_y[0] * rx + sens_y[1] * ry + sens_y[2] * rz;
			az = sens_z[0] * rx + sens_z[1] * ry + sens_z[2] * rz;
   
			// calculate heel(rho) and pitch(phi)
			rho = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
			phi = atan(ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
		 
			// normalize accelerometer readings
			double norm = sqrt(ax * ax + ay * ay + az * az);
			ax /= norm;
			ay /= norm;
			az /= norm;
		 
   			// tilt compensation
			double one_ax2 = 1.0 - ax * ax;
			y_ax_ay = my * ax * ay;
			xc = mx * one_ax2 - y_ax_ay - mz * ax * az;
			yc = my * az - mz * ay;
		 
			if(mode == 4)
			{
				double head_corr = atan2(yc, xc) * RAD_TO_DEG;
				if(head_corr < 0.0)
					head_corr += 360.0;
				// $HCHDG,55.6,0.0,E,,*1F
				sprintf(buffer, "$HCHDG,%5.1f", head_corr);
				for(i = 0; i < strlen(buffer); i++)
					uart_putchar0((unsigned char)(buffer[i]));
				uart_putchar0('\r');
				uart_putchar0('\n');
		   
				
				//$PFEC,GPatt,,-8.7,+4.8*63   (pitch angle : -8.7 deg,  heel angle: 4.8 deg)
				sprintf(buffer, "$PFEC,GPatt,,%5.1f,%5.1f", phi, rho);
				for(i = 0; i < strlen(buffer); i++)
					uart_putchar0((unsigned char)(buffer[i]));
				uart_putchar0('\r');
				uart_putchar0('\n');
				
			}
					
			iflag =  0;
			
			n_acc = 0;
			axreads = 0;
			ayreads = 0;
			azreads = 0;
		}
		
		while(!(read_reg(STATUS_REG_A) & 0x01));
		ReadAccelerometer();
	  
		if(mode == 1)
		{
			if(click == 1)
			{
				if(ncalib < 64)
				{
					sprintf(buffer, "%5i %5i %5i", reads[0], reads[1], reads[2]);
					for(i = 0; i < strlen(buffer); i++)
						uart_putchar0((unsigned char)(buffer[i]));
					uart_putchar0('\r');
					uart_putchar0('\n');
					
					ncalib++;
				}
				else    // ncalib = 64
				{
					ncalib = 0;
					click = 0;
				}
			}
		}
		
		axreads += reads[0];
		ayreads += reads[1];
		azreads += reads[2];
		 
		n_acc++;
	
    }
}

