#include "Uart.h"

void Uart_Base_Class::print(long n, int base)
{
	if (base == 0)
		write(n);
	else if (base == 10)
	{
		if (n < 0)
		{
			print('-');
			n = -n;
		}
		printNumber(n, 10);
	}
	else
		printNumber(n, base);
}

void Uart_Base_Class::print(unsigned long n, int base)
{
	if (base == 0)
		write(n);
	else
		printNumber(n, base);
}

void Uart_Base_Class::printNumber(unsigned long n, const uint8_t base)
{
	if (n)
	{
		unsigned char buf[8 * sizeof(long)]; // Enough space for base 2
		int8_t i = 0;
		while (n)
		{
			buf[i++] = n % base;
			n /= base;
		}
		while (i--)
			print((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
	}
	else
		print('0');
}

void Uart_Base_Class::printFloat(float number, uint8_t digits)
{
	// Handle negative numbers
	if (number < 0.0)
	{
		print('-');
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	double rounding = 0.5;
	for (uint8_t i = 0; i < digits; ++i)
		rounding *= 0.1;

	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long int_part = (unsigned long)number;
	double remainder = number - (double)int_part;
	print(int_part);

	// Print the decimal point, but only if there are digits beyond
	if (digits)
	{
		print('.');
		// Extract digits from the remainder one at a time
		while (digits--)
		{
			remainder *= 10.0;
			int toPrint = int(remainder);
			print(toPrint);
			remainder -= toPrint;
		}
	}
}
