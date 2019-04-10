
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "robin_itoa.h"

static void reverse(char* str, int length) {
	int start = 0;
	int end = length -1;
	char tmp;
	while (start < end) {
		tmp = str[start];
		str[start] = str[end];
		str[end] = tmp;
		start++;
		end--;
	}
}

void robin_itoa(char* str, int len, int num, int base) {
	int i = 0;
	bool isNegative = false;

	/* Handle 0 explicitely, otherwise empty string is printed for 0 */
	if (num == 0) {
		str[i++] = '0';
		str[i] = '\0';
		return;
	}

	// In standard itoa(), negative numbers are handled only with
	// base 10. Otherwise numbers are considered unsigned.
	if (num < 0 && base == 10) {
		isNegative = true;
		num = -num;
	}

	// Process individual digits
	// Loop through, but stay less than the
	// string len
	while ( (num != 0) && ( i < len ) ) {
		int rem = num % base;
		str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
		num = num/base;
	}


	if( (i < len) && !isNegative) {
		// Append string terminator
		str[i] = '\0';
	} else if ( (i < ( len - 1) ) && isNegative) {
		// If number is negative, append '-'
		str[i++] = '-';
	} else {
		// We can't fit the number in so put
		// in the placeholder if it will fit
		if( len >= 4 ) {

			str[0] = ']';
			str[1] = '-';
			str[2] = '[';
			str[3] = '\0';
			i = 4;
		} else {
			str[0] = '\0';
			i = 1;
		}
	}

	// Reverse the string
	reverse(str, i);
}
