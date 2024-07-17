#include "utils.h"

void itos(int num, char* dest){
	if (num == 0) {
		dest = "0";
		return;
	}
	char string[255];
	int stringPos = 0;
	while(num != 0){
		int rem = num % 10;
		string[stringPos] = rem + '0';
		num /= 10;
	}
	
	dest[stringPos+1] = '\0';
	int reversedPos = 0;
	while(stringPos >= 0){
		dest[reversedPos] = string[stringPos];
		reversedPos++;
		stringPos--;
	}
}
