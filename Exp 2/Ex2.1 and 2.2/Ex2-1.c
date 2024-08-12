/**********************************************************
*	Project Name: ADC Lab Experiment
*	File name: Ex2-1
*	Author:
*	Date: 12-09-23
*	Description: Example 2.1 and 2.2 of Exp 2
*
**********************************************************/

// Including files
// #include "lm4f120h5qr.h"
#include <stdio.h>

void SystemInit (void) {}
	
struct Books
	{
		char title[50];
		char author[50];
		char subject[100];
		int book_id;
	} book;
	
int main ()
	{
	struct Books Book1 ; /* Declare Book1 of type Book */
	strcpy(Book1.title, "C Programming");
	strcpy(Book1.author, "Richard C. Dorf" );
	strcpy(Book1.subject, "C Programming Tutorial");
	Book1.book_id = 6495407;
	printf("Book1 title : %s \n", Book1.title);
	printf("Book1 author : %s \n", Book1.author);
	printf("Book1 subject : %s \n", Book1.subject);
	printf("Book1 book_id : %d \n ", Book1.book_id);
	return 0;
	}
	