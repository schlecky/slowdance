slowdance : slowdance.c
	msp430-gcc -O2 -mmcu=msp430g2553 slowdance.c -o slowdance
debug : slowdance.c
	msp430-gcc -g -mmcu=msp430g2553 slowdance.c -o slowdance
prog : slowdance
	mspdebug rf2500 "prog slowdance"
