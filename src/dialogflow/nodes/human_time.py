# endcoding: utf8

from __future__ import print_function
import random

def printPL(h, m, exact = False):
	mian = ["pierwsza", "druga", "trzecia", "czwarta", "piąta", "szósta",
	        "siódma", "ósma", "dziewiąta", "dziesiąta", "jedenasta", "dwunasta",
	        "trzynasta", "czternasta", "piętnasta", "szesnasta", "siedemnasta", "osiemnasta",
	        "dziewiętnasta", "dwudziesta", "dwudziesta pierwsza", "dwudziesta druga", "dwudziesta trzecia", "dwudziesta czwarta"]
	        
	miej = ["pierwszej", "drugiej", "trzeciej", "czwartej", "piątej", "szóstej",
	        "siódmej", "ósmej", "dziewiątej", "dziesiątej", "jedenastej", "dwunastej",
	        "trzynastej", "czternastej", "piętnastej", "szesnastej", "siedemnastej", "osiemnastej",
	        "dziewiętnastej", "dwudziestej", "dwudziestej pierwszej", "dwudziestej drugiej", "dwudziestej trzeciej", "dwudziestej czwartej"]
	        
	liczby = ["zero zero", "jedna", "dwie", "trzy", "cztery", "pięć", "sześć", "siedem", "osiem", "dziewięć", "dziesięć",
	          "jedenaście", "dwanaście", "trzynaście", "czternaście", "piętnaście", "szesnaście", "siedemnaście", "osiemnaście", "dziewiętnaście", "dwadzieścia",
	          "dwadzieścia jeden", "dwadzieścia dwie", "dwadzieścia trzy", "dwadzieścia cztery", "dwadzieścia pięć", "dwadzieścia sześć", "dwadzieścia siedem", "dwadzieścia osiem", "dwadzieścia dziewięć", "trzydzieści",
	          "31", "32", "33", "34", "35", "36", "37", "38", "39", "40"]
	        
	
	        
	if (exact):
		pass
	else:
		godz_mian = mian[h-1]
		godz_miej = miej[h-1]
		godz_mian_next = mian[h]
		if (h == 0):
			godz_mian = "północ"
			godz_miej = "północy"
		
		if (h == 12):
			godz_mian = "południe"
			godz_miej = "południu"
		
		if (m < 2):
			print("Jest", godz_mian)
		elif (m < 5):
			print("Właśnie minęła", godz_mian)
		elif (m < 14):
			print("Jest", liczby[m], "po", godz_miej)
		elif (m<17):
			print("Kwadrans po", godz_miej);
		elif (m < 29):
			print("Jest", godz_mian, liczby[m])
		elif (m < 32):
			print("Jest w pół do", godz_miej)
		elif (m < 39):
			print("Jest", godz_mian, liczby[m])
		elif (m < 43):
			print("Jest za dwadzieścia", godz_mian_next)
		elif (m < 47):
			print("Za kwadrans", godz_mian_next)
		elif (m < 56):
			print("Za", liczby[60-m], godz_mian_next)
		elif (m < 59):
			print("Dochodzi", godz_mian_next)
		else:
			print("Jest", godz_mian_next)
	
	
for i in range(100):
	h = random.randint(0, 23);
	m = random.randint(0, 59);
	print(h, m)
	printPL(h, m)
  
