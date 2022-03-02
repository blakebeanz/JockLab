from machine import Pin, UART
from time import sleep

uart0 = UART(0, 9600, bits = 8, parity=None, stop=1, tx=Pin(16), rx=Pin(17))

while True:
    if uart0.any() > 0:
        data = uart0.read(10)
        print(data.decode("UTF-8"))
        

        
