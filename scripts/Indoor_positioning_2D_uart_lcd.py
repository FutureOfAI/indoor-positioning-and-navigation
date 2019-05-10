import time
import serial
import Adafruit_CharLCD as LCD
import binascii

# Raspberry Pi pin configuration:
lcd_rs        = 6  # Note this might need to be changed to 21 for older revision Pi's.
lcd_en        = 22
lcd_d4        = 25
lcd_d5        = 24
lcd_d6        = 23
lcd_d7        = 18

# Alternatively specify a 16x2 LCD.
lcd_columns = 16
lcd_rows    = 2

# Initialize the LCD using the pins above.
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                           lcd_columns, lcd_rows)

distance=[0]*10000
ser = serial.Serial("/dev/ttyAMA0", 57600,bytesize = 8,parity = 'N',stopbits = 1)
if ser.isOpen == False:
    ser.open()
n = 0
A10 = 0
A11 = 0
A12 = 0
A13 = 0
x = 0
y = 0
sum = 0

lcd.message("test!")

try:
    while True:
        # ser.write("DistanceOutON"+"\r"+"\n")
        ser.write("PostionOutON"+"\r"+"\n")
        size = ser.inWaiting()
        if size != 0:
            response = ser.read(size)
            # print binascii.hexlify(response)
            x = int(binascii.hexlify(response[1]),16)*256+int(binascii.hexlify(response[2]),16)
            y = int(binascii.hexlify(response[3]),16)*256+int(binascii.hexlify(response[4]),16)

            x = float(x)/1024*100
            y = float(y)/1024*100
            # print x, y
            # lcd.message("x:" + str(round(x,1)) + "cm; x:" + str(round(y,1)) + "cm")
            lcd.message("test!")
            # A10 = int(binascii.hexlify(response[13]),16)*256+int(binascii.hexlify(response[14]),16)
            # A11 = int(binascii.hexlify(response[15]),16)*256+int(binascii.hexlify(response[16]),16)
            # A12 = int(binascii.hexlify(response[17]),16)*256+int(binascii.hexlify(response[18]),16)
            # A13 = int(binascii.hexlify(response[19]),16)*256+int(binascii.hexlify(response[20]),16)

            # A10 = float(A10)/1024*100
            # A11 = float(A11)/1024*100
            # A12 = float(A12)/1024*100
            # A13 = float(A13)/1024*100
            # print A10, A11, A12, A13

            # distance[n]=A10
            # sum=sum+distance[n]
            # if n==39:
            #     # print sum/40
            #     sum=0
            #     n=0
            # ser.flushInput()
            # n=n+1
        time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()
