#!/usr/bin/python
# Filename: komm_04.py
# S.R. 29.03
#AT+CGNSPWR      GNSS power control
# AT+CGNSSEQ    Define the last NMEA sentence that parsed
# AT+CGNSINF    GNSS  navigation  information parsed from NMEA sentences.
# AT+CGNSURC    GNSS  navigation,  GEO-fences and speed alarm URC report
# AT+CGNSTST    Send  data  received from GNSS to AT UART..


import serial
import time
from datetime import datetime
import types
import xml.etree.ElementTree as ET
import math
import socket
import sys
import os
#import ADS1256
#import DAC8532
##import RPi.GPIO as GPIO
#*********************************************************************************************************
#                       Konfigurationseinstellungen
# TODO: Konfigurationseinstellungen optimieren
# Server IP im WLAN
#Ports /client/Server




W_buff = [b"AT+CGNSPWR=1\r\n", b"AT+CGNSSEQ=\"RMC\"\r\n", b"AT+CGNSINF\r\n", b"AT+CGNSURC=2\r\n",b"AT+CGNSTST=1\r\n"]
data = ""
num = 0
zaehler = 0
Folge=[]
gps_status=[6]
ini_ok = False  #ist initilaisiert
de_mo = True  #debug mode
sat_rdy = False # brauchbare Daten
produktiv = True
gps_status_time =""
gps_long = "0"
gps_atti = "0"
gps_count = "0"
gps_qual = "0"
Wasserdruck ="0"
Spannung_1 ="0"
Spannung_2 = "0"
Strom_1 = "0"
Strom_2 = "0"
Laenge_start = 0
Länge_gem = 0
Breite_start = 0
Breite_gem =0

Server_IP = "" # für die TCP-Verbindung, wird aus regner-config.xml gelesen
Port = 0       # wie oben
HOST = '192.168.178.36'  # The remote host
PORT = 50007  # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
	uart = serial.Serial("com12",115200)
	#uart = serial.Serial("/dev/ttyS0", 115200)
	print ("Serial port to UART is open")
except Exception as e:
	print ("error open serial port: " + str(e) + "  to UART")
	exit()

# Python code to demonstrate
# working of radians()

# for radians
import math

# Printing radians equivalents.
print("180 / pi Degrees is equal to Radians : ", end="")
print(math.radians(180 / math.pi))

print("180 Degrees is equal to Radians : ", end="")
print(math.radians(180))

print("1 Degrees is equal to Radians : ", end="")
print(math.radians(52.316))


def tcp_initialisieren():

	s.sendall(b'TCP ist initialisiert')
	data = s.recv(1024)
	#s.close()
	print('Received', repr(data))
	#tcp_Verbindung_starten()

def initialisieren():
	#GPIO.setmode(GPIO.BCM)
	#GPIO.setup(4, GPIO.OUT)
	uart.write(W_buff[0])
	uart.flushInput()
	data =""
	num  = 0
	while uart.inWaiting() > 0:
		data = data + str(uart.read(uart.inWaiting()))

	if num < 4:  # th1e string have ok
		while num < 4:
			time.sleep(0.5)
			uart.write(W_buff[num + 1])
			num = num + 1
			print("num: ",num)

lat1 =13.12349041
lon1 = 52.31609594
lat2  =13.12355
lon2  =52.4


def haversine(lat1, lon1, lat2, lon2):

      #R = 3959.87433 # this is in miles.  For Earth radius in kilometers use 6372.8 km
    R = 6372.8  # this is in miles.  For Earth radius in kilometers use 6372.8 km
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    a = math.sin(dLat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dLon/2)**2
    c = 2*math.asin(math.sqrt(a))

    return R * c

print(haversine(lat1, lon1, lat2, lon2))





def read_ad_werte():


	try:
		#ADC = ADS1256.ADS1256()
		#DAC = DAC8532.DAC8532()
		#ADC.ADS1256_init()

		#DAC.DAC8532_Out_Voltage(0x30, 3)
		#DAC.DAC8532_Out_Voltage(0x34, 3)
		while(1):
			ADC_Value = ADC.ADS1256_GetAll()
			print ("0 ADC = %lf"%(ADC_Value[0]*5.0/0x7fffff))
			print ("1 ADC = %lf"%(ADC_Value[1]*5.0/0x7fffff))
			print ("2 ADC = %lf"%(ADC_Value[2]*5.0/0x7fffff))
			print ("3 ADC = %lf"%(ADC_Value[3]*5.0/0x7fffff))
			print ("4 ADC = %lf"%(ADC_Value[4]*5.0/0x7fffff))
			print ("5 ADC = %lf"%(ADC_Value[5]*5.0/0x7fffff))
			print ("6 ADC = %lf"%(ADC_Value[6]*5.0/0x7fffff))
			print ("7 ADC = %lf"%(ADC_Value[7]*5.0/0x7fffff))

		#temp = (ADC_Value[0]>>7)*5.0/0xffff
		#print ("DAC :",temp)
		#print ("\33[10A")
		#DAC.DAC8532_Out_Voltage(DAC8532.channel_A, temp)
		#DAC.DAC8532_Out_Voltage(DAC8532.channel_B, 3.3 - temp)

	except :
		#GPIO.cleanup()
		print ("\r\nProgram end     ")
		exit()




def read_gps_werte():

	global gps_long
	global gps_status_time
	global gps_atti
	global gps_count
	global gps_qual
	global sat_rdy
	global Laenge_start, Laenge_gem, Breite_start, Breite_gem
	data = ""
	gps_status_time=""
	uart.write(W_buff[4])
	time.sleep(1)
	#print("daten: ", uart.read(uart.inWaiting))
	while uart.inWaiting() > 0:
		data = data + str(uart.read(uart.inWaiting()))
		print("Länge:  ", len(data))
		kette = str(data)
		Folge = kette.split(",")




		if len(data) > 650:
			if str(Folge[3])=="N":

				if de_mo:
					print("Zeit:             ", Zeit_format(Folge[1]))
					#print("Länegengrad:      ", Folge[2], "  ", Folge[3])
					print("Länegengrad:      ", Pos_format(Folge[2]), "  ", Folge[3])
					print("Folge von 2: ",Folge[2])
					print("Folge von 2: /100 ", float(Folge[2])/100)
					#print("Breitengrad:      ", Folge[4], " ", Folge[5])
					print("Breitengrad:      ", Pos_format(Folge[4]), " ", Folge[5])
					print("Anzahl Satelliten:", Folge[7])
					print("Qualität         :", Folge[6])
				print("Höhe         :", Folge[9])
				print("Check         :", Folge[14])
				gps_status_time = Zeit_format(Folge[1])
				gps_long = Pos_format(Folge[2])+ " " + Folge[3]
				if Laenge_start == 0:
				    Laenge_start = Pos_format(Folge[2])
				    print("Laenge initial", Laenge_start)
				#DeltaLong = Laenge_start - Pos_format(Folge[2]   funktioniert nicht
				#print("Delta Long: ",DeltaLong)
				gps_atti = Pos_format(Folge[4]) + " " +Folge[5]
				gps_count = Folge[7]
				gps_qual = Folge[6]
				sat_rdy = True # jetzt kommen richtige Werte

def status_test_1():
	uart.write(b'AT\r\n')
	read_quittung()

def status_test_2():
	uart.write(b'AT+CPIN?\r\n')
	read_quittung()


def status_test_3():
	uart.write(b'AT+CMGF=1\r\n')  # set to text mode
	print("Textmodus")
	read_quittung()
	uart.write((b'AT+CMGS="+491702155968"\r'))

	print("Tel.-Nr. gesetzt")
	read_quittung()

	hour = 16
	min =12
	sec = 34

	#f_zeit ='{:%H:%M:%S}'.format(datetime(hour,min,sec))
	#gps_status_time= gps_status_time.rstrip("0")
	print("zeit ", gps_status_time)

	text_tm =str.encode(gps_status_time.rstrip("."))
	text_long = str.encode(gps_long)
	#gps_atti= "atti: "+ gps_atti
	text_atti = str.encode(gps_atti)
	text_qual = str.encode(gps_qual)
	text_count = str.encode(gps_count)
	print("text_tm ", text_tm)
	print("text_long ", text_long)
	print("text_atti ", text_atti)
	text2 = (text_tm)
	# text = text1 + text2
	text1 = (b'Regnerstatus="OK"  ')
	print("text1 ", text1)
	text = (b' sr')
	uart.write(text1 )
	uart.write(b'\r')
	uart.write(b'GPS-Zeit: ')
	uart.write(text_tm)
	uart.write(b'\r')
	uart.write(b'Wasserdruck: \r')
	uart.write(b'L: ')
	uart.write(text_long)
	uart.write(b'\r')
	uart.write(b'B: ')
	uart.write(text_atti)
	uart.write(b'\r')
	uart.write(b'GPS-Qualitaet: ')
	uart.write(text_qual)
	uart.write(b'\r')
	uart.write(b'Anzahl Sats: ')
	uart.write(text_count)
	# TODO: Funktion "SMS-Bildschirm-Inhalt" entwickeln
	#uart.write(text2)
	#*************************Einschalten zum Senden
	uart.write(b"\x1a\r\n")
	print("abgeschickt")
	read_quittung()

def Nachricht_bauen(Variante):
	text_tm = str.encode(gps_status_time.rstrip("."))
	if Variante ==1:
		Nachricht = b"Regnerstatus OK"

	return Nachricht

def tcp_client():
	import socket

	def client_program():
		#host = socket.gethostname()  # as both code is running on same pc
		host = "192.168.178.36"     # as both code is running on same pc
		port = 5000  # socket server port number

		client_socket = socket.socket()  # instantiate

		try:
			client_socket.connect((host, port))  # connect to the server
		except socket.timeout:
			print("Socket timeout")
		except socket.error:
			print("Socket error")
		except Exception as e:
			print(e)
			print("Unknown error")

		print("Verbindung steht\n")

		message = input(" -> ")  # take input

		while message.lower().strip() != 'bye':
			client_socket.send(message.encode())  # send message
			data = client_socket.recv(1024).decode()  # receive response

			print('Received from server: ' + data)  # show in terminal

			message = input(" -> ")  # again take input

		client_socket.close()  # close the connection

	if __name__ == '__main__':
		client_program()


def tcp_server():
	# get the hostname
	host = socket.gethostname()
	port = 5000  # initiate port no above 1024

	server_socket = socket.socket()  # get instance
	# look closely. The bind() function takes tuple as argument
	server_socket.bind((host, port))  # bind host address and port together

	# configure how many client the server can listen simultaneously
	server_socket.listen(2)
	conn, address = server_socket.accept()  # accept new connection
	print("Connection from: " + str(address))
	while True:
		# receive data stream. it won't accept data packet greater than 1024 bytes
		data = conn.recv(1024).decode()
		if not data:
			# if data is not received break
			break
		print("from connected user: " + str(data))
		data = input(' -> ')
		conn.send(data.encode())  # send data to the client

	conn.close()  # close the connection



def tcp_Transfer():
	# TODO  tcp_verbindung  sicher machen

	HOST = '192.168.178.36'  # The remote host
	PORT = 50007  # The same port as used by the server
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((HOST, PORT))
	s.sendall(b'Hello, world')
	data = s.recv(1024)
	s.close()
	print('Received', repr(data))

def tcp_Verbindung_stoppen():
	print("\nClosing socket")
	srvsock.close()
	print("\n\nBye Bye")

def tcp_senden():
	#TODO:tcp Routine senden
	print()
def tcp_empfangen():
	# TODO:tcp Routine empfangen
	print()
def read_voltage():
	# TODO:read voltage
	print()
def read_gsm_quali():
	# TODO:read gsm Qualität
	print()
def schalte_SIM():
	# TODO:SIM_einschalten
	#GPIO.output(4, GPIO.HIGH)
	time.sleep(4)
	#GPIO.output(4, GPIO.LOW)
	time.sleep(1)
	print()



def konfigurieren():
	tree = ET.parse('regner_config.xml')
	xml_root = tree.getroot()

	template_name = xml_root.find('Template').text.strip()

	root_dir_elem = xml_root.find('RootDir')
	if root_dir_elem is not None:
		print("root dir")
	else:
		print("No RootDir tag found in config.xml, running from current directory")

	Parameter_01 = xml_root.find('Para_01').text.strip()
	print(Parameter_01)

	Parameter_02 = xml_root.find('Para_02').text.strip()
	print(Parameter_02)

	Parameter_03 = xml_root.find('Para_03').text.strip()
	print(Parameter_03)

	Parameter_04 = xml_root.find('Para_04').text.strip()
	print(Parameter_04)

	Parameter_05 = xml_root.find('Para_05').text.strip()
	print(Parameter_05)

	Demo_Mode = xml_root.find('Demo_Mode').text.strip()
	if Demo_Mode == "Ja":
		# tempUnits = xml_root.find('Temp_Units').text.strip()
		print(tempUnits)
	else:
		print("Display.NoDisplay()")

	telefon = xml_root.find('Telefon').text.strip()
	if telefon != "0":
		print("Telefon", telefon)
	else:
		ON = 0
		OFF = 1


def quittung(Wert):
	print("quittung " ,Wert)


def read_sms():
	uart.write(b'AT+CMGR=1\r\n')
	time.sleep(2)

	#time.sleep(10)
	reply = uart.read(uart.inWaiting())  # Clean buf
	time.sleep(2)

	print(reply)
	Meldung = str(reply)
	if len(Meldung) == 3: quittung("SIM nicht vorhanden")
	print(isinstance(Meldung, str))
	print("Länge der Meldung", len(Meldung))
	print("gefunden: GPGSA ", Meldung.find("n$GPGSA"))
	print("gefunden:  SM ", Meldung.find("SM"))
	print("gefunden: UNREAD", Meldung.find("UNREAD"))
	if Meldung.find("UNREAD") != -1:
		quittung("neue SMS")
		Nachricht = Meldung[20:90]
		quittung (Nachricht)
		print("Nachricht ", Nachricht)
		uart.write(b'AT+CMGDA="DEL ALL"\r')  # delete all SMS
		time.sleep(2)

        #protokolliere()


def read_quittung():
	time.sleep(2)
	reply = uart.read(uart.inWaiting())  # Clean buf
	print(reply)
def checke_Nachrichten():
	# TODO:checke Nachrichten
	#vom System
	#von SMS
	read_sms()
	#von TCP
	if de_mo: print("checke Nachrichten ausgeführt")
	if de_mo: print()

def sende_Nachricht():
	# TODO:sende Nachricht
	status_test_3()
	if de_mo: print("sende Nachricht ausgeführt")
	if de_mo: print()


def Pos_format(Roh_pos):

    Ergebnis = str(Roh_pos).strip("0")
    #if de_mo: print("Ergebnis",Ergebnis)
    Ergebnis = Ergebnis.replace(".","")
    #print("Ergebnis",Ergebnis)
    LG = str(Ergebnis)[0:2]
    LG_d=str(Ergebnis)[2:11]
    LG_d_g = round(float(LG_d)/60,6)
    print("umgerechnet: ", LG_d_g)
    Ergebnis = LG+ "."+LG_d
    #if de_mo: print("Ergebnis",Ergebnis)
    return Ergebnis

def Zeit_format(Roh_zeit):

    Stunde = str(Roh_zeit)[0:2]
    int_Stunde = int(Stunde)
    #print("Stunde",Stunde)
    int_Stunde += 2
    Stunde = str(int_Stunde)
    Minute = str(Roh_zeit)[2:4]
    #print("Minute",Minute)
    Sekunde = str(Roh_zeit)[4:6]
    #print("Sekunde",Sekunde)
    Zeit = (Stunde+ ":" +Minute+ ":"+ Sekunde)
    return Zeit

def logdata (Aktion, Parameter1,Parameter2):
	f = open("regner"  + ".csv", "ab")
	#f.write("%3.1f;%3.3f;%3.3f\n" % (getdata(), temp, heat))
	print("Laenge: ", gps_long)
	print("Breite: ", gps_atti)
	print("Zeit: ", gps_status_time)
	l_data = gps_status_time + ","+ gps_atti +","+gps_long + "\n"
	l_data= (str.encode(l_data))
	print(l_data)
	f.write(l_data)
	l_data = gps_status_time + "," + Wasserdruck + "," + Spannung_1 + ","+ Strom_1+ ","+ Spannung_2+ "," + Strom_2 + "\n"
	l_data = (str.encode(l_data))
	f.write(l_data)
	f.close()


def working():
	# TODO:tcp Routine senden
	global produktiv
	tmp=0
	if de_mo: print("jetzt wird beregnet")

	while produktiv:
		checke_Nachrichten()
		read_gps_werte()
		logdata("zeit","1",tmp)
		#sende_Nachricht()

		#s.sendall(b'Hello, world')
		#s.sendall(Nachricht_bauen(1))
		#data = s.recv(1024)
		# s.close()
		#print('Received', repr(data))

		time.sleep(5)
		tmp += 1
		if tmp > 6: produktiv = False


#*******************************************************Ablaufsteuerung*******************************

#s.connect((HOST, PORT))

wartezeit = 0
konfigurieren()
status_test_1()
status_test_2()


#tcp_Verbindung_stoppen()
#tcp_initialisieren()
initialisieren ()
#schalte_SIM()
#read_ad_werte()
while not sat_rdy:
	print ("warten auf GPS seit", wartezeit, " s")
	time.sleep(5)
	wartezeit += 5
	read_gps_werte()
ini_ok = True
status_test_1()
status_test_2()

#read_unread_sms()

print("GPS-Zeit ",gps_status_time)
working()



#uart.close()

#status_test_3()
#abschicken()


