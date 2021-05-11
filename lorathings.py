
import time
import serial
import requests

def main():
 try:
  ser = serial.Serial('/dev/ttyUSB0',115200,timeout = 3.0)
  print("conectado")

  while True:
      
   ser.reset_input_buffer()
   time.sleep(5)
   if (ser.in_waiting > 0):
    var1 = ser.readline().strip()
    #lista = line.decode().split(',')
    print(var1.decode())
    #Escritura de datos a Thingsboard
    #if len(lista) == 2:
    #enviar = requests.get("https://api.thingspeak.com/update?api_key=XXXXXXXXXXXXXXXX&field1="+str(lista[0])+"&field2="+str(lista[1]))  #cuando se quiere enviar dos o mas datos
    enviar = requests.get("https://api.thingspeak.com/update?api_key=XXXXXXXXXXXXXXX&field1="+str(var1.decode()))
    if enviar.status_code == requests.codes.ok:
     if enviar.text != '0':
      print("Datos enviados correctamente")
     else:
      print("Tiempo de espera insuficiente (>15seg)")
    else:
     print("Error en el request: ",enviar.status_code)
     #else:
     #print("La cadena recibida no contiene 2 elementos, sino:",len(lista),"elementos")

   time.sleep(10)

 except KeyboardInterrupt: #Cierra el serial cuando el usuario cierra forzosamente el proceso
  print ()
  ser.close()	

if  __name__ ==  '__main__':
 main()
