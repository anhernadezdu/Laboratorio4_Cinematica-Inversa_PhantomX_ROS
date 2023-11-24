# Laboratorio4_Cinematica-Inversa_PhantomX_ROS

## Solucion plateada

Se realizaron las mediciones pertinentes de los eslabones del phanton, una vez con estas medidas se saco el modelo de cinematica inversa del robot usando el modelo geometrico con desacople de muñeca

## Rutinas de trabajo
Se crean 5 rutinas de trabajo en total, las cuales consisten en dibujar ciertas figuras y coger o devolver el marcador al porta-herramientas. Estas rutinas se encuentran en las siguientes funciones dentro del codigo [Interfaz](lab5Interfazv2.py). La primera rutina es _home()_ el cual lleva el robot a la posicion home la cuales todas las articualciones en 0 cero, excepto la articulación 3 que está a -90°(-pi/2). Esta funcion se encuentra al final de cada funcion mostrada a continuación para que el robot ejecute cierta acción y vuelva a una posición donde pueda seguir otra nueva trayectoria.

Despues esta la función _herramienta()_, la cual dependiendo del argumento ingresado 1 ó 0, procede a agarrar o dejar la herramienta, respectivamente. Esta usa 3 puntos en su trayectoria para poder agarrar el marcador de tal forma que la pinza este a 90°, paralela al tablero. Esto ultimo se logra cambiando el valor de la articulacion 4 en cada uno de los puntos 2 y 3 antes de usar el publisher y mover el robot. Adicionalmente, se usa una variable global de tal modo que una vez se agarra el marcador o se deja, este estado se mantiene hasta que se ejecute la accion contraria,usando los botones de la interfaz grafica. 

```
def herramienta(coger):
    global estado
    punto1=IrAPunto(0.2,-20,10)
    enviarAngulos(punto1)
    punto3=IrAPunto(-1.99,-24,9)
    punto3[3]=-(pi/2+punto3[1]+punto3[2])
    enviarAngulos(punto3)
    punto2=IrAPunto(-1.99,-24,3.2)
    punto2[3]=-(pi/2+punto2[1]+punto2[2])
    #print(punto2)
    enviarAngulos(punto2)
    #holm=input("presione prueba coger: ")
    if coger==1:
        estado=agarrar
    else:
        estado=suelto
        #punto2=IrAPunto(-1.98,-22.2,2.2)
    punto3[4]=estado
    punto2[4]=estado
    punto1[4]=estado
    #time.sleep(0.15)
    enviarAngulos(punto2)
    #time.sleep(0.15)
    enviarAngulos(punto3)
    #time.sleep(0.15)
    #print(punto1)
    enviarAngulos(punto1)
    #time.sleep(0.15)
    home()
```
Luego están las funciones _circuloInt()_ y _circuloExt()_ que definen el espacio de trabajo del robot, estos circulos se definieron antes usando la cinematica inversa siendo los puntos cercanos los limites establecidos para que no halla choques entre articulaciones, ni entre estas y el tablero o el porta-herramientas. Las funciones envian los puntos a dibujar de circulos con radios concentricos de aproximadamente 14 y 25 cm; sin embargo, dado que el ciruclo externo posee un radio mas grande que el del tablero, este se modificó para que solo dibujara una parte del circulo que se viera en el tablero.
```
def circuloInt():
    punto2=IrAPunto(0,-14,2)
    enviarAngulos(punto2)
    #time.sleep(0.5)
    punto1=IrAPunto(0,-14,1.5)
    enviarAngulos(punto1)
    #time.sleep(0.5)
    punto3=IrAPunto(0,14,1.5)
    enviarAngulos(punto3)
    #time.sleep(0.5)
    punto4=IrAPunto(0,12,3)
    enviarAngulos(punto4)
    #time.sleep(0.5)
    home()

def circuloExt():
    punto3=IrAPunto(25*mth.cos(70*pi/180),-25*mth.sin(70*pi/180),3)
    enviarAngulos(punto3)
    time.sleep(0.5)
    punto1=IrAPunto(25*mth.cos(70*pi/180),-25*mth.sin(70*pi/180),2.83)
    enviarAngulos(punto1)
    #time.sleep(0.5)
    punto2=IrAPunto(25*mth.cos(70*pi/180),25*mth.sin(70*pi/180),2.84)
    enviarAngulos(punto2)
    #time.sleep(0.5)
    punto4=IrAPunto(24*mth.cos(70*pi/180),24*mth.sin(70*pi/180),6)
    enviarAngulos(punto4)
    #time.sleep(0.5)
    home()
```
Finalmente estan las funciones _AS()_ y _empanada()_ las cuales son un compendio de puntos los cuales al unirse forman la trayectoria la cual el marcador traza, dibujando asi las iniciales de los integrandes del grupo (A y S) y dibuja la figura libre elegida la cual fue una empanada. Para estas trayectorias, se tuvo que modificar los valores en z de cada punto debido a que, tanto el espacio de trabajo como la forma en la que el publisher del phantom dibujaba la trayectoria punto a punto, tenian imperfecciones que desfiguraban los trazos. A lo anterior tambien se suma el error en la pinza debido a una mala colocacion del marcador.
```
def AS():
    #puntos de A
    ...
    punto6aux=IrAPunto(12, 9.12, 3)
    punto7=IrAPunto(11.8, 14.56, 5)
    #puntos de S
    ...
    punto14=IrAPunto(21.33, 8.3, 1.92)
    punto15=IrAPunto(20.92, 9.12, 5)
    #dibujar A
    ...
    enviarAngulos(punto3)
    enviarAngulos(punto7)
    #dibujar S
    ...
    enviarAngulos(punto14)
    enviarAngulos(punto15)
    home()

def empanada():
    punto1=IrAPunto(14.56, -2.43, 3)
    punto2=IrAPunto(14.56, -2.43, 1.4)
    ...
    enviarAngulos(punto2)
    enviarAngulos(punto1)
    home()
```

## Interfaz HMI
Se muestra el video con la interfaz en el siguiente link:
https://drive.google.com/file/d/1HARtszz2kdGGHv8oBotCvTdXesUnPbDf/view?usp=sharing


## Portaherramienta y marcador
Para poder realizar el presente laboratorio, se diseñó el siguiente portaherramientas el cual habría de sostener el marcador borrable que se usó durante la practica. Este marcador se uso junto con un anillo de plastilina el cual ayudaba a evitar el movimiento del marcador en direcciones indeseadas, fruto de la friccion con el tablero. Cabe destacar que el marcador siempre 

![68c2fc1c-5974-4199-93bf-b2abe766fb57](https://github.com/anhernadezdu/Laboratorio5_Cinematica-Inversa_PhantomX_ROS/assets/70985250/4933d62c-1221-410f-9ec4-9723f49d0091)
