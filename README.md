# Laboratorio4_Cinematica-Inversa_PhantomX_ROS

## Solucion plateada

Se realizaron las mediciones pertinentes de los eslabones del phanton, una vez con estas medidas se sacó el modelo de cinematica inversa del robot usando el modelo geometrico con desacople de muñeca. Las medidas L0 a L5 corresponden a las distancias entre: base-articulación1, art1-art2, art2-art3, art3-art4, art4-centro de pinza y centro de pinza-punta de marcador. Estas distancias se encuentran en al primera línea de la función _CinInv()_ de [Interfaz](lab5Interfazv2.py).
Adicionalmente se hallan las siguientes ecuaciones para determinar los valores de las articulaciones q1, q2, q3 y q4 usando los siguientes diagramas.
![46b739a9-2a76-4600-85a5-8ed56ef3df30](https://github.com/anhernadezdu/Laboratorio5_Cinematica-Inversa_PhantomX_ROS/assets/70985250/388d460b-e36f-4b96-9b70-d68104b9971a)
$$P= O- R\cdot P4 \qquad P=[px;\quad py;\quad pz]\qquad O=[x;\quad y;\quad z]\qquad P4=[-l5;\quad 0;\quad L4]$$
Donde P4 es la traslación de la articulación 3 al TCP; P es el punto de la muñeca al realizarsele el desacople cinematico y sobre se ha de calcular los demas valores de las articulaciones; R es la matriz de rotacion correspondiente a la orientación deseada en el TPC la cual se escogio con los siguientes angulos de euler: 

$$ \Phi= q1\quad\theta=120°\quad\psi=-180°$$

Con esto en mente entonces se hallan los q de la siguietne forma:
$$q1=atan2(y,x) $$
$$r=\sqrt(Px^2+Py^2) \qquad r1=\sqrt(r^2+(L0+L1-Pz)^2)$$
$$cos(q3) =\frac{-L2^2-L3^2+r1^2}{2L2L3} $$
$$q3=atan2(\sqrt(1-cos(q3)^2),cos(q3)) $$
$$\delta=atan2(L0+L1-Pz, r) $$
$$\theta=atan2(L3sin(q3),L2+L3cos(q3)) $$
$$q2=pi/2-(\theta-\delta) $$
$$q4=2pi/3-q2-q3 $$

Las anteriores ecuaciones se compilan en la funcion vista abajo, la cual devuelve los valores de las articulaciones para alcanzar un punto especifico con la orientacion deseada.
```
def CinInv(x,y,z): #retorna angulos en radianes
    [l0,l1,l2, l3, l4, l5]=[7.3, 4.5, 10.5, 10.5, 9.0, 5.5]
    Q1=mth.atan2(y,x)
    phi=Q1
    theta=2*pi/3-pi
    psi=-pi
    R=np.dot(np.dot(rotz(phi),roty(theta)),rotx(psi))
    O=np.array([[x],[y],[z]])
    P4=np.array([[-l5],[0],[l4]])
    P=O-np.dot(R,P4)
    px=P[0]
    py=P[1]
    pz=P[2]
    r=mth.sqrt(px**2+py**2)
    r1=mth.sqrt(r**2+(l0+l1-pz)**2)
    cosq3=-(l2**2+l3**2-r1**2)/(2*l2*l3)
    Q3=mth.atan2(mth.sqrt(1-cosq3**2),cosq3)
    delta=mth.atan2(l0+l1-pz,r)
    theta=mth.atan2(l3*mth.sin(Q3),l2+l3*mth.cos(Q3))
    Q2=pi/2-theta+delta
    Q4=2*pi/3-Q2-Q3
    angulos=[Q1, -Q2, -Q3, -Q4]
    return angulos
```
Sin embargo, dado que para enviar estos valores a las articulaciones se debe tambien incluir el valor de la articulacion 5 el cual es el estado de al pinza, se estabelce una variable global llamada estado con valores de 0 y 75° para establecer el estado de pinza abierta y cerrada, el cual solo cambia al usarse una funcion que se vera mas adelante. Para añadir el estado de la pinza y enviar los angulos a los servomotores se usan las funciones _IrAPunto()_ y _enviarAngulos()_ .

La primera recibe los puntos cartesianos x,y,z de donde se desea poner la punta del marcador, hace un llamado a la funcion _CinInv()_ y a la lista que retorna le añade el valor correspondiente al estado de la pinza. Posteriormente, para enviar los angulos, se usa un *Publisher* con topic *jointTrayectory* para enviar simultaneamente los valores de las articualciones y de tal forma que el TCP vaya en linea recta desde el punto de inicio hasta el punto enviado.
```
def IrAPunto(x,y,z):
    lista=CinInv(x,y,z)
    lista.append(estado)
    return lista
def enviarAngulos(puntos):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    point = JointTrajectoryPoint()
    point.positions = puntos
    point.time_from_start = rospy.Duration(0.5)
    state.points.append(point)
    pub.publish(state)
    #print("publicados")
    rospy.sleep(5)
```

## Rutinas de trabajo
Se crean 5 rutinas de trabajo en total, las cuales consisten en dibujar ciertas figuras y coger o devolver el marcador al porta-herramientas. Estas rutinas se encuentran en las siguientes funciones dentro del codigo [Interfaz](lab5Interfazv2.py). La primera rutina es _home()_ el cual lleva el robot a la posicion home la cuales todas las articualciones en 0 cero, excepto la articulación 3 que está a -90°(-pi/2). Esta funcion se encuentra al final de cada funcion mostrada a continuación para que el robot ejecute cierta acción y vuelva a una posición donde pueda seguir otra nueva trayectoria.

Despues esta la función _herramienta()_, la cual dependiendo del argumento ingresado 1 ó 0, procede a agarrar o dejar la herramienta, respectivamente. Esta usa 3 puntos en su trayectoria para poder agarrar el marcador de tal forma que la pinza este a 90°, paralela al tablero. Esto ultimo se logra cambiando el valor de la articulacion 4 en cada uno de los puntos 2 y 3 antes de usar el publisher y mover el robot. Adicionalmente, se usa una variable global de tal modo que una vez se agarra el marcador o se deja, este estado se mantiene hasta que se ejecute la accion contraria, usando los botones de la interfaz grafica. 

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
Finalmente estan las funciones _AS()_ y _empanada()_ las cuales son un compendio de puntos los cuales al unirse forman la trayectoria la cual el marcador traza, dibujando asi las iniciales de los integrandes del grupo (A y S) y dibuja la figura libre elegida la cual fue una empanada. Estos puntos fueron hallados haciendo uso de la calculadora gráfica de Geogebra, los puntos se exportan al script en python. Para estas trayectorias, se tuvo que modificar los valores en z de cada punto debido a que, tanto el espacio de trabajo como la forma en la que el publisher del phantom dibujaba la trayectoria punto a punto, tenian imperfecciones que desfiguraban los trazos. A lo anterior tambien se suma el error en la pinza debido a una mala colocacion del marcador.
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

Cabe destacar que para la realizacion de esta interfaz se necesitó de modificar el archivo (CMakeLists.txt)[CMakeLists.txt] para incluir imagenes en el codigo de python dentro de una carpeta *resources* creada dentro del paquete *px_robot*; junto con el archivo yaml (joints)[joints.yaml], esto para que el movimiento sea más rapido pero sin exceder cierta velocidad que haga que el robot este propenso a errores y paradas de emergencia. 

Por otra parte, teniendo en cuenta estas velocidades modificadas; durante la practica se vió como al realizar movimientos muy rapidos, la base del robot tiende a moverse del punto inicial, haciendo que todas las trayectorias previamente planeadas ya no sean efectivas, por lo que se evidenciaron errores al cargar y descargar la herramienta; por tal motivo se buscó una forma de mantener la base del robot estática y asi se obtuivieran los trazos mas reproducibles posibles.

## Comparación de Trazos
Trazos de Phantom X
![recortada](https://github.com/anhernadezdu/Laboratorio5_Cinematica-Inversa_PhantomX_ROS/assets/70985250/a6beebc7-c809-41a3-9f45-333c65ded246)


Trazos en Geogebra

![image](https://github.com/anhernadezdu/Laboratorio5_Cinematica-Inversa_PhantomX_ROS/assets/70985250/f87b0714-7d03-45eb-885d-7d30e116abc9)

Superposicion de Trazos
![comparacion](https://github.com/anhernadezdu/Laboratorio5_Cinematica-Inversa_PhantomX_ROS/assets/70985250/c1ff99ca-4066-43d6-85f0-c3ab57330602)

Realizando una superposicion de las dos imagenes de los trazos generados y los deseados, se obtuvo la siguietne imagen, la cual al analizarce se observó como el error maximo en los trazos, entendido como la maxima diferencia entre puntos especificados fue en aumento conforme se acercaba al ultimo trazo dibujado por el phantom. Analizando la imagen, teniendo una resolución tal que 1pixel=1cm se obtuvo que el error maximo para los arcos internos y externos es de 2cm, para la empanada de 3cm aprox, mientras que para las iniciales: A con 8cm y S con 12cm aporximados de error entre puntos. Con estos se puede afirmar que conforme el robot fue dibuajndo los trazos requeridos fue aumentando el error, ya sea por errores en el control de posicion de los servos y/o por movimientos indeseados del marcador a pesar de las adaptaciones hechas al mismo para evitar eso mismo.

## Portaherramienta y marcador
Para poder realizar el presente laboratorio, se diseñó el siguiente portaherramientas el cual habría de sostener el marcador borrable que se usó durante la practica. Este marcador se uso junto con un anillo de plastilina el cual ayudaba a evitar el movimiento del marcador en direcciones indeseadas, fruto de la friccion con el tablero. Cabe destacar que el marcador siempre 

![68c2fc1c-5974-4199-93bf-b2abe766fb57](https://github.com/anhernadezdu/Laboratorio5_Cinematica-Inversa_PhantomX_ROS/assets/70985250/4933d62c-1221-410f-9ec4-9723f49d0091)
