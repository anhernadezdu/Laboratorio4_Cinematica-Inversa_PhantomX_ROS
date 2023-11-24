import tkinter as tk
from tkinter import ttk
from PIL import Image
from PIL import ImageTk
import rospy
import rospkg
import os
import numpy as np
import math as mth
import time
from cmath import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.srv import DynamixelCommand

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
def rotx(x: float):
    matriz=np.array([[1,0,0],[0,mth.cos(x),-mth.sin(x)],[0,mth.sin(x),mth.cos(x)]])
    return matriz
def roty(y: float):
    matriz=np.array([[mth.cos(y),0,mth.sin(y)],[0,1,0],[-mth.sin(y),0,mth.cos(y)]])
    return matriz
def rotz(z: float):
    matriz=np.array([[mth.cos(z),-mth.sin(z),0],[mth.sin(z),mth.cos(z),0],[0,0,1]])
    return matriz
#devuelve angulos de motores en grados usando punto deseado y orientacion :
#phi= lo que tenga que girar el motor 1 para alcanzar el punto
#theta=120 psi=-180
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

def IrAPunto(x,y,z):
    lista=CinInv(x,y,z)
    lista.append(estado)
    return lista

def home():
    punto=[0,0,-pi/2,0,estado]
    enviarAngulos(punto)

def herramienta(coger):
    global estado
    punto1=IrAPunto(0.2,-20,10)
    #print(punto1)
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
    #time.sleep(0.15)

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

def AS():
    #puntos de A
    punto1=IrAPunto(8.03, 13.48, 3)
    punto2=IrAPunto(8.03, 13.48, 1.75)
    punto3=IrAPunto(11.19, 14.56, 1.6)
    punto4=IrAPunto(15.68, 16.08, 1.75)
    punto5=IrAPunto(13.33, 11.64, 1.75)
    punto5aux=IrAPunto(13.33, 11.64, 2.5)
    punto6=IrAPunto(12, 9.12, 1.7)
    punto6aux=IrAPunto(12, 9.12, 3)
    punto7=IrAPunto(11.8, 14.56, 5)
    #puntos de S
    punto8=IrAPunto(13.07, 8.09, 3)
    punto9=IrAPunto(13.1, 8.1, 1.5)
    punto10=IrAPunto(15.02, 5, 1.5)
    punto11=IrAPunto(18.13, 7.02, 1.7)
    punto12=IrAPunto(16, 10, 1.99)
    punto13=IrAPunto(18.9, 11.97, 1.92)
    punto14=IrAPunto(21.33, 8.3, 1.92)
    punto15=IrAPunto(20.92, 9.12, 5)
    #dibujar A
    enviarAngulos(punto1)
    enviarAngulos(punto2)
    enviarAngulos(punto3)
    enviarAngulos(punto4)
    enviarAngulos(punto5)
    enviarAngulos(punto6)
    enviarAngulos(punto6aux)
    enviarAngulos(punto5aux)
    enviarAngulos(punto5)
    enviarAngulos(punto3)
    enviarAngulos(punto7)
    #dibujar S
    enviarAngulos(punto8)
    enviarAngulos(punto9)
    enviarAngulos(punto10)
    enviarAngulos(punto11)
    enviarAngulos(punto12)
    enviarAngulos(punto13)
    enviarAngulos(punto14)
    enviarAngulos(punto15)
    home()

def empanada():
    punto1=IrAPunto(14.56, -2.43, 3)
    punto2=IrAPunto(14.56, -2.43, 1.4)
    punto3=IrAPunto(15.67, -1.36, 1.3)
    punto4=IrAPunto(16.98, -0.63, 1.4)
    punto5=IrAPunto(18.21, -0.29, 1.5)
    punto6=IrAPunto(19.29, -0.21, 1.6)
    punto7=IrAPunto(20.38, -0.33, 1.6)
    punto8=IrAPunto(21.29, -0.6, 1.6)
    punto9=IrAPunto(22.26, -1.08 , 1.6)
    punto10=IrAPunto(23.16, -1.75, 1.6)
    #vertice
    punto11=IrAPunto(23.96, -2.65, 1.6)
    punto12=IrAPunto(24.17, -1.22, 1.6)
    punto13=IrAPunto(23.97, 0.15, 1.6)
    punto14=IrAPunto(23.34, 1.48, 1.6)
    punto15=IrAPunto(22.30, 2.61, 1.6)
    punto16=IrAPunto(21.03, 3.33, 1.6)
    punto17=IrAPunto(19.53, 3.64, 1.4)
    punto18=IrAPunto(17.93, 3.46, 1.6)
    punto19=IrAPunto(16.28, 2.61, 1.5)
    punto20=IrAPunto(15.24, 1.50, 1.4)
    punto21=IrAPunto(14.67, 0.34, 1.3)
    punto22=IrAPunto(14.41, -1.0, 1.3)
    #ir a punto2 despeus de 22 o 21
    #dibujar empanada
    enviarAngulos(punto1)
    enviarAngulos(punto2)
    enviarAngulos(punto3)
    enviarAngulos(punto4)
    enviarAngulos(punto5)
    enviarAngulos(punto6)
    enviarAngulos(punto7)
    enviarAngulos(punto8)
    enviarAngulos(punto9)
    enviarAngulos(punto10)
    enviarAngulos(punto11)
    enviarAngulos(punto12)
    enviarAngulos(punto13)
    enviarAngulos(punto14)
    enviarAngulos(punto15)
    enviarAngulos(punto16)
    enviarAngulos(punto17)
    enviarAngulos(punto18)
    enviarAngulos(punto19)
    enviarAngulos(punto20)
    enviarAngulos(punto21)
    enviarAngulos(punto22)
    enviarAngulos(punto2)
    enviarAngulos(punto1)
    home()

def generate_text(text):
    text_box.insert(tk.END, text)


# Function for the buttons
def button_click(pos):
    inicial=time.time()
    text_box.delete(1.0, tk.END) 
    line="Volviendo a home \n"
    if pos==1:
        herramienta(1)
        line=line+"Cogiendo Herramienta \n"+"Posicion: (-2, 8, 5)\n"
    elif pos==2:
        herramienta(0)
        line=line+"Cogiendo Herramienta \n"+"Posicion: (-2, 8, 5)\n"
    elif pos==3:
        circuloInt()
        circuloExt()
        line=line+"Dibujando Espacio de Trabajo \n"+"Radio interno: 13 cm \n"+"Radio Externo: 25 cm \n"
    elif pos==4:
        AS()
        line=line+"Escribinedo iniciales \n"+"Posicion: (8.03, 13.48, 0)\n"+"..... \n"
    else: 
        empanada()
        line=line+"Dibujando Empanada \n"+"Posicion: (14.56, -2.56, 0)\n"+"..... \n"
    final=time.time()
    tiempo=final-inicial
    line=line+"Tiempo de ejecucion: "+str("{0:.2f}".format(tiempo))+"\n"
    if estado==0:
        line=line+"Estado de herramienta: Suelto \n Rutina FINALIZADA"
    else:
        line=line+"Estado de herramienta: Agarrado \n Rutina FINALIZADA"
    generate_text(line)


def crearInterfaz():
    root = tk.Tk()
    root.title("Interfaz PhantomX")
    logo_frame = ttk.Frame(root)
    logo_frame.grid(row=0, column=0, padx=10, pady=10)
    logo_image = Image.open(png_path)
    logo_image = logo_image.resize((150, 150), Image.ANTIALIAS) 
    logo_image = ImageTk.PhotoImage(logo_image)
    logo_label = ttk.Label(logo_frame, image=logo_image)
    logo_label.image = logo_image
    logo_label.grid(row=0, column=0, padx=10, pady=5)
    name_label = ttk.Label(logo_frame, text="Laboratorio 5 Robotica 2023-2S")
    name_label.grid(row=1, column=0, padx=10, pady=5)
    first_name_label = ttk.Label(logo_frame, text="Andres David Hernandez y Juan Sebastian Daleman")
    first_name_label.grid(row=2, column=0, padx=10, pady=5)
    second_name_label = ttk.Label(logo_frame, text="Interfaz de control")
    second_name_label.grid(row=3, column=0, padx=10, pady=5)
    buttons_frame = ttk.Frame(root)
    buttons_frame.grid(row=1, column=0, padx=10, pady=10)
    buttons = []
    button1 = ttk.Button(buttons_frame, text="Coger Herramienta", command=lambda i=1: button_click(i))
    button1.grid(row=0, column=0, padx=10, pady=5)
    buttons.append(button1)
    button2 = ttk.Button(buttons_frame, text="Descargar Herramienta", command=lambda i=2: button_click(i))
    button2.grid(row=0, column=1, padx=10, pady=5)
    buttons.append(button2)
    button3 = ttk.Button(buttons_frame, text="Espacio de Trabajo", command=lambda i=3: button_click(i))
    button3.grid(row=0, column=2, padx=10, pady=5)
    buttons.append(button3)
    button4 = ttk.Button(buttons_frame, text="Iniciales", command=lambda i=4: button_click(i))
    button4.grid(row=0, column=3, padx=10, pady=5)
    buttons.append(button4)
    button5 = ttk.Button(buttons_frame, text="Empanada", command=lambda i=5: button_click(i))
    button5.grid(row=0, column=4, padx=10, pady=5)
    buttons.append(button5)
    global text_box
    text_box = tk.Text(root, height=7, width=70)
    text_box.grid(row=2, column=0, padx=10, pady=10)
    root.mainloop()
agarrar=75*pi/180
suelto=0
estado=suelto
if __name__ == '__main__':
    try:
        enviarAngulos([0,0,0,0,0])
        home()
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('px_robot')
        global png_path
        png_path = os.path.join(package_path, 'resources', 'unlogo.png')
        crearInterfaz()
    except rospy.ROSInterruptException:
        pass



