import cv2
import cv2.aruco as aruco
import numpy as np
import time
import serial
import time

"""
SI LES BALISES DE CALIBRAGE SONT MAL PLACEES ALLER LIGNE 74
SI AUCUNE IMAGE NE SAFFICHE A LECRAN : VERIFIER VARIABLE DEVICE (DOIT ETRE EGAL A 2) LIGNE 23
                                        MODIFIER LE PARAMETRE  DE LA FONCTION LIGNE 37 (TESTER 0, 1, 2 ETC)
POUR IMLEMENTER LE TRAITEMENT DU DICO ET LA COMM AVEC ARDUINO, ALLER LIGNE 222
"""


#############" globales et setup ########################
## communication serial avec la arduino
port = "COM9"
ser = serial.Serial(port,9600)

cv2.namedWindow("frame", cv2.WINDOW_NORMAL)           # truc d'affichage tqt
detectorParams = aruco.DetectorParameters()                 #trucs pour detection de aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary, detectorParams)

matx=None                       # matrice de correction de perspective
POSITIONS = {}                  # dictionnaire qui associe une position et une date de màj à un ID de balise aruco
                                # mis à jour parallèlement à l'exécution du programme complet

width=3800     # permet de dézoomer, 3000=aucun zoom, plus c'est grand plus ca dézoome

# /!\ a regler avant l'execution!!!
DEVICE = 2  # 0 pour webcam
            # 1 pour gopro
            # 2 pour telephone (solution conservée)
            # 3 pour jpeg statique
image="complete2.png"     # image utilisee lorsque DEVICE vaut 3
no_gui="no_gui.png"       # image utilisee lorsque l'option gui est desactivee (voir dans la section PROGRAMME, deuxieme boucle while), c'est juste cosmetique
11
match DEVICE:
    case 0:
        cap=cv2.VideoCapture(0)     # des fois c'est 1 au lieu de 0, faut tester au pif
    case 1:
        pass
    case 2:
        cap=cv2.VideoCapture(1)
    case 3:
        pass    # implémenté

def getFrame(disp=False,warp=True):
    # recupere une image depuis la source specifiee par DEVICE
    # disp : si True, fournit une image destinee a l'affichage sur ordi
    # warp : si True, applique la matrice de perspective si elle existe
    if DEVICE!=3:
        ret,frame=cap.read()
    else:
        ret,frame=True,cv2.imread(image)
    frame=cv2.resize(frame,(width,int(width*2/3)))
    if type(matx)==np.ndarray and warp==True:

        frame=cv2.warpPerspective(frame,matx,(width,int(width*2/3)))
    if disp:
        return ret,frame,cv2.resize(frame,(1500,1000))
    else:
        return ret,frame

if DEVICE!=3 and not cap.isOpened():
    print("Cannot open camera")
    exit()

def perspective_matrix(markerIds, markerCorners):       
    ####### SI LES BALISES DE CALIBRAGE SONT MAL PLACEES CEST LA QUIL FAUT ALLER ################""
    listecoord= [0,0,0,0]
    for i in range(len(markerIds)):
        match markerIds[i][0]:
            case 20:          # bas droite
                listecoord[0] = markerCorners[i][0][0]
            case 21:          # bas gauche
                listecoord[1] = markerCorners[i][0][1]
            case 22:          # haut droite
                listecoord[2] = markerCorners[i][0][3]
            case 23:           # haut gauche
                listecoord[3] = markerCorners[i][0][2]
    listecoord = np.float32(listecoord)
    listecoord_ajust=np.float32([[2300+int((width-3000)/2),1530+int((width*2/3-2000)/2)],\
                                 [710.+int((width-3000)/2),1530.+int((width*2/3-2000)/2)],\
                                    [2300.+int((width-3000)/2),440.+int((width*2/3-2000)/2)],[710.+int((width-3000)/2),440.+int((width*2/3-2000)/2)]]) 
    return cv2.getPerspectiveTransform(listecoord,listecoord_ajust)

def calibrage():
    while True:
        print("appuyer sur c pour capturer une image ou q pour quitter")
        while True:
            _,_,frameS=getFrame(True,False)
            cv2.imshow("frame",frameS)
            key=cv2.waitKey(1)
            if key==ord('c'):
                break
            if key==ord('q'):
                exit()
        print("recherche d'image....")
        def isFrameOk(markerIds):
            #verifie que les quatre marqueurs de calibration sont detectes
            l=0
            if markerIds is None:
                return False
            for elt in markerIds:
                if elt[0] in [20,21,22,23]:
                    l+=1
                if l==4:
                    return True
            return False
        markerIds=[]
        t0=time.time()
        while not isFrameOk(markerIds) and time.time()-t0<2:
            #tant que les quatre marqueurs ne sont pas détectés, on continue de chercher une image valide, pendant 2 secondes
            _,frame,frameS=getFrame(True,False)
            cv2.imshow('frame',frameS)
            markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
        sortie=False   # quand True, termine la calibration et passe à la suite
        if not isFrameOk(markerIds):
            print("pas d'image valide trouvée, appuyer sur c pour retenter la calibration ou q pour tout abandonner")
            while True:
                _,frame,frameS=getFrame(True,False)
                cv2.imshow('frame',frameS)
                key=cv2.waitKey(1)
                if key==ord('q'):
                    exit()
                if key==ord('c'):
                    break
        else:
            global matx
            matx=perspective_matrix(markerIds,markerCorners)
            print("calibration terminee")
            print("appuyer sur v pour valider la calibration ou c pour retenter la calibration.")
            print("appuyer sur m pour appliquer ou pas la correction de perspective")
            print("appuyer sur q pour quitter")
            warp=True    # si True, applique la matrice, si False applique pas la matrice
            while True:
                _,frame=getFrame(False,False)
                if warp:
                    frame=cv2.warpPerspective(frame,matx,(width,int(width*2/3)))
                frame=cv2.resize(frame,(1500,1000))
                cv2.imshow('frame',frame)
                key=cv2.waitKey(1)
                if key==ord('m'):
                    warp=not warp
                    #print("click")
                elif key==ord('v'):
                    sortie=True
                    break
                elif key==ord('c'):
                    break
                elif key==ord('q'):
                    exit()
        if sortie:
            break
        

# def getAngle(x1,x2,y1,y2):
#     if  x2 == x1:
#         if y1>y2:
#             angle = 90
#         else:
#             angle = 270
#     else:
#         angle = 180/np.pi*np.arctan(np.abs(y2-y1)/np.abs(x2-x1))
#         match (x1>=x2,y1>=y2):
#             case (True,True):
#                 angle += 90
#             case(True,False):
#                 angle += 180
#             case (False,True):
#                 angle += 0
#             case(False,False):
#                 angle += 270
#     return angle

#def getAngle(x1,x2,y1,y2):
    #if  x2 == x1:
        #if y2 > y1:
            #angle = np.pi/2
        #else:
            #angle = -np.pi/2
    #elif x2 > x1 :
        #angle = np.arctan((y2-y1)/(x2-x1))
    #else:
        #angle = np.pi + np.arctan((y1-y2)/(x1-x2))
    #return(180/np.pi*angle)

def getAngle(x1, x2, y1, y2):
    if x2 == x1:
        if y2 > y1:
            angle = 90
        else:
            angle = -90
    else:
        angle = np.arctan2((y2 - y1), (x2 - x1)) * 180 / np.pi
        if angle > 180:
            angle -= 360

    return angle

def sendPos():
    toSend = '{:.0f};{:.0f};{:.0f}'.format(POSITIONS[36][0][0],POSITIONS[36][0][1],POSITIONS[36][0][2])
    print(toSend)
    ser.write(toSend.encode())

def updatePos(gui=False,warp=False):
    # met a jour le dictionnaire des positions
    # gui : si True, renvoie une image destinee au retour graphique
    # warp : si True, applique la correction de perspective
    global POSITIONS
    if gui:
        if warp:
            _,frame,frameS=getFrame(True)
        else:
            _,frame,frameS=getFrame(True,False)
    else:
        _,frame=getFrame()
    markerCorners,markerIds,rejectedCandidates=detector.detectMarkers(frame)
    if type(markerIds)==np.ndarray:
        for i in range(len(markerIds)):      # on parcourt tous les tags detectes
            x1=markerCorners[i][0][0][0]# coordonnees des quatres coins du tag aruco
            y1=markerCorners[i][0][0][1]
            x2=markerCorners[i][0][1][0]
            y2=markerCorners[i][0][1][1]
            x3=markerCorners[i][0][2][0]
            y3=markerCorners[i][0][2][1]
            x4=markerCorners[i][0][3][0]
            y4=markerCorners[i][0][3][1]
            if x3==x1 or x4==x2:
                print("okokokok")
                x=float((x1+x3))/2
                y=float((y1+y3))/2
            else:
                x=(y2-y1+(y3-y1)/(x3-x1)*x1-(y4-y2)/(x4-x2)*x2)/((y3-y1)/(x3-x1)-(y4-y2)/(x4-x2))    # on calcule les coord du centre du tag aruco en trouvant
                y=(y3-y1)/(x3-x1)*(x-x1)+y1                                                          # l'intersection entre les deux diagonales de la balise
            x=int(-(x-width/2))                                                     # on place l'origine au centre et les axes comme il faut (y vers le bas, x vers la gauche)
            y=int(y-width*1/3)
            angle= getAngle(x1,x2,y1,y2)
            angle = int(angle)  
            POSITIONS[markerIds[i][0]]=[(x,y,angle),time.time()]
    if gui:
        return frameS


####################################### CALIBRAGE #################################

calibrage()

####################################### PROGRAMME ###############################

print("pret au lancement")
print("appuyer sur:\n\tg pour activer le retour graphique (au prix de la performance)\n\tm pour activer\
       ou desactiver la correction de perspective\n\tv pour lancer le programme\n\tq pour quitter le programme")
warp=True
gui=True
while True:
    _,frame=getFrame(False,False)
    if warp:
        frame=cv2.warpPerspective(frame,matx,(width,int(width*2/3)))
    frameS=cv2.resize(frame,(1500,1000))
    cv2.imshow('frame',frame)
    key=cv2.waitKey(1)
    if key==ord('m'):
        warp=not warp
    elif key==ord('v'):
        break
    elif key==ord('q'):
        exit()

print("programme lance")   
    
while True:      ################ CEST ICI QUIL FAUT IMPLEMENTER LA COMM ####################
    if gui:
        frameS=updatePos(gui,warp)
        cv2.imshow('frame',frameS)
    else:
        updatePos()
    try:
        ################# TRAITEMENT DICO + COMM ICI #######################
        print('{:.2f} {:.2f} {:.2f}'.format(POSITIONS[36][0][0],POSITIONS[36][0][1],POSITIONS[36][0][2]))
        sendPos()
        
    except KeyError:
        print("jamais detecte")
    key=cv2.waitKey(1)
    if key==ord('g'):
        gui=not gui
        if gui:
            print("retour actif")
        else:
            frame=cv2.imread(no_gui)
            cv2.imshow("frame",frame)
            print("retour inactif")
    elif key==ord('m'):
        warp=not warp
    elif key==ord('q'):
        print("hell yeah")
        exit()
        
    
        


