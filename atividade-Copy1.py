import cv2
from math import sqrt, atan, degrees
import numpy as np

cap = cv2.VideoCapture(0)

# lower_magenta = (130,0,0)
# upper_magenta = (170,255,255)

lower_magenta = (320/2-20,100,100)
upper_magenta = (322/2+20,255,255)

lower_ciano = (78, 100, 100)
upper_ciano = (112, 255, 255)

def find_center(contorno):
    M = cv2.moments(contorno)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return cx, cy

def cruzes(img, cx, cy):
    l = 10
    cor = (255, 100, 0)
    cv2.line(img,(cx-l,cy),(cx+l,cy),cor,3)
    cv2.line(img,(cx,cy-l),(cx,cy+l),cor,3)

def draw_line(img, cx1, cy1, cx2, cy2):
    cor = (0,165,255)
    cv2.line(img,(cx1,cy1),(cx2,cy2),cor,3)

def pitagoras(cx1, cy1, cx2, cy2):
    return round(sqrt(pow(cx1 - cx2, 2) + pow(cy1 - cy2, 2)), 2)

def escreve(img, cx1, cy1, cx2, cy2, texto):
    mx = int((cx1 + cx2)/2.0)
    my = int((cy1 + cy2)/2.0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,texto,(mx-25,my-25), font, 1 ,(255,255,255),2,cv2.LINE_AA)

def calc_dis(hi):
    f = 385.7
    h = 140
    return round(f*h/hi, 2)

def angle(cx1, cy1, cx2, cy2):
    dx = abs(cx1-cx2)
    dy = abs(cy1-cy2)
    if dx < 0.1:
        return 90.0
    return round(degrees(atan(dy/dx)), 2)

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

def hough_circles(bordas, bordas_color):
    circles=cv2.HoughCircles(bordas,cv2.HOUGH_GRADIENT,2,40,param1=50,param2=50,minRadius=5,maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            print(i)
            # draw the outer circle
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
            cv2.circle(bordas_color,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(bordas_color,(i[0],i[1]),2,(0,0,255),3)

    
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_magenta = cv2.inRange(hsv, lower_magenta, upper_magenta)
    mask_ciano = cv2.inRange(hsv, lower_ciano, upper_ciano)
    
    draw = frame.copy()
        
    #juntas as duas coisas
    masks = cv2.bitwise_or(mask_ciano, mask_magenta)
    
    #circulas as coisas
    bordas = auto_canny(masks)
    
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)
    hough_circles(bordas, bordas_color)
    
    contornos_img, contornos, arvore = cv2.findContours(masks.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rgb_copy = cv2.cvtColor(contornos_img, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(rgb_copy, contornos, -1, [0, 0, 255], 3);
    
    maior = None
    maior_area = 0
    maior2 = None
    maior_area2 = 0
    
    for c in contornos:
        area = cv2.contourArea(c)
        if area > maior_area:
            maior_area2= maior_area
            maior2=maior
            maior_area = area
            maior = c
            
    cx1 = cy1 = -1  
    if maior is not None:
        cv2.drawContours(draw, [maior], -1, [0, 255, 255], 5);
        cx1, cy1 = find_center(maior)
        cruzes(draw, cx1, cy1)
        
    if maior2 is not None:
        cv2.drawContours(draw, [maior2], -1, [0, 255, 0], 5);
        cx2, cy2 = find_center(maior2)
        cruzes(draw, cx2, cy2)
        draw_line(draw, cx1, cy1, cx2, cy2)
        hi = pitagoras(cx1, cy1, cx2, cy2)
        d = calc_dis(hi)
        angulo = angle(cx1, cy1, cx2, cy2)
        escreve(draw, cx1, cy1, cx2, cy2, str(d))
        escreve(draw, cx1, cy1+50, cx2, cy2+50, str(angulo))

        

    
    print("Codigo de retorno", ret)

    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Display the resulting frame
    #cv2.imshow('frame',frame)
    #cv2.imshow('frame',frame)
    #cv2.imshow('mask_m',mask_magenta)
#     cv2.imshow('mask_c',mask_ciano)
    #cv2.imshow('masks', masks)
    cv2.imshow('color', draw)   
    #cv2.imshow('bordas', bordas)   
    #cv2.imshow('bordas', bordas_color)   

    
    cv2.imshow('masks', rgb_copy)   


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

