import numpy as np
import cv2

import smtplib
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart

smtp_server = "smtp.gmail.com"
smtp_port = 587

password = ""
MailFrom = "@gmail.com"
MailTo1 = ""
MailTo2 = ""

file_image = ""

def init_smtp():
    s = smtplib.SMTP(smtp_server,smtp_port)
    return s

def send_mail_image(file_image,s):
    fp = open(file_image, 'rb')
    
    msg = MIMEMultipart()
    emails = [MailTo1,MailTo2]
    msg['Subject'] = 'prueba imagen'
    msg['From'] = MailFrom
    msg['To'] = ', '.join(emails)
    text = MIMEText("texto test")
    msg.attach(text)
    image = MIMEImage(fp.read(),name="image_camara")
    msg.attach(image)
    s.ehlo()
    s.starttls()
    s.ehlo()
    s.login(MailFrom,password)
    s.sendmail(MailFrom,emails,msg.as_string())
    s.quit()
    

def camera_control():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,640)
    ret,frame = cap.read()
    
    w = cv2.imwrite(file_image,frame)
    print(w)
    s = init_smtp()
    send_mail_image(file_image,s)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print('Ejecutando como programa principal')
    camera_control()
