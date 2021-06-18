import pyb, sensor, image, time, math


THRESHOLD = (5, 70, -23, 15, -57, 0) # Grayscale threshold for dark things...

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

def degrees(radians):
   return (180 * radians) / math.pi

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)

while(True):
   clock.tick()

   i = 0;
   a = uart.readchar()
   
   if (a == 76):
       sensor.set_windowing((17,95,131,23))
       img  = sensor.snapshot().binary([THRESHOLD])
       line = img.get_regression([(255,255)], robust = True)
       if (line):
          img.draw_line(line.line(), color = 127)
          print(line.theta())
          uart.write(("%d,%d\n" % (line.theta(), line.x1())).encode())
   elif (a == 65):
       sensor.set_windowing((1,2,158,117))
       img = sensor.snapshot()
       for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
          img.draw_rectangle(tag.rect(), color = (255, 0, 0))
          img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
                   # The conversion is nearly 6.2cm to 1 -> translation
          print_args = (tag.x_translation(), tag.y_translation(), tag.z_translation(), \
          degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
                   # Translation units are unknown. Rotation units are in degrees.
          uart.write(("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % print_args).encode())
          print("Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f" % print_args)
