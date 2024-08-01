# NOTE: make sure the gc heap memory is large enough to run this demo ,  The recommended size is 1M

import sensor, image, time, lcd, machine
from maix import KPU #导入KPU库（用于做卷积神经网络加速器调用�?
import gc #导入gc�?
from maix import GPIO, utils    #导入GPIO、utils�?(需要使用到按键相关函数)
from fpioa_manager import fm    #导入fm库作文件（需要使用到按键相关函数，读取TF卡文件内容）
from board import board_info    #导入board_info�?
from modules import ybserial #导入自定义的串口�?(需要使用到串口相关函数)

fm.register(board_info.BOOT_KEY, fm.fpioa.GPIOHS0)  #“设置寄存器，关联到板载按键信息�?
key_gpio = GPIO(GPIO.GPIOHS0, GPIO.IN)  #设置按键引脚为输�?

#初始化显示屏
lcd.init()
#sensor.reset(dual_buff=True) # improve fps
sensor.reset()                      # Reset and initialize the sensor. It will
                                    # run automatically, call sensor.run(0) to stop
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 1000)     # Wait for settings take effect.
# sensor.set_hmirror(0)
# sensor.set_vflip(1)
clock = time.clock()                # Create a clock object to track the FPS.

#初始化串�?
serial = ybserial()

#设置串口波特�?
uart = machine.UART(1, baudrate=115200)

#设置图像大小�?64*64
feature_img = image.Image(size=(64,64), copy_to_fb=False)
feature_img.pix_to_ai()

#设置人脸变量大小�?64
FACE_PIC_SIZE = 64

#用于图像处理的对比数�?
dst_point =[(int(38.2946 * FACE_PIC_SIZE / 112), int(51.6963 * FACE_PIC_SIZE / 112)),
            (int(73.5318 * FACE_PIC_SIZE / 112), int(51.5014 * FACE_PIC_SIZE / 112)),
            (int(56.0252 * FACE_PIC_SIZE / 112), int(71.7366 * FACE_PIC_SIZE / 112)),
            (int(41.5493 * FACE_PIC_SIZE / 112), int(92.3655 * FACE_PIC_SIZE / 112)),
            (int(70.7299 * FACE_PIC_SIZE / 112), int(92.2041 * FACE_PIC_SIZE / 112)) ]

#人脸模型数据（通过官方网站进行模型训练得到的结果）
anchor = (0.1075, 0.126875, 0.126875, 0.175, 0.1465625, 0.2246875, 0.1953125, 0.25375, 0.2440625, 0.351875, 0.341875, 0.4721875, 0.5078125, 0.6696875, 0.8984375, 1.099687, 2.129062, 2.425937)
kpu = KPU()
kpu.load_kmodel("/sd/KPU/yolo_face_detect/face_detect_320x240.kmodel")  #指定人脸识别模型文件的路�?
kpu.init_yolo2(anchor, anchor_num=9, img_w=320, img_h=240, net_w=320 , net_h=240 ,layer_w=10 ,layer_h=8, threshold=0.5, nms_value=0.2, classes=1)

ld5_kpu = KPU()
print("ready load model")
ld5_kpu.load_kmodel("/sd/KPU/face_recognization/ld5.kmodel") #指定ld5模型文件的路�?

fea_kpu = KPU()
print("ready load model")
fea_kpu.load_kmodel("/sd/KPU/face_recognization/feature_extraction.kmodel")#指定feature_extraction模型文件的路�?

#按键标志位状�?
start_processing = False
BOUNCE_PROTECTION = 50  #延时时间

#设置按键状�?
def set_key_state(*_):
    global start_processing
    start_processing = True #设置标志位为1
    time.sleep_ms(BOUNCE_PROTECTION) #延时50ms时间
#按键的中断处理函�? set_key_state：回调函�?/处理函数
key_gpio.irq(set_key_state, GPIO.IRQ_RISING, GPIO.WAKEUP_NOT_SUPPORT)

record_ftrs = []
THRESHOLD = 80.5    #人脸数据的对比值， 如果该对比值大�? 80.5 ，说明识别到一张人脸，且人脸是已记录到系统

RATIO = 0
def extend_box(x, y, w, h, scale):  #设置“信息包�?
    x1_t = x - scale*w
    x2_t = x + w + scale*w
    y1_t = y - scale*h
    y2_t = y + h + scale*h
    x1 = int(x1_t) if x1_t>1 else 1
    x2 = int(x2_t) if x2_t<320 else 319
    y1 = int(y1_t) if y1_t>1 else 1
    y2 = int(y2_t) if y2_t<240 else 239
    cut_img_w = x2-x1+1
    cut_img_h = y2-y1+1
    return x1, y1, cut_img_w, cut_img_h

recog_flag = False
while 1:
    gc.collect()    #分配堆栈空间
    #print("mem free:",gc.mem_free())
    #print("heap free:",utils.heap_free())
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot() #拍一张照�?
    kpu.run_with_output(img) #输出图片给到KPU
    dect = kpu.regionlayer_yolo2()  #启动yolo2算法处理
    fps = clock.fps()
    if len(dect) > 0:
        for l in dect :         #获取图像中的人脸信息中的x,y,w,h,scale信息
            x1, y1, cut_img_w, cut_img_h= extend_box(l[0], l[1], l[2], l[3], scale=RATIO)
            face_cut = img.cut(x1, y1, cut_img_w, cut_img_h)
            #a = img.draw_rectangle(l[0],l[1],l[2],l[3], color=(255, 255, 255))
            # img.draw_image(face_cut, 0,0)
            face_cut_128 = face_cut.resize(128, 128)
            face_cut_128.pix_to_ai()
            out = ld5_kpu.run_with_output(face_cut_128, getlist=True)
            face_key_point = []

            for j in range(5):
                x = int(KPU.sigmoid(out[2 * j])*cut_img_w + x1) #KPU结果输出 x 的坐�?
                y = int(KPU.sigmoid(out[2 * j + 1])*cut_img_h + y1)#KPU结果输出 x 的坐�?
                # a = img.draw_cross(x, y, size=5, color=(0, 0, 255))

                face_key_point.append((x,y))    #

            #处理图像
            T = image.get_affine_transform(face_key_point, dst_point)   #仿射变换，用来作拍摄到的人脸图片的旋转、缩放、平移、裁剪处�?
            a = image.warp_affine_ai(img, feature_img, T)
            # feature_img.ai_to_pix()
            # img.draw_image(feature_img, 0,0)
            feature = fea_kpu.run_with_output(feature_img, get_feature = True)
            del face_key_point

            scores = []
            for j in range(len(record_ftrs)):
                score = kpu.feature_compare(record_ftrs[j], feature)    #进行人脸图像数据对比，结果输出到score
                scores.append(score)    #拼接score列表（拼接数组）

            if len(scores):
                max_score = max(scores) #求数组最大值，�? max_score
                index = scores.index(max_score)
                if max_score > THRESHOLD:   #判断识别结果最大值是�? 超出 指定的阈值，如果超出说明识别到人�?
                    #识别到人脸直接绘制文字信�?
                    a=img.draw_string(0, 195, "persion:%d,score:%2.1f" %(index, max_score), color=(0, 255, 0), scale=2)
                    recog_flag = True
                    serial.send("1")
                else:
                    a=img.draw_string(0, 195, "unregistered,score:%2.1f" %(max_score), color=(255, 0, 0), scale=2)
                    serial.send("0")
            del scores  #删除列表

            #检测到按键按下，则print打印信息
            if start_processing and len(record_ftrs)<=3:
                record_ftrs.append(feature) #把需要处理的图像拼接到要处理的”任务�?
                print("record_ftrs:%d" % len(record_ftrs))
                start_processing = False

            #如果识别到人脸，则对人脸进行绘制一个绿色的矩形
            if recog_flag:
                a = img.draw_rectangle(l[0],l[1],l[2],l[3], color=(0, 255, 0))
                serial.send("B")
                #直接利用串口输出 一个信�? 到串口接�? ，再传输到其他设备：STM32设备
                print("face True")  #输出 信息�? 电脑�?
                                     #输出 信息�? 外部设备�?

                recog_flag = False
            else:
                a = img.draw_rectangle(l[0],l[1],l[2],l[3], color=(255, 255, 255))
                serial.send("A")
            del (face_cut_128)
            del (face_cut)

    a = img.draw_string(0, 0, "%2.1ffps" %(fps), color=(0, 60, 255), scale=2.0)
    a = img.draw_string(0, 215, "press boot key to regist face", color=(255, 100, 0), scale=2.0)
    lcd.display(img)    #显示图像到屏幕上


kpu.deinit()    #重置kpu
ld5_kpu.deinit()     #重置ld5_kpu
fea_kpu.deinit()     #重置fea_kpu
