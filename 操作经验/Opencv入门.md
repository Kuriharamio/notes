# Opencv入门

## 1. 初识

```python
import cv2	#引入库
```

```python
print(cv2.getVersionString())	#获取版本号
```

```python
image = cv2.imread("opencv_logo.jpg")	#图片文件要放在同一目录下
#读取图片文件，存放在numpy.ndarray类型的变量image中

print(image.shape)			#打印维度
#（250,250,3） 横行（像素）、纵列（像素）、三原色彩色通道
#对数字化图像做变换就是对像素网格进行数学变换

cv2.imshow("image",image)	#在窗口中显示图片
cv2.waitKey()				#等待键盘输入以退出，否则窗口将一闪而过
```

## 2. 彩色通道

1. 灰度图（0～255）

2. 顺序是BGR

   ```python
   cv2.imshow("blue", image[, , 0])
   cv2.imshow("green", image[, , 1])
   cv2.imshow("red", image[, , 2])
   #将后者图片内容在窗口中显示，窗口名为前者
   
   gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   cv2.imshow("gray",gray)
   #对三个彩色通道的图像，作平方和加权平均，描述了图像的明暗分布
   #称其为灰度图
   #大量图像算法基于灰度图操作
   
   ```

## 3. 图像裁剪

```python
import cv2

image = cv2.imread("opencv_logo.jpg")

crop = image[10:170, 40:200]	#裁剪图像，一行二列，三是裁剪BGR通道
cv2.imshow("crop", crop)
cv2.waitKey()
```

## 4. 绘制功能

```python
import cv2
import numpy as np		#引用numpy工具包（图像为numpy数组数据结构）

image = np.zeros([300, 300, 3], dtype=np.uint8)
#画布大小 灰度数值类型为无符号8位整型

cv2.line(image, (200, 100), (250, 250), (255, 0, 0), 2)			#画线段
#画布变量名称 线段起始坐标 线段结束坐标 线段颜色（蓝色） 线段粗细（像素）

cv2.rectangle(image, (50, 50), (250, 250), (0, 255, 0), 2)		#画矩形
#画布变量名称 对角线起始坐标 对角线结束坐标 线段颜色（绿色） 粗细（像素）

cv2.circle(image, (100, 100), 10, (0, 0, 255), 2)				#画圆
#画布变量名称 圆心坐标 半径 线段颜色（红色） 粗细（像素）

cv2.putText(image, "Hello", (100, 50), 0, 1, (255, 255, 255), 1)#写字
#画布变量名称 字符串 开始坐标 字体（默认） 字体大小 字体颜色（白色） 县条粗细（像素）

cv2.imshow("image", image)
cv2.waitKey()

```

## 5. 均值滤波

```python
import cv2

image = cv2.imread("plane.jpg")

gauss = cv2.GaussianBlur(image, (5, 5), 0 )			#高斯滤波
#图片变量名 高斯内核像素 Sigma x （设为0即由内核大小决定）

median = cv2.medianBlur(image, 5)					#均值滤波（一般用这个）
#图片变量名 均值滤波内核

cv2.imshow("image", image)
cv2.imshow("gauss", gauss)
cv2.imshow("median", median)

cv2.waitKey()


```

## 6. 图像特征提取

```python
import cv2

image = cv2.imread("opencv_logo.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

corners = cv2.goodFeaturesToTrack(gray, 500, 0.1, 10)
for corner in corners:
    x, y = corner.ravel()
    cv2.circle(image, (int(x), int(y)), 3, (255, 0, 255), -1)

cv2.imshow("image", image)

cv2.waitKey()
```

## 7. 模板匹配

```python
import cv2
import numpy as np

image = cv2.imread("poker.jpg")					
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

template = gray[75:105, 235:265]

match = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
#待匹配图片 模板 标准相关匹配算法（将图片和模板各自标准化，再计算匹配度，保证匹配结果不受光照强度影响）

locations = np.where(match >= 0.9)
#找出匹配系数大于0.9的点

w, h = template.shape[0:2]
#取出模板的长宽, 以确定对角线位置

for p in zip(*locations[::-1]):
    x1, y1 = p[0], p[1]
    x2, y2 = x1 + w, y1 + h
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

cv2.imshow("image", image)
cv2.waitKey()

```

## 8. 图像梯度算法

1. 特征点提取和匹配都使用了图像梯度
2. 图像梯度就是图像的明暗变化
3. 分别计算图像沿水平和垂直的明暗变化，再取平方和，得到梯度

```python

import cv2
import numpy as np

gray = cv2.imread("1.JPG", cv2.IMREAD_GRAYSCALE)	#直接读取灰度图

laplacian = cv2.Laplacian(gray, cv2.CV_64F)			
#拉普拉斯算子（大致对应图像的二阶导数）
#几何图形边缘的明暗程度变化剧烈，可以检测
#使均一颜色的部分变为黑色，有梯度变化的变为白色
canny = cv2.Canny(gray, 100, 200)
#canny边缘检测
#梯度区间100-200
cv2.imshow("gray", gray)
cv2.imshow("laplacian", laplacian)
cv2.imshow("canny", canny)

cv2.waitKey()
```

![image-20231114233937012](/home/mio/.config/Typora/typora-user-images/image-20231114233937012.png)

梯度变化区间

## 9. 阈值算法

1. 将灰度图像分为i黑与白
2. 阈值下为黑色，上为白色

```python

import cv2

gray = cv2.imread("bookpage.jpg", cv2.IMREAD_GRAYSCALE)

ret, binary = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
#定义一个阈值10, 最大灰度255

binary_adaptive = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 115, 1)
#自适应阈值算法，将图片分为多个区域，使光照不均的部分阈值不同
#区域大小 115

ret1, binary_otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
#自动计算恰当阈值，使灰度差异最大化，是剧烈算法

cv2.imshow("gray", gray)
cv2.imshow("threshold", binary)
cv2.imshow("adpative", binary_adaptive)
cv2.imshow("otsu", binary_otsu)

cv2.waitKey()


```

## 10. 图像的形态学算法

```python

import cv2
import numpy as np

gray = cv2.imread("opencv_logo.jpg", cv2.IMREAD_GRAYSCALE)

_, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
kernel = np.ones((5, 5), np.uint8)
#定义一个算法内核
erosion = cv2.erode(binary, kernel)
#腐蚀算法
dilation = cv2.dilate(binary, kernel)
#膨胀算法
cv2.imshow("binary", binary)
cv2.imshow("erosion", erosion)
cv2.imshow("dilation", dilation)
cv2.waitKey()
#可以多次交替使用服饰与膨胀实现打开图形空腔等操作
```

## 11. 调用摄像头

```python
import cv2

capture = cv2.VideoCapture(0)

while True:
    ret, frame = capture.read()
    cv2.imshow("camera", frame)
    key = cv2.waitKey(1)
    if key != -1:
        break
capture.release()
```

