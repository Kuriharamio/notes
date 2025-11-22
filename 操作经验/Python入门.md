# Python入门

## 1. Hello World!

```python
printf("Hello World!")
```

## 2. 变量类型

```python
name = "hello" 	#字符串（单引号双引号无区别）
online = True  	#布尔类型 (要大写)
age = 20		#整数类型
price = 6.3		#浮点数类型

print(name, online, age, price)		#hello True 20 6.3

print(name[0:2])					#he (打印0-2)

print(type(name), type(online), type(age), type(price)) #数据类型
#<class 'str'> <class 'bool'> <class 'int'> <class 'float'>

new_price = float("3")				#数据类型转换 （字符串转换为浮点数）
print(new_price, type(new_price))	#3.0 <class 'float'>

array = [1, 2, 3, 4, 5, 6]			#定义数组（方括号）
print(len(array))					#6（计算数组长度）
print(array[0], array[1])			#1 2

my_dict = {"a": 1, "b": 2, "c": "Hello"}	#定义字典
print(my_dict["a"])							#1（方括号访问字典）
my_dict["a"] = 100							#修改字典数据
print(my_dict)								#{'a': 100, 'b': 2, 'c': 'hello'}					

```

## 3. 条件语句

```python
online = True
if online:
    print("hello")
else:
    print("good bye")
```

```python
age = 20
if age < 35:
    print("young")
elif age < 65:
    print("middle")
else:
    print("old")
```

## 4. 循环语句

```python
array = [1, 3, 5, 7, 9]
for x in array:
    print(x)
'''
1
3
5
7
9
'''
for (i, x) in enumerate(array):
    print(i, x)
'''
0 1
1 3
2 5
3 7
4 9
'''
    
```

```python
counter = 100
while counter > 10 :
    print(counter)
    counter = counter - 1
```

## 5. 函数

```python
def sum_values(values):
    s = 0
    for v in values:
        s = s + v
    return s

array = [1, 3, 5, 7, 9]
print(sum_values(array))

def show_name(name = "frank"):
    print(name)
    
show_name("peter")
show_name()
show_name(name = "peter")
```

## 6. 导入别人的代码

```python
import time

print("staet")
time.sleep(2)
print("finish")
```

```python
import requests

response = requests.get("http://www.example.com")
print(response.content)
```

