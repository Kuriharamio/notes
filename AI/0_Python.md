# Python

## 1、字典推导式

- ```python
  prompt_templates = {k: v+ext for k, v in prompt_templates.items()}
  ```

这行代码是一个 Python 字典推导式（dictionary comprehension），它用于创建一个新的字典`prompt_templates`。这个新字典通过在原有`prompt_templates`字典中的每个值后面添加`ext`字符串来更新每个键值对。

让我们逐步分解这个表达式：

- `prompt_templates`: 这是一个已经存在的字典变量，它的每个键（`k`）对应一个值（`v`）。
- `for k, v in prompt_templates.items()`: 这是一个循环，迭代 `prompt_templates` 字典的所有键值对。`.items()` 方法返回一个包含字典中所有键值对的列表。
- `{k: v+ext for ...}`: 这是字典推导式的结构，它将循环中每次迭代的键（`k`）与修改后的值（`v + ext`）组合起来，形成新的键值对，然后将所有的新键值对组合到一个新字典中。

`ext` 应该是一个已经定义的字符串，它将被添加（连接）到原字典的每个值后面。

例如：

```python
prompt_templates = {
    "greeting": "Hello, ",
    "farewell": "Goodbye, "
}

ext = "have a nice day!"

# 应用字典推导式
prompt_templates = {k: v+ext for k, v in prompt_templates.items()}

print(prompt_templates)
```

输出将会是：

```python
{
    "greeting": "Hello, have a nice day!",
    "farewell": "Goodbye, have a nice day!"
}
```

在这个例子中，`ext` 被添加到了原始 `prompt_templates` 字典中的每个值后面，创建了一个新的字典，其值包含了额外的问候语。

## 2、Del删除变量

`del`不是一个方法，而是Python中的一个语句，用于删除对象。在Python中，`del`可以用来删除变量、列表中的元素、字典中的键值对等。使用`del`语句可以释放被它删除的对象所占用的内存空间（如果没有其他引用指向该对象）。

下面是一些使用`del`的例子：

### 删除变量

```python
x = 10
print(x)  # 输出: 10
del x
# 尝试访问x将会抛出错误，因为x已经被删除
# print(x)  # NameError: name 'x' is not defined
```

### 删除列表元素

```python
my_list = ['a', 'b', 'c', 'd']
del my_list[1]
print(my_list)  # 输出: ['a', 'c', 'd']
```

### 删除字典中的键值对

```python
my_dict = {'name': 'Alice', 'age': 25}
del my_dict['age']
print(my_dict)  # 输出: {'name': 'Alice'}
```

在上下文中，`del`被用来从字典中删除一个特定的键（及其对应的值）。例如，如果有一个字典`state`和一个键`slot`，`del state[slot]`会从`state`字典中删除键`slot`及其对应的值。这是管理字典内容的一种常见方式，特别是在需要根据程序的逻辑动态调整字典结构时。

## 3、**kwargs

`**kwargs` 是一个在 Python 函数定义中使用的特殊语法，它允许你将不定数量的关键字参数传递给一个函数。`kwargs` 代表 "keyword arguments"（关键字参数），而 `**` 是一个解包操作符，它将传入函数的关键字参数打包成一个字典。

下面是一些关于 `**kwargs` 的关键点：

1. **用法**：在函数定义中，你可以使用 `**kwargs` 来收集那些没有明确指定的关键字参数。这些参数被打包进一个名为 `kwargs` 的字典中（实际上，字典的名称可以是任意的，`kwargs` 只是一种约定俗成的命名方式）。

2. **字典**：在函数体内，`kwargs` 是一个字典，你可以像操作任何字典那样操作它。例如，可以检查它是否包含某个关键字 `key in kwargs`，访问某个关键字的值 `kwargs[key]`，或者遍历所有键值对。

3. **灵活性**：`**kwargs` 使得函数调用时更加灵活，能够接受任意数量的关键字参数。这在编写通用函数或装饰器时非常有用，当你预先不知道用户会传递哪些附加信息时。

4. **与 `*args` 结合使用**：`*args` 和 `**kwargs` 可以在同一个函数定义中使用，其中 `*args` 接收任意数量的位置参数，`**kwargs` 接收任意数量的关键字参数。注意它们的顺序，`*args` 必须出现在 `**kwargs` 之前。

下面是一个简单的例子，展示了如何在函数中使用 `**kwargs`：

```python
def greet(name, **kwargs):
    print(f"Hello, {name}!")
    for key, value in kwargs.items():
        print(f"{key}: {value}")

# 调用函数
greet("Alice", age=30, city="New York")
```

输出：
```python
Hello, Alice!
age: 30
city: New York
```

在这个例子中，`name` 是一个普通的位置参数，而任何额外的关键字参数都会被 `**kwargs` 捕获并存储在一个字典中。这使得 `greet` 函数可以灵活地处理额外的信息，而不需要事先定义所有可能的参数。



## 4、return 循环

在Python中，可以在一个函数的`return`语句后面使用循环语句（如`for`循环、`while`循环等），这样做被称为生成器表达式或者列表推导式。

生成器表达式和列表推导式允许在一个单一的语句中迭代一个可迭代对象，并使用迭代的结果来构建一个新的可迭代对象。在这种情况下，`return`语句返回的是一个生成器对象或者一个列表对象。

具体到你提到的代码中的字典推导式：`{k: v for k, v in semantics.items() if v}`，它使用了字典推导式的语法，在迭代`semantics.items()`的过程中，只有当`v`的值不为空（`if v`条件为真）时，才会将键值对加入到新的字典中。

简单来说，这个生成器表达式的作用是从`semantics`字典中过滤掉值为空的键值对，然后将非空值的键值对构建成一个新的字典。

## 5、json

### json.loads()

`json.loads()`是一个Python标准库中的方法，用于将JSON字符串解析为Python对象。

JSON（JavaScript Object Notation）是一种常用的数据交换格式，它使用简洁的文本格式来表示结构化的数据。而`json.loads()`方法则是将这些JSON字符串解析为相应的Python数据类型，例如字典、列表、字符串、整数等。

使用`json.loads()`方法时，需传入一个JSON格式的字符串作为参数。该方法将字符串解析成相应的Python对象，并返回解析后的对象。如果JSON字符串不符合JSON格式，则会抛出`json.JSONDecodeError`异常。

下面是一个简单的示例，演示了如何使用`json.loads()`方法将JSON字符串解析为Python字典：

```python
import json

json_str = '{"name": "John", "age": 30, "city": "New York"}'
data = json.loads(json_str)

print(data)
# 输出: {'name': 'John', 'age': 30, 'city': 'New York'}
print(type(data))
# 输出: <class 'dict'>
```

在上述示例中，`json_str`是一个JSON字符串。通过调用`json.loads(json_str)`，将其解析为Python字典对象`data`。然后，我们可以通过操作`data`来访问和处理其中的数据。

### json.dumps()

`json.dumps()` 是Python中`json`模块提供的一个函数，用于将Python对象编码成JSON字符串。这个函数非常有用，因为它允许将Python中的数据结构（如字典、列表、元组等）转换为JSON格式的字符串，这种格式在网络传输和数据存储中非常流行。

**参数**

`json.dumps()` 函数接受几个参数，其中最常用的包括：

- `obj`：要被编码成JSON字符串的Python对象。
- `skipkeys`：默认为`False`。如果设置为`True`，那么字典中不可序列化的键将会被跳过，而不是抛出`TypeError`异常。
- `ensure_ascii`：默认为`True`，意味着输出保证将所有输入的非ASCII字符转义。如果设置为`False`，这些字符将会原样输出。
- `indent`：如果指定了一个非负整数，则JSON数组元素和对象成员将会以这个缩进级别进行美化打印。如果为`None`（默认值），则最紧凑的表示将会被使用。
- `separators`：一个`(item_separator, key_separator)`元组，用来指定分隔符。默认为`(', ', ': ')`，如果`indent`为`None`则为`(',', ':')`。
- `sort_keys`：默认为`False`。如果设置为`True`，则字典的输出将按键排序。

**示例**

```python
import json

# 一个Python字典
data = {
    "name": "John",
    "age": 30,
    "city": "New York"
}

# 将字典转换为JSON字符串
json_str = json.dumps(data, indent=4, ensure_ascii=False)

print(json_str)
```

输出：

```json
{
    "name": "John",
    "age": 30,
    "city": "New York"
}
```

在这个示例中，`json.dumps()` 函数接受了一个字典`data`，并将其转换为一个格式化的JSON字符串。通过设置`indent=4`，输出的JSON字符串是格式化的，每个级别的缩进为4个空格，而`ensure_ascii=False`允许字符串中的非ASCII字符（如中文等）原样输出，而不是被转义。

`json.dumps()` 是处理JSON数据时非常重要的工具，特别是在需要将数据转换为JSON格式进行网络传输或存储时。

## 6、hasattr() 与 isinstance()

`hasattr()` 和 `isinstance()` 是Python中的两个内置函数，它们用于检查对象的特定属性和类型。

### hasattr()

`hasattr(object, name)` 用于判断一个对象是否具有名为`name`的属性或方法。

- `object`：需要检查的对象。
- `name`：一个字符串，表示要检查的属性名称。

如果对象具有该属性，则`hasattr()` 返回`True`；否则返回`False`。

**示例：**

```python
class Sample:
    def __init__(self):
        self.name = "SampleClass"

obj = Sample()
print(hasattr(obj, 'name'))  # 输出: True
print(hasattr(obj, 'age'))   # 输出: False
```

在这个示例中，`Sample`类有一个属性`name`。`hasattr()`函数用于检查`obj`对象是否有`name`和`age`属性。

### isinstance()

`isinstance(object, classinfo)` 用于判断`object`是否是`classinfo`的实例或者是一个（或多个）子类的实例。

- `object`：需要判断的对象。
- `classinfo`：一个类、类型或包含多个类、类型的元组。

如果`object`是`classinfo`的实例或者是一个（或多个）子类的实例，则返回`True`；否则返回`False`。

**示例：**

```python
lst = [1, 2, 3]
print(isinstance(lst, list))      # 输出: True
print(isinstance(lst, (list, dict)))  # 输出: True（因为lst是list的实例）
print(isinstance(lst, dict))      # 输出: False
```

在这个示例中，`lst`是一个列表。`isinstance()`用于检查`lst`是否是`list`或`dict`的实例。

这两个函数在Python编程中非常有用，尤其是在进行类型检查或特性检查时，它们能帮助你编写更加健壯且易于维护的代码

## 7、DFS 深度优先搜索

```python
name = "小明"
performance = "100米跑成绩：10.5秒，1500米跑成绩：3分20秒，铅球成绩：12米。"
category = "搏击"

talents = performance_analyser(name+performance)
print("===talents===")
print(talents)

cache = set()
# 深度优先

# 第一层节点
for k, v in talents.items():
    if v < 3:  # 剪枝
        continue
    leafs = possible_sports(k, category)
    print(f"==={k} leafs===")
    print(leafs)
    # 第二层节点
    for sports in leafs:
        if sports in cache:
            continue
        cache.add(sports)
        suitable = True
        for t, p in talents.items():
            if t == k:
                continue
            # 第三层节点
            if not evaluate(sports, t, p):  # 剪枝
                suitable = False
                break
        if suitable:
            report = report_generator(name, performance, talents, sports)
            print("****")
            print(report)
            print("****")
```

这段代码是一个深度优先搜索（DFS）算法的示例，通常用于遍历或搜索树或图结构。以下是代码块的逐行解释：

1. `cache = set()`：初始化一个空集合`cache`，用于存储已经考虑过的运动项目，以避免重复处理。

2. `for k, v in talents.items():`：在`talents`字典中进行迭代，其中`k`代表某项才能（如速度、耐力等），`v`是对应才能的分档（3代表强，2代表中，1代表弱）。

3. `if v < 3: continue`：这是一个剪枝步骤，如果当前才能的分档小于3（即不是强），则跳过当前迭代，这样只有才能分档为“强”的运动项目才会被进一步考虑。

4. `leafs = possible_sports(k, category)`：调用`possible_sports`函数获取需要当前才能（`k`）强的运动项目列表，`category`是一个特定的体育类别（如搏击）。

5. `print(f"==={k} leafs===")`和`print(leafs)`：打印出当前才能对应的可能适合的运动项目列表。

6. `for sports in leafs:`：遍历上一步获取的运动项目列表。

7. `if sports in cache: continue`：检查当前运动项目是否已在`cache`中，如果是，则跳过以避免重复评估。

8. `cache.add(sports)`：将当前运动项目添加到`cache`集合中，表示这个运动项目已被考虑。

9. `suitable = True`：初始化一个标志变量`suitable`，假定当前运动项目适合当前考虑的个体。

10. `for t, p in talents.items():`：再次在`talents`字典中进行迭代，以评估个体在所有其他才能方面的适应性。

11. `if t == k: continue`：如果迭代到的才能是当前已经作为强项考虑过的才能，则跳过。

12. `if not evaluate(sports, t, p): suitable = False; break`：使用`evaluate`函数评估当前运动是否对于迭代到的其他才能也有高要求；如果个体在这项才能上不满足要求，则设置`suitable`为`False`并结束当前循环。

13. `if suitable:`：如果个体在所有必要才能方面都符合要求，进入这个判断。

14. `report = report_generator(name, performance, talents, sports)`：调用`report_generator`函数生成针对个体和当前考虑的运动项目的分析报告。

15. `print("****")`和`print(report)`和`print("****")`：打印分析报告的开始和结束，并展示报告内容。

总体来看，这段代码意在遍历所有可能的运动项目，并且在考虑个体的才能评分后，为那些个体可能适合的运动项目生成分析报告。
