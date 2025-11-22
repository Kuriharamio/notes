# C++部分知识：

## 1、智能指针

### ①概述

- 指针
  - 原始指针（raw_ptr）
  - 智能指针
    - 不需要手动释放内存
    - 并不是所有的指针都可以封装成智能指针。很多时候原始指针更方便
- 种类
  - `std::unique_ptr`
  - `std::shared_ptr`
  - `std::weak_ptr` 



### ②独占指针：std::unique_ptr

- 特点
  - 在任何给定的时刻，只能有一个指针管理内存
  - 当指针超出作用域时，内存将自动释放
  - 不可 Copy，只可以 Move

- 创建方式

  - 通过已有裸指针创建

  - 通过 new 来创建

  - 通过 `std::make_unique` 创建

    ```Cpp
    //cat.h
    #ifndef CAT_H
    #define CAT_H
    
    #include <string.h>
    #include <iostream>
    
    class Cat{
    public:
        Cat(std::string name);
        Cat() = default;
        ~Cat();
        
        //->
        void cat_info() const{
            std::cout << "cat info name : " << name << std::endl;
        }
        
        std::string get_name() const{
            return name;
        }
        
        void set_cat_name(const std::string &name){
            this->name = name
        }
        
    private:
        std::string name("Mimi");
    }
    
    
    #endif
    ```

    ```cpp
    //cat.cpp
    #include "cat.h"
    
    Cat::Cat(std::string name) : name(name)
    {
        std::cout << "Constructor of Cat : " << name << std::endl;
    }
    
    Cat::~Cat()
    {
        std::cout << "Destructor of Cat : " << name << std::endl;
    }
    
    ```

    ```cpp
    #include <iostream>
    #include <memory>
    #include "cat.h"
    using namespace std;
    
    int main(int argc, char *argv[])
    {
        //stack
        Cat c1("OK");
        c1.cat_info();
        {
            Cat c1("OK");
            c1.cat_info();
        }
        
        /******output******   在作用域外立即调用析构函数
        Constructor of Cat : OK
        cat info name : OK
        Constructor of Cat : OK_scope
        cat info name : OK_scope
        Destructor of Cat : OK_scope
        Destructor of Cat : OK
        *******************/
        
        //raw pointer
        Cat *c_p1 = new Cat("yy");
        c_p1->cat_info();
        {
            Cat *c_p1 = new Cat("yy_scope");
            c_p1->cat_info();
            //delete c_p1;  //手动调用析构函数
        }
        //delete c_p1;  //手动调用析构函数
        
        /******output******   未调用析构函数，需要我们手动调用析构函数
        Constructor of Cat : yy
        cat info name : yy
        Constructor of Cat : yy_scope
        cat info name : yy_scope
        *******************/
        
        //unique_pointer的三种创建方式
        //1
        Cat *c_p2 = new Cat("yz");
        std::unique_ptr<Cat> u_c_p1{c_p2};
        delete c_p2;
        c_p2 = nullptr;
        u_c_p2->cat_info();
        //c_p2->cat_info();
        //u_c_p2->cat_info();
        //c_p2->set_cat_name("ok");
        //u_c_p2->cat_info();
        
        //2
        std::unique_ptr<Cat> u_c_p3 {new Cat("dd")};
        u_c_p3->cat_info();
        
        //3 推荐
        std::unique_ptr<Cat> u_c_p4 = make_unique<Cat>();
        u_c_p4->cat_info();
        
        
        return 0;
    }
    
    ```

    

- 优点

  - 可以通过 `get()` 获取地址

    ```cpp
    std::unique_ptr<int> u_i_p3{new int(100)};
    cout << "int address : " << u_i_p3.get() << endl;
    ```

    

  - 实现了 -> 与 *

    - 可通过 -> 调用成员函数
    - 可通过 * 调用 dereferencing （解引用）



### ③计数指针 shared_ptr

- 特点
  - 与 `unique_ptr` 不同，可以共享数据
  - `shared_ptr` 创建了一个计数器与类对象所指的内存相关联
  - Copy 则计数器加一，Delete 则计数器减一
  - api 为 `use_count()`

- 创建

  ```cpp
  std::shared_ptr<int> i_p_1 = make_shared<int>(10);
  cout << "value : " << *i_p_1 << endl;
  
  //copy
  std::shared_ptr<int> i_p_2 = i_p_1;
  cout << "use count : " << i_p_1.use_count() << endl;
  
  //改变 *i_p_2 的值也会改变 *i_p_1 的值，但删除的时候只删除其自身
  ```

  

## 2、vector

`vector`包含着一系列连续存储的元素,其行为和数组类似。访问Vector中的任意元素或从末尾添加元素都可以在`常量级时间复杂度`内完成，而查找特定值的元素所处的位置或是在Vector中插入元素则是`线性时间复杂度`。

构造函数:

- `vector();` 无参数 - 构造一个空的vector

- `vector(size_type num);` 数量(num) - 构造一个大小为num，值为Type默认值的Vector

- `vector(size_type num, const TYPE &val);` 数量(num)和值(val) - 构造一个初始放入num个值为val的元素的Vector

- `vector(const vector &from);` vector(from) - 构造一个与vector from 相同的vector

- `vector(input_iterator start, input_iterator end);` 迭代器(start)和迭代器(end) - 构造一个初始值为[start,end)区间元素的Vector(注:半开区间).

- ```
  vector(initializer_list<value_type> il, const allocator_type& alloc = allocator_type());
  ```

   

  C++11新提供的方法，类似如下方式：

  - `std::vector<int>a{1, 2, 3, 4, 5};`
  - `std::vector<int>a = {1, 2, 3, 4, 5};`

常用API：

- Operators : 对vector进行赋值或比较
  - `v1 == v2`
  - `v1 != v2`
  - `v1 <= v2`
  - `v1 >= v2`
  - `v1 < v2`
  - `v1 > v2`
  - `v[]`
- `assign()`对Vector中的元素赋值
- `at()` : 返回指定位置的元素
- `back()` : 返回最末一个元素
- `begin()` : 返回第一个元素的迭代器
- `capacity()` : 返回vector所能容纳的元素数量(在不重新分配内存的情况下）
- `clear()` : 清空所有元素
- `empty()` : 判断Vector是否为空（返回true时为空）
- `end()` : 返回最末元素的迭代器(译注:实指向最末元素的下一个位置)
- `erase()` : 删除指定元素
- `front()` : 返回第一个元素
- `get_allocator()` : 返回vector的内存分配器
- `insert()` : 插入元素到Vector中
- `max_size()` : 返回Vector所能容纳元素的最大数量（上限）
- `pop_back()` : 移除最后一个元素
- `push_back()` : 在Vector最后添加一个元素
- `rbegin()` : 返回Vector尾部的逆迭代器
- `rend()` : 返回Vector起始的逆迭代器
- `reserve()` : 设置Vector最小的元素容纳数量
- `resize()` : 改变Vector元素数量的大小
- `size()` : 返回Vector元素数量的大小
- `swap()` : 交换两个Vector

```cpp
#include <vector>
using namespace std;

//创建
vector<int> a;  //开一个vector，没有容量

vector<int> a(n);  //开一个大小为n的vector，值为初始值(0或其他)
vector<int> a(n,x);	//开一个大小为n的vector，并全部复制为x

vector<int> b(a);  //开一个vector名字为b并将a的值全部复制给b
vector<int> b(a.begin(),a.end());  //同上
vector<int> b(&a[0],&a[5]);  //同上

//使用
a[i];  //直接访问a中第i个元素
a.at(i);  //同上
a.empty();  //判断是否为空，是则返回true，否则返回false
a.size();  //返回a中的所存储元素的数量
a.push_back(x);  //将x压入a的末尾
a.pop_back();  //将a的末尾元素出栈(返回值为void类型)
a.front();  //返回第一个元素
a.clear();  //清除a中所有的元素
a.insert(it,x);  //在it前添加x
a.insert(it,n,x);  //在it前添加n个x
a.max_size();  //返回vector的最大容量
a.capacity();  //返回a所能容纳的最大元素值
a.erase(it);  //删除it这个迭代器所指向的值
a.erase(first,last);  //删除从[first，last)的值
a.resize(n);  //改变长度，超过的话则删除多余部分，少的话则增添默认值
a.resize(n,x);  //同上，默认值改为x
a.assign(first,last);  //a中替换first，last，first到last这个区间的值不能为a



```

