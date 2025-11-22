# C++编程小点

[TOC]



## 1. 命名空间 `Namespace`

- 基本目的

  - `namespace` 命名空间用于组织代码并避免命名冲突。当有多个相同名称的函数或变量的时候，命名空间可以帮助区分它们。

    ```cpp
    namespace MyNamespace {
        void MyFunction() {
            // 实现
        }
    }
    ```

    - 在这个示例中，`MyFunction` 是在 `MyNamespace` 命名空间中的，可以通过 `MyNamespace::MyFunction()` 调用。

- 其它情况

  - 匿名命名空间：不提供名称的 `namespace` 则是一个匿名命名空间：

    ```cpp
    namespace {
        void InternalHelperFunction() {
            // 该函数只在本文件内可用
        }
    }
    
    // 在其他文件中无法访问InternalHelperFunction
    ```

    - **文件内部链接**: 匿名命名空间中的所有标识符（函数、变量、类等）只有在定义它们的翻译单元（通常是一个源文件）内可见。这意味着它们不会与其他文件中的同名标识符冲突。
      
- **隐藏实现细节**: 匿名命名空间的使用可以帮助封装实现细节，使得这些标识符不会暴露给其他文件（其他文件无法使用），从而减少了全局命名冲突的风险。


  - 嵌套命名空间：命名空间可以嵌套在其他命名空间中，从而创建层次结构。这有助于进一步组织代码。

    ```cpp
    namespace Outer {
        namespace Inner {
            void function() {
                // 内部函数实现
            }
        }
    }
    
    // 调用嵌套命名空间中的函数
    Outer::Inner::function();
    ```

  - 使用声明和使用指令：可以使用 `using` 语句将命名空间中的特定标识符引入当前作用域，从而简化代码。

    ```cpp
    namespace MyNamespace {
        void func();
    }
    
    using MyNamespace::func;  // 引入特定函数
    
    func();  // 调用 MyNamespace 中的 func
    ```

    使用 `using namespace` 声明可以将整个命名空间引入当前作用域，但可能会引入命名冲突，因此在大型程序中应谨慎使用。

    ```cpp
    using namespace MyNamespace;  // 引入整个命名空间
    ```

  - 默认命名空间：`using` 指令可以设置一个“默认命名空间”，从而减少在调用函数时需要的前缀。

    ```cpp
    namespace A {
        void func();
    }
    
    namespace B {
        void func();
    }
    
    using namespace A;  // 设置默认命名空间
    
    func();  // 调用 A::func()
    ```

  - 命名空间的扩展：可以在不同的文件或代码块中继续定义同一个命名空间。这使得代码可以有组织地分散在多个文件中。

    **File1.cpp**

    ```cpp
    namespace MyNamespace {
        void functionA() {
            // 实现
        }
    }
    ```

    **File2.cpp**
    ```cpp
    namespace MyNamespace {
        void functionB() {
            // 实现
        }
    }
    ```
    在不同的文件中定义了相同的命名空间 `MyNamespace`，可以将 `functionA` 和 `functionB` 组织在一起。

  - 命名空间别名：可以为命名空间创建别名，从而简化代码的书写。

    ```cpp
    namespace LongNamespaceName {
        void function();
    }
    
    namespace LNN = LongNamespaceName;  // 创建别名
    
    LNN::function();  // 使用别名调用函数
    ```

  -  内联命名空间（`inline namespace`）：允许后面定义的函数覆盖之前同名的函数

    ```cpp
    namespace Base {
        void func() {}
    }
    
    inline namespace Version1 {
        void func() { 
            // 新实现 
        }
    }
    
    // 调用时可以直接调用 version1 版本的 func
    func();  // 实际调用的是 Version1::func
    ```

  - 自动构建的命名空间：当在特定作用域（例如类的成员、函数等）中使用命名空间时，C++会自动将它们的内容视为属于该作用域。

    ```cpp
    class MyClass {
    public:
        namespace Inner {
            void method() {
                // ...
            }
        }
    };
    
    // 通过 MyClass::Inner 访问
    MyClass::Inner::method();
    ```

    


## 2. `constexpr` 关键字

- 基本目的

  - `constexpr` 是C++11引入的一个关键字，用于定义常量表达式. 它可以被用来指定变量或函数是常量，通常用于编译时求值。这意味着使用 `constexpr` 定义的变量在编译期就已经确定了值，而不是在运行时才进行计算。
  
    ```cpp
    constexpr int max_size = 100;  // 定义一个编译时常量
    ```

- 其他特性

  - **编译时计算**: 使用 `constexpr` 定义的变量或函数，可以在编译时进行计算，避免了运行时的计算开销。这不仅有助于提高性能，还可以使代码更加安全，因为编译器可以检测出常量表达式的错误。

  - **适用于函数**: `constexpr` 也可以用于函数，意味着这个函数也可以在编译时进行求值。这样，我们可以在使用这些函数作为模板参数或数组大小等地方。例如：

    ```cpp
    constexpr int square(int x) { return x * x; }
    constexpr int result = square(5);  // result 在编译时计算为 25
    ```

  - **需要返回常量**: 在函数被声明为 `constexpr` 的情况下，函数体中的表达式必须能够在编译时求值，且不能包含运行时计算的代码。

  - **可以用于 STL**: `constexpr` 可以与标准模板库一起使用，允许你在编译期创建自定义数据结构。

    ```cpp
    #include <array>

    constexpr std::array<int, 3> createArray() {
        return {1, 2, 3};
    }

    constexpr auto myArray = createArray();
    ```

  - **C++14 及以后的增强**: C++14 增强了 `constexpr` 的功能，允许在 `constexpr` 函数中包含更多的语句（如循环和条件语句）。C++20 进一步扩展了 `constexpr` 的能力，可以用于更复杂的场景，如动态内存分配。

  - **与 `const` 区别**: `const` 定义了一个常量，但它的值可以在运行时确定，而 `constexpr` 的值必须在编译时已知。使用 `constexpr` 可以更强烈地表达意图，这不仅增强了类型安全，也可能提高程序性能。

    ```cpp
    const int runtimeValue = getValue();  // 运行时值
    constexpr int compileTimeValue = 100; // 编译时值
    ```

  - **结构体、类中的使用**: 在 C++11 和更高版本中，可以在结构体或类中定义 `constexpr` 成员函数，使得这些函数也能在编译期被调用。

    ```cpp
    struct Point {
        int x, y;
        
        constexpr Point(int xVal, int yVal) : x(xVal), y(yVal) {}
        
        constexpr int getX() const { return x; }
    };
    
    constexpr Point p(10, 20);
    constexpr int xCoordinate = p.getX(); // 在编译期计算
    ```



## 3. `std::make_tuple` 函数

- 基本目的

  - `std::make_tuple` 是 C++11 中引入的一个函数，用于创建一个 `std::tuple` 对象。`std::tuple` 是一个可以存储不同类型元素的固定大小的集合，通过 `std::make_tuple` 可以方便地将多个值组合在一起，形成一个单一的元组对象。

    ```cpp
    #include <tuple>
    
    auto myTuple = std::make_tuple(1, 2.5, "Hello");
    ```

    上面的代码创建了一个包含整数、浮点数和字符串的元组。

- 其他特性

  - **类型推导**: 使用 `std::make_tuple` 时，类型会根据传入参数自动推导。这使得用户可以在创建元组时不需要显式指定类型，代码更简洁。

    ```cpp
    auto myTuple = std::make_tuple(10, 3.14);  // 推导为 std::tuple<int, double>
    ```

  - **支持不同类型**: `std::tuple` 允许存储任意数量和类型的元素。这使得它非常灵活，可以用于存储复杂的数据结构。

    ```cpp
    std::tuple<int, double, std::string> data = std::make_tuple(42, 3.14, "example");
    ```

  - **与 `std::tie` 配合使用**: `std::make_tuple` 常常与 `std::tie` 一起使用，以便在从函数返回多个值时对它们进行解包。`std::make_tuple` 将多个值打包成一个元组，而 `std::tie` 可以将这个元组的值解包到多个变量中。

    ```cpp
    std::tuple<int, double> computeValues() {
        return std::make_tuple(4, 5.6);
    }

    int a;
    double b;
    std::tie(a, b) = computeValues();  // a = 4, b = 5.6
    ```

  - **助手函数**: `std::make_tuple` 是一个非常有用的助手函数，可用于将多个值打包到一个 `std::tuple` 中。它常用于需要函数返回多个值的场景，提供了简单的语法。

  - **可变参数**: `std::make_tuple` 支持可变参数模板，这意味可以传递任意数量的参数，而不需要事先指定参数的数量。

    ```cpp
    auto myTuple = std::make_tuple(1, 2, 3, 4, 5);  // 打包多个参数
    ```

  - **与 STL 算法结合使用**: `std::tuple` 和 `std::make_tuple` 可以与标准库算法结合使用，轻松处理复杂的数据组合和操作。

    ```cpp
    std::vector<std::tuple<int, std::string>> vec;
    vec.emplace_back(std::make_tuple(1, "apple"));
    vec.emplace_back(std::make_tuple(2, "banana"));
    ```

  - **类型安全**: 使用 `std::make_tuple` 创建的元组是类型安全的，这意味着每个元素都有自己明确的类型，并且类型不匹配时会导致编译错误。

- **总结**: `std::make_tuple` 是一个方便的工具，用于创建元组并简化数据组织结构。它支持类型推导和可变参数，使得创建包含不同类型元素的元组变得直观和简单。同时，在处理多个返回值、集合以及与标准库算法结合时，`std::make_tuple` 提供了极大的灵活性与便利性。



## 4. `std::tie` 函数

- 基本目的

  - `std::tie` 是 C++11 中引入的一个函数，用于创建一个 `std::tuple` 的引用。它允许将多个变量（通常是引用）绑定到一个元组中，从而能够以简单的方式同时处理多个值，特别是在解包（unpacking）多个返回值时非常有用。

    ```cpp
    int a, b, c;
    std::tie(a, b, c) = std::make_tuple(1, 2, 3);
    ```

    在这个示例中，`std::tie` 为 `a`, `b`, `c` 创建了一个引用元组，然后将一个包含三个值的元组解包到这三个变量中。

- 其他特性

  - **解包元组的简便性**: 通过 `std::tie`，可以轻松地将一个元组的值分配给多个变量，而无需手动索引。这种用法特别适合从可以返回多个值的函数中提取值。

    ```cpp
    std::tuple<int, double, std::string> getValues() {
        return std::make_tuple(42, 3.14, "example");
    }

    int a;
    double b;
    std::string c;
    std::tie(a, b, c) = getValues();  // 解包返回的元组
    ```

  - **支持引用**: 与常规的元组不同，`std::tie` 返回的是一个引用的元组，因此在某些情况下，它不会复制被绑定的值，而是直接引用原始变量。这对于性能是有利的，尤其是在处理大对象时。

  - **与 `std::ignore` 结合使用**: 使用 `std::tie` 时，如果不需要某个值，可以使用 `std::ignore`。这允许我们选择性地解包元组中的部分值，而忽略掉不需要的部分。

    ```cpp
    std::tuple<int, double, std::string> values = std::make_tuple(42, 3.14, "example");
    int a;
    std::ignore = std::tie(a, std::ignore, std::ignore) = values;  // 仅提取第一个值
    ```

  - **适用于 `std::array`**: 尽管 `std::tie` 主要与 `std::tuple` 配合使用，但它也可以与其他类似的可解包数据结构（如 `std::array`）结合使用，只要将数组类型转换为 `std::tie` 能接受的形式即可。

  - **搭配 `std::tuple` 和其他标准库算法**: `std::tie` 与其他标准库算法（如 `std::sort`, `std::find_if` 等）结合使用时，可以实现复杂的自定义比较功能，尤其是在需要对多个值进行排序或查找时。

  - **“返回多个值”的替代方案**: 在 C++ 中，`std::tie` 是一个非常方便的方式，可以作为返回多个值的替代方案，尤其是当函数只能返回一个值时。通过返回一个元组，可以通过 `std::tie` 进行解包，简化变量的处理过程。

    ```cpp
    std::tuple<int, double> compute(int x) {
        return std::make_tuple(x * 2, x * 3.14);
    }
    
    int a;
    double b;
    std::tie(a, b) = compute(5);  // a = 10, b = 15.7
    ```

- **总结**: `std::tie` 是 C++ 中一个强大的工具，可以简化多个值处理和解包的工作，提高代码的可读性与可维护性。在函数返回多个值或在需要绑定多个引用时，`std::tie` 显得尤为有用。



## 5. `absl::MutexLock` 的使用

- 基本目的

  - `absl::MutexLock` 是 Google 的 Abseil 库中的一个类，用于实现互斥锁（mutex）的简单和安全的管理。它的主要目的是保护共享资源以避免数据竞争，确保同一时间只有一个线程可以访问被保护的资源。

    ```cpp
    absl::Mutex mutex_;
    {
        absl::MutexLock lock(&mutex_);  // 构造函数获取锁
        // ... 访问和修改共享资源
    }  // 离开作用域，析构函数释放锁
    ```

- 其他特性

  - **RAII（资源获取即初始化）**: `absl::MutexLock` 遵循 RAII 原则，创建 `MutexLock` 实例时会自动获取锁，当 `MutexLock` 实例超出作用域或被销毁时，会自动释放锁。这样可以避免由于异常或早期返回而导致的锁未释放的问题，有助于减少死锁的风险。

  - **构造和析构**: `absl::MutexLock` 的构造函数会尝试获取给定互斥锁的所有权。如果锁已经被其他线程持有，则当前线程会阻塞，直到锁变为可用。析构函数会释放锁。

    ```cpp
    absl::MutexLock lock(&mutex_);  // 锁定
    // 执行一些操作
    // 退出作用域后自动解锁
    ```

  - **异常安全**: 由于 `absl::MutexLock` 是 RAII 设计，它提供了异常安全性。即使在临界区抛出异常，锁也会被正确释放，避免死锁。

  - **避免死锁**: 使用 `absl::MutexLock` 可以减少因手动管理锁而可能引入的死锁风险。通过遵循 RAII 原则，程序员无需显式调用解锁，减少了出错的机会。

  - **性能优化**: Abseil 库对锁的实现通常进行优化，以提高性能。`absl::Mutex` 是基于平台的 mutex 实现，能在不同操作系统上有效工作。

  - **支持多线程编程**: `absl::MutexLock` 适用于在多线程环境中保护共享数据。通过在访问共享资源的代码段使用 `MutexLock`，可以确保线程安全，防止数据竞争和竞态条件。

  - **与其他同步原语结合使用**: `absl::MutexLock` 可以与其他 Abseil 或标准库中的同步原语（如条件变量、读写锁等）结合使用，以提供更复杂的线程同步机制。

  - **性能考量**: 在极高并发的场景中，应考虑锁的粒度，尽量缩小临界区（即获取锁的代码区域），以提高性能。

- **示例用法**:

```cpp
#include "absl/synchronization/mutex.h"

class SharedResource {
public:
    void SafeAccess() {
        absl::MutexLock lock(&mutex_);  // 锁定互斥量
        // 访问和修改其他共享资源
    }

private:
    absl::Mutex mutex_;  // 定义互斥量
};
```

- **总结**: `absl::MutexLock` 是一个高效、安全的互斥锁管理工具，适用于多线程编程。通过 RAII 原则，它确保在进入临界区时获取锁，并在退出临界区自动释放锁，极大地降低了死锁和数据竞争的风险，使得多线程编程变得更简单和安全。





## 6. `std::set` 的使用

- 基本目的

  - `std::set` 是 C++ 标准库中的一个关联容器，用于存储唯一的元素。它根据特定的比较准则（默认是小于运算符 `<`）自动对元素进行排序，因此在插入时可以确保元素的唯一性和顺序。这使得 `std::set` 非常适合需要快速查找、插入和删除操作的场景。

    ```cpp
    #include <set>
    #include <iostream>
    
    int main() {
        std::set<int> numbers;
        numbers.insert(3);
        numbers.insert(1);
        numbers.insert(2);
    
        for (const auto& num : numbers) {
            std::cout << num << " ";  // 输出: 1 2 3
        }
        return 0;
    }
    ```

- 其他特性

  - **唯一性**: `std::set` 自动保证元素的唯一性，如果试图插入已经存在的元素，插入操作将不起作用，不会抛出异常。

  - **有序性**: `std::set` 中的元素会根据特定的比较准则保持有序，默认情况下是升序。可以通过提供比较函数来自定义排序规则。

    ```cpp
    struct Compare {
        bool operator() (const int& a, const int& b) const {
            return a > b;  // 降序排序
        }
    };

    std::set<int, Compare> reverseSet;
    reverseSet.insert(1);
    reverseSet.insert(3);
    reverseSet.insert(2);
    // reverseSet 的顺序为: 3 2 1
    ```

  - **复杂度**: `std::set` 的插入、删除和查找操作的时间复杂度为 O(log n)，这是因为它通常基于红黑树（一种自平衡的二叉搜索树）实现。

  - **迭代器**: `std::set` 支持双向迭代器，可以使用 `begin()`、`end()` 来遍历容器中的元素。

    ```cpp
    for (auto it = numbers.begin(); it != numbers.end(); ++it) {
        std::cout << *it << " ";  // 输出: 1 2 3
    }
    ```

  - **成员函数**: `std::set` 提供许多有用的方法，例如 `find()` 查找特定元素，`count()` 统计某元素出现的次数（在 set 中只能是 0 或 1），`erase()` 删除指定元素等。

    ```cpp
    if (numbers.find(2) != numbers.end()) {
        std::cout << "2 is in the set." << std::endl;  // 输出: 2 is in the set.
    }
    ```

  - **性能考量**: `std::set` 适用于较大的数据集，但若数据频繁变化且对有序性要求较低，考虑使用 `std::unordered_set`（基于哈希表实现）可能会提高性能，尤其是在查找和插入操作的效率上。

  - **容器兼容性**: `std::set` 也支持与 STL 算法良好兼容，如 `std::sort()`、`std::copy()`，尽管严格来说，`std::set` 本身是已排序的，所以不需要单独调用排序。

- **示例用法**:

```cpp
#include <set>
#include <iostream>

int main() {
    std::set<std::string> fruits;
    fruits.insert("apple");
    fruits.insert("banana");
    fruits.insert("cherry");

    if (fruits.insert("banana").second) {
        std::cout << "Inserted banana!" << std::endl;  // 这个输出不会执行
    } else {
        std::cout << "Banana is already in the set." << std::endl;  // 输出: Banana is already in the set.
    }

    // 遍历并输出所有水果
    for (const auto& fruit : fruits) {
        std::cout << fruit << " ";  // 输出: apple banana cherry
    }
    return 0;
}
```

- **总结**: `std::set` 是一个强大的工具，适用于需要保持唯一元素及其有序性的场景。通过高级的插入、查找和删除操作支持，它使得开发者能够高效地管理集合数据。对于需要快速查找但不关心顺序的情况，可以考虑使用 `std::unordered_set`。





## 7. 双冒号 `::` 的使用

### 1.**基本目的**

- 在 C++ 中，双冒号 `::` 是作用域解析运算符（scope resolution operator），主要用于指定命名空间、类或结构体中的成员，以及全局作用域的访问。它允许程序员清晰地访问类成员、命名空间或全局变量，避免名字冲突。

### 2. 使用场景

#### 2.1. 命名空间

- **命名空间** 是一个组织代码及避免名称冲突的工具。当多个库或模块定义了相同名称的函数或变量时，可以通过命名空间区分它们。

```cpp
#include <iostream>

namespace MyNamespace {
    void display() {
        std::cout << "Hello from MyNamespace!" << std::endl;
    }
}

int main() {
    MyNamespace::display(); // 使用命名空间中的函数
    return 0;
}
```

#### 2.2. 类和结构体成员访问

- 通过双冒号可以访问类的静态成员或类的非静态成员函数。在类体外可以用 `ClassName::member` 的格式来调用。

```cpp
#include <iostream>

class MyClass {
public:
    static int staticValue;
    void show() {
        std::cout << "Instance method called." << std::endl;
    }
};

int MyClass::staticValue = 5; // 定义和初始化静态成员

int main() {
    std::cout << "Static value: " << MyClass::staticValue << std::endl; // 访问静态成员
    MyClass obj;
    obj.show(); // 调用实例方法
    return 0;
}
```

#### 2.3. 全局作用域

- 当局部作用域中存在变量与全局变量同名时，可以使用双冒号访问全局变量。

```cpp
#include <iostream>

int value = 10; // 全局变量

int main() {
    int value = 20; // 局部变量
    std::cout << "Local value: " << value << std::endl; // 访问局部变量
    std::cout << "Global value: " << ::value << std::endl; // 访问全局变量
    return 0;
}
```

#### 2.4. 继承中的作用

- 在继承关系中，双冒号可以用于显示调用基类的成员。

```cpp
#include <iostream>

class Base {
public:
    void show() {
        std::cout << "Base class show" << std::endl;
    }
};

class Derived : public Base {
public:
    void show() {
        std::cout << "Derived class show" << std::endl;
    }
    void display() {
        Base::show(); // 显示调用基类的 show 方法
    }
};

int main() {
    Derived d;
    d.display(); // 输出: Base class show
    return 0;
}
```

### 3. 小结

- **清晰性**: 使用双冒号可以明确指示哪个作用域中的成员被访问，使代码更易于理解。
- **避免命名冲突**: 在大规模项目或使用多个库时，双冒号有效避免了名称冲突，确保了唯一性。

### 4. 示例代码总汇

```cpp
#include <iostream>

namespace MyNamespace {
    void greet() {
        std::cout << "Hello from MyNamespace!" << std::endl;
    }
}

class MyClass {
public:
    static int staticValue;
    void show() {
        std::cout << "Instance method called." << std::endl;
    }
};

int MyClass::staticValue = 42; // 静态成员定义

int main() {
    // 使用命名空间
    MyNamespace::greet();

    // 访问和显示静态成员
    std::cout << "Static value: " << MyClass::staticValue << std::endl;

    // 访问全局变量
    int value = 100; // 局部变量
    std::cout << "Local value: " << value << std::endl;
    std::cout << "Global value: " << ::value << std::endl; // 需要定义全局变量 ; 
    return 0;
}
```

- **总结**: 双冒号 `::` 是一个强大的工具，可以有效地控制变量和函数的作用域，提升代码的可读性与可维护性。在编写复杂的 C++ 程序时，合理使用双冒号可以避免潜在的错误与混淆。



## 8. `explicit` 的使用

### 1.**基本目的**

- `explicit` 是 C++ 中的一个关键字，用于修饰构造函数和转换运算符，以防止它们在不经意间被用于隐式类型转换。通过使用 `explicit`，开发者可以避免不必要或潜在错误的类型转换，从而提高代码的安全性与可读性。

### 2. 使用场景

- **构造函数中的 `explicit`**

  - 当构造函数接受单个参数时，未使用 `explicit` 修饰的构造函数可以被用于自动类型转换，这可能导致意外的行为。例如，当将某个类型的对象传递给一个接受另一个类型作为参数的函数时，编译器可能会执行隐式转换。

#### 2.1. 示例

```cpp
#include <iostream>

class MyClass {
public:
    // 显式构造函数
    explicit MyClass(int x) : value(x) {
        std::cout << "Constructor called with value: " << value << std::endl;
    }

    void display() const {
        std::cout << "Value: " << value << std::endl;
    }

private:
    int value;
};

void func(MyClass obj) {
    obj.display();
}

int main() {
    // MyClass obj1 = 10; // 错误：不能隐式转换
    MyClass obj2(10);  // 正确：显式调用构造函数
    func(obj2);        // 正确：传递 obj2

    return 0;
}
```

在上述示例中，如果构造函数不是显式的，编译器会允许 `MyClass obj1 = 10;` 这样的隐式转换，但使用 `explicit` 后，必须使用 `MyClass obj2(10);` 显式地调用构造函数。

### 3. 转换运算符中的 `explicit`

- 除了构造函数，`explicit` 也可以应用于转换运算符，以避免在不经意间进行隐式转换。

#### 3.2. 示例

```cpp
#include <iostream>

class MyClass {
public:
    explicit operator int() const {
        return value;
    }

    MyClass(int x) : value(x) {}

private:
    int value;
};

int main() {
    MyClass obj(42);
    
    // int num = obj; // 错误：不能隐式转换
    int num = static_cast<int>(obj); // 正确：显式转换
    std::cout << "Converted value: " << num << std::endl;

    return 0;
}
```

在这个例子中，当 `MyClass` 的对象 `obj` 被转换为 `int` 时，必须使用 `static_cast<int>(obj)`，而不能直接进行隐式转换。

### 4. 小结

- **提高代码安全性**: `explicit` 防止了潜在的隐式类型转换，从而降低了错误发生的概率，使得代码更加安全和可读。
  
- **必要性**: 在需要类型转换时，使用 `explicit` 可以让用户明确地知道何时和如何进行转换，而不是依赖于编译器的自动处理。

### 5. 小示例

以下是一个综合示例，展示了如何使用 `explicit` 关键字来改善代码的可读性和安全性：

```cpp
#include <iostream>

class Fraction {
public:
    explicit Fraction(int numerator, int denominator = 1)
        : num(numerator), denom(denominator) {
        std::cout << "Fraction created: " << num << "/" << denom << std::endl;
    }

    void display() const {
        std::cout << "Fraction: " << num << "/" << denom << std::endl;
    }

private:
    int num;
    int denom;
};

void processFraction(Fraction f) {
    f.display();
}

int main() {
    Fraction f1(3); // 显式构造
    processFraction(f1); // 传递对象

    // Fraction f2 = 5; // 错误：不能隐式转换
    Fraction f2(5, 2); // 正确
    processFraction(f2); // 传递对象

    return 0;
}
```

### 6. 总结

- `explicit` 是 C++ 中控制构造函数和转换运算符的重要工具，通过强制性阻止隐式转换，可以提升代码质量和降低错误风险。学习并合理使用 `explicit` 可以帮助程序员编写更清晰和更安全的代码。





## 8. `std::move` 的使用

- **基本目的**

  - `std::move` 是 C++11 引入的一个标准库函数，位于 `<utility>` 头文件中。其主要作用是将一个对象标记为右值，使得该对象可以安全地被移动而不是复制。这在资源管理和性能优化方面非常有用，尤其是在处理动态分配的内存或大对象时。

### 2. 特性与应用场景

- **移动语义**:
  - 使用 `std::move` 可以实现移动语义，这允许资源（如动态内存、文件句柄等）的高效转移。被移动的对象在移动后通常会处于“空”或“无效”状态，可以避免不必要的内存复制和资源开销。

- **避免不必要的复制**:
  - 在函数返回大型对象时，避免不必要的复制开销。如果对象是可移动的，使用 `std::move` 可以直接移动对象的资源，而不是创建其副本。

### 3. 示例用法

#### 3.1. 基本示例

下面是一个简单的示例，展示了如何使用 `std::move` 来实现对象的移动。

```cpp
#include <iostream>
#include <utility>
#include <vector>

class MyClass {
public:
    MyClass() { std::cout << "Constructor called" << std::endl; }
    MyClass(const MyClass&) { std::cout << "Copy Constructor called" << std::endl; }
    MyClass(MyClass&&) noexcept { std::cout << "Move Constructor called" << std::endl; }
    ~MyClass() { std::cout << "Destructor called" << std::endl; }
};

int main() {
    MyClass obj1;                  // 调用构造函数
    MyClass obj2 = std::move(obj1); // 调用移动构造函数
    return 0;
}
```

**输出**:
```
Constructor called
Move Constructor called
Destructor called
Destructor called
```

- 在这个示例中，`obj1` 的资源通过 `std::move` 被移动到 `obj2`，而不是使用复制操作。

#### 3.2. 在容器中的应用

`std::move` 通常与 STL 容器一起使用，以提高性能。例如，在使用 `std::vector` 时：

```cpp
#include <iostream>
#include <vector>
#include <utility> // for std::move

class MyClass {
public:
    int value;
    MyClass(int v) : value(v) {
        std::cout << "Constructing MyClass with value " << value << std::endl;
    }
    MyClass(MyClass&& other) noexcept : value(other.value) {
        other.value = 0; // 将原对象的状态设置为空或默认值
        std::cout << "Moving MyClass with value " << value << std::endl;
    }
};

int main() {
    std::vector<MyClass> vec;
    vec.reserve(3); // 预留空间，避免多次分配

    vec.emplace_back(1);        // 利用构造函数
    vec.emplace_back(2);        // 利用构造函数
    vec.emplace_back(3);        // 利用构造函数

    MyClass obj = std::move(vec.back()); // 移动最后的元素
    std::cout << "Moved object value: " << obj.value << std::endl; // 输出移动对象的值
    return 0;
}
```

**输出**:
```
Constructing MyClass with value 1
Constructing MyClass with value 2
Constructing MyClass with value 3
Moving MyClass with value 3
Moved object value: 3
```

### 4. 重要注意事项

- **只适用于右值**:
  - `std::move` 本身并不移动数据，调用 `std::move` 会将对象转换为右值引用，实际的移动是通过相应的移动构造函数或移动赋值操作符实现的。

- **移动后的状态**:
  - 被移动的对象处于一个未定义状态，应该避免继续使用它，除非它被重新赋值。

- **重载与性能**:
  - 可以通过重载移动构造函数和移动赋值操作符来实现自定义的移动语义，进一步优化性能，尤其是对于资源管理类。

### 5. 示例：自定义移动构造和赋值

```cpp
#include <iostream>
#include <utility>

class Resource {
public:
    Resource() : data(new int[100]) { std::cout << "Resource acquired." << std::endl; }
    Resource(Resource&& other) noexcept : data(other.data) {
        other.data = nullptr; // 避免释放同一资源
        std::cout << "Resource moved." << std::endl;
    }
    ~Resource() { 
        delete[] data;
        std::cout << "Resource released." << std::endl; 
    }

private:
    int* data;
};

int main() {
    Resource res1;                // 资源被获取
    Resource res2 = std::move(res1); // 资源被移动
    return 0;
}
```

**输出**:
```
Resource acquired.
Resource moved.
Resource released.
```

### 6. 总结

- `std::move` 是 C++11 引入的高效工具，帮助实现移动语义。它能够减少复制开销，提高性能，尤其在处理大对象和资源管理时。
- 理解 `std::move` 的使用和它对资源管理的影响，可以帮助开发者编写更高效和可维护的 C++ 代码。在使用过程中，应注意被移动对象的状态和生命周期管理。



## 9. 模板（Template）的定义

### 1.**基本目的**

- 在 C++ 中，模板是一种强大的语言特性，用于定义泛型数据结构和函数，以便在编译时生成适用于不同数据类型的代码。模板允许程序员编写一次代码，却可以对多种类型生效，从而提升代码的复用性和可维护性。

### 2. 模板的类型

C++模板主要分为两种类型：**函数模板**和**类模板**。

#### 2.1. 函数模板

- **定义**: 函数模板是用来创建泛型函数的，通过模板参数在编译时生成对应类型的函数实例。

##### 示例：

```cpp
#include <iostream>

// 函数模板定义
template <typename T>
T add(T a, T b) {
    return a + b;
}

int main() {
    std::cout << "Int addition: " << add(3, 4) << std::endl;       // 调用 int 版本
    std::cout << "Double addition: " << add(3.5, 2.5) << std::endl; // 调用 double 版本
    return 0;
}
```

**输出**:
```
Int addition: 7
Double addition: 6
```

在这个示例中，`add` 是一个函数模板，能够对不同数据类型进行加法操作。

#### 2.2. 类模板

- **定义**: 类模板用于创建泛型类，其成员函数、属性等可以根据模板参数变化。

##### 示例：

```cpp
#include <iostream>

// 类模板定义
template <typename T>
class Box {
public:
    Box(T item) : item(item) {}
    
    T getItem() const {
        return item;
    }

private:
    T item;
};

int main() {
    Box<int> intBox(123); // 实例化一个整数类型的 Box
    Box<std::string> strBox("Hello, Templates!"); // 实例化一个字符串类型的 Box
    
    std::cout << "Int Box contains: " << intBox.getItem() << std::endl; 
    std::cout << "String Box contains: " << strBox.getItem() << std::endl; 

    return 0;
}
```

**输出**:
```
Int Box contains: 123
String Box contains: Hello, Templates!
```

在这个示例中，`Box` 是一个类模板，可以容纳不同类型的对象（`int` 和 `std::string`）。

### 3. 模板的优点

- **代码复用**: 模板允许一段代码适用于多种数据类型，避免了重复编写相同的逻辑。
  
- **类型安全**: 模板在编译期间进行类型检查，能够实现类型安全。

- **性能优化**: 由于模板在编译期生成实际类型的代码，通常不会有运行时的性能损失，每个类型都有独立的代码实例。

### 4. 模板的注意事项

- **类型推导**: 模板参数可以通过类型推导进行自动确定，但也可以显式指定。

- **特化**: C++支持模板特化，允许为特定类型提供定制实现。

  #### 4.1. 示例（特化）：

  ```cpp
  #include <iostream>
  
  // 类模板定义
  template <typename T>
  class Box {
  public:
      Box(T item) : item(item) {}
      
      T getItem() const {
          return item;
      }
  
  private:
      T item;
  };
  
  // 模板特化，仅用于字符串类型
  template <>
  class Box<std::string> {
  public:
      Box(std::string item) : item(item) {}
  
      std::string getItem() const {
          return "String: " + item;
      }
  
  private:
      std::string item;
  };
  
  int main() {
      Box<int> intBox(123);
      Box<std::string> strBox("Hello");
  
      std::cout << "Int Box contains: " << intBox.getItem() << std::endl; 
      std::cout << "String Box contains: " << strBox.getItem() << std::endl; 
  
      return 0;
  }
  ```

### 5. 小结

- **模板是C++的泛型编程工具**，通过定义函数模板和类模板，使得代码更加灵活，能够适应多种数据类型。
- 在编写复杂或高度重用的代码时，模板是不可或缺的工具，有助于提高程序的可读性与可维护性。正确理解和使用模板特性，将大大增强开发者的编程能力。

### 6. 示例代码总汇

```cpp
#include <iostream>
#include <string>

// 函数模板
template <typename T>
T add(T a, T b) {
    return a + b;
}

// 类模板
template <typename T>
class Box {
public:
    Box(T item) : item(item) {}

    T getItem() const {
        return item;
    }

private:
    T item;
};

// 模板特化
template <>
class Box<std::string> {
public:
    Box(std::string item) : item(item) {}

    std::string getItem() const {
        return "String: " + item;
    }

private:
    std::string item;
};

int main() {
    // 函数模板
    std::cout << "Int addition: " << add(3, 4) << std::endl;       
    std::cout << "Double addition: " << add(3.5, 2.5) << std::endl; 

    // 类模板
    Box<int> intBox(123);
    Box<std::string> strBox("Hello Templates");

    std::cout << "Int Box contains: " << intBox.getItem() << std::endl;
    std::cout << "String Box contains: " << strBox.getItem() << std::endl;

    return 0;
}
```

- **总结**: 在处理多种数据类型时，模板为程序员提供了强大的灵活性与性能，能够有效支持泛型编程。







## 10、智能指针

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

  

## 11、vector

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