import os
import json
from datetime import datetime
from typing import Dict, List, Optional

class Student:
    """学生类，用于存储学生基本信息和成绩"""
    
    def __init__(self, student_id: str, name: str, age: int, major: str):
        self.student_id = student_id
        self.name = name
        self.age = age
        self.major = major
        self.grades: Dict[str, float] = {}
        self.enrollment_date = datetime.now().strftime("%Y-%m-%d")
    
    def add_grade(self, subject: str, grade: float) -> None:
        """添加科目成绩"""
        if 0 <= grade <= 100:
            self.grades[subject] = grade
        else:
            raise ValueError("成绩应在0-100之间")
    
    def get_average_grade(self) -> float:
        """计算平均成绩"""
        if not self.grades:
            return 0.0
        return sum(self.grades.values()) / len(self.grades)
    
    def get_grade_summary(self) -> Dict:
        """获取成绩摘要"""
        return {
            "student_id": self.student_id,
            "name": self.name,
            "average_grade": self.get_average_grade(),
            "total_subjects": len(self.grades),
            "highest_grade": max(self.grades.values()) if self.grades else 0,
            "lowest_grade": min(self.grades.values()) if self.grades else 0
        }

class GradeManager:
    """成绩管理器，负责管理多个学生的成绩数据"""
    
    def __init__(self, data_file: str = "students_data.json"):
        self.data_file = data_file
        self.students: Dict[str, Student] = {}
        self.load_data()
    
    def add_student(self, student: Student) -> bool:
        """添加学生"""
        if student.student_id not in self.students:
            self.students[student.student_id] = student
            return True
        return False
    
    def remove_student(self, student_id: str) -> bool:
        """删除学生"""
        if student_id in self.students:
            del self.students[student_id]
            return True
        return False
    
    def get_student(self, student_id: str) -> Optional[Student]:
        """获取学生信息"""
        return self.students.get(student_id)
    
    def update_student_grade(self, student_id: str, subject: str, grade: float) -> bool:
        """更新学生成绩"""
        student = self.get_student(student_id)
        if student:
            student.add_grade(subject, grade)
            return True
        return False
    
    def get_class_ranking(self) -> List[Dict]:
        """获取班级排名"""
        rankings = []
        for student in self.students.values():
            summary = student.get_grade_summary()
            rankings.append(summary)
        
        # 按平均成绩排序
        rankings.sort(key=lambda x: x["average_grade"], reverse=True)
        return rankings
    
    def search_students_by_major(self, major: str) -> List[Student]:
        """按专业搜索学生"""
        result = []
        for student in self.students.values():
            if student.major.lower() == major.lower():
                result.append(student)
        return result
    
    def save_data(self) -> None:
        """保存数据到文件"""
        data = {}
        for student_id, student in self.students.items():
            data[student_id] = {
                "student_id": student.student_id,
                "name": student.name,
                "age": student.age,
                "major": student.major,
                "grades": student.grades,
                "enrollment_date": student.enrollment_date
            }
        
        try:
            with open(self.data_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"保存数据时出错: {e}")
    
    def load_data(self) -> None:
        """从文件加载数据"""
        if not os.path.exists(self.data_file):
            return
        
        try:
            with open(self.data_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            for student_id, student_data in data.items():
                student = Student(
                    student_data["student_id"],
                    student_data["name"],
                    student_data["age"],
                    student_data["major"]
                )
                student.grades = student_data["grades"]
                student.enrollment_date = student_data["enrollment_date"]
                self.students[student_id] = student
                
        except Exception as e:
            print(f"加载数据时出错: {e}")

def generate_sample_data(manager: GradeManager) -> None:
    """生成示例数据"""
    # 创建一些示例学生
    students_data = [
        ("S001", "张三", 20, "计算机科学"),
        ("S002", "李四", 19, "数学"),
        ("S003", "王五", 21, "物理学"),
        ("S004", "赵六", 20, "计算机科学"),
        ("S005", "钱七", 19, "数学")
    ]
    
    subjects = ["高等数学", "英语", "程序设计", "数据结构", "操作系统"]
    
    for stu_data in students_data:
        student = Student(*stu_data)
        # 为每个学生添加随机成绩
        import random
        for subject in subjects:
            student.add_grade(subject, random.randint(60, 100))
        manager.add_student(student)

def display_menu() -> None:
    """显示菜单"""
    print("\n" + "="*50)
    print("           学生成绩管理系统")
    print("="*50)
    print("1. 添加学生")
    print("2. 删除学生")
    print("3. 录入成绩")
    print("4. 查询学生成绩")
    print("5. 查看班级排名")
    print("6. 按专业查询学生")
    print("7. 生成示例数据")
    print("0. 退出系统")
    print("="*50)

def main():
    """主函数"""
    manager = GradeManager()
    
    while True:
        display_menu()
        choice = input("请选择操作 (0-7): ").strip()
        
        if choice == "1":
            # 添加学生
            student_id = input("请输入学号: ")
            name = input("请输入姓名: ")
            try:
                age = int(input("请输入年龄: "))
                major = input("请输入专业: ")
                student = Student(student_id, name, age, major)
                if manager.add_student(student):
                    print("学生添加成功!")
                else:
                    print("该学号已存在!")
            except ValueError:
                print("年龄输入无效!")
        
        elif choice == "2":
            # 删除学生
            student_id = input("请输入要删除的学号: ")
            if manager.remove_student(student_id):
                print("学生删除成功!")
            else:
                print("未找到该学号的学生!")
        
        elif choice == "3":
            # 录入成绩
            student_id = input("请输入学号: ")
            subject = input("请输入科目名称: ")
            try:
                grade = float(input("请输入成绩: "))
                if manager.update_student_grade(student_id, subject, grade):
                    print("成绩录入成功!")
                else:
                    print("未找到该学号的学生!")
            except ValueError:
                print("成绩输入无效!")
        
        elif choice == "4":
            # 查询学生成绩
            student_id = input("请输入学号: ")
            student = manager.get_student(student_id)
            if student:
                print(f"\n学号: {student.student_id}")
                print(f"姓名: {student.name}")
                print(f"年龄: {student.age}")
                print(f"专业: {student.major}")
                print(f"入学日期: {student.enrollment_date}")
                print("成绩列表:")
                for subject, grade in student.grades.items():
                    print(f"  {subject}: {grade}")
                print(f"平均成绩: {student.get_average_grade():.2f}")
            else:
                print("未找到该学号的学生!")
        
        elif choice == "5":
            # 查看班级排名
            rankings = manager.get_class_ranking()
            if rankings:
                print("\n班级排名:")
                print("-" * 60)
                print(f"{'排名':<4} {'学号':<10} {'姓名':<10} {'平均成绩':<10} {'科目数':<8}")
                print("-" * 60)
                for i, rank in enumerate(rankings, 1):
                    print(f"{i:<4} {rank['student_id']:<10} {rank['name']:<10} "
                          f"{rank['average_grade']:<10.2f} {rank['total_subjects']:<8}")
            else:
                print("暂无学生数据!")
        
        elif choice == "6":
            # 按专业查询学生
            major = input("请输入专业名称: ")
            students = manager.search_students_by_major(major)
            if students:
                print(f"\n{major}专业的学生列表:")
                for student in students:
                    print(f"  {student.student_id} - {student.name} (平均成绩: {student.get_average_grade():.2f})")
            else:
                print(f"未找到{major}专业的学生!")
        
        elif choice == "7":
            # 生成示例数据
            generate_sample_data(manager)
            print("示例数据生成完成!")
        
        elif choice == "0":
            # 退出系统
            manager.save_data()
            print("数据已保存，感谢使用!")
            break
        
        else:
            print("无效选项，请重新选择!")

if __name__ == "__main__":
    main()