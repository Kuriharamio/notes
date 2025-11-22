## Introduction

注册 ChatGPT、Copilot 教程：https://agiclass.feishu.cn/docx/Jt8ydP0RroFCPaxcWGDcUzVrnnd#YtxodWqgdofsa8xb0GOcaD1nny5

OpenAI文档：[Introduction - OpenAI API](https://platform.openai.com/docs/introduction)

OpenAI API:[API Reference - OpenAI API](https://platform.openai.com/docs/api-reference)

###### 

### 对话产品与大模型

| 国家 | 对话产品           | 大模型         | 链接                           |
| ---- | ------------------ | -------------- | ------------------------------ |
| 美国 | OpenAI ChatGPT     | GPT-3.5、GPT-4 | https://chat.openai.com/       |
| 美国 | Microsoft Copilot  | GPT-4 和未知   | https://copilot.microsoft.com/ |
| 美国 | Google Bard        | Gemini         | https://bard.google.com/       |
| 中国 | 百度文心一言       | 文心 4.0       | https://yiyan.baidu.com/       |
| 中国 | 讯飞星火           | 星火 3.5       | https://xinghuo.xfyun.cn/      |
| 中国 | 智谱清言           | GLM-4          | https://chatglm.cn/            |
| 中国 | 月之暗面 Kimi Chat | Moonshot       | https://kimi.moonshot.cn/      |
| 中国 | MiniMax 星野       | abab6          | https://www.xingyeai.com/      |

##### 

### 用途		

- **舆情分析：**从公司产品的评论中，分析哪些功能/元素是用户讨论最多的，评价是正向还是负向
- **坐席质检：**检查客服/销售人员与用户的对话记录，判断是否有争吵、辱骂、不当言论，话术是否符合标准
- **知识库：**让大模型基于私有知识回答问题
- **零代码开发/运维：**自动规划任务，生成指令，自动执行
- **AI 编程：**用 AI 编写代码，提升开发效率

##### 

### 典型业务架构

![business_arch](D:\Study\Material\AI\资料\课程\lecture-notes\01-intro\business_arch.webp)

​	Agent 还太超前，Copilot 值得追求。

##### 

### 技术架构

这套生成机制的内核叫「Transformer 架构」

| 架构        | 设计者                                               | 特点                                     | 链接                                                         |
| ----------- | ---------------------------------------------------- | ---------------------------------------- | ------------------------------------------------------------ |
| Transformer | Google                                               | 最流行，几乎所有大模型都用它             | [OpenAI 的代码](https://github.com/openai/finetune-transformer-lm/blob/master/train.py) |
| RWKV        | [PENG Bo](https://www.zhihu.com/people/bopengbopeng) | 可并行训练，推理性能极佳，适合在端侧使用 | [官网](https://www.rwkv.com/)、[RWKV 5 训练代码](https://github.com/BlinkDL/RWKV-LM/tree/main/RWKV-v5) |
| Mamba       | CMU & Princeton University                           | 性能更佳，尤其适合长文本生成             | [GitHub](https://github.com/state-spaces/mamba)              |

#### 纯 Prompt

就像和一个人对话，你说一句，ta 回一句，你再说一句，ta 再回一句……

![prompt_arch](D:\Study\Material\AI\资料\课程\lecture-notes\01-intro\prompt_arch.png)



#### Agent + Function Calling

- Agent：AI 主动提要求
- Function Calling：AI 要求执行某个函数
- 场景举例：你问过年去哪玩，ta 先反问你有多少预算

![func_arch](D:\Study\Material\AI\资料\课程\lecture-notes\01-intro\func_arch.png)



#### RAG（Retrieval-Augmented Generation）

- Embeddings：把文字转换为更易于相似度计算的编码。这种编码叫**向量**
- 向量数据库：把向量存起来，方便查找
- 向量搜索：根据输入向量，找到最相似的向量
- 场景举例：考试时，看到一道题，到书上找相关内容，再结合题目组成答案。然后，就都忘了

![embeddings_arch](D:\Study\Material\AI\资料\课程\lecture-notes\01-intro\embeddings_arch.png)



#### Fine-tuning

努力学习考试内容，长期记住，活学活用。

![tech_arch](D:\Study\Material\AI\资料\课程\lecture-notes\01-intro\tech_arch.png)

##### 

### 如何选择技术路线

面对一个需求，如何选择技术方案？下面是个不严谨但常用思路。

![tech_solution](D:\Study\Material\AI\资料\课程\lecture-notes\01-intro\tech_solution.png)

值得尝试 Fine-tuning 的情况：

1. 提高大模型的稳定性
2. 用户量大，降低推理成本的意义很大
3. 提高大模型的生成速度

基础模型选型，也是个重要因素。合规和安全是首要考量因素。

| 需求             | 国外大模型 | 国产大模型 | 开源大模型 |
| ---------------- | ---------- | ---------- | ---------- |
| 国内 2C          | 🛑          | ✅          | ✅          |
| 国内 2G          | 🛑          | ✅          | ✅          |
| 国内 2B          | ✅          | ✅          | ✅          |
| 出海             | ✅          | ✅          | ✅          |
| 数据安全特别重要 | 🛑          | 🛑          | ✅          |

然后用测试数据，在可以选择的模型里，做测试，找出最优。

##### 

### 生成下一个字（示例代码）

```python
from openai import OpenAI
from dotenv import load_dotenv, find_dotenv
_ = load_dotenv(find_dotenv())

client = OpenAI()

prompt = "今天我很"  

#使用completion，即续写
response = client.completions.create(
    model="gpt-3.5-turbo-instruct",
    prompt=prompt,
    max_tokens=512,
    stream=True
)

for chunk in response:
    print(chunk.choices[0].text, end='')
```

##### 

### 发送一条消息（示例代码）

```python
from openai import OpenAI

# 加载 .env 文件到环境变量
from dotenv import load_dotenv, find_dotenv
_ = load_dotenv(find_dotenv())

# 初始化 OpenAI 服务。会自动从环境变量加载 OPENAI_API_KEY 和 OPENAI_BASE_URL
client = OpenAI()

# 消息
messages = [
    {
        "role": "system",
        "content": "你是AI助手小瓜，是 AGI 课堂的助教。这门课每周二、四上课。"  # 注入新知识
    },
    {
        "role": "user",
        "content": "周末上课吗？"  # 问问题。可以改改试试
    },
]

# 调用 GPT-3.5，使用chat，即对话
chat_completion = client.chat.completions.create(
    model="gpt-3.5-turbo",
    messages=messages
)

# 输出回复
print(chat_completion.choices[0].message.content)
```
