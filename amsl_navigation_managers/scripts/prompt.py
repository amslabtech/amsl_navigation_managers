from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI
import get_data

llm = ChatOpenAI(temperature=0.1,                                     # 値が大きいほど出力のランダム性が大きくなる
                openai_api_base="http://dgx1:2718",                   # サーバURL
                openai_api_key="dummy")                               # 適当なキーを入力

# 変数
filename = "/home/user/ws/src/advanced_navigation_launcher/map/graph/ikuta_graph.yaml"
ids = []
labels = []
label_list = ""

# コマンドプロンプトからリクエストを取得
q = input("Please enter your request: ")

# yamlファイルからIDとlabelを取得
ids, labels = get_data.from_yaml(filename)
for i in range(len(ids)):
    label_list += str(ids[i]) + "." + labels[i] + ", "

prompt = ChatPromptTemplate.from_messages(
    [
        (
            "system",
            "You are helpfull assistant that select the most appropriate of the options shown now and answer with the number associated with the option : {choices}",
        ),
        ("human", "{request}")
    ]
)

chain = prompt | llm
answer = chain.invoke(
    {
        "choices": f"{label_list}",
        "request": f"{q}",
    }
)

answer = int(answer.content)
print(answer)
