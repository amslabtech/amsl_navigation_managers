from llama_cpp import Llama
import get_data
import re

llm = Llama(model_path="/home/amsl/llama.cpp/models/llama-2-7b-chat.q4_K_M.gguf")
# 変数
filename = "/home/amsl/catkin_ws/src/rwrc23/map/graph/ikuta_graph.yaml"
ids = []
labels = []
choices = ""

# コマンドプロンプトからリクエストを取得
q = input("Please enter your request: ")

# yamlファイルからIDとlabelを取得
ids, labels = get_data.from_yaml(filename)
for i in range(len(ids)):
    choices += str(ids[i]) + "." + labels[i] + ", "

prompt = (
    "User: "
    + q
    + " Please select the most appropriate of the options shown now and answer with the number associated with the option. "
    + choices
    + "\n"
    + "Assistant: "
)

output = llm(
    prompt,
    temperature=0.1,
    stop=["User:", "Assistant:", "\n"],
    echo=True,
)

answer_text = output["choices"][0]["text"].replace(prompt, "")
print(answer_text)
answer = re.sub(r"\D", "", answer_text)
