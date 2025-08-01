from openai_client import OpenAIClient

client = OpenAIClient(model="gpt-4o-mini", temperature=0.1)

yaml_path = "./drug_database/no000001.yaml"
image_path = "./test_data/S__170033155_0.jpg"

# 讀入 YAML 原始文字
with open(yaml_path, encoding="utf-8") as f:
    yaml_text = f.read()



messages = [
    {"role": "system", "content": "你是一位資深藥學與影像辨識專家，請根據下列藥物的影像和藥物的資料。"},
    # 把 YAML 放在 code block
    {"role": "user", "content": f"以下是藥物的描述（YAML）：/n```yaml/n{yaml_text}```"},
    {"role": "user", "content": [
        {"type": "text", "text": "請根據下列藥物的影像和藥物的資料，判斷圖片中的藥品是否與JSON描述完全一致。如果藥物為正確的請回答'yes'，如果不正確則回答'no'，請不要提供理由或其他文字。"},
        {"type": "image_path", "path": image_path}
    ]}
]

response = client.chat_completion(messages)
print(response.choices[0].message.content)
