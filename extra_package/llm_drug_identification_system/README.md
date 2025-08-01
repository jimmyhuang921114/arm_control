# LLM Drug Identification System (LLM-DIS)

LLM-DIS uses ChatGPT to identify drugs, using prompt engineering to let GPT compare drug photos and drug databases to determine whether the drug is correct.

---

## Installation

### 1. Install dependencies
You need to install the openai_api_wrapper library, see[openai_api_wrapper](https://github.com/TKUwengkunduo/openai_api_wrapper.git)
```bash
git clone https://github.com/TKUwengkunduo/openai_api_wrapper.git
cd openai_api_wrapper
pip install -e .
```

### 2. Install this repository 
```bash
cd ..
git clone https://github.com/TKUwengkunduo/llm_drug_identification_system.git
```



---

## Execution example

### 1. Compile package
```bash
cd llm_drug_identification_system/ros2_ws
colcon build
source install/setup.bash
```

### 2. Run Drug Identify Service
```bash
ros2 run dis_service llm_dis
```

### 3. Run test node
```bash
ros2 run dis_service test ../test_data/S__170033155_0.jpg ../drug_database/no000001.yaml
ros2 run dis_service test ../test_data/S__170033158_0.jpg ../drug_database/no000001.yaml
```

#### You will see information similar to the following
![Output example](<image/Output example.png>)