# ROS 프로젝트 1조 저장소. 팀 BLACK PIG
![ROS2](https://img.shields.io/badge/ROS2-jazzy-blue?logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?style=flat-square&logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=flat-square&logo=python&logoColor=white)\
![SLAM&Navigation](https://img.shields.io/badge/SLAM&Navigation-grey?logo=ros&logoColor=white)
![yolo](https://img.shields.io/badge/yolo-grey?logo=yolo&logoColor=white)

<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/addinedu-ros-8th/ros-repo-1">
    <img src="https://github.com/addinedu-ros-8th/ros-repo-1/blob/main/careGiver.gif" alt="Logo" width="700px" height="300px">
  </a>

  <h1 align="center">요양병원 AI 보조 시스템(Nursing AI Assistant System)</h1>

  <p align="center">
    <a href="https://docs.google.com/presentation/d/1I2sNBpPB-KJUQkXDyhFU6J7U_7m9f__Qezk_5R1yaMA/edit?slide=id.g35a51b25e1e_0_0#slide=id.g35a51b25e1e_0_0">Presentation</a>
  </p>
</p>

<hr>

<!-- ABOUT THE PROJECT -->
## Overview


---
<!-- ABOUT THE PROJECT -->
## Overview


ROS 기반 요양보호사 보조 로봇
- **업무 부담 감소** :  단순 반복적이거나 시간이 오래 걸리는 업무를 로봇이 대신 수행(e.g. 물품 운반, 순찰)
- **정확하고 안전한 작업** : 센서와 제어 시스템을 통해 낙상 등의 안전사고 위험 감지  
- **돌봄 서비스** : LLM을 이용하여 정서적 대화를 통해 환자 개개인에게 관심과 맞춤형 서비스 제공 


<br>

| position | name | job |
|:-----:|------|-----|
| leader | 이정림 |   |   
| worker | 심재헌 |   |   
| worker | 신동철 |   |    
| worker | 황한문 |   |    

Project Period: 2025.04.16~2025.05.27
<br>

## Requirements
- ROS2(Jazzy): https://docs.ros.org/en/jazzy/index.html
- Robot Courtesy by <a href="https://pinklab.art/?page_id=5849"> PINKLAB </a>
<img src="https://github.com/user-attachments/assets/cba18e75-6183-4275-9e25-5836891e04e6" width="300">


## Installation
1. Create Python env:
``` 
    python3 -m venv <name>
    source ~/venv/<name>/bin/activate
```
2. Clone this repository:
``` 
    git clone https://github.com/addinedu-ros-8th/deeplearning-repo-1.git
    code deeplearning-repo-1
```
3. Install dependencies:
 ```
     pip install -r requirements.txt
 ```

## Usage

Run the main script:

```bash
    ros2 run nuri_bot main_node
    ros2 run ai_server server 
```

## Project Structure
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/f8328f8f-84b7-4b65-892a-0dbe00d41cb3" width="300"></td>
    <td><img src="https://github.com/user-attachments/assets/b32a9728-d3cf-4545-bb27-ef4b69a7ec96" width="300"></td>
    <td><img src="https://github.com/user-attachments/assets/b9585b2b-6db7-4c05-a505-c576dede9ad1" width="300"></td>
  </tr>
  <tr>
    <td align="center">Ai_server</td>
    <td align="center">nuri_bot</td>
    <td align="center">nuri_io_controller</td>
  </tr>
</table>


## Design

### Architecture  
#### 1) System Atchitecture
<img src="https://github.com/user-attachments/assets/6a3c3848-cbd6-4c5f-b20f-647442e2d375" width="500">

#### 2) ERD 
<img src="https://github.com/user-attachments/assets/02708103-1585-4e23-9ed0-a95824b2a708" width="900">

 

