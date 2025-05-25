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
### 프로젝트 주제 및 선정 배경
<img src="https://github.com/user-attachments/assets/10fedff1-ca7e-4a37-8f4f-fa48211b649f" width="400">
<img src="https://github.com/user-attachments/assets/662e9007-1f6f-4530-baf1-40958fe352da" width="400">                

 * 노인 인구 수가 증가하고 그 외 인구수가 감소하며 2024년 12월부터 대한민국은 초고령 사회에 진입                                 
 * 요양보호사 수요는 증가하지만 처우, 업무 환경, 업무 강도의 문제 등으로 인해 요양보호사의 수는 증가하지 않음
<br><br>
 -> **위와 같은 문제를 해결할 방안을 고민해 보고자 프로젝트 구성**


### 시스템 소개
<table>
  <tr>
    <td width="40%">
      <img src="https://github.com/user-attachments/assets/cb6cceb3-02f1-4d24-9dfc-5541dca58e3f" width="100%"><br>
    </td>
    <td width="60%">
      <strong>Nuri System (요양보호사 보조 시스템)</strong><br>
      요양보호사의 육체적, 반복적 작업을 도와주는 시스템<br><br>
      <ul>
        <li>정해진 시간에 순찰하며 인원 확인 및 이상 유무 체크</li>
        <li>삼시세끼 배식 진행</li>
        <li>필요 시 호출 → 대화, 산책, 심리 안정 도모</li>
        <li>돌아다니며 낙상, 화재 등 감지 시 즉시 알림</li>
        <li>건강 상태 모니터링으로 이상 징후 조기 발견</li>
      </ul>
    </td>
  </tr>
</table>

 
### 기능 리스트
| 기능 항목           | 설명 |
|--------------------|------|
| **장애물 회피**       | 주행 시 장애물 감지 후 회피 |
| **순찰**             | 매일 아침/저녁 예약된 시간에 순찰 수행<br>– 인원 확인<br>– 이상 상태 확인 |
| **위험 상황 판단**    | 주행 중 위험 상황 감지 시 알림<br>– 화재<br>– 쓰러짐 |
| **대기장소로 복귀**   | 동작 완료 후 대기 및 충전 스테이션으로 복귀 후 도킹 |
| **호출 응답 주행**    | QR코드 호출 시 해당 장소로 이동<br>– 다용도실<br>– 노인 병실 침대 |
| **인지 재활 치료**    | 대화 및 산책을 통한 고독감 완화, 심리 안정감 제공<br>– 정해진 코스 산책<br>– 대화 |
| **건강 이상 체크**    | 손목밴드를 통한 건강 데이터 업데이트<br>– 심박수, 체온<br>– 산소포화도 |
| **인터페이스**       | 로봇 및 어르신 상태 확인 및 관리<br>– 로봇 현상태 확인<br>– 로봇 호출<br>– 어르신 정보 및 건강 확인 |

### 결과 영상

### 프로젝트 실행
#### Requirements
<table>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/cba18e75-6183-4275-9e25-5836891e04e6" width="100%">
    </td>
    <td width="40%">
      <ul>
        <li>ROS2(Jazzy): <a href="https://docs.ros.org/en/jazzy/index.html">https://docs.ros.org/en/jazzy/index.html</a></li>
        <li>Robot Courtesy by <a href="https://pinklab.art/?page_id=5849">PINKLAB</a></li>
      </ul>
    </td>
  </tr>
</table>

#### Installation
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

#### Usage

Run the main script:

```bash
    ros2 run nuri_bot main_node
    ros2 run ai_server server 
```
<hr>

## 프로젝트 설계
### 시스템 아키텍처

### ER Diagram

<hr>

## 기능 및 기술 설명
### 전체 시나리오

### 기본 주행

### 순찰

### 호출

### 위험 상황 감지

### 산책

### 대화

### 건강 상태 모니터링

<hr>

## 문제 상황 및 해결 방안
### 주행

### AI

### 건강 상태 모니터링

<hr>

## TEAM. BLACK PIG
| position | name | job | contacts |
|:-----:|------|-----|-----|
| leader | 이정림 |   |   |
| worker | 심재헌 |   |   |
| worker | 신동철 |   |   |
| worker | 황한문 |   |   |


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

 

