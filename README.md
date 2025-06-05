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
<table>
  <tr>
    <td width="50%">
      <img src="https://github.com/user-attachments/assets/050bb257-d4b0-4d40-a96b-9316eebbc71d"><br>
    </td>
    <td width="50%">
      <img src="https://github.com/user-attachments/assets/dbaf59f4-1230-4a45-b7aa-616ddc4eaeaa"><br>
    </td>
      <tr>
    <td width="50%">
      <img src="https://github.com/user-attachments/assets/6ddd0c79-31ff-4e19-bce2-78e61f1504b9"><br>
    </td>
    <td width="50%">
      <img src="https://github.com/user-attachments/assets/6875798f-0265-47c4-839d-0f97f8cc6e5b"><br>
    </td>
  </tr>
</table>
<table>
  <tr>
    <td width="50%">
      <img src="https://github.com/user-attachments/assets/079f564e-58db-4258-8f19-9ea1735688fb"><br>
    </td>
    <td width="50%">
      <img src="https://github.com/user-attachments/assets/108ed4c4-0486-484e-8143-71740fdc44c0"><br>
    </td>
  </tr>
</table>

#### 프로젝트 주제 및 선정 배경
<img src="https://github.com/user-attachments/assets/10fedff1-ca7e-4a37-8f4f-fa48211b649f" width="400">
<img src="https://github.com/user-attachments/assets/662e9007-1f6f-4530-baf1-40958fe352da" width="400">                

 * 노인 인구 수가 증가하고 그 외 인구수가 감소하며 2024년 12월부터 대한민국은 초고령 사회에 진입                                 
 * 요양보호사 수요는 증가하지만 처우, 업무 환경, 업무 강도의 문제 등으로 인해 요양보호사의 수는 증가하지 않음
<br><br>
 -> **위와 같은 문제를 해결할 방안을 고민해 보고자 프로젝트 구성**


#### 시스템 소개
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

 
#### 기능 리스트
<table cellspacing="0" cellpadding="8">
  <thead>
    <tr>
      <th>ID</th>
      <th>기능 항목</th>
      <th>설명</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>SR-01</td>
      <td>장애물 회피</td>
      <td>주행 시 장애물 감지 후 회피</td>
    </tr>
    <tr>
      <td>SR-02</td>
      <td>순찰</td>
      <td>
        매일 아침/저녁 예약된 시간에 순찰 수행
        <ul>
          <li>인원 확인</li>
          <li>이상 상태 확인</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>SR-03</td>
      <td>위험 상황 판단</td>
      <td>
        주행 중 위험 상황 감지 시 알림
        <ul>
          <li>화재</li>
          <li>쓰러짐</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>SR-04</td>
      <td>대기장소로 복귀</td>
      <td>동작 완료 후 대기 및 충전 스테이션으로 복귀 후 도킹</td>
    </tr>
    <tr>
      <td>SR-05</td>
      <td>호출 응답 주행</td>
      <td>
        QR코드 호출 시 해당 장소로 이동
        <ul>
          <li>다용도실</li>
          <li>노인 병실 침대</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>SR-06</td>
      <td>인지 재활 치료</td>
      <td>
        대화 및 산책을 통한 고독감 완화 및 심리적 안정 제공
        <ul>
          <li>정해진 코스 산책</li>
          <li>대화</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>SR-07</td>
      <td>건강 이상 체크</td>
      <td>
        손목 밴드를 통한 건강 상태 업데이트
        <ul>
          <li>심박수, 체온</li>
          <li>산소포화도</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>SR-08</td>
      <td>인터페이스</td>
      <td>
        로봇 및 어르신 상태 확인 및 관리
        <ul>
          <li>로봇 현황 확인 및 호출</li>
          <li>어르신 정보 및 건강 상태 확인</li>
        </ul>
      </td>
    </tr>
  </tbody>
</table>

#### 프로젝트 실행
###### Requirements
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

###### Installation
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

###### Usage

Run the main script:

```bash
    ros2 run nuri_bot main_node
    ros2 run ai_server server 
```

<hr>

## 기능 및 기술 설명
#### 전체 시나리오

#### 기본 주행

#### 순찰

#### 호출

#### 위험 상황 감지

#### 산책

#### 대화

#### 건강 상태 모니터링

<hr>

## 문제 상황 및 해결 방안
#### 주행

#### AI

#### 건강 상태 모니터링

<hr>

## TEAM. BLACK PIG
| position | name | job | contacts |
|:-----:|------|-----|-----|
| leader | 이정림 | 프로젝트 설계<br>유저 디바이스 구현<br>PPT 제작<br>GitHub README 작성 | jeongliml2002@gmail.com |
| worker | 심재헌 | 프로젝트 설계<br>주행 구현<br>PPT 제작 | sysbmy905@gmail.com |
| worker | 황한문 | 비상상황 감지 구현<br>Human Following 구현<br>AI Server 구현<br>GitHub README 작성 | hhm9124@gmail.com |
| worker | 신동철 | 통신 설계<br>Main Server 구현<br>주행 구현<br>PPT 제작 | lt00104@gmail.com |

<hr>


## 프로젝트 설계
#### 시스템 아키텍처
<table>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/35fb0dd5-97a6-400d-ae11-ac6dc091f7c3" width="100%">
    </td>
    <td width="40%">
      전체 시스템 아키텍처
      <ul>
        <li>하나의 시스템은 총 3대의 컴퓨터, 1대의 로봇, 1개의 유저 디바이스(와치)로 구성</li>
        <li>통신은 영상은 UDP, 유저 디바이스와 컴퓨터들 간은 TCP, 로봇과의 통신은 ROS 통신으로 구성</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/0310e2f8-edab-454c-be73-dd2b0d43b966" width="100%">
    </td>
    <td>
      UDP 통신
      <ul>
        <li>영상 데이터 전달 위한 통신</li>
        <li>Nuribot -> AI Server
          <ul style="list-style-type: circle;">
            <li>Human Following 기능을 위한 후방 카메라 영상 전송</li>
            <li>Fire, Fall Detection 기능을 위한 전방 카메라 영상 전송</li>
          </ul>
        </li>
        <li>Nuri Controller -> Nurse GUI
          <ul style="list-style-type: circle;">
            <li>GUI 화면에서 실시간으로 볼 수 있도록 전방 카메라 영상 전송</li>
          </ul>
        </li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/72a83d7c-dead-47bd-a760-1ec88fb8856e" width="100%">
    </td>
    <td>
      ROS 통신
      <ul>
        <li>제어 명렁 및 로봇 상태 데이터 전달을 위한 통신</li>
        <li>Nuribot -> AI Server
          <ul style="list-style-type: circle;">
            <li>LLM 기반 어르신 대화 기능을 위한 데이터 전송</li>
          </ul>
        </li>
        <li>Nuribot -> Main Server
          <ul style="list-style-type: circle;">
            <li>누리봇에 작업을 할당하고 작업에 필요한 데이터 전송</li>
          </ul>
        </li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/f3a01e72-4acc-4e73-93ac-dbfcd94b5c0d" width="100%">
    </td>
    <td>
      TCP 통신
      <ul>
        <li>하나의 시스템은 총 3대의 컴퓨터, 1대의 로봇, 1개의 유저 디바이스(와치)로 구성</li>
        <li>Main Server -> User PC
          <ul style="list-style-type: circle;">
            <li>로봇에 이동을 지시하고 로봇 상태 및 DB 정보를 GUI에 표시하기 위한 데이터 전송</li>
          </ul>
        </li>
        <li>Main Server -> Nuriwatch
          <ul style="list-style-type: circle;">
            <li>건강 정보 전송 및 비상 알림 기능을 위한 데이터 전송</li>
          </ul>
        </li>
        <li>AI Server -> Main Server
          <ul style="list-style-type: circle;">
            <li>비상 이벤트 데이터 전송</li>
          </ul>
        </li>
      </ul>
    </td>
  </tr>
</table>


#### 데이터 구조
<table>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/448a2fc5-47d9-4c21-abe7-06e5de91fa3b" width="100%">
    </td>
    <td width="40%">
      <ul>
        <li>요양원 입주 어르신 정보 관리</li>
        <li>요양원 정보 관리</li>
        <li>로봇 상태 정보 관리</li>
        <li>작업 정보 관리</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/5c43f371-6fe5-403c-b84b-ac44ea6442fb" width="100%">
    </td>
    <td>
      입주한 어르신들 정보 관리 테이블
      <ul>
        <li>이름, 생년월일, 성별 등 기본 정보 저장</li>
        <li>건강 상태 정보 저장</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/d019ff4a-bfcb-4c18-9321-853a00e0feed" width="100%">
    </td>
    <td>
      요양원 정보 관리 테이블
      <ul>
        <li>요양원 내부 위치 정보 저장</li>
        <li>어르신 침대 위치 정보 저장</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/f6d96622-ddf0-432b-8591-28ceefd9bf52" width="100%">
    </td>
    <td>
      로봇 상태 정보 관리 테이블
      <ul>
        <li>로봇 기본 정보 저장</li>
        <li>로봇 상태 정보 저장</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/3bb7810d-fdfc-4add-b050-4dc23143716b" width="100%">
    </td>
    <td>
      작업 정보 관리 테이블
      <ul>
        <li>작업 진행 목록 저장</li>
        <li>작업 목록 저장</li>
        <li>작업 스케줄 저장</li>
        <li>산책 스케줄 저장</li>
        <li>작업 진행 로그 저장</li>
      </ul>
    </td>
  </tr>
</table>
