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
<div align="center">
  <img src="https://github.com/user-attachments/assets/cba18e75-6183-4275-9e25-5836891e04e6" width="500" />
</div>
<ul>
  <li>ROS2(Jazzy): <a href="https://docs.ros.org/en/jazzy/index.html">https://docs.ros.org/en/jazzy/index.html</a></li>
  <li>Robot Courtesy by <a href="https://pinklab.art/?page_id=5849">PINKLAB</a></li>
</ul>
      

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
<table>
  <tr>
    <td width="60%" rowspan="2">
      <img src="https://github.com/user-attachments/assets/a67f2bbf-c161-4730-b205-e0eb8d9ad8f5" width="100%" />
    </td>
    <td>
      <ul>
        <li>상태 다이어그램을 기반으로 중요 시나리오만 표현한 간단한 다이어그램</li>
        <li>시나리오 설명을 위한 간단한 다이어그램</li>
        <li>충전을 하는 중에 배터리 상태에 따라 충전/대기 상태 변환
          <ul>
            <li>충전 : 충전 스테이션에서 충전 진행 중이며 다음 명령을 받을 수 없는 상태</li>
            <li>대기 : 충전 스테이션에서 충전 진행 중이나 다음 명령을 받을 수 있는 상태</li>
          </ul>
        </li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/91162e40-9b58-4758-93c8-e78ef28cc23c" width=100%" />
      <h4>로봇의 전체 시나리오 State Diagram</h4>
      <ul>
        <li>노란색 박스 상태 : 명령을 받을 수 있는 상태
          <ul>
            <li>나머지 파란색 박스 상태는 작업 수행중이라 판단하여 명령 받을 수 없음</li>
          </ul>
        </li>
        <li>진한 태두리가 표시되어 있는 상태 : 복합 상태
          <ul>
            <li>기본 주행 시나리오</li>
            <li>주행 시 장애물 회피 등의 상태 변경이 있음</li>
          </ul>
        </li>
      </ul>
    </td>
  </tr>
</table>

#### 복합 상태 동작 시나리오
<table>
  <tr>
    <td colspan="2" align="center">
      <h4>기본 주행</h4>
    </td>
  </tr>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/0e9d1402-5ec5-4922-84d8-1bb181895a3b" width="100%" />
    </td>
    <td>
      <ul>
        <li></li>
      </ul>
    </td>
  </tr>
</table>
<h5>Domain Bridge</h5>
<h5>SLAM&Navigation</h5>
<h5>ArUco Marker</h5>

<table>
  <tr>
    <td colspan="3" align="center">
      <h4>순찰</h4>
    </td>
  </tr>
  <tr>
    <td width="40%" rowspan="2">
      <img src="https://github.com/user-attachments/assets/ee81e78f-f006-4bbf-972c-cc0b064256e7" width="100%" />
    </td>
    <td width="20%" rowspan="2">
      지정 된 시간에 정해진 루트를 따라서 순찰 진행
      <ul>
        <li>시간은 User GUI에서 설정 가능</li>
        <li>검은 색 점으로 표시 된 부분은 Way Point로 중간 목표 지점</li>
        <li>주황색 테두리 부분은 인원 체크 지점으로 블루투스로 인원 체크하는 지점</li>
        <li>네모 박스 표시 부분은 침대가 있는 부분</li>
      </ul>
    </td>
    <td width="40%">
      <img src="https://github.com/user-attachments/assets/3ad9e6fa-a297-4f33-993e-0c5191e5ec65" width="100%" />
      인원 체크 지점 도달 시 어르신들 워치에서 쏘고 있는 블루투스 신호를 통해 인원 확인
      <ul>
        <li>확인 되지 않는 인원이 존재하면 알림</li>
        <li>알림 후, 혹은 이상 없다고 판단 후에 순찰 지속</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td>
      <img src="https://github.com/user-attachments/assets/b945e7ae-e366-45cb-b65e-2b56ba349cf2" width"100%" />
      위험 상황 감지
      <ul>
        <li>위험 상황 감지 후 사용자 GUI로 위험 상황 요양보호사에게 알림</li>
        <li>위험 상황 종류
          <ul>
            <li>화재</li>
            <li>쓰러짐</li>
          </ul>
        </li>
      </ul>
    </td>
  </tr>
</table>
<h5>Object Detection</h5>
<h5>Pose Estimation</h5>
<h5>BLE DeviceName, MAC Address 사용한 인원 확인</h5>

<table>
  <tr>
    <td colspan="2" align="center">
      <h4>호출</h4>
    </td>
  </tr>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/03d88f87-8348-4098-af63-9fa7e5a785ff" width=100%" />
    </td>
    <td width="40%">
      <ul>
        <li>요양보호사는 필요에 따라 지정된 위치에 QR 코드, 혹은 GUI를 통해 로봇 호출</li>
        <li>로봇이 호출 될 시, 호출 된 지점으로 로봇이 이동 한 후 요양보호사가 지정하는 작업 진행</li>
        <li>
          호출 가능 위치
          <ul>
            <li>요양보호사 작업실</li>
            <li>침실 1</li>
            <li>침실 2</li>
          </ul>
        </li>
      </ul>
    </td>
  </tr>
</table>
<h5>QR code</h5>
<h5>Call</h5>

<table>
  <tr>
    <td colspan="2" align="center">
      <h4>산책</h4>
    </td>
  </tr>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/ddcf1ea9-1d75-4dae-958d-79f7cd729c50" width=100%" />
    </td>
    <td width="40%">
      등록 된 스케줄에 따라 산책 진행
      <ul>
        <li>등록 된 시간이 되면 산책 스케줄이 지정 된 어르신의 침대로 이동</li>
        <li>대상자 확인 후 의사 물어봄</li>
        <li>어르신이 승낙하면 정해진 경로로 산책 진행</li>
        <li>어르신이 거부하면 3번 다시 물어본 후 전부 다 거부할 시 로그 남겨두고 산책 진행 종료</li>
        <li>산책 진행 후 대상 어르신의 침대로 도달, 혹은 3번의 물음에 거부 의사 표현 시, 산책 종료로 인식 후 대기 상태로 전환하고 충전 스테이션으로 복귀</li>
      </ul>
    </td>
  </tr>
</table>
<h5>ReID</h5>
<h5>Depth Estimation</h5>

<table>
  <tr>
    <td colspan="2" align="center">
      <h4>대화</h4>
    </td>
  </tr>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/0e7516fb-a4b7-4321-ac0f-71b97e78d7d5" width=100%" />
    </td>
    <td width="40%">
      어르신이 대화하고 싶은 경우에 호출하여 대화 진행
      <ul>
        <li>어르신이 호출하면 호출한 어르신의 침대로 이동</li>
        <li>호출한 어르신이 "누리야"하고 트리거 워드 말하면 대화 상태로 전환</li>
        <li>대화 진행 하다 어르신이 종료 의사 내비치면 대화 종료로 인식</li>
        <li>대화 종료 멘트 출력 후, 대기 상태로 전환하고 충전 스테이션으로 복귀</li>
      </ul>
    </td>
  </tr>
</table>
<h5>STT</h5>
<h5>LLM</h5>
<h5>TTS</h5>

<table>
  <tr>
    <td colspan="2" align="center">
      <h4>건강 상태 모니터링</h4>
    </td>
  </tr>
  <tr>
    <td width="60%">
      <img src="https://github.com/user-attachments/assets/efb45bf9-3347-4643-8df1-94d122e18776" width=100%" />
    </td>
    <td width="20%">
      <ul>
        <li></li>
      </ul>
    </td>
  </tr>
</table>
<h5>OLED</h5>
<h5>MAX30105</h5>
<h5>MLX90614</h5>

<hr>

## 문제 상황 및 해결 방안
#### 주행

#### 대화

#### 건강 상태 모니터링

<hr>

## TEAM. BLACK PIG
| position | name | job | contacts |
|:-----:|------|-----|-----|
| leader | 이정림 | 프로젝트 설계<br>유저 디바이스 구현<br>PPT 제작<br>GitHub README 작성 | [JlimL(LeeJ)](https://github.com/JlimL) <br> [jeongliml2002@gmail.com](mailto:jeongliml2002@gmail.com) |
| worker | 심재헌 | 프로젝트 설계<br>주행 구현<br>PPT 제작 | [jaeheon7134(jaeheon)](https://github.com/jaeheon7134) <br> [sysbmy905@gmail.com](mailto:sysbmy905@gmail.com) |
| worker | 황한문 | 비상상황 감지 구현<br>Human Following 구현<br>AI Server 구현<br>GitHub README 작성 | [1-moon(H.M. Hwang/1-moon😆](https://github.com/1-moon) <br> [hhm9124@gmail.com](mailto:hhm9124@gmail.com) |
| worker | 신동철 | 통신 설계<br>Main Server 구현<br>주행 구현<br>PPT 제작 | [copper-iron(Shin)](https://github.com/copper-iron) <br> [lt00104@gmail.com](mailto:lt00104@gmail.com) |

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
