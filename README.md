# Capstone2021

## Project : Trailer Reversing Assistance System Considering Driver Input (feat.Capstone)  
  
지도 교수 : 백주훈 교수님  
참여 인원 : 강동운, 백승혁, 김예린, 이혜진,
참여 기간 : 2021.1 ~ 2021.6

## Development Background 
COVID-19의 장기화로 비교적 사람들과의 접촉이 적은 캠핑 트레일러를 이용한 여행 수요가 증가하고 있음 
하지만 트레일러를 부착한 차량의 경우, 일반 차량의 후진 주행방식과 달라 트레일러 운전이 익숙하지 않은 사람들은 많은 불편을 느낀다. 
이러한 불편함을 해소하고자 일부 자동차 회사에서 트레일러 관련 후진 보조 기능이 탑재된 차들을 출시해왔으며 연구[1] 또한 꾸준히 되어왔다.
본 논문은 자율주행 기능이 없는 차량에도 적용할 수 있는, 모델 기반 트레일러 후진 보조 시스템을 제안하며, 이를 시뮬레이션을 이용해 검증하였다.
  
## Main Feature  
1. 고래모양의 펫 하드웨어 모델링  
2. 모델링 파일을 불러와 V-rep 가상환경에 적용  
3. Keras Library를 사용한 강화학습(DQN) 환경구축 및 학습  
4. 펫의 상태를 확인하고 Reward를 줄 수 있는 GUI 제작  
5. Google STT API  
6. User FaceDetection  
7. 프로그램 간 Packet 통신
  
## Using Language  
1. V-rep Simulation : C++, Lua Script
2. ReinForce : Python
3. DataServer : Python
  
## System Architecture (BEFORE)
<img src="https://user-images.githubusercontent.com/52673977/73717132-40538480-475c-11ea-8513-fb641a1aa128.png" width="800" height="500" />  
  
## System Architecture (AFTER)
<img src="https://user-images.githubusercontent.com/52377778/86989541-962dcc00-c1d5-11ea-8c95-35a59bc03dde.PNG" width="800" height="500" />  
  
## Hardware Architecture
<img width="582" alt="hw" src="https://user-images.githubusercontent.com/52673977/77333956-83e36b80-6d67-11ea-809c-83a00d6180e6.png">  
  
## Project GIF  
학습프로그램 GUI  
![Comebot](https://user-images.githubusercontent.com/52377778/86989198-da6c9c80-c1d4-11ea-82a9-291e643d15d4.gif)  

