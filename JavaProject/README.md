# A* 알고리즘 기반 자율주행 경로 탐색 시뮬레이터

Java Swing 환경에서 구현된 자율주행 경로 탐색 및 모바일 로봇 시뮬레이터 프로젝트입니다. 사용자가 실시간으로 화면에 장애물과 목적지를 설정하면, 그리드 맵을 기반으로 최적의 경로를 생성하고 로봇이 물리 연산을 통해 목적지까지 추종 주행합니다.

--- 주요 기능 ---

- A* 최적 경로 탐색
  8방향 탐색 알고리즘과 유클리디안 휴리스틱을 활용하여 실시간으로 최적의 이동 경로를 계산합니다.

- 반응형 화면 대응
  창 크기가 변경되더라도 절대 좌표가 아닌 해상도 비율을 유지하도록 설계하여 장애물과 로봇의 위치가 깨지지 않고 자동으로 보정됩니다.

- 동적 편집 모드
  마우스 드래그를 통해 장애물을 연속으로 배치할 수 있으며, 우클릭 드래그를 이용한 연속 삭제 기능도 지원합니다.

- 작업 실행 취소 및 재실행
  스택 자료구조를 활용하여 사용자가 내린 명령의 히스토리를 저장하고 관리합니다.

- 2D 물리 기반 추종 제어
  타겟 각도와 현재 로봇 각도의 오차를 분석하고 정규화하여, 급커브 구간에서는 스스로 감속하고 직진 구간에서는 가속하는 부드러운 거동을 구현했습니다.

--- 기술 스택 ---

- 사용 언어: Java 17 이상
- 사용 라이브러리: Java Swing, Java AWT
- 개발 환경: IntelliJ IDEA

--- 핵심 알고리즘 및 기술 용어 설명 ---

- A* 알고리즘
  출발지에서 목적지까지의 최적 경로을 찾기 위해 현재까지 이동한 거리 비용과 목적지까지의 예상 거리 비용을 더해 탐색하는 그래프 탐색 알고리즘입니다. 목적지까지의 예상 거리를 구할 때는 피타고라스 정리를 바탕으로 한 유클리디안 거리 공식을 사용했으며, 탐색 효율을 극대화하기 위해 우선순위 큐 자료구조를 적용했습니다.

- 반응형 좌표계 관리
  해상도가 바뀔 때 데이터의 위치가 어긋나는 것을 방지하기 위해 로봇, 목적지, 장애물의 위치를 0과 1 사이의 상대적인 비율로 관리합니다. 화면을 실제로 그리는 시점에만 현재 윈도우의 가로, 세로 크기를 곱해주는 방식으로 해상도 독립성을 확보했습니다.

- 조향 오차 정규화
  로봇이 바라보는 각도와 타겟 노드 사이의 각도 오차를 계산할 때 불필요한 제자리 회전이나 부자연스러운 꺾임을 방지하기 위해 각도 범위를 제한하고 정규화했습니다. 제어 입력의 급격한 변화를 막아주는 필터 효과를 적용하여 움직임을 안정화했습니다.


# A* Algorithm Based Autonomous Driving Path Planning Simulator

This project is an autonomous driving path planning and mobile robot simulator implemented in a Java Swing environment. When a user sets obstacles and a destination on the screen in real time, it generates the optimal path based on a grid map, and the robot tracks and drives to the destination through physics calculations.

--- Key Features ---

- A* Optimal Path Planning
  Calculates the optimal travel path in real time utilizing an 8-way search algorithm and Euclidean heuristic.

- Responsive Screen Support
  Designed to maintain coordinate ratios relative to the resolution rather than absolute coordinates, ensuring that the positions of obstacles and the robot do not break and are automatically calibrated even when the window size changes.

- Dynamic Edit Mode
  Supports continuous obstacle placement via mouse dragging, as well as continuous deletion functionality using right-click dragging.

- Undo and Redo Operations
  Utilizes stack data structures to store and manage the history of commands executed by the user.

- 2D Physics-Based Tracking Control
  Analyzes and normalizes the error between the target angle and the current robot angle, implementing smooth behavior that automatically decelerates in sharp curves and accelerates in straight sections.

--- Tech Stack ---

- Language: Java 17 or higher
- Libraries: Java Swing, Java AWT
- IDE: IntelliJ IDEA

--- Core Algorithms & Technical Terms ---

- A* Algorithm
  A graph search algorithm that calculates the path from a starting point to a destination by combining the distance cost traveled so far with the estimated distance cost to the destination. To find the estimated distance to the goal, the Euclidean distance formula based on the Pythagorean theorem was used, and a priority queue data structure was applied to maximize search efficiency.

- Responsive Coordinate Management
  Manages the positions of the robot, destination, and obstacles as relative ratios between 0 and 1 to prevent data misalignment when the resolution changes. Resolution independence was achieved by multiplying these ratios by the current window's width and height only at the actual rendering stage.

- Steering Error Normalization
  When calculating the angle error between the robot's heading and the target node, the angle range is constrained and normalized to prevent unnecessary in-place rotations or unnatural turning behaviors. Additionally, a filtering effect was applied to mitigate abrupt changes in control inputs, stabilizing the robot's motion.


--- References ---

- Stanford University - Introduction to A* Pathfinding : https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html

- Oracle Java Documentation - 2D Graphics : https://docs.oracle.com/javase/8/docs/technotes/guides/2d/spec/j2d-bookTOC.html

- PythonRobotics - Path Tracking : https://github.com/AtsushiSakai/PythonRobotics#path-tracking

- 한국로봇학회 - 격자 기반 맵에서의 A* 알고리즘 경로 탐색 기법 : https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE02102146

- 정보과학회논문지 - 모바일 로봇의 자율주행을 위한 경로 추종 및 조향 제어 : https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE09301548
