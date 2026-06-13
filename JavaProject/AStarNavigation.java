import javax.swing.*; // GUI 기본 창이랑 버튼, 타이머 같은 것을 쓰기 위한 라이브러리
import java.awt.*; // 색상, 폰트, 그래픽스2D 같은 그리기 도구를 쓰기 위한 클래스 모음
import java.awt.event.ComponentAdapter; // 창 크기 바뀔 때 반응하기 위한 이벤트 리스너 추상 클래스
import java.awt.event.ComponentEvent; // 창 크기 변경 이벤트를 처리하기 위한 클래스
import java.awt.event.MouseAdapter; // 마우스 클릭이랑 드래그를 한방에 처리하기 위한 이벤트 리스너
import java.awt.event.MouseEvent; // 마우스 클릭 좌표(X, Y)나 좌우클릭을 감지하기 위한 이벤트 클래스
import java.awt.geom.Point2D; // 로봇이랑 목적지 사이 거리를 계산하기 위한 2차원 좌표 클래스
import java.awt.geom.Rectangle2D; // 사각형 장애물 영역이랑 충돌을 체크하기 위한 사각형 클래스
import java.util.List; // 장애물들이랑 A* 경로를 묶어서 관리하기 위한 리스트 인터페이스
import java.util.ArrayList; // 실시간으로 추가되는 장애물 데이터를 담기 위한 배열 리스트 클래스
import java.util.Stack; // 언두/리두 기능을 구현하기 위해 액션을 쌓아두는 스택 클래스
import java.util.PriorityQueue; // A*에서 fScore 낮은 노드를 먼저 뽑기 위한 우선순위 큐 클래스
import java.util.Map; // A* 노드들을 좌표 키값으로 묶어 중복 없이 찾기 위한 맵 인터페이스
import java.util.HashMap; // 해시 맵 구조로 노드 데이터를 매핑해서 저장하기 위한 클래스
import java.util.Comparator; // 우선순위 큐 안에서 fScore 기준으로 정렬하기 위한 비교기 인터페이스

public class AStarNavigation extends JFrame {

    public AStarNavigation() {
        setTitle("A* 내비게이션 자율주행 시뮬레이터");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        AStarPanel panel = new AStarPanel();
        add(panel);

        pack();
        setLocationRelativeTo(null);
        setResizable(true);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            new AStarNavigation().setVisible(true);
        });
    }
}

class AStarPanel extends JPanel {
    // 편집, 주행 상태 나누기
    private enum SystemState { EDIT, RUN }
    private SystemState currentState = SystemState.EDIT;

    // 장애물 넣을지 목적지 찍을지 고르는 모드
    private enum EditMode { ADD_OBSTACLE, SET_GOAL }
    private EditMode currentEditMode = EditMode.ADD_OBSTACLE;

    // 언두 리두용 액션 타입 정의
    private enum ActionType { ADD, DELETE }
    private static class HistoryAction {
        ActionType type;
        ScalingObstacle obstacle;
        HistoryAction(ActionType type, ScalingObstacle obstacle) {
            this.type = type;
            this.obstacle = obstacle;
        }
    }

    // 화면 해상도 대응용 기본 베이스 크기
    private int lastWidth = 1920;
    private int lastHeight = 1080;

    // 로봇이 처음 스폰될 위치 비율
    private final double initialXRatio = 150.0 / 1920.0;
    private final double initialYRatio = 540.0 / 1080.0;

    // 로봇 실시간 좌표랑 물리엔진 변수들
    private double robotXRatio = initialXRatio;
    private double robotYRatio = initialYRatio;
    private double robotTheta = 0;
    private double speed = 0;
    private double steering = 0;
    private final double robotRadius = 16;

    // 목적지 기본 좌표 비율이랑 크기
    private double goalXRatio = 1750.0 / 1920.0;
    private double goalYRatio = 540.0 / 1080.0;
    private final double goalRadius = 15;

    // 창 크기 변해도 장애물 안깨지게 비율로 들고있는 클래스
    private static class ScalingObstacle {
        double xRatio, yRatio, wRatio, hRatio;
        ScalingObstacle(double xRatio, double yRatio, double wRatio, double hRatio) {
            this.xRatio = xRatio; this.yRatio = yRatio;
            this.wRatio = wRatio; this.hRatio = hRatio;
        }
        // 그릴 때만 현재 윈도우 크기 곱해서 사각형으로 뱉도록 함
        Rectangle2D toRectangle(int width, int height) {
            return new Rectangle2D.Double(xRatio * width, yRatio * height, wRatio * width, hRatio * height);
        }
    }

    private final List<ScalingObstacle> obstacles;
    private final Stack<HistoryAction> undoStack = new Stack<>();
    private final Stack<HistoryAction> redoStack = new Stack<>();
    private Point lastObstaclePoint = null;
    private final double dragSpacing = 35; // 드래그할 때 장애물이 겹치지 않도록 간격 설정

    private String errorMessage = "";
    private int errorDisplayCounter = 0;

    private final int gridSize = 15; // A* 격자 사이즈 15픽셀
    private List<Point> calculatedPath = new ArrayList<>();
    private int currentPathTargetIndex = 0;

    private JButton btnObstacleMode;
    private JButton btnGoalMode;
    private JButton btnUndo;
    private JButton btnRedo;
    private JButton btnStartSimulation;
    private JButton btnResetRobot;
    private JButton btnResetSimulation;

    public AStarPanel() {
        setPreferredSize(new Dimension(1920, 1080));
        setBackground(new Color(240, 242, 245));
        setLayout(null);

        obstacles = new ArrayList<>();
        initControlUI();

        // 주행 중일 때 창 크기 조절되면 경로 재탐색
        addComponentListener(new ComponentAdapter() {
            @Override
            public void componentResized(ComponentEvent e) {
                lastWidth = Math.max(getWidth(), 800);
                lastHeight = Math.max(getHeight(), 500);
                if (currentState == SystemState.RUN) {
                    replanAStarPath();
                }
                repaint();
            }
        });

        // 마우스 클릭하고 드래그하는거 처리하는 핸들러
        MouseAdapter mouseHandler = new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (e.getY() < 60) return; // 상단바 메뉴 영역은 클릭 씹음

                if (currentState == SystemState.EDIT) {
                    // 우클릭하면 장애물 지우기
                    if (SwingUtilities.isRightMouseButton(e)) {
                        deleteObstacleAt(e.getX(), e.getY());
                        repaint();
                        return;
                    }

                    // 좌클릭은 모드따라 다름
                    if (SwingUtilities.isLeftMouseButton(e)) {
                        if (currentEditMode == EditMode.ADD_OBSTACLE) {
                            if (checkObstacleCollision(e.getX(), e.getY())) {
                                triggerError("로봇이나 목적지 위에는 설치할 수 없습니다!");
                            } else {
                                addObstacleAt(e.getX(), e.getY());
                                lastObstaclePoint = e.getPoint();
                                clearError();
                            }
                        } else if (currentEditMode == EditMode.SET_GOAL) {
                            if (checkGoalCollision(e.getX(), e.getY())) {
                                triggerError("이 곳에는 설치할 수 없습니다!");
                            } else {
                                goalXRatio = (double) e.getX() / lastWidth;
                                goalYRatio = (double) e.getY() / lastHeight;
                                clearError();
                            }
                        }
                    }
                }
                repaint();
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                lastObstaclePoint = null; // 드래그 끝났으니 초기화
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                if (e.getY() < 60) return;
                if (currentState != SystemState.EDIT) return;

                // 우클릭 드래그로 연속 삭제 기능
                if (SwingUtilities.isRightMouseButton(e)) {
                    deleteObstacleAt(e.getX(), e.getY());
                    repaint();
                    return;
                }

                // 좌클릭 드래그로 장애물 연속 배치
                if (SwingUtilities.isLeftMouseButton(e) && currentEditMode == EditMode.ADD_OBSTACLE) {
                    if (lastObstaclePoint == null) {
                        if (!checkObstacleCollision(e.getX(), e.getY())) {
                            addObstacleAt(e.getX(), e.getY());
                            lastObstaclePoint = e.getPoint();
                            clearError();
                        } else {
                            triggerError("로봇이나 목적지 위에는 설치할 수 없습니다!");
                        }
                    } else {
                        // 촘촘하게 박히는 것을 방지하기 위한 거리 계산
                        double dist = Point2D.distance(lastObstaclePoint.x, lastObstaclePoint.y, e.getX(), e.getY());
                        if (dist >= dragSpacing) {
                            if (!checkObstacleCollision(e.getX(), e.getY())) {
                                addObstacleAt(e.getX(), e.getY());
                                lastObstaclePoint = e.getPoint();
                                clearError();
                            } else {
                                triggerError("로봇이나 목적지 위에는 설치할 수 없습니다!");
                            }
                        }
                    }
                }
                repaint();
            }
        };

        addMouseListener(mouseHandler);
        addMouseMotionListener(mouseHandler);

        // 30ms 주기로 물리엔진 돌리고 에러메시지 타이머 깎는 메인 루프
        javax.swing.Timer timer = new javax.swing.Timer(30, e -> {
            if (currentState == SystemState.RUN) {
                navigateAStarPath();
                updatePhysics();
            }

            if (errorDisplayCounter > 0) {
                errorDisplayCounter--;
                if (errorDisplayCounter == 0) {
                    errorMessage = "";
                }
            }
            repaint();
        });
        timer.start();
    }

    // 장애물 스폰할 때 로봇이나 목표지점 겹치는지 체크
    private boolean checkObstacleCollision(int x, int y) {
        Rectangle2D newObRect = new Rectangle2D.Double(x - 20, y - 20, 40, 40);
        double rX = robotXRatio * lastWidth;
        double rY = robotYRatio * lastHeight;
        double gX = goalXRatio * lastWidth;
        double gY = goalYRatio * lastHeight;

        if (newObRect.intersects(rX - robotRadius, rY - robotRadius, robotRadius * 2, robotRadius * 2)) {
            return true;
        }
        if (newObRect.intersects(gX - goalRadius, gY - goalRadius, goalRadius * 2, goalRadius * 2)) {
            return true;
        }
        return false;
    }

    // 목적지 옮길 때 로봇이나 장애물 위에 찍히는지 감시
    private boolean checkGoalCollision(int x, int y) {
        double rX = robotXRatio * lastWidth;
        double rY = robotYRatio * lastHeight;
        if (Point2D.distance(rX, rY, x, y) < (robotRadius + goalRadius)) {
            return true;
        }
        for (ScalingObstacle obs : obstacles) {
            Rectangle2D obstacle = obs.toRectangle(lastWidth, lastHeight);
            if (obstacle.contains(x, y) || obstacle.intersects(x - goalRadius, y - goalRadius, goalRadius * 2, goalRadius * 2)) {
                return true;
            }
        }
        return false;
    }

    // 하단 팝업 띄우기 위한 카운터 세팅
    private void triggerError(String msg) {
        errorMessage = msg;
        errorDisplayCounter = 40;
    }

    private void clearError() {
        errorMessage = "";
        errorDisplayCounter = 0;
    }

    // 장애물 추가하고 언두 스택에 넣음. 리두 스택은 날려야함
    private void addObstacleAt(int x, int y) {
        ScalingObstacle newObstacle = new ScalingObstacle(
                (double)(x - 20) / lastWidth,
                (double)(y - 20) / lastHeight,
                40.0 / lastWidth,
                40.0 / lastHeight
        );
        obstacles.add(newObstacle);
        undoStack.push(new HistoryAction(ActionType.ADD, newObstacle));
        redoStack.clear();
        updateButtonStates();
    }

    // 장애물 지우고 언두 스택에 '삭제 액션' 저장함
    private void deleteObstacleAt(int x, int y) {
        for (int i = obstacles.size() - 1; i >= 0; i--) {
            ScalingObstacle obs = obstacles.get(i);
            if (obs.toRectangle(lastWidth, lastHeight).contains(x, y)) {
                obstacles.remove(i);
                undoStack.push(new HistoryAction(ActionType.DELETE, obs));
                redoStack.clear();
                updateButtonStates();
                return;
            }
        }
    }

    // 뒤로가기 실행, 스택에서 뽑아서 반대로 처리
    private void performUndo() {
        if (!undoStack.isEmpty()) {
            HistoryAction action = undoStack.pop();
            if (action.type == ActionType.ADD) {
                obstacles.remove(action.obstacle);
            } else if (action.type == ActionType.DELETE) {
                obstacles.add(action.obstacle);
            }
            redoStack.push(action);
            updateButtonStates();
            repaint();
        }
    }

    // 앞으로가기 실행, 언두했던거 다시 복구
    private void performRedo() {
        if (!redoStack.isEmpty()) {
            HistoryAction action = redoStack.pop();
            if (action.type == ActionType.ADD) {
                obstacles.add(action.obstacle);
            } else if (action.type == ActionType.DELETE) {
                obstacles.remove(action.obstacle);
            }
            undoStack.push(action);
            updateButtonStates();
            repaint();
        }
    }

    // 핵심 A* 알고리즘 돌리는 구역임
    private boolean replanAStarPath() {
        double rX = robotXRatio * lastWidth;
        double rY = robotYRatio * lastHeight;
        double gX = goalXRatio * lastWidth;
        double gY = goalYRatio * lastHeight;

        int cols = lastWidth / gridSize + 1;
        int rows = lastHeight / gridSize + 1;

        boolean[][] isBlocked = new boolean[cols][rows];

        // 맵에 박힌 장애물들 전부 그리드 맵에 락 걸기
        for (ScalingObstacle obs : obstacles) {
            Rectangle2D rect = obs.toRectangle(lastWidth, lastHeight);
            int minC = Math.max(0, (int) (rect.getMinX() / gridSize));
            int maxC = Math.min(cols - 1, (int) (rect.getMaxX() / gridSize));
            int minR = Math.max(0, (int) (rect.getMinY() / gridSize));
            int maxR = Math.min(rows - 1, (int) (rect.getMaxY() / gridSize));

            for (int c = minC; c <= maxC; c++) {
                for (int r = minR; r <= maxR; r++) {
                    isBlocked[c][r] = true;
                }
            }
        }

        // 상단바 메뉴 60픽셀 안에는 경로 못 짜게 방지
        int menuRows = 60 / gridSize;
        for (int c = 0; c < cols; c++) {
            for (int r = 0; r <= menuRows; r++) {
                if (r < rows) isBlocked[c][r] = true;
            }
        }

        // 시작점 인덱스랑 끝점 인덱스 구하기
        int startC = Math.max(0, Math.min(cols - 1, (int) (rX / gridSize)));
        int startR = Math.max(0, Math.min(rows - 1, (int) (rY / gridSize)));
        int targetC = Math.max(0, Math.min(cols - 1, (int) (gX / gridSize)));
        int targetR = Math.max(0, Math.min(rows - 1, (int) (gY / gridSize)));

        // 시작이랑 끝은 버그 방지로 블로킹 풀기
        if (startC < cols && startR < rows) isBlocked[startC][startR] = false;
        if (targetC < cols && targetR < rows) isBlocked[targetC][targetR] = false;

        // fScore 낮은 순으로 뽑아주는 오픈셋 큐
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(n -> n.fScore));
        Map<String, Node> allNodes = new HashMap<>();

        Node startNode = new Node(startC, startR);
        startNode.gScore = 0;
        startNode.fScore = heuristic(startC, startR, targetC, targetR);
        openSet.add(startNode);
        allNodes.put(startC + "," + startR, startNode);

        boolean success = false;
        Node targetNodeRef = null;

        // 대각선까지 포함해서 8방향 탐색용 배열
        int[] dC = {-1, 1, 0, 0, -1, -1, 1, 1};
        int[] dR = {0, 0, -1, 1, -1, 1, -1, 1};
        double[] dCost = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414}; // 대각선은 루트2 배율임

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            // 골인 지점 도달하면 멈춤
            if (current.c == targetC && current.r == targetR) {
                success = true;
                targetNodeRef = current;
                break;
            }

            current.isClosed = true;

            // 주변 8칸 탐색
            for (int i = 0; i < 8; i++) {
                int nextC = current.c + dC[i];
                int nextR = current.r + dR[i];

                if (nextC < 0 || nextC >= cols || nextR < 0 || nextR >= rows) continue;
                if (isBlocked[nextC][nextR]) continue;

                String key = nextC + "," + nextR;
                Node neighbor = allNodes.computeIfAbsent(key, k -> new Node(nextC, nextR));

                if (neighbor.isClosed) continue;

                double tentativeG = current.gScore + dCost[i];
                if (tentativeG < neighbor.gScore) {
                    neighbor.parent = current;
                    neighbor.gScore = tentativeG;
                    neighbor.fScore = tentativeG + heuristic(nextC, nextR, targetC, targetR);

                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }
            }
        }

        // 경로 찾았으면 뒤에서부터 부모 타고 역추적해서 경로 리스트에 꽂아넣기
        if (success) {
            calculatedPath.clear();
            Node curr = targetNodeRef;
            while (curr != null) {
                calculatedPath.add(0, new Point(curr.c * gridSize + gridSize / 2, curr.r * gridSize + gridSize / 2));
                curr = curr.parent;
            }
            currentPathTargetIndex = 0;
            return true;
        }

        return false;
    }

    // 유클리디안 거리로 피타고라스를 이용해 휴리스틱 값 계산
    private double heuristic(int c1, int r1, int c2, int r2) {
        return Math.sqrt((c1 - c2) * (c1 - c2) + (r1 - r2) * (r1 - r2));
    }

    // A* 전용 노드 구조체
    private static class Node {
        int c, r;
        double gScore = Double.MAX_VALUE;
        double fScore = Double.MAX_VALUE;
        boolean isClosed = false;
        Node parent = null;

        Node(int c, int r) {
            this.c = c;
            this.r = r;
        }
    }

    // 사용자가 작성한 경로 따라서 조향각이랑 속도 계산
    private void navigateAStarPath() {
        double rX = robotXRatio * lastWidth;
        double rY = robotYRatio * lastHeight;
        double gX = goalXRatio * lastWidth;
        double gY = goalYRatio * lastHeight;

        // 최종 목적지 근처에 다 오면 종료 팝업 띄우고 모드 바꾸기
        double distanceToGoal = Point2D.distance(rX, rY, gX, gY);
        if (distanceToGoal < 22) {
            speed = 0;
            steering = 0;
            currentState = SystemState.EDIT;
            updateButtonStates();
            JOptionPane.showMessageDialog(this, "주행 종료 목적지에 안전하게 도달했습니다!");
            return;
        }

        if (calculatedPath.isEmpty()) {
            speed = 0;
            steering = 0;
            return;
        }

        // 지금 가야 할 노드 지나쳤으면 다음 노드로 인덱스 넘김
        while (currentPathTargetIndex < calculatedPath.size()) {
            Point targetPt = calculatedPath.get(currentPathTargetIndex);
            double distToTargetNode = Point2D.distance(rX, rY, targetPt.x, targetPt.y);
            if (distToTargetNode < 18 && currentPathTargetIndex < calculatedPath.size() - 1) {
                currentPathTargetIndex++;
            } else {
                break;
            }
        }

        // 타겟 노드 바라보는 각도 따서 조향 에러 구함
        Point currentTargetPoint = calculatedPath.get(Math.min(currentPathTargetIndex, calculatedPath.size() - 1));
        double angleToNode = Math.atan2(currentTargetPoint.y - rY, currentTargetPoint.x - rX);

        double steeringError = angleToNode - robotTheta;
        steeringError = Math.atan2(Math.sin(steeringError), Math.cos(steeringError)); // -pi에서 pi 사이로 정규화함

        // 로패스 필터 비슷하게 조향각 먹여서 덜 부들대게 만듦
        double rawSteering = steeringError * 0.4;
        steering = (steering * 0.4) + (rawSteering * 0.6);

        // 꺾어야 될 각도 크면 속도 좀 줄이고 직진할 땐 밟음
        double baseSpeed = (lastWidth / 1920.0) * 4.2 + 1.2;
        speed = baseSpeed * Math.cos(steeringError);
        if (speed < 2.0) speed = 2.0;
    }

    // 2D 평면 킹홀 물리엔진 업데이트 연산 구역
    private void updatePhysics() {
        double rX = robotXRatio * lastWidth;
        double rY = robotYRatio * lastHeight;

        robotTheta += steering;
        rX += speed * Math.cos(robotTheta);
        rY += speed * Math.sin(robotTheta);

        // 화면 밖으로 이동하는 현상 방지
        if (rX < 25) rX = 25;
        if (rX > lastWidth - 25) rX = lastWidth - 25;
        if (rY < 75) rY = 75;
        if (rY > lastHeight - 25) rY = lastHeight - 25;

        robotXRatio = rX / lastWidth;
        robotYRatio = rY / lastHeight;
    }

    // 모드 전환될 때 버튼들 누를 수 있는지 없는지 상태 동기화
    private void updateButtonStates() {
        if (currentState == SystemState.EDIT) {
            btnObstacleMode.setEnabled(true);
            btnGoalMode.setEnabled(true);
            btnUndo.setEnabled(!undoStack.isEmpty());
            btnRedo.setEnabled(!redoStack.isEmpty());
            btnResetRobot.setEnabled(true);
            btnStartSimulation.setText("자율주행 시작 (RUN)");
            btnStartSimulation.setBackground(new Color(46, 204, 113));
        } else if (currentState == SystemState.RUN) {
            btnObstacleMode.setEnabled(false);
            btnGoalMode.setEnabled(false);
            btnUndo.setEnabled(false);
            btnRedo.setEnabled(false);
            btnResetRobot.setEnabled(false);
            btnStartSimulation.setText("주행 일시정지");
            btnStartSimulation.setBackground(new Color(231, 76, 60));
        }
    }

    // UI랑 각종 오브젝트들 실시간 렌더링 하는 메인 컴포넌트
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON); // 안티앨리어싱 켜서 부드럽게 함

        double rX = robotXRatio * lastWidth;
        double rY = robotYRatio * lastHeight;
        double gX = goalXRatio * lastWidth;
        double gY = goalYRatio * lastHeight;

        // 상단바 영역 어두운 색 배경 색칠
        g2d.setColor(new Color(44, 62, 80));
        g2d.fillRect(0, 0, lastWidth, 60);

        // 등록된 장애물 리스트 돌면서 전부 화면에 그리기
        g2d.setColor(new Color(100, 110, 120));
        for (ScalingObstacle obs : obstacles) {
            Rectangle2D obstacle = obs.toRectangle(lastWidth, lastHeight);
            g2d.fill(obstacle);
            g2d.setColor(new Color(80, 90, 100));
            g2d.draw(obstacle);
            g2d.setColor(new Color(100, 110, 120));
        }

        // 목적지 범위 원형 이펙트
        g2d.setColor(new Color(192, 57, 43, 30));
        g2d.fillOval((int) gX - 15, (int) gY - 15, 30, 30);

        // 목적지 깃대 그리기
        g2d.setColor(new Color(50, 50, 50));
        g2d.setStroke(new BasicStroke(2.5f));
        g2d.drawLine((int)gX - 6, (int)gY + 12, (int)gX - 6, (int)gY - 15);

        // 빨간 깃발 폴리곤 따서 채우기
        Polygon flagPoly = new Polygon();
        flagPoly.addPoint((int)gX - 4, (int)gY - 15);
        flagPoly.addPoint((int)gX + 14, (int)gY - 7);
        flagPoly.addPoint((int)gX - 4, (int)gY);

        g2d.setColor(new Color(231, 76, 60));
        g2d.fillPolygon(flagPoly);
        g2d.setColor(new Color(192, 57, 43));
        g2d.setStroke(new BasicStroke(1.0f));
        g2d.drawPolygon(flagPoly);

        g2d.setColor(Color.DARK_GRAY);
        g2d.fillOval((int)gX - 10, (int)gY + 8, 8, 5);

        g2d.setFont(new Font("맑은 고딕", Font.BOLD, 11));
        g2d.drawString("GOAL", (int) gX - 14, (int) gY - 20);

        // 파란색 자율주행 경로선 예쁘게 이어그리기
        if (currentState == SystemState.RUN && !calculatedPath.isEmpty()) {
            g2d.setColor(new Color(41, 128, 185));
            g2d.setStroke(new BasicStroke(3.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));

            for (int i = 0; i < calculatedPath.size() - 1; i++) {
                Point p1 = calculatedPath.get(i);
                Point p2 = calculatedPath.get(i + 1);
                g2d.drawLine(p1.x, p1.y, p2.x, p2.y);
            }
        }

        // 차량 그릴 때 회전각 반영하려고 어핀 변환 사용
        java.awt.geom.AffineTransform oldTransform = g2d.getTransform();
        g2d.translate(rX, rY);
        g2d.rotate(robotTheta);

        // 바퀴 4개
        g2d.setColor(Color.BLACK);
        g2d.fillRect(8, -17, 10, 4);
        g2d.fillRect(8, 13, 10, 4);
        g2d.fillRect(-16, -17, 10, 4);
        g2d.fillRect(-16, 13, 10, 4);

        // 파란색 차체 바디
        g2d.setColor(new Color(52, 152, 219));
        g2d.fillRect(-18, -14, 34, 28);
        g2d.fillOval(4, -14, 14, 28);

        // 노란 헤드라이트랑 앞유리창 데코
        g2d.setColor(new Color(241, 196, 15));
        g2d.fillRect(2, -10, 5, 20);

        g2d.setColor(new Color(41, 128, 185));
        g2d.fillRect(-10, -9, 10, 18);

        // 회전 변환 다 썼으면 원상복구
        g2d.setTransform(oldTransform);

        // 우측 상단 현재 상태 텍스트 띄우기
        g2d.setColor(Color.WHITE);
        g2d.setFont(new Font("맑은 고딕", Font.BOLD, 13));

        String stateText = (currentState == SystemState.EDIT) ? "편집 모드 (EDIT)" : "A* 알고리즘 기동 중 (RUNNING...)";
        g2d.drawString("상태: " + stateText, lastWidth - 760, 36);

        if (currentState == SystemState.EDIT) {
            g2d.setColor(Color.YELLOW);
            String modeText = (currentEditMode == EditMode.ADD_OBSTACLE) ? "장애물 배치 [우클릭 드래그: 연속 삭제]" : "목적지 재설정 활성화";
            g2d.drawString("[" + modeText + "]", lastWidth - 390, 36);
        }

        // 크래시 났거나 에러 터지면 화면 정중앙에 반투명 커스텀 경고창 알림 띄우기
        if (!errorMessage.isEmpty()) {
            int popupWidth = 380;
            int popupHeight = 60;
            int popupX = (lastWidth - popupWidth) / 2;
            int popupY = 60 + (lastHeight - 60 - popupHeight) / 2;

            g2d.setColor(new Color(0, 0, 0, 190));
            g2d.fillRoundRect(popupX, popupY, popupWidth, popupHeight, 15, 15);

            g2d.setColor(new Color(231, 76, 60));
            g2d.setStroke(new BasicStroke(2.0f));
            g2d.drawRoundRect(popupX, popupY, popupWidth, popupHeight, 15, 15);

            g2d.setColor(Color.WHITE);
            g2d.setFont(new Font("맑은 고딕", Font.BOLD, 15));
            g2d.drawString("⚠️ " + errorMessage, popupX + 25, popupY + 36);
        }
    }

    // 상단바에 들어갈 컴포넌트 버튼들과 액션 매핑
    private void initControlUI() {
        btnObstacleMode = new JButton("장애물 배치");
        btnGoalMode = new JButton("목적지 설정");
        btnUndo = new JButton("되돌리기 (Undo)");
        btnRedo = new JButton("다시실행 (Redo)");
        btnStartSimulation = new JButton("자율주행 시작 (RUN)");
        btnResetRobot = new JButton("차량 위치 초기화");
        btnResetSimulation = new JButton("전체 리셋");

        Font btnFont = new Font("맑은 고딕", Font.BOLD, 12);

        // 상단 UI 컴포넌트 배치 픽셀 좌표 하드코딩한 구역
        btnObstacleMode.setBounds(20, 12, 110, 35);
        btnGoalMode.setBounds(140, 12, 110, 35);
        btnUndo.setBounds(260, 12, 135, 35);
        btnRedo.setBounds(405, 12, 135, 35);
        btnStartSimulation.setBounds(550, 12, 170, 35);
        btnResetRobot.setBounds(730, 12, 140, 35);
        btnResetSimulation.setBounds(880, 12, 90, 35);

        JButton[] buttons = {btnObstacleMode, btnGoalMode, btnUndo, btnRedo, btnStartSimulation, btnResetRobot, btnResetSimulation};
        for (JButton btn : buttons) {
            btn.setFont(btnFont);
        }

        btnStartSimulation.setForeground(Color.WHITE);
        updateButtonStates();

        // 버튼 리스너 세팅
        btnObstacleMode.addActionListener(e -> { currentEditMode = EditMode.ADD_OBSTACLE; clearError(); });
        btnGoalMode.addActionListener(e -> { currentEditMode = EditMode.SET_GOAL; clearError(); });
        btnUndo.addActionListener(e -> { performUndo(); clearError(); });
        btnRedo.addActionListener(e -> { performRedo(); clearError(); });

        btnStartSimulation.addActionListener(e -> {
            clearError();
            if (currentState == SystemState.EDIT) {
                boolean pathFound = replanAStarPath();
                if (!pathFound) {
                    triggerError("목적지까지 갈 수 있는 경로가 막혀있습니다!");
                    return;
                }
                currentState = SystemState.RUN;
            } else {
                currentState = SystemState.EDIT;
                speed = 0;
                steering = 0;
            }
            updateButtonStates();
            repaint();
        });

        // 차만 처음 시작점으로 텔레포트 시킴 (이미 설치된 장애물들은 유지)
        btnResetRobot.addActionListener(e -> {
            clearError();
            if (currentState == SystemState.EDIT) {
                robotXRatio = initialXRatio;
                robotYRatio = initialYRatio;
                robotTheta = 0;
                speed = 0;
                steering = 0;
                calculatedPath.clear();
                currentPathTargetIndex = 0;
                repaint();
            }
        });

        // 완전 초기화 (하드리셋)
        btnResetSimulation.addActionListener(e -> {
            clearError();
            currentState = SystemState.EDIT;
            currentEditMode = EditMode.ADD_OBSTACLE;
            robotXRatio = initialXRatio;
            robotYRatio = initialYRatio;
            robotTheta = 0;
            speed = 0;
            steering = 0;
            obstacles.clear();
            undoStack.clear();
            redoStack.clear();
            calculatedPath.clear();
            currentPathTargetIndex = 0;
            goalXRatio = 1750.0 / 1920.0;
            goalYRatio = 540.0 / 1080.0;
            updateButtonStates();
            repaint();
        });

        add(btnObstacleMode);
        add(btnGoalMode);
        add(btnUndo);
        add(btnRedo);
        add(btnStartSimulation);
        add(btnResetRobot);
        add(btnResetSimulation);
    }
}