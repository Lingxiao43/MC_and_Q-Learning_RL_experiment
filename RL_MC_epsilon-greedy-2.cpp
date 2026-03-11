#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <iomanip>
#include <thread>
#include <chrono>
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits> 

// ==========================================
// 全局配置与常数
// ==========================================
const int GRID_SIZE = 18;
const int NUM_ACTIONS = 5; // 上, 下, 左, 右, 原地

// 动作枚举
enum Action { UP = 0, DOWN, LEFT, RIGHT, STAY };
// 状态结构体
struct State {
    int x, y;
    bool operator==(const State& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const State& other) const {
        return !(*this == other);
    }
};

// 记录的一步经历
struct Step {
    State s;
    int a;
    double r;
};

// 地图格子类型
enum CellType { WHITE, YELLOW, BLUE };

// ==========================================
// 地图环境类
// ==========================================
class GridWorld {
public:
    CellType grid[GRID_SIZE][GRID_SIZE];
    
private:
    State startPos;
    State goalPos;
    
    // 奖励设置
    double r_white;
    double r_yellow;
    double r_blue;
    double r_boundary;

public:
    GridWorld() {
        r_white = 0.0;
        r_yellow = -10.0;
        r_blue = 100.0;
        r_boundary = -5.0;
    }

    void setRewards(double w, double y, double b, double bound) {
        r_white = w;
        r_yellow = y;
        r_blue = b;
        r_boundary = bound;
    }

    // 获取地图中白色格子（有效起点）的总数
    int getWhiteCellCount() const {
        int count = 0;
        for(int i=0; i<GRID_SIZE; ++i)
            for(int j=0; j<GRID_SIZE; ++j)
                if(grid[i][j] == WHITE) count++;
        return count;
    }

    bool isMapValid(const std::vector<State>& yellows, State goal) {
        bool visited[GRID_SIZE][GRID_SIZE] = {false};
        std::queue<State> q;
        
        q.push(goal);
        visited[goal.x][goal.y] = true;

        int reachableWhiteCount = 0;
        int totalWhiteCount = (GRID_SIZE * GRID_SIZE) - 1 - yellows.size(); 

        while (!q.empty()) {
            State curr = q.front();
            q.pop();

            int dx[] = {-1, 1, 0, 0};
            int dy[] = {0, 0, -1, 1};

            for (int i = 0; i < 4; ++i) {
                int nx = curr.x + dx[i];
                int ny = curr.y + dy[i];

                if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
                    bool isYellow = false;
                    for(auto& y : yellows) if(y.x == nx && y.y == ny) isYellow = true;
                    
                    if (!visited[nx][ny] && !isYellow) {
                        visited[nx][ny] = true;
                        q.push({nx, ny});
                        reachableWhiteCount++;
                    }
                }
            }
        }
        return reachableWhiteCount >= totalWhiteCount;
    }

    void generateMap() {
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> dist(0, GRID_SIZE - 1);

        bool valid = false;
        while (!valid) {
            for (int i = 0; i < GRID_SIZE; ++i)
                for (int j = 0; j < GRID_SIZE; ++j)
                    grid[i][j] = WHITE;

            goalPos = {dist(rng), dist(rng)};
            grid[goalPos.x][goalPos.y] = BLUE;

            int numYellows = GRID_SIZE * GRID_SIZE * 0.35; 
            std::vector<State> yellows;
            int placed = 0;
            while (placed < numYellows) {
                int r = dist(rng);
                int c = dist(rng);
                if (grid[r][c] == WHITE) {
                    grid[r][c] = YELLOW;
                    yellows.push_back({r, c});
                    placed++;
                }
            }

            if (isMapValid(yellows, goalPos)) {
                valid = true;
            }
        }
        
        do {
            startPos = {dist(rng), dist(rng)};
        } while (grid[startPos.x][startPos.y] != WHITE);
    }

    State getStartPos() const { return startPos; }
    State getGoalPos() const { return goalPos; }

    std::tuple<State, double, bool> step(State current, int action) {
        int dx = 0, dy = 0;
        if (action == UP) dx = -1;
        else if (action == DOWN) dx = 1;
        else if (action == LEFT) dy = -1;
        else if (action == RIGHT) dy = 1;

        int nx = current.x + dx;
        int ny = current.y + dy;
        double reward = 0;
        State nextState = current;

        if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) {
            reward = r_boundary;
            nextState = current; 
        } else {
            nextState = {nx, ny};
            if (grid[nx][ny] == YELLOW){ 
                reward = r_yellow;
                return {nextState, reward, true}; // 踩入陷阱直接结束
            }
            else if (grid[nx][ny] == BLUE) reward = r_blue;
            else reward = r_white;
        }

        bool done = (grid[nextState.x][nextState.y] == BLUE);
        return {nextState, reward, done};
    }

    void render(State robotPos) {
        #ifdef _WIN32
            system("cls");
        #else
            system("clear");
        #endif

        std::cout << "  ";
        for(int j=0; j<GRID_SIZE; ++j) std::cout << std::setw(2) << j << " ";
        std::cout << "\n";

        for (int i = 0; i < GRID_SIZE; ++i) {
            std::cout << std::setw(2) << i << " ";
            for (int j = 0; j < GRID_SIZE; ++j) {
                if (i == robotPos.x && j == robotPos.y) {
                    std::cout << "@  "; 
                } else if (grid[i][j] == BLUE) {
                    std::cout << "B  "; 
                } else if (grid[i][j] == YELLOW) {
                    std::cout << "Y  "; 
                } else {
                    std::cout << ".  "; 
                }
            }
            std::cout << "\n";
        }
        std::cout << "-------------------\n";
        std::cout << "Map: [.]White [Y]Yellow [B]Blue[@]Robot\n";
    }
};

// ==========================================
// MC Agent 类
// ==========================================
class MCAgent {
private:
    double Q[GRID_SIZE][GRID_SIZE][NUM_ACTIONS];

    double epsilon;
    double gamma;

public:
    MCAgent() {
        epsilon = 1.0;
        gamma = 1.0; 

        for(int i=0; i<GRID_SIZE; ++i)
            for(int j=0; j<GRID_SIZE; ++j)
                for(int a=0; a<NUM_ACTIONS; ++a) {
                    Q[i][j][a] = 0.0; 
                }
    }

    // 将 Epsilon 的控制权完全交给 Main 函数的外部闭环
    void setEpsilon(double eps) {
        epsilon = eps;
    }

    double getEpsilon() const { 
        return epsilon; 
    }

    int chooseAction(State s) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        if (dis(gen) < epsilon) {
            std::uniform_int_distribution<> actionDis(0, NUM_ACTIONS - 1);
            return actionDis(gen);
        } else {
            return getBestAction(s); // 直接复用 getBestAction (打破平局已在下方处理)
        }
    }

    void update(const std::vector<Step>& episode) {
        double G = 0;
        
        int firstVisitTime[GRID_SIZE][GRID_SIZE][NUM_ACTIONS];
        for(int i=0; i<GRID_SIZE; ++i)
            for(int j=0; j<GRID_SIZE; ++j)
                for(int a=0; a<NUM_ACTIONS; ++a)
                    firstVisitTime[i][j][a] = -1;

        for (int t = 0; t < episode.size(); ++t) {
            State s = episode[t].s;
            int a = episode[t].a;
            if (firstVisitTime[s.x][s.y][a] == -1) {
                firstVisitTime[s.x][s.y][a] = t;
            }
        }

        for (int t = episode.size() - 1; t >= 0; --t) {
            State St = episode[t].s;
            int At = episode[t].a;
            double Rt = episode[t].r;

            G = gamma * G + Rt;

            if (t == firstVisitTime[St.x][St.y][At]) {
                double alpha = 0.05; 
                Q[St.x][St.y][At] += alpha * (G - Q[St.x][St.y][At]);
            }
        }
    }
    
    int getBestAction(State s) {
        int bestA = 0;
        double maxQ = -1e9;
        std::vector<int> bestActions;
        for (int a = 0; a < NUM_ACTIONS; ++a) {
            if (Q[s.x][s.y][a] > maxQ) {
                maxQ = Q[s.x][s.y][a];
                bestActions.clear();
                bestActions.push_back(a);
            } else if (std::abs(Q[s.x][s.y][a] - maxQ) < 1e-5) {
                bestActions.push_back(a);
            }
        }
        
        // 随机打破平局，防止陷入某种确定性的死循环
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> idxDist(0, bestActions.size() - 1);
        return bestActions[idxDist(gen)];
    }
};

// ==========================================
// 主程序
// ==========================================
int main() {
    GridWorld env;
    
    double r_w, r_y, r_b, r_out;
    double initial_eps;
    int ep_len;
    int max_episodes;
    
    // 新增自适应参数
    double window_k; 
    double decay_power;

    std::string GRID_SIZE_STR = std::to_string(GRID_SIZE);

    std::cout << "=== 自适应滑动窗口 MC 强化学习 (" << GRID_SIZE_STR << "x" << GRID_SIZE_STR << ") ===\n";
    std::cout << "设置 White 格子奖励 (建议 -1): "; std::cin >> r_w;
    std::cout << "设置 Yellow 格子奖励 (建议 -1000): "; std::cin >> r_y;
    std::cout << "设置 Blue 格子奖励 (建议 1000): "; std::cin >> r_b;
    std::cout << "设置 越界 奖励 (建议 -1000): "; std::cin >> r_out;
    env.setRewards(r_w, r_y, r_b, r_out);

    std::cout << "最大 Episode 步数 (建议 500): "; std::cin >> ep_len;
    std::cout << "训练最大上限 Episode 次数 (防止无限跑, 建议 1000000): "; std::cin >> max_episodes;
    std::cout << "初始 Epsilon (建议 0.8): "; std::cin >> initial_eps;
    
    std::cout << "--------------------------------------------------\n";
    std::cout << "[自适应参数] 窗口缩放系数 K (窗口大小 = K * 白色格子总数)。\n";
    std::cout << "这意味着窗口要包容每个起点平均被遍历 K 次，建议 50~200: "; std::cin >> window_k;
    std::cout << "[自适应参数] Epsilon 衰减幂率 (Eps = 初始Eps * (1 - 成功率)^幂)。\n";
    std::cout << "建议 1~5。越大则 Epsilon 掉得越快: "; std::cin >> decay_power;

    std::cout << "正在生成地图...\n";
    env.generateMap();
    std::cout << "地图生成完毕！\n";
    env.render({-1, -1}); 

    // 计算滑动窗口大小
    int num_white_cells = env.getWhiteCellCount();
    int window_size = static_cast<int>(num_white_cells * window_k);
    std::cout << "-> 探测到 " << num_white_cells << " 个白色合法起点。\n";
    std::cout << "-> 动态确定的滑动窗口大小为: " << window_size << " 个 Episodes。\n";
    std::cout << "--------------------------------------------------\n";

    MCAgent agent;
    agent.setEpsilon(initial_eps);

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, GRID_SIZE - 1);

    // ==========================================
    // 环形数组实现的高效滑动窗口
    // ==========================================
    std::vector<bool> success_window(window_size, false);
    int window_idx = 0;
    int current_successes = 0;
    int filled_size = 0; // 记录窗口目前蓄了多少水

    int print_interval = std::max(10000, window_size / 2); // 动态打印频率

    std::cout << "开始自适应训练 (达到 100% 成功率将自动早停)...\n";

    for (int e = 0; e < max_episodes; ++e) {
        std::vector<Step> episode;
        State state;
        do {
            state.x = dist(rng);
            state.y = dist(rng);
        } while (env.grid[state.x][state.y] != WHITE);
        
        bool done = false;
        bool is_success = false;

        for (int step = 0; step < ep_len; ++step) {
            int action = agent.chooseAction(state);
            auto [nextState, reward, isDone] = env.step(state, action);

            if (!isDone && step == ep_len - 1) {
                reward = -200.0; 
            }
            
            episode.push_back({state, action, reward});
            state = nextState;
            done = isDone;
            
            if (done) {
                if (env.grid[nextState.x][nextState.y] == BLUE) {
                    is_success = true;
                }
                break;
            }
        }
        
        agent.update(episode);

        // ==========================================
        // 核心：维护滑动窗口并计算实时成功率
        // ==========================================
        if (filled_size < window_size) {
            success_window[window_idx] = is_success;
            if (is_success) current_successes++;
            filled_size++;
        } else {
            if (success_window[window_idx]) current_successes--;
            success_window[window_idx] = is_success;
            if (is_success) current_successes++;
        }
        window_idx = (window_idx + 1) % window_size;

        double current_sr = (double)current_successes / filled_size;

        // ==========================================
        // 核心：基于实时成功率的自适应 Epsilon 下降
        // ==========================================
        // 为了防止刚开始窗口很小时数据波动太大，我们稍微抑制一下前期的衰减
        double effective_sr = current_sr;
        if (filled_size < window_size / 5) {
            effective_sr = 0.0; // 窗口蓄满 20% 之前，保持满 Epsilon 尽情拓荒
        }

        // 你构想的终极数学公式：eps = initial_eps * (1 - SR)^k
        double last_eps = agent.getEpsilon();
        double new_eps = initial_eps * std::pow(1.0 - effective_sr, decay_power);
        if (new_eps < double(0.00000001)) new_eps = 0.0;
        agent.setEpsilon(new_eps);

        // ==========================================
        // 核心：完美的早停机制 (Early Stopping)
        // ==========================================
        // 只有当窗口完全蓄满，且成功率 100% 时（或者极度接近 1.0），才认定出师
        if (filled_size == window_size && current_successes == window_size) {
            std::cout << "\n==================================================\n";
            std::cout << " 奇迹降临！在第 " << e + 1 << " 个 Episode 达成完美 100% 成功率！\n";
            std::cout << " 智能体已彻底掌握全局最优策略，触发自动早停。\n";
            std::cout << "==================================================\n";
            break;
        }

        if ((e + 1) % print_interval == 0 || e == 0) {
            std::cout << "Episode " << std::setw(7) << e + 1 
                      << " | 窗口填充: " << std::setw(3) << (int)((double)filled_size/window_size*100) << "%"
                      << " | Epsilon: " << std::fixed << std::setprecision(8) << agent.getEpsilon() 
                      << " | 滑动成功率: " << current_successes << "/" << filled_size 
                      << " (" << std::fixed << std::setprecision(2) << current_sr * 100 << "%)" << std::endl;
        }
    }

    // ==========================================
    // 交互演示
    // ==========================================
    std::cout << "\n训练结束。进入交互演示模式。\n";
    #ifdef _WIN32
        system("pause");
    #else
        std::cout << "Press Enter to continue...";
    #endif
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    while (true) {
        env.render({-1, -1}); 

        std::cout << "\n=== 测试新起点 ===\n";
        std::cout << "请输入起点坐标 (行 列)，范围 0-" << GRID_SIZE - 1 << " (例如: 0 0)\n";
        std::cout << "输入 '-1 -1' 退出程序: ";
        
        int startX, startY;
        if (!(std::cin >> startX >> startY)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        if (startX == -1) break;

        if (startX < 0 || startX >= GRID_SIZE || startY < 0 || startY >= GRID_SIZE) {
            std::cout << "错误: 坐标越界。\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        State currentState = {startX, startY};
        
        if (env.grid[startX][startY] == BLUE) {
            std::cout << "出生在终点！\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            continue;
        }

        bool done = false;
        int steps = 0;

        while (!done && steps < ep_len) {
            env.render(currentState);
            
            int bestA = agent.getBestAction(currentState);
            
            std::cout << "Step: " << steps 
                      << " | Pos: (" << currentState.x << ", " << currentState.y << ")"
                      << " | Action: ";
            
            if(bestA==UP) std::cout<<"UP";
            if(bestA==DOWN) std::cout<<"DOWN";
            if(bestA==LEFT) std::cout<<"LEFT";
            if(bestA==RIGHT) std::cout<<"RIGHT";
            if(bestA==STAY) std::cout<<"STAY";
            std::cout << std::endl;

            auto [nextState, reward, isDone] = env.step(currentState, bestA);
            currentState = nextState;
            done = isDone;
            steps++;

            std::this_thread::sleep_for(std::chrono::milliseconds(150)); 
        }

        env.render(currentState);

        if (done) {
            if (env.grid[currentState.x][currentState.y] == BLUE) {
                std::cout << "\n>>>  恭喜！成功到达蓝色目标!  <<<\n";
            } else if (env.grid[currentState.x][currentState.y] == YELLOW) {
                std::cout << "\n>>>  惨死！你踩到了黄色陷阱，当场去世！ <<<\n";
            } else {
                std::cout << "\n>>> 莫名其妙地结束了... <<<\n";
            }
        } else {
            std::cout << "\n>>>  失败：步数耗尽 (超时)  <<<\n";
        }

        std::cout << "\n(按回车继续...)";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();
    }

    return 0;
}