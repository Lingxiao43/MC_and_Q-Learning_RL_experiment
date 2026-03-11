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
    double r_white, r_yellow, r_blue, r_boundary;

public:
    GridWorld() {
        r_white = -1.0;
        r_yellow = -1000.0;
        r_blue = 1000.0;
        r_boundary = -1000.0;
    }

    void setRewards(double w, double y, double b, double bound) {
        r_white = w; r_yellow = y; r_blue = b; r_boundary = bound;
    }

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
            if (isMapValid(yellows, goalPos)) valid = true;
        }
    }

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
            if (grid[nx][ny] == YELLOW) return {nextState, r_yellow, true}; // 踩雷即死
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
                if (i == robotPos.x && j == robotPos.y) std::cout << "@  "; 
                else if (grid[i][j] == BLUE) std::cout << "B  "; 
                else if (grid[i][j] == YELLOW) std::cout << "Y  "; 
                else std::cout << ".  "; 
            }
            std::cout << "\n";
        }
        std::cout << "-------------------\n";
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
                for(int a=0; a<NUM_ACTIONS; ++a) 
                    Q[i][j][a] = 0.0; 
    }

    void setEpsilon(double eps) { epsilon = eps; }
    double getEpsilon() const { return epsilon; }

    int chooseAction(State s) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        if (dis(gen) < epsilon) {
            std::uniform_int_distribution<> actionDis(0, NUM_ACTIONS - 1);
            return actionDis(gen);
        } else {
            return getBestAction(s);
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
            if (firstVisitTime[s.x][s.y][a] == -1) firstVisitTime[s.x][s.y][a] = t;
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
    
    double initial_eps;
    int ep_len;
    int max_episodes;
    
    // 自适应滑动窗口与平台期衰减参数
    double window_k; 
    int patience;
    double threshold;
    double decay_rate;

    std::cout << "=== 基于平台期检测 (Plateau Detection) 的强化学习 ===\n";
    std::cout << "最大 Episode 步数 (建议 500): "; std::cin >> ep_len;
    std::cout << "绝对最大 Episode 训练上限 (建议 1000000): "; std::cin >> max_episodes;
    std::cout << "初始 Epsilon (建议 0.8): "; std::cin >> initial_eps;
    
    std::cout << "\n--- 平台期(瓶颈)检测机制配置 ---\n";
    std::cout << "1. 窗口系数 K (窗口大小=K*白格数，建议 50): "; std::cin >> window_k;
    std::cout << "2. 容忍次数 Patience (连续 N 个窗口不进步则衰减，建议 3): "; std::cin >> patience;
    std::cout << "3. 变化阈值 Threshold (成功率变化小于此值视为停滞，建议 0.01 即1%): "; std::cin >> threshold;
    std::cout << "4. 衰减率 Decay Rate (触发停滞时 Eps = Eps * Rate，建议 0.5): "; std::cin >> decay_rate;

    std::cout << "正在生成地图...\n";
    env.generateMap();

    env.render({-1, -1}); 
    
    int num_white_cells = env.getWhiteCellCount();
    // 强制把窗口大小设置为偶数，方便计算
    int window_size = static_cast<int>(num_white_cells * window_k);
    std::cout << "-> 探测到 " << num_white_cells << " 个白色合法起点。\n";
    std::cout << "-> 滑动窗口大小确立为: " << window_size << " 个 Episodes。\n";

    MCAgent agent;
    agent.setEpsilon(initial_eps);

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, GRID_SIZE - 1);

    // 环形数组
    std::vector<bool> success_window(window_size, false);
    int window_idx = 0;
    int current_successes = 0;
    int filled_size = 0;

    // 平台期检测变量
    double last_checked_sr = 0.0;
    double max_sr = 0.0;
    int patience_counter = 0;
    
    // 我们每隔一个完整的 window_size 检查一次，保证统计样本是完全独立刷新的
    int check_interval = window_size * 0.4; 

    std::cout << "开始自适应训练 (Epsilon将在遇到瓶颈时自动下降)...\n";

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

            if (!isDone && step == ep_len - 1) reward = -200.0; 
            
            episode.push_back({state, action, reward});
            state = nextState;
            done = isDone;
            
            if (done) {
                if (env.grid[nextState.x][nextState.y] == BLUE) is_success = true;
                break;
            }
        }
        
        agent.update(episode);

        // 维护滑动窗口
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

        // ==========================================
        // 核心：基于平台期 (Plateau) 的 Epsilon 衰减
        // ==========================================
        if (filled_size == window_size) {
            double current_sr = (double)current_successes / window_size;

            // 完美的早停：100% 成功且 Epsilon 已经降到底
            if (current_successes == window_size && agent.getEpsilon() == 0.0) {
                std::cout << "\n==================================================\n";
                std::cout << " 奇迹降临！达成完美 100% 成功率 (Eps=0)！早停触发！\n";
                std::cout << "==================================================\n";
                break;
            }

            // 每隔 check_interval (即一个完整窗口长度)，评估一次是否停滞
            if ((e + 1) % check_interval == 0) {
                //double sr_diff = std::abs(current_sr - last_checked_sr);
                //double sr_diff = current_sr - last_checked_sr;
                double sr_diff = current_sr - max_sr;
                
                // 如果变化率极小，说明潜力榨干了，耐心值增加
                if (std::abs(sr_diff) <= threshold) {
                    patience_counter++;
                } 
                else if(sr_diff < 0){
                    patience_counter++; // 甚至如果成功率下降了，也视为停滞，增加耐心值
                }
                else {
                    // 如果变化率依然可观，重置耐心值，继续让它在这个 Eps 下学习
                    patience_counter = 0; 
                }

                // 耐心耗尽，必须下降 Epsilon 帮它突破瓶颈！
                if (patience_counter >= patience) {
                    double current_eps = agent.getEpsilon();
                    double new_eps = agent.getEpsilon() * decay_rate;
                    
                    // 物理清零阈值：当 Epsilon 小于 0.01 时，彻底关掉随机性，进入终极考试模式
                    // if(current_eps - new_eps < 0.0001) new_eps = current_eps * 0.2;
                    // else if(current_eps - new_eps < 0.01) new_eps = current_eps * 0.5;
                    if (new_eps < 0.0000001) new_eps = 0.0; 
                    
                    agent.setEpsilon(new_eps);
                    patience_counter = 0; // 重置耐心
                    
                    std::cout << "\n>>>[自动降维打击] 连续 " << patience << " 个窗口成功率停滞在 " 
                              << std::fixed << std::setprecision(2) << current_sr * 100 << "% 附近。\n"
                              << ">>> 榨干当前潜力，正在将 Epsilon 降低至: " 
                              << std::fixed << std::setprecision(8) << new_eps << " <<<\n\n";
                }
                if(current_sr > max_sr) {
                    max_sr = current_sr;
                }
                last_checked_sr = current_sr; // 记录本次检查的 SR
            }
        }

        // 打印信息：只在整窗口节点打印，画面更清爽
        if (filled_size == window_size && (e + 1) % check_interval == 0) {
            double current_sr = (double)current_successes / window_size;
            std::cout << "Episode " << std::setw(7) << e + 1 
                      << " | Epsilon: " << std::fixed << std::setprecision(8) << agent.getEpsilon() 
                      << " | 最近一窗成功率: " << std::fixed << std::setprecision(2) << current_sr * 100 << "%"
                      << " | 耐心值: " << patience_counter << "/" << patience << std::endl;
        } else if (filled_size < window_size && (e + 1) % (window_size/4) == 0) {
            // 窗口蓄水期打印
            std::cout << "Episode " << std::setw(7) << e + 1 
                      << " | 窗口蓄水中 (" << (int)((double)filled_size/window_size*100) << "%)..." << std::endl;
        }
    }

    // (演示代码保持原样...此处省略以节省空间，可直接复用你之前的 while(true) 演示部分)
    // ...

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