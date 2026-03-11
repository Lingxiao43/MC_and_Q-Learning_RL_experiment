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
#include <numbers>  // std::numbers::pi（C++20）

// ==========================================
// 跨平台非阻塞键盘检测 (用于按 'q' 随时退出演示)
// ==========================================
#ifdef _WIN32
    #include <conio.h>
    bool check_quit() {
        if (_kbhit()) {
            char ch = _getch();
            return (ch == 'q' || ch == 'Q');
        }
        return false;
    }
#else
    #include <termios.h>
    #include <unistd.h>
    #include <fcntl.h>
    bool check_quit() {
        struct termios oldt, newt;
        int ch;
        int oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if(ch != EOF) {
            if(ch == 'q' || ch == 'Q') return true;
        }
        return false;
    }
#endif

// ==========================================
// 全局配置与常数
// ==========================================
const int GRID_SIZE = 38;
const int NUM_ACTIONS = 5; // 上, 下, 左, 右, 原地

const double pi = 3.14159265358979323846;

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
    // *** 修改：将 grid 设为 public，以便 main 函数检查随机起点是否合法 ***
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
        r_white = -1.0;
        r_yellow = -1000.0;
        r_blue = 1000.0;
        r_boundary = -1000.0;
    }

    void setRewards(double w, double y, double b, double bound) {
        r_white = w;
        r_yellow = y;
        r_blue = b;
        r_boundary = bound;
    }

    // 使用BFS检查地图连通性
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

        // 辅助函数：执行修剪，把无法到达终点的白格变成陷阱
    bool pruneUnreachableAreas(State goal) {
        bool visited[GRID_SIZE][GRID_SIZE] = {false};
        std::queue<State> q;
        
        // 1. 从终点开始反向 BFS
        q.push(goal);
        visited[goal.x][goal.y] = true;

        while (!q.empty()) {
            State curr = q.front();
            q.pop();

            int dx[] = {-1, 1, 0, 0};
            int dy[] = {0, 0, -1, 1};

            for (int i = 0; i < 4; ++i) {
                int nx = curr.x + dx[i];
                int ny = curr.y + dy[i];

                if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
                    // 只要不是黄色陷阱，都可以通行（反向）
                    if (grid[nx][ny] != YELLOW && !visited[nx][ny]) {
                        visited[nx][ny] = true;
                        q.push({nx, ny});
                    }
                }
            }
        }

        // 2. 遍历全图，把所有“没被访问到”的白色格子，强制变成黄色陷阱
        int converted_count = 0;
        for(int i=0; i<GRID_SIZE; ++i) {
            for(int j=0; j<GRID_SIZE; ++j) {
                if (grid[i][j] == WHITE && !visited[i][j]) {
                    grid[i][j] = YELLOW; // <--- 关键修改：物理填埋死胡同
                    //yellows.push_back({i, j});
                    converted_count++;
                }
            }
        }
        
        std::cout << "-> 地图生成优化：已自动填埋 " << converted_count << " 个无法到达终点的孤岛格子。\n";
        return converted_count < (GRID_SIZE * GRID_SIZE * 0.05); // 如果填埋过多，可能说明地图设计有问题
    }

    void generateMap() {
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> dist(0, GRID_SIZE - 1);

        bool valid = false;
        while (!valid) {
            // 1. 重置全白
            for (int i = 0; i < GRID_SIZE; ++i)
                for (int j = 0; j < GRID_SIZE; ++j)
                    grid[i][j] = WHITE;

            // 2. 放置唯一的蓝色终点
            //goalPos = {dist(rng), dist(rng)};
            goalPos = {GRID_SIZE - 1, 0}; // 固定终点在左下角，增加挑战性
            grid[goalPos.x][goalPos.y] = BLUE;
            

            // 3. 放置黄色陷阱
            int numYellows = GRID_SIZE * GRID_SIZE * 0.38; 
            int placed = 0;
            int attempts = 0;
            std::vector<State> yellows;
            
            while (placed < numYellows && attempts < numYellows * 5) {
                int r = dist(rng);
                int c = dist(rng);
                if (grid[r][c] == WHITE) {
                    grid[r][c] = YELLOW;
                    yellows.push_back({r, c});
                    placed++;
                }
                attempts++;
            }

            if(pruneUnreachableAreas(goalPos))
                valid = true; // 只要修剪后地图还算合理，就接受这个地图

            // // 4. 验证连通性
            // if (isMapValid(yellows, goalPos)) {
            //     valid = true;
            // }
        }
        
        // 随机生成一个默认起点
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
                // ==========================================
                // 核心修改：踩入陷阱直接结束回合！
                // ==========================================
                //reward = -500.0; // 给一个极度恐怖的惩罚
                return {nextState, reward, true}; // true 表示 done (游戏结束)
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
        std::cout << "Map: [.]White [Y]Yellow [B]Blue [@]Robot\n";
    }
};

// ==========================================
// MC Agent 类
// ==========================================
class MCAgent {
public:
    double Q[GRID_SIZE][GRID_SIZE][NUM_ACTIONS];
    int returnsCount[GRID_SIZE][GRID_SIZE][NUM_ACTIONS];

    double epsilon;
    double min_epsilon;
    double initial_epsilon; // 加上这个，记住初始值
    //double linear_step; // 线性衰减步长
    bool epsilon_decay;
    double gamma;

    // === 新增：用于分段衰减的进度记录 ===
    int total_episodes;
    int current_episode; 

public:
    // *** 修改：构造函数增加 total_episodes 用于计算线性衰减 ***
    MCAgent(double eps, bool decay, int total_episodes, double min_eps = 0.0) {
        epsilon = eps;
        initial_epsilon = eps; // 记录最初输入的 epsilon (比如 0.8)
        epsilon_decay = decay;
        min_epsilon = min_eps;
        gamma = 1.0; // 建议设为 1.0

        // === 线性衰减设置 ===
        // 让 Epsilon 在训练 80% 进度时降到最低，剩下 20% 稳定利用
        // if (decay && total_episodes > 0) {
        //     double decay_duration = total_episodes * 0.8;
        //     linear_step = (eps - min_eps) / decay_duration;
        // } else {
        //     linear_step = 0;
        // }

        this->total_episodes = total_episodes;
        current_episode = 0;

        for(int i=0; i<GRID_SIZE; ++i)
            for(int j=0; j<GRID_SIZE; ++j)
                for(int a=0; a<NUM_ACTIONS; ++a) {
                    Q[i][j][a] = 0.0; 
                    returnsCount[i][j][a] = 0;
                }
    }

    int chooseAction(State s) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);

        // 如果随机数小于 epsilon，就随机选择一个动作（探索）
        if (dis(gen) < epsilon) {
            std::uniform_int_distribution<> actionDis(0, NUM_ACTIONS - 1);
            return actionDis(gen);
        } else {
            // 否则选择 Q 值最高的动作（利用），如果有多个同样高的动作，则随机选择一个
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
            std::uniform_int_distribution<> idxDist(0, bestActions.size() - 1);
            return bestActions[idxDist(gen)];
        }
    }

    //getBestActionQ 的实现，返回当前状态下 最高Q值（服务于Q-learning）
    double getBestActionQ(State s) {
        double maxQ = -1e9;
        for (int a = 0; a < NUM_ACTIONS; ++a) {
            if (Q[s.x][s.y][a] > maxQ) {
                maxQ = Q[s.x][s.y][a];
            }
        }
        return maxQ;
    }

    // *** 优化：First-Visit MC 更新 (O(N) 复杂度) ***
    void update(const std::vector<Step>& episode) {
        //double G = 0;
        
        // // 1. 记录每个 (状态, 动作) 第一次出现的时间步
        // int firstVisitTime[GRID_SIZE][GRID_SIZE][NUM_ACTIONS];
        // for(int i=0; i<GRID_SIZE; ++i)
        //     for(int j=0; j<GRID_SIZE; ++j)
        //         for(int a=0; a<NUM_ACTIONS; ++a)
        //             firstVisitTime[i][j][a] = -1;

        // // 正向遍历
        // for (int t = 0; t < episode.size(); ++t) {
        //     State s = episode[t].s;
        //     int a = episode[t].a;
        //     if (firstVisitTime[s.x][s.y][a] == -1) {
        //         firstVisitTime[s.x][s.y][a] = t;
        //     }
        // }

        // // 2. 反向遍历
        // for (int t = episode.size() - 1; t >= 0; --t) {
        //     State St = episode[t].s;
        //     int At = episode[t].a;
        //     double Rt = episode[t].r;

        //     G = gamma * G + Rt;

        //     // 常数α学习率的MC 更新
        //     if (t == firstVisitTime[St.x][St.y][At]) {
        //         //returnsCount[St.x][St.y][At]++;
        //         // // 标准增量平均
        //         //Q[St.x][St.y][At] += (G - Q[St.x][St.y][At]) / returnsCount[St.x][St.y][At];

        //         // 【启用】常数学习率 Alpha
        //         // 这意味着它更看重“最近的成功经验”，快速遗忘早期的“乱走经验”
        //         double alpha = 0.05; // 0.05 或 0.1 都可以
        //         Q[St.x][St.y][At] += alpha * (G - Q[St.x][St.y][At]);
        //     }
        // }

        // *** 线性衰减 ***
        // ----------------------------------
        // if (epsilon_decay) {
        //     epsilon -= linear_step;
        //     if (epsilon < min_epsilon) epsilon = min_epsilon;
        // }


        // 指数衰减
        // ---------------------------------
        // if (epsilon_decay) {
        //     current_episode++; // 每次 update 代表完成了一个 episode
            
        //     // 计算当前训练进度 (0.0 到 1.0 之间)
        //     double progress = (double)current_episode / total_episodes;

        //     int k = 4;
        //     epsilon = initial_epsilon;
        //     epsilon *= std::pow(1.0 - progress, k); // 进度越大，epsilon 越小
            
        //     // 确保不低于设定的最小值 (如果是 0.0 就无所谓了)
        //     if (epsilon < min_epsilon) epsilon = min_epsilon;
        // }

        // cos衰减
        // if (epsilon_decay) {
        //      current_episode++; // 每次 update 代表完成了一个 episode
        //      double progress = (double)current_episode / total_episodes;

        //      epsilon = initial_epsilon * std::pow((std::cos(pi * progress / 2)), 2); // 从 initial_epsilon 平滑衰减到 min_epsilon

        //      if (epsilon < min_epsilon) epsilon = min_epsilon;
        // }

        //断层衰减
         if (epsilon_decay) {
            current_episode++;
            double progress = (double)current_episode / total_episodes;
            if (progress >= 0.95) {
                epsilon = min_epsilon; // 95% 进度后直接降到最低，保持稳定利用
            }


        }
    }

    double getEpsilon() const { return epsilon; }
    
    int getBestAction(State s) {
         int bestA = 0;
         double maxQ = -1e9;
         for(int a=0; a<NUM_ACTIONS; ++a){
             if(Q[s.x][s.y][a] > maxQ){
                 maxQ = Q[s.x][s.y][a];
                 bestA = a;
             }
         }
         return bestA;
    }
};

// ==========================================
// 主程序
// ==========================================
int main() {
    GridWorld env;
    
    double r_w, r_y, r_b, r_out;
    double eps;
    int ep_len;
    int episodes;
    int choice_decay;

    std::string GRID_SIZE_STR = std::to_string(GRID_SIZE);

    std::cout << "=== Monte Carlo RL " << GRID_SIZE_STR << "x" << GRID_SIZE_STR << " GridWorld (Advanced) ===\n";
    // std::cout << "设置 White 格子奖励 (建议 -1): "; std::cin >> r_w;
    // std::cout << "设置 Yellow 格子奖励 (建议 -1000): "; std::cin >> r_y;
    // std::cout << "设置 Blue 格子奖励 (建议 1000): "; std::cin >> r_b;
    // std::cout << "设置 越界 奖励 (建议 -1000): "; std::cin >> r_out;
    
    // env.setRewards(r_w, r_y, r_b, r_out);

    //std::cout << "设置初始 Epsilon (建议 1.0 或 0.8): "; std::cin >> eps;
    eps = 1.0;
    //std::cout << "Epsilon 是否衰减? (1:是, 0:否): "; std::cin >> choice_decay;
    choice_decay = 1;    
    //std::cout << "最大Episode步数 (建议 500-1000): "; std::cin >> ep_len;
    ep_len = 5000;
    //std::cout << "训练 Episode 次数 (建议 500000以上): "; std::cin >> episodes;
    episodes = 1000000;

    std::cout << "正在生成地图...\n";
    env.generateMap();
    std::cout << "地图生成完毕！\n";
    env.render({-1, -1}); // 只画地图

    // 初始化智能体 (传入总次数用于线性衰减)
    MCAgent agent(eps, choice_decay == 1, episodes);

    // ==========================================
    // 训练循环 (Exploring Starts)
    // ==========================================
    std::cout << "开始训练 (启用随机起点 Exploring Starts)...\n";
    
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, GRID_SIZE - 1);

    int success_count = 0;

    for (int e = 0; e < episodes; ++e) {
        std::vector<Step> episode;
        
        // --- 随机起点逻辑 ---
        State state;
        // 强制在非墙、非终点的白色区域出生
        do {
            state.x = dist(rng);
            state.y = dist(rng);
        } while (env.grid[state.x][state.y] != WHITE);
        
        //bool done = false;
        //bool yellow_trap_triggered = false; // 记录是否踩过陷阱
        // for (int step = 0; step < ep_len; ++step) {
        //     int action = agent.chooseAction(state);
        //     auto [nextState, reward, isDone] = env.step(state, action);

        //     // 如果走到了最后一步依然没有结束，说明它在耗时间
        //     if (!isDone && step == ep_len - 1) {
        //         reward = -200.0; // 给一个比踩陷阱(-500)还严重的惩罚！逼它去冒险！
        //     }
            
        //     episode.push_back({state, action, reward});
        //     state = nextState;
        //     done = isDone;
            
            
        //     // if (env.grid[nextState.x][nextState.y] == YELLOW) {
        //     //     yellow_trap_triggered = true; // 记录踩过陷阱
        //     // }
        //     // if (done) {
        //     //     success_count++;
        //     //     break;
        //     // }
        //     if (done) {
        //         // ==========================================
        //         // 核心修复 2：严格判定成功
        //         // ==========================================
        //         // 确保只有真正踩到蓝色终点，才算 success
        //         if (env.grid[nextState.x][nextState.y] == BLUE) {
        //             success_count++;
        //         }
        //         break;
        //     }
        // }

        // Q-Learning
        bool done = false;
        double G = 0.0;
        for (int step = 0; step < ep_len; ++step) {
            int action = agent.chooseAction(state);
            auto [nextState, reward, isDone] = env.step(state, action);

            // 如果走到了最后一步依然没有结束，说明它在耗时间
            if (!isDone && step == ep_len - 1) {
                reward = -200.0; // 给一个比踩陷阱(-500)还严重的惩罚！逼它去冒险！
            }
            
            // 如果 isDone 为 true，说明游戏结束，后面没有未来收益了，gamma * maxQ 应该为 0
            if (isDone) {
                G = reward; 
            } else {
                G = reward + agent.gamma * agent.getBestActionQ(nextState); 
            }

            // 然后再进行更新
            agent.Q[state.x][state.y][action] += 0.05 * (G - agent.Q[state.x][state.y][action]);
            //episode.push_back({state, action, reward});
            state = nextState;
            done = isDone;
            
            
            // if (env.grid[nextState.x][nextState.y] == YELLOW) {
            //     yellow_trap_triggered = true; // 记录踩过陷阱
            // }
            // if (done) {
            //     success_count++;
            //     break;
            // }
            if (done) {
                // ==========================================
                // 核心修复 2：严格判定成功
                // ==========================================
                // 确保只有真正踩到蓝色终点，才算 success
                if (env.grid[nextState.x][nextState.y] == BLUE) {
                    success_count++;
                }
                break;
            }
        }
        agent.update(episode);

        int t = 50;
        if ((e + 1) % (episodes / t) == 0) {
        //if(agent.current_episode / agent.total_episodes > 0.9){
            std::cout << "Episode " << e + 1 << " / " << episodes 
                      << " | Epsilon: " << std::fixed << std::setprecision(8) << agent.getEpsilon() 
                      << " | Success Rate: " << success_count << "/" << (episodes/t) << '(' << std::fixed << std::setprecision(2) << (double)success_count / (episodes/t) * 100 << "%)" << std::endl;
            if(success_count == (episodes/t)) {
                std::cout << ">>> 训练中途达成 100% 成功率！提前结束训练。 <<<\n";
                break;
            }
            success_count = 0; 
        }
    }

    // ==========================================
    // 交互演示
    // ==========================================
    std::cout << "\n训练结束。进入交互演示模式。\n";
    //system("pause"); // Windows 下暂停，等待用户按键
    std::cout << "\n(按回车继续...)";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();

    while (true) {
        env.render({-1, -1}); // 只画地图

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
            

            // ==========================================
            // 新增：检测是否按下 'q' 键中断
            // ==========================================
            std::cout << " (按 'q' 键可随时强制停止当前演示)\n";
            if (check_quit()) {
                std::cout << "\n>>>  已手动强制终止当前演示！ <<<\n";
                break; // 跳出当前动画循环，但不会退出整个程序
            }

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
        // if (done) std::cout << "\n>>> 成功到达! <<<\n";
        // else std::cout << "\n>>> 失败 <<<\n";

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