#include <bits/stdc++.h>
#include <fstream>
#include <sstream>
#include <chrono>

#include "thread_pool.h"

using namespace std;
using namespace std::chrono;

constexpr std::array<pair<int, int>, 4> deltas{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
void copy_str(Fixed * fir, Fixed * sec,  size_t& cnt) {
    for (int j = 0; j < cnt; j++) {
        fir[j] = sec[j];
    }
}
void clear_func(array<Fixed, 4> * v, size_t& cnt) {
    for (int j = 0; j < cnt; j++) {
        v[j] = {};
    }
}


class Simulation{
public:
    void init(vector<vector<char>> &te, size_t init_T, ThreadPool * threads) {
        thread_pool = threads;
		Time = init_T;
        N = te.size();
        M = te[0].size();
        rnd.seed(1337);
        rho[' '] = 0.01;
        rho['.'] = 1000;
        g = 0.1;
 
        velocity.init(N, M, threads);
        velocity_flow.init(N, M, threads);

        field = new char*[N]{};
        dirs = new int*[N]{};
        p = new Fixed *[N]{};
        old_p = new Fixed*[N]{};
        last_use = new int*[N]{};
       
		for(size_t i = 0; i < N; i++) {
            field[i] = new char[M + 1]{};
            dirs[i] = new int[M]{};
            last_use[i] = new int[M]{};
            p[i] = new Fixed[M]{};
            old_p[i] = new Fixed[M]{};
			for(size_t j = 0; j < M; j++) {
				field[i][j] = te[i][j];
			}
		}


        for (size_t x = 0; x < N; x++) {
           
            for (size_t y = 0; y < M; y++) {
                if (field[x][y] == '#')
                    continue;
                for (auto [dx, dy] : deltas) {        
                    dirs[x][y] += (field[x + dx][y + dy] != '#');
                }
            }
        }
		
    }

    void start() {
        for (size_t i = 0; i < Time; ++i) {
            
            Fixed total_delta_p = 0;
            // Apply external forces
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
                    if (field[x][y] == '#')
                        continue;
                    if (field[x + 1][y] != '#')
                        velocity.add(x, y, 1, 0, g);
                }
            }

            // Apply forces from p
            for (int i = 0; i < N; i++) {
                thread_pool->put(copy_str, ref(old_p[i]), ref(p[i]), ref(M));
            }
            thread_pool -> wait();
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
                    if (field[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas) {
                        int nx = x + dx, ny = y + dy;
                        if (field[nx][ny] != '#' && old_p[nx][ny] < old_p[x][y]) {
                            auto delta_p = old_p[x][y] - old_p[nx][ny];
                            auto force = delta_p;
                            auto &contr = velocity.get(nx, ny, -dx, -dy);
                            if (contr * rho[(int) field[nx][ny]] >= force) {
                                contr -= force / rho[(int) field[nx][ny]];
                                continue;
                            }
                            force -= contr * rho[(int) field[nx][ny]];
                            contr = 0;
                            velocity.add(x, y, dx, dy, (force / rho[(int) field[x][y]]));
                            p[x][y] -= force / dirs[x][y];
                            total_delta_p -= force / dirs[x][y];
                        }
                    }
                }
            }

            velocity_flow.clear();
            bool prop = false;
            do {
                UT += 2;
                prop = 0;
                for (size_t x = 0; x < N; ++x) {
                    for (size_t y = 0; y < M; ++y) {
                        if (field[x][y] != '#' && last_use[x][y] != UT) {
                            auto [t, local_prop, _] = propagate_flow(x, y, 1);
                            if (t > 0) {
                                prop = 1;
                            }
                        }
                    }
                }
            } while (prop);
            // Recalculate p with kinetic energy
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
                    if (field[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas) {
                        auto old_v = velocity.get(x, y, dx, dy);
                        auto new_v = velocity_flow.get(x, y, dx, dy);
                        if (old_v > 0) {
                            assert(new_v <= old_v);
                            velocity.get(x, y, dx, dy) = new_v;
                            auto force = (old_v - new_v) * rho[(int) field[x][y]];
                            if (field[x][y] == '.')
                                force *= 0.8;
                            if (field[x + dx][y + dy] == '#') {
                                p[x][y] += force / dirs[x][y];
                                total_delta_p += force / dirs[x][y];
                            } else {
                                p[x + dx][y + dy] += force / dirs[x + dx][y + dy];
                                total_delta_p += force / dirs[x + dx][y + dy];
                            }
                        }
                    }
                }
            }

            UT += 2;
            prop = false;
            for (size_t x = 0; x < N; ++x) {
                for (size_t y = 0; y < M; ++y) {
                    if (field[x][y] != '#' && last_use[x][y] != UT) {
                        if (random01() < move_prob(x, y)) {
                            prop = true;
                            propagate_move(x, y, true);
                        } else {
                            propagate_stop(x, y, true);
                        }
                    }
                }
            }

            if (prop) {
                cout << "Tick " << i << ":\n";
                for (size_t x = 0; x < N; ++x) {
                      
                    cout << field[x] << "\n";
                }
            }
        }
     
    }

    bool propagate_move(int x, int y, bool is_first) {
        last_use[x][y] = UT - is_first;
        bool ret = false;
        int nx = -1, ny = -1;
        do {
            std::array<Fixed, deltas.size()> tres;
            Fixed sum = 0;
            for (size_t i = 0; i < deltas.size(); ++i) {
                auto [dx, dy] = deltas[i];
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] == '#' || last_use[nx][ny] == UT) {
                    tres[i] = sum;
                    continue;
                }
                auto v = velocity.get(x, y, dx, dy);
                if (v < 0) {
                    tres[i] = sum;
                    continue;
                }
                sum += v;
                tres[i] = sum;
            }

            if (sum == 0) {
                break;
            }

            Fixed p = random01() * sum;
            size_t d = std::ranges::upper_bound(tres, p) - tres.begin();

            auto [dx, dy] = deltas[d];
            nx = x + dx;
            ny = y + dy;
            assert(velocity.get(x, y, dx, dy) > 0 && field[nx][ny] != '#' && last_use[nx][ny] < UT);

            ret = (last_use[nx][ny] == UT - 1 || propagate_move(nx, ny, false));
        } while (!ret);
        last_use[x][y] = UT;
        for (size_t i = 0; i < deltas.size(); ++i) {
            auto [dx, dy] = deltas[i];
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 && velocity.get(x, y, dx, dy) < 0) {
                propagate_stop(nx, ny);
            }
        }
        if (ret) {
            if (!is_first) {
                swap(field[x][y], field[nx][ny]);
                swap(p[x][y], p[nx][ny]);
                swap(velocity.v[x][y], velocity.v[nx][ny]);
            }
        }
        return ret;
    }


    Fixed move_prob(int x, int y) {
        Fixed sum = 0;
        for (size_t i = 0; i < deltas.size(); ++i) {
            auto [dx, dy] = deltas[i];
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] == '#' || last_use[nx][ny] == UT) {
                continue;
            }
            auto v = velocity.get(x, y, dx, dy);
            if (v < 0) {
                continue;
            }
            sum += v;
        }
        return sum;
    }

    void propagate_stop(int x, int y, bool force = false) {
        if (!force) {
            bool stop = true;
            for (auto [dx, dy] : deltas) {
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 && velocity.get(x, y, dx, dy) > 0) {
                    stop = false;
                    break;
                }
            }
            if (!stop) {
                return;
            }
        }
        last_use[x][y] = UT;
        for (auto [dx, dy] : deltas) {
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] == '#' || last_use[nx][ny] == UT || velocity.get(x, y, dx, dy) > 0) {
                continue;
            }
            propagate_stop(nx, ny);
        }
    }

    tuple<Fixed, bool, pair<int, int>> propagate_flow(int x, int y, Fixed lim) {
        last_use[x][y] = UT - 1;
        Fixed ret = 0;
        for (auto [dx, dy] : deltas) {
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] != '#' && last_use[nx][ny] < UT) {
                auto cap = velocity.get(x, y, dx, dy);
                auto flow = velocity_flow.get(x, y, dx, dy);
                if (flow == cap) {
                    continue;
                }
                // assert(v >= velocity_flow.get(x, y, dx, dy));
                auto vp = (lim < (cap - flow) ? lim : (cap - flow));
                if (last_use[nx][ny] == UT - 1) {
                    velocity_flow.add(x, y, dx, dy, vp);
                    last_use[x][y] = UT;
                    // cerr << x << " " << y << " -> " << nx << " " << ny << " " << vp << " / " << lim << "\n";
                    return make_tuple(vp, 1, make_pair(nx, ny));
                }
                auto [t, prop, end] = propagate_flow(nx, ny, vp);
                ret += t;
                if (prop) {
                    velocity_flow.add(x, y, dx, dy, t);
                    last_use[x][y] = UT;
                    // cerr << x << " " << y << " -> " << nx << " " << ny << " " << t << " / " << lim << "\n";
                    return make_tuple(t, prop && end != pair(x, y), end);
                }
            }
        }
        last_use[x][y] = UT;
        return {ret, 0, {0, 0}};
    }

    
    Fixed random01() {
        Fixed a;
        return a.from_raw((rnd() & ((1 << 16) - 1)));
    }

    ~Simulation() {
         for (int i = 0; i < N; i++) {
             
            if (dirs[i] != nullptr) {
                delete[] dirs[i];
            } 
            if (field[i] != nullptr) {
                delete[] field[i];
            } 
            if (p[i] != nullptr) {
                delete[] p[i];
            } 
             if (old_p[i] != nullptr) {
                delete[] old_p[i];
            } 
            if (last_use[i] != nullptr) {
                delete[] last_use[i];
            }
            
        }
        if (dirs != nullptr) {
            delete[] dirs;
        }
        if (field != nullptr) {
            delete[] field;
        }
        if (p != nullptr) {
            delete[] p;
        }
        if (old_p != nullptr) {
            delete[] old_p;
        }
        if (last_use != nullptr) {
            delete[] last_use;
        }
            
    }

private:
    ThreadPool * thread_pool;
    size_t N;
    size_t M;
    Fixed g;
    Fixed rho[256];
    int** dirs;
    char** field;
    Fixed ** p, ** old_p;
    size_t Time;

    struct VectorField {
        size_t N, M;
        ThreadPool * thread_pool;
        void init(size_t _N, size_t _M, ThreadPool * _thread_pool){
            N= _N;
            M = _M;
            thread_pool = _thread_pool;
            v = new array<Fixed, deltas.size()>*[N]{};
            for (int i = 0; i < N; i++) {
                v[i] = new array<Fixed, deltas.size()>[M]{};
            }
        }
        void clear() {
            for (int i = 0; i < N; i++) {
                thread_pool->put(clear_func, ref(v[i]), ref(M));
            }
            thread_pool -> wait();
        }
        ~VectorField() {
            
            for (unsigned i = 0; i < N; i++) {
                if (v[i] == nullptr) continue;
                delete[] v[i];
            }
            if (v != nullptr) {
                delete[] v;
            }
            
        }
        array<Fixed, deltas.size()> ** v;
        Fixed &add(int x, int y, int dx, int dy, Fixed dv) {
            return get(x, y, dx, dy) += dv;
        }

        Fixed &get(int x, int y, int dx, int dy) {
            size_t i = (dx == 0? (dy < 0 ? 2 : 3) : (dx < 0? 0: 1));
            return v[x][y][i];
        } 
    };


    VectorField velocity;
    VectorField velocity_flow;
    int** last_use;
    int UT = 0;

    mt19937 rnd;

};


int main() {
    cout << "Максимальное возможное количество потоков: " << std::thread::hardware_concurrency() << endl;
    int thread_cnt;
    cin >> thread_cnt;
    ThreadPool threads(thread_cnt);

    string filename = "input.txt";
    int input_n, input_m, input_Time;

    // Считывание размера картинки симуляции и количества итераций симуляции из файла
    ifstream file;
    file.open(filename);
    if (!file.is_open()) {
        cout << "Файл " << filename << "не удалось открыть :(\n";
        exit(-1);
    }

    string tempstring;
    getline(file, tempstring);
    istringstream SS(tempstring);
    SS >> input_n >> input_m >> input_Time;
    
    vector<vector<char>>  te(input_n, vector<char> (input_m));
    for (int i = 0; i < input_n; i++) {
        getline(file, tempstring);
        for (int j = 0; j < input_m; j++) {
            te[i][j] = tempstring[j];
        }
    }

    // Замер времени симуляции

    Simulation sim;
    sim.init(te, input_Time, &threads);

    auto start = high_resolution_clock::now();

    sim.start();

    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    cout << "Время выполнения: " << duration.count() << endl;

}
